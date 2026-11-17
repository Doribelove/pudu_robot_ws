"""External persistence adapter for the frozen L1 implementations.

Only constructor state is serialized. Routes/query caches never enter the
index. Loading uses an allowlisted class and data codecs, never pickle.
This persists existing semantic/coarse graphs; automatic skeleton extraction
is a separate V12 task and is not claimed by this adapter.
"""
from collections import OrderedDict, defaultdict
from pathlib import Path
from types import SimpleNamespace
import hashlib
import json
import os
import shutil
import tempfile
import numpy as np
from scipy.sparse import csr_matrix, isspmatrix_csr
from .v12_contracts import ContractError, FailureCode
from .v12_tiles import digest

SCHEMA = '3yd-v12-topology-index-v1'
TRANSIENT = {'world', 'static_hard', 'tasks', 'contexts', '_cache', '_first_phase_cache',
             '_components', '_probe_stats', 'v12_persistence'}
TIMES = {'preparation_ms', '_directed_precompute_ms'}


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def binding(world, cls, graph_path=None):
    from . import free_space_topology, topology, topology_astar
    inputs = {}
    if graph_path is not None:
        for name in ('topology.json', 'semantic.json', 'parking_access.json'):
            p = Path(graph_path).parent/name; inputs[name] = digest(p)
    flags = {k: v for k, v in os.environ.items()
             if k.startswith(('THREEYD_L1_', 'THREEYD_V6_', 'THREEYD_V7_'))}
    return {'schema': SCHEMA, 'class': cls.__name__, 'class_schema': cls.SCHEMA,
            'map_sha256': world.map.sha256, 'map_yaml_sha256': digest(world.map.yaml_path),
            'world_key': world.key, 'shape': [world.map.height, world.map.width],
            'resolution': world.map.resolution, 'origin': list(world.map.origin),
            'vehicle': list(world.vehicle), 'inputs': inputs, 'constructor_flags': flags,
            'implementation': {Path(m.__file__).name: digest(m.__file__)
                               for m in (free_space_topology, topology, topology_astar)}}


class Codec:
    def __init__(self, root, files=None):
        self.root = Path(root); self.files = {} if files is None else files

    def encode(self, value):
        if isinstance(value, np.ndarray):
            if value.dtype.hasobject:
                raise TypeError('object array')
            name = f'a{len(self.files):05d}.npy'; np.save(self.root/name, value, allow_pickle=False)
            self.files[name] = digest(self.root/name)
            return {'type': 'array', 'file': name}
        if isspmatrix_csr(value):
            return {'type': 'csr', 'shape': list(value.shape),
                    'data': self.encode(value.data), 'indices': self.encode(value.indices),
                    'indptr': self.encode(value.indptr)}
        if isinstance(value, np.generic):
            return self.encode(value.item())
        if isinstance(value, SimpleNamespace):
            return {'type': 'namespace', 'value': self.encode(vars(value))}
        if isinstance(value, dict):
            kind = ('default_list' if isinstance(value, defaultdict) and value.default_factory is list else
                    'default_set' if isinstance(value, defaultdict) and value.default_factory is set else
                    'ordered' if isinstance(value, OrderedDict) else 'dict')
            return {'type': kind, 'items': [[self.encode(k), self.encode(v)] for k,v in value.items()]}
        if isinstance(value, (list, tuple, set)):
            values = sorted(value) if isinstance(value, set) else value
            return {'type': type(value).__name__, 'items': [self.encode(v) for v in values]}
        if value is None or type(value) in (str, int, float, bool):
            return value
        raise TypeError('unsupported topology state: '+str(type(value)))

    def decode(self, value):
        if not isinstance(value, dict):
            return value
        kind = value['type']
        if kind == 'array':
            name = value['file']
            if name not in self.files or Path(name).name != name:
                raise ValueError('array outside index')
            a = np.load(self.root/name, allow_pickle=False)
            if a.dtype.hasobject:
                raise ValueError('object array')
            from .v9_maps import seal
            return seal(a)
        if kind == 'csr':
            a = csr_matrix((self.decode(value['data']), self.decode(value['indices']),
                            self.decode(value['indptr'])), shape=tuple(value['shape']))
            a.check_format(full_check=True)
            return a
        if kind == 'namespace':
            return SimpleNamespace(**self.decode(value['value']))
        if kind in ('list', 'tuple', 'set'):
            items = [self.decode(x) for x in value['items']]
            return {'list': list, 'tuple': tuple, 'set': set}[kind](items)
        if kind in ('dict', 'ordered', 'default_list', 'default_set'):
            pairs = [(self.decode(k), self.decode(v)) for k,v in value['items']]
            if kind == 'default_list': return defaultdict(list, pairs)
            if kind == 'default_set': return defaultdict(set, pairs)
            return (OrderedDict if kind == 'ordered' else dict)(pairs)
        raise ValueError('unknown index data type')


def save(destination, topology, expected):
    destination = Path(destination); destination.parent.mkdir(parents=True, exist_ok=True)
    temp = Path(tempfile.mkdtemp(prefix='.index-', dir=destination.parent))
    try:
        codec = Codec(temp)
        state = {k:v for k,v in vars(topology).items() if k not in TRANSIENT | TIMES}
        manifest = {'schema': SCHEMA, 'binding': expected, 'state': codec.encode(state),
                    'files': codec.files, 'query_results_stored': False,
                    'automatic_channel_extraction': False}
        (temp/'index.json').write_text(canonical(manifest)+'\n')
        (temp/'index.sha256').write_text(digest(temp/'index.json')+'\n')
        os.rename(temp, destination)
    except BaseException:
        shutil.rmtree(temp, ignore_errors=True)
        raise


def load(root, world, cls, expected):
    from .free_space_topology import FreeSpaceTopology
    from .topology_astar import AStarTopology
    from .topology import V5Topology
    if cls not in (FreeSpaceTopology, AStarTopology, V5Topology):
        raise ContractError(FailureCode.INVALID_INPUT, 'topology class not allowlisted')
    root = Path(root)
    try:
        if digest(root/'index.json') != (root/'index.sha256').read_text().strip():
            raise ValueError('index manifest digest')
        m = json.loads((root/'index.json').read_text())
        if m['schema'] != SCHEMA:
            raise ValueError('index schema')
        if m['binding'] != expected:
            raise ContractError(FailureCode.INDEX_STALE, 'index binding')
        for name, sha in m['files'].items():
            if Path(name).name != name or digest(root/name) != sha:
                raise ValueError('index payload digest')
        state = Codec(root, m['files']).decode(m['state'])
        if set(state) & (TRANSIENT | TIMES):
            raise ValueError('runtime state in static index')
        obj = cls.__new__(cls); obj.__dict__.update(state)
        obj.world = world
        if cls is not FreeSpaceTopology: obj.static_hard = world.hard
        obj.tasks = {}; obj.contexts = {}; obj._cache = {}
        obj._first_phase_cache = OrderedDict(); obj._components = {}; obj._probe_stats = {}
        obj.preparation_ms = 0.; obj._directed_precompute_ms = 0.
        obj._last_first_phase_resume = False
    except ContractError:
        raise
    except FileNotFoundError as e:
        raise ContractError(FailureCode.INDEX_MISSING, e) from e
    except (ValueError, KeyError, TypeError, OSError) as e:
        raise ContractError(FailureCode.INDEX_CORRUPT, e) from e
    obj.v12_persistence = {'loaded': True, 'path': str(root), 'constructor_called': False,
                           'query_results_restored': False, 'index_sha256': digest(root/'index.json')}
    return obj


def persistent(world, cls, cache_root, graph_path=None):
    expected = binding(world, cls, graph_path)
    key = hashlib.sha256(canonical(expected).encode()).hexdigest()
    root = Path(cache_root)/key
    if root.exists():
        return load(root, world, cls, expected)
    # No fallback from a corrupt index. Only a missing identity is built.
    args = (world, graph_path, root.parent/'construction') if graph_path else (world, root.parent/'construction')
    obj = cls(*args)
    save(root, obj, expected)
    obj.v12_persistence = {'loaded': False, 'path': str(root), 'constructor_called': True,
                           'query_results_restored': False, 'index_sha256': digest(root/'index.json')}
    return obj
