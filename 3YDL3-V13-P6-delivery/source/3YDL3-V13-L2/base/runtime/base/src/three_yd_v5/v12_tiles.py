"""Disk tiles, bounded immutable LRU and the unchanged native sweep kernel.

P1 prototype: each certificate checker owns its temporary hard/scope store.
The legacy world and L2 still own global arrays; this is not P2 acceptance.
No mmap is used and no hidden full-image fallback exists in this reader.
"""
from collections import OrderedDict
from dataclasses import asdict
from pathlib import Path
import hashlib
import json
import math
import os
import shutil
import tempfile
import numpy as np
from .v12_contracts import ContractError, FailureCode, TileSpec


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


class TileStore:
    SCHEMA = '3yd-v12-tiles-v1'

    @classmethod
    def write(cls, destination, array, spec):
        destination = Path(destination)
        if destination.exists():
            raise FileExistsError(destination)
        a = np.asarray(array)
        if a.shape != spec.shape or a.dtype.hasobject or a.ndim != 2:
            raise ContractError(FailureCode.INVALID_INPUT, 'tile array')
        destination.parent.mkdir(parents=True, exist_ok=True)
        temp = Path(tempfile.mkdtemp(prefix='.tiles-', dir=destination.parent))
        try:
            tiles = {}
            for r in range(0, a.shape[0], spec.tile_cells):
                for c in range(0, a.shape[1], spec.tile_cells):
                    name = f'{r//spec.tile_cells}_{c//spec.tile_cells}.npy'
                    p = temp/name
                    np.save(p, a[r:r+spec.tile_cells, c:c+spec.tile_cells], allow_pickle=False)
                    tiles[name] = digest(p)
            manifest = {'schema': cls.SCHEMA, 'spec': asdict(spec), 'dtype': a.dtype.str,
                        'tiles': tiles, 'cell_bytes': int(a.nbytes)}
            (temp/'manifest.json').write_text(json.dumps(manifest, sort_keys=True)+'\n')
            (temp/'manifest.sha256').write_text(digest(temp/'manifest.json')+'\n')
            os.rename(temp, destination)
        except BaseException:
            shutil.rmtree(temp, ignore_errors=True)
            raise

    def __init__(self, root, expected_map_sha256=None):
        self.root = Path(root)
        try:
            if digest(self.root/'manifest.json') != (self.root/'manifest.sha256').read_text().strip():
                raise ValueError('manifest digest')
            m = json.loads((self.root/'manifest.json').read_text())
            if m['schema'] != self.SCHEMA:
                raise ValueError('schema')
            raw = m['spec']; raw['shape'] = tuple(raw['shape']); raw['origin'] = tuple(raw['origin'])
            self.spec = TileSpec(**raw); self.dtype = np.dtype(m['dtype'])
            if self.dtype.hasobject or self.spec.cache_budget_bytes < self.spec.tile_cells**2*self.dtype.itemsize:
                raise ValueError('dtype/cache budget')
            self.hashes = m['tiles']
            expected_names = {f'{r}_{c}.npy' for r in range(math.ceil(self.spec.shape[0]/self.spec.tile_cells))
                              for c in range(math.ceil(self.spec.shape[1]/self.spec.tile_cells))}
            if set(self.hashes) != expected_names:
                raise ValueError('incomplete tile coverage')
        except FileNotFoundError as e:
            raise ContractError(FailureCode.TILE_MISSING, e) from e
        except (ValueError, KeyError, TypeError) as e:
            raise ContractError(FailureCode.TILE_CORRUPT, e) from e
        if expected_map_sha256 is not None and self.spec.map_sha256 != expected_map_sha256:
            raise ContractError(FailureCode.INDEX_STALE, 'tile map binding')
        self.cache = OrderedDict(); self.cached_bytes = 0
        self.stats = {'loads': 0, 'hits': 0, 'evictions': 0, 'peak_cached_bytes': 0,
                      'read_rects': 0, 'peak_rectangle_bytes': 0, 'touched_tiles': set()}

    def _tile(self, r, c):
        name = f'{r}_{c}.npy'
        if name in self.cache:
            self.stats['hits'] += 1
            self.cache.move_to_end(name)
            return self.cache[name]
        path = self.root/name
        try:
            if digest(path) != self.hashes[name]:
                raise ValueError('tile digest')
            a = np.load(path, allow_pickle=False)
            expected = (min(self.spec.tile_cells, self.spec.shape[0]-r*self.spec.tile_cells),
                        min(self.spec.tile_cells, self.spec.shape[1]-c*self.spec.tile_cells))
            if a.shape != expected or a.dtype != self.dtype:
                raise ValueError('tile geometry')
        except FileNotFoundError as e:
            raise ContractError(FailureCode.TILE_MISSING, name) from e
        except (ValueError, KeyError, OSError) as e:
            raise ContractError(FailureCode.TILE_CORRUPT, name) from e
        while self.cache and self.cached_bytes+a.nbytes > self.spec.cache_budget_bytes:
            _, removed = self.cache.popitem(last=False)
            self.cached_bytes -= removed.nbytes; self.stats['evictions'] += 1
        if a.nbytes > self.spec.cache_budget_bytes:
            raise ContractError(FailureCode.TILE_BUDGET, name)
        a.flags.writeable = False; self.cache[name] = a; self.cached_bytes += a.nbytes
        self.stats['loads'] += 1; self.stats['touched_tiles'].add(name)
        self.stats['peak_cached_bytes'] = max(self.stats['peak_cached_bytes'], self.cached_bytes)
        return a

    def read(self, box):
        r0, r1, c0, c1 = box
        h, w = self.spec.shape
        if not (0 <= r0 < r1 <= h and 0 <= c0 < c1 <= w):
            raise ContractError(FailureCode.INVALID_INPUT, 'tile rectangle outside map')
        result = np.empty((r1-r0, c1-c0), self.dtype); t = self.spec.tile_cells
        for r in range(r0//t, (r1-1)//t+1):
            for c in range(c0//t, (c1-1)//t+1):
                a, b = max(r0, r*t), min(r1, (r+1)*t)
                x, y = max(c0, c*t), min(c1, (c+1)*t)
                result[a-r0:b-r0, x-c0:y-c0] = self._tile(r,c)[a-r*t:b-r*t, x-c*t:y-c*t]
        self.stats['read_rects'] += 1
        self.stats['peak_rectangle_bytes'] = max(self.stats['peak_rectangle_bytes'], result.nbytes)
        return result

    def clear(self):
        self.cache.clear(); self.cached_bytes = 0

    def diagnostics(self):
        return {**self.stats, 'touched_tiles': sorted(self.stats['touched_tiles']),
                'cached_bytes': self.cached_bytes, 'cache_budget_bytes': self.spec.cache_budget_bytes,
                'shape': list(self.spec.shape), 'disk_bytes': sum(p.stat().st_size for p in self.root.iterdir()),
                'full_map_materialization_fallback': False}


SWEEPS = []


def prepare_static_tiles(world, root):
    """Map-only preparation; no query/scope/ACK/certificate work is performed."""
    from .v9_maps import immutable
    if not immutable(world.hard):
        raise ContractError(FailureCode.INVALID_INPUT, 'immutable static hard map required')
    spec = TileSpec(world.map.sha256, tuple(world.hard.shape), float(world.map.resolution),
                    tuple(world.map.origin), tile_cells=256, cache_budget_bytes=256*256*4)
    root = Path(root)/world.key
    if not root.exists():
        TileStore.write(root,world.hard.view(np.uint8),spec)
    store = TileStore(root,spec.map_sha256)
    if store.spec != spec:
        raise ContractError(FailureCode.INDEX_STALE, 'static tile geometry')
    # Verify the exact predicate too: image SHA alone does not identify an
    # unknown-cell policy. Stream rectangles, never retain another full map.
    for r in range(0,spec.shape[0],spec.tile_cells):
        for c in range(0,spec.shape[1],spec.tile_cells):
            box=(r,min(r+spec.tile_cells,spec.shape[0]),c,min(c+spec.tile_cells,spec.shape[1]))
            if not np.array_equal(store.read(box),world.hard[box[0]:box[1],box[2]:box[3]]):
                raise ContractError(FailureCode.TILE_CORRUPT,'static predicate differs from current map')
    store.clear(); world.v12_static_tile_root=root; world.v12_static_hard_ref=world.hard
    return {'path':str(root),'shape':list(spec.shape),'disk_bytes':store.diagnostics()['disk_bytes'],
            'all_static_hard_cells_verified':True,'contains_query_data':False,
            'static_cache_bytes_after_preparation':store.cached_bytes}


class TiledSweptChecker:
    """Split only native sweep calls; all path edges are checked, including joins."""
    def __init__(self, hard, world):
        self.spec = TileSpec(world.map.sha256, tuple(hard.shape), float(world.map.resolution),
                             tuple(world.map.origin), tile_cells=256, cache_budget_bytes=256*256*4)
        self.temp = None; self.overlay = None; self.static = None
        if hasattr(world,'v12_static_tile_root'):
            if world.hard is not world.v12_static_hard_ref:
                raise ContractError(FailureCode.INDEX_STALE,'static predicate owner changed')
            self.store=TileStore(world.v12_static_tile_root,self.spec.map_sha256)
            self.overlay=hard; self.static=world.hard
        else:
            self.temp = tempfile.TemporaryDirectory(prefix='v12-sweep-')
            TileStore.write(Path(self.temp.name)/'hard', np.asarray(hard, dtype=np.uint8), self.spec)
            self.store = TileStore(Path(self.temp.name)/'hard', self.spec.map_sha256)
        self.half_length, self.half_width = world.vehicle[:2]
        self.calls = 0; self.closed = False

    def check(self, poses):
        from arena_3d_v1.semantic_world import SweptChecker
        p = np.asarray(poses, float)
        if p.ndim != 2 or p.shape[1] != 3 or not len(p) or not np.isfinite(p).all():
            return False
        height, width = self.spec.shape; res = self.spec.resolution_m; ox, oy = self.spec.origin[:2]
        body_radius = math.hypot(self.half_length, self.half_width)
        # Fixed pose batches are spatially small for certificate samples.
        # The first endpoint is shared between consecutive batches.
        for first in range(0, max(1, len(p)-1), 32):
            q = p[first:min(first+33,len(p))]
            # The original kernel encloses each edge at its midpoint with
            # isotropic translation/rotation enlargement. A crop must cover
            # that enclosure, including sparse diagnostic input edges.
            delta = np.diff(q,axis=0)
            travel = float(np.linalg.norm(delta[:,:2],axis=1).max(initial=0.))
            turn = float(np.abs((delta[:,2]+np.pi)%(2*np.pi)-np.pi).max(initial=0.))
            radius = body_radius + travel/2 + body_radius*turn/2 + 2*res
            low = q[:,:2].min(axis=0)-radius; high = q[:,:2].max(axis=0)+radius
            c0 = max(0, math.floor((low[0]-ox)/res)); c1 = min(width, math.floor((high[0]-ox)/res)+1)
            r0 = max(0, height-1-math.floor((high[1]-oy)/res)); r1 = min(height, height-math.floor((low[1]-oy)/res))
            if not (r0 < r1 and c0 < c1):
                return False
            hard = self.store.read((r0,r1,c0,c1))
            if self.overlay is not None:
                # Static obstacles come from verified disk tiles. Request
                # scope additions remain legacy dense data in this P1 adapter.
                overlay=np.asarray(self.overlay[r0:r1,c0:c1],bool) & ~self.static[r0:r1,c0:c1]
                np.bitwise_or(hard,overlay,out=hard)
            origin = (ox+c0*res, oy+(height-r1)*res, 0.)
            checker = SweptChecker(hard, res, origin, self.half_length, self.half_width)
            self.calls += 1
            if not checker.check(q):
                return False
        return True

    def close(self):
        if self.closed:
            return
        d = self.store.diagnostics(); self.store.clear()
        SWEEPS.append({**d, 'cached_bytes_after_release': self.store.cached_bytes,
                       'native_calls': self.calls, 'all_edges_including_chunk_joins_checked': True,
                       'prototype_scope': 'certificate_hard_and_scope_sweeps',
                       'request_scope_source':'legacy_dense_overlay' if self.overlay is not None else 'temporary_predicate_tiles',
                       'legacy_world_L2_and_native_task_rectangle_still_resident': True})
        if self.temp is not None:self.temp.cleanup()
        self.overlay=None;self.static=None;self.closed = True

    def __del__(self):
        if hasattr(self, 'store'):
            self.close()
