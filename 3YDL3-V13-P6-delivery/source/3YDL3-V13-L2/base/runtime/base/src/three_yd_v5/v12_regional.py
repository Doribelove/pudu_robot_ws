"""Request-scoped regional soft costs; obstacle and traffic masks are unchanged."""
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import numpy as np

from .v12_contracts import RegionPolicy

SCHEMA = '3YDL3-V12-regional-policy-v1'
_CURRENT = ContextVar('v12_regional_policy', default=None)


def _hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def point_segment_distance(xy, points):
    """Exact point-to-polyline distance, bounded by caller's point chunk."""
    xy = np.asarray(xy, float); points = np.asarray(points, float)
    best = np.full(len(xy), np.inf)
    for a, b in zip(points[:-1], points[1:]):
        v = b-a; vv = float(v@v)
        if vv <= 1e-16: continue
        u = np.clip((xy-a)@v/vv, 0., 1.)
        best = np.minimum(best, np.linalg.norm(xy-a-u[:, None]*v, axis=1))
    return best


class PolicySet:
    def __init__(self, data=None, *, map_sha256=None, region_ids=()):
        self.source = data or {}; self.policies = {}; self.walls = {}
        self.default = None; self.fingerprint = _hash(self.source)
        if data is None: return
        if not isinstance(data,dict):raise ValueError('V12_POLICY_BINDING_SCHEMA_TYPE')
        if data.get('schema') != SCHEMA or data.get('map_sha256') != map_sha256:
            raise ValueError('V12_POLICY_BINDING_MAP_OR_SCHEMA')
        known = {str(r) for r in region_ids}
        for key, value in data.get('walls', {}).items():
            points = np.asarray(value.get('points', []), float)
            if (not key or str(value.get('region_id')) not in known
                    or value.get('side') not in ('left', 'right')
                    or points.ndim != 2 or points.shape[1:] != (2,) or len(points) < 2
                    or len(points) > 4096 or not np.isfinite(points).all()
                    or np.any(np.linalg.norm(np.diff(points, axis=0), axis=1) < 1e-8)):
                raise ValueError('V12_POLICY_BINDING_WALL_GEOMETRY')
            self.walls[key] = {'points': tuple(map(tuple, points.tolist())),
                              'region_id': str(value['region_id']), 'side': value['side']}
        for value in data.get('policies', []):
            p = RegionPolicy(**value)
            if p.hard_traffic_policy_id!='inherited':
                raise ValueError('V12_POLICY_BINDING_UNSUPPORTED_HARD_TRAFFIC_POLICY')
            if p.region_id not in known or p.region_id in self.policies:
                raise ValueError('V12_POLICY_BINDING_REGION')
            if p.mode == 'edge':
                wall = self.walls.get(p.wall_id)
                if wall is None or wall['region_id'] != p.region_id or wall['side'] != p.side:
                    raise ValueError('V12_POLICY_BINDING_SIDEWALL')
            self.policies[p.region_id] = p
        if 'default' in data:
            self.default = RegionPolicy(region_id='*', **data['default'])
            if self.default.hard_traffic_policy_id!='inherited':
                raise ValueError('V12_POLICY_BINDING_UNSUPPORTED_HARD_TRAFFIC_POLICY')
            if self.default.mode == 'edge':
                raise ValueError('V12_POLICY_BINDING_EDGE_REQUIRES_REGION')
        self.fingerprint = _hash({'schema': SCHEMA, 'map_sha256': map_sha256,
            'policies': [asdict(self.policies[k]) for k in sorted(self.policies)],
            'default': None if self.default is None else asdict(self.default), 'walls': self.walls})

    @property
    def active(self): return bool(self.policies or self.default is not None)

    def get(self, region_id): return self.policies.get(str(region_id), self.default)

    def evidence(self):
        return {'schema': SCHEMA, 'fingerprint': self.fingerprint,
                'policies': [asdict(p) for p in self.policies.values()],
                'default': None if self.default is None else asdict(self.default),
                'walls': self.walls, 'hard_traffic_unchanged': True,
                'L3_target': 'the regional-cost L2 geometric reference',
                'distance_semantics': 'static obstacle grid distance at vehicle center',
                'weight_semantics': 'regional weight multiplied by existing global L2 weight'}

    def verify_walls(self, world):
        """Reject arbitrary guide lines masquerading as static sidewalls."""
        for wall in self.walls.values():
            samples = []
            for a, b in zip(wall['points'][:-1], wall['points'][1:]):
                a = np.asarray(a); b = np.asarray(b)
                if np.linalg.norm(b-a) > 1000:
                    raise ValueError('V12_POLICY_BINDING_WALL_TOO_LONG')
                n = max(2, int(np.ceil(np.linalg.norm(b-a)/world.map.resolution))+1)
                samples.append(a+(b-a)*np.linspace(0, 1, n)[:, None])
            for points in samples:
                for start in range(0, len(points), 65536):
                    r, c = world.cells(points[start:start+65536])
                    if (np.any(r < 0) or np.any(c < 0) or np.any(r >= world.map.height)
                            or np.any(c >= world.map.width)
                            or np.any(world.map.distance_m[r, c] > 2*world.map.resolution)):
                        raise ValueError('V12_POLICY_BINDING_WALL_NOT_STATIC_OBSTACLE')


EMPTY = PolicySet()


def current(): return _CURRENT.get() or EMPTY


@contextmanager
def request_scope(service):
    path = os.environ.get('THREEYD_V12_REGIONAL_CONFIG', '')
    raw = Path(path).read_bytes() if path else None
    try:
        policies = EMPTY if raw is None else PolicySet(json.loads(raw),
            map_sha256=service.world.map.sha256, region_ids=service.topology.regions)
    except (TypeError,json.JSONDecodeError) as error:
        raise ValueError('V12_POLICY_BINDING_MALFORMED_CONFIGURATION') from error
    if policies.walls: policies.verify_walls(service.world)
    token = _CURRENT.set(policies)
    try:
        yield policies
        if (os.environ.get('THREEYD_V12_REGIONAL_CONFIG', '') != path or
                (path and Path(path).read_bytes() != raw)):
            raise ValueError('V12_POLICY_BINDING_CHANGED_DURING_REQUEST')
    finally:
        _CURRENT.reset(token)


def apply_cost(controller, view, bbox, inherited, chunk=65536):
    policies = controller.v12_policies
    controller.v12_policy_evidence = policies.evidence() if policies.active else {
        'mode': 'inherited', 'fingerprint': policies.fingerprint, 'hard_traffic_unchanged': True}
    from .v12_degradation import Phase
    phase=getattr(controller,'v12_phase',Phase())
    controller.v12_policy_evidence['degradation_phase']=phase.name
    if getattr(controller,'task',{}).get('v12_full_l2_grid_reference'):
        controller.v12_policy_evidence.update(L1_right_reference_available=False,
            fallback_guide_source='full_map_2d_grid_reference',
            configured_regional_binding_preserved=True)
        if not policies.active:
            controller.v12_policy_evidence['mode']='grid_reference_after_L1_failure'
    if phase.relax_preference:
        controller.v12_policy_evidence.update(effective_soft_preference='none',
            soft_preference_relaxed=True,degradation_phase=phase.name)
        return np.zeros_like(inherited)
    if not policies.active: return inherited
    result = inherited.copy(); r0, r1, c0, c1 = bbox
    counts = {'none': 0, 'center': 0, 'edge': 0, 'inherited': 0}
    shape = result.shape; world = controller.world
    for start in range(0, result.size, chunk):
        ix = np.arange(start, min(start+chunk, result.size))
        xy = view.world_points(ix//shape[1]+r0, ix%shape[1]+c0)
        rr, cc = world.cells(xy)
        valid = (rr >= 0) & (cc >= 0) & (rr < world.map.height) & (cc < world.map.width)
        labels = np.full(len(ix), -1, int)
        labels[valid] = controller.topology.subregions[rr[valid], cc[valid]]
        for rid in np.unique(labels[valid]):
            p = policies.get(rid); selected = valid & (labels == rid)
            counts['inherited' if p is None else p.mode] += int(selected.sum())
            if p is None: continue
            if p.mode == 'none': value = np.zeros(int(selected.sum()))
            elif p.mode == 'center':
                clearance = world.map.distance_m[rr[selected], cc[selected]]
                value = p.weight/(1.+np.maximum(clearance, 0.))
            else:
                distance = point_segment_distance(xy[selected], policies.walls[p.wall_id]['points'])
                value = p.weight*np.minimum(np.abs(distance-p.desired_clearance_m), 4.)
            result.ravel()[ix[selected]] = value
    controller.v12_policy_evidence['ROI_cell_counts'] = counts
    return result


def accepted_reference(controller, xy):
    if not controller.v12_policies.active: return
    controller.l2_soft_target_xy = np.array(xy, copy=True)
    controller.l2_guide_offset_info = {**controller.l2_guide_offset_info,
        'mode': 'regional_cost_L2_reference', 'straight_offset_m': 0.}
    controller.timing['v12_regional_policy'] = controller.v12_policy_evidence
