import json, time, math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from .multires_pipeline import MultiresController, reference_route
from .multires_grid import GridView
from .semantic_reference import preference_field
from .semantic_world import digest, json_write
from .local_fine import LocalFineGraph
from arena_evaluation.semantic_map import canonical_hash
RECOVERABLE = {'COARSE_ENDPOINT_UNREPRESENTABLE', 'GUIDE_NO_PATH', 'GUIDE_SEARCH_TIMEOUT', 'WEIGHTED_L2_NO_PATH', 'L2_NO_PATH', 'L2_TIMEOUT', 'L2_UNVERIFIED_NO_PATH'}

class RecoveryController(MultiresController):

    def __init__(self, *args, **kwargs):
        kwargs['cache_root'] = Path(kwargs['cache_root']) / 'recovery-junction-axis-v4'
        super().__init__(*args, **kwargs)
        self.timing.setdefault('completion_mode', 'full_fine_fallback' if self.fallbacks else 'coarse_only')
        self.timing['reference_resolutions_m'] = [0.15, 0.05] if self.timing['completion_mode'] == 'local_fine_assisted' else [self.view.map.resolution]

    def _reference_route(self, world, plan, xy):
        route = reference_route(world, plan, xy)
        poses = route.poses.copy()
        labels = world.sample_labels(poses)
        junction_ids = [k for (k, f) in world.features.items() if f.semantic_class == 'junction_area']
        junction = np.flatnonzero(np.isin(labels, junction_ids))
        if not len(junction):
            return route
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1))]
        influence = math.ceil(2.0 + world.vehicle[2] + math.hypot(*world.vehicle[:2]))
        near = np.min(np.abs(arc[:, None] - arc[junction][None, :]), axis=1) <= influence
        for (label, feature) in world.features.items():
            if feature.semantic_class != 'lane':
                continue
            vertices = np.asarray(feature.coordinates, float)
            if len(vertices) > 1 and np.allclose(vertices[0], vertices[-1]):
                vertices = vertices[:-1]
            (vals, axes) = np.linalg.eigh(np.cov(vertices[:, :2].T))
            if vals[-1] < 9 * max(vals[0], 1e-12):
                continue
            axis = axes[:, -1]
            sel = np.flatnonzero((labels == label) & near)
            tangent = np.column_stack((np.cos(poses[sel, 2]), np.sin(poses[sel, 2])))
            dot = tangent @ axis
            sel = sel[np.abs(dot) >= math.cos(math.pi / 4)]
            dot = dot[np.abs(dot) >= math.cos(math.pi / 4)]
            poses[sel, 2] = math.atan2(axis[1], axis[0]) + np.where(dot >= 0, 0.0, math.pi)
        route.poses = poses
        route.edges[0].path = poses
        return route

    def _activate(self, factor, verify):
        if factor != 3:
            return super()._activate(factor, verify)
        try:
            return super()._activate(factor, verify)
        except (ValueError, RuntimeError, TimeoutError) as error:
            if str(error) not in RECOVERABLE:
                raise
            try:
                self._activate_local(verify)
            except (RuntimeError, TimeoutError) as local:
                if not str(local).startswith(('LOCAL_FINE_', 'GUIDE_')):
                    raise
                self.timing['local_fine_failure'] = str(local)
                raise error

    def _activate_local(self, verify):
        tick = time.monotonic()
        world = self.world
        for factor in (1, 3):
            if factor not in self.views:
                self.views[factor] = GridView(world, factor)
        v = self.views[3]
        key = canonical_hash(['local-fine-junction-axis-v4', world.key, self.plan.route_signature, digest(self.plan.corridor_mask), self.side, self.weight])
        cache = self.root / 'local_fine' / key
        cache.mkdir(parents=True, exist_ok=True)
        hit = (cache / 'path.npz').exists()
        if hit:
            with np.load(cache / 'path.npz', allow_pickle=False) as data:
                rc = data['reference']
                guide = data['guide']
            meta = json.loads((cache / 'path.json').read_text())
            if meta['key'] != key or meta['hash'] != canonical_hash([digest(rc), digest(guide)]):
                raise ValueError('LOCAL_REFERENCE_CACHE_CORRUPT')
            stats = meta['stats']
            prefkey = meta['preference_key']
            resident = 0
        else:
            graph = LocalFineGraph(world, v, self.plan.corridor_mask, self.plan.start_cell, self.plan.goal_cell, self.plan.route_signature)
            (guide, gt) = graph.search()
            xy = world.world_points(guide[:, 0], guide[:, 1])
            rp = SimpleNamespace(corridor_mask=graph.active, route_signature=self.plan.route_signature)
            pref = preference_field(world, self._reference_route(world, rp, xy), side=self.side)
            (rc, wt) = graph.search(pref.potential, self.weight)
            if verify:
                (_, oracle) = graph.search(pref.potential, self.weight, heuristic=False)
                if abs(oracle['cost_cells'] - wt['cost_cells']) > 1e-08:
                    raise AssertionError('LOCAL_FINE_ORACLE_MISMATCH')
            coarse_steps = int(np.sum(np.max(np.abs(np.diff(rc, axis=0)), axis=1) == 3))
            if not coarse_steps:
                raise RuntimeError('LOCAL_FINE_NO_COARSE_SEGMENT')
            stats = {**graph.stats, 'guide': gt, 'weighted': wt, 'coarse_steps': coarse_steps}
            prefkey = pref.key
            resident = int(graph.active.nbytes + graph.patch.nbytes + graph.coarse_centers.nbytes + graph.safe.nbytes + pref.potential.nbytes)
            np.savez_compressed(cache / 'path.npz', reference=rc, guide=guide)
            json_write(cache / 'path.json', {'key': key, 'hash': canonical_hash([digest(rc), digest(guide)]), 'stats': stats, 'preference_key': prefkey})
        if not np.all(self.plan.corridor_mask[rc[:, 0], rc[:, 1]] & world.safe[rc[:, 0], rc[:, 1]]):
            raise ValueError('LOCAL_REFERENCE_UNSAFE')
        self.reference_cells = rc
        self.route = self._reference_route(world, self.plan, world.world_points(guide[:, 0], guide[:, 1]))
        self.view = self.views[1]
        self.preference = SimpleNamespace(key=prefkey)
        diagnostics = {'backend': 'bounded_hybrid_astar', 'activation': {'state_cache_hit': False, 'reference_cache_hit': hit}, 'state_memory_bytes': resident, **stats}
        self.initial_l2_result = SimpleNamespace(success=True, diagnostics=diagnostics, oracle_cost_error=0.0 if verify else None)
        self.l2 = SimpleNamespace(path_global=rc)
        self.lifecycle = SimpleNamespace(clear=lambda : None)
        self.timing.update(completion_mode='local_fine_assisted', local_fine=stats, reference_cache_hit=hit, local_fine_ms=(time.monotonic() - tick) * 1000)
