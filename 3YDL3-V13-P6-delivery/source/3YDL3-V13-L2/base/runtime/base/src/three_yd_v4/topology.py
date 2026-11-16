"""V4 edge tasks. L1 never produces a raster corridor or crossing gates."""
import copy
import json
import time
from pathlib import Path
import numpy as np
from arena_evaluation.semantic_map import canonical_hash
from arena_3d_v1.multires_topology import TopologyRoute
from three_yd_v3.topology import OutlineTopology
from three_yd_v3.edge_routing import EdgeRouter


class SoftEdgeTopology(OutlineTopology):
    SCHEMA = '3YD-V4-r1-original-edge-task-v2'

    def __init__(self, world, graph_path, cache_root):
        started = time.monotonic()
        super().__init__(world, graph_path, cache_root)
        root = Path(graph_path).parent
        self.router = EdgeRouter(self.graph, json.loads((root/'semantic.json').read_text()),
                                 json.loads((root/'parking_access.json').read_text()))
        self.key = canonical_hash([self.SCHEMA, self.key, self.router.key])
        self.tasks = {}
        self.preparation_ms = (time.monotonic()-started)*1000

    def plan(self, query, *, excluded_edge_transitions=(), start_road=None, goal_road=None, **kwargs):
        endpoint_regions = []
        for p in (query.start, query.goal):
            rc = self.world.map.world_to_cell(*p[:2])
            if rc is None or not self.subregions[rc]:
                raise ValueError('ENDPOINT_HAS_NO_SEMANTIC_REGION')
            endpoint_regions.append(int(self.subregions[rc]))
        edge = self.router.plan(query.query_id, query.start, query.goal,
                                *endpoint_regions, excluded=excluded_edge_transitions, start_road=start_road, goal_road=goal_road)
        uses = edge['ordered_edge_uses']
        regions = []
        for u in uses:
            if not regions or regions[-1] != u['region_id']:
                regions.append(u['region_id'])
        for role, index in [('start', 0), ('goal', -1)]:
            rid = edge[role+'_attachment']['semantic_region_id']
            if self.regions[rid]['kind'] == 'parking_area':
                regions.insert(0, rid) if index == 0 else regions.append(rid)
        ids = list(dict.fromkeys(u['edge_id'] for u in uses))
        task = {
            'schema': self.SCHEMA, 'frame_id': 'map',
            'original_edges': [copy.deepcopy(self.router.edges[i]) for i in ids],
            'ordered_uses': copy.deepcopy(uses),
            'start': list(query.start), 'goal': list(query.goal),
            'endpoint_access': {role: edge[role+'_attachment'] for role in ('start','goal')},
            'priority_region_ids': list(dict.fromkeys(regions)),
            'map_hash': self.world.map.sha256, 'semantic_hash': self.world.semantic.semantic_map_hash,
            'topology_hash': self.source_hash, 'rules_hash': self.router.key,
            'signature': edge['signature'], 'edge_plan': edge,
            'raster_corridor_produced': False, 'mandatory_crossing_gates': False,
            'geometry_note': 'Whole original edges. Per-query use intervals only activate soft guidance; no graph splits.',
        }
        self.tasks[edge['signature']] = task
        return TopologyRoute(tuple(regions), tuple(u['use_id'] for u in uses),
                             tuple(self.parent_semantics[r] for r in regions),
                             tuple(query.start), tuple(query.goal), edge['signature'],
                             {'edge_task': task, **edge['diagnostics']},
                             {'world_key': self.world.key, 'topology_key': self.key, **self.world.binding})

    def task(self, route):
        return self.tasks[route.signature]

    def channel_plan(self, route):
        # Compatibility with the unchanged experiment writer, not an L1 channel.
        return self.task(route)

    def fine_mask(self, route):
        # Runtime mask transport still uses its old name. It contains only the
        # immutable physical map, never the computational L2 search ROI.
        return ~self.static_hard
