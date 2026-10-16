"""Bounded integration smoke, deliberately excluded from formal L1 metrics."""
import argparse
import json
import os
import traceback
from pathlib import Path

import numpy as np

from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.path_audit import PathAuditor
from arena_evaluation.planner_benchmark.models import Query
from arena_3d_v1.multires_grid import GridView
from arena_3d_v1.semantic_world import json_write
from three_yd_v5.performance_session import PerformanceSession
from three_yd_v5.free_space_topology import load_map_only_world, FreeSpaceTopology
from three_yd_v5.recovery import V5Runner
from three_yd_v5.controller import V5Controller

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--query-id', default='A2B-01')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    inputs = ROOT/'inputs'/args.map_id
    row = next(q for q in json.loads((inputs/'queries.json').read_text())['queries']
               if q['id'] == args.query_id)
    query = Query(row['id'], tuple(row['start']), tuple(row['goal']))
    world = load_map_only_world(inputs/'map.yaml', args.output/'world')
    topology = FreeSpaceTopology(world, args.output/'graph')
    views = {3: GridView(world, 3), 1: GridView(world, 1)}
    context = runtime.MapContext(args.map_id, world.map, world.safe,
                                 world.map.distance_m, world.map.sha256,
                                 world.map.sha256, inputs/'map.yaml')
    auditor = PathAuditor(context, source_commit='3YD-V7-r1-map-only-smoke')
    spec = runtime.backend_availability()['hybrid_astar']
    session = PerformanceSession(context, args.output/'smac',
                                 map_yaml=inputs/'map.yaml', local_mask_updates=True,
                                 optimization_profile='v7_candidate',
                                 smac_parameter_profile='lighter_smoother',
                                 optimization_stage='step3_delta_map',
                                 enable_mask_reuse_noop=True,
                                 planner_parameter_overrides={'angle_quantization_bins': 48},
                                 costmap_ack_timeout_s=3., tile_interval_s=.015,
                                 enable_semantic_repair=True)
    session.local_map_update_strategy = 'roi_ack'
    session.full_grid_settle_cycles = 0
    result = {'version': '3YD-V7-r1', 'mode': 'free_space',
              'map_id': args.map_id, 'query_id': args.query_id,
              'formal_benchmark': False}
    try:
        session.start()
        session.reset_query_state(args.query_id, restore_base_map=False)
        controller, route, outcome = V5Runner(world, topology, args.output/'controller', views,
                                              dynamic=None, controller_class=V5Controller,
                                              max_region_routes=1).plan(query, session, auditor, spec)
        path = outcome.pop('result', None)
        result.update(success=bool(outcome.get('success')),
                      geometry_success=bool(outcome.get('geometry_success')),
                      failure_code=outcome.get('failure_code'),
                      l1_task=topology.channel_plan(route),
                      l2_points=len(controller.reference_plan.world_xy) if controller is not None else 0,
                      l2_soft_target_offset_m=(controller.l2_guide_offset_info['straight_offset_m']
                                               if controller is not None else None),
                      traffic_audit=outcome.get('traffic_audit'),
                      independent_audit=outcome.get('independent_audit'),
                      map_sync=outcome.get('map_sync'),
                      result_points=len(path.points) if path is not None else 0)
        if path is not None and path.points:
            np.save(args.output/'trajectory.npy', np.asarray(
                [[p['x'], p['y'], p['yaw']] for p in path.points]))
    except Exception as error:
        result.update(success=False, failure_code=type(error).__name__+':'+str(error),
                      traceback=traceback.format_exc())
    finally:
        session.close()
    json_write(args.output/'result.json', result)
    print(json.dumps({key: result.get(key) for key in
                      ('map_id', 'query_id', 'success', 'failure_code', 'l2_points',
                       'l2_soft_target_offset_m', 'result_points')}), flush=True)


if __name__ == '__main__':
    main()
