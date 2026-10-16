"""V8-r5 fixed-query full benchmark of the free-space branch."""
if __name__ == '__main__':
    from owned_process import supervised_entry
    supervised_entry()

import argparse
import gc
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import time
import traceback

import numpy as np
from benchmark import Memory
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.path_audit import PathAuditor
from arena_evaluation.planner_benchmark.models import Query
from arena_3d_v1.multires_grid import GridView
from arena_3d_v1.semantic_world import json_write
from three_yd_v5.controller import V5Controller
from three_yd_v5.free_space_topology import FreeSpaceTopology, load_map_only_world
from three_yd_v5.performance_session import PerformanceSession
from three_yd_v5.recovery import V5Runner
from three_yd_v5.r10_connector import stats as connector_stats
from run_wanda3w import QueryBudgetExceeded, deadline_handler, json_native, process_memory

ROOT = Path(__file__).resolve().parents[1]


def registered_input(map_id):
    entry = json.loads((ROOT/'inputs/registry.json').read_text())['maps'][map_id]
    directory = ROOT/'inputs'/map_id
    for name, expected in entry['sha256'].items():
        if hashlib.sha256((directory/name).read_bytes()).hexdigest() != expected:
            raise ValueError('MAP_OR_QUERY_INPUT_CHANGED:'+name)
    rows = json.loads((directory/'queries.json').read_text())['queries']
    if [r['id'] for r in rows] != entry['query_ids'] or len(rows) != 10:
        raise ValueError('FIXED_TEN_QUERIES_CHANGED')
    return directory, entry, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--domain', type=int, default=227)
    parser.add_argument('--guard', type=int, default=90)
    parser.add_argument('--query-file', type=Path, help='Verified moving-start derived dataset, not fixed-ten replacement')
    parser.add_argument('--profile', action='store_true', help='Diagnostic cProfile; excluded from formal results')
    parser.add_argument('--queries', nargs='+', help='Diagnostic subset only; never a formal fixed-ten result')
    args = parser.parse_args()
    if args.repeats < 1 or not 0 <= args.domain <= 232:
        raise ValueError('INVALID_EXPERIMENT_CONFIGURATION')
    directory, entry, scenarios = registered_input(args.map_id)
    if args.query_file:
        from moving_starts import load
        scenarios=load(args.query_file,args.map_id)
    if args.queries:
        if not set(args.queries)<=set(entry['query_ids']): raise ValueError('UNKNOWN_DIAGNOSTIC_QUERY')
        scenarios=[q for q in scenarios if q['id'] in args.queries]
    args.output.mkdir(parents=True, exist_ok=False)
    signal.signal(signal.SIGALRM, deadline_handler)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    started = time.monotonic()
    with Memory() as memory:
        world = load_map_only_world(directory/'map.yaml', args.output/'world_cache')
        topology = FreeSpaceTopology(world, args.output/'graph_cache')
        views = {3: GridView(world, 3), 1: GridView(world, 1)}
    json_write(args.output/'preparation.json', {
        'wall_ms': (time.monotonic()-started)*1000,
        'rss_peak_bytes': memory.peak, 'excluded_from_query_timing': True})
    context = runtime.MapContext(args.map_id, world.map, world.safe,
        world.map.distance_m, world.map.sha256, world.map.sha256, directory/'map.yaml')
    auditor = PathAuditor(context, source_commit='3YD-V8-r7-fullroute-free-space')
    spec = runtime.backend_availability()['hybrid_astar']
    session = PerformanceSession(context, args.output/'smac',
        map_yaml=directory/'map.yaml', local_mask_updates=True,
        optimization_profile='v7_candidate', smac_parameter_profile='lighter_smoother',
        optimization_stage='step3_delta_map', enable_mask_reuse_noop=True,
        planner_parameter_overrides={'angle_quantization_bins': 48},
        costmap_ack_timeout_s=3., tile_interval_s=.015, enable_semantic_repair=True)
    session.local_map_update_strategy = 'roi_ack'
    session.full_grid_settle_cycles = 0
    json_write(args.output/'protocol.json', {
        'fullroute_reuse': os.environ.get('THREEYD_V8_FULLROUTE_REUSE','0')=='1',
        'version': '3YD-V8-r7-fullroute', 'parent': '3YD-V8-r5', 'r6_ablation': os.environ.get('THREEYD_V8_R6_MODE','control'), 'r5_repair': os.environ.get('THREEYD_V8_R5_REPAIR','0'), 'r4_ablation': os.environ.get('THREEYD_V8_R4_MODE','ABC'), 'r3_ablation': os.environ.get('THREEYD_V8_R3_MODE','AB'), 'r2_ablation': os.environ.get('THREEYD_V8_R2_MODE','AB'), 'mode': 'free_space',
        'formal_benchmark': not bool(args.queries or args.profile or args.query_file), 'firstplan_mode':os.environ.get('THREEYD_V8_FIRST_MODE','control'), 'derived_query_file':str(args.query_file) if args.query_file else None, 'derived_query_sha256':hashlib.sha256(args.query_file.read_bytes()).hexdigest() if args.query_file else None, 'profiler': args.profile, 'map_id': args.map_id,
        'map_file': str(directory/'map.yaml'), 'map_sha256': world.map.sha256,
        'input_sha256': entry['sha256'], 'query_ids': [q['id'] for q in scenarios],
        'diagnostic_subset': bool(args.queries),
        'repeats': args.repeats, 'schedule': 'each fixed query followed by its repeats',
        'CPU': sorted(os.sched_getaffinity(0)), 'vehicle': world.vehicle,
        'L2_resolution_m': .15, 'L2_state': ['x', 'y'], 'L2_soft_offset_m': 0.,
        'L1_scope_half_width_m': 2., 'L3_resolution_m': .05,
        'L3_normal_radius_m': .8, 'L3_corner_radius_m': 1.2,
        'max_search_cells': V5Controller.max_search_cells,
        'max_map_fraction': V5Controller.max_map_fraction,
        'global_L3_fallback': False, 'unknown_is_collision': True,
        'guard_s': args.guard, 'dynamic_validation': False, 'real_robot_execution': False,
        'memory_profile': os.environ.get('THREEYD_R9_MEMORY_PROFILE'),
        'r10_ablation': os.environ.get('THREEYD_R10_MODE'),
        'r11_ablation': os.environ.get('THREEYD_R11_MODE'),
        'costmap_ack_timeout_s': 3., 'angle_quantization_bins': 48,
        'timing': 'actual per-call clocks; preparation/startup/export excluded; L3 includes adapter',
        'memory': 'query tree sampled peak RSS; component PSS snapshots/peaks separately',
        'no_semantic_rules': True, 'L1_frozen': str(ROOT/'L1_LOCK.json'),
        'r1_repair': ['bounded CSC/CCC local steering fit after old repair fails',
                      'iteration-limit-only 4-section recovery with unchanged mask and soft cost',
                      '8s shared recovery deadline; all sections and stitched whole path audited'],
        'legacy_scenario_preferences': 'ignored in free_space; retained in immutable query file'})
    snapshot = args.output/'source_snapshot'
    snapshot.mkdir()
    sources = [*ROOT.glob('env.bash'), *(ROOT/'src').rglob('*.py'), *(ROOT/'tools').glob('*.py')]
    for source in sources:
        shutil.copy2(source, snapshot/str(source.relative_to(ROOT)).replace('/', '_'))
    json_write(snapshot/'hashes.json', {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    rows = []
    try:
        started = time.monotonic()
        session.start()
        json_write(args.output/'session_startup.json', {'wall_ms': (time.monotonic()-started)*1000})
        from native_snapshot import record
        record(args.output)
        for item in scenarios:
            query = Query(item['id'], tuple(item['start']), tuple(item['goal']), seed=20260929)
            for repeat in range(1, args.repeats+1):
                tag = f'{query.query_id}_{repeat}' if 'moving_start' not in item else f"{query.query_id}_{item['moving_start']['step']}_{repeat}"
                case = args.output/'cases'/tag
                case.mkdir(parents=True)
                controller = route = path = None
                outcome = {}
                row = {'version': '3YD-V8-r7-fullroute', 'mode': 'free_space', 'map_id': args.map_id,
                    'moving_start':item.get('moving_start'), 'query_id': query.query_id, 'repeat': repeat, 'success': False,
                    'geometry_success': False, 'start': query.start, 'goal': query.goal}
                before_connector = connector_stats()
                before_tiles = len(getattr(session, 'r10_tile_events', []))
                started = time.monotonic()
                profiler = None
                if args.profile:
                    import cProfile
                    profiler = cProfile.Profile()
                cpu = time.process_time()
                with Memory() as memory:
                    try:
                        signal.alarm(args.guard)
                        session.reset_query_state(tag, restore_base_map=False)
                        if profiler: profiler.enable()
                        controller, route, outcome = V5Runner(world, topology,
                            args.output/'l2_cache', views, dynamic=None,
                            controller_class=V5Controller, max_region_routes=1).plan(
                                query, session, auditor, spec)
                        path = outcome.pop('result', None)
                        row.update(success=bool(outcome.get('success')),
                            geometry_success=bool(outcome.get('geometry_success')),
                            failure_code=outcome.get('failure_code', ''))
                    except QueryBudgetExceeded:
                        row['failure_code'] = 'QUERY_WALL_BUDGET'
                        outcome = {'success': False, 'failure_code': 'QUERY_WALL_BUDGET',
                                   'failure_is_physical_blockage': False}
                    except Exception as error:
                        row['failure_code'] = type(error).__name__+':'+str(error)
                        (case/'exception.txt').write_text(traceback.format_exc())
                    finally:
                        signal.alarm(0)
                        if profiler:
                            profiler.disable()
                            profiler.dump_stats(str(case/'planning.prof'))
                row.update(wall_ms=(time.monotonic()-started)*1000,
                    cpu_ms=(time.process_time()-cpu)*1000, tree_cpu_ms=memory.tree_cpu_ms,
                    rss_peak_bytes=memory.peak, l1_valid=route is not None,
                    l2_valid=controller is not None and hasattr(controller, 'reference_plan'))
                if controller and hasattr(controller, 'initial_l2_result'):
                    hit = bool(controller.timing.get('r6_route_cache_hit'))
                    row['l2'] = {**controller.initial_l2_result.diagnostics,
                        'source_response_ms': controller.initial_l2_result.response_ms,
                        'response_ms': 0. if hit else controller.initial_l2_result.response_ms,
                        'reused_search_result': hit}
                    row['timing'] = dict(controller.timing)
                if controller and hasattr(controller, 'reference_plan'):
                    np.save(case/'l2_reference_xy.npy', controller.reference_plan.world_xy)
                    np.save(case/'l2_raw_reference_xy.npy', controller.raw_reference_xy)
                    json_write(case/'l2_corners.json', controller.corner_records)
                    json_write(case/'l2_soft_target.json', {
                        'offset_m': controller.l2_guide_offset_info['straight_offset_m'],
                        'points': controller.l2_soft_target_xy.tolist(),
                        'corner_policy': controller.l2_guide_offset_info})
                    if controller._search_mask is not None:
                        np.savez_compressed(case/'l3_scope.npz', mask=controller._target_mask())
                if route:
                    json_write(case/'l1_output.json', topology.channel_plan(route))
                if path and path.points:
                    points = np.asarray([[p['x'], p['y'], p['yaw']] for p in path.points])
                    np.save(case/('path.npy' if row['success'] else 'rejected_candidate.npy'), points)
                    row['length_m'] = float(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1).sum())
                for field, source in [('audit', 'independent_audit'), ('constraints', 'constraint_audit'),
                    ('l3', 'diagnostics'), ('recovery', 'candidate_recovery'),
                    ('adapter', 'reference_diagnostics'), ('bounded_l3', 'bounded_l3'),
                    ('curvature_repair', 'curvature_repair'),
                    ('local_geometry_repair', 'local_geometry_repair'),
                    ('r1_segmented_recovery','r1_segmented_recovery'),
                    ('r3_early_sections','r3_early_sections')]:
                    row[field] = outcome.get(source, {})
                row['fullroute'] = outcome.get('fullroute', {})
                row['fullroute_cache'] = outcome.get('fullroute_cache', {})
                row['right_side_fraction'] = None
                row['right_metric'] = 'not applicable: no semantic right-side rule'
                row['memory_processes'] = process_memory()
                if os.environ.get('THREEYD_R6_INVENTORY')=='1':
                    from three_yd_v5.r11_memory import inventory
                    json_write(case/'array_inventory.json',inventory({'world':world,'map':world.map,'topology':topology,'views':views,'controller':controller,'session':session}))
                row['r11'] = {'mode': os.environ.get('THREEYD_R11_MODE')}
                after_connector = connector_stats()
                row['r10'] = {'connector_cache': after_connector,
                    'connector_delta': {k: after_connector[k]-before_connector[k]
                        for k in ('calls', 'hits', 'failure_hits', 'misses', 'evictions')},
                    'tile_updates': getattr(session, 'r10_tile_events', [])[before_tiles:]}
                if not controller and route:
                    failed = args.output/'l2_cache'/('failed_reference_'+route.signature[:12]+'.json')
                    if failed.exists():
                        row['timing'] = json.loads(failed.read_text())
                json_write(case/'outcome.json', outcome)
                rows.append(row)
                with (args.output/'runs.jsonl').open('a') as stream:
                    stream.write(json.dumps(json_native(row), allow_nan=False)+'\n')
                print(tag, row['success'], row.get('failure_code'), round(row['wall_ms']), flush=True)
                if controller:
                    controller.lifecycle.clear()
                session.active_controller = None
                controller = path = outcome = None
                gc.collect()
    finally:
        session.close()
    json_write(args.output/'summary.json', {'runs': len(rows),
        'strict_success': sum(r['success'] for r in rows),
        'geometry_success': sum(r['geometry_success'] for r in rows),
        'failures': [r.get('failure_code') for r in rows if not r['success']]})


if __name__ == '__main__':
    main()
