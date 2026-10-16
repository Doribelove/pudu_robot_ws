"""One fresh-process V7 map-only L1 benchmark for ten registered queries."""
import argparse
import gc
import hashlib
import json
import os
import resource
import time
from pathlib import Path

import numpy as np
import psutil

from arena_evaluation.planner_benchmark.models import Query
from arena_evaluation.semantic_map import canonical_hash
from arena_3d_v1.semantic_world import json_write
from three_yd_v5.free_space_topology import FreeSpaceTopology, load_map_only_world

ROOT = Path(__file__).resolve().parents[1]


def memory():
    info = psutil.Process().memory_full_info()
    return {'rss_bytes': int(info.rss), 'pss_bytes': int(getattr(info, 'pss', 0)),
            'peak_rss_bytes': int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    registry = json.loads((ROOT / 'inputs/registry.json').read_text())
    if args.map_id not in registry['maps']:
        raise ValueError('MAP_ID_NOT_REGISTERED')
    entry = registry['maps'][args.map_id]
    input_dir = ROOT / 'inputs' / args.map_id
    for filename, expected in entry['sha256'].items():
        actual = hashlib.sha256((input_dir / filename).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError('MAP_OR_QUERY_INPUT_CHANGED:' + filename)
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    if 12 in os.sched_getaffinity(0):
        os.sched_setaffinity(0, {12})
    os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
    before = memory()
    started = time.perf_counter()
    world = load_map_only_world(input_dir / 'map.yaml', args.output / 'generated')
    world_ms = (time.perf_counter()-started)*1000
    started = time.perf_counter()
    topology = FreeSpaceTopology(world, args.output / 'graph_cache')
    graph_ms = (time.perf_counter()-started)*1000
    after_init = memory()
    scenarios = json.loads((input_dir / 'queries.json').read_text())['queries']
    assert len(scenarios) == 10
    rows = []
    for item in scenarios:
        gc.collect()
        query = Query(item['id'], tuple(item['start']), tuple(item['goal']))
        pre = memory()
        tick = time.perf_counter()
        cpu = time.process_time()
        try:
            routes = topology.routes(query, limit=3)
            plans = [topology.channel_plan(route) for route in routes]
            status, error = True, None
        except (ValueError, RuntimeError) as exc:
            routes = []
            plans = []
            status, error = False, str(exc)
        wall_ms = (time.perf_counter()-tick)*1000
        cpu_ms = (time.process_time()-cpu)*1000
        # Validation is deliberately outside the measured L1 call, matching
        # the older L1-only protocol.  It still determines output validity.
        if status:
            try:
                for plan in plans:
                    line = np.asarray(plan['right_reference_polyline'], float)
                    if len(line) < 2 or not all(topology._clear_line(a, b) for a, b in zip(line, line[1:])):
                        raise RuntimeError('V7_L1_GUIDE_NOT_CLEAR')
                    if plan['mode'] != 'free_space' or plan['right_side_rule'] is not None:
                        raise RuntimeError('V7_L1_NEUTRAL_MODE_VIOLATION')
            except (ValueError, RuntimeError) as exc:
                status, error = False, str(exc)
        post = memory()
        json_write(args.output / (item['id'] + '_output.json'), {'candidates': plans})
        rows.append({'query_id': item['id'], 'success': status, 'failure': error,
                     'wall_ms': wall_ms, 'cpu_ms': cpu_ms,
                     'route_count': len(routes), 'guide_points': 0 if not plans else len(plans[0]['right_reference_polyline']),
                     'guide_length_m': None if not plans else float(np.linalg.norm(np.diff(
                         np.asarray(plans[0]['right_reference_polyline']), axis=0), axis=1).sum()),
                     'diagnostics': dict(topology._probe_stats),
                     'output_sha256': canonical_hash(plans),
                     'memory_before': pre, 'memory_after': post})
    result = {'version': '3YD-V7-r1', 'mode': 'free_space', 'map_id': args.map_id,
              'optimization_flags': {k: os.environ.get(k, '1') for k in
                  ('THREEYD_V7_GOAL_STOP', 'THREEYD_V7_COMPACT_LEGACY')},
              'query_ids': [r['query_id'] for r in rows], 'map_hash': world.map.sha256,
              'input_sha256': entry['sha256'], 'cpu_affinity': sorted(os.sched_getaffinity(0)),
              'source_sha256': {
                  path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                  for path in (ROOT/'src/three_yd_v5/free_space_topology.py', Path(__file__))},
              'metrics_scope': 'L1 only: world adapter + coarse topology initialization separately; ten route+plan calls, no L2/L3 or ROS',
              'world_init_ms': world_ms, 'l1_graph_init_ms': graph_ms,
              'memory_before_init': before, 'memory_after_init': after_init,
              'memory_final': memory(), 'queries': rows}
    json_write(args.output / 'runs.json', result)
    print(json.dumps({'map_id': args.map_id, 'success': sum(x['success'] for x in rows),
                      'query_count': len(rows), 'query_median_ms': float(np.median([x['wall_ms'] for x in rows])),
                      'query_p95_ms': float(np.quantile([x['wall_ms'] for x in rows], .95)),
                      'peak_rss_mib': result['memory_final']['peak_rss_bytes']/2**20,
                      'output': str(args.output / 'runs.json')}))


if __name__ == '__main__':
    main()
