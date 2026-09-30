"""Paired L1 first-route comparison for the direction-aware goal potential."""

import argparse
import json
import os
import time
from pathlib import Path

from compare_l1 import REFERENCE, Optimized, harness


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--queries', nargs='+', required=True)
    parser.add_argument('--rounds', type=int, default=3)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ['THREEYD_V6_TRANSITION_TABLE'] = '1'
    os.environ['THREEYD_L1_FIRST_BUDGET'] = '50000'
    os.environ['THREEYD_L1_ALTERNATE_BUDGET'] = '10000'
    entry = next(item for item in json.loads((REFERENCE/'manifest.json').read_text())['maps']
                 if item['map_id'] == args.map_id)
    cpu = json.loads((REFERENCE/'manifest.json').read_text())['cpu']
    if cpu in os.sched_getaffinity(0):
        os.sched_setaffinity(0, {cpu})
    source = REFERENCE/entry['inputs']
    world = harness.L1World(source, harness.Tracker())
    graph_root = args.output.parent/'probe_graph'
    graph_root.mkdir(exist_ok=True)
    graph = Optimized(world, source/'topology.json', graph_root)
    scenarios = {item['id']: item for item in json.loads((source/'queries.json').read_text())['scenarios']}
    records = []
    for query_id in args.queries:
        query = harness.make_query(scenarios[query_id])
        for round_id in range(args.rounds):
            for directed in ((False, True) if round_id % 2 == 0 else (True, False)):
                os.environ['THREEYD_V6_DIRECTED_POTENTIAL'] = '1' if directed else '0'
                graph._cache.clear()
                graph._first_phase_cache.clear()
                graph.tasks.clear()
                graph.contexts.clear()
                before = time.perf_counter()
                routes = graph.routes(query, limit=1)
                elapsed = (time.perf_counter()-before)*1000
                probe = graph._probe_stats
                record = dict(map_id=args.map_id, query_id=query_id, round=round_id+1,
                              directed=directed, wall_ms=elapsed,
                              candidate_count=len(routes),
                              output_sha256=harness.canonical_hash(
                                  [graph.task(route) for route in routes]),
                              first_expanded=probe['phases']['first']['expanded'],
                              first_nodes=probe['phases']['first']['allocated_nodes'],
                              goal_potential_ms=probe['goal_potential_ms'])
                records.append(record)
                print(query_id, round_id+1, 'directed' if directed else 'control',
                      round(elapsed, 2), 'ms', record['first_expanded'], 'expanded', flush=True)
    harness.save(args.output, dict(schema='3YD-V6-r5-directed-potential-probe-v1',
                                   map_id=args.map_id, CPU=cpu,
                                   records=records))


if __name__ == '__main__':
    main()
