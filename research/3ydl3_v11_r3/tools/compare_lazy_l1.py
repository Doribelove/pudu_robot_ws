"""Compare the staged L1 route search with frozen r0 candidates on all maps."""
import json
import argparse
import os
import tempfile
import time
from pathlib import Path

from compare_l1 import REFERENCE, Optimized, harness

ROOT = Path(__file__).resolve().parents[1]
BASE = Path('/home/robot/workspaces/3yd_v6_r0_20260924')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'results/lazy_l1_exact.json')
    args = parser.parse_args()
    os.environ['THREEYD_V6_LAZY_RESUME'] = '1'
    os.environ['THREEYD_L1_FIRST_BUDGET'] = '50000'
    os.environ['THREEYD_L1_ALTERNATE_BUDGET'] = '10000'
    os.environ['THREEYD_L1_LEGACY_COMPAT'] = '1'
    os.environ['THREEYD_L1_GOAL_POTENTIAL'] = '1'
    manifest = json.loads((REFERENCE / 'manifest.json').read_text())
    records = []
    for entry in manifest['maps']:
        source = REFERENCE / entry['inputs']
        descriptors = json.loads((source / 'queries.json').read_text())['scenarios']
        world = harness.L1World(source, harness.Tracker())
        with tempfile.TemporaryDirectory() as temporary:
            graph = Optimized(world, source / 'topology.json', Path(temporary) / 'graph')
            for descriptor in descriptors:
                query = harness.make_query(descriptor)
                expected = json.loads((BASE / 'results/l1_only' / entry['map_id'] / 'round_1' /
                                       f'{query.query_id}_measurement.json').read_text())
                first_started = time.perf_counter()
                first = graph.routes(query, limit=1)
                first_ms = (time.perf_counter() - first_started) * 1000
                staged_started = time.perf_counter()
                complete = graph.routes(query, limit=3)
                expansion_ms = (time.perf_counter() - staged_started) * 1000
                outputs = [graph.channel_plan(route) for route in complete]
                actual_hash = harness.canonical_hash(outputs)
                row = {
                    'map_id': entry['map_id'], 'query_id': query.query_id,
                    'first_ms': first_ms, 'expansion_ms': expansion_ms,
                    'first_phase_resumed': graph._probe_stats['first_phase_resumed'],
                    'candidate_count': len(complete),
                    'expected_hash': expected['output_sha256'], 'actual_hash': actual_hash,
                    'exact': expected['output_sha256'] == actual_hash,
                    'first_candidate_exact': bool(first and complete and first[0].signature == complete[0].signature),
                    'first_expansions': graph._probe_stats['phases']['first']['expanded'],
                    'alternate_expansions': graph._probe_stats['phases'].get('alternates', {}).get('expanded', 0),
                }
                records.append(row)
                graph._cache.clear(); graph.tasks.clear(); graph.contexts.clear()
        print(entry['map_id'], sum(row['exact'] for row in records if row['map_id'] == entry['map_id']), '/10', flush=True)
    output = args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps({'total': len(records), 'exact': sum(row['exact'] for row in records),
                                  'resumed': sum(row['first_phase_resumed'] for row in records),
                                  'queries': records}, ensure_ascii=False, indent=2) + '\n')
    assert len(records) == 50 and all(row['exact'] and row['first_candidate_exact'] and row['first_phase_resumed'] for row in records)


if __name__ == '__main__':
    main()
