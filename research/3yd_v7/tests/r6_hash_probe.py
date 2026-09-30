"""Isolated W2 probe for the static-hash and bounded-potential invariants."""

import json
import os
import sys
import tempfile
from pathlib import Path

REFERENCE = Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_r13_astar_experiment_record_20260923')
sys.path.insert(0, str(REFERENCE/'tools'))
import worker as harness

for name in list(sys.modules):
    if name.startswith('three_yd_v5'):
        del sys.modules[name]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from three_yd_v5.topology_astar import AStarTopology


def main():
    os.environ['THREEYD_V6_TRANSITION_TABLE'] = '1'
    os.environ['THREEYD_V6_DIRECTED_POTENTIAL'] = '1'
    os.environ['THREEYD_V6_PRECOMPUTED_POTENTIAL'] = '1'
    os.environ['THREEYD_V6_SKIP_RELAXED_POTENTIAL'] = '1'
    manifest = json.loads((REFERENCE/'manifest.json').read_text())
    entry = next(item for item in manifest['maps']
                 if item['map_id'] == 'pudu_wanda_3f')
    source = REFERENCE/entry['inputs']
    tracker = harness.Tracker()
    world = harness.L1World(source, tracker)
    scenarios = json.loads((source/'queries.json').read_text())['scenarios']
    with tempfile.TemporaryDirectory() as directory:
        graph = AStarTopology(world, source/'topology.json', Path(directory))
        fingerprint_calls = tracker.query.get('map_fingerprint', {}).get('calls', 0)
        assert fingerprint_calls >= 1
        assert graph._directed_distances is not None
        assert graph._directed_distances.nbytes <= 16 * 2**20
        counts = []
        for query_id in ('Q01', 'Q03'):
            descriptor = next(item for item in scenarios if item['id'] == query_id)
            routes = graph.routes(harness.make_query(descriptor), limit=3)
            counts.append(len(routes))
        assert tracker.query.get('map_fingerprint', {}).get('calls', 0) == fingerprint_calls
    print(json.dumps({'candidate_counts': counts,
                      'query_time_map_fingerprint_calls': 0}))


if __name__ == '__main__':
    main()
