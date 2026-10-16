"""Standalone real-map L1 probe for the pytest subprocess."""

import json
import os
import sys
import tempfile
from pathlib import Path


REFERENCE = Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_r13_astar_experiment_record_20260923')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REFERENCE / 'tools'))
from worker import L1World, Tracker, make_query  # noqa: E402

for module_name in list(sys.modules):
    if module_name.startswith('three_yd_v5'):
        del sys.modules[module_name]
sys.path.insert(0, str(ROOT / 'src'))
from three_yd_v5.topology_astar import AStarTopology  # noqa: E402


def main():
    manifest = json.loads((REFERENCE / 'manifest.json').read_text())
    entry = next(item for item in manifest['maps']
                 if item['map_id'] == '3yd_synth_trizone_c_v2')
    source = REFERENCE / entry['inputs']
    descriptor = next(item for item in json.loads((source / 'queries.json').read_text())['scenarios']
                      if item['id'] == 'Q02')
    query = make_query(descriptor)
    os.environ.update(THREEYD_L1_LEGACY_COMPAT='1', THREEYD_V6_SINGLE_LANE_CERTIFICATE='1',
                      THREEYD_V6_LAZY_RESUME='1', THREEYD_L1_GOAL_POTENTIAL='1',
                      THREEYD_L1_ALTERNATE_BUDGET='10000', THREEYD_L1_FIRST_BUDGET='50000')
    with tempfile.TemporaryDirectory() as temp:
        graph = AStarTopology(L1World(source, Tracker()), source / 'topology.json', Path(temp))
        first = graph.routes(query, limit=1)
        complete = graph.routes(query, limit=3)
        normal = dict(graph._probe_stats)
        graph._cache.clear()
        graph._first_phase_cache.clear()
        os.environ['THREEYD_L1_FIRST_BUDGET'] = '1'
        recovered = graph.routes(query, limit=3)
        budget = dict(graph._probe_stats)
    assert len(first) == len(complete) == len(recovered) == 1
    print(json.dumps({
        'candidate_count': len(complete),
        'same_signature': first[0].signature == complete[0].signature == recovered[0].signature,
        'first_phase_resumed': normal['first_phase_resumed'],
        'certificate': normal['single_lane_candidate_certificate'],
        'alternate_expansions': normal['phases'].get('alternates', {}).get('expanded', 0),
        'legacy_on_normal': normal['legacy_compat_candidate_count'],
        'legacy_on_budget_loss': budget['legacy_compat_candidate_count'],
    }))


if __name__ == '__main__':
    main()
