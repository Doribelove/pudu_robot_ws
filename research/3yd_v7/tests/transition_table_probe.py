"""Check that an unmatched contact entry uses the original L1 transition."""

import json
import os
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from compare_l1 import REFERENCE, Optimized, harness  # noqa: E402


def main():
    entry = next(item for item in json.loads((REFERENCE/'manifest.json').read_text())['maps']
                 if item['map_id'] == 'pudu_wanda_3f')
    source = REFERENCE/entry['inputs']
    world = harness.L1World(source, harness.Tracker())
    descriptor = next(item for item in json.loads((source/'queries.json').read_text())['scenarios']
                      if item['id'] == 'Q01')
    query = harness.make_query(descriptor)
    with tempfile.TemporaryDirectory() as temporary:
        os.environ['THREEYD_V6_TRANSITION_TABLE'] = '0'
        original = Optimized(world, source/'topology.json', Path(temporary)/'control')
        os.environ['THREEYD_V6_TRANSITION_TABLE'] = '1'
        prepared = Optimized(world, source/'topology.json', Path(temporary)/'prepared')
        assert prepared._static_transitions
        for entries in prepared._static_transitions.values():
            for key, record in entries.items():
                entries[key] = ((float('inf'), float('inf')), *record[1:])
        expected = [original.task(route) for route in original.routes(query, limit=3)]
        actual = [prepared.task(route) for route in prepared.routes(query, limit=3)]
        assert actual == expected
        print(json.dumps({'candidate_count': len(actual), 'fallback_exact': True}))


if __name__ == '__main__':
    main()
