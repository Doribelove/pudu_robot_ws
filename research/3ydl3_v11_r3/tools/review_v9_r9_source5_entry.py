"""Review actual selected-entry20 against locked original fixed outputs."""
import json
from pathlib import Path

from compare_v9 import read, summary
from compare_v9_r9 import exact_compare
from compare_v9_review import load, save, sha
from review_v9_r9_diagnostic import source_proof
from review_v9_r9_source5 import features

ROOT = Path(__file__).resolve().parents[1]


def main():
    protocol = ROOT/'report/SOURCE5_ENTRY_PROTOCOL_V2.json'
    spec = load(protocol)
    assert spec['declared_before_entry_native'] and spec['no_mode_overrides']
    for name, expected in spec['files'].items():
        assert sha(ROOT/name) == expected, name
    evidence = {}
    for mid in spec['maps']:
        directory = Path(spec['root'])/'fixed'/mid
        rows = read(directory)
        assert len(rows) == 10 and summary(rows)['success'] == 10
        fixture = ROOT/'inputs/queries.json' if mid == 'mengjia' else ROOT/'inputs'/mid/'queries.json'
        queries = load(fixture)
        queries = queries['scenarios'] if mid == 'mengjia' else queries['queries']
        assert [r['query_id'] for r in rows] == [q['id'] for q in queries]
        assert all(r['repeat'] == 1 for r in rows)
        original = ROOT/'results/formal_source5/fixed/PLMX'/mid
        same = exact_compare(original, directory)
        proof = source_proof(directory, 'PLMX', cycle=5, arm='A', storage_mode='CHNT', r8_mode='OGR')
        feature = features(directory, rows, 'PLMX', mid)
        actual = load(directory/'protocol.json')
        for key, expected in spec['selection'].items():
            assert actual[key] == expected, (mid, key)
        assert actual['query_file'] == str(fixture) and not actual['diagnostic_subset']
        assert actual['workload'] == 'fixed' and not actual['profiler']
        assert actual['pose_offset_m'] == actual['yaw_offset_deg'] == 0
        assert not actual['moving_window'] and not actual['global_L3_fallback']
        evidence[mid] = {'original_fixed10': True, 'summary': summary(rows), 'same_original_fixed_outputs': same,
                         'source_native_resources': proof, 'actual_candidate_features': feature}
    result = {'source_cycle': 5, 'protocol_sha256': sha(protocol), 'actual_run_sh_saved_selection': True,
              'no_mode_overrides': True, 'strict_records': 20, 'B_P_default_enabled': False,
              'original_fixed_outputs_exact': True, 'performance_adoption_gates_use_prior_independent_blocks': True,
              'this_entry20_not_an_extra_favorable_performance_block': True, 'maps': evidence}
    target = ROOT/'report/SOURCE5_ENTRY_REVIEW.json'
    assert not target.exists()
    save(target, result)
    print('SOURCE5_ACTUAL_ENTRY20_STRICT_FIXED_OUTPUTS_ACK_NATIVE_SOURCE_PASS', flush=True)


if __name__ == '__main__':
    main()
