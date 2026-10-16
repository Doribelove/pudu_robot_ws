"""Original seed+three shared tasks; B/P needs its own two-map benefit gate."""
import argparse
from pathlib import Path
from compare_v9 import read, summary
from compare_v9_review import load, save, sha
from compare_v9_r9 import exact_compare, compare
from review_v9_r9_diagnostic import source_proof, ROOT, MAPS


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-root', type=Path, required=True)
    args = parser.parse_args()
    report = {'source_cycle': 1, 'version': '3YD-V9-r9', 'candidate': 'P', 'diagnostic_only': True,
              'profiler': False, 'adopted': False, 'maps': {}}
    for mid in MAPS:
        original = ROOT / 'datasets/v9_shared_segments_v2' / (mid + '.json')
        definition = load(original)
        assert not definition['fixed10'] and len(definition['queries']) == 4
        assert sha(Path(definition['source_path'])) == definition['source_sha256']
        ids = [q['id'] for q in definition['queries']]
        assert len(set(ids)) == 4
        info = {'arms': {}, 'B_P_relative_to_A': {}}
        for arm in ('A', 'ABP'):
            directories = {mode: args.results_root.resolve() / 'shared' / mode / arm / mid for mode in ('control', 'P')}
            data = {mode: read(directory) for mode, directory in directories.items()}
            evidence = {}
            for mode, directory in directories.items():
                protocol = load(directory / 'protocol.json')
                assert protocol['queries_sha256'] == sha(original)
                assert protocol['workload'] == 'fixed' and protocol['repeats'] == 1
                assert [row['query_id'] for row in data[mode]] == ids
                for row in data[mode]:
                    assert not row['fullroute'].get('reused') and not row['task_progress'].get('reused')
                    assert row['r9_record_storage']['internal_points_packed'] == (mode == 'P')
                evidence[mode] = source_proof(directory, mode, arm=arm)
            parity = exact_compare(directories['control'], directories['P'])
            groups = {mode: {'initial': summary(rows[:1]), 'new_tasks': summary(rows[1:])} for mode, rows in data.items()}
            info['arms'][arm] = {'groups': groups, 'exact_parity': parity, 'source_native_cleanup': evidence}
        for mode in ('control', 'P'):
            a = args.results_root.resolve() / 'shared' / mode / 'A' / mid
            b = args.results_root.resolve() / 'shared' / mode / 'ABP' / mid
            quality = compare(a, b)
            assert quality['quality_gate'] and quality['upstream_gate']
            before = info['arms']['A']['groups'][mode]['new_tasks']['metrics']['e2e_s']['median']
            after = info['arms']['ABP']['groups'][mode]['new_tasks']['metrics']['e2e_s']['median']
            change = 100 * (after / before - 1)
            info['B_P_relative_to_A'][mode] = {'median_change_pct': change, 'benefit_at_least3': change <= -3,
                                              'quality_and_upstream': quality}
        report['maps'][mid] = info
    report['strict_total'] = 32
    report['B_P_two_map_benefit_pass'] = all(info['B_P_relative_to_A']['P']['benefit_at_least3'] for info in report['maps'].values())
    report['B_P_default_disabled_pending_full_review'] = True
    save(ROOT / 'report/SOURCE1_SHARED_REVIEW.json', report)
    print('SOURCE1 ORIGINAL SHARED32 STRICT/EXACT; CURRENT SOURCE/NATIVE/CLEANUP PASS')
    for mid, info in report['maps'].items():
        print(mid, {mode: result['median_change_pct'] for mode, result in info['B_P_relative_to_A'].items()})


if __name__ == '__main__':
    main()
