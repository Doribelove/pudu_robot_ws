"""Review all original fixed10x3 requests and the predeclared R9 gates."""
import argparse
from pathlib import Path
from compare_v9 import read, pair, parts, summary
from compare_v9_review import load, save
from compare_v9_r9 import exact_compare
from review_v9_r9_diagnostic import source_proof, ROOT, MAPS


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-root', type=Path, required=True)
    args = parser.parse_args()
    report = {'source_cycle': 1, 'version': '3YD-V9-r9', 'candidate': 'P',
              'diagnostic_only': False, 'profiler': False, 'adopted': False, 'maps': {}}
    for mid in MAPS:
        directories = {mode: args.results_root.resolve() / 'fixed' / mode / mid for mode in ('control', 'P')}
        data = {mode: read(directory) for mode, directory in directories.items()}
        evidence = {}
        for mode, directory in directories.items():
            protocol = load(directory / 'protocol.json')
            assert protocol['workload'] == 'fixed' and not protocol['diagnostic_subset'] and protocol['repeats'] == 3
            queries = load(Path(protocol['query_file']))
            queries = queries.get('scenarios', queries.get('queries'))
            ids = [query['id'] for query in queries]
            assert len(ids) == len(set(ids)) == 10 and len(data[mode]) == 30
            assert set(ids) == {row['query_id'] for row in data[mode]}
            for identifier in ids:
                assert sorted(row['repeat'] for row in data[mode] if row['query_id'] == identifier) == [1, 2, 3]
            for row in data[mode]:
                storage = row['r9_record_storage']
                assert storage['mode_enabled'] == storage['internal_points_packed'] == (mode == 'P')
                assert not storage['audit_or_ACK_results_cached']
                if mode == 'P':
                    assert storage['float_buffers_immutable']
            evidence[mode] = source_proof(directory, mode)
        parity = exact_compare(directories['control'], directories['P'])
        parent = Path(load(ROOT / 'PARENT.json')['parent_workspace']) / 'results/formal_source4/fixed/OGR' / mid
        inherited = pair(parent, directories['control'], strict=True)
        assert len(inherited) == 30
        groups = {mode: {key: summary(value) for key, value in parts(rows, 'fixed').items()}
                  for mode, rows in data.items()}
        before, after = groups['control'], groups['P']
        changes = {
            'first_median': 100 * (after['first']['metrics']['e2e_s']['median'] / before['first']['metrics']['e2e_s']['median'] - 1),
            'repeat_median': 100 * (after['repeat']['metrics']['e2e_s']['median'] / before['repeat']['metrics']['e2e_s']['median'] - 1),
            'first_p95': 100 * (after['first']['metrics']['e2e_s']['p95'] / before['first']['metrics']['e2e_s']['p95'] - 1),
            'repeat_p95': 100 * (after['repeat']['metrics']['e2e_s']['p95'] / before['repeat']['metrics']['e2e_s']['p95'] - 1),
            'tree_rss_all_median': 100 * (after['all']['metrics']['tree_rss_gib']['median'] / before['all']['metrics']['tree_rss_gib']['median'] - 1),
        }
        gates = {'first_benefit_at_least3': changes['first_median'] <= -3,
                 'repeat_median_not_over5': changes['repeat_median'] <= 5,
                 'first_p95_not_over5': changes['first_p95'] <= 5,
                 'repeat_p95_not_over5': changes['repeat_p95'] <= 5,
                 'rss_not_over3': changes['tree_rss_all_median'] <= 3,
                 'strict_and_exact': True}
        report['maps'][mid] = {'groups': groups, 'exact_parity': parity, 'parent_control_exact': inherited,
                               'source_native_cleanup': evidence, 'change_pct': changes, 'gates': gates}
    report['strict_total'] = 120
    report['both_fixed_first_gates_pass'] = all(info['gates']['first_benefit_at_least3'] for info in report['maps'].values())
    report['remaining_fixed_gates_pass'] = all(all(v for k, v in info['gates'].items() if k != 'first_benefit_at_least3') for info in report['maps'].values())
    report['cold_not_run'] = True
    report['full_adoption_still_requires'] = ['first_or_cold', 'full_progress_and_original_AS_recovery', 'events', 'contemporary_AS_fixed', 'cold', 'default_entry', 'all_protected_and_resource']
    save(ROOT / 'report/SOURCE1_FIXED_REVIEW.json', report)
    print('SOURCE1 FIXED: 120/120 STRICT; PHYSICAL OUTPUT/MAP/SEALED PARENT EXACT')
    for mid, info in report['maps'].items():
        print(mid, info['change_pct'], info['gates'])


if __name__ == '__main__':
    main()
