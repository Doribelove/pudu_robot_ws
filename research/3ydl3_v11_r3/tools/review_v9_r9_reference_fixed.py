"""Original fixed fixtures, exact data outputs, contemporary AS and subgroup gates."""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, cbook
from compare_v9 import read, pair, parts, summary
from compare_v9_review import load, save, sha
from compare_v9_r9 import compare, exact_compare
from review_v9_r9_source2 import pose, changes, fixed_gates
from review_v9_r9_source4 import features
from review_v9_r9_full_progress import source
from plot_v9_r9_source2 import setup, FONT, MAPS

ROOT = Path(__file__).resolve().parents[1]
MODES = ('AS', 'control', 'PLM', 'PLMU')
CANDIDATES = ('PLM', 'PLMU')


def plot(root, report_path):
    setup()
    fig, axes = plt.subplots(2, 3, figsize=(18, 9))
    data = {'source_cycle': 4, 'font': FONT, 'font_name': 'SimSun', 'linear_axes': True,
            'same_query_repeats_are_related': True, 'review_sha256': sha(report_path),
            'box': 'Q1 to Q3; median; furthest raw points within1.5IQR; outliers displayed', 'groups': []}
    for rr, (mid, caption) in enumerate(MAPS):
        groups = {mode: parts(read(root/'fixed'/mode/mid), 'fixed') for mode in MODES}
        for cc, (split, metric, divisor, unit, title) in enumerate([
            ('first', 'wall_ms', 1000, 's', '首次端到端耗时'),
            ('repeat', 'wall_ms', 1000, 's', '重复端到端耗时'),
            ('all', 'rss_peak_bytes', 2**30, 'GiB', '整套规划进程树峰值RSS')]):
            values = [[float(r[metric])/divisor for r in groups[mode][split]] for mode in MODES]
            ax = axes[rr, cc]
            ax.boxplot(values, labels=[f'{mode}\nn={len(raw)}' for mode, raw in zip(MODES, values)], whis=1.5, showfliers=True)
            ax.set_title(caption+'\n'+title+' / '+unit)
            ax.grid(axis='y', alpha=.25)
            for mode, raw in zip(MODES, values):
                s = cbook.boxplot_stats(raw, whis=1.5)[0]
                data['groups'].append({'map_id': mid, 'mode': mode, 'split': split, 'metric': metric,
                    'unit': unit, 'n': len(raw), 'raw': raw,
                    'quartiles': [float(s['q1']), float(s['med']), float(s['q3'])],
                    'whiskers': [float(s['whislo']), float(s['whishi'])], 'outliers': s['fliers'].tolist()})
    fig.suptitle('V9-r9 source4：原AS与当前候选同批固定10组×3', fontsize=20)
    fig.text(.5, .03, '箱体Q1至Q3；中线中位数；须为1.5×IQR内最远原始值；点为离群点；线性坐标。\n'
        '首次每组n=10，重复n=20且来自同一查询；RSS每组n=30，含Python及ROS后代20ms采样峰值；严格结果另表。', ha='center')
    fig.tight_layout(rect=(0, .10, 1, .93))
    fig.savefig(ROOT/'report/reference_fixed_source4.png', dpi=160)
    plt.close(fig)
    save(ROOT/'report/REFERENCE_FIXED_SOURCE4_BOXPLOT_DATA.json', data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--protocol', type=Path, default=ROOT/'report/SOURCE4_REFERENCE_FIXED_PROTOCOL.json')
    parser.add_argument('--report-path', type=Path, default=ROOT/'report/SOURCE4_REFERENCE_FIXED_REVIEW.json')
    args = parser.parse_args()
    assert not args.report_path.exists(), 'Keep reviewed evidence immutable'
    spec = load(args.protocol)
    root = Path(spec['root'])
    assert spec['source_cycle'] == 4 and spec['declared_before_reference_fixed_native']
    assert tuple(spec['modes']) == MODES and spec['expected_records'] == 240
    assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json') == spec['planner_lock_sha256']
    assert sha(ROOT/'report/SOURCE4_COLD_REVIEW_LOCK.json') == spec['cold_review_lock_sha256']
    for name, expected in spec['analysis_tools'].items():
        assert sha(ROOT/name) == expected, name
    for name, expected in spec['prior_negative_evidence'].items():
        assert sha(ROOT/'report'/name) == expected, name
    cold = load(ROOT/'report/SOURCE4_COLD_REVIEW.json')
    assert cold['strict_total'] == 24
    parent = Path(load(ROOT/'PARENT.json')['parent_workspace'])
    report = {'version': '3YD-V9-r9', 'source_cycle': 4, 'profiler': False, 'storage_ledger': False,
        'diagnostic_subset': False, 'adopted': False, 'goal_remains_active': True,
        'historical_performance_pooled': False, 'current_default_mode': 'control', 'B_P_default_enabled': False,
        'predeclared_protocol_sha256': sha(args.protocol), 'results_root': str(root),
        'prior_negative_evidence': spec['prior_negative_evidence'], 'maps': {}}
    count = 0
    for mid, query in spec['original_queries'].items():
        qfile = Path(query['source'])
        assert sha(qfile) == query['sha256']
        definition = load(qfile)
        queries = definition.get('scenarios', definition.get('queries'))
        assert len(queries) == len({q['id'] for q in queries}) == 10
        assert [q['id'] for q in queries] == query['ids_in_original_order']
        directories = {mode: root/'fixed'/mode/mid for mode in MODES}
        records = {mode: read(directory) for mode, directory in directories.items()}
        evidence, observed, preserved = {}, {}, {}
        for mode, directory in directories.items():
            protocol = load(directory/'protocol.json')
            assert protocol['workload'] == 'fixed' and protocol['repeats'] == 3
            assert not protocol['diagnostic_subset'] and not protocol['profiler']
            assert not protocol.get('storage_ledger', False)
            assert protocol['pose_offset_m'] == protocol['yaw_offset_deg'] == 0
            assert Path(protocol['query_file']) == qfile and protocol['queries_sha256'] == query['sha256']
            rows = records[mode]
            assert len(rows) == 30
            assert [(r['query_id'], r['repeat']) for r in rows] == [(q['id'], repeat) for q in queries for repeat in (1, 2, 3)]
            for row, q in zip(rows, [q for q in queries for _ in range(3)]):
                assert row['start'] == pose(q['start']) and row['goal'] == pose(q['goal'])
            evidence[mode] = source(directory, mode, rows, 4)
            observed[mode] = features(rows, 'control' if mode == 'AS' else mode, mid)
            old = (parent/'results/reference_source4/fixed/AS'/mid if mode == 'AS' else
                   parent/'results/formal_source4/fixed/OGR'/mid if mode == 'control' else
                   ROOT/'results/formal_source4/fixed'/mode/mid)
            preserved[mode] = pair(old, directory, strict=True)
            assert len(preserved[mode]) == 30
            count += len(rows)
        groups = {mode: {key: summary(rows) for key, rows in parts(records[mode], 'fixed').items()} for mode in MODES}
        assert all(g['all']['success'] == 30 and g['first']['n'] == 10 and g['repeat']['n'] == 20 for g in groups.values())
        exact = {mode: exact_compare(directories['control'], directories[mode]) for mode in CANDIDATES}
        quality = {mode: compare(directories['AS'], directories[mode]) for mode in ('control', *CANDIDATES)}
        assert all(v['quality_gate'] and v['upstream_gate'] for v in quality.values())
        current, original = {}, {}
        for mode in CANDIDATES:
            delta = {g: changes(groups['control'][g], groups[mode][g]) for g in ('all', 'first', 'repeat')}
            current[mode] = {'original_fixed_gates': fixed_gates(groups, mode), 'change_pct_by_group': delta,
                'all_memory_subgroup_flags': {g: {k+'_not_over3': delta[g]['tree_rss_gib'][k] <= 3 for k in ('median', 'p95')}
                                             for g in ('all', 'first', 'repeat')}}
        for mode in ('control', *CANDIDATES):
            delta = {g: changes(groups['AS'][g], groups[mode][g]) for g in ('all', 'first', 'repeat')}
            cold_delta = (cold['maps'][mid]['median_change_pct']['AS'][mode]['cold_first_response_s']
                          if mode in CANDIDATES else None)
            gates = {'fixed_first_median_at_least3': delta['first']['e2e_s']['median'] <= -3,
                'repeat_median_not_over5': delta['repeat']['e2e_s']['median'] <= 5,
                'first_p95_not_over5': delta['first']['e2e_s']['p95'] <= 5,
                'repeat_p95_not_over5': delta['repeat']['e2e_s']['p95'] <= 5,
                'query_tree_RSS_all30_median_reduction_at_least5': delta['all']['tree_rss_gib']['median'] <= -5}
            original[mode] = {'change_pct_by_group': delta, 'original_AS_fixed_gates': gates,
                'separate_dedicated_cold_median_change_pct': cold_delta,
                'fixed_first_OR_separate_cold_at_least3': gates['fixed_first_median_at_least3'] or (cold_delta is not None and cold_delta <= -3),
                'no_cold_RSS_substitution_for_all30_query_RSS': True}
        component = {g: changes(groups['PLM'][g], groups['PLMU'][g]) for g in ('all', 'first', 'repeat')}
        report['maps'][mid] = {'groups': groups, 'source_native_cleanup': evidence, 'actual_features': observed,
            'preserved_same_mode_outputs': preserved, 'candidate_control_exact': exact,
            'original_AS_upstream_quality': quality, 'relative_to_control': current,
            'relative_to_AS': original, 'U_incremental_vs_same_batch_PLM': component,
            'all_memory_subgroups_explicit': True, 'runs_sha256': {m: sha(d/'runs.jsonl') for m, d in directories.items()}}
    assert count == 240
    report['strict_total'] = count
    report['both_current_fixed_first_gates'] = {mode: all(v['relative_to_control'][mode]['original_fixed_gates']['gates']['first_benefit_at_least3']
                                                        for v in report['maps'].values()) for mode in CANDIDATES}
    report['both_current_remaining_fixed_gates'] = {mode: all(
        all(flag for key, flag in v['relative_to_control'][mode]['original_fixed_gates']['gates'].items() if key != 'first_benefit_at_least3')
        and all(all(flags.values()) for flags in v['relative_to_control'][mode]['all_memory_subgroup_flags'].values())
        for v in report['maps'].values()) for mode in CANDIDATES}
    report['both_AS_first_OR_cold_and_memory_repeat_tail_gates'] = {mode: all(
        v['relative_to_AS'][mode]['fixed_first_OR_separate_cold_at_least3'] and all(
            flag for key, flag in v['relative_to_AS'][mode]['original_AS_fixed_gates'].items() if key != 'fixed_first_median_at_least3')
        for v in report['maps'].values()) for mode in CANDIDATES}
    report['not_independent_confirmation_of_repaired_candidate'] = True
    report['remaining'] = spec['remaining']
    save(args.report_path, report)
    plot(root, args.report_path)
    print('REFERENCE_FIXED240 strict/current exact/originalAS quality/inputs/source/Native/cleanup checked')
    print('current_first3', report['both_current_fixed_first_gates'], 'current_rest', report['both_current_remaining_fixed_gates'],
          'AS', report['both_AS_first_OR_cold_and_memory_repeat_tail_gates'])
    for mid, info in report['maps'].items():
        print(mid, {mode: {'control': info['relative_to_control'][mode]['original_fixed_gates'],
                           'AS': info['relative_to_AS'][mode]['change_pct_by_group']} for mode in CANDIDATES})


if __name__ == '__main__':
    main()
