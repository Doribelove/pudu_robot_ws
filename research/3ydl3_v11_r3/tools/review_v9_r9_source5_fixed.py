"""Source5 contemporary original AS/current/PLM/X/U original fixed10x3."""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, cbook
from compare_v9 import read, pair, parts, summary
from compare_v9_review import load, save, sha
from compare_v9_r9 import compare, exact_compare
from review_v9_r9_source2 import pose, changes, fixed_gates
from review_v9_r9_source5 import features
from review_v9_r9_full_progress import source
from plot_v9_r9_source2 import setup, FONT, MAPS

ROOT = Path(__file__).resolve().parents[1]
MODES = ('AS', 'control', 'PLM', 'PLMX', 'PLMUX')
CANDIDATES = MODES[2:]


def plot(root, review_path):
    setup()
    fig, axes = plt.subplots(2, 3, figsize=(20, 9))
    raw = {'source_cycle': 5, 'font': FONT, 'font_name': 'SimSun', 'linear_axes': True,
        'same_query_repeats_are_related': True, 'review_sha256': sha(review_path),
        'box': 'Q1 to Q3; median; whiskers furthest original values within1.5IQR; outliers displayed', 'groups': []}
    for rr, (mid, label) in enumerate(MAPS):
        groups = {mode: parts(read(root/'fixed'/mode/mid), 'fixed') for mode in MODES}
        for cc, (split, metric, divisor, unit, title) in enumerate([
            ('first', 'wall_ms', 1000, 's', '首次端到端耗时'),
            ('repeat', 'wall_ms', 1000, 's', '重复请求端到端耗时'),
            ('all', 'rss_peak_bytes', 2**30, 'GiB', '整套规划进程树峰值RSS')]):
            values = [[float(r[metric])/divisor for r in groups[mode][split]] for mode in MODES]
            ax = axes[rr, cc]
            ax.boxplot(values, labels=[f'{mode}\nn={len(v)}' for mode, v in zip(MODES, values)], whis=1.5, showfliers=True)
            ax.set_title(label+'\n'+title+' / '+unit); ax.grid(axis='y', alpha=.25)
            for mode, values in zip(MODES, values):
                stats = cbook.boxplot_stats(values, whis=1.5)[0]
                raw['groups'].append({'map_id': mid, 'mode': mode, 'split': split, 'unit': unit,
                    'n': len(values), 'raw': values,
                    'quartiles': [float(stats['q1']), float(stats['med']), float(stats['q3'])],
                    'whiskers': [float(stats['whislo']), float(stats['whishi'])], 'outliers': stats['fliers'].tolist()})
    fig.suptitle('V9-r9 source5：精确L2范围数据准备，原固定10组×3', fontsize=20)
    fig.text(.5, .03, '箱体Q1至Q3；中线中位数；须为1.5×IQR内最远原始值；点为离群点；线性坐标。\n'
        '首次n=10，重复n=20来自同一查询；整树RSS n=30包含Python与ROS后代20ms采样峰值；严格结果另表。', ha='center')
    fig.tight_layout(rect=(0, .10, 1, .93)); fig.savefig(ROOT/'report/fixed_source5.png', dpi=160); plt.close(fig)
    save(ROOT/'report/FIXED_SOURCE5_BOXPLOT_DATA.json', raw)


def review(root):
    spec = load(ROOT/'report/SOURCE5_PROTOCOL.json')
    declared = load(ROOT/'report/SOURCE5_FIXED_VALIDATION_PROTOCOL.json')
    assert declared['declared_before_fixed_native'] and declared['source_cycle'] == 5
    assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json') == declared['planner_lock_sha256']
    assert sha(ROOT/'report/SOURCE5_DIAGNOSTIC_REVIEW_LOCK.json') == declared['diagnostic_review_lock_sha256']
    assert sha(ROOT/'tools/review_v9_r9_source5_fixed.py') == declared['fixed_review_tool_sha256']
    assert root == Path(spec['formal_fixed']['root']) and tuple(spec['formal_fixed']['modes']) == MODES
    parent = Path(load(ROOT/'PARENT.json')['parent_workspace'])
    report = {'source_cycle': 5, 'adopted': False, 'goal_remains_active': True, 'profiler': False,
        'full_original_fixed10': True, 'historical_performance_pooled': False, 'default_r9_mode': 'control',
        'predeclared_fixed_protocol_sha256': sha(ROOT/'report/SOURCE5_FIXED_VALIDATION_PROTOCOL.json'),
        'results_root': str(root), 'maps': {}}
    count = 0
    for mid, qspec in spec['original_queries'].items():
        file = Path(qspec['source']); assert sha(file) == qspec['sha256']
        definition = load(file); queries = definition.get('scenarios', definition.get('queries'))
        assert len(queries) == 10 and [q['id'] for q in queries] == qspec['ids_in_original_order']
        dirs = {mode: root/'fixed'/mode/mid for mode in MODES}
        data = {mode: read(path) for mode, path in dirs.items()}
        evidence, observed, previous = {}, {}, {}
        for mode, path in dirs.items():
            protocol = load(path/'protocol.json'); rows = data[mode]
            assert protocol['workload'] == 'fixed' and protocol['repeats'] == 3
            assert not protocol['diagnostic_subset'] and not protocol['profiler']
            assert protocol['pose_offset_m'] == protocol['yaw_offset_deg'] == 0
            assert Path(protocol['query_file']) == file and protocol['queries_sha256'] == qspec['sha256']
            assert len(rows) == 30
            assert [(r['query_id'], r['repeat']) for r in rows] == [(q['id'], rep) for q in queries for rep in (1, 2, 3)]
            for row, query in zip(rows, [q for q in queries for _ in range(3)]):
                assert row['start'] == pose(query['start']) and row['goal'] == pose(query['goal'])
            evidence[mode] = source(path, mode, rows, 5)
            observed[mode] = features(path, rows, 'control' if mode == 'AS' else mode, mid)
            old = (parent/'results/reference_source4/fixed/AS'/mid if mode == 'AS' else
                   parent/'results/formal_source4/fixed/OGR'/mid if mode == 'control' else
                   ROOT/'results/formal_source4/fixed'/mode.replace('X', '')/mid)
            previous[mode] = pair(old, path, strict=True); assert len(previous[mode]) == 30
            count += len(rows)
        groups = {mode: {key: summary(rows) for key, rows in parts(data[mode], 'fixed').items()} for mode in MODES}
        assert all(g['all']['success'] == 30 and g['first']['n'] == 10 and g['repeat']['n'] == 20 for g in groups.values())
        parity = {mode: exact_compare(dirs['control'], dirs[mode]) for mode in CANDIDATES}
        original = {mode: compare(dirs['AS'], dirs[mode]) for mode in ('control', *CANDIDATES)}
        assert all(v['upstream_gate'] and v['quality_gate'] for v in original.values())
        current, reference = {}, {}
        for mode in CANDIDATES:
            delta = {g: changes(groups['control'][g], groups[mode][g]) for g in ('all', 'first', 'repeat')}
            current[mode] = {'original_fixed_gates': fixed_gates(groups, mode), 'change_pct_by_group': delta,
                'all_memory_subgroup_flags': {g: {key+'_not_over3': delta[g]['tree_rss_gib'][key] <= 3 for key in ('median', 'p95')}
                                             for g in ('all', 'first', 'repeat')}}
            delta = {g: changes(groups['AS'][g], groups[mode][g]) for g in ('all', 'first', 'repeat')}
            reference[mode] = {'change_pct_by_group': delta, 'gates': {
                'fixed_first_median_at_least3': delta['first']['e2e_s']['median'] <= -3,
                'repeat_median_not_over5': delta['repeat']['e2e_s']['median'] <= 5,
                'first_p95_not_over5': delta['first']['e2e_s']['p95'] <= 5,
                'repeat_p95_not_over5': delta['repeat']['e2e_s']['p95'] <= 5,
                'query_tree_RSS_all30_median_reduction_at_least5': delta['all']['tree_rss_gib']['median'] <= -5},
                'source5_cold_not_run_and_no_source4_cold_performance_substitution': True}
        increments = {label: {g: changes(groups[base][g], groups[candidate][g]) for g in ('all', 'first', 'repeat')}
                      for label, base, candidate in [('X_vs_PLM', 'PLM', 'PLMX'), ('U_vs_PLMX', 'PLMX', 'PLMUX')]}
        prep = {mode: [r['timing'].get('r11_l2_obstacles_scope_ms') for r in data[mode] if r['repeat'] == 1] for mode in CANDIDATES}
        report['maps'][mid] = {'groups': groups, 'source_native_cleanup': evidence, 'actual_features': observed,
            'source4_or_sealed_parent_same_base_exact': previous, 'candidate_control_exact': parity,
            'original_AS_upstream_quality': original, 'relative_to_control': current,
            'relative_to_AS': reference, 'component_incremental_change_pct': increments,
            'original_L2_obstacle_scope_preparation_first_raw_ms': prep,
            'all_memory_subgroups_explicit': True, 'runs_sha256': {mode: sha(path/'runs.jsonl') for mode, path in dirs.items()}}
    assert count == spec['formal_fixed']['expected_records'] == 300
    report['strict_total'] = count
    report['both_current_fixed_first_gates'] = {mode: all(v['relative_to_control'][mode]['original_fixed_gates']['gates']['first_benefit_at_least3']
                                                       for v in report['maps'].values()) for mode in CANDIDATES}
    report['both_current_remaining_fixed_gates'] = {mode: all(
        all(flag for key, flag in v['relative_to_control'][mode]['original_fixed_gates']['gates'].items() if key != 'first_benefit_at_least3')
        and all(all(flags.values()) for flags in v['relative_to_control'][mode]['all_memory_subgroup_flags'].values())
        for v in report['maps'].values()) for mode in CANDIDATES}
    report['both_original_AS_fixed_gates'] = {mode: all(all(v['relative_to_AS'][mode]['gates'].values())
                                                     for v in report['maps'].values()) for mode in CANDIDATES}
    report['prior_failed_source4_and_other_source_blocks_retained'] = True
    report['remaining'] = spec['remaining_validation']
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--results-root', type=Path, required=True); args = p.parse_args()
    path = ROOT/'report/SOURCE5_FIXED_REVIEW.json'; assert not path.exists()
    result = review(args.results_root.resolve()); save(path, result); plot(args.results_root.resolve(), path)
    print('SOURCE5 FIXED strict300/current and old outputs/maps exact/originalAS quality/inputs/source124/Native/ACK/cleanup')
    print('first', result['both_current_fixed_first_gates'], 'rest', result['both_current_remaining_fixed_gates'],
          'AS', result['both_original_AS_fixed_gates'])
    for mid, v in result['maps'].items():
        print(mid, {mode: v['relative_to_control'][mode]['original_fixed_gates'] for mode in CANDIDATES})
