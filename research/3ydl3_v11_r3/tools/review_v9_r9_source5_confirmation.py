"""Independent source5 confirmation retains original gates, exact outputs and first block."""
import argparse
from pathlib import Path
from compare_v9 import read, pair, parts, summary
from compare_v9_review import load, save, sha
from compare_v9_r9 import exact_compare
from review_v9_r9_source2 import pose, changes, fixed_gates
from review_v9_r9_source5 import features
from review_v9_r9_full_progress import source

ROOT = Path(__file__).resolve().parents[1]
MODES = ('control', 'PLMX', 'PLMUX')
CANDIDATES = MODES[1:]

def review(root):
    spec = load(ROOT/'report/SOURCE5_CONFIRMATION_PROTOCOL.json')
    assert spec['source_cycle'] == 5 and spec['declared_before_confirmation_native']
    assert root == Path(spec['root'])
    assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json') == spec['planner_lock_sha256']
    assert sha(ROOT/'report/SOURCE5_FIXED_REVIEW_LOCK.json') == spec['fixed_review_lock_sha256']
    for name, expected in spec['analysis_tools'].items():
        assert sha(ROOT/name) == expected, name
    initial = load(ROOT/'report/SOURCE5_FIXED_REVIEW.json')
    parent = Path(load(ROOT/'PARENT.json')['parent_workspace'])
    out = {'source_cycle': 5, 'adopted': False, 'goal_remains_active': True, 'profiler': False,
        'default_r9_mode': 'control', 'independent_reversed_confirmation': True,
        'full_original_fixed10': True, 'historical_performance_pooled': False,
        'predeclared_protocol_sha256': sha(ROOT/'report/SOURCE5_CONFIRMATION_PROTOCOL.json'),
        'maps': {}, 'results_root': str(root)}
    count = 0
    for mid, qspec in spec['original_queries'].items():
        file = Path(qspec['source']); assert sha(file) == qspec['sha256']
        d = load(file); queries = d.get('scenarios', d.get('queries'))
        assert len(queries) == 10 and [q['id'] for q in queries] == qspec['ids_in_original_order']
        paths = {mode: root/'fixed'/mode/mid for mode in MODES}
        data = {mode: read(p) for mode, p in paths.items()}
        proof, actual, old = {}, {}, {}
        for mode, p in paths.items():
            protocol = load(p/'protocol.json'); rows = data[mode]
            assert protocol['workload'] == 'fixed' and protocol['repeats'] == 3
            assert not protocol['diagnostic_subset'] and not protocol['profiler']
            assert protocol['pose_offset_m'] == protocol['yaw_offset_deg'] == 0
            assert Path(protocol['query_file']) == file and protocol['queries_sha256'] == qspec['sha256']
            assert len(rows) == 30 and [(r['query_id'], r['repeat']) for r in rows] == [(q['id'], i) for q in queries for i in (1,2,3)]
            for row, query in zip(rows, [q for q in queries for _ in range(3)]):
                assert row['start'] == pose(query['start']) and row['goal'] == pose(query['goal'])
            proof[mode] = source(p, mode, rows, 5)
            actual[mode] = features(p, rows, mode, mid)
            old[mode] = pair(ROOT/'results/formal_source5/fixed'/mode/mid, p, strict=True)
            assert len(old[mode]) == 30
            if mode == 'control':
                assert len(pair(parent/'results/formal_source4/fixed/OGR'/mid, p, strict=True)) == 30
            count += len(rows)
        groups = {mode: {k: summary(rows) for k, rows in parts(data[mode], 'fixed').items()} for mode in MODES}
        parity = {mode: exact_compare(paths['control'], paths[mode]) for mode in CANDIDATES}
        current = {}
        for mode in CANDIDATES:
            delta = {g: changes(groups['control'][g], groups[mode][g]) for g in ('all','first','repeat')}
            current[mode] = {'original_fixed_gates': fixed_gates(groups, mode), 'change_pct_by_group': delta,
                'all_memory_subgroup_flags': {g: {key+'_not_over3': delta[g]['tree_rss_gib'][key] <= 3 for key in ('median','p95')}
                                             for g in ('all','first','repeat')}}
        out['maps'][mid] = {'groups': groups, 'source_native_cleanup': proof, 'actual_features': actual,
            'initial_block_outputs_exact': old, 'candidate_control_exact': parity, 'relative_to_control': current,
            'U_incremental_vs_same_batch_PLMX': {g: changes(groups['PLMX'][g], groups['PLMUX'][g]) for g in ('all','first','repeat')},
            'runs_sha256': {mode:sha(p/'runs.jsonl') for mode,p in paths.items()}}
    assert count == spec['expected_records'] == 180
    out['strict_total'] = count
    out['both_current_fixed_first_gates'] = {mode: all(v['relative_to_control'][mode]['original_fixed_gates']['gates']['first_benefit_at_least3']
                                                       for v in out['maps'].values()) for mode in CANDIDATES}
    out['both_current_remaining_fixed_gates'] = {mode: all(
        all(flag for key, flag in v['relative_to_control'][mode]['original_fixed_gates']['gates'].items() if key!='first_benefit_at_least3')
        and all(all(flags.values()) for flags in v['relative_to_control'][mode]['all_memory_subgroup_flags'].values())
        for v in out['maps'].values()) for mode in CANDIDATES}
    out['source5_initial_plus_independent_first_stability'] = {mode: initial['both_current_fixed_first_gates'][mode] and out['both_current_fixed_first_gates'][mode]
                                                              for mode in CANDIDATES}
    out['source5_initial_plus_independent_remaining_fixed_gates'] = {mode: initial['both_current_remaining_fixed_gates'][mode] and out['both_current_remaining_fixed_gates'][mode]
                                                                  for mode in CANDIDATES}
    out['all_prior_negative_blocks_preserved'] = True
    out['remaining'] = ['full_original_exact_and3cm2deg_progress','independent_cold','events44','actual_selected_entry','BP_newtask_speed','final_protected_resources']
    return out

if __name__ == '__main__':
    p = argparse.ArgumentParser();p.add_argument('--results-root', type=Path, required=True);args=p.parse_args()
    target = ROOT/'report/SOURCE5_CONFIRMATION_FIXED_REVIEW.json';assert not target.exists()
    d = review(args.results_root.resolve());save(target,d)
    print('SOURCE5_CONFIRM_STRICT180 ORIGINAL_INPUTS_OUTPUTS_MAPS_NATIVE_ACK_SOURCE_CLEANUP')
    print('first',d['both_current_fixed_first_gates'],'rest',d['both_current_remaining_fixed_gates'], 'both blocks first',d['source5_initial_plus_independent_first_stability'])
