"""Source5 diagnostics retain original strict checks and exact current inputs."""
import argparse
from pathlib import Path
import numpy as np
from compare_v9 import read, tag, parts, summary, pair
from compare_v9_review import load, save, sha
from compare_v9_r9 import exact_compare
from review_v9_r9_diagnostic import source_proof, expected_queries
from review_v9_r9_source4 import features as inherited_features
from review_v9_r9_source2 import pose, changes

ROOT = Path(__file__).resolve().parents[1]


def features(directory, rows, mode, mid):
    out = inherited_features(rows, mode.replace('X', '') or 'control', mid)
    want = 'X' in mode
    previous = None
    shape = np.load(directory/'cases'/tag(rows[0])/'l3_scope.npz')['mask'].shape
    h, w = shape
    fine_cells = h*w
    padded_cells = ((h+2)//3*3)*((w+2)//3*3)
    keys = ('exact_scope_calls', 'original_fallback_calls', 'disabled_calls',
            'fine_cells_processed', 'logical_padded_bool_buffer_bytes_avoided')
    for index, row in enumerate(rows):
        s = row['r9_scope_aggregation']
        assert s['mode_enabled'] == want and s['inspection_outside_request_timing']
        assert not s['current_input_or_scope_result_cached'] and not s['audit_or_ACK_results_cached']
        assert s['logical_padding_byte_counter_is_not_RSS']
        assert s['aggregation'] == 'original bottom-left3x3 any, false padding'
        assert s['original_fallback_calls'] == 0
        for key in keys:
            assert s[key] >= 0 and s[key+'_this_request'] >= 0
            if previous is not None:
                assert s[key]-previous[key] == s[key+'_this_request']
        calls = s['exact_scope_calls_this_request']
        assert s['fine_cells_processed_this_request'] == calls*fine_cells
        assert s['logical_padded_bool_buffer_bytes_avoided_this_request'] == calls*padded_cells
        if want:
            assert s['disabled_calls'] == 0
            if index == 0:
                assert calls > 0, (mode, mid, 'new data path not exercised')
        else:
            assert s['exact_scope_calls'] == s['fine_cells_processed'] == s['logical_padded_bool_buffer_bytes_avoided'] == 0
        previous = s
    out['X_actually_exercised'] = want
    out['X_total_exact_scope_calls'] = previous['exact_scope_calls']
    out['X_counters_are_logical_not_RSS'] = True
    out['X_original_full_scope_shape'] = list(shape)
    return out


def diagnostic(root):
    spec = load(ROOT/'report/SOURCE5_PROTOCOL.json')
    declared = load(ROOT/'report/SOURCE5_RUN_PROTOCOL.json')
    assert declared['declared_before_native'] and declared['source_cycle'] == 5
    assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json') == declared['planner_lock_sha256']
    assert sha(ROOT/'report/SOURCE5_PROTOCOL.json') == declared['implementation_protocol_sha256']
    for name, expected in declared['analysis_tools'].items():
        assert sha(ROOT/name) == expected, name
    modes = tuple(spec['preliminary_diagnostic']['modes'])
    assert root == Path(spec['preliminary_diagnostic']['root'])
    report = {'source_cycle': 5, 'diagnostic_only': True, 'profiler': False, 'adopted': False,
        'full_fixed10_acceptance': False, 'n_changed_per_map_mode_kind': 3,
        'changed_requests_are_related_not_tail_evidence': True, 'historical_performance_pooled': False,
        'predeclared_run_protocol_sha256': sha(ROOT/'report/SOURCE5_RUN_PROTOCOL.json'), 'maps': {}}
    count = 0
    for kind in ('moving', 'perturbed'):
        report['maps'][kind] = {}
        for mid in declared['map_order']:
            paths = {mode: root/kind/mode/mid for mode in modes}
            data = {mode: read(path) for mode, path in paths.items()}
            evidence, observed, old_exact = {}, {}, {}
            fixture = declared['moving_fixtures'][mid]
            for mode, path in paths.items():
                protocol = load(path/'protocol.json')
                assert protocol['workload'] == 'moving' and protocol['repeats'] == 1 and protocol['diagnostic_subset']
                assert not protocol['profiler']
                assert Path(protocol['query_file']) == Path(fixture['fixture']) and protocol['queries_sha256'] == fixture['sha256']
                assert (protocol['pose_offset_m'], protocol['yaw_offset_deg']) == ((0., 0.) if kind == 'moving' else (.03, 2.))
                queries = expected_queries(protocol, mid)
                rows = data[mode]
                assert len(rows) == len(queries) == 4
                assert [r['moving_start']['step'] for r in rows] == ['initial', 'p20', 'p50', 'p80']
                for row, query in zip(rows, queries):
                    assert row['query_id'] == query['id'] and row['start'] == pose(query['start']) and row['goal'] == pose(query['goal'])
                evidence[mode] = source_proof(path, mode, cycle=5)
                observed[mode] = features(path, rows, mode, mid)
                old_mode = mode.replace('X', '')
                previous = ROOT/'results/diagnostic_source4'/kind/old_mode/mid
                assert previous.is_dir()
                old_exact[mode] = pair(previous, path, strict=True)
                assert len(old_exact[mode]) == 4
                count += len(rows)
            groups = {mode: {key: summary(rows) for key, rows in parts(data[mode], 'moving').items()} for mode in modes}
            parity = {mode: exact_compare(paths['control'], paths[mode]) for mode in modes if mode != 'control'}
            local_parity = {mode: exact_compare(paths['PLM'], paths[mode]) for mode in ('PLMX', 'PLMUX')}
            for index, before in enumerate(data['control']):
                for mode in modes[1:]:
                    after = data[mode][index]
                    assert before['task_progress'].get('match') == after['task_progress'].get('match')
                    assert (before['task_progress'].get('connector') or {}).get('trials') == (after['task_progress'].get('connector') or {}).get('trials')
            report['maps'][kind][mid] = {'groups': groups, 'source_native_cleanup': evidence,
                'actual_features': observed, 'source4_same_original_base_outputs_exact': old_exact,
                'candidate_control_exact': parity, 'candidate_PLM_exact': local_parity,
                'changed_diagnostic_median_change_pct': {mode: changes(groups['control']['moving'], groups[mode]['moving'])
                                                       for mode in modes[1:]},
                'X_and_U_increment_diagnostic_change_pct': {
                    'PLMX_vs_PLM': changes(groups['PLM']['moving'], groups['PLMX']['moving']),
                    'PLMUX_vs_PLMX': changes(groups['PLMX']['moving'], groups['PLMUX']['moving'])},
                'no_full10_or_tail_or_RSS_adoption_gate_decision': True}
    assert count == spec['preliminary_diagnostic']['expected_records'] == 64
    report['strict_total'] = count
    report['remaining'] = spec['remaining_validation']
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--results-root', type=Path, required=True)
    args = p.parse_args()
    target = ROOT/'report/SOURCE5_DIAGNOSTIC_REVIEW.json'
    assert not target.exists()
    result = diagnostic(args.results_root.resolve())
    save(target, result)
    print('SOURCE5 DIAGNOSTIC strict64 actualX/inputs/source124/native/source4output/currentoutput/ACK/quality/cleanup exact')
