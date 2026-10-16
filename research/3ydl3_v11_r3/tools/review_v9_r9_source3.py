"""Source3 exact mask preparation; raw fixed gates and diagnostic limits."""
import argparse
from pathlib import Path
from compare_v9 import read, pair, parts, summary, tag
from compare_v9_review import load, save
from compare_v9_r9 import exact_compare
from review_v9_r9_diagnostic import source_proof, expected_queries, ROOT, MAPS
from review_v9_r9_source2 import pose, fixed_gates, changes

ALL_MODES = ('control', 'P', 'L', 'PL', 'M', 'PM', 'LM', 'PLM')


def features(rows, mode, mid):
    previous_lookup = previous_membership = None
    first_lut, first_mask = [], []
    want_p, want_l, want_m = ('P' in mode, 'L' in mode, 'M' in mode)
    for row in rows:
        packed = row['r9_record_storage']
        assert packed['mode_enabled'] == packed['internal_points_packed'] == want_p
        assert not packed['audit_or_ACK_results_cached']
        if want_p:
            assert packed['float_buffers_immutable']
        lut = row['r9_static_lookup']
        masks = row['r9_semantic_membership']
        assert lut['mode_enabled'] == want_l and masks['mode_enabled'] == want_m
        for stat in (lut, masks):
            assert not stat['input_content_cached'] and stat['inspection_outside_request_timing']
            assert stat['OpenCV_threads'] == 1
        assert not masks['table_cached'] and not masks['audit_or_ACK_results_cached']
        for value in (lut['native_LUT_calls_this_request'], lut['original_fallback_calls_this_request'],
                      masks['native_membership_calls_this_request'], masks['original_fallback_calls_this_request'],
                      masks['disabled_calls_this_request'], masks['lookup_cells_this_request']):
            assert value >= 0
        if not want_l:
            assert lut['native_LUT_calls'] == lut['original_fallback_calls'] == 0
        if not want_m:
            assert masks['native_membership_calls'] == masks['original_fallback_calls'] == 0
        if want_m and mid == 'mengjia':
            assert masks['original_fallback_calls'] == 0
            assert set(masks['source_dtypes']) <= {'int16'}
        if mid != 'mengjia':
            assert masks['native_membership_calls'] == 0
        if previous_lookup is not None:
            for total, delta in [('native_LUT_calls', 'native_LUT_calls_this_request'),
                                 ('original_fallback_calls', 'original_fallback_calls_this_request')]:
                assert lut[total] - previous_lookup[total] == lut[delta]
        if previous_membership is not None:
            for total, delta in [('native_membership_calls', 'native_membership_calls_this_request'),
                                 ('original_fallback_calls', 'original_fallback_calls_this_request'),
                                 ('disabled_calls', 'disabled_calls_this_request'),
                                 ('lookup_cells', 'lookup_cells_this_request')]:
                assert masks[total] - previous_membership[total] == masks[delta]
        first = row['repeat'] == 1 and (not row.get('moving_start') or row['moving_start']['kind'] == 'initial')
        if first:
            first_lut.append(lut['native_LUT_calls_this_request'])
            first_mask.append(masks['native_membership_calls_this_request'])
            if row is rows[0]:
                if want_l:assert lut['native_LUT_calls_this_request'] > 0
                if want_m and mid == 'mengjia':assert masks['native_membership_calls_this_request'] >= 6
        previous_lookup, previous_membership = lut, masks
    return {'P_actually_packed': want_p, 'L_enabled': want_l,
            'first_request_LUT_calls': first_lut, 'M_enabled': want_m,
            'M_applicable_to_semantic_map': mid == 'mengjia',
            'first_request_membership_calls': first_mask,
            'total_membership_calls': masks['native_membership_calls'],
            'total_membership_cells': masks['lookup_cells'], 'actual_source_dtypes': masks['source_dtypes'],
            'mutable_input_or_audit_cached': False, 'actual_OpenCV_threads': 1}


def review(root, phase, modes, validation_protocol=None):
    declared = load(validation_protocol or ROOT / 'report/SOURCE3_PROTOCOL.json')
    fixed_spec = declared['formal_fixed']
    expected_modes = (fixed_spec.get('review_modes', fixed_spec['modes_in_actual_serial_order']) if phase == 'fixed'
                      else declared['preliminary_diagnostic']['modes'])
    assert list(modes) == expected_modes
    report = {'source_cycle': 3, 'version': '3YD-V9-r9', 'modes': modes,
              'diagnostic_only': phase == 'diagnostic', 'profiler': False, 'adopted': False,
              'historical_timing_pooled': False, 'results_root': str(root), 'maps': {}}
    strict = 0
    candidates = modes[1:]
    kinds = ('fixed',) if phase == 'fixed' else ('moving', 'perturbed')
    for kind in kinds:
        report['maps'][kind] = {}
        for mid in MAPS:
            directories = {mode: root / kind / mode / mid for mode in modes}
            data = {mode: read(directory) for mode, directory in directories.items()}
            source, observed = {}, {}
            for mode, directory in directories.items():
                protocol = load(directory / 'protocol.json')
                assert not protocol['profiler']
                rows = data[mode]
                if phase == 'diagnostic':
                    assert protocol['diagnostic_subset'] and protocol['workload'] == 'moving' and protocol['repeats'] == 1
                    assert (protocol['pose_offset_m'], protocol['yaw_offset_deg']) == ((0., 0.) if kind == 'moving' else (.03, 2.))
                    expected = expected_queries(protocol, mid)
                    assert len(rows) == len(expected) == 4
                    assert [r['moving_start']['step'] for r in rows] == ['initial', 'p20', 'p50', 'p80']
                else:
                    assert protocol['workload'] == 'fixed' and not protocol['diagnostic_subset'] and protocol['repeats'] == 3
                    definition = load(Path(protocol['query_file']))
                    queries = definition.get('scenarios', definition.get('queries'))
                    assert len(queries) == len({q['id'] for q in queries}) == 10 and len(rows) == 30
                    assert set(q['id'] for q in queries) == set(r['query_id'] for r in rows)
                    for query in queries:
                        assert sorted(r['repeat'] for r in rows if r['query_id'] == query['id']) == [1, 2, 3]
                    by_id = {q['id']: q for q in queries}
                    expected = [by_id[r['query_id']] for r in rows]
                for row, query in zip(rows, expected):
                    assert row['success'] and row['query_id'] == query['id']
                    assert row['start'] == pose(query['start']) and row['goal'] == pose(query['goal'])
                observed[mode] = features(rows, mode, mid)
                source[mode] = source_proof(directory, mode, cycle=3)
                strict += len(rows)
            groups = {mode: {key: summary(value) for key, value in parts(rows, 'fixed' if phase == 'fixed' else 'moving').items()}
                      for mode, rows in data.items()}
            parity = {mode: exact_compare(directories['control'], directories[mode]) for mode in candidates}
            parent = Path(load(ROOT / 'PARENT.json')['parent_workspace'])
            parent = parent / ('results/formal_source4/fixed/OGR' if phase == 'fixed' else f'results/regression_source4/{kind}/OGR') / mid
            inherited = pair(parent, directories['control'], strict=True)
            assert len(inherited) == (30 if phase == 'fixed' else 4)
            info = {'groups': groups, 'exact_parity': parity, 'parent_control_exact': inherited,
                    'source_native_cleanup': source, 'actual_features': observed}
            if phase == 'fixed':
                info['relative_to_control'] = {mode: fixed_gates(groups, mode) for mode in candidates}
                info['M_incremental'] = {mode: fixed_gates(groups, mode, baseline)
                                         for mode, baseline in [('M', 'control'), ('PM', 'P'), ('LM', 'L'), ('PLM', 'PL')]
                                         if mode in groups and baseline in groups}
            else:
                info['changed_request_change_pct'] = {mode: changes(groups['control']['moving'], groups[mode]['moving']) for mode in candidates}
                steps = []
                for index, before in enumerate(data['control']):
                    for mode in candidates:
                        after = data[mode][index]
                        assert before['task_progress'].get('match') == after['task_progress'].get('match')
                        trials = lambda v: (v.get('connector') or {}).get('trials')
                        assert trials(before['task_progress']) == trials(after['task_progress'])
                    steps.append({'tag': tag(before), 'exact_match_and_connector_trials': True})
                info['per_step'] = steps
            report['maps'][kind][mid] = info
    assert strict == (fixed_spec['expected_records'] if phase == 'fixed' else declared['preliminary_diagnostic']['expected_records'])
    report['strict_total'] = strict
    if phase == 'fixed':
        report['both_fixed_first_gates_pass'] = {mode: all(info['relative_to_control'][mode]['gates']['first_benefit_at_least3']
                                                       for info in report['maps']['fixed'].values()) for mode in candidates}
        report['remaining_fixed_gates_pass'] = {mode: all(all(value for key, value in info['relative_to_control'][mode]['gates'].items()
                                                           if key != 'first_benefit_at_least3') for info in report['maps']['fixed'].values())
                                                  for mode in candidates}
        report['cold_not_run'] = True
        report['full_adoption_still_requires'] = declared['remaining_full_objective']
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-root', type=Path, required=True)
    parser.add_argument('--phase', choices=('diagnostic', 'fixed'), required=True)
    parser.add_argument('--modes', choices=ALL_MODES, nargs='+', required=True)
    parser.add_argument('--validation-protocol', type=Path)
    parser.add_argument('--report-path', type=Path)
    args = parser.parse_args()
    result = review(args.results_root.resolve(), args.phase, tuple(args.modes), args.validation_protocol)
    save(args.report_path.resolve() if args.report_path else ROOT / 'report' / f'SOURCE3_{args.phase.upper()}_REVIEW.json', result)
    print('SOURCE3', args.phase, 'STRICT', result['strict_total'], 'original source/Native/outputs/maps/cleanup exact')
    for kind, maps in result['maps'].items():
        for mid, info in maps.items():
            print(kind, mid, info['relative_to_control'] if args.phase == 'fixed' else info['changed_request_change_pct'])
