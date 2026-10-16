"""Source4 exact immutable buffer sharing; original fixed gates retained."""
import argparse
from pathlib import Path
from compare_v9 import read, pair, parts, summary, tag
from compare_v9_review import load, save
from compare_v9_r9 import exact_compare
from review_v9_r9_diagnostic import source_proof, expected_queries, ROOT, MAPS
from review_v9_r9_source2 import pose, fixed_gates, changes

ALL_MODES = ('control', 'P', 'L', 'PL', 'M', 'PM', 'LM', 'PLM', 'U', 'PLU', 'PLMU')


from review_v9_r9_source3 import features as inherited_features


def features(rows, mode, mid):
    observed = inherited_features(rows, mode, mid)
    want_u = 'U' in mode
    previous = None
    for row in rows:
        values = row['r9_buffer_storage']
        assert values['mode_enabled'] == want_u
        assert values['inspection_outside_request_timing']
        assert not values['static_or_scope_content_cached']
        assert not values['audit_or_ACK_results_cached']
        assert values['byte_counters_are_logical_duplicate_storage_not_RSS']
        assert values['static_shared_calls'] == (1 if want_u else 0)
        assert values['static_original_seal_calls'] == (0 if want_u else 1)
        cells = row['bounded_l3']['full_map_cells']
        assert values['static_duplicate_buffer_bytes_avoided'] == (cells if want_u else 0)
        assert values['scope_duplicate_buffer_bytes_avoided_this_request'] == cells*values['scope_shared_calls_this_request']
        if not want_u or mid == 'mengjia':
            assert values['scope_shared_calls'] == values['scope_shared_calls_this_request'] == 0
        first = row['repeat'] == 1 and (not row.get('moving_start') or row['moving_start']['kind'] == 'initial')
        if want_u and mid != 'mengjia' and first:
            assert values['scope_shared_calls_this_request'] >= 1
        if previous is not None:
            assert values['scope_shared_calls'] - previous['scope_shared_calls'] == values['scope_shared_calls_this_request']
        previous = values
    observed['U_actual_static_shared'] = want_u
    observed['U_generic_scope_shared_calls'] = values['scope_shared_calls']
    observed['U_counters_are_not_RSS'] = True
    return observed


def review(root, phase, modes, validation_protocol=None):
    declared = load(validation_protocol or ROOT / 'report/SOURCE4_PROTOCOL.json')
    fixed_spec = declared['formal_fixed']
    expected_modes = (fixed_spec.get('review_modes', fixed_spec['modes_in_actual_serial_order']) if phase == 'fixed'
                      else declared['preliminary_diagnostic']['modes'])
    assert list(modes) == expected_modes
    report = {'source_cycle': 4, 'version': '3YD-V9-r9', 'modes': modes,
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
                source[mode] = source_proof(directory, mode, cycle=4)
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
                info['U_incremental_change_pct'] = {
                    mode:{group:changes(groups[baseline][group],groups[mode][group])
                          for group in ('all','first','repeat')}
                    for mode,baseline in [('PLU','PL'),('PLMU','PLM')]
                    if mode in groups and baseline in groups}
                info['M_incremental'] = {mode: fixed_gates(groups, mode, baseline)
                                         for mode, baseline in [('M', 'control'), ('PM', 'P'), ('LM', 'L'), ('PLM', 'PL'), ('PLMU', 'PLU')]
                                         if mode in groups and baseline in groups}
                info['memory_subgroup_change_pct'] = {
                    mode:{group:changes(groups['control'][group],groups[mode][group])['tree_rss_gib']
                          for group in ('all','first','repeat')}
                    for mode in candidates}
                info['memory_subgroup_flags'] = {
                    mode:{group:{key+'_not_over3':value<=3 for key,value in values.items()}
                          for group,values in subgroups.items()}
                    for mode,subgroups in info['memory_subgroup_change_pct'].items()}
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
                owner_checks = {}
                for mode in modes:
                    want_u = 'U' in mode
                    checks = []
                    for row in data[mode]:
                        ledger = load(directories[mode]/'cases'/tag(row)/'storage.json')
                        assert ledger['diagnostic_only'] and not ledger['properties_evaluated'] and not ledger['truncated']
                        arrays = {v['path']:v for v in ledger['arrays']}
                        left = arrays['world.hard']; right = arrays['service.dynamic.static_hard']
                        assert (left['owner']==right['owner']) == want_u
                        assert left['dtype']==right['dtype']=='bool' and not left['writeable'] and not right['writeable']
                        if mid != 'mengjia':
                            p=arrays['service.controller.priority']; q=arrays['service.controller.search_legal']
                            assert (p['owner']==q['owner']) == want_u
                            if want_u:assert not p['writeable'] and not q['writeable']
                        checks.append({'tag':tag(row),'static_shared':want_u,'generic_scope_shared':want_u and mid!='mengjia',
                                       'reachable_array_bytes':ledger['unique_array_buffer_bytes']})
                    owner_checks[mode]=checks
                info['actual_ledger_owner_checks']=owner_checks
            report['maps'][kind][mid] = info
    assert strict == (fixed_spec['expected_records'] if phase == 'fixed' else declared['preliminary_diagnostic']['expected_records'])
    report['strict_total'] = strict
    if phase == 'fixed':
        report['both_fixed_first_gates_pass'] = {mode: all(info['relative_to_control'][mode]['gates']['first_benefit_at_least3']
                                                       for info in report['maps']['fixed'].values()) for mode in candidates}
        report['remaining_fixed_gates_pass'] = {mode: all(all(value for key, value in info['relative_to_control'][mode]['gates'].items()
                                                           if key != 'first_benefit_at_least3')
                                                        and all(all(flags.values()) for flags in info['memory_subgroup_flags'][mode].values())
                                                        for info in report['maps']['fixed'].values())
                                                  for mode in candidates}
        report['cold_not_run'] = True
        report['full_adoption_still_requires'] = declared['after_fixed_remaining']
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
    save(args.report_path.resolve() if args.report_path else ROOT / 'report' / f'SOURCE4_{args.phase.upper()}_REVIEW.json', result)
    print('SOURCE4', args.phase, 'STRICT', result['strict_total'], 'original source/Native/outputs/maps/cleanup exact')
    for kind, maps in result['maps'].items():
        for mid, info in maps.items():
            print(kind, mid, info['relative_to_control'] if args.phase == 'fixed' else info['changed_request_change_pct'])
