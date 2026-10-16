"""Four-arm source2 ablation: actual features, original inputs and exact outputs."""
import argparse
from pathlib import Path
from compare_v9 import read, pair, parts, summary, tag
from compare_v9_review import load, save, sha
from compare_v9_r9 import exact_compare, compare
from review_v9_r9_diagnostic import source_proof, expected_queries, ROOT, MAPS

MODES = ('control', 'P', 'L', 'PL')
CANDIDATES = MODES[1:]


def pose(value):
    return [value[k] for k in ('x', 'y', 'yaw_rad')] if isinstance(value, dict) else list(value)


def features(rows, mode):
    previous = None
    first_calls = []
    for row in rows:
        packed = row['r9_record_storage']
        want_p = mode in ('P', 'PL')
        assert packed['mode_enabled'] == packed['internal_points_packed'] == want_p
        assert not packed['audit_or_ACK_results_cached']
        if want_p:
            assert packed['float_buffers_immutable']
        lookup = row['r9_static_lookup']
        want_l = mode in ('L', 'PL')
        assert lookup['mode_enabled'] == want_l
        assert not lookup['input_content_cached'] and lookup['inspection_outside_request_timing']
        assert lookup['OpenCV_threads'] == 1
        assert lookup['native_LUT_calls_this_request'] >= 0
        assert lookup['original_fallback_calls_this_request'] >= 0
        if not want_l:
            assert lookup['native_LUT_calls'] == lookup['native_LUT_calls_this_request'] == 0
            assert lookup['original_fallback_calls'] == 0
        if previous is not None:
            for total, delta in [('native_LUT_calls', 'native_LUT_calls_this_request'),
                                 ('original_fallback_calls', 'original_fallback_calls_this_request')]:
                assert lookup[total] - previous[total] == lookup[delta]
            assert lookup['disabled_calls'] >= previous['disabled_calls']
        first = row['repeat'] == 1 and (not row.get('moving_start') or row['moving_start']['kind'] == 'initial')
        if first:
            first_calls.append(lookup['native_LUT_calls_this_request'])
            if want_l and row is rows[0]:
                assert lookup['native_LUT_calls_this_request'] > 0, (mode, tag(row), 'L not exercised')
        previous = lookup
    return {'P_actually_packed': mode in ('P', 'PL'), 'L_actually_exercised': mode in ('L', 'PL'),
            'first_request_LUT_calls': first_calls,
            'total_request_LUT_calls': sum(r['r9_static_lookup']['native_LUT_calls_this_request'] for r in rows),
            'total_request_original_fallback_calls': sum(r['r9_static_lookup']['original_fallback_calls_this_request'] for r in rows),
            'mutable_input_or_audit_cached': False, 'actual_OpenCV_threads': 1}


def changes(before, after, metrics=('e2e_s', 'tree_rss_gib')):
    return {metric: {key: 100 * (after['metrics'][metric][key] / before['metrics'][metric][key] - 1)
                     for key in ('median', 'p95')} for metric in metrics}


def fixed_gates(groups, candidate, baseline='control'):
    a, b = groups[baseline], groups[candidate]
    delta = {
        'first_median': changes(a['first'], b['first'])['e2e_s']['median'],
        'repeat_median': changes(a['repeat'], b['repeat'])['e2e_s']['median'],
        'first_p95': changes(a['first'], b['first'])['e2e_s']['p95'],
        'repeat_p95': changes(a['repeat'], b['repeat'])['e2e_s']['p95'],
        'tree_rss_all_median': changes(a['all'], b['all'])['tree_rss_gib']['median'],
    }
    gates = {'first_benefit_at_least3': delta['first_median'] <= -3,
             'repeat_median_not_over5': delta['repeat_median'] <= 5,
             'first_p95_not_over5': delta['first_p95'] <= 5,
             'repeat_p95_not_over5': delta['repeat_p95'] <= 5,
             'rss_not_over3': delta['tree_rss_all_median'] <= 3,
             'strict_and_exact': True}
    return {'change_pct': delta, 'gates': gates}


def review_standard(root, phase):
    report = {'source_cycle': 2, 'version': '3YD-V9-r9', 'modes': MODES,
              'diagnostic_only': phase == 'diagnostic', 'profiler': False, 'adopted': False, 'maps': {}}
    kinds = ('moving', 'perturbed') if phase == 'diagnostic' else ('fixed',)
    strict = 0
    for kind in kinds:
        report['maps'][kind] = {}
        for mid in MAPS:
            directories = {mode: root / kind / mode / mid for mode in MODES}
            data = {mode: read(d) for mode, d in directories.items()}
            evidence, feature_evidence = {}, {}
            for mode, directory in directories.items():
                protocol = load(directory / 'protocol.json')
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
                    assert row['query_id'] == query['id']
                    assert row['start'] == pose(query['start']) and row['goal'] == pose(query['goal'])
                feature_evidence[mode] = features(rows, mode)
                evidence[mode] = source_proof(directory, mode, cycle=2)
                strict += len(rows)
            groups = {mode: {key: summary(value) for key, value in parts(rows, 'fixed' if phase == 'fixed' else 'moving').items()}
                      for mode, rows in data.items()}
            parity = {mode: exact_compare(directories['control'], directories[mode]) for mode in CANDIDATES}
            parent = Path(load(ROOT / 'PARENT.json')['parent_workspace'])
            parent = parent / ('results/formal_source4/fixed/OGR' if phase == 'fixed' else f'results/regression_source4/{kind}/OGR') / mid
            inherited = pair(parent, directories['control'], strict=True)
            assert len(inherited) == (30 if phase == 'fixed' else 4)
            info = {'groups': groups, 'exact_parity': parity, 'parent_control_exact': inherited,
                    'source_native_cleanup': evidence, 'actual_features': feature_evidence}
            if phase == 'fixed':
                info['relative_to_control'] = {mode: fixed_gates(groups, mode) for mode in CANDIDATES}
                info['PL_incremental'] = {baseline: fixed_gates(groups, 'PL', baseline) for baseline in ('P', 'L')}
            else:
                info['changed_request_change_pct'] = {mode: changes(groups['control']['moving'], groups[mode]['moving']) for mode in CANDIDATES}
                steps = []
                for index, before in enumerate(data['control']):
                    step = {'tag': tag(before), 'modes': {}}
                    for mode in MODES:
                        after = data[mode][index]
                        x, y = before['task_progress'], after['task_progress']
                        assert x.get('match') == y.get('match')
                        trials = lambda v: (v.get('connector') or {}).get('trials')
                        assert trials(x) == trials(y)
                        step['modes'][mode] = {'wall_s': after['wall_ms']/1000, 'tree_RSS_GiB': after['rss_peak_bytes']/2**30,
                                               'match': y.get('match'), 'connector_trials': trials(y)}
                    steps.append(step)
                info['per_step'] = steps
            report['maps'][kind][mid] = info
    assert strict == (240 if phase == 'fixed' else 64)
    report['strict_total'] = strict
    if phase == 'fixed':
        report['both_fixed_first_gates_pass'] = {mode: all(info['relative_to_control'][mode]['gates']['first_benefit_at_least3']
                                                      for info in report['maps']['fixed'].values()) for mode in CANDIDATES}
        report['remaining_fixed_gates_pass'] = {mode: all(all(v for k,v in info['relative_to_control'][mode]['gates'].items()
                                                          if k != 'first_benefit_at_least3') for info in report['maps']['fixed'].values())
                                                for mode in CANDIDATES}
        report['cold_not_run'] = True
        report['full_adoption_still_requires'] = ['first_or_cold', 'full_progress_and_original_AS_recovery', 'events',
                                                  'contemporary_AS_fixed', 'cold', 'default_entry', 'all_protected_and_resource']
    return report


def review_shared(root):
    report = {'source_cycle': 2, 'version': '3YD-V9-r9', 'modes': MODES,
              'diagnostic_only': True, 'profiler': False, 'adopted': False, 'maps': {}}
    strict = 0
    for mid in MAPS:
        fixture = ROOT / 'datasets/v9_shared_segments_v2' / (mid + '.json')
        definition = load(fixture)
        assert not definition['fixed10'] and len(definition['queries']) == 4
        assert sha(Path(definition['source_path'])) == definition['source_sha256']
        ids = [q['id'] for q in definition['queries']]
        info = {'arms': {}, 'B_P_relative_to_A': {}}
        for arm in ('A', 'ABP'):
            directories = {mode: root / 'shared' / mode / arm / mid for mode in MODES}
            data = {mode: read(d) for mode,d in directories.items()}
            evidence, feature_evidence = {}, {}
            for mode, directory in directories.items():
                protocol = load(directory / 'protocol.json')
                assert protocol['queries_sha256'] == sha(fixture) and protocol['workload'] == 'fixed' and protocol['repeats'] == 1
                assert [r['query_id'] for r in data[mode]] == ids
                for row, query in zip(data[mode], definition['queries']):
                    assert row['start'] == pose(query['start']) and row['goal'] == pose(query['goal'])
                    assert not row['fullroute'].get('reused') and not row['task_progress'].get('reused')
                feature_evidence[mode] = features(data[mode], mode)
                evidence[mode] = source_proof(directory, mode, cycle=2, arm=arm)
                strict += len(data[mode])
            groups = {mode: {'initial': summary(rows[:1]), 'new_tasks': summary(rows[1:])} for mode,rows in data.items()}
            info['arms'][arm] = {'groups': groups, 'exact_parity': {mode: exact_compare(directories['control'], directories[mode]) for mode in CANDIDATES},
                                 'source_native_cleanup': evidence, 'actual_features': feature_evidence}
        for mode in MODES:
            quality = compare(root/'shared'/mode/'A'/mid, root/'shared'/mode/'ABP'/mid)
            assert quality['quality_gate'] and quality['upstream_gate']
            a = info['arms']['A']['groups'][mode]['new_tasks']
            b = info['arms']['ABP']['groups'][mode]['new_tasks']
            change = changes(a,b)['e2e_s']['median']
            info['B_P_relative_to_A'][mode] = {'median_change_pct': change, 'benefit_at_least3': change <= -3,
                                              'quality_and_upstream': quality}
        report['maps'][mid] = info
    assert strict == 64
    report['strict_total'] = strict
    report['B_P_two_map_benefit_pass'] = {mode: all(info['B_P_relative_to_A'][mode]['benefit_at_least3']
                                                 for info in report['maps'].values()) for mode in MODES}
    report['B_P_default_disabled_pending_full_review'] = True
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-root', type=Path, required=True)
    parser.add_argument('--phase', choices=('diagnostic','fixed','shared'), required=True)
    parser.add_argument('--report-path', type=Path, help='separate evidence file for a new validation stage')
    args = parser.parse_args()
    report = review_shared(args.results_root.resolve()) if args.phase == 'shared' else review_standard(args.results_root.resolve(), args.phase)
    report['results_root'] = str(args.results_root.resolve())
    output = args.report_path.resolve() if args.report_path else ROOT / 'report' / f'SOURCE2_{args.phase.upper()}_REVIEW.json'
    save(output, report)
    print('SOURCE2', args.phase, 'strict', report['strict_total'], 'original inputs/outputs/Native/source/cleanup/actual feature checks passed')
    if args.phase == 'shared':
        for mid, info in report['maps'].items():
            print(mid, {mode:r['median_change_pct'] for mode,r in info['B_P_relative_to_A'].items()})
    else:
        for kind, maps in report['maps'].items():
            for mid, info in maps.items():
                print(kind, mid, info['relative_to_control'] if args.phase == 'fixed' else info['changed_request_change_pct'])


if __name__ == '__main__':
    main()
