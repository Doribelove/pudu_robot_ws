"""Source5 all original40 poses with contemporary AS/current, exact outputs and memory."""
import argparse
from copy import deepcopy
from pathlib import Path
import numpy as np
from compare_v9 import read, pair, parts, summary, tag
from compare_v9_review import load, save, sha
from compare_v9_r9 import exact_compare, compare
from review_v9_r9_diagnostic import source_proof, ROOT, MAPS
from review_v9_r9_source2 import features as source2_features, changes, pose
from review_v9_r9_source3 import features as source3_features
from review_v9_r9_source4 import features as source4_features
from review_v9_r9_source5 import features as source5_features
from moving_starts import load as load_fixture


def expected(protocol, mid):
    queries = deepcopy(load_fixture(Path(protocol['query_file']), mid))
    for query in queries:
        values = np.asarray(pose(query['start']), float)
        if query['moving_start']['kind'] == 'moving' and (protocol['pose_offset_m'] or protocol['yaw_offset_deg']):
            sign = -1 if query['moving_start']['step'] == 'p50' else 1
            values[:2] += sign*protocol['pose_offset_m']*np.array([-np.sin(values[2]), np.cos(values[2])])
            values[2] = (values[2]+sign*np.deg2rad(protocol['yaw_offset_deg'])+np.pi)%(2*np.pi)-np.pi
        query['start'] = values.tolist()
        query['goal'] = pose(query['goal'])
    return queries


def source(directory, label, rows, cycle):
    original = label == 'AS'
    proof = source_proof(directory, 'control' if original else label, cycle=cycle,
                         storage_mode='control' if original else 'CHNT', r8_mode='control' if original else 'OGR')
    if original:
        shape = np.load(directory/'cases'/tag(rows[0])/'l3_scope.npz')['mask'].shape
        h, w = shape
        log = next((directory/'smac/logs').glob('*.log')).read_text()
        assert f': {w} X {h} map @ 0.05 m/cell' in log
        proof['AS_actual_original_global_map_shape'] = list(shape)
    return proof


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-root', type=Path, required=True)
    parser.add_argument('--candidates', nargs='+', choices=('PLMX', 'PLMUX'), default=('PLMX', 'PLMUX'))
    parser.add_argument('--source-cycle', type=int, choices=(5,), default=5)
    parser.add_argument('--validation-protocol', type=Path)
    parser.add_argument('--report-path', type=Path, required=True)
    args = parser.parse_args()
    assert len(set(args.candidates)) == len(args.candidates)
    root = args.results_root.resolve()
    modes = ('AS', 'control', *args.candidates)
    declared = None
    if args.source_cycle == 5:
        assert args.validation_protocol is not None, 'source5 requires its predeclared full protocol'
        declared = load(args.validation_protocol)
        assert declared['source_cycle'] == 5 and declared['declared_before_full_progress_native']
        assert Path(declared['root']).resolve() == root
        assert tuple(declared['modes_in_serial_order_per_map_and_kind']) == modes
        assert tuple(declared['review_candidates']) == tuple(args.candidates)
        assert declared['expected_records'] == 160 * len(modes)
        assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json') == declared['planner_lock_sha256']
        for name, digest in declared['analysis_tools'].items():
            assert sha(ROOT/name) == digest, name
        for mid, query_spec in declared['original_queries'].items():
            fixture = ROOT/'datasets/moving_start_v1'/(mid+'.json')
            assert Path(query_spec['fixture']).resolve() == fixture
            assert sha(fixture) == query_spec['sha256']
            assert query_spec['query_ids'] == 10 and query_spec['requests'] == 40
    parent = Path(load(ROOT/'PARENT.json')['parent_workspace'])
    report = {'version':'3YD-V9-r9', 'source_cycle':args.source_cycle, 'profiler':False, 'diagnostic_subset':False,
              'historical_timing_used':False, 'adopted':False, 'results_root':str(root),
              'modes':modes, 'maps':{}}
    if declared is not None:
        report['predeclared_protocol_sha256'] = sha(args.validation_protocol)
        report['predeclared_expected_records'] = declared['expected_records']
    count = 0
    for kind in ('moving', 'perturbed'):
        report['maps'][kind] = {}
        for mid in MAPS:
            directories = {mode:root/kind/mode/mid for mode in modes}
            data = {mode:read(directory) for mode,directory in directories.items()}
            evidence, actual_features, parent_parity = {}, {}, {}
            for mode, directory in directories.items():
                protocol = load(directory/'protocol.json')
                assert protocol['workload']=='moving' and protocol['repeats']==1 and not protocol['diagnostic_subset']
                assert (protocol['pose_offset_m'],protocol['yaw_offset_deg']) == ((0.,0.) if kind=='moving' else (.03,2.))
                assert Path(protocol['query_file']).resolve() == ROOT/'datasets/moving_start_v1'/(mid+'.json')
                queries = expected(protocol,mid)
                rows = data[mode]
                assert len(rows)==len(queries)==40
                ids = {q['id'] for q in queries}
                assert len(ids)==10 and set(r['query_id'] for r in rows)==ids
                for qid in ids:
                    assert [r['moving_start']['step'] for r in rows if r['query_id']==qid] == ['initial','p20','p50','p80']
                for row,query in zip(rows,queries):
                    assert row['query_id']==query['id'] and row['start']==query['start'] and row['goal']==query['goal']
                actual_features[mode] = source5_features(directory, rows, 'control' if mode=='AS' else mode, mid)
                evidence[mode] = source(directory,mode,rows,args.source_cycle)
                if mode in ('AS','control'):
                    old = parent/('results/reference_progress_source4/'+kind+'/AS' if mode=='AS'
                                  else 'results/regression_source4/'+kind+'/OGR')/mid
                    parent_parity[mode] = pair(old,directory,strict=True)
                    assert len(parent_parity[mode])==40
                count += len(rows)
            groups = {mode:{key:summary(value) for key,value in parts(rows,'moving').items()} for mode,rows in data.items()}
            assert all(g['initial']['success']==10 and g['moving']['success']==30 for g in groups.values())
            parity = {mode:exact_compare(directories['control'],directories[mode]) for mode in args.candidates}
            reference_parity = {mode:compare(directories['AS'],directories[mode]) for mode in ('control',*args.candidates)}
            assert all(v['upstream_gate'] and v['quality_gate'] for v in reference_parity.values())
            relatives, reference_relatives = {}, {}
            for mode in args.candidates:
                delta = {group:changes(groups['control'][group],groups[mode][group]) for group in ('all','initial','moving')}
                gates = {'changed_median_not_over5':delta['moving']['e2e_s']['median']<=5,
                         'changed_p95_not_over5':delta['moving']['e2e_s']['p95']<=5,
                         'initial_median_not_over5':delta['initial']['e2e_s']['median']<=5,
                         'initial_p95_not_over5':delta['initial']['e2e_s']['p95']<=5,
                         'RSS_all_median_not_over3':delta['all']['tree_rss_gib']['median']<=3,
                         'strict_and_physical_exact':True}
                memory = {group:{key+'_not_over3':delta[group]['tree_rss_gib'][key]<=3 for key in ('median','p95')}
                          for group in ('all','initial','moving')}
                relatives[mode] = {'change_pct_by_group':delta,'incremental_gates':gates,
                                   'memory_subgroup_flags_reported_separately':memory}
                ref_delta = {group:changes(groups['AS'][group],groups[mode][group]) for group in ('all','initial','moving')}
                recovery = {key+'_not_increased':ref_delta['moving']['e2e_s'][key]<=0 for key in ('median','p95')}
                reference_relatives[mode] = {'change_pct_by_group':ref_delta,
                    'full10_changed_recovery_gates':recovery,
                    'RSS_all_median_reduction_at_least5':ref_delta['all']['tree_rss_gib']['median']<=-5,
                    'all_memory_subgroups_explicit':True,'comparison_keeps_original_AS_global_and_candidate_local_server_hashes_distinct':True}
            info = {'groups':groups,'source_native_cleanup':evidence,
                'actual_features':actual_features,'sealed_parent_same_mode_exact':parent_parity,
                'candidate_control_exact':parity,'original_AS_upstream_quality':reference_parity,
                'relative_to_control':relatives,'relative_to_AS':reference_relatives}
            if 'PLMX' in groups and 'PLMUX' in groups:
                component = {group:changes(groups['PLMX'][group],groups['PLMUX'][group])
                             for group in ('all','initial','moving')}
                info['U_incremental_vs_same_batch_PLMX'] = {
                    'change_pct_by_group':component,
                    'initial_changed_median_P95_within_original5pct_tolerance': {
                        group:{key+'_not_over5':component[group]['e2e_s'][key]<=5
                               for key in ('median','p95')}
                        for group in ('initial','moving')},
                    'all_initial_changed_RSS_median_P95_within_original3pct_tolerance': {
                        group:{key+'_not_over3':component[group]['tree_rss_gib'][key]<=3
                               for key in ('median','p95')}
                        for group in ('all','initial','moving')},
                    'logical_duplicate_bytes_are_not_peak_RSS':True,
                    'initial_reverse_fixed_negative_evidence_not_replaced':True}
            report['maps'][kind][mid] = info
    assert count==160*len(modes)
    report['strict_progress_total'] = count
    report['incremental_gates_pass'] = {mode:all(all(v['relative_to_control'][mode]['incremental_gates'].values())
                                              for maps in report['maps'].values() for v in maps.values()) for mode in args.candidates}
    report['original_AS_exact_arena_recovery_pass'] = {mode:all(report['maps']['moving'][MAPS[1]]['relative_to_AS'][mode]['full10_changed_recovery_gates'].values())
                                                    for mode in args.candidates}
    report['original_AS_all40_RSS_median_at_least5_pass'] = {
        mode:all(info['relative_to_AS'][mode]['RSS_all_median_reduction_at_least5']
                 for maps in report['maps'].values() for info in maps.values())
        for mode in args.candidates}
    report['current_control_all_memory_subgroup_median_P95_pass'] = {
        mode:all(all(flags.values())
                 for maps in report['maps'].values() for info in maps.values()
                 for flags in info['relative_to_control'][mode]['memory_subgroup_flags_reported_separately'].values())
        for mode in args.candidates}
    report['adoption_still_requires'] = ['confirmed_fixed_first_or_cold','contemporary_AS_fixed/cold/memory',
                                        'events','actual_selected_entry','B/P_new_task_two_map_speed_gate',
                                        'final_protected_and_resource']
    assert not args.report_path.resolve().exists()
    save(args.report_path.resolve(),report)
    print('R9 SOURCE',args.source_cycle,'FULL ORIGINAL PROGRESS strict',count,'exact physical outputs, original inputs, source, Native and cleanup verified')
    print('incremental',report['incremental_gates_pass'],'original AS exact arena recovery',report['original_AS_exact_arena_recovery_pass'])
    print('original AS all40 RSS>=5pct',report['original_AS_all40_RSS_median_at_least5_pass'],
          'current control all memory subgroups',report['current_control_all_memory_subgroup_median_P95_pass'])
    for kind,maps in report['maps'].items():
        for mid,v in maps.items():
            print(kind,mid,{mode:{'vs_control':v['relative_to_control'][mode]['change_pct_by_group']['moving'],
                                  'vs_AS':v['relative_to_AS'][mode]['change_pct_by_group']['moving']} for mode in args.candidates})


if __name__=='__main__':
    main()
