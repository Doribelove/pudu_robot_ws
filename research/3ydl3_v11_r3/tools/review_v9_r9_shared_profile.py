"""Current strict/Native checks and per-case profile costs; no performance gates."""
import argparse
import inspect
from pathlib import Path
import pstats
import hashlib
import numpy as np

from arena_3d_v1.semantic_world import SweptChecker
from compare_v9 import read, tag, summary
from compare_v8_r3 import libraries
from compare_v9_review import load, save, sha
from review_v9_r9_diagnostic import source_proof
from review_v9_r9_source2 import pose
from review_v9_r9_source4 import features
from quality_v9_r6 import quality

ROOT = Path(__file__).resolve().parents[1]


def profile_pair(before, after):
    """Original output/quality/ACK checks; formal pair deliberately excludes profiles."""
    protocols = [load(directory/'protocol.json') for directory in (before,after)]
    assert all(protocol['profiler'] for protocol in protocols)
    for field in ('queries_sha256','map_sha256','vehicle','CPU','pose_offset_m','yaw_offset_deg','optimization_mode'):
        assert protocols[0][field] == protocols[1][field]
    assert {Path(file).name:value for file,value in libraries(before).items()} == {
        Path(file).name:value for file,value in libraries(after).items()}
    old = {tag(row):row for row in read(before)}
    new = {tag(row):row for row in read(after)}
    assert old.keys() == new.keys()
    records = []
    for key,row in new.items():
        previous = old[key]
        assert previous['success'] and row['success']
        outputs = {name:sha(before/'cases'/key/name)==sha(after/'cases'/key/name)
                   for name in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','path.npy')}
        left = np.load(before/'cases'/key/'l3_scope.npz')['mask']
        right = np.load(after/'cases'/key/'l3_scope.npz')['mask']
        outputs['l3_scope.npz'] = bool(np.array_equal(left,right))
        assert all(outputs[name] for name in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','l3_scope.npz'))
        l2 = {field:previous['l2'][field]==row['l2'][field]
              for field in ('actual_cost_hash','path_cost','dstar_expanded','state_memory_bytes')}
        assert all(l2.values())
        detail = row['l3']
        for source_field, fallback in [('r5_source_full_grid_hash','semantic_expected_grid_hash'),
                                       ('r5_source_full_master_hash','semantic_expected_master_hash')]:
            assert detail[source_field] == previous['l3'].get(source_field,previous['l3'][fallback])
        mask_hash = hashlib.sha256(np.ascontiguousarray(right).tobytes()).hexdigest()
        assert detail['r5_source_full_mask_hash'] == detail['canonical_mask_hash'] == mask_hash
        assert detail['canonical_path_within_mask']
        assert detail['costmap_update_acknowledged'] and detail['costmap_ack_mismatch_cells']==0
        assert detail['semantic_exact_stable_observations']>=2
        assert detail['server_costmap_content_hash']==detail['semantic_expected_server_content_hash']
        assert not detail.get('semantic_ack_reused')
        assert detail['r5_complete_scope_not_rolling'] and not detail['r5_original_map_loaded_in_nav2']
        r0,r1,c0,c1 = detail['r5_complete_task_bounds']
        h,w = detail['r5_native_shape']
        assert [h,w]==[r1-r0,c1-c0]
        assert 0<detail['costmap_ack_checked_cells']<=h*w
        assert not(right[:r0].any() or right[r1:].any() or right[r0:r1,:c0].any() or right[r0:r1,c1:].any())
        path=np.load(after/'cases'/key/'path.npy');origin=detail['r5_native_origin']
        assert np.all(path[:,0]>=origin[0]) and np.all(path[:,0]<origin[0]+w*.05)
        assert np.all(path[:,1]>=origin[1]) and np.all(path[:,1]<origin[1]+h*.05)
        assert row['audit']['valid'] and row['constraints']['traffic']['valid'] and row['constraints']['steering_jump_valid']
        metrics=[load(directory/'cases'/key/'outcome.json')['metrics'] for directory in (before,after)]
        checked=quality(previous['length_m'],row['length_m'],metrics[0]['minimum_clearance_m'],metrics[1]['minimum_clearance_m'])
        assert checked['quality_gate']
        records.append({'tag':key,'same_outputs':outputs,'L2_parity':l2,
            'effective_map_parity':{field:previous['l3'][field]==detail[field] for field in (
                'semantic_expected_grid_hash','semantic_expected_master_hash','server_costmap_content_hash')},
            'quality':checked,'upstream_exact':True,'fresh_current_ACK_and_original_audits':True,
            'diagnostic_only':True})
    return {'n':len(records),'upstream_gate':True,'quality_gate':True,'records':records,
            'formal_performance_claim':False}


def profiles(case):
    stats = pstats.Stats(str(case/'query.prof')).stats
    checker_file = str(Path(inspect.getfile(SweptChecker)).resolve())
    checker_line = inspect.getsourcelines(SweptChecker.__init__)[1]
    selected = []
    for (filename,line,function),(primitive,calls,self_s,cumulative_s,callers) in stats.items():
        label = None
        if (str(Path(filename).resolve()),line,function) == (checker_file,checker_line,'__init__'):
            label = 'SweptChecker.__init__'
        elif filename.endswith('/v9_segments.py') and function in ('propose','cache_for','store'):
            label = 'v9_segments.'+function
        elif filename.endswith('/v9_local.py') and function == 'plan':
            label = 'v9_local.plan'
        elif filename.endswith('/v9_risk.py') and function == 'risk_segments':
            label = 'v9_risk.risk_segments'
        elif filename.endswith('/three_yd_v5/controller.py') and function in ('_audit','activate'):
            label = 'controller.'+function
        elif function == 'build' and ('costmap' in filename or 'adapter' in filename):
            label = 'adapter_or_costmap.build'
        if label:
            selected.append({'label':label,'file':filename,'line':line,'function':function,
                'primitive_calls':primitive,'calls':calls,'self_ms':1000*self_s,
                'cumulative_ms':1000*cumulative_s})
    return {'profile_sha256':sha(case/'query.prof'),'functions':selected,
            'overlapping_cumulative_timers_not_summed':True}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--protocol',type=Path,default=ROOT/'report/SOURCE4_SHARED_PROFILE_PROTOCOL.json')
    parser.add_argument('--report-path',type=Path,default=ROOT/'report/SOURCE4_SHARED_PROFILE_REVIEW.json')
    args = parser.parse_args()
    declared = load(args.protocol)
    assert declared['diagnostic_only'] and declared['profiler'] and declared['source_cycle'] == 4
    assert declared['planner_lock_sha256'] == sha(ROOT/'report/PLANNER_SOURCE_LOCK.json')
    root = Path(declared['root'])
    report = {'source_cycle':4,'diagnostic_only':True,'profiler':True,'adopted':False,
              'timing_not_used_for_any_formal_gate':True,'protocol_sha256':sha(args.protocol),
              'n3_related_new_tasks_not_fixed10_or_tail_acceptance':True,'maps':{}}
    count = 0
    for mid, fixture_spec in declared['original_queries'].items():
        fixture = Path(fixture_spec['fixture']); definition = load(fixture)
        assert sha(fixture) == fixture_spec['fixture_sha256'] and len(definition['queries']) == 4
        assert sha(Path(definition['source_path'])) == definition['source_sha256']
        assert [q['id'] for q in definition['queries']] == fixture_spec['ids']
        info = {'modes':{}}
        for mode in declared['modes']:
            arms = {}
            for arm, configuration in declared['arms'].items():
                directory = root/arm/mode/mid
                protocol = load(directory/'protocol.json'); rows = read(directory)
                assert protocol['profiler'] and protocol['workload'] == 'fixed' and protocol['repeats'] == 1
                assert Path(protocol['query_file']) == fixture and len(rows) == 4
                assert [r['query_id'] for r in rows] == fixture_spec['ids']
                assert summary(rows)['success'] == 4
                proof = source_proof(directory,mode,cycle=4,profile=True,arm=arm,r2_mode=configuration['r2_mode'])
                observed = features(rows,mode,mid)
                raw = []
                for row,query in zip(rows,definition['queries']):
                    assert row['success'] and row['start'] == pose(query['start']) and row['goal'] == pose(query['goal'])
                    assert not row['fullroute'].get('reused') and not row['task_progress'].get('reused')
                    local = row.get('v9_local') or {}; segments = local.get('segments') or {}
                    raw.append({'tag':tag(row),'strict_success':True,'wall_ms_diagnostic_only':row['wall_ms'],
                        'tree_RSS_bytes_diagnostic_only':row['rss_peak_bytes'],
                        'local_accepted_in_strict_task':bool(local.get('success')),
                        'segment_accepted_in_strict_task':bool(local.get('success') and segments.get('matched_segments',0)>0),
                        'local':local,'timing':row['timing'],
                        'backend_calls':(row.get('l3') or {}).get('backend_call_count',0),
                        'profile':profiles(directory/'cases'/tag(row))})
                count += len(rows)
                arms[arm] = {'source_native_cleanup':proof,'actual_features':observed,'raw':raw}
            quality = {}
            for arm in ('ABP','ABPM'):
                observed = profile_pair(root/'A'/mode/mid,root/arm/mode/mid)
                assert observed['upstream_gate'] and observed['quality_gate']
                quality[arm] = observed
            direct = profile_pair(root/'ABP'/mode/mid,root/'ABPM'/mode/mid)['records']
            assert len(direct) == 4
            info['modes'][mode] = {'arms':arms,'A_reference_original_quality_upstream':quality,
                'ABPM_vs_ABP_exact_outputs_and_maps_observed':all(all(z['same_outputs'].values()) and all(z['effective_map_parity'].values()) for z in direct),
                'ABPM_vs_ABP_per_case':direct,
                'profiler_may_change_deadline_fallbacks_preserved':True}
        report['maps'][mid] = info
    assert count == declared['expected_records'] == 48
    report['strict_total'] = count
    report['remaining'] = ['unprofiled original same-batch B/P new-task speed and memory gates',
                           'full original fixed/progress/AS/cold/events/entry/final scope unchanged']
    save(args.report_path.resolve(),report)
    print('PROFILE48 strict original source/Native/poses/quality/cleanup checked; NOADOPTIONTIMING')
    for mid,info in report['maps'].items():
        for mode,z in info['modes'].items():
            print(mid,mode,'ABPM_ABP_EXACT_OBSERVED',z['ABPM_vs_ABP_exact_outputs_and_maps_observed'])
            for arm,a in z['arms'].items():
                print(arm,'NEW_TASK_CHECKER',[
                    [(f['calls'],round(f['self_ms'],3),round(f['cumulative_ms'],3))
                     for f in row['profile']['functions'] if f['label']=='SweptChecker.__init__']
                    for row in a['raw'][1:]])


if __name__ == '__main__':
    main()
