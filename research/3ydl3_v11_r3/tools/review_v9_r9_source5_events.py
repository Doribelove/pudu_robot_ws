"""Original44 event assertions, exact returned paths, source5 and actualX."""
import argparse
from pathlib import Path
import numpy as np
from compare_v9_review import load,save,sha
from compare_v9_r9 import mode_evidence
from compare_v8_r3 import libraries
from compare_v8_r6 import verify_cleanup

ROOT=Path(__file__).resolve().parents[1]
MODES=('control','PLMX')
LABELS=('initial','progress80','past_add','duplicate','stale','unobserved_clear_rejected','past_clear','future_add','future_clear','current_block','current_clear')

def source(directory):
    archive=ROOT/'report/source_cycles/source5'
    planning=load(archive/'PLANNER_SOURCE_LOCK.json')['planning_source_files']
    copies=load(directory/'source_snapshot/hashes.json');found=set()
    for original,digest in copies.items():
        relative=Path(original).relative_to(ROOT)
        assert sha(directory/'source_snapshot'/str(relative).replace('/','_'))==digest
        if str(relative) in planning:
            assert digest==planning[str(relative)]==sha(archive/'planning_source'/relative)
            found.add(str(relative))
    assert found==set(planning)
    libs=libraries(directory)
    assert libs and all(sha(Path(name))==digest for name,digest in libs.items())
    return {'planning_files':len(planning),'copied_files':len(copies),'loaded_libraries':libs,'cleanup':verify_cleanup(directory)}

def review(root):
    spec=load(ROOT/'report/SOURCE5_EVENTS_PROTOCOL.json')
    assert spec['source_cycle']==5 and spec['declared_before_events_native']
    assert root==Path(spec['root']) and tuple(spec['modes'])==MODES and spec['expected_assertion_events']==44
    assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json')==spec['planner_lock_sha256']
    for name,digest in spec['analysis_tools'].items():assert sha(ROOT/name)==digest,name
    parent=Path(load(ROOT/'PARENT.json')['parent_workspace'])
    report={'source_cycle':5,'adopted':False,'goal_remains_active':True,'profiler':False,
        'static_event_replay':True,'live_execution':False,'intentional_failure_events_not_strict_successful_planning_records':True,
        'protocol_sha256':sha(ROOT/'report/SOURCE5_EVENTS_PROTOCOL.json'),'maps':{}}
    count=0
    for mid in spec['map_order']:
        paths={mode:root/'events'/mode/mid for mode in MODES}
        records={mode:load(p/'events.json') for mode,p in paths.items()}
        outcome=lambda rows:[(r['label'],r.get('success'),r.get('trajectory_available')) for r in rows]
        assert outcome(records['control'])==outcome(records['PLMX'])
        original=parent/'results/regression_source4/events/OGR'/mid
        assert outcome(load(original/'events.json'))==outcome(records['control'])
        arms={}
        for mode,rows in records.items():
            path=paths[mode];summary=load(path/'summary.json')
            assert summary['assertions_passed'] and summary['events']==len(rows)==11
            assert tuple(r['label'] for r in rows)==LABELS
            assert summary['optimization_mode']=='AB' and summary['v9_mode']=='A' and summary['review_mode']=='S'
            assert summary['r9_mode']==mode and summary['CPU']==[12]
            assert summary['static_event_replay'] and not summary['live_execution']
            features=load(path/'source5_event_features.json')
            assert features['source_cycle']==5 and features['original_run_progress_events_main_called_unchanged']
            assert features['inspection_after_all_original_event_clocks'] and features['logical_counters_not_RSS']
            actual=features['after'];assert actual['mode_enabled']==('X' in mode)
            assert not actual['current_input_or_scope_result_cached'] and not actual['audit_or_ACK_results_cached']
            if mode=='PLMX':assert features['actual_X_exercised'] and actual['exact_scope_calls']>features['before']['exact_scope_calls']
            else:assert not features['actual_X_exercised'] and actual['exact_scope_calls']==0
            for row in rows:
                if row.get('trajectory_available'):
                    assert row['whole_suffix_body_audit']['valid'] and row['original_task_traffic_audit']['valid']
                    assert (path/(row['label']+'_path.npy')).exists()
                if row.get('success') and 'constraint_audit' in row:
                    assert row['constraint_audit']['steering_jump_valid'] and row['independent_audit']['valid']
                diag=row.get('diagnostics',{})
                if diag.get('costmap_update_acknowledged'):
                    assert diag['costmap_ack_mismatch_cells']==0
                    if diag.get('semantic_expected_server_content_hash'):
                        assert diag['server_costmap_content_hash']==diag['semantic_expected_server_content_hash']
                        assert diag['semantic_exact_stable_observations']>=2
            indexed={r['label']:r for r in rows}
            assert not indexed['current_block']['success'] and not indexed['current_block']['trajectory_available']
            assert indexed['current_clear']['success'] and indexed['current_clear']['trajectory_available']
            assert indexed['unobserved_clear_rejected']['passed'] and indexed['unobserved_clear_rejected']['failure']=='CLEAR_REQUIRES_FREE_OBSERVATION'
            if mid=='mengjia':
                assert not indexed['future_add']['success'] and not indexed['future_add']['trajectory_available']
                assert indexed['future_clear']['success'] and indexed['future_clear']['trajectory_available']
            geometry=mode_evidence(path,'CHNT',protocol=False,expected_r8_mode='OGR',expected_r9_mode=mode)
            proof=source(path)
            assert {Path(name).name:digest for name,digest in proof['loaded_libraries'].items()}=={Path(name).name:digest for name,digest in libraries(original).items()}
            arms[mode]={'summary':summary,'outcomes':outcome(rows),'actual_X':features,'source_native_cleanup':proof,'actual_complete_geometry':geometry}
            count+=len(rows)
        exact_paths={}
        for row in records['control']:
            label=row['label']
            if row.get('trajectory_available'):
                a=paths['control']/(label+'_path.npy');b=paths['PLMX']/(label+'_path.npy')
                assert np.array_equal(np.load(a),np.load(b)),(mid,label,'returned trajectory changed')
                exact_paths[label]={'control_sha256':sha(a),'candidate_sha256':sha(b),'arrays_exact':True}
        report['maps'][mid]={'arms':arms,'same_event_outcomes':True,'parent_OGR_event_outcomes_same':True,'candidate_control_paths_exact':exact_paths}
    assert count==44
    report['assertion_events_passed']=count
    report['planning_success_denominator_not44']=True
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--results-root',type=Path,required=True);args=p.parse_args()
    target=ROOT/'report/SOURCE5_EVENTS_REVIEW.json';assert not target.exists();d=review(args.results_root.resolve());save(target,d)
    print('SOURCE5_EVENTS44 ORIGINAL_ASSERTIONS_SAFE_WITHDRAWAL_CLEAR_RESTORE_PHYSICAL_PATHS_SOURCE_NATIVE_ACK_CLEANUP')
