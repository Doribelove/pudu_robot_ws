"""Seal verified R7 ASCHNT; preserve unmet progress/cache speed goals."""
from pathlib import Path
from compare_v9_review import load,save
from compare_v9_r7_all import sha
from compare_v8_r6 import verify_cleanup
ROOT=Path(__file__).resolve().parents[1]


def main():
 assert not (ROOT/'RELEASE.json').exists(),'completed release is immutable'
 b=load(ROOT/'BASELINE.json');d=load(ROOT/'report/R7_ALL_COMPARISON.json');s=load(ROOT/'report/R7_SHARED_COMPARISON.json')
 assert b['selected_mode']==d['selected_mode']=='ASCHNT'
 assert b['r5_mode']=='C' and b['r6_mode']=='H' and b['r7_mode']=='NT'
 assert b['v9_mode']=='A' and b['review_mode']=='S' and b['r2_mode']==b['r3_mode']=='control'
 assert b['reviewed_default_entry'] and d['default_entry_verified'] and d['candidate_CHNT_eligible_before_default_entry_check']
 assert d['regression_predeclared_gates'] and all(d['fixed']['predeclared_fixed_gates'].values())
 assert s['joint_functional_and_quality_gate'] and not s['adopt_B_or_P_by_default']
 assert not any(s['both_maps_median_improve_at_least_3_percent'].values())
 assert '345 passed' in d['verification']['tests']
 lock=load(ROOT/'report/PLANNER_SOURCE_LOCK.json');planning=lock['planning_source_files']
 assert lock['source_cycle']==2 and len(planning)==114 and all(sha(ROOT/f)==h for f,h in planning.items())
 old=load(ROOT/'report/source_cycles/source1/PLANNER_SOURCE_LOCK.json')['planning_source_files']
 assert len(old)==113 and all(sha(ROOT/'report/source_cycles/source1'/f)==h for f,h in old.items())
 manifests=[]
 for p in sorted((ROOT/'results').rglob('source_snapshot/hashes.json')):
  files=load(p)
  assert all(sha(p.parent/str(Path(f).relative_to(ROOT)).replace('/','_'))==h for f,h in files.items())
  manifests.append({'manifest':str(p),'copied_files_verified':len(files)})
 cleanup=[verify_cleanup(Path(str(p)[:-len('.supervisor.json')])) for p in sorted((ROOT/'results').rglob('*.supervisor.json'))]
 assert len(manifests)==len(cleanup)==b['cleanup_batches']==47
 assert len(d['verification']['formal_source_snapshots'])==28
 evidence={'selected_mode':'ASCHNT','B_P_default_disabled':True,'source_cycle':2,'planning_files_verified':114,
           'source1_archived_planning_files_verified':113,'original_copy_manifests':manifests,'cleanup_batches':47,
           'formal_current_source_snapshots':28,'fixed_cold_start_planning_strict_records':452,
           'shared_diagnostic_strict_records':24,'selected_entry_strict_records':2,'events_separate':44,
           'tests':345,'all_adoption_gates_passed':True,'historical_production_entry_unchanged':True,
           'remaining_overall_goal_work':['exact-progress median regression22.31/17.84percent','B/P cross-task speed improvement absent'],
           'goal_remains_active':True}
 save(ROOT/'report/RELEASE_EVIDENCE.json',evidence)
 b.update(status='selected_static_fulltask_candidate_sealed',goal_remains_active=True);save(ROOT/'BASELINE.json',b)
 save(ROOT/'report/SELECTED_DATA.json',{'formal_root':str(ROOT/'results/accepted_r7_v2'),'source_cycle':2,
      'formal_subdirs':['fixed','cold','moving','perturbed','events'],'shared_diagnostic':str(ROOT/'results/shared_joint_v2'),
      'selected_entry':str(ROOT/'results/selected_entry_v2'),
      'excluded_from_formal_performance':{str(ROOT/'results/neutral_connection_v1'):'source1 subset diagnostic',
         str(ROOT/'results/batch_probe_v1'):'preserved source1 real batch-binding failure',
         str(ROOT/'results/batch_probe_v2'):'source2 one original query real Native compatibility, no performance claim'},
      'no_resampling_or_slow_record_deletion':True,'original_source_copy_manifests_verified':47})
 (ROOT/'report/CONTINUATION_STATE.md').write_text('''# R7_RELEASE_SEALED\n\n/home/robot/workspaces/3yd_v9_r7_storage_batch_20261009 已完成采用 ASCHNT，并保存所有代码、原输入、Native、raw数据、各源周期快照和报告。本目录及其 RELEASE 覆盖文件不得再编辑。R7默认仅本独立目录；所有历史生产和冻结L1/V8-r11/V9-r0至r6保持。\n\nsource2 114规划源/345测试；固定候选60/60、精确/偏差各60变化+20初始、cold两臂12/12、44事件、24共享诊断、真实run.sh2/2；47批清理与原source复制/28正式快照，父、冻结、原生、外审计/生产及原L1/L2/upstream/质量已全验。源周期1真实批次失败和完整原复制源保留。\n\n两图first median -15.54%/-21.01%，query-tree RSS median -12.92%/-8.44%，repeat/first P95门槛过；cold n3 med-16.81%/-31.76%，只逐点。精确推进median +22.31%/+17.84%未改善；偏差请求改善。原float32净空边界两处direct gate False、预声明半ULP判定True单列，不改硬审计。\n\n共享B/P功能/上游/质量通过但无两图提速（B +3.54/+20.85%，B+P +4.50/+0.57%），默认关闭。T真实内部Smac已完成，局部候选仍拒绝且原受限回退成功，不算局部成功。3个新端点只诊断，不推广fixed10/更多图/尾部。Q09前方阻塞仍失败撤回，清除恢复。\n\n所有执行sessions已终态，无Native/测试/分析运行。整体goal继续active，未达到精确推进/跨任务缓存提速目标。下一步另建R8，先同原输入CPU12真实profile定位局部checker/数据与batch绑定成本，保持L1/L2算法、原Native和全部硬审计，不能覆盖R7或把profile混正式性能。RESULTS.md/EXPERIMENT_PROTOCOL.md/SELECTED_DATA.json 为此版最终口径。\n''')
 files={str(p.relative_to(ROOT)):sha(p) for p in sorted(ROOT.rglob('*')) if p.is_file()
       and not any(v in p.parts for v in ('__pycache__','.pytest_cache','.git'))
       and p.suffix!='.pyc' and p.name not in ('RELEASE.json','VERIFY.json')}
 save(ROOT/'RELEASE.json',{**b,'files':files})
 assert all(sha(ROOT/f)==h for f,h in files.items())
 out={'all_match':True,'files':len(files),'release_sha256':sha(ROOT/'RELEASE.json'),'selected_mode':'ASCHNT',
      'cleanup_batches':47,'original_copy_manifests':47,'formal_source_snapshots':28,'goal_remains_active':True}
 save(ROOT/'VERIFY.json',out);print(out)
if __name__=='__main__':main()
