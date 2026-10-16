"""Save a tested candidate; preserve the existing r6 main and frozen releases."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
PROJECT=Path('/home/robot/文档/ChatGPT/dddd-任务')

def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(2**20),b''):h.update(b)
 return h.hexdigest()

def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')

def main():
 c=json.loads((ROOT/'report/COMPARISON.json').read_text())
 assert '184 passed' in (ROOT/'report/tests_final.txt').read_text()
 base={'version':'3YD-V8-r7','branch':'fullroute','date':'2026-10-08','status':'tested_independent_candidate_not_production',
  'workspace':str(ROOT),'parent':'3YD-V8-r6 ABC','parent_workspace':'/home/robot/workspaces/3yd_v8_r6_20261008',
  'L1_frozen':True,'moving_window':False,'unrestricted_global_smac':False,
  'primary_success':[60,60],'additional_success':[18,20],'distinct_query_success':[38,40],
  'main_final_path_parity':[60,60],'additional_successful_path_parity':[18,18],
  'repeat_full_trajectory_reuse':[40,40],'interleaved_reuse':[3,3],
  'original_acceptance_unchanged':True,'costmap_ack_unchanged':True,
  'trajectory_cache_max_entries':8,'trajectory_cache_estimated_limit_bytes':8*2**20,
  'dynamic_event_history_entries':32,'tests_passed':184,'cleanup_batches':len(c['cleanup_journals']),
  'report':str(ROOT/'report/RESULTS.md'),'comparison':str(ROOT/'report/COMPARISON.json'),
  'archive_index':'/home/robot/workspaces/3yd_frozen_baselines/3YD-V8-rolling-r7-r10_20261008/INDEX.json',
  'limitations':['exact-input route reuse only; different start pose normally replans',
    'first-query median +2.3% / +4.0%; all-query P95 +1.8% / +6.7%',
    'arena_medium A2B-03/04 still fail in L3',
    'dynamic replay only, no sensor/tracking/vehicle integration',
    'Nav2 still stores full map; dynamic event peak memory has no matched r6 control']}
 write(ROOT/'BASELINE.json',base)
 files={}
 for p in sorted(ROOT.rglob('*')):
  if p.is_file() and not any(x in p.parts for x in ('__pycache__','.pytest_cache','.git')) and p.suffix!='.pyc' and p.name not in ('RELEASE.json','VERIFY.json'):
   files[str(p.relative_to(ROOT))]=sha(p)
 write(ROOT/'RELEASE.json',{**base,'files':files})
 assert all(sha(ROOT/f)==h for f,h in files.items())
 verified={'registered_files':len(files),'all_match':True,'release_sha256':sha(ROOT/'RELEASE.json')}
 write(ROOT/'VERIFY.json',verified)
 pointer=PROJECT/'3YD_V8_CURRENT.json';old=json.loads(pointer.read_text())
 assert old['version']=='3YD-V8-r6'
 old.pop('active_development',None)
 old['latest_fullroute_candidate']={**base,'release_sha256':verified['release_sha256']}
 old['next_work']='use new V8-r7 fullroute branch; optimize first-query/map-update cost while preserving success; rolling r7-r10 archived'
 write(pointer,old)
 section='''## 2026-10-08 新3YD-V8-r7（完整路线版）已完成

旧移动窗口r7–r10已封存，不作为下一步代码基线。新r7从r6 ABC独立建立：完整路线一次返回、重复请求复用、地图变化按实际影响重规划。r6主版本、L1和历史生产入口不变。

- 工作区：`/home/robot/workspaces/3yd_v8_r7_fullroute_20261008`
- 报告：`/home/robot/workspaces/3yd_v8_r7_fullroute_20261008/report/RESULTS.md`
- 两图固定10×3：60/60严格成功，L1/L2/软目标/最终路径与r6同批对照60/60相同；额外合成A10/10、arena_medium8/10（03/04仍失败）。
- 重复端到端中位：万达3w 0.829→0.436s（−47.4%）；arena_large 2.507→0.778s（−69.0%）。首次+2.3%/+4.0%，总体P95+1.8%/+6.7%，不能写成所有阶段均加速。
- RSS中位：2.073→2.050GiB、3.141→2.750GiB；峰值2.194→2.168、3.198→3.220GiB。动态回放峰值2.450/3.776GiB，没有r6同工作负载对照。
- 无关障碍不重搜、路径障碍重规划、封堵起点撤回轨迹、清除后恢复，两个模式事件回放均通过；尚未接连续控制或实车。
- 184项测试、15批进程/共享资源清理通过。正式数据仅report/SELECTED_DATA.json；早期失败诊断保留但排除。

'''
 for name in ('3YD_CURRENT_PROGRESS.md','3YD_CURRENT_VERSION.md'):
  p=PROJECT/name;prior=p.read_text() if p.exists() else ''
  if section not in prior:p.write_text(section+prior)
 rule='''
- 新V8-r7完整路线候选已完成于`/home/robot/workspaces/3yd_v8_r7_fullroute_20261008`：主两图60/60、额外18/20，全部原成功路径保持；重复耗时降低47.4%/69.0%，首次和总体P95未改善。184项测试、15批清理通过。两种模式的障碍增删回放已接入，非实车/连续执行验收。下一步从此fullroute目录继续，不得误用旧`3yd_v8_r7_20261008`移动窗口目录。r6主版本、L1封存及历史生产入口保持。
'''
 p=PROJECT/'AGENTS.md'
 if rule not in p.read_text():p.write_text(p.read_text()+rule)
 print(json.dumps(verified,ensure_ascii=False))
if __name__=='__main__':main()
