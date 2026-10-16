"""Save an independent tested first-plan candidate, preserving prior releases."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
PROJECT=Path('/home/robot/文档/ChatGPT/dddd-任务')
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for chunk in iter(lambda:f.read(2**20),b''):h.update(chunk)
 return h.hexdigest()
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def main():
 c=json.loads((ROOT/'report/COMPARISON.json').read_text())
 assert '190 passed' in (ROOT/'report/tests_final_AB.txt').read_text()
 for m in c['results']['fixed'].values():
  assert all(a['all']['success']==30 for a in m['arms'].values())
  assert all(n==30 for n in m['equal_file_count'].values())
 for mid,m in c['results']['moving'].items():
  expected=38 if mid=='mengjia' else 40
  assert all(a['all']['success']==expected for a in m['arms'].values())
  assert all(n==expected for n in m['equal_file_count'].values())
  assert all(a['moving']['exact_route_hits']==0 for a in m['arms'].values())
 assert all(v['assertions_passed'] for v in c['dynamic_regression'].values())
 selected={w:{a:{mid:str(ROOT/'results'/w/a/mid) for mid in c['results'][w]} for a in ('control','candidate')} for w in ('fixed','moving')}
 selected.update(dynamic={m:str(ROOT/'results/dynamic'/m) for m in ('semantic','free_space')},excluded=['results/profile','results/smoke_AB'],reason='diagnostic profiles and smoke are not performance samples')
 write(ROOT/'report/SELECTED_DATA.json',selected)
 base={'version':'3YD-V8-r8','branch':'firstplan','date':'2026-10-08','workspace':str(ROOT),
  'status':'tested_independent_candidate_not_production','parent':'3YD-V8-r7-fullroute',
  'parent_workspace':'/home/robot/workspaces/3yd_v8_r7_fullroute_20261008','selected_mode':'AB',
  'L1_frozen':True,'moving_window':False,'unrestricted_global_smac':False,
  'fixed_strict_success':[60,60],'fixed_same_L1_L2_soft_final':[60,60],
  'moving_start_strict_success':[58,60],'moving_initial_success':[20,20],
  'moving_same_successful_paths':[78,78],'moving_exact_trajectory_cache_hits':[0,60],
  'tests_passed':190,'dynamic_events_passed':[11,11],
  'cleanup_batches':len(c['verification']['cleaned_batches']),
  'report':str(ROOT/'report/RESULTS.md'),'comparison':str(ROOT/'report/COMPARISON.json'),
  'limitations':['Q09 p50 no legal L1 semantic route; p80 unresolved start parking ownership; unchanged from control',
    'same-goal moving-start remains full replanning; no route-progress continuation interface yet',
    'Nav2 retains full map; no live localization/tracking/vehicle validation',
    'performance samples are serial independent batches, not a randomized significance study']}
 write(ROOT/'BASELINE.json',base)
 files={}
 for p in sorted(ROOT.rglob('*')):
  if p.is_file() and not any(x in p.parts for x in ('__pycache__','.pytest_cache','.git')) and p.suffix!='.pyc' and p.name not in ('RELEASE.json','VERIFY.json'):
   files[str(p.relative_to(ROOT))]=sha(p)
 write(ROOT/'RELEASE.json',{**base,'files':files})
 assert all(sha(ROOT/f)==h for f,h in files.items())
 verify={'registered_files':len(files),'all_match':True,'release_sha256':sha(ROOT/'RELEASE.json')}
 write(ROOT/'VERIFY.json',verify)
 pointer=PROJECT/'3YD_V8_CURRENT.json';state=json.loads(pointer.read_text())
 # Keep main r6 and the retained r7 candidate; append this independent result.
 state['latest_firstplan_candidate']={**base,'release_sha256':verify['release_sha256']}
 state['next_work']='preserve L1 freeze; design task-progress continuation for Q09 moving-start ownership/direction; then verified remaining-route reuse without moving windows'
 write(pointer,state)
 section='''## 2026-10-08 V8-r8 firstplan：首次完整规划与变化起点测试

工作区 `/home/robot/workspaces/3yd_v8_r8_firstplan_20261008`，从新r7 fullroute独立继承；默认AB，r7/r6和封存L1不变。原两图固定10×3仍60/60，所有路径一致。首次中位万达3w 3.199→2.615s（−18.3%），arena_large 6.398→5.247s（−18.0%）；整体峰值RSS 2.176→2.171GiB、3.193→2.827GiB。

固定目标、起点取原轨迹20%/50%/80%的60次新增请求：两版均58/60，所有成功路径相同，整轨迹缓存0/60命中。万达3w变化起点中位2.542→2.104s，无语义5.428→4.074s。Q09 p50在L1因旧接触点位于车后而方向不符，p80因已进入车位区3却缺当前起点绑定失败；不是L3失败，不能宣称真实闭环全部可用。下一步处理任务进度/停车区内起点，再探索验证复用剩余完整路径。

190项测试、两模式各11事件回放通过；本轮所有批次进程与共享内存已清理。报告在 `report/RESULTS.md`，正式数据索引 `report/SELECTED_DATA.json`。旧移动窗口r8不属于本轮。

'''
 for name in ('3YD_CURRENT_PROGRESS.md','3YD_CURRENT_VERSION.md'):
  p=PROJECT/name;prior=p.read_text() if p.exists() else ''
  if section not in prior:p.write_text(section+prior)
 rule='''\n- 2026-10-08 新V8-r8 firstplan独立候选在`/home/robot/workspaces/3yd_v8_r8_firstplan_20261008`，默认AB；原固定两图60/60且路径一致，首次中位降低18.3%/18.0%。固定目标变化起点58/60，两处Q09 L1失败与对照一致（p50方向/旧接触点，p80停车起点归属）；不能写成闭环全部通过。190测试、两模式22事件功能回归及资源清理通过。保留新r7 fullroute、r6主版本和封存L1；不要误用旧移动窗口r8目录。\n'''
 p=PROJECT/'AGENTS.md'
 if rule not in p.read_text():p.write_text(p.read_text()+rule)
 print(json.dumps(verify,ensure_ascii=False))
if __name__=='__main__':main()
