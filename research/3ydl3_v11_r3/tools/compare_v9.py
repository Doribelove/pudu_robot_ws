"""Independent V9 comparison, immutable evidence and raw distributions."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from analyze_v8 import MAPS,METRICS,measure,stats
from compare_v8_r3 import checked_success,audit_ast,libraries
from compare_v8_r6 import verify_cleanup
ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/home/robot/workspaces/3yd_v8_r11_firstcost_20261009')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(d):return [json.loads(v) for v in (d/'runs.jsonl').read_text().splitlines()]
def tag(r):return f"{r['query_id']}_{r['moving_start']['step']}_{r['repeat']}" if r.get('moving_start') else f"{r['query_id']}_{r['repeat']}"
def summary(rows):
 for r in rows:checked_success(r)
 return {'n':len(rows),'success':sum(r['success'] for r in rows),
  'metrics':{m:stats([measure(r,m) for r in rows]) for m in [*METRICS,'strict_audit_s']},
  'length_m':stats([r.get('length_m') for r in rows]),'right_fraction':stats([r.get('right_side_fraction') for r in rows]),
  'backend_calls':sum(r.get('l3',{}).get('backend_call_count',0) for r in rows),
  'local_attempts':sum(bool(r.get('v9_local')) for r in rows),
  'local_candidate_success':sum(r.get('v9_local',{}).get('success',False) for r in rows),
  'local_success':sum(r['success'] and r.get('v9_local',{}).get('success',False) for r in rows),
  'segment_candidate_success':sum(r.get('v9_local',{}).get('success',False) and r.get('v9_local',{}).get('segments',{}).get('matched_segments',0)>0 for r in rows),
  'segment_reuse_success':sum(r['success'] and r.get('v9_local',{}).get('success',False) and r.get('v9_local',{}).get('segments',{}).get('matched_segments',0)>0 for r in rows),
  'local_success_counting':'complete task strict success AND local candidate accepted; segment reuse counts tasks, not number of segments',
  'bounded_fallbacks':sum(r.get('v9_local',{}).get('bounded_whole_fallback',False) for r in rows),
  'whole_route_hits':sum(r.get('fullroute',{}).get('reused',False) or r.get('task_progress',{}).get('reused',False) for r in rows),
  'failures':[{'tag':tag(r),'code':r.get('failure_code')} for r in rows if not r['success']]}
def parts(rs,kind):
 if kind=='fixed':return {'all':rs,'first':[r for r in rs if r['repeat']==1],'repeat':[r for r in rs if r['repeat']>1]}
 return {'all':rs,'initial':[r for r in rs if r['moving_start']['kind']=='initial'],'moving':[r for r in rs if r['moving_start']['kind']=='moving']}
def pair(a,b,strict=False,require_upstream=True):
 x={tag(r):r for r in read(a)};y={tag(r):r for r in read(b)};assert y.keys()<=x.keys()
 pa,pb=[json.loads((p/'protocol.json').read_text()) for p in (a,b)]
 for k in ('queries_sha256','map_sha256','vehicle','CPU','pose_offset_m','yaw_offset_deg','optimization_mode'):
  assert pa[k]==pb[k],(a,b,k)
 assert pa['optimization_mode']=='AB' and not pa['profiler'] and not pb['profiler']
 assert {Path(p).name:h for p,h in libraries(a).items()}=={Path(p).name:h for p,h in libraries(b).items()}
 rows=[]
 for t,r in y.items():
  prev=x[t];eq={}
  for f in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','path.npy','l3_scope.npz'):
   p,q=a/'cases'/t/f,b/'cases'/t/f
   eq[f]=(np.array_equal(np.load(p)['mask'],np.load(q)['mask']) if f.endswith('.npz') else sha(p)==sha(q)) if p.exists() and q.exists() else None
  maps={k:prev.get('l3',{}).get(k)==r.get('l3',{}).get(k) for k in ('semantic_expected_grid_hash','semantic_expected_master_hash','server_costmap_content_hash')}
  l2={k:prev.get('l2',{}).get(k)==r.get('l2',{}).get(k) for k in ('actual_cost_hash','path_cost','dstar_expanded','state_memory_bytes')}
  if require_upstream:
   assert all(eq[f] for f in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','l3_scope.npz')) and all(l2.values()),(t,eq,l2)
  if strict:assert prev['success']==r['success'] and all(eq.values()) and all(maps.values()) and all(l2.values()),(t,eq,maps,l2)
  rows.append({'tag':t,'success':[prev['success'],r['success']],'same_outputs':eq,'effective_map_parity':maps,'L2_parity':l2,
   'length_change_pct':None if not prev.get('length_m') or not r.get('length_m') else 100*(r['length_m']/prev['length_m']-1),
   'max_steering_jump_deg':r.get('constraints',{}).get('max_raw_steering_jump_deg'),
   'local':r.get('v9_local',{})})
 return rows

def verify(results,shared):
 info={};lock=json.loads((ROOT/'L1_LOCK.json').read_text())
 planner=json.loads((ROOT/'report/PLANNER_SOURCE_LOCK.json').read_text())['planning_source_files']
 assert all(sha(ROOT/f)==h for f,h in planner.items())
 info['final_planner_source_lock_unchanged']=len(planner)
 for section in ('source_files','input_files','pixel_input_files'):
  assert all(sha(ROOT/f)==h for f,h in lock[section].items());info[section+'_unchanged']=len(lock[section])
 for f in ('src/three_yd_v4/controller.py','src/three_yd_v4/r5_controller.py','src/three_yd_v5/controller.py'):
  assert audit_ast(ROOT/f)==audit_ast(PARENT/f),f
 info['original_strict_audits_unchanged']=True
 for f in ('src/three_yd_v5/task_progress.py','src/three_yd_v5/progress_connector.py','native/libcold_dstar.so','native/cold_dstar.cpp'):
  assert sha(ROOT/f)==sha(PARENT/f),f
 info['continuation_and_L2_native_unchanged']=True
 parent=json.loads((PARENT/'RELEASE.json').read_text())
 assert sha(PARENT/'RELEASE.json')=='3e19256a3c09c95cf3f42dc9ce3bc73a5d396580445f12e7bb34fe5cae867bbf'
 assert all(sha(PARENT/f)==h for f,h in parent['files'].items())
 info['parent_files_unchanged']=len(parent['files'])
 frozen=Path('/home/robot/workspaces/3yd_frozen_baselines/3YD-V8-r11-AB_20261009/snapshot.tar.gz')
 assert sha(frozen)=='df7efcd136edb48e452e60dd14a40899c360ee30572fa3f0a37bf040b6f42d3c'
 info['parent_archive_unchanged']=True
 snapshots=[]
 for p in sorted(set([*results.rglob('source_snapshot/hashes.json'),*shared.rglob('source_snapshot/hashes.json')])):
  sources={f:h for f,h in json.loads(p.read_text()).items() if '/src/' in f}
  assert all(sha(f)==h for f,h in sources.items()),p
  snapshots.append({'path':str(p),'planner_source_files':len(sources)})
 info['source_snapshots']=snapshots
 journals=[*results.rglob('*.supervisor.json'),*ROOT.joinpath('results').glob('diagnostic_v*/*/*.supervisor.json'),*ROOT.joinpath('results/accepted_v9').rglob('*.supervisor.json'),*ROOT.joinpath('results/accepted_v9_final').rglob('*.supervisor.json'),*ROOT.joinpath('results/accepted_v9_final2').rglob('*.supervisor.json')]
 journals+=list(shared.rglob('*.supervisor.json'))
 info['cleanup']=[verify_cleanup(Path(str(p)[:-len('.supervisor.json')])) for p in sorted(set(journals))]
 info['tests']=(ROOT/'report/tests_final.txt').read_text().splitlines()[-1]
 return info

def main():
 p=argparse.ArgumentParser();p.add_argument('--results-root',type=Path,default=ROOT/'results/accepted_v9_final3');p.add_argument('--report-dir',type=Path,default=ROOT/'report')
 p.add_argument('--shared-root',type=Path,default=ROOT/'results/accepted_v9_shared_final/shared')
 args=p.parse_args();rs=args.results_root.resolve();shared=args.shared_root.resolve();report=args.report_dir.resolve();report.mkdir(parents=True,exist_ok=True)
 out={'version':'3YD-V9-r0','parent':str(PARENT),'selected_mode':'control','tested_latency_mode':'A','memory_goal_achieved':False,'fixed':{},'regression':{},'shared':{},'events':{},'verification':{}}
 out['data_selection']={'formal':[str(rs/k) for k in ('fixed','moving','perturbed','events')]+[str(shared)],'excluded_shared':str(rs/'shared'),'shared_queries':str(ROOT/'datasets/v9_shared_segments_v2')}
 raw={}
 for mid,label in MAPS:
  ds={mode:rs/'fixed'/mode/mid for mode in ('control','A','B','C')};data={m:read(d) for m,d in ds.items()}
  assert all(len(v)==(30 if m in ('control','A') else 10) for m,v in data.items())
  raw['fixed',mid]=data
  out['fixed'][mid]={'label':label,'arms':{m:{k:summary(v) for k,v in parts(rows,'fixed').items()} for m,rows in data.items()},
   'parity':{m:pair(ds['control'],ds[m],strict=m=='A') for m in ('A','B','C')},
   'runs_sha256':{m:sha(d/'runs.jsonl') for m,d in ds.items()},
   'preparation':{m:{'base':json.loads((d/'preparation.json').read_text()),'V9':json.loads((d/'v9_preparation.json').read_text())} for m,d in ds.items()}}
 for kind in ('moving','perturbed'):
  out['regression'][kind]={}
  for mid,label in MAPS:
   ds={mode:rs/kind/mode/mid for mode in ('control','A')};data={m:read(d) for m,d in ds.items()}
   assert all(len(v)==40 for v in data.values());raw[kind,mid]=data
   out['regression'][kind][mid]={'label':label,'arms':{m:{k:summary(v) for k,v in parts(rows,kind).items()} for m,rows in data.items()},
    'parity':pair(ds['control'],ds['A'],strict=True),'runs_sha256':{m:sha(d/'runs.jsonl') for m,d in ds.items()}}
 for mid,label in MAPS:
  ds={mode:shared/mode/mid for mode in ('control','B','C','ABC')};data={m:read(d) for m,d in ds.items()};raw['shared',mid]=data
  assert all(len(v)==4 for v in data.values())
  query=ROOT/'datasets/v9_shared_segments_v2'/(mid+'.json');query_data=json.loads(query.read_text())
  assert len({json.dumps(q['start'],sort_keys=True) for q in query_data['queries']})==4
  assert len({json.dumps(q['goal'],sort_keys=True) for q in query_data['queries']})==4
  assert all(json.loads((d/'protocol.json').read_text())['queries_sha256']==sha(query) for d in ds.values())
  assert not any(r.get('fullroute',{}).get('reused') or r.get('task_progress',{}).get('reused') for rows in data.values() for r in rows[1:])
  out['shared'][mid]={'label':label,'arms':{m:{'initial':summary(rows[:1]),'new_tasks':summary(rows[1:])} for m,rows in data.items()},
   'parity':{m:pair(ds['control'],ds[m]) for m in ('B','C','ABC')},
   'per_task':{m:[{'id':r['query_id'],'wall_s':r['wall_ms']/1000,'L3_s':measure(r,'l3_s'),'strict':r['success'],
      'whole_route_hit':r.get('fullroute',{}).get('reused',False),'task_continuation_hit':r.get('task_progress',{}).get('reused',False),
      'segment_match':r.get('v9_local',{}).get('segments',{}),'local_success':r.get('v9_local',{}).get('success',False),
      'stage_wall_ms':r.get('v9_local',{}).get('wall_ms'),'failure':r.get('v9_local',{}).get('failure')} for r in rows[1:]] for m,rows in data.items()},
   'runs_sha256':{m:sha(d/'runs.jsonl') for m,d in ds.items()}}
 for mode in ('A','ABC'):
  out['events'][mode]={}
  for label in ('semantic','free_space'):
   d=rs/'events'/mode/label;s=json.loads((d/'summary.json').read_text());assert s['assertions_passed'] and s['events']==11
   out['events'][mode][label]=s
 out['verification']=verify(rs,shared)
 (report/'COMPARISON.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
 plot(raw,report)
 for mid,v in out['fixed'].items():
  print(mid,{m:{'strict':[a['all']['success'],a['all']['n']],'first_e2e':a['first']['metrics']['e2e_s'],'L3':a['first']['metrics']['l3_s']} for m,a in v['arms'].items()})

def plot(raw,report):
 import matplotlib;matplotlib.use('Agg')
 from matplotlib import pyplot as plt,font_manager as fm
 font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
 fm.fontManager.addfont(font);assert fm.FontProperties(fname=font).get_name()=='SimSun'
 plt.rcParams.update({'font.family':'SimSun','font.size':11,'axes.unicode_minus':False})
 for kind in ('fixed','repeat','moving','perturbed'):
  source_kind='fixed' if kind=='repeat' else kind
  fig,axes=plt.subplots(2,3,figsize=(15,8))
  for r,(mid,label) in enumerate(MAPS):
   panels=[('e2e_s','first','首次端到端 / s'),('l3_s','first','首次L3流水线 / s'),('tree_rss_gib','all','进程树峰值RSS / GiB')] if kind=='fixed' else ([('e2e_s','repeat','重复端到端 / s'),('l3_s','repeat','重复L3流水线 / s'),('tree_rss_gib','repeat','重复进程树峰值RSS / GiB')] if kind=='repeat' else [('e2e_s','moving','变化请求端到端 / s'),('l3_s','moving','L3流水线 / s'),('tree_rss_gib','moving','进程树峰值RSS / GiB')])
   for c,(metric,split,title) in enumerate(panels):
    vals=[[measure(x,metric) for x in parts(raw[source_kind,mid][m],source_kind)[split] if measure(x,metric) is not None] for m in ('control','A')]
    ax=axes[r,c];ax.boxplot(vals,labels=[f'r11对照 n={len(vals[0])}',f'V9 A n={len(vals[1])}'],whis=1.5,showfliers=True);ax.set_title(label+'\n'+title);ax.grid(axis='y',alpha=.25)
  fig.suptitle('V9-r0：'+{'fixed':'地图处理复用','repeat':'重复查询分布','moving':'变化起点回归','perturbed':'3 cm／2°偏差回归'}[kind],fontsize=17)
  fig.text(.5,.025,'箱体Q1–Q3；中线中位数；须为1.5×IQR内最远值；圆点离群值；线性坐标。\nL3含适配、实际地图ACK与严格终验；不同分项中位数不能相加。成功率另列表。',ha='center')
  fig.tight_layout(rect=(0,.1,1,.93));fig.savefig(report/(kind+'_boxplots.png'),dpi=170);plt.close(fig)
 fig,axes=plt.subplots(2,3,figsize=(16,8))
 for r,(mid,label) in enumerate(MAPS):
  modes=['control','A','B','C']
  for c,(metric,title) in enumerate([('e2e_s','首次端到端 / s'),('l3_s','首次L3 / s'),('tree_rss_gib','首次进程树峰值RSS / GiB')]):
   vals=[[measure(x,metric) for x in raw['fixed',mid][m] if x['repeat']==1] for m in modes]
   ax=axes[r,c];ax.boxplot(vals,labels=[f'{m} n={len(v)}' for m,v in zip(modes,vals)],whis=1.5,showfliers=True);ax.set_title(label+'\n'+title);ax.grid(axis='y',alpha=.25)
 fig.suptitle('V9独立对照：A地图复用；B轨迹段缓存；C直段验证及局部连接',fontsize=16)
 fig.text(.5,.02,'逐次首次查询，n=10／组；Q1–Q3、中位线、1.5×IQR须、圆点离群值；线性坐标。\nB/C保留严格验收，失败后有界回退的实际成本计入。',ha='center')
 fig.tight_layout(rect=(0,.1,1,.93));fig.savefig(report/'ablation_boxplots.png',dpi=170);plt.close(fig)
 fig,axes=plt.subplots(1,2,figsize=(13,5))
 for ax,(mid,label) in zip(axes,MAPS):
  for i,m in enumerate(('control','B','C','ABC')):
   v=[x['wall_ms']/1000 for x in raw['shared',mid][m][1:]]
   ax.scatter(np.full(len(v),i)+np.linspace(-.08,.08,len(v)),v,label=m,s=40)
  ax.set_xticks(range(4),['control','B','C','ABC']);ax.set_title(label+'：新任务 n=3／臂');ax.set_ylabel('实际端到端 / s');ax.grid(axis='y',alpha=.25)
 fig.suptitle('同一道路的不同起点／目标／任务ID：原始逐点结果',fontsize=16)
 fig.text(.5,.02,'每图仅3个相关诊断任务，样本不足以评估分布，显示原始点；不并入固定10组。\n整路径及任务续接命中与道路段复用分别报告；线性坐标。',ha='center')
 fig.tight_layout(rect=(0,.15,1,.9));fig.savefig(report/'shared_points.png',dpi=170);plt.close(fig)
if __name__=='__main__':main()
