"""V9-r1 raw records, exact output/audit parity and ownership verification."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from analyze_v8 import MAPS,METRICS,measure,stats
from compare_v9 import read,tag,pair,summary,parts
from compare_v8_r3 import audit_ast,libraries
from compare_v8_r6 import verify_cleanup
ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/home/robot/workspaces/3yd_v9_r0_l3_20261009')
FROZEN=Path('/home/robot/workspaces/3yd_v8_r11_firstcost_20261009')

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):return json.loads(Path(p).read_text())
def save(p,v):Path(p).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')

def exact(a,b):
 rows=pair(a,b,True)
 left={tag(r):r for r in read(a)}
 fields=('canonical_path_hash','canonical_pose_hash','canonical_mask_hash','canonical_path_within_mask',
         'canonical_sampled_pose_count','canonical_exact_footprint_check_count')
 for row in read(b):
  t=tag(row)
  assert all(left[t]['l3'].get(k)==row['l3'].get(k) for k in fields),(t,'canonical checks')
  x,y=[load(d/'cases'/t/'outcome.json') for d in (a,b)]
  for k in ('metrics','constraint_audit','traffic_audit'):
   assert x[k]==y[k],(t,k)
  keys=set(x['independent_audit'])-{'audit_cpu_ms','audit_wall_ms'}
  assert {k:x['independent_audit'][k] for k in keys}=={k:y['independent_audit'][k] for k in keys},(t,'independent audit')
 return {'n':len(rows),'all_outputs_and_audit_decisions_identical':True,'records':rows}

def verify(formal):
 out={};lock=load(ROOT/'L1_LOCK.json')
 for section in ('source_files','input_files','pixel_input_files'):
  assert all(sha(ROOT/f)==h for f,h in lock[section].items());out[section]=len(lock[section])
 for f in ('src/three_yd_v4/controller.py','src/three_yd_v4/r5_controller.py','src/three_yd_v5/controller.py'):
  assert audit_ast(ROOT/f)==audit_ast(FROZEN/f),f
 out['original_strict_audit_AST_unchanged']=True
 for f in ('src/three_yd_v5/task_progress.py','src/three_yd_v5/progress_connector.py','native/libcold_dstar.so','native/cold_dstar.cpp'):
  assert sha(ROOT/f)==sha(FROZEN/f),f
 out['continuation_L2_native_unchanged']=True
 for parent,expected in [(PARENT,'3e5aee3c83587a166226c0b2d1764aae6024da9a0b0382b86f18dd847d481a76'),
                         (FROZEN,'3e19256a3c09c95cf3f42dc9ce3bc73a5d396580445f12e7bb34fe5cae867bbf')]:
  assert sha(parent/'RELEASE.json')==expected
  files=load(parent/'RELEASE.json')['files'];assert all(sha(parent/f)==h for f,h in files.items())
  out[parent.name]={'files_verified':len(files),'release_sha256':expected}
 archive=Path('/home/robot/workspaces/3yd_frozen_baselines/3YD-V8-r11-AB_20261009/snapshot.tar.gz')
 assert sha(archive)=='df7efcd136edb48e452e60dd14a40899c360ee30572fa3f0a37bf040b6f42d3c'
 out['frozen_archive_unchanged']=True
 for manifest in ('PRODUCTION_ENTRY_LOCK.json','PLANNER_SOURCE_LOCK.json'):
  files=load(ROOT/'report'/manifest);files=files.get('planning_source_files',files)
  assert all(sha(Path(f) if Path(f).is_absolute() else ROOT/f)==h for f,h in files.items())
  out[manifest]={'files_verified':len(files)}
 snaps=[]
 for p in formal.rglob('source_snapshot/hashes.json'):
  source={f:h for f,h in load(p).items() if '/src/' in f or Path(f).name in ('env.bash','run_task_progress.py','run_progress_events.py')}
  assert all(sha(f)==h for f,h in source.items()),p
  snaps.append({'path':str(p),'planning_files':len(source)})
 out['formal_source_snapshots']=snaps
 out['cleanup']=[verify_cleanup(Path(str(p)[:-len('.supervisor.json')])) for p in ROOT.joinpath('results').rglob('*.supervisor.json')]
 out['tests']=(ROOT/'report/tests_final.txt').read_text().splitlines()[-1]
 return out

def plots(raw,report,candidate):
 import matplotlib;matplotlib.use('Agg')
 from matplotlib import pyplot as plt,font_manager as fm
 font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
 fm.fontManager.addfont(font);assert fm.FontProperties(fname=font).get_name()=='SimSun'
 plt.rcParams.update({'font.family':'SimSun','font.size':11,'axes.unicode_minus':False})
 for kind in ('fixed','repeat','moving','perturbed'):
  source='fixed' if kind=='repeat' else kind
  modes=['control','S','AS','ASK'] if source=='fixed' else ['control',candidate]
  split='first' if kind=='fixed' else 'repeat' if kind=='repeat' else 'moving'
  fig,axes=plt.subplots(2,3,figsize=(15,8))
  for r,(mid,label) in enumerate(MAPS):
   for c,(metric,title) in enumerate([('e2e_s','实际端到端 / s'),('tree_rss_gib','规划进程树峰值RSS / GiB'),('python_pss_gib','独立Python进程PSS快照 / GiB')]):
    vals=[[measure(x,metric) for x in parts(raw[source,mid][mode],source)[split]] for mode in modes]
    ax=axes[r,c];ax.boxplot(vals,labels=[f'{mode} n={len(v)}' for mode,v in zip(modes,vals)],whis=1.5,showfliers=True)
    ax.set_title(label+'\n'+title);ax.grid(axis='y',alpha=.25)
  fig.suptitle('V9-r1：'+{'fixed':'首次完整规划','repeat':'重复完整规划','moving':'精确变化起点','perturbed':'3 cm／2°偏差'}[kind],fontsize=17)
  fig.text(.5,.025,'原始逐次记录；箱体Q1–Q3，中线中位数；须为1.5×IQR内最远值；圆点为离群值；线性坐标。\nRSS包含Python与ROS子进程，PSS是单进程快照；成功率另列表。初始化与导出单独记录。',ha='center')
  fig.tight_layout(rect=(0,.1,1,.93));fig.savefig(report/(kind+'_boxplots.png'),dpi=160);plt.close(fig)
 fig,axes=plt.subplots(1,2,figsize=(13,5))
 for ax,(mid,label) in zip(axes,MAPS):
  modes=['A','AB','ACJ','ABCJ']
  for i,m in enumerate(modes):
   vals=[measure(r,'e2e_s') for r in raw['shared',mid][m][1:]]
   ax.scatter(np.full(len(vals),i)+np.linspace(-.08,.08,len(vals)),vals,s=40)
  ax.set_xticks(range(len(modes)),modes);ax.set_title(label+'：新任务 n=3／组');ax.set_ylabel('实际端到端 / s');ax.grid(axis='y',alpha=.25)
 fig.suptitle('共享道路诊断：A/AB及ACJ/ABCJ分别隔离B的影响',fontsize=16)
 fig.text(.5,.02,'每图仅3条相关新任务，样本不足以评估分布，显示逐点值；不并入固定10组。\n所有组启用S+K，J仅在后一对启用；无整路径或任务续接命中；线性坐标。',ha='center')
 fig.tight_layout(rect=(0,.15,1,.9));fig.savefig(report/'shared_points.png',dpi=160);plt.close(fig)

def main():
 p=argparse.ArgumentParser();p.add_argument('--results-root',type=Path,required=True);p.add_argument('--candidate',default='ASK');a=p.parse_args()
 rs=a.results_root.resolve();candidate=a.candidate;report=ROOT/'report';raw={}
 out={'version':'3YD-V9-r1','candidate':candidate,'fixed':{},'regression':{},'shared':{},'events':{},
      'data_selection':{'formal':str(rs),'diagnostic':str(ROOT/'results/review_diagnostic_v2'),'profile_excluded':True}}
 for kind in ('fixed','moving','perturbed'):
  for mid,label in MAPS:
   modes=['control','S','AS','ASK'] if kind=='fixed' else ['control',candidate]
   dirs={mode:rs/kind/mode/mid for mode in modes};rows={m:read(d) for m,d in dirs.items()};raw[kind,mid]=rows
   assert all(len(v)==(30 if kind=='fixed' else 40) for v in rows.values())
   item={'label':label,'arms':{m:{k:summary(v) for k,v in parts(r,kind).items()} for m,r in rows.items()},
         'parity':{m:exact(dirs['control'],dirs[m]) for m in modes[1:]},'runs_sha256':{m:sha(d/'runs.jsonl') for m,d in dirs.items()},
         'initialization':{m:{key:load(d/(key+'.json')) for key in ('preparation','v9_preparation','audit_storage','session_startup')} for m,d in dirs.items()}}
   assert all(v['all']['success']==v['all']['n'] for v in item['arms'].values())
   (out['fixed'] if kind=='fixed' else out['regression'].setdefault(kind,{}))[mid]=item
 for mid,label in MAPS:
  dirs={m:rs/'shared'/m/mid for m in ('A','AB','ACJ','ABCJ')};rows={m:read(d) for m,d in dirs.items()};raw['shared',mid]=rows
  assert all(len(v)==4 for v in rows.values())
  assert not any(r.get('fullroute',{}).get('reused') or r.get('task_progress',{}).get('reused') for rr in rows.values() for r in rr[1:])
  query=load(ROOT/'datasets/v9_shared_segments_v2'/(mid+'.json'))['queries']
  for key in ('start','goal'):assert len({json.dumps(q[key],sort_keys=True) for q in query})==4
  out['shared'][mid]={'label':label,'arms':{m:{'initial':summary(r[:1]),'new_tasks':summary(r[1:])} for m,r in rows.items()},
      'B_parity':{'AB':pair(dirs['A'],dirs['AB']),'ABCJ':pair(dirs['ACJ'],dirs['ABCJ'])},
      'per_task':{m:[{'id':r['query_id'],'wall_s':measure(r,'e2e_s'),'strict':r['success'],'local':r.get('v9_local',{})} for r in rr[1:]] for m,rr in rows.items()}}
 for m in ('control',candidate):
  out['events'][m]={}
  for mid,_ in MAPS:
   s=load(rs/'events'/m/mid/'summary.json');assert s['assertions_passed'] and s['events']==11
   out['events'][m][mid]=s
 for mid,_ in MAPS:
  aevents=load(rs/'events/control'/mid/'events.json');bevents=load(rs/'events'/candidate/mid/'events.json')
  assert [(r['label'],r.get('success'),r.get('trajectory_available')) for r in aevents]==[(r['label'],r.get('success'),r.get('trajectory_available')) for r in bevents]
  for f in (rs/'events/control'/mid).glob('*_path.npy'):assert sha(f)==sha(rs/'events'/candidate/mid/f.name),f
 out['verification']=verify(rs)
 save(report/'COMPARISON.json',out);plots(raw,report,candidate)
 for mid,data in out['fixed'].items():
  print(mid,{m:{part:{k:round(v['metrics'][k]['median'],4) for k in ('e2e_s','tree_rss_gib','python_pss_gib')} for part,v in arm.items()} for m,arm in data['arms'].items()})
if __name__=='__main__':main()
