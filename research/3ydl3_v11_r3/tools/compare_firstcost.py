"""V8-r11 same-batch data preparation comparison; all clocks are actual calls."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from analyze_v8 import MAPS,METRICS,measure,stats
from compare_v8_r3 import checked_success,audit_ast,libraries
from compare_v8_r6 import verify_cleanup
ROOT=Path(__file__).resolve().parents[1]
RESULTS=ROOT/'results'
REPORT=ROOT/'report'
PARENT=Path('/home/robot/workspaces/3yd_v8_r10_continuation_20261008')
FILES=('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','path.npy','l3_scope.npz')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(d):return [json.loads(s) for s in (d/'runs.jsonl').read_text().splitlines()]
def tag(r):return f"{r['query_id']}_{r['moving_start']['step']}_{r['repeat']}" if r.get('moving_start') else f"{r['query_id']}_{r['repeat']}"
def summary(rs):
 for r in rs:checked_success(r)
 return {'n':len(rs),'success':sum(r['success'] for r in rs),
         'metrics':{m:stats([measure(r,m) for r in rs]) for m in [*METRICS,'strict_audit_s']},
         'length_m':stats([r.get('length_m') for r in rs]),'right_fraction':stats([r.get('right_side_fraction') for r in rs]),
         'reuse_hits':sum(bool(r.get('task_progress',{}).get('reused')) for r in rs),
         'failures':[{'tag':tag(r),'code':r.get('failure_code')} for r in rs if not r['success']]}
def parts(rs,w):
 if w=='fixed':return {'all':rs,'first':[r for r in rs if r['repeat']==1],'repeat':[r for r in rs if r['repeat']>1]}
 return {'all':rs,'initial':[r for r in rs if r['moving_start']['kind']=='initial'],'moving':[r for r in rs if r['moving_start']['kind']=='moving']}
def detail(rs):
 keys=sorted({k for r in rs for a in r.get('recovery',{}).get('attempts',[]) for k in a.get('l2_timing',{}) if k.startswith('r11_')})
 return {k:stats([sum(a.get('l2_timing',{}).get(k,0.) for a in r.get('recovery',{}).get('attempts',[])) for r in rs]) for k in keys}
def pair(a,b,subset=False):
 old={tag(r):r for r in read(a)};new={tag(r):r for r in read(b)}
 assert new.keys()<=old.keys() if subset else old.keys()==new.keys()
 pa,pb=[json.loads((d/'protocol.json').read_text()) for d in (a,b)]
 for k in ('queries_sha256','map_sha256','vehicle','CPU','pose_offset_m','yaw_offset_deg','profiler'):
  assert pa[k]==pb[k],(k,a,b)
 assert not pa['profiler']
 assert {Path(p).name:h for p,h in libraries(a).items()}=={Path(p).name:h for p,h in libraries(b).items()}
 rows=[]
 for t,r in new.items():
  prev=old[t];assert prev['success']==r['success'] and r['success'],(t,r.get('failure_code'))
  eq={}
  for f in FILES:
   x,y=a/'cases'/t/f,b/'cases'/t/f
   if f.endswith('.npz'):
    eq[f]=np.array_equal(np.load(x)['mask'],np.load(y)['mask'])
   else:eq[f]=sha(x)==sha(y)
  assert all(eq.values()),(a,b,t,eq)
  for k in ('semantic_expected_grid_hash','semantic_expected_master_hash','server_costmap_content_hash'):
   assert prev['l3'].get(k)==r['l3'].get(k),(t,k)
  for k in ('actual_cost_hash','path_cost','dstar_expanded','state_memory_bytes'):
   assert prev.get('l2',{}).get(k)==r.get('l2',{}).get(k),(t,k)
  rows.append({'tag':t,'same_outputs':eq,'same_effective_costmap':True})
 return rows

def main():
 global RESULTS,REPORT
 parser=argparse.ArgumentParser()
 parser.add_argument('--results-root',type=Path,default=RESULTS)
 parser.add_argument('--report-dir',type=Path,default=REPORT)
 args=parser.parse_args();RESULTS=args.results_root.resolve();REPORT=args.report_dir.resolve()
 REPORT.mkdir(parents=True,exist_ok=True)
 result={'version':'3YD-V8-r11-firstcost','parent':str(PARENT),'fixed':{},'ablation':{},'verification':{}}
 raw={}
 for mid,label in MAPS:
  paths={a:RESULTS/'accepted_fixed'/a/mid for a in ('control','candidate')}
  data={a:read(p) for a,p in paths.items()};raw['fixed',mid]=data
  assert all(len(rs)==30 for rs in data.values())
  result['fixed'][mid]={'label':label,'arms':{a:{k:summary(v) for k,v in parts(rs,'fixed').items()} for a,rs in data.items()},
    'parity':pair(paths['control'],paths['candidate']),
    'stages_ms':{a:detail([r for r in rs if r['repeat']==1]) for a,rs in data.items()},
    'input_sha256':{a:sha(p/'runs.jsonl') for a,p in paths.items()}}
  result['ablation'][mid]={}
  for mode in ('A','B'):
   p=RESULTS/'accepted_ablation'/mode/mid;rs=read(p);assert len(rs)==10
   result['ablation'][mid][mode]={'first':summary(rs),'stages_ms':detail(rs),'parity':pair(paths['control'],p,True),'input_sha256':sha(p/'runs.jsonl')}
 for w in ('moving','perturbed'):
  if not (RESULTS/w).exists():continue
  result[w]={}
  for mid,label in MAPS:
   ds={a:RESULTS/w/a/mid for a in ('control','candidate')};data={a:read(p) for a,p in ds.items()}
   assert all(len(rs)==40 for rs in data.values());raw[w,mid]=data
   result[w][mid]={'label':label,'arms':{a:{k:summary(v) for k,v in parts(rs,w).items()} for a,rs in data.items()},
      'parity':pair(ds['control'],ds['candidate']),'input_sha256':{a:sha(p/'runs.jsonl') for a,p in ds.items()}}
 for kind,files in json.loads((ROOT/'L1_LOCK.json').read_text()).items():
  if kind in ('source_files','input_files','pixel_input_files'):
   assert all(sha(ROOT/f)==v for f,v in files.items());result['verification'][kind+'_unchanged']=len(files)
 for f in ('src/three_yd_v4/controller.py','src/three_yd_v4/r5_controller.py','src/three_yd_v5/controller.py'):
  assert audit_ast(ROOT/f)==audit_ast(PARENT/f)
 result['verification']['original_strict_audits_unchanged']=True
 for f in ('src/three_yd_v5/task_progress.py','src/three_yd_v5/progress_connector.py','src/three_yd_v5/fullroute_service.py','native/libcold_dstar.so','native/cold_dstar.cpp'):
  assert sha(ROOT/f)==sha(PARENT/f),f
 result['verification']['continuation_and_L2_native_unchanged']=True
 for p in [p for family in ('accepted_fixed','accepted_ablation','moving','perturbed') for p in (RESULTS/family).rglob('source_snapshot/hashes.json')]:
  assert all(sha(f)==v for f,v in json.loads(p.read_text()).items() if '/src/' in f),p
 parent=json.loads((PARENT/'RELEASE.json').read_text());assert all(sha(PARENT/f)==v for f,v in parent['files'].items())
 result['verification']['parent_files_unchanged']=len(parent['files'])
 result['verification']['cleanup']=[verify_cleanup(Path(str(p)[:-len('.supervisor.json')])) for p in RESULTS.rglob('*.supervisor.json')]
 result['verification']['tests']=(ROOT/'report/tests_final.txt').read_text().splitlines()[-1]
 result['dynamic_events']={}
 for mode in ('semantic','free_space'):
  p=RESULTS/'events'/mode/'summary.json'
  if p.exists():
   value=json.loads(p.read_text());assert value['assertions_passed'];result['dynamic_events'][mode]=value
 (REPORT/'COMPARISON.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 plot(raw)
 for mid,x in result['fixed'].items():
  print(mid,{a:{k:{'success':v['success'],'n':v['n'],'time':v['metrics']['e2e_s'],'L2':v['metrics']['l2_s'],'ACK':v['metrics']['map_sync_s'],'RSS':v['metrics']['tree_rss_gib']} for k,v in arm.items()} for a,arm in x['arms'].items()})

def plot(raw):
 import matplotlib;matplotlib.use('Agg')
 from matplotlib import pyplot as plt,font_manager as fm
 font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
 fm.fontManager.addfont(font);assert fm.FontProperties(fname=font).get_name()=='SimSun'
 plt.rcParams.update({'font.family':'SimSun','axes.unicode_minus':False,'font.size':11})
 for w in sorted({k[0] for k in raw}):
  fig,axes=plt.subplots(2,3,figsize=(15,8))
  for rr,(mid,label) in enumerate(MAPS):
   panels=[('e2e_s','first','首次完整规划 / s'),('e2e_s','repeat','重复请求 / s'),('tree_rss_gib','all','进程树峰值RSS / GiB')] if w=='fixed' else [('e2e_s','moving','变化起点 / s'),('l3_s','moving','L3流水线 / s'),('tree_rss_gib','moving','进程树峰值RSS / GiB')]
   for cc,(m,s,title) in enumerate(panels):
    vals=[[measure(r,m) for r in parts(raw[w,mid][a],w)[s] if measure(r,m) is not None] for a in ('control','candidate')]
    ax=axes[rr,cc];ax.boxplot(vals,labels=[f'r10对照 n={len(vals[0])}',f'r11候选 n={len(vals[1])}'],whis=1.5,showfliers=True)
    ax.set_title(label+'\n'+title);ax.grid(axis='y',alpha=.25)
  fig.suptitle('V8-r11：'+{'fixed':'首次数据处理优化','moving':'目标固定、起点沿原路线变化','perturbed':'起点偏差3 cm／2°'}[w],fontsize=17,fontweight='bold')
  fig.text(.5,.025,'箱体Q1–Q3；中线中位数；须为1.5×IQR内最远点；圆点离群值；线性坐标。成功率另列表。\n排除一次性初始化和导出；L3包含适配、ACK及终验，各阶段中位数不能相加。',ha='center',fontsize=11)
  fig.tight_layout(rect=(0,.09,1,.93));fig.savefig(REPORT/(w+'_boxplots.png'),dpi=170);plt.close(fig)
 # First-only independent ablations: n=10 each, do not mix repeats.
 fig,axes=plt.subplots(2,3,figsize=(16,8))
 for rr,(mid,label) in enumerate(MAPS):
  control=[r for r in raw['fixed',mid]['control'] if r['repeat']==1]
  data=[control,read(RESULTS/'accepted_ablation/A'/mid),read(RESULTS/'accepted_ablation/B'/mid),[r for r in raw['fixed',mid]['candidate'] if r['repeat']==1]]
  for cc,(m,title) in enumerate([('e2e_s','首次端到端 / s'),('l2_s','首次L2整体处理 / s'),('map_sync_s','首次地图同步与ACK / s')]):
   vals=[[measure(r,m) for r in rs] for rs in data];ax=axes[rr,cc]
   ax.boxplot(vals,labels=['r10 n=10','A n=10','B n=10','AB n=10'],whis=1.5,showfliers=True)
   ax.set_title(label+'\n'+title);ax.grid(axis='y',alpha=.25)
 fig.suptitle('V8-r11独立对照：A无语义数据处理；B稀疏软代价ACK分类',fontweight='bold',fontsize=16)
 fig.text(.5,.025,'逐次首次查询，n=10/组；箱体Q1–Q3，中线中位数，须1.5×IQR内最远点，圆点离群值；线性坐标。\n不同批次串行运行，无profiler；初始化与导出排除。',ha='center',fontsize=11)
 fig.tight_layout(rect=(0,.09,1,.93));fig.savefig(REPORT/'ablation_boxplots.png',dpi=170);plt.close(fig)
if __name__=='__main__':main()
