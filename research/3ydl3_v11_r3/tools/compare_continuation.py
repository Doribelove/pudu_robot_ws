import hashlib,json
from pathlib import Path
import numpy as np
from analyze_v8 import measure,stats,METRICS,MAPS
from compare_v8_r3 import checked_success,audit_ast
from compare_v8_r6 import verify_cleanup
ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/home/robot/workspaces/3yd_v8_r9_progress_20261008')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return [json.loads(x) for x in (p/'runs.jsonl').read_text().splitlines()]
def tag(r):return f"{r['query_id']}_{r['moving_start']['step']}_{r['repeat']}" if r.get('moving_start') else f"{r['query_id']}_{r['repeat']}"
def subsets(rs,w):
 return {'all':rs,'first':[r for r in rs if r['repeat']==1],'repeat':[r for r in rs if r['repeat']>1]} if w=='fixed' else {'all':rs,'initial':[r for r in rs if r['moving_start']['kind']=='initial'],'moving':[r for r in rs if r['moving_start']['kind']=='moving']}
def summarize(rs):
 for r in rs:checked_success(r)
 return {'n':len(rs),'success':sum(r['success'] for r in rs),'reuse_hits':sum(bool(r['task_progress'].get('reused')) for r in rs),
  'metrics':{m:stats([measure(r,m) for r in rs]) for m in [*METRICS,'strict_audit_s']},
  'successful_e2e_s':stats([r['wall_ms']/1000 for r in rs if r['success']]),
  'length_m':stats([r.get('length_m') for r in rs if r['success']]),
  'right_fraction':stats([r.get('right_side_fraction') for r in rs if r['success']]),
  'failures':[{'tag':tag(r),'code':r.get('failure_code')} for r in rs if not r['success']]}
def main():
 result={'version':'3YD-V8-r10-fullroute-continuation','parent':str(PARENT),'results':{},'verification':{}}
 raw={}
 for w in ('fixed','moving','perturbed'):
  result['results'][w]={}
  for mid,label in MAPS:
   ds={a:ROOT/'results'/w/a/mid for a in ('control','candidate')};data={a:read(d) for a,d in ds.items()};raw[w,mid]=data
   index={a:{tag(r):r for r in v} for a,v in data.items()}
   assert index['control'].keys()==index['candidate'].keys()
   pairs=[]
   for t,b in index['candidate'].items():
    a=index['control'][t];pair={'tag':t,'old_success':a['success'],'success':b['success'],'path_equal':None}
    if a['success'] and b['success']:
     p,q=ds['control']/'cases'/t/'path.npy',ds['candidate']/'cases'/t/'path.npy'
     pair['path_equal']=np.array_equal(np.load(p),np.load(q))
    if w=='fixed':
     for f in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json'):
      assert sha(ds['control']/'cases'/t/f)==sha(ds['candidate']/'cases'/t/f),(t,f)
    if b['task_progress'].get('reused'):
     assert b['recovery']['l1_actual_ms']==0 and b['recovery']['all_candidates_controller_ms']==0,t
     assert b['l3']['backend_call_count']==0 and b['l2']['response_ms']==0,t
     assert b['l3']['costmap_update_acknowledged'] is True,t
    if w=='perturbed' and b['task_progress'].get('reused'):
     p=np.load(ds['candidate']/'cases'/f"{b['query_id']}_initial_1"/'path.npy')
     q=np.load(ds['candidate']/'cases'/t/'path.npy')
     connector=b['task_progress'].get('connector');assert connector is not None,t
     n=connector['connector_vertices']
     assert np.array_equal(q[0],np.asarray(b['start'])),t
     assert np.array_equal(q[n-1:],p[-len(q[n-1:]):]),(t,'retained tail modified')
     nominal=p[b['moving_start']['parent_index']:]
     expected=float(np.linalg.norm(np.diff(nominal[:,:2],axis=0),axis=1).sum())
     pair['length_delta_vs_nominal_suffix_percent']=100*(b['length_m']/expected-1)
     pair['connector_vertices']=n;pair['rest_is_original_suffix']=True
    if w=='moving' and b['success'] and b['task_progress'].get('reused') and b.get('moving_start'):
     initial=ds['candidate']/'cases'/f"{b['query_id']}_initial_1"/'path.npy'
     p=np.load(initial);q=np.load(ds['candidate']/'cases'/t/'path.npy')
     hits=np.flatnonzero(np.all(np.isclose(p,np.array(b['start']),atol=1e-7,rtol=0),axis=1))
     assert len(hits)==1 and np.array_equal(q,p[hits[0]:]),(t,'suffix changed')
     pair['exact_initial_path_suffix']=True
    pairs.append(pair)
   arms={a:{s:summarize(v) for s,v in subsets(rs,w).items()} for a,rs in data.items()}
   item={'label':label,'arms':arms,'paired_paths':pairs,'runs_sha256':{a:sha(d/'runs.jsonl') for a,d in ds.items()}}
   item['change_percent']={s:{m:{k:(100*(arms['candidate'][s]['metrics'][m][k]/arms['control'][s]['metrics'][m][k]-1)
    if arms['control'][s]['metrics'][m][k] not in (None,0) and arms['candidate'][s]['metrics'][m][k] is not None else None)
    for k in ('median','p95','max')} for m in METRICS} for s in arms['control']}
   shared=[t for t,a in index['control'].items() if a['success'] and index['candidate'][t]['success'] and (w=='fixed' or a['moving_start']['kind']=='moving')]
   item['paired_successful']={a:summarize([index[a][t] for t in shared]) for a in data}
   if w=='fixed':assert all(v['all']['success']==30 for v in arms.values()) and all(p['path_equal'] for p in pairs)
   else:assert arms['candidate']['all']['success']==40
   result['results'][w][mid]=item
 for kind,files in json.loads((ROOT/'L1_LOCK.json').read_text()).items():
  if kind in ('source_files','input_files','pixel_input_files'):
   assert all(sha(ROOT/f)==v for f,v in files.items());result['verification'][kind+'_unchanged']=len(files)
 for f in ('src/three_yd_v4/controller.py','src/three_yd_v4/r5_controller.py','src/three_yd_v5/controller.py'):
  assert audit_ast(ROOT/f)==audit_ast(PARENT/f)
 result['verification']['original_strict_audits_unchanged']=True
 for p in [ROOT/'results'/w/a/mid/'source_snapshot/hashes.json' for w in ('fixed','moving','perturbed') for a in ('control','candidate') for mid,_ in MAPS]:
  assert all(sha(f)==v for f,v in json.loads(p.read_text()).items() if '/src/' in f),p
 result['verification']['cleanup']=[verify_cleanup(Path(str(p)[:-len('.supervisor.json')])) for p in (ROOT/'results').rglob('*.supervisor.json')]
 parent=json.loads((PARENT/'RELEASE.json').read_text());assert all(sha(PARENT/f)==v for f,v in parent['files'].items())
 result['verification']['parent_files_unchanged']=len(parent['files'])
 result['verification']['tests']=(ROOT/'report/tests_final.txt').read_text().splitlines()[-1]
 for f in ('native/cold_dstar.cpp','native/libcold_dstar.so'):
  assert sha(ROOT/f)==sha(PARENT/f),f
 result['verification']['L2_native_unchanged']=True
 result['dynamic_events']={}
 for mode in ('semantic','free_space'):
  p=ROOT/'results/events'/mode/'summary.json';value=json.loads(p.read_text());assert value['assertions_passed']
  result['dynamic_events'][mode]=value
 (ROOT/'report/COMPARISON.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 plot(raw)
 for w,maps in result['results'].items():
  for mid,item in maps.items():print(w,mid,{a:{s:{'success':v['success'],'n':v['n'],'time':v['metrics']['e2e_s'],'RSS':v['metrics']['tree_rss_gib'],'hits':v['reuse_hits']} for s,v in arm.items()} for a,arm in item['arms'].items()})
def plot(raw):
 import matplotlib;matplotlib.use('Agg')
 from matplotlib import pyplot as plt,font_manager as fm
 font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
 fm.fontManager.addfont(font);assert fm.FontProperties(fname=font).get_name()=='SimSun'
 plt.rcParams.update({'font.family':'SimSun','axes.unicode_minus':False,'font.size':12})
 for w in ('fixed','moving','perturbed'):
  fig,axes=plt.subplots(2,3,figsize=(15,8))
  for rr,(mid,label) in enumerate(MAPS):
   metrics=[('e2e_s','first','首次完整规划 / s'),('e2e_s','repeat','相同起终点重复 / s'),('tree_rss_gib','all','进程树峰值RSS / GiB')] if w=='fixed' else [('e2e_s','moving','变化起点 / s'),('l3_s','moving','变化起点 L3流水线 / s'),('tree_rss_gib','moving','变化起点进程树峰值RSS / GiB')]
   for cc,(m,s,title) in enumerate(metrics):
    vals=[[measure(r,m) for r in subsets(raw[w,mid][a],w)[s] if measure(r,m) is not None] for a in ('control','candidate')]
    ax=axes[rr,cc];ax.boxplot(vals,labels=[f'r8逻辑对照 n={len(vals[0])}',f'r10完整续接 n={len(vals[1])}'],whis=1.5,showfliers=True)
    ax.set_title(label+'\n'+title);ax.grid(axis='y',alpha=.25)
  fig.suptitle('V8-r10：'+('固定两图10组×3次' if w=='fixed' else ('目标固定、起点沿已验证路线变化' if w=='moving' else '起点横向偏差3 cm、航向偏差2°')),fontsize=17,fontweight='bold')
  fig.text(.5,.02,'箱体Q1–Q3；中线中位数；须为1.5×IQR内最远点；圆点离群值；线性坐标。失败耗时计入，成功率另列表。\n排除一次性初始化、导出；L3包含适配及终验。复用层级显示本次实际工作，不沿用历史搜索计时。',ha='center',fontsize=11)
  fig.tight_layout(rect=(0,.09,1,.93));fig.savefig(ROOT/'report'/(w+'_boxplots.png'),dpi=170);plt.close(fig)
if __name__=='__main__':main()
