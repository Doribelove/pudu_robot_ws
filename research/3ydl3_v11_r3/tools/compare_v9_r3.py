"""R3 same-batch review; cold setup and query costs remain separate."""
import argparse,json
from pathlib import Path
import numpy as np
from compare_v9_review import sha,load,save,exact,verify
from compare_v9 import read,pair,parts,summary,tag
from analyze_v8 import MAPS,measure,stats
ROOT=Path(__file__).resolve().parents[1]

def median(item,arm,part,metric):return item['arms'][arm][part]['metrics'][metric]['median']
def check_mode(d,mode,protocol=True):
 if protocol:
  p=load(d/'protocol.json')
  assert p['r3_mode']==mode and p['review_mode']=='S' and p['v9_mode']=='A' and p['CPU']==[12] and p['r2_mode']=='control',d
 rows=load(d/'smac/lifecycle_evidence/readbacks.json')
 marked=[r for r in rows if r.get('r3_nonfast_delta')]
 assert bool(marked)==(mode=='I'),d
 assert all(r['r3_immutable_snapshot'] and r['r6_complete_snapshot_verified'] and r['success'] for r in marked),d
 return {'initial_delta_reads':len(marked),'all_readback_count':len(rows),'wire_bytes':sum(r.get('bytes',0) for r in rows)}
def fixed_review(rs):
 out={}
 for mid,label in MAPS:
  dirs={m:rs/'fixed'/m/mid for m in ('control','I')};rows={m:read(d) for m,d in dirs.items()}
  assert all(len(v)==30 for v in rows.values())
  modes={m:check_mode(d,m) for m,d in dirs.items()}
  item={'label':label,'arms':{m:{k:summary(v) for k,v in parts(r,'fixed').items()} for m,r in rows.items()},
        'parity':{'I':exact(dirs['control'],dirs['I'])},
        'runs_sha256':{m:sha(d/'runs.jsonl') for m,d in dirs.items()},
        'actual_mode_evidence':modes,
        'readback_wire_bytes':{m:sum(x.get('bytes',0) for x in load(d/'smac/lifecycle_evidence/readbacks.json')) for m,d in dirs.items()},
        'initialization':{m:{k:load(d/(k+'.json')) for k in ('preparation','auditor_preparation','cold_stack_ready','session_startup')} for m,d in dirs.items()}}
  assert all(v['all']['success']==30 for v in item['arms'].values())
  out[mid]=item
 return out,'I'
def parent_parity(rs):
 parent=Path('/home/robot/workspaces/3yd_v9_r2_firstplan_20261009/results/accepted_r2_v1/fixed/control')
 out={'parent':'V9-r2 AS','candidate':'V9-r3 control AS','maps':{},
      'canonical_path_hash_difference':'source_commit changes with version; raw geometry, pose/mask hashes and original check counts must agree'}
 for mid,_ in MAPS:
  a,b=parent/mid,rs/'fixed/control'/mid
  rows=pair(a,b,True);left={tag(r):r for r in read(a)}
  for r in read(b):
   t=tag(r);prev=left[t]
   for k in ('canonical_pose_hash','canonical_mask_hash','canonical_path_within_mask','canonical_sampled_pose_count','canonical_exact_footprint_check_count'):
    assert prev['l3'].get(k)==r['l3'].get(k),(t,k)
   x,y=[load(p/'cases'/t/'outcome.json') for p in (a,b)]
   for k in ('metrics','constraint_audit','traffic_audit'):assert x[k]==y[k],(t,k)
   keys=set(x['independent_audit'])-{'audit_cpu_ms','audit_wall_ms'}
   assert {k:x['independent_audit'][k] for k in keys}=={k:y['independent_audit'][k] for k in keys},t
  out['maps'][mid]={'n':len(rows),'records':rows,'all_raw_paths_maps_geometry_constraints_and_check_counts_identical':True}
 save(ROOT/'report/PARENT_OUTPUT_PARITY.json',out)
 return out

def plot(rs,out):
 import matplotlib;matplotlib.use('Agg')
 from matplotlib import pyplot as plt,font_manager as fm
 font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
 fm.fontManager.addfont(font);assert fm.FontProperties(fname=font).get_name()=='SimSun'
 plt.rcParams.update({'font.family':'SimSun','font.size':11,'axes.unicode_minus':False})
 for kind in ('first','repeat'):
  fig,axes=plt.subplots(2,3,figsize=(15,8));modes=['control','I']
  for r,(mid,label) in enumerate(MAPS):
   for c,(metric,title) in enumerate([('e2e_s','实际查询端到端 / s'),('tree_rss_gib','查询进程树峰值RSS / GiB'),('python_pss_gib','独立Python进程PSS快照 / GiB')]):
    vals=[[measure(x,metric) for x in parts(read(rs/'fixed'/m/mid),'fixed')[kind]] for m in modes]
    ax=axes[r,c];ax.boxplot(vals,labels=[f'{m} n={len(v)}' for m,v in zip(modes,vals)],whis=1.5,showfliers=True);ax.set_title(label+'\n'+title);ax.grid(axis='y',alpha=.25)
  fig.suptitle('V9-r3：'+('首次完整查询' if kind=='first' else '重复完整查询'),fontsize=17)
  fig.text(.5,.02,'逐次原始记录；箱体Q1–Q3，中线中位数；须为1.5×IQR内最远值；圆点离群值；线性坐标。\ncontrol为父AS配置；各查询三次，首次n=10、重复n=20，共享同一批服务进程。初始化另计。',ha='center')
  fig.tight_layout(rect=(0,.1,1,.93));fig.savefig(ROOT/'report'/(kind+'_boxplots.png'),dpi=160);plt.close(fig)
 if 'cold' in out:
  fig,axes=plt.subplots(2,3,figsize=(15,8));candidate=out['candidate']
  for r,(mid,label) in enumerate(MAPS):
   for c,(field,title,scale) in enumerate([('cold_first_response_ms','准备至首次响应 / s',1000),('cold_tree_rss_peak_bytes','冷进程树峰值RSS / GiB',2**30),('startup_wall_ms','ROS服务启动 / s',1000)]):
    ax=axes[r,c]
    for i,m in enumerate(('control',candidate)):
     v=[x[field]/scale for x in out['cold'][mid]['arms'][m]['raw']];ax.scatter(np.full(len(v),i)+np.linspace(-.07,.07,len(v)),v,s=45)
    ax.set_xticks([0,1],['control n=3',candidate+' n=3']);ax.set_title(label+'\n'+title);ax.grid(axis='y',alpha=.25)
  fig.suptitle('独立冷进程：每图每组3次，显示逐点值',fontsize=17)
  fig.text(.5,.02,'准备至响应包含地图/审计/服务准备、源码导出、ROS启动和首条查询；排除模块导入及结果导出。\nRSS按20 ms采样，包含Python与其ROS子进程；n=3不足以评估分布；线性坐标。',ha='center')
  fig.tight_layout(rect=(0,.1,1,.93));fig.savefig(ROOT/'report/cold_points.png',dpi=160);plt.close(fig)

def main():
 p=argparse.ArgumentParser();p.add_argument('--results-root',type=Path,required=True);p.add_argument('--phase',choices=['fixed','all'],default='all');a=p.parse_args();rs=a.results_root.resolve()
 fixed,candidate=fixed_review(rs)
 out={'version':'3YD-V9-r3','parent':'3YD-V9-r2 AS','fixed':fixed,'candidate':candidate,
      'I_adoption_rule':'first median improves >=3% on each map OR cold first-response median improves >=3% on each map; repeat and query-tree RSS regress <=5%; exact output/audit parity and all regressions pass',
      'data_selection':{'formal':str(rs),'diagnostic_excluded':True}}
 out['parent_output_parity']=parent_parity(rs)
 save(ROOT/'report/FIXED_COMPARISON.json',out)
 if a.phase=='fixed':
  plot(rs,out);print('CANDIDATE',candidate);print({mid:{m:round(median(v,m,'first','e2e_s'),4) for m in ('control','I')} for mid,v in fixed.items()});return
 out['regression']={};out['cold']={};out['events']={}
 for kind in ('moving','perturbed'):
  out['regression'][kind]={}
  for mid,label in MAPS:
   dirs={m:rs/kind/m/mid for m in ('control',candidate)};rows={m:read(d) for m,d in dirs.items()};assert all(len(v)==40 for v in rows.values())
   for m,d in dirs.items():check_mode(d,m)
   out['regression'][kind][mid]={'label':label,'arms':{m:{k:summary(v) for k,v in parts(r,kind).items()} for m,r in rows.items()},'parity':exact(dirs['control'],dirs[candidate])}
   assert all(v['moving']['success']==30 for v in out['regression'][kind][mid]['arms'].values())
 for mid,label in MAPS:
  arms={}
  for m in ('control',candidate):
   raw=[]
   for rep in range(1,4):
    d=rs/'cold'/f'r{rep}'/m/mid;r=read(d);assert len(r)==1 and r[0]['success']
    check_mode(d,m)
    q=r[0];raw.append({'round':rep,'cold_first_response_ms':q['cold_first_response_ms'],'cold_tree_rss_peak_bytes':q['cold_tree_rss_peak_bytes'],
      'first_query_wall_ms':q['wall_ms'],'startup_wall_ms':load(d/'session_startup.json')['wall_ms'],'wire_bytes':sum(x.get('bytes',0) for x in load(d/'smac/lifecycle_evidence/readbacks.json')),'auditor_wall_ms':load(d/'auditor_preparation.json')['wall_ms'],
      'auditor_python_peak_bytes':load(d/'auditor_preparation.json')['python_rss_peak_bytes'],'runs_sha256':sha(d/'runs.jsonl')})
   arms[m]={'raw':raw,'metrics':{k:stats([r[k] for r in raw]) for k in raw[0] if k not in ('round','runs_sha256')}}
  parity=[exact(rs/'cold'/f'r{rep}'/'control'/mid,rs/'cold'/f'r{rep}'/candidate/mid) for rep in range(1,4)]
  for rep in range(1,4):
   for m in ('control',candidate):exact(rs/'fixed'/m/mid,rs/'cold'/f'r{rep}'/m/mid)
  out['cold'][mid]={'label':label,'arms':arms,'n_independent_processes_per_arm':3,'parity':parity,'each_matches_fixed_first':True}
 for m in ('control',candidate):
  out['events'][m]={}
  for mid,_ in MAPS:
   d=rs/'events'/m/mid;s=load(d/'summary.json');assert s['assertions_passed'] and s['events']==11
   s['actual_mode_evidence']=check_mode(d,m,protocol=False);out['events'][m][mid]=s
 for mid,_ in MAPS:
  ae=load(rs/'events/control'/mid/'events.json');be=load(rs/'events'/candidate/mid/'events.json')
  assert [(r['label'],r.get('success'),r.get('trajectory_available')) for r in ae]==[(r['label'],r.get('success'),r.get('trajectory_available')) for r in be]
  for f in (rs/'events/control'/mid).glob('*_path.npy'):assert sha(f)==sha(rs/'events'/candidate/mid/f.name)
 out['verification']=verify(rs)
 parent=Path('/home/robot/workspaces/3yd_v9_r1_audit_20261009');expected='23a20dff44bfea224d977f0581b695ed28aeb6786c34afd7c0289a37ed5a9518'
 assert sha(parent/'RELEASE.json')==expected;files=load(parent/'RELEASE.json')['files'];assert all(sha(parent/f)==h for f,h in files.items())
 out['verification']['V9_r1_unchanged']={'files_verified':len(files),'release_sha256':expected}
 parent2=Path('/home/robot/workspaces/3yd_v9_r2_firstplan_20261009');expected2='a8bb54df366fb0e75ff8b4aa85f321eb217d8e14e3fc1537b1f25719aa6cdc19'
 assert sha(parent2/'RELEASE.json')==expected2;files2=load(parent2/'RELEASE.json')['files'];assert all(sha(parent2/f)==h for f,h in files2.items())
 out['verification']['V9_r2_unchanged']={'files_verified':len(files2),'release_sha256':expected2}
 external=load(ROOT/'report/EXTERNAL_AUDIT_LOCK.json');assert all(sha(f)==h for f,h in external.items());out['verification']['external_audit_unchanged']=external
 out['cold_benefit_verified']=all(v['arms'][candidate]['metrics']['cold_first_response_ms']['median']<=.97*v['arms']['control']['metrics']['cold_first_response_ms']['median'] for v in out['cold'].values())
 out['first_benefit_verified']=all(median(v,'I','first','e2e_s')<=.97*median(v,'control','first','e2e_s') for v in out['fixed'].values())
 out['repeat_and_memory_gate']=all(median(v,'I','repeat','e2e_s')<=1.05*median(v,'control','repeat','e2e_s') and median(v,'I','all','tree_rss_gib')<=1.05*median(v,'control','all','tree_rss_gib') for v in out['fixed'].values())
 out['selected_r3_mode']='I' if (out['cold_benefit_verified'] or out['first_benefit_verified']) and out['repeat_and_memory_gate'] else 'control'
 save(ROOT/'report/COMPARISON.json',out);plot(rs,out);print('TESTED_CANDIDATE',candidate,'SELECTED',out['selected_r3_mode'],'COLD_BENEFIT',out['cold_benefit_verified'])
if __name__=='__main__':main()
