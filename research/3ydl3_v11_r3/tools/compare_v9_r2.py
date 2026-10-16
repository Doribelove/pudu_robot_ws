"""R2 same-batch review; cold setup and query costs remain separate."""
import argparse,json
from pathlib import Path
import numpy as np
from compare_v9_review import sha,load,save,exact,verify
from compare_v9 import read,pair,parts,summary
from analyze_v8 import MAPS,measure,stats
ROOT=Path(__file__).resolve().parents[1]

def median(item,arm,part,metric):return item['arms'][arm][part]['metrics'][metric]['median']
def fixed_review(rs):
 out={}
 for mid,label in MAPS:
  dirs={m:rs/'fixed'/m/mid for m in ('control','D','DP')};rows={m:read(d) for m,d in dirs.items()}
  assert all(len(v)==30 for v in rows.values())
  for d in dirs.values():
   p=load(d/'protocol.json');assert p['review_mode']=='S' and p['v9_mode']=='A' and p['CPU']==[12]
  item={'label':label,'arms':{m:{k:summary(v) for k,v in parts(r,'fixed').items()} for m,r in rows.items()},
        'parity':{'D':exact(dirs['control'],dirs['D']),'DP':pair(dirs['control'],dirs['DP'])},
        'runs_sha256':{m:sha(d/'runs.jsonl') for m,d in dirs.items()},
        'initialization':{m:{k:load(d/(k+'.json')) for k in ('preparation','auditor_preparation','cold_stack_ready','session_startup')} for m,d in dirs.items()}}
  assert all(v['all']['success']==30 for v in item['arms'].values())
  out[mid]=item
 # Predeclared: do not adopt P's additional geometry if either map fails to
 # improve first median by 3% over D, or repeat/query RSS regresses >5%.
 adopt=all(median(v,'DP','first','e2e_s')<=.97*median(v,'D','first','e2e_s') and
           median(v,'DP','repeat','e2e_s')<=1.05*median(v,'D','repeat','e2e_s') and
           median(v,'DP','all','tree_rss_gib')<=1.05*median(v,'D','all','tree_rss_gib') for v in out.values())
 return out,'DP' if adopt else 'D'

def plot(rs,out):
 import matplotlib;matplotlib.use('Agg')
 from matplotlib import pyplot as plt,font_manager as fm
 font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
 fm.fontManager.addfont(font);assert fm.FontProperties(fname=font).get_name()=='SimSun'
 plt.rcParams.update({'font.family':'SimSun','font.size':11,'axes.unicode_minus':False})
 for kind in ('first','repeat'):
  fig,axes=plt.subplots(2,3,figsize=(15,8));modes=['control','D','DP']
  for r,(mid,label) in enumerate(MAPS):
   for c,(metric,title) in enumerate([('e2e_s','实际查询端到端 / s'),('tree_rss_gib','查询进程树峰值RSS / GiB'),('python_pss_gib','独立Python进程PSS快照 / GiB')]):
    vals=[[measure(x,metric) for x in parts(read(rs/'fixed'/m/mid),'fixed')[kind]] for m in modes]
    ax=axes[r,c];ax.boxplot(vals,labels=[f'{m} n={len(v)}' for m,v in zip(modes,vals)],whis=1.5,showfliers=True);ax.set_title(label+'\n'+title);ax.grid(axis='y',alpha=.25)
  fig.suptitle('V9-r2：'+('首次完整查询' if kind=='first' else '重复完整查询'),fontsize=17)
  fig.text(.5,.02,'逐次原始记录；箱体Q1–Q3，中线中位数；须为1.5×IQR内最远值；圆点离群值；线性坐标。\ncontrol为父AS配置；各查询三次，首次n=10、重复n=20，共享同一批服务进程。初始化另计。',ha='center')
  fig.tight_layout(rect=(0,.1,1,.93));fig.savefig(ROOT/'report'/(kind+'_boxplots.png'),dpi=160);plt.close(fig)
 if 'cold' in out:
  fig,axes=plt.subplots(2,3,figsize=(15,8));candidate=out['candidate']
  for r,(mid,label) in enumerate(MAPS):
   for c,(field,title,scale) in enumerate([('cold_first_response_ms','准备至首次响应 / s',1000),('cold_tree_rss_peak_bytes','冷进程树峰值RSS / GiB',2**30),('auditor_wall_ms','审计字段初始化 / s',1000)]):
    ax=axes[r,c]
    for i,m in enumerate(('control',candidate)):
     v=[x[field]/scale for x in out['cold'][mid]['arms'][m]['raw']];ax.scatter(np.full(len(v),i)+np.linspace(-.07,.07,len(v)),v,s=45)
    ax.set_xticks([0,1],['control n=3',candidate+' n=3']);ax.set_title(label+'\n'+title);ax.grid(axis='y',alpha=.25)
  fig.suptitle('独立冷进程：每图每组3次，显示逐点值',fontsize=17)
  fig.text(.5,.02,'准备至响应包含地图/审计/服务准备、源码导出、ROS启动和首条查询；排除模块导入及结果导出。\nRSS按20 ms采样，包含Python与其ROS子进程；n=3不足以评估分布；线性坐标。',ha='center')
  fig.tight_layout(rect=(0,.1,1,.93));fig.savefig(ROOT/'report/cold_points.png',dpi=160);plt.close(fig)
 if 'shared' in out:
  fig,axes=plt.subplots(1,2,figsize=(13,5))
  for ax,(mid,label) in zip(axes,MAPS):
   modes=['A','AB','ABM','AP','ABPM']
   for i,m in enumerate(modes):
    vals=[measure(r,'e2e_s') for r in read(rs/'shared'/m/mid)[1:]];ax.scatter(np.full(len(vals),i)+np.linspace(-.07,.07,len(vals)),vals,s=40)
   ax.set_xticks(range(len(modes)),modes);ax.set_title(label+'：新任务 n=3／组');ax.set_ylabel('实际端到端 / s');ax.grid(axis='y',alpha=.25)
  fig.suptitle('共享道路诊断：AB/ABM隔离延迟检查准备M的影响',fontsize=16)
  fig.text(.5,.02,'每组仅3条相关新任务，显示逐点值；无整路径或任务续接命中，不并入固定10组。\n所有组启用父S和D；AP/ABPM另外启用P；线性坐标。',ha='center')
  fig.tight_layout(rect=(0,.15,1,.9));fig.savefig(ROOT/'report/shared_points.png',dpi=160);plt.close(fig)

def main():
 p=argparse.ArgumentParser();p.add_argument('--results-root',type=Path,required=True);p.add_argument('--phase',choices=['fixed','all'],default='all');a=p.parse_args();rs=a.results_root.resolve()
 fixed,candidate=fixed_review(rs)
 out={'version':'3YD-V9-r2','parent':'3YD-V9-r1 AS','fixed':fixed,'candidate':candidate,
      'P_adoption_rule':'first median improves >=3% on each map versus D; repeat and query-tree RSS regress <=5%; all strict checks pass',
      'data_selection':{'formal':str(rs),'diagnostic_excluded':True}}
 save(ROOT/'report/FIXED_COMPARISON.json',out)
 if a.phase=='fixed':
  plot(rs,out);print('CANDIDATE',candidate);print({mid:{m:round(median(v,m,'first','e2e_s'),4) for m in ('control','D','DP')} for mid,v in fixed.items()});return
 out['regression']={};out['cold']={};out['shared']={};out['events']={}
 for kind in ('moving','perturbed'):
  out['regression'][kind]={}
  for mid,label in MAPS:
   dirs={m:rs/kind/m/mid for m in ('control',candidate)};rows={m:read(d) for m,d in dirs.items()};assert all(len(v)==40 for v in rows.values())
   out['regression'][kind][mid]={'label':label,'arms':{m:{k:summary(v) for k,v in parts(r,kind).items()} for m,r in rows.items()},'parity':exact(dirs['control'],dirs[candidate]) if candidate=='D' else pair(dirs['control'],dirs[candidate])}
   assert all(v['moving']['success']==30 for v in out['regression'][kind][mid]['arms'].values())
 for mid,label in MAPS:
  arms={}
  for m in ('control',candidate):
   raw=[]
   for rep in range(1,4):
    d=rs/'cold'/f'r{rep}'/m/mid;r=read(d);assert len(r)==1 and r[0]['success']
    q=r[0];raw.append({'round':rep,'cold_first_response_ms':q['cold_first_response_ms'],'cold_tree_rss_peak_bytes':q['cold_tree_rss_peak_bytes'],
      'first_query_wall_ms':q['wall_ms'],'auditor_wall_ms':load(d/'auditor_preparation.json')['wall_ms'],
      'auditor_python_peak_bytes':load(d/'auditor_preparation.json')['python_rss_peak_bytes'],'runs_sha256':sha(d/'runs.jsonl')})
   arms[m]={'raw':raw,'metrics':{k:stats([r[k] for r in raw]) for k in raw[0] if k not in ('round','runs_sha256')}}
  out['cold'][mid]={'label':label,'arms':arms,'n_independent_processes_per_arm':3}
  ds={m:rs/'shared'/m/mid for m in ('A','AB','ABM','AP','ABPM')};rows={m:read(d) for m,d in ds.items()};assert all(len(v)==4 for v in rows.values())
  assert not any(r.get('fullroute',{}).get('reused') or r.get('task_progress',{}).get('reused') for rr in rows.values() for r in rr[1:])
  out['shared'][mid]={'label':label,'arms':{m:{'initial':summary(r[:1]),'new_tasks':summary(r[1:])} for m,r in rows.items()},
    'B_parity':pair(ds['A'],ds['AB']),'M_exact_parity':exact(ds['AB'],ds['ABM']),'P_B_parity':pair(ds['AP'],ds['ABPM'])}
 for m in ('control',candidate):
  out['events'][m]={}
  for mid,_ in MAPS:
   s=load(rs/'events'/m/mid/'summary.json');assert s['assertions_passed'] and s['events']==11;out['events'][m][mid]=s
 for mid,_ in MAPS:
  ae=load(rs/'events/control'/mid/'events.json');be=load(rs/'events'/candidate/mid/'events.json')
  assert [(r['label'],r.get('success'),r.get('trajectory_available')) for r in ae]==[(r['label'],r.get('success'),r.get('trajectory_available')) for r in be]
  if candidate=='D':
   for f in (rs/'events/control'/mid).glob('*_path.npy'):assert sha(f)==sha(rs/'events'/candidate/mid/f.name)
 out['verification']=verify(rs)
 parent=Path('/home/robot/workspaces/3yd_v9_r1_audit_20261009');expected='23a20dff44bfea224d977f0581b695ed28aeb6786c34afd7c0289a37ed5a9518'
 assert sha(parent/'RELEASE.json')==expected;files=load(parent/'RELEASE.json')['files'];assert all(sha(parent/f)==h for f,h in files.items())
 out['verification']['V9_r1_unchanged']={'files_verified':len(files),'release_sha256':expected}
 external=load(ROOT/'report/EXTERNAL_AUDIT_LOCK.json');assert all(sha(f)==h for f,h in external.items());out['verification']['external_audit_unchanged']=external
 out['cold_benefit_verified']=all(v['arms'][candidate]['metrics']['cold_first_response_ms']['median']<v['arms']['control']['metrics']['cold_first_response_ms']['median'] for v in out['cold'].values())
 out['selected_r2_mode']=candidate if out['cold_benefit_verified'] else 'control'
 save(ROOT/'report/COMPARISON.json',out);plot(rs,out);print('TESTED_CANDIDATE',candidate,'SELECTED',out['selected_r2_mode'],'COLD_BENEFIT',out['cold_benefit_verified'])
if __name__=='__main__':main()
