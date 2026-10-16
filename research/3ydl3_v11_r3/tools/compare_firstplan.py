"""Actual-clock comparison of fixed and changing-start full-route requests."""
import ast,hashlib,json
from pathlib import Path
import numpy as np
from analyze_v8 import measure,stats,METRICS
from compare_v8_r3 import checked_success,read,common_protocol,libraries,audit_ast
from compare_v8_r6 import verify_cleanup
ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/home/robot/workspaces/3yd_v8_r7_fullroute_20261008')
MAPS=[('mengjia','万达3w地库（语义）'),('arena_large_250x300_2d_005','arena_large（无语义）')]
FILES=('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','path.npy')

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def key(r):return r['query_id'],(r.get('moving_start') or {}).get('step',''),r['repeat']
def tag(r):return '_'.join(str(x) for x in key(r) if x!='')
def subsets(rows,workload):
 if workload=='fixed':return {'all':rows,'first':[r for r in rows if r['repeat']==1],'repeat':[r for r in rows if r['repeat']>1]}
 return {'all':rows,'initial':[r for r in rows if r['moving_start']['kind']=='initial'],'moving':[r for r in rows if r['moving_start']['kind']=='moving']}
def summarize(rows,workload):
 for r in rows:checked_success(r)
 return {s:{'n':len(rs),'success':sum(r['success'] for r in rs),
   'metrics':{m:stats([measure(r,m) for r in rs]) for m in [*METRICS,'strict_audit_s']},
   'success_e2e_s':stats([r['wall_ms']/1000 for r in rs if r['success']]),
   'right_fraction':stats([r.get('right_side_fraction') for r in rs if r['success']]),
   'length_m':stats([r.get('length_m') for r in rs if r['success']]),
   'exact_route_hits':sum(bool(r.get('fullroute',{}).get('reused')) for r in rs),
   'failures':[{'tag':tag(r),'code':r.get('failure_code'),'L2_reference':bool(r.get('l2')),
        'attempts':[{'code':a.get('failure'),'native':a.get('l3_diagnostics',{}).get('termination_reason')} for a in r.get('recovery',{}).get('attempts',[])]} for r in rs if not r['success']]
 } for s,rs in subsets(rows,workload).items()}

def main():
 result={'version':'3YD-V8-r8','branch':'firstplan','parent':'new V8-r7 fullroute',
   'timing':'query wall clock excludes startup/preparation/export; includes all candidates, synchronisation and strict audits',
   'method':'CPU12 serial control then AB by map/workload; profiler off; historical parent only for dataset construction',
   'moving_query_definition':'fixed parent trajectory sampled at 20/50/80 percent arc; saved x/y/yaw; same fixed goal; no interpolated or resampled pose',
   'results':{},'verification':{}}
 data={}
 for workload in ('fixed','moving'):
  result['results'][workload]={}
  for mid,label in MAPS:
   dirs={a:ROOT/'results'/workload/a/mid for a in ('control','candidate')}
   rows={a:read(d) for a,d in dirs.items()};data[workload,mid]=rows
   assert all(len(v)==(30 if workload=='fixed' else 40) for v in rows.values())
   index={a:{key(r):r for r in rs} for a,rs in rows.items()};assert index['control'].keys()==index['candidate'].keys()
   pairs=[]
   for k,b in index['candidate'].items():
    a=index['control'][k];record={'tag':tag(b),'success_control':a['success'],'success_candidate':b['success']}
    for f in FILES:
     p,q=dirs['control']/'cases'/tag(a)/f,dirs['candidate']/'cases'/tag(b)/f
     record[f]=sha(p)==sha(q) if p.exists() and q.exists() else None
    record['length_delta_percent']=(100*(b['length_m']/a['length_m']-1) if a['success'] and b['success'] else None)
    record['right_delta']=(b['right_side_fraction']-a['right_side_fraction'] if a['success'] and b['success'] and a.get('right_side_fraction') is not None else None)
    pairs.append(record)
   prot=common_protocol(dirs['control'],dirs['candidate'])
   assert {Path(k).name:v for k,v in libraries(dirs['control']).items()}=={Path(k).name:v for k,v in libraries(dirs['candidate']).items()}
   item={'label':label,'arms':{a:summarize(r,workload) for a,r in rows.items()},'parity':pairs,
     'equal_file_count':{f:sum(r[f] is True for r in pairs) for f in FILES},'protocol_equal':prot,
     'runs_sha256':{a:sha(d/'runs.jsonl') for a,d in dirs.items()}}
   item['change_percent']={s:{m:{v:None if item['arms']['control'][s]['metrics'][m][v] in (None,0) or item['arms']['candidate'][s]['metrics'][m][v] is None else 100*(item['arms']['candidate'][s]['metrics'][m][v]/item['arms']['control'][s]['metrics'][m][v]-1) for v in ('median','p95','max')} for m in METRICS} for s in item['arms']['control']}
   result['results'][workload][mid]=item
 lock=json.loads((ROOT/'L1_LOCK.json').read_text())
 for kind in ('source_files','input_files','pixel_input_files'):
  assert all(sha(ROOT/f)==h for f,h in lock[kind].items()),kind
  result['verification'][kind+'_unchanged']=len(lock[kind])
 for f in ('src/three_yd_v5/controller.py','src/three_yd_v4/r5_controller.py','src/three_yd_v4/controller.py'):
  assert audit_ast(ROOT/f)==audit_ast(PARENT/f)
 result['verification']['strict_audit_unchanged']=True
 snapshots=[]
 for d in [ROOT/'results'/w/a/mid for w in ('fixed','moving') for a in ('control','candidate') for mid,_ in MAPS]:
  hashes=json.loads((d/'source_snapshot/hashes.json').read_text());src=[(p,h) for p,h in hashes.items() if '/src/' in p]
  assert all(sha(p)==h for p,h in src)
  snapshots.append({'directory':str(d),'planning_files_unchanged':len(src)})
 result['verification']['tested_source_snapshots']=snapshots
 result['dynamic_regression']={}
 for mode in ('semantic','free_space'):
  d=ROOT/'results/dynamic'/mode
  summary=json.loads((d/'summary.json').read_text());assert summary['assertions_passed']
  result['dynamic_regression'][mode]=summary
 parent=json.loads((PARENT/'RELEASE.json').read_text());assert all(sha(PARENT/f)==h for f,h in parent['files'].items())
 result['verification']['parent_release_files_unchanged']=len(parent['files'])
 result['verification']['cleaned_batches']=[verify_cleanup(Path(str(p)[:-len('.supervisor.json')])) for p in (ROOT/'results').rglob('*.supervisor.json')]
 (ROOT/'report/COMPARISON.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 plot(data)
 print(json.dumps({w:{mid:{a:{s:{'success':v['success'],'n':v['n'],'e2e':v['metrics']['e2e_s'],'rss':v['metrics']['tree_rss_gib']} for s,v in arm.items()} for a,arm in m['arms'].items()} for mid,m in maps.items()} for w,maps in result['results'].items()},ensure_ascii=False,indent=2))


def plot(data):
 import matplotlib;matplotlib.use('Agg')
 from matplotlib import pyplot as plt,font_manager as fm
 font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
 fm.fontManager.addfont(font);assert fm.FontProperties(fname=font).get_name()=='SimSun'
 plt.rcParams.update({'font.family':'SimSun','axes.unicode_minus':False,'font.size':12})
 for workload in ('fixed','moving'):
  fig,axes=plt.subplots(2,3,figsize=(15,8))
  for row,(mid,label) in enumerate(MAPS):
   configs=[('e2e_s','first','首次完整规划（秒）'),('e2e_s','repeat','相同起终点重复（秒）'),('tree_rss_gib','all','进程树峰值 RSS（GiB）')] if workload=='fixed' else [('e2e_s','moving','起点沿路线变化（秒）'),('l3_s','moving','变化起点 L3 流水线（秒）'),('tree_rss_gib','moving','变化起点进程树峰值 RSS（GiB）')]
   for col,(metric,subset,title) in enumerate(configs):
    groups=[[measure(r,metric) for r in subsets(data[workload,mid][a],workload)[subset] if measure(r,metric) is not None] for a in ('control','candidate')]
    ax=axes[row,col];bp=ax.boxplot(groups,labels=[f'r7 对照 n={len(groups[0])}',f'r8 AB n={len(groups[1])}'],patch_artist=True,whis=1.5,showfliers=True)
    for box,color in zip(bp['boxes'],('#bfd2ed','#b6d9c0')):box.set_facecolor(color)
    ax.set_title(label+'\n'+title);ax.grid(axis='y',alpha=.25)
  fig.suptitle('V8-r8：'+('固定 10 组 × 3 次' if workload=='fixed' else '目标固定，起点取原轨迹 20% / 50% / 80% 位置'),fontsize=17,fontweight='bold')
  fig.text(.5,.02,'箱体 Q1–Q3；中线中位数；须为 1.5×IQR 内最远点；圆点为离群点；线性坐标。失败查询也计入；缺失的层级计时不补零。\n排除初始化与导出；L3 包含层间适配及终验。成功率另见结果表。',ha='center',fontsize=11)
  fig.tight_layout(rect=(0,.085,1,.93));fig.savefig(ROOT/'report'/(workload+'_boxplots.png'),dpi=180);plt.close(fig)
if __name__=='__main__':main()
