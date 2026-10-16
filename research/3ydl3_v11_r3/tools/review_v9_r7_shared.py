"""Original related but distinct tasks; never fixed10 or tail-distribution proof."""
import argparse,json
from pathlib import Path
from compare_v9 import read,summary
from compare_v9_r7 import compare,mode_evidence
from compare_v9_review import load,save,sha
from compare_v8_r6 import verify_cleanup
from analyze_v8 import MAPS,measure
ROOT=Path(__file__).resolve().parents[1]
ARMS={'A':('A','control'),'AB':('AB','control'),'ABP':('AB','P')}


def main():
 p=argparse.ArgumentParser();p.add_argument('--results-root',type=Path,required=True);a=p.parse_args()
 root=a.results_root.resolve();out={'version':'3YD-V9-r7','source_cycle':2,'diagnostic_only':True,
     'fixed10':False,'results_root':str(root),'number_distinct_new_tasks_per_map_per_arm':3,'maps':{},
     'complete_task_strict_AND_candidate_accepted_required_for_local_or_segment_success':True,
     'timing_original_raw_points_only_no_significance_or_P95_claim':True}
 raw={}
 for mid,label in MAPS:
  query=ROOT/'datasets/v9_shared_segments_v2'/(mid+'.json');definition=load(query)
  assert not definition['fixed10'] and sha(Path(definition['source_path']))==definition['source_sha256']
  assert len({json.dumps(q['start'],sort_keys=True) for q in definition['queries']})==4
  assert len({json.dumps(q['goal'],sort_keys=True) for q in definition['queries']})==4
  dirs={arm:root/arm/mid for arm in ARMS};rows={arm:read(d) for arm,d in dirs.items()};raw[mid]=rows
  item={'label':label,'query_definition_sha256':sha(query),'arms':{},'parity':{}}
  for arm,(v9,r2) in ARMS.items():
   d=dirs[arm];rs=rows[arm];assert len(rs)==4 and all(r['success'] for r in rs)
   protocol=load(d/'protocol.json');assert protocol['queries_sha256']==sha(query) and protocol['diagnostic_subset']
   assert [r['query_id'] for r in rs]==[q['id'] for q in definition['queries']]
   assert not any(r['fullroute'].get('reused') or r['task_progress'].get('reused') for r in rs)
   evidence=mode_evidence(d,'CHNT',expected_v9_mode=v9,expected_r2_mode=r2)
   for r in rs:
    assert not r['l2']['reused_search_result'] and r['l2']['response_ms']>0
   item['arms'][arm]={'initial':summary(rs[:1]),'new_tasks':summary(rs[1:]),
       'no_whole_route_or_task_continuation_reuse':True,'new_L1_L2_bindings_per_request':True,
       'actual_native_scope':evidence,'cleanup':verify_cleanup(d),'runs_sha256':sha(d/'runs.jsonl'),
       'per_task':[{'query_id':r['query_id'],'strict_success':r['success'],'wall_ms':r['wall_ms'],
         'local_candidate':r['v9_local'],'L3':r['l3']} for r in rs]}
  for arm in ('AB','ABP'):
   parity=compare(dirs['A'],dirs[arm]);assert parity['quality_gate'] and parity['upstream_gate']
   item['parity'][arm]=parity
  baseline=item['arms']['A']['new_tasks']['metrics']['e2e_s']['median']
  item['new_task_median_change_pct']={arm:100*(item['arms'][arm]['new_tasks']['metrics']['e2e_s']['median']/baseline-1) for arm in ('AB','ABP')}
  out['maps'][mid]=item
 out['joint_functional_and_quality_gate']=all(v['parity'][a]['quality_gate'] and v['parity'][a]['upstream_gate'] for v in out['maps'].values() for a in ('AB','ABP'))
 out['both_maps_median_improve_at_least_3_percent']={arm:all(v['new_task_median_change_pct'][arm]<=-3 for v in out['maps'].values()) for arm in ('AB','ABP')}
 out['adopt_B_or_P_by_default']=False
 out['reason']='Only three related diagnostic new tasks per map; neither combined option improves both medians by >=3%. Retain B/P as explicit diagnostics.'
 save(ROOT/'report/R7_SHARED_COMPARISON.json',out);plot(raw)
 print('JOINT_PASS',out['joint_functional_and_quality_gate'],'B_P_NOT_DEFAULT',out['both_maps_median_improve_at_least_3_percent'])
 for mid,v in out['maps'].items():print(mid,v['new_task_median_change_pct'])


def plot(raw):
 import numpy as np
 import matplotlib;matplotlib.use('Agg')
 from matplotlib import pyplot as plt,font_manager as fm
 font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
 fm.fontManager.addfont(font);assert fm.FontProperties(fname=font).get_name()=='SimSun'
 plt.rcParams.update({'font.family':'SimSun','font.size':10,'axes.unicode_minus':False})
 fig,axes=plt.subplots(2,3,figsize=(15,8))
 for rr,(mid,label) in enumerate(MAPS):
  for cc,(metric,title) in enumerate((('e2e_s','新任务实际端到端 / s'),('tree_rss_gib','新任务进程树RSS / GiB'),('python_pss_gib','独立Python进程PSS快照 / GiB'))):
   ax=axes[rr,cc]
   for i,arm in enumerate(ARMS):ax.scatter(i+np.linspace(-.07,.07,3),[measure(r,metric) for r in raw[mid][arm][1:]],s=40)
   ax.set_xticks([0,1,2],[arm+' n=3' for arm in ARMS]);ax.set_title(label+'\n'+title);ax.grid(axis='y',alpha=.25)
 fig.suptitle('V9-r7：完整任务存图与跨任务段缓存联合诊断',fontsize=17)
 fig.text(.5,.025,'原固定v2三个不同起点与终点的新任务；每任务一次；均共用CHNT；首次建立缓存任务单列。\n'
          '逐点原始值，n=3不足以评价分布/尾部或推广fixed10；线性坐标；RSS含ROS后代，PSS为Python快照；严格成功另列表。',ha='center')
 fig.tight_layout(rect=(0,.1,1,.93));fig.savefig(ROOT/'report/shared_r7.png',dpi=160);plt.close(fig)

if __name__=='__main__':main()
