"""Raw paired ten-map metrics and original strict-delivery checks."""
from pathlib import Path
import argparse,json,hashlib,collections,csv
import numpy as np
E=Path(__file__).resolve().parents[1];P=Path(json.loads((E/'TASK.json').read_text())['parent']);R=E/'report'
a=argparse.ArgumentParser();a.add_argument('--root',type=Path,default=E/'results/paired');a.add_argument('--partial',action='store_true');args=a.parse_args();root=args.root
maps=json.loads((E/'MAPS.json').read_text())['maps'];rows=[];checks=[];same=collections.Counter();diffs=[];cold=[]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
pose=lambda q:[q[k] for k in ('x','y','yaw_rad')] if isinstance(q,dict) else q
for rnd in range(1,4):
 for m in maps:
  mid=m['map_id'];base=root/f'round{rnd}'/mid
  if not all((base/arm/'summary.json').exists() for arm in ('v9','v10')):
   assert args.partial, str(base)
   continue
  paired={}
  for arm in ('v9','v10'):
   d=base/arm;rr=[json.loads(v) for v in (d/'runs.jsonl').read_text().splitlines()]
   assert len(rr)==10 and [v['query_id'] for v in rr]==m['query_ids']
   assert json.loads((d/'map_input_verified.json').read_text())==m
   protocol=json.loads((d/'protocol.json').read_text());assert protocol['CPU']==[14] and protocol['l3_backend']=='independent' and not protocol['profiler']
   assert protocol['curve_config_sha256']==sha((P if arm=='v9' else E)/'config/l3.json')
   sup=json.loads(d.with_suffix('.supervisor.json').read_text());assert sup['cleanup']['complete'] and sup['returncode']==0
   for r in rr:
    q=next(v for v in m['poses'] if v['id']==r['query_id']);case=d/'cases'/f"{r['query_id']}_1"
    assert list(r['start'])==pose(q['start']) and list(r['goal'])==pose(q['goal'])
    if r['success']:
     assert r['audit']['valid'] and r['constraints']['steering_jump_valid'] and r['constraints']['traffic']['valid']
     assert r['l3']['costmap_update_acknowledged'] and r['l3']['costmap_ack_mismatch_cells']==0 and r['l3']['costmap_ack_status']=='exact_effective_master_content_verified'
     assert r['l3_backend']['strict_success'] and not r['task_progress']['reused']
     path=np.load(case/'path.npy')
     for actual,want in ((path[0],pose(q['start'])),(path[-1],pose(q['goal']))):
      delta=actual-np.asarray(want);assert np.linalg.norm(delta[:2])<1e-8 and abs(np.arctan2(np.sin(delta[2]),np.cos(delta[2])))<1e-8
    else:assert not (case/'path.npy').exists()
    for at in r.get('l3_backend',{}).get('attempts',[]):assert at['old_l3_calls']==0
   paired[arm]=rr;checks.append({'round':rnd,'map_id':mid,'arm':arm,'strict':sum(r['success'] for r in rr),'cleanup':True})
   cold.append({'round':rnd,'map_id':mid,'arm':arm,**{k:json.loads((d/f).read_text())['wall_ms']/1000 for k,f in [('preparation_s','preparation.json'),('auditor_s','auditor_preparation.json'),('startup_s','session_startup.json')]},'first_response_s':rr[0]['cold_first_response_ms']/1000})
  for old,new in zip(paired['v9'],paired['v10']):
   qid=old['query_id'];item={'round':rnd,'map_id':mid,'map_name':m['name'],'query_id':qid}
   for arm,r in [('v9',old),('v10',new)]:
    ps=r['memory_processes'];item.update({arm+'_success':r['success'],arm+'_failure':r['failure_code'],arm+'_wall_s':r['wall_ms']/1000,arm+'_tree_cpu_s':r['tree_cpu_ms']/1000,arm+'_tree_rss_mib':r['rss_peak_bytes']/2**20,arm+'_tree_pss_mib':sum(x['pss'] for x in ps)/2**20,arm+'_worker_rss_mib':ps[0]['rss']/2**20})
   if old['success'] and new['success']:
    parts={f:sha(base/'v9/cases'/f'{qid}_1'/f)==sha(base/'v10/cases'/f'{qid}_1'/f) for f in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','path.npy')}
    for k,v in parts.items():same[k]+=v
    if not all(parts.values()):diffs.append({**item,'identical':parts})
   rows.append(item)
def stats(group,key):
 aa=np.array([r['v9_'+key] for r in group]);bb=np.array([r['v10_'+key] for r in group])
 return {'n':len(group),'v9_median':float(np.median(aa)),'v10_median':float(np.median(bb)),'median_change_pct':float((np.median(bb)/np.median(aa)-1)*100),'v9_p95':float(np.quantile(aa,.95)),'v10_p95':float(np.quantile(bb,.95)),'sum_change_pct':float((bb.sum()/aa.sum()-1)*100),'paired_ratio_median':float(np.median(bb/aa))}
metrics=('wall_s','tree_cpu_s','tree_rss_mib','tree_pss_mib','worker_rss_mib');summary=[]
for m in maps:
 group=[r for r in rows if r['map_id']==m['map_id']]
 if not group:continue
 both=[r for r in group if r['v9_success'] and r['v10_success']]
 summary.append({'map_id':m['map_id'],'name':m['name'],'v9_success':sum(r['v9_success'] for r in group),'v10_success':sum(r['v10_success'] for r in group),'remaining':sorted({r['query_id'] for r in group if not r['v10_success']}),'all':{k:stats(group,k) for k in metrics},'common_success':{k:stats(both,k) for k in metrics}})
both=[r for r in rows if r['v9_success'] and r['v10_success']]
if not rows:print('NO_COMPLETED_PAIRS');raise SystemExit()
reg=[r for r in rows if r['v9_success'] and not r['v10_success']];fix=[r for r in rows if not r['v9_success'] and r['v10_success']]
result={'complete':len(rows)==300,'paired_n':len(rows),'v9_strict':sum(r['v9_success'] for r in rows),'v10_strict':sum(r['v10_success'] for r in rows),'regressions':reg,'new_successes':fix,'common_success_n':len(both),'common_success_identical_outputs':same,'output_differences':diffs,'all':{k:stats(rows,k) for k in metrics},'common_success':{k:stats(both,k) for k in metrics},'maps':summary,'cold':cold,'batch_checks':checks,'map_equal_weight':{k:{'geometric_mean_median_ratio':float(np.exp(np.mean([np.log(m['common_success'][k]['v10_median']/m['common_success'][k]['v9_median']) for m in summary]))),'median_map_ratio':float(np.median([m['common_success'][k]['v10_median']/m['common_success'][k]['v9_median'] for m in summary]))} for k in metrics}}
result['gates']={'correctness':len(rows)==300 and not reg,'wall_5pct':len(rows)==300 and result['common_success']['wall_s']['median_change_pct']<=-5,'tree_rss_5pct':len(rows)==300 and result['common_success']['tree_rss_mib']['median_change_pct']<=-5}
file=R/('PARTIAL.json' if args.partial else 'COMPARISON.json');file.write_text(json.dumps(result,ensure_ascii=False,indent=2))
if not args.partial:
 with (R/'raw_paired_metrics.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
print(json.dumps({k:result[k] for k in ['complete','paired_n','v9_strict','v10_strict','gates','common_success','common_success_identical_outputs']},ensure_ascii=False))
