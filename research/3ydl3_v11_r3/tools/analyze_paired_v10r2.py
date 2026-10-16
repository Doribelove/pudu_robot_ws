"""Raw paired ten-map metrics and original strict-delivery checks."""
from pathlib import Path
import argparse,json,hashlib,collections,csv
import numpy as np
E=Path(__file__).resolve().parents[1];P=Path(json.loads((E/'TASK.json').read_text())['parent']);R=E/'report'
a=argparse.ArgumentParser();a.add_argument('--root',type=Path,default=E/'results/paired');a.add_argument('--partial',action='store_true');a.add_argument('--report-dir',type=Path,default=R);args=a.parse_args();root=args.root;R=args.report_dir;R.mkdir(parents=True,exist_ok=True)
maps=json.loads((E/'MAPS.json').read_text())['maps'];rows=[];checks=[];same=collections.Counter();diffs=[];cold=[]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
pose=lambda q:[q[k] for k in ('x','y','yaw_rad')] if isinstance(q,dict) else q
for rnd in range(1,4):
 for m in maps:
  mid=m['map_id'];base=root/f'round{rnd}'/mid
  if not all((base/arm/'summary.json').exists() for arm in ('r1','r2')):
   assert args.partial, str(base)
   continue
  paired={}
  for arm in ('r1','r2'):
   d=base/arm;rr=[json.loads(v) for v in (d/'runs.jsonl').read_text().splitlines()]
   assert len(rr)==10 and [v['query_id'] for v in rr]==m['query_ids']
   assert json.loads((d/'map_input_verified.json').read_text())==m
   protocol=json.loads((d/'protocol.json').read_text());assert protocol['CPU']==[14] and protocol['l3_backend']=='independent' and not protocol['profiler']
   assert protocol['r2_mode']=='D'
   assert json.loads((d/'auditor_preparation.json').read_text())['diagnostics']['direct_initialized'] is True
   if arm=='r2':
    ready=json.loads((d/'session_startup.json').read_text())['readiness']
    assert ready['nodes_created'] and ready['lifecycle_active_observed'] and ready['fixed_settle_sleep_s']==0
   if arm=='r2' and m['kind']!='map_only':assert json.loads((d/'raster_storage.json').read_text())['enabled']
   assert protocol['curve_config_sha256']==sha((P if arm=='r1' else E)/'config/l3.json')
   sup=json.loads(d.with_suffix('.supervisor.json').read_text());assert sup['cleanup']['complete'] and sup['returncode']==0
   for r in rr:
    q=next(v for v in m['poses'] if v['id']==r['query_id']);case=d/'cases'/f"{r['query_id']}_1"
    assert list(r['start'])==pose(q['start']) and list(r['goal'])==pose(q['goal'])
    if r['success']:
     assert r['audit']['valid'] and r['constraints']['steering_jump_valid'] and r['constraints']['traffic']['valid']
     assert r['l3']['costmap_update_acknowledged'] and r['l3']['costmap_ack_mismatch_cells']==0 and r['l3']['costmap_ack_status']=='exact_effective_master_content_verified'
     assert r['l3']['semantic_exact_stable_observations']>=2
     assert r['l3_backend']['strict_success'] and not r['task_progress']['reused']
     path=np.load(case/'path.npy')
     for actual,want in ((path[0],pose(q['start'])),(path[-1],pose(q['goal']))):
      delta=actual-np.asarray(want);assert np.linalg.norm(delta[:2])<1e-8 and abs(np.arctan2(np.sin(delta[2]),np.cos(delta[2])))<1e-8
    else:assert not (case/'path.npy').exists()
    for at in r.get('l3_backend',{}).get('attempts',[]):assert at['old_l3_calls']==0
   paired[arm]=rr;checks.append({'round':rnd,'map_id':mid,'arm':arm,'strict':sum(r['success'] for r in rr),'cleanup':True})
   cold.append({'round':rnd,'map_id':mid,'arm':arm,**{k:json.loads((d/f).read_text())['wall_ms']/1000 for k,f in [('preparation_s','preparation.json'),('auditor_s','auditor_preparation.json'),('startup_s','session_startup.json')]},'first_response_s':rr[0]['cold_first_response_ms']/1000,'cold_tree_rss_mib':rr[0]['cold_tree_rss_peak_bytes']/2**20,'preparation_tree_rss_mib':json.loads((d/'preparation.json').read_text())['rss_peak_bytes']/2**20,'first_query_s':rr[0]['wall_ms']/1000})
  for old,new in zip(paired['r1'],paired['r2']):
   qid=old['query_id'];item={'round':rnd,'map_id':mid,'map_name':m['name'],'query_id':qid}
   for arm,r in [('r1',old),('r2',new)]:
    ps=r['memory_processes'];item.update({arm+'_success':r['success'],arm+'_failure':r['failure_code'],arm+'_wall_s':r['wall_ms']/1000,arm+'_tree_cpu_s':r['tree_cpu_ms']/1000,arm+'_tree_rss_mib':r['rss_peak_bytes']/2**20,arm+'_tree_pss_mib':sum(x['pss'] for x in ps)/2**20,arm+'_worker_rss_mib':ps[0]['rss']/2**20})
   if old['success'] and new['success']:
    parts={f:sha(base/'r1/cases'/f'{qid}_1'/f)==sha(base/'r2/cases'/f'{qid}_1'/f) for f in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','path.npy')}
    for k,v in parts.items():same[k]+=v
    if not all(parts.values()):diffs.append({**item,'identical':parts})
   rows.append(item)
def stats(group,key):
 aa=np.array([r['r1_'+key] for r in group]);bb=np.array([r['r2_'+key] for r in group])
 return {'n':len(group),'r1_median':float(np.median(aa)),'r2_median':float(np.median(bb)),'median_change_pct':float((np.median(bb)/np.median(aa)-1)*100),'r1_p95':float(np.quantile(aa,.95)),'r2_p95':float(np.quantile(bb,.95)),'sum_change_pct':float((bb.sum()/aa.sum()-1)*100),'paired_ratio_median':float(np.median(bb/aa))}
metrics=('wall_s','tree_cpu_s','tree_rss_mib','tree_pss_mib','worker_rss_mib');summary=[]
for m in maps:
 group=[r for r in rows if r['map_id']==m['map_id']]
 if not group:continue
 both=[r for r in group if r['r1_success'] and r['r2_success']]
 summary.append({'map_id':m['map_id'],'name':m['name'],'r1_success':sum(r['r1_success'] for r in group),'r2_success':sum(r['r2_success'] for r in group),'remaining':sorted({r['query_id'] for r in group if not r['r2_success']}),'all':{k:stats(group,k) for k in metrics},'common_success':{k:stats(both,k) for k in metrics}})
both=[r for r in rows if r['r1_success'] and r['r2_success']]
if not rows:print('NO_COMPLETED_PAIRS');raise SystemExit()
reg=[r for r in rows if r['r1_success'] and not r['r2_success']];fix=[r for r in rows if not r['r1_success'] and r['r2_success']]
result={'complete':len(rows)==300,'paired_n':len(rows),'r1_strict':sum(r['r1_success'] for r in rows),'r2_strict':sum(r['r2_success'] for r in rows),'regressions':reg,'new_successes':fix,'common_success_n':len(both),'common_success_identical_outputs':same,'output_differences':diffs,'all':{k:stats(rows,k) for k in metrics},'common_success':{k:stats(both,k) for k in metrics},'maps':summary,'cold':cold,'batch_checks':checks,'map_equal_weight':{k:{'geometric_mean_median_ratio':float(np.exp(np.mean([np.log(m['common_success'][k]['r2_median']/m['common_success'][k]['r1_median']) for m in summary]))),'median_map_ratio':float(np.median([m['common_success'][k]['r2_median']/m['common_success'][k]['r1_median'] for m in summary]))} for k in metrics}}
cold_pairs=[]
for rnd in range(1,4):
 for m in maps:
  pair={v['arm']:v for v in cold if v['round']==rnd and v['map_id']==m['map_id']}
  if set(pair)!=set(('r1','r2')):continue
  row={'round':rnd,'map_id':m['map_id'],'map_name':m['name']}
  for arm,v in pair.items():
   for k in ('preparation_s','auditor_s','startup_s','first_response_s','cold_tree_rss_mib','preparation_tree_rss_mib','first_query_s'):row[arm+'_'+k]=v[k]
  cold_pairs.append(row)
cold_keys=('preparation_s','auditor_s','startup_s','first_response_s','cold_tree_rss_mib','preparation_tree_rss_mib','first_query_s')
result['cold_common']={k:stats(cold_pairs,k) for k in cold_keys}
for m in summary:
 group=[v for v in cold_pairs if v['map_id']==m['map_id']]
 m['cold']={k:stats(group,k) for k in cold_keys}
result['cold_map_equal_weight']={k:float(np.exp(np.mean([np.log(m['cold'][k]['r2_median']/m['cold'][k]['r1_median']) for m in summary]))-1)*100 for k in cold_keys}
result['gates']={'correctness':len(rows)==300 and not reg,'request_no_material_regression_5pct':len(rows)==300 and result['common_success']['wall_s']['median_change_pct']<=5,'tree_rss_no_material_regression_2pct':len(rows)==300 and result['common_success']['tree_rss_mib']['median_change_pct']<=2,'first_response_map_balanced_10pct':len(rows)==300 and result['cold_map_equal_weight']['first_response_s']<=-10}
if not args.partial:
 with (R/'raw_cold_metrics.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(cold_pairs[0]));w.writeheader();w.writerows(cold_pairs)
file=R/('PARTIAL.json' if args.partial else 'COMPARISON.json');file.write_text(json.dumps(result,ensure_ascii=False,indent=2))
if not args.partial:
 with (R/'raw_paired_metrics.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
print(json.dumps({k:result[k] for k in ['complete','paired_n','r1_strict','r2_strict','gates','common_success','common_success_identical_outputs']},ensure_ascii=False))
