from pathlib import Path
import json,csv,hashlib,collections
import numpy as np
E=Path(__file__).resolve().parents[1];REPORT=E/'report';maps=json.loads((E/'MAPS.json').read_text())['maps'];rows=[];checks=[]
for rnd in range(1,4):
 for m in maps:
  d=E/'results/formal'/f'round{rnd}'/m['map_id'];p=d/'runs.jsonl'
  if not p.exists():continue
  protocol=json.loads((d/'protocol.json').read_text());verified=json.loads((d/'map_input_verified.json').read_text())
  assert verified==m
  rs=[json.loads(l) for l in p.read_text().splitlines()]
  for r in rs:
   q=next(q for q in m['poses'] if q['id']==r['query_id'])
   def pose(p):return [p[k] for k in ('x','y','yaw_rad')] if isinstance(p,dict) else p
   assert list(r['start'])==pose(q['start']) and list(r['goal'])==pose(q['goal'])
   assert not r.get('task_progress',{}).get('reused',False)
   c=d/'cases'/f"{r['query_id']}_1"
   if r['success']:
    assert r['audit']['valid'] and r['constraints']['steering_jump_valid'] and r['constraints']['traffic']['valid']
    assert r['l3_backend']['strict_success'] and (c/'path.npy').exists()
   for a in r.get('l3_backend',{}).get('attempts',[]):assert a.get('old_l3_calls',0)==0
   ps=r['memory_processes'];L2=r.get('l2',{}).get('v5',{}).get('success',False)
   flat={'map_id':m['map_id'],'map_name':m['name'],'round':rnd,'query_id':r['query_id'],'success':r['success'],'failure_code':r.get('failure_code',''),'wall_s':r['wall_ms']/1000,'tree_cpu_s':r['tree_cpu_ms']/1000,'tree_rss_peak_mib':r['rss_peak_bytes']/2**20,'tree_pss_snapshot_mib':sum(x['pss'] for x in ps)/2**20,'worker_rss_snapshot_mib':ps[0]['rss']/2**20,'l1_output_saved':(c/'l1_output.json').exists(),'l2_reference_saved':(c/'l2_reference_xy.npy').exists(),'l2_reported_success':L2,'length_m':r.get('length_m'),'case_path':str(c)}
   rows.append(flat)
  if (d/'summary.json').exists():
   assert [r['query_id'] for r in rs]==m['query_ids']
   checks.append({'round':rnd,'map_id':m['map_id'],'queries_match':True,'strict_success_audited':sum(r['success'] for r in rs),'reuse_hits':0,'old_l3_calls':0,'config_sha256':protocol['curve_config_sha256']})
if rows:
 with (REPORT/'raw_metrics.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
summary=[]
for m in maps:
 group=[r for r in rows if r['map_id']==m['map_id']]
 if not group:continue
 ok=[r for r in group if r['success']]
 item={'map_id':m['map_id'],'name':m['name'],'n':len(group),'strict_success':len(ok),'l1_outputs_saved':sum(r['l1_output_saved'] for r in group),'l2_references_saved':sum(r['l2_reference_saved'] for r in group),'success_wall_median_s':float(np.median([r['wall_s'] for r in ok])) if ok else None,'failure_queries':dict(collections.Counter(r['query_id'] for r in group if not r['success'])),'failure_codes':dict(collections.Counter(r['failure_code'] for r in group if not r['success']))}
 for key in ('wall_s','tree_cpu_s','tree_rss_peak_mib','tree_pss_snapshot_mib'):
  arr=np.array([r[key] for r in group]);item[key]={'median':float(np.median(arr)),'q1':float(np.quantile(arr,.25)),'q3':float(np.quantile(arr,.75)),'p95':float(np.quantile(arr,.95)),'min':float(arr.min()),'max':float(arr.max())}
 preps=[]
 for rnd in range(1,4):
  d=E/'results/formal'/f'round{rnd}'/m['map_id']
  if (d/'summary.json').exists():
   preps.append({'round':rnd,'preparation_s':json.loads((d/'preparation.json').read_text())['wall_ms']/1000,'auditor_preparation_s':json.loads((d/'auditor_preparation.json').read_text())['wall_ms']/1000,'session_startup_s':json.loads((d/'session_startup.json').read_text())['wall_ms']/1000})
 item['separate_startup']=preps;summary.append(item)
result={'total_n':len(rows),'strict_success':sum(r['success'] for r in rows),'maps':summary,'batch_checks':checks,'complete':len(rows)==300 and len(checks)==30}
(REPORT/'SUMMARY.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps({k:v for k,v in result.items() if k not in ('maps','batch_checks')},ensure_ascii=False))
for s in summary:print(s['name'],f"{s['strict_success']}/{s['n']}",s['failure_queries'])
