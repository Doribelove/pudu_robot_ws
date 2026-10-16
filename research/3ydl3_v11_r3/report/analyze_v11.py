"""Reconcile V11 policy success separately from exact-endpoint success."""
from pathlib import Path
import hashlib,json,csv,math
import numpy as np
R=Path(__file__).resolve().parents[1];D=R/'results/paired_final';O=R/'report'
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
rows=lambda p:[json.loads(x) for x in p.read_text().splitlines()]
def save(name,data):(O/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
def stats(a):
 a=np.asarray(a,float)
 return dict(n=len(a),median=float(np.median(a)),p95=float(np.quantile(a,.95)),maximum=float(a.max()),q1=float(np.quantile(a,.25)),q3=float(np.quantile(a,.75)))
def compare(a,b):
 x,y=stats(a),stats(b)
 return dict(control=x,candidate=y,change_percent={k:100*(y[k]/x[k]-1) for k in ('median','p95','maximum')})
def main():
 assert read(D/'COMPLETE.json')['complete']
 maps=read(R/'MAPS.json')['maps'];records={};flat=[];certs=0;cleanup=[];new=[];sources=0
 equal=['map_sha256','queries_sha256','vehicle','CPU','actual_query_order','common_harness_sha256','L1_frozen','moving_window','global_L3_fallback','repeats','workload','timing','pose_offset_m','yaw_offset_deg']
 for rnd in range(1,4):
  for m in maps:
   mid=m['map_id'];protocols={}
   for arm in ('control','candidate'):
    p=D/f'round{rnd}'/mid/arm;j=read(p.with_suffix('.supervisor.json'))
    assert j['returncode']==0 and j['cleanup']['complete'];cleanup.append(str(p))
    protocol=read(p/'protocol.json');protocols[arm]=protocol
    memory_policy=protocol['runtime_memory_policy']
    assert memory_policy['request_finish_heap_release']==(arm=='candidate')
    if arm=='candidate':
     assert memory_policy['free_threshold_bytes']==64*2**20 and memory_policy['top_reserve_bytes']==32*2**20
     assert memory_policy['inside_navigation_and_request_deadline']
    data=rows(p/'runs.jsonl');assert len(data)==10 and {r['query_id'] for r in data}==set(m['query_ids'])
    for name,h in read(p/'source_snapshot/hashes.json').items():assert sha(name)==h,name
    sources+=1
    for r in data:
     assert not r.get('task_progress',{}).get('reused',False), 'NOT_A_FIRST_REQUEST'
     assert not r.get('fullroute',{}).get('reused',False), 'FINAL_ROUTE_CACHE_HIT'
     q=r['query_id'];case=p/'cases'/f'{q}_1';out=read(case/'outcome.json');key=(rnd,mid,q)
     records.setdefault(key,{})[arm]=(r,case)
     tol=r.get('endpoint_tolerance',{});used=bool(tol.get('used'))
     if r['success']:
      certs+=1
      assert r['audit']['valid'] and not r['audit']['failures']
      assert r['constraints']['traffic']['valid'] and r['constraints']['steering_jump_valid']
      assert r['curve_l3']['certificate']['max_curvature_bound_1pm']<=2.5+1e-8
      assert out['diagnostics']['costmap_update_acknowledged'] is True
      if arm=='candidate':
       assert out['r11_request_heap']['included_in_navigation']
       assert out['r11_request_heap']['wall_ms']<=r['wall_ms']
      path=np.load(case/'path.npy')
      if arm in ('control','candidate'):
       assert tol['requested']=={'start':r['start'],'goal':r['goal']}
       for role,actual in [('start',path[0]),('goal',path[-1])]:
        resolved=tol['resolved'][role];requested=r[role]
        delta=np.linalg.norm(actual[:2]-np.asarray(requested)[:2]);yaw=abs(math.atan2(math.sin(actual[2]-requested[2]),math.cos(actual[2]-requested[2])))
        assert delta<=.2+1e-9 and yaw<=math.radians(30)+1e-8
        assert np.linalg.norm(actual[:2]-np.asarray(resolved)[:2])<=1e-8
        assert abs(math.atan2(math.sin(actual[2]-resolved[2]),math.cos(actual[2]-resolved[2])))<=1e-8
        if not used:assert delta<=1e-8 and yaw<=1e-8
       if used:
        assert not out['legacy_contract_success'] and not r['exact_endpoint_success']
        assert not tol['attempts'][0]['success']
        assert tol['attempts'][0]['start']==r['start'] and tol['attempts'][0]['goal']==r['goal']
        new.append(dict(arm=arm,round=rnd,map_id=mid,query=q,navigation_s=r['wall_ms']/1000,offsets=tol['offsets'],curvature_bound=r['curve_l3']['certificate']['max_curvature_bound_1pm']))
     l1=np.asarray(read(case/'l1_output.json')['right_reference_polyline'],float)
     l1_length=float(np.linalg.norm(np.diff(l1[:,:2],axis=0),axis=1).sum());assert l1_length>0
     flat.append(dict(round=rnd,map_id=mid,query_id=q,arm=arm,success=r['success'],exact_success=r['success'] and not used,tolerant_success=r['success'] and used,navigation_s=r['wall_ms']/1000,tree_rss_mib=r['rss_peak_bytes']/2**20,l1_length_m=l1_length,navigation_s_per_l1_m=r['wall_ms']/1000/l1_length,tree_rss_mib_per_l1_m=r['rss_peak_bytes']/2**20/l1_length,failure_code=r['failure_code']))
   assert all(protocols['control'].get(k)==protocols['candidate'].get(k) for k in equal)
   assert protocols['control']['runtime_memory_policy']['allocator_environment']==protocols['candidate']['runtime_memory_policy']['allocator_environment']
   a=dict(protocols['control']['curve_config']);b=dict(protocols['candidate']['curve_config']);assert a==b
 baseline={(mid,q) for (_,mid,q),pair in records.items() if pair['control'][0].get('exact_endpoint_success')};assert len(baseline)==99
 regressions=[];unchanged=[]
 for key,pair in records.items():
  _,mid,q=key;a,ap=pair['control'];b,bp=pair['candidate']
  assert a['start']==b['start'] and a['goal']==b['goal']
  if (mid,q) in baseline:
   if not (a['success'] and b['success'] and b.get('exact_endpoint_success')):regressions.append(key)
   if a['success'] and b['success']:
    same={f:sha(ap/f)==sha(bp/f) for f in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','path.npy')}
    same['effective_costmap']=a['l3']['applied_mask_hash']==b['l3']['applied_mask_hash']
    unchanged.append(dict(round=key[0],map_id=mid,query=q,same=same))
 cohorts={}
 for name,predicate in [('original_99',lambda r:(r['map_id'],r['query_id']) in baseline),('all_requests',lambda r:True)]:
  data=[r for r in flat if predicate(r)]
  cohorts[name]={k:compare([r[k] for r in data if r['arm']=='control'],[r[k] for r in data if r['arm']=='candidate']) for k in ('navigation_s','tree_rss_mib')}
 bymap={m['map_id']:{arm:{'success':sum(r['success'] for r in flat if r['map_id']==m['map_id'] and r['arm']==arm),'exact_success':sum(r['exact_success'] for r in flat if r['map_id']==m['map_id'] and r['arm']==arm),'tolerant_success':sum(r['tolerant_success'] for r in flat if r['map_id']==m['map_id'] and r['arm']==arm)} for arm in ('control','candidate')} for m in maps}
 for p,h in read(D/'source.json').items():assert sha(p)==h,p
 for p,h in read(R/'PARENT_PROTECTION.json').items():assert sha(p)==h,p
 frozen=read(R/'L1_LOCK.json')
 for p,h in frozen['source_files'].items():assert sha(R/p)==h,p
 gates={'control_300_policy_success':sum(r['success'] for r in flat if r['arm']=='control')==300,'all_300_policy_success':sum(r['success'] for r in flat if r['arm']=='candidate')==300,'original_99_exact_preserved':not regressions}
 for label,k,stat in [('navigation_median','navigation_s','median'),('navigation_p95','navigation_s','p95'),('rss_median','tree_rss_mib','median')]:gates[label+'_reduction_at_least_5_percent']=cohorts['original_99'][k]['change_percent'][stat]<=-5.
 report=dict(version='3YDL3-V11-r3',requests=600,certificates_checked=certs,batches_cleaned=len(cleanup),batch_source_locks_verified=sources,baseline_queries=99,cohorts=cohorts,maps=bymap,new_tolerant_successes=new,regressions=regressions,gates=gates,all_gates_passed=all(gates.values()),original_output_checks=unchanged,protected_parent_files=len(read(R/'PARENT_PROTECTION.json')),frozen_L1_files=len(frozen['source_files']))
 save('PAIRED_RESULTS.json',report)
 with (O/'paired_rows.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(flat[0])+['original_99']);w.writeheader()
  for r in flat:w.writerow({**r,'original_99':(r['map_id'],r['query_id']) in baseline})
 save('SELECTED_DATA.json',{'formal':'results/paired_final','rounds':3,'each_Q_first_request':True,'success_definition':'exact endpoints or explicitly reported 0.20m/30deg endpoint fallback; unchanged 0.40m minimum radius and full audits','excluded':['all checks and diagnostic/pilot results','parent historical measurements']})
 print(json.dumps({k:report[k] for k in ('requests','certificates_checked','batches_cleaned','gates','cohorts')},indent=2))
if __name__=='__main__':main()
