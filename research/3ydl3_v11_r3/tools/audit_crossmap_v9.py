"""Verify fixed poses, strict successful delivery, unchanged audits and source locks."""
from pathlib import Path
import hashlib,json,collections,ast,csv
import numpy as np
E=Path(__file__).resolve().parents[1];OLD=Path('/home/robot/workspaces/3ydl3_v8_crossmap10_20261011');PARENT=Path('/home/robot/workspaces/3ydl3_v8_20261011');sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
report=E/'report';report.mkdir(exist_ok=True);maps=json.loads((E/'MAPS.json').read_text())['maps'];assert sha(E/'MAPS.json')==sha(OLD/'MAPS.json')
parent=json.loads((E/'PARENT_SOURCE_HASHES.json').read_text());assert all(Path(p).exists() and sha(p)==h for p,h in parent.items())
lock=json.loads((E/'L1_LOCK.json').read_text());locked={}
for kind in ('source_files','input_files','pixel_input_files'):
 for p,h in lock[kind].items():assert sha(Path(p) if Path(p).is_absolute() else E/p)==h,(kind,p)
 locked[kind]=len(lock[kind])
assert sha(E/'src/three_yd_v5/controller.py')==sha(PARENT/'src/three_yd_v5/controller.py')
def func(path,name):return ast.dump(next(n for n in ast.parse(path.read_text()).body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name==name),include_attributes=False)
for name in ('certify','optimize','rectangles','feasible_guide'):
 assert func(E/'src/three_yd_v5/curve_l3.py',name)==func(PARENT/'src/three_yd_v5/curve_l3.py',name),name
for name in ('validate_record','displaced_continuation'):
 assert func(E/'src/three_yd_v5/curve_backend.py',name)==func(PARENT/'src/three_yd_v5/curve_backend.py',name),name
config=json.loads((E/'config/l3.json').read_text());oldconfig=json.loads((PARENT/'config/l3.json').read_text());assert {k:v for k,v in config.items() if k!='schema'}=={k:v for k,v in oldconfig.items() if k!='schema'}
rows=[];regressions=[];fixes=[];checks=[];path_changes=[];digest=collections.defaultdict(set);old_good=0;same_original=0
for rnd in range(1,4):
 for m in maps:
  for p,h in m['input_sha256'].items():assert sha(p)==h
  d=E/'results/formal'/f'round{rnd}'/m['map_id'];od=OLD/'results/formal'/f'round{rnd}'/m['map_id']
  rr=[json.loads(v) for v in (d/'runs.jsonl').read_text().splitlines()];oo={v['query_id']:v for v in [json.loads(v) for v in (od/'runs.jsonl').read_text().splitlines()]}
  assert len(rr)==10 and [v['query_id'] for v in rr]==m['query_ids']
  assert json.loads((d/'map_input_verified.json').read_text())==m
  protocol=json.loads((d/'protocol.json').read_text());assert protocol['CPU']==[14] and protocol['l3_backend']=='independent' and protocol['curve_config_sha256']==sha(E/'config/l3.json')
  sup=json.loads(d.with_name(d.name+'.supervisor.json').read_text());assert sup['cleanup']['complete'] and sup['returncode']==0
  for r in rr:
   q=next(v for v in m['poses'] if v['id']==r['query_id']);pose=lambda a:[a[k] for k in ('x','y','yaw_rad')] if isinstance(a,dict) else a
   assert list(r['start'])==pose(q['start']) and list(r['goal'])==pose(q['goal'])
   o=oo[r['query_id']];case=d/'cases'/f"{r['query_id']}_1";oc=od/'cases'/case.name
   identity={'round':rnd,'map_id':m['map_id'],'query_id':r['query_id']}
   if o['success'] and not r['success']:regressions.append(identity)
   if not o['success'] and r['success']:fixes.append(identity)
   if r['success']:
    assert r['audit']['valid'] and r['constraints']['steering_jump_valid'] and r['constraints']['traffic']['valid']
    assert r['l3']['costmap_update_acknowledged'] and r['l3']['costmap_ack_mismatch_cells']==0 and r['l3']['costmap_ack_status']=='exact_effective_master_content_verified'
    assert r['l3_backend']['strict_success'] and not r['task_progress']['reused']
    p=np.load(case/'path.npy')
    for actual,requested in ((p[0],pose(q['start'])),(p[-1],pose(q['goal']))):
     delta=actual-np.asarray(requested)
     assert np.linalg.norm(delta[:2])<1e-8 and abs(np.arctan2(np.sin(delta[2]),np.cos(delta[2])))<1e-8,(identity,delta.tolist())
    digest[(m['map_id'],r['query_id'])].add(sha(case/'path.npy'))
   else:assert not (case/'path.npy').exists()
   for attempt in r['l3_backend']['attempts']:assert attempt['old_l3_calls']==0
   if o['success']:
    old_good+=1;parts={f:(case/f).exists() and sha(case/f)==sha(oc/f) for f in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','path.npy')}
    if all(parts.values()):same_original+=1
    else:path_changes.append({**identity,'same':parts})
   rows.append({**identity,'map_name':m['name'],'old_success':o['success'],'success':r['success'],'old_wall_s':o['wall_ms']/1000,'wall_s':r['wall_ms']/1000,'old_tree_rss_mib':o['rss_peak_bytes']/2**20,'tree_rss_mib':r['rss_peak_bytes']/2**20,'old_tree_cpu_s':o['tree_cpu_ms']/1000,'tree_cpu_s':r['tree_cpu_ms']/1000,'failure_code':r['failure_code']})
  checks.append({'round':rnd,'map_id':m['map_id'],'clean':True,'strict':sum(v['success'] for v in rr)})
cleanup=[{'file':str(p),'complete':json.loads(p.read_text())['cleanup']['complete']} for p in (E/'results').rglob('*.supervisor.json')];assert all(v['complete'] for v in cleanup)
summary=[]
for m in maps:
 group=[v for v in rows if v['map_id']==m['map_id']];both=[v for v in group if v['old_success'] and v['success']]
 item={'map_id':m['map_id'],'name':m['name'],'n':len(group),'old_success':sum(v['old_success'] for v in group),'success':sum(v['success'] for v in group),'remaining':sorted({v['query_id'] for v in group if not v['success']}),'common_success_n':len(both)}
 for key in ('wall_s','tree_rss_mib','tree_cpu_s'):
  item[key]={scope:{'old_median':float(np.median([v['old_'+key] for v in values])),'median':float(np.median([v[key] for v in values]))} for scope,values in (('all_attempts',group),('common_successes',both))}
 summary.append(item)
result={'strict':sum(v['success'] for v in rows),'old_strict':sum(v['old_success'] for v in rows),'n':len(rows),'maps':summary,'regressions':regressions,'fixed_requests':fixes,'original_successes':old_good,'original_successes_all_four_outputs_identical':same_original,'original_output_changes':path_changes,'stable_successful_paths_all_rounds':all(len(v)==1 for v in digest.values()),'parent_files_unchanged':len(parent),'L1_LOCK':locked,'original_audit_and_certificate_unchanged':True,'fixed_poses_unchanged':True,'all_strict_ACK_verified':True,'formal_batches':checks,'all_cleanup':cleanup,'comparison_protocol':'Candidate 3 independent rounds vs archived V8 3 independent rounds; not a contemporaneous randomized performance experiment.'}
(report/'COMPARISON.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
with (report/'comparison_raw.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
print({k:v for k,v in result.items() if k not in ('maps','formal_batches','all_cleanup','fixed_requests','original_output_changes')})
