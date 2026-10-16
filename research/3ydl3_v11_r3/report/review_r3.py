"""Inspect measured implementation changes, all attempts and actual path effects."""
from pathlib import Path
import hashlib,json
import numpy as np
R=Path(__file__).resolve().parents[1];P=Path('/home/robot/workspaces/3ydl3_v11_r2_tailperf_20261016');O=R/'report'
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert read(R/'results/paired_final/COMPLETE.json')['complete']
changes=[]
for folder in ['src','tools','native','config','install']:
 for p in (R/folder).rglob('*'):
  if not p.is_file() or '__pycache__' in str(p):continue
  old=P/p.relative_to(R)
  if not old.exists() or sha(p)!=sha(old):changes.append({'path':str(p.relative_to(R)),'before_sha256':sha(old) if old.exists() else None,'after_sha256':sha(p)})
assert not any(c['path'].split('/')[0] in ['native','config','install'] for c in changes)
heap=[];schedule=[];paths=[];budgets=[];aux=[];cpu={'control':[],'candidate':[]};firstq={arm:[] for arm in cpu}
for rnd in range(1,4):
 for m in read(R/'MAPS.json')['maps']:
  base=R/'results/paired_final'/f'round{rnd}'/m['map_id'];pair={}
  for arm in cpu:
   runs=[json.loads(x) for x in (base/arm/'runs.jsonl').read_text().splitlines()];pair[arm]={x['query_id']:x for x in runs}
   firstq[arm].append({'round':rnd,'map_id':m['map_id'],'query_id':runs[0]['query_id'],'navigation_s':runs[0]['wall_ms']/1000,'tree_rss_mib':runs[0]['rss_peak_bytes']/2**20})
   for r in runs:
    cpu[arm].append(r['tree_cpu_ms']/1000)
    used=r['l3_backend']['new_used_s'];assert used<=15.+1e-6,(m['map_id'],r['query_id'],used)
    assert r['wall_ms']<=30000.,r['wall_ms']
    budgets.append({'arm':arm,'round':rnd,'map_id':m['map_id'],'query_id':r['query_id'],'l3_used_s':used,'navigation_s':r['wall_ms']/1000})
    if arm=='candidate':
     out=read(base/arm/'cases'/f"{r['query_id']}_1"/'outcome.json')
     h=out['r11_request_heap'];assert h['included_in_navigation'];heap.append(h)
     ref=out.get('reference_diagnostics',{})
     if ref.get('r11_compact_auxiliary'):
      aux.append({'round':rnd,'map_id':m['map_id'],'query_id':r['query_id'],**{k:v for k,v in ref.items() if k.startswith('r11_auxiliary') or k in ['r11_static_storage','r11_static_LUT_all_cells_verified','r11_hard_mask_exact_packed']}})
      assert ref['r11_hard_mask_exact_packed']
      assert ref['r11_auxiliary_stored_bytes']<ref['r11_auxiliary_dense_bytes']
      if ref['r11_static_storage']=='derived_original_LUT':assert ref['r11_static_LUT_all_cells_verified']
     if out.get('r11_request_schedule'):schedule.append({'round':rnd,'map_id':m['map_id'],'query_id':r['query_id'],**out['r11_request_schedule']})
  for q,a in pair['control'].items():
   b=pair['candidate'][q];change=100*(b['length_m']/a['length_m']-1)
   paths.append({'round':rnd,'map_id':m['map_id'],'query_id':q,'length_change_percent':change,'control_length_m':a['length_m'],'candidate_length_m':b['length_m'],'curvature_bound':b['curve_l3']['certificate']['max_curvature_bound_1pm']})
def stats(a):return {'n':len(a),'median':float(np.median(a)),'p95':float(np.quantile(a,.95)),'max':float(np.max(a))}
out={'version':'3YDL3-V11-r3','changed_files':changes,'auxiliary_storage_records':aux,'immutable_native_config_install':True,'tests':1442,'heap_release':{'calls':len(heap),'triggered':sum(h['triggered'] for h in heap),'returned_release_true':sum(h['released'] for h in heap),'all_in_navigation_clock':True,'threshold_bytes':64*2**20,'top_reserve_bytes':32*2**20,'call_wall_ms':stats([h['wall_ms'] for h in heap]),'interpretation':'malloc_trim may madvise free heap interiors without reducing mallinfo arena/free virtual bytes; no reclaimed RSS amount is inferred from these counters.'},'schedule_records':schedule,'yielded_requests':sum(s['yielded'] for s in schedule),'original_schedule_retries':sum(s['original_schedule_retried'] for s in schedule),'path_length_change_percent':{'minimum':min(p['length_change_percent'] for p in paths),'maximum':max(p['length_change_percent'] for p in paths)},'path_changes':paths,'budget_maxima':{'l3_used_s':max(b['l3_used_s'] for b in budgets),'navigation_s':max(b['navigation_s'] for b in budgets)},'tree_cpu_s':{arm:stats(v) for arm,v in cpu.items()},'first_Q_supplement':{'definition':'Explicit supplemental subset only: first actual Q after service ready; not the all-Q primary metrics. No map/startup time.', 'records':firstq,'stats':{arm:{k:stats([r[k] for r in rs]) for k in ['navigation_s','tree_rss_mib']} for arm,rs in firstq.items()}}}
(O/'IMPLEMENTATION_REVIEW.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in out.items() if k not in ['changed_files','schedule_records','path_changes','first_Q_supplement','auxiliary_storage_records']},indent=2))
