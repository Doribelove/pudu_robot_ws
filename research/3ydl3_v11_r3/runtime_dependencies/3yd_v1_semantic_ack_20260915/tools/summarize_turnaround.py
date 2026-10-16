"""Final paired results with startup, injection and query denominators separate."""
import json,hashlib,math,os
from pathlib import Path
from types import SimpleNamespace
from collections import Counter
import numpy as np,yaml
from benchmark import arc_metrics
from arena_3d_v1.semantic_world import WORK,SemanticWorld,json_write

def read(root,name='runs.jsonl'):
 p=Path(root)/name
 return [json.loads(s) for s in p.read_text().splitlines()] if p.exists() else []
def quant(values):
 a=np.array([v for v in values if v is not None and math.isfinite(v)],float)
 return {'n':len(a),**{k:float(np.quantile(a,q)) if len(a) else None for k,q in [('min',0),('p50',.5),('p95',.95),('max',1)]}}
def summarize(rows):
 return {'n':len(rows),'success':sum(r['success'] for r in rows),'accepted_invalid':sum(r['success'] and not r.get('audit',{}).get('valid',False) for r in rows),
  'failures':[{'query':r['query_id'],'repetition':r['repetition'],'failure':r['failure_code'],'repair':r.get('local_curvature_repair',{})} for r in rows if not r['success']],
  'timing_s':{kind:quant([r['wall_ms']/1000 for r in rows if kind=='all' or (r['repetition']==1 if kind=='first' else r['repetition']>1)]) for kind in ['all','first','repeat']},
  'peak_rss_gib':max([r.get('rss_peak_bytes',0) for r in rows],default=0)/2**30,
  'completion_modes':dict(Counter(r.get('multires',{}).get('completion_mode','full_fine_fallback' if r.get('fallbacks') else 'coarse_only') for r in rows if r['success'])),
  'local_repair_attempts':sum(bool(r.get('local_curvature_repair',{}).get('attempted')) for r in rows),
  'local_repair_successes':sum(bool(r.get('local_curvature_repair',{}).get('success')) for r in rows),
  'local_repair_ms':quant([r.get('local_curvature_repair',{}).get('wall_ms') for r in rows]),
  'length_m':quant([r.get('length_m') for r in rows if r['success']]),
  'curvature_1pm':quant([r.get('audit',{}).get('max_curvature_1pm') for r in rows if r['success']]),
  'map_repair_count':sum(bool(r.get('l3',{}).get('local_map_update_fallback')) for r in rows),
  'ack_statuses':dict(Counter(r.get('l3',{}).get('costmap_ack_status') for r in rows)),
  'candidate_recoveries':sum(r['success'] and len(r.get('recovery',{}).get('attempts',[]))>1 for r in rows)}

def main():
 os.sched_setaffinity(0,{14});out={'main':{},'controls':{},'holdouts':{},'startup':{},'integrity':{}}
 roots={'3YD-V1-r1':WORK/'results/formal_r1','candidate':WORK/'results/formal_candidate_v3'}
 out['primary_roots']={k:str(v) for k,v in roots.items()}
 data={k:read(v) for k,v in roots.items()}
 for name,rows in data.items():out['main'][name]=summarize(rows)
 cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());w=SemanticWorld(cfg['inputs']['map_yaml'],cfg['inputs']['semantic_path'],cache_root=WORK/'cache/report_world')
 quality=[];paired=[]
 base=Path('/home/robot/workspaces/3yd_multires_static_20260914')
 for query in dict.fromkeys(r['query_id'] for r in data['3YD-V1-r1']):
  field=base/'results/formal_01_baseline/cases'/f'{query}__1__right_0/evaluation_field.npz'
  with np.load(field) as d:pref=SimpleNamespace(normalized_right=d['q'],eligible=d['eligible'])
  item={'query':query}
  for arm,rows in data.items():
   rr=[r for r in rows if r['query_id']==query];scores=[]
   for r in rr:
    if not r['success']:continue
    points=np.load(Path(r['case'])/'path.npy');score=arc_metrics(w,points,pref);scores.append(score)
    quality.append({'arm':arm,'query':query,'repetition':r['repetition'],**score})
   item[arm]={'success':sum(r['success'] for r in rr),'n':len(rr),'length_m':quant([r.get('length_m') for r in rr])['p50'],
     'right_band_ratio':quant([s['right_band_ratio'] for s in scores])['p50']}
  old=next((r for r in data['3YD-V1-r1'] if r['query_id']==query and r['repetition']==1),None)
  new=next((r for r in data['candidate'] if r['query_id']==query and r['repetition']==1),None)
  item['exact_same_poses']=bool(old and new and old['success'] and new['success'] and np.array_equal(np.load(Path(old['case'])/'path.npy'),np.load(Path(new['case'])/'path.npy')))
  paired.append(item)
 out['paired_queries']=paired;out['exact_same_query_paths']=sum(x['exact_same_poses'] for x in paired)
 selected_controls={'final_calibration_v3','final_v3_return_bulb',*['final_v2_'+s for s in ['return_bulb_rotated','rotated','narrow','two_routes','two_routes_optimistic_edge']]}
 out['previous_iteration_controls']={}
 for p in sorted((WORK/'results').glob('final_*')):
  if p.name not in selected_controls:
   out['previous_iteration_controls'][p.name]=summarize(read(p));continue
  rows=read(p);out['controls'][p.name]=summarize(rows)
  if p.name in ['final_calibration_v3','final_v2_rotated']:
   out['controls'][p.name]['right_left_right_pose_restored']=all(rows[i].get('pose_hash')==rows[i+2].get('pose_hash') for i in range(0,len(rows),3)) if len(rows)%3==0 else False
 out['previous_iteration_holdouts']={};out['sealed_holdouts']={}
 for p in sorted((WORK/'results').glob('holdout_*')):
  group=out['holdouts'] if p.name.startswith('holdout_v2_') or p.name.endswith('_r1') else out['previous_iteration_holdouts']
  group[p.name]=summarize(read(p))
 for p in sorted((WORK/'results').glob('sealed_*')):out['sealed_holdouts'][p.name]=summarize(read(p))
 for name in ['startup_v3_small','startup_v3_real']:
  p=WORK/'results'/name;rows=read(p,'cycles.jsonl')
  out['startup'][name]={'cycles':len(rows),'startup_success':sum(r['startup_success'] for r in rows),
   'complete_success':sum(r['success'] for r in rows),'updates_success':sum(len(r['updates']) for r in rows),
   'startup_ms':quant([r.get('startup_ms') for r in rows]),'abrupt_exits_injected':sum(r.get('injected_planner_kill',False) for r in rows),
   'shm_after_bytes':quant([r['shm_after'] for r in rows]),'failures':[r for r in rows if not r['success']]}
  pending=[];errors=[];snapshots=0
  for f in p.glob('cycle_*/lifecycle_evidence/readbacks.json'):
   records=json.loads(f.read_text());snapshots+=len(records);errors.extend(r for r in records if not r.get('success'))
  for f in p.glob('cycle_*/lifecycle_evidence/shutdown.json'):
   d=json.loads(f.read_text());pending.extend(d.get('deferred_remaining',[]))
  out['startup'][name].update(readback_requests=snapshots,readback_errors=errors,deferred_files_still_present=sorted({n for n in pending if (Path('/dev/shm')/n).exists()}))
 fault=WORK/'results/ack_fault_injection_v3/results.json';out['ack_fault_injection']=json.loads(fault.read_text()) if fault.exists() else []
 out['dispatch']={phase:read(WORK/'report',phase+'_dispatch.jsonl') for phase in ['startup','planning','final_v2','sealed','final_v3']}
 out['previous_startup_batch']={name:json.loads((WORK/'results'/name/'summary.json').read_text()) for name in ['startup_final_small','startup_final_real','startup_v2_small','startup_v2_real']}
 out['previous_main_batches']={name:summarize(read(WORK/'results'/name)) for name in ['formal_candidate','formal_candidate_v2']}
 out['geometry_validation_note']='Controls with final_v2 names, expanded and sealed cases use the final curvature implementation. Only nonsemantic mask ACK changed afterwards; final_v3 controls/main and startup verify the delivered session.'
 out['preserved_natural_startup_failure']={'directory':'results/final_calibration','entered_query_loop':False,
  'reason':'cold initialization retained 96000 mismatching cells; exceeded inherited repair threshold; no resend before timeout',
  'acks':json.loads((WORK/'results/final_calibration/smac/lifecycle_evidence/acks.json').read_text())}
 sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
 freeze=json.loads((WORK/'report/IMPLEMENTATION_FREEZE.json').read_text());baseline=json.loads((WORK/'R1_BASELINE_MANIFEST.json').read_text());r1=Path(baseline['root'])
 out['integrity']={'frozen_files':len(freeze),'changed_frozen_files':[k for k,v in freeze.items() if sha(WORK/k)!=v],
  'baseline_runtime_files':len(baseline['files']),'baseline_runtime_changed':[k for k,v in baseline['files'].items() if sha(r1/k)!=v],
  'identical_smac_parameters':sha(roots['3YD-V1-r1']/'smac/logs/smac_strict_real.yaml')==sha(roots['candidate']/'smac/logs/smac_strict_real.yaml'),
  'identical_vehicle_sweep_binary':sha(WORK/'build/libsemantic_sweep.so')==sha(r1/'build/libsemantic_sweep.so')}
 json_write(WORK/'report/performance.json',out);json_write(WORK/'report/path_quality.json',quality)
 print(json.dumps({'main':out['main'],'startup':out['startup'],'same_paths':out['exact_same_query_paths'],'integrity':out['integrity']},indent=2))

if __name__=='__main__':main()
