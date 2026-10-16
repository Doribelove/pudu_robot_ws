"""Paired timing and path outcomes, including every failed query."""
import json,hashlib,math,os
from collections import Counter
from pathlib import Path
from types import SimpleNamespace
import numpy as np,yaml
from benchmark import arc_metrics
from arena_3d_v1.semantic_world import WORK,SemanticWorld,json_write
BASE=Path('/home/robot/workspaces/3yd_multires_static_20260914')
def read(root):return [json.loads(s) for s in (root/'runs.jsonl').read_text().splitlines()]
def quant(values):
 a=np.array([v for v in values if v is not None and math.isfinite(v)],float)
 return {'n':len(a),**{k:float(np.quantile(a,q)) if len(a) else None for k,q in [('min',0),('p05',.05),('p50',.5),('p95',.95),('max',1)]}}
def case(row):
 p=Path(row['case']);return p if p.is_absolute() else WORK/p
def graph(root):
 files=list((root/'preparation_cache/graph').rglob('graph.json'));d=json.loads(files[0].read_text());ins=Counter(n['to_region'] for n in d['nodes']);outs=Counter(n['from_region'] for n in d['nodes'])
 return {'nodes':len(d['nodes']),'physical_portals':d['portal_count'],'candidate_directed_edges':sum(v*outs[k] for k,v in ins.items()),'graph_cache_bytes':sum(p.stat().st_size for p in files[0].parent.iterdir() if p.is_file()),'build_and_write_ms':json.loads((root/'preparation.json').read_text())['graph_preparation_ms']}
def main():
 os.sched_setaffinity(0,{14})
 cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());w=SemanticWorld(cfg['inputs']['map_yaml'],cfg['inputs']['semantic_path'],cache_root=WORK/'cache/world')
 roots={'3YD-V1':WORK/'results/formal_v1','recovery':WORK/'results/formal_recovery_final'};rs={k:read(p) for k,p in roots.items()};out={'arms':{},'paired_queries':[],'controls':{},'source_integrity':{},'startup_failures':[],'previous_iteration_controls':{}};quality=[]
 fields={}
 for arm,rows in rs.items():
  for r in rows:
   if not r['success']:continue
   if r['query_id'] not in fields:
    field=BASE/'results/formal_01_baseline/cases'/f"{r['query_id']}__1__right_0/evaluation_field.npz"
    with np.load(field) as d:fields[r['query_id']]=SimpleNamespace(normalized_right=d['q'],eligible=d['eligible'])
   r['quality']=arc_metrics(w,np.load(case(r)/'path.npy'),fields[r['query_id']]);quality.append({'arm':arm,'query_id':r['query_id'],'repetition':r['repetition'],**r['quality']})
  out['arms'][arm]={'n':len(rows),'success':sum(r['success'] for r in rows),'accepted_invalid':sum(r['success'] and not r.get('audit',{}).get('valid',False) for r in rows),'failures':[{'query':r['query_id'],'repetition':r['repetition'],'failure':r['failure_code']} for r in rows if not r['success']],
   'completion_modes':dict(Counter(r.get('multires',{}).get('completion_mode','full_fine_fallback' if r.get('fallbacks') else 'coarse_only') for r in rows if r['success'])),
   'timing_s':{kind:quant([r['wall_ms']/1000 for r in rows if kind=='all' or (r['repetition']==1 if kind=='first' else r['repetition']>1)]) for kind in ['all','first','repeat']},
   'peak_rss_gib':max(r['rss_peak_bytes'] for r in rows)/2**30,'rss_gib':quant([r['rss_peak_bytes']/2**30 for r in rows]),
   'l2_ms':quant([r.get('l2_ms') for r in rows]),'local_fine_cells':quant([r.get('multires',{}).get('local_fine',{}).get('local_fine_cells') for r in rows]),
   'right_band_ratio':quant([r.get('quality',{}).get('right_band_ratio') for r in rows]),'graph':graph(roots[arm]),
   'map_repair_count':sum(bool(r.get('l3',{}).get('local_map_update_fallback')) for r in rows),
   'ack_statuses':dict(Counter(r.get('l3',{}).get('costmap_ack_status') for r in rows)),
   'reference_adapter_ms':quant([r.get('reference',{}).get('adapter_ms') for r in rows])}
 for query in dict.fromkeys(r['query_id'] for r in rs['3YD-V1']):
  item={'query':query}
  for arm,rows in rs.items():
   rr=[r for r in rows if r['query_id']==query]
   item[arm]={'success':sum(r['success'] for r in rr),'n':len(rr),'length_m':quant([r.get('length_m') for r in rr])['p50'],'band_ratio':quant([r.get('quality',{}).get('right_band_ratio') for r in rr])['p50'],'eligible_arc_m':quant([r.get('quality',{}).get('trimmed_eligible_arc_length_m') for r in rr])['p50'],'first_s':rr[0]['wall_ms']/1000,'repeat_s':quant([r['wall_ms']/1000 for r in rr[1:]])['p50'],'mode':rr[0].get('multires',{}).get('completion_mode','full_fine_fallback' if rr[0].get('fallbacks') else 'coarse_only'),'local_fine_cells':rr[0].get('multires',{}).get('local_fine',{}).get('local_fine_cells')}
  a,b=item['3YD-V1'],item['recovery'];item['length_ratio']=b['length_m']/a['length_m'] if a['length_m'] and b['length_m'] else None;item['band_delta_pp']=100*(b['band_ratio']-a['band_ratio']) if a['band_ratio'] is not None and b['band_ratio'] is not None else None
  out['paired_queries'].append(item)
 out['paired_length_ratio']=quant([i['length_ratio'] for i in out['paired_queries']]);out['paired_band_delta_pp']=quant([i['band_delta_pp'] for i in out['paired_queries']])
 for folder in sorted((WORK/'results').glob('control_*')):
  if not (folder/'runs.jsonl').exists():
   if (folder/'protocol.json').exists():
    protocol=json.loads((folder/'protocol.json').read_text());out['startup_failures'].append({'directory':str(folder),'scene':protocol['scene'],'arm':protocol['arm'],'planned_queries':protocol['repeats'],'executed_queries':0,'reason':'startup costmap content ACK failed; early attempts also exhausted SHM, and one failure persisted after cleanup; subsequent fresh ROS domains succeeded; logs retained'})
   continue
  rows=read(folder);scene=json.loads((folder/'protocol.json').read_text())['scene'];arm=json.loads((folder/'protocol.json').read_text())['arm'];item={'scene':scene,'arm':arm,'n':len(rows),'success':sum(r['success'] for r in rows),'accepted_invalid':sum(r['success'] and not r.get('audit',{}).get('valid',False) for r in rows),'failure_codes':dict(Counter(r['failure_code'] for r in rows if not r['success'])),'recoveries':sum(r['success'] and len(r.get('recovery',{}).get('attempts',[]))>1 for r in rows),'attempts':[r.get('recovery',{}).get('attempts',[]) for r in rows]}
  if arm=='recovery' and not folder.name.startswith('control_final_'):
   out['previous_iteration_controls'][folder.name]=item;continue
  if scene in ('calibration','rotated'):
   bands=[]
   for r in rows:
    if not r['success']:continue
    pts=np.load(case(r)/'path.npy');ds=np.linalg.norm(np.diff(pts[:,:2],axis=0),axis=1);s=np.r_[0,np.cumsum(ds)];bounds=np.r_[np.arange(0,s[-1],.025),s[-1]];mid=(bounds[:-1]+bounds[1:])/2;weights=np.diff(bounds);xy=np.column_stack([np.interp(mid,s,pts[:,i]) for i in (0,1)])
    right=(xy[:,1]-.5)/5 if scene=='calibration' else (7.5-xy[:,0])/7;sel=(mid>=5)&(mid<=s[-1]-5);lo,hi=(.125,.375) if r['side']=='right' else (.625,.875);bands.append(float(weights[sel&(right>=lo)&(right<=hi)].sum()/weights[sel].sum()))
   item['minimum_target_band_ratio']=min(bands) if bands else None
   item['right_left_right_pose_restored']=all(rows[i].get('pose_hash')==rows[i+2].get('pose_hash') for i in range(0,len(rows),3))
  out['controls'][folder.name]=item
 freeze=json.loads((WORK/'report/IMPLEMENTATION_FREEZE.json').read_text());base=json.loads((WORK/'V1_BASELINE_MANIFEST.json').read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
 out['source_integrity']['implementation_files_checked']=len(freeze);out['source_integrity']['implementation_changed']=[k for k,v in freeze.items() if sha(WORK/k)!=v];out['source_integrity']['base_files_checked']=len(base['files']);out['source_integrity']['base_changed']=[k for k,v in base['files'].items() if sha(BASE/k)!=v]
 params=[roots[a]/'smac/logs/smac_strict_real.yaml' for a in roots];out['source_integrity']['same_smac_parameters']=len({sha(p) for p in params})==1
 runtime='external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/'
 out['source_integrity']['original_world_reference_l3_unchanged']=all(sha(WORK/runtime/name)==sha(BASE/runtime/name) for name in ['semantic_world.py','semantic_reference.py','semantic_pipeline.py','pipeline.py'])
 out['source_integrity']['sweep_binary_unchanged']=sha(WORK/'build/libsemantic_sweep.so')==sha(BASE/'build/libsemantic_sweep.so')
 json_write(WORK/'report/performance.json',out);json_write(WORK/'report/path_quality.json',quality)
 print(json.dumps(out['arms'],indent=2))
if __name__=='__main__':main()
