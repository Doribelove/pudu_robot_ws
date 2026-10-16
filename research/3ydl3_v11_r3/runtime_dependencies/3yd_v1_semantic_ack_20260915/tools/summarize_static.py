"""Common-denominator comparison; rejected paths and cold/fallback cases retained."""
import json,math,argparse
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import yaml
from benchmark import arc_metrics
from arena_3d_v1.semantic_world import SemanticWorld,WORK,json_write

def quant(v):
 v=[float(x) for x in v if x is not None and math.isfinite(x)]
 return {'n':len(v),'p50':float(np.median(v)) if v else None,'p95':float(np.quantile(v,.95)) if v else None,'max':max(v) if v else None}
def read(root):return [json.loads(s) for s in (root/'runs.jsonl').read_text().splitlines()]
def key(r):return r['query_id'],r['repetition']
def main():
 p=argparse.ArgumentParser();p.add_argument('--tag',default='formal_01');a=p.parse_args();roots={k:WORK/'results'/f'{a.tag}_{k}' for k in ['baseline','topology_005','topology_015']};rs={k:read(v) for k,v in roots.items()}
 cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());w=SemanticWorld(cfg['inputs']['map_yaml'],cfg['inputs']['semantic_path'],cache_root=WORK/'cache/world');summary={'arms':{},'paired':{},'preparation':{},'controls':[],'narrow':[]};fields={};quality=[]
 for b in rs['baseline']:
  if b['repetition']==1:
   file=Path(b['case'])/'evaluation_field.npz'
   if file.exists():fields[b['query_id']]=file
 for arm,rows in rs.items():
  for r in rows:
   pathfile=Path(r['case'])/'path.npy'
   if r['success'] and pathfile.exists() and r['query_id'] in fields:
    with np.load(fields[r['query_id']]) as d:pref=SimpleNamespace(normalized_right=d['q'],eligible=d['eligible'])
    poses=np.load(pathfile);r['quality']=arc_metrics(w,poses,pref)
    quality.append({'arm':arm,'query_id':r['query_id'],'repetition':r['repetition'],**r['quality']})
  cold=[r for r in rows if r['repetition']==1];warm=[r for r in rows if r['repetition']>1]
  simple=['wall_ms','cpu_ms','process_tree_cpu_ms','rss_peak_bytes','post_query_tree_pss_bytes','l1_ms','l2_ms','length_m']
  item={'count':len(rows),'success':sum(r['success'] for r in rows),'accepted_invalid':sum(r['success'] and not r.get('audit',{}).get('valid',False) for r in rows),'failures':[{'query_id':r['query_id'],'repetition':r['repetition'],'reason':r['failure_code']} for r in rows if not r['success']],
    'fallback_count':sum(bool(r.get('fallbacks')) for r in rows),'fallback_queries':sorted({r['query_id'] for r in rows if r.get('fallbacks')}),
    'all':{m:quant([r.get(m) for r in rows]) for m in simple},'cold':{m:quant([r.get(m) for r in cold]) for m in simple},'warm':{m:quant([r.get(m) for r in warm]) for m in simple},
    'l2_state_bytes':quant([r.get('l2',{}).get('state_memory_bytes') for r in rows]),
    'l2_roi_cells':quant([r.get('l2',{}).get('roi_array_cells') for r in rows]),
    'l3':{m:quant([r.get('l3',{}).get(m) for r in rows]) for m in ['l3_planning_time_ms','local_map_update_ms','costmap_ack_wait_ms','l3_action_wall_ms','planner_rss_peak_bytes','planner_pss_peak_bytes','stack_rss_peak_bytes','stack_pss_peak_bytes']},
    'reference_adapter_ms':quant([r.get('reference',{}).get('adapter_ms') for r in rows]),
    'right_band_ratio':quant([r.get('quality',{}).get('right_band_ratio') for r in rows]),'eligible_arc_m':quant([r.get('quality',{}).get('trimmed_eligible_arc_length_m') for r in rows]),
    'exact_content_ack_count':sum(r.get('l3',{}).get('costmap_ack_status')=='exact_effective_master_content_verified' for r in rows),
    'reused_exact_content_ack_count':sum(r.get('l3',{}).get('costmap_ack_status')=='reused_exact_effective_content_ack' for r in rows),
    'map_update_repair_count':sum(bool(r.get('l3',{}).get('local_map_update_fallback')) for r in rows),
    'actual_resolutions':{str(res):sum(abs(r.get('multires',{}).get('actual_resolution_m',.05)-res)<1e-6 for r in rows) for res in [.05,.15]},
    'preparation':json.loads((roots[arm]/'preparation.json').read_text())}
  summary['arms'][arm]=item
 base={key(r):r for r in rs['baseline']};fine={key(r):r for r in rs['topology_005']}
 for arm in ['topology_005','topology_015']:
  pairs=[(base[key(r)],r) for r in rs[arm] if key(r) in base and r['success'] and base[key(r)]['success']]
  summary['paired'][arm]={'n':len(pairs),'wall_speedup':quant([b['wall_ms']/r['wall_ms'] for b,r in pairs]),'length_ratio':quant([r['length_m']/b['length_m'] for b,r in pairs]),'band_ratio_delta':quant([r['quality']['right_band_ratio']-b['quality']['right_band_ratio'] for b,r in pairs if r.get('quality',{}).get('right_band_ratio') is not None and b.get('quality',{}).get('right_band_ratio') is not None])}
 coarse=rs['topology_015'];summary['paired']['new_arms_same_hard_corridor']=sum(r.get('hard_corridor_hash')==fine[key(r)].get('hard_corridor_hash') for r in coarse if key(r) in fine)
 summary['paired']['coarse_only_cold']={'n':sum(r['repetition']==1 and not r.get('fallbacks') for r in coarse),'fine_l2_ms':quant([fine[key(r)]['l2_ms'] for r in coarse if r['repetition']==1 and not r.get('fallbacks')]),'coarse_l2_ms':quant([r['l2_ms'] for r in coarse if r['repetition']==1 and not r.get('fallbacks')])}
 for arm in roots:
  d=WORK/'results'/f'prep_{a.tag}_{arm}'
  repaired=WORK/'results'/f'prep_formal_02_{arm}'
  if a.tag=='formal_01' and repaired.exists():d=repaired
  for phase in ['build','restore']:
   if (d/(phase+'.json')).exists():summary['preparation'].setdefault(arm,{})[phase]={**json.loads((d/(phase+'.json')).read_text()),'source_directory':str(d)}
 for scene in ['calibration','rotated']:
  for arm in roots:
   d=WORK/'results'/f'control_{a.tag}_{scene}_{arm}'
   if not (d/'runs.jsonl').exists():continue
   for r in read(d):
    file=Path(r['case'])/'path.npy'
    if not file.exists():summary['controls'].append({'scene':scene,'arm':arm,'success':False});continue
    pth=np.load(file);ds=np.linalg.norm(np.diff(pth[:,:2],axis=0),axis=1);s=np.r_[0,np.cumsum(ds)];bounds=np.r_[np.arange(0,s[-1],.025),s[-1]];mid=(bounds[:-1]+bounds[1:])/2;weights=np.diff(bounds);xy=np.column_stack([np.interp(mid,s,pth[:,i]) for i in [0,1]])
    right=(xy[:,1]-.5)/5. if scene=='calibration' else (7.5-xy[:,0])/7.
    mask=(mid>=5)&(mid<=s[-1]-5);low,high=(.125,.375) if r['side']=='right' else (.625,.875)
    band=float(weights[mask&(right>=low)&(right<=high)].sum()/weights[mask].sum())
    summary['controls'].append({'scene':scene,'arm':arm,'repetition':r['repetition'],'side':r['side'],'switch_index':r['switch_index'],'success':r['success'],'target_band_ratio':band,'eligible_arc_m':float(weights[mask].sum()),'hard_corridor_hash':r['hard_corridor_hash'],'pose_hash':r.get('pose_hash'),'mean_transverse_m':float(np.average(xy[mask,1 if scene=='calibration' else 0],weights=weights[mask]))})
 d=WORK/'results'/f'narrow_{a.tag}_topology_015'
 if (d/'runs.jsonl').exists():summary['narrow']=[{'success':r['success'],'fallbacks':r.get('fallbacks'),'audit':r['audit']} for r in read(d)]
 json_write(WORK/'report/performance.json',summary);json_write(WORK/'report/path_quality.json',quality)
 print(json.dumps({k:{'success':x['success'],'wall_ms':x['all']['wall_ms'],'cold':x['cold']['wall_ms'],'warm':x['warm']['wall_ms'],'fallback':x['fallback_count']} for k,x in summary['arms'].items()},indent=2))
if __name__=='__main__':main()
