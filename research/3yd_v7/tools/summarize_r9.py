"""Summarize serialized, unprofiled r8/r9 controls and exact path parity."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]

def rows(name):
 return [json.loads(x) for x in (ROOT/'results'/name/'runs.jsonl').read_text().splitlines()]

def stats(values):
 a=np.asarray(values,float)
 return {'n':len(a),'median':float(np.median(a)),'p95':float(np.quantile(a,.95)),
         'max':float(a.max()),'mean':float(a.mean())}

def summarize(rs):
 out={'runs':len(rs),'success':sum(r['success'] for r in rs)}
 getters={
 'wall_s':lambda r:r['wall_ms']/1000,
 'rss_gib':lambda r:r['rss_peak_bytes']/2**30,
 'l1_ms':lambda r:r['recovery'].get('l1_actual_ms',0.),
 'controller_all_s':lambda r:r['recovery']['all_candidates_controller_ms']/1000,
 'adapter_all_ms':lambda r:r['recovery']['all_candidates_adapter_ms'],
 'adapter_selected_ms':lambda r:r['adapter']['adapter_actual_ms'],
 'l3_pipeline_all_s':lambda r:r['recovery']['all_candidates_l3_pipeline_ms']/1000,
 'map_update_all_s':lambda r:sum(a.get('l3_diagnostics',{}).get('local_map_update_ms',0.) for a in r['recovery']['attempts'])/1000,
 'l3_action_all_s':lambda r:sum(a.get('l3_diagnostics',{}).get('l3_action_wall_ms',0.) for a in r['recovery']['attempts'])/1000,
 'curvature_repair_all_s':lambda r:sum(a.get('curvature_repair_ms',0.) for a in r['recovery']['attempts'])/1000,
 'strict_audit_all_ms':lambda r:sum(a.get('l2_timing',{}).get('strict_audit_total_ms',0.) for a in r['recovery']['attempts']),
 'planner_pss_gib':lambda r:r['l3']['planner_pss_peak_bytes']/2**30,
 'planner_rss_gib':lambda r:r['l3']['planner_rss_peak_bytes']/2**30,
 'stack_pss_gib':lambda r:r['l3']['stack_pss_peak_bytes']/2**30,
 'stack_rss_gib':lambda r:r['l3']['stack_rss_peak_bytes']/2**30,
 'post_export_tree_pss_gib':lambda r:sum(p['pss'] for p in r['memory_processes'])/2**30,
 'l2_states_mib':lambda r:r['l2']['state_memory_bytes']/2**20,
 'right_fraction':lambda r:r['right_side_fraction'],
 'length_m':lambda r:r['length_m'],
 }
 for subset,items in [('all',rs),('first',[r for r in rs if r['repeat']==1]),('warm',[r for r in rs if r['repeat']>1])]:
  out[subset]={k:stats([fn(r) for r in items]) for k,fn in getters.items()}
 out['cache_hit_adapter_ms']=stats([r['adapter']['adapter_actual_ms'] for r in rs if r['adapter']['adapter_cache_hit']])
 out['cache_miss_adapter_ms']=stats([r['adapter']['adapter_actual_ms'] for r in rs if not r['adapter']['adapter_cache_hit']])
 out['fallback_runs']=sum(r['l2']['selected_backend']=='v4_bounded_astar_fallback' for r in rs if not r['l2'].get('reused_search_result',False))
 out['l3_ack_all']=all(r['l3'].get('costmap_update_acknowledged') for r in rs)
 out['within_mask_all']=all(r['l3'].get('returned_path_within_mask') for r in rs)
 out['per_query']={q:stats([r['wall_ms']/1000 for r in rs if r['query_id']==q]) for q in sorted({r['query_id'] for r in rs})}
 return out

def main():
 names=['r8_isolated_10x3','r9_balanced_10x3']
 data={name:rows(name) for name in names}
 for name,rs in data.items():
  if len(rs)!=30:raise RuntimeError(f'{name}: expected 30, got {len(rs)}')
 out={name:summarize(rs) for name,rs in data.items()}
 parity=[]
 for q in range(1,11):
  for rep in range(1,4):
   case=f'Q{q:02}_{rep}';checks={}
   for file in ('l2_reference_xy.npy','path.npy'):
    paths=[ROOT/'results'/name/'cases'/case/file for name in names]
    checks[file]=all(p.exists() for p in paths) and np.array_equal(np.load(paths[0]),np.load(paths[1]))
   for file in ('l1_output.json','l2_soft_target.json'):
    values=[json.loads((ROOT/'results'/name/'cases'/case/file).read_text()) for name in names]
    if file=='l1_output.json':
     checks[file]=values[0]['right_reference_polyline']==values[1]['right_reference_polyline']
    else:checks[file]=values[0]['points']==values[1]['points']
   parity.append({'case':case,**checks})
 out['path_parity']=parity
 (ROOT/'report/PERFORMANCE.json').write_text(json.dumps(out,indent=2))
 print(json.dumps({k:{'success':v['success'],'time':v['all']['wall_s'],'RSS':v['all']['rss_gib'],'first':v['first']['wall_s'],'warm':v['warm']['wall_s'],'adapter_hit_ms':v['cache_hit_adapter_ms'],'adapter_miss_ms':v['cache_miss_adapter_ms'],'planner_PSS':v['all']['planner_pss_gib'],'stack_PSS':v['all']['stack_pss_gib'],'fallback_runs':v['fallback_runs']} for k,v in out.items() if k!='path_parity'},indent=2))
 print('parity',sum(all(v for k,v in p.items() if k!='case') for p in parity),'/',len(parity))
if __name__=='__main__':main()
