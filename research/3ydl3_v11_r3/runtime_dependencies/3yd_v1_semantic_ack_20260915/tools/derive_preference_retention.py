"""Post-run metrics on saved paths; verify reconstructed L1 signature first."""
import argparse,json,os,math
from pathlib import Path
import numpy as np,yaml
from arena_evaluation.planner_benchmark.models import Query
from arena_3d_v1.semantic_world import WORK,SemanticWorld,json_write
from arena_3d_v1.semantic_graph import SemanticPoseGraph
from arena_3d_v1.semantic_reference import preference_field,ordered_match

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--run',required=True);args=parser.parse_args();root=Path(args.run);os.sched_setaffinity(0,{15})
 cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());w=SemanticWorld(cfg['inputs']['map_yaml'],cfg['inputs']['semantic_path'],cache_root=WORK/'cache');g=SemanticPoseGraph(w,WORK/'cache/graph')
 queries={q['query_id']:q for q in yaml.safe_load((WORK/'config/queries.yaml').read_text())['queries']}
 rows=list(map(json.loads,(root/'runs.jsonl').read_text().splitlines()));results=[]
 for name in dict.fromkeys(r['query_id'] for r in rows):
  d=queries[name];q=Query(name,tuple(map(float,d['start'])),tuple(map(float,d['goal'])),seed=d.get('seed',20260914));route=g.plan(q)
  if route is None:continue
  pref=preference_field(w,route);output=root/'derived'/name;output.mkdir(parents=True,exist_ok=True)
  np.save(output/'l1_poses.npy',route.poses);np.save(output/'corridor.npy',route.plan.corridor_mask)
  json_write(output/'l1.json',{'derived_after_run':True,'route_signature':route.plan.route_signature,'nodes':[n.__dict__ for n in route.nodes],'edge_ids':list(route.plan.route_edge_ids),'graph_key':g.key})
  for row in rows:
   if row['query_id']!=name or row['arm']!='C' or not row['success']:continue
   if route.plan.route_signature!=row['reference_diagnostics']['route_signature']:raise ValueError('reconstructed route differs from recorded signature')
   case=Path(row['outcome_path']).parent;p=np.load(case/'path.npy');rc=np.load(case/'l2.npy');l2=w.world_points(rc[:,0],rc[:,1]);dy=np.diff(l2,axis=0);yaw=np.arctan2(dy[:,1],dy[:,0]);reference=np.column_stack([l2,np.r_[yaw,yaw[-1]]])
   ds=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1);arc=np.r_[0,np.cumsum(ds)];bounds=np.r_[np.arange(0,arc[-1],.025),arc[-1]];mid=(bounds[:-1]+bounds[1:])/2;weights=np.diff(bounds)
   xy=np.column_stack([np.interp(mid,arc,p[:,0]),np.interp(mid,arc,p[:,1])]);r,c=w.cells(xy)
   refs,_,ix,distance,amb=ordered_match(w,route,r,c,reference=reference);valid=(ix>=0)&~amb
   match=np.zeros((len(ix),2));match[valid]=refs[ix[valid],:2];mr,mc=w.cells(match)
   selected=valid&pref.eligible[r,c]&(mid>=5)&(mid<=arc[-1]-5)
   mr=np.clip(mr,0,w.map.height-1);mc=np.clip(mc,0,w.map.width-1)
   reference_q=pref.normalized_right[mr,mc];output_q=pref.normalized_right[r,c]
   source_band=(reference_q>=.125)&(reference_q<=.375)&pref.eligible[mr,mc]
   target_band=(output_q>=.125)&(output_q<=.375)
   denom=selected&source_band;length=float(weights[denom].sum());kept=float(weights[denom&target_band].sum())
   result={'query_id':name,'repetition':row['repetition'],'reference_preferred_matched_arc_m':length,'retained_preferred_arc_m':kept,'conditional_retention':kept/length if length else None,'ambiguous_output_arc_m':float(weights[amb].sum()),'eligible_output_arc_m':float(weights[selected].sum()),'route_signature_verified':True,'source_case':str(case)}
   results.append(result)
 json_write(root/'derived/preference_retention.json',{'definition':'L3 arc in target band divided by L3 arc whose ordered same-semantic L2 match was in target band; 5m transitions excluded, ambiguous matches excluded and reported','sample_spacing_m':.025,'partial_final_interval_included':True,'rows':results})
 print('derived',len(results),'C trajectories',flush=True)
if __name__=='__main__':main()
