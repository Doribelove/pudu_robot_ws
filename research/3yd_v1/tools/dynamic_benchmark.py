"""One frozen calibration route, confirmed obstacle add/remove, actual Smac."""
import argparse,os,json,time,hashlib
from pathlib import Path
import numpy as np
from controlled_probe import controlled_inputs
from benchmark import Memory,arc_metrics
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.path_audit import PathAuditor
from arena_evaluation.planner_benchmark.models import Query
from arena_evaluation.dynamic_snapshot import DynamicSnapshot
from arena_3d_v1.semantic_world import WORK,SemanticWorld,json_write,digest
from arena_3d_v1.semantic_graph import SemanticPoseGraph
from arena_3d_v1.semantic_pipeline import SemanticR1Controller,ReferenceSmacSession

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
 os.environ['ROS_DOMAIN_ID']='226';os.environ['ROS_LOCALHOST_ONLY']='1';os.sched_setaffinity(0,{15})
 mp,sp=controlled_inputs(WORK/'data/calibration');w=SemanticWorld(mp,sp,cache_root=WORK/'cache/controlled');g=SemanticPoseGraph(w,WORK/'cache/controlled_graph');g.prepare()
 q=Query('dynamic_control',(5.,3.,0.),(35.,3.,0.),seed=20260914);r=g.plan(q)
 ctx=runtime.MapContext('dynamic_control',w.map,w.safe,w.map.distance_m,w.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp);spec=runtime.backend_availability()['hybrid_astar'];auditor=PathAuditor(ctx,source_commit='isolated-explicit-r1-semantic-dynamic')
 c=SemanticR1Controller(w,r,cache_root=out/'l2_cache');path=c.l2.path_global;cell=path[len(path)//2]
 session=ReferenceSmacSession(ctx,out/'smac',map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.)
 session.local_map_update_strategy='roi_ack';session.full_grid_settle_cycles=0;rows=[];index=0
 try:
  session.start()
  for name,occupied in [('initial',None),('obstacle_added',[cell]),('obstacle_removed',[])]:
   row={'stage':name,'source_cell':list(cell)};started=time.monotonic();cpu=time.process_time();case=out/name;case.mkdir()
   with Memory() as mem:
    try:
     session.current_query_id=name
     if occupied is not None:
      for _ in range(2):
       index+=1;s=DynamicSnapshot.from_cells(str(index),occupied,timestamp=float(index),map_version=w.map.sha256,map_shape=w.hard.shape)
       step=c.process_snapshot(s,graph=g,query=q,now=float(index))
      row['update']={'l3_required':step.l3_required,'l1_called':step.l1_graph_astar_called,'l1_reroute_succeeded':step.l1_reroute_succeeded,'scheduler':step.scheduler.reason,'blocked_count':len(c.confirmation.blocked_cells),'dirty_cells':step.dirty_roi.changed_cells if step.dirty_roi else 0,'l2':step.l2_result.diagnostics if step.l2_result else None}
     np.save(case/'l2.npy',np.asarray(c.l2.path_global));np.save(case/'hard.npy',w.hard|~c._target_mask())
     outcome=c.plan_l3(q,session,auditor,spec);result=outcome.pop('result',None)
     if result and result.points:
      pts=np.asarray([[p['x'],p['y'],p['yaw']] for p in result.points]);np.save(case/'path.npy',pts);row['pose_hash']=digest(pts)
      row['preference']=arc_metrics(w,pts,c.preference)
     row.update(success=bool(outcome.get('success')),failure_code=outcome.get('failure_code',''),outcome=outcome)
    except Exception as e:row.update(success=False,failure_code=type(e).__name__+': '+str(e))
   row.update(wall_ms=(time.monotonic()-started)*1000,cpu_ms=(time.process_time()-cpu)*1000,peak_process_tree_rss_bytes=mem.peak,process_tree_cpu_ms=mem.tree_cpu_ms)
   rows.append(row);json_write(case/'result.json',row);json_write(out/'runs.json',rows);print(name,row['success'],row['failure_code'],flush=True)
 finally:session.close()
if __name__=='__main__':main()
