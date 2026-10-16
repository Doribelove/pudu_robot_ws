"""Diagnostic-only subdivision of the frozen failed L2 reference; no formal result."""
if __name__=='__main__':
 from owned_process import supervised_entry
 supervised_entry()
import json,time,math,os
from pathlib import Path
import numpy as np
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.path_audit import PathAuditor
from arena_evaluation.planner_benchmark.models import Query
from arena_3d_v1.multires_grid import GridView
from arena_3d_v1.semantic_world import json_write
from three_yd_v5.free_space_topology import load_map_only_world,FreeSpaceTopology
from three_yd_v5.controller import V5Controller
from three_yd_v5.geometry import variable_neighborhood
from three_yd_v5.performance_session import PerformanceSession
from three_yd_v4.r5_controller import SoftEdgeController
ROOT=Path(__file__).resolve().parents[1]
def main():
 import argparse
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True); args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
 os.environ['ROS_DOMAIN_ID']='225'
 mp=ROOT/'inputs/arena_large_250x300_2d_005';row=next(q for q in json.loads((mp/'queries.json').read_text())['queries'] if q['id']=='A2B-03');q=Query(row['id'],tuple(row['start']),tuple(row['goal']))
 w=load_map_only_world(mp/'map.yaml',args.output/'world');top=FreeSpaceTopology(w,args.output/'graph');route=top.routes(q,limit=1)[0]
 c=V5Controller(w,top,route,cache_root=args.output/'l2_cache',views={3:GridView(w,3),1:GridView(w,1)})
 forced=np.concatenate((c.rectangle_bridge_points,c.obstacle_repair_points));c._search_mask,info=variable_neighborhood(w,c.reference_plan.world_xy,c.corner_records,forced_turning_points=forced)
 json_write(args.output/'scope_info.json',info); np.savez_compressed(args.output/'scope.npz',mask=c._target_mask())
 xy=c.reference_plan.world_xy;arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
 def pose(s):
  i=min(len(xy)-2,max(0,int(np.searchsorted(arc,s,side='right')-1)));f=(s-arc[i])/max(1e-9,arc[i+1]-arc[i]);d=xy[i+1]-xy[i];return (*list(xy[i]+f*d),math.atan2(d[1],d[0]))
 bounds=[0.,100.,200.,300.,float(arc[-1])];poses=[q.start]+[pose(s) for s in bounds[1:-1]]+[q.goal]
 context=runtime.MapContext('arena_large_250x300_2d_005',w.map,w.safe,w.map.distance_m,w.map.sha256,w.map.sha256,mp/'map.yaml');auditor=PathAuditor(context,source_commit='V8-r1-diagnostic');spec=runtime.backend_availability()['hybrid_astar']
 session=PerformanceSession(context,args.output/'smac',map_yaml=mp/'map.yaml',local_mask_updates=True,optimization_profile='v7_candidate',smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.,tile_interval_s=.015,enable_semantic_repair=True);session.local_map_update_strategy='roi_ack';session.full_grid_settle_cycles=0
 try:
  session.start()
  for j,(a,b) in enumerate(zip(poses,poses[1:])):
   qq=Query(f'part{j}',a,b);session.reset_query_state(qq.query_id,restore_base_map=False);session.active_controller=c;session.active_route=route;tick=time.monotonic();out=SoftEdgeController.plan_l3(c,qq,session,auditor,spec);result=out.pop('result',None);out['wall_ms']=(time.monotonic()-tick)*1000;json_write(args.output/f'part{j}.json',out)
   if result and result.points:np.save(args.output/f'part{j}.npy',np.array([[p['x'],p['y'],p['yaw']] for p in result.points]))
   print(j,a,b,out.get('success'),out.get('failure_code'),out['wall_ms'],flush=True)
 finally:session.close()
if __name__=='__main__':main()
