from pathlib import Path
import argparse,json,math,os,time,hashlib
import numpy as np
import yaml
from PIL import Image
from arena_evaluation.semantic_map import SemanticMapV1,SemanticFeature
from arena_evaluation.planner_benchmark.models import Query
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.path_audit import PathAuditor
from arena_3d_v1.semantic_world import SemanticWorld,WORK,json_write
from arena_3d_v1.semantic_graph import SemanticPoseGraph
from arena_3d_v1.semantic_pipeline import SemanticR1Controller,ReferenceSmacSession

def controlled_inputs(root,heldout=False):
 root.mkdir(parents=True,exist_ok=True)
 width,height=(8,48) if heldout else (40,6);res=.05
 img=np.zeros((int((height+2)/res),int((width+2)/res)),np.uint8)
 img[20:-20,20:-20]=255;Image.fromarray(img).save(root/'map.pgm')
 (root/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':res,'origin':[-1.,-1.,0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
 feature=SemanticFeature('controlled_lane','lane','polygon',[[0,0],[width,0],[width,height],[0,height],[0,0]],soft=True,direction_rule='route_tangent_right')
 semantic=SemanticMapV1('map',res,(-1.,-1.,0.),img.shape[1],img.shape[0],hashlib.sha256(img.tobytes()).hexdigest(),[feature],{'right_hand_drive':True})
 json_write(root/'semantic.json',semantic.to_dict())
 return root/'map.yaml',root/'semantic.json'

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--side',default='right');p.add_argument('--arm',default='C');p.add_argument('--cap',type=int,default=140);p.add_argument('--heldout',action='store_true');p.add_argument('--l2-only',action='store_true');args=p.parse_args()
 out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
 mp,sp=controlled_inputs(WORK/'data'/('heldout' if args.heldout else 'calibration'),args.heldout)
 world=SemanticWorld(mp,sp,cache_root=WORK/'cache/controlled');graph=SemanticPoseGraph(world,WORK/'cache/controlled_graph');graph.prepare()
 start=(4.,5.,math.pi/2) if args.heldout else (5.,3.,0.);goal=(4.,41.,math.pi/2) if args.heldout else (35.,3.,0.)
 q=Query('controlled',start,goal,seed=20260914)
 t=time.monotonic();route=graph.plan(q);print('L1',bool(route),flush=True)
 if route is None:json_write(out/'result.json',{'success':False,'reason':'L1_NO_VERIFIED_ROUTE'});return
 controller=SemanticR1Controller(world,route,cache_root=WORK/'cache/controlled_l2',side=args.side,l2_weight=0. if args.arm=='A' else 2.)
 print('L2',controller.initial_l2_result.success,controller.initial_l2_result.response_ms,flush=True)
 l2=controller.l2.path_global
 if l2:np.save(out/'l2_cells.npy',np.asarray(l2))
 np.save(out/'corridor.npy',route.plan.corridor_mask)
 if args.l2_only:return
 ctx=runtime.MapContext('semantic_controlled',world.map,world.safe,world.map.distance_m,world.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp)
 spec=runtime.backend_availability()['hybrid_astar'];auditor=PathAuditor(ctx,source_commit='semantic-dual-map-r1-isolated')
 session=ReferenceSmacSession(ctx,out/'smac',map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.)
 session.local_map_update_strategy='roi_ack';session.full_grid_settle_cycles=0
 try:
  session.start();outcome=controller.plan_l3(q,session,auditor,spec,enabled=args.arm=='C',cap=args.cap)
  result=outcome.pop('result',None)
  if result and result.points:
   json_write(out/'path.json',result.points)
   path=np.asarray([[v['x'],v['y'],v['yaw']] for v in result.points]);np.save(out/'path.npy',path)
  json_write(out/'result.json',outcome);print('RESULT',outcome.get('success'),outcome.get('failure_code'),outcome.get('independent_audit'),flush=True)
 finally:session.close()
if __name__=='__main__':main()
