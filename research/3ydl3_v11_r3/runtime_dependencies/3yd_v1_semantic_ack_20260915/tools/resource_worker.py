"""Small real ROS worker used to verify whole-process ownership cleanup."""
if __name__=='__main__':
 from owned_process import supervised_entry
 supervised_entry()
from native_snapshot import configure
configure()
import argparse,hashlib,os,time
from pathlib import Path
from arena_3d_v1.semantic_world import SemanticWorld,json_write
from arena_3d_v1.semantic_ack_session import SemanticAckSession
from arena_evaluation.unified_four_backends_smoke import MapContext
from recovery_scenes import return_bulb_inputs
p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--wait',action='store_true');p.add_argument('--domain',type=int,default=106);a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=False);os.environ['ROS_DOMAIN_ID']=str(a.domain);os.sched_setaffinity(0,{15})
mp,sp,_=return_bulb_inputs(out/'inputs');w=SemanticWorld(mp,sp);ctx=MapContext('resource_worker',w.map,w.safe,w.map.distance_m,w.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp)
session=SemanticAckSession(ctx,out/'smac',map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.)
session.local_map_update_strategy='roi_ack';session.full_grid_settle_cycles=0
try:
 session.start();__import__('native_snapshot').record(out);session.update_local_mask(w.safe)
 json_write(out/'ready.json',{'pid':os.getpid(),'startup_and_update_ack':True})
 if a.wait:
  until=time.monotonic()+60
  while time.monotonic()<until:time.sleep(.1)
finally:session.close()
