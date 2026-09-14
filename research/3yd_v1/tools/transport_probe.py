"""Transport-only diagnostics. No solver or preference tuning."""
import os,time,json
from pathlib import Path
import numpy as np
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation import two_layer_v2_semantic_benchmark as context_util
from arena_3d_v1.semantic_world import WORK,json_write
from arena_3d_v1.semantic_pipeline import ReferenceSmacSession
os.environ['ROS_DOMAIN_ID']='227';os.environ['ROS_LOCALHOST_ONLY']='1';os.sched_setaffinity(0,{14})
mp=Path('/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/extracted/optemap.yaml');ctx=context_util._context(mp)
out=WORK/'results/cold_transport_probe_03';out.mkdir(exist_ok=False);rows=[]
for i in range(3):
 s=ReferenceSmacSession(ctx,out/str(i),map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.)
 s.local_map_update_strategy='roi_ack';s.full_grid_settle_cycles=0
 t=time.monotonic()
 try:s.start();r={'repeat':i+1,'success':True,'startup_wall_ms':(time.monotonic()-t)*1000}
 except Exception as e:r={'repeat':i+1,'success':False,'reason':str(e)}
 finally:s.close()
 rows.append(r);print(r,flush=True);json_write(out/'results.json',rows)
