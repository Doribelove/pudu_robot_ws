import os,json,sys,time,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
for k,v in json.loads((ROOT/'BASELINE.json').read_text()).items():
 if k.endswith('_mode') and k!='selected_mode':os.environ['THREEYD_V9_'+{'v9_mode':'MODE','review_mode':'R1_MODE','r2_mode':'R2_MODE','r3_mode':'R3_MODE','r5_mode':'R5_MODE','r6_mode':'R6_MODE','r7_mode':'R7_MODE','r8_mode':'R8_MODE','r9_mode':'R9_MODE'}[k] if k not in ('v9_mode',) else 'THREEYD_V9_MODE']=v
os.environ['THREEYD_L3_BACKEND']='independent'
import numpy as np
from crossmap_inputs import load_world
from arena_3d_v1.multires_grid import GridView
from arena_evaluation.planner_benchmark.models import Query
from three_yd_v5.controller import V5Controller
from three_yd_v5.v9_maps import prepare_world
from three_yd_v3.dynamic import DynamicObstacles
from three_yd_v5.curve_l3 import optimize
mid,qid=sys.argv[1:3];out=ROOT/'results/inspect'/f'{mid}_{qid}';out.mkdir(parents=True,exist_ok=True)
semantic,mp,w,g,qs,qf=load_world(mid,out);views={3:GridView(w,3),1:GridView(w,1)};prepare_world(w);dynamic=DynamicObstacles(w)
qd=next(q for q in qs if q['id']==qid)
def pose(v):return tuple(v[k] for k in ('x','y','yaw_rad')) if isinstance(v,dict) else tuple(v)
q=Query(qid,pose(qd['start']),pose(qd['goal']),seed=20260916 if semantic else 20260929)
for i,r in enumerate(g.routes(q,limit=1)):
 c=V5Controller.__new__(V5Controller)
 try:c.__init__(w,g,r,cache_root=out/'cache',views=views,dynamic=dynamic,weight=2.)
 except Exception as e:
  from run_wanda3w import json_native
  detail={'error':str(e),'timing':getattr(c,'timing',{}),'traceback':traceback.format_exc()};(out/f'rank{i+1}_error.json').write_text(json.dumps(json_native(detail),indent=2))
  if hasattr(c,'search_legal'):np.savez_compressed(out/f'rank{i+1}_scope.npz',mask=c.search_legal,hard=w.hard,guide=np.asarray(g.task(r)['right_reference_polyline']),start=q.start,goal=q.goal)
  print('L2_ERROR',str(e),json.dumps(json_native(detail['timing'])),flush=True);continue
 from three_yd_v5.geometry import variable_neighborhood
 forced=np.concatenate((np.asarray(getattr(c,'rectangle_bridge_points',[]),float).reshape(-1,2),np.asarray(getattr(c,'obstacle_repair_points',[]),float).reshape(-1,2)))
 mask,info=variable_neighborhood(w,c.reference_plan.world_xy,c.corner_records,forced_turning_points=forced);c._search_mask=mask;target=c._target_mask()
 # Save diagnostics only; original planner masks and input geometry remain unchanged.
 np.savez_compressed(out/f'rank{i+1}.npz',mask=target,physical=c.plan.corridor_mask,guide=c.reference_plan.world_xy,hard=w.hard,start=q.start,goal=q.goal)
 from arena_3d_v1.semantic_world import densify
 def path(xy):
  d=np.gradient(xy,axis=0);return densify(np.c_[xy,np.arctan2(d[:,1],d[:,0])])
 print('GUIDE_TRAFFIC',i+1,c.traffic.audit(path(c.reference_plan.world_xy)),flush=True)
 from three_yd_v5.curve_l3 import CONFIG
 for weight in (2.,5.,15.,40.):
  cfg=dict(CONFIG);cfg['tracking_weight']=weight
  pp,mm,dd=optimize(w,target,c.reference_plan.world_xy,q.start,q.goal,time.monotonic()+12,cfg)
  ta=c.traffic.audit(pp) if pp is not None else {}
  print('WEIGHT',weight,dd.get('success'),dd.get('failure'),ta,flush=True)
  (out/f'traffic_weight_{weight}.json').write_text(json.dumps({'curve':mm,'diagnostics':dd,'traffic':ta}))
