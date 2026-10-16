import sys,os,json,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
from crossmap_inputs import load_world
# Rejected radius/initializer variants are isolated from the production package.
import importlib.util
_spec=importlib.util.spec_from_file_location('three_yd_v5._diagnostic_curve_l3',ROOT/'experiments/diagnostic_rejected/curve_l3_radius_probe.py')
_mod=importlib.util.module_from_spec(_spec);sys.modules[_spec.name]=_mod;_spec.loader.exec_module(_mod)
optimize,CONFIG,feasible_guide,endpoint_seeds=(_mod.optimize,_mod.CONFIG,_mod.feasible_guide,_mod.endpoint_seeds)
mid,qid,variant=sys.argv[1:4];out=ROOT/'results/probe'/f'{mid}_{qid}_{variant}';out.mkdir(parents=True,exist_ok=False)
_,_,w,_,qs,_=load_world(mid,out);q=next(q for q in qs if q['id']==qid)
def pose(v):return tuple(v[k] for k in ('x','y','yaw_rad')) if isinstance(v,dict) else tuple(v)
old=Path('/home/robot/workspaces/3ydl3_v8_crossmap10_20261011/results/formal/round1')/mid/'cases'/f'{qid}_1'
prepared=np.load(ROOT/'results/inspect'/f'{mid}_{qid}'/'rank1.npz');guide=prepared['guide'];mask=prepared['mask'];cfg=dict(CONFIG)
if variant=='free':cfg.update(free_endpoint_tangent_scale=True)
if variant=='adaptive':cfg.update(adaptive_constraint_stations=True)
if variant.startswith('both'):cfg.update(adaptive_constraint_stations=True,free_endpoint_tangent_scale=True)
if variant=='smooth':cfg.update(adaptive_constraint_stations=True,free_endpoint_tangent_scale=True,bending_weight=.5,jerk_weight=.03,curvature_target_ratio=.9)
if variant=='radius':cfg.update(adaptive_constraint_stations=True,free_endpoint_tangent_scale=True,seed_radius_floor=.65)
if variant=='radius_fixed':cfg.update(seed_radius_floor=.65)
if variant.startswith('wide'):
 from three_yd_v5.geometry import variable_neighborhood,corners
 start=pose(q['start']);goal=pose(q['goal'])
 p,s,_=feasible_guide(guide,start,goal,w,prepared['physical'])
 p,s,seeds=endpoint_seeds(p,s,start,goal,w,prepared['physical'],time.monotonic()+5,.65 if 'radius' in variant else .405)
 print('SEEDS',seeds,flush=True);guide=p
 mask,info=variable_neighborhood(w,p,corners(p),forced_turning_points=p[np.r_[np.arange(len(p))<30,np.zeros(0,dtype=bool)]])
 mask &= prepared['physical']
 cfg.update(adaptive_constraint_stations=True)
 if 'radius' in variant:cfg.update(seed_radius_floor=.65)
 np.savez_compressed(out/'prepared.npz',mask=mask,guide=guide)
t=time.monotonic();p,model,detail=optimize(w,mask,guide,pose(q['start']),pose(q['goal']),t+15,cfg)
(out/'result.json').write_text(json.dumps(detail,indent=2));print(mid,qid,variant,detail.get('success'),detail.get('failure'),round(time.monotonic()-t,2),flush=True)
if model:(out/'model.json').write_text(json.dumps(model))
if p is not None:np.save(out/'path.npy',p)
