"""Offline initializer diagnostic. Original physical certificate is retained."""
import ctypes,json,os,time,types
from pathlib import Path
import numpy as np
from replay_curve import load
from three_yd_v5 import curve_l3,curve_native

R=Path(__file__).resolve().parents[1]
O=R/'results/mentor_continuation_probe';O.mkdir(exist_ok=False)
os.sched_setaffinity(0,{15})
code=Path(curve_l3.__file__).read_text()
code=code.replace('B.T@B+1e-4*(E.T@E)','B.T@B+1e-7*(E.T@E)')
needle="free_tangent=config.get('free_endpoint_tangent_scale',False)"
code=code.replace(needle,"fitA=(B.T@B+1e-7*(E.T@E)+sparse.eye(n)*1e-8).tocsc();fitfixed=np.array([0,1,n-2,n-1]);fitfree=np.arange(2,n-2);cp[fitfree]=spsolve(fitA[fitfree][:,fitfree],(B.T@targets)[fitfree]-fitA[fitfree][:,fitfixed]@cp[fitfixed])\n            "+needle)
mod=types.ModuleType('three_yd_v5.curve_pose_probe');mod.__file__=curve_l3.__file__
exec(compile(code,mod.__file__,'exec'),mod.__dict__)
foot=mod.footprint_constraints;viol=mod.violation
mod.footprint_constraints=lambda B,Y,v,hl,hw,axes:foot(B,Y,v,0.,0.,axes)
mod.violation=lambda p,v,speed,k,boxes,vehicle,axes,curvature_ratio=.98:viol(p,v,speed,k,boxes,(0.,0.,vehicle[2]),axes,curvature_ratio)
lib=ctypes.CDLL(str(R/'experiments/libcurve_rectangles_center.so'));fn=lib.curve_rectangles_center
fn.argtypes=curve_native._lib.curve_rectangles.argtypes;fn.restype=ctypes.c_int
def rectangles(hard,res,origin,guide,deadline,tangents=None):
    h=np.ascontiguousarray(hard,dtype=np.uint8);p=np.ascontiguousarray(guide,dtype=float);t=np.ascontiguousarray(tangents,dtype=float)
    out=np.empty((len(p),4));axes=np.empty_like(out)
    result=fn(h.ctypes.data,*h.shape,float(res),*map(float,origin[:2]),p.ctypes.data,t.ctypes.data,len(p),curve_native._offsets.ctypes.data,0,max(0.,deadline-time.monotonic()),out.ctypes.data,axes.ctypes.data)
    if result:raise curve_l3.CurveFailure('CURVE_BUDGET' if result==1 else 'CURVE_GUIDE_OUTSIDE_SCOPE')
    out+=np.array([-.0195,.0195,-.0195,.0195])
    return out,axes
curve_native.rectangles=rectangles

w,d,m=load(R/'results/diagnostic_initial/mentor_map_20260825_005/curve_inputs/A2B-07_000.npz')
from three_yd_v5.curve_l3 import clean_guide,interp
from three_yd_v5.geometry import variable_neighborhood,corners
w.v7_map_only=True
w.cells=lambda xy:(w.hard.shape[0]-1-np.floor((xy[:,1]-w.map.origin[1])/w.map.resolution).astype(int),np.floor((xy[:,0]-w.map.origin[0])/w.map.resolution).astype(int))
w.world_points=lambda rr,cc:np.c_[w.map.origin[0]+(cc+.5)*w.map.resolution,w.map.origin[1]+(w.hard.shape[0]-rr-.5)*w.map.resolution]
p,s=clean_guide(d['guide'],d['start'],d['goal'],.22)
for radius in [.39,.38,.36]:
    seed=np.load(R/f'results/mentor_radius_sensitivity/{radius}.npy')[::-1].copy();seed[:,2]=np.arctan2(np.sin(seed[:,2]+np.pi),np.cos(seed[:,2]+np.pi))
    guide=np.vstack((p[s<s[-1]-3.],seed[:,:2]));mask,_=variable_neighborhood(w,guide,corners(guide),forced_turning_points=np.array([d['start'][:2],d['goal'][:2]]));mask=(mask|d['mask'])&d['physical']
    for ratio in [.99,.999]:
        cfg=dict(m['config'],r4_rounded_guide=True,tracking_weight=1000.,bending_weight=.0001,jerk_weight=.000001,r4_guide_epsilon_cap=0.,constraint_step_m=.02,r4_adaptive_stations=False,adaptive_constraint_stations=True,certified_quadratic_start=False,r4_defer_recovery=False,r4_recovery_only=False,curvature_target_ratio=ratio,r4_tighter_primary=False,max_refinements=1,max_iterations=50,r5_early_seed=False,straight_knot_m=2.,turn_knot_m=.08,body_refinement_passes=6)
        path,model,log=mod._optimize_mesh(w,mask,guide,d['start'],d['goal'],time.monotonic()+5.,cfg)
        print(radius,ratio,path is not None,log.get('failure'),flush=True)
        name=f'{radius}_{ratio}';(O/f'{name}.json').write_text(json.dumps({'log':log,'model':model},indent=2))
        if path is not None:np.save(O/f'{name}.npy',path)
