"""Offline initializer diagnostic. Original physical certificate is retained."""
import ctypes,json,os,time,types
from pathlib import Path
import numpy as np
from replay_curve import load
from three_yd_v5 import curve_l3,curve_native

R=Path(__file__).resolve().parents[1]
O=R/'results/full_native_sat_probe';O.mkdir(exist_ok=False)
os.sched_setaffinity(0,{15})
code=Path(curve_l3.__file__).read_text()
code=code.replace('B.T@B+1e-4*(E.T@E)','B.T@B+1e-7*(E.T@E)')
needle="free_tangent=config.get('free_endpoint_tangent_scale',False)"
code=code.replace(needle,"fitA=(B.T@B+1e-7*(E.T@E)+sparse.eye(n)*1e-8).tocsc();fitfixed=np.array([0,1,n-2,n-1]);fitfree=np.arange(2,n-2);cp[fitfree]=spsolve(fitA[fitfree][:,fitfree],(B.T@targets)[fitfree]-fitA[fitfree][:,fitfixed]@cp[fitfixed])\n            "+needle)
code=code.replace("G=sparse.vstack((*G,J,-J),format='csc');m=G.shape[0]", "G=sparse.vstack((*G,J,-J),format='csc');collisionG,collisiong=posecells.linearize(B,Y,pos,v);G=sparse.vstack((G,collisionG),format='csc');g=np.r_[g,collisiong];limit=np.r_[limit,np.zeros(len(collisiong))];m=G.shape[0]")
mod=types.ModuleType('three_yd_v5.curve_pose_probe');mod.__file__=curve_l3.__file__
exec(compile(code,mod.__file__,'exec'),mod.__dict__)
foot=mod.footprint_constraints;viol=mod.violation
mod.footprint_constraints=lambda B,Y,v,hl,hw,axes:foot(B,Y,v,0.,0.,axes)
def violation(p,v,speed,k,boxes,vehicle,axes,curvature_ratio=.98):
    a,b=viol(p,v,speed,k,boxes,(0.,0.,vehicle[2]),axes,curvature_ratio);c,d=mod.posecells.violation(p,v)
    return a+c,max(b,d)
mod.violation=violation
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

import sys
sys.path.insert(0,str(R/'experiments'))
from pose_sat_constraints import PoseCells
w,d,m=load(R/'results/scope_recovery_probe/pudu_wanda_3f/curve_inputs/Q09_004.npz')
p=np.load(R/'results/full_discrete_pose_probe/whole_pose.npy')
mod.posecells=PoseCells(w,w.hard|~d['physical'],p[:,:2])
for step in [.08]:
    mod.knot_vector=lambda pp,ss,config,refinement=0:np.r_[[0.]*3,np.linspace(0,ss[-1],int(np.ceil(ss[-1]/step))+1),[ss[-1]]*3]
    cfg=dict(m['config'],r4_rounded_guide=False,tracking_weight=100.,slack_penalty=1e5,bending_weight=.0001,jerk_weight=.000001,r4_guide_epsilon_cap=0.,constraint_step_m=.02,r4_adaptive_stations=False,adaptive_constraint_stations=True,certified_quadratic_start=False,r4_defer_recovery=False,r4_recovery_only=False,curvature_target_ratio=.999,r4_tighter_primary=False,max_refinements=0,max_iterations=75,r5_early_seed=False,body_refinement_passes=6)
    path,model,log=mod._optimize_mesh(w,d['physical'],p[:,:2],p[0],p[-1],time.monotonic()+12.,cfg)
    print(step,path is not None,log.get('failure'),flush=True)
    (O/f'{step}.json').write_text(json.dumps({'log':log,'model':model},indent=2))
    if path is not None:np.save(O/f'{step}.npy',path)
