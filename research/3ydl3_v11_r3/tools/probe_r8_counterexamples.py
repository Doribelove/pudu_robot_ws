"""Offline initializer diagnostic. Original physical certificate is retained."""
import ctypes,json,os,time,types
from pathlib import Path
import numpy as np
from replay_curve import load
from three_yd_v5 import curve_l3,curve_native

R=Path(__file__).resolve().parents[1]
O=R/'results/pose_counterexample_probe';O.mkdir(exist_ok=False)
os.sched_setaffinity(0,{15})
code=Path(curve_l3.__file__).read_text()
code=code.replace('B.T@B+1e-4*(E.T@E)','B.T@B+1e-7*(E.T@E)')
needle="free_tangent=config.get('free_endpoint_tangent_scale',False)"
code=code.replace(needle,"fitA=(B.T@B+1e-7*(E.T@E)+sparse.eye(n)*1e-8).tocsc();fitfixed=np.array([0,1,n-2,n-1]);fitfree=np.arange(2,n-2);cp[fitfree]=spsolve(fitA[fitfree][:,fitfree],(B.T@targets)[fitfree]-fitA[fitfree][:,fitfixed]@cp[fitfixed])\n            "+needle)
code=code.replace('targets=interp(p,s,u);tangents=',"u=np.unique(np.r_[u,config.get('extra_stations',[])]);targets=interp(p,s,u);tangents=")
code=code.replace(needle,"if config.get('warm_model') is not None and len(config['warm_model']['knots'])==len(knots) and np.array_equal(config['warm_model']['knots'],knots):cp=np.array(config['warm_model']['coefficients'])\n            "+needle)
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
w,d,m=load(R/'results/scope_recovery_probe/pudu_wanda_3f/curve_inputs/Q09_004.npz')
p=np.load(R/'results/semantic_witness_probe/guide.npy')

from scipy.interpolate import BSpline
step=.08
cfg=dict(m['config'],r4_rounded_guide=True,straight_knot_m=4.,turn_knot_m=step,body_refinement_passes=6,tracking_weight=1000.,bending_weight=.0001,jerk_weight=.000001,r4_guide_epsilon_cap=0.,constraint_step_m=.05,r4_adaptive_stations=False,adaptive_constraint_stations=True,certified_quadratic_start=False,r4_defer_recovery=False,r4_recovery_only=False,curvature_target_ratio=.99,r4_tighter_primary=False,max_refinements=0,max_iterations=35,r5_early_seed=False)
model=json.loads((R/'results/pose_full_probe/0.08.json').read_text())['model']
extra=[]
for trial in range(5):
    sp=BSpline(model['knots'],model['coefficients'],3);u=np.unique(np.r_[np.arange(sp.t[3],sp.t[-4],.005),sp.t[-4]]);v=sp(u,1);a=sp(u,2);k=abs(v[:,0]*a[:,1]-v[:,1]*a[:,0])/np.maximum(np.linalg.norm(v,axis=1),1e-9)**3
    bad=np.flatnonzero(k>2.49)
    if len(bad):
        cuts=np.r_[0,np.flatnonzero(np.diff(bad)>1)+1,len(bad)]
        for lo,hi in zip(cuts[:-1],cuts[1:]):
            group=bad[lo:hi];at=u[group[np.argmax(k[group])]];extra.extend([at-.01,at,at+.01])
    cfg.update(extra_stations=extra,warm_model=model)
    path,model,log=mod._optimize_mesh(w,d['physical'],p[:,:2],d['start'],d['goal'],time.monotonic()+10.,cfg)
    print(trial,len(extra),path is not None,log.get('failure'),flush=True)
    (O/f'{trial}.json').write_text(json.dumps({'log':log,'model':model,'extra':extra},indent=2))
    if path is not None:np.save(O/f'{trial}.npy',path);break
    if model is None:break
