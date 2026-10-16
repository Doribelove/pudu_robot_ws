"""Diagnostic alternate convex cells; physical and continuous audits unchanged."""
from pathlib import Path
import ctypes,time,json,os
import numpy as np
from replay_curve import load
from three_yd_v5 import curve_l3,curve_native
from three_yd_v5.geometry import variable_neighborhood,corners
R=Path(__file__).resolve().parents[1];O=R/'results/tight_corridor_probe';O.mkdir(exist_ok=False)
os.sched_setaffinity(0,{14})
lib=ctypes.CDLL(str(R/'experiments/libcurve_rectangles_tight.so'));fn=lib.curve_rectangles_tight
fn.argtypes=curve_native._lib.curve_rectangles.argtypes;fn.restype=ctypes.c_int
def rectangles(hard,res,origin,guide,deadline,tangents=None):
    h=np.ascontiguousarray(hard,dtype=np.uint8);p=np.ascontiguousarray(guide,dtype=float);t=np.ascontiguousarray(tangents,dtype=float);out=np.empty((len(p),4));axes=np.empty_like(out)
    code=fn(h.ctypes.data,*h.shape,float(res),*map(float,origin[:2]),p.ctypes.data,t.ctypes.data,len(p),curve_native._offsets.ctypes.data,len(curve_native._offsets),max(0.,deadline-time.monotonic()),out.ctypes.data,axes.ctypes.data)
    if code:raise curve_l3.CurveFailure('CURVE_BUDGET' if code==1 else 'CURVE_GUIDE_OUTSIDE_SCOPE')
    # Mesh applies its inherited .02 inset. Compensate only in this diagnostic
    # so the final cell is inset .0005 and still strictly inside the clear cell.
    out+=np.array([-.0195,.0195,-.0195,.0195])
    return out,axes
curve_native.rectangles=rectangles
w,d,m=load(R/'results/scope_recovery_probe/pudu_wanda_3f/curve_inputs/Q09_004.npz')
for tag in ['original','witness']:
    guide=d['guide'] if tag=='original' else np.load(R/'results/semantic_witness_probe/guide.npy')
    mask=d['mask']
    if tag=='witness':
        w.cells=lambda xy:(w.hard.shape[0]-1-np.floor((xy[:,1]-w.map.origin[1])/w.map.resolution).astype(int),np.floor((xy[:,0]-w.map.origin[0])/w.map.resolution).astype(int))
        w.world_points=lambda rr,cc:np.c_[w.map.origin[0]+(cc+.5)*w.map.resolution,w.map.origin[1]+(w.hard.shape[0]-rr-.5)*w.map.resolution]
        mask,_=variable_neighborhood(w,guide,corners(guide),forced_turning_points=np.array([d['start'][:2],d['goal'][:2]]));mask=(mask|d['mask'])&d['physical']
    for ratio in [.98,.999]:
        cfg=dict(m['config'],r4_rounded_guide=True,tracking_weight=100.,adaptive_constraint_stations=True,r4_guide_epsilon_cap=.06,curvature_target_ratio=ratio)
        path,model,log=curve_l3._optimize_once(w,mask,guide,d['start'],d['goal'],time.monotonic()+6.,cfg);name=f'{tag}_{ratio}';print(name,path is not None,log.get('failure'),flush=True);(O/f'{name}.json').write_text(json.dumps({'log':log,'model':model},indent=2))
        if path is not None:np.save(O/f'{name}.npy',path)
