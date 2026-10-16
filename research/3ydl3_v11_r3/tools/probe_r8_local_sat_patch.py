"""Offline spline coefficient repair around a continuous-certificate witness."""
import json,time,sys,os
from pathlib import Path
import numpy as np
from scipy import sparse
from scipy.interpolate import BSpline
from replay_curve import load
from three_yd_v5 import curve_l3
from three_yd_v5.curve_qp import clarabel_setup,clarabel_solve
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'experiments'))
from pose_sat_constraints import PoseCells
os.sched_setaffinity(0,{14});O=R/'results/local_sat_patch_probe_b';O.mkdir(exist_ok=False)
w,d,m=load(R/'results/scope_recovery_probe/pudu_wanda_3f/curve_inputs/Q09_004.npz')
model=json.loads((R/'results/full_native_spline_probe/0.08.json').read_text())['model']
knots=np.array(model['knots']);cp=np.array(model['coefficients']);n=len(cp);z=np.r_[cp[:,0],cp[:,1]]
rec=json.loads((R/'results/certificate_counterexamples/0.08.json').read_text())
lo=min(rec['mids'])-.4;hi=max(rec['mids'])+.4
u=np.r_[np.arange(lo-.2,hi+.2,.001),hi+.2]
B=curve_l3.basis(knots,u);D=curve_l3.basis(knots,u,1);E=curve_l3.basis(knots,u,2);F=curve_l3.basis(knots,u,3)
free=np.unique(curve_l3.basis(knots,[lo,hi]).nonzero()[1])
free=np.arange(max(2,free.min()-2),min(n-2,free.max()+3));variables=np.r_[free,n+free]
cells=PoseCells(w,w.hard|~d['physical'],B@cp)
records=[]
for it in range(10):
    pos,v,acc,j,speed,k,J,Y=curve_l3.differential(B,D,E,F,z)
    G,g=cells.linearize(B,Y,pos,v);g+=.0005-1e-6
    A=sparse.vstack([G[:,variables],J[:,variables],-J[:,variables],sparse.eye(len(variables))],format='csc')
    lower=np.r_[np.full(len(g)+len(u)*2,-np.inf),np.full(len(variables),-.02)]
    upper=np.r_[-g,2.49-k,2.49+k,np.full(len(variables),.02)]
    solver=clarabel_setup(sparse.eye(len(variables),format='csc')*2,np.zeros(len(variables)),A,lower,upper,time.monotonic()+3.,1e-8)
    result=clarabel_solve(solver);log={'iteration':it,'status':result.info.status,'free_control_points':len(free)}
    if result.info.status_val not in (1,2):records.append(log);print(log,flush=True);break
    z[variables]+=result.x;cp=np.c_[z[:n],z[n:]];sp=BSpline(knots,cp,3)
    pos,v,acc,j,speed,k,_,_=curve_l3.differential(B,D,E,F,z);_,gap,_,_=cells.gaps(pos,v)
    log.update(min_gap=float(gap.min()),max_k=float(abs(k).max()),max_move=float(abs(result.x).max()))
    try:
        path,cert=curve_l3.certify(sp,w,d['physical'],time.monotonic()+5.,m['config'],model['corridor'])
        log.update(success=True,certificate=cert);np.save(O/'path.npy',path)
        (O/'model.json').write_text(json.dumps(dict(model,coefficients=cp.tolist()),indent=2))
    except curve_l3.CurveFailure as error:log.update(success=False,failure=str(error))
    records.append(log);print({k:v for k,v in log.items() if k!='certificate'},flush=True)
    if log['success']:break
(O/'records.json').write_text(json.dumps(records,indent=2))
(O/'last_model.json').write_text(json.dumps(dict(model,coefficients=cp.tolist()),indent=2))
