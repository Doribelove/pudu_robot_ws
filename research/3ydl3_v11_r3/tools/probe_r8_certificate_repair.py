"""Offline counterexample-guided repair with the unmodified final certifier."""
import inspect,json,time,sys,os
from pathlib import Path
import numpy as np
from scipy import sparse
from scipy.interpolate import BSpline
from replay_curve import load
from three_yd_v5 import curve_l3
from three_yd_v5.curve_qp import clarabel_setup,clarabel_solve
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'experiments'))
from pose_sat_constraints import PoseCells
os.sched_setaffinity(0,{14});O=R/'results/certificate_repair_probe_b';O.mkdir(exist_ok=False)
w,d,m=load(R/'results/scope_recovery_probe/pudu_wanda_3f/curve_inputs/Q09_004.npz')
model=json.loads((R/'results/local_sat_patch_probe_b/last_model.json').read_text())
knots=np.array(model['knots']);cp=np.array(model['coefficients']);history=[]
source=inspect.getsource(curve_l3.certify)
source=source.replace("raise CurveFailure('CURVE_CONTINUOUS_BODY_OR_SCOPE')", "raise CurveFailure(json.dumps({'kind':'body','left':left[bad].tolist(),'right':right[bad].tolist()}))")
diagnostic=dict(curve_l3.__dict__);exec(source,diagnostic)

def insert(knots,cp,x):
    k=np.searchsorted(knots,x,side='right')-1;s=int(np.count_nonzero(knots==x));p=3
    if s>=3 or np.min(abs(knots-x))<1e-9:return knots,cp
    out=np.empty((len(cp)+1,2));out[:k-p+1]=cp[:k-p+1];out[k-s+1:]=cp[k-s:]
    for i in range(k-p+1,k-s+1):
        a=(x-knots[i])/(knots[i+p]-knots[i]);out[i]=a*cp[i]+(1-a)*cp[i-1]
    return np.insert(knots,k+1,x),out

for trial in range(8):
    sp=BSpline(knots,cp,3);record={'trial':trial,'control_points':len(cp)}
    try:
        path,cert=curve_l3.certify(sp,w,d['physical'],time.monotonic()+5.,m['config'],model['corridor'])
        record.update(success=True,certificate=cert);history.append(record)
        np.save(O/'path.npy',path);(O/'model.json').write_text(json.dumps(dict(model,knots=knots.tolist(),coefficients=cp.tolist()),indent=2));print('WHOLE_CERTIFIED',trial,flush=True);break
    except curve_l3.CurveFailure as e:record['failure']=str(e)
    try:diagnostic['certify'](sp,w,d['physical'],time.monotonic()+5.,m['config'],model['corridor'])
    except curve_l3.CurveFailure as e:
        try:witness=json.loads(str(e))
        except ValueError:record['diagnostic_failure']=str(e);history.append(record);print(record,flush=True);break
    left=np.array(witness['left']);right=np.array(witness['right']);order=np.argsort(left);left=left[order];right=right[order]
    bands=[]
    for lo,hi in zip(left,right):
        if bands and lo<=bands[-1][1]+.01:bands[-1][1]=max(bands[-1][1],hi)
        else:bands.append([lo,hi])
    record['bands']=bands;record['patches']=[];record['inserted_knots']=0
    for lo,hi in bands:
        sp=BSpline(knots,cp,3);u=np.r_[np.arange(lo,hi,.0005),hi];cells=PoseCells(w,w.hard|~d['physical'],sp(u));idx,gap,_,_=cells.gaps(sp(u),sp(u,1))
        if gap.min(initial=1.)>=.00025:
            before=sp(u);count=0
            for x in np.linspace(lo,hi,max(2,int(np.ceil((hi-lo)/.005)))+1)[1:-1]:
                if knots[3]+1e-9<x<knots[-4]-1e-9:knots,cp=insert(knots,cp,float(x));count+=1
            assert np.max(abs(BSpline(knots,cp,3)(u)-before))<1e-10
            record['inserted_knots']+=count;continue
        lo=max(knots[3],lo-.4);hi=min(knots[-4],hi+.4);u=np.r_[np.arange(lo,hi,.001),hi]
        n=len(cp);z=np.r_[cp[:,0],cp[:,1]];B=curve_l3.basis(knots,u);D=curve_l3.basis(knots,u,1);E=curve_l3.basis(knots,u,2);F=curve_l3.basis(knots,u,3)
        free=curve_l3.basis(knots,[lo,hi]).nonzero()[1];free=np.arange(max(2,free.min()-2),min(n-2,free.max()+3));variables=np.r_[free,n+free]
        cells=PoseCells(w,w.hard|~d['physical'],B@cp);patch={'band':[lo,hi],'minimum_gap_before':float(gap.min(initial=1.))}
        for iteration in range(3):
            pos,v,_,_,_,k,J,Y=curve_l3.differential(B,D,E,F,z);G,g=cells.linearize(B,Y,pos,v);g+=.0005-1e-6
            A=sparse.vstack([G[:,variables],J[:,variables],-J[:,variables],sparse.eye(len(variables))],format='csc')
            lower=np.r_[np.full(len(g)+2*len(u),-np.inf),np.full(len(variables),-.02)];upper=np.r_[-g,2.49-k,2.49+k,np.full(len(variables),.02)]
            sol=clarabel_solve(clarabel_setup(sparse.eye(len(variables),format='csc')*2,np.zeros(len(variables)),A,lower,upper,time.monotonic()+2.,1e-8));patch['status']=sol.info.status
            if sol.info.status_val not in (1,2):break
            z[variables]+=sol.x;cp=np.c_[z[:n],z[n:]]
            if abs(sol.x).max()<1e-5:break
        record['patches'].append(patch)
    history.append(record);print({k:v for k,v in record.items() if k!='certificate'},flush=True)
    (O/'history.json').write_text(json.dumps(history,indent=2))
(O/'history.json').write_text(json.dumps(history,indent=2))
(O/'last_model.json').write_text(json.dumps(dict(model,knots=knots.tolist(),coefficients=cp.tolist()),indent=2))
