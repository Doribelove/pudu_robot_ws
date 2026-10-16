"""Offline continuous-control initialization experiment, not navigation."""
import time,math,json,os
from pathlib import Path
import numpy as np
import cv2
from scipy.optimize import minimize
from scipy.ndimage import map_coordinates
from replay_curve import load
from three_yd_v5.curve_l3 import feasible_guide,interp
from three_yd_v5.v10_storage import collision_checker
from arena_evaluation.semantic_constraint_core import dubins_choices
from arena_3d_v1.semantic_world import densify
R=Path(__file__).resolve().parents[1];O=R/'results/mentor_control_fit';O.mkdir(exist_ok=False)
os.sched_setaffinity(0,{14})
w,d,m=load(R/'results/diagnostic_initial/mentor_map_20260825_005/curve_inputs/A2B-07_000.npz')
p,s,_=feasible_guide(d['guide'],d['start'],d['goal'],w,d['physical']);at=s[-1]-3;xy=interp(p,s,[at])[0];v=np.diff(interp(p,s,[at-.05,at+.05]),axis=0)[0]
a=d['goal'].copy();a[2]+=math.pi;b=np.r_[xy,np.arctan2(v[1],v[0])+math.pi]
distance=(cv2.distanceTransform((~w.hard).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)-cv2.distanceTransform(w.hard.astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE))*.05
fx,fy=np.meshgrid(np.linspace(-.265,.265,9),np.linspace(-.225,.225,9));foot=np.c_[fx.ravel(),fy.ravel()];checker=collision_checker(w.hard,w);records=[]
for radius in [.5,.405,.8]:
 for word,params in dubins_choices(a,b,radius):
    length=radius*sum(params)
    if length>8:continue
    n=max(40,int(math.ceil(length/.07)));ds=length/n;bins=np.arange(n)*ds;lengths=np.array(params)*radius;seed=np.zeros(n)
    for j,kind in enumerate(word):
        lo=sum(lengths[:j]);hi=lo+lengths[j];seed+=np.maximum(0,np.minimum(bins+ds,hi)-np.maximum(bins,lo))/ds*(0 if kind=='S' else (1 if kind=='L' else -1)/radius)
    winding=sum((0 if kind=='S' else 1 if kind=='L' else -1)*par for kind,par in zip(word,params));deadline=time.monotonic()+6
    def coords(k):
        delta=k*ds;mid=a[2]+np.cumsum(delta)-delta/2;q=delta/2;amplitude=ds*np.sinc(q/np.pi)
        return amplitude*np.cos(mid),amplitude*np.sin(mid),mid,q
    def poses(k):
        dx,dy,_,_=coords(k);return np.c_[a[0]+np.r_[0,np.cumsum(dx)],a[1]+np.r_[0,np.cumsum(dy)],a[2]+np.r_[0,np.cumsum(k*ds)]]
    def equality(k):
        dx,dy,_,_=coords(k);return np.array([a[0]+sum(dx)-b[0],a[1]+sum(dy)-b[1],sum(k)*ds-winding])
    def clearance(k):
        pp=poses(k);co=np.cos(pp[:,2,None]);si=np.sin(pp[:,2,None]);xx=pp[:,0,None]+co*foot[:,0]-si*foot[:,1];yy=pp[:,1,None]+si*foot[:,0]+co*foot[:,1]
        rr=w.hard.shape[0]-.5-yy/.05;cc=xx/.05-.5
        return map_coordinates(distance,[rr,cc],order=1,mode='constant',cval=-1.).min(axis=1)-.015
    def objective(k):return float(.02*np.sum((k-seed)**2)+.1*np.sum(np.diff(k)**2))
    def callback(k):
        if time.monotonic()>deadline:raise TimeoutError
    try:
        sol=minimize(objective,np.clip(seed,-2.499,2.499),method='SLSQP',bounds=[(-2.499,2.499)]*n,constraints=[{'type':'eq','fun':equality},{'type':'ineq','fun':clearance}],callback=callback,options={'maxiter':100,'ftol':1e-9})
        path=poses(sol.x);ok=np.max(abs(equality(sol.x)))<1e-6 and checker.check(densify(path));rec={'word':word,'radius':radius,'status':str(sol.message),'residual':float(np.max(abs(equality(sol.x)))),'clearance':float(clearance(sol.x).min()),'body_valid':bool(ok),'full_navigation_success':False};np.save(O/f'{len(records)}.npy',path)
    except TimeoutError:rec={'word':word,'radius':radius,'status':'diagnostic_timeout','full_navigation_success':False}
    records.append(rec);print(rec,flush=True)
    (O/'records.json').write_text(json.dumps(records,indent=2)+'\n')
