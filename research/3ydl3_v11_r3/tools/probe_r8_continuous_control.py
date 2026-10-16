"""Bounded offline control diagnostic. No constraint changes to runtime.

Optimize piecewise constant curvature with exact circular integration and
analytic control Jacobians. The fixed query goal is the backward initial pose;
its position/yaw never move. Final native body checks are independent of fit.
"""
import argparse,json,math,os,time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from replay_curve import load
from three_yd_v5.failure_pose_cells import PoseCells
from three_yd_v5.v10_storage import collision_checker

R=Path(__file__).resolve().parents[1]

def integrate(a,k,ds):
    n=len(k);before=a[2]+np.r_[0.,np.cumsum(k[:-1])*ds]
    z=k*ds/2;sc=np.sinc(z/np.pi)
    deriv=np.where(abs(z)<1e-5,-z/3+z**3/30,(z*np.cos(z)-np.sin(z))/np.maximum(z*z,1e-30))
    angle=before+z;co=np.cos(angle);si=np.sin(angle)
    delta=ds*sc[:,None]*np.c_[co,si]
    xy=np.vstack([a[:2],a[:2]+np.cumsum(delta,axis=0)])
    theta=a[2]+np.r_[0.,np.cumsum(k)*ds]
    local=ds*ds/2*np.c_[deriv*co-sc*si,deriv*si+sc*co]
    row=np.arange(n+1)[:,None];col=np.arange(n)[None,:];active=row>col
    Jx=np.where(active,local[None,:,0]-ds*(xy[:,None,1]-xy[None,1:,1]),0.)
    Jy=np.where(active,local[None,:,1]+ds*(xy[:,None,0]-xy[None,1:,0]),0.)
    Jt=active.astype(float)*ds
    return np.c_[xy,theta],Jx,Jy,Jt

def sampled_integrate(a,k,ds,subdivisions):
    p,Jx,Jy,Jt=integrate(a,k,ds)
    if subdivisions==1:return p,Jx,Jy,Jt
    n=len(k);idx=np.repeat(np.arange(n),subdivisions)
    tau=np.tile(np.arange(subdivisions)*ds/subdivisions,n)
    angle=p[idx,2]+k[idx]*tau/2;z=k[idx]*tau/2;sc=np.sinc(z/np.pi)
    deriv=np.where(abs(z)<1e-5,-z/3+z**3/30,(z*np.cos(z)-np.sin(z))/np.maximum(z*z,1e-30))
    co=np.cos(angle);si=np.sin(angle);dx=tau*sc*co;dy=tau*sc*si
    X=Jx[idx]-dy[:,None]*Jt[idx];Y=Jy[idx]+dx[:,None]*Jt[idx];T=Jt[idx].copy()
    X[np.arange(len(idx)),idx]+=tau*tau/2*(deriv*co-sc*si)
    Y[np.arange(len(idx)),idx]+=tau*tau/2*(deriv*si+sc*co)
    T[np.arange(len(idx)),idx]+=tau
    pp=np.c_[p[idx,0]+dx,p[idx,1]+dy,p[idx,2]+k[idx]*tau]
    return np.vstack([pp,p[-1]]),np.vstack([X,Jx[-1]]),np.vstack([Y,Jy[-1]]),np.vstack([T,Jt[-1]])

def residuals(cells,p,Jx,Jy,Jt,seed,goal,body_weight):
    # The cell set is frozen per station to keep least-squares dimensions fixed.
    idx,gap,grad,yaw=cells.gaps(p[:,:2],np.c_[np.cos(p[:,2]),np.sin(p[:,2])])
    need=.0003-gap;active=need>0
    G=-grad[:,0,None]*Jx[idx]-grad[:,1,None]*Jy[idx]-yaw[:,None]*Jt[idx]
    G[~active]=0
    r=np.r_[body_weight*np.maximum(need,0.),.1*(p[:,:2]-seed[:,:2]).ravel(),
            10*(p[-1,:2]-goal[:2]),100*(p[-1,2]-goal[2])]
    J=np.vstack([body_weight*G,.1*np.stack([Jx,Jy],axis=1).reshape(-1,Jx.shape[1]),
                 10*Jx[-1],10*Jy[-1],100*Jt[-1]])
    return r,J,float(gap.min(initial=1.))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--n',type=int,default=150)
    parser.add_argument('--limit',type=float,default=2.5);parser.add_argument('--seconds',type=float,default=50)
    parser.add_argument('--weight',type=float,default=1000);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--subdivisions',type=int,default=1);parser.add_argument('--warm',type=Path)
    parser.add_argument('--seed',type=Path)
    args=parser.parse_args();args.output.mkdir(exist_ok=False,parents=True);os.sched_setaffinity(0,{14})
    w,d,m=load(R/'results/diagnostic_initial/mentor_map_20260825_005/curve_inputs/A2B-07_000.npz')
    seed=np.load(args.seed or R/'results/mentor_radius_sensitivity/0.39.npy')
    assert np.max(abs(seed[0,:2]-d['goal'][:2]))<1e-9
    assert abs(math.atan2(math.sin(seed[0,2]-d['goal'][2]-math.pi),math.cos(seed[0,2]-d['goal'][2]-math.pi)))<1e-9
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(seed[:,:2],axis=0),axis=1))]
    seed[:,2]=np.unwrap(seed[:,2]);u=np.linspace(0,arc[-1],args.n+1);ds=arc[-1]/args.n
    reference=np.c_[[np.interp(u,arc,seed[:,j]) for j in range(3)]].T
    a=reference[0];goal=reference[-1];k=np.diff(reference[:,2])/ds;k=np.clip(k,-args.limit,args.limit)
    if args.warm:k=np.load(args.warm)
    if args.subdivisions>1:
        u=np.linspace(0,arc[-1],args.n*args.subdivisions+1)
        reference=np.c_[[np.interp(u,arc,seed[:,j]) for j in range(3)]].T
    cells=PoseCells(w,w.hard,reference[:,:2]);fixed=cells.tree.query_ball_point(reference[:,:2],.95)
    class Fixed:
        def query_ball_point(self,*args):return fixed
    cells.tree=Fixed();best=None;cached=None;started=time.monotonic();calls=0
    def evaluate(k):
        nonlocal cached,best,calls
        if cached is not None and np.array_equal(k,cached[0]):return cached[1:3]
        if time.monotonic()-started>args.seconds:raise TimeoutError('diagnostic deadline')
        p,Jx,Jy,Jt=sampled_integrate(a,k,ds,args.subdivisions);r,J,gap=residuals(cells,p,Jx,Jy,Jt,reference,goal,args.weight)
        score=np.linalg.norm(r)
        if best is None or score<best[0]:best=(score,k.copy(),p.copy(),gap)
        calls+=1;cached=(k.copy(),r,J)
        if calls%20==0:print({'calls':calls,'residual_norm':float(score),'gap':gap},flush=True)
        return r,J
    status=None
    try:
        result=least_squares(lambda z:evaluate(z)[0],k,jac=lambda z:evaluate(z)[1],bounds=(-args.limit,args.limit),max_nfev=300,ftol=1e-10,xtol=1e-10,gtol=1e-10)
        status=result.message
    except TimeoutError as error:status=str(error)
    _,k,p,gap=best;check=collision_checker(w.hard,w)
    dense=[];current=a.copy()
    for control in k:
        many,_x,_y,_t=integrate(current,np.full(max(1,int(np.ceil(ds/.002))),control),ds/max(1,int(np.ceil(ds/.002))))
        dense.extend(many[:-1]);current=many[-1]
    dense=np.vstack([dense,current]);valid=check.check(dense)
    report={'status':str(status),'calls':calls,'elapsed_s':time.monotonic()-started,'min_sample_gap_m':gap,'max_control_curvature':float(abs(k).max()),'body_valid':valid,'endpoint_error_m':float(np.linalg.norm(p[-1,:2]-goal[:2])),'yaw_error_rad':float(p[-1,2]-goal[2]),'exact_query_goal_unchanged':True,'full_navigation':False,'physical_infeasibility_proven':False}
    np.save(args.output/'controls.npy',k);np.save(args.output/'poses.npy',dense)
    (args.output/'result.json').write_text(json.dumps(report,indent=2));print(report,flush=True)

if __name__=='__main__':main()
