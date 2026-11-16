"""Bounded spline proposal with rectangle/cell SAT; independent sweep certifies output.

No occupancy edits or footprint shrink. End positions and tangent directions fixed.
L2 adherence remains a nonzero soft residual. Optimizer convergence is not success.
"""
import time, math
import numpy as np
from scipy.interpolate import BSpline
from scipy.optimize import least_squares
from scipy.sparse import diags, vstack, hstack, csr_matrix
from scipy.sparse.linalg import lsqr
from scipy.spatial import cKDTree
from scipy import ndimage
from arena_3d_v1.semantic_world import wrap


def bases(t, knots, order=3):
    n=len(knots)-order-1
    b=BSpline.design_matrix(t,knots,order).tocsr()
    den=knots[order+1:order+n]-knots[1:n]
    d=diags([-order/den,order/den],[0,1],shape=(n-1,n)).tocsr()
    b1=BSpline.design_matrix(t,knots[1:-1],order-1).tocsr()@d
    den2=knots[order+1:order+n-1]-knots[2:n]
    d2=diags([-(order-1)/den2,(order-1)/den2],[0,1],shape=(n-2,n-1)).tocsr()
    b2=BSpline.design_matrix(t,knots[2:-2],order-2).tocsr()@d2@d
    return b,b1,b2


def rectangle_sat(xy,theta,tree,half_length,half_width,half_cell,all_contacts=False):
    # Hard cell centres near the body. Nearest queries only propose geometry;
    # the separate final checker examines every overlapped cell and full sweep.
    _, ids=tree.query(xy,k=min(48,tree.n),workers=1)
    if ids.ndim==1:ids=ids[:,None]
    delta=xy[:,None,:]-tree.data[ids];dx,dy=delta[:,:,0],delta[:,:,1]
    c=np.cos(theta)[:,None];s=np.sin(theta)[:,None]
    ac,ass=abs(c),abs(s);dc=-np.sign(c)*s;ds=np.sign(s)*c
    dot=dx*c+dy*s;cross=-dx*s+dy*c
    sep=np.stack([abs(dx)-half_length*ac-half_width*ass-half_cell,
                  abs(dy)-half_length*ass-half_width*ac-half_cell,
                  abs(dot)-half_length-half_cell*(ac+ass),
                  abs(cross)-half_width-half_cell*(ac+ass)],axis=2)
    axis=sep.argmax(axis=2);separation=sep.max(axis=2)
    ones=np.ones_like(dx);c=c*ones;s=s*ones;dc=dc*ones;ds=ds*ones
    gx=np.choose(axis,[np.sign(dx),np.zeros_like(dx),np.sign(dot)*c,-np.sign(cross)*s])
    gy=np.choose(axis,[np.zeros_like(dx),np.sign(dy),np.sign(dot)*s,np.sign(cross)*c])
    gt=np.choose(axis,[-half_length*dc-half_width*ds,-half_length*ds-half_width*dc,
                    np.sign(dot)*cross-half_cell*(dc+ds),-np.sign(cross)*dot-half_cell*(dc+ds)])
    if all_contacts:return separation,gx,gy,gt
    j=separation.argmin(axis=1);i=np.arange(len(xy))
    return separation[i,j],gx[i,j],gy[i,j],gt[i,j]



def optimize_path(world,xy,start,goal,mask,*,budget_s=10.,spacing=.45,max_shift=.65,obstacle_weight=80.):
    began=time.monotonic();deadline=began+budget_s
    xy=np.vstack((np.asarray(start)[:2],np.asarray(xy)[:,:2],np.asarray(goal)[:2]))
    xy=xy[np.r_[True,np.linalg.norm(np.diff(xy,axis=0),axis=1)>1e-6]]
    arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))];length=arc[-1]
    breaks=np.linspace(0,length,max(4,int(np.ceil(length/spacing)))+1)
    knots=np.r_[[0]*3,breaks,[length]*3];n=len(knots)-4
    ts=np.linspace(0,length,max(3,int(np.ceil(length/.05)))+1)
    B,D,E=bases(ts,knots);target=np.column_stack([np.interp(ts,arc,xy[:,k]) for k in range(2)])
    fixed=np.array([0,1,n-2,n-1]);free=np.arange(2,n-2)
    ends=np.array([start[:2],np.asarray(start[:2])+breaks[1]/3*np.array([math.cos(start[2]),math.sin(start[2])]),
        np.asarray(goal[:2])-(length-breaks[-2])/3*np.array([math.cos(goal[2]),math.sin(goal[2])]),goal[:2]])
    A=B[:,free];V=D[:,free];W=E[:,free]
    p0=B[:,fixed]@ends;v0=D[:,fixed]@ends;e0=E[:,fixed]@ends
    initial=np.column_stack([lsqr(A,target[:,j]-p0[:,j],atol=1e-8,btol=1e-8)[0] for j in range(2)])
    hard=world.hard|~mask
    boundary=hard & ~ndimage.binary_erosion(hard)
    rr,cc=np.nonzero(boundary);lo=xy.min(axis=0)-max_shift-1.;hi=xy.max(axis=0)+max_shift+1.
    points=world.world_points(rr,cc);take=((points>=lo)&(points<=hi)).all(axis=1);tree=cKDTree(points[take])
    if not tree.n:raise ValueError('EMPTY_HARD_BOUNDARY')
    z0=np.r_[initial[:,0],initial[:,1]];nf=len(free);zero=csr_matrix(A.shape)
    last={};calls=0;best=z0.copy();best_score=float('inf')
    class Budget(Exception):pass
    def evaluate(z,jac=False):
        nonlocal calls,best,best_score
        if time.monotonic()>deadline:raise Budget()
        if 'z' in last and np.array_equal(last['z'],z):return last['jac'] if jac else last['r']
        calls+=1;ctrl=np.column_stack((z[:nf],z[nf:]));p=A@ctrl+p0;v=V@ctrl+v0;a=W@ctrl+e0
        vx,vy=v[:,0],v[:,1];ax,ay=a[:,0],a[:,1];speed2=np.maximum(vx*vx+vy*vy,1e-8);speed=np.sqrt(speed2)
        theta=np.arctan2(vy,vx);cross=vx*ay-vy*ax;curv=cross/speed**3
        sep,gx,gy,gt=rectangle_sat(p,theta,tree,*world.vehicle[:2],world.map.resolution/2,True)
        repeats=sep.shape[1];row_ids=np.repeat(np.arange(len(ts)),repeats);AA=A[row_ids];VV=V[row_ids]
        sep,gx,gy,gt=[v.ravel() for v in (sep,gx,gy,gt)];vxo=np.repeat(vx,repeats);vyo=np.repeat(vy,repeats);so=np.repeat(speed2,repeats)
        # A tiny numerical separation targets closed-square contact. This is
        # not a map inflation or any relaxation of the original final audit.
        active=sep<.004;obs=obstacle_weight*np.minimum(sep-.004,0)
        Jox=diags(obstacle_weight*active*gx)@AA+diags(-obstacle_weight*active*gt*vyo/so)@VV
        Joy=diags(obstacle_weight*active*gy)@AA+diags(obstacle_weight*active*gt*vxo/so)@VV
        kc=40.*(abs(curv)>2.35)*np.sign(curv)
        Jkx=diags(kc*(ay/speed**3-3*cross*vx/speed**5))@V+diags(-kc*vy/speed**3)@W
        Jky=diags(kc*(-ax/speed**3-3*cross*vy/speed**5))@V+diags(kc*vx/speed**3)@W
        residual=[.2*(p[:,0]-target[:,0]),.2*(p[:,1]-target[:,1]),.035*ax,.035*ay,
                  .2*(speed-1),40*np.maximum(abs(curv)-2.35,0),obs]
        mats=[hstack((.2*A,zero)),hstack((zero,.2*A)),hstack((.035*W,zero)),hstack((zero,.035*W)),
              hstack((diags(.2*vx/speed)@V,diags(.2*vy/speed)@V)),hstack((Jkx,Jky)),hstack((Jox,Joy))]
        residual=np.concatenate(residual);matrix=vstack(mats,format='csr');score=np.dot(residual,residual)
        if score<best_score:best_score=score;best=z.copy()
        last.update(z=z.copy(),r=residual,jac=matrix)
        return matrix if jac else residual
    status='OPTIMIZER_COMPLETE'
    try:
        result=least_squares(evaluate,z0,jac=lambda z:evaluate(z,True),bounds=(z0-max_shift,z0+max_shift),
                    max_nfev=150,ftol=1e-7,xtol=1e-7,gtol=1e-5,tr_solver='lsmr',tr_options={'maxiter':40})
        best=result.x;status=str(result.message)
    except Budget:status='OPTIMIZER_BUDGET'
    ctrl=np.empty((n,2));ctrl[fixed]=ends;ctrl[free]=np.column_stack((best[:nf],best[nf:]))
    # Denser output derives heading from the geometric derivative, never
    # overwrites yaw on an unrelated polyline.
    tout=np.linspace(0,length,max(3,int(np.ceil(length/.02)))+1)
    spl=BSpline(knots,ctrl,3);p=spl(tout);v=spl(tout,nu=1);out=np.column_stack((p,np.arctan2(v[:,1],v[:,0])))
    return out,{'method':'bounded_cubic_spline_rectangle_SAT','status':status,'wall_ms':(time.monotonic()-began)*1000,
        'evaluations':calls,'control_count':n,'soft_L2_adherence':.2,'max_control_shift_m':max_shift,'optimizer_is_certificate':False,'objective':best_score}
