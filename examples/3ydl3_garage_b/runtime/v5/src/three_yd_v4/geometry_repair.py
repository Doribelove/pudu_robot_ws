"""Local geometric repair with obstacle residuals and independent exact checks.

The spline is a proposal only: sampled distance fields are never a collision
certificate. Pose headings and curvature are derived from its real geometry.
"""
import time,math
import numpy as np
from scipy import ndimage
from scipy.interpolate import CubicSpline
from scipy.optimize import least_squares
from arena_3d_v1.semantic_world import SweptChecker,densify,wrap
from arena_3d_v1.curvature_repair import violating_segments

def repair_segment(world,p,a,b,mask,deadline):
    tick=time.monotonic();segment=p[a:b+1];lengths=np.linalg.norm(np.diff(segment[:,:2],axis=0),axis=1);arc=np.r_[0.,np.cumsum(lengths)]
    if arc[-1]<.2 or arc[-1]>15:return None,{'failure':'WINDOW_LENGTH_BUDGET'}
    length=arc[-1];knots=np.linspace(0,length,max(6,math.ceil(length/.35)+1))
    base=np.c_[np.interp(knots,arc,segment[:,0]),np.interp(knots,arc,segment[:,1])]
    # True anchor headings set clamped derivatives. They are not yaw metadata
    # applied to an unrelated polyline.
    first=np.array([math.cos(p[a,2]),math.sin(p[a,2])]);last=np.array([math.cos(p[b,2]),math.sin(p[b,2])])
    rc=np.column_stack(world.cells(segment));pad=40
    r0=max(0,int(rc[:,0].min())-pad);r1=min(world.map.height,int(rc[:,0].max())+pad+1)
    c0=max(0,int(rc[:,1].min())-pad);c1=min(world.map.width,int(rc[:,1].max())+pad+1)
    hard=world.hard|~mask;local=hard[r0:r1,c0:c1]
    sdf=(ndimage.distance_transform_edt(~local)-ndimage.distance_transform_edt(local))*world.map.resolution
    sdf-=np.sign(sdf)*world.map.resolution/2
    # Filled rectangle samples support the optimizer; exact whole rectangles
    # and continuous sweeps below remain the acceptance authority.
    u=np.linspace(-world.vehicle[0],world.vehicle[0],17);v=np.linspace(-world.vehicle[1],world.vehicle[1],17)
    body=np.array(np.meshgrid(u,v)).reshape(2,-1).T
    ts=np.linspace(0,length,max(12,math.ceil(length/.06)+1));maximum=1/world.vehicle[2]
    whole_lengths=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
    left=wrap(p[a,2]-p[a-1,2])/whole_lengths[a-1] if a else 0.
    right=wrap(p[b+1,2]-p[b,2])/whole_lengths[b] if b<len(p)-1 else 0.
    checker=SweptChecker(hard,world.map.resolution,world.map.origin,*world.vehicle[:2]);trials=[]
    def spline(x):
        pts=np.vstack((base[0],x.reshape(-1,2),base[-1]))
        return CubicSpline(knots,pts,bc_type=((1,first),(1,last)))
    def residual(x):
        if time.monotonic()>=deadline:raise TimeoutError('LOCAL_GEOMETRY_REPAIR_BUDGET')
        s=spline(x);xy=s(ts);d=s(ts,1);dd=s(ts,2);speed=np.maximum(np.linalg.norm(d,axis=1),1e-8)
        normal=d/speed[:,None];cross=d[:,0]*dd[:,1]-d[:,1]*dd[:,0];k=cross/speed**3
        points=xy[:,None,:]+body[None,:,0,None]*normal[:,None,:]+body[None,:,1,None]*np.stack((-normal[:,1],normal[:,0]),axis=1)[:,None,:]
        col=(points[:,:,0]-world.map.origin[0])/world.map.resolution-.5-c0
        row=world.map.height-.5-(points[:,:,1]-world.map.origin[1])/world.map.resolution-r0
        clearance=ndimage.map_coordinates(sdf,[row.ravel(),col.ravel()],order=1,mode='constant',cval=-1.)
        return np.r_[80*np.maximum(.032-clearance,0),
          .15*(x-base[1:-1].ravel()),.25*np.diff(k),
          15*np.maximum(abs(k)-(maximum-.03),0),
          4*np.array([k[0]-left,k[-1]-right]),2*np.maximum(.2-speed,0)]
    bounds=(base[1:-1].ravel()-.9,base[1:-1].ravel()+.9)
    for offset in [0.,.15,-.15,.3,-.3]:
        if time.monotonic()>=deadline:break
        direction=base[-1]-base[0];normal=np.array([-direction[1],direction[0]])/max(np.linalg.norm(direction),1e-9)
        seed=base[1:-1]+offset*np.sin(np.linspace(0,math.pi,len(base)))[1:-1,None]*normal
        try:sol=least_squares(residual,seed.ravel(),bounds=bounds,max_nfev=120,ftol=1e-5,xtol=1e-5,gtol=1e-5)
        except TimeoutError:break
        s=spline(sol.x);dense_t=np.linspace(0,length,max(20,math.ceil(length/.025)+1));xy=s(dense_t);d=s(dense_t,1)
        patch=np.c_[xy,np.arctan2(d[:,1],d[:,0])];patch[0]=p[a];patch[-1]=p[b]
        splice=np.vstack((p[max(0,a-1):a],patch,p[b+1:min(len(p),b+2)]))
        physical=checker.check(densify(splice));bad=violating_segments(splice,maximum)[0]
        trial={'seed_offset_m':offset,'evaluations':sol.nfev,'cost':float(sol.cost),'exact_sweep_passed':physical,'kinematic_bad':len(bad)};trials.append(trial)
        if physical and not len(bad):return patch,{'success':True,'trials':trials,'wall_ms':(time.monotonic()-tick)*1000}
    return None,{'success':False,'failure':'NO_VALID_GEOMETRIC_PATCH','trials':trials,'wall_ms':(time.monotonic()-tick)*1000}
