"""3YDL3-V7: ordered Cartesian rectangles and cubic B-spline trust-region SQP.

No call to Smac, prior L3, or geometric repair. Each QP is a sum of positive
weighted least squares plus nonnegative quadratic elastic penalties. Final acceptance
uses polynomial interval bounds and a conservatively enlarged vehicle sweep.
"""
from dataclasses import dataclass
import json, math, time
from pathlib import Path
import cv2
import numpy as np
import osqp
from scipy import sparse
from scipy.interpolate import BSpline
from scipy.sparse.linalg import spsolve

ROOT=Path(__file__).resolve().parents[2]
CONFIG=json.loads((ROOT/'config/l3.json').read_text())

class CurveFailure(RuntimeError):pass

def check_deadline(deadline):
    if time.monotonic()>=deadline:raise CurveFailure('CURVE_BUDGET')

def clean_guide(points,start,goal,epsilon=.22):
    p=np.asarray(points,float).reshape(-1,2)
    p=np.vstack((np.asarray(start)[:2],p,np.asarray(goal)[:2]))
    p=p[np.r_[True,np.linalg.norm(np.diff(p,axis=0),axis=1)>1e-5]]
    if len(p)<2:raise CurveFailure('CURVE_EMPTY_GUIDE')
    # Removes grid staircases, not selected road stages; final original traffic
    # audit and ordered cell constraints remain responsible for correctness.
    p=cv2.approxPolyDP(p.astype(np.float32),epsilon,False).reshape(-1,2).astype(float)
    p[0]=start[:2];p[-1]=goal[:2]
    s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
    if s[-1]<.01:raise CurveFailure('CURVE_DEGENERATE_TASK')
    return p,s

def interp(p,s,u):return np.column_stack([np.interp(u,s,p[:,i]) for i in range(2)])

def feasible_guide(points,start,goal,world,mask,*,hard=None,epsilon_cap=None):
    hard=world.hard|~mask if hard is None else hard
    for epsilon in (.22,.12,.06,0.):
        if epsilon_cap is not None and epsilon>epsilon_cap:continue
        p,s=clean_guide(points,start,goal,epsilon)
        dense=interp(p,s,np.r_[np.arange(0,s[-1],.02),s[-1]])
        col=np.floor((dense[:,0]-world.map.origin[0])/world.map.resolution).astype(int)
        row=world.hard.shape[0]-1-np.floor((dense[:,1]-world.map.origin[1])/world.map.resolution).astype(int)
        valid=(row>=0)&(row<mask.shape[0])&(col>=0)&(col<mask.shape[1])
        if valid.all() and not np.any(hard[row,col]):return p,s,epsilon
    # The guide is not a required vehicle path. Repair only small raster
    # corner excursions inside the SAME ordered local neighbourhood.
    p,s=clean_guide(points,start,goal,0.)
    u=np.r_[np.arange(0,s[-1],.05),s[-1]];dense=interp(p,s,u)
    res=world.map.resolution;origin=world.map.origin;h,w=hard.shape
    for i,xy in enumerate(dense):
        col=int(np.floor((xy[0]-origin[0])/res));row=h-1-int(np.floor((xy[1]-origin[1])/res))
        if 0<=row<h and 0<=col<w and not hard[row,col]:continue
        rad=int(np.ceil(.3/res));r0=max(0,row-rad);r1=min(h,row+rad+1);c0=max(0,col-rad);c1=min(w,col+rad+1)
        yy,xx=np.nonzero(~hard[r0:r1,c0:c1]);choices=np.c_[origin[0]+(xx+c0+.5)*res,origin[1]+(h-yy-r0-.5)*res]
        if not len(choices):raise CurveFailure('CURVE_RAW_GUIDE_OUTSIDE_SCOPE')
        dist=np.linalg.norm(choices-xy,axis=1);nearest=int(np.argmin(dist))
        if dist[nearest]>.3 or i in (0,len(dense)-1):raise CurveFailure('CURVE_ENDPOINT_OR_GUIDE_OUTSIDE_SCOPE')
        dense[i]=choices[nearest]
    p,s=clean_guide(dense,start,goal,.015)
    return p,s,'local_raster_projection_le_0.3m'


def endpoint_seeds(p,s,start,goal,world,mask,deadline,*,hard=None,threshold_override=None,radius_min=0.):
    # Independent analytic INITIALIZATION only. The delivered solution is
    # always reoptimized as a cubic spline; no Dubins curve bypasses its audit.
    from arena_evaluation.semantic_constraint_core import dubins_choices,dubins_edge_from_parameters
    from arena_3d_v1.semantic_world import SweptChecker,densify
    from .v10_storage import collision_checker
    checker=collision_checker(world.hard|~mask if hard is None else hard,world)
    logs=[]
    for endpoint in ('start','goal'):
        pose=start if endpoint=='start' else goal
        direction=(p[1]-p[0]) if endpoint=='start' else (p[-1]-p[-2])
        diff=(math.atan2(direction[1],direction[0])-pose[2]+math.pi)%(2*math.pi)-math.pi
        final_delta=p[-1]-p[-2]
        final_mismatch=abs((math.atan2(final_delta[1],final_delta[0])-goal[2]+math.pi)%(2*math.pi)-math.pi)
        threshold=25 if endpoint=='start' and final_mismatch>math.radians(75) else 75
        if threshold_override is not None:threshold=float(threshold_override)
        if abs(diff)<math.radians(threshold):continue
        best=None;tested=0
        # Existing seed choices retain priority and therefore preserve successful
        # outputs. A failed coarse-anchor search may try a tight terminal loop.
        for distance in (1.,2.,3.,4.,5.,.25,.5,.75):
            if distance<1. and best is not None:continue
            if distance>s[-1]*.4:continue
            at=distance if endpoint=='start' else s[-1]-distance
            anchor=interp(p,s,[at])[0]
            before,after=interp(p,s,[max(0,at-.1),min(s[-1],at+.1)])
            yaw=math.atan2(*(after-before)[::-1]);anchor=tuple(anchor)+(yaw,)
            a,b=(tuple(pose),anchor) if endpoint=='start' else (anchor,tuple(pose))
            for radius in (.405,.42,.45,.5,.65,.8):
                if radius<radius_min:continue
                for word,parameters in dubins_choices(a,b,radius):
                    check_deadline(deadline);length=radius*sum(parameters)
                    if length>distance+5. or (best is not None and length-distance>=best[0]):continue
                    edge=dubins_edge_from_parameters(a,b,radius,word,parameters)
                    poses=densify(np.vstack((a,edge.samples)))
                    tested+=1
                    if checker.check(poses):best=(length-distance,at,poses,word,radius)
        log={'endpoint':endpoint,'tested':tested,'selected':best is not None};logs.append(log)
        if best is None:continue
        _,at,poses,word,radius=best
        anchor_distance=float(at if endpoint=='start' else s[-1]-at)
        # Retain the original guide's order outside this bounded endpoint seed.
        p=np.vstack((poses[:,:2],p[s>at])) if endpoint=='start' else np.vstack((p[s<at],poses[:,:2]))
        p=p[np.r_[True,np.linalg.norm(np.diff(p,axis=0),axis=1)>1e-7]]
        s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
        log.update(word=word,radius_m=radius,anchor_distance_m=anchor_distance)
    return p,s,logs

def knot_vector(p,s,config,refinement=0):
    if config.get('r8_pose_curve'):
        return np.r_[[0.]*3,np.linspace(0,s[-1],int(np.ceil(s[-1]/.08))+1),[s[-1]]*3]
    step=config['straight_knot_m']/(1+refinement*.5)
    length=np.linalg.norm(np.diff(p,axis=0),axis=1);d=np.diff(p,axis=0)/length[:,None]
    angles=np.arccos(np.clip(np.sum(d[1:]*d[:-1],axis=1),-1,1))
    curvature=angles/np.maximum((length[:-1]+length[1:])/2,.01)
    ranges=[(0.,min(1.5,s[-1]),config['turn_knot_m']),(max(0,s[-1]-1.5),s[-1],config['turn_knot_m'])]
    for at,angle,k in zip(s[1:-1],angles,curvature):
        if angle>np.deg2rad(28) or k>.4:
            ranges.append((max(0,at-.9),min(s[-1],at+.9),min(config['turn_knot_m'],.15 if k>1.5 else .3)))
    merged=[]
    for lo,hi,spacing in sorted(ranges):
        if merged and lo<=merged[-1][1]:merged[-1]=(merged[-1][0],max(hi,merged[-1][1]),min(spacing,merged[-1][2]))
        else:merged.append((lo,hi,spacing))
    knots=[v for v in np.arange(0,s[-1],step) if not any(lo<=v<=hi for lo,hi,_ in merged)]+[0.,s[-1]]
    for lo,hi,spacing in merged:knots.extend(np.linspace(lo,hi,int(np.ceil((hi-lo)*(1+refinement)/spacing))+1))
    knots=np.unique(knots);kept=[knots[0]]
    for v in knots[1:]:
        if v-kept[-1]>=.07/(1+refinement):kept.append(v)
    if s[-1]-kept[-1]<.07/(1+refinement):kept[-1]=s[-1]
    else:kept.append(s[-1])
    return np.r_[[0.]*3,kept,[s[-1]]*3]


def basis(knots,u,order=0):
    # A cubic span touches four consecutive basis functions. Four cyclic
    # coefficient channels therefore recover the same local basis (including
    # derivatives) without evaluating an n-column dense identity matrix.
    n=len(knots)-4;u=np.asarray(u)
    span=np.clip(np.searchsorted(knots,u,side='right')-1,3,n-1)
    columns=span[:,None]+np.arange(-3,1)
    values=BSpline(knots,np.eye(4)[np.arange(n)%4],3)(u,nu=order)
    data=np.take_along_axis(values,columns%4,axis=1)
    out=sparse.csc_matrix((data.ravel(),(np.repeat(np.arange(len(u)),4),columns.ravel())),shape=(len(u),n))
    out.eliminate_zeros()
    return out

def project_centers(points,hard,res,origin):
    p=np.asarray(points,float).copy();h,w=hard.shape;count=0
    for i,xy in enumerate(p):
        c=int(np.floor((xy[0]-origin[0])/res));r=h-1-int(np.floor((xy[1]-origin[1])/res))
        if 0<=r<h and 0<=c<w and not hard[r,c]:continue
        rad=int(np.ceil(.3/res));r0=max(0,r-rad);r1=min(h,r+rad+1);c0=max(0,c-rad);c1=min(w,c+rad+1)
        yy,xx=np.nonzero(~hard[r0:r1,c0:c1]);choices=np.c_[origin[0]+(xx+c0+.5)*res,origin[1]+(h-yy-r0-.5)*res]
        if not len(choices):raise CurveFailure('CURVE_CORRIDOR_NO_LOCAL_FREE_CELL')
        dist=np.linalg.norm(choices-xy,axis=1);idx=int(np.argmin(dist))
        if dist[idx]>.3:raise CurveFailure('CURVE_CORRIDOR_LOCAL_PROJECTION_LIMIT')
        p[i]=choices[idx];count+=1
    return p,count

def rectangles(hard,res,origin,guide,deadline,tangents=None):
    """Ordered oriented convex rectangles, excluding every intersecting cell.

    Rectangle/cell intersection uses all four separating axes (two world
    axes and two rectangle axes); map exterior is always an obstacle.
    """
    h,w=hard.shape;bounds=[];axes=[]
    if tangents is None:tangents=np.tile([1.,0.],(len(guide),1))
    for i,(xy,tangent) in enumerate(zip(guide,tangents)):
        if i%32==0:check_deadline(deadline)
        norm=np.linalg.norm(tangent)
        if norm<1e-10:tangent=np.array([1.,0.]);norm=1.
        t=tangent/norm;n=np.array([-t[1],t[0]])
        c=int(np.floor((xy[0]-origin[0])/res));r=h-1-int(np.floor((xy[1]-origin[1])/res))
        if not (0<=r<h and 0<=c<w) or hard[r,c]:raise CurveFailure('CURVE_GUIDE_OUTSIDE_SCOPE')
        rad=int(np.ceil(3.0/res));r0=max(0,r-rad);r1=min(h,r+rad+1);c0=max(0,c-rad);c1=min(w,c+rad+1)
        yy,xx=np.nonzero(hard[r0:r1,c0:c1]);obs=np.c_[origin[0]+(xx+c0+.5)*res,origin[1]+(h-yy-r0-.5)*res]-xy
        ot=obs@t;on=obs@n;cell=res/2;projected=cell*(abs(t[0])+abs(t[1]))
        def clear(ext):
            mid=((ext[1]-ext[0])*t+(ext[3]-ext[2])*n)/2
            half_t=(ext[0]+ext[1])/2;half_n=(ext[2]+ext[3])/2
            wx=half_t*abs(t[0])+half_n*abs(n[0]);wy=half_t*abs(t[1])+half_n*abs(n[1])
            center=xy+mid
            if center[0]-wx<origin[0] or center[0]+wx>origin[0]+w*res or center[1]-wy<origin[1] or center[1]+wy>origin[1]+h*res:return False
            overlap=(abs(obs[:,0]-mid[0])<=wx+cell)&(abs(obs[:,1]-mid[1])<=wy+cell)&(ot>=-ext[0]-projected)&(ot<=ext[1]+projected)&(on>=-ext[2]-projected)&(on<=ext[3]+projected)
            return not np.any(overlap)
        ext=np.array([.285,.285,.245,.245])
        if not clear(ext):
            found=False;original_xy=xy.copy();original_obs=obs.copy()
            offsets=np.array([(a,b) for a in np.arange(-.45,.451,.075) for b in np.arange(-.45,.451,.075)])
            offsets=offsets[np.argsort(np.linalg.norm(offsets,axis=1))]
            for offset in offsets:
                xy=original_xy+offset;obs=original_obs-offset;ot=obs@t;on=obs@n
                if clear(ext):found=True;break
            if not found:
                xy=original_xy;obs=original_obs;ot=obs@t;on=obs@n;ext=np.array([.001]*4)
        # Grow normal before tangent: body-aligned cells in a corner should
        # not become a long sliver that excludes a physically feasible body.
        for step in (.1,.025):
            for _ in range(int(1.4/step)+1):
                changed=False
                for k in (2,3,0,1):
                    test=ext.copy();test[k]+=step
                    if test[k]<=1.401 and clear(test):ext=test;changed=True
                if not changed:break
        bounds.append([xy@t-ext[0],xy@t+ext[1],xy@n-ext[2],xy@n+ext[3]])
        axes.append([*t,*n])
    return np.asarray(bounds),np.asarray(axes)

def differential(B,D,E,F,z):
    n=z.size//2;cp=np.column_stack((z[:n],z[n:]));p=B@cp;v=D@cp;a=E@cp;j=F@cp
    speed=np.linalg.norm(v,axis=1);safe=np.maximum(speed,1e-6)
    cross=v[:,0]*a[:,1]-v[:,1]*a[:,0]
    k=cross/safe**3
    jx=sparse.diags(a[:,1]/safe**3-3*cross*v[:,0]/safe**5)@D-sparse.diags(v[:,1]/safe**3)@E
    jy=sparse.diags(-a[:,0]/safe**3-3*cross*v[:,1]/safe**5)@D+sparse.diags(v[:,0]/safe**3)@E
    J=sparse.hstack((jx,jy),format='csc')
    Y=sparse.hstack((-sparse.diags(v[:,1]/safe**2)@D,sparse.diags(v[:,0]/safe**2)@D),format='csc')
    return p,v,a,j,speed,k,J,Y

def footprint_constraints(B,Y,v,half_length,half_width,axes):
    yaw=np.arctan2(v[:,1],v[:,0])-np.arctan2(axes[:,1],axes[:,0]);co=np.cos(yaw);si=np.sin(yaw)
    hx=half_length*np.abs(co)+half_width*np.abs(si)
    hy=half_length*np.abs(si)+half_width*np.abs(co)
    dx=-half_length*np.sign(co)*si+half_width*np.sign(si)*co
    dy=half_length*np.sign(si)*co-half_width*np.sign(co)*si
    X=sparse.hstack((sparse.diags(axes[:,0])@B,sparse.diags(axes[:,1])@B));V=sparse.hstack((sparse.diags(axes[:,2])@B,sparse.diags(axes[:,3])@B))
    return [X+sparse.diags(dx)@Y,-X+sparse.diags(dx)@Y,
            V+sparse.diags(dy)@Y,-V+sparse.diags(dy)@Y],hx,hy

def violation(p,v,speed,k,boxes,vehicle,axes,curvature_ratio=.98):
    p=np.c_[np.sum(p*axes[:,:2],axis=1),np.sum(p*axes[:,2:],axis=1)]
    yaw=np.arctan2(v[:,1],v[:,0])-np.arctan2(axes[:,1],axes[:,0]);hx=vehicle[0]*abs(np.cos(yaw))+vehicle[1]*abs(np.sin(yaw))
    hy=vehicle[0]*abs(np.sin(yaw))+vehicle[1]*abs(np.cos(yaw))
    values=np.column_stack((p[:,0]+hx-boxes[:,1],boxes[:,0]-(p[:,0]-hx),
                            p[:,1]+hy-boxes[:,3],boxes[:,2]-(p[:,1]-hy),
                            (abs(k)-curvature_ratio/vehicle[2])*.1, (.15-speed)*.1))
    return float(np.maximum(values,0).sum()),float(np.maximum(values,0).max())

def bezier_pieces(spline):
    knots=np.unique(spline.t)
    # Each BSpline evaluation is independent at its parameter. Batch the same
    # endpoints once; retain the original per-piece arithmetic and ordering.
    batched=type(spline) is BSpline
    if batched:
        positions=spline(knots);velocities=spline(knots,nu=1)
    for i,(a,b) in enumerate(zip(knots,knots[1:])):
        if b<=a:continue
        if batched:p=positions[i:i+2];v=velocities[i:i+2]
        else:p=spline(np.array([a,b]));v=spline(np.array([a,b]),nu=1)
        h=b-a
        yield a,b,np.vstack((p[0],p[0]+v[0]*h/3,p[1]-v[1]*h/3,p[1]))

def split_bezier(c):
    a=(c[:-1]+c[1:])/2;b=(a[:-1]+a[1:])/2;m=(b[0]+b[1])/2
    return np.vstack((c[0],a[0],b[0],m)),np.vstack((m,b[1],a[2],c[3]))

def certify(spline,world,mask,deadline,config=CONFIG,corridor=None,parameters=None,*,hard=None):
    """Bernstein convex-hull bounds, no unchecked gaps between samples.

    For each cubic interval: derivatives lie in their Bernstein hulls. A
    projection lower-bounds speed. Pairwise derivative/acceleration crosses
    bound the curvature numerator. Position/yaw interpolation errors obey
    sup|f''| h²/8. Inflate the original rectangle sweep by the summed bound.
    """
    started=time.monotonic();breaks=np.unique(spline.t);left=[];right=[]
    if parameters is None:
        for a,b,c in bezier_pieces(spline):
            vmax=np.linalg.norm(3*np.diff(c,axis=0)/(b-a),axis=1).max()
            edges=np.linspace(a,b,max(1,int(np.ceil(vmax*(b-a)/config['max_output_step_m'])))+1)
            left.extend(edges[:-1]);right.extend(edges[1:])
    left=np.asarray(left);right=np.asarray(right)
    if parameters is not None:
        params=np.asarray(parameters,float)
        if len(params)<2 or not np.isfinite(params).all() or np.any(np.diff(params)<=0) or params[0]<spline.t[3] or params[-1]>spline.t[-4]:raise CurveFailure('CURVE_PARAMETER_BINDING')
        internal=breaks[(breaks>params[0])&(breaks<params[-1])]
        if len(internal) and not np.all(np.isclose(params[np.searchsorted(params,internal)],internal,atol=1e-12,rtol=0)):
            raise CurveFailure('CURVE_PARAMETER_CROSSES_POLYNOMIAL_PIECE')
        left=params[:-1];right=params[1:]
    from arena_3d_v1.semantic_world import SweptChecker
    from .v10_storage import collision_checker
    checker=collision_checker(world.hard|~mask if hard is None else hard,world)
    body_refinements=0;body_groups=0
    if corridor is not None:
        # This call's corridor is fixed. Refinement changes intervals only;
        # retain its exact numerical arrays throughout all refinement depths.
        stations=np.asarray(corridor['stations']);boxes0=np.asarray(corridor['bounds']);axes0=np.asarray(corridor['axes'])
    for depth in range(18):
        check_deadline(deadline);h=right-left
        ends0=spline(left);ends1=spline(right);v0=spline(left,nu=1);v1=spline(right,nu=1)
        v=np.stack((v0,3*(ends1-ends0)/h[:,None]-v0-v1,v1),axis=1)
        acc=2*np.diff(v,axis=1)/h[:,None,None];jerk=(acc[:,1]-acc[:,0])/h[:,None]
        vm=(v[:,0]+2*v[:,1]+v[:,2])/4;vmag=np.linalg.norm(vm,axis=1)
        minimum=np.min(np.sum(v*vm[:,None,:],axis=2)/np.maximum(vmag[:,None],1e-15),axis=1)
        vmax=np.max(np.linalg.norm(v,axis=2),axis=1);amax=np.max(np.linalg.norm(acc,axis=2),axis=1)
        cross=np.max(np.abs(v[:,:,None,0]*acc[:,None,:,1]-v[:,:,None,1]*acc[:,None,:,0]),axis=(1,2))
        curvature=cross/np.maximum(minimum,1e-15)**3
        radius=math.hypot(*world.vehicle[:2]);poserr=amax*h*h/8
        yaw2=vmax*np.linalg.norm(jerk,axis=1)/np.maximum(minimum,1e-15)**2+2*vmax*amax*cross/np.maximum(minimum,1e-15)**4
        error=poserr+radius*yaw2*h*h/8
        bad=(minimum<=1e-6)|(curvature>1/world.vehicle[2]+1e-8)|(error>config['max_curve_chord_error_m'])|(vmax*h>config['max_output_step_m']*1.000001)
        if corridor is not None:
            index=np.clip(np.searchsorted(stations,(left+right)/2),0,len(stations)-1)
            ctrl=np.stack((ends0,ends0+v0*h[:,None]/3,ends1-v1*h[:,None]/3,ends1),axis=1)
            covered=np.zeros(len(left),bool)
            for shift in (-1,0,1):
                idx=np.clip(index+shift,0,len(stations)-1);aa=axes0[idx];bb=boxes0[idx]
                xx=np.sum(ctrl*aa[:,None,:2],axis=2);yy=np.sum(ctrl*aa[:,None,2:],axis=2)
                covered|=(xx.min(axis=1)>=bb[:,0]-1e-9)&(xx.max(axis=1)<=bb[:,1]+1e-9)&(yy.min(axis=1)>=bb[:,2]-1e-9)&(yy.max(axis=1)<=bb[:,3]+1e-9)
            if np.any(~covered & (h<config['certificate_min_interval'])):raise CurveFailure('CURVE_ORDERED_CORRIDOR')
            bad|=~covered
        if not np.any(bad):
            # Each polynomial piece has its own proven interpolation error.
            # A bend must not enlarge the vehicle on every distant straight.
            order=np.argsort(left);u=np.r_[left[order],right[order][-1]]
            xy=spline(u);vv=spline(u,nu=1)
            path=np.column_stack((xy,np.arctan2(vv[:,1],vv[:,0])))
            piece=np.searchsorted(breaks,left[order],side='right')-1
            cuts=np.r_[0,np.flatnonzero(np.diff(piece))+1,len(order)]
            body_groups=len(cuts)-1
            for lo,hi in zip(cuts[:-1],cuts[1:]):
                check_deadline(deadline)
                enlargement=float(error[order[lo:hi]].max())+1e-9
                checker.half_length=world.vehicle[0]+enlargement
                checker.half_width=world.vehicle[1]+enlargement
                if not checker.check(path[lo:hi+1]):bad[order[lo:hi]]=True
            if not np.any(bad):break
            body_refinements+=1
            if body_refinements>config.get('body_refinement_passes',0):
                error=CurveFailure('CURVE_CONTINUOUS_BODY_OR_SCOPE')
                error.intervals=(left[bad].copy(),right[bad].copy())
                raise error
        # A direct actual-curve violation cannot be fixed by finer bounds.
        mids=(left[bad]+right[bad])/2;mv=spline(mids,nu=1);ma=spline(mids,nu=2)
        mk=np.abs(mv[:,0]*ma[:,1]-mv[:,1]*ma[:,0])/np.maximum(np.linalg.norm(mv,axis=1),1e-15)**3
        if np.any(mk>1/world.vehicle[2]+1e-7) or np.any(h[bad]<config['certificate_min_interval']):
            raise CurveFailure('CURVE_INTERVAL_CURVATURE_OR_REGULARITY')
        left=np.r_[left[~bad],left[bad],mids];right=np.r_[right[~bad],mids,right[bad]]
    else:raise CurveFailure('CURVE_INTERVAL_REFINEMENT_LIMIT')
    order=np.argsort(left);left=left[order];right=right[order]
    maxk=float(curvature.max());maxerr=float(error.max());u=np.r_[left,right[-1]]
    xy=spline(u);v=spline(u,nu=1);path=np.column_stack((xy,np.arctan2(v[:,1],v[:,0])))
    return path,{'intervals':len(left),'max_curvature_bound_1pm':maxk,'body_enlargement_m':maxerr+1e-9,
                 'bound_method':'Bernstein derivative hulls + second-derivative interpolation error + per-polynomial-piece enlarged native rectangle sweep',
                 'body_sweep_groups':body_groups,'body_refinement_passes':body_refinements,
                 'curve_output_position_error_m':0.,'curve_output_yaw_error_rad':0.,'sample_parameters':u.tolist(),
                 'certificate_ms':(time.monotonic()-started)*1000}

def _optimize_mesh(world,mask,guide,start,goal,deadline,config=CONFIG):
    log={'algorithm':'ordered-convex-corridor-cubic-bspline-trust-region-SQP','old_l3_calls':0,'qp_solver':config.get('qp_solver','osqp'),
         'corridor_ms':0.,'initialization_ms':0.,'assembly_ms':0.,'setup_ms':0.,'optimization_ms':0.,
         'certificate_ms':0.,'iterations':[],'refinements':[],'success':False,'accepted_quadratic_initialization':False}
    started=time.monotonic();best_curve=None
    try:
        check_deadline(deadline)
        hard=world.hard|~mask
        # This alternate optimization uses center cells; physical certification
        # below always retains world.vehicle and the original occupied map.
        opt_vehicle=(0.,0.,world.vehicle[2]) if config.get('r8_pose_curve') else world.vehicle
        original_length=None
        if config.get('r4_clearance_guide',False):
            _,original_arc,_=feasible_guide(guide,start,goal,world,mask,hard=hard)
            original_length=float(original_arc[-1])
            from .tail_clearance import clearance_guide
            guide,log['r4_clearance_guide']=clearance_guide(guide,start,goal,world,hard,deadline,
                smoothing_m=config.get('r4_clearance_smoothing_m',0.),stringpull=config.get('r4_clearance_stringpull',False))
        tick=time.monotonic();p,s,epsilon=feasible_guide(guide,start,goal,world,mask,hard=hard,epsilon_cap=config.get('r4_guide_epsilon_cap'))
        if original_length is None:original_length=float(s[-1])
        if config.get('r4_rounded_guide',False):
            from .tail_guide import rounded_guide
            p,s,log['rounded_guide']=rounded_guide(p,s,world,hard,deadline)
        if config.get('r8_coupled_bends',False):
            from .failure_seed import coupled_bends
            p,s,log['r8_coupled_bends']=coupled_bends(p,s,world,hard,deadline)
        if config.get('r4_lattice_endpoint_seed',False):
            from .tail_lattice import seed
            p,s,log['r4_lattice_endpoints']=seed(p,s,start,goal,world,hard,deadline)
        p,s,seedlog=endpoint_seeds(p,s,start,goal,world,mask,deadline,hard=hard,threshold_override=config.get('r4_endpoint_threshold_deg'),radius_min=config.get('r4_seed_min_radius_m',0.))
        if config.get('r4_lateral_endpoint_seed',False):
            from .tail_endpoint import seed_failed_endpoints
            p,s,seedlog=seed_failed_endpoints(p,s,start,goal,world,mask,deadline,seedlog,hard=hard)
        log.update(simplification_epsilon_m=epsilon,endpoint_seeds=seedlog,original_guide_length_m=original_length)
        if (config.get('r11_primary_slice_s') and config.get('r4_defer_recovery')
                and not config.get('r4_rounded_guide') and len(seedlog)>=2
                and all(item.get('selected') for item in seedlog)):
            sliced=min(deadline,started+float(config['r11_primary_slice_s']))
            if sliced<deadline:
                deadline=sliced;log['r11_primary_slice_s']=float(config['r11_primary_slice_s'])
        if config.get('r4_adaptive_stations'):
            from .tail_stations import stations
            u=stations(p,s,config['constraint_step_m'])
        elif config.get('adaptive_constraint_stations'):
            # Geometry knots already become dense around curved seed sections.
            # Collocate at their boundaries and midpoints plus a bounded grid,
            # rather than introducing six slacks at every seed sample.
            events=np.unique(knot_vector(p,s,config,0))
            u=np.unique(np.r_[np.arange(0,s[-1],config['constraint_step_m']),events,(events[:-1]+events[1:])/2,s[-1]])
            # Near-identical floating point stations corrupt nonuniform
            # finite differences and create artificial progress constraints.
            u=u[np.r_[True,np.diff(u)>1e-6]]
            if s[-1]-u[-1]<1e-6:u[-1]=s[-1]
            else:u=np.r_[u,s[-1]]
        else:u=np.unique(np.r_[np.arange(0,s[-1],config['constraint_step_m']),s,s[-1]])
        targets=interp(p,s,u);tangents=np.gradient(targets,u,axis=0)
        targets,projected=project_centers(targets,hard,world.map.resolution,world.map.origin)
        log['projected_corridor_centers']=projected
        corridor_u=u;corridor_targets=targets;corridor_tangents=tangents
        if config.get('r4_adaptive_stations'):
            # Preserve the original ordered convex cells and dense certificate
            # corridor. Only the optimization collocation stations are thinner.
            corridor_u=np.unique(np.r_[np.arange(0,s[-1],config['constraint_step_m']),s,s[-1]])
            raw=interp(p,s,corridor_u);corridor_tangents=np.gradient(raw,corridor_u,axis=0)
            corridor_targets,_=project_centers(raw,hard,world.map.resolution,world.map.origin)
        anchor_indices=np.unique(np.clip(np.searchsorted(corridor_u,np.r_[np.arange(0,s[-1],.4),np.arange(0,min(5.,s[-1]),.1),np.arange(max(0,s[-1]-5.),s[-1],.1),s[-1]]),0,len(corridor_u)-1))
        anchor_stations=corridor_u[anchor_indices]
        if config.get('r8_pose_curve'):
            from .failure_pose_curve import center_rectangles as build_rectangles
        elif config.get('native_corridor'):
            from .curve_native import rectangles as build_rectangles
        else:build_rectangles=rectangles
        anchor_boxes,anchor_axes=build_rectangles(hard,world.map.resolution,world.map.origin,corridor_targets[anchor_indices],deadline,corridor_tangents[anchor_indices])
        nearest=np.clip(np.searchsorted((anchor_stations[1:]+anchor_stations[:-1])/2,u),0,len(anchor_indices)-1)
        boxes,axes=anchor_boxes[nearest],anchor_axes[nearest]
        log['convex_cells']=len(anchor_indices)
        boxes+=np.array([.02,-.02,.02,-.02])
        if config.get('r4_adaptive_stations'):
            dense_nearest=np.clip(np.searchsorted((anchor_stations[1:]+anchor_stations[:-1])/2,corridor_u),0,len(anchor_indices)-1)
            dense_boxes=anchor_boxes[dense_nearest]+np.array([.02,-.02,.02,-.02])
            corridor={'stations':corridor_u.tolist(),'bounds':dense_boxes.tolist(),'axes':anchor_axes[dense_nearest].tolist()}
            log['r4_solver_stations']=len(u);log['r4_original_certificate_stations']=len(corridor_u)
        else:corridor={'stations':u.tolist(),'bounds':boxes.tolist(),'axes':axes.tolist()}
        log['corridor_ms']=(time.monotonic()-tick)*1000;log['guide_length_m']=float(s[-1]);log['corridor_count']=len(boxes)
        for refinement in range(config['max_refinements']+1):
            check_deadline(deadline);tick=time.monotonic();knots=knot_vector(p,s,config,refinement)
            n=len(knots)-4;B=basis(knots,u);D=basis(knots,u,1);E=basis(knots,u,2);F=basis(knots,u,3)
            H=(config['tracking_weight']*(B.T@B)+config['bending_weight']*(E.T@E)+config['jerk_weight']*(F.T@F)+sparse.eye(n)*1e-7).tocsc()
            q0=-config['tracking_weight']*np.r_[B.T@targets[:,0],B.T@targets[:,1]]
            cp=spsolve((B.T@B+1e-4*(E.T@E)+sparse.eye(n)*1e-8).tocsc(),np.column_stack((B.T@targets[:,0],B.T@targets[:,1])))
            cp[0]=start[:2];cp[-1]=goal[:2]
            cp[1]=cp[0]+(knots[4]-knots[3])/3*np.array([math.cos(start[2]),math.sin(start[2])])
            cp[-2]=cp[-1]-(knots[-4]-knots[-5])/3*np.array([math.cos(goal[2]),math.sin(goal[2])])
            if config.get('r8_pose_curve'):
                fitA=(B.T@B+1e-7*(E.T@E)+sparse.eye(n)*1e-8).tocsc()
                fixed=np.array([0,1,n-2,n-1]);free=np.arange(2,n-2)
                cp[free]=spsolve(fitA[free][:,free],(B.T@targets)[free]-fitA[free][:,fixed]@cp[fixed])
            free_tangent=config.get('free_endpoint_tangent_scale',False)
            z=np.r_[cp[:,0],cp[:,1]];fixed_indices=np.r_[0,n-1,n,2*n-1] if free_tangent else np.r_[0,1,n-2,n-1,n,n+1,2*n-2,2*n-1];fixed_values=z[fixed_indices].copy();trust=config['trust_radius_m'];history=[]
            log['initialization_ms']+=(time.monotonic()-tick)*1000
            # Exact clamped position + derivative (unit positive geometric
            # speed) endpoints. No endpoint yaw metadata substitution.
            endpoint=sparse.vstack((B[0],B[-1],D[0],D[-1]),format='csc')
            Aeq=sparse.block_diag((endpoint,endpoint),format='csc')
            beq=np.array([start[0],goal[0],math.cos(start[2]),math.cos(goal[2]),start[1],goal[1],math.sin(start[2]),math.sin(goal[2])])
            tangent=sparse.csc_matrix((0,2*n));tangent_lower=np.zeros(0);tangent_upper=np.zeros(0)
            if free_tangent:
                positions=sparse.block_diag((B[[0,-1]],B[[0,-1]]),format='csc')
                normal=sparse.vstack([sparse.hstack((-math.sin(theta)*D[j],math.cos(theta)*D[j])) for j,theta in [(0,start[2]),(-1,goal[2])]],format='csc')
                tangent=sparse.vstack([sparse.hstack((math.cos(theta)*D[j],math.sin(theta)*D[j])) for j,theta in [(0,start[2]),(-1,goal[2])]],format='csc')
                Aeq=sparse.vstack((positions,normal),format='csc');beq=np.r_[start[0],goal[0],start[1],goal[1],0.,0.]
                tangent_lower=np.full(2,.15);tangent_upper=np.full(2,3.)
            dtarget=np.gradient(targets,u,axis=0);dtarget/=np.maximum(np.linalg.norm(dtarget,axis=1)[:,None],1e-9)
            # Pose constraints take precedence near endpoints. Do not impose
            # the raster guide's heading on a parking pose with a distinct yaw.
            progress_lower=np.where((u<1.)|(u>s[-1]-1.),-np.inf,.05)
            progress=sparse.hstack((sparse.diags(dtarget[:,0])@D,sparse.diags(dtarget[:,1])@D),format='csc')
            baseH=sparse.block_diag((H,H),format='csc')
            # First solve the SAME quadratic objective with exact fixed endpoint
            # coefficients. It is merely a candidate until the original whole
            # curve certificate and backend's complete traffic/body audit pass.
            if config.get('certified_quadratic_start',False) and refinement==0 and not free_tangent and deadline-time.monotonic()>.5:
                trial_tick=time.monotonic();trial={'selected':False}
                fixed=np.array([0,1,n-2,n-1]);free=np.arange(2,n-2)
                candidate=cp.copy();rhs=config['tracking_weight']*(B.T@targets)
                if len(free):candidate[free]=spsolve(H[free,:][:,free],rhs[free]-H[free,:][:,fixed]@candidate[fixed])
                vv=D@candidate;aa=E@candidate;ss=np.linalg.norm(vv,axis=1)
                kk=(vv[:,0]*aa[:,1]-vv[:,1]*aa[:,0])/np.maximum(ss,1e-6)**3
                trial['max_sample_curvature']=float(np.max(abs(kk)))
                locations=B@candidate
                projected=np.c_[np.sum(locations*axes[:,:2],axis=1),np.sum(locations*axes[:,2:],axis=1)]
                centers_inside=bool(np.all((projected[:,0]>=boxes[:,0])&(projected[:,0]<=boxes[:,1])&(projected[:,1]>=boxes[:,2])&(projected[:,1]<=boxes[:,3])))
                from .v10_storage import collision_checker
                sampled=np.c_[locations,np.arctan2(vv[:,1],vv[:,0])]
                cheap_valid=centers_inside and np.max(abs(kk))<1/world.vehicle[2] and np.min(ss)>.05
                if cheap_valid:cheap_valid=collision_checker(hard,world).check(sampled)
                if cheap_valid:
                    try:
                        trial_curve=BSpline(knots,candidate,3)
                        path,cert=certify(trial_curve,world,mask,min(deadline,trial_tick+.05),config,corridor,hard=hard)
                        length=float(np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1).sum())
                        if length>config['max_length_ratio_to_guide']*original_length:raise CurveFailure('CURVE_LENGTH_GATE')
                        parameters=cert.pop('sample_parameters');trial['selected']=True
                        trial['wall_ms']=(time.monotonic()-trial_tick)*1000
                        log.update(success=True,certificate=cert,control_points=n,length_m=length,refinement=refinement,
                                   accepted_quadratic_initialization=True,quadratic_initialization=trial)
                        log['certificate_ms']+=trial['wall_ms']
                        return path,{'knots':knots.tolist(),'coefficients':candidate.tolist(),'degree':3,'corridor':corridor,'parameters':parameters},log
                    except CurveFailure as error:trial['certificate_failure']=str(error)
                else:trial['certificate_failure']='PRELIMINARY_CENTER_BODY_CURVATURE_OR_REGULARITY'
                trial['wall_ms']=(time.monotonic()-trial_tick)*1000;log['quadratic_initialization']=trial
                if config.get('r4_quadratic_warm_start',False):
                    from .tail_warm import select
                    cp,warm=select(cp,candidate,B,D,E,boxes,world.vehicle,axes,
                                   config.get('curvature_target_ratio',.98))
                    z=np.r_[cp[:,0],cp[:,1]]
                    log['r4_warm_start']=warm
            for iteration in range(config['max_iterations']):
                if (refinement==0 and iteration==1 and config.get('r5_early_seed')
                        and not config.get('r4_rounded_guide')
                        and not (config.get('r5_interior_seed_only') and seedlog)
                        and not getattr(world,'v7_map_only',False)):
                    # Only after the established first QP failed to certify.
                    # A bounded alternate seed shares the aggregate deadline;
                    # failure resumes the original iterate and trust radius.
                    seed_started=time.monotonic()
                    aggregate=config.get('_r5_aggregate_deadline',deadline)
                    if aggregate-seed_started>.05:
                        trial=_optimize_mesh(world,mask,guide,start,goal,
                            min(aggregate,seed_started+.75),
                            dict(config,r5_early_seed=False,r4_rounded_guide=True,
                                 adaptive_constraint_stations=True,
                                 r4_adaptive_stations=False,r4_tighter_primary=False))
                        seed_elapsed=time.monotonic()-seed_started
                        deadline=min(aggregate,deadline+seed_elapsed)
                        if trial[0] is not None:
                            trial[2]['r5_early_seed']={'after_original_qp':1,
                                'original_iterations':list(log['iterations']),
                                'seed_wall_ms':trial[2]['wall_ms']}
                            trial[2]['wall_ms']=(time.monotonic()-started)*1000
                            return trial
                        log['r5_early_seed_failure']=trial[2]
                check_deadline(deadline);tick=time.monotonic()
                pos,v,acc,j,speed,k,J,Y=differential(B,D,E,F,z)
                G,hx,hy=footprint_constraints(B,Y,v,*opt_vehicle[:2],axes)
                localpos=np.c_[np.sum(pos*axes[:,:2],axis=1),np.sum(pos*axes[:,2:],axis=1)]
                g=np.r_[localpos[:,0]+hx,-localpos[:,0]+hx,localpos[:,1]+hy,-localpos[:,1]+hy,k,-k]
                curvature_ratio=config.get('curvature_target_ratio',.98)
                if config.get('r4_tighter_primary'):curvature_ratio=min(curvature_ratio,.97)
                limit=np.r_[boxes[:,1],-boxes[:,0],boxes[:,3],-boxes[:,2],np.full(len(u)*2,curvature_ratio/world.vehicle[2])]
                G=sparse.vstack((*G,J,-J),format='csc');m=G.shape[0]
                elastic=np.flatnonzero(g-limit>=-config.get('elastic_activation_margin',.02)) if config.get('active_elastic_variables') else np.arange(m)
                ns=len(elastic)
                S=sparse.csc_matrix((np.ones(ns),(elastic,np.arange(ns))),shape=(m,ns))
                original_m=m
                linear_upper=limit-g
                if config.get('trust_region_constraint_screen',False):
                    # |delta_i| <= trust and slack >= 0 imply G delta - S slack
                    # <= trust * ||G_row||_1. Only proven redundant rows leave
                    # this QP; the full nonlinear/continuous audits remain.
                    reach=trust*np.asarray(abs(G).sum(axis=1)).ravel()
                    keep=linear_upper<reach+1e-10*(1+abs(linear_upper)+reach)
                    G=G[keep].tocsc();S=S[keep].tocsc();linear_upper=linear_upper[keep];m=int(keep.sum())
                # Elastic constraints aid infeasible initial guesses. Slack is
                # never an acceptance exemption: exact certification follows.
                P=sparse.block_diag((baseH,sparse.eye(ns)*config['slack_penalty']*min(100.,2.**iteration)),format='csc')*2
                q=np.r_[2*(q0+baseH@z),np.zeros(ns)]
                Z=sparse.csc_matrix((Aeq.shape[0],ns));I=sparse.eye(2*n,format='csc')
                A=sparse.vstack((sparse.hstack((Aeq,Z)),sparse.hstack((G,-S)),
                    sparse.hstack((I,sparse.csc_matrix((2*n,ns)))),
                    sparse.hstack((sparse.csc_matrix((ns,2*n)),sparse.eye(ns))),
                    sparse.hstack((progress,sparse.csc_matrix((len(u),ns)))),
                    sparse.hstack((tangent,sparse.csc_matrix((tangent.shape[0],ns))))),format='csc')
                lo=np.r_[beq-Aeq@z,np.full(m,-np.inf),np.full(2*n,-trust),np.zeros(ns),progress_lower-progress@z,tangent_lower-tangent@z]
                hi=np.r_[beq-Aeq@z,linear_upper,np.full(2*n,trust),np.full(ns,np.inf),np.full(len(u),np.inf),tangent_upper-tangent@z]
                log['assembly_ms']+=(time.monotonic()-tick)*1000;check_deadline(deadline)
                tick=time.monotonic();qp_solver=config.get('qp_solver','osqp')
                if qp_solver=='clarabel':
                    from .curve_qp import clarabel_setup,clarabel_solve
                    solver=clarabel_setup(P,q,A,lo,hi,deadline,config['qp_eps'])
                else:
                    solver=osqp.OSQP();solver.setup(P=sparse.triu(P,format='csc'),q=q,A=A,l=lo,u=hi,
                        verbose=False,eps_abs=config['qp_eps'],eps_rel=config['qp_eps'],max_iter=config['qp_max_iter'],
                        polish=True,adaptive_rho_interval=50,time_limit=max(.001,deadline-time.monotonic()))
                    solver.warm_start(x=np.r_[np.zeros_like(z),np.maximum(g-limit,0)[elastic]])
                log['setup_ms']+=(time.monotonic()-tick)*1000;tick=time.monotonic();res=clarabel_solve(solver) if qp_solver=='clarabel' else solver.solve()
                log['optimization_ms']+=(time.monotonic()-tick)*1000
                detail={'refinement':refinement,'iteration':iteration,'status':res.info.status,'qp_solver':qp_solver,'qp_iterations':int(res.info.iter),'control_points':n,'trust_m':trust,'elastic_variables':ns,'geometric_constraint_rows':m,'original_geometric_constraint_rows':original_m,'redundant_rows_removed':original_m-m}
                log['iterations'].append(detail)
                if res.info.status_val not in (1,2):
                    detail['accepted']=False;trust*=.5
                    if trust<.01:break
                    continue
                proposal=z+res.x[:2*n];proposal[fixed_indices]=fixed_values
                if free_tangent:
                    # Enforce the solved linear tangent alignment exactly;
                    # the actual spline derivative supplies delivered yaw.
                    for a,b,theta in [(0,1,start[2]),(n-1,n-2,goal[2])]:
                        unit=np.array([math.cos(theta),math.sin(theta)])
                        base=np.array([proposal[a],proposal[n+a]])
                        delta=np.array([proposal[b],proposal[n+b]])-base
                        point=base+unit*np.dot(delta,unit)
                        proposal[b]=point[0];proposal[n+b]=point[1]
                new=differential(B,D,E,F,proposal)
                olderr,oldmax=violation(pos,v,speed,k,boxes,opt_vehicle,axes,curvature_ratio);err,errmax=violation(new[0],new[1],new[4],new[5],boxes,opt_vehicle,axes,curvature_ratio)
                oldobj=float(z@(baseH@z)+2*q0@z);obj=float(proposal@(baseH@proposal)+2*q0@proposal)
                accepted=(err<olderr-1e-7 or (err<=1e-5 and obj<=oldobj+1e-5) or (err<=olderr+.001 and obj<oldobj))
                detail.update(violation=err,max_violation=errmax,accepted=bool(accepted),max_sample_curvature=float(abs(new[5]).max()))
                if accepted:
                    movement=float(abs(proposal-z).max());z=proposal;trust=min(config['trust_radius_m'],trust*1.25)
                    best_curve=BSpline(knots,np.column_stack((z[:n],z[n:])),3)
                    if np.max(abs(new[5]))<1/world.vehicle[2] and np.min(new[4])>.05:
                        tick=time.monotonic()
                        try:
                            path,cert=certify(best_curve,world,mask,deadline,config,corridor,hard=hard)
                            length=float(np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1).sum())
                            if length>config['max_length_ratio_to_guide']*original_length:raise CurveFailure('CURVE_LENGTH_GATE')
                            parameters=cert.pop('sample_parameters')
                            log.update(success=True,certificate=cert,control_points=n,length_m=length,refinement=refinement)
                            log['certificate_ms']+=(time.monotonic()-tick)*1000
                            return path,{'knots':knots.tolist(),'coefficients':best_curve.c.tolist(),'degree':3,'corridor':corridor,'parameters':parameters},log
                        except CurveFailure as error:
                            log['certificate_ms']+=(time.monotonic()-tick)*1000;detail['certificate_failure']=str(error)
                            if str(error)=='CURVE_BUDGET':raise
                            if config.get('r8_pose_curve') and str(error)=='CURVE_CONTINUOUS_BODY_OR_SCOPE':
                                log.update(failure=str(error),wall_ms=(time.monotonic()-started)*1000)
                                return None,{'knots':knots.tolist(),'coefficients':best_curve.c.tolist(),'degree':3,'corridor':corridor},log
                    if movement<1e-5:break
                else:
                    trust*=.5
                    if trust<.005:break
            log['refinements'].append({'refinement':refinement,'control_points':n,'last':log['iterations'][-1] if log['iterations'] else None})
        raise CurveFailure('CURVE_NO_CERTIFIED_SOLUTION')
    except CurveFailure as error:
        log['failure']=str(error)
        if str(error)=='CURVE_BUDGET' and 'r11_primary_slice_s' in log:
            log['r11_primary_yielded']=True
        return None, None if best_curve is None else {'knots':best_curve.t.tolist(),'coefficients':best_curve.c.tolist(),'degree':3,'corridor':corridor},log
    finally:log['wall_ms']=(time.monotonic()-started)*1000


def _optimize_once(world,mask,guide,start,goal,deadline,config=CONFIG):
    # Alternative initialization consumes the SAME primary/aggregate deadline.
    # The original dense solve stays available after any uncertified attempt.
    if config.get('r4_rounded_guide') or not getattr(world,'v7_map_only',False):
        return _optimize_mesh(world,mask,guide,start,goal,deadline,
                              dict(config,r4_adaptive_stations=False,r4_tighter_primary=False))
    if not config.get('r4_adaptive_stations'):
        return _optimize_mesh(world,mask,guide,start,goal,deadline,config)
    began=time.monotonic()
    result=_optimize_mesh(world,mask,guide,start,goal,min(deadline,began+.75),config)
    if result[0] is not None:return result
    attempt=result[2]
    if time.monotonic()>=deadline:return result
    result=_optimize_mesh(world,mask,guide,start,goal,deadline,
                          dict(config,r4_adaptive_stations=False,r4_tighter_primary=False))
    result[2]['r4_adaptive_attempt']=attempt
    result[2]['wall_ms']=(time.monotonic()-began)*1000
    return result


def optimize(world,mask,guide,start,goal,deadline,config=CONFIG):
    """Keep the established solver first; bounded alternative seeds on failure."""
    whole_started=time.monotonic()
    detour_attempt=None
    if config.get('r7_detour_seed') and not config.get('r4_recovery_only'):
        # A local L2 detour can have many sharp corners. Try the existing
        # tightly tracked rounded initialization before the ordinary solver.
        # This is a heuristic, never a rejection test; all old attempts remain
        # available within the unchanged aggregate deadline if it fails.
        trial_cfg=dict(config,r7_detour_seed=False,r4_rounded_guide=True,
                       adaptive_constraint_stations=True,
                       tracking_weight=config['tracking_weight']*100.)
        trial=_optimize_once(world,mask,guide,start,goal,
                             min(deadline,whole_started+2.5),trial_cfg)
        detour_attempt=trial[2]
        if trial[0] is not None:
            trial[2]['r7_detour_seed']={'selected':True,'original_fallback_retained':True}
            trial[2]['wall_ms']=(time.monotonic()-whole_started)*1000
            return trial
    if config.get('r5_early_seed'):config=dict(config,_r5_aggregate_deadline=deadline)
    primary_deadline=min(deadline,time.monotonic()+3.) if config.get('r4_failure_recovery') else deadline
    primary=((None,None,{'failure':'CURVE_PRIOR_PRIMARY_REJECTED','wall_ms':0.})
             if config.get('r4_recovery_only') else
             _optimize_once(world,mask,guide,start,goal,primary_deadline,config))
    if detour_attempt is not None:primary[2]['r7_detour_seed_failure']=detour_attempt
    if config.get('r4_defer_recovery'):return primary
    if not config.get('r4_failure_recovery') or primary[0] is not None:return primary
    if primary[2].get('failure') not in ('CURVE_NO_CERTIFIED_SOLUTION','CURVE_BUDGET','CURVE_PRIOR_PRIMARY_REJECTED'):return primary
    started=time.monotonic();attempts=[]
    choices=(((config['tracking_weight'],.75),) if config.get('r5_speculative_seed') else
             ((config['tracking_weight'],1.25),(config['tracking_weight']*100.,2.)))
    for weight,allowance in choices:
        if deadline-time.monotonic()<.05:break
        trial_cfg=dict(config,r4_rounded_guide=True,adaptive_constraint_stations=True,tracking_weight=weight)
        result=_optimize_once(world,mask,guide,start,goal,min(deadline,time.monotonic()+allowance),trial_cfg)
        attempts.append(result[2])
        if result[0] is not None:
            result[2]['r4_original_failure']=primary[2]
            result[2]['r4_recovery_attempts']=attempts[:-1]
            result[2]['r4_recovered']=True
            result[2]['r4_selected_solver_ms']=result[2]['wall_ms']
            result[2]['wall_ms']=(time.monotonic()-whole_started)*1000
            result[2]['r4_total_solver_ms']=result[2]['wall_ms']
            return result
    primary[2]['r4_recovery_attempts']=attempts
    primary[2]['r4_recovered']=False
    primary[2]['wall_ms']=(time.monotonic()-whole_started)*1000
    primary[2]['r4_total_solver_ms']=primary[2]['wall_ms']
    if primary[2].get('failure')=='CURVE_PRIOR_PRIMARY_REJECTED':
        primary[2]['failure']='CURVE_BUDGET' if time.monotonic()>=deadline else 'CURVE_NO_CERTIFIED_SOLUTION'
    return primary
