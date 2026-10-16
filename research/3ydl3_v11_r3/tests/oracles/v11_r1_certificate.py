"""Frozen V11-r1 interval certificate oracle; mathematical code unchanged."""
import math,time
import numpy as np
from three_yd_v5.curve_l3 import CONFIG,CurveFailure,check_deadline

def bezier_pieces(spline):
    knots=np.unique(spline.t)
    for a,b in zip(knots,knots[1:]):
        if b<=a:continue
        p=spline(np.array([a,b]));v=spline(np.array([a,b]),nu=1);h=b-a
        yield a,b,np.vstack((p[0],p[0]+v[0]*h/3,p[1]-v[1]*h/3,p[1]))

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
    from three_yd_v5.v10_storage import collision_checker
    checker=collision_checker(world.hard|~mask if hard is None else hard,world)
    body_refinements=0;body_groups=0
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
            stations=np.asarray(corridor['stations']);boxes0=np.asarray(corridor['bounds']);axes0=np.asarray(corridor['axes'])
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
