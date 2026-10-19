"""Local cubic deformation with an exactly retained complete curve suffix.

The compact-support displacement vanishes with its first two derivatives at
the join. Every candidate is certified against the current complete map and
the original ordered corridor; the caller still runs all original audits.
"""
import math
import time
import numpy as np
from scipy.interpolate import BSpline, insert
from .curve_l3 import CurveFailure, certify, check_deadline


def insert_knots(spline, values):
    t, c = spline.t.copy(), spline.c.copy()
    for value, count in values:
        missing = count - int(np.count_nonzero(t == value))
        if missing > 0:
            t, coefficients, _ = insert(float(value), (t, c.T, 3), m=missing)
            c = np.asarray(coefficients).T[:len(t)-4]
    return BSpline(t, c, 3)


def suffix_spline(model):
    spline = BSpline(model['knots'], model['coefficients'], 3)
    start = float(model['parameters'][0])
    spline = insert_knots(spline, [(start, 4)])
    index = int(np.searchsorted(spline.t, start))
    return BSpline(spline.t[index:], spline.c[index:], 3)


def add_splines(spline, displacement):
    union = np.unique(np.r_[spline.t, displacement.t])
    multiplicities = [(v, max(np.count_nonzero(spline.t == v),
                             np.count_nonzero(displacement.t == v))) for v in union]
    base = insert_knots(spline, multiplicities)
    shift = insert_knots(displacement, multiplicities)
    assert np.array_equal(base.t, shift.t)
    return BSpline(base.t, base.c+shift.c, 3)


def deform(spline, pose, span):
    """Match actual position/yaw without altering the curve after span."""
    start, end = float(spline.t[3]), float(spline.t[-4])
    join = min(start + span, end)
    if join - start < 1e-5:
        raise CurveFailure('CURVE_CONTINUATION_TOO_SHORT')
    middle = (start + join) / 2
    internal = [middle] + ([join] if join < end else [])
    knots = np.r_[[start]*4, internal, [end]*4]
    delta = np.zeros((len(knots)-4, 2))
    old_velocity = spline(start, nu=1)
    velocity = np.linalg.norm(old_velocity)*np.array([math.cos(pose[2]), math.sin(pose[2])])
    delta[0] = np.asarray(pose[:2])-spline(start)
    delta[1] = delta[0] + (middle-start)/3*(velocity-old_velocity)
    displacement = BSpline(knots, delta, 3)
    return add_splines(spline, displacement), join


def length_gate(path, base, model, config):
    xy=base(np.asarray(model['parameters']))
    original=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum())
    length=float(np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1).sum())
    if length>config['max_length_ratio_to_guide']*original:
        raise CurveFailure('CURVE_LENGTH_GATE')
    return length


def continue_curve(world, mask, model, start, goal, deadline, config):
    started = time.monotonic()
    log = {'algorithm': 'compact-support-cubic-continuation', 'old_l3_calls': 0,
           'success': False, 'candidates': [], 'full_remaining_route': True}
    try:
        base = suffix_spline(model)
        for span in (.5, 1., 1.5, 2., 3., 4., 6.):
            check_deadline(deadline)
            candidate, join = deform(base, start, span)
            item = {'parameter_span': span, 'join_parameter': join}
            log['candidates'].append(item)
            try:
                path, certificate = certify(candidate, world, mask, deadline, config,
                                            corridor=model.get('corridor'))
                length=length_gate(path,base,model,config)
                parameters = certificate.pop('sample_parameters')
                if not np.allclose(path[0, :2], start[:2], atol=1e-9, rtol=0):
                    raise CurveFailure('CURVE_CONTINUATION_START_BINDING')
                if abs((path[0, 2]-start[2]+np.pi)%(2*np.pi)-np.pi)>1e-9:
                    raise CurveFailure('CURVE_CONTINUATION_YAW_BINDING')
                item['success'] = True
                log.update(success=True, certificate=certificate,
                           retained_from_parameter=join, control_points=len(candidate.c),length_m=length)
                result = {'knots': candidate.t.tolist(), 'coefficients': candidate.c.tolist(),
                          'degree': 3, 'corridor': model.get('corridor'), 'parameters': parameters}
                return path, result, log
            except CurveFailure as exc:
                item['failure'] = str(exc)
                if str(exc) == 'CURVE_BUDGET':
                    raise
        raise CurveFailure('CURVE_CONTINUATION_NO_CERTIFIED_SOLUTION')
    except CurveFailure as exc:
        log['failure'] = str(exc)
        return None, None, log
    finally:
        log['wall_ms'] = (time.monotonic()-started)*1000


def bump(spline, center, half_width, offset):
    """A cubic basis bump, zero position/first/second derivative outside it."""
    start,end=float(spline.t[3]),float(spline.t[-4])
    inner=center+np.arange(-2,3)*half_width
    if inner[0]<=start or inner[-1]>=end:
        raise CurveFailure('CURVE_OBSTACLE_NEAR_ENDPOINT')
    knots=np.r_[[start]*4,inner,[end]*4]
    coefficients=np.zeros((len(knots)-4,2))
    coefficients[4]=np.asarray(offset)*1.5
    return add_splines(spline,BSpline(knots,coefficients,3))


def repair_obstacles(world, mask, model, dynamic_mask, deadline, config):
    from arena_3d_v1.semantic_world import SweptChecker
    from scipy.spatial import cKDTree
    started=time.monotonic()
    log={'algorithm':'compact-support-cubic-obstacle-repair','old_l3_calls':0,
         'success':False,'candidates':[],'full_remaining_route':True}
    try:
        base=suffix_spline(model);u=np.asarray(model['parameters']);xy=base(u)
        rows,cols=np.nonzero(dynamic_mask)
        if not len(rows):raise CurveFailure('CURVE_NO_DYNAMIC_OBSTACLE')
        resolution=world.map.resolution;origin=world.map.origin
        obstacles=np.c_[origin[0]+(cols+.5)*resolution,
                        origin[1]+(world.hard.shape[0]-rows-.5)*resolution]
        distance,_=cKDTree(obstacles).query(xy)
        affected=np.flatnonzero(distance<math.hypot(*world.vehicle[:2])+resolution)
        if not len(affected):raise CurveFailure('CURVE_NO_NEARBY_DYNAMIC_OBSTACLE')
        center=float(u[int(affected[np.argmin(distance[affected])])])
        tangent=base(center,nu=1);normal=np.array([-tangent[1],tangent[0]])/np.linalg.norm(tangent)
        checker=SweptChecker(world.hard|~mask,resolution,origin,*world.vehicle[:2])
        # All added obstacles remain in the final checker. A candidate that
        # handles one obstruction but hits another is rejected, never merged
        # with an unchecked suffix.
        for half_width in (.5,1.,1.5,2.,3.):
            for amplitude in (.3,-.3,.45,-.45,.6,-.6,.8,-.8,1.,-1.):
                check_deadline(deadline)
                item={'center_parameter':center,'half_width_parameter':half_width,'offset_m':amplitude}
                log['candidates'].append(item)
                try:
                    candidate=bump(base,center,half_width,amplitude*normal)
                    sample=np.unique(np.r_[u,candidate.t])
                    v=candidate(sample,nu=1);pos=candidate(sample)
                    poses=np.c_[pos,np.arctan2(v[:,1],v[:,0])]
                    if not checker.check(poses):raise CurveFailure('CURVE_SAMPLED_BODY_COLLISION')
                    path,certificate=certify(candidate,world,mask,deadline,config,corridor=model.get('corridor'))
                    length=length_gate(path,base,model,config)
                    parameters=certificate.pop('sample_parameters')
                    result={'knots':candidate.t.tolist(),'coefficients':candidate.c.tolist(),'degree':3,
                            'corridor':model.get('corridor'),'parameters':parameters}
                    item['success']=True
                    log.update(success=True,certificate=certificate,length_m=length,control_points=len(candidate.c))
                    return path,result,log
                except CurveFailure as exc:
                    item['failure']=str(exc)
                    if str(exc)=='CURVE_BUDGET':raise
        raise CurveFailure('CURVE_OBSTACLE_NO_CERTIFIED_SOLUTION')
    except CurveFailure as exc:
        log['failure']=str(exc)
        return None,None,log
    finally:log['wall_ms']=(time.monotonic()-started)*1000
