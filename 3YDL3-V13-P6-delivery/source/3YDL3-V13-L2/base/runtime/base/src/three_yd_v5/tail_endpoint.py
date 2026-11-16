"""Bounded lateral endpoint initializations inside the existing L3 mask.

These are guesses for the spline only. Original whole-curve certification and
the backend's traffic/body audits still decide whether a path is executable.
"""
import math,time
import numpy as np


def seed_failed_endpoints(p,s,start,goal,world,mask,deadline,logs,*,hard):
    from arena_evaluation.semantic_constraint_core import dubins_choices,dubins_edge_from_parameters
    from arena_3d_v1.semantic_world import densify
    from .curve_l3 import interp,check_deadline
    from .v10_storage import collision_checker
    checker=collision_checker(hard,world)
    for previous in list(logs):
        if previous.get('selected'):continue
        endpoint=previous['endpoint'];pose=start if endpoint=='start' else goal
        best=None;tested=0;tick=time.monotonic()
        for distance in (.5,1.,1.5,2.,3.,4.):
            if distance>s[-1]*.4:continue
            at=distance if endpoint=='start' else s[-1]-distance
            center=interp(p,s,[at])[0]
            before,after=interp(p,s,[max(0,at-.1),min(s[-1],at+.1)])
            direction=after-before;direction/=max(np.linalg.norm(direction),1e-15)
            normal=np.array([-direction[1],direction[0]])
            yaw=math.atan2(direction[1],direction[0])
            for offset in (-.2,.2,-.4,.4,-.6,.6):
                anchor=tuple(center+offset*normal)+(yaw,)
                a,b=(tuple(pose),anchor) if endpoint=='start' else (anchor,tuple(pose))
                for scale in (1.0125,1.125,1.5):
                    radius=world.vehicle[2]*scale
                    for word,parameters in dubins_choices(a,b,radius):
                        check_deadline(deadline);length=radius*sum(parameters)
                        score=length-distance+abs(offset)*.25
                        if length>distance+5. or (best is not None and score>=best[0]):continue
                        edge=dubins_edge_from_parameters(a,b,radius,word,parameters)
                        poses=densify(np.vstack((a,edge.samples)));tested+=1
                        if checker.check(poses):best=(score,at,poses,word,radius,offset)
        log={'endpoint':endpoint,'kind':'lateral_anchor_initialization','tested':tested,
             'selected':best is not None,'wall_ms':(time.monotonic()-tick)*1000,
             'existing_scope_only':True,'not_executable_before_original_certificate':True}
        logs.append(log)
        if best is None:continue
        _,at,poses,word,radius,offset=best
        distance=at if endpoint=='start' else s[-1]-at
        p=np.vstack((poses[:,:2],p[s>at])) if endpoint=='start' else np.vstack((p[s<at],poses[:,:2]))
        p=p[np.r_[True,np.linalg.norm(np.diff(p,axis=0),axis=1)>1e-7]]
        s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
        log.update(word=word,radius_m=radius,anchor_distance_m=float(distance),lateral_offset_m=offset)
    return p,s,logs
