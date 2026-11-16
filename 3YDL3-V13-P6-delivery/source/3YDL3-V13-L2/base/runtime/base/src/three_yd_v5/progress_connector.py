"""Small actual-pose to retained-route splice; no vehicle search or snapping.

At most four <=3m quintic proposals, <=0.15m initial route deviation, <=10deg
heading deviation (R6 H candidate filter: <=15deg). Each proposal is checked with the ORIGINAL whole-suffix
body/curvature and original full-task traffic audit. Final ACK/audits follow.
"""
import time
import numpy as np
from .task_progress import wrap,ProgressTraffic
from .v9_r6_options import heading_limit_degrees


def quintic(start,goal,end_curvature,scale):
    start=np.asarray(start,float);goal=np.asarray(goal,float)
    delta=goal[:2]-start[:2]
    v0=scale*np.array([np.cos(start[2]),np.sin(start[2])])
    v1=scale*np.array([np.cos(goal[2]),np.sin(goal[2])])
    a1=end_curvature*scale**2*np.array([-np.sin(goal[2]),np.cos(goal[2])])
    co=np.array([start[:2],v0,np.zeros(2),10*delta-6*v0-4*v1+.5*a1,
        -15*delta+8*v0+7*v1-a1,6*delta-3*v0-3*v1+.5*a1])
    t=np.linspace(0,1,max(16,int(np.ceil(scale/.025))+1))
    xy=np.stack([t**i for i in range(6)],axis=1)@co
    vel=np.stack([i*t**(i-1) for i in range(1,6)],axis=1)@co[1:]
    p=np.column_stack((xy,np.arctan2(vel[:,1],vel[:,0])))
    p[0]=start;p[-1]=goal
    return p


def propose(service,query):
    limit=heading_limit_degrees()
    tick=time.monotonic();p=service.remaining;pose=np.asarray(query.start,float)
    distances=np.linalg.norm(p[:,:2]-pose[:2],axis=1);nearest=int(np.argmin(distances))
    if distances[nearest]>.15+1e-9:return None,'POSE_TOO_FAR_FROM_REMAINING_ROUTE'
    if abs(wrap(p[nearest,2]-pose[2]))>np.deg2rad(limit):return None,'HEADING_NOT_ALONG_ROUTE'
    near=np.flatnonzero(distances<=min(.15,distances[nearest]+.015))
    if len(near) and np.any(np.diff(near)>2):return None,'AMBIGUOUS_NEAR_ROUTE_VISIT'
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1))]
    prefix=np.concatenate((service.prefix,p[:nearest]));traffic=ProgressTraffic(service.base_traffic,prefix)
    mask=service.controller._target_mask();trials=[]
    for span in (.8,1.2,2.,3.):
        if time.monotonic()-tick>.8:break
        j=int(np.searchsorted(arc,arc[nearest]+span));
        if j>=len(p)-1:continue
        ds=float(np.linalg.norm(p[j+1,:2]-p[j,:2]))
        k=float(wrap(p[j+1,2]-p[j,2])/ds) if ds>1e-8 else 0.
        scale=max(float(arc[j]-arc[nearest]),float(np.linalg.norm(p[j,:2]-pose[:2])))
        curve=quintic(pose,p[j],k,scale)
        tail=np.concatenate((curve[:-1],p[j:]))
        body=service.world.audit(query,tail,mask)
        semantic=traffic.audit(tail)
        d=np.diff(tail,axis=0);length=np.linalg.norm(d[:,:2],axis=1)
        steer=np.arctan(.5*np.divide(wrap(d[:,2]),length,out=np.zeros_like(length),where=length>1e-8))
        jump=float(np.max(abs(np.diff(steer)),initial=0)*180/np.pi)
        trial={'span_m':span,'join_vertex':j,'body_failures':body.get('failures',[]),
            'traffic_failures':semantic.get('failures',[]),'steering_jump_deg':jump}
        trials.append(trial)
        if body['valid'] and semantic['valid'] and jump<=15+1e-6:
            return (nearest,tail,{'kind':'bounded_actual_pose_connector','trials':trials,
                'position_deviation_m':float(distances[nearest]),'heading_deviation_deg':float(abs(wrap(p[nearest,2]-pose[2]))*180/np.pi),
                'candidate_heading_limit_deg':limit,'connector_vertices':len(curve),'wall_ms':(time.monotonic()-tick)*1000}), 'LOCAL_CONNECTOR'
    return None,'NO_STRICT_LOCAL_CONNECTOR'
