"""Evidence for query-local recovery; search limits never certify blockage."""
from pathlib import Path
import time,math
import numpy as np
from arena_3d_v1.semantic_world import SweptChecker,densify,wrap,digest
from arena_3d_v1.semantic_ack_session import SemanticAckSession


def search_reason(text, planner_success=False):
    if planner_success:return 'PLANNER_RETURNED_CANDIDATE'
    low=text.lower()
    if 'exceeded maximum iterations' in low:return 'SEARCH_ITERATION_LIMIT'
    if 'maximum planning time' in low or 'timed out' in low:return 'SEARCH_TIME_LIMIT'
    if 'occupied' in low and ('start' in low or 'goal' in low):return 'ENDPOINT_REJECTED'
    return 'UNCLASSIFIED_ACTION_FAILURE'


class DiagnosticSession(SemanticAckSession):
    """Capture only this action's log suffix, without changing Smac parameters."""
    def plan(self,*args,**kwargs):
        log=self.params_file.with_suffix('.log')
        offset=log.stat().st_size if log.exists() else 0
        result=super().plan(*args,**kwargs)
        if log.exists():
            with log.open('rb') as f:f.seek(offset);text=f.read().decode(errors='replace')
        else:text=''
        warnings=[line for line in text.splitlines() if any(k in line.lower() for k in ['failed','exceeded','aborting','error'])]
        result.diagnostics=dict(result.diagnostics or {})
        result.diagnostics.update(termination_reason=search_reason(text,result.planner_success),
            native_log_evidence=warnings[-8:],failure_is_physical_blockage=False)
        return result


def localize(world,query,poses,mask):
    """Locate first defects and separate source obstacles from semantic masks."""
    p=np.asarray(poses,float);started=time.monotonic()
    result={'map_hash':world.key,'path_hash':digest(p),'physical_blockage_proven':False,'feedback_scope':'query_local'}
    if len(p)<2:return {**result,'failure':'NO_CANDIDATE_GEOMETRY'}
    delta=np.diff(p,axis=0);ds=np.linalg.norm(delta[:,:2],axis=1);arc=np.r_[0.,np.cumsum(ds)]
    yaw=wrap(delta[:,2]);curvature=np.divide(2*np.sin(abs(yaw)/2),ds,out=np.zeros_like(ds),where=ds>1e-8)
    a,b=delta[:-1,:2],delta[1:,:2];den=np.linalg.norm(a,axis=1)*np.linalg.norm(b,axis=1)*np.linalg.norm(a+b,axis=1)
    geom=np.divide(2*abs(a[:,0]*b[:,1]-a[:,1]*b[:,0]),den,out=np.zeros_like(den),where=den>1e-12)
    steering=np.arctan(.5*np.divide(yaw,ds,out=np.zeros_like(ds),where=ds>1e-8))
    bad_curv=sorted(set(np.flatnonzero(curvature>1/world.vehicle[2]+1e-6).tolist()+np.flatnonzero(geom>1/world.vehicle[2]+1e-6).tolist()))
    bad_jump=np.flatnonzero(abs(np.diff(steering))>math.pi/12+1e-6).tolist()
    physical=[];restricted=[];checker=SweptChecker(world.hard|~mask,world.map.resolution,world.map.origin,*world.vehicle[:2])
    for i in range(len(p)-1):
        segment=densify(p[i:i+2])
        if not world.checker.check(segment):physical.append(i)
        elif not checker.check(segment):restricted.append(i)
    groups=[]
    for i in physical:
        if groups and arc[i]-groups[-1]['end_arc_m']<.5:groups[-1].update(end_index=i+1,end_arc_m=float(arc[i+1]))
        else:groups.append({'start_index':i,'end_index':i+1,'start_arc_m':float(arc[i]),'end_arc_m':float(arc[i+1]),'pose':p[i].tolist()})
    result.update(physical_collision_indices=physical,semantic_mask_collision_indices=restricted,
        curvature_indices=bad_curv,steering_jump_indices=bad_jump,physical_collision_groups=groups,
        wall_ms=(time.monotonic()-started)*1000)
    return result


def feedback_from_evidence(evidence,poses,max_zones=3):
    """Penalize a failed local connection; never erase obstacles or hard-close L1."""
    p=np.asarray(poses,float);zones=[]
    for g in evidence.get('physical_collision_groups',[])[:max_zones]:
        middle=(g['start_index']+g['end_index'])//2
        zones.append({'xy':p[middle,:2].tolist(),'yaw':float(p[middle,2]),'radius_m':1.,'weight':8.,'reason':'RECTANGLE_SWEEP_COLLISION'})
    if not zones:
        for i in evidence.get('curvature_indices',[])[:max_zones]:
            if any(np.linalg.norm(p[i,:2]-np.array(z['xy']))<1. for z in zones):continue
            zones.append({'xy':p[i,:2].tolist(),'yaw':float(p[i,2]),'radius_m':1.5,'weight':5.,'reason':'LOCAL_CURVATURE_INFEASIBLE'})
    return zones


def feedback_cost(points,zones):
    points=np.asarray(points,float);cost=np.zeros(len(points))
    for z in zones:
        distance=np.linalg.norm(points-np.asarray(z['xy']),axis=1)
        cost+=z['weight']*np.maximum(0.,1-distance/z['radius_m'])**2
    return cost
