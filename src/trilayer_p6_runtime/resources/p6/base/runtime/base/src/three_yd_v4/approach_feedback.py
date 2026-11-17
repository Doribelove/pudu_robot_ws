"""Version-bound, locally checked approach connections; finite L2 guidance."""
from types import SimpleNamespace
import math
import numpy as np
from arena_3d_v1.semantic_world import digest,wrap


def connection(world,poses,*,route_signature,map_version):
    p=np.asarray(poses,float)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)<3 or not np.all(np.isfinite(p)):
        raise ValueError('INVALID_APPROACH_CONNECTION')
    q=SimpleNamespace(start=p[0],goal=p[-1])
    audit=world.audit(q,p,~world.hard)
    ds=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
    steer=np.arctan(.5*np.divide(wrap(np.diff(p[:,2])),ds,out=np.zeros_like(ds),where=ds>1e-8))
    if not audit['valid'] or np.max(abs(np.diff(steer)),initial=0)>math.pi/12+1e-6:
        raise ValueError('APPROACH_CONNECTION_NOT_VEHICLE_FEASIBLE')
    i=len(p)//2
    return {'world_key':world.key,'vehicle':list(world.vehicle),'route_signature':route_signature,
            'map_version':map_version,'poses':p.tolist(),'pose':p[i].tolist(),'path_hash':digest(p),
            'support_radius_m':1.2,'weight':.2,'certificate':'local_geometry_only; full traffic and trajectory audit still required'}


def validate_connections(world,records,*,route_signature,map_version):
    out=[]
    for r in records[:3]:
        if (r['world_key']!=world.key or tuple(r['vehicle'])!=tuple(world.vehicle) or
            r['route_signature']!=route_signature or r['map_version']!=map_version):
            raise ValueError('APPROACH_FEEDBACK_BINDING_MISMATCH')
        checked=connection(world,r['poses'],route_signature=route_signature,map_version=map_version)
        if checked['path_hash']!=r['path_hash']:raise ValueError('APPROACH_FEEDBACK_CONTENT_MISMATCH')
        out.append(checked)
    return out


def guidance(points,records):
    points=np.asarray(points,float);potential=np.zeros(len(points));vectors=np.zeros((len(points),2));strength=np.zeros(len(points))
    for r in records:
        pose=np.asarray(r['pose']);v=np.array([math.cos(pose[2]),math.sin(pose[2])]);delta=points-pose[:2]
        support=np.maximum(0.,1-np.linalg.norm(delta,axis=1)/r['support_radius_m'])**2*r['weight']
        lateral=np.abs(delta[:,0]*v[1]-delta[:,1]*v[0])
        potential+=support*np.minimum(1.,lateral/.4)
        vectors+=support[:,None]*v;strength+=support
    n=np.linalg.norm(vectors,axis=1);vectors=np.divide(vectors,n[:,None],out=np.zeros_like(vectors),where=n[:,None]>1e-12)
    return potential,vectors,strength


def export_connections(world,p,*,route_signature,map_version):
    """Only called after the whole trajectory passes the strict acceptance."""
    p=np.asarray(p);arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1))]
    indices=np.argsort(-abs(wrap(np.diff(p[:,2]))));records=[];used=[]
    for i in indices:
        if len(records)>=3:break
        if arc[i]<1. or arc[-1]-arc[i]<1. or any(abs(arc[i]-s)<3. for s in used):continue
        a=max(0,np.searchsorted(arc,arc[i]-.5));b=min(len(p),np.searchsorted(arc,arc[i]+.5)+1)
        try:r=connection(world,p[a:b],route_signature=route_signature,map_version=map_version)
        except ValueError:continue
        records.append(r);used.append(arc[i])
    return records
