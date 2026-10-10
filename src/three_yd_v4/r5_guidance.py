"""R5 local right-boundary guidance. No contour edge identifiers are used."""
import math,time
import numpy as np
import cv2
from arena_3d_v1.semantic_reference import RoutePreference
from arena_3d_v1.semantic_world import digest
from arena_evaluation.semantic_map import canonical_hash
from scipy.spatial import cKDTree
from .guidance import TrafficAudit

def cross_section(poly,d,points):
    points=np.asarray(points);right=np.array([d[1],-d[0]]);t=points@d
    low=np.full(len(points),np.inf);high=np.full(len(points),-np.inf)
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        span=(b-a)@d
        if abs(span)<1e-9:continue
        f=(t-a@d)/span;ok=(f>=-1e-8)&(f<=1+1e-8);lat=a@right+f*((b-a)@right)
        low[ok]=np.minimum(low[ok],lat[ok]);high[ok]=np.maximum(high[ok],lat[ok])
    width=high-low;normal=np.divide(high-points@right,width,out=np.full(len(points),np.nan),where=np.isfinite(width)&(width>1e-8))
    return normal,low,high

def local_right_boundary(topology,visit,spacing=.15):
    """Sample the actual right polygon boundary between this visit's gates."""
    d=np.array(visit['direction']);poly=topology.router.polygons[visit['region_id']]
    a=np.array(visit['entry_hint']);b=np.array(visit['exit_hint']);t=np.linspace(a@d,b@d,max(2,int(abs((b-a)@d)/spacing)+1))
    p=t[:,None]*d;_,lo,hi=cross_section(poly,d,p);right=np.array([d[1],-d[0]])
    result=p+hi[:,None]*right;result=result[np.all(np.isfinite(result),axis=1)]
    if len(result)<=2:return result
    # L1 carries geometry breakpoints, not a dense raster-like sample. L2
    # deterministically re-samples this polyline at its own resolution.
    return cv2.approxPolyDP(result.astype(np.float32)[:,None,:],.02,False)[:,0,:].astype(float)

def densify_polyline(points,spacing=.15):
    points=np.asarray(points,float);out=[]
    for i,(a,b) in enumerate(zip(points,points[1:])):
        length=float(np.linalg.norm(b-a));count=max(2,int(math.ceil(length/spacing))+1)
        segment=a+(b-a)*np.linspace(0.,1.,count)[:,None]
        out.extend(segment[:-1] if i<len(points)-2 else segment)
    return np.asarray(out,float)

def road_guide(topology,visit,spacing=.15):
    stored=np.asarray(visit.get('right_boundary',[]),float)
    if stored.ndim!=2 or len(stored)<2:stored=local_right_boundary(topology,visit,spacing)
    return densify_polyline(stored,spacing) if len(stored)>=2 else stored

class RegionTraffic(TrafficAudit):
    def __init__(self,world,topology,route):
        self.world=world;self.topology=topology;self.route=route;self.task=topology.task(route)
        self.visits=[v for v in self.task['visits'] if v['kind']=='lane'];self.roads=[(v['region_id'],np.array(v['direction'])) for v in self.visits]
        self.endpoint_parking={a['semantic_region_id'] for a in self.task['endpoint_access'].values() if topology.regions[a['semantic_region_id']]['kind']=='parking_area'}
    def audit(self,poses):
        base=super().audit(poses)
        # Original traffic checks remain; L2 separately enforces exact region occurrence order.
        return base

def region_field(world,view,topology,task):
    started=time.monotonic();shape=view.safe.shape;rr,cc=np.indices(shape);xy=view.world_points(rr.ravel(),cc.ravel());fr,fc=world.cells(xy)
    valid=(fr>=0)&(fc>=0)&(fr<world.map.height)&(fc<world.map.width);labels=np.zeros(len(xy),np.int16);labels[valid]=topology.subregions[fr[valid],fc[valid]]
    pot=np.zeros(len(xy),np.float32);normal=np.full(len(xy),np.nan,np.float32);eligible=np.zeros(len(xy),bool);amb=np.zeros(len(xy),bool);direction=np.zeros((len(xy),2),np.float32);progress=np.full(len(xy),-1,np.int16)
    seen=set();boundary_lengths=[]
    for stage,v in enumerate(task['visits']):
        if v['kind']!='lane':continue
        ids=np.flatnonzero(labels==v['region_id']);d=np.array(v['direction']);norm,lo,hi=cross_section(topology.router.polygons[v['region_id']],d,xy[ids]);ok=np.isfinite(norm)
        if v['region_id'] in seen:amb[ids]=True;continue
        seen.add(v['region_id']);normal[ids]=norm;eligible[ids]=ok;direction[ids]=d;progress[ids]=stage
        boundary=road_guide(topology,v)
        if len(boundary)>=2:
            boundary_lengths.append(float(np.linalg.norm(np.diff(boundary,axis=0),axis=1).sum()))
            distance=cKDTree(boundary).query(xy[ids[ok]],k=1)[0]
            # A finite positive cost everywhere in the road: farther from the
            # selected local right boundary is increasingly expensive, but
            # remains soft so physical obstacles can force a detour.
            pot[ids[ok]]=np.minimum(1.,distance/3.).astype(np.float32)
        else:pot[ids[ok]]=1.
    pref=RoutePreference(pot.reshape(shape),normal.reshape(shape),eligible.reshape(shape),amb.reshape(shape),canonical_hash(['r5-local-right-boundary-v1',task['signature'],view.key,digest(pot)]),{'build_ms':(time.monotonic()-started)*1000,'source':'actual local right polygon boundary between entry and exit','local_boundary_length_m':float(sum(boundary_lengths)),'junction_attraction':False,'parking_attraction':False,'soft_not_hard':True,'not_collision_certificate':True})
    return pref,direction.reshape((*shape,2)),progress.reshape(shape),labels.reshape(shape)
