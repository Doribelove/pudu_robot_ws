"""Explicit origin-aligned coarse/fine coordinates and conservative aggregation."""
from types import SimpleNamespace
import math, heapq, itertools, time
import numpy as np
from .semantic_world import digest
from arena_evaluation.semantic_map import canonical_hash
from .l2_incremental import CorridorROI, CorridorBinding

def aggregate(mask,factor,*,all_free=False,pad_value=False):
    a=np.asarray(mask);h,w=a.shape
    # Images are top-down; anchor blocks at the physical bottom-left origin.
    bottom=np.flipud(a)
    padded=np.full((math.ceil(h/factor)*factor,math.ceil(w/factor)*factor),pad_value,dtype=a.dtype)
    padded[:h,:w]=bottom
    blocks=padded.reshape(padded.shape[0]//factor,factor,padded.shape[1]//factor,factor)
    out=blocks.all(axis=(1,3)) if all_free else blocks.any(axis=(1,3))
    return np.ascontiguousarray(np.flipud(out))

class GridView:
    def __init__(self,fine,factor):
        if factor not in (1,3):raise ValueError('L2_FACTOR_MUST_BE_1_OR_3')
        self.fine=fine;self.factor=factor;self.rules=fine.rules;self.features=fine.features;self.vehicle=fine.vehicle
        self.key=canonical_hash(['3yd-grid-v1',fine.key,factor,'bottom-left-all-safe-any-hard'])
        self.map=SimpleNamespace(resolution=fine.map.resolution*factor,origin=fine.map.origin,
            height=math.ceil(fine.map.height/factor),width=math.ceil(fine.map.width/factor),sha256=fine.map.sha256)
        if factor==1:self.labels=fine.labels;self.safe=fine.safe;self.hard=fine.hard
        else:
            self.safe=aggregate(fine.safe,factor,all_free=True,pad_value=False)
            self.hard=aggregate(fine.hard,factor,pad_value=True)
            rr,cc=np.indices(self.safe.shape);xy=self.world_points(rr.ravel(),cc.ravel())
            r,c=fine.cells(xy);valid=(r>=0)&(r<fine.map.height)&(c>=0)&(c<fine.map.width)
            flat=np.zeros(len(r),fine.labels.dtype);flat[valid]=fine.labels[r[valid],c[valid]]
            self.labels=flat.reshape(self.safe.shape)
        self.directions=[]
        for rule in self.rules.get('one_way',[]):
            matches=[k for k,v in self.features.items() if v.semantic_id==rule['semantic_id']]
            if not matches:raise ValueError('UNKNOWN_RULE_SEMANTIC')
            membership=np.isin(fine.labels,matches)
            if factor>1:membership=aggregate(membership,factor)
            self.directions.append((membership,float(rule['yaw'])))

    def cells(self,p):
        p=np.asarray(p);c=np.floor((p[:,0]-self.map.origin[0])/self.map.resolution).astype(int)
        r=self.map.height-1-np.floor((p[:,1]-self.map.origin[1])/self.map.resolution).astype(int)
        return r,c
    def world_points(self,r,c):
        return np.column_stack((self.map.origin[0]+(np.asarray(c)+.5)*self.map.resolution,
            self.map.origin[1]+(self.map.height-np.asarray(r)-.5)*self.map.resolution))
    def sample_labels(self,p):
        r,c=self.cells(p);v=(r>=0)&(c>=0)&(r<self.map.height)&(c<self.map.width)
        out=np.full(len(r),-1);out[v]=self.labels[r[v],c[v]];return out
    def cell(self,pose):
        r,c=self.cells(np.asarray(pose)[None,:]);return int(r[0]),int(c[0])
    def corridor(self,fine_mask):
        return fine_mask if self.factor==1 else aggregate(fine_mask,self.factor,all_free=True,pad_value=False)

def make_roi(view,mask,start,goal,route_signature):
    for rc in [start,goal]:
        if not (0<=rc[0]<view.map.height and 0<=rc[1]<view.map.width and view.safe[rc] and mask[rc]):
            raise ValueError('COARSE_ENDPOINT_UNREPRESENTABLE' if view.factor>1 else 'FINE_ENDPOINT_UNSAFE')
    r,c=np.nonzero(mask);r0=max(0,int(r.min())-1);r1=min(mask.shape[0],int(r.max())+2)
    c0=max(0,int(c.min())-1);c1=min(mask.shape[1],int(c.max())+2)
    binding=CorridorBinding(view.key,mask.shape,view.map.origin,view.map.resolution,route_signature,(route_signature,),digest(mask),
        start,goal,canonical_hash(view.vehicle))
    return CorridorROI((r0,r1,c0,c1),np.ascontiguousarray((mask&view.safe)[r0:r1,c0:c1]),
        (start[0]-r0,start[1]-c0),(goal[0]-r0,goal[1]-c0),binding,int(mask.sum()))

def guide_astar(view,roi,*,timeout=20.):
    tick=time.monotonic();mask=roi.base_free;h,w=mask.shape;start=roi.start_local;goal=roi.goal_local
    serial=itertools.count();dist={start:0.};parent={};q=[(math.dist(start,goal),0.,next(serial),start)];expanded=0
    dirs=[(a,b,math.hypot(a,b)) for a in [-1,0,1] for b in [-1,0,1] if a or b]
    rules=[(m[roi.bbox[0]:roi.bbox[1],roi.bbox[2]:roi.bbox[3]],yaw) for m,yaw in view.directions]
    while q:
        if time.monotonic()-tick>timeout:raise TimeoutError('GUIDE_SEARCH_TIMEOUT')
        _,g,_,u=heapq.heappop(q)
        if g!=dist.get(u):continue
        if u==goal:
            path=[u]
            while u!=start:u=parent[u];path.append(u)
            path.reverse();rc=np.asarray(path)+[roi.bbox[0],roi.bbox[2]]
            return rc,{'guide_ms':(time.monotonic()-tick)*1000,'guide_expanded':expanded}
        expanded+=1
        for dr,dc,cost in dirs:
            r,c=u[0]+dr,u[1]+dc
            if not(0<=r<h and 0<=c<w and mask[r,c]):continue
            if dr and dc and not(mask[u[0],c] and mask[r,u[1]]):continue
            if any((m[u] or m[r,c]) and dc*math.cos(yaw)-dr*math.sin(yaw)<-1e-6 for m,yaw in rules):continue
            v=(r,c);ng=g+cost
            if ng<dist.get(v,math.inf):dist[v]=ng;parent[v]=u;heapq.heappush(q,(ng+math.dist(v,goal),ng,next(serial),v))
    raise RuntimeError('GUIDE_NO_PATH')
