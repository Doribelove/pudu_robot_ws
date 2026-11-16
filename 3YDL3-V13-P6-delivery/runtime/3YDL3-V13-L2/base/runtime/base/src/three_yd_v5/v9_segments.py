"""Bounded, process-local verified geometry cache shared across tasks.

Entries contain poses only, never an audit or ACK. Matching proposes geometry;
the current footprint/scope and whole-task traffic audit decide acceptance.
Any map/vehicle/rule/search-contract change clears the cache conservatively.
"""
import math
import numpy as np
from scipy.spatial import cKDTree
from arena_evaluation.semantic_map import canonical_hash
from arena_3d_v1.semantic_world import SweptChecker,densify,wrap
from .v9_maps import content_digest,seal
from .r6_cache import BoundedLRU

class SegmentCache(BoundedLRU):
    def __init__(self):
        super().__init__(32,4*2**20)
        self.binding=None;self.hits=0;self.invalidations=0

    def bind(self,key):
        if self.binding!=key:
            if self.binding is not None:self.invalidations+=1
            self.clear();self.binding=key

    def diagnostics(self):
        return {**super().diagnostics(),'hits':self.hits,'invalidations':self.invalidations}

def cache_for(c):
    key=('V9','verified-road-segments')
    cache=c.views.get(key)
    if cache is None:cache=c.views[key]=SegmentCache()
    w=c.world
    cache.bind(canonical_hash(['V9-segments-1',w.key,w.map.sha256,w.map.resolution,w.map.origin,
        content_digest(w.map.occupancy),content_digest(w.hard),content_digest(w.labels),
        w.semantic.semantic_map_hash,w.vehicle,w.rules,c.max_search_cells,c.max_map_fraction,
        'L2=.15;L3=.05;normal=.8;corner=1.2;Jackal;strict;Smac48']))
    return cache

def store(c,out):
    cache=cache_for(c);result=out.get('result')
    if not out.get('success') or result is None or not result.points:return cache.diagnostics()
    p=np.asarray([[v['x'],v['y'],v['yaw']] for v in result.points],float)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1))]
    # Fixed length pieces, with small overlap for task-independent matching.
    # Never store a partial result from a failed planner or a parking-only run.
    for at in np.arange(1.,max(1.,arc[-1]-4.),16.):
        a=int(np.searchsorted(arc,at));b=min(len(p)-1,int(np.searchsorted(arc,at+20.)))
        if b-a<2 or arc[b]-arc[a]<4.:continue
        chunk=p[a:b+1]
        if not c.generic_mode:
            rr,cc=c.world.cells(chunk[:,:2]);labels=c.topology.subregions[rr,cc]
            lane_ids=[rid for rid,v in c.topology.regions.items() if v['kind']=='lane']
            if not np.any(np.isin(labels,lane_ids)):continue
        key=content_digest(chunk)
        value={'poses':seal(chunk),'source_task':str(c.topology_route.signature)}
        # Reject oversize values before insertion (BoundedLRU permits one
        # oversize generic object; this cache has a strict byte ceiling).
        from .r6_cache import resident_bytes
        if resident_bytes(value)>cache.max_bytes:continue
        cache[key]=value
    return cache.diagnostics()

def propose(c,p,mask):
    cache=cache_for(c);tree=cKDTree(p[:,:2]);choices=[]
    from .v9_r2_options import enabled as r2_enabled
    checker=None
    if not r2_enabled('M'):checker=SweptChecker(c.world.hard|~mask,c.world.map.resolution,c.world.map.origin,*c.world.vehicle[:2])
    for key,value in list(cache._data.items()):
        q=value['poses'];distance,index=tree.query(q[:,:2])
        okay=(distance<=.30)&(np.abs(wrap(q[:,2]-p[index,2]))<=math.radians(10))
        # Match travel order; reverse travel, repeated visits and gaps cannot
        # silently acquire a previously accepted direction certificate.
        okay[1:]&=(np.diff(index)>=0)&(np.diff(index)<=8)
        starts=np.flatnonzero(okay&np.r_[True,~okay[:-1]])
        ends=np.flatnonzero(okay&np.r_[~okay[1:],True])
        for a,b in zip(starts,ends):
            if b-a<2:continue
            x,y=int(index[a]),int(index[b])
            length=float(np.linalg.norm(np.diff(q[a:b+1,:2],axis=0),axis=1).sum())
            if x<20 or y>len(p)-21 or y<=x or length<4.:continue
            choices.append((length,key,x,y,q[a:b+1]))
    selected=[];occupied=[]
    if choices and checker is None:
        checker=SweptChecker(c.world.hard|~mask,c.world.map.resolution,c.world.map.origin,*c.world.vehicle[:2])
    for length,key,a,b,q in sorted(choices,key=lambda x:-x[0]):
        if any(not(b+25<x or a-25>y) for x,y in occupied):continue
        if not checker.check(densify(q)):continue
        occupied.append((a,b));selected.append((a,b,q.copy(),key,length))
        if len(selected)>=4:break
    for a,b,q,key,length in sorted(selected,reverse=True,key=lambda x:x[0]):
        p=np.vstack((p[:a],q,p[b+1:]));cache.get(key);cache.hits+=1
    return p,{'matched_segments':len(selected),'matched_length_m':sum(v[4] for v in selected),
        'sources':[v[3] for v in selected],'current_body_and_scope_checked':True,
        'cache':cache.diagnostics()}
