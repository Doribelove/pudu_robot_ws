"""Versioned world-coordinate obstacle deltas; each ID owns only its overlay.
Loss of sight never means free. Explicit free-space evidence is required to
remove an obstacle. No dynamic object mutates the approved outline graph.
"""
from dataclasses import dataclass
import threading,time,math
import numpy as np
import cv2
from arena_evaluation.semantic_map import canonical_hash
from arena_3d_v1.multires_grid import aggregate
from arena_3d_v1.semantic_world import digest

class DynamicObstacles:
    def __init__(self,world,*,static_hard=None):
        if static_hard is not None:
            from three_yd_v5.v9_maps import immutable
            if static_hard is not world.hard or not immutable(static_hard):
                raise ValueError('STATIC_STORAGE_MUST_BE_CURRENT_IMMUTABLE_WORLD')
        self.world=world;self.static_hard=world.hard.copy() if static_hard is None else static_hard;self.obstacles={};self.sequence=-1;self.version=0
        from three_yd_v5.v10_storage import empty_overlay
        self.mask=empty_overlay(world.hard.shape) if static_hard is not None else np.zeros(world.hard.shape,bool);self.lock=threading.RLock();self.last_payload=None
        self.content_hash=digest(self.mask)
    def apply(self,sequence,*,upsert=None,clear=(),observed_free=False):
        with self.lock:
            if isinstance(sequence,bool) or not isinstance(sequence,int):raise ValueError('INVALID_SEQUENCE')
            upsert=upsert or {};clear=tuple(clear)
            payload=canonical_hash([sequence,upsert,clear,observed_free])
            if sequence<self.sequence:return {'accepted':False,'reason':'STALE_SEQUENCE','version':self.version}
            if sequence==self.sequence:
                if payload!=self.last_payload:raise ValueError('CONFLICTING_SAME_SEQUENCE')
                return {'accepted':True,'reason':'DUPLICATE','version':self.version,'changed_cells':0}
            if clear and not observed_free:raise ValueError('CLEAR_REQUIRES_FREE_OBSERVATION')
            candidate=dict(self.obstacles)
            for key in clear:candidate.pop(str(key),None)
            for key,polygon in upsert.items():
                p=np.asarray(polygon,float)
                if p.ndim!=2 or p.shape[1]!=2 or len(p)<3 or not np.all(np.isfinite(p)):raise ValueError('INVALID_WORLD_POLYGON')
                r,c=self.world.cells(p)
                if not np.all((r>=0)&(r<self.mask.shape[0])&(c>=0)&(c<self.mask.shape[1])):raise ValueError('DYNAMIC_POLYGON_OUTSIDE_MAP')
                if abs(np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1)))<1e-10:raise ValueError('DEGENERATE_WORLD_POLYGON')
                candidate[str(key)]=p.copy()
            mask=np.zeros(self.mask.shape,np.uint8)
            for p in candidate.values():
                r,c=self.world.cells(p);cv2.fillPoly(mask,[np.column_stack((c,r)).astype(np.int32)],1)
            mask=mask.astype(bool);changed=mask!=self.mask;rows,cols=np.nonzero(changed)
            self.sequence=sequence;self.last_payload=payload;self.obstacles=candidate
            if len(rows):self.version+=1
            self.mask=mask;self.content_hash=digest(mask);self.world.hard=self.static_hard|mask
            return {'accepted':True,'reason':'APPLIED','version':self.version,'sequence':sequence,'changed_cells':len(rows),'dirty_bbox':None if not len(rows) else [int(rows.min()),int(rows.max()+1),int(cols.min()),int(cols.max()+1)],'content_hash':self.content_hash,'l1_map_mutated':False}
    def blocked_cells(self,view):
        # Static safety geometry stays intact so removal can reuse D* state.
        radius=math.hypot(*self.world.vehicle[:2])+.05+math.sqrt(2)*self.world.map.resolution/2
        pixels=math.ceil(radius/self.world.map.resolution)
        kernel=np.ones((2*pixels+1,2*pixels+1),np.uint8)  # conservative square envelope covers the circle
        inflated=cv2.dilate(self.mask.astype(np.uint8),kernel)>0
        coarse=inflated if view.factor==1 else aggregate(inflated,view.factor)
        return set(zip(*np.nonzero(coarse)))
