"""Serialized full-route service with obstacle-impact replanning.

Map updates are event-driven; the consumer reads current_trajectory() rather
than retaining a previously returned path. A failed update leaves no executable
trajectory. This is a planner API, not a vehicle tracking/controller loop.
"""
from copy import deepcopy
from collections import deque
from dataclasses import replace
import math
import threading
import time
import cv2
import numpy as np
from arena_3d_v1.semantic_world import SweptChecker
from three_yd_v3.dynamic import DynamicObstacles
from .controller import V5Controller
from .recovery import V5Runner
from .fullroute import record_from, validated_reuse


def impact(controller, added, bbox):
    """Conservative local test, including L2 footprint/coarse aggregation."""
    if bbox is None or not np.any(added):
        return {'kind':'no_new_blockage','reference_affected':False,'scope_affected':False}
    w=controller.world;r0,r1,c0,c1=bbox
    radius=math.hypot(*w.vehicle[:2])+math.sqrt(2)*.15+w.map.resolution
    rad=int(math.ceil(radius/w.map.resolution))
    a=max(0,r0-rad);b=min(w.hard.shape[0],r1+rad)
    d=max(0,c0-rad);e=min(w.hard.shape[1],c1+rad)
    line=np.zeros((b-a,e-d),np.uint8)
    rr,cc=w.cells(controller.reference_plan.world_xy)
    cv2.polylines(line,[np.column_stack((cc-d,rr-a)).astype(np.int32)],False,1,1)
    near=cv2.dilate(line,np.ones((2*rad+1,2*rad+1),np.uint8))
    ref=bool(np.any(added[r0:r1,c0:c1] & near[r0-a:r1-a,c0-d:c1-d].astype(bool)))
    scope=bool(np.any(added[r0:r1,c0:c1] & controller._search_mask[r0:r1,c0:c1]
                      & controller.plan.corridor_mask[r0:r1,c0:c1]))
    return {'kind':'reference_affected' if ref else ('scope_only' if scope else 'unrelated'),
            'reference_affected':ref,'scope_affected':scope,'reference_influence_radius_m':radius}


class FullRouteService:
    def __init__(self,world,topology,cache_root,views,session,auditor,spec,*,max_region_routes=3):
        from .v9_maps import prepare_world
        prepare_world(world)
        from .v9_options import enabled as v9_enabled
        if v9_enabled('A') and 1 in views:
            # The fine view was built before service initialization. Rebind
            # its identical static data instead of retaining the old buffers.
            views[1].labels=world.labels;views[1].hard=world.hard
        self.world=world;self.topology=topology;self.root=cache_root;self.views=views
        self.session=session;self.auditor=auditor;self.spec=spec
        from .v10_storage import shared_static
        static=shared_static(world)
        self.dynamic=DynamicObstacles(world,static_hard=static);self.max_region_routes=max_region_routes
        if v9_enabled('A'):
            from .v9_maps import seal
            self.dynamic.mask=seal(self.dynamic.mask)
            from .v9_r9_buffers import initial_static_hard
            if static is None:self.dynamic.static_hard=initial_static_hard(world.hard,self.dynamic.static_hard)
        self.lock=threading.RLock();self.query=None;self.controller=None;self.route=None
        self.record=None;self.events=deque(maxlen=32)
        from .recovery_options import enabled as recovery_enabled
        self.r5_heap_preparation={'enabled':recovery_enabled('H')}
        if recovery_enabled('H'):
            from .recovery_heap import release_preparation_heap
            self.r5_heap_preparation.update(release_preparation_heap())

    def _remember(self,event):
        # Full diagnostics belong in the caller's log, not in resident state.
        fields=('update','action','replanned','impact','success','trajectory_available','failure_code','wall_ms')
        self.events.append({k:deepcopy(event[k]) for k in fields if k in event})

    def _solve(self,query):
        old=self.controller
        if old is not None:
            old.trajectory=None;old.trajectory_version=None;old.lifecycle.clear()
        self.controller=None;self.route=None;self.record=None;self.query=query
        from .request_cost import enabled
        if enabled('R'):
            # The former executable trajectory was invalidated above. Drop
            # publication ownership before constructing a new task controller.
            # Current map/ACK state is retained and checked by the session.
            if getattr(self.session,'active_controller',None) is old:
                self.session.active_controller=None;self.session.active_route=None
            old=None
        from .localfix_options import enabled as localfix_enabled
        if localfix_enabled('D'):
            from .source_retirement import retire
            retire(self.session)
        c,r,out=V5Runner(self.world,self.topology,self.root,self.views,self.dynamic,
            controller_class=V5Controller,max_region_routes=self.max_region_routes).plan(
                query,self.session,self.auditor,self.spec)
        self.controller=c;self.route=r;self.query=query
        self.record=record_from(out)
        # Old dynamic inflation arrays must not accumulate per update.
        for key in list(self.views):
            if isinstance(key,tuple) and key and key[0]=='v5_r4_conservative_circle' and key[-1]!=self.dynamic.version:
                self.views.pop(key)
        return out

    def plan(self,query):
        with self.lock:
            return self._solve(query)

    def _impact(self,controller,added,bbox):
        return impact(controller,added,bbox)

    def current_trajectory(self):
        # Do not wait for a slow replan: a consumer can immediately observe
        # invalidation after apply() advances the obstacle version.
        with self.dynamic.lock:
            c=self.controller
            if c is None or self.record is None or c.trajectory_version!=self.dynamic.version:return None
            p=c.trajectory
            return None if p is None else p.copy()

    def update(self,sequence,*,upsert=None,clear=(),observed_free=False):
        started=time.monotonic()
        with self.lock:
            before=self.world.hard
            update=self.dynamic.apply(sequence,upsert=upsert,clear=clear,observed_free=observed_free)
            from .v9_options import enabled as v9_enabled
            if v9_enabled('A'):
                from .v9_maps import seal
                self.world.hard=seal(self.world.hard)
                self.dynamic.mask=seal(self.dynamic.mask)
            event={'update':update,'whole_route':True,'global_fallback':False}
            if not update['accepted'] or not update.get('changed_cells',0):
                event.update(action='no_change',replanned=False,trajectory_available=self.current_trajectory() is not None)
                event['wall_ms']=(time.monotonic()-started)*1000;self._remember(event)
                return event
            c=self.controller
            if c is not None:c.trajectory=None;c.trajectory_version=None
            checker=SweptChecker(self.world.hard,self.world.map.resolution,
                self.world.map.origin,*self.world.vehicle[:2])
            # The frozen map-only adapter exposes checker as a read-only lazy
            # property; refresh its backing value without modifying L1 code.
            if getattr(self.world,'v7_map_only',False):self.world._checker=checker
            else:self.world.checker=checker
            added=self.world.hard & ~before
            if c is None or self.record is None:
                decision={'kind':'no_valid_path','reference_affected':True,'scope_affected':True}
            else:decision=self._impact(c,added,update.get('dirty_bbox'))
            event['impact']=decision
            out=None
            if self.record is not None and not decision['reference_affected']:
                # Keep only geometry. Rebuild the bounded map for the actual
                # new version; exact ACK and original whole-path audit follow.
                c._soft_grid_cache.clear();c._bounded_grid=None
                c.reference_plan=replace(c.reference_plan,occupancy_version=str(self.dynamic.version))
                out=validated_reuse(c,self.query,self.session,self.auditor,deepcopy(self.record))
                event.update(action='revalidate_existing_route',replanned=False)
                if out['success']:self.record=record_from(out)
            infrastructure=out is not None and any(s in out.get('failure_code','') for s in ('ACK','VERSION','BINDING'))
            if self.query is not None and (out is None or (not out['success'] and not infrastructure)):
                out=self._solve(self.query)
                event.update(action='replan_affected_route',replanned=True)
            if out is not None and not out['success']:
                self.record=None
                if self.controller is not None:
                    self.controller.trajectory=None;self.controller.trajectory_version=None
            if out is None:event.update(action='await_query',replanned=False)
            event.update(success=bool(out and out.get('success')),trajectory_available=self.current_trajectory() is not None,
                failure_code='' if out is None else out.get('failure_code',''),
                diagnostics={} if out is None else out.get('diagnostics',{}),
                recovery={} if out is None else out.get('candidate_recovery',{}),
                wall_ms=(time.monotonic()-started)*1000)
            self._remember(event)
            return event

    def close(self):
        with self.lock:
            if self.controller is not None:self.controller.lifecycle.clear()
            self.controller=None;self.record=None;self.views.clear();self.events.clear()
            from .v9_maps import clear
            clear()
