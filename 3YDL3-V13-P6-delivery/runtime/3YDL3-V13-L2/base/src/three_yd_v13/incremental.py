"""Fixed-domain D* repair. No L1 or L3 solver changes.

Dynamic additions are projected with the existing circle-vs-cell inflation.
The static geometry is retained; removals reopen only its original states.
Outside-domain/new-road recovery is an explicit cold reconstruction.
"""
from dataclasses import replace
import math
import time
import numpy as np
from three_yd_v5.controller import V5Controller, inflate_circle
from .state import NativeIncrementalState
from three_yd_v5.geometry import corners
from .shortcut import shortcut_path
from three_yd_v5.r6_cache import RoiGrid
from arena_3d_v1.multires_grid import aggregate


class IncrementalController(V5Controller):
    def _restore_static_route(self):
        # A mutable D* state belongs to one task, never to a route-result cache.
        return False

    def _store_static_route(self):
        pass

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        # Initial navigation retains the parent's native initialization. Both
        # event arms subsequently use this identical native D* implementation,
        # including removal events with zero blocked cells.
        if not getattr(self,'fresh_astar',False):self.l2.state.__class__=NativeIncrementalState
        from .search import EventDStar
        if not getattr(self,'fresh_astar',False):self.l2.__class__=EventDStar
        self.v13_initial_dynamic_version=0 if self.dynamic is None else self.dynamic.version
        self.v13_graph_epoch=0
        self.v13_binding=self._v13_binding()

    def _v13_binding(self):
        return (self.world.key,tuple(self.world.vehicle),tuple(self.topology_route.start),
            tuple(self.topology_route.goal),self.topology_route.signature,self.weight,self.v12_policies.fingerprint)

    def update_dynamic(self,*,cold=False):
        tick=time.monotonic();phase=tick;phases={};planner=self.l2;old=planner.state
        if self._v13_binding()!=self.v13_binding:
            return {'success':False,'failure_code':'V13_L2_BINDING_CHANGED','state_identity_reused':False}
        self.trajectory=None;self.trajectory_version=None
        self._soft_grid_cache.clear();self._bounded_grid=None;self._search_mask=None;self._v9_target=None
        project=time.monotonic();phases['preamble_ms']=(project-phase)*1000
        radius=math.hypot(*self.world.vehicle[:2])
        from .projection import blocked_state_ids
        rc=np.column_stack((old._rows,old._cols)).astype(np.int32)
        ids,projection_detail=blocked_state_ids(self.world,self.dynamic,rc[:,0],rc[:,1])
        projection_ms=(time.monotonic()-project)*1000
        phases['projection_ms']=projection_ms;phase=time.monotonic()
        before_bytes=planner.state_memory_bytes()
        if cold:
            # Release previous g/rhs/OPEN before constructing cold state. Only
            # immutable graph, preference and direction inputs remain shared.
            args=(planner.geometry,old.binding,old.potential,old.weight,old.xy_direction,old.progress)
            planner.state=None;old=None
            planner.state=NativeIncrementalState(*args)
        state=planner.state;update_tick=time.monotonic()
        phases['state_prepare_ms']=(update_tick-phase)*1000
        if __import__('os').environ.get('V13_CAPTURE_TRACE')=='1':
            changed_ids=set(map(int,np.flatnonzero(state.blocked))).symmetric_difference(ids)
            affected=set(changed_ids)
            for node in changed_ids:affected.update(planner.geometry.neighbor_ids(node))
            state.v13_changed_ids=sorted(changed_ids);state.v13_affected_ids=sorted(affected)
        changed=state.set_blocked_ids(ids)
        update_ms=(time.monotonic()-update_tick)*1000
        # Equal update budgets in both arms, no fresh-navigation allowance.
        state.ready=True
        phases['vertex_update_ms']=update_ms;phase=time.monotonic()
        result=planner.event_solve(changed)
        phases['solve_ms']=(time.monotonic()-phase)*1000;phase=time.monotonic()
        self.initial_l2_result=result
        detail={'success':bool(result.success),'failure_code':result.failure_code,
            'arm':'H-A' if getattr(self,'fresh_astar',False) else ('H-C' if cold else 'H-I'),'state_identity_reused':state is old,
            'state_count':planner.geometry.state_count,'changed_state_count':changed,
            'blocked_state_count':len(ids),'projection_ms':projection_ms,'projection':projection_detail,'vertex_update_ms':update_ms,
            'dstar_expanded':result.diagnostics.get('dstar_expanded',0),
            'dstar_unique_expanded':getattr(state,'v13_unique_expanded',None),
            'dstar_stats':vars(result.dstar_stats),
            'astar_stats':None if result.fallback_stats is None else {k:v for k,v in vars(result.fallback_stats).items() if k not in ('path','cost')},
            'search_state_reused':not cold and not getattr(self,'fresh_astar',False),
            'geometry_hash':planner.geometry.binding.digest,'cost_hash':state.cost_hash,
            'blocked_hash':__import__('hashlib').sha256(state.blocked.tobytes()).hexdigest(),
            'start_id':state.start_id,'goal_id':state.goal_id,
            'memory_estimate_excludes_native_temporary_queue_workspace':True,
            'astar_expanded':result.diagnostics.get('astar_expanded',0),
            'path_cost':result.diagnostics.get('path_cost'),
            'backend':result.selected_backend,
            'state_bytes_before':before_bytes,'state_bytes_after':planner.state_memory_bytes(),
            'graph_epoch':self.v13_graph_epoch,'map_version':self.dynamic.version,
            'static_domain_version':self.v13_initial_dynamic_version,
            'solver':'native fresh A*' if getattr(self,'fresh_astar',False) else 'native resumable D*',
            'cost_model':'unchanged route preference and directed static edges; dynamic hard blocks',
            'budgets_s':({'fresh_astar':2.5} if getattr(self,'fresh_astar',False) else {'dstar':.5,'astar_fallback':2.}),'dstar_node_budget':80000}
        detail['repair_work']=getattr(state,'v13_repair_work',{})
        detail['native_segments']=__import__('os').environ.get('V13_NATIVE_SEGMENTS','1')=='1'
        detail['search_internal_ms']=getattr(state,'v13_search_internal_ms',{})
        detail['solve_phase_ms']=result.diagnostics.get('phase_ms',{})
        detail['extraction_backend']=result.diagnostics.get('extraction_backend','unknown')
        phases['diagnostics_ms']=(time.monotonic()-phase)*1000;phase=time.monotonic()
        if result.success:
            raw=np.asarray(planner.path_global,int)
            self.raw_reference_xy=self.view.world_points(raw[:,0],raw[:,1])
            safe=np.zeros(self.view.safe.shape,bool)
            safe[rc[:,0],rc[:,1]]=~state.blocked
            self.view.safe=safe;self.view.hard=~safe
            r0,r1,c0,c1=planner.roi.bbox
            direction_local=np.zeros((r1-r0,c1-c0,2),float)
            direction_local[rc[:,0]-r0,rc[:,1]-c0]=state.xy_direction
            direction=RoiGrid(direction_local,(r0,r1,c0,c1),(*safe.shape,2),0.)
            phases['reference_masks_ms']=(time.monotonic()-phase)*1000;phase=time.monotonic()
            nodes=shortcut_path(raw,safe,self.preference.potential,self.weight,direction)
            xy=self.view.world_points(nodes[:,0],nodes[:,1])
            xy[0]=self.topology_route.start[:2];xy[-1]=self.topology_route.goal[:2]
            xy=xy[np.r_[True,np.linalg.norm(np.diff(xy,axis=0),axis=1)>1e-7]]
            self.corner_records=corners(xy)
            if len(xy)>1:
                arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
                for i,d,yaw,s in [(0,xy[1]-xy[0],self.topology_route.start[2],0.),
                                  (len(xy)-1,xy[-1]-xy[-2],self.topology_route.goal[2],float(arc[-1]))]:
                    a=abs(math.degrees(math.atan2(math.sin(math.atan2(d[1],d[0])-yaw),math.cos(math.atan2(d[1],d[0])-yaw))))
                    if a>=30.:self.corner_records.append({'index':i,'position':xy[i].tolist(),'turn_angle_deg':a,'arc_m':s,'kind':'endpoint_heading_mismatch'})
                self.corner_records.sort(key=lambda x:x['arc_m'])
            phases['shortcut_and_corners_ms']=(time.monotonic()-phase)*1000;phase=time.monotonic()
            from three_yd_v5.v10_storage import retained_mask
            physical=retained_mask(self.global_drivable&~self.world.hard)
            self.plan=replace(self.plan,corridor_mask=physical)
            self.pending_l3_mask=physical;self.server_l3_mask=physical;self._pending_l3_hash=''
            phases['physical_mask_ms']=(time.monotonic()-phase)*1000;phase=time.monotonic()
            self._set_reference(xy)
            from three_yd_v5.v12_global_l2 import bind_reference_traffic
            from three_yd_v5.v12_regional import accepted_reference
            bind_reference_traffic(self,xy);accepted_reference(self,xy)
            detail['reference_length_m']=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum())
            phases['reference_bind_ms']=(time.monotonic()-phase)*1000
        detail['wall_ms']=(time.monotonic()-tick)*1000
        phases['unaccounted_ms']=detail['wall_ms']-sum(phases.values())
        detail['phase_ms']=phases
        self.v13_last_update=detail
        return detail

from .astar import AStarGraph,FreshAStar
class AStarController(IncrementalController):
    fresh_astar=True
    l2_state_class=AStarGraph
    l2_planner_class=FreshAStar
