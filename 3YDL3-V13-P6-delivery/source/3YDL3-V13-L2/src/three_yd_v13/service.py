"""V13 obstacle-event adapter: update L2 before unchanged L3/ACK/audits."""
import time
import numpy as np
from arena_3d_v1.semantic_world import SweptChecker
from three_yd_v5.task_progress import TaskProgressService
from three_yd_v5.recovery import V5Runner
from three_yd_v5.fullroute import record_from
from .incremental import IncrementalController,AStarController


class IncrementalService(TaskProgressService):
    def __init__(self,*args,arm='H-I',**kwargs):
        if arm not in ('H-I','H-C','H-A'):raise ValueError('INVALID_V13_ARM')
        super().__init__(*args,continuation=False,**kwargs)
        self.arm=arm;self.v13_attempts=[]

    def _solve(self,query):
        if getattr(self,'_p5_active',False):
            from .schedule import solve
            return solve(self,query)
        return self._p4_solve(query)

    def _guarded_update(self,*args,**kwargs):
        import os
        with self.lock:
            self._p5_active=os.environ.get('V13_P5_SCHEDULE','1')=='1'
            self._p5_cache={};self._p5_attempts=[];self._p5_legacy=False
            self._p7_runtime_topology=None;self._p7_detail=None
            try:
                out=self._p4_guarded_update(*args,**kwargs)
                if self._p7_detail is not None:out['runtime_obstacle_L1']=self._p7_detail
                if self._p5_active:out['p5_schedule']={'attempts':self._p5_attempts,'cache_enabled':os.environ.get('V13_P5_CACHE','1')=='1','all_attempts_inside_request':True}
                return out
            finally:
                self.last_p7_diagnostics={'L1':self._p7_detail,'attempts':self._p5_attempts}
                if self._p7_runtime_topology is not None:self._p7_runtime_topology.release_workspaces()
                self._p7_runtime_topology=None
                self._p5_active=False;self._p5_cache={};self._p5_attempts=[]

    def _p4_solve(self,query):
        old=self.controller
        if old is not None:old.trajectory=None;old.trajectory_version=None;old.lifecycle.clear()
        self.controller=None;self.route=None;self.record=None;self.query=query
        self.remaining=None;self.retained_record=None;self.retained_path=None
        if getattr(self.session,'active_controller',None) is old:
            self.session.active_controller=None;self.session.active_route=None
        old=None
        from three_yd_v5.source_retirement import retire
        retire(self.session)
        c,r,out=V5Runner(self.world,getattr(self,'_p7_runtime_topology',None) or self.topology,self.root,self.views,self.dynamic,
            controller_class=AStarController if self.arm=='H-A' else IncrementalController,max_region_routes=self.max_region_routes).plan(
                query,self.session,self.auditor,self.spec)
        from .corridor_recovery import eligible,recover
        if eligible(self,out):
            c,r,out=recover(self,query,out,AStarController if self.arm=='H-A' else IncrementalController)
        self.controller=c;self.route=r;self.record=record_from(out)
        if out.get('success'):
            self.task_query=query;self.remaining=self.current_trajectory();self.base_traffic=c.traffic
        for key in list(self.views):
            if isinstance(key,tuple) and key and key[0]=='v5_r4_conservative_circle' and key[-1]!=self.dynamic.version:
                self.views.pop(key)
        return out

    def _p4_guarded_update(self,sequence,*,upsert=None,clear=(),observed_free=False,current_pose=None):
        from three_yd_v5.tail_filehash import request_scope
        from three_yd_v5.v12_regional import request_scope as regional_scope
        with self.lock,request_scope(),regional_scope(self):
            tick=time.monotonic()
            self.v13_domain_recoveries=[]
            pose_changed=False;query_changed=False
            if current_pose is not None:
                from .corridor_recovery import bind_current_pose
                incoming,pose_changed,query_changed=bind_current_pose(self,current_pose)
            update=self.dynamic.apply(sequence,upsert=upsert,clear=clear,observed_free=observed_free)
            event={'update':update,'v13':{'arm':self.arm,'size_does_not_prescribe_response':True},'replanned':False}
            if not update['accepted'] or (not update.get('changed_cells',0) and not query_changed):
                event.update(action='no_change',success=self.current_trajectory() is not None,
                    trajectory_available=self.current_trajectory() is not None,wall_ms=(time.monotonic()-tick)*1000)
                return event
            if current_pose is not None:
                self.query=incoming
                self.requested_query=self.query
            event['v13']['current_start']=list(self.query.start)
            event['v13']['fixed_goal']=list(self.query.goal)
            event['v13']['pose_changed']=pose_changed
            event['v13']['request_binding_changed']=query_changed
            event['v13']['corridor_recovery_attempts']=self.v13_domain_recoveries
            event['v13']['moving_start_state_reuse']=False if pose_changed else None
            c=self.controller
            old_signature=None if c is None else c.topology_route.signature
            old_regions=None if c is None else list(c.topology_route.region_ids)
            if c is not None:c.trajectory=None;c.trajectory_version=None
            self.record=None;self.remaining=None;self.retained_record=None;self.retained_path=None
            from three_yd_v5.v9_maps import seal
            self.world.hard=seal(self.world.hard);self.dynamic.mask=seal(self.dynamic.mask)
            checker=SweptChecker(self.world.hard,self.world.map.resolution,self.world.map.origin,*self.world.vehicle[:2])
            if getattr(self.world,'v7_map_only',False):self.world._checker=checker
            else:self.world.checker=checker
            # Feedback belongs to this accepted obstacle transaction. It is
            # cleared after every request, including exceptions and removals.
            import os
            if (os.environ.get('V13_P7_OBSTACLE_L1','1')=='1'
                    and getattr(self.topology,'mode',None)!='free_space'
                    and self.dynamic.obstacles):
                from .runtime_l1 import RuntimeSemanticTopology
                overlay=RuntimeSemanticTopology(self.topology,self.world,self.dynamic)
                self._p7_detail=overlay.detail
                if overlay.blocked and (old_regions is None or any(item[1] in old_regions for item in overlay.blocked)):
                    self._p7_runtime_topology=overlay
                    self._p7_detail['trigger']='blocked transition on active route region; fresh domain'
                else:
                    overlay.release_workspaces()
                    self._p7_detail['trigger']='no active-route transition blocked; original L2 retained'
            out=None;detail=None;fallback=None
            if c is not None and isinstance(c,IncrementalController) and not query_changed and self._p7_runtime_topology is None:
                if clear and c.v13_initial_dynamic_version>0:
                    detail={'success':False,'failure_code':'V13_STATIC_DOMAIN_RESTORE_AFTER_CLEAR',
                            'state_identity_reused':False}
                else:detail=c.update_dynamic(cold=self.arm=='H-C')
                event['v13']['l2']=detail
                if detail['success']:
                    l3_tick=time.monotonic()
                    out=c.plan_l3(self.query,self.session,self.auditor,self.spec)
                    event['v13']['l3_and_certification_ms']=(time.monotonic()-l3_tick)*1000
                    event.update(action='fresh_Astar_then_L3' if self.arm=='H-A' else ('cold_L2_then_L3' if self.arm=='H-C' else 'incremental_L2_then_L3'),replanned=True)
            if out is None or not out.get('success'):
                reason=(detail or {}).get('failure_code') if out is None else out.get('failure_code')
                # Infrastructure errors are not repaired by another search.
                if out is None or not any(k in str(reason) for k in ('ACK','VERSION','BINDING','CORRUPT')):
                    fallback={'reason':reason or ('CURRENT_POSE_CHANGED' if query_changed else 'NO_REUSABLE_L2_STATE'),'kind':'cold_domain_and_existing_L1_candidates',
                              'timeout_is_not_blockage_proof':True}
                    ft=time.monotonic();out=self._solve(self.query)
                    fallback['wall_ms']=(time.monotonic()-ft)*1000
                    fallback['candidate_recovery']=out.get('candidate_recovery',{})
                    event.update(action='cold_domain_recovery',replanned=True)
            if out is None:out={'success':False,'failure_code':'V13_NO_ACTIVE_QUERY'}
            self.record=record_from(out)
            success=bool(out.get('success'))
            if not success:
                self.record=None
                if self.controller is not None:self.controller.trajectory=None;self.controller.trajectory_version=None
            else:
                self.remaining=self.current_trajectory();self._retain()
            c=self.controller
            event['v13'].update(fallback=fallback,original_route_signature=old_signature,
                current_route_signature=None if c is None else c.topology_route.signature,
                original_regions=old_regions,current_regions=None if c is None else list(c.topology_route.region_ids),
                route_changed=bool(c and old_signature!=c.topology_route.signature),
                road_sequence_changed=bool(c and old_regions!=list(c.topology_route.region_ids)))
            event.update(out)
            trace=event['v13']
            from .schedule import after_update
            event=after_update(self,event,getattr(self,'requested_query',self.query))
            event['v13']=trace
            # Endpoint resolution may have selected a different query/route.
            # Report final publication identity, retaining the prior attempt.
            trace['pre_tolerance_route_signature']=trace.get('current_route_signature')
            accepted=self.controller if event.get('success') else None
            trace['current_route_signature']=None if accepted is None else accepted.topology_route.signature
            trace['current_regions']=None if accepted is None else list(accepted.topology_route.region_ids)
            trace['route_changed']=bool(accepted and old_signature!=accepted.topology_route.signature)
            trace['road_sequence_changed']=bool(accepted and old_regions!=list(accepted.topology_route.region_ids))
            requested=getattr(self,'requested_query',self.query)
            trace['current_start']=list(requested.start);trace['fixed_goal']=list(requested.goal)
            trace['resolved_start']=list(self.query.start) if accepted else None
            trace['resolved_goal']=list(self.query.goal) if accepted else None
            if event.get('success'):
                self.remaining=self.current_trajectory();self._retain()
            from three_yd_v5.v12_provenance import attach
            event=attach(self,event)
            event.update(trajectory_available=self.current_trajectory() is not None,
                wall_ms=(time.monotonic()-tick)*1000)
            self._remember(event)
            return event
