"""Task-level continuation of a verified COMPLETE route; never a rolling window.

The frozen L1 is only called for a fresh task or an incompatible start. A start
on the last delivered trajectory carries the already traversed traffic stages.
The remaining trajectory is independently rechecked for the actual pose, body,
curvature, steering and current acknowledged costmap on EVERY delivery.
"""
from copy import deepcopy
from dataclasses import replace
import os
import time
import numpy as np
from .fullroute_service import FullRouteService
from .fullroute import record_from, validated_reuse


def wrap(v):
    return (v+np.pi) % (2*np.pi)-np.pi


def match_pose(path, pose, position_tolerance=1e-7, yaw_tolerance=1e-7):
    """No nearest-road assignment, snapping or interpolation of vehicle pose."""
    p=np.asarray(path,float);q=np.asarray(pose,float)
    hits=np.flatnonzero((np.linalg.norm(p[:,:2]-q[:2],axis=1)<=position_tolerance)
                       & (np.abs(wrap(p[:,2]-q[2]))<=yaw_tolerance))
    if not len(hits):return None,'POSE_NOT_ON_REMAINING_ROUTE'
    # Repeated same-pose vertices are harmless; a later visit is ambiguous.
    if len(hits)>1 and np.any(np.diff(hits)>1):return None,'AMBIGUOUS_ROUTE_VISIT'
    return int(hits[0]),'EXACT_POSE_MATCH'


class ProgressTraffic:
    """Apply the unchanged task traffic audit with the committed visit prefix.

    Only traffic history is prepended. Vehicle geometry is always audited on
    the remaining CURRENT trajectory. A passed past segment never excuses a
    new direction/parking/order violation in the suffix.
    """
    def __init__(self, base, prefix):
        self.base=base;self.prefix=np.asarray(prefix,float).copy()

    def __getattr__(self,name):return getattr(self.base,name)

    def audit(self, poses):
        tail=np.asarray(poses,float)
        full=np.concatenate((self.prefix,tail)) if len(self.prefix) else tail
        result=dict(self.base.audit(full))
        result['task_progress']={'committed_prefix_vertices':len(self.prefix),
            'remaining_vertices':len(tail),'original_traffic_audit':True,
            'parking_binding_inherited_from_accepted_task':True}
        return result


class TaskProgressService(FullRouteService):
    def __init__(self,*args,continuation=True,local_connector=True,**kwargs):
        super().__init__(*args,**kwargs)
        self.continuation=continuation;self.local_connector=local_connector
        self.task_query=None;self.prefix=np.empty((0,3))
        self.remaining=None;self.base_traffic=None;self.progress_m=0.
        self.retained_record=None;self.retained_path=None

    def _solve(self,query):
        if (self.continuation and getattr(self,'_in_dynamic_update',False) and self.task_query is not None
                and self.controller is not None and len(self.prefix)>0
                and tuple(query.goal)==tuple(self.task_query.goal)):
            # The selected task and its passed parking/road stages remain
            # valid. Changed obstacles are hard in the new costmap. Try the
            # original bounded L3 once on the current suffix; no frozen-L1
            # reinterpretation of an already entered parking endpoint.
            c=self.controller;tick=time.monotonic()
            from .curve_backend import identity,displaced_continuation
            if identity()[1]!='control' and self.remaining is not None and self.record is not None:
                start=np.asarray(query.start,float);actual=self.remaining[0]
                if np.linalg.norm(start[:2]-actual[:2])>1e-8 or abs(wrap(start[2]-actual[2]))>1e-8:
                    # A failed pose request may precede this obstacle clear.
                    # Restore task history, then join from that actual pose;
                    # the old retained curve is only an initialization.
                    c._soft_grid_cache.clear();c._bounded_grid=None
                    c.reference_plan=replace(c.reference_plan,occupancy_version=str(self.dynamic.version))
                    displaced=displaced_continuation(self,query)
                    if displaced is not None:return displaced
            if self.remaining is not None:c._curve_progress_guide=self.remaining[:,:2].copy()
            c._curve_progress_model=None if self.record is None else self.record.get('curve_model')
            c._curve_obstacle_repair=True
            self.record=None;self.remaining=None
            c.trajectory=None;c.trajectory_version=None;c.timing={'adapter_calls':[]}
            c._soft_grid_cache.clear();c._bounded_grid=None
            c.reference_plan=replace(c.reference_plan,occupancy_version=str(self.dynamic.version))
            c.traffic=ProgressTraffic(self.base_traffic,self.prefix)
            c.r13_prefix_connector=None;c.r13_remainder_mask=None
            self.session.active_controller=c;self.session.active_route=self.route
            from three_yd_v4.r5_controller import SoftEdgeController
            mask=c._target_mask();count=int(mask.sum())
            limit=min(c.max_search_cells,int(mask.size*c.max_map_fraction))
            from .curve_backend import identity
            dynamic_planner=SoftEdgeController.plan_l3 if identity()[1]=='control' else type(c).plan_l3
            try:
                out=({'success':False,'failure_code':'L3_SPATIAL_BUDGET'} if count>limit else
                    dynamic_planner(c,query,self.session,self.auditor,self.spec))
            finally:
                c._curve_progress_model=None;c._curve_obstacle_repair=False
            out['bounded_l3']={'search_cells':count,'limit_cells':limit,
                'map_fraction':count/mask.size,'full_map_cells':mask.size,'global_fallback':False}
            pipeline=(time.monotonic()-tick)*1000
            out['candidate_recovery']={'l1_actual_ms':0.,'all_candidates_controller_ms':0.,
                'all_candidates_adapter_ms':c.timing.get('adapter_actual_ms',0.),
                'all_candidates_l3_pipeline_ms':pipeline,'global_fallback':False,
                'attempts':[{'success':bool(out.get('success')),'l2_timing':dict(c.timing),
                    'l3_diagnostics':out.get('diagnostics',{}),'l3_pipeline_actual_ms':pipeline}]}
            out['task_progress']={'reused':False,'retained_task_context':True,
                'action':'bounded_L3_repair_after_obstacle_change','L1_search_called':False,
                'L2_search_called':False,'current_start':query.start,'fixed_goal':query.goal,
                'soft_reference_retained':True,'full_remaining_route':True}
            self.record=record_from(out)
            if self.record is not None:self.remaining=self.current_trajectory()
            return out
        # An independent global request has no inferred ownership or history.
        self.task_query=None;self.prefix=np.empty((0,3));self.remaining=None
        self.base_traffic=None;self.progress_m=0.
        self.retained_record=None;self.retained_path=None
        out=super()._solve(query)
        if out.get('success'):
            self.task_query=query;self.remaining=self.current_trajectory()
            self.base_traffic=self.controller.traffic
        return out

    def plan(self,query):
        from .curve_budget import request_guard
        from .curve_backend import settings
        with request_guard(settings()[1]['total_request_budget_s']):return self._guarded_plan(query)

    def _guarded_plan(self,query):
        try:
            from .curve_backend import request_start,request_finish,settings,settings_scope
            from .tail_filehash import request_scope
            with request_scope(),settings_scope():
                request_start(self)
                from .endpoint_tolerance import execute
                out=execute(self,query,self._plan)
                out=request_finish(self,out)
                if out.get('success'):self._retain()
                # Release only unused allocator pages after all request-local
                # temporary solver fields have returned. This remains inside
                # the navigation clock and the same outer request deadline.
                if settings()[0]=='independent':
                    from .request_heap import release_retired_heap
                    out['r11_request_heap']=release_retired_heap()
                return out
        except BaseException:
            # A rejected/aborted current-pose request must never expose the
            # previous start's executable path through current_trajectory().
            self.record=None;self.remaining=None
            if self.controller is not None:
                self.controller.trajectory=None;self.controller.trajectory_version=None
            raise

    def _plan(self,query):
        with self.lock:
            from .curve_backend import identity
            if (identity()[1]!='control' and self.record is None
                    and getattr(self,'retained_record',None) is not None
                    and self.task_query is not None and query.query_id==self.task_query.query_id
                    and tuple(query.goal)==tuple(self.task_query.goal)):
                # Retain task ownership after a failed displaced request. This
                # restores an untrusted candidate only; trajectory/version
                # remain invalid until current ACK and every audit pass.
                from .curve_records import copy_internal_record
                self.record=copy_internal_record(self.retained_record)
                self.remaining=self.retained_path.copy()
                self.controller.traffic=ProgressTraffic(self.base_traffic,self.prefix)
            same=(self.continuation and self.record is not None and self.remaining is not None
                and self.task_query is not None and query.query_id==self.task_query.query_id
                and tuple(query.goal)==tuple(self.task_query.goal))
            reason='NEW_TASK' if not same else None
            if same:
                index,reason=match_pose(self.remaining,query.start)
                if (self.query is not None and tuple(query.start)==tuple(self.query.start)
                        and self.controller.trajectory is not None):
                    # An identical request may reuse the SAME already audited
                    # endpoint residual permitted by the original planner.
                    # This is not a new displaced pose or an increased tolerance.
                    index,reason=0,'IDENTICAL_CURRENT_REQUEST'
                tail=None;connector=None;proposal_started=time.monotonic()
                if index is None and getattr(self,'local_connector',False) and identity()[1]=='control':
                    from .progress_connector import propose
                    proposal,reason=propose(self,query)
                    if proposal is not None:index,tail,connector=proposal
                if index is None and identity()[1]!='control':
                    from .curve_backend import displaced_continuation
                    displaced=displaced_continuation(self,query)
                    if displaced is not None:return displaced
                if index is not None:
                    c=self.controller;tick=time.monotonic()
                    old_timing=deepcopy(c.timing);c.timing={'adapter_calls':[]}
                    prefix=np.concatenate((self.prefix,self.remaining[:index]))
                    c.traffic=ProgressTraffic(self.base_traffic,prefix)
                    from .curve_records import copy_internal_record,FrozenCurve
                    record=copy_internal_record(self.record)
                    if tail is None:
                        points=record['result'].points[index:]
                        if record.get('curve_model') is not None:
                            model=record['curve_model']
                            if type(model) is FrozenCurve:record['curve_model']=model.suffix(index)
                            else:model['parameters']=model['parameters'][index:]
                    else:
                        from arena_3d_v1.curvature_repair import truthful_points
                        points=truthful_points(tail,record['result'],0,len(tail)-1)
                    record['result']=replace(record['result'],points=points)
                    self.session.active_controller=c;self.session.active_route=self.route
                    out=validated_reuse(c,query,self.session,self.auditor,record)
                    pipeline_ms=(time.monotonic()-proposal_started)*1000
                    adapter_ms=c.timing.get('adapter_actual_ms',0.)
                    out['candidate_recovery']={'l1_actual_ms':0.,'all_candidates_controller_ms':0.,
                        'all_candidates_l3_pipeline_ms':pipeline_ms,'all_candidates_adapter_ms':adapter_ms,
                        'global_fallback':False,'attempts':[{'success':out['success'],
                        'controller_actual_ms':0.,'l3_pipeline_actual_ms':pipeline_ms,
                        'adapter_actual_ms':adapter_ms,'l2_timing':dict(c.timing),
                        'l3_diagnostics':out.get('diagnostics',{})}]}
                    out['task_progress']={'attempted':True,'reused':bool(out['success']),
                        'matched_vertex':index,'match':reason,'task_start':self.task_query.start,
                        'current_start':query.start,'fixed_goal':query.goal,
                        'full_remaining_route':True,'L1_search_called':False,'L2_search_called':False,
                        'committed_prefix_vertices':len(prefix),'source_clocks_reused':False}
                    out['task_progress']['connector']=connector
                    if out['success']:
                        self.progress_m+=float(np.linalg.norm(np.diff(self.remaining[:index+1,:2],axis=0),axis=1).sum())
                        internal_points=(self.record['result'].points if index==0 and tail is None
                            and type(record.get('curve_model')) is FrozenCurve else None)
                        self.prefix=prefix;self.query=query
                        self.record=record_from(out) if internal_points is None else record_from(out,internal_points)
                        self.remaining=self.current_trajectory()
                        out['task_progress']['progress_m']=self.progress_m
                        return out
                    c.timing=old_timing
                    c.traffic=self.base_traffic
                    # ACK/version errors never trigger a blind search retry.
                    if any(x in out.get('failure_code','') for x in ('ACK','VERSION','BINDING')):
                        self.record=None;self.remaining=None;return out
                    reason='STRICT_REUSE_REJECTED:'+out.get('failure_code','')
            out=self._solve(query)
            out['task_progress']={'attempted':False,'reused':False,'reason':reason,
                'full_remaining_route':True,'L1_search_called':True,'L2_search_called':True}
            return out

    def update(self,*args,**kwargs):
        from .curve_budget import request_guard
        from .curve_backend import request_start,request_finish,settings,settings_scope
        try:
            with request_guard(settings()[1]['total_request_budget_s']),settings_scope():
                request_start(self)
                out=self._guarded_update(*args,**kwargs)
                return request_finish(self,out)
        except BaseException:
            # A rejected obstacle-clear command must keep the old untrusted
            # task candidate for later recertification, as in the parent.
            # It still must not expose a currently executable trajectory.
            self.record=None;self.remaining=None
            if self.controller is not None:
                self.controller.trajectory=None;self.controller.trajectory_version=None
            raise

    def _guarded_update(self,*args,**kwargs):
        with self.lock:
            requested=getattr(self,'requested_query',self.query)
            if self.record is None and self.retained_record is not None:
                # This remains only a candidate. trajectory/version stay
                # invalid until current ACK + all original audits pass.
                from .curve_records import copy_internal_record
                self.record=copy_internal_record(self.retained_record)
                self.remaining=self.retained_path.copy()
                self.controller.traffic=ProgressTraffic(self.base_traffic,self.prefix)
            self._in_dynamic_update=True
            try:out=super().update(*args,**kwargs)
            except BaseException:
                self.record=None;self.remaining=None
                if self.controller is not None:
                    self.controller.trajectory=None;self.controller.trajectory_version=None
                raise
            finally:self._in_dynamic_update=False
            from .endpoint_tolerance import after_update
            try:out=after_update(self,out,requested)
            except BaseException:
                from .endpoint_tolerance import withdraw
                withdraw(self)
                raise
            if out.get('success'):self.remaining=self.current_trajectory()
            if out.get('success'):self._retain()
            if not out.get('trajectory_available'):
                self.remaining=None;self.record=None
            return out

    def _retain(self):
        from .curve_records import record_bytes,copy_internal_record
        if self.record is not None and self.remaining is not None and record_bytes(self.record)<8*2**20:
            self.retained_record=copy_internal_record(self.record);self.retained_path=self.remaining.copy()

    def _impact(self,controller,added,bbox):
        if self.continuation and self.remaining is not None:
            from types import SimpleNamespace
            from .fullroute_service import impact
            proxy=SimpleNamespace(world=controller.world,reference_plan=replace(
                controller.reference_plan,world_xy=self.remaining[:,:2]),
                _search_mask=controller._search_mask,plan=controller.plan)
            decision=impact(proxy,added,bbox)
            decision['evaluated_route']='current_remaining_trajectory'
            return decision
        return super()._impact(controller,added,bbox)

    def close(self):
        self.remaining=None;self.prefix=np.empty((0,3));self.base_traffic=None;self.task_query=None
        self.retained_record=None;self.retained_path=None
        super().close()
