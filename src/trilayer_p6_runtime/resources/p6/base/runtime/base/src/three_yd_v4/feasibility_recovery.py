"""r2: strict-first planning plus one evidence-driven reference recovery.

No frozen input or final acceptance change. Nothing is published until the
whole canonical/independent/traffic checks pass on the current map version.
"""
from pathlib import Path
import time
import numpy as np
from .strict_baseline import SoftEdgeController as FrozenStrictController
from .feedback_reference import FeedbackReferenceController
from .diagnostics import localize,feedback_from_evidence
from .local_l3 import reference_poses
from arena_3d_v1.semantic_world import digest


class StrictController(FrozenStrictController):
    def _search(self,factor,clearance,radius=None,guide=None,only_guide=False):
        if factor==1:
            view=self._view(factor,clearance);scope=self._scope(view,radius,guide)
            if int(scope.sum())>120000:
                self.timing['activation_attempts'].append({'resolution':view.map.resolution,'clearance_filter':clearance,
                    'scope_radius_m':radius,'scope_cells':int(scope.sum()),'success':False,
                    'failure':'FINE_STATE_BUDGET_BEFORE_ALLOCATION','not_vehicle_feasibility_proof':True})
                return None
        return super()._search(factor,clearance,radius,guide,only_guide)


class FeasibilityRecoveryController:
    def __init__(self,world,topology,route,*,cache_root,views=None,**kwargs):
        self._published=None;self._published_version=None
        self._feedback_enabled=kwargs.pop('feedback_enabled',False)
        self._args=(world,topology,route);self._kwargs=dict(cache_root=cache_root,views=views,**kwargs)
        self._active=StrictController(*self._args,**self._kwargs)
    def __getattr__(self,name):return getattr(self._active,name)
    def current_trajectory(self):
        dynamic=getattr(self._active,'dynamic',None)
        if dynamic and dynamic.version!=self._published_version:return None
        return None if self._published is None else self._published.copy()
    @property
    def trajectory(self):return self.current_trajectory()
    def update_dynamic(self,*args,**kwargs):
        self._published=None;self._published_version=None
        return self._active.update_dynamic(*args,**kwargs)
    def _publish(self,out):
        if out.get('success'):
            self._published=self._active.current_trajectory();self._published_version=self._active.trajectory_version
            out['legacy_contract_success']=True
    def plan_l3(self,query,session,auditor,spec,**kwargs):
        started=time.monotonic();self._published=None;self._published_version=None
        out=self._active.plan_l3(query,session,auditor,spec,**kwargs)
        log={'strategy':'strict_first_then_query_local_reference_feedback','strict_first_success':bool(out.get('success')),
             'feedback_attempted':False,'persistent_topology_mutations':0,'max_feedback_attempts':1,'initial_failure':out.get('failure_code','')}
        if out.get('success'):
            self._publish(out);out['feasibility_recovery']=log;return out
        if any(v in str(out.get('failure_code','')).upper() for v in ['ACK','CONTENT','BINDING','VERSION','CORRUPT']):
            log['skipped']='INPUT_OR_VERSION_FAILURE';out['feasibility_recovery']=log;return out
        original=self._active
        result=out.get('result')
        if result and result.points:
            p=np.asarray([[v['x'],v['y'],v['yaw']] for v in result.points]);source='returned_rejected_candidate'
        elif self._feedback_enabled:
            p=reference_poses(original.reference_plan.world_xy,query.start,query.goal);source='L2_geometry_proposal_not_vehicle_path'
        else:
            log.update(skipped='EXPERIMENTAL_REFERENCE_FEEDBACK_DISABLED',
                native_termination_reason=out.get('diagnostics',{}).get('termination_reason'),
                physical_blockage_proven=False)
            out['feasibility_recovery']=log;return out
        evidence=localize(original.world,query,p,original._target_mask())
        zones=feedback_from_evidence(evidence,p)
        log.update(evidence=evidence,evidence_source=source,feedback_zones=zones,initial_reference_hash=digest(original.reference_cells))
        if not self._feedback_enabled:
            log['skipped']='EXPERIMENTAL_REFERENCE_FEEDBACK_DISABLED'
            out['feasibility_recovery']=log;return out
        options=dict(self._kwargs);options['cache_root']=Path(options['cache_root'])/'r2_feedback';options['feedback_zones']=zones
        original.trajectory=None;original.trajectory_version=None
        try:
            candidate=FeedbackReferenceController(*self._args,**options)
        except (RuntimeError,ValueError,TimeoutError) as error:
            if any(v in str(error).upper() for v in ['CORRUPT','BINDING','VERSION']):raise
            log.update(feedback_failure=str(error),wall_ms=(time.monotonic()-started)*1000)
            out['feasibility_recovery']=log;return out
        changed=digest(candidate.reference_cells)!=log['initial_reference_hash']
        log.update(feedback_reference_changed=changed,feedback_reference_hash=digest(candidate.reference_cells))
        if not changed:
            candidate.lifecycle.clear();log['skipped']='UNCHANGED_REFERENCE_NO_DUPLICATE_L3';out['feasibility_recovery']=log;return out
        original.lifecycle.clear();self._active=candidate;log['feedback_attempted']=True
        recovered=candidate.plan_l3(query,session,auditor,spec,**kwargs)
        log.update(feedback_success=bool(recovered.get('success')),feedback_failure=recovered.get('failure_code',''),wall_ms=(time.monotonic()-started)*1000)
        recovered['feasibility_recovery']=log;self._publish(recovered)
        return recovered
