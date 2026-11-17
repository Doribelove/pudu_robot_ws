"""Observable recovery actions sharing the existing request and L3 clocks."""
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass
import os
import time


@dataclass(frozen=True)
class Phase:
    name: str = 'primary'
    relax_preference: bool = False
    widen_corridor: bool = False
    refine_l3: bool = False

    @property
    def key(self): return (self.name,self.relax_preference,self.widen_corridor,self.refine_l3)

    @property
    def geometry_key(self):
        # L3 knot/collocation changes do not change the L2 problem. Reuse
        # only its already computed geometry, never a curve, ACK or audit.
        return (self.relax_preference,self.widen_corridor)


_PHASE=ContextVar('v12_degradation_phase',default=Phase())


def current(): return _PHASE.get()


@contextmanager
def phase_scope(phase):
    token=_PHASE.set(phase)
    try:yield
    finally:_PHASE.reset(token)


def failure_stage(code):
    code=str(code).upper()
    if any(x in code for x in ('ACK','CONTENT','VERSION','BINDING','CORRUPT','INVALID_INPUT',
                              'TILE_MISSING','INDEX_MISSING','VEHICLE','FRAME_MISMATCH')):
        return 'infrastructure'
    if any(x in code for x in ('NO_LEGAL_SEMANTIC_ROUTE','NO_LEGAL_REGION_ROUTE',
                               'V7_L1_NO_FREE_SPACE_ROUTE','L1_NO_ROUTE')):return 'L1'
    if any(x in code for x in ('V5_L2_','REFERENCE_GRID_DISCONNECTED','V12_FULL_L2_')):return 'L2'
    if any(x in code for x in ('CURVE_','L3_','SWEPT_BODY','FOOTPRINT','STEERING','ROAD_SEQUENCE')):return 'L3'
    return 'input_or_unclassified'


def result_stage(out):
    found=failure_stage(out.get('failure_code',''))
    return found if found=='infrastructure' else out.get('failure_stage',found)


def _release_failed_controller(session):
    c=getattr(session,'active_controller',None)
    if c is not None:
        c.trajectory=None;c.trajectory_version=None
        if hasattr(c,'lifecycle'):c.lifecycle.clear()
    session.active_controller=None;session.active_route=None


def plan(runner,query,session,auditor,spec,solve):
    if os.environ.get('THREEYD_V12_FALLBACK','0')!='1':
        return solve(runner,query,session,auditor,spec)
    state=session._curve_request;deadline=state['deadline'];budget=state['config']['new_l3_budget_s']
    traces=[];recoveries=[];base=runner.topology;active=base
    state.setdefault('v12_degradation_runs',[]).append(traces)

    def attempt(phase):
        nonlocal active
        tick=time.monotonic();global_detail=None
        before=state['new_used_s']
        event={'phase':phase.name,'status':'running','request_deadline':deadline,
               'L3_budget_s':budget,'L3_used_before_s':before}
        traces.append(event)
        cap=None
        if phase.name!='primary':
            # Finer discretization is a short rescue before the original
            # endpoint resolver. Other geometric recoveries retain their
            # two-second allowance. Both debit the same cumulative clock.
            slice_s=.15 if phase.refine_l3 else 2.
            cap=min(budget,before+min(slice_s,max(0.,budget-before)*.25))
            session._v12_phase_L3_cap=cap
        try:
            with phase_scope(phase):
                if phase.name=='full_map_L2':
                    from .v12_global_l2 import topology_from_grid
                    active,global_detail=topology_from_grid(base,query,session,dynamic=runner.dynamic)
                    event['full_map_L2']=global_detail
                runner.topology=active
                result=solve(runner,query,session,auditor,spec)
        except (ValueError,RuntimeError,TimeoutError) as error:
            if failure_stage(error)=='infrastructure':raise
            result=(None,None,{'success':False,'failure_code':str(error),
                'full_map_L2_failure':getattr(error,'trials',None)})
        finally:
            if cap is not None:del session._v12_phase_L3_cap
        out=result[2];recoveries.append(out.get('candidate_recovery',{}))
        # A phase's smaller allowance is not the global15s solver deadline.
        # Keep its legacy evidence but let the original endpoint resolver use
        # genuinely remaining time instead of treating this as global expiry.
        if (cap is not None and not out.get('success') and
                out.get('failure_code')=='CURVE_BUDGET' and
                state['new_used_s']<budget-.02 and time.monotonic()<deadline-.02):
            out['v12_local_slice_legacy_failure']='CURVE_BUDGET'
            out['failure_code']='CURVE_V12_PHASE_SLICE_EXHAUSTED'
        event.update({'phase':phase.name,'status':'returned','success':bool(out.get('success')),
            'failure_code':out.get('failure_code',''),'failure_stage':result_stage(out),
            'wall_ms':(time.monotonic()-tick)*1000,'elapsed_s':time.monotonic()-state['started'],
            'remaining_request_s':max(0.,deadline-time.monotonic()),
            'L3_used_before_s':before,'L3_used_after_s':state['new_used_s'],
            'request_deadline':deadline,'L3_budget_s':budget,'full_map_L2':global_detail,
            'phase_L3_total_cap_s':cap,'full_map_L2_failure':out.get('full_map_L2_failure'),
            'effective_profile':{'soft_preference_relaxed':phase.relax_preference,
                                 'corridor_widened':phase.widen_corridor,
                                 'L3_discretization_refined':phase.refine_l3}})
        if not out.get('success'):
            from .v12_contracts import planner_failure_record
            event['failure_record']=planner_failure_record(out.get('failure_code',''),
                stage=event['failure_stage'],attempt_id=f"{query.query_id if query else 'request'}:{len(state['v12_degradation_runs'])}:{phase.name}",
                elapsed_s=max(0.,event['elapsed_s']),remaining_s=event['remaining_request_s'])
        return result

    try:
        result=attempt(Phase())
        first_stage=result_stage(result[2])
        if not result[2].get('success') and first_stage in ('L1','L2','L3'):
            phases=[]
            if first_stage=='L1':phases.append(Phase('full_map_L2'))
            if first_stage in ('L1','L2'):
                phases.extend([Phase('relax_soft_preference',True),Phase('widen_corridor',True,True)])
            phases.append(Phase('refine_L3_discretization',first_stage!='L3',first_stage!='L3',True))
            for phase in phases:
                if result[2].get('success'):break
                if failure_stage(result[2].get('failure_code',''))=='infrastructure':break
                remaining=deadline-time.monotonic();l3_remaining=budget-state['new_used_s']
                if remaining<=.02 or l3_remaining<=.02:
                    traces.append({'phase':phase.name,'executed':False,
                        'skip_reason':'shared_request_or_L3_budget_exhausted',
                        'request_deadline':deadline,'remaining_request_s':max(0.,remaining),
                        'remaining_L3_s':max(0.,l3_remaining)})
                    break
                _release_failed_controller(session)
                result=attempt(phase)
                if phase.name=='full_map_L2' and active is base:break
    finally:
        runner.topology=base
    out=result[2]
    if len(recoveries)>1:
        combined=dict(recoveries[-1]);combined['attempts']=[a for r in recoveries for a in r.get('attempts',[])]
        for key in ('wall_ms','l1_actual_ms','all_candidates_controller_ms',
                    'all_candidates_l3_pipeline_ms','all_candidates_adapter_ms'):
            combined[key]=sum(r.get(key,0.) for r in recoveries)
        out['candidate_recovery']=combined
    out['v12_degradation']={'schema':'V12-shared-budget-degradation-v1','phases':traces,
        'selected_phase':next((t['phase'] for t in reversed(traces) if t.get('success')),None),
        'request_deadline':deadline,'L3_budget_s':budget,'L3_used_s':state['new_used_s'],
        'collision_traffic_radius_unchanged':True,'full_map_Smac':False,
        'L3_discretization_is_continuous_solver_alternative_pending_mentor_review':True}
    return result


def attach_final_failure(out,state):
    if os.environ.get('THREEYD_V12_FALLBACK','0')=='1' and not out.get('success'):
        from .v12_contracts import planner_failure_record
        now=time.monotonic()
        out['failure_record']=planner_failure_record(out.get('failure_code',''),
            stage=result_stage(out),attempt_id='final',
            elapsed_s=max(0.,now-state['started']),remaining_s=max(0.,state['deadline']-now))
    return out


def widen_target(controller,target):
    phase=getattr(controller,'v12_phase',Phase())
    if not phase.widen_corridor:return target
    from .route_quality import scope_mask
    widened=scope_mask(controller.world,controller.reference_plan.world_xy,4.)
    widened &= controller.plan.corridor_mask
    if controller.dynamic is not None:widened &= ~controller.dynamic.mask
    widened &= ~controller.world.hard
    controller.timing['v12_widened_L3']={'half_width_m':4.,'bounded_search_cells':int(widened.sum()),
        'existing_spatial_budget_unchanged':True,'selected_traffic_region_contract_unchanged':True}
    return target|widened
