"""Event-local scheduling around unchanged geometric and curve solvers."""
import os
import time
from pathlib import Path
from three_yd_v5.recovery import V5Runner
from three_yd_v5.fullroute import record_from
from three_yd_v5.v12_degradation import Phase, phase_scope, result_stage, failure_stage
from three_yd_v5 import endpoint_tolerance as endpoints


class ShortRunner(V5Runner):
    def plan(self,query,session,auditor,spec):
        from three_yd_v5.route_quality import plan
        state=session._curve_request
        old=getattr(session,'_v12_phase_L3_cap',None)
        cap=min(state['config']['new_l3_budget_s'],state['new_used_s']+.6)
        if old is not None:cap=min(cap,old)
        session._v12_phase_L3_cap=cap
        try:
            from three_yd_v5.p6_hint import request_scope
            with phase_scope(Phase()), request_scope(self,session):
                result=plan(self,query,session,auditor,spec)
        except (ValueError,RuntimeError,TimeoutError) as error:
            if failure_stage(error)=='infrastructure':raise
            result=(None,None,{'success':False,'failure_code':str(error),'failure_stage':failure_stage(error)})
        finally:
            if old is None:del session._v12_phase_L3_cap
            else:session._v12_phase_L3_cap=old
        out=result[2]
        if (not out.get('success') and out.get('failure_code')=='CURVE_BUDGET'
                and state['new_used_s']<state['config']['new_l3_budget_s']-.02
                and time.monotonic()<state['deadline']-.02):
            out['failure_code']='CURVE_P5_SCHEDULE_SLICE_EXHAUSTED'
            out['failure_stage']='L3'
        return result


def remaining(service):
    s=service.session._curve_request
    return min(s['deadline']-time.monotonic(),s['config']['new_l3_budget_s']-s['new_used_s'])


def cache_identity(service):
    from three_yd_v5.v9_maps import content_digest
    from three_yd_v5.v12_regional import current
    return (service.world.key,tuple(service.world.vehicle),current().fingerprint,
            service.dynamic.version,content_digest(service.dynamic.mask),content_digest(service.world.hard))


def release(service):
    endpoints.withdraw(service)
    service.controller=None;service.route=None
    active=getattr(service.session,'active_controller',None)
    if active is not None:
        active.trajectory=None;active.trajectory_version=None;active.lifecycle.clear()
    service.session.active_controller=None;service.session.active_route=None
    from three_yd_v5.source_retirement import retire
    retire(service.session)


def install(service,query,result):
    c,r,out=result
    service.query=query;service.controller=c;service.route=r;service.record=record_from(out)
    if out.get('success'):
        service.task_query=query;service.remaining=service.current_trajectory();service.base_traffic=c.traffic
    return out


def solve(service,query):
    if service._p5_legacy:
        tick=time.monotonic();out=service._p4_solve(query)
        service._p5_attempts.append({'kind':'original_full_recovery','start':list(query.start),'goal':list(query.goal),
            'wall_ms':(time.monotonic()-tick)*1000,'success':bool(out.get('success')),
            'failure_code':out.get('failure_code'),'candidate_recovery':out.get('candidate_recovery'),
            'corridor_recovery':out.get('v13_corridor_recovery')})
        return out
    from .incremental import AStarController,IncrementalController
    cls=AStarController if service.arm=='H-A' else IncrementalController
    lead=getattr(service,'_p5_lead',0.)
    if lead:
        from .heading_reference import controller_class
        cls=controller_class(cls,lead)
    identity=cache_identity(service)
    cache_enabled=os.environ.get('V13_P5_CACHE','1')=='1'
    result=(None,None,{'success':False,'failure_code':'V13_P5_NO_ATTEMPT'})
    modes=['original_domain']
    if getattr(service.topology,'mode',None)=='free_space' and service.dynamic.mask.any():
        modes.append('frozen_L1_current_obstacles')
    for mode in modes:
        if mode!='original_domain' and result_stage(result[2]) not in ('L1','L2'):
            # A connected reference with rejected endpoint geometry first
            # yields to the existing bounded tolerance lattice.
            break
        if remaining(service)<=.02:
            result=(None,None,{'success':False,'failure_code':'V13_L3_RECOVERY_BUDGET_EXHAUSTED','failure_stage':'L3'})
            break
        release(service);service.query=query
        tick=time.monotonic();event={'kind':mode,'start':list(query.start),'goal':list(query.goal),
            'phase_L3_slice_s':.6,'cache_hit':False};service._p5_attempts.append(event)
        before=service.session._curve_request['new_used_s']
        key=(identity,mode,lead)
        cached=service._p5_cache.get(key) if cache_enabled else None
        if cached is None:
            if mode=='original_domain':top=service.topology;views=service.views
            else:
                from three_yd_v5.free_space_topology import FreeSpaceTopology
                top=FreeSpaceTopology(service.world,Path(service.root)/f'p5_{service.dynamic.version}_topology')
                event['topology_preparation_ms']=top.preparation_ms
                views={k:v for k,v in service.views.items() if isinstance(k,int)}
            if cache_enabled:service._p5_cache[key]=(top,views)
        else:
            top,views=cached;event['cache_hit']=True
        runner=ShortRunner(service.world,top,Path(service.root)/f'p5_{service.dynamic.version}_{mode}',
            views,service.dynamic,controller_class=cls,max_region_routes=service.max_region_routes)
        result=runner.plan(query,service.session,service.auditor,service.spec)
        out=result[2]
        event.update(wall_ms=(time.monotonic()-tick)*1000,success=bool(out.get('success')),
            failure_code=out.get('failure_code'),candidate_recovery=out.get('candidate_recovery'),
            L3_used_s=service.session._curve_request['new_used_s']-before)
        event['L2_heading_lead_m']=lead
        if out.get('success'):
            service.views=views
            if mode!='original_domain':
                result[0].v13_recovery_domain={'kind':mode,'map_version':service.dynamic.version}
                out['v13_corridor_recovery']={'selected':mode,'physical_infeasibility_proven':False,
                    'L1_L3_algorithms_unchanged':True,'schedule':'P5'}
            break
        if failure_stage(out.get('failure_code'))=='infrastructure':break
    return install(service,query,result)


def after_update(service,out,requested):
    if not getattr(service,'_p5_active',False) or out.get('success') or out.get('action')=='no_change':
        return endpoints.after_update(service,out,requested)
    cfg=endpoints.policy()
    if (requested is None or not cfg['enabled'] or not endpoints.eligible(out.get('failure_code'))
            or len(getattr(service,'prefix',[]))):
        return endpoints.after_update(service,out,requested)
    original_event=out
    attempts=[]
    def run(query):
        for name in ('_r4_seed_phase','_r5_speculative_seed','_r8_scope_recovery'):
            if hasattr(service.session,name):delattr(service.session,name)
        tick=time.monotonic()
        result=service._plan(query)
        attempts.append({'start':list(query.start),'goal':list(query.goal),
            'success':bool(result.get('success')),'failure_code':result.get('failure_code'),
            'schedule':'original_full' if service._p5_legacy else 'short_primary',
            'wall_ms':(time.monotonic()-tick)*1000})
        return result
    q=service.query
    same=(q is not None and tuple(q.start)==tuple(requested.start) and tuple(q.goal)==tuple(requested.goal))
    if not same:out=run(requested)
    else:attempts.append({'source':'already_executed_exact_update','start':list(q.start),'goal':list(q.goal),
        'success':False,'failure_code':out.get('failure_code'),'wall_ms':0.})
    resolved=requested
    if out.get('success'):
        return endpoints.attach(service,out,requested,requested,cfg,attempts)
    options=list(endpoints.candidates(service,requested,cfg))
    for query in options[:2]:
        if out.get('success') or not endpoints.eligible(out.get('failure_code')) or remaining(service)<=.02:break
        endpoints.withdraw(service);out=run(query);resolved=query
    if (not out.get('success') and endpoints.eligible(out.get('failure_code'))
            and os.environ.get('V13_P5_HEADING_REFERENCE','0')=='1'):
        try:
            for lead in (.4,.8):
                service._p5_lead=lead
                for query in [requested,*options[:1]]:
                    if remaining(service)<=.02:break
                    endpoints.withdraw(service);out=run(query);resolved=query
                    if out.get('success') or not endpoints.eligible(out.get('failure_code')):break
                if out.get('success') or not endpoints.eligible(out.get('failure_code')):break
        finally:service._p5_lead=0.
    if not out.get('success') and endpoints.eligible(out.get('failure_code')) and remaining(service)>.02:
        service._p5_legacy=True
        try:
            # Preserve original full alternatives after the bounded early
            # candidates. Shared clocks never reset.
            for query in [requested,*options]:
                if remaining(service)<=.02:break
                endpoints.withdraw(service);out=run(query);resolved=query
                if out.get('success') or not endpoints.eligible(out.get('failure_code')):break
        finally:service._p5_legacy=False
    if not out.get('success'):
        endpoints.withdraw(service)
        state=service.session._curve_request
        if remaining(service)<=.02:
            request_expired=time.monotonic()>=state['deadline']-.02
            prior=out.get('failure_code')
            out.update(failure_code='V13_REQUEST_RECOVERY_BUDGET_EXHAUSTED' if request_expired else 'V13_L3_RECOVERY_BUDGET_EXHAUSTED',
                failure_stage='budget' if request_expired else 'L3',prior_failure_code=prior,
                physical_infeasibility_proven=False)
    out=endpoints.attach(service,out,requested,resolved,cfg,attempts)
    out.update(update=original_event.get('update'),action='endpoint_tolerance_replan',
        replanned=True,prior_update_failure=original_event.get('failure_code'),
        trajectory_available=bool(out.get('success')))
    return out
