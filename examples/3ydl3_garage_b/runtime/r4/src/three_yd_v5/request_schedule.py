"""Try existing recovery sooner after two difficult endpoint initializations.

Only the first primary tranche is shortened. A rejected early schedule can
run the original schedule in the SAME request and L3 budgets. No cached result
or relaxed endpoint, footprint, curvature, corridor or traffic test is used.
"""
import time


def scheduled_plan(runner, query, session, auditor, spec):
    from .curve_backend import settings
    mode,cfg,_=settings()
    if mode!='independent' or getattr(runner.topology,'mode',None)=='free_space':
        return runner._plan_original(query,session,auditor,spec)
    started=time.monotonic()
    state=getattr(session,'_curve_request',None)
    offset=len(state.get('attempts',[])) if state is not None else 0
    if hasattr(session,'_r11_primary_slice_s'):
        raise RuntimeError('NESTED_PRIMARY_SCHEDULE')
    session._r11_primary_slice_s=1.0
    try:
        first=runner._plan_original(query,session,auditor,spec)
    finally:
        del session._r11_primary_slice_s
    state=getattr(session,'_curve_request',None)
    yielded=bool(state and any(a.get('r11_primary_yielded') for a in state['attempts'][offset:]))
    out=first[2]
    detail={'primary_slice_s':1.,'requires_two_selected_endpoint_seeds':True,
        'yielded':yielded,'original_schedule_retried':False,'shared_request_and_l3_budget':True}
    remaining=0. if state is None else min(state['deadline']-time.monotonic(),
                                          cfg['new_l3_budget_s']-state['new_used_s'])
    infrastructure=any(code in str(out.get('failure_code','')) for code in
        ('ACK','CONTENT','VERSION','BINDING','CORRUPT'))
    if not out.get('success') and yielded and not infrastructure and remaining>.05:
        second=runner._plan_original(query,session,auditor,spec)
        old=out.get('candidate_recovery',{});current=second[2].get('candidate_recovery',{})
        combined={**current,'attempts':old.get('attempts',[])+current.get('attempts',[]),
                  'early_schedule':old,'original_schedule':current}
        for key in ('wall_ms','l1_actual_ms','all_candidates_controller_ms',
                    'all_candidates_l3_pipeline_ms','all_candidates_adapter_ms'):
            combined[key]=old.get(key,0.)+current.get(key,0.)
        second[2]['candidate_recovery']=combined
        detail['original_schedule_retried']=True
        first=second;out=second[2]
    detail['wall_ms']=(time.monotonic()-started)*1000
    out['r11_request_schedule']=detail
    return first
