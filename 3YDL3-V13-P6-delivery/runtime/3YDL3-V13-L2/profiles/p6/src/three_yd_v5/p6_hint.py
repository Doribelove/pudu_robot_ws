"""Request-local L2 search-hint recovery; no physical or L3 relaxation.

Only a failed event-time shorter-channel L2 proposal can enter this branch.
Semantic routes are excluded. A new domain always builds a new controller;
g/rhs/OPEN from the previous geometry are never transferred.
"""
from contextlib import contextmanager
from contextvars import ContextVar
import os
import time
import numpy as np

_context = ContextVar('p6_hint_request', default=None)


@contextmanager
def request_scope(runner, session):
    mode = os.environ.get('V13_P6_HINT_MODE', 'local')
    if mode not in ('off', 'tube', 'local'):
        raise ValueError('INVALID_P6_HINT_MODE')
    dynamic = runner.dynamic
    active = (mode != 'off' and getattr(runner.topology, 'mode', None) == 'free_space'
              and dynamic is not None and bool(dynamic.obstacles))
    value = dict(mode=mode, variant='base', dynamic=dynamic, topology=runner.topology,
                 masks=[], session=session) if active else None
    token = _context.set(value)
    try:
        yield
    finally:
        _context.reset(token)


def cache_key():
    context = _context.get()
    return ('P6', context['mode'], context['variant']) if context else ('P5',)


def expand_scope(world, line, base, half_width_m):
    context = _context.get()
    if context is None or context['variant'] == 'base':
        return base
    before = int(np.count_nonzero(base))
    if context['variant'] == 'tube':
        # The integration ablation reproduces the previously diagnosed tube.
        from .route_quality import scope_mask
        context['variant'] = 'base'
        try:
            expanded = scope_mask(world, line, 5.)
        finally:
            context['variant'] = 'tube'
        centers = []
    else:
        from .local_detour import extend_hint
        expanded = base.copy()
        centers = []
        for polygon in context['dynamic'].obstacles.values():
            rr, cc = world.cells(np.asarray(polygon))
            r0, r1 = max(0,int(rr.min())), min(base.shape[0],int(rr.max())+1)
            c0, c1 = max(0,int(cc.min())), min(base.shape[1],int(cc.max())+1)
            if r1 <= r0 or c1 <= c0 or not np.any(base[r0:r1,c0:c1]):
                continue
            center = np.mean(polygon, axis=0)
            extend_hint(expanded, world, context['topology'].subregions, center, 5.)
            centers.append(center.tolist())
    context['masks'].append(dict(kind=context['variant'], half_width_or_radius_m=5.,
        base_cells=before, added_cells=int(np.count_nonzero(expanded))-before,
        centers_xy=centers, resolution_m=world.map.resolution,
        neutral_regions_only=True, physical_obstacles_unchanged=True))
    return expanded


def merge_attempts(previous, current):
    merged = {**current, 'attempts':previous.get('attempts',[])+current.get('attempts',[])}
    for key in ('wall_ms','l1_actual_ms','all_candidates_controller_ms',
                'all_candidates_l3_pipeline_ms','all_candidates_adapter_ms'):
        merged[key] = previous.get(key,0.)+current.get(key,0.)
    return merged


def retry_short_hint(runner, query, session, auditor, spec, first, detail):
    context = _context.get()
    from .v12_degradation import result_stage
    state = getattr(session, '_curve_request', None)
    available = bool(state and state['deadline']-time.monotonic()>.02
                     and state['config']['new_l3_budget_s']-state['new_used_s']>.02)
    if context is None or result_stage(first[2]) != 'L2' or not available:
        return first, detail
    from .request_schedule import scheduled_plan
    previous = first[2].get('candidate_recovery',{})
    trigger = first[2].get('failure_code')
    first = None  # release failed controller before allocating a larger domain
    context['variant'] = context['mode']
    tick = time.monotonic()
    try:
        second = scheduled_plan(runner, query, session, auditor, spec)
    finally:
        context['variant'] = 'base'
    evidence = dict(mode=context['mode'], trigger=trigger,
        success=bool(second[2].get('success')), failure_code=second[2].get('failure_code'),
        wall_ms=(time.monotonic()-tick)*1000, masks=list(context['masks']),
        cold_domain_rebuild=True, old_mutable_Dstar_state_reused=False,
        same_request_and_L3_budget=True)
    second[2]['candidate_recovery'] = merge_attempts(previous,second[2].get('candidate_recovery',{}))
    detail = {**detail, 'p6_hint_recovery':evidence,
              'selected_for_final_output':bool(second[2].get('success'))}
    second[2]['p6_hint_recovery'] = evidence
    return second, detail
