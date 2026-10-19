"""Exact-first endpoint resolution for a global reference, never a connector.

Every candidate is a new immutable query passed through the unchanged L1/L2/L3
and audits. Offsets are measured against the user's request, never accumulated.
All work is inside the existing request timer and shared L3 budget.
"""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import json
import math
import os
import time
import numpy as np

CONFIG = Path(__file__).resolve().parents[2] / 'config/endpoint_tolerance.json'
FATAL = ('ACK', 'CONTENT', 'VERSION', 'BINDING', 'CORRUPT', 'BUDGET', 'TIMEOUT')
GEOMETRY = ('CURVE_', 'NO_LEGAL_SEMANTIC_ROUTE', 'NO_PATH', 'DISCONNECTED',
            'RECTANGLE_BRIDGE', 'FOOTPRINT', 'COLLISION', 'ENDPOINT',
            'START_', 'GOAL_', 'OUTSIDE_SCOPE')
ENDPOINT_INPUT_REJECTIONS = {'PARKING_OWNERSHIP_UNRESOLVED',
    'PARKING_BINDING_POSITION_MISMATCH', 'NO_LEGAL_REGION_ROUTE', 'V7_L1_NO_FREE_SPACE_ROUTE'}


def policy():
    cfg = json.loads(CONFIG.read_text())
    cfg['enabled'] = cfg['enabled'] and os.environ.get('THREEYD_ENDPOINT_TOLERANCE', '1') == '1'
    # Configuration may tighten, but cannot silently exceed user permission.
    if not (0 <= cfg['position_m'] <= .2 and 0 <= cfg['yaw_deg'] <= 30):
        raise ValueError('ENDPOINT_TOLERANCE_PERMISSION_LIMIT')
    if not 1 <= cfg['max_attempts'] <= 32:
        raise ValueError('ENDPOINT_TOLERANCE_ATTEMPT_LIMIT')
    return cfg


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def deviation(requested, resolved):
    return {'position_m': float(np.linalg.norm(np.asarray(resolved)[:2] - np.asarray(requested)[:2])),
            'yaw_deg': math.degrees(abs(wrap(float(resolved[2]) - float(requested[2]))))}


def eligible(code):
    code = str(code).upper()
    if code in ENDPOINT_INPUT_REJECTIONS:
        return True
    return not any(x in code for x in FATAL) and any(x in code for x in GEOMETRY)


def pose_options(pose, checker, cfg, deadline):
    """A bounded lattice inside a disk, with the original exact pose first."""
    pose = np.asarray(pose, float)
    if pose.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError('ENDPOINT_NONFINITE_POSE')
    offsets = [(0., 0.)]
    for radius in (.05, .10, .15, .20):
        if radius > cfg['position_m'] + 1e-12:
            continue
        offsets.extend((radius * math.cos(a), radius * math.sin(a))
                       for a in np.arange(8) * math.pi / 4)
    yaws = [0.] + [sign * degrees for degrees in (5., 10., 15., 20., 30.)
                   if degrees <= cfg['yaw_deg'] for sign in (1., -1.)]
    options = []
    for dx, dy in offsets:
        for degrees in yaws:
            if time.monotonic() >= deadline:
                return options
            p = (float(pose[0]+dx), float(pose[1]+dy), float(pose[2]+math.radians(degrees)))
            if not checker.check(np.asarray([p])):
                continue
            d = deviation(pose, p)
            score = d['position_m']/.2 + d['yaw_deg']/30.
            options.append((score, p))
    options.sort(key=lambda v: v[0])
    return options


def candidates(service, query, cfg):
    from .v10_storage import collision_checker
    state = service.session._curve_request
    started = time.monotonic()
    deadline = min(state['deadline'], started + max(0., state['config']['new_l3_budget_s']-state['new_used_s']))
    checker = collision_checker(service.world.hard, service.world)
    starts = pose_options(query.start, checker, cfg, deadline)
    goals = pose_options(query.goal, checker, cfg, deadline)
    # Bound enumeration as well as solver calls. Including both endpoints
    # permits recovery when both original footprints lie in obstacles.
    pairs = []
    for si, (ss, start) in enumerate(starts[:24]):
        for gi, (gs, goal) in enumerate(goals[:24]):
            if ss+gs <= 1e-12:
                continue
            # A one-endpoint change is preferred at equal displacement cost.
            pairs.append((ss+gs, int(ss > 0)+int(gs > 0), si, gi, start, goal))
    pairs.sort(key=lambda v: v[:4])
    state['new_used_s'] += time.monotonic() - started
    for *_, start, goal in pairs[:cfg['max_attempts']]:
        yield replace(query, start=tuple(start), goal=tuple(goal))


def withdraw(service):
    service.record = None
    service.remaining = None
    service.retained_record = None
    service.retained_path = None
    c = service.controller
    if c is not None:
        c.trajectory = None
        c.trajectory_version = None


def describe(query, resolved, cfg, attempts, success):
    offsets = {name: deviation(getattr(query, name), getattr(resolved, name))
               for name in ('start', 'goal')}
    used = any(v['position_m'] > 1e-8 or v['yaw_deg'] > 1e-6 for v in offsets.values())
    return {'policy': 'exact_first_then_endpoint_tolerance_v1',
            'position_tolerance_m': cfg['position_m'], 'yaw_tolerance_deg': cfg['yaw_deg'],
            'requested': {'start': list(query.start), 'goal': list(query.goal)},
            'resolved': {'start': list(resolved.start), 'goal': list(resolved.goal)} if success else None,
            'offsets': offsets if success else None, 'used': bool(success and used),
            'success_class': ('tolerant_endpoints' if used else 'exact_endpoints') if success else 'failure',
            'min_turn_radius_m': .4, 'unchanged_collision_and_traffic_checks': True,
            'shared_request_and_l3_budget': True, 'attempts': attempts,
            'reference_only': bool(success and used),
            'connector_from_requested_start_validated': False}


def attach(service, out, query, resolved, cfg, attempts):
    success = bool(out.get('success'))
    detail = describe(query, resolved, cfg, attempts, success)
    if success and detail['used']:
        # Independently bind the delivered curve to both the exact resolved
        # query and the tolerance around the ORIGINAL request.
        p = service.current_trajectory()
        if p is None or not np.isfinite(p).all():
            raise ValueError('ENDPOINT_RESULT_BINDING_MISSING')
        for name, actual in (('start', p[0]), ('goal', p[-1])):
            d = deviation(getattr(query, name), actual)
            exact = deviation(getattr(resolved, name), actual)
            if d['position_m'] > cfg['position_m']+1e-9 or d['yaw_deg'] > cfg['yaw_deg']+1e-7:
                raise ValueError('ENDPOINT_TOLERANCE_RESULT_OUTSIDE_PERMISSION')
            if exact['position_m'] > 1e-8 or exact['yaw_deg'] > 1e-6:
                raise ValueError('ENDPOINT_RESOLVED_POSE_BINDING_MISMATCH')
        out['legacy_contract_success'] = False
    out['endpoint_tolerance'] = detail
    out['exact_endpoint_success'] = success and not detail['used']
    out['tolerant_endpoint_success'] = success and detail['used']
    service.requested_query = query
    # Keep only resolution metadata resident. Per-attempt diagnostic trees
    # belong to the caller's output, not the next request's memory baseline.
    service.endpoint_resolution = deepcopy({k:v for k,v in detail.items() if k!='attempts'})
    if not success:
        # A later obstacle-clear event must start at the requested poses,
        # not at the last unsuccessful candidate from the tolerance search.
        service.query = query
    return out


def execute(service, query, solve, *, first_result=None):
    cfg = policy()
    if not cfg['enabled'] or service.session._curve_request.get('mode','independent')!='independent':
        return solve(query)
    attempts = []
    def run(q):
        before = time.monotonic()
        # Phase hints belong to one candidate. The shared clocks do not reset.
        for name in ('_r4_seed_phase','_r5_speculative_seed','_r8_scope_recovery'):
            if hasattr(service.session,name):delattr(service.session,name)
        try:
            out = solve(q)
        except (ValueError, RuntimeError) as error:
            if not eligible(str(error)):
                raise
            withdraw(service)
            out = {'success': False, 'failure_code': str(error)}
        attempts.append({'start': list(q.start), 'goal': list(q.goal),
                         'success': bool(out.get('success')), 'failure_code': out.get('failure_code', ''),
                         'candidate_recovery': out.get('candidate_recovery',{}) if not out.get('success') else {},
                         'wall_ms': (time.monotonic()-before)*1000})
        return out
    if first_result is None:
        out = run(query)
    else:
        out = first_result
        attempts.append({'start':list(query.start),'goal':list(query.goal),
            'success':bool(out.get('success')),'failure_code':out.get('failure_code',''),
            'wall_ms':out.get('wall_ms',0.),'source':'already_executed_exact_update',
            'candidate_recovery':out.get('recovery',{})})
    if out.get('success') or not eligible(out.get('failure_code')):
        return attach(service, out, query, query, cfg, attempts)
    if len(getattr(service,'prefix',[])):
        # Never reinterpret passed parking/traffic ownership as a new task.
        # Existing continuation uses its unchanged task-context repair path.
        out['endpoint_tolerance_skip']='COMMITTED_TASK_PROGRESS_CONTEXT'
        return attach(service,out,query,query,cfg,attempts)
    state = service.session._curve_request
    for resolved in candidates(service, query, cfg):
        if (time.monotonic() >= state['deadline'] or
                state['new_used_s'] >= state['config']['new_l3_budget_s']):
            break
        withdraw(service)
        out = run(resolved)
        if out.get('success'):
            return attach(service, out, query, resolved, cfg, attempts)
        if not eligible(out.get('failure_code')):
            break
    withdraw(service)
    return attach(service, out, query, query, cfg, attempts)


def after_update(service, out, requested):
    """A resolved endpoint must not become a new tolerance origin on updates."""
    if requested is None:
        return out
    cfg = policy()
    previous = getattr(service,'endpoint_resolution',{})
    if out.get('success'):
        return attach(service,out,requested,service.query,cfg,previous.get('attempts',[]))
    if out.get('action') == 'no_change':
        out['endpoint_tolerance'] = deepcopy(previous)
        return out
    if cfg['enabled'] and eligible(out.get('failure_code')) and not len(getattr(service,'prefix',[])):
        q=service.query
        same=(q is not None and q.query_id==requested.query_id
              and tuple(q.start)==tuple(requested.start) and tuple(q.goal)==tuple(requested.goal))
        recovery = execute(service,requested,service._plan,first_result=out if same else None)
        recovery.update(update=out.get('update'),action='endpoint_tolerance_replan',
                        replanned=True,prior_update_failure=out.get('failure_code'),
                        trajectory_available=bool(recovery.get('success')))
        return recovery
    out['endpoint_tolerance'] = describe(requested,requested,cfg,[],False)
    return out
