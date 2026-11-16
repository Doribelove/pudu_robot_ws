"""L2 shorter-channel proposals; the frozen L1 task and output are untouched.

An all-free 1.20 m coarse cell can close a physically usable narrow opening.
For map-only tasks, examine the already available 0.60 m graph once before L2.
The proposal is only a search hint: original body inflation, L2 connectivity,
bounded complete-route L3, continuous certificates, and real ACK still apply.
"""
import math
import os
import time
import cv2
import numpy as np
from scipy import ndimage
from arena_evaluation.semantic_map import canonical_hash

SCHEMA = 'V11-r5-L2-shorter-channel-v1'
MINIMUM_GAIN = .05


def enabled():
    return os.environ.get('THREEYD_L2_ROUTE_QUALITY', '0') == '1'


def length(line):
    return float(np.linalg.norm(np.diff(np.asarray(line, float), axis=0), axis=1).sum())


def propose(topology, route, *, disabled=False):
    detail = {'schema': SCHEMA, 'selected': False, 'frozen_L1_unchanged': True,
              'L2_proposal_only': True}
    if not enabled() or disabled or getattr(topology, 'mode', None) != 'free_space':
        detail['reason'] = 'disabled_or_semantic'
        return None, detail
    if route.diagnostics.get('factor') != 24 or 12 not in topology._graphs:
        detail['reason'] = 'no_coarser_than_0.60m_selected_graph'
        return None, detail
    tick = time.monotonic()
    original = np.asarray(topology.task(route)['right_reference_polyline'], float)
    detail.update(original_length_m=length(original), original_L1_signature=route.signature,
                  original_guide_sha256=canonical_hash(original.tolist()), factor=12)
    try:
        free = topology._free_scales[12]
        start = topology._snap(topology._cell(route.start[:2], 12), free)
        goal = topology._snap(topology._cell(route.goal[:2], 12), free)
        if start is None or goal is None:
            detail['reason'] = 'candidate_endpoint_not_connected'
            return None, detail
        labels = ndimage.label(free)[0]
        connected = labels[start] != 0 and labels[start] == labels[goal]
        del labels
        if not connected:
            detail['reason'] = 'candidate_components_disconnected'
            return None, detail
        ends = topology._xy([start, goal], 12)
        if not (topology._clear_line(route.start[:2], ends[0])
                and topology._clear_line(ends[1], route.goal[:2])):
            detail['reason'] = 'candidate_endpoint_segment_blocked'
            return None, detail
        cells, expanded = topology._astar(start, goal, free, 12)
        detail['expanded'] = expanded
        if cells is None:
            detail['reason'] = 'candidate_search_failed'
            return None, detail
        coords = topology._xy(cells, 12)
        turns = [coords[0]]
        last = None
        for a, b in zip(cells, cells[1:]):
            direction = (b[0]-a[0], b[1]-a[1])
            if last is not None and direction != last:
                turns.append(topology._xy([a], 12)[0])
            last = direction
        turns.append(coords[-1])
        points = [np.asarray(route.start[:2]), *turns, np.asarray(route.goal[:2])]
        simplified = [points[0]]
        index = 0
        while index < len(points)-1:
            farthest = index+1
            for j in range(index+2, min(len(points), index+42)):
                if topology._clear_line(points[index], points[j]):
                    farthest = j
                else:
                    break
            simplified.append(points[farthest])
            index = farthest
        candidate = np.asarray(simplified, float)
        candidate = candidate[np.r_[True, np.linalg.norm(np.diff(candidate, axis=0), axis=1) > 1e-6]]
        candidate_length = length(candidate)
        detail.update(candidate_length_m=candidate_length,
                      candidate_guide_sha256=canonical_hash(candidate.tolist()),
                      geometric_gain_fraction=1-candidate_length/max(detail['original_length_m'], 1e-9))
        if len(candidate) < 2 or candidate_length > detail['original_length_m']*(1-MINIMUM_GAIN):
            detail['reason'] = 'less_than_5_percent_geometric_gain'
            return None, detail
        detail.update(selected=True, reason='shorter_existing_coarse_channel',
                      must_pass_original_L2_L3_and_ACK=True)
        return candidate, detail
    finally:
        detail['actual_request_probe_ms'] = (time.monotonic()-tick)*1000


def scope_mask(world, line, half_width_m):
    """Same bounded metric tube as the original neutral L1 scope rasterizer."""
    factor = 3
    shape = (math.ceil(world.map.height/factor), math.ceil(world.map.width/factor))
    coarse = np.zeros(shape, np.uint8)
    rr, cc = world.cells(line)
    cv2.polylines(coarse, [np.column_stack((cc//factor, rr//factor)).astype(np.int32)], False, 1, 1)
    radius = math.ceil(half_width_m/(factor*world.map.resolution))
    yy, xx = np.mgrid[-radius:radius+1, -radius:radius+1]
    coarse = cv2.dilate(coarse, ((xx*xx+yy*yy) <= radius*radius).astype(np.uint8))
    fine = np.repeat(np.repeat(coarse, factor, axis=0), factor, axis=1)
    base = np.asarray(fine[:world.map.height, :world.map.width], bool)
    from .p6_hint import expand_scope
    return expand_scope(world, line, base, half_width_m)


def dispatch_l3(controller, query, session, auditor, spec, *, enabled=True, cap=140):
    from .fullroute import plan as original_plan
    limited = (controller.l2_route_quality.get('selected', False)
               and not getattr(controller, '_quality_first_attempt_done', False))
    if not limited:
        return original_plan(controller, query, session, auditor, spec, enabled=enabled, cap=cap)
    if hasattr(session, '_l2_quality_attempt_budget_s'):
        raise RuntimeError('NESTED_L2_QUALITY_ATTEMPT')
    # Reserve the original recovery budget instead of spending all 15 seconds
    # on a newly proposed channel that has not yet passed L3. This is a
    # smaller per-attempt budget, never additional request or L3 time.
    session._l2_quality_attempt_budget_s = 3.
    try:
        result = original_plan(controller, query, session, auditor, spec, enabled=enabled, cap=cap)
    finally:
        del session._l2_quality_attempt_budget_s
        controller._quality_first_attempt_done = True
    result['l2_quality_attempt_budget'] = {'maximum_solver_s': 3.,
        'inside_original_15s_L3_and_30s_request': True}
    return result


def plan(runner, query, session, auditor, spec):
    from .request_schedule import scheduled_plan
    runner._quality_context = {}
    runner._quality_original_fallback = False
    first = scheduled_plan(runner, query, session, auditor, spec)
    detail = dict(runner._quality_context)
    if (first[2].get('success') or not detail.get('selected')
            or any(code in str(first[2].get('failure_code', '')) for code in
                   ('ACK', 'CONTENT', 'VERSION', 'BINDING', 'CORRUPT'))):
        first[2]['l2_route_quality'] = detail
        return first
    from .p6_hint import retry_short_hint
    first, detail = retry_short_hint(runner,query,session,auditor,spec,first,detail)
    if first[2].get('success'):
        first[2]['l2_route_quality'] = detail
        return first
    previous = first[2].get('candidate_recovery', {})
    # Diagnostics are sufficient for the final combined result. Do not keep
    # the failed controller's large fields alive during original-scope setup.
    first = None
    runner._quality_original_fallback = True
    try:
        second = scheduled_plan(runner, query, session, auditor, spec)
    finally:
        runner._quality_original_fallback = False
    current = second[2].get('candidate_recovery', {})
    combined = {**current, 'attempts': previous.get('attempts', [])+current.get('attempts', []),
                'shorter_channel_attempt': previous, 'original_scope_attempt': current}
    for key in ('wall_ms', 'l1_actual_ms', 'all_candidates_controller_ms',
                'all_candidates_l3_pipeline_ms', 'all_candidates_adapter_ms'):
        combined[key] = previous.get(key, 0.)+current.get(key, 0.)
    second[2]['candidate_recovery'] = combined
    second[2]['l2_route_quality'] = {**detail, 'original_scope_retried': True,
                                   'selected_for_final_output': False,
                                   'same_request_and_L3_budget': True}
    return second
