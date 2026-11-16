"""Bind final spline intervals to their real generator and layer contributions.

The spline parameter is neither time nor arc length. Region IDs denote the
selected route's contributing regions, not a certified spatial partition.
No path, ACK, successful audit or certificate is reused by this module.
"""
import hashlib
import json
import os
import time
import numpy as np

from .v12_contracts import SourceSpan, validate_spans

SCHEMA = '3YDL3-V12-final-curve-provenance-v1'


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def geometry_binding(model):
    return {name: model[name] for name in ('degree', 'knots', 'coefficients')}


def build(model, *, route, grid_xy, policy, certificate, audit, ack, attempt_id,
          remaining_s):
    """Use nonzero polynomial knot spans, clipped to the delivered suffix."""
    if remaining_s <= 0:
        raise RuntimeError('V12_PROVENANCE_REQUEST_DEADLINE')
    if not audit.get('valid') or not certificate:
        raise ValueError('V12_PROVENANCE_UNCERTIFIED_OUTPUT')
    if (not ack.get('costmap_update_acknowledged') or
            ack.get('costmap_ack_mismatch_cells', 0) or
            ack.get('costmap_ack_hash_mismatch', 0) or
            not ack.get('server_costmap_content_hash')):
        raise ValueError('V12_PROVENANCE_ACK_BINDING')
    degree = int(model['degree'])
    knots = np.asarray(model['knots'], float)
    parameters = np.asarray(model['parameters'], float)
    coefficients = np.asarray(model['coefficients'], float)
    if (degree != 3 or knots.ndim != 1 or parameters.ndim != 1 or len(parameters) < 2
            or coefficients.ndim != 2 or coefficients.shape[1] != 2
            or len(knots) != len(coefficients) + degree + 1
            or not all(np.isfinite(a).all() for a in (knots, parameters, coefficients))
            or np.any(np.diff(knots) < 0) or np.any(np.diff(parameters) <= 0)):
        raise ValueError('V12_PROVENANCE_SPLINE_BINDING')
    lo, hi = float(parameters[0]), float(parameters[-1])
    if lo < knots[degree] or hi > knots[-degree-1] or lo >= hi:
        raise ValueError('V12_PROVENANCE_DOMAIN_BINDING')
    geometry = geometry_binding(model)
    model_ref = digest(geometry)
    topology_ref = digest(route)
    grid_ref = digest(np.asarray(grid_xy, float).tolist())
    policy_ref = digest(policy)
    certificate_ref = digest({'model_ref': model_ref, 'certificate': certificate,
                              'audit': audit, 'ack': ack})
    edges = np.unique(np.r_[lo, knots[(knots > lo) & (knots < hi)], hi])
    spans = [SourceSpan(float(a), float(b), 'kinematic_curve', topology_ref,
                        grid_ref, policy_ref, attempt_id, certificate_ref,
                        tuple(str(r) for r in route['region_ids']))
             for a, b in zip(edges[:-1], edges[1:])]
    payload = validate_spans(spans, (lo, hi))
    return {**payload, 'schema': SCHEMA, 'parameter_semantics': 'spline_parameter',
            'region_semantics': 'selected_route_contributions_not_spatial_partition',
            'generator': 'whole_route_cubic_bspline_SQP', 'model_ref': model_ref,
            'topology': route, 'grid_ref': grid_ref, 'policy': policy,
            'certificate_ref': certificate_ref,
            'certificate_binding': {'certificate': certificate, 'audit': audit,
                                    'ack': ack},
            'no_claim_of_local_Hybrid_segments': True}


def attach(service, out):
    if os.environ.get('THREEYD_V12_PROVENANCE', '0') != '1':
        return out
    if not out.get('success'):
        out['path_provenance'] = {'schema': SCHEMA, 'spans': [],
                                  'failure_code': out.get('failure_code', '')}
        return out
    started = time.monotonic()
    c = service.controller
    model = out.get('curve_model') or (service.record or {}).get('curve_model')
    if model is None or c is None or service.route is None:
        raise ValueError('V12_PROVENANCE_MODEL_MISSING')
    log = out.get('curve_l3') or (service.record or {}).get('curve_l3', {})
    certificate = log.get('certificate', {})
    # Reuse-validation may place its NEW certificate in the diagnostics.
    diagnostics = out.get('diagnostics', {})
    certificate = diagnostics.get('curve_revalidation', certificate)
    audit = out.get('independent_audit', {})
    ack_keys = ('costmap_update_acknowledged', 'costmap_ack_status',
                'costmap_ack_sequence', 'costmap_ack_mismatch_cells',
                'costmap_ack_hash_mismatch', 'server_costmap_content_hash',
                'applied_mask_hash')
    ack = {key: diagnostics[key] for key in ack_keys if key in diagnostics}
    state = service.session._curve_request
    route = {'signature': service.route.signature,
             'source': service.route.diagnostics.get('source','frozen_L1'),
             'region_ids': list(service.route.region_ids),
             'map_sha256': service.world.map.sha256,
             'start': list(service.route.start), 'goal': list(service.route.goal)}
    policy = getattr(c, 'v12_policy_evidence', {'mode': 'inherited',
                        'hard_traffic_unchanged': True})
    attempt_id = policy.get('degradation_phase','primary')+':final:' + str(len(state.get('attempts', [])))
    out['path_provenance'] = build(model, route=route,
        grid_xy=c.reference_plan.world_xy, policy=policy, certificate=certificate,
        audit=audit, ack=ack, attempt_id=attempt_id,
        remaining_s=state['deadline']-time.monotonic())
    out['path_provenance']['actual_request_ms'] = (time.monotonic()-started)*1000
    return out
