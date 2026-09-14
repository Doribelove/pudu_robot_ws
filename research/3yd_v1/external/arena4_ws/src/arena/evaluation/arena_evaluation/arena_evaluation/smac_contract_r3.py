"""Bind the declared r3 safety contract to actual ROS parameter readbacks."""
from __future__ import annotations

import hashlib
import json
import struct
import time

PLANNER_CONTRACT = {
    'angle_quantization_bins': 48,
    'motion_model_for_search': 'DUBIN',
    'minimum_turning_radius': 0.4,
    'allow_unknown': False,
    'max_iterations': 1000000,
    'downsample_costmap': False,
}
COSTMAP_CONTRACT = {
    'resolution': 0.05,
    'track_unknown_space': True,
    # Pinned Costmap2DROS declares ParameterValue(0.01f), promoted to double.
    # Match that exact binary value; do not introduce a numerical tolerance.
    'footprint_padding': struct.unpack('f',struct.pack('f',0.01))[0],
    'inflation_layer.inflation_radius': 0.55,
    'inflation_layer.cost_scaling_factor': 3.0,
}


def frozen_planner_overrides(overrides=None):
    result = dict(overrides or {})
    for key, expected in PLANNER_CONTRACT.items():
        if key in result and (type(result[key]) is not type(expected) or result[key] != expected):
            raise ValueError('SMAC_SAFETY_CONFIG_CONFLICT:' + key)
        result[key] = expected
    return result


def validate_runtime_contract(planner, costmap, footprint, map_hash):
    for prefix, actual, expected in (
        ('planner', planner, {'GridBased.' + k: v for k, v in PLANNER_CONTRACT.items()}),
        ('costmap', costmap, COSTMAP_CONTRACT),
    ):
        for key, value in expected.items():
            if key not in actual or type(actual[key]) is not type(value) or actual[key] != value:
                raise ValueError('RUNTIME_SAFETY_CONTRACT_MISMATCH:' + prefix + ':' + key +
                                 ':expected=' + repr(value) + ':observed=' + repr(actual.get(key)))
    try:
        actual_footprint = json.loads(costmap['footprint'])
    except (ValueError, KeyError, TypeError) as exc:
        raise ValueError('RUNTIME_SAFETY_CONTRACT_MISMATCH:footprint') from exc
    if actual_footprint != [list(v) for v in footprint]:
        raise ValueError('RUNTIME_SAFETY_CONTRACT_MISMATCH:footprint')
    binding = {'version': 'r3-runtime-safety-readback-v1', 'map_hash': map_hash,
               'planner': dict(planner), 'costmap': dict(costmap),
               'scope': 'startup ROS GetParameters; only max_planning_time may change per request'}
    payload = json.dumps(binding, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return {**binding, 'verified': True, 'sha256': hashlib.sha256(payload.encode()).hexdigest()}


def read_runtime_contract(session, footprint):
    from rcl_interfaces.srv import GetParameters
    deadline = time.monotonic() + 10.

    def read(service, names):
        client = session.client.node.create_client(GetParameters, service)
        try:
            if not client.wait_for_service(timeout_sec=max(0., deadline-time.monotonic())):
                raise ValueError('RUNTIME_SAFETY_READBACK_UNAVAILABLE:' + service)
            request = GetParameters.Request(); request.names = names
            future = client.call_async(request)
            while not future.done() and time.monotonic() < deadline:
                session.client.executor.spin_once(timeout_sec=.01)
            if not future.done():
                raise ValueError('RUNTIME_SAFETY_READBACK_TIMEOUT:' + service)
            values = future.result().values
            if len(values) != len(names):
                raise ValueError('RUNTIME_SAFETY_READBACK_LENGTH:' + service)
            fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value'}
            if any(v.type not in fields for v in values):
                raise ValueError('RUNTIME_SAFETY_READBACK_TYPE:' + service)
            return {name: getattr(v, fields[v.type]) for name, v in zip(names, values)}
        finally:
            session.client.node.destroy_client(client)

    planner = read('/planner_server/get_parameters', ['GridBased.' + k for k in PLANNER_CONTRACT])
    costmap = read('/global_costmap/global_costmap/get_parameters', [*COSTMAP_CONTRACT, 'footprint'])
    with (session.params_file.parent/'runtime_safety_observed.json').open('x') as stream:
        json.dump({'planner':planner,'costmap':costmap},stream,indent=2)
    return validate_runtime_contract(planner, costmap, footprint, session.ctx.map_sha256)
