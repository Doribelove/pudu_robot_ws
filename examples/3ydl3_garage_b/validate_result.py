# SPDX-License-Identifier: Apache-2.0
"""Fail closed on missing audits, endpoint drift, or unbound curve outputs."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import BSpline

ROOT = Path(__file__).resolve().parent


def require(condition, message):
    if not condition:
        raise ValueError(message)



def validate_libraries(folder):
    manifest = json.loads((ROOT / 'MANIFEST.json').read_text())['files']
    snapshot = json.loads((Path(folder) / 'loaded_nav2_libraries.json').read_text())
    require(snapshot['snapshot'] == 'locked', 'Unlocked native runtime')
    active_layers = ('runtime/r4/install/nav2_snapshot_local/lib/liblayers.so'
                     if 'runtime/r4/install/nav2_snapshot_local/lib/liblayers.so' in manifest
                     else 'runtime/v5/install/nav2_snapshot_local/lib/liblayers.so')
    required = [active_layers, 'runtime/r4/install/nav2_snapshot_local/lib/libnav2_costmap_2d_core.so',
                'runtime/nav2_sdk/lib/libnav2_smac_planner.so']
    # Keep every loaded copy: another process or path with the correct hash
    # must not hide an incompatible library with the same basename.
    observed = [(Path(name).name, value) for process in snapshot['processes']
                for name, value in process['libraries'].items()]
    for relative in required:
        name = Path(relative).name
        hashes = [value for observed_name, value in observed if observed_name == name]
        require(bool(hashes) and all(value == manifest[relative] for value in hashes),
                f'Loaded native library differs: {name}')
    return {Path(name).name: manifest[name] for name in required}


def validate_case(case, expected):
    outcome = json.loads((case / 'outcome.json').read_text())
    for key in ['success', 'geometry_success', 'exact_endpoint_success']:
        require(outcome.get(key) is True, f'{case.name}: {key} is not true')
    require(outcome.get('tolerant_endpoint_success') is False, 'Unexpected tolerant endpoint result')
    require(outcome['endpoint_tolerance']['used'] is False, 'Unexpected pose relaxation')
    for key in ['independent_audit', 'traffic_audit']:
        require(outcome[key].get('valid') is True, f'{key} failed')
        require(not outcome[key].get('failures'), f'{key} reports failures')
    require(outcome['constraint_audit']['steering_jump_valid'] is True, 'Steering audit failed')
    diagnostics = outcome['diagnostics']
    require(diagnostics['costmap_update_acknowledged'] is True, 'Actual costmap ACK missing')
    for key in ['costmap_ack_mismatch_cells', 'costmap_ack_hash_mismatch', 'costmap_ack_sequence_mismatch']:
        require(diagnostics[key] == 0, f'{key} is nonzero')
    require(diagnostics['costmap_ack_checked_cells'] > 0, 'ACK checked no cells')
    curve = outcome['curve_l3']
    require(curve['success'] is True and curve['mode'] == 'independent', 'Independent L3 result missing')
    bound = curve['certificate']['max_curvature_bound_1pm']
    require(np.isfinite(bound) and 0 <= bound <= 2.5 + 1e-6, 'Continuous curvature certificate failed')
    metrics = outcome['metrics']
    for key in ['static_footprint_valid', 'kinematic_valid', 'final_valid_success']:
        require(metrics[key] is True, f'{key} failed')
    require(not outcome['failure_code'], 'Planner reported failure')
    path = np.load(case / 'path.npy', allow_pickle=False)
    require(path.ndim == 2 and path.shape[1] == 3 and len(path) >= 2, 'Expected N x 3 path')
    require(np.isfinite(path).all(), 'Non-finite path')
    for index, role in [(0, 'start'), (-1, 'goal')]:
        target = np.asarray(expected[role])
        require(np.linalg.norm(path[index, :2] - target[:2]) <= 1e-6, f'{role} position drift')
        yaw_error = np.arctan2(np.sin(path[index, 2] - target[2]), np.cos(path[index, 2] - target[2]))
        require(abs(yaw_error) <= 1e-6, f'{role} yaw drift')
    model = outcome['curve_model']
    require(model['degree'] == 3, 'Expected cubic B-spline')
    spline = BSpline(model['knots'], model['coefficients'], model['degree'])
    t = np.asarray(model['parameters'])
    require(t.shape == (len(path),) and np.isfinite(t).all(), 'Curve/path parameter mismatch')
    require(np.max(np.linalg.norm(spline(t) - path[:, :2], axis=1)) <= 1e-6, 'Path detached from certified curve')
    tangent = spline(t, 1)
    yaw = np.arctan2(tangent[:, 1], tangent[:, 0])
    delta = np.arctan2(np.sin(path[:, 2] - yaw), np.cos(path[:, 2] - yaw))
    require(np.max(abs(delta)) <= 1e-6, 'Path yaw detached from curve tangent')
    length = float(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum())
    require(abs(length - metrics['path_length_m']) <= 1e-6, 'Path/metric mismatch')
    require(abs(length / expected['path_length_m'] - 1) <= .01, 'Path length differs over 1% from accepted example')
    for name in ['l1_output.json', 'l2_reference_xy.npy', 'l2_soft_target.json', 'l3_scope.npz']:
        require((case / name).is_file(), f'Missing layer output: {name}')
    return {'query': case.name.removesuffix('_1'), 'exact': True,
            'path_length_m': round(length, 6), 'curvature_bound_1pm': round(bound, 6),
            'actual_costmap_ack': True, 'curve_output_bound': True,
            'path_file_sha256': hashlib.sha256((case / 'path.npy').read_bytes()).hexdigest()}


def validate(folder, queries):
    folder = Path(folder)
    expected = json.loads((ROOT / 'expected.json').read_text())
    require(bool(queries) and len(queries) == len(set(queries))
            and set(queries) <= set(expected['queries']),
            'Select nonempty, unique query IDs from expected.json')
    summary = json.loads((folder / 'summary.json').read_text())
    require(summary['runs'] == summary['success'] == summary['exact_endpoint_success'] == len(queries), 'Unexpected success counts')
    require(summary['tolerant_endpoint_success'] == 0, 'Unexpected tolerance fallback')
    protocol = json.loads((folder / 'protocol.json').read_text())
    require(protocol['version'] == expected['version'] and protocol['map_id'] == expected['map_id'], 'Wrong source/map')
    require(protocol['l3_backend'] == 'independent' and protocol['L1_frozen'] is True, 'Wrong architecture')
    require(protocol['global_L3_fallback'] is False and protocol['moving_window'] is False, 'Unexpected fallback/window')
    require(set(protocol['actual_query_order']) == set(queries) and len(protocol['actual_query_order']) == len(queries), 'Wrong query selection')
    libraries = validate_libraries(folder)
    results = [validate_case(folder / 'cases' / (qid + '_1'), expected['queries'][qid]) for qid in queries]
    return {'version': expected['version'], 'map_id': expected['map_id'], 'passed': len(results), 'total': len(queries), 'cases': results, 'loaded_native_sha256': libraries}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('folder', type=Path)
    p.add_argument('--queries', nargs='+', default=['Q01'])
    a = p.parse_args()
    print(json.dumps(validate(a.folder, a.queries), indent=2))
