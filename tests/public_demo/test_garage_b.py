"""Public-input integrity and fail-closed result tests; no ROS nodes required."""
from pathlib import Path
import copy
import importlib.util
import json
import shutil
import sys
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2] / 'examples/3ydl3_garage_b'
sys.path.insert(0, str(ROOT))
import demo
from validate_result import validate_case


def test_shipped_manifest():
    demo.verify_files()


def test_frozen_map_query_binding(tmp_path):
    fixed = demo.prepare_map(tmp_path / 'fixed')
    spec = importlib.util.spec_from_file_location('public_fixed10_binding', fixed / 'fixed10_binding.py')
    binding = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(binding)
    inputs, _, queries = binding.resolve(demo.MAP_ID)
    assert [q['id'] for q in queries['scenarios']] == demo.QUERY_IDS
    image = inputs / 'optemap.pgm'
    with image.open('r+b') as f:
        f.seek(-1, 2)
        byte = f.read(1)
        f.seek(-1, 2)
        f.write(bytes([byte[0] ^ 1]))
    with pytest.raises(ValueError, match='FROZEN_INPUT_CHANGED'):
        binding.resolve(demo.MAP_ID)


def test_modified_shipped_file_is_rejected(tmp_path, monkeypatch):
    (tmp_path / 'x').write_text('changed')
    (tmp_path / 'MANIFEST.json').write_text(json.dumps({'files': {'x': '0' * 64}}))
    monkeypatch.setattr(demo, 'ROOT', tmp_path)
    with pytest.raises(ValueError, match='DEMO_FILE_CHANGED'):
        demo.verify_files()


def fixture_case(tmp_path):
    # Straight, exactly represented cubic B-spline. Negative cases below alter
    # individual evidence channels rather than trusting a summary success flag.
    case = tmp_path / 'Q01_1'
    case.mkdir()
    path = np.array([[0., 0., 0.], [.5, 0., 0.], [1., 0., 0.]])
    np.save(case / 'path.npy', path)
    for name in ['l1_output.json', 'l2_reference_xy.npy', 'l2_soft_target.json', 'l3_scope.npz']:
        (case / name).write_text('fixture layer presence')
    result = dict(success=True, geometry_success=True, exact_endpoint_success=True,
                  tolerant_endpoint_success=False, endpoint_tolerance={'used': False},
                  independent_audit={'valid': True, 'failures': []}, traffic_audit={'valid': True, 'failures': []},
                  constraint_audit={'steering_jump_valid': True}, failure_code='',
                  diagnostics={'costmap_update_acknowledged': True, 'costmap_ack_mismatch_cells': 0,
                               'costmap_ack_hash_mismatch': 0, 'costmap_ack_sequence_mismatch': 0,
                               'costmap_ack_checked_cells': 10},
                  curve_l3={'success': True, 'mode': 'independent', 'certificate': {'max_curvature_bound_1pm': 0.}},
                  metrics={'static_footprint_valid': True, 'kinematic_valid': True,
                           'final_valid_success': True, 'path_length_m': 1.},
                  curve_model={'degree': 3, 'knots': [0., 0., 0., 0., 1., 1., 1., 1.],
                               'coefficients': [[0., 0.], [1/3, 0.], [2/3, 0.], [1., 0.]],
                               'parameters': [0., .5, 1.]})
    (case / 'outcome.json').write_text(json.dumps(result))
    expected = {'start': [0., 0., 0.], 'goal': [1., 0., 0.], 'path_length_m': 1.}
    return case, result, expected


def test_valid_curve_output(tmp_path):
    case, _, expected = fixture_case(tmp_path)
    assert validate_case(case, expected)['curve_output_bound'] is True


@pytest.mark.parametrize('change', ['ack', 'curvature', 'traffic', 'tolerance', 'missing_audit', 'detached_path', 'endpoint'])
def test_invalid_result_never_accepted(tmp_path, change):
    case, result, expected = fixture_case(tmp_path)
    if change == 'ack':
        result['diagnostics']['costmap_ack_hash_mismatch'] = 1
    elif change == 'curvature':
        result['curve_l3']['certificate']['max_curvature_bound_1pm'] = 2.51
    elif change == 'traffic':
        result['traffic_audit']['valid'] = False
    elif change == 'tolerance':
        result['endpoint_tolerance']['used'] = True
    elif change == 'missing_audit':
        del result['independent_audit']
    else:
        path = np.load(case / 'path.npy')
        path[1 if change == 'detached_path' else -1, 1] = .2
        np.save(case / 'path.npy', path)
    (case / 'outcome.json').write_text(json.dumps(result))
    with pytest.raises((ValueError, KeyError)):
        validate_case(case, expected)
