from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from arena_evaluation.semantic_transition_r2_preflight import full_gate, explicit_hard_audit


def passing_inputs():
    invariants = dict.fromkeys(("canonical_full_path_passed", "padded_effective_master_collision_free",
        "exact_endpoint_xy_yaw", "same_lane_instance_full_path", "maximum_curvature_strict_passed",
        "path_length_bound_passed"), True)
    invariants.update(no_stopping_goal_violation=False, relaxation_level="R0", path_length_bound_m=40.)
    return [dict(full_path_invariants=invariants), dict(control_replay_passed=True, arc_length_m=32.),
            dict(hard_feature_gate_passed=True), dict(semantic_gate_passed=True),
            dict(revisit_screen_passed=True)]


def test_all_independent_gates_are_required():
    args = passing_inputs()
    assert full_gate(*args)
    assert not full_gate(*args[:-1])
    for i, key in ((1, 'control_replay_passed'), (2, 'hard_feature_gate_passed'),
                   (3, 'semantic_gate_passed'), (4, 'revisit_screen_passed')):
        failed = deepcopy(args)
        failed[i][key] = False
        assert not full_gate(*failed)


@pytest.mark.parametrize('key', ['canonical_full_path_passed', 'padded_effective_master_collision_free',
    'exact_endpoint_xy_yaw', 'same_lane_instance_full_path', 'maximum_curvature_strict_passed',
    'path_length_bound_passed'])
def test_missing_full_path_safety_cannot_pass(key):
    args = passing_inputs()
    del args[0]['full_path_invariants'][key]
    assert not full_gate(*args)


def test_analytic_arc_length_is_not_replaced_by_chord_length():
    args = passing_inputs()
    args[1]['arc_length_m'] = 40.000001
    assert not full_gate(*args)


def test_no_stopping_checks_both_task_endpoints():
    world = SimpleNamespace(start=(0., 0., 0.), goal=(1., 0., 0.),
        map=SimpleNamespace(world_to_cell=lambda x, y: (0, int(x))),
        grids={'no_stopping': np.array([[True, False]])})
    semantic_map = SimpleNamespace(features=[], semantic_map_hash='test')
    result = explicit_hard_audit(world, [dict(x=0., y=0., yaw=0.), dict(x=1., y=0., yaw=0.)], semantic_map)
    assert result['no_stopping_task_endpoint_violations'] == 1
    assert not result['hard_feature_gate_passed']
