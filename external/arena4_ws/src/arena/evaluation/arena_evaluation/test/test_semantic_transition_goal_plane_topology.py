import numpy as np
import pytest

from arena_evaluation.semantic_transition_goal_plane_topology import (
    _oriented_terminal_tangent,
    classify_target_progress,
)


def test_all_reachable_target_after_goal_fails_necessary_condition():
    components = np.array([[1, 1, 1], [2, 2, 2]])
    free = np.array([[True, True, True], [True, False, True]])
    target = np.array([[False, True, True], [True, True, False]])
    progress = np.array([[-1.0, 0.2, 1.2], [-2.0, -1.0, 0.0]])
    result = classify_target_progress(
        components, target, free, start_component=1,
        signed_goal_progress=progress, numeric_tolerance_m=0.01,
    )
    assert result["status"] == "NO_PREGOAL_TARGET_IN_SAMPLED_OPTIMISTIC_START_COMPONENT"
    assert not result["sampled_projection_pre_goal_target_present"]
    assert result["reachable_target_after_goal_count"] == 2
    assert result["footprint_infeasible_target_count"] == 1
    assert result["free_other_component_target_count"] == 1


def test_pre_goal_target_is_a_necessary_condition_only():
    components = np.ones((1, 3), dtype=int)
    target = np.array([[True, True, False]])
    progress = np.array([[-0.2, 0.0, 1.0]])
    result = classify_target_progress(
        components, target, np.ones_like(target), start_component=1,
        signed_goal_progress=progress, numeric_tolerance_m=0.01,
    )
    assert result["status"] == "PREGOAL_TARGET_PRESENT_IN_SAMPLED_OPTIMISTIC_START_COMPONENT"
    assert result["sampled_projection_pre_goal_target_present"]
    assert result["reachable_target_before_goal_count"] == 1
    assert result["reachable_target_on_goal_plane_count"] == 1


def test_target_progress_input_mismatch_fails_closed():
    with pytest.raises(ValueError, match="identical shapes"):
        classify_target_progress(
            np.ones((2, 2)), np.ones((2, 3), bool), np.ones((2, 2), bool), start_component=1,
            signed_goal_progress=np.ones((2, 2)), numeric_tolerance_m=0.01,
        )


def test_route_is_oriented_to_query_and_terminal_tangent_is_unit():
    route = [[2.0, 0.0], [1.0, 0.0], [0.0, 0.0]]
    points, tangent, diagnostics = _oriented_terminal_tangent(
        route, start=(0.0, 0.0, 0.0), goal=(2.0, 0.0, 0.0),
    )
    assert diagnostics["route_reversed_for_query"]
    assert np.allclose(points[0], [0.0, 0.0])
    assert np.allclose(tangent, [1.0, 0.0])


def test_terminal_tangent_ignores_single_cell_attachment_jitter():
    route = [[0.0, 0.0], [0.0, 0.5], [-0.05, 0.55]]
    _, tangent, diagnostics = _oriented_terminal_tangent(
        route, start=(0.0, 0.0, 0.0), goal=(-0.05, 0.55, 0.0),
    )
    assert tangent[1] > 0.99
    assert abs(tangent[0]) < 0.1
    assert diagnostics["terminal_tangent_window_source"] == "one_full_padded_footprint_length"
