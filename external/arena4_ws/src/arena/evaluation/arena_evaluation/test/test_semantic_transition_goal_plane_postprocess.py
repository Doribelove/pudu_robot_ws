import numpy as np
import pytest

from arena_evaluation.semantic_transition_goal_plane_postprocess import (
    classify_reachable_target_progress,
    partition_target_space,
)


def test_all_reachable_target_after_goal_fails_necessary_condition():
    target = np.asarray([[True, True], [False, True]])
    progress = np.asarray([[0.02, 0.50], [-1.0, 1.0]])
    result = classify_reachable_target_progress(target, progress, tolerance_m=0.01)
    assert result["status"] == "NO_PREGOAL_TARGET_IN_FROZEN_SAMPLED_PROJECTION"
    assert result["reachable_target_after_goal_count"] == 3
    assert not result["sampled_projection_pre_goal_target_present"]


def test_on_plane_target_satisfies_necessary_condition():
    result = classify_reachable_target_progress(
        [[True, False]], [[0.005, 10.0]], tolerance_m=0.01
    )
    assert result["reachable_target_on_goal_plane_count"] == 1
    assert result["sampled_projection_pre_goal_target_present"]


@pytest.mark.parametrize("tolerance", [-0.1, float("nan")])
def test_invalid_tolerance_fails_closed(tolerance):
    with pytest.raises(ValueError, match="tolerance"):
        classify_reachable_target_progress([[True]], [[0.0]], tolerance_m=tolerance)


def test_shape_mismatch_fails_closed():
    with pytest.raises(ValueError, match="identical 2D shapes"):
        classify_reachable_target_progress([[True]], [0.0], tolerance_m=0.01)


def test_target_partition_separates_footprint_and_connectivity_failures():
    target = np.array([[True, True, True, False]])
    free = np.array([[False, True, True, True]])
    components = np.array([[0, 1, 2, 2]])
    progress = np.array([[-1.0, 0.0, 1.0, -2.0]])
    result = partition_target_space(
        target, free, components, progress,
        start_component=1, tolerance_m=0.01,
    )
    assert result["sampled_footprint_infeasible"]["count"] == 1
    assert result["sampled_free_start_component"]["on_goal_plane_count"] == 1
    assert result["sampled_free_other_component"]["after_goal_count"] == 1
    assert result["raw_target"]["count"] == 3
    assert result["partition_count_matches_raw_target"]
