import math

import numpy as np
import pytest

from arena_evaluation.semantic_path_necessity_audit import (
    FOOTPRINT_AREA_M2,
    audit_path_necessity,
    padded_footprint_overlap_area,
)


def line(start, goal, yaw, count=101):
    xy = np.linspace(start, goal, count)
    return np.column_stack((xy, np.full(count, yaw)))


def test_straight_directed_path_has_no_suspect_excursion():
    path = line((0.0, 0.0), (2.0, 0.0), 0.0)
    target = path[:, 0] < 1.0
    result = audit_path_necessity(path, start=(0.0, 0.0), goal=(2.0, 0.0), target_mask=target)
    assert result["audit_passed"]
    assert result["failure_codes"] == []
    assert result["maximum_goal_plane_overshoot_m"] == 0.0
    assert result["cumulative_backward_route_progress_m"] == 0.0
    assert result["target_sample_attribution"]["after_goal_plane_count"] == 0


@pytest.mark.parametrize(
    "kwargs,code",
    [
        ({"path": [[0.0, 0.0], [1.0, 0.0]], "start": (0.0, 0.0),
          "goal": (1.0, 0.0), "target_mask": [False, False]}, "PATH_YAW_MISSING"),
        ({"path": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], "start": (0.0, 0.0),
          "goal": (1.0, 0.0), "target_mask": None}, "TARGET_MASK_MISSING"),
        ({"path": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], "start": (0.0, 0.0),
          "goal": (1.0, 0.0), "target_mask": [False]}, "TARGET_MASK_LENGTH_MISMATCH"),
        ({"path": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], "start": (0.1, 0.0),
          "goal": (1.0, 0.0), "target_mask": [False, False]}, "PATH_START_MISMATCH"),
    ],
)
def test_missing_or_unbound_inputs_fail_closed(kwargs, code):
    result = audit_path_necessity(**kwargs)
    assert not result["audit_passed"]
    assert not result["input_complete"]
    assert code in result["failure_codes"]
    assert result["status"] == "INVALID_INPUT"


def test_route_is_oriented_to_query_before_terminal_plane_is_used():
    path = line((0.0, 0.0), (0.0, 2.0), math.pi / 2)
    result = audit_path_necessity(
        path,
        start=(0.0, 0.0),
        goal=(0.0, 2.0),
        target_mask=np.zeros(len(path), dtype=bool),
        route_polyline=[(0.0, 2.0), (0.0, 1.0), (0.0, 0.0)],
    )
    assert result["audit_passed"]
    assert result["route_reversed_for_query"]
    assert result["direction_source"] == "oriented_route_terminal_tangent"
    assert result["direction_xy"] == pytest.approx([0.0, 1.0])


def test_terminal_tangent_ignores_one_cell_diagonal_endpoint_attachment():
    path = line((0.0, 0.0), (0.05, 1.05), math.atan2(1.05, 0.05))
    route = [(0.0, value) for value in np.linspace(0.0, 1.0, 11)]
    route.append((0.05, 1.05))
    result = audit_path_necessity(
        path,
        start=path[0, :2],
        goal=path[-1, :2],
        target_mask=np.zeros(len(path), dtype=bool),
        route_polyline=route,
    )
    assert result["input_complete"]
    assert result["terminal_tangent_window_m"] >= 0.53
    assert result["terminal_tangent_window_source"] == "one_full_padded_footprint_length"
    assert result["direction_xy"][1] > 0.99
    assert abs(result["direction_xy"][0]) < 0.1


def test_unbound_route_endpoint_fails_closed():
    path = line((0.0, 0.0), (2.0, 0.0), 0.0)
    result = audit_path_necessity(
        path,
        start=path[0, :2],
        goal=path[-1, :2],
        target_mask=np.zeros(len(path), dtype=bool),
        route_polyline=[(10.0, 10.0), (11.0, 10.0)],
    )
    assert not result["audit_passed"]
    assert not result["input_complete"]
    assert "ROUTE_ENDPOINT_UNBOUND" in result["failure_codes"]


def test_route_goal_gap_must_follow_terminal_ray():
    path = line((0.0, 0.0), (1.0, 1.0), math.pi / 4)
    result = audit_path_necessity(
        path,
        start=path[0, :2],
        goal=path[-1, :2],
        target_mask=np.zeros(len(path), dtype=bool),
        route_polyline=[(0.0, 0.0), (0.0, 0.5)],
    )
    assert not result["audit_passed"]
    assert not result["input_complete"]
    assert "ROUTE_GOAL_RAY_UNBOUND" in result["failure_codes"]


def test_real_footprint_geometry_not_tuned_center_distance_controls_overlap():
    # With north/south headings, 0.44 m is inside the true 0.45 m footprint
    # width and 0.46 m is outside it.  No independent proximity threshold is
    # involved.
    overlap = padded_footprint_overlap_area((0.0, 0.0, math.pi / 2),
                                            (0.44, 0.0, -math.pi / 2))
    separate = padded_footprint_overlap_area((0.0, 0.0, math.pi / 2),
                                             (0.46, 0.0, -math.pi / 2))
    assert overlap == pytest.approx(0.01 * 0.53)
    assert overlap / FOOTPRINT_AREA_M2 > 0.0
    assert separate == pytest.approx(0.0, abs=1e-12)


def _parallel_return(offset=0.04):
    outward = line((0.0, 0.0), (0.0, 2.0), math.pi / 2, 81)
    across = line((0.0, 2.0), (offset, 2.0), 0.0, 5)[1:]
    returning = line((offset, 2.0), (offset, 0.5), -math.pi / 2, 61)[1:]
    return np.vstack((outward, across, returning))


def _subdivide(path, pieces=5):
    result = [path[0]]
    for first, second in zip(path, path[1:]):
        delta_yaw = (second[2] - first[2] + math.pi) % (2 * math.pi) - math.pi
        delta = np.array((second[0] - first[0], second[1] - first[1], delta_yaw))
        for index in range(1, pieces + 1):
            pose = first + delta * (index / pieces)
            pose[2] = (pose[2] + math.pi) % (2 * math.pi) - math.pi
            result.append(pose)
    return np.asarray(result)


def test_nonintersecting_parallel_counterflow_is_caught_by_footprint_overlap():
    path = _parallel_return()
    target = np.zeros(len(path), dtype=bool)
    target[60:100] = True
    result = audit_path_necessity(
        path,
        start=path[0, :2],
        goal=path[-1, :2],
        target_mask=target,
        route_polyline=[(0.0, 0.0), (0.0, 0.5)],
    )
    assert not result["audit_passed"]
    assert "NONLOCAL_REVERSE_FOOTPRINT_OVERLAP" in result["failure_codes"]
    assert result["nonlocal_reverse_footprint_overlap_count"] > 0
    event = max(result["nonlocal_reverse_footprint_overlaps"],
                key=lambda value: value["footprint_overlap_fraction"])
    assert event["center_distance_m"] < 0.05
    assert event["heading_delta_abs_rad"] > math.radians(170)
    assert event["footprint_overlap_fraction"] > 0.85


def test_overshoot_regression_and_target_credit_are_independent_failure_codes():
    path = _parallel_return()
    target = path[:, 1] > 1.25
    result = audit_path_necessity(
        path,
        start=path[0, :2],
        goal=path[-1, :2],
        target_mask=target,
        route_polyline=[(0.0, 0.0), (0.0, 0.5)],
    )
    assert {
        "GOAL_PLANE_OVERSHOOT",
        "BACKWARD_ROUTE_PROGRESS",
        "NONLOCAL_REVERSE_FOOTPRINT_OVERLAP",
        "TARGET_CREDIT_AFTER_GOAL_PLANE",
    }.issubset(result["failure_codes"])
    assert result["maximum_goal_plane_overshoot_m"] == pytest.approx(1.5)
    assert result["cumulative_backward_route_progress_m"] == pytest.approx(1.5)
    assert result["nonlocal_backward_route_progress_m"] == pytest.approx(1.5)
    attribution = result["target_sample_attribution"]
    assert attribution["target_sample_count"] > 0
    assert attribution["after_goal_plane_count"] == attribution["target_sample_count"]
    assert attribution["after_goal_plane_ratio"] == 1.0


def test_target_on_goal_plane_is_not_counted_after_goal():
    path = line((0.0, 0.0), (1.0, 0.0), 0.0, 41)
    target = np.zeros(len(path), dtype=bool)
    target[-1] = True
    result = audit_path_necessity(path, start=path[0, :2], goal=path[-1, :2], target_mask=target)
    assert result["audit_passed"]
    assert result["target_sample_attribution"] == {
        "target_sample_count": 1,
        "before_goal_plane_count": 0,
        "on_goal_plane_count": 1,
        "after_goal_plane_count": 0,
        "after_goal_plane_ratio": 0.0,
    }


def test_natural_wide_hairpin_without_overshoot_or_footprint_revisit_passes():
    path = np.asarray([
        (0.0, 0.0, 0.0),
        (3.0, 0.0, 0.0),
        (3.5, 0.5, math.pi / 2),
        (3.0, 1.0, math.pi),
        (0.0, 1.0, math.pi),
    ])
    result = audit_path_necessity(
        path,
        start=path[0, :2],
        goal=path[-1, :2],
        target_mask=np.zeros(len(path), dtype=bool),
        route_polyline=[(0.0, 0.0), (0.0, 1.0)],
    )
    assert result["audit_passed"]
    assert result["status"] == "NO_SUSPECT_EXCURSION"
    assert result["nonlocal_reverse_footprint_overlap_count"] == 0


def test_short_endpoint_attachment_regression_is_diagnostic_not_failure():
    path = np.asarray([
        (0.0, 0.0, -math.pi / 2),
        (0.0, -0.25, 0.0),
        (0.35, 0.25, math.pi / 3),
        (0.0, 1.0, math.pi / 2),
    ])
    result = audit_path_necessity(
        path,
        start=path[0, :2],
        goal=path[-1, :2],
        target_mask=np.zeros(len(path), dtype=bool),
        route_polyline=[(0.0, 0.0), (0.0, 1.0)],
    )
    assert result["audit_passed"]
    assert result["cumulative_backward_route_progress_m"] == pytest.approx(0.25)
    assert result["nonlocal_backward_route_progress_m"] == 0.0
    assert result["backward_progress_runs"][0]["nonlocal"] is False


def test_repeated_short_regressions_fail_when_cumulative_pi_rmin_is_reached():
    poses = [(0.0, 0.0, math.pi / 2)]
    y = 0.0
    for _ in range(6):
        y -= 0.22
        poses.append((0.8, y, -math.pi / 2))
        y += 0.42
        poses.append((0.8, y, math.pi / 2))
    poses.append((0.0, 2.0, math.pi / 2))
    path = np.asarray(poses)
    result = audit_path_necessity(
        path,
        start=path[0, :2],
        goal=path[-1, :2],
        target_mask=np.zeros(len(path), dtype=bool),
        route_polyline=[(0.0, 0.0), (0.0, 2.0)],
    )
    assert not result["audit_passed"]
    assert "BACKWARD_ROUTE_PROGRESS" in result["failure_codes"]
    assert result["repeated_local_regression_limit_exceeded"]


def test_geometric_gate_is_invariant_to_equivalent_input_subdivision():
    sparse = np.asarray([
        (0.0, 0.0, math.pi / 2),
        (0.0, 2.0, math.pi / 2),
        (0.04, 2.0, -math.pi / 2),
        (0.04, 1.0, -math.pi / 2),
    ])
    dense = _subdivide(sparse, 8)
    results = [audit_path_necessity(
        path,
        start=path[0, :2],
        goal=path[-1, :2],
        target_mask=np.zeros(len(path), dtype=bool),
        route_polyline=[(0.0, 0.0), (0.0, 1.0)],
    ) for path in (sparse, dense)]
    assert results[0]["audit_passed"] == results[1]["audit_passed"]
    assert results[0]["failure_codes"] == results[1]["failure_codes"]
    assert results[0]["maximum_goal_plane_overshoot_m"] == pytest.approx(
        results[1]["maximum_goal_plane_overshoot_m"])
    assert results[0]["cumulative_backward_route_progress_m"] == pytest.approx(
        results[1]["cumulative_backward_route_progress_m"])
    assert all(result["nonlocal_reverse_footprint_overlap_count"] > 0 for result in results)
