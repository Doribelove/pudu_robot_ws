from __future__ import annotations

import math

import numpy as np
import pytest
import yaml
from PIL import Image

from arena_evaluation.unified_four_backends_smoke import (
    ALGORITHMS,
    FOOTPRINT,
    MAX_PATH_SAMPLE_SPACING_M,
    MapContext,
    PlanResult,
    _annotate_geometric_metadata,
    _enrich_path,
    _record_backend_call,
    backend_availability,
    plan_grid_astar,
    plan_local_smac,
    plan_ompl,
    unavailable_plan,
    validate_path,
)
from arena_evaluation.planner_benchmark.map_utils import HospitalMap, sha256_file
from arena_evaluation.planner_benchmark.models import Query
from arena_evaluation.topology import preprocess_static_map


def test_backend_contract_does_not_alias_planner_names():
    specs = backend_availability()
    assert tuple(specs) == ALGORITHMS
    assert len({spec.backend for spec in specs.values()}) == len(ALGORITHMS)


def test_unavailable_mature_backend_returns_structured_failure_without_path():
    spec = backend_availability()["geometric_rrt_star"]
    if spec.available:
        return
    result = unavailable_plan(spec, source="l2")
    assert result.planner_success is False
    assert result.points is None
    assert result.failure_code == "BACKEND_UNAVAILABLE"
    assert result.planner_backend == "OMPL geometric::RRTstar"


def test_strict_protocol_constants_are_frozen():
    from arena_evaluation import unified_four_backends_smoke as smoke

    assert smoke.FOOTPRINT == [
        [0.255, 0.215], [0.255, -0.215], [-0.255, -0.215], [-0.255, 0.215]
    ]
    assert set(smoke.MAP_PATHS) == {"hospital_005", "hospital_boundary_100x100_005"}


def _context(tmp_path, *, wall=False):
    pixels = np.full((200, 200), 254, dtype=np.uint8)
    pixels[[0, -1], :] = 0
    pixels[:, [0, -1]] = 0
    if wall:
        pixels[:, 98:102] = 0
    image = tmp_path / ("wall.pgm" if wall else "free.pgm")
    Image.fromarray(pixels).save(image)
    config = tmp_path / ("wall.yaml" if wall else "free.yaml")
    config.write_text(yaml.safe_dump({
        "image": image.name, "resolution": 0.05, "origin": [-5.0, -5.0, 0.0],
        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196,
    }))
    hospital_map = HospitalMap.load(config)
    _, free, distance, _ = preprocess_static_map(
        hospital_map, FOOTPRINT, padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    return MapContext(
        "test", hospital_map, free, distance,
        sha256_file(image), sha256_file(config), config,
    )


def _path(samples):
    result = []
    for x, y, yaw, steering in samples:
        result.append({
            "x": x, "y": y, "yaw": yaw, "steering": steering,
            "motion_direction": "forward", "source": "scenario",
            "planner_backend": "scenario_fixture", "backend_version": "1",
        })
    _enrich_path(result, "test")
    return result


def test_scenario_straight_is_strictly_valid(tmp_path):
    context = _context(tmp_path)
    points = _path([(x, -2.0, 0.0, 0.0) for x in np.linspace(-3.0, 3.0, 121)])
    metrics = validate_path(context, Query("straight", [-3.0, -2.0, 0.0], [3.0, -2.0, 0.0]), points)
    assert metrics["static_footprint_valid"] and metrics["kinematic_valid"]


def test_grid_diagonal_sample_spacing_is_continuous(tmp_path):
    context = _context(tmp_path)
    points = _path([
        (-2.0 + 0.05 * i, -2.0 + 0.05 * i, math.pi / 4.0, 0.0)
        for i in range(41)
    ])
    query = Query("diagonal", [-2.0, -2.0, math.pi / 4.0], [0.0, 0.0, math.pi / 4.0])
    metrics = validate_path(context, query, points)
    assert math.sqrt(2.0) * 0.05 < MAX_PATH_SAMPLE_SPACING_M
    assert metrics["position_discontinuity_count"] == 0


def test_footprint_collision_is_sampled_between_path_points(tmp_path):
    context = _context(tmp_path, wall=True)
    points = _path([(-1.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)])
    metrics = validate_path(context, Query("swept", [-1.0, 0.0, 0.0], [1.0, 0.0, 0.0]), points)
    assert not metrics["static_footprint_valid"]


def test_geometric_metadata_reports_sharp_turn_steering_and_direction():
    points = [
        {"x": 0.0, "y": 0.0, "yaw": math.pi, "steering": 0.0, "motion_direction": "forward"},
        {"x": 0.1, "y": 0.0, "yaw": 0.0, "steering": 0.0, "motion_direction": "forward"},
        {"x": 0.1, "y": 0.1, "yaw": math.pi / 2.0, "steering": 0.0, "motion_direction": "forward"},
    ]
    _annotate_geometric_metadata(points)
    assert points[0]["motion_direction"] == "reverse"
    assert abs(points[1]["steering"]) > math.radians(45.0)


def test_backend_call_records_actual_invocation_state():
    diagnostics = {"backend_calls": []}
    result = PlanResult(
        planner_success=False, failure_code="NO_EXACT_SOLUTION",
        planner_backend="OMPL control::SST", backend_version="1.5.2",
        diagnostics={"backend_called": True},
    )
    _record_backend_call(diagnostics, result, "l3_local_kinodynamic_fallback", fallback_trigger="ACTION_ABORTED")
    assert diagnostics["backend_calls"] == [{
        "role": "l3_local_kinodynamic_fallback",
        "planner_backend": "OMPL control::SST",
        "backend_version": "1.5.2",
        "called": True,
        "planner_success": False,
        "failure_code": "NO_EXACT_SOLUTION",
        "fallback_trigger": "ACTION_ABORTED",
    }]


@pytest.mark.parametrize("turn", [1.0, -1.0], ids=["left_turn", "right_turn"])
def test_scenario_single_turn_respects_radius(tmp_path, turn):
    context = _context(tmp_path)
    radius = 1.0
    angles = np.linspace(0.0, math.pi / 2.0, 33)
    points = _path([(radius * math.sin(a), -1.0 + turn * radius * (1.0 - math.cos(a)), turn * a, math.atan(0.5 * turn / radius)) for a in angles])
    query = Query("turn", list(points[0][key] for key in ("x", "y", "yaw")), list(points[-1][key] for key in ("x", "y", "yaw")))
    metrics = validate_path(context, query, points)
    assert metrics["kinematic_valid"]
    assert metrics["maximum_curvature"] <= 1.01


def test_scenario_s_bend_is_strictly_valid(tmp_path):
    context = _context(tmp_path)
    xs = np.linspace(-3.0, 3.0, 121)
    ys = 0.25 * np.sin(math.pi * xs / 3.0)
    slopes = 0.25 * math.pi / 3.0 * np.cos(math.pi * xs / 3.0)
    points = _path([(float(x), float(y), math.atan(float(slope)), math.atan(0.5 * float(slope))) for x, y, slope in zip(xs, ys, slopes)])
    query = Query("s", list(points[0][key] for key in ("x", "y", "yaw")), list(points[-1][key] for key in ("x", "y", "yaw")))
    assert validate_path(context, query, points)["kinematic_valid"]


def test_scenario_narrow_or_blocked_passage_returns_no_grid_path(tmp_path):
    context = _context(tmp_path, wall=True)
    query = Query("blocked", [-3.0, 0.0, 0.0], [3.0, 0.0, 0.0])
    result = plan_grid_astar(context, query, 0.5)
    assert not result.planner_success
    assert result.failure_code in {"NO_PATH", "TIMEOUT"}


def test_scenario_different_endpoint_yaw_is_checked(tmp_path):
    context = _context(tmp_path)
    points = _path([(x, 2.0, 0.0, 0.0) for x in np.linspace(-2.0, 2.0, 81)])
    metrics = validate_path(context, Query("yaw", [-2.0, 2.0, 0.0], [2.0, 2.0, math.pi / 2.0]), points)
    assert metrics["failure_code"] == "ENDPOINT_YAW_ERROR"


def test_scenario_static_obstacle_collision_is_separate_from_kinematics(tmp_path):
    context = _context(tmp_path, wall=True)
    points = _path([(x, 0.0, 0.0, 0.0) for x in np.linspace(-2.0, 2.0, 81)])
    metrics = validate_path(context, Query("collision", [-2.0, 0.0, 0.0], [2.0, 0.0, 0.0]), points)
    assert not metrics["static_footprint_valid"]
    assert metrics["kinematic_valid"]
    assert metrics["failure_code"] == "STATIC_FOOTPRINT_COLLISION"


def test_scenario_reverse_and_in_place_rotation_are_rejected(tmp_path):
    context = _context(tmp_path)
    points = _path([(0.0, 0.0, 0.0, 0.0), (-0.05, 0.0, 0.0, 0.0), (-0.05, 0.0, 0.5, 0.0)])
    metrics = validate_path(context, Query("forbidden", [0.0, 0.0, 0.0], [-0.05, 0.0, 0.5]), points)
    assert metrics["reverse_distance_m"] > 0.0
    assert metrics["in_place_rotation_count"] == 1
    assert not metrics["kinematic_valid"]


def test_compiled_ompl_adapter_version_is_traceable_when_available():
    spec = backend_availability()["geometric_rrt_star"]
    if spec.available:
        assert spec.version.count(".") == 2


@pytest.mark.parametrize(
    ("algorithm", "start", "goal", "expect_samples"),
    [
        ("geometric_rrt_star", [-2.0, -2.0, 0.0], [2.0, -2.0, 0.0], True),
        ("kinodynamic_rrt", [-1.0, 2.0, 0.0], [1.0, 2.0, 0.0], False),
    ],
)
def test_actual_ompl_backends_return_nonempty_straight_paths(
    tmp_path, algorithm, start, goal, expect_samples,
):
    context = _context(tmp_path)
    spec = backend_availability()[algorithm]
    if not spec.available:
        pytest.skip(spec.reason)
    result = plan_ompl(
        context, Query(f"{algorithm}_straight", start, goal), algorithm, spec,
        0.5, source="integration_test",
    )
    assert result.planner_success, result.failure_code
    assert result.points
    assert result.diagnostics["backend_called"] is True
    assert result.generated_states > 0
    assert (result.samples is not None) is expect_samples
    assert result.rewires is None


def test_actual_smac_dubin_backend_returns_valid_straight_path(tmp_path, monkeypatch):
    context = _context(tmp_path)
    spec = backend_availability()["hybrid_astar"]
    if not spec.available:
        pytest.skip(spec.reason)
    monkeypatch.setenv("ROS_DOMAIN_ID", "194")
    query = Query("smac_straight", [-2.0, -2.0, 0.0], [2.0, -2.0, 0.0])
    try:
        result = plan_local_smac(context, query, spec, context.free_mask, tmp_path / "smac")
        _enrich_path(result.points, "test")
        metrics = validate_path(context, query, result.points)
        assert result.planner_success, result.failure_code
        assert result.points
        assert result.diagnostics["backend_called"] is True
        assert metrics["static_footprint_valid"]
        assert metrics["kinematic_valid"]
    finally:
        import rclpy

        if rclpy.ok():
            rclpy.shutdown()
