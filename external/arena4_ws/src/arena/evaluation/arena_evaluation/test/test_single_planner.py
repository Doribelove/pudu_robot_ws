from __future__ import annotations

import math

import numpy as np
import pytest
import yaml
from PIL import Image

from arena_evaluation.single_planner_benchmark import (
    ALGORITHMS,
    AckermannConfig,
    MapContext,
    Query,
    _astar,
    _delta,
    _integrate_bicycle,
    _resample,
    validate_path,
)
from arena_evaluation.planner_benchmark.map_utils import HospitalMap, sha256_file
from arena_evaluation.topology import preprocess_static_map


def _context(tmp_path):
    pixels = np.full((120, 120), 254, dtype=np.uint8)
    pixels[55:65, 58:62] = 0
    image = tmp_path / "map.pgm"
    Image.fromarray(pixels).save(image)
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(yaml.safe_dump({
        "image": image.name, "resolution": 0.05, "origin": [-3.0, -3.0, 0.0],
        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196,
    }))
    hospital_map = HospitalMap.load(yaml_path)
    _, free, distance, _ = preprocess_static_map(
        hospital_map,
        [[0.255, 0.215], [0.255, -0.215], [-0.255, -0.215], [-0.255, 0.215]],
        padding_m=0.05,
        safety_margin_m=0.05,
        allow_unknown=False,
    )
    return MapContext("test", hospital_map, free, distance, sha256_file(image), sha256_file(yaml_path), {})


def test_ackermann_radius_and_curvature_boundary():
    config = AckermannConfig()
    assert math.tan(config.max_steering_angle_rad) / config.wheelbase_m == pytest.approx(1.155, abs=0.001)
    assert config.minimum_turning_radius_m == pytest.approx(1.0 / 1.155, abs=0.001)
    assert config.allow_reverse is False
    assert config.allow_in_place_rotation is False


def test_yaw_wrap_delta_is_shortest_turn():
    assert _delta(-math.pi + 0.05, math.pi - 0.05) == pytest.approx(0.1)


def test_bicycle_integrator_is_forward_and_nonrotating():
    points = _integrate_bicycle(0.0, 0.0, 0.0, 0.0, 1.0, 0.5, samples=10)
    assert points[-1][0] == pytest.approx(1.0)
    assert points[-1][1] == pytest.approx(0.0)
    assert points[-1][2] == pytest.approx(0.0)


def test_resample_preserves_endpoint_yaw():
    points = [{"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": 1.0, "y": 0.0, "yaw": math.pi / 2.0}]
    sampled = _resample(points, 0.1)
    assert len(sampled) == 11
    assert sampled[0]["yaw"] == pytest.approx(0.0)
    assert sampled[-1]["yaw"] == pytest.approx(math.pi / 2.0)


def test_astar_path_is_planner_success_but_ackermann_is_checked_separately(tmp_path):
    context = _context(tmp_path)
    query = Query("q", [-2.0, -2.0, 0.0], [2.0, 2.0, 0.0])
    result = _astar(context, query, 5.0)
    assert result.planner_success
    assert result.points
    metrics = validate_path(context, query, result.points, AckermannConfig())
    assert "static_footprint_valid" in metrics
    assert "kinematic_valid" in metrics


def test_algorithm_names_are_explicit_and_no_ompl_claim():
    assert ALGORITHMS == ("astar", "hybrid_astar", "rrt_star_dubins_surrogate", "kinodynamic_rrt_star_bicycle")
