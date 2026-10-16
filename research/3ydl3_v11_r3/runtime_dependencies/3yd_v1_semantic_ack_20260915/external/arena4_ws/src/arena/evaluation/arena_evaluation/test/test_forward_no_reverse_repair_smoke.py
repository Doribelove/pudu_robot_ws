from __future__ import annotations

import math
import inspect

import pytest

from arena_evaluation import forward_no_reverse_repair_smoke as repair
from arena_evaluation.forward_no_reverse_repair_smoke import CONFIG, _rollout, _strict_dubins


def test_repair_protocol_is_strict_and_derived():
    assert CONFIG.minimum_turning_radius_m == pytest.approx(0.4)
    assert CONFIG.maximum_curvature_per_m == pytest.approx(2.5)
    assert CONFIG.max_steering_rad == pytest.approx(math.atan(0.5 / 0.4))
    assert CONFIG.angle_bins == 72
    assert CONFIG.sample_spacing_m <= 0.05
    assert CONFIG.allow_reverse is False
    assert CONFIG.allow_in_place_rotation is False


def test_corrected_dubins_sampler_reaches_pose_and_keeps_forward_fields():
    points, reason, word = _strict_dubins((0.0, 0.0, 0.0), (2.0, 0.0, 0.0))
    assert reason == "OK"
    assert word is not None
    assert points
    assert all(p["motion_direction"] == "forward" for p in points)
    assert all("steering" in p for p in points)
    assert points[-1]["x"] == pytest.approx(2.0, abs=1e-6)


class _NeverCollides:
    @staticmethod
    def footprint_collision(*_args, **_kwargs):
        return False


class _FreeContext:
    hospital_map = _NeverCollides()


def test_rollout_subsamples_large_steering_change_and_matches_geometry():
    points = _rollout(_FreeContext(), (0.0, 0.0, 0.0), -CONFIG.max_steering_rad, CONFIG.max_steering_rad, 0.25)
    assert points is not None
    assert len(points) >= math.ceil(2.0 * CONFIG.max_steering_rad / CONFIG.jump_tolerance_rad)
    previous = {"x": 0.0, "y": 0.0, "yaw": 0.0, "steering": -CONFIG.max_steering_rad}
    for point in points:
        assert abs(point["steering"] - previous["steering"]) <= CONFIG.jump_tolerance_rad + 1e-12
        distance = math.hypot(point["x"] - previous["x"], point["y"] - previous["y"])
        observed = repair._delta(point["yaw"], previous["yaw"]) / distance
        expected = math.tan(point["steering"]) / CONFIG.wheelbase_m
        assert observed == pytest.approx(expected, abs=CONFIG.steering_geometry_tolerance_per_m)
        previous = point


def test_four_dispatch_targets_are_distinct_and_do_not_call_old_route_generator():
    targets = [repair._astar_kinematic, repair._hybrid_astar, repair._rrt_star, repair._kinodynamic_rrt_star]
    assert len({target.__name__ for target in targets}) == 4
    for target in targets:
        assert "_route_from_grid" not in inspect.getsource(target)
