from __future__ import annotations

import math

import pytest

from arena_evaluation.forward_no_reverse_smoke import (
    CONFIG,
    StrictForwardConfig,
    _smooth_steering_metadata,
    _wrap,
)


def test_strict_forward_protocol_is_derived_and_forbids_reverse_or_rotation():
    assert CONFIG.minimum_turning_radius_m == pytest.approx(0.40)
    assert CONFIG.maximum_curvature_per_m == pytest.approx(2.50)
    assert CONFIG.max_steering_angle_rad == pytest.approx(math.atan(0.5 / 0.4))
    assert CONFIG.allow_reverse is False
    assert CONFIG.allow_in_place_rotation is False
    assert CONFIG.motion_model == "forward_only_dubins"


def test_strict_protocol_rejects_in_place_rotation_or_wrong_sampling():
    with pytest.raises(ValueError):
        StrictForwardConfig(allow_in_place_rotation=True)
    with pytest.raises(ValueError):
        StrictForwardConfig(sample_spacing_m=0.1)


def test_wrap_has_no_negative_pi_drift():
    assert -math.pi <= _wrap(9.0) < math.pi
    assert _wrap(math.pi) == pytest.approx(-math.pi)


def test_steering_metadata_has_bounded_adjacent_jumps():
    points = [
        {"x": 0.0, "y": 0.0, "yaw": 0.0, "steering": 0.0},
        {"x": 0.05, "y": 0.0, "yaw": 0.0, "steering": math.radians(50.0)},
        {"x": 0.1, "y": 0.0, "yaw": 0.0, "steering": math.radians(-50.0)},
        {"x": 0.15, "y": 0.0, "yaw": 0.0, "steering": 0.0},
    ]
    _smooth_steering_metadata(points)
    jumps = [abs(points[i]["steering"] - points[i - 1]["steering"]) for i in range(1, len(points))]
    assert max(jumps) <= CONFIG.steering_continuity_step_rad + 1e-9

