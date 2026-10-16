from __future__ import annotations

import math

import numpy as np
import pytest

from arena_evaluation.se2_semantic_guide import (
    FAILURE_CODES,
    EffectiveMasterCollisionChecker,
    ExplicitSE2GuideOracle,
    SE2GuidePolicy,
    bin_to_yaw,
    build_smac_dubin_primitives,
    sample_dubins_path,
    sample_primitive,
    yaw_to_bin,
)
from test_two_layer_v2_semantic_r1 import cell, fixture


def _uniform_guide(hospital_map):
    shape = hospital_map.occupancy.shape
    master = np.zeros(shape, dtype=np.uint8)
    master[hospital_map.occupancy == 100] = 254
    master[hospital_map.occupancy < 0] = 255
    labels = np.ones(shape, dtype=np.int32)
    error = np.zeros(shape, dtype=np.float32)
    correct = np.ones(shape, dtype=bool)
    return master, labels, error, correct


def test_r4_policy_freezes_48_bin_dubin_and_hard_curvature():
    policy = SE2GuidePolicy()
    policy.validate()
    primitives = build_smac_dubin_primitives(policy)
    assert [item.name for item in primitives] == [
        "FORWARD", "FORWARD_LEFT", "FORWARD_RIGHT",
    ]
    assert [item.delta_yaw_bins for item in primitives] == [0, 2, -2]
    assert primitives[1].curvature_1pm == pytest.approx(2.5)
    assert primitives[2].curvature_1pm == pytest.approx(-2.5)
    assert all(item.chord_length_m > 0.05 for item in primitives)
    with pytest.raises(ValueError, match="exactly 48"):
        SE2GuidePolicy(yaw_bins=72).validate()


def test_r4_primitives_are_forward_only_and_never_rotate_in_place():
    policy = SE2GuidePolicy()
    for primitive in build_smac_dubin_primitives(policy):
        samples = sample_primitive((0.0, 0.0, 0.0), primitive, policy)
        assert samples
        previous = (0.0, 0.0, 0.0)
        for pose in samples:
            dx, dy = pose[0] - previous[0], pose[1] - previous[1]
            assert math.hypot(dx, dy) > 0.0
            assert dx * math.cos(previous[2]) + dy * math.sin(previous[2]) >= -1.0e-9
            previous = pose
        assert abs(primitive.curvature_1pm) <= policy.maximum_curvature_1pm


def test_r4_continuous_dubin_connector_is_endpoint_exact_and_forward_only():
    result = sample_dubins_path((0.0, 0.0, 0.0), (4.0, 1.0, math.pi / 2.0), 0.4, 0.025)
    assert result is not None
    _word, samples, length = result
    assert samples[-1] == pytest.approx((4.0, 1.0, math.pi / 2.0), abs=1.0e-12)
    assert length >= math.hypot(4.0, 1.0)
    previous = (0.0, 0.0, 0.0)
    for pose in samples:
        dx, dy = pose[0] - previous[0], pose[1] - previous[1]
        assert dx * math.cos(previous[2]) + dy * math.sin(previous[2]) >= -1.0e-7
        previous = pose


def test_r4_complete_footprint_detects_off_center_collision():
    hospital_map, _semantic_map, _raster = fixture()
    master, _labels, _error, _correct = _uniform_guide(hospital_map)
    pose = (4.0, 2.5, 0.0)
    obstacle_cell = cell(hospital_map, 4.20, 2.50)
    master[obstacle_cell] = 254
    checker = EffectiveMasterCollisionChecker(hospital_map, master, SE2GuidePolicy())
    assert int(master[cell(hospital_map, pose[0], pose[1])]) == 0
    valid, _margin = checker.pose_status(pose)
    assert valid is False
    assert checker.full_polygon_checks == 1


def test_r4_mirror_oracle_is_deterministic_endpoint_exact_and_metric_valid():
    hospital_map, _semantic_map, _raster = fixture()
    master, labels, error, correct = _uniform_guide(hospital_map)
    policy = SE2GuidePolicy(analytic_expansion_max_length_m=10.0, timeout_s=2.0)
    cases = (
        ("forward", (0.8, 2.5, 0.0), (8.0, 2.5, 0.0)),
        ("reverse", (8.0, 2.5, math.pi), (0.8, 2.5, math.pi)),
    )
    hashes = []
    for query_id, start, goal in cases:
        oracle = ExplicitSE2GuideOracle(
            hospital_map, master, labels, error, correct, [1],
            policy=policy, binding={"query_id": query_id},
            reference_polylines=[(start[:2], goal[:2])],
        )
        first = oracle.search(query_id, start, goal)
        second = ExplicitSE2GuideOracle(
            hospital_map, master, labels, error, correct, [1],
            policy=policy, binding={"query_id": query_id},
            reference_polylines=[(start[:2], goal[:2])],
        ).search(query_id, start, goal)
        assert first.witness_exists and second.witness_exists
        assert first.path[-1][:2] == pytest.approx(goal[:2], abs=1.0e-12)
        assert math.remainder(first.path[-1][2] - goal[2], 2.0 * math.pi) == pytest.approx(
            0.0, abs=1.0e-12,
        )
        assert first.diagnostics["path_hash"] == second.diagnostics["path_hash"]
        assert first.diagnostics["semantic_metric_gate_passed"] is True
        assert first.diagnostics["reverse_distance_m"] == 0.0
        assert first.diagnostics["in_place_rotation_count"] == 0
        assert first.diagnostics["maximum_curvature_1pm"] <= 2.5
        assert first.diagnostics["explicit_se2_reference_consumed"] is True
        assert first.diagnostics["guide_yaw_bin_coverage"]
        hashes.append(first.diagnostics["path_hash"])
    assert hashes[0] != hashes[1]


def test_r4_binding_hash_is_sensitive_and_failure_codes_are_explicit():
    hospital_map, _semantic_map, _raster = fixture()
    master, labels, error, correct = _uniform_guide(hospital_map)
    first = ExplicitSE2GuideOracle(
        hospital_map, master, labels, error, correct, [1], binding={"route_hash": "a"},
    )
    second = ExplicitSE2GuideOracle(
        hospital_map, master, labels, error, correct, [1], binding={"route_hash": "b"},
    )
    assert first.binding_hash != second.binding_hash
    result = first.search("outside", (-100.0, -100.0, 0.0), (1.0, 1.0, 0.0))
    assert result.failure_code == "START_ATTACH_FAILED"
    assert result.failure_code in FAILURE_CODES


def test_r4_yaw_bin_roundtrip_and_mirror_symmetry():
    for index in range(48):
        assert yaw_to_bin(bin_to_yaw(index), 48) == index
    assert yaw_to_bin(math.pi / 2.0, 48) == 12
    assert yaw_to_bin(-math.pi / 2.0, 48) == 36
