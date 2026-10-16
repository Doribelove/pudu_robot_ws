from pathlib import Path

import pytest
import yaml

from arena_evaluation.semantic_parking_applicability_r16 import (
    ParkingApplicabilityPolicyR16,
    summarize_station_records,
)
from arena_evaluation.semantic_route_phase_r16 import (
    METHOD_ID,
    WholePrimitiveSelectionPolicyR16,
    whole_edge_semantic_rank,
)
from arena_evaluation.two_layer_v3_semantic_r16_applicability import (
    ARCHITECTURE_ID,
    IMPLEMENTATION_REVISION,
    PROTOCOL_ID,
    _load,
)


CONFIG = (
    Path(__file__).resolve().parents[1]
    / "config/two_layer_v3_semantic_r16_applicability.yaml"
)


def test_r16_identity_and_parent_hash_are_frozen():
    config, r15_config, *_rest = _load(CONFIG)
    assert config["architecture_id"] == ARCHITECTURE_ID == "2A-V3"
    assert config["implementation_revision"] == IMPLEMENTATION_REVISION
    assert config["protocol_id"] == PROTOCOL_ID
    assert r15_config["architecture_id"] == "2A-V3"
    assert config["interpretation"]["changes_r2_metric"] is False
    assert config["interpretation"]["changes_safety_or_kinematics"] is False
    assert (
        config["interpretation"][
            "r15_all_three_parking_witness_gate_is_b_acceptance_gate"
        ]
        is False
    )


def test_applicability_ratio_is_strictly_greater_than_half():
    policy = ParkingApplicabilityPolicyR16()
    exactly_half = [
        {"station_m": index * 0.25, "target_available": index < 2,
         "route_local_best_deviation": 0.1 if index < 2 else 0.4}
        for index in range(4)
    ]
    result = summarize_station_records(exactly_half, policy)
    assert result["target_available_station_ratio"] == 0.5
    assert result["applicable"] is False
    assert result["dispatch"] == "E0_NATIVE_SMAC_FALLBACK"
    assert result["semantic_success_counted_on_fallback"] is False

    majority = [dict(value) for value in exactly_half]
    majority[2]["target_available"] = True
    majority[2]["route_local_best_deviation"] = 0.2
    result = summarize_station_records(majority, policy)
    assert result["target_available_station_ratio"] == 0.75
    assert result["applicable"] is True
    assert result["dispatch"] == "E5_SEMANTIC"


def test_empty_parking_window_fails_closed_to_e0():
    result = summarize_station_records([])
    assert result["applicable"] is False
    assert result["active_parking_station_count"] == 0
    assert result["failure_code"] == "ROUTE_LOCAL_TARGET_COVERAGE_BELOW_CONTRACT"


def test_unavailable_station_runs_are_deterministic():
    records = [
        {"station_m": station, "target_available": available,
         "route_local_best_deviation": 0.1 if available else 0.4}
        for station, available in (
            (0.0, False), (0.25, False), (0.5, True),
            (0.75, False), (1.0, False), (1.5, False),
        )
    ]
    result = summarize_station_records(records)
    assert result["unavailable_station_runs_m"] == [
        [0.0, 0.25], [0.75, 1.0], [1.5, 1.5]
    ]


def test_whole_primitive_policy_is_bounded():
    policy = WholePrimitiveSelectionPolicyR16()
    assert policy.maximum_materialized_edges_per_target_layer == 96
    assert policy.retained_edges_per_target_layer == 12
    with pytest.raises(ValueError):
        WholePrimitiveSelectionPolicyR16(
            maximum_materialized_edges_per_target_layer=4,
            retained_edges_per_target_layer=5,
        )


def test_whole_edge_rank_uses_edge_interior_semantics():
    class Edge:
        target_layer = 1
        target = 2
        dubins_choice = 0
        edge_id = 9
        semantic_error_integral = 0.0
        lane_wrong_m = 0.0
        lane_outside_m = 0.0
        length_m = 1.0

    centered = Edge()
    centered.parking_outside_m = 0.1
    outside = Edge()
    outside.parking_outside_m = 0.8
    assert whole_edge_semantic_rank(centered) < whole_edge_semantic_rank(outside)
    assert METHOD_ID == "phase_owned_whole_primitive_semantic_ranking_r16_v1"


def test_config_preserves_frozen_motion_contract():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    frozen = config["frozen_bindings"]
    assert frozen["yaw_bins"] == 48
    assert frozen["motion_model"] == "DUBIN"
    assert frozen["allow_reverse"] is False
    assert frozen["allow_in_place_rotation"] is False
    assert frozen["minimum_turning_radius_m"] == 0.40
    assert frozen["maximum_curvature_1pm"] == 2.50
