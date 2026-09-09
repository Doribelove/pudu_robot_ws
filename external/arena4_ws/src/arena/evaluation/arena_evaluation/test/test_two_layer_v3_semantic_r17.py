from pathlib import Path

import numpy as np
import pytest
import yaml

from arena_evaluation.semantic_parking_aisle_r17 import (
    METHOD_ID,
    ParkingAisleFieldResultR17,
    ParkingAislePolicyR17,
    _active_interval,
)
from arena_evaluation.two_layer_v3_semantic_r17_parking_aisle import (
    ARCHITECTURE_ID,
    IMPLEMENTATION_REVISION,
    PROTOCOL_ID,
    _load,
)


CONFIG = (
    Path(__file__).resolve().parents[1]
    / "config/two_layer_v3_semantic_r17_parking_aisle.yaml"
)


def test_r17_identity_parent_chain_and_frozen_bindings():
    config, r15_config, algorithm, *_rest = _load(CONFIG)
    assert config["architecture_id"] == ARCHITECTURE_ID == "2A-V3"
    assert config["implementation_revision"] == IMPLEMENTATION_REVISION
    assert config["protocol_id"] == PROTOCOL_ID
    assert r15_config["architecture_id"] == "2A-V3"
    assert algorithm["frozen_bindings"]["yaw_bins"] == 48
    assert algorithm["frozen_bindings"]["motion_model"] == "DUBIN"
    frozen = config["frozen_bindings"]
    assert frozen["allow_reverse"] is False
    assert frozen["allow_in_place_rotation"] is False
    assert frozen["minimum_turning_radius_m"] == 0.40
    assert frozen["maximum_curvature_1pm"] == 2.50


def test_r17_candidate_metric_does_not_rewrite_r2_contract():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    metric = config["parking_aisle_metric"]
    assert metric["metric_revision"] == (
        "parking-route-local-aisle-normalized-r3-research"
    )
    assert metric["relation_to_r2"].startswith("separate_candidate_metric")
    assert config["gates"]["r2_component_metric"] == "report_only_not_redefined"
    assert config["interpretation"]["changes_r2_component_metric"] is False
    assert config["interpretation"]["center_preference_within_aisle_remains_soft"]


def test_r17_transition_window_matches_frozen_r2_contract():
    assert _active_interval(12.0) == (0.0, 12.0)
    assert _active_interval(12.01) == pytest.approx((6.0, 6.01))
    assert _active_interval(20.0) == (6.0, 14.0)


def test_r17_policy_rejects_invalid_or_relaxed_thresholds():
    with pytest.raises(ValueError):
        ParkingAislePolicyR17(station_spacing_m=0.0)
    with pytest.raises(ValueError):
        ParkingAislePolicyR17(target_deviation_max=1.0)
    with pytest.raises(ValueError):
        ParkingAislePolicyR17(required_target_station_ratio_exclusive=0.0)


def test_r17_field_summary_is_small_and_deterministic():
    result = ParkingAisleFieldResultR17(
        deviation=np.zeros((2, 2), dtype=np.float32),
        aisle_labels=np.ones((2, 2), dtype=np.int16),
        gate_passed=True,
        failure_code="",
        diagnostics={"field_sha256": "fixed", "assigned_cell_count": 4},
    )
    assert result.summary() == {
        "gate_passed": True,
        "failure_code": "",
        "diagnostics": {"field_sha256": "fixed", "assigned_cell_count": 4},
    }


def test_r17_local_connection_is_tighter_not_a_budget_relaxation():
    config, _r15_config, _algorithm, parent_algorithm, *_rest = _load(CONFIG)
    local = config["se2_local_connection"]
    inherited = parent_algorithm["route_phase_policy"]
    assert local["maximum_station_skip"] <= inherited["maximum_station_skip"]
    assert (
        local["maximum_local_edge_length_m"]
        <= inherited["maximum_local_edge_length_m"]
    )
    assert config["calibration"]["early_stop_on_first_failure"] is True
    assert config["calibration"]["run_online_ros"] is False


def test_r17_method_is_route_attached_and_obstacle_separated():
    assert METHOD_ID == "route_attached_obstacle_separated_parking_aisle_field_r17_v1"
