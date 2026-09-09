import math
from pathlib import Path

import pytest
import yaml

from three_d_v1_nav2.contracts import (
    EXPECTED_QUERY_IDS, ContractError, controller_goal_yaw_tolerance,
    verify_frozen_sources, verify_vehicle_contract,
)


def test_frozen_r1_runtime_sources_equal_authoritative_snapshot():
    hashes = verify_frozen_sources()
    assert len(hashes) == 7
    assert all(len(value) == 64 for value in hashes.values())


def test_query_order_is_exactly_frozen_eight():
    assert len(EXPECTED_QUERY_IDS) == 8
    assert EXPECTED_QUERY_IDS[0] == "cmp2-01-lane-north"
    assert EXPECTED_QUERY_IDS[-1] == "cmp2-08-parking-internal"


def test_vehicle_contract_refuses_relaxation():
    verify_vehicle_contract({
        "allow_reverse": False, "allow_rotate_in_place": False,
        "minimum_turning_radius_m": 0.40, "maximum_curvature_1pm": 2.50,
    })
    with pytest.raises(ContractError):
        verify_vehicle_contract({
            "allow_reverse": True, "allow_rotate_in_place": False,
            "minimum_turning_radius_m": 0.40, "maximum_curvature_1pm": 2.50,
        })


def test_nav2_goal_checker_reserves_margin_for_final_mission_audit():
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "config" / "nav2_seq8.yaml").read_text()
    )
    checker = config["controller_server"]["ros__parameters"]["stopped_goal_checker"]
    assert checker["stateful"] is False
    assert checker["xy_goal_tolerance"] == pytest.approx(0.110)
    assert checker["xy_goal_tolerance"] + 0.015 <= 0.125
    assert checker["yaw_goal_tolerance"] == pytest.approx(0.017453292519943295)
    assert checker["yaw_goal_tolerance"] < 0.08726646259971647
    trajectory = config["controller_server"]["ros__parameters"]["FollowPath"]["trajectory"]
    assert trajectory["control_look_ahead_poses"] == 3
    assert trajectory["terminal_slowdown_distance"] == pytest.approx(7.8125)
    assert trajectory["max_lateral_acceleration"] == pytest.approx(0.20)
    weights = config["controller_server"]["ros__parameters"]["FollowPath"]["weights"]
    assert weights["weight_terminal_waypoint"] == pytest.approx(1.0e6)
    assert weights["weight_kinematics_nh"] >= 1.0e5
    assert weights["weight_kinematics_forward_drive"] >= 1.0e6
    assert weights["weight_kinematics_turning_radius"] >= 1.0e6


def test_per_query_yaw_budget_reserves_endpoint_quantization_and_audit_margin():
    q3 = controller_goal_yaw_tolerance(-1.2793395323170191, -1.308996979)
    q5 = controller_goal_yaw_tolerance(-1.76819188664478, -1.8325956503497525)
    assert q3 == pytest.approx(0.05260901557581862)
    assert q5 == pytest.approx(0.01786269889474418)
    assert q3 + abs(-1.308996979 + 1.2793395323170191) + 0.005 <= math.radians(5) + 1e-12
    assert q5 + abs(-1.8325956503497525 + 1.76819188664478) + 0.005 <= math.radians(5) + 1e-12
