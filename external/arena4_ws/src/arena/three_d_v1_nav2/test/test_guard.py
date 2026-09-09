import math
import random

from three_d_v1_nav2.guard_core import CommandGuard, GuardLimits


def test_forward_command_respects_all_hard_bounds():
    guard = CommandGuard()
    sample = guard.apply(2.0, 5.0, 0.1)
    assert 0.0 <= sample.out_v <= 2.5
    assert abs(sample.out_w) <= 1.5
    assert sample.curvature <= 2.5 + 1.0e-12
    assert abs(sample.output_linear_accel) <= 2.5 + 1.0e-12
    assert abs(sample.output_angular_accel) <= 2.0 + 1.0e-12
    assert "ANGULAR_ACCEL_SATURATION" in sample.events


def test_reverse_is_rejected_and_never_reaches_output():
    guard = CommandGuard()
    sample = guard.apply(-0.4, 0.5, 0.1)
    assert sample.out_v == 0.0
    assert sample.out_w == 0.0
    assert math.isclose(guard.raw_reverse_distance, 0.04)
    assert guard.output_reverse_distance == 0.0
    assert sample.events == frozenset({"REVERSE_HARD_STOP"})


def test_rotate_in_place_is_rejected():
    guard = CommandGuard()
    sample = guard.apply(0.0, 0.7, 0.05)
    assert sample.out_v == 0.0
    assert sample.out_w == 0.0
    assert "ROTATE_IN_PLACE_HARD_STOP" in sample.events


def test_rejected_command_decelerates_without_breaking_acceleration_contract():
    guard = CommandGuard(GuardLimits(max_linear_accel=0.5, max_angular_accel=1.0))
    for _ in range(20):
        moving = guard.apply(0.5, 0.4, 0.1)
    stopped = guard.apply(-1.0, 0.0, 0.1)
    assert stopped.out_v >= 0.0
    assert stopped.output_linear_accel >= -0.5 - 1.0e-12
    assert abs(stopped.output_angular_accel) <= 1.0 + 1.0e-12
    assert stopped.curvature <= 2.5 + 1.0e-12


def test_nonfinite_input_fails_closed():
    guard = CommandGuard()
    sample = guard.apply(math.nan, math.inf, 0.1)
    assert sample.out_v == sample.out_w == 0.0
    assert "INVALID_INPUT_HARD_STOP" in sample.events


def test_turning_stop_preserves_joint_slew_and_curvature_bounds():
    guard = CommandGuard(GuardLimits(max_linear_accel=0.8, max_angular_accel=0.2))
    for _ in range(100):
        guard.apply(0.4, 1.0, 0.1)
    sample = guard.apply(0.0, 0.0, 0.1)
    assert abs(sample.output_angular_accel) <= 0.2 + 1e-12
    assert sample.curvature <= 2.5 + 1e-12


def test_command_sequences_preserve_all_joint_bounds():
    rng = random.Random(218)
    for angular_accel in (0.1, 2.0):
        limits = GuardLimits(max_angular_accel=angular_accel)
        guard = CommandGuard(limits)
        for _ in range(10000):
            sample = guard.apply(rng.uniform(-0.3, 3.5), rng.uniform(-3, 3), rng.uniform(0.00001, 0.2))
            assert 0 <= sample.out_v <= limits.max_linear_speed
            assert abs(sample.out_w) <= limits.max_angular_speed
            assert abs(sample.out_w) <= limits.maximum_curvature * sample.out_v + 1e-10
            assert abs(sample.output_linear_accel) <= limits.max_linear_accel + 1e-8
            assert abs(sample.output_angular_accel) <= angular_accel + 1e-8
            if sample.out_v < limits.near_zero_linear_speed:
                assert abs(sample.out_w) <= limits.angular_epsilon + 1e-12


def test_user_speed_limit_is_reachable_and_stops_within_braking_budget():
    guard = CommandGuard()
    for _ in range(100):
        sample = guard.apply(3., 0., .05)
    assert math.isclose(sample.out_v, 2.5)
    distance, duration = 0., 0.
    while sample.out_v > 1e-9:
        previous = sample.out_v
        sample = guard.apply(0., 0., .05)
        assert abs(sample.output_linear_accel) <= 2.5 + 1e-9
        distance += (previous + sample.out_v) * .025
        duration += .05
    assert duration <= 1.01
    assert distance <= 1.251


def test_speed_and_observation_horizon_match_the_current_contract():
    from pathlib import Path
    import yaml
    config = yaml.safe_load((Path(__file__).parents[1] / 'config/nav2_seq8.yaml').read_text())
    controller = config['controller_server']['ros__parameters']['FollowPath']
    smoother = config['velocity_smoother']['ros__parameters']
    limits = GuardLimits()
    assert controller['robot']['v_max_x'] == limits.max_linear_speed == 2.5
    assert smoother['max_velocity'][0] == 2.4999
    assert config['controller_server']['ros__parameters']['odom_topic'] == smoother['odom_topic'] == '/model/jackal/odometry'
    assert controller['robot']['a_max_x'] == limits.max_linear_accel == 2.5
    assert smoother['max_accel'][0] == 2.0
    assert smoother['max_decel'][0] == -smoother['max_accel'][0]
    assert smoother['feedback'] == 'OPEN_LOOP'
    braking = limits.max_linear_speed ** 2 / (2 * limits.max_linear_accel)
    assert controller['trajectory']['max_global_plan_lookahead_dist'] > braking + .5
    assert controller['obstacles']['feasibility_check'] > braking + .5
    assert config['local_costmap']['local_costmap']['ros__parameters']['track_unknown_space'] is True
    assert config['global_costmap']['global_costmap']['ros__parameters']['track_unknown_space'] is True


def test_stop_waits_for_measured_angular_settling_without_pivot():
    guard = CommandGuard()
    guard.apply(0.02, 0.04, .1)
    for measured_w in (0.04, .01, .001, .0001):
        sample = guard.apply(0.0, 0.0, .05, measured_w)
        assert sample.out_v >= .005
        assert sample.out_w == 0.0
        assert 'ANGULAR_SETTLE_FORWARD_HOLD' in sample.events
        assert abs(sample.output_linear_accel) <= 2.5
        assert abs(sample.output_angular_accel) <= 2.0
    assert guard.apply(0.0, 0.0, .05, .00001).out_v == 0.0
    # Startup / teleport odometry cannot initiate translation from rest.
    assert guard.apply(0.0, 0.0, .05, 1.0).out_v == 0.0
