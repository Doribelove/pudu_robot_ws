from types import SimpleNamespace
import time

from three_d_v1_nav2.cmd_vel_guard_node import CommandGuardNode, stop_target_velocity
from three_d_v1_nav2.mission_node import SequentialMission


def test_enabling_commands_starts_first_message_deadline():
    node = SimpleNamespace(force_stop=True, last_input_ns=0)
    before = time.monotonic_ns()
    CommandGuardNode._force_stop(node, SimpleNamespace(data=False))
    assert node.last_input_ns >= before


def test_old_zero_odometry_cannot_pass_stopped_gate():
    node = SimpleNamespace(odom_mono_ns=time.monotonic_ns() - 1_000_000_000,
                           stopped_since_ns=time.monotonic_ns() - 10_000_000_000,
                           stop_hold_s=1.)
    assert not SequentialMission._is_stopped(node)


def test_valid_recent_stopped_observation_can_pass():
    node = SimpleNamespace(odom_mono_ns=time.monotonic_ns(),
                           stopped_since_ns=time.monotonic_ns() - 2_000_000_000,
                           stop_hold_s=1.)
    assert SequentialMission._is_stopped(node)


def test_dynamic_hold_parameter_stays_inside_stopped_gate():
    hold = 1.0e-3
    assert 0.0 < hold <= 2.0e-3
    assert stop_target_velocity(False, True, hold) == hold
    assert stop_target_velocity(True, True, hold) == 0.0
    assert stop_target_velocity(False, False, hold) == 0.0
