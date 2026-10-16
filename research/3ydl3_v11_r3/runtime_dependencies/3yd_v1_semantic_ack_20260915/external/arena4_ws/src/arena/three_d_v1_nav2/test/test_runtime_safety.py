from three_d_v1_nav2.runtime_safety import MotionEvidence, telemetry_failure


def test_reverse_is_measured_and_rejected():
    evidence = MotionEvidence()
    evidence.observe(1000000000, -.1, 0)
    evidence.observe(1100000000, -.1, 0)
    assert evidence.reverse_distance > 0
    assert evidence.failure_code == "MEASURED_SPEED_OR_REVERSE_VIOLATION"


def test_bad_motion_latches_failure():
    evidence = MotionEvidence()
    evidence.observe(1000000000, .1, .3)
    evidence.observe(1100000000, .1, 0)
    assert evidence.failure_code == "MEASURED_CURVATURE_VIOLATION"


def test_successful_teb_flag_cannot_hide_collision_stop():
    assert telemetry_failure(dict(planner_success=True, collision_stop=True), controller=True)


def test_watchdog_is_a_failure_but_bounded_saturation_is_allowed():
    assert telemetry_failure(dict(events="COMMAND_WATCHDOG_STOP"))
    assert not telemetry_failure(dict(events="CURVATURE_SATURATION", output_linear_accel=2.5,
                                      output_angular_accel=-2.))
    assert telemetry_failure(dict(events="", output_linear_accel=2.51, output_angular_accel=0.))
