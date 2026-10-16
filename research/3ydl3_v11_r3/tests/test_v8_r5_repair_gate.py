import pytest
from three_yd_v5.steering_rejection import repairable_steering_failure

def test_two_auditors_reporting_the_same_defect_can_trigger_repair(monkeypatch):
    monkeypatch.setenv('THREEYD_V8_R5_REPAIR','1')
    assert repairable_steering_failure(['RAW_STEERING_JUMP','STEERING_DISCONTINUITY'])
    assert repairable_steering_failure(['RAW_STEERING_JUMP'])
    assert repairable_steering_failure(['STEERING_DISCONTINUITY'])
    assert not repairable_steering_failure([])

@pytest.mark.parametrize('reason',['STATIC_FOOTPRINT_COLLISION','SWEPT_BODY_COLLISION_OR_CORRIDOR_EXIT','CURVATURE','ENDPOINT','ACK','ROAD_SEQUENCE_OR_OWNERSHIP','SOMETHING_NEW'])
def test_other_failures_are_never_hidden(monkeypatch,reason):
    monkeypatch.setenv('THREEYD_V8_R5_REPAIR','1')
    assert not repairable_steering_failure(['RAW_STEERING_JUMP',reason])

def test_control_reproduces_old_gate(monkeypatch):
    monkeypatch.setenv('THREEYD_V8_R5_REPAIR','0')
    assert repairable_steering_failure(['RAW_STEERING_JUMP'])
    assert not repairable_steering_failure(['RAW_STEERING_JUMP','STEERING_DISCONTINUITY'])
