import numpy as np
import pytest
from types import SimpleNamespace as S
from test_progress_connector import service
from three_yd_v5.progress_connector import propose
from three_yd_v5.v9_r6_options import heading_limit_degrees
from quality_v9_r6 import quality


def test_default_and_invalid_mode(monkeypatch):
    monkeypatch.delenv('THREEYD_V9_R6_MODE', raising=False)
    assert heading_limit_degrees() == 10
    monkeypatch.setenv('THREEYD_V9_R6_MODE', 'bad')
    with pytest.raises(ValueError): propose(service(), S(start=(.5, 0., 0.)))


def test_heading_filter_actual_pose_and_hard_audits(monkeypatch):
    s = service(); calls = []
    s.world.audit = lambda q,p,m: (calls.append(p.copy()) or {'valid':True,'failures':[]})
    q = S(start=(.5, .0265, np.deg2rad(11.58)))
    monkeypatch.setenv('THREEYD_V9_R6_MODE', 'control')
    assert propose(s, q)[1] == 'HEADING_NOT_ALONG_ROUTE' and not calls
    monkeypatch.setenv('THREEYD_V9_R6_MODE', 'H')
    value, reason = propose(s, q)
    assert value is not None and reason == 'LOCAL_CONNECTOR' and calls
    assert np.array_equal(value[1][0], q.start)
    assert value[2]['candidate_heading_limit_deg'] == 15
    assert len(value[2]['trials']) <= 4
    assert value[2]['trials'][-1]['steering_jump_deg'] <= 15+1e-6
    s.world.audit = lambda *a: {'valid':False,'failures':['COLLISION']}
    assert propose(s, q)[0] is None
    s.world.audit = lambda *a: {'valid':True,'failures':[]}
    s.base_traffic.audit = lambda p: {'valid':False,'failures':['PARKING_TRANSIT']}
    assert propose(s, q)[0] is None


def test_h_keeps_position_reverse_heading_and_ambiguity_limits(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R6_MODE', 'H'); s=service()
    for yaw in (15.01, 90, 180):
        assert propose(s, S(start=(.5,0,np.deg2rad(yaw))))[1] == 'HEADING_NOT_ALONG_ROUTE'
    assert propose(s,S(start=(.5,.151,0)))[1] == 'POSE_TOO_FAR_FROM_REMAINING_ROUTE'
    s.remaining = np.concatenate((s.remaining, s.remaining.copy()))
    assert propose(s,S(start=(.5,.01,0)))[1] == 'AMBIGUOUS_NEAR_ROUTE_VISIT'


def test_h_cannot_bypass_original_steering_jump(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R6_MODE','H');s=service()
    s.remaining[40:,2]=np.tile([.4,-.4],20)
    assert propose(s,S(start=(.5,.0265,np.deg2rad(11.58))))[0] is None


def test_half_ulp_only_allows_float32_boundary():
    before=float(np.float32(.7090698480606079)); after=float(np.float32(.659069836139679))
    r=quality(100,101,before,after)
    assert not r['quality_gate_direct'] and r['quality_gate']
    assert r['minimum_clearance_delta_m'] < -.05
    assert 0 < r['float32_input_precision_allowance_m'] <= 1e-6
    worse=float(np.nextafter(np.float32(after), np.float32(-np.inf)))
    assert not quality(100,101,before,worse)['quality_gate']
    assert not quality(100,102.001,before,after)['quality_gate']
    with pytest.raises(ValueError): quality(100,100,.70906984,after)
    with pytest.raises(ValueError): quality(100,100,float('nan'),after)


def test_precision_cap_is_not_a_large_clearance_exemption():
    r=quality(100,100,float(np.float32(100)),float(np.float32(99.94999)))
    assert r['float32_input_precision_allowance_m']==1e-6
    assert not r['quality_gate']
