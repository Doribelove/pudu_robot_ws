import json,time
from types import SimpleNamespace as S
import numpy as np
import pytest
from three_yd_v5.tail_guide import rounded_guide
from three_yd_v5.curve_l3 import optimize,CONFIG,CurveFailure


def world():
    return S(hard=np.zeros((160,160),bool),vehicle=(.265,.225,.4),
             map=S(resolution=.05,origin=(0.,0.)))


@pytest.mark.parametrize('sign',[-1,1])
def test_rounded_seed_preserves_endpoints_and_is_initialization_only(sign):
    w=world();p=np.array([[1.,4.],[4.,4.],[4.,4.+sign*2.]])
    s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
    old=p.copy();p.flags.writeable=False
    seed,arc,log=rounded_guide(p,s,w,w.hard,time.monotonic()+3.)
    assert np.array_equal(p,old)
    assert np.array_equal(seed[[0,-1]],p[[0,-1]]) and np.all(np.diff(arc)>0)
    assert len(log['accepted_corners'])==1 and log['initialization_only']
    assert log['accepted_corners'][0]['radius_m']>=w.vehicle[2]
    assert len(seed)>len(p)


def test_blocked_arc_falls_back_to_original_polyline():
    w=world();p=np.array([[1.,1.],[4.,1.],[4.,4.]])
    seed,arc,log=rounded_guide(p,np.array([0.,3.,6.]),w,np.ones_like(w.hard),time.monotonic()+3.)
    assert np.array_equal(seed,p) and not log['accepted_corners']


def test_rounded_spline_still_requires_original_full_certificate():
    w=world();p=np.array([[1.,1.],[4.,1.],[4.,4.]])
    cfg=dict(CONFIG,r4_rounded_guide=True,adaptive_constraint_stations=True,tracking_weight=100.)
    path,curve,log=optimize(w,np.ones_like(w.hard),p,(1.,1.,0.),(4.,4.,np.pi/2),time.monotonic()+5.,cfg)
    assert path is not None and log['success']
    assert log['certificate']['max_curvature_bound_1pm']<=1/w.vehicle[2]+1e-8
    assert np.allclose(path[0],(1.,1.,0.),atol=1e-10)
    assert np.allclose(path[-1],(4.,4.,np.pi/2),atol=1e-10)


def test_initialization_obeys_existing_deadline():
    w=world();p=np.array([[1.,1.],[4.,1.],[4.,4.]])
    with pytest.raises(CurveFailure,match='CURVE_BUDGET'):
        rounded_guide(p,np.array([0.,3.,6.]),w,w.hard,time.monotonic()-1.)


def test_r4_modes_keep_physical_limits_and_budgets(monkeypatch):
    from three_yd_v5.curve_backend import settings
    monkeypatch.setenv('THREEYD_V10_R4_MODE','control');base=settings()[1]
    monkeypatch.setenv('THREEYD_V10_R4_MODE','PGS');cfg=settings()[1]
    allowed={'tracking_weight','adaptive_constraint_stations','r4_rounded_guide','r4_mode'}
    assert all(cfg[k]==v for k,v in base.items() if k not in allowed)
    assert cfg['total_request_budget_s']==30 and cfg['new_l3_budget_s']==15
