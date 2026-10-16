import time,math
from types import SimpleNamespace as S
import numpy as np
import pytest
from three_yd_v5.failure_terminal import search,terminal_guide
from three_yd_v5.v10_storage import collision_checker

def world():
    return S(hard=np.zeros((600,800),bool),map=S(resolution=.05,origin=(0.,0.,0.)),vehicle=(.265,.225,.4))

def test_native_seed_preserves_poses_and_actual_footprint():
    w=world();a=np.array([2.,5.,0.]);b=np.array([8.,6.,1.2])
    p,log=search(a,b,w,w.hard,1.,.8)
    assert p is not None and log['status']==0
    np.testing.assert_allclose(p[0],a,atol=1e-7,rtol=0)
    np.testing.assert_allclose(p[-1],b,atol=1e-7,rtol=0)
    assert collision_checker(w.hard,w).check(p)
    ds=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
    da=np.arctan2(np.sin(np.diff(p[:,2])),np.cos(np.diff(p[:,2])))
    assert np.max(abs(da)/np.maximum(ds,1e-12))<1/.8+.001

def test_blocked_endpoint_is_never_a_success():
    w=world();hard=w.hard.copy();hard[475:485,35:45]=True
    p,log=search((2.,6.,0.),(8.,6.,0.),w,hard,1.,.8)
    assert p is None and log['status']==3

def test_matching_heading_does_not_run_extra_search(monkeypatch):
    w=world()
    monkeypatch.setattr('three_yd_v5.failure_terminal.search',lambda *a,**k:(_ for _ in ()).throw(AssertionError('unexpected search')))
    p,log=terminal_guide(w,~w.hard,np.array([[2.,6.],[20.,6.]]),(2.,6.,0.),(20.,6.,0.),time.monotonic()+1)
    assert p is None and not log['trials']

def test_expired_budget_keeps_original_query_without_search(monkeypatch):
    w=world();guide=np.array([[2.,6.],[20.,6.]])
    monkeypatch.setattr('three_yd_v5.failure_terminal.search',lambda *a,**k:(_ for _ in ()).throw(AssertionError('unexpected search')))
    p,log=terminal_guide(w,~w.hard,guide,(2.,6.,0.),(20.,6.,math.pi),time.monotonic()-1)
    assert p is None and not log['trials']
    np.testing.assert_array_equal(guide,[[2.,6.],[20.,6.]])

def test_backward_initialization_preserves_forward_terminal_heading():
    w=world();goal=(30.,10.,math.pi)
    p,log=terminal_guide(w,~w.hard,np.array([[2.,10.],[30.,10.]]),(2.,10.,0.),goal,time.monotonic()+2)
    assert p is not None and log['selected'] and log['initialization_only']
    np.testing.assert_allclose(p[0],[2.,10.],atol=1e-7,rtol=0)
    np.testing.assert_allclose(p[-1],goal[:2],atol=1e-7,rtol=0)
    direction=p[-1]-p[-2]
    assert direction[0]<0 and abs(math.atan2(direction[1],direction[0]))>math.radians(175)

@pytest.mark.parametrize('corruption',['position','heading'])
def test_bad_native_binding_never_becomes_a_guide(monkeypatch,corruption):
    def malformed(a,b,*args,**kwargs):
        p=np.array([a,b],float)
        p[0,0 if corruption=='position' else 2]+=.1
        return p,{}
    monkeypatch.setattr('three_yd_v5.failure_terminal.search',malformed)
    with pytest.raises(ValueError,match='TERMINAL_SEED_'):
        terminal_guide(world(),np.ones((600,800),bool),[[2.,10.],[30.,10.]],
                       (2.,10.,0.),(30.,10.,math.pi),time.monotonic()+1.)

def test_failed_search_preserves_guide_and_legal_scope(monkeypatch):
    w=world();scope=~w.hard;scope[:,300:310]=False;before=scope.copy()
    guide=np.array([[2.,10.],[30.,10.]])
    calls=[]
    def unavailable(a,b,world,hard,budget,radius,**kwargs):
        assert np.array_equal(hard,w.hard|~scope)
        assert 0<budget<=.2 and radius>=w.vehicle[2]
        calls.append((a.copy(),b.copy()));return None,{'status':1}
    monkeypatch.setattr('three_yd_v5.failure_terminal.search',unavailable)
    p,log=terminal_guide(w,scope,guide,(2.,10.,0.),(30.,10.,math.pi),time.monotonic()+1.)
    assert p is None and not log['selected'] and calls
    np.testing.assert_array_equal(scope,before)
    np.testing.assert_array_equal(guide,[[2.,10.],[30.,10.]])

def test_zero_budget_native_call_cannot_report_success():
    w=world();p,detail=search((2.,5.,0.),(8.,6.,1.2),w,w.hard,0.,.8)
    assert p is None and detail['native_return']<0
