import json,time
from types import SimpleNamespace as S
import numpy as np
import pytest
from scipy.interpolate import BSpline
from three_yd_v5.curve_l3 import CONFIG,CurveFailure,optimize,certify,rectangles,clean_guide
from three_yd_v5.curve_backend import identity,validate_record

def world():
    hard=np.zeros((300,300),bool);hard[[0,-1],:]=True;hard[:,[0,-1]]=True
    return S(hard=hard,map=S(resolution=.05,origin=(0.,0.,0.)),vehicle=(.265,.225,.4))

@pytest.mark.parametrize('guide,start,goal',[
 ([[1,2],[12,2]],(1,2,0),(12,2,0)),
 ([[2,1],[2,6],[7,6]],(2,1,np.pi/2),(7,6,0)),
 ([[2,2],[2,7],[4,7],[4,2]],(2,2,np.pi/2),(4,2,-np.pi/2)),
 ([[1,2],[3,2],[4,3],[4,6]],(1,2,0),(4,6,np.pi/2)),
])
def test_geometric_cases(guide,start,goal):
    w=world();p,model,log=optimize(w,~w.hard,guide,start,goal,time.monotonic()+8)
    assert p is not None,log
    assert log['old_l3_calls']==0
    assert np.allclose(p[0],start,atol=2e-5)
    assert np.allclose(p[-1],goal,atol=2e-5)
    assert log['certificate']['max_curvature_bound_1pm']<=2.50000001
    # The U-shaped route cannot jump between close branches.
    if len(guide)==4 and guide[-1]==[4,2]:assert p[:,1].max()>6.5

def test_deadline_and_failure_are_explicit():
    w=world();p,model,log=optimize(w,~w.hard,[[1,2],[12,2]],(1,2,0),(12,2,0),time.monotonic()-1)
    assert p is None and log['failure']=='CURVE_BUDGET'

def test_center_clear_does_not_mean_body_clear():
    w=world();w.hard[300-1-40,70]=True
    # Straight center y=2.2 clears the cell, but the 0.225m half-width does not.
    spline=BSpline([0,0,0,0,1,1,1,1],[[2,2.2],[3,2.2],[4,2.2],[5,2.2]],3)
    with pytest.raises(CurveFailure,match='BODY'):certify(spline,w,~w.hard,time.monotonic()+2)

def test_between_sample_curvature_is_not_ignored():
    w=world();spline=BSpline([0,0,0,0,1,1,1,1],[[2,2],[3,2],[2,2.01],[3,2.01]],3)
    with pytest.raises(CurveFailure,match='CURVATURE'):certify(spline,w,~w.hard,time.monotonic()+2)

def test_rectangle_uses_image_row_direction():
    w=world();w.hard[260:,:]=True
    # World y=1 lies in the blocked bottom image rows, not the top.
    with pytest.raises(CurveFailure,match='OUTSIDE'):rectangles(w.hard,.05,(0,0),np.array([[5,1]]),time.monotonic()+2)

def test_backend_identity_isolated_before_reuse(monkeypatch):
    monkeypatch.setenv('THREEYD_L3_BACKEND','control');old=list(identity())
    monkeypatch.setenv('THREEYD_L3_BACKEND','independent')
    failures,_=validate_record(None,None,{'l3_backend_identity':old},None)
    assert failures==['CURVE_BACKEND_BINDING_CHANGED']

def test_independent_rejects_record_without_curve(monkeypatch):
    monkeypatch.setenv('THREEYD_L3_BACKEND','independent')
    failures,_=validate_record(None,None,{},None)
    assert failures==['CURVE_MODEL_MISSING']

def test_hard_deadline_cannot_be_swallowed_by_candidate_recovery():
    from three_yd_v5.curve_budget import request_guard,L3RequestBudgetExceeded
    with pytest.raises(L3RequestBudgetExceeded):
        with request_guard(.01):time.sleep(.025)


def test_ordered_corridor_not_only_global_free_space():
    w=world();sp=BSpline([0,0,0,0,1,1,1,1],[[2,2],[3,2],[4,2],[5,2]],3)
    corridor={'stations':[0,1],'bounds':[[0,1,0,1],[0,1,0,1]],'axes':[[1,0,0,1],[1,0,0,1]]}
    with pytest.raises(CurveFailure):certify(sp,w,~w.hard,time.monotonic()+2,corridor=corridor)


def test_saved_parameter_suffix_is_the_same_certified_curve():
    w=world();p,model,log=optimize(w,~w.hard,[[2,1],[2,6],[7,6]],
        (2,1,np.pi/2),(7,6,0),time.monotonic()+8)
    assert p is not None,log
    sp=BSpline(model['knots'],model['coefficients'],3)
    for fraction in (.2,.5,.8):
        index=int(len(p)*fraction)
        suffix,detail=certify(sp,w,~w.hard,time.monotonic()+2,
            corridor=model['corridor'],parameters=model['parameters'][index:])
        assert suffix.shape==p[index:].shape
        assert np.allclose(suffix,p[index:],atol=1e-12,rtol=0)


def test_saved_parameter_binding_rejects_unordered_domain():
    w=world();sp=BSpline([0,0,0,0,1,1,1,1],[[2,2],[3,2],[4,2],[5,2]],3)
    with pytest.raises(CurveFailure,match='PARAMETER_BINDING'):
        certify(sp,w,~w.hard,time.monotonic()+2,parameters=[0,.5,.4,1])

def test_certificate_interval_never_crosses_polynomial_piece():
    w=world();sp=BSpline([0,0,0,0,.5,1,1,1,1],[[2,2],[3,2],[4,2],[5,2],[6,2]],3)
    with pytest.raises(CurveFailure,match='CROSSES_POLYNOMIAL'):
        certify(sp,w,~w.hard,time.monotonic()+2,parameters=[0,1])

def test_live_obstacle_retains_only_untrusted_candidate_but_vehicle_invalidates(monkeypatch):
    from three_yd_v5.curve_backend import request_start,request_finish
    monkeypatch.setenv('THREEYD_L3_BACKEND','independent')
    w=world();w.key='test';w.labels=np.zeros_like(w.hard,dtype=np.int16)
    w.map.occupancy=w.hard.copy();w.semantic=S(semantic_map_hash='test');w.rules={}
    session=S(bound_vehicle=w.vehicle)
    service=S(world=w,controller=None,dynamic=S(static_hard=w.hard.copy()),session=session,
        record={'curve_l3':{}},remaining=np.ones((2,3)),retained_record={'candidate':True},retained_path=np.ones((2,3)))
    request_start(service);request_finish(service,{'success':True})
    w.hard=w.hard.copy();w.hard[150,150]=True
    request_start(service)
    assert service.record is None and service.remaining is None
    assert service.retained_record=={'candidate':True}
    w.vehicle=(.3,.25,.4)
    request_start(service)
    assert service.retained_record is None and service.retained_path is None
