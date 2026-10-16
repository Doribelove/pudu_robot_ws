import time
import numpy as np
import pytest
from scipy.interpolate import BSpline
from three_yd_v5.curve_continuation import suffix_spline, deform, continue_curve, bump, repair_obstacles
from three_yd_v5.curve_l3 import CONFIG, certify
from test_curve_l3 import world


def fixture():
    return BSpline([0,0,0,0,2,4,6,6,6,6],
        [[2,2],[3,2],[4,2.5],[6,2.5],[7,2],[8,2]],3)


@pytest.mark.parametrize('at',[0.,.13,2.,2.001,4.])
def test_exact_suffix_knot_insertion(at):
    old=fixture();model={'knots':old.t,'coefficients':old.c,'parameters':[at,6.]}
    new=suffix_spline(model);u=np.linspace(at,6,301)
    for derivative in (0,1,2):
        assert np.allclose(new(u,nu=derivative),old(u,nu=derivative),atol=1e-11,rtol=0)


def test_deformation_matches_actual_pose_and_preserves_tail_c2():
    old=fixture();pose=(2.,2.03,np.deg2rad(2))
    new,join=deform(old,pose,1.5)
    assert np.allclose(new(0),pose[:2],atol=1e-12,rtol=0)
    v=new(0,nu=1)
    assert abs(np.arctan2(v[1],v[0])-pose[2])<1e-12
    for derivative in (0,1,2):
        u=np.linspace(join,6,100)
        assert np.allclose(new(u,nu=derivative),old(u,nu=derivative),atol=1e-11,rtol=0)
    assert np.allclose(old.c,fixture().c)


def test_complete_curve_is_recertified_and_new_obstacle_rejected():
    w=world();old=fixture();mask=~w.hard
    path,cert=certify(old,w,mask,time.monotonic()+3)
    model={'knots':old.t.tolist(),'coefficients':old.c.tolist(),'parameters':cert['sample_parameters']}
    moved=(2.,2.03,np.deg2rad(2));goal=tuple(path[-1])
    path,new,log=continue_curve(w,mask,model,moved,goal,time.monotonic()+3,CONFIG)
    assert path is not None,log
    assert np.allclose(path[0],moved,atol=1e-9,rtol=0)
    assert np.allclose(path[-1],goal,atol=1e-9,rtol=0)
    # A newly blocked retained tail cannot be excused by its old certificate.
    w.hard[250:270,135:145]=True
    path,new,log=continue_curve(w,mask,model,moved,goal,time.monotonic()+3,CONFIG)
    assert path is None
    assert log['old_l3_calls']==0


def test_obstacle_bump_preserves_both_sides_through_second_derivative():
    old=fixture();new=bump(old,3.,.5,np.array([0.,.5]))
    assert np.allclose(new(3.)-old(3.),[0,.5],atol=1e-12,rtol=0)
    for derivative in (0,1,2):
        u=np.r_[np.linspace(0,2,31),np.linspace(4,6,31)]
        assert np.allclose(new(u,nu=derivative),old(u,nu=derivative),atol=1e-11,rtol=0)


def test_obstacle_repair_is_fully_certified_and_preserves_endpoints():
    w=world();old=fixture();mask=~w.hard
    path,cert=certify(old,w,mask,time.monotonic()+3)
    model={'knots':old.t.tolist(),'coefficients':old.c.tolist(),'parameters':cert['sample_parameters']}
    xy=old(3.);c=int(xy[0]/.05);r=w.hard.shape[0]-1-int(xy[1]/.05)
    dynamic=np.zeros_like(w.hard);dynamic[r-1:r+2,c-1:c+2]=True;w.hard|=dynamic
    new,record,log=repair_obstacles(w,mask,model,dynamic,time.monotonic()+5,CONFIG)
    assert new is not None,log
    assert np.allclose(new[[0,-1]],path[[0,-1]],atol=1e-9,rtol=0)
    assert log['certificate']['max_curvature_bound_1pm']<=2.50000001
    # Replaying the saved record checks the same current obstacles again.
    checked,_=certify(BSpline(record['knots'],record['coefficients'],3),w,mask,
                      time.monotonic()+3,parameters=record['parameters'])
    assert np.allclose(checked,new,atol=1e-9,rtol=0)


def test_failed_displacement_retains_task_candidate_without_exposing_it(monkeypatch):
    from test_task_progress import fake_service
    from three_yd_v5 import curve_backend
    from types import SimpleNamespace as S
    s,q,path=fake_service()
    s.retained_record=s.record;s.retained_path=path.copy()
    s.record=None;s.remaining=None;s.controller.trajectory=None;s.controller.trajectory_version=None
    # This failed query is deliberately identical to the next actual request;
    # it must not enable the identical-current-query shortcut on the old path.
    moved=S(query_id=q.query_id,start=(.75,.53,.03),goal=q.goal);s.query=moved
    called=[]
    monkeypatch.setattr(curve_backend,'identity',lambda:('test','independent','test'))
    def displaced(service,query):
        called.append(query)
        assert service.task_query is q and service.current_trajectory() is None
        assert np.array_equal(service.remaining,path)
        service.record=None;service.remaining=None
        return {'success':False,'failure_code':'TEST_REJECTED'}
    monkeypatch.setattr(curve_backend,'displaced_continuation',displaced)
    result=s._plan(moved)
    assert result['failure_code']=='TEST_REJECTED' and called==[moved]
    assert s.current_trajectory() is None and s.task_query is q
