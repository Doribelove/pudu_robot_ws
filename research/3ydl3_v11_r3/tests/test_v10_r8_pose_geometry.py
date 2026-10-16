import math,time
from types import SimpleNamespace as S
import numpy as np
from scipy.interpolate import BSpline
from three_yd_v5.failure_pose_curve import insert_knot
from three_yd_v5.failure_pose_cells import PoseCells
from three_yd_v5.failure_pose_chain import PoseSearch
from three_yd_v5.v10_storage import collision_checker


def world():
    return S(hard=np.zeros((160,200),bool),map=S(resolution=.05,origin=(0.,0.,0.)),vehicle=(.265,.225,.4))


def test_exact_refinement_preserves_position_derivatives_and_endpoints():
    rng=np.random.default_rng(684)
    knots=np.r_[[0.]*3,np.linspace(0.,3.,11),[3.]*3]
    cp=rng.normal(size=(len(knots)-4,2));original=BSpline(knots,cp,3)
    for x in [.003,.14,.9,1.711,2.999]:knots,cp=insert_knot(knots,cp,x)
    refined=BSpline(knots,cp,3);u=np.linspace(0.,3.,1711)
    for order in range(3):np.testing.assert_allclose(refined(u,order),original(u,order),atol=1e-9,rtol=1e-9)
    old=len(cp);knots,cp=insert_knot(knots,cp,1.711)
    assert len(cp)==old


def test_rectangle_cell_gap_agrees_with_original_body_checker():
    w=world();w.hard[70:78,90:98]=True
    cells=PoseCells(w,w.hard,np.array([[4.,3.],[6.,5.]]));check=collision_checker(w.hard,w)
    rng=np.random.default_rng(73)
    poses=np.c_[rng.uniform([4.,3.5],[5.6,4.8],(500,2)),rng.uniform(-math.pi,math.pi,500)]
    for pose in poses:
        _,gap,_,_=cells.gaps(pose[None,:2],np.array([[math.cos(pose[2]),math.sin(pose[2])]]))
        # Compare away from exact contact, where native float tolerances differ.
        if len(gap) and abs(gap.min())<1e-7:continue
        assert check.check(pose[None,:])==bool(gap.min(initial=1.)>0.)


def test_separating_axis_position_and_yaw_gradients():
    w=world();w.hard[75,95]=True;cells=PoseCells(w,w.hard,np.array([[4.,3.],[6.,5.]]))
    rng=np.random.default_rng(91)
    for _ in range(100):
        p=np.array([[4.775,4.225]])+rng.uniform(-.2,.2,(1,2));theta=rng.uniform(.2,1.2)
        v=np.array([[math.cos(theta),math.sin(theta)]])
        idx,gap,gradient,yaw=cells.gaps(p,v)
        assert len(gap)==1
        for axis in range(3):
            h=1e-7;shift=np.zeros_like(p)
            if axis<2:shift[0,axis]=h
            a=theta+(h if axis==2 else 0);b=theta-(h if axis==2 else 0)
            plus=cells.gaps(p+shift,np.array([[math.cos(a),math.sin(a)]]))[1]
            minus=cells.gaps(p-shift,np.array([[math.cos(b),math.sin(b)]]))[1]
            np.testing.assert_allclose((plus-minus)/(2*h),gradient[:,axis] if axis<2 else yaw,atol=2e-7,rtol=2e-7)


def test_directed_initializer_preserves_actual_forward_heading():
    w=world();labels=np.ones_like(w.hard,dtype=np.int16);engine=PoseSearch(w,~w.hard,labels,[(1,[1.,0.])])
    a=np.array([2.,3.,0.]);b=np.array([7.,3.,0.])
    for reverse in (False,True):
        path,log=engine.search(a,b,.8,time.monotonic()+1,.2,reverse)
        assert path is not None
        np.testing.assert_allclose(path[[0,-1]],np.array([a,b]),atol=1e-7,rtol=0)
        assert np.cos(path[:,2]).min()>=-.085
    path,log=engine.search([7.,3.,math.pi],[2.,3.,math.pi],.8,time.monotonic()+1,.2)
    assert path is None and log['status']==3


def test_pose_search_expired_budget_and_ambiguous_road():
    w=world();labels=np.ones_like(w.hard,dtype=np.int16);engine=PoseSearch(w,~w.hard,labels,[(1,[1.,0.])])
    path,log=engine.search([2.,3.,0.],[7.,3.,0.],.8,time.monotonic()-1,.2)
    assert path is None and log['status']=='deadline'
    import pytest
    with pytest.raises(ValueError,match='LABEL_SHAPE'):
        PoseSearch(w,~w.hard,labels[:-1],[(1,[1.,0.])])
    with pytest.raises(ValueError,match='AMBIGUOUS_ROAD_DIRECTION'):
        PoseSearch(w,~w.hard,labels,[(1,[1.,0.]),(1,[-1.,0.])])
