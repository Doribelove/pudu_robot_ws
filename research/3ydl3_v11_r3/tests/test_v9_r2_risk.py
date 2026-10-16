from types import SimpleNamespace as S
import time
import numpy as np
from arena_3d_v1.semantic_world import SweptChecker,densify
from three_yd_v4.local_l3 import risk_segments as original
from three_yd_v5.v9_risk import risk_segments
from three_yd_v5.v9_guided_joint import guided_connect


def test_indexed_collision_location_matches_original_at_turns_and_cell_edges():
    world=S(vehicle=(.265,.225,.4),map=S(resolution=.05,origin=(0.,0.)))
    rng=np.random.default_rng(38)
    for blocked in (False,True):
        hard=np.zeros((180,180),bool)
        if blocked:hard[35:80,70:105]=True
        world.hard=hard;mask=np.ones(hard.shape,bool)
        checker=SweptChecker(hard,.05,(0.,0.),*world.vehicle[:2])
        for _ in range(6):
            xy=rng.uniform(1.,7.,(7,2));d=np.diff(xy,axis=0)
            p=np.c_[xy,np.r_[np.arctan2(d[:,1],d[:,0]),.13]]
            a,al=original(world,p,mask);b,bl=risk_segments(world,p,mask,checker)
            assert np.array_equal(a,b) and np.array_equal(al,bl)


def test_compound_corner_stays_inside_narrow_l_road_and_obstacle_rejects():
    res=.05;r,c=np.indices((180,180));x=(c+.5)*res;y=(179-r+.5)*res
    mask=((x>=.5)&(x<=4.6)&(y>=1.4)&(y<=2.6))|((x>=3.4)&(x<=4.6)&(y>=2)&(y<=5.5))
    world=S(vehicle=(.265,.225,.4));checker=SweptChecker(~mask,res,(0.,0.),*world.vehicle[:2])
    p=np.array([[.9,2.,0.],[1.,2.,0.],[4.,2.,np.pi/2],[4.,5.,np.pi/2],[4.,5.1,np.pi/2]])
    q,detail=guided_connect(world,p,1,3,checker,time.monotonic()+1)
    assert q is not None and detail['method']=='compound_G2_corner'
    assert np.array_equal(q[0],p[1]) and np.array_equal(q[-1],p[3])
    assert checker.check(densify(q)) and max(abs(np.array(detail['straight_bend_curvature'])))<1e-12
    blocked=SweptChecker(np.ones_like(mask),res,(0.,0.),*world.vehicle[:2])
    q,detail=guided_connect(world,p,1,3,blocked,time.monotonic()+.08)
    assert q is None and not detail['success']


def test_legal_straight_collision_intervals_have_identical_pose_bytes(monkeypatch):
    import three_yd_v4.local_l3 as parent
    hard=np.zeros((180,180),bool);hard[60:90,60:75]=True
    world=S(hard=hard,vehicle=(.265,.225,.4),map=S(resolution=.05,origin=(0.,0.)))
    mask=np.ones(hard.shape,bool);old_calls=[];new_calls=[]
    class Recorder:
        def __init__(self,calls,*args):self.calls=calls;self.actual=SweptChecker(*args)
        def check(self,p):self.calls.append(p.copy());return self.actual.check(p)
    monkeypatch.setattr(parent,'SweptChecker',lambda *args:Recorder(old_calls,*args))
    for y in (4.51,5.025,5.501,6.025):
        p=np.c_[np.linspace(1.025,7.025,121),np.full(121,y),np.zeros(121)]
        old_calls.clear();new_calls.clear()
        a,_=original(world,p,mask)
        b,_=risk_segments(world,p,mask,Recorder(new_calls,hard,.05,(0.,0.),*world.vehicle[:2]))
        assert 0<len(a)<len(p)-1 and np.array_equal(a,b)
        assert len(old_calls)==len(new_calls)
        assert all(x.dtype==z.dtype and x.tobytes()==z.tobytes() for x,z in zip(old_calls,new_calls))
