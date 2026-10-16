from types import SimpleNamespace as S
import time
import numpy as np
from three_yd_v5.v9_joints import evaluate,connect
from three_yd_v5.v9_local import heading_error
from arena_3d_v1.curvature_repair import violating_segments
from arena_3d_v1.semantic_world import SweptChecker,wrap


def test_quintic_matches_analytic_position_heading_curvature_and_rotation():
    a=np.array([0.,0.,.2]);b=np.array([2.,1.,.7])
    p,k=evaluate(a,b,.1,-.2,(2.,2.))
    assert np.array_equal(p[0],a) and np.array_equal(p[-1],b)
    assert np.allclose(k[[0,-1]],[.1,-.2],atol=1e-12)
    assert heading_error(p)<np.deg2rad(.1)
    theta=.9;rotation=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    aa=np.r_[a[:2]@rotation.T+[4.,5.],a[2]+theta]
    bb=np.r_[b[:2]@rotation.T+[4.,5.],b[2]+theta]
    q,kk=evaluate(aa,bb,.1,-.2,(2.,2.))
    assert np.allclose(q[:,:2],p[:,:2]@rotation.T+[4.,5.])
    assert np.allclose(wrap(q[:,2]-p[:,2]),theta)
    assert np.allclose(k,kk)


def test_real_corner_is_bounded_swept_and_blocked_candidate_rejected():
    world=S(vehicle=(.265,.225,.4),map=S(resolution=.05,origin=(0.,0.)))
    p=np.array([[.5,2.,0.],[1.,2.,0.],[2.,3.,np.pi/2],[2.,3.5,np.pi/2]])
    clear=np.zeros((120,120),np.uint8)
    checker=SweptChecker(clear,.05,(0.,0.),*world.vehicle[:2])
    q,detail=connect(world,p,1,2,checker,time.monotonic()+1)
    assert q is not None and detail['success']
    assert np.array_equal(q[0],p[1]) and np.array_equal(q[-1],p[2])
    assert len(violating_segments(np.vstack((p[:1],q,p[3:])),2.5)[0])==0
    assert heading_error(q)<np.deg2rad(5)
    blocked=SweptChecker(np.ones_like(clear),.05,(0.,0.),*world.vehicle[:2])
    q,detail=connect(world,p,1,2,blocked,time.monotonic()+1)
    assert q is None and detail['rejections']['current_swept_body_or_scope']>0
    q,detail=connect(world,p,1,2,checker,time.monotonic()-1)
    assert q is None and detail['failure']=='JOINT_BUDGET'
