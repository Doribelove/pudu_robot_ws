"""A vehicle can need a terminal loop closer than the old 1 m anchor grid."""
import time
from types import SimpleNamespace as S
import numpy as np
from arena_3d_v1.semantic_world import SweptChecker
from three_yd_v5.curve_l3 import endpoint_seeds

def test_near_anchor_can_initialize_narrow_corridor_u_turn():
    hard=np.ones((200,300),bool);hard[68:100,10:290]=False
    w=S(hard=hard,map=S(resolution=.05,origin=(0.,0.,0.)),vehicle=(.265,.225,.4))
    guide=np.array([[2.,5.8],[11.,5.8]]);arc=np.array([0.,9.])
    start=(2.,5.8,0.);goal=(11.,5.8,2.4394)
    p,s,log=endpoint_seeds(guide,arc,start,goal,w,~hard,time.monotonic()+5)
    assert log[-1]['selected'] and log[-1]['anchor_distance_m']<1.
    assert np.array_equal(p[0],start[:2]) and np.allclose(p[-1],goal[:2],atol=1e-12)
    # Reconstruct the actual analytic seed, whose orientation differs from
    # a one-sided finite difference of the XY-only initialization guide.
    from arena_evaluation.semantic_constraint_core import dubins_choices,dubins_edge_from_parameters
    from arena_3d_v1.semantic_world import densify
    anchor=(goal[0]-log[-1]['anchor_distance_m'],goal[1],0.)
    word,pars=next((word,pars) for word,pars in dubins_choices(anchor,goal,log[-1]['radius_m']) if word==log[-1]['word'])
    edge=dubins_edge_from_parameters(anchor,goal,log[-1]['radius_m'],word,pars)
    poses=densify(np.vstack((anchor,edge.samples)))
    assert SweptChecker(hard,.05,(0.,0.,0.),.265,.225).check(poses)
    assert np.all(np.diff(s)>0)

def test_straight_endpoints_keep_original_guide_exactly():
    w=S(hard=np.zeros((200,300),bool),map=S(resolution=.05,origin=(0.,0.,0.)),vehicle=(.265,.225,.4))
    p=np.array([[2.,5.8],[11.,5.8]]);s=np.array([0.,9.])
    got,arc,log=endpoint_seeds(p,s,(2.,5.8,0.),(11.,5.8,0.),w,~w.hard,time.monotonic()+2)
    assert np.array_equal(got,p) and np.array_equal(arc,s) and not log
