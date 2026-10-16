import math

import numpy as np

from arena_evaluation.semantic_constraint_core import dubins_edge
from arena_evaluation.semantic_transition_negative_probe import naturalness


def test_straight_connection_is_natural():
    edge=dubins_edge((0.,0.,0.),(10.,0.,0.))
    path=np.vstack((edge.start,edge.samples))
    report=naturalness(path,[edge],edge.start,edge.goal)
    assert report["naturalness_passed"]
    assert report["backward_route_progress_m"]==0.


def test_large_progress_regression_is_rejected():
    edge=dubins_edge((0.,0.,0.),(10.,0.,0.))
    path=np.array([[0.,0.,0.],[3.,0.,0.],[1.,1.,0.],[10.,0.,0.]])
    report=naturalness(path,[edge],edge.start,edge.goal)
    assert not report["naturalness_passed"]
    assert report["backward_route_progress_m"]==2.
