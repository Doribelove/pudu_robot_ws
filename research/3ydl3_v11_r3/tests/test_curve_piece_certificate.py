import time
from types import SimpleNamespace as S
import numpy as np
from scipy.interpolate import BSpline
from arena_3d_v1.semantic_world import SweptChecker
from three_yd_v5.curve_l3 import certify,CONFIG

def test_distant_bend_does_not_inflate_a_straight_piece():
    hard=np.zeros((240,240),bool)
    hard[240-1-39,68]=True # obstacle ends at y=2m
    w=S(hard=hard,map=S(resolution=.05,origin=(0.,0.,0.)),vehicle=(.265,.225,.4))
    # The original native raster sweep needs this extra half-cell margin.
    y=2.25001
    sp=BSpline([0]*4+[1]*3+[2]*4,
        [[2,y],[3,y],[4,y],[5,y],[6,y],[7,2.5],[8,3]],3)
    p,d=certify(sp,w,~hard,time.monotonic()+3,CONFIG)
    assert d['body_sweep_groups']==2
    assert d['body_enlargement_m']<=.0002+1e-9
    # The global maximum comes from the remote curved piece; applying it to
    # the straight piece gives a false rejection in the original sweep.
    checker=SweptChecker(hard,.05,(0,0),.265+d['body_enlargement_m'],.225+d['body_enlargement_m'])
    assert not checker.check(p)
    # The actual original-size sweep still passes; the per-piece proof adds
    # its own nonnegative error bound on every interval.
    checker.half_length=.265;checker.half_width=.225
    assert checker.check(p)
