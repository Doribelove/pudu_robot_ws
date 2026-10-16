"""Output spacing may change; continuous geometry bounds may not."""
import time
from types import SimpleNamespace as S
import numpy as np
import pytest
from scipy.interpolate import BSpline
from three_yd_v5.curve_l3 import CONFIG,certify,CurveFailure

def world():
    hard=np.zeros((240,240),bool)
    hard[[0,-1],:]=True;hard[:,[0,-1]]=True
    return S(hard=hard,map=S(resolution=.05,origin=(0.,0.,0.)),vehicle=(.265,.225,.4))

def test_straight_curve_emits_fewer_exact_poses_with_same_bounds():
    w=world();sp=BSpline([0,0,0,0,1,1,1,1],[[2,2],[4,2],[6,2],[8,2]],3)
    fine,a=certify(sp,w,~w.hard,time.monotonic()+3,{**CONFIG,'max_output_step_m':.025})
    coarse,b=certify(sp,w,~w.hard,time.monotonic()+3,{**CONFIG,'max_output_step_m':.1})
    assert len(coarse)<len(fine)/3
    assert np.array_equal(coarse[[0,-1]],fine[[0,-1]])
    assert np.max(np.linalg.norm(np.diff(coarse[:,:2],axis=0),axis=1))<=.100001
    assert max(a['max_curvature_bound_1pm'],b['max_curvature_bound_1pm'])<1e-10
    assert b['body_enlargement_m']<=CONFIG['max_curve_chord_error_m']+1e-9

def test_curved_intervals_refine_to_original_error_and_curvature_limits():
    w=world();sp=BSpline([0,0,0,0,1,1,1,1],[[2,2],[3,2],[4,3],[4,4]],3)
    p,detail=certify(sp,w,~w.hard,time.monotonic()+3,{**CONFIG,'max_output_step_m':.1})
    assert detail['body_enlargement_m']<=.0002+1e-9
    assert detail['max_curvature_bound_1pm']<=2.50000001
    u=np.array(detail['sample_parameters']);v=sp(u,nu=1)
    assert np.allclose(p[:,:2],sp(u),rtol=0,atol=1e-12)
    assert np.allclose(p[:,2],np.arctan2(v[:,1],v[:,0]),rtol=0,atol=1e-12)

@pytest.mark.parametrize('step',[.025,.1])
def test_longer_spacing_still_rejects_body_collision_between_endpoints(step):
    w=world();w.hard[240-1-40,87]=True
    sp=BSpline([0,0,0,0,1,1,1,1],[[2,2.2],[4,2.2],[6,2.2],[8,2.2]],3)
    with pytest.raises(CurveFailure,match='BODY'):
        certify(sp,w,~w.hard,time.monotonic()+3,{**CONFIG,'max_output_step_m':step})
