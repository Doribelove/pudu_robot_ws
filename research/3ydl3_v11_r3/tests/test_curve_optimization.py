import time
import numpy as np
import pytest
from scipy import sparse
from three_yd_v5.curve_l3 import rectangles as original,CurveFailure
from three_yd_v5.curve_native import rectangles as native
from three_yd_v5.curve_qp import clarabel_setup,clarabel_solve

@pytest.mark.parametrize('seed',range(8))
def test_native_rectangle_matches_four_axis_reference(seed):
 rng=np.random.RandomState(seed);hard=rng.rand(100,110)<.08
 hard[25:80,25:85]=False
 points=rng.uniform([1.8,1.8],[3.7,3.4],(12,2));tangents=rng.randn(12,2)
 a,b=original(hard,.05,(0.,0.),points,time.monotonic()+5,tangents)
 c,d=native(hard,.05,(0.,0.),points,time.monotonic()+5,tangents)
 assert np.allclose(a,c,rtol=0,atol=1e-10)
 assert np.allclose(b,d,rtol=0,atol=1e-12)

def test_native_rejects_blocked_outside_and_expired():
 for hard,point,deadline,reason in [(np.ones((20,20),bool),[[.5,.5]],time.monotonic()+2,'OUTSIDE'),
                                  (np.zeros((20,20),bool),[[-1,.5]],time.monotonic()+2,'OUTSIDE'),
                                  (np.zeros((20,20),bool),[[.5,.5]],time.monotonic()-1,'BUDGET')]:
  with pytest.raises(CurveFailure,match=reason):native(hard,.05,(0.,0.),point,deadline)

def test_qp_preserves_equality_and_both_infinite_bound_directions():
 P=sparse.eye(2,format='csc')*2;q=np.array([-8.,-6.])
 A=sparse.csc_matrix([[1.,1.],[1.,0.],[0.,1.]])
 result=clarabel_solve(clarabel_setup(P,q,A,np.array([3.,-np.inf,2.]),np.array([3.,1.,np.inf]),time.monotonic()+2,1e-8))
 assert result.info.status_val in (1,2)
 assert np.allclose(result.x,[1,2],atol=1e-6)

def test_qp_infeasibility_never_becomes_a_success():
 result=clarabel_solve(clarabel_setup(sparse.eye(1,format='csc'),np.zeros(1),sparse.csc_matrix([[1.],[1.]]),
  np.array([2.,-np.inf]),np.array([np.inf,1.]),time.monotonic()+2,1e-8))
 assert result.info.status_val not in (1,2)
