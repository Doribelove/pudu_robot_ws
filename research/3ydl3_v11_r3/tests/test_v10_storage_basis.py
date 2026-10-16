from types import SimpleNamespace as S
import numpy as np
import pytest
from scipy.interpolate import BSpline
from three_yd_v5.curve_l3 import basis
from three_yd_v5.v9_maps import seal,immutable
from three_yd_v5.v10_storage import collision_checker
from three_yd_v3.dynamic import DynamicObstacles

@pytest.mark.parametrize('order',range(4))
@pytest.mark.parametrize('seed',range(5))
def test_sparse_basis_exact_against_dense_identity(order,seed):
    rng=np.random.default_rng(seed)
    internal=np.sort(rng.uniform(0,20,150));knots=np.r_[[0.]*4,internal,[20.]*4]
    u=np.r_[rng.uniform(-.1,20.1,777),knots];n=len(knots)-4
    old=BSpline(knots,np.eye(n),3)(u,nu=order)
    np.testing.assert_array_equal(basis(knots,u,order).toarray(),old)

def test_checker_borrows_exact_bool_bytes_and_keeps_body_check():
    hard=np.zeros((100,100),bool);hard[50,50]=True
    w=S(map=S(resolution=.05,origin=(0.,0.,0.)),vehicle=(.265,.225,.4))
    check=collision_checker(hard,w)
    assert np.shares_memory(check.hard,hard)
    assert check.check(np.array([[1.,1.,0.]]))
    assert not check.check(np.array([[2.525,2.475,0.]]))

def test_dynamic_shared_base_survives_add_and_clear():
    h=seal(np.zeros((100,100),bool));w=S(hard=h)
    w.cells=lambda p: (99-np.floor(np.asarray(p)[:,1]/.05).astype(int),np.floor(np.asarray(p)[:,0]/.05).astype(int))
    d=DynamicObstacles(w,static_hard=h)
    assert d.static_hard is h and immutable(h)
    with pytest.raises(ValueError): h.setflags(write=True)
    d.apply(1,upsert={'box':[[2,2],[2.2,2],[2.2,2.2],[2,2.2]]})
    assert w.hard.any() and not h.any()
    d.apply(2,clear=['box'],observed_free=True)
    np.testing.assert_array_equal(w.hard,h)
    with pytest.raises(ValueError):DynamicObstacles(w,static_hard=w.hard)
