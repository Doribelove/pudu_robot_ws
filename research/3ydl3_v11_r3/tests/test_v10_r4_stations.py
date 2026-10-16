from types import SimpleNamespace
import time
import numpy as np
from three_yd_v5.tail_stations import stations
from three_yd_v5.curve_l3 import _optimize_once,CONFIG


def test_sparse_solver_grid_keeps_endpoints_and_dense_bends():
    p=np.array([[1.,1.],[21.,1.],[21.,21.]])
    s=np.array([0.,20.,40.]);u=stations(p,s,.1)
    assert u[0]==0. and u[-1]==40. and np.all(np.diff(u)>1e-6)
    assert len(u)<len(np.arange(0,40.,.1))
    dense=np.arange(0,40.,.1)
    for t in dense[(abs(dense-20)<1.19)|(dense<5)|(dense>35)]:
        assert np.min(abs(u-t))<1e-6


def test_sparse_solver_retains_original_dense_certificate_corridor():
    w=SimpleNamespace(v7_map_only=True,hard=np.zeros((160,900),bool),vehicle=(.265,.225,.4),
                      map=SimpleNamespace(resolution=.05,origin=(0.,0.)))
    guide=np.array([[1.,3.],[12.,3.],[14.,3.5],[25.,3.5]])
    cfg=dict(CONFIG,curvature_target_ratio=.97)
    old=_optimize_once(w,np.ones_like(w.hard),guide,(1.,3.,0.),(25.,3.5,0.),time.monotonic()+5.,cfg)
    new=_optimize_once(w,np.ones_like(w.hard),guide,(1.,3.,0.),(25.,3.5,0.),time.monotonic()+5.,dict(cfg,r4_adaptive_stations=True))
    assert old[0] is not None and new[0] is not None
    assert old[1]['corridor']==new[1]['corridor']
    assert new[2]['r4_solver_stations']<new[2]['r4_original_certificate_stations']
    assert new[2]['certificate']['max_curvature_bound_1pm']<=2.5+1e-8
    assert np.allclose(new[0][[0,-1]],[(1.,3.,0.),(25.,3.5,0.)],atol=1e-10)


def test_sampling_change_cannot_accept_blocked_or_expired_request():
    w=SimpleNamespace(v7_map_only=True,hard=np.ones((160,160),bool),vehicle=(.265,.225,.4),
                      map=SimpleNamespace(resolution=.05,origin=(0.,0.)))
    p=np.array([[1.,1.],[4.,1.]])
    for deadline in (time.monotonic()+2.,time.monotonic()-1.):
        path,_,log=_optimize_once(w,np.ones_like(w.hard),p,(1.,1.,0.),(4.,1.,0.),deadline,
                                  dict(CONFIG,r4_adaptive_stations=True,curvature_target_ratio=.97))
        assert path is None and not log['success']


def test_semantic_world_keeps_original_curve_despite_adaptive_flags():
    w=SimpleNamespace(hard=np.zeros((160,900),bool),vehicle=(.265,.225,.4),
                      map=SimpleNamespace(resolution=.05,origin=(0.,0.)))
    guide=np.array([[1.,3.],[12.,3.],[14.,3.5],[25.,3.5]])
    args=(w,np.ones_like(w.hard),guide,(1.,3.,0.),(25.,3.5,0.))
    old=_optimize_once(*args,time.monotonic()+5.,CONFIG)
    new=_optimize_once(*args,time.monotonic()+5.,dict(CONFIG,r4_adaptive_stations=True,r4_tighter_primary=True))
    assert np.array_equal(old[0],new[0]) and old[1]==new[1]
