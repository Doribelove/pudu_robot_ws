import time
import math
from types import SimpleNamespace
import numpy as np
import pytest
from three_yd_v5.r10_tiles import rectangles
from three_yd_v5.r10_preflight import optimistic_reachable

def test_sparse_updates_cover_opens_closes_and_payload():
    rng=np.random.default_rng(9)
    for shape in [(91,127),(400,1000),(7,2200)]:
        old=np.zeros(shape,np.int8);new=old.copy()
        new[1:80,4:28]=100;new[-70:-1,-30:-5]=70
        old[shape[0]//2:shape[0]//2+3,40:80]=50
        changed=old!=new;boxes,info=rectangles(changed,1024)
        result=old.copy();cover=np.zeros(shape,bool)
        for a,b,c,d in boxes:
            assert (b-a)*(d-c)<=1024
            result[a:b,c:d]=new[a:b,c:d];cover[a:b,c:d]=True
        assert np.array_equal(result,new)
        assert np.all(cover[changed])
        assert info['published_cells']<=info['baseline_cells']

def test_empty_and_dense_updates():
    assert rectangles(np.zeros((3,4),bool))[0]==[]
    boxes,info=rectangles(np.ones((50,70),bool),1024)
    assert info['published_cells']==3500
    assert len(boxes)==info['baseline_messages']

def test_direction_preflight_rejects_reverse_not_lateral():
    safe=np.ones((5,8),bool);direction=np.zeros((5,8,2));direction[:,:,0]=1
    assert optimistic_reachable(safe,direction,(2,1),(2,6))[0]
    assert not optimistic_reachable(safe,direction,(2,6),(2,1))[0]
    assert optimistic_reachable(safe,direction,(1,4),(4,4))[0]

def test_preflight_preserves_relaxed_bridge_and_forbids_diagonal_corner_cut():
    safe=np.ones((5,7),bool);safe[:,3]=False;direction=np.zeros((5,7,2))
    assert not optimistic_reachable(safe,direction,(2,0),(2,6))[0]
    safe[2,3]=True
    assert optimistic_reachable(safe,direction,(2,0),(2,6))[0]
    assert not optimistic_reachable(np.eye(3,dtype=bool),np.zeros((3,3,2)),(0,0),(2,2))[0]

def test_numerical_connector_matches_original_and_cache_is_detached(monkeypatch):
    from three_yd_v5 import r10_connector as module
    from arena_3d_v1.curvature_repair import fit_connector as original
    monkeypatch.setenv('THREEYD_R10_MODE','C');module._CACHE.clear()
    args=(np.array([0.,0.,0.]),np.array([1.,0.,0.]),'LSL',[0.,2.,0.],.5,0.,0.)
    a,detail=original(*args,time.monotonic()+5,2.5)
    b,other=module.fit_connector(*args,time.monotonic()+5,2.5)
    assert np.array_equal(a,b);assert detail==other
    b[:]=99
    c,_=module.fit_connector(*args,time.monotonic()+5,2.5)
    assert np.array_equal(a,c)
    with pytest.raises(TimeoutError):module.fit_connector(*args,time.monotonic()-1,2.5)

def test_connector_exact_key_includes_slack_and_boundary_curvature(monkeypatch):
    from three_yd_v5 import r10_connector as module
    monkeypatch.setenv('THREEYD_R10_MODE','C');module._CACHE.clear()
    calls=[]
    def fake(*args):
        calls.append(args);return np.zeros((2,3)),{'ok':True}
    monkeypatch.setattr(module,'_fit_uncached',fake)
    args=([0.,0.,0.],[1.,0.,0.],'LSL',[0.,2.,0.],.5,0.,0.)
    module.fit_connector(*args,time.monotonic()+5,2.5)
    module.fit_connector(*args,time.monotonic()+5,2.5)
    modified=list(args);modified[3]=[0.,2.00000001,0.]
    module.fit_connector(*modified,time.monotonic()+5,2.5)
    modified=list(args);modified[5]=.00000001
    module.fit_connector(*modified,time.monotonic()+5,2.5)
    assert len(calls)==3


def test_preflight_matches_existing_directed_edge_graph_randomized():
    from arena_3d_v1.multires_grid import make_roi
    from arena_3d_v1.l2_state_lifecycle import CompactCorridorGeometry,MutableStateBinding
    from three_yd_v4.r5_controller import EdgeState
    rng=np.random.default_rng(710)
    for _ in range(12):
        safe=rng.random((7,9))>.22;safe[0,0]=safe[-1,-1]=True
        dirs=rng.choice([-1.,0.,1.],(7,9,2)).astype(np.float32)
        view=SimpleNamespace(key='x',safe=safe,factor=3,vehicle=(.265,.225,.4),
            map=SimpleNamespace(height=7,width=9,origin=(0.,0.,0.),resolution=.15))
        roi=make_roi(view,np.ones_like(safe),(0,0),(6,8),'x')
        geom=CompactCorridorGeometry.build(roi)
        rc=np.array([geom.global_cell(i) for i in range(geom.state_count)])
        binding=MutableStateBinding(geom.binding.digest,roi.binding.start_cell,roi.binding.goal_cell,'static')
        state=EdgeState(geom,binding,np.zeros(len(rc)),2.,dirs[rc[:,0],rc[:,1]],np.zeros(len(rc)))
        todo=[state.start_id];seen=set(todo)
        while todo:
            u=todo.pop()
            for v in geom.neighbor_ids(u):
                if v not in seen and math.isfinite(state._edge_cost(u,v)):
                    seen.add(v);todo.append(v)
        assert optimistic_reachable(safe,dirs,(0,0),(6,8))[0]==(state.goal_id in seen)
