"""Grouped dispatch must reproduce the original ordered per-cell certificates."""
from types import SimpleNamespace
import numpy as np
import pytest
from arena_3d_v1.semantic_world import SweptChecker
from three_yd_v5.rectangle_bridge import RectanglePoseCertifier,certify_relaxed_path
from three_yd_v5.pose_groups import dispatch,certify_many

def fixture(variant):
    world=SimpleNamespace(map=SimpleNamespace(resolution=.05,origin=(-.2,.3,0.)),vehicle=(.265,.225,.4))
    view=SimpleNamespace(safe=np.ones((18,22),bool),world_points=lambda r,c:np.column_stack((-.2+(c+.5)*.15,.3+2.7-(r+.5)*.15)))
    rng=np.random.default_rng(831);hard=rng.random((54,66))<.01
    if variant=='empty':hard[:]=False
    if variant=='solid':hard[:]=True
    if variant=='boundary':
        hard[:]=False;hard[16:24,16:28]=True;hard[38:41,8:45]=True
    if variant=='sliced':hard=hard[::-1,:]
    bits=rng.integers(0,(1<<24)-1,size=view.safe.shape,dtype=np.uint32)
    if variant=='ties':bits[:,::2]=0
    cells=[(r,c) for r in range(18) for c in range(22)]+[(-1,0),(18,0),(0,22),(3,4),(3,4)]
    yaws=[None if i%5==0 else [0.,np.pi/24,-np.pi,17.25][i%4] for i in range(len(cells))]
    return world,view,hard,bits,cells,yaws

@pytest.mark.parametrize('variant',['random','empty','solid','boundary','sliced','ties'])
@pytest.mark.parametrize('chunk',[1,17,64])
def test_all_certificates_and_counts_match_original(monkeypatch,variant,chunk):
    monkeypatch.setenv('THREEYD_V8_R3_MODE','A')
    world,view,hard,bits,cells,yaws=fixture(variant)
    old=RectanglePoseCertifier(world,hard,view,bits);new=RectanglePoseCertifier(world,hard,view,bits)
    expected=[old.certify(cell,yaw) for cell,yaw in zip(cells,yaws)]
    actual=certify_many(new,cells,yaws,chunk)
    assert actual==expected and new.cache==old.cache
    assert new.exact_pose_checks==old.exact_pose_checks and new.batch_calls==old.batch_calls
    calls=new.native_group_calls
    assert certify_many(new,cells[::-1],[1.7]*len(cells),chunk)==expected[::-1]
    assert new.native_group_calls==calls

def test_groups_never_form_a_sweep():
    hard=np.zeros((40,50),bool);hard[:,24:26]=True
    checker=SweptChecker(hard,.05,(0.,0.,0.),.265,.225)
    centers=np.array([[.65,1.],[1.85,1.]])
    assert not checker.check(np.c_[centers,[0.,0.]])
    index,counts=dispatch(checker,centers,np.zeros((2,1)),np.zeros((1,2)))
    assert index.tolist()==[0,0] and counts.tolist()==[1,1]

def test_duplicate_cache_prefers_first_occurrence(monkeypatch):
    monkeypatch.setenv('THREEYD_V8_R3_MODE','A')
    world,view,hard,bits,_,_=fixture('empty')
    old=RectanglePoseCertifier(world,hard,view,bits);new=RectanglePoseCertifier(world,hard,view,bits)
    cells=[(8,8),(8,8),(9,9),(8,8)];yaws=[0.,2.,None,-3.]
    assert certify_many(new,cells,yaws)==[old.certify(c,y) for c,y in zip(cells,yaws)]
    assert new.batch_calls==2

def test_invalid_inputs_and_empty_groups():
    checker=SweptChecker(np.zeros((20,20)),.05,(0.,0.,0.),.265,.225)
    indices,counts=dispatch(checker,np.empty((0,2)),np.empty((0,24)),np.zeros((1,2)))
    assert not len(indices) and not len(counts)
    with pytest.raises(ValueError):dispatch(checker,np.array([[np.nan,0.]]),np.zeros((1,24)),np.zeros((1,2)))
    with pytest.raises(ValueError):dispatch(checker,np.zeros((1,2)),np.zeros((2,24)),np.zeros((1,2)))
    with pytest.raises(ValueError):dispatch(checker,np.zeros((1,2)),np.zeros((1,24)),np.zeros((0,2)))

def test_relaxed_path_order_and_invalid_cells_match(monkeypatch):
    world,view,hard,bits,cells,_=fixture('boundary');path=np.array(cells[:396]+cells[:30])
    safe=np.zeros(view.safe.shape,bool);safe[::3,::4]=True
    monkeypatch.setenv('THREEYD_V8_R3_MODE','A')
    old=RectanglePoseCertifier(world,hard,view,bits);new=RectanglePoseCertifier(world,hard,view,bits)
    monkeypatch.setenv('THREEYD_V10_R8_MODE','STP');expected=certify_relaxed_path(path,safe,old)
    monkeypatch.setenv('THREEYD_V10_R8_MODE','STPB');actual=certify_relaxed_path(path,safe,new)
    assert actual==expected and old.cache==new.cache
