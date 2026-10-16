"""Exact-equivalence checks for the original footprint predicate dispatch."""
from types import SimpleNamespace
import time,hashlib,json
from pathlib import Path
import numpy as np
import pytest
from arena_3d_v1.semantic_world import SweptChecker
from three_yd_v5.pose_batch import first_clear
from three_yd_v5.rectangle_bridge import RectanglePoseCertifier


def world_view():
    world=SimpleNamespace(map=SimpleNamespace(resolution=.05,origin=(-.2,.3,0.)),vehicle=(.265,.225,.4))
    view=SimpleNamespace(safe=np.ones((15,18),bool),world_points=lambda r,c:np.column_stack((-.2+(c+.5)*.15,.3+2.25-(r+.5)*.15)))
    return world,view


def test_batch_uses_single_pose_checks_not_candidate_sweep():
    hard=np.zeros((40,50),bool);hard[:,24:26]=True
    checker=SweptChecker(hard,.05,(0.,0.,0.),.265,.225)
    p=np.array([[.65,1.,0.],[1.85,1.,0.]])
    assert not checker.check(p)  # sweep crosses wall
    assert checker.check(p[:1]) and checker.check(p[1:])
    assert first_clear(checker,p)==(0,1)


@pytest.mark.parametrize('variant',['random','empty','solid','boundary','unknown','sliced'])
def test_batch_matches_each_original_pose(variant):
    rng=np.random.default_rng(83);hard=rng.random((43,57))<.035
    if variant=='empty':hard[:]=False
    if variant=='solid':hard[:]=True
    if variant=='unknown':hard=hard.astype(np.int8)*-1
    if variant=='sliced':hard=hard[::-1,::2]
    checker=SweptChecker(hard,.05,(-.2,.3,0.),.265,.225)
    for _ in range(24):
        p=np.column_stack((rng.uniform(-.4,2.8,24),rng.uniform(.1,2.7,24),rng.uniform(-np.pi,np.pi,24)))
        accepted=[i for i in range(len(p)) if checker.check(p[i:i+1])]
        expected=accepted[0] if accepted else -1
        index,checked=first_clear(checker,p)
        assert index==expected and checked==(index+1 if index>=0 else len(p))


def test_empty_and_nonfinite_pose_guard():
    checker=SweptChecker(np.zeros((10,10)),.05,(0.,0.,0.),.265,.225)
    assert first_clear(checker,np.empty((0,3)))==(-1,0)
    with pytest.raises(ValueError):first_clear(checker,np.array([[np.nan,0.,0.]]))


def test_all_cell_certificates_identical_with_original(monkeypatch):
    world,view=world_view();rng=np.random.default_rng(830)
    hard=np.zeros((45,54),bool);hard[14:19,17:22]=True;hard[29:31,10:40]=True
    bits=rng.integers(0,(1<<24)-1,size=view.safe.shape,dtype=np.uint32)
    cells=[(r,c) for r in range(15) for c in range(18)]+[(-1,0),(0,25)]
    monkeypatch.setenv('THREEYD_V8_R3_MODE','control')
    baseline=RectanglePoseCertifier(world,hard,view,bits)
    expected=[baseline.certify(cell,.2 if i%2 else None) for i,cell in enumerate(cells)]
    monkeypatch.setenv('THREEYD_V8_R3_MODE','A')
    batched=RectanglePoseCertifier(world,hard,view,bits)
    actual=[batched.certify(cell,.2 if i%2 else None) for i,cell in enumerate(cells)]
    assert actual==expected and baseline.cache==batched.cache
    assert batched.batch_calls==270
    assert any(x is None for x in expected) and any(x is not None for x in expected)
    # Cache hits do not dispatch or generate new poses.
    calls=batched.batch_calls
    for cell in cells:batched.certify(cell)
    assert calls==batched.batch_calls


def test_section_policy_depends_on_structure_not_ids():
    from three_yd_v5.early_sections import evidence
    xy=np.array([[0.,0.],[70.,0.],[70.,70.],[140.,70.],[140.,140.],[210.,140.],[210.,210.]])
    c=SimpleNamespace(generic_mode=True,reference_plan=SimpleNamespace(world_xy=xy),rectangle_bridge_cells=np.zeros((100,2),int))
    assert evidence(c)['selected']
    c.generic_mode=False
    assert not evidence(c)['selected']
    c.generic_mode=True;c.rectangle_bridge_cells=np.zeros((0,2),int)
    assert not evidence(c)['selected']
    c.rectangle_bridge_cells=np.zeros((100,2),int);c.reference_plan.world_xy=np.array([[0.,0.],[1000.,0.]])
    assert not evidence(c)['selected']


def test_disabled_policy_does_not_even_measure_or_bind(monkeypatch):
    from three_yd_v5.early_sections import plan
    monkeypatch.setenv('THREEYD_V8_R3_MODE','A')
    assert plan(None,None,None,None,None) is None


def test_rejected_early_sections_retain_one_whole_attempt(monkeypatch):
    import three_yd_v5.early_sections as m
    monkeypatch.setenv('THREEYD_V8_R3_MODE','AB')
    calls=[];cache={}
    c=SimpleNamespace(views={},trajectory=np.zeros((2,3)),trajectory_version=0,
       sync_only=lambda session:{'costmap_update_acknowledged':True,'costmap_ack_mismatch_cells':0,'local_map_update_ms':1.})
    session=SimpleNamespace(client=SimpleNamespace(timeout=7.))
    monkeypatch.setattr(m,'evidence',lambda c:{'selected':True})
    monkeypatch.setattr(m,'strategy_cache',lambda *args:({},'same-input-key'))
    monkeypatch.setattr(m,'get_cache',lambda *args,**kwargs:cache)
    def rejected(*args,**kwargs):
        calls.append('sections');return {'success':False,'failure_code':'SECTION_REJECTED',
           'diagnostics':{'costmap_update_acknowledged':True,'costmap_ack_mismatch_cells':0,'backend_call_count':2},'curvature_repair':{'wall_ms':4}}
    monkeypatch.setattr(m,'recover',rejected)
    def whole(*args,**kwargs):
        assert c.trajectory is None
        calls.append('whole');return {'success':True,'diagnostics':{'costmap_update_acknowledged':True,'costmap_ack_mismatch_cells':0,'backend_call_count':1},'curvature_repair':{'wall_ms':3}}
    monkeypatch.setattr(m.SoftEdgeController,'plan_l3',whole)
    result=m.plan(c,None,session,None,None)
    assert calls==['sections','whole'] and result['success']
    assert result['diagnostics']['backend_call_count']==3 and result['curvature_repair']['wall_ms']==7
    assert result['r3_early_sections']['whole_fallback_called']
    # Same rejected strategy does not repeat its section attempt next time.
    assert m.plan(c,None,session,None,None) is None
