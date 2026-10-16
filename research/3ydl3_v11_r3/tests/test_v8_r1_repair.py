from pathlib import Path
from types import SimpleNamespace
import hashlib,json,time
import numpy as np
import pytest
from three_yd_v5.bounded_segments import segment_anchors
from three_yd_v5 import endpoint_repair

ROOT=Path(__file__).resolve().parents[1]


def test_l1_and_fixed_inputs_still_frozen():
    lock=json.loads((ROOT/'L1_LOCK.json').read_text())
    for section in ('source_files','input_files','pixel_input_files'):
        for relative,expected in lock[section].items():
            assert hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()==expected,relative


def test_section_anchors_are_on_reference_straights_and_bounded():
    xy=np.array([[0.,0.],[140.,0.],[140.,140.],[260.,140.]])
    a=segment_anchors(xy,(0.,0.,.5),(260.,140.,-.4))
    assert len(a)==5 and a[0]==(0.,0.,.5) and a[-1]==(260.,140.,-.4)
    for p in a[1:-1]:
        assert min(np.linalg.norm(np.asarray(p[:2])-v) for v in xy)>2.
        on_line=any(abs(np.cross(y-x,np.asarray(p[:2])-x))<1e-8 and
                    0<np.dot(np.asarray(p[:2])-x,y-x)<np.dot(y-x,y-x)
                    for x,y in zip(xy,xy[1:]))
        assert on_line


def test_missing_straight_anchor_does_not_force_corner():
    with pytest.raises(ValueError,match='NO_STRAIGHT_SEGMENT_ANCHOR'):
        segment_anchors(np.array([[0.,0.],[.1,0.],[.1,.1]]),(0.,0.,0.),(.1,.1,1.))


def test_ccc_seeds_are_attempted_and_slack_preserves_heading(monkeypatch):
    calls=[]
    monkeypatch.setattr(endpoint_repair,'dubins_choices',lambda *a:[('RLR',(.1,.2,.1))])
    def probe(start,goal,word,params,*args):
        calls.append((word,params));raise ValueError('PROBE_NO_FIT')
    monkeypatch.setattr(endpoint_repair,'fit_connector',probe)
    world=SimpleNamespace(vehicle=(.265,.225,.4))
    p=np.array([[0.,0.,0.],[.1,0.,.5],[.2,.01,0.]])
    fixed,log=endpoint_repair.repair_path_extended(world,None,None,p,None,budget_s=.5)
    assert fixed is None and calls
    assert all(word=='RLR' and abs(-par[0]+par[1]-par[2])<1e-12 for word,par in calls)


def test_repair_budget_is_not_reset_for_new_window(monkeypatch):
    def unexpected(*a,**kw):raise AssertionError('fit after deadline')
    monkeypatch.setattr(endpoint_repair,'fit_connector',unexpected)
    p=np.array([[0.,0.,0.],[.1,0.,.5],[.2,.01,0.]])
    fixed,log=endpoint_repair.repair_path_extended(SimpleNamespace(vehicle=(.265,.225,.4)),None,None,p,None,budget_s=0.)
    assert fixed is None and 'BUDGET' in log['failure']
