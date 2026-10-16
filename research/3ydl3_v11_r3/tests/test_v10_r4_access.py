from dataclasses import dataclass
from types import SimpleNamespace as S
import cv2
import numpy as np
import pytest
from three_yd_v5.controller import V5Controller
from three_yd_v5.tail_access import remember,reuse


@dataclass
class Plan:
    corridor_mask: np.ndarray
    diagnostics: dict


def controller(seed):
    rng=np.random.default_rng(seed);h,w=45,60;res=.05
    labels=rng.integers(0,5,(h,w),dtype=np.int16)
    hard=rng.random((h,w))<.15
    c=V5Controller.__new__(V5Controller)
    c.world=S(hard=hard,map=S(resolution=res,origin=(0.,0.),height=h,width=w))
    c.world.cells=lambda xy:(h-1-np.floor(np.asarray(xy)[:,1]/res).astype(int),np.floor(np.asarray(xy)[:,0]/res).astype(int))
    c.topology_route=S(region_ids=[1,2],signature='route')
    c.topology=S(subregions=labels,regions={0:{'kind':'unknown'},1:{'kind':'lane'},2:{'kind':'junction_area'},3:{'kind':'parking_area'},4:{'kind':'lane'}},
       contexts={'route':{'endpoint_access':{'start':{'semantic_region_id':1,'road_id':1},'goal':{'semantic_region_id':2,'road_id':1}}}})
    c.generic_mode=False;c.r13_semantic_seam=None;c.traffic=S(endpoint_parking=set());c.timing={}
    selected=np.isin(labels,[1,2]);margin=cv2.dilate(selected.astype(np.uint8),np.ones((19,19),np.uint8))>0
    c.global_drivable=np.isin(labels,[1,2,4])&margin
    c.plan=Plan(~hard&c.global_drivable,{'v5_runtime_mask':True})
    return c


@pytest.mark.parametrize('seed',range(16))
def test_no_parking_mask_and_diagnostics_match_original(monkeypatch,seed):
    old=controller(seed);new=controller(seed);xy=np.array([[.5,.5],[1.2,1.5],[2.,1.]])
    monkeypatch.setenv('THREEYD_V10_R4_MODE','control')
    assert old._bind_endpoint_parking_access(xy)==0
    monkeypatch.setenv('THREEYD_V10_R4_MODE','P')
    remember(new)
    assert new._bind_endpoint_parking_access(xy)==0
    assert new.timing['r4_identical_nonparking_access_reused']
    assert np.array_equal(old.global_drivable,new.global_drivable)
    assert np.array_equal(old.plan.corridor_mask,new.plan.corridor_mask)
    assert old.plan.diagnostics==new.plan.diagnostics
    assert not new.global_drivable.flags.writeable


@pytest.mark.parametrize('change',['labels','hard','regions','route','context','parking','seam','global','mask'])
def test_actual_current_changes_prevent_reuse(monkeypatch,change):
    monkeypatch.setenv('THREEYD_V10_R4_MODE','P');c=controller(21);remember(c)
    assert c._r4_access_binding is not None
    if change=='labels':c.topology.subregions[0,0]=42
    elif change=='hard':c.world.hard[0,0]=not c.world.hard[0,0]
    elif change=='regions':c.topology.regions[4]['kind']='parking_area'
    elif change=='route':c.topology_route.region_ids=[1]
    elif change=='context':c.topology.contexts['route']['endpoint_access']['start']['road_id']=4
    elif change=='parking':c.traffic.endpoint_parking={3}
    elif change=='seam':c.r13_semantic_seam=np.zeros_like(c.world.hard)
    elif change=='global':c.global_drivable=np.logical_not(c.global_drivable)
    elif change=='mask':c.plan.corridor_mask=np.logical_not(c.plan.corridor_mask)
    assert not reuse(c)


def test_parking_route_is_never_eligible(monkeypatch):
    monkeypatch.setenv('THREEYD_V10_R4_MODE','P');c=controller(21)
    c.topology_route.region_ids.append(3);remember(c)
    assert not reuse(c)
