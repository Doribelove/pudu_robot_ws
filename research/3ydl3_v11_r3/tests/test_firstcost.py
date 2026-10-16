from dataclasses import dataclass
from types import SimpleNamespace
import cv2
import numpy as np
from three_yd_v5.firstcost import region_membership,soft_positive_rows,neutral_adapter
from three_yd_v5.first_ack import ComparisonPlan
from three_yd_v5.r6_ack import exact_compare
from three_yd_v5.r4_tiles import TiledSoft
from three_yd_v5.controller import V5Controller
from three_yd_v5.free_space_topology import FreeSpaceTopology


def test_id_lookup_handles_all_small_integer_domain_and_unknowns():
    rng=np.random.default_rng(911)
    for dtype in (np.int8,np.uint8,np.int16,np.uint16,np.int32,np.float32):
        limits=np.iinfo(dtype) if np.dtype(dtype).kind in 'iu' else SimpleNamespace(min=-20,max=1000)
        a=rng.integers(limits.min,min(limits.max,1200)+1,(77,93)).astype(dtype)
        ids=[-32768,-1,0,1,36,255,1000,32767,65535,65536,1.25]
        assert np.array_equal(region_membership(a,ids),np.isin(a,ids))
        assert np.array_equal(region_membership(a,[]),np.zeros(a.shape,bool))


def test_sparse_soft_classification_matches_dense_values_without_float_expansion():
    rng=np.random.default_rng(71);shape=(173,237)
    drivable=rng.random(shape)>.15
    pieces=[((0,65,3,70),rng.choice([0.,10.,140.,-1.,np.nan],(65,67)).astype(np.float32)),
            ((65,173,80,200),rng.choice([0.,10.,140.],(108,120)).astype(np.float32))]
    for cap in (0.,140.,-1.):
        soft=TiledSoft(shape,cap,drivable,pieces)
        for a,b in ((0,1),(0,64),(50,130),(130,173)):
            assert np.array_equal(soft_positive_rows(soft,a,b),np.asarray(soft)[a:b]>0)


def test_ack_classes_hashes_corruption_and_consecutive_actual_reads(monkeypatch):
    rng=np.random.default_rng(112);shape=(171,213)
    drivable=rng.random(shape)>.2
    soft=TiledSoft(shape,140,drivable,[((60,120,5,200),rng.choice([0.,140.,200.],(60,195)).astype(np.float32))])
    master=rng.integers(0,256,shape,dtype=np.uint8)
    sem=SimpleNamespace(expected_master_cost=master,hard_semantic_mask=rng.random(shape)>.3,soft_cost=soft)
    expected=rng.integers(-1,101,shape,dtype=np.int8);changed=rng.random(shape)>.3
    for previous in (None,np.flipud(master).copy(),rng.integers(0,256,shape,dtype=np.uint8)):
        monkeypatch.setenv('THREEYD_V8_R11_MODE','control');old=ComparisonPlan(sem,expected,changed,previous)
        monkeypatch.setenv('THREEYD_V8_R11_MODE','B');new=ComparisonPlan(sem,expected,changed,previous)
        assert np.array_equal(new.flags,old.flags)
        for server in (np.flipud(master).copy(),rng.integers(0,256,shape,dtype=np.uint8)):
            assert new.compare(server)==old.compare(server)==exact_compare(sem,expected,changed,previous,server)
        actual=np.flipud(master).copy();actual[33,17]^=1
        assert new.compare(actual)==exact_compare(sem,expected,changed,previous,actual)
        assert new.compare(actual)['mismatch']>0


def test_neutral_parking_algebra_preserves_all_obstacles_and_scope(monkeypatch):
    rng=np.random.default_rng(72);hard=rng.random((111,171))>.7
    @dataclass
    class Plan:
        corridor_mask: np.ndarray
        diagnostics: dict
    c=V5Controller.__new__(V5Controller)
    c.generic_mode=True;c.topology=FreeSpaceTopology.__new__(FreeSpaceTopology)
    c.world=SimpleNamespace(hard=hard);c.timing={}
    c.topology.world=c.world;c.topology._subregions=None
    c.plan=Plan(np.zeros_like(hard),{'test':True})
    c._search_mask=rng.random(hard.shape)>.5
    scope=c._search_mask.copy()
    # The original adapter's only region=1 iff the original cell is free.
    labels=(~hard).astype(np.int16)
    selected=np.isin(labels,[1]);margin=cv2.dilate(selected.astype(np.uint8),np.ones((19,19),np.uint8))>0
    expected=(~hard)&np.isin(labels,[1])&margin
    monkeypatch.setenv('THREEYD_V8_R11_MODE','A')
    assert c._bind_endpoint_parking_access(np.array([[1.,2.],[10.,20.]]))==0
    assert np.array_equal(c.plan.corridor_mask,expected)
    assert np.array_equal(c._search_mask,scope)
    assert not np.any(c.plan.corridor_mask[hard])
    class CustomTopology(FreeSpaceTopology):pass
    c.topology=CustomTopology.__new__(CustomTopology)
    assert not neutral_adapter(c)


def test_neutral_labels_stay_static_through_add_clear_and_fresh_controller(monkeypatch):
    @dataclass
    class Plan:
        corridor_mask: np.ndarray
        diagnostics: dict
    hard=np.zeros((91,111),bool);hard[:,0]=True;hard[40:51,60:71]=True
    world=SimpleNamespace(hard=hard.copy())
    top=FreeSpaceTopology.__new__(FreeSpaceTopology);top.world=world;top._subregions=None
    monkeypatch.setenv('THREEYD_V8_R11_MODE','A')
    def fresh():
        c=V5Controller.__new__(V5Controller);c.generic_mode=True;c.topology=top;c.world=world
        c.timing={};c.plan=Plan(np.zeros_like(hard),{});c._bind_endpoint_parking_access(np.empty((0,2)))
        return c
    first=fresh();original=top.subregions.copy()
    world.hard=hard.copy();world.hard[22:29,22:29]=True
    changed=fresh()
    assert np.array_equal(top.subregions,original)
    assert np.array_equal(changed.global_drivable,original==1)
    assert not changed.plan.corridor_mask[25,25]
    world.hard=hard.copy();cleared=fresh()
    assert np.array_equal(cleared.plan.corridor_mask,first.plan.corridor_mask)
    assert cleared.plan.corridor_mask[25,25]
