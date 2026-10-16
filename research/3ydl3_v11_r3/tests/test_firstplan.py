import os
from dataclasses import asdict
from types import SimpleNamespace
import cv2
import numpy as np
from three_yd_v5.firstplan import sparse_dilate,state_coordinates
from three_yd_v5.first_ack import ComparisonPlan
from three_yd_v5.r6_ack import exact_compare
from three_yd_v5.dirty_transition import corridor_dirty_transition


def test_sparse_dilation_exact_empty_edges_random():
    rng=np.random.default_rng(8)
    for shape in ((17,23),(180,310),(61,61)):
        for pos in ('empty','corner','center','dense'):
            a=np.zeros(shape,np.uint8)
            if pos=='corner':a[0,0]=1;a[-1,-1]=1
            elif pos=='center':a[shape[0]//2,shape[1]//2]=1
            elif pos=='dense':a[:]=(rng.random(shape)>.8)
            for r in (1,9,24):
                y,x=np.mgrid[-r:r+1,-r:r+1];k=(x*x+y*y<=r*r).astype(np.uint8)
                assert np.array_equal(sparse_dilate(a,k),cv2.dilate(a,k))


def test_coordinate_vectorization():
    g=SimpleNamespace(state_cells_linear=np.array([0,1,7,19,30],np.int32),roi_shape=(5,7),bbox=(100,105,33,40))
    assert np.array_equal(state_coordinates(g),[[100,33],[100,34],[101,33],[102,38],[104,35]])


def test_dirty_transition_equivalence(monkeypatch):
    rng=np.random.default_rng(18);monkeypatch.setenv('THREEYD_V8_R2_MODE','AB')
    for shape in ((13,29),(180,170)):
        for old,new in [(np.zeros(shape,bool),np.ones(shape,bool)),(np.zeros(shape,bool),np.zeros(shape,bool)),(rng.random(shape)>.6,rng.random(shape)>.7)]:
            monkeypatch.setenv('THREEYD_V8_FIRST_MODE','control');a=corridor_dirty_transition(old,new)
            monkeypatch.setenv('THREEYD_V8_FIRST_MODE','A');b=corridor_dirty_transition(old,new)
            assert asdict(a)==asdict(b)


def test_ack_same_fields_for_every_readback_and_changed_class():
    rng=np.random.default_rng(16)
    for shape in ((13,19),(130,128)):
        master=rng.integers(0,256,shape,dtype=np.uint8)
        sem=SimpleNamespace(expected_master_cost=master,hard_semantic_mask=rng.random(shape)>.8,
                            soft_cost=rng.choice([0.,20.,140.,200.],shape))
        expected=rng.integers(-1,101,shape,dtype=np.int8);changed=rng.random(shape)>.5
        for prev in (None,np.flipud(master),rng.integers(0,256,shape,dtype=np.uint8)):
            plan=ComparisonPlan(sem,expected,changed,prev)
            for server in (np.flipud(master).copy(),rng.integers(0,256,shape,dtype=np.uint8)):
                assert plan.compare(server)==exact_compare(sem,expected,changed,prev,server)
            # A changed server cannot be accepted from the preceding good read.
            server=np.flipud(master).copy();server[0,0]^=1
            assert plan.compare(server)==exact_compare(sem,expected,changed,prev,server)


def test_moving_dataset_is_common_fixed_goal_with_distinct_poses():
    from pathlib import Path
    from moving_starts import load
    root=Path(__file__).resolve().parents[1]
    for mid in ('mengjia','arena_large_250x300_2d_005'):
        rows=load(root/'datasets/moving_start_v1'/(mid+'.json'),mid)
        for i in range(0,40,4):
            group=rows[i:i+4]
            assert len({r['id'] for r in group})==1
            assert all(r['goal']==group[0]['goal'] for r in group)
            indexes=[r['moving_start']['parent_index'] for r in group]
            assert indexes==sorted(set(indexes))
            assert [r['moving_start']['fraction_requested'] for r in group]==[0.,.2,.5,.8]


def test_classification_storage_is_one_byte_per_cell():
    shape=(15,28);z=np.zeros(shape,np.uint8)
    sem=SimpleNamespace(expected_master_cost=z,hard_semantic_mask=z.astype(bool),soft_cost=z)
    plan=ComparisonPlan(sem,z.astype(np.int8),z.astype(bool),z)
    assert plan.flags.nbytes==np.prod(shape)
