"""Independent dense oracles for scope bounds and all occupancy policy bytes."""
import numpy as np
import pytest
from arena_evaluation.semantic_costmap_composer import internal_soft_to_occupancy
from three_yd_v5.r11_fields import bbox
from three_yd_v5.r4_tiles import TiledSoft
from three_yd_v5.v9_r8_fields import sparse_occupancy
from three_yd_v5.tailcost_occupancy import occupancy_on_free_cells


@pytest.mark.parametrize('kind', ['empty','full','corners','sparse','strided'])
def test_axis_bounds_match_coordinate_extrema(kind):
    rng=np.random.default_rng(111)
    mask=np.zeros((73,91),bool)
    if kind=='full':mask[:]=True
    if kind=='corners':mask[0,-1]=mask[-1,0]=True
    if kind in ('sparse','strided'):mask=rng.random(mask.shape)<.002
    if kind=='strided':mask=mask[::-2,::3]
    coordinates=np.column_stack(np.nonzero(mask))
    expected=None if not len(coordinates) else (
        max(0,int(coordinates[:,0].min())-3),min(mask.shape[0],int(coordinates[:,0].max())+4),
        max(0,int(coordinates[:,1].min())-3),min(mask.shape[1],int(coordinates[:,1].max())+4))
    assert bbox(mask,3)==expected


@pytest.mark.parametrize('outside', [0.,.4,1.])
@pytest.mark.parametrize('reverse', [False,True])
def test_masked_conversion_preserves_unknown_hard_and_overlapping_pieces(outside,reverse):
    rng=np.random.default_rng(112);shape=(37,259)
    raw=np.resize(np.arange(-128,128,dtype=np.int16).astype(np.int8),shape)
    hard=rng.random(shape)<.3;drivable=rng.random(shape)>=outside
    if reverse:raw=raw[::-1,::-1];hard=hard[:,::-1];drivable=drivable[::-1]
    pieces=[((1,22,3,250),rng.uniform(-10,400,(21,247)).astype(np.float32)),
            ((4,37,2,256),rng.uniform(-10,400,(33,254)).astype(np.float32))]
    soft=TiledSoft(shape,140.,drivable,pieces)
    expected=np.maximum(raw.astype(np.int16),internal_soft_to_occupancy(np.asarray(soft))).astype(np.int8)
    expected[raw<0]=-1;expected[hard]=100
    originals=[x.copy() for x in (raw,hard,drivable)]
    assert np.array_equal(sparse_occupancy(raw,soft,hard,rows=7)[0],expected)
    assert np.array_equal(occupancy_on_free_cells(raw,soft,hard,rows=7),expected)
    for before,after in zip(originals,(raw,hard,drivable)):assert np.array_equal(before,after)


@pytest.mark.parametrize('tiled',[False,True])
@pytest.mark.parametrize('reverse',[False,True])
@pytest.mark.parametrize('firstcost',['control','AB'])
def test_first_publication_matches_original_full_ack_on_every_observation(monkeypatch,tiled,reverse,firstcost):
    from types import SimpleNamespace
    from three_yd_v5.first_ack import ComparisonPlan
    from three_yd_v5.r6_ack import exact_compare
    monkeypatch.setenv('THREEYD_V10_R3_MODE','ABRTHI')
    monkeypatch.setenv('THREEYD_V8_R11_MODE',firstcost)
    monkeypatch.setenv('THREEYD_V10_R4_MODE','D')
    rng=np.random.default_rng(113);shape=(77,129)
    master=rng.integers(0,256,shape,dtype=np.uint8)
    hard=rng.random(shape)<.3;changed=rng.random(shape)<.15
    soft=rng.choice([0.,-1.,140.,np.nan],shape).astype(np.float32)
    expected=rng.integers(-1,101,shape,dtype=np.int8)
    if reverse:master=master[::-1,::-1];hard=hard[:,::-1];expected=expected[:,::-1]
    if tiled:soft=TiledSoft(shape,140.,rng.random(shape)<.8,[((0,77,0,129),soft)])
    semantic=SimpleNamespace(expected_master_cost=master,hard_semantic_mask=hard,soft_cost=soft)
    plan=ComparisonPlan(semantic,expected,changed,None,rows=11)
    for actual in (plan.master.copy(),rng.integers(0,256,shape,dtype=np.uint8)):
        assert plan.compare(actual,rows=17)==exact_compare(semantic,expected,changed,None,actual,rows=13)
    actual=plan.master.copy();actual[-1,0]^=1
    assert plan.compare(actual)==exact_compare(semantic,expected,changed,None,actual)
    assert plan.compare(actual)['mismatch']==1
    # A later observation must be read again, even after an equal observation.
    assert plan.compare(plan.master.copy())['mismatch']==0
    assert plan.compare(actual)['mismatch']==1
