from types import SimpleNamespace
import numpy as np
import cv2
import pytest
from three_yd_v5.r4_tiles import reference_soft,bounded_inflation
from three_yd_v5.r11_fields import reference_soft as original_soft,bounded_inflation as original_inflation
from three_yd_v5.r4_fields import road_fields,point_free,resident

@pytest.mark.parametrize('shape',[(31,49),(513,777),(801,1301)])
def test_tile_halos_match_full_fields_at_edges_unknowns_and_turns(shape):
    rng=np.random.default_rng(194);h,w=shape
    references=[np.array([[0,0],[h-1,w-1]]),np.array([[h//4,w//4],[h//4,3*w//4],[3*h//4,3*w//4]]),np.array([[h-1,0],[h-1,w-1]])]
    for ref in references:
        drivable=rng.random(shape)>.2
        expected,_=original_soft(shape,ref,.05,drivable)
        got,detail=reference_soft(shape,ref,.05,drivable,tile=128)
        assert np.array_equal(expected,np.asarray(got))
        assert np.array_equal(got>0,expected>0)
        assert np.array_equal(got[3:h-2,7:w-4],expected[3:h-2,7:w-4])
        rr=rng.integers(0,h,80);cc=rng.integers(0,w,80)
        assert np.array_equal(got[rr,cc],expected[rr,cc])
    static=np.full(shape,254,np.uint8)
    mask=np.zeros(shape,np.uint8);cv2.polylines(mask,[references[1][:,::-1]],False,1,25)
    static[mask>0]=rng.integers(0,256,int(mask.sum()),dtype=np.uint8)
    expected,_=original_inflation(static,.05)
    got,detail=bounded_inflation(static,.05,tile=128)
    assert np.array_equal(expected,got)


def test_no_semantics_uses_constant_values_not_large_arrays():
    c=SimpleNamespace(generic_mode=True)
    norm,eligible,direction=road_fields(c,None,(0,2000,0,2000))
    assert resident(norm,eligible,direction)==9
    assert np.isnan(norm[1999,1999]) and not eligible[9,9]
    assert np.array_equal(direction[5,5],[0.,0.])
    assert not direction.flags.writeable


def test_point_free_preserves_scope_and_map_bounds():
    world=SimpleNamespace(map=SimpleNamespace(height=7,width=9),cells=lambda xy:(xy[:,0].astype(int),xy[:,1].astype(int)))
    view=SimpleNamespace(world_points=lambda r,c:np.column_stack((r,c)))
    scope=np.array([[1,2],[3,4],[7,8],[6,8]])
    hard=np.zeros((7,9),bool);hard[3,4]=True
    got=point_free(world,view,scope,hard,(9,10))
    assert np.array_equal(np.argwhere(got),[[1,2],[6,8]])


def test_tiled_compose_retains_exact_occupancy_master_ack_and_policy(monkeypatch):
    from three_yd_v5.fast_costmap import compose_bounded
    shape=(737,1397);rng=np.random.default_rng(715)
    w=SimpleNamespace(hard=rng.random(shape)<.015,map=SimpleNamespace(occupancy=rng.choice(np.array([-1,0,100],np.int8),shape,p=[.01,.98,.01]),resolution=.05),semantic=SimpleNamespace(semantic_map_hash='map'))
    ref=np.array([[50,50],[50,900],[620,900],[620,1300]])
    c=SimpleNamespace(world=w,reference_cells=ref,global_drivable=rng.random(shape)>.01,plan=SimpleNamespace(corridor_mask=np.ones(shape,bool)),dynamic=None,task={'signature':'route'})
    mask=np.zeros(shape,np.uint8);cv2.polylines(mask,[ref[:,::-1]],False,1,35);mask=mask.astype(bool)
    monkeypatch.setenv('THREEYD_R11_MODE','AB');monkeypatch.setenv('THREEYD_V8_R4_MODE','control')
    old=compose_bounded(c,mask)
    monkeypatch.setenv('THREEYD_V8_R4_MODE','B');new=compose_bounded(c,mask)
    for key in ('internal_cost','occupancy_grid','expected_master_cost','soft_cost','hard_semantic_mask','affected_mask'):
        assert np.array_equal(np.asarray(getattr(old,key)),np.asarray(getattr(new,key))),key
    assert old.policy_hash==new.policy_hash
    assert old.expected_grid_hash==new.expected_grid_hash and old.expected_master_hash==new.expected_master_hash
    assert new.soft_cost.nbytes<old.soft_cost.nbytes


def test_large_real_reference_preserves_original_distance_domain():
    # A small image does not expose OpenCV EDT's size-dependent result. Keep
    # this exact 5000x6000 long-route counterexample as a regression fixture.
    import json
    from pathlib import Path
    f=json.loads(Path(__file__).with_name('long_reference_fixture.json').read_text())
    shape=tuple(f['shape']);ref=np.array(f['reference'],np.int32)
    drivable=np.broadcast_to(np.array(True),shape)
    expected,_=original_soft(shape,ref,.05,drivable)
    got,detail=reference_soft(shape,ref,.05,drivable)
    assert detail['r4_edt_domain_preserved']
    for row in range(0,shape[0],96):
        assert np.array_equal(got[row:row+96],expected[row:row+96])
