from types import SimpleNamespace
import math
import numpy as np
import pytest
from scipy import ndimage
from three_yd_v5 import v10_firstready as r

@pytest.mark.parametrize('kind',['polygon','line','point','unknown'])
def test_feature_indices_preserve_pixel_assignment_and_hard_union(kind):
    rng=np.random.default_rng(722)
    world=SimpleNamespace(height=300,width=360,resolution=.05,origin=(-2.,1.,0))
    for count in (0,1,2,3,5,10):
        for center in ((-3.,0.),(2.,5.),(15.,18.)):
            for _ in range(5):
                feature=SimpleNamespace(geometry_type=kind,coordinates=(rng.random((count,2))*.9+center).tolist())
                expected=r.ORIGINAL(world,feature);index=r.feature_index(world,feature)
                labels=np.zeros(expected.shape,np.int16);labels[index]=7
                assert np.array_equal(labels,expected.astype(np.int16)*7)
                base=rng.random(expected.shape)<.03;hard=base.copy();hard|=index
                assert np.array_equal(hard,base|expected)
                assert np.array_equal(base|index,base|expected)

def test_large_feature_keeps_dense_mask_and_overlapping_assignment_order():
    s=SimpleNamespace(height=100,width=130,resolution=.1,origin=(0.,0.,0))
    features=[SimpleNamespace(geometry_type='polygon',coordinates=p) for p in (
        [[-1,-1],[20,0],[20,20],[0,20]],[[4,4],[4.7,4],[4.7,4.8],[4,4.8]],[[4.5,4],[4.9,4],[4.9,5],[4.5,5]])]
    assert isinstance(r.feature_index(s,features[0]),np.ndarray)
    expected=np.zeros((100,130),np.int16);actual=expected.copy()
    for i,f in enumerate(features,1):expected[r.ORIGINAL(s,f)]=i;actual[r.feature_index(s,f)]=i
    assert np.array_equal(expected,actual)

@pytest.mark.parametrize('shape',[(1,1),(17,23),(281,309)])
def test_exact_safe_predicate_matches_original_edt_including_row_seams_and_ties(shape):
    rng=np.random.default_rng(64)
    cases=[np.ones(shape,bool),np.zeros(shape,bool),rng.random(shape)>.04]
    corner=np.ones(shape,bool);corner[-1,-1]=False;cases.append(corner)
    for free in cases:
        for resolution in (.05,.15):
            for cutoff in (.42,math.hypot(.265,.225)+.05+math.sqrt(2)*resolution/2,2*resolution,math.sqrt(2)*resolution):
                info={};got=r.threshold_safe(free,resolution,cutoff,info)
                assert np.array_equal(got,ndimage.distance_transform_edt(free)*resolution>=cutoff)
                if free.all():assert not info['direct']

def test_firstready_restores_all_aliases_after_nested_scope_and_exception():
    from arena_3d_v1 import semantic_world
    from three_yd_v5 import topology
    original=(semantic_world.rasterize_feature,topology.rasterize_feature,semantic_world.ndimage)
    with pytest.raises(RuntimeError):
        with r.firstready_construction() as outer:
            assert outer['enabled'];active=topology.rasterize_feature
            with r.firstready_construction() as inner:
                assert not inner['enabled'] and topology.rasterize_feature is active
            raise RuntimeError('failed map load')
    assert (semantic_world.rasterize_feature,topology.rasterize_feature,semantic_world.ndimage)==original

def test_unknown_constructor_uses_parent_storage_without_threshold_hook(monkeypatch):
    from arena_3d_v1 import semantic_world
    monkeypatch.setattr(r,'WORLD_INIT_SHA256','unknown')
    with r.firstready_construction() as info:
        assert not info['enabled'] and info['parent']['enabled']
        assert semantic_world.ndimage is ndimage

def test_disable_switch_uses_parent_storage(monkeypatch):
    monkeypatch.setenv('THREEYD_V10_FIRSTREADY','0')
    with r.firstready_construction() as info:assert not info['enabled'] and info['parent']['enabled']
