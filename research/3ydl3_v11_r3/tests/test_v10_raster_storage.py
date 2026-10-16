import inspect
from types import SimpleNamespace
import numpy as np
import pytest
from three_yd_v5 import v10_raster_storage as r

@pytest.mark.parametrize('kind',['polygon','line','point','unknown'])
def test_binary_view_preserves_raster_values_and_boolean_writes(kind):
    rng=np.random.default_rng(123)
    s=SimpleNamespace(height=29,width=37,resolution=.05,origin=(-.1,.2,0))
    for n in (0,1,2,3,8):
        for _ in range(8):
            coords=(rng.random((n,2))*[1.7,1.3]+[0.,.2]).tolist()
            f=SimpleNamespace(geometry_type=kind,coordinates=coords)
            expected=r.ORIGINAL(s,f);got=r.binary_view(s,f)
            assert got.dtype==np.bool_ and np.array_equal(got,expected)
            assert isinstance(got.base,np.ndarray) and got.base.dtype==np.uint8
            got.flat[0]=not got.flat[0]
            assert got.base.flat[0] in (0,1)

def test_scoped_hooks_restore_on_exception_and_nested_scope():
    from arena_3d_v1 import semantic_world
    from three_yd_v5 import topology
    before=[m.rasterize_feature for m in (r.raster,semantic_world,topology)]
    with pytest.raises(RuntimeError):
        with r.construction_storage() as outer:
            assert outer['enabled']
            active=topology.rasterize_feature
            with r.construction_storage() as inner:
                assert not inner['enabled']
                assert topology.rasterize_feature is active
            raise RuntimeError('construction failure')
    assert [m.rasterize_feature for m in (r.raster,semantic_world,topology)]==before

def test_unrecognized_original_or_disabled_uses_parent(monkeypatch):
    monkeypatch.setattr(r,'PINNED_SHA256','invalid')
    with r.construction_storage() as info:assert not info['enabled']
    monkeypatch.setenv('THREEYD_V10_RASTER_STORAGE','0')
    with r.construction_storage() as info:assert not info['enabled']
