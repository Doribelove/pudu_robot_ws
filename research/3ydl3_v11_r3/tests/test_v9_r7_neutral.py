from types import SimpleNamespace as S
import numpy as np
import pytest
from three_yd_v5.free_space_topology import MapOnlyWorld
from three_yd_v5.v9_r7_neutral import neutral_labels
from three_yd_v5.v9_maps import immutable,prepare_world
from three_yd_v5.v9_r7_options import enabled
from storage_ledger import inventory


def world(labels=None):
    w=MapOnlyWorld.__new__(MapOnlyWorld)
    w.map=S(height=130,width=90,occupancy=np.zeros((130,90),np.int8))
    w.hard=np.zeros((130,90),bool);w.v7_map_only=True;w.features={};w.rules={};w._labels=labels
    return w


@pytest.mark.parametrize('evaluated',[False,True])
def test_exact_full_shape_values_all_indexing_and_immutable(evaluated):
    old=np.zeros((130,90),np.int16);w=world(old if evaluated else None)
    value,log=neutral_labels(w)
    assert log['enabled'] and value.shape==old.shape and value.dtype==old.dtype
    assert np.array_equal(value,old) and immutable(value)
    assert value.strides==(0,0) and value[129,89]==0 and value[-1,-1]==0
    for k in [(slice(4,55),slice(7,90)),([0,129,65],[89,0,22]),(old==0,)]:
        assert np.array_equal(value[k],old[k])
    with pytest.raises(ValueError):value[1,1]=3
    with pytest.raises(ValueError):value.flags.writeable=True
    for k in [(130,0),(0,90)]:
        with pytest.raises(IndexError):value[k]
    ledger=inventory({'labels':value})
    assert ledger['unique_array_buffer_bytes']==2
    old[2,3]=8;assert value[2,3]==0


@pytest.mark.parametrize('kind',['semantic','nonzero','dtype','shape','subclass','flag','hard_shape','map_shape'])
def test_guard_rejects_any_incompatible_world(kind):
    w=world(np.zeros((130,90),np.int16))
    if kind=='semantic':w.features={'new':S(semantic_class='lane')}
    elif kind=='nonzero':w._labels[-1,-1]=1
    elif kind=='dtype':w._labels=w._labels.astype(np.int32)
    elif kind=='shape':w._labels=w._labels[:120]
    elif kind=='subclass':
        class Sub(MapOnlyWorld):pass
        v=Sub.__new__(Sub);v.__dict__.update(w.__dict__);w=v
    elif kind=='flag':w.v7_map_only=False
    elif kind=='hard_shape':w.hard=w.hard[:120]
    elif kind=='map_shape':w.map.height=0
    result,log=neutral_labels(w)
    assert result is None and not log['enabled'] and log['fallback_original_storage']


def test_rules_and_property_changes_are_not_treated_as_neutral(monkeypatch):
    w=world();w.rules={'direction':1};assert neutral_labels(w)[0] is None
    w.rules={};monkeypatch.setattr(MapOnlyWorld,'labels',property(lambda self:np.ones((130,90),np.int16)))
    result,log=neutral_labels(w)
    assert result is None and log['rejection']=='ORIGINAL_NEUTRAL_LABEL_PROPERTY_CHANGED'
    monkeypatch.setenv('THREEYD_V9_MODE','A');monkeypatch.setenv('THREEYD_V9_R7_MODE','N')
    prepare_world(w);assert np.all(w.labels==1) and np.all(w._labels==1) and immutable(w._labels)
    # A changed property may create a fresh writable result on every access.
    # The fallback preserves the original prepare_world assignment behavior.


@pytest.mark.parametrize('candidate',[False,True])
def test_prepare_keeps_original_values_and_control_storage(monkeypatch,candidate):
    monkeypatch.setenv('THREEYD_V9_MODE','A');monkeypatch.setenv('THREEYD_V9_R7_MODE','N' if candidate else 'control')
    w=world(np.zeros((130,90),np.int16));old=w.labels.copy();hard=w.hard.copy();occ=w.map.occupancy.copy()
    prepare_world(w)
    assert np.array_equal(w.labels,old) and np.array_equal(w.hard,hard) and np.array_equal(w.map.occupancy,occ)
    assert immutable(w.labels) and immutable(w.hard) and immutable(w.map.occupancy)
    assert (w.labels.strides==(0,0))==candidate
    assert inventory({'labels':w.labels})['unique_array_buffer_bytes']==(2 if candidate else old.nbytes)


def test_nonzero_maponly_falls_back_and_preserves_value(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_MODE','A');monkeypatch.setenv('THREEYD_V9_R7_MODE','N')
    old=np.zeros((130,90),np.int16);old[129,89]=42;w=world(old)
    prepare_world(w);assert np.array_equal(w.labels,old) and immutable(w.labels)
    assert not w.v9_r7_neutral_diagnostics['enabled']


def test_control_default_and_invalid_mode(monkeypatch):
    monkeypatch.delenv('THREEYD_V9_R7_MODE',raising=False);assert not enabled()
    monkeypatch.setenv('THREEYD_V9_R7_MODE','bad')
    with pytest.raises(ValueError):enabled()
