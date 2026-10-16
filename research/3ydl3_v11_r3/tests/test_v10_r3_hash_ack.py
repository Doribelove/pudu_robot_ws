from types import SimpleNamespace
import numpy as np
import pytest
from three_yd_v5.first_ack import ComparisonPlan
from three_yd_v5.request_cost import request_digest
from three_yd_v5.v9_maps import seal
from arena_3d_v1.semantic_world import digest


@pytest.mark.parametrize('seed',range(12))
@pytest.mark.parametrize('previous_kind',['none','equal','random'])
@pytest.mark.parametrize('reverse_columns',[False,True])
def test_native_classification_and_each_observation_are_exact(monkeypatch,seed,previous_kind,reverse_columns):
    rng=np.random.default_rng(seed);shape=(19+seed*9,33+seed*7)
    master=rng.integers(0,256,shape,dtype=np.uint8)
    hard=rng.random(shape)>.6;soft=rng.choice([0.,-1.,20.,140.,np.nan],shape)
    columns=slice(None,None,-1) if reverse_columns else slice(None)
    semantic=SimpleNamespace(expected_master_cost=master[:,columns],hard_semantic_mask=hard[:,columns],soft_cost=soft[:,columns])
    expected=rng.integers(-1,101,shape,dtype=np.int8)[:,columns]
    changed=(rng.random(shape)>.7)[:,columns]
    previous={'none':None,'equal':np.flipud(semantic.expected_master_cost),
              'random':rng.integers(0,256,shape,dtype=np.uint8)}[previous_kind]
    monkeypatch.setenv('THREEYD_V10_R3_MODE','control')
    old=ComparisonPlan(semantic,expected,changed,previous,rows=17)
    monkeypatch.setenv('THREEYD_V10_R3_MODE','K')
    new=ComparisonPlan(semantic,expected,changed,previous,rows=17)
    assert np.array_equal(old.flags,new.flags)
    for name in ('counts','dirty_bbox','expected_affected_hash','expected_full_hash'):
        assert getattr(old,name)==getattr(new,name)
    for server in [new.master.copy(),new.master[:,::-1].copy()[:,::-1],
                   rng.integers(0,256,shape,dtype=np.uint8)]:
        monkeypatch.setenv('THREEYD_V10_R3_MODE','control');a=old.compare(server)
        monkeypatch.setenv('THREEYD_V10_R3_MODE','K');assert new.compare(server)==a
    server=new.master.copy();assert new.compare(server)['mismatch']==0
    server[-1,-1]^=1
    monkeypatch.setenv('THREEYD_V10_R3_MODE','control');a=old.compare(server)
    monkeypatch.setenv('THREEYD_V10_R3_MODE','K');assert new.compare(server)==a


@pytest.mark.parametrize('dtype',[np.bool_,np.uint8,np.int8,np.int16,np.float64])
def test_request_hash_retains_current_bytes_and_alias_semantics(monkeypatch,dtype):
    monkeypatch.setenv('THREEYD_V10_R3_MODE','H');monkeypatch.setenv('THREEYD_V9_MODE','A')
    mutable=np.arange(64).reshape(8,8).astype(dtype)
    for value in (mutable,mutable.T,mutable[::-1],seal(mutable),np.flipud(seal(mutable))):
        assert request_digest(value)==digest(value)
    before=request_digest(mutable)
    mutable.view()[0,0]=True if dtype==np.bool_ else 21
    assert request_digest(mutable)==digest(mutable) and request_digest(mutable)!=before
    alias=mutable.view();alias.flags.writeable=False
    previous=request_digest(alias);mutable[0,0]=False if dtype==np.bool_ else 22
    assert request_digest(alias)==digest(alias) and request_digest(alias)!=previous
