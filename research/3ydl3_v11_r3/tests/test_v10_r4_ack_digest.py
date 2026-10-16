from types import SimpleNamespace
import numpy as np
import pytest
from three_yd_v5.first_ack import ComparisonPlan
from three_yd_v5.v9_maps import seal
from three_yd_v5.r6_ack import grid_digest


@pytest.mark.parametrize('immutable',[False,True])
@pytest.mark.parametrize('reverse',[False,True])
def test_expected_hash_reuse_preserves_actual_readback_checks(monkeypatch,immutable,reverse):
    monkeypatch.setenv('THREEYD_V10_R3_MODE','ABRTHI')
    monkeypatch.setenv('THREEYD_V9_MODE','A')
    rng=np.random.default_rng(128);master=rng.integers(0,256,(83,91),dtype=np.uint8)
    if immutable:master=seal(master)
    if reverse:master=master[:,::-1]
    semantic=SimpleNamespace(expected_master_cost=master,hard_semantic_mask=master>180,
                             soft_cost=master.astype(float))
    expected=np.zeros(master.shape,np.int8);changed=np.ones(master.shape,bool)
    monkeypatch.setenv('THREEYD_V10_R4_MODE','control')
    old=ComparisonPlan(semantic,expected,changed,None)
    monkeypatch.setenv('THREEYD_V10_R4_MODE','D')
    new=ComparisonPlan(semantic,expected,changed,None)
    assert new.expected_full_hash==old.expected_full_hash==grid_digest(np.flipud(master))
    actual=np.flipud(master).copy()
    assert new.compare(actual)==old.compare(actual)
    actual[2,7]^=1
    assert new.compare(actual)==old.compare(actual)
    assert new.compare(actual)['mismatch']==1
    if not immutable:
        old_hash=new.expected_full_hash;master[2,7]^=1
        fresh=ComparisonPlan(semantic,expected,changed,None)
        assert fresh.expected_full_hash!=old_hash
        assert fresh.expected_full_hash==grid_digest(np.flipud(master))


@pytest.mark.parametrize('kind',['immutable_view','mutable_view','readonly_alias','copy','different_slice'])
def test_publication_digest_uses_exact_current_expected_cells(monkeypatch,kind):
    from three_yd_v5.tail_digest import publication_source_digest
    monkeypatch.setenv('THREEYD_V10_R4_MODE','D');monkeypatch.setenv('THREEYD_V9_MODE','A')
    source=np.arange(80,dtype=np.int8).reshape(8,10)
    if kind in ('immutable_view','copy','different_slice'):source=seal(source)
    expected=np.flipud(source)
    if kind=='readonly_alias':expected=expected.view();expected.flags.writeable=False
    if kind=='copy':expected=expected.copy();expected[0,0]=3
    if kind=='different_slice':expected=expected[:,::-1]
    assert publication_source_digest(source,expected)==grid_digest(expected)
    if kind in ('mutable_view','readonly_alias'):
        before=publication_source_digest(source,expected);source[0,0]=99
        assert publication_source_digest(source,expected)==grid_digest(expected)
        assert publication_source_digest(source,expected)!=before
