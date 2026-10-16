from types import SimpleNamespace as S
import numpy as np
import pytest
from three_yd_v5.v9_r8_fields import freeze_owned_scope
from three_yd_v5.v9_maps import immutable, content_digest, seal
from three_yd_v5.controller import V5Controller


@pytest.mark.parametrize('mode', ['control', 'O', 'F', 'OF', 'G', 'OG', 'FG', 'OFG'])
def test_only_explicit_g_seals_owned_scope_with_all_values_identical(monkeypatch, mode):
    monkeypatch.setenv('THREEYD_V9_R8_MODE', mode); monkeypatch.setenv('THREEYD_V9_MODE', 'A')
    raw = np.random.default_rng(8).random((71, 97)) < .17
    sealed = freeze_owned_scope(raw)
    assert immutable(sealed) == ('G' in mode)
    assert np.array_equal(raw, sealed) and raw.shape == sealed.shape and raw.dtype == sealed.dtype
    assert content_digest(raw) == content_digest(sealed)
    if 'G' in mode:
        with pytest.raises(ValueError): sealed.flags.writeable = True


def test_mutable_alias_views_and_unknown_fields_remain_live(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R8_MODE', 'G'); monkeypatch.setenv('THREEYD_V9_MODE', 'A')
    raw = np.ones((29, 37), bool); alias = raw.view()
    assert freeze_owned_scope(alias) is alias
    before = content_digest(alias); raw[3, 4] = False
    assert content_digest(alias) != before
    unknown = raw.astype(np.uint8); assert freeze_owned_scope(unknown) is unknown
    transposed = raw.T
    assert freeze_owned_scope(transposed) is transposed and not immutable(transposed)


def test_g_does_not_activate_without_v9_a(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R8_MODE', 'G'); monkeypatch.setenv('THREEYD_V9_MODE', 'control')
    raw = np.ones((29, 37), bool); assert freeze_owned_scope(raw) is raw


def test_dynamic_source_mutation_and_scope_replacement_invalidate_original_target_cache(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R8_MODE', 'G'); monkeypatch.setenv('THREEYD_V9_MODE', 'A')
    c = object.__new__(V5Controller); shape = (29, 37)
    c.plan = S(corridor_mask=seal(np.ones(shape, bool)))
    c._search_mask = freeze_owned_scope(np.ones(shape, bool)); c.r13_remainder_mask = None
    c.dynamic = S(mask=np.zeros(shape, bool)); c._v9_target = None
    first = c._target_mask(); assert c._target_mask() is first
    c.dynamic.mask[3, 4] = True
    changed = c._target_mask(); assert changed is not first and not changed[3, 4]
    assert np.array_equal(changed, c.plan.corridor_mask & c._search_mask & ~c.dynamic.mask)
    replacement = np.ones(shape, bool); replacement[7, 8] = False
    c._search_mask = freeze_owned_scope(replacement)
    final = c._target_mask(); assert final is not changed and not final[7, 8] and not final[3, 4]
    c.r13_remainder_mask = freeze_owned_scope(np.zeros(shape, bool))
    assert not c._target_mask().any()
