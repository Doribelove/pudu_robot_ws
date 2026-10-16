from dataclasses import replace
import numpy as np
import pytest
from arena_evaluation.semantic_costmap_composer import SemanticCostmap, internal_soft_to_occupancy
from three_yd_v5.r4_tiles import TiledSoft
from three_yd_v5.v9_storage import occupancy_on_free_support
from three_yd_v5.v9_maps import immutable, content_digest, seal_grid
from three_yd_v5.v9_r8_fields import sparse_occupancy, freeze_array, freeze_auxiliary
from three_yd_v5.v9_r8_options import enabled


@pytest.mark.parametrize('cap', [-10., 0., .01, 1., 140., 200., 252., 400.])
@pytest.mark.parametrize('strided', [False, True])
def test_sparse_conversion_matches_frozen_full_conversion_all_cells(cap, strided):
    rng = np.random.default_rng(20261009); shape = (157, 211)
    source = rng.integers(-128, 128, shape, dtype=np.int8)
    hard = rng.random(shape) < .47; drivable = rng.random(shape) < .71
    if strided:
        source = source[:, ::-1]; hard = hard[::-1]; drivable = drivable[:, ::-1]
    pieces = [((3, 82, 7, 127), rng.uniform(-10, 270, (79, 120)).astype(np.float32)),
              ((61, 157, 53, 211), rng.uniform(-10, 270, (96, 158)).astype(np.float32))]
    soft = TiledSoft(shape, cap, drivable, pieces)
    old = np.maximum(source.astype(np.int16), internal_soft_to_occupancy(np.asarray(soft))).astype(np.int8)
    old[source < 0] = -1; old[hard] = 100
    actual, info = sparse_occupancy(source, soft, hard, rows=13)
    assert info['r8_sparse_occupancy_used'] and actual.dtype == np.int8
    assert np.array_equal(old, actual)
    assert np.array_equal(occupancy_on_free_support(source, soft, hard, rows=19), actual)


def test_original_quantization_boundaries_and_float32_background_rounding():
    boundary = (np.arange(1, 100)*252/99).astype(np.float32)
    values = np.stack([np.nextafter(boundary, -np.inf), boundary,
                       np.nextafter(boundary, np.inf)]).astype(np.float32)
    shape = (5, 99); raw = np.zeros(shape, np.int8); hard = np.zeros(shape, bool)
    for cap in (float(np.nextafter(np.float32(140), np.inf)), 140.+1e-7):
        soft = TiledSoft(shape, cap, np.ones(shape, bool), [((1, 4, 0, 99), values)])
        actual, info = sparse_occupancy(raw, soft, hard)
        assert info['r8_sparse_occupancy_used']
        assert np.array_equal(actual, internal_soft_to_occupancy(np.asarray(soft)).astype(np.int8))


@pytest.mark.parametrize('kind', ['dense', 'subclass', 'nonfloat_tile', 'nonfinite_cap'])
def test_unsupported_sparse_forms_use_original_conversion(kind):
    shape = (31, 43); raw = np.zeros(shape, np.int8); hard = np.zeros(shape, bool)
    class OtherSoft(TiledSoft): pass
    if kind == 'dense':
        soft = np.ones(shape, np.float32)*17
    else:
        cls = OtherSoft if kind == 'subclass' else TiledSoft
        tile = np.ones((10, 11), np.float64 if kind == 'nonfloat_tile' else np.float32)*17
        soft = cls(shape, np.inf if kind == 'nonfinite_cap' else 140., np.ones(shape, bool),
                   [((2, 12, 4, 15), tile)])
    actual, info = sparse_occupancy(raw, soft, hard)
    assert not info['r8_sparse_occupancy_used']
    assert np.array_equal(actual, occupancy_on_free_support(raw, soft, hard))


def make_grid(tiled=True):
    shape = (43, 67); raw = np.arange(np.prod(shape), dtype=np.int8).reshape(shape)
    master = raw.view(np.uint8).copy(); internal = master.copy()
    hard = master > 200; affected = np.broadcast_to(np.array(True), shape)
    soft = TiledSoft(shape, 140., ~hard, [((3, 9, 4, 18), np.ones((6, 14), np.float32))]) if tiled else master.astype(np.float32)
    return SemanticCostmap(internal, raw, master, soft, hard, affected, 'p', 's', 'f', True, {})


@pytest.mark.parametrize('tiled', [False, True])
def test_auxiliary_storage_has_identical_values_and_genuine_immutable_owners(tiled):
    grid = make_grid(tiled); saved = np.asarray(grid.soft_cost).copy(); new = freeze_auxiliary(grid)
    for name in ('internal_cost', 'hard_semantic_mask', 'affected_mask'):
        before, after = getattr(grid, name), getattr(new, name)
        assert before.shape == after.shape and before.dtype == after.dtype
        assert np.array_equal(before, after) and immutable(after)
        with pytest.raises(ValueError): after.flags.writeable = True
    assert np.array_equal(np.asarray(new.soft_cost), saved)
    if tiled:
        assert type(new.soft_cost) is TiledSoft
        assert immutable(new.soft_cost.drivable) and all(immutable(v) for _, v in new.soft_cost.pieces)
        grid.soft_cost.pieces[0][1][:] = 77
    else:
        assert immutable(new.soft_cost); grid.soft_cost[:] = 77
    grid.internal_cost[:] = 23; grid.hard_semantic_mask[:] = False
    assert np.array_equal(np.asarray(new.soft_cost), saved)
    assert new.policy_hash == grid.policy_hash and new.semantic_map_hash == grid.semantic_map_hash
    # A full logical affected map still owns one byte, with exact bool values.
    base = new.affected_mask
    while isinstance(base, np.ndarray): base = base.base
    assert isinstance(base, bytes) and len(base) == 1 and new.affected_mask.all()


def test_readonly_flag_with_mutable_alias_is_not_trusted():
    raw = np.arange(200, dtype=np.uint8).reshape(10, 20); alias = raw.view()
    raw.flags.writeable = False; sealed = freeze_array(raw)
    assert immutable(sealed)
    alias[:] = 99
    assert not np.array_equal(sealed, raw)


@pytest.mark.parametrize('r8,v9,expected', [('control', 'A', False), ('F', 'control', False), ('F', 'A', True), ('OF', 'A', True)])
def test_auxiliary_sealing_only_enabled_for_explicit_candidate_and_v9_a(monkeypatch, r8, v9, expected):
    monkeypatch.setenv('THREEYD_V9_R8_MODE', r8); monkeypatch.setenv('THREEYD_V9_MODE', v9)
    monkeypatch.setenv('THREEYD_V9_R1_MODE', 'S'); grid = make_grid()
    new = seal_grid(grid)
    assert bool(new.diagnostics.get('r8_immutable_auxiliary_fields')) == expected
    assert immutable(new.internal_cost) == expected


def test_mutable_fields_still_rehash_and_replaced_frozen_fields_change_binding(monkeypatch):
    from three_yd_v5.v9_r7_batch import field_binding
    monkeypatch.setenv('THREEYD_V9_MODE', 'A')
    raw = np.zeros((11, 13), np.uint8); before = content_digest(raw)
    raw[1, 2] = 3; assert content_digest(raw) != before
    grid = freeze_auxiliary(make_grid()); before = field_binding(grid.soft_cost)
    grid.soft_cost.pieces[0] = (grid.soft_cost.pieces[0][0], np.zeros((6, 14), np.float32))
    assert field_binding(grid.soft_cost) != before


def test_unknown_auxiliary_form_keeps_original_grid():
    grid = make_grid(); unsupported = replace(grid, internal_cost=object())
    assert freeze_auxiliary(unsupported) is unsupported
    unsupported = replace(grid, soft_cost=object())
    assert freeze_auxiliary(unsupported) is unsupported


def test_invalid_candidate_mode_rejected(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R8_MODE', 'unknown')
    with pytest.raises(ValueError, match='invalid V9-r8'): enabled('O')
