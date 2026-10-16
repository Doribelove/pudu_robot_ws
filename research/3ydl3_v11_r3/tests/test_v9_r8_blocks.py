from types import SimpleNamespace as S
import numpy as np
import pytest
from three_yd_v5.r4_tiles import blocks
from three_yd_v5.v9_r8_blocks import tile_support
from three_yd_v5.fast_costmap import compose_bounded


@pytest.mark.parametrize('shape,tile', [((1, 1), 256), ((3, 5), 1), ((7, 11), 2),
    ((17, 31), 7), ((256, 256), 256), ((257, 257), 256), ((511, 513), 256), ((513, 769), 256)])
@pytest.mark.parametrize('neighbors', [0, 1, 2])
def test_exact_original_tile_boxes_including_partial_edges_and_neighbor_halo(monkeypatch, shape, tile, neighbors):
    rng = np.random.default_rng(20261009)
    for mask in (np.zeros(shape, bool), np.ones(shape, bool), rng.random(shape) < .03):
        mask[0, -1] = True; mask[-1, 0] = True
        monkeypatch.setenv('THREEYD_V9_R8_MODE', 'control'); old = blocks(mask, tile, neighbors)
        monkeypatch.setenv('THREEYD_V9_R8_MODE', 'R'); new = blocks(mask, tile, neighbors)
        assert old == new


@pytest.mark.parametrize('kind', ['strided', 'transpose', 'float', 'uint8'])
def test_unsupported_layout_or_dtype_keeps_original_reduction(monkeypatch, kind):
    raw = np.arange(77).reshape(7, 11) % 3 == 0
    mask = raw[:, ::2] if kind == 'strided' else raw.T if kind == 'transpose' else raw.astype(np.float32) if kind == 'float' else raw.astype(np.uint8)
    assert tile_support(mask, 3) is None
    monkeypatch.setenv('THREEYD_V9_R8_MODE', 'control'); old = blocks(mask, 3, 1)
    monkeypatch.setenv('THREEYD_V9_R8_MODE', 'R'); assert old == blocks(mask, 3, 1)


@pytest.mark.parametrize('generic', [False, True])
def test_full_original_soft_policy_occupancy_static_and_inflation_bytes_unchanged(monkeypatch, generic):
    monkeypatch.setenv('THREEYD_R11_MODE', 'AB'); monkeypatch.setenv('THREEYD_V8_R4_MODE', 'B')
    monkeypatch.setenv('THREEYD_V9_R1_MODE', 'S')
    shape = (513, 769); raw = np.zeros(shape, np.int8); raw[100:109, 330:390] = 100
    hard = raw != 0; drivable = ~hard; scope = np.zeros(shape, bool); scope[130:400, 50:700] = True
    c = S(world=S(hard=hard, map=S(occupancy=raw, resolution=.05), semantic=S(semantic_map_hash='semantic')),
          reference_cells=np.array([[151, 80], [199, 350], [370, 650]]), global_drivable=drivable,
          plan=S(corridor_mask=drivable), dynamic=None, task={'signature': 'original'}, generic_mode=generic)
    monkeypatch.setenv('THREEYD_V9_R8_MODE', 'OG'); old = compose_bounded(c, scope)
    monkeypatch.setenv('THREEYD_V9_R8_MODE', 'OGR'); new = compose_bounded(c, scope)
    for name in ('internal_cost', 'occupancy_grid', 'expected_master_cost', 'hard_semantic_mask', 'affected_mask'):
        assert np.array_equal(getattr(old, name), getattr(new, name)), name
    assert np.array_equal(np.asarray(old.soft_cost), np.asarray(new.soft_cost))
    assert (old.policy_hash, old.preference_field_hash, old.semantic_map_hash) == (new.policy_hash, new.preference_field_hash, new.semantic_map_hash)
    assert old.diagnostics['r4_soft_tiles'] == new.diagnostics['r4_soft_tiles']
    assert old.diagnostics['soft_computed_cells'] == new.diagnostics['soft_computed_cells']
    assert old.diagnostics['inflation_computed_cells'] == new.diagnostics['inflation_computed_cells']


def test_complete_tile_reshape_is_a_view_without_padding_or_dense_copy():
    mask = np.zeros((6000, 5000), bool); t = 256
    view = mask[:(6000//t)*t, :(5000//t)*t].reshape(6000//t, t, 5000//t, t)
    assert np.shares_memory(mask, view)
