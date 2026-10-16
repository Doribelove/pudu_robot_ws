"""Compare L2 scope data with the original bottom-aligned aggregation oracle."""
import numpy as np
import pytest
from arena_3d_v1.multires_grid import aggregate as original
from three_yd_v5 import v9_r9_scope as module


def exact(mask, factor=3):
    before = np.asarray(mask).copy()
    want = original(mask, factor)
    got = module.aggregate_scope(mask, factor)
    assert type(got) is np.ndarray and got.dtype == want.dtype
    assert got.shape == want.shape and got.flags.c_contiguous
    assert np.array_equal(got, want)
    if not want.dtype.hasobject:
        assert got.tobytes() == want.tobytes()
    assert not np.shares_memory(got, np.asarray(mask))
    assert np.array_equal(np.asarray(mask), before, equal_nan=np.asarray(mask).dtype.kind == 'f')
    assert np.asarray(mask).tobytes() == before.tobytes()
    return got


@pytest.mark.parametrize('mode', ['X', 'PLMX', 'PLMUX'])
def test_every_nine_cell_boolean_pattern_matches_original(monkeypatch, mode):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', mode)
    before = module.scope_stats()
    for bits in range(512):
        mask = np.array([(bits >> i) & 1 for i in range(9)], bool).reshape(3, 3)
        exact(mask)
    assert module.scope_stats()['exact_scope_calls'] == before['exact_scope_calls']+512


@pytest.mark.parametrize('shape', [(h, w) for h in (1, 2, 4, 7) for w in (1, 3, 5, 8)])
def test_partial_blocks_keep_physical_bottom_origin_and_false_padding(monkeypatch, shape):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'X')
    rng = np.random.default_rng(20261010)
    exact(rng.integers(0, 2, shape, dtype=np.uint8).view(bool))
    mask = np.zeros(shape, bool)
    mask[-1, 0] = True
    got = exact(mask)
    assert got[-1, 0] and got.sum() == 1
    mask[:] = False
    mask[0, -1] = True
    got = exact(mask)
    assert got[0, -1] and got.sum() == 1


def test_current_alias_changes_and_mutable_output_are_independent(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'PLMX')
    mask = np.zeros((7, 8), bool)
    first = exact(mask)
    mask.view()[-1, 0] = True
    second = exact(mask)
    assert not first.any() and second[-1, 0]
    second[:] = False
    assert mask[-1, 0] and exact(mask)[-1, 0]
    stats = module.scope_stats()
    assert not stats['current_input_or_scope_result_cached'] and not stats['audit_or_ACK_results_cached']


def test_readonly_source_and_noncanonical_true_bytes_match_original(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'X')
    raw = np.arange(56, dtype=np.uint8).reshape(7, 8)
    mask = raw.view(bool)
    exact(mask)
    readonly = np.frombuffer(raw.tobytes(), dtype=bool).reshape(7, 8)
    result = exact(readonly)
    assert not readonly.flags.writeable and result.flags.writeable


@pytest.mark.parametrize('kind', ['transpose', 'Fortran', 'stride', 'negative', 'empty_height',
    'empty_width', 'uint8', 'float', 'object', 'list', 'subclass', 'one_dimension', 'scalar'])
def test_unknown_inputs_delegate_original_values_or_errors(monkeypatch, kind):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'X')
    mask = np.arange(20).reshape(4, 5) > 5
    if kind == 'transpose':mask = mask.T
    elif kind == 'Fortran':mask = np.asfortranarray(mask)
    elif kind == 'stride':mask = mask[:, ::2]
    elif kind == 'negative':mask = mask[::-1]
    elif kind == 'empty_height':mask = np.empty((0, 5), bool)
    elif kind == 'empty_width':mask = np.empty((4, 0), bool)
    elif kind == 'uint8':mask = mask.astype(np.uint8)
    elif kind == 'float':
        mask = mask.astype(float);mask[1, 1] = np.nan
    elif kind == 'object':mask = mask.astype(object)
    elif kind == 'list':mask = mask.tolist()
    elif kind == 'subclass':
        class Custom(np.ndarray):pass
        mask = mask.view(Custom)
    elif kind == 'one_dimension':mask = mask.ravel()
    elif kind == 'scalar':mask = np.array(True)
    before = module.scope_stats()['original_fallback_calls']
    try:
        original(mask, 3)
    except Exception as error:
        with pytest.raises(type(error)) as observed:module.aggregate_scope(mask, 3)
        assert str(observed.value) == str(error)
    else:exact(mask)
    assert module.scope_stats()['original_fallback_calls'] == before+1


@pytest.mark.parametrize('factor', [1, 2, 4, np.int64(3), 0, -1, 3.0])
def test_unknown_factors_preserve_original_behavior(monkeypatch, factor):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'X')
    mask = np.zeros((7, 8), bool)
    before = module.scope_stats()['original_fallback_calls']
    try:original(mask, factor)
    except Exception as error:
        with pytest.raises(type(error)) as observed:module.aggregate_scope(mask, factor)
        assert str(observed.value) == str(error)
    else:exact(mask, factor)
    assert module.scope_stats()['original_fallback_calls'] == before+1


@pytest.mark.parametrize('mode', ['control', 'P', 'L', 'PLM', 'PLMU'])
def test_disabled_modes_forward_original_call(monkeypatch, mode):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', mode)
    mask = np.ones((7, 8), bool)
    calls = []
    def delegated(value, factor):
        calls.append((value, factor))
        return original(value, factor)
    monkeypatch.setattr(module, '_original', delegated)
    before = module.scope_stats()
    exact(mask)
    after = module.scope_stats()
    assert len(calls) == 1 and calls[0][0] is mask and calls[0][1] == 3
    assert after['exact_scope_calls'] == before['exact_scope_calls']
    assert after['original_fallback_calls'] == before['original_fallback_calls']
    assert after['disabled_calls'] == before['disabled_calls']+1


def test_supported_path_avoids_original_padding_and_reports_logical_bytes(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'X')
    mask = np.ones((7, 8), bool)
    want = original(mask, 3)
    def forbidden(*args):raise AssertionError('Supported scope should avoid original full padding')
    monkeypatch.setattr(module, '_original', forbidden)
    before = module.scope_stats()
    assert np.array_equal(module.aggregate_scope(mask, 3), want)
    after = module.scope_stats()
    assert after['fine_cells_processed']-before['fine_cells_processed'] == 56
    assert after['logical_padded_bool_buffer_bytes_avoided']-before['logical_padded_bool_buffer_bytes_avoided'] == 81
    assert after['logical_padding_byte_counter_is_not_RSS']
