import cv2
import numpy as np
import pytest
from three_yd_v5.v9_storage import static_lut
from three_yd_v5.v9_r9_lookup import exact_lookup, lookup_stats
from arena_evaluation.semantic_costmap_composer import occupancy_to_static_layer


def original(value):
    table = occupancy_to_static_layer(np.arange(256, dtype=np.uint8).view(np.int8))
    return table[value.view(np.uint8)]


@pytest.mark.parametrize('mode', ['L', 'PL'])
def test_all256_signed_source_bytes_exact_independent_and_no_thread_change(monkeypatch, mode):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', mode)
    value = np.arange(256, dtype=np.uint8).view(np.int8).reshape(16, 16)
    before = value.copy()
    threads = cv2.getNumThreads()
    result = static_lut(value)
    assert result.dtype == np.uint8 and result.shape == value.shape
    assert np.array_equal(result, original(value)) and np.array_equal(value, before)
    assert not np.shares_memory(result, value)
    assert cv2.getNumThreads() == threads
    assert lookup_stats()['mode_enabled'] and not lookup_stats()['input_content_cached']


def test_mutable_source_and_alias_are_observed_on_every_call(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'L')
    value = np.zeros((4, 5), np.int8)
    first = static_lut(value)
    alias = value.view()
    alias[2, 3] = -1
    second = static_lut(value)
    assert np.array_equal(second, original(value)) and first[2, 3] != second[2, 3]
    second[0, 0] = 33
    assert np.array_equal(static_lut(value), original(value))


@pytest.mark.parametrize('shape', [(1, 1), (1, 17), (19, 1), (35, 27)])
def test_non_square_and_one_cell_shapes_preserve_original_bytes(monkeypatch, shape):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'L')
    value = np.resize(np.arange(256, dtype=np.uint8).view(np.int8), shape)
    assert np.array_equal(static_lut(value), original(value))


@pytest.mark.parametrize('layout', ['transpose', 'negative', 'stride', 'Fortran', 'empty', 'one_dimension', 'scalar', 'wide_dtype', 'subclass'])
def test_unknown_or_empty_layouts_take_original_fallback(monkeypatch, layout):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'L')
    value = np.arange(256, dtype=np.uint8).view(np.int8).reshape(16, 16)
    if layout == 'transpose': value = value.T
    elif layout == 'negative': value = value[::-1]
    elif layout == 'stride': value = value[:, ::2]
    elif layout == 'Fortran': value = np.asfortranarray(value)
    elif layout == 'empty': value = np.empty((0, 3), np.int8)
    elif layout == 'one_dimension': value = value.ravel()
    elif layout == 'scalar': value = np.array(5, np.int8)
    elif layout == 'wide_dtype': value = value.astype(np.int16)
    elif layout == 'subclass':
        class Custom(np.ndarray): pass
        value = value.view(Custom)
    table = occupancy_to_static_layer(np.arange(256, dtype=np.uint8).view(np.int8))
    before = lookup_stats()['original_fallback_calls']
    assert exact_lookup(value, table) is None
    assert lookup_stats()['original_fallback_calls'] == before + 1
    assert np.array_equal(static_lut(value), original(value))


def test_readonly_input_is_read_fresh_and_remains_readonly(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'L')
    value = np.frombuffer(bytes(range(256)), np.int8).reshape(16, 16)
    result = static_lut(value)
    assert np.array_equal(result, original(value)) and not value.flags.writeable
    assert result.flags.writeable


@pytest.mark.parametrize('mode', ['control', 'P'])
def test_disabled_L_uses_parent_lookup(monkeypatch, mode):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', mode)
    value = np.array([[-1, 0, 1, 80, 100, 127, -128]], np.int8)
    before = lookup_stats()['native_LUT_calls']
    assert np.array_equal(static_lut(value), original(value))
    assert lookup_stats()['native_LUT_calls'] == before
