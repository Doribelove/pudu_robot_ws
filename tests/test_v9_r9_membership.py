import cv2
import numpy as np
import pytest
from three_yd_v5.v9_r9_membership import semantic_membership, membership_stats


@pytest.mark.parametrize('mode', ['M', 'PM', 'LM', 'PLM'])
@pytest.mark.parametrize('dtype', [np.int8, np.uint8, np.int16, np.uint16,
                                  np.int32, np.uint32, np.int64, np.uint64])
def test_every_represented_label_and_bool_bytes_exact(monkeypatch, mode, dtype):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', mode)
    raw = np.arange(256, dtype=np.uint8)
    labels = (raw.view(np.int8) if dtype == np.int8 else raw.astype(dtype)).reshape(16, 16)
    ids = labels.ravel()[::3].astype(np.int64)
    original = labels.copy()
    threads = cv2.getNumThreads()
    before = membership_stats()['native_membership_calls']
    actual = semantic_membership(labels, ids)
    assert actual.shape == labels.shape and actual.dtype == np.bool_
    assert np.array_equal(actual, np.isin(labels, ids))
    assert np.array_equal(actual.view(np.uint8), np.isin(labels, ids).view(np.uint8))
    assert np.array_equal(labels, original) and not np.shares_memory(labels, actual)
    assert membership_stats()['native_membership_calls'] == before + 1
    assert cv2.getNumThreads() == threads


@pytest.mark.parametrize('ids', [[], (), [0, 0, 17, 255], (0, 17),
                                np.array([[0, 3], [17, 255]]), np.array([True, False])])
def test_empty_duplicate_multidimensional_and_bool_IDs(monkeypatch, ids):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'M')
    labels = np.arange(256, dtype=np.int16).reshape(16, 16)
    before = membership_stats()['native_membership_calls']
    result = semantic_membership(labels, ids)
    assert np.array_equal(result, np.isin(labels, ids))
    assert membership_stats()['native_membership_calls'] == before + 1


def test_each_call_observes_label_alias_ID_alias_and_output_isolation(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'PLM')
    labels = np.zeros((4, 5), np.int16)
    ids = np.array([0], np.int16)
    first = semantic_membership(labels, ids)
    labels.view()[2, 3] = 7
    ids.view()[0] = 7
    second = semantic_membership(labels, ids)
    assert first.all() and second.sum() == 1 and second[2, 3]
    second[:] = True
    assert semantic_membership(labels, ids).sum() == 1
    dynamic_ids = [0]
    old = semantic_membership(labels, dynamic_ids)
    dynamic_ids[0] = 7
    assert not np.array_equal(old, semantic_membership(labels, dynamic_ids))
    assert not membership_stats()['input_content_cached']
    assert not membership_stats()['table_cached']
    assert not membership_stats()['audit_or_ACK_results_cached']


@pytest.mark.parametrize('shape', [(1, 1), (1, 17), (19, 1), (35, 27)])
def test_non_square_shapes_and_readonly_source(monkeypatch, shape):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'M')
    labels = np.resize(np.arange(256, dtype=np.int16), shape)
    labels.flags.writeable = False
    result = semantic_membership(labels, [0, 5, 17])
    assert np.array_equal(result, np.isin(labels, [0, 5, 17]))
    assert not labels.flags.writeable and result.flags.writeable


@pytest.mark.parametrize('layout', ['transpose', 'negative', 'stride', 'Fortran', 'empty',
                                   'one_dimension', 'scalar', 'negative_wide', 'too_large',
                                   'float', 'object', 'bool', 'subclass', 'non_native'])
def test_unknown_layouts_types_and_ranges_use_original(monkeypatch, layout):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'M')
    labels = np.arange(256, dtype=np.int16).reshape(16, 16)
    if layout == 'transpose': labels = labels.T
    elif layout == 'negative': labels = labels[::-1]
    elif layout == 'stride': labels = labels[:, ::2]
    elif layout == 'Fortran': labels = np.asfortranarray(labels)
    elif layout == 'empty': labels = np.empty((0, 3), np.int16)
    elif layout == 'one_dimension': labels = labels.ravel()
    elif layout == 'scalar': labels = np.array(5, np.int16)
    elif layout == 'negative_wide': labels[1, 1] = -1
    elif layout == 'too_large': labels[1, 1] = 300
    elif layout == 'float': labels = labels.astype(float)
    elif layout == 'object': labels = labels.astype(object)
    elif layout == 'bool': labels = labels.astype(bool)
    elif layout == 'non_native': labels = labels.astype('>i2')
    elif layout == 'subclass':
        class Custom(np.ndarray): pass
        labels = labels.view(Custom)
    before = membership_stats()['original_fallback_calls']
    result = semantic_membership(labels, [0, 17])
    assert np.array_equal(result, np.isin(labels, [0, 17]))
    assert membership_stats()['original_fallback_calls'] == before + 1


@pytest.mark.parametrize('ids', [set([1, 2]), range(3), [1., 2.], ['1'], [None],
                                [2**80], np.array([2**64-1], np.uint64), [-1], [256]])
def test_unknown_ID_types_and_out_of_domain_IDs_preserve_numpy(monkeypatch, ids):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'M')
    labels = np.arange(256, dtype=np.int16).reshape(16, 16)
    before = membership_stats()['original_fallback_calls']
    try:
        expected = np.isin(labels, ids)
    except (TypeError, ValueError, OverflowError, IndexError) as original_error:
        with pytest.raises(type(original_error)) as observed:
            semantic_membership(labels, ids)
        assert str(observed.value) == str(original_error)
    else:
        assert np.array_equal(semantic_membership(labels, ids), expected)
    assert membership_stats()['original_fallback_calls'] == before + 1


@pytest.mark.parametrize('mode', ['control', 'P', 'L', 'PL'])
def test_disabled_modes_have_no_native_membership_call(monkeypatch, mode):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', mode)
    labels = np.arange(256, dtype=np.int16).reshape(16, 16)
    before = membership_stats()
    assert np.array_equal(semantic_membership(labels, [0, 17]), np.isin(labels, [0, 17]))
    after = membership_stats()
    assert after['native_membership_calls'] == before['native_membership_calls']
    assert after['disabled_calls'] == before['disabled_calls'] + 1


@pytest.mark.parametrize('bad_result', ['error', 'wrong_shape', 'wrong_dtype', 'none'])
def test_native_error_or_unexpected_result_falls_back_exactly(monkeypatch, bad_result):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'M')
    def failed(*args):
        if bad_result == 'error':raise cv2.error('lookup failed')
        if bad_result == 'wrong_shape':return np.zeros((1, 1), np.uint8)
        if bad_result == 'wrong_dtype':return np.zeros((16, 16), np.float32)
        return None
    monkeypatch.setattr(cv2, 'LUT', failed)
    labels = np.arange(256, dtype=np.int16).reshape(16, 16)
    before = membership_stats()['original_fallback_calls']
    assert np.array_equal(semantic_membership(labels, [0, 17]), np.isin(labels, [0, 17]))
    assert membership_stats()['original_fallback_calls'] == before + 1
