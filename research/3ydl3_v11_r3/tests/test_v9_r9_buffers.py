from types import SimpleNamespace
import numpy as np
import pytest
from three_yd_v5.v9_maps import seal, immutable
from three_yd_v5.v9_r9_buffers import initial_static_hard, initial_search_scope, buffer_stats
from three_yd_v3.dynamic import DynamicObstacles


@pytest.fixture
def enabled(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_MODE', 'A')
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'PLMU')


@pytest.mark.parametrize('mode', ['U', 'PLU', 'PLMU'])
@pytest.mark.parametrize('shape', [(1, 1), (1, 19), (35, 1), (129, 17)])
def test_current_scope_exact_bytes_single_immutable_owner(monkeypatch, mode, shape):
    monkeypatch.setenv('THREEYD_V9_MODE', 'A')
    monkeypatch.setenv('THREEYD_V9_R9_MODE', mode)
    source = np.arange(np.prod(shape)).reshape(shape) % 3 == 0
    old = source.copy(); before = buffer_stats()
    priority, legal = initial_search_scope(source, True)
    assert priority is legal and immutable(priority)
    assert priority.dtype == source.dtype and priority.shape == source.shape
    assert priority.tobytes() == source.tobytes()
    assert not np.shares_memory(source, priority)
    source[:] = ~old
    assert np.array_equal(priority, old)
    with pytest.raises(ValueError): priority.setflags(write=True)
    with pytest.raises(ValueError): legal.view()[0, 0] = False
    assert buffer_stats()['scope_shared_calls'] == before['scope_shared_calls'] + 1
    assert buffer_stats()['scope_duplicate_buffer_bytes_avoided'] == before['scope_duplicate_buffer_bytes_avoided'] + old.nbytes


@pytest.mark.parametrize('mode', ['control', 'P', 'L', 'PL', 'M', 'PM', 'LM', 'PLM'])
def test_old_modes_keep_original_mutable_scope_copy(monkeypatch, mode):
    monkeypatch.setenv('THREEYD_V9_MODE', 'A')
    monkeypatch.setenv('THREEYD_V9_R9_MODE', mode)
    original = np.ones((3, 5), bool)
    priority, legal = initial_search_scope(original, True)
    assert priority is original and legal.flags.writeable and legal.flags.owndata
    assert np.array_equal(legal, original) and not np.shares_memory(legal, original)
    legal[0, 0] = False
    assert original[0, 0]
    world = seal(original)
    static = initial_static_hard(world, original)
    assert immutable(static) and static is not world and np.array_equal(static, world)


@pytest.mark.parametrize('generic', [False, None, 1, np.bool_(True)])
def test_only_exact_map_only_flag_shares_scope(enabled, generic):
    original = np.ones((4, 5), bool)
    priority, legal = initial_search_scope(original, generic)
    assert priority is original and legal.flags.writeable and not np.shares_memory(original, legal)


@pytest.mark.parametrize('factory', [
    lambda: np.ones((6, 7), bool)[:, ::2],
    lambda: np.ones((6, 7), bool).T,
    lambda: seal(np.ones((6, 7), bool)),
    lambda: np.ones(12, bool),
    lambda: np.empty((0, 7), bool),
    lambda: np.ones((6, 7), np.uint8),
    lambda: np.ones((6, 7), bool).view(np.ma.MaskedArray),
])
def test_unknown_scope_layout_type_preserves_original_copy(enabled, factory):
    original = factory()
    priority, legal = initial_search_scope(original, True)
    assert priority is original and type(legal) is type(original.copy())
    assert np.array_equal(legal, original.copy())
    assert not np.shares_memory(legal, original)


@pytest.mark.parametrize('cell', [(0, 0), (63, 16), (64, 0), (128, 16)])
def test_static_checks_each_actual_row_and_current_content(enabled, cell):
    raw = np.zeros((129, 17), bool); raw[1, 1] = True
    world = seal(raw); static = raw.copy()
    assert initial_static_hard(world, static) is world
    static[cell] = not static[cell]
    changed = initial_static_hard(world, static)
    assert changed is not world and immutable(changed)
    assert np.array_equal(changed, static)
    static[:] = raw
    assert initial_static_hard(world, static) is world
    assert not buffer_stats()['static_or_scope_content_cached']
    assert not buffer_stats()['audit_or_ACK_results_cached']


@pytest.mark.parametrize('case', ['mutable_world', 'readonly_flag_only', 'noncontiguous', 'different_shape', 'different_dtype'])
def test_unsafe_static_binding_keeps_original_independent_seal(enabled, case):
    source = np.zeros((8, 9), bool); world = seal(source); static = source.copy()
    if case == 'mutable_world': world = source
    if case == 'readonly_flag_only':
        world = source.view(); world.setflags(write=False)
    if case == 'noncontiguous':
        world = world[:, ::2]; static = static[:, ::2]
    if case == 'different_shape': static = np.zeros((7, 9), bool)
    if case == 'different_dtype': static = static.astype(np.uint8)
    result = initial_static_hard(world, static)
    assert result is not world and immutable(result)
    assert result.dtype == static.dtype and result.shape == static.shape
    assert np.array_equal(result, static)


def test_original_dynamic_apply_with_shared_static_never_mutates_static(enabled):
    static = np.zeros((12, 13), bool); static[0, :] = True
    world = SimpleNamespace(hard=seal(static))
    world.cells = lambda p: (np.asarray(p)[:, 1].astype(int), np.asarray(p)[:, 0].astype(int))
    manager = DynamicObstacles(world)
    manager.static_hard = initial_static_hard(world.hard, manager.static_hard)
    original = manager.static_hard
    assert original is world.hard and immutable(original)
    update = manager.apply(1, upsert={'block': [[3, 3], [5, 3], [5, 5], [3, 5]]})
    assert update['changed_cells'] > 0 and manager.version == 1
    assert world.hard is not original and world.hard[4, 4] and not original[4, 4]
    assert np.array_equal(original, static)
    world.hard = seal(world.hard)
    assert not initial_static_hard(world.hard, static.copy()) is world.hard
    with pytest.raises(ValueError): manager.apply(2, clear=['block'])
    manager.apply(2, clear=['block'], observed_free=True)
    assert manager.version == 2 and np.array_equal(world.hard, static)
    assert manager.static_hard is original and immutable(original)


def test_maps_mode_off_preserves_original_even_if_U_enabled(enabled, monkeypatch):
    monkeypatch.setenv('THREEYD_V9_MODE', 'control')
    source = np.ones((3, 4), bool)
    priority, legal = initial_search_scope(source, True)
    assert priority is source and legal.flags.writeable and not np.shares_memory(priority, legal)
    world = seal(source)
    assert initial_static_hard(world, source) is not world


def test_each_scope_call_uses_current_priority_and_no_result_cache(enabled):
    source = np.zeros((3, 4), bool)
    first, _ = initial_search_scope(source, True)
    source[1, 2] = True
    second, _ = initial_search_scope(source, True)
    assert second[1, 2] and not first[1, 2]
    assert first is not second and not np.shares_memory(first, second)
