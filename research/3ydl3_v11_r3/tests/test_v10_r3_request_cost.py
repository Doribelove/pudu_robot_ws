import numpy as np
import pytest
from three_yd_v5.request_cost import wide_membership
from test_v8_r4_native import pair


@pytest.mark.parametrize('dtype', [np.int16, np.uint16])
@pytest.mark.parametrize('ids', [[], [0, 255, 256, 32767, -32768, -1, 65535],
    [65536, -65537], np.array([2**64-1, 65535], np.uint64),
    np.array([[True, False]]), np.arange(-40000, 80000, 97)])
def test_full_word_domain_matches_original(monkeypatch, dtype, ids):
    monkeypatch.setenv('THREEYD_V10_R3_MODE', 'B')
    labels = np.arange(65536, dtype=np.uint16).view(dtype).reshape(256, 256)
    labels.flags.writeable = False
    actual = wide_membership(labels, ids)
    assert actual.dtype == np.bool_ and actual.flags.writeable
    assert np.array_equal(actual.view(np.uint8), np.isin(labels, ids).view(np.uint8))
    assert not np.shares_memory(actual, labels)


def test_current_source_and_ids_observed_each_call(monkeypatch):
    monkeypatch.setenv('THREEYD_V10_R3_MODE', 'AB')
    labels = np.zeros((7, 9), np.int16)
    ids = np.array([0], np.int32)
    before = wide_membership(labels, ids)
    labels[3, 4] = -32768
    ids[0] = -32768
    after = wide_membership(labels, ids)
    assert before.all() and after.sum() == 1 and after[3, 4]
    after[:] = True
    assert wide_membership(labels, ids).sum() == 1


@pytest.mark.parametrize('labels', [np.zeros((5, 6), np.int32),
    np.zeros((5, 6), np.int16).T, np.zeros((5, 6), np.int16)[::-1],
    np.zeros((5, 6), dtype='>i2'), np.zeros((5,), np.int16)])
def test_unsupported_layouts_decline(monkeypatch, labels):
    monkeypatch.setenv('THREEYD_V10_R3_MODE', 'B')
    assert wide_membership(labels, [0]) is None


@pytest.mark.parametrize('seed', range(12))
def test_compact_state_complete_incremental_equivalence(monkeypatch, seed):
    monkeypatch.setenv('THREEYD_V10_R3_MODE', 'control')
    old = pair(seed)[1]
    monkeypatch.setenv('THREEYD_V10_R3_MODE', 'A')
    new = pair(seed)[1]
    for key in ('cells', 'xy_direction', 'progress', 'edge_weights', '_start_heuristic'):
        assert np.array_equal(getattr(old, key), getattr(new, key))
    for key in ('_rows', '_cols', '_dx', '_dy'):
        assert list(getattr(old, key)) == list(getattr(new, key))
        assert isinstance(getattr(new, key), memoryview)
    for state in (old, new):
        state.compute_shortest_path(max_expansions=3)
    assert sorted(old.open) == sorted(new.open) and old.queued == new.queued
    for step in range(3):
        for state in (old, new):
            if step == 1:
                state.initialized = True
                state.set_blocked_ids({5, 12, 35})
            if step == 2:
                state.set_blocked_ids(set())
                state.replace_potential(state.potential + .25)
            state.compute_shortest_path(timeout_s=5.)
        assert np.array_equal(old.g, new.g) and np.array_equal(old.rhs, new.rhs)
        assert sorted(old.open) == sorted(new.open) and old.queued == new.queued
        assert old.extract_path_ids() == new.extract_path_ids()


def test_replaced_task_controller_released_before_new_construction(monkeypatch):
    import weakref
    from types import SimpleNamespace
    from three_yd_v5 import fullroute_service as module
    monkeypatch.setenv('THREEYD_V10_R3_MODE', 'R')
    class Controller:
        trajectory = object()
        trajectory_version = 1
        lifecycle = SimpleNamespace(clear=lambda: None)
    service = module.FullRouteService.__new__(module.FullRouteService)
    service.controller = Controller()
    previous = weakref.ref(service.controller)
    service.session = SimpleNamespace(active_controller=service.controller,active_route=object())
    service.world=service.topology=service.root=service.auditor=service.spec=None
    service.views={};service.dynamic=SimpleNamespace(version=0);service.max_region_routes=3
    class Runner:
        def __init__(self,*args,**kwargs): pass
        def plan(self,*args):
            assert previous() is None
            assert service.session.active_controller is None
            return None,None,{'success':False,'failure_code':'NO_PATH'}
    monkeypatch.setattr(module,'V5Runner',Runner)
    assert not service._solve(object())['success']


def test_scalar_fields_keep_original_snapshot_semantics(monkeypatch):
    monkeypatch.setenv('THREEYD_V10_R3_MODE', 'A')
    state=pair(3)[1]
    before={k:list(getattr(state,k)) for k in ('_rows','_cols','_dx','_dy')}
    state.cells[:]=0
    state.xy_direction[:]=17
    assert all(list(getattr(state,k))==v for k,v in before.items())
