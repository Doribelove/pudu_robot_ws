"""Transaction binding and fail-closed r3 ACK tests; no ROS processes required."""

from dataclasses import FrozenInstanceError, replace
from types import SimpleNamespace

import numpy as np
import pytest


def test_frozen_expected_master_owns_bytes_and_rejects_changed_readback():
    from arena_evaluation.exact_ack_r3 import FrozenExpectedMaster, Publication, compare_exact, grid_hash
    source=np.arange(12,dtype=np.uint8).reshape(3,4)
    expected=FrozenExpectedMaster(source)
    original=source.copy()
    token=Publication(1,'source',(0,0,4,3),grid_hash(original),original.shape,'map','request')
    source[:]=255
    assert compare_exact(token,token,expected,original)['acknowledged']
    assert not compare_exact(token,token,expected,source)['acknowledged']
    with pytest.raises(ValueError):expected.array[0,0]=100
    with pytest.raises(ValueError):expected.array.setflags(write=True)


def test_frozen_expected_strided_hash_matches_complete_canonical_bytes():
    from arena_evaluation.exact_ack_r3 import FrozenExpectedMaster, grid_hash
    source=np.arange(48,dtype=np.uint8).reshape(6,8)[::-1,::2]
    expected=FrozenExpectedMaster(source)
    assert expected.sha256==grid_hash(source)
    assert np.array_equal(expected.array,source)
    with pytest.raises(ValueError):FrozenExpectedMaster(source.astype(np.int16))

from arena_evaluation import exact_ack_r3 as ack


def _publication(expected):
    return ack.Publication(
        sequence=7,
        source_grid_hash="source-grid",
        roi_bbox=(0, 0, 1, 1),
        expected_effective_hash=ack.grid_hash(expected),
        expected_shape=expected.shape,
        map_hash="frozen-map",
        request_id="A2B-01-measured-1",
    )


def test_publication_is_immutable_and_hash_is_deterministic():
    expected = np.zeros((3, 3), dtype=np.uint8)
    first = _publication(expected)
    assert first == _publication(expected.copy())
    assert first.hash == _publication(expected.copy()).hash
    with pytest.raises(FrozenInstanceError):
        first.sequence = 8


@pytest.mark.parametrize(
    "field,value",
    [
        ("sequence", 8),
        ("source_grid_hash", "other-source"),
        ("roi_bbox", (0, 0, 2, 1)),
        ("expected_effective_hash", "other-effective"),
        ("expected_shape", (1, 9)),
        ("map_hash", "other-map"),
        ("request_id", "A2B-01-measured-2"),
    ],
)
def test_exact_content_cannot_ack_another_publication(field, value):
    expected = np.zeros((3, 3), dtype=np.uint8)
    publication = _publication(expected)
    active = replace(publication, **{field: value})
    result = ack.compare_exact(publication, active, expected, expected.copy())
    assert result["acknowledged"] is False
    assert result["binding_mismatch"] == 1
    assert result["sequence_mismatch"] == int(field == "sequence")
    assert result["mismatch_cells"] == 0
    assert active.hash != publication.hash


def test_equal_full_content_and_binding_are_acknowledged():
    expected = np.array([[254, 40, 0], [253, 0, 255]], dtype=np.uint8)
    token = _publication(expected)
    result = ack.compare_exact(
        token, token, expected, expected.copy(), readback_hash=ack.grid_hash(expected)
    )
    assert result["acknowledged"] is True
    for key in (
        "mismatch_cells", "hard_mismatch", "soft_mismatch", "stale_cells",
        "sequence_mismatch", "binding_mismatch", "shape_mismatch", "hash_mismatch",
    ):
        assert result[key] == 0


@pytest.mark.parametrize("wrong_side", ["expected", "server", "both"])
def test_wrong_shape_cannot_ack_even_when_flat_bytes_match(wrong_side):
    original = np.zeros((3, 3), dtype=np.uint8)
    token = _publication(original)
    expected = original.reshape(1, 9) if wrong_side in {"expected", "both"} else original
    server = original.reshape(1, 9) if wrong_side in {"server", "both"} else original
    result = ack.compare_exact(token, token, expected, server)
    assert result["acknowledged"] is False
    assert result["shape_mismatch"] == 1


def test_expected_hash_corruption_fails_even_if_expected_and_server_equal():
    expected = np.zeros((3, 3), dtype=np.uint8)
    token = replace(_publication(expected), expected_effective_hash="incorrect-hash")
    result = ack.compare_exact(token, token, expected, expected.copy())
    assert result["acknowledged"] is False
    assert result["mismatch_cells"] == 0
    assert result["hash_mismatch"] == 1


def test_claimed_readback_hash_corruption_fails():
    expected = np.zeros((3, 3), dtype=np.uint8)
    token = _publication(expected)
    result = ack.compare_exact(token, token, expected, expected.copy(), readback_hash="incorrect")
    assert result["acknowledged"] is False
    assert result["hash_mismatch"] == 1


def test_one_soft_stale_cell_outside_roi_is_rejected():
    expected = np.zeros((3, 3), dtype=np.uint8)
    server = expected.copy()
    server[2, 2] = 40
    token = _publication(expected)
    assert token.roi_bbox == (0, 0, 1, 1)
    result = ack.compare_exact(token, token, expected, server)
    assert result["acknowledged"] is False
    assert result["mismatch_cells"] == 1
    assert result["soft_mismatch"] == 1
    assert result["hard_mismatch"] == 0
    assert result["stale_cells"] == 1
    assert result["hash_mismatch"] == 1


@pytest.mark.parametrize("expected_cost", [253, 254, 255])
def test_hard_or_unknown_cell_cannot_be_acknowledged_as_free(expected_cost):
    expected = np.zeros((3, 3), dtype=np.uint8)
    expected[2, 2] = expected_cost
    server = expected.copy()
    server[2, 2] = 0
    token = _publication(expected)
    result = ack.compare_exact(token, token, expected, server)
    assert result["acknowledged"] is False
    assert result["mismatch_cells"] == result["hard_mismatch"] == 1


class _Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def spin_once(self, timeout_sec=0.0):
        self.now += max(0.001, timeout_sec)


def _fake_session(monkeypatch, tmp_path, snapshots):
    """Exercise actual update_local_mask/plan with only transport and action faked."""
    clock = _Clock()
    monkeypatch.setattr(ack, "time", SimpleNamespace(monotonic=clock.monotonic))
    session = object.__new__(ack.ExactAckSmacSession)
    source = np.zeros((3, 3), dtype=np.int8)
    expected = np.zeros((3, 3), dtype=np.uint8)
    session.ctx = SimpleNamespace(map_sha256="frozen-map")
    session.runtime_safety_contract={'verified':True,'map_hash':'frozen-map','sha256':'mock-runtime-contract'}
    session._grid_for_mask = lambda mask: (np.asarray(mask, bool), source.copy())
    session._expected = lambda grid: expected.copy()
    session._current_grid = source.copy()
    session._current_grid[0, 0] = 100
    session._current_allowed_mask = None
    session._costmap_state_trusted = False
    session._local_mask_info = {}
    session.publication_sequence = 0
    session.active_publication = None
    session.request_id = ""
    session.request_deadline = float("inf")
    session.last_exact_ack = None
    session.ack_trace = []
    session.trace_file = tmp_path / "exact_ack.jsonl"
    session.client = SimpleNamespace(executor=clock, timeout=7.0)
    session.begin_request("A2B-01-measured-1", clock.monotonic() + 0.6)
    transport = SimpleNamespace(publications=[], snapshot_count=0, actions=[])

    def publish(grid, dirty, **kwargs):
        assert not transport.actions, "Publication must precede action dispatch"
        clock.now += 0.005
        transport.publications.append(np.asarray(dirty, bool).copy())
        cells = int(np.count_nonzero(dirty))
        return {"serialization_ms": 0.0, "publication_ms": 0.0,
                "cells": cells, "messages": int(cells > 0), "chunks": []}

    def snapshot(deadline):
        assert not transport.actions, "Search must wait for complete ACK"
        clock.now += 0.05
        index = min(transport.snapshot_count, len(snapshots) - 1)
        transport.snapshot_count += 1
        result = snapshots[index]
        if isinstance(result, Exception):
            raise result
        if callable(result):
            result = result(session)
        return result.copy(), int(clock.now * 1e9)

    def action(self, query, spec, **kwargs):
        assert self is session
        assert session.last_exact_ack == session.active_publication
        assert session.last_exact_ack.request_id == session.request_id
        assert session._local_mask_info["costmap_ack_mismatch_cells"] == 0
        assert session._costmap_state_trusted is True
        assert kwargs["allowed_mask"] is None
        transport.actions.append({"snapshots": transport.snapshot_count})
        return ack.PlanResult(planner_success=True, points=[{"x": 0.0, "y": 0.0}],
                              failure_code="", diagnostics={})

    session._publish_chunks = publish
    session._server_costmap_snapshot = snapshot
    monkeypatch.setattr(ack.SmacSession, "plan", action)
    return session, transport, clock


def _plan(session):
    return session.plan(SimpleNamespace(query_id="A2B-01"),
                        SimpleNamespace(backend="fake-smac", version="test"),
                        allowed_mask=np.ones((3, 3), dtype=bool))


def test_repair_must_pass_full_exact_ack_before_search(monkeypatch, tmp_path):
    expected = np.zeros((3, 3), dtype=np.uint8)
    stale = expected.copy()
    stale[2, 2] = 40
    session, transport, _ = _fake_session(monkeypatch, tmp_path, [stale, expected, expected])
    result = _plan(session)
    assert result.planner_success is True
    assert result.diagnostics["planner_search_started"] is True
    assert result.diagnostics["costmap_ack_mismatch_cells"] == 0
    assert result.diagnostics["costmap_ack_repair_count"] == 1
    assert transport.actions == [{"snapshots": 3}]
    assert transport.publications[0][0, 0]
    assert not transport.publications[0][2, 2]
    assert transport.publications[1][2, 2], "Repair must include mismatch outside original ROI"
    readbacks = [row for row in session.ack_trace if row["event"] == "readback"]
    assert [row["mismatch_cells"] for row in readbacks] == [1, 0, 0]


def test_one_cell_remaining_after_repair_and_full_fallback_never_searches(monkeypatch, tmp_path):
    stale = np.zeros((3, 3), dtype=np.uint8)
    stale[2, 2] = 40
    session, transport, _ = _fake_session(monkeypatch, tmp_path, [stale])
    result = _plan(session)
    assert result.planner_success is False
    assert result.failure_code == "EXACT_ACK_FAILED_CLOSED"
    assert result.diagnostics["planner_search_started"] is False
    assert result.diagnostics["costmap_ack_mismatch_cells"] == 1
    assert result.diagnostics["costmap_ack_repair_count"] == 2
    assert result.diagnostics["full_update_fallback"] is True
    assert len(transport.publications) == 4
    assert np.all(transport.publications[-1])
    assert transport.actions == []
    assert session.last_exact_ack is None
    assert session._costmap_state_trusted is False


def test_full_fallback_is_recorded_and_search_waits_for_post_fallback_exact_reads(monkeypatch, tmp_path):
    expected = np.zeros((3, 3), dtype=np.uint8)
    stale = expected.copy()
    stale[2, 2] = 40
    session, transport, _ = _fake_session(
        monkeypatch, tmp_path, [stale, stale, stale, expected, expected]
    )
    result = _plan(session)
    assert result.planner_success is True
    assert result.diagnostics["full_update_fallback"] is True
    assert result.diagnostics["costmap_ack_repair_count"] == 2
    assert transport.actions == [{"snapshots": 5}]
    assert any(row["event"] == "full_update_fallback" for row in session.ack_trace)


def test_server_readback_errors_fail_closed_without_search(monkeypatch, tmp_path):
    session, transport, _ = _fake_session(monkeypatch, tmp_path, [RuntimeError("injected readback error")])
    result = _plan(session)
    assert result.planner_success is False
    assert result.failure_code == "EXACT_ACK_FAILED_CLOSED"
    assert result.diagnostics["planner_search_started"] is False
    assert result.diagnostics["readback_error"] > 0
    assert transport.actions == []
    assert any(row["event"] == "readback_error" for row in session.ack_trace)


def test_concurrent_stale_sequence_cannot_search_even_with_exact_bytes(monkeypatch, tmp_path):
    def stale_sequence(session):
        session.active_publication = replace(session.active_publication, sequence=999)
        return np.zeros((3, 3), dtype=np.uint8)

    session, transport, _ = _fake_session(monkeypatch, tmp_path, [stale_sequence])
    result = _plan(session)
    assert result.planner_success is False
    assert result.diagnostics["sequence_mismatch"] == 1
    assert result.diagnostics["costmap_ack_mismatch_cells"] == 0
    assert result.diagnostics["planner_search_started"] is False
    assert transport.actions == []


def test_new_request_cannot_reuse_previous_exact_ack(monkeypatch, tmp_path):
    expected = np.zeros((3, 3), dtype=np.uint8)
    session, transport, clock = _fake_session(monkeypatch, tmp_path, [expected, expected])
    session.update_local_mask(np.ones((3, 3), dtype=bool))
    assert session.last_exact_ack is not None
    session.begin_request("A2B-02-measured-1", clock.monotonic() + 0.6)
    assert session.last_exact_ack is None
    # No current-request readback is available; advance virtual time to bound the loop.
    def failed_snapshot(deadline):
        clock.now += 0.1
        raise RuntimeError("unavailable")
    session._server_costmap_snapshot = failed_snapshot
    result = _plan(session)
    assert result.planner_success is False
    assert result.diagnostics["planner_search_started"] is False
    assert transport.actions == []

@pytest.mark.parametrize('remaining,expected', [(2., .5), (9., 5.)])
def test_remaining_server_budget_never_increases_frozen_limit(monkeypatch, tmp_path, remaining, expected):
    session, transport, clock = _fake_session(monkeypatch, tmp_path, [np.zeros((3, 3), dtype=np.uint8)] * 2)
    captured = []
    response = SimpleNamespace(results=[SimpleNamespace(successful=True)])
    future = SimpleNamespace(done=lambda: True, result=lambda: response)
    def call_async(request):
        assert not ack.gc.isenabled(), 'RPC must remain responsive through confirmation'
        captured.extend(request.parameters)
        return future
    session._budget_client = SimpleNamespace(wait_for_service=lambda **kw: True, call_async=call_async)
    value = session._set_server_budget(remaining)
    assert value == pytest.approx(expected)
    assert captured[0].name == 'GridBased.max_planning_time'
    assert captured[0].value.double_value == pytest.approx(expected)
    assert value <= 5. and value < remaining


@pytest.mark.parametrize('initially_enabled', [True, False])
@pytest.mark.parametrize('raises', [True, False])
def test_transport_gc_restores_prior_state_even_on_exception(monkeypatch, initially_enabled, raises):
    state=SimpleNamespace(enabled=initially_enabled)
    monkeypatch.setattr(ack, 'gc', SimpleNamespace(
        isenabled=lambda:state.enabled,
        disable=lambda:setattr(state,'enabled',False),
        enable=lambda:setattr(state,'enabled',True)))
    try:
        with ack.defer_transport_gc():
            assert not state.enabled
            with ack.defer_transport_gc():
                assert not state.enabled
            assert not state.enabled
            if raises:raise RuntimeError('transport failed')
    except RuntimeError:
        assert raises
    assert state.enabled is initially_enabled


def test_pending_collection_after_budget_confirmation_still_consumes_deadline(monkeypatch,tmp_path):
    session,transport,clock=_fake_session(monkeypatch,tmp_path,[np.zeros((3,3),np.uint8)]*2)
    session._enforce_server_budget=True
    session.request_deadline=clock.now+7.
    state=SimpleNamespace(enabled=True)
    def restore():
        assert session._confirmed_server_budget is not None
        assert not session._budget_state_uncertain
        state.enabled=True
        clock.now=session.request_deadline+.01
    monkeypatch.setattr(ack,'gc',SimpleNamespace(isenabled=lambda:state.enabled,
        disable=lambda:setattr(state,'enabled',False),enable=restore))
    response=SimpleNamespace(results=[SimpleNamespace(successful=True)])
    session._budget_client=SimpleNamespace(wait_for_service=lambda **kw:True,
        call_async=lambda request:SimpleNamespace(done=lambda:True,result=lambda:response))
    result=_plan(session)
    assert state.enabled and not transport.actions
    assert result.failure_code=='REQUEST_DEADLINE_BEFORE_SEARCH'
    assert not result.diagnostics['planner_search_started']


def test_action_exchange_defers_gc_and_restores_before_local_result_handling(monkeypatch,tmp_path):
    session,transport,clock=_fake_session(monkeypatch,tmp_path,[np.zeros((3,3),np.uint8)]*2)
    enabled=ack.gc.isenabled()
    def action(self,query,spec,**kwargs):
        assert not ack.gc.isenabled()
        assert self.last_exact_ack==self.active_publication
        return ack.PlanResult(failure_code='CLIENT_TIMEOUT',diagnostics={})
    monkeypatch.setattr(ack.SmacSession,'plan',action)
    result=_plan(session)
    assert ack.gc.isenabled() is enabled
    assert result.failure_code=='CLIENT_TIMEOUT' and session._unresolved_timeout


def test_failed_server_budget_binding_prevents_search(monkeypatch, tmp_path):
    session, transport, clock = _fake_session(monkeypatch, tmp_path, [np.zeros((3, 3), dtype=np.uint8)] * 2)
    session._enforce_server_budget = True
    session.request_deadline=clock.now+7.
    session._budget_client = SimpleNamespace(wait_for_service=lambda **kw: False)
    result = _plan(session)
    assert result.planner_success is False
    assert result.failure_code == 'REQUEST_BUDGET_SERVICE_UNAVAILABLE'
    assert result.diagnostics['planner_search_started'] is False
    assert not transport.actions


def test_budget_service_latency_cannot_dispatch_expired_search(monkeypatch, tmp_path):
    session, transport, clock = _fake_session(monkeypatch, tmp_path, [np.zeros((3, 3), np.uint8)] * 2)
    session._enforce_server_budget = True

    def delayed_parameter_result(remaining):
        clock.now = session.request_deadline + .01
        return max(.001, remaining - .3)

    session._set_server_budget = delayed_parameter_result
    result = _plan(session)
    print('deadline', session.request_deadline, 'action_dispatch_after', clock.now,
          'actions', transport.actions, 'result', result.failure_code)
    assert not transport.actions, 'Server budget response arrived after absolute request deadline'
    assert result.diagnostics['planner_search_started'] is False


def test_next_request_publication_failure_does_not_report_previous_exact_ack(monkeypatch, tmp_path):
    session, transport, clock = _fake_session(monkeypatch, tmp_path, [np.zeros((3, 3), np.uint8)] * 2)
    first = session.update_local_mask(np.ones((3, 3), bool))
    assert first['costmap_update_acknowledged'] is True
    session.begin_request('A2B-02-measured-1', clock.now + .1)

    def exhausted_publication(*args, **kwargs):
        raise ack.ExactAckFailure('REQUEST_DEADLINE_PUBLICATION')

    session._publish_chunks = exhausted_publication
    result = _plan(session)
    print('current_request', session.request_id, 'active_publication', session.active_publication,
          'returned_failure', result.failure_code, 'returned_ack', result.diagnostics)
    assert result.diagnostics['planner_search_started'] is False
    assert result.diagnostics.get('costmap_update_acknowledged') is not True
    assert result.diagnostics.get('publication', {}).get('request_id') != 'A2B-01-measured-1'


def test_same_confirmed_safe_budget_does_not_rebuild_smac(monkeypatch, tmp_path):
    session, _, _ = _fake_session(monkeypatch, tmp_path, [np.zeros((3,3),np.uint8)]*2)
    session._confirmed_server_budget=5.
    session._budget_client=SimpleNamespace(call_async=lambda _:pytest.fail('Repeated budget must not reinitialize Smac'))
    assert session._set_server_budget(7.)==5.
    assert session._budget_diagnostics['server_budget_cache_hit'] is True


def test_budget_larger_than_remaining_after_reconfiguration_never_searches(monkeypatch, tmp_path):
    session, transport, clock = _fake_session(monkeypatch, tmp_path, [np.zeros((3,3),np.uint8)]*2)
    session.request_deadline=clock.now+2.;session._enforce_server_budget=True
    def expensive_reconfiguration(remaining):
        clock.now+=.7
        return 1.5
    session._set_server_budget=expensive_reconfiguration
    result=_plan(session)
    assert clock.now<session.request_deadline, 'This regression must retain a positive client time balance'
    assert result.failure_code=='SERVER_BUDGET_EXCEEDS_REMAINING_DEADLINE'
    assert result.diagnostics['planner_search_started'] is False and not transport.actions


def test_small_budget_upgrade_cannot_spend_request_on_lookup_rebuild(monkeypatch,tmp_path):
    session,_,_=_fake_session(monkeypatch,tmp_path,[np.zeros((3,3),np.uint8)]*2)
    session._confirmed_server_budget=2.
    session._budget_client=SimpleNamespace(
        wait_for_service=lambda **kw:pytest.fail('An avoidable upgrade must not touch the service'),
        call_async=lambda _:pytest.fail('A half-second gain cannot justify rebuilding the lookup table'))
    assert session._set_server_budget(3.8)==2.
    assert session._budget_diagnostics['server_budget_upgrade_gain_s']==.5
    assert session._budget_diagnostics['server_budget_update_ms']==0.
    assert session._confirmed_server_budget==2.


def test_in_flight_update_can_settle_without_premature_repair(monkeypatch, tmp_path):
    expected=np.zeros((3,3),np.uint8);stale=expected.copy();stale[2,2]=40
    session,transport,_=_fake_session(monkeypatch,tmp_path,[stale,expected,expected])
    session._effective_compute_ms=160.
    result=_plan(session)
    assert result.planner_success is True
    assert result.diagnostics['costmap_ack_repair_count']==0
    assert result.diagnostics['costmap_ack_mismatch_cells']==0
    assert transport.actions==[{'snapshots':3}]
    assert len(transport.publications)==1


def test_expected_effective_cache_is_bounded_and_invalidates_configuration(monkeypatch):
    from collections import OrderedDict
    from arena_evaluation import _nav2_effective_costmap as native
    calls=[]
    def inflate(grid,*args):
        calls.append(args)
        return grid.tobytes()
    monkeypatch.setattr(native,'inflate',inflate)
    s=object.__new__(ack.ExactAckSmacSession);s.ctx=SimpleNamespace(map_sha256='one')
    s.smac_config_hash='config-a';s._expected_cache=OrderedDict();s._expected_cache_bytes=0
    s.expected_cache_limit_bytes=18
    source=np.zeros((3,3),np.int8)
    first=s._expected(source);assert not s._expected_cache_hit
    assert np.array_equal(s._expected(source),first) and s._expected_cache_hit and len(calls)==1
    with pytest.raises(ValueError):first[0,0]=99
    s.smac_config_hash='config-b';s._expected(source)
    assert not s._expected_cache_hit and len(calls)==2
    s.ctx.map_sha256='two';s._expected(source)
    assert not s._expected_cache_hit and len(calls)==3 and s._expected_cache_bytes<=18
    s.smac_config_hash='config-a';s.ctx.map_sha256='one';s._expected(source)
    assert not s._expected_cache_hit and len(calls)==4, 'Old binding must be evicted at the byte cap'


def test_unanswered_budget_update_retires_old_confirmation(monkeypatch,tmp_path):
    session,transport,clock=_fake_session(monkeypatch,tmp_path,[np.zeros((3,3),np.uint8)]*2)
    session._confirmed_server_budget=2.;session.request_deadline=clock.now+.05
    session._budget_client=SimpleNamespace(wait_for_service=lambda **kw:True,
        call_async=lambda request:SimpleNamespace(done=lambda:False))
    with pytest.raises(ack.ExactAckFailure,match='REQUEST_BUDGET_UPDATE_FAILED'):
        session._set_server_budget(7.)
    assert session._confirmed_server_budget is None and session._budget_state_uncertain
    session.begin_request('next-request',clock.now+3.5)
    result=_plan(session)
    assert result.failure_code=='SESSION_SERVER_BUDGET_UNCERTAIN'
    assert result.diagnostics['planner_search_started'] is False and not transport.actions


def test_budget_transport_exception_preserves_current_exact_ack(monkeypatch,tmp_path):
    session,transport,clock=_fake_session(monkeypatch,tmp_path,[np.zeros((3,3),np.uint8)]*2)
    session._enforce_server_budget=True;session.request_deadline=clock.now+7.
    def broken_transport(request):raise RuntimeError('injected parameter transport failure')
    session._budget_client=SimpleNamespace(wait_for_service=lambda **kw:True,call_async=broken_transport)
    result=_plan(session)
    assert result.failure_code=='REQUEST_BUDGET_TRANSPORT_ERROR'
    assert result.diagnostics['costmap_update_acknowledged'] is True
    assert result.diagnostics['publication']['request_id']==session.request_id
    assert result.diagnostics['server_budget_state_uncertain'] is True
    assert result.diagnostics['server_budget_update_ms']>=0
    assert result.diagnostics['planner_search_started'] is False and not transport.actions


@pytest.mark.parametrize("reliabilities", [[], [2], [1, 2]])
def test_unreliable_or_missing_static_update_reader_fails_before_requests(reliabilities):
    endpoints=[SimpleNamespace(qos_profile=SimpleNamespace(reliability=value)) for value in reliabilities]
    with pytest.raises(ack.ExactAckFailure, match='RELIABLE_UPDATE_SUBSCRIPTION_REQUIRED'):
        ack.require_reliable_subscriptions(endpoints,1)


def test_reliable_update_transport_remains_separate_from_exact_content_ack():
    ack.require_reliable_subscriptions([SimpleNamespace(qos_profile=SimpleNamespace(reliability=1))],1)
    expected=np.array([[254,253,0]],dtype=np.uint8)
    token=_publication(expected)
    # Reliable DDS delivery does not certify callback processing or inflation.
    intermediate=np.array([[254,0,0]],dtype=np.uint8)
    assert ack.compare_exact(token,token,expected,intermediate)['acknowledged'] is False
    assert ack.compare_exact(token,token,expected,expected.copy())['acknowledged'] is True


def test_xml_scopes_static_update_reader_and_lifecycle_replies_and_bounds_history():
    import xml.etree.ElementTree as ET
    from pathlib import Path
    xml=Path(__file__).resolve().parents[1]/'config/two_layer_v1_r3_transport.xml'
    root=ET.parse(xml).getroot();ns={'p':'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    profiles=list(root)
    prefix='{http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles}'
    assert [(p.tag.removeprefix(prefix),p.attrib) for p in profiles]==[
        ('subscriber',{'profile_name':'/map_updates'}),
        ('publisher',{'profile_name':'rr/map_server/change_stateReply'}),
        ('publisher',{'profile_name':'rr/planner_server/change_stateReply'}),
    ]
    reader=profiles[0]
    assert reader.find('p:qos/p:reliability/p:kind',ns).text=='RELIABLE'
    assert reader.find('p:topic/p:historyQos/p:kind',ns).text=='KEEP_LAST'
    assert int(reader.find('p:topic/p:historyQos/p:depth',ns).text)==512
    for reply in profiles[1:]:
        assert reply.find('p:qos/p:reliability/p:kind',ns).text=='RELIABLE'
        assert reply.find('p:qos/p:reliability/p:max_blocking_time/p:sec',ns).text=='2'
        assert reply.find('p:qos/p:reliability/p:max_blocking_time/p:nanosec',ns).text=='0'


def test_static_reader_discovery_can_arrive_after_client_creation(monkeypatch):
    clock=_Clock();monkeypatch.setattr(ack,'time',SimpleNamespace(monotonic=clock.monotonic))
    endpoint=SimpleNamespace(node_name='global_costmap',qos_profile=SimpleNamespace(reliability=1))
    observations=iter([[],[],[endpoint]])
    node=SimpleNamespace(get_subscriptions_info_by_topic=lambda _:next(observations))
    assert ack.discover_reliable_updates(node,clock,1,100.1)==[endpoint]
    assert clock.now==pytest.approx(100.02)


def test_missing_static_reader_discovery_is_bounded(monkeypatch):
    clock=_Clock();monkeypatch.setattr(ack,'time',SimpleNamespace(monotonic=clock.monotonic))
    # A different reliable reader cannot stand in for the actual StaticLayer.
    endpoint=SimpleNamespace(node_name='other_reader',qos_profile=SimpleNamespace(reliability=1))
    node=SimpleNamespace(get_subscriptions_info_by_topic=lambda _:[endpoint])
    with pytest.raises(ack.ExactAckFailure,match='UPDATE_SUBSCRIPTION_DISCOVERY_TIMEOUT'):
        ack.discover_reliable_updates(node,clock,1,100.1)
    assert clock.now==pytest.approx(100.1)


def test_discovered_unreliable_static_reader_is_rejected_immediately(monkeypatch):
    clock=_Clock();monkeypatch.setattr(ack,'time',SimpleNamespace(monotonic=clock.monotonic))
    endpoint=SimpleNamespace(node_name='global_costmap',qos_profile=SimpleNamespace(reliability=2))
    node=SimpleNamespace(get_subscriptions_info_by_topic=lambda _:[endpoint])
    with pytest.raises(ack.ExactAckFailure,match='RELIABLE_UPDATE_SUBSCRIPTION_REQUIRED'):
        ack.discover_reliable_updates(node,clock,1,100.1)
    assert clock.now==100.


def _atomic_message(stamp_ns=200, **overrides):
    import struct
    origin=SimpleNamespace(position=SimpleNamespace(x=0.,y=0.,z=0.),
                           orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
    meta=SimpleNamespace(layer='master',size_x=3,size_y=2,
                         resolution=struct.unpack('f',struct.pack('f',.05))[0],origin=origin)
    msg=SimpleNamespace(header=SimpleNamespace(frame_id='map',stamp=SimpleNamespace(sec=0,nanosec=stamp_ns)),
                        metadata=meta,data=bytearray([0,0,253,254,255,0]))
    for path,value in overrides.items():
        obj=msg;parts=path.split('.')
        for part in parts[:-1]:obj=getattr(obj,part)
        setattr(obj,parts[-1],value)
    return msg


def _atomic_buffer():
    return ack.AtomicReadbackBuffer(SimpleNamespace(width=3,height=2,resolution=.05,origin=(0.,0.,0.)))


def test_atomic_resolution_matches_exact_ros_float32_encoding():
    buffer=_atomic_buffer();message=_atomic_message()
    assert message.metadata.resolution!=.05
    buffer.push(message)
    assert buffer.consume() is not None
    import struct
    bits=struct.unpack('I',struct.pack('f',message.metadata.resolution))[0]
    message.metadata.resolution=struct.unpack('f',struct.pack('I',bits+1))[0]
    with pytest.raises(ack.ExactAckFailure,match='ATOMIC_READBACK_MAP_METADATA_MISMATCH'):
        buffer.push(message)


def test_old_latched_snapshot_cannot_ack_new_publication():
    buffer=_atomic_buffer();buffer.reset_floor(200)
    buffer.push(_atomic_message(199));buffer.push(_atomic_message(200))
    assert buffer.consume() is None
    buffer.push(_atomic_message(201))
    value=buffer.consume();assert value[1]==201
    buffer.reset_floor(300)
    buffer.push(_atomic_message(250))
    assert buffer.consume() is None


def test_duplicate_snapshot_is_not_two_matching_observations():
    buffer=_atomic_buffer();buffer.push(_atomic_message(201));buffer.push(_atomic_message(201))
    assert buffer.consume()[1]==201
    assert buffer.consume() is None
    buffer.push(_atomic_message(202));assert buffer.consume()[1]==202


def test_atomic_snapshot_copies_and_bounds_the_received_buffer():
    buffer=_atomic_buffer();message=_atomic_message(201);buffer.push(message)
    message.data[0]=254
    value=buffer.consume()[0]
    assert value[0,0]==0 and value.flags.writeable is False
    for t in range(202,250):buffer.push(_atomic_message(t))
    assert len(buffer.frames)==2
    assert buffer.consume()[1]==248 and buffer.consume()[1]==249
    assert buffer.consume() is None


@pytest.mark.parametrize('path,value',[
    ('header.frame_id','other'),('metadata.layer','static'),
    ('metadata.size_x',2),('metadata.size_y',3),('metadata.resolution',.1),
    ('metadata.origin.position.x',.05),('metadata.origin.position.y',.05),
    ('metadata.origin.position.z',.01),('metadata.origin.orientation.z',.1),
    ('metadata.origin.position.z',float('nan')),('metadata.origin.orientation.x',float('nan')),
    ('metadata.origin.orientation.w',0.),('data',bytearray([0,0])),
])
def test_atomic_readback_rejects_wrong_grid_metadata_or_bytes(path,value):
    buffer=_atomic_buffer()
    with pytest.raises(ack.ExactAckFailure,match='ATOMIC_READBACK_'):
        buffer.push(_atomic_message(201,**{path:value}))
    assert buffer.consume() is None


def test_missing_atomic_snapshot_fails_closed_before_action(monkeypatch,tmp_path):
    session,transport,clock=_fake_session(monkeypatch,tmp_path,[np.zeros((3,3),np.uint8)])
    session._atomic_buffer=_atomic_buffer()
    session._server_costmap_snapshot=ack.ExactAckSmacSession._server_costmap_snapshot.__get__(session)
    result=_plan(session)
    assert result.planner_success is False
    assert result.diagnostics['planner_search_started'] is False
    assert transport.actions==[]
    assert result.failure_code=='EXACT_ACK_FAILED_CLOSED'
    assert clock.now>=session.request_deadline


def test_chunk_publication_does_not_spend_remaining_budget_on_idle_callback_waits(monkeypatch):
    clock=SimpleNamespace(now=10.)
    monkeypatch.setattr(ack,'time',SimpleNamespace(monotonic=lambda:clock.now))
    sent=[]
    def spin_once(timeout_sec=0.):clock.now+=timeout_sec
    def now():return SimpleNamespace(nanoseconds=int(clock.now*1e9),to_msg=lambda:SimpleNamespace())
    session=object.__new__(ack.ExactAckSmacSession)
    session.request_deadline=clock.now+.0025
    session.client=SimpleNamespace(node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=now)),
                                   executor=SimpleNamespace(spin_once=spin_once))
    floors=[];session._atomic_buffer=SimpleNamespace(reset_floor=floors.append)
    session.OccupancyGridUpdate=lambda:SimpleNamespace(header=SimpleNamespace())
    session._local_update_publisher=SimpleNamespace(publish=sent.append)
    source=np.arange(4*700,dtype=np.int16).reshape(4,700)%101
    source=source.astype(np.int8)
    stats=session._publish_chunks(source,np.ones(source.shape,bool))
    assert stats['messages']==len(sent)==4
    replay=np.full(source.shape,-1,dtype=np.int8)
    for message in sent:
        replay[message.y:message.y+message.height,message.x:message.x+message.width]=np.asarray(message.data).reshape(message.height,message.width)
    np.testing.assert_array_equal(replay,source)
    assert floors==[10000000000]
    assert clock.now<session.request_deadline


@pytest.mark.parametrize('side',[1024,2048])
@pytest.mark.parametrize('cost',[0,254])
def test_equal_snapshot_scratch_memory_has_one_grid_bound(side,cost):
    """Large exact snapshots need no full-grid hard/soft/stale classification."""
    import tracemalloc
    expected=np.full((side,side),cost,np.uint8);server=expected.copy()
    token=ack.Publication(1,ack.grid_hash(expected),(0,0,side,side),
                          ack.grid_hash(expected),expected.shape,'map','request')
    tracemalloc.start()
    try:
        before=tracemalloc.get_traced_memory()[0];tracemalloc.reset_peak()
        result=ack.compare_exact(token,token,expected,server)
        peak=tracemalloc.get_traced_memory()[1]-before
    finally:tracemalloc.stop()
    assert result['acknowledged'] is True
    assert all(result[k]==0 for k in ['mismatch_cells','hard_mismatch','soft_mismatch',
                                    'stale_cells','hash_mismatch','sequence_mismatch'])
    # Includes the existing SHA-256 contiguous byte serialization.  A constant
    # allowance covers Python metadata; extra whole-grid masks exceed this bound.
    assert peak<=expected.nbytes*1.5+128*1024
    np.testing.assert_array_equal(server,expected)


@pytest.mark.parametrize("dtype", [np.uint8, np.int8, np.int16, np.float32])
@pytest.mark.parametrize("layout", ["C", "F", "reversed", "strided", "empty"])
def test_grid_hash_preserves_complete_c_order_bytes_for_all_layouts(dtype, layout):
    import hashlib
    grid=np.arange(120).astype(dtype).reshape(10,12)
    if layout=="F":grid=np.asfortranarray(grid)
    elif layout=="reversed":grid=grid[::-1,::-1]
    elif layout=="strided":grid=grid[::2,1::3]
    elif layout=="empty":grid=grid[:0]
    before=grid.copy()
    expected=hashlib.sha256(np.ascontiguousarray(grid).tobytes()).hexdigest()
    assert ack.grid_hash(grid)==expected
    np.testing.assert_array_equal(grid,before)


@pytest.mark.parametrize("side", [1024,2048])
def test_contiguous_grid_hash_does_not_allocate_another_grid(side):
    import hashlib
    import tracemalloc
    grid=np.full((side,side),254,dtype=np.uint8);grid.flags.writeable=False
    expected=hashlib.sha256(grid.tobytes()).hexdigest()
    tracemalloc.start()
    try:
        actual=ack.grid_hash(grid)
        _,peak=tracemalloc.get_traced_memory()
    finally:tracemalloc.stop()
    assert actual==expected and not grid.flags.writeable
    assert peak<=64*1024, (side,peak)


def test_native_snapshot_cadence_can_supply_two_fresh_frames_within_60ms(monkeypatch,tmp_path):
    """Nominal configured cadence, not a claim about actual ROS transport latency."""
    import sys
    import yaml
    monkeypatch.setenv("FASTRTPS_DEFAULT_PROFILES_FILE","test-placeholder")
    monkeypatch.setitem(sys.modules,"rclpy.qos",SimpleNamespace(
        QoSProfile=lambda **kwargs:SimpleNamespace(**kwargs),
        QoSReliabilityPolicy=SimpleNamespace(RELIABLE="reliable")))
    def base_init(session,**kwargs):
        assert kwargs['planner_parameter_overrides']['angle_quantization_bins']==48
        session.params_file=tmp_path/"params.yaml"
        session.params_file.write_text(yaml.safe_dump({"global_costmap":{"global_costmap":{"ros__parameters":{"publish_frequency":1.,"update_frequency":100.}}}}))
        session.ctx=SimpleNamespace(hospital_map=SimpleNamespace(width=3,height=2,resolution=.05,origin=(0.,0.,0.)))
    monkeypatch.setattr(ack.SmacSession,"__init__",base_init)
    session=ack.ExactAckSmacSession()
    configured=yaml.safe_load(session.params_file.read_text())["global_costmap"]["global_costmap"]["ros__parameters"]
    frequency=configured["publish_frequency"]
    assert 0<frequency<=configured["update_frequency"]
    clock=SimpleNamespace(now=.1,next_frame=.1+1./frequency)
    def spin_once(timeout_sec):
        clock.now+=timeout_sec
        while clock.next_frame<=clock.now+1e-12:
            session._receive_atomic_snapshot(_atomic_message(int(round(clock.next_frame*1e9))))
            clock.next_frame+=1./frequency
    monkeypatch.setattr(ack,"time",SimpleNamespace(monotonic=lambda:clock.now))
    session.client=SimpleNamespace(executor=SimpleNamespace(spin_once=spin_once))
    session._atomic_buffer.reset_floor(100000000)
    expected=np.array([0,0,253,254,255,0],dtype=np.uint8).reshape(2,3)
    publication=_publication(expected);stamps=[]
    for _ in range(2):
        frame,stamp=session._server_costmap_snapshot(.16)
        assert ack.compare_exact(publication,publication,expected,frame)["acknowledged"]
        stamps.append(stamp)
    assert 100000000<stamps[0]<stamps[1] and clock.now<=.16


@pytest.mark.parametrize("node", ["map_server","planner_server"])
def test_lifecycle_reply_profile_covers_bounded_response_reader_discovery(node):
    import xml.etree.ElementTree as ET
    from pathlib import Path
    root=ET.parse(Path(ack.__file__).resolve().parents[1]/"config/two_layer_v1_r3_transport.xml").getroot()
    ns={"d":"http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    profile=root.find(f"d:publisher[@profile_name='rr/{node}/change_stateReply']",ns)
    assert profile is not None, "Default100ms discovery race drops the lifecycle response"
    reliability=profile.find("d:qos/d:reliability",ns)
    assert reliability.find("d:kind",ns).text=="RELIABLE"
    duration=reliability.find("d:max_blocking_time",ns)
    seconds=int(duration.find("d:sec",ns).text)+int(duration.find("d:nanosec",ns).text)/1e9
    assert .25<=seconds<=2., "Allow delayed discovery but retain a finite startup bound"


def test_lifecycle_discovery_profile_is_scoped_away_from_online_services():
    import xml.etree.ElementTree as ET
    from pathlib import Path
    root=ET.parse(Path(ack.__file__).resolve().parents[1]/"config/two_layer_v1_r3_transport.xml").getroot()
    ns={"d":"http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    allowed={"rr/map_server/change_stateReply","rr/planner_server/change_stateReply"}
    publishers=root.findall("d:publisher",ns)
    assert all(p.attrib=={"profile_name":p.attrib["profile_name"]} and p.attrib["profile_name"] in allowed for p in publishers)
    subscribers=root.findall("d:subscriber",ns)
    assert len(subscribers)==1 and subscribers[0].attrib=={"profile_name":"/map_updates"}
    assert subscribers[0].find("d:topic/d:historyQos/d:depth",ns).text=="512"
    assert subscribers[0].find("d:qos/d:reliability/d:kind",ns).text=="RELIABLE"
