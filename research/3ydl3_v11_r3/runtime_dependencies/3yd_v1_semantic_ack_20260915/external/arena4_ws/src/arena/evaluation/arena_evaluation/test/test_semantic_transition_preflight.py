import json

import pytest

from arena_evaluation.semantic_constraint_core import dubins_edge
from arena_evaluation.semantic_transition_preflight import REQUIRED, aggregate, replay_controls


def test_empty_missing_duplicate_or_reordered_queries_cannot_pass():
    good = [{"query_id": query, "offline_gate_passed": True} for query in REQUIRED]
    for rows in ([], good[:2], good + good[:1], good[::-1]):
        gate = aggregate(rows)
        assert gate["online_eligible"] is False
        assert gate["promotion_gate_passed"] is False


def test_short_negative_blocks_online_and_selected8():
    rows = [{"query_id": query, "offline_gate_passed": query != REQUIRED[1]} for query in REQUIRED]
    gate = aggregate(rows)
    assert gate["offline_pass_count"] == 2
    assert gate["grade"] == "C1"
    assert gate["online_planner"] == "NOT_RUN_OFFLINE_GATE_FAILED"
    assert gate["selected8"] == "NOT_RUN"


def test_offline_success_alone_never_promotes():
    gate = aggregate([{"query_id": query, "offline_gate_passed": True} for query in REQUIRED])
    assert gate["online_eligible"] is True
    assert gate["promotion_gate_passed"] is False
    assert gate["exact_server_ack"] == "NOT_APPLICABLE_OFFLINE"


def test_control_replay_detects_altered_path(tmp_path):
    edge = dubins_edge((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    points = [dict(zip(("x", "y", "yaw"), pose)) for pose in [edge.start, *edge.samples]]
    path = tmp_path / "path.json"
    certificate = tmp_path / "certificate.json"
    path.write_text(json.dumps(points))
    certificate.write_text(json.dumps({"edges": [edge.certificate()]}))
    assert replay_controls(path, certificate, "certificate")["control_replay_passed"]
    points[-1]["x"] += 0.001
    path.write_text(json.dumps(points))
    assert not replay_controls(path, certificate, "certificate")["control_replay_passed"]
    with pytest.raises(ValueError, match="unsupported"):
        replay_controls(path, certificate, "unknown")
