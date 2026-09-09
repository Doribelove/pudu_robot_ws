from __future__ import annotations

import json

import numpy as np

from arena_evaluation.semantic_transition_online_adapter import (
    CONTRACT_ID,
    PROTOCOL_ID,
    _load_points,
    _resample_points,
    _semantic_metrics,
)


def test_load_points_preserves_exact_se2_and_adds_adapter_metadata(tmp_path):
    path = tmp_path / "path.json"
    path.write_text(json.dumps([{"x": 1, "y": 2, "yaw": 0.3}]))
    points = _load_points(path)
    assert points[0]["x"] == 1.0
    assert points[0]["yaw"] == 0.3
    assert points[0]["motion_direction"] == "forward"
    assert points[0]["planner_backend"] == "semantic_transition_reference"


def test_contract_and_protocol_are_explicit():
    assert PROTOCOL_ID.startswith("PLN-02-")
    assert CONTRACT_ID == "endpoint-transition-each-side-6m-diagnostic-candidate"


def test_fixed_arc_resampling_does_not_depend_on_input_density():
    sparse = [{"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": 1.0, "y": 0.0, "yaw": 0.0}]
    dense = [{"x": float(x), "y": 0.0, "yaw": 0.0} for x in np.linspace(0.0, 1.0, 101)]
    sampled_sparse, stations_sparse = _resample_points(sparse, 0.025)
    sampled_dense, stations_dense = _resample_points(dense, 0.025)
    assert sampled_sparse == sampled_dense
    assert np.array_equal(stations_sparse, stations_dense)
    assert stations_sparse[-1] == 1.0


def test_short_or_zero_active_length_is_invalid_before_semantic_lookup():
    for length in (10.553370567893, 12.0):
        path = [{"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": length, "y": 0.0, "yaw": 0.0}]
        metrics = _semantic_metrics(None, path, 6.0)
        assert metrics["contract_metric_status"] == "INVALID_CONTRACT_METRIC"
        assert not metrics["semantic_gate_passed"]
        assert metrics["correct_side_ratio"] is None
