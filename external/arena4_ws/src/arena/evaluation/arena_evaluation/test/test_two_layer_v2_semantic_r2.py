from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np
import pytest

from arena_evaluation.regional_preference_r2 import (
    RegionalPreferenceBuilderR2,
    classify_semantic_query_feasibility,
)
from arena_evaluation.semantic_costmap_composer import SemanticCostmap
from arena_evaluation.semantic_costmap_r2 import (
    SemanticCostmapComposerR2,
    pinned_nav2_effective_master,
)
from arena_evaluation.semantic_smac_session import SemanticSmacSession
from arena_evaluation.semantic_smac_session_r2 import ExactSemanticSmacSessionR2
from test_two_layer_v2_semantic_r1 import cell, fixture


def test_pinned_effective_mapping_has_frozen_multi_source_result():
    source = np.zeros((7, 9), dtype=np.uint8)
    source[1, 1] = 254
    source[5, 7] = 254
    source[3, 4] = 255
    actual = pinned_nav2_effective_master(
        source, resolution=0.05, inflation_radius_m=0.2,
        cost_scaling_factor=3.0, inscribed_radius_m=0.075,
    )
    expected = np.asarray([
        [253, 253, 253, 225, 196, 0, 0, 0, 0],
        [253, 254, 253, 233, 201, 173, 0, 173, 0],
        [253, 253, 253, 225, 196, 183, 196, 201, 196],
        [225, 233, 225, 206, 255, 206, 225, 233, 225],
        [196, 201, 196, 183, 196, 225, 253, 253, 253],
        [0, 173, 0, 173, 201, 233, 253, 254, 253],
        [0, 0, 0, 0, 196, 225, 253, 253, 253],
    ], dtype=np.uint8)
    assert np.array_equal(actual, expected)


def test_r2_mirror_geometry_is_lane_instance_local_and_not_cross_contaminated():
    hospital_map, semantic_map, raster = fixture()
    route = [[0.8, 2.5], [3.8, 2.5], [5.0, 2.5], [8.0, 2.5]]
    allowed = hospital_map.occupancy == 0
    builder = RegionalPreferenceBuilderR2(
        hospital_map, raster, semantic_map=semantic_map,
    )
    forward = builder.build(route, goal=route[-1], allowed_mask=allowed)
    reverse = builder.build(list(reversed(route)), goal=route[0], allowed_mask=allowed)
    south = cell(hospital_map, 2.0, 1.4)
    north = cell(hospital_map, 2.0, 3.6)
    assert forward.lane_correct_side[south] and not forward.lane_correct_side[north]
    assert reverse.lane_correct_side[north] and not reverse.lane_correct_side[south]
    assert forward.lane_distance_to_right_m[south] == pytest.approx(
        reverse.lane_distance_to_left_m[south], abs=0.051,
    )
    assert forward.lane_distance_to_left_m[north] == pytest.approx(
        reverse.lane_distance_to_right_m[north], abs=0.051,
    )
    assert forward.diagnostics["lane_instance_propagation"] == "same_semantic_feature_only"
    assert int(forward.lane_instance_id[south]) == int(reverse.lane_instance_id[south]) > 0


def test_r2_query_validity_is_algorithm_independent_and_fail_closed():
    hospital_map, semantic_map, raster = fixture()
    route = [[0.8, 2.5], [8.0, 2.5]]
    allowed = hospital_map.occupancy == 0
    field = RegionalPreferenceBuilderR2(
        hospital_map, raster, semantic_map=semantic_map,
    ).build(route, goal=route[-1], allowed_mask=allowed)
    valid = classify_semantic_query_feasibility(
        hospital_map, raster, field, allowed, allowed,
        (*route[0], 0.0), (*route[-1], 0.0),
    )
    assert valid["classification"] == "VALID"
    unsafe = allowed.copy()
    unsafe[cell(hospital_map, *route[-1])] = False
    invalid = classify_semantic_query_feasibility(
        hospital_map, raster, field, allowed, unsafe,
        (*route[0], 0.0), (*route[-1], 0.0),
    )
    assert invalid["classification"] == "SEMANTIC_QUERY_INFEASIBLE"
    assert invalid["query_valid"] is False


def test_r2_composer_is_source_exact_and_inflation_lru_is_bounded():
    hospital_map, semantic_map, raster = fixture()
    route = [[0.8, 2.5], [8.0, 2.5]]
    allowed = hospital_map.occupancy == 0
    field = RegionalPreferenceBuilderR2(
        hospital_map, raster, semantic_map=semantic_map,
    ).build(route, goal=route[-1], allowed_mask=allowed)
    composer = SemanticCostmapComposerR2(
        policy={"class_soft_cost": {"speed_bumps": 72, "parking_area": 4}},
        inflation_cache_capacity=1,
    )
    first = composer.compose(hospital_map.occupancy, raster, field, allowed_mask=allowed)
    second = composer.compose(hospital_map.occupancy, raster, field, allowed_mask=allowed)
    assert np.array_equal(first.occupancy_grid, second.occupancy_grid)
    assert np.array_equal(first.expected_master_cost, second.expected_master_cost)
    assert second.diagnostics["base_geometry_cache_hit"] is True
    assert second.diagnostics["inflation_template_cache_hit"] is True
    restricted = allowed.copy()
    restricted[:, restricted.shape[1] // 2] = False
    third = composer.compose(hospital_map.occupancy, raster, field, allowed_mask=restricted)
    assert third.diagnostics["inflation_cache_entries"] == 1
    assert third.diagnostics["inflation_cache_evictions"] == 1
    assert composer.resident_cache_bytes > 0


def _semantic(master: np.ndarray, occupancy: np.ndarray | None = None) -> SemanticCostmap:
    occupancy = np.asarray(master if occupancy is None else occupancy, dtype=np.int8)
    soft = np.zeros(master.shape, dtype=np.uint8)
    soft[master == 17] = 17
    return SemanticCostmap(
        internal_cost=np.asarray(master, dtype=np.uint8), occupancy_grid=occupancy,
        expected_master_cost=np.asarray(master, dtype=np.uint8), soft_cost=soft,
        hard_semantic_mask=np.asarray(master == 254), affected_mask=np.ones(master.shape, bool),
        policy_hash="policy", semantic_map_hash="map", preference_field_hash="field",
        semantics_enabled=True,
    )


def _ack_session(tmp_path: Path, semantic: SemanticCostmap, server: np.ndarray):
    session = ExactSemanticSmacSessionR2.__new__(ExactSemanticSmacSessionR2)
    session._semantic_costmap = semantic
    session._semantic_publication_sequence = 7
    session._active_publication_bbox = (0, 0, server.shape[1], server.shape[0])
    session._active_publication_baseline_timestamp_ns = 1
    session._last_exact_expected_master = np.full(server.shape, 254, np.uint8)
    session._last_exact_signature = None
    session._last_exact_ack = None
    session._exact_ack_cache = {}
    session.exact_stable_observations = 2
    session.costmap_ack_timeout_s = 0.003
    session._last_server_update_time_ns = 1
    session._costmap_ack_sequence = 0
    session._semantic_roi_sequence = 0
    session._semantic_ack_cache = {}
    session._semantic_output = tmp_path
    session.current_query_id = "E4:test"
    session.client = None
    session._costmap_state_trusted = True
    session._force_full_next_update = False
    session._server_costmap_snapshot = lambda deadline: (server.copy(), 2)
    return session


def test_exact_ack_accepts_only_exact_effective_master(tmp_path):
    top_down = np.asarray([[0, 17], [254, 0]], dtype=np.uint8)
    semantic = _semantic(top_down, np.asarray([[0, 7], [100, 0]], dtype=np.int8))
    expected_server = np.flipud(top_down)
    expected_occupancy_server = np.flipud(semantic.occupancy_grid)
    session = _ack_session(tmp_path, semantic, expected_server)
    result = session._wait_for_costmap_ack(
        expected_occupancy_server, np.ones(top_down.shape, bool), timeout_s=0.01,
    )
    assert result["costmap_update_acknowledged"] is True
    assert result["costmap_ack_soft_exact_mismatch_cells"] == 0
    assert result["costmap_ack_stale_roi_cells"] == 0
    assert result["costmap_ack_hash_mismatch"] == 0
    assert result["costmap_ack_sequence_mismatch"] == 0
    assert result["semantic_publication_sequence"] == 7
    assert len(session._exact_ack_cache) == 1


def test_exact_ack_rejects_value_error_and_reports_old_new_stale_cell(tmp_path):
    top_down = np.asarray([[0, 17], [254, 0]], dtype=np.uint8)
    semantic = _semantic(top_down, np.asarray([[0, 7], [100, 0]], dtype=np.int8))
    server = np.flipud(top_down).copy()
    # This cell was lethal in the prior master and must be cleared to zero in
    # the new publication; retaining 254 is an old/new dirty-ROI stale value.
    server[0, 1] = 254
    session = _ack_session(tmp_path, semantic, server)
    result = session._wait_for_costmap_ack(
        np.flipud(semantic.occupancy_grid), np.ones(top_down.shape, bool), timeout_s=0.002,
    )
    assert result["costmap_update_acknowledged"] is False
    assert result["costmap_ack_mismatch_cells"] == 1
    assert result["costmap_ack_stale_roi_cells"] == 1
    assert result["costmap_ack_hash_mismatch"] == 1


def test_noop_ack_requires_complete_exact_key(monkeypatch, tmp_path):
    master = np.zeros((2, 2), dtype=np.uint8)
    semantic = _semantic(master)
    session = _ack_session(tmp_path, semantic, master)
    source_hash = hashlib.sha256(
        np.ascontiguousarray(np.flipud(semantic.occupancy_grid)).tobytes()
    ).hexdigest()
    evidence = {
        "semantic_source_grid_hash": source_hash,
        "semantic_expected_master_hash": semantic.expected_master_hash,
        "semantic_policy_hash": semantic.policy_hash,
        "costmap_ack_semantics": "exact_effective_master",
        "costmap_ack_hard_mismatch_cells": 0,
        "costmap_ack_soft_exact_mismatch_cells": 0,
        "costmap_ack_stale_roi_cells": 0,
        "semantic_publication_sequence": 7,
        "semantic_ack_roi_bbox": [0, 0, 2, 2],
        "server_costmap_content_hash": "server",
    }
    session._last_exact_signature = session._content_signature(semantic)
    session._last_exact_ack = evidence
    complete_key = session._exact_cache_key(
        source_grid_hash=source_hash,
        expected_master_hash=semantic.expected_master_hash,
        policy_hash=semantic.policy_hash,
        publication_sequence=7, roi_bbox=(0, 0, 2, 2),
        server_content_hash="server",
    )
    session._exact_ack_cache = {complete_key: evidence}
    session.enable_mask_reuse_noop = True
    monkeypatch.setattr(
        SemanticSmacSession, "update_local_mask",
        lambda self, allowed_mask, **kwargs: {"local_map_update_mode": "reuse_noop"},
    )
    reused = session.update_local_mask(np.ones((2, 2), bool))
    assert reused["semantic_noop_complete_key_verified"] is True
    assert reused["costmap_ack_status"] == "reused_exact_effective_content_ack"
    semantic.policy_hash = "changed-policy"
    with pytest.raises(RuntimeError, match="evidence missing"):
        session.update_local_mask(np.ones((2, 2), bool))


def test_exact_ack_rejects_concurrent_publication_sequence_change(tmp_path):
    top_down = np.asarray([[0, 17], [254, 0]], dtype=np.uint8)
    semantic = _semantic(top_down, np.asarray([[0, 7], [100, 0]], dtype=np.int8))
    server = np.flipud(top_down)
    session = _ack_session(tmp_path, semantic, server)

    def concurrent_snapshot(deadline):
        session._semantic_publication_sequence = 8
        return server.copy(), 2

    session._server_costmap_snapshot = concurrent_snapshot
    result = session._wait_for_costmap_ack(
        np.flipud(semantic.occupancy_grid), np.ones(top_down.shape, bool), timeout_s=0.002,
    )
    assert result["costmap_update_acknowledged"] is False
    assert result["costmap_ack_mismatch_cells"] == 0
    assert result["costmap_ack_hash_mismatch"] == 0
    assert result["costmap_ack_sequence_mismatch"] == 1


def test_exact_ack_key_is_sensitive_to_every_binding_field(tmp_path):
    session = ExactSemanticSmacSessionR2.__new__(ExactSemanticSmacSessionR2)
    base = dict(
        source_grid_hash="source", expected_master_hash="master",
        policy_hash="policy", publication_sequence=1,
        roi_bbox=(1, 2, 3, 4), server_content_hash="server",
    )
    key = session._exact_cache_key(**base)
    variants = []
    for name, value in (
        ("source_grid_hash", "source-2"), ("expected_master_hash", "master-2"),
        ("policy_hash", "policy-2"), ("publication_sequence", 2),
        ("roi_bbox", (1, 2, 3, 5)), ("server_content_hash", "server-2"),
    ):
        changed = {**base, name: value}
        variants.append(session._exact_cache_key(**changed))
    assert all(value != key for value in variants)
    assert len(set(variants)) == len(variants)
