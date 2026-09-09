from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from arena_evaluation import semantic_parking_reference_v3 as parking
from arena_evaluation import semantic_static_cache_v3 as cache
from arena_evaluation import semantic_v3_ack_r14 as ack
from arena_evaluation import two_layer_v3_semantic_r13_online as parent_online
from arena_evaluation import two_layer_v3_semantic_r14_benchmark as offline
from arena_evaluation import two_layer_v3_semantic_r14_expanded as expanded
from arena_evaluation.semantic_map import canonical_hash, sha256_file
from arena_evaluation.semantic_rasterizer import grid_hash


PACKAGE_ROOT = Path(expanded.__file__).resolve().parents[1]


def test_r14_identity_and_frozen_motion_contract():
    config, _parent_algorithm, _parent, _selected = offline._load_config()
    assert offline.ARCHITECTURE_ID == "2A-V3"
    assert offline.IMPLEMENTATION_REVISION == "r14-parking-dispatch-ack-cache"
    assert config["status"] in {
        "calibration_after_formal_v2_ack_lifecycle_failure",
        "frozen_for_formal_evaluation",
    }
    assert config["frozen_bindings"] | {
        "yaw_bins": 48,
        "motion_model": "DUBIN",
        "allow_reverse": False,
        "allow_in_place_rotation": False,
        "minimum_turning_radius_m": .40,
        "maximum_curvature_1pm": 2.50,
    } == config["frozen_bindings"]


def test_parking_policy_cannot_relax_curvature_and_astar_forbids_corner_cut():
    with pytest.raises(ValueError, match="cannot relax"):
        parking.ParkingReferencePolicy(maximum_reference_curvature_1pm=2.51)
    traversable = np.asarray([[1, 0], [0, 1]], dtype=bool)
    path, _expanded = parking._astar(
        traversable, np.zeros((2, 2), np.float32), (0, 0), (1, 1), 20,
    )
    assert path is None


def test_parking_reference_curvature_measure_distinguishes_line_and_corner():
    assert parking._maximum_curvature([(0.0, 0.0), (2.0, 0.0)]) == 0.0
    assert parking._maximum_curvature(
        [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], spacing=.1,
    ) > 2.5


def test_bounded_lru_enforces_count_bytes_and_deterministic_eviction():
    values = cache.BoundedLRU(capacity=2, maximum_bytes=5)
    assert values.put("a", 1, 2)
    assert values.put("b", 2, 2)
    assert values.get("a") == 1
    assert values.put("c", 3, 2)
    assert values.get("b") is None
    assert values.get("a") == 1
    assert values.get("c") == 3
    assert values.active_count == 2
    assert values.resident_bytes == 4
    assert values.evictions == 1
    assert values.put("oversize", 4, 6) is False


def test_static_cache_missing_manifest_fails_closed(tmp_path):
    with pytest.raises(ValueError, match="CACHE_MANIFEST_MISSING"):
        cache._load(tmp_path, expected_key="bound")


def test_balanced_pair_order_is_exactly_balanced_per_32_query_repetition():
    for repetition in range(1, 6):
        first = [
            expanded._balanced_pair_order(
                ("E0", "ADAPTIVE"), repetition=repetition, query_index=index,
            )[0]
            for index in range(32)
        ]
        assert first.count("E0") == 16
        assert first.count("ADAPTIVE") == 16
    assert expanded._balanced_pair_order(
        ("E0",), repetition=1, query_index=0,
    ) == ("E0",)


def test_json_evidence_replaces_non_finite_values_without_mutating_booleans():
    assert expanded._json_safe({"x": np.inf, "y": np.float32(1.5), "z": np.bool_(True)}) == {
        "x": None, "y": 1.5, "z": True,
    }


def test_deterministic_ack_orders_roi_clear_full_source_then_readback(monkeypatch):
    order = []
    session = object.__new__(ack.DeterministicReinflationSessionR14)
    session._r14_roi_publication_pending = True
    session._r14_ack_server_snapshot = np.zeros((3, 4), np.uint8)
    session._semantic_publication_sequence = 7
    session._clear_global_costmap = lambda: order.append("clear") or 1.25

    def publish(_self, values, *, clear_costmap=True):
        order.append(("full_source", clear_costmap, int(np.asarray(values).size)))
        return 2.5

    def observe(_self, expected, changed, *, timeout_s=None):
        order.append(("exact_readback", timeout_s, int(np.asarray(changed).sum())))
        return {
            "costmap_update_acknowledged": True,
            "server_costmap_content_hash": grid_hash(np.zeros((3, 4), np.uint8)),
            "semantic_publication_sequence": 7,
        }

    monkeypatch.setattr(parent_online.RoutePhaseV3PlannerSession, "_publish_full_grid", publish)
    monkeypatch.setattr(parent_online.RoutePhaseV3PlannerSession, "_wait_for_costmap_ack", observe)
    result = session._wait_for_costmap_ack(
        np.zeros((3, 4), np.int8), np.ones((3, 4), bool), timeout_s=3.0,
    )
    assert order == ["clear", ("full_source", False, 12), ("exact_readback", 3.0, 12)]
    assert result["deterministic_reinflation_reset_count"] == 1
    assert result["deterministic_reinflation_replay_scope"] == "FULL_CURRENT_SOURCE_GRID"
    assert result["deterministic_reinflation_replay_cells"] == 12


def test_deterministic_ack_mismatch_fails_closed_without_timeout_repair(monkeypatch):
    session = object.__new__(ack.DeterministicReinflationSessionR14)
    session._r14_roi_publication_pending = False
    session._costmap_state_trusted = True
    session._force_full_next_update = False
    monkeypatch.setattr(
        parent_online.RoutePhaseV3PlannerSession,
        "_wait_for_costmap_ack",
        lambda *_args, **_kwargs: {"costmap_update_acknowledged": False},
    )
    with pytest.raises(RuntimeError, match="FAILED_CLOSED"):
        session._wait_for_costmap_ack(np.zeros((1, 1)), np.zeros((1, 1), bool))
    assert session._costmap_state_trusted is False
    assert session._force_full_next_update is True


def test_verified_master_uses_exact_ack_proof_without_second_service_read():
    session = object.__new__(ack.DeterministicReinflationSessionR14)
    expected_top = np.arange(12, dtype=np.uint8).reshape(3, 4)
    expected_server = np.flipud(expected_top)
    session._semantic_costmap = SimpleNamespace(
        expected_master_cost=expected_top,
        expected_master_hash=grid_hash(expected_top),
    )
    session._semantic_publication_sequence = 9
    session._server_costmap_snapshot = lambda _deadline: pytest.fail(
        "post-ACK GetCostmap must not be called"
    )
    proof = {
        "costmap_update_acknowledged": True,
        "costmap_ack_semantics": "exact_effective_master",
        "costmap_ack_hard_mismatch_cells": 0,
        "costmap_ack_soft_exact_mismatch_cells": 0,
        "costmap_ack_stale_roi_cells": 0,
        "costmap_ack_hash_mismatch": 0,
        "costmap_ack_sequence_mismatch": 0,
        "semantic_publication_sequence": 9,
        "server_costmap_content_hash": grid_hash(expected_server),
        "semantic_expected_server_content_hash": grid_hash(expected_server),
        "server_costmap_update_time_ns": 123,
    }
    observed, telemetry = session.verified_master_snapshot(proof)
    assert np.array_equal(observed, expected_top)
    assert telemetry["post_ack_get_costmap_requests"] == 0
    assert telemetry["verified_master_snapshot_source"] == (
        "EXPECTED_MASTER_PROVEN_EQUAL_BY_EXACT_ACK"
    )


def test_expanded_query_set_and_algorithm_are_content_bound_and_input_only():
    config = yaml.safe_load(expanded.DEFAULT_CONFIG.read_text())
    algorithm_path = PACKAGE_ROOT / "config" / config["algorithm_config"]["path"]
    query_path = PACKAGE_ROOT / "config" / config["query_set"]["path"]
    assert config["status"] in {
        "calibration_after_formal_v2_ack_lifecycle_failure",
        "frozen_for_formal_evaluation",
    }
    assert sha256_file(algorithm_path) == config["algorithm_config"]["sha256"]
    assert sha256_file(query_path) == config["query_set"]["file_sha256"]
    query_set = yaml.safe_load(query_path.read_text())
    assert query_set["selection_used_planner_outcome"] is False
    assert len(query_set["queries"]) == 32
    assert canonical_hash([
        {key: query[key] for key in ("query_id", "start", "goal", "category")}
        for query in query_set["queries"]
    ]) == config["query_set"]["query_hash"]


def test_formal_resource_and_tail_gates_are_frozen_before_measurement():
    config, _parent_algorithm, _parent, _selected = offline._load_config()
    gates = config["gates"]
    assert gates["measured_samples_per_arm_min"] >= 100
    assert gates["static_prepare_ms_max"] == 1000.0
    assert gates["peak_rss_bytes_max"] == 1_800_000_000
    assert gates["steady_rss_growth_bytes_max"] == 128 * 1024 * 1024
    assert gates["adaptive_p95_ms_max"] == 5000.0
    assert gates["adaptive_p99_ms_max"] == 6000.0
