import math
import json
import time
from types import SimpleNamespace

import numpy as np

from arena_evaluation.endpoint_heading import sample_dubins
from arena_evaluation.path_audit import PathAuditor
from arena_evaluation import unified_four_backends_smoke as legacy
from arena_evaluation import l1_l3_corridor_hybrid_smoke as hybrid
from arena_evaluation import two_layer_v1_r2_a2b19_ablation as ablation
from arena_evaluation import two_layer_v1_r2_endpoint_diagnostics as endpoint_diagnostics
from arena_evaluation import two_layer_v1_r2_roi_pathaudit_benchmark as benchmark


class _AuditMap:
    resolution = 0.05
    width = 100
    height = 100
    origin = (0.0, 0.0, 0.0)

    def __init__(self, *, collision_x=None):
        self.occupancy = np.zeros((self.height, self.width), dtype=np.int8)
        self.distance_m = np.full((self.height, self.width), 10.0, dtype=float)
        self.collision_x = collision_x
        if collision_x is not None:
            # Force the vectorized safe-cell filter to run the exact legacy
            # predicate around the synthetic collision.
            column = int(float(collision_x) / self.resolution)
            row = self.height - 1 - int(2.0 / self.resolution)
            self.occupancy[row, column] = -1

    def world_to_cell(self, x, y):
        column = math.floor(float(x) / self.resolution)
        row = self.height - 1 - math.floor(float(y) / self.resolution)
        if 0 <= row < self.height and 0 <= column < self.width:
            return row, column
        return None

    def footprint_collision(self, pose, _footprint, unknown_is_collision=True):
        del unknown_is_collision
        return self.collision_x is not None and abs(float(pose[0]) - float(self.collision_x)) < 0.03

    def clearance(self, x, y):
        cell = self.world_to_cell(x, y)
        return None if cell is None else float(self.distance_m[cell])


def _points():
    points = []
    for index in range(11):
        points.append({
            "x": 0.5 + 0.1 * index,
            "y": 2.0,
            "yaw": 0.0,
            "source": "l3_hybrid_smac",
            "motion_direction": "forward",
            "steering": 0.0,
            "planner_backend": "nav2_smac_hybrid",
            "backend_version": "test",
            "source_commit": "commit",
            "path_hash": "placeholder",
        })
    digest = legacy._path_hash(points)
    for point in points:
        point["path_hash"] = digest
    return points


def test_dubins_sampler_reaches_pose_and_keeps_radius():
    cases = [
        ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (1.0, 1.0, math.pi / 2.0)),
        ((1.0, 2.0, 1.1), (-0.5, 3.0, -2.2)),
    ]
    for start, goal in cases:
        result = sample_dubins(start, goal, radius_m=0.4, spacing_m=0.05)
        assert result is not None
        poses, length, word = result
        assert len(poses) >= 2
        assert length > 0.0
        assert len(word) == 3
        assert math.hypot(poses[-1][0] - goal[0], poses[-1][1] - goal[1]) < 1.0e-9
        assert abs(legacy._delta(poses[-1][2], goal[2])) < 1.0e-9
        for first, second, third in zip(poses, poses[1:], poses[2:]):
            annotated = [
                {"x": first[0], "y": first[1]},
                {"x": second[0], "y": second[1]},
                {"x": third[0], "y": third[1]},
            ]
            assert legacy._curvature(*annotated) <= 2.5 + 1.0e-2


def test_canonical_audit_matches_frozen_validator_on_valid_path():
    hospital_map = _AuditMap()
    ctx = SimpleNamespace(hospital_map=hospital_map)
    query = SimpleNamespace(start=[0.5, 2.0, 0.0], goal=[1.5, 2.0, 0.0])
    points = _points()
    expected = legacy.validate_path(ctx, query, points)
    result = PathAuditor(ctx, source_commit="commit").audit(
        query, points, np.ones((hospital_map.height, hospital_map.width), dtype=bool),
    )
    for field in (
        "static_footprint_valid", "kinematic_valid", "failure_code",
        "path_length_m", "maximum_curvature", "reverse_distance_m",
    ):
        assert result.metrics[field] == expected[field]
    assert result.final_valid_success is True
    assert result.sampled_pose_count > len(points)
    assert result.timings["canonical_path_audit_ms"] >= 0.0


def test_canonical_audit_preserves_unknown_collision_semantics():
    hospital_map = _AuditMap(collision_x=1.0)
    ctx = SimpleNamespace(hospital_map=hospital_map)
    query = SimpleNamespace(start=[0.5, 2.0, 0.0], goal=[1.5, 2.0, 0.0])
    points = _points()
    expected = legacy.validate_path(ctx, query, points)
    result = PathAuditor(ctx, source_commit="commit").audit(
        query, points, np.ones((hospital_map.height, hospital_map.width), dtype=bool),
    )
    assert expected["failure_code"] == "STATIC_FOOTPRINT_COLLISION"
    assert result.metrics["failure_code"] == expected["failure_code"]
    assert result.metrics["static_footprint_valid"] is False
    assert result.exact_footprint_check_count > 0


class _Header:
    def __init__(self):
        self.frame_id = ""
        self.stamp = None


class _Update:
    def __init__(self):
        self.header = _Header()
        self.x = self.y = self.width = self.height = 0
        self.data = []


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_roi_contains_both_closed_and_opened_cells():
    session = object.__new__(legacy.SmacSession)
    session._current_grid = np.full((10, 12), 100, dtype=np.int8)
    session._current_grid[2, 2] = 0
    expected = session._current_grid.copy()
    expected[2, 2] = 100  # close old corridor
    expected[8, 10] = 0   # open new corridor
    changed = session._current_grid != expected
    session.OccupancyGridUpdate = _Update
    session._local_update_publisher = _Publisher()
    session.client = SimpleNamespace(
        node=SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "stamp")),
        ),
    )
    applied, timing = session._publish_dirty_roi(expected, changed)
    assert np.array_equal(applied, expected)
    assert timing["roi_bbox"] == [2, 2, 9, 7]
    assert timing["roi_changed_cells"] == 2
    assert timing["roi_published_cells"] == 63
    assert len(session._local_update_publisher.messages) == 1


def test_server_ack_checks_server_content_not_client_hash():
    session = object.__new__(legacy.SmacSession)
    session.costmap_ack_timeout_s = 0.05
    session._last_server_update_time_ns = -1
    session._costmap_ack_sequence = 0
    session.client = None
    expected = np.full((4, 5), 100, dtype=np.int8)
    expected[1, 3] = 0
    changed = np.zeros_like(expected, dtype=bool)
    changed[0, 0] = True
    changed[1, 3] = True
    server = np.zeros_like(expected, dtype=np.uint8)
    server[0, 0] = 254
    server[1, 3] = 253
    session._server_costmap_snapshot = lambda _deadline: (server, 1234)
    info = session._wait_for_costmap_ack(expected, changed)
    assert info["costmap_update_acknowledged"] is True
    assert info["costmap_ack_checked_cells"] == 2
    assert info["costmap_ack_sequence"] == 1


def test_roi_tiles_are_idempotent_under_reordering_and_duplication():
    session = object.__new__(legacy.SmacSession)
    session._current_grid = np.full((200, 20), 100, dtype=np.int8)
    expected = session._current_grid.copy()
    expected[1, 1] = 0
    expected[198, 18] = 0
    changed = session._current_grid != expected
    session.roi_max_payload_bytes = 1024
    session.roi_publish_pacing_s = 0.0
    session.OccupancyGridUpdate = _Update
    session._local_update_publisher = _Publisher()
    session.client = SimpleNamespace(
        node=SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "stamp")),
        ),
    )
    applied, timing = session._publish_dirty_roi(expected, changed)
    messages = session._local_update_publisher.messages
    assert timing["roi_message_count"] > 1

    server = session._current_grid.copy()
    for message in [*reversed(messages), messages[0]]:
        patch = np.asarray(message.data, dtype=np.int8).reshape(message.height, message.width)
        server[
            message.y:message.y + message.height,
            message.x:message.x + message.width,
        ] = patch
    assert np.array_equal(server, applied)
    assert np.array_equal(server, expected)


def test_server_ack_repairs_a_dropped_tile_before_success():
    session = object.__new__(legacy.SmacSession)
    session.costmap_ack_timeout_s = 0.2
    session._last_server_update_time_ns = -1
    session._costmap_ack_sequence = 0
    session.client = SimpleNamespace(
        executor=SimpleNamespace(spin_once=lambda timeout_sec: time.sleep(timeout_sec)),
    )
    expected = np.full((4, 5), 100, dtype=np.int8)
    expected[1, 3] = 0
    changed = np.zeros_like(expected, dtype=bool)
    changed[1, 3] = True
    state = {"repaired": False, "calls": 0}

    def snapshot(_deadline):
        state["calls"] += 1
        server = np.full(expected.shape, 254, dtype=np.uint8)
        server[1, 3] = 0 if state["repaired"] else 254
        return server, state["calls"]

    def repair(_expected, mismatch_mask):
        assert np.count_nonzero(mismatch_mask) == 1
        state["repaired"] = True
        return expected.copy(), {
            "roi_message_count": 1,
            "roi_published_cells": 1,
            "local_map_serialization_ms": 0.1,
            "local_map_publication_ms": 0.2,
        }

    session._server_costmap_snapshot = snapshot
    session._publish_dirty_roi = repair
    info = session._wait_for_costmap_ack(expected, changed)
    assert info["costmap_update_acknowledged"] is True
    assert info["costmap_ack_status"] == "server_content_verified"
    assert info["costmap_ack_repair_count"] == 1
    assert info["costmap_ack_repair_messages"] == 1
    assert info["costmap_ack_repair_cells"] == 1
    assert info["costmap_ack_mismatch_cells"] == 0


def test_server_ack_timeout_reports_content_mismatch():
    session = object.__new__(legacy.SmacSession)
    session.costmap_ack_timeout_s = 0.001
    session._last_server_update_time_ns = -1
    session._costmap_ack_sequence = 0
    session.client = None
    expected = np.full((2, 2), 100, dtype=np.int8)
    changed = np.ones_like(expected, dtype=bool)
    stale_server = np.zeros_like(expected, dtype=np.uint8)
    session._server_costmap_snapshot = lambda _deadline: (stale_server, 1)
    info = session._wait_for_costmap_ack(expected, changed)
    assert info["costmap_update_acknowledged"] is False
    assert info["costmap_ack_status"] == "server_content_mismatch"
    assert info["costmap_ack_mismatch_cells"] == 4
    assert info["costmap_ack_sequence"] == 0


def test_server_ack_readback_failure_is_not_treated_as_success():
    session = object.__new__(legacy.SmacSession)
    session.costmap_ack_timeout_s = 0.001
    session._last_server_update_time_ns = -1
    session._costmap_ack_sequence = 0
    session.client = None
    expected = np.zeros((2, 2), dtype=np.int8)
    changed = np.ones_like(expected, dtype=bool)

    def fail(_deadline):
        raise RuntimeError("injected packet loss")

    session._server_costmap_snapshot = fail
    info = session._wait_for_costmap_ack(expected, changed)
    assert info["costmap_update_acknowledged"] is False
    assert info["costmap_ack_status"] == "readback_error"
    assert "injected packet loss" in info["costmap_ack_error"]


def test_roi_ack_failure_performs_one_full_update_and_rechecks_content():
    session = object.__new__(legacy.SmacSession)
    session.supports_local_mask = True
    session._local_update_publisher = object()
    session._local_map_publisher = object()
    session.client = SimpleNamespace()
    session.local_map_update_strategy = "roi_ack"
    session.enable_mask_reuse_noop = False
    session._current_grid = np.full((2, 3), 100, dtype=np.int8)
    session._current_allowed_mask = np.zeros((2, 3), dtype=bool)
    session._force_full_next_update = False
    session._costmap_state_trusted = True
    session._last_update_had_fallback = False
    session._costmap_ack_sequence = 0
    expected = np.zeros((2, 3), dtype=np.int8)
    session._grid_for_mask = lambda allowed: (np.asarray(allowed, dtype=bool), expected.copy())
    session._publish_dirty_roi = lambda values, changed: (values.copy(), {
        "local_map_serialization_ms": 0.1,
        "local_map_publication_ms": 0.2,
        "costmap_settle_ms": 0.0,
        "roi_bbox": [0, 0, 3, 2],
        "roi_changed_cells": 6,
        "roi_published_cells": 6,
        "roi_message_count": 1,
        "roi_max_message_bytes": 6,
        "roi_publish_pacing_ms": 1.0,
    })
    ack_results = iter((
        {
            "costmap_update_acknowledged": False,
            "costmap_ack_status": "server_content_mismatch",
            "costmap_ack_wait_ms": 1.0,
            "costmap_ack_attempts": 1,
            "costmap_ack_checked_cells": 6,
            "costmap_ack_mismatch_cells": 2,
        },
        {
            "costmap_update_acknowledged": True,
            "costmap_ack_status": "server_content_verified",
            "costmap_ack_wait_ms": 2.0,
            "costmap_ack_attempts": 1,
            "costmap_ack_checked_cells": 6,
            "costmap_ack_mismatch_cells": 0,
            "costmap_ack_sequence": 1,
        },
    ))
    session._wait_for_costmap_ack = lambda values, changed: next(ack_results)
    full_calls = []

    def publish_full(values):
        full_calls.append(values.copy())
        session._last_publish_timing = {
            "local_map_serialization_ms": 0.3,
            "local_map_publication_ms": 0.4,
            "costmap_settle_ms": 0.0,
        }
        return 0.5

    session._publish_full_grid = publish_full
    result = session.update_local_mask(np.ones((2, 3), dtype=bool))
    assert len(full_calls) == 1
    assert result["local_map_update_mode"] == "roi_ack_full_fallback"
    assert result["local_map_update_fallback"] is True
    assert result["local_map_update_fallback_reason"] == "server_content_mismatch"
    assert result["costmap_update_acknowledged"] is True
    assert result["roi_ack_initial_mismatch_cells"] == 2
    assert session._costmap_state_trusted is True


def test_smac_internal_metrics_parser_uses_last_complete_record():
    record = (
        "PLN02_SMAC_METRICS search_ms=12.5 smoothing_ms=1.25 "
        "expanded_states=42 generated_states=84 heuristic_reset_ms=2.5 "
        "heuristic_eval_ms=3.5 analytic_expansion_ms=4.5 "
        "analytic_attempts=6 analytic_successes=1 success=1"
    )
    parsed = hybrid._parse_smac_benchmark_metrics("noise\n" + record)
    assert parsed == {
        "smac_search_ms": 12.5,
        "smac_smoothing_ms": 1.25,
        "expanded_states": 42,
        "generated_states": 84,
        "smac_heuristic_reset_ms": 2.5,
        "smac_heuristic_eval_ms": 3.5,
        "smac_analytic_expansion_ms": 4.5,
        "smac_analytic_attempts": 6,
        "smac_analytic_successes": 1,
        "smac_instrumented_success": True,
        "smac_internal_metrics_measured": True,
    }


def test_failure_parity_is_bound_to_exact_source_hash(tmp_path):
    evidence = tmp_path / "parity.json"
    evidence.write_text(json.dumps({
        "schema": "2a-v1-r2-deterministic-failure-parity-v1",
        "source_hash": "expected",
        "queries": {"A2B-16": {"failure_code": "NO_PATH_IN_CORRIDOR"}},
    }), encoding="utf-8")
    assert "A2B-16" in benchmark._load_parity(evidence, "expected")
    try:
        benchmark._load_parity(evidence, "different")
    except ValueError as exc:
        assert "source hash" in str(exc)
    else:
        raise AssertionError("stale parity evidence was accepted")


def test_endpoint_failure_classification_matches_a2b07_and_a2b16(tmp_path):
    corridor = tmp_path / "corridor"
    corridor.mkdir()
    (corridor / "runs.csv").write_text(
        "run_mode,query_id,final_valid_success\n"
        "measured,A2B-07,False\n"
        "measured,A2B-16,False\n",
        encoding="utf-8",
    )
    rows = []
    for query_id, original_valid in (("A2B-07", True), ("A2B-16", False)):
        for variant in (*endpoint_diagnostics.VARIANTS, "allow_reverse"):
            rows.append({
                "run_mode": "measured",
                "base_query_id": query_id,
                "variant": variant,
                "diagnostic_valid": original_valid and variant != "allow_reverse",
            })
    classified = {
        row["query_id"]: row["classification"]
        for row in endpoint_diagnostics._classification(rows, corridor)
    }
    assert classified == {
        "A2B-07": "ENDPOINT_MANEUVER_ENVELOPE_OR_CORRIDOR_ATTACHMENT",
        "A2B-16": "FULL_MAP_ALL_VARIANTS_FAILED_INVESTIGATE_MAP_OR_SMAC",
    }


def test_a2b19_adopted_profile_changes_only_heading_bins():
    profiles = dict(ablation.PROFILES)
    assert profiles["baseline"] == {}
    assert profiles["angle_bins_48"] == {"angle_quantization_bins": 48}
    assert "benchmark_instrumentation" not in profiles["angle_bins_48"]
