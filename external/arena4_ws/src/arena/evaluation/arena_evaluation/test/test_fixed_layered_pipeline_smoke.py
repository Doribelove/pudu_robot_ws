from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from arena_evaluation import fixed_layered_pipeline_smoke as fixed
from arena_evaluation import fixed_layered_pipeline_efficiency_smoke as efficiency


def _point(x: float, y: float, yaw: float = 0.0, steering: float = 0.0):
    return {
        "x": x,
        "y": y,
        "yaw": yaw,
        "steering": steering,
        "motion_direction": "forward",
        "source": "test",
        "planner_backend": "test",
        "backend_version": "test",
    }


def test_default_pipeline_excludes_optional_ompl_backends():
    assert fixed.DEFAULT_MAP_IDS == ("hospital_005",)
    assert fixed.DEFAULT_QUERY_IDS == ("q02", "q06", "q07", "q09")
    assert all("RRT" not in name and "SST" not in name for name in fixed.OPTIONAL_BACKENDS) is False
    assert fixed.OUTPUT_NAME == "fixed_layered_pipeline_smoke_v2"


def test_violation_groups_include_each_disjoint_corner():
    points = [_point(index * 0.05, 0.0) for index in range(120)]
    points[40]["y"] = 0.05
    points[41]["y"] = 0.10
    points[70]["y"] = 0.05
    points[71]["y"] = 0.0
    groups = fixed._violation_groups(points)
    assert len(groups) >= 2
    assert groups[0][0] <= 39 <= groups[0][-1]
    assert groups[-1][0] <= 70 <= groups[-1][-1]


def test_refresh_metadata_preserves_endpoints_and_continuous_steering():
    query = fixed.Query("metadata", [0.0, 0.0, 0.0], [2.0, 1.0, math.pi / 2.0])
    points = [_point(0.0, 0.0), _point(0.5, 0.0), _point(1.0, 0.2), _point(1.5, 0.7), _point(2.0, 1.0)]
    fixed._refresh_metadata(points, query)
    assert points[0]["yaw"] == 0.0
    assert math.isclose(points[-1]["yaw"], math.pi / 2.0)
    assert all(
        abs(float(b["steering"]) - float(a["steering"])) <= math.radians(15.0) + 1.0e-9
        for a, b in zip(points, points[1:])
    )


def test_smoke_query_only_aligns_q00_terminal_yaw():
    queries = fixed._queries()
    source = queries["q00"]
    smoke = queries["q00_forward_terminal"]
    assert smoke.start == source.start
    assert smoke.goal[:2] == source.goal[:2]
    assert smoke.goal[2] == math.pi


def test_window_indices_expand_around_entire_group():
    points = [_point(index * 0.05, 0.0) for index in range(420)]
    first, last = fixed._window_indices(points, [40, 41, 42])
    assert first < 40
    assert last > 42


def test_overlapping_windows_are_merged_before_repair():
    points = [_point(index * 0.05, 0.0) for index in range(160)]
    ranges = fixed._merged_window_ranges(points, [[10], [30], [45]])
    assert len(ranges) == 1
    assert ranges[0]["window_start_index"] <= 10
    assert ranges[0]["window_end_index"] > 45
    assert ranges[0]["merge_extent_start_index"] == 0


def test_derived_query_is_diagnostic_only():
    queries = fixed._queries()
    assert queries[fixed.SMOKE_QUERY_ID].query_id == "q00_forward_terminal"
    assert fixed.SMOKE_QUERY_ID not in fixed.DEFAULT_QUERY_IDS
    assert fixed.SMOKE_QUERY_ID in fixed.DIAGNOSTIC_QUERY_IDS


def test_violation_scan_includes_final_heading_segment():
    points = [_point(index * 0.05, 0.0) for index in range(5)]
    points[-1]["yaw"] = math.pi / 2.0
    assert len(points) - 2 in fixed._violation_indices(points)


def test_successful_two_meter_retry_stops_before_larger_radii(monkeypatch, tmp_path):
    points = [_point(index * 0.05, 0.0) for index in range(80)]
    calls = []
    scan_count = {"value": 0}

    def violation_scan(_points):
        scan_count["value"] += 1
        return [30] if scan_count["value"] == 1 else []

    class FakeSession:
        def plan(self, query, _spec, source="l3_hybrid_smac"):
            calls.append(query.query_id)
            replacement = [_point(query.start[0], query.start[1]), _point(query.goal[0], query.goal[1])]
            for item in replacement:
                item["source"] = "smac"
            return fixed.legacy.PlanResult(
                planner_success=True, points=replacement, planner_backend="smac", backend_version="test",
                source=source, diagnostics={"backend_called": True, "planning_time_ms": 1.0, "wall_time_ms": 2.0},
            )

    monkeypatch.setattr(fixed, "_violation_indices", violation_scan)
    monkeypatch.setattr(fixed, "_raw_local_mask", lambda *args, **kwargs: __import__("numpy").ones((2, 2), dtype=bool))
    monkeypatch.setattr(
        fixed.legacy, "validate_path",
        lambda *_args, **_kwargs: {"static_footprint_valid": True, "kinematic_valid": True, "failure_code": "", "failure_detail": ""},
    )
    spec = fixed.legacy.BackendSpec("hybrid_astar", "smac", "test", True, "")
    query = fixed.Query("q", [0.0, 0.0, 0.0], [3.95, 0.0, 0.0])
    l2 = fixed.legacy.PlanResult(planner_success=True, points=points, planner_backend="grid", backend_version="test")
    result, call_rows, _window_rows = fixed.repair_all_windows(
        fixed.legacy.MapContext("map", object(), __import__("numpy").ones((2, 2), dtype=bool), __import__("numpy").ones((2, 2)), "", ""),
        query, l2, spec, tmp_path, "commit", 5.0, smac_session=FakeSession(),
    )
    assert result.planner_success
    assert len(calls) == 1
    assert calls[0].endswith("_a0")
    assert {row["radius_m"] for row in call_rows} == {2.0}


def test_repair_calls_share_supplied_session(monkeypatch, tmp_path):
    points = [_point(index * 0.05, 0.0) for index in range(420)]
    scan_count = {"value": 0}

    def violation_scan(_points):
        scan_count["value"] += 1
        return [10, 390] if scan_count["value"] == 1 else []

    class FakeSession:
        def __init__(self):
            self.calls = 0

        def plan(self, query, _spec, source="l3_hybrid_smac"):
            self.calls += 1
            return fixed.legacy.PlanResult(
                planner_success=True,
                points=[_point(query.start[0], query.start[1]), _point(query.goal[0], query.goal[1])],
                planner_backend="smac", backend_version="test", source=source,
                diagnostics={"backend_called": True, "planning_time_ms": 1.0, "wall_time_ms": 2.0},
            )

    monkeypatch.setattr(fixed, "_violation_indices", violation_scan)
    monkeypatch.setattr(fixed, "_raw_local_mask", lambda *args, **kwargs: __import__("numpy").ones((2, 2), dtype=bool))
    monkeypatch.setattr(
        fixed.legacy, "validate_path",
        lambda *_args, **_kwargs: {"static_footprint_valid": True, "kinematic_valid": True, "failure_code": "", "failure_detail": ""},
    )
    session = FakeSession()
    spec = fixed.legacy.BackendSpec("hybrid_astar", "smac", "test", True, "")
    query = fixed.Query("q", [0.0, 0.0, 0.0], [20.95, 0.0, 0.0])
    l2 = fixed.legacy.PlanResult(planner_success=True, points=points, planner_backend="grid", backend_version="test")
    result, call_rows, _window_rows = fixed.repair_all_windows(
        fixed.legacy.MapContext("map", object(), __import__("numpy").ones((2, 2), dtype=bool), __import__("numpy").ones((2, 2)), "", ""),
        query, l2, spec, tmp_path, "commit", 5.0, smac_session=session,
    )
    assert result.planner_success
    assert session.calls == sum(bool(row["called"]) for row in call_rows)
    assert session.calls == 2


def test_map_session_query_reset_clears_only_transient_diagnostics():
    session = object.__new__(fixed.legacy.SmacSession)
    session.client = object()
    session.context = SimpleNamespace(ok=lambda: True)
    session.session_start_count = 1
    session.session_close_count = 0
    session.session_restart_count = 0
    session.restart_reasons = []
    session.supports_local_mask = False
    session._local_mask_info = {"local_mask_hash": "previous"}
    session.current_query_id = "old"
    values = session.reset_query_state("q02")
    assert session.current_query_id == "q02"
    assert session._local_mask_info == {}
    assert values["query_session_reused"] is True
    assert values["session_restart_count"] == 0


def test_l2_simplification_keeps_raw_path_and_checks_corridor_segments():
    class FakeMap:
        resolution = 0.05
        width = 200
        height = 200
        origin = [0.0, 0.0, 0.0]

        @staticmethod
        def world_to_cell(x, y):
            return int(round(y / 0.05)) + 50, int(round(x / 0.05)) + 20

        @staticmethod
        def footprint_collision(_pose, _footprint, unknown_is_collision=True):
            return False

    points = [
        _point(index * 0.05, 0.01 if index % 2 else 0.0)
        for index in range(80)
    ]
    original = [dict(point) for point in points]
    context = fixed.legacy.MapContext(
        "map", FakeMap(), np.ones((200, 200), dtype=bool),
        np.ones((200, 200), dtype=float), "map", "yaml",
    )
    topology = SimpleNamespace(graph=SimpleNamespace(nodes=[]))
    query = fixed.Query("q", [0.0, 0.0, 0.0], [3.95, 0.01, 0.0])
    simplified, diagnostics = fixed.simplify_l2_path(
        context, query, points, np.ones((200, 200), dtype=bool), topology, {},
    )
    assert points == original
    assert diagnostics["simplification_accepted"] is True
    assert len(simplified) < len(points)
    assert all(
        fixed._segment_is_safe(context, a, b, np.ones((200, 200), dtype=bool))
        for a, b in zip(simplified, simplified[1:])
    )


def test_topology_cache_metadata_binds_map_footprint_and_source():
    hospital_map = SimpleNamespace(
        resolution=0.05, width=10, height=20, origin=[1.0, 2.0, 0.0],
    )
    context = fixed.legacy.MapContext(
        "map", hospital_map, np.ones((20, 10), dtype=bool),
        np.ones((20, 10), dtype=float), "image-hash", "yaml-hash",
    )
    metadata = fixed._topology_cache_expected("map", context, "commit", "source-hash")
    assert metadata["map_file_hash"] == "image-hash"
    assert metadata["map_yaml_hash"] == "yaml-hash"
    assert metadata["footprint_hash"] == fixed.footprint_hash(fixed.FOOTPRINT)
    assert metadata["source_commit"] == "commit"
    assert metadata["source_hash"] == "source-hash"


def test_simplification_without_l3_benefit_skips_expensive_validation(monkeypatch):
    class FakeMap:
        resolution = 0.05
        width = 200
        height = 200
        origin = [0.0, 0.0, 0.0]

        @staticmethod
        def world_to_cell(x, y):
            return int(round(y / 0.05)) + 20, int(round(x / 0.05)) + 20

    points = [_point(index * 0.05, 0.0) for index in range(80)]
    context = fixed.legacy.MapContext(
        "map", FakeMap(), np.ones((200, 200), dtype=bool),
        np.ones((200, 200), dtype=float), "map", "yaml",
    )
    query = fixed.Query("q", [0.0, 0.0, 0.0], [3.95, 0.0, 0.0])
    topology = SimpleNamespace(graph=SimpleNamespace(nodes=[]))
    monkeypatch.setattr(
        fixed, "_segment_is_safe",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("full validation must be skipped")),
    )
    monkeypatch.setattr(
        fixed, "_path_minimum_inflated_clearance",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("clearance scan must be skipped")),
    )
    simplified, diagnostics = fixed.simplify_l2_path(
        context, query, points, np.ones((200, 200), dtype=bool), topology, {},
    )
    assert simplified == points
    assert diagnostics["simplification_accepted"] is False
    assert diagnostics["simplification_skip_reason"] == "no_l3_work_reduction"
    assert diagnostics["simplification_validation_time_ms"] == 0.0
    assert diagnostics["candidate_l3_window_count"] == diagnostics["raw_l3_window_count"]


def test_macro_window_partition_uses_full_violation_span_and_hard_limit():
    points = [_point(index * 0.05, 0.0) for index in range(589)]
    groups = [
        [0, 1], [31, 32, 33, 34], [116, 117], [122, 123], [131, 132, 133, 134, 135],
        [149, 150, 151, 152, 153], [193, 194], list(range(227, 235)), [272, 273],
        list(range(385, 391)), [392, 393], [431, 432], list(range(531, 536)),
        list(range(537, 544)), list(range(547, 554)), list(range(566, 570)), list(range(574, 588)),
    ]
    ranges = fixed._merged_window_ranges(
        points, groups, merge_gap_m=0.50,
        max_path_length_m=fixed.WINDOW_MAX_PATH_LENGTH_HARD_M,
    )
    assert len(ranges) == 2
    assert all(
        item["window_path_length_m"] <= fixed.WINDOW_MAX_PATH_LENGTH_HARD_M + 1.0e-9
        for item in ranges
    )
    assert all(item["merge_attempted"] for item in ranges)


def test_failed_merged_window_falls_back_to_bounded_children(monkeypatch, tmp_path):
    points = [_point(index * 0.05, 0.0) for index in range(120)]
    child_a = {
        "group_start_index": 25, "group_end_index": 30, "group_indices": [25, 30],
        "window_start_index": 10, "window_end_index": 45, "trigger_type": "curvature",
        "merge_attempted": False, "merge_children": [],
    }
    child_b = {
        "group_start_index": 70, "group_end_index": 75, "group_indices": [70, 75],
        "window_start_index": 55, "window_end_index": 90, "trigger_type": "curvature",
        "merge_attempted": False, "merge_children": [],
    }
    parent = {
        "group_start_index": 25, "group_end_index": 75,
        "group_indices": [25, 30, 70, 75],
        "window_start_index": 10, "window_end_index": 90,
        "trigger_type": "curvature", "merge_attempted": True,
        "merge_children": [child_a, child_b],
    }

    class FakeSession:
        supports_local_mask = False

        def __init__(self):
            self.calls = []

        def plan(self, query, _spec, source="l3_hybrid_smac"):
            self.calls.append(query.query_id)
            if "_w000_" in query.query_id:
                return fixed.legacy.PlanResult(
                    planner_success=False, failure_code="ACTION_ABORTED",
                    planner_backend="smac", backend_version="test", source=source,
                    diagnostics={"backend_called": True, "planning_time_ms": 1.0, "wall_time_ms": 2.0},
                )
            return fixed.legacy.PlanResult(
                planner_success=True,
                points=[_point(query.start[0], query.start[1]), _point(query.goal[0], query.goal[1])],
                planner_backend="smac", backend_version="test", source=source,
                diagnostics={"backend_called": True, "planning_time_ms": 1.0, "wall_time_ms": 2.0},
            )

    monkeypatch.setattr(fixed, "_violation_indices", lambda _points: [])
    monkeypatch.setattr(fixed, "_raw_local_mask", lambda *args, **kwargs: np.ones((2, 2), dtype=bool))
    monkeypatch.setattr(
        fixed.legacy, "validate_path",
        lambda *_args, **_kwargs: {
            "static_footprint_valid": True, "kinematic_valid": True,
            "failure_code": "", "failure_detail": "", "maximum_curvature": 0.0,
        },
    )
    session = FakeSession()
    spec = fixed.legacy.BackendSpec("hybrid_astar", "smac", "test", True, "")
    query = fixed.Query("q", [0.0, 0.0, 0.0], [5.95, 0.0, 0.0])
    l2 = fixed.legacy.PlanResult(
        planner_success=True, points=points, planner_backend="grid", backend_version="test",
    )
    result, calls, windows = fixed.repair_all_windows(
        fixed.legacy.MapContext(
            "map", object(), np.ones((2, 2), dtype=bool), np.ones((2, 2)), "", "",
        ),
        query, l2, spec, tmp_path, "commit", 5.0, smac_session=session,
        _pending_ordered=[parent],
    )
    assert result.planner_success
    assert len(session.calls) == 5
    assert int(result.diagnostics["l3_backend_call_count"]) == 5
    assert int(result.diagnostics["repair_window_count"]) == 3
    assert any(row["merge_fallback_used"] for row in calls if row["window_index"] == 0)
    assert any(row["merge_fallback_reason"] == "ACTION_ABORTED" for row in windows)


def _simplification_fixture():
    class FakeMap:
        resolution = 0.05
        width = 200
        height = 200
        origin = [0.0, 0.0, 0.0]

        @staticmethod
        def world_to_cell(x, y):
            return int(round(y / 0.05)) + 20, int(round(x / 0.05)) + 20

    points = [_point(index * 0.05, 0.0) for index in range(80)]
    context = fixed.legacy.MapContext(
        "map", FakeMap(), np.ones((200, 200), dtype=bool),
        np.ones((200, 200), dtype=float), "map", "yaml",
    )
    query = fixed.Query("q", [0.0, 0.0, 0.0], [3.95, 0.0, 0.0])
    topology = SimpleNamespace(graph=SimpleNamespace(nodes=[]))
    return context, query, points, topology


def test_v7_low_window_precheck_does_not_build_candidate(monkeypatch):
    context, query, points, topology = _simplification_fixture()
    precomputed = [{"window_start_index": 0}, {"window_start_index": 40}]
    monkeypatch.setattr(
        fixed, "_violation_groups",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("raw violations must use the caller's precomputed windows")
        ),
    )
    monkeypatch.setattr(
        fixed, "_merged_window_ranges",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("raw window merge must not be repeated")
        ),
    )
    monkeypatch.setattr(
        fixed, "_build_simplification_candidate",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("candidate must be skipped")),
    )
    result, diagnostics = fixed.simplify_l2_path(
        context, query, points, np.ones((200, 200), dtype=bool), topology, {},
        optimization_profile="v7_candidate",
        precomputed_raw_windows=precomputed,
    )
    assert result == points
    assert diagnostics["simplification_skip_reason"] == "low_l3_window_count"
    assert diagnostics["raw_l3_window_count"] == 2
    assert diagnostics["simplification_candidate_time_ms"] == 0.0


def test_v7_three_raw_windows_enters_candidate_flow(monkeypatch):
    context, query, points, topology = _simplification_fixture()
    called = {"candidate": 0, "violations": 0, "merge": 0}

    def merge(*_args, **_kwargs):
        called["merge"] += 1
        return [{"window_start_index": index} for index in range(3)]

    def violations(_points):
        called["violations"] += 1
        return [[10], [30], [60]]

    def candidate(*_args, **_kwargs):
        called["candidate"] += 1
        return [dict(point) for point in points], {}

    monkeypatch.setattr(fixed, "_violation_groups", violations)
    monkeypatch.setattr(fixed, "_merged_window_ranges", merge)
    monkeypatch.setattr(fixed, "_build_simplification_candidate", candidate)
    result, diagnostics = fixed.simplify_l2_path(
        context, query, points, np.ones((200, 200), dtype=bool), topology, {},
        optimization_profile="v7_candidate",
        precomputed_raw_windows=[
            {"window_start_index": 10},
            {"window_start_index": 30},
            {"window_start_index": 60},
        ],
    )
    assert result == points
    assert called["candidate"] == 1
    assert called["violations"] == 1
    assert called["merge"] == 1
    assert diagnostics["simplification_skip_reason"] == "no_l3_work_reduction"


def test_v6_compatible_simplification_ignores_precomputed_v7_windows(monkeypatch):
    context, query, points, topology = _simplification_fixture()
    called = {"violations": 0}

    def violations(_points):
        called["violations"] += 1
        return []

    monkeypatch.setattr(fixed, "_violation_groups", violations)
    result, diagnostics = fixed.simplify_l2_path(
        context, query, points, np.ones((200, 200), dtype=bool), topology, {},
        optimization_profile="v6_compatible",
        precomputed_raw_windows=[{"window_start_index": 10}],
    )
    assert result == points
    assert called["violations"] == 2
    assert diagnostics["raw_l3_window_count"] == 0


def test_selected_v7_rerun_failure_automatically_resolves_to_v6():
    selection = efficiency._resolve_final_profile(
        "v7_candidate", "lighter_smoother", False, ["p50_above_600ms"],
    )
    assert selection == {
        "profile": "v6_compatible",
        "smac_profile": "baseline",
        "optimization_stage": "baseline",
        "fallback_applied": True,
        "reasons": ["p50_above_600ms"],
    }


def test_selected_v7_rerun_success_keeps_candidate_profile():
    selection = efficiency._resolve_final_profile(
        "v7_candidate", "lighter_smoother", True, [],
    )
    assert selection["profile"] == "v7_candidate"
    assert selection["smac_profile"] == "lighter_smoother"
    assert selection["optimization_stage"] == "step3_delta_map"
    assert selection["fallback_applied"] is False


def test_v7_ab_refuses_to_overwrite_existing_stage_root(tmp_path):
    (tmp_path / "baseline_v6_compatible").mkdir()
    with pytest.raises(ValueError, match="non-empty A/B root"):
        efficiency._prepare_rollback_bundle(tmp_path)


def _bare_session(*, trusted=True, supports=True):
    session = object.__new__(fixed.legacy.SmacSession)
    session.client = object()
    session.context = SimpleNamespace(ok=lambda: True)
    session.session_start_count = 1
    session.session_close_count = 0
    session.session_restart_count = 0
    session.restart_reasons = []
    session.supports_local_mask = supports
    session._local_mask_info = {"local_mask_hash": "previous"}
    session.current_query_id = "old"
    session._action_in_progress = False
    session._costmap_state_trusted = trusted
    session._force_full_next_update = False
    return session


def test_light_reset_does_not_publish_full_map_and_clears_query_state(monkeypatch):
    session = _bare_session()
    monkeypatch.setattr(
        session, "update_local_mask",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("light reset published a map")),
    )
    first = session.reset_query_state("q02", restore_base_map=False)
    session._local_mask_info = {"local_mask_hash": "q02"}
    second = session.reset_query_state("q06", restore_base_map=False)
    assert first["query_session_reset_mode"] == "light"
    assert second["query_session_reset_mode"] == "light"
    assert not second["session_reset_fallback"]
    assert session.current_query_id == "q06"
    assert session._local_mask_info == {}


def test_untrusted_light_reset_falls_back_to_v6_full(monkeypatch):
    session = _bare_session(trusted=False)
    session.ctx = SimpleNamespace(
        hospital_map=SimpleNamespace(occupancy=np.zeros((2, 3), dtype=np.int8)),
    )
    updates = []
    monkeypatch.setattr(
        session, "update_local_mask",
        lambda mask, **kwargs: updates.append((np.asarray(mask).copy(), kwargs)) or {},
    )
    result = session.reset_query_state("q07", restore_base_map=False)
    assert result["query_session_reset_mode"] == "v6_full"
    assert result["session_reset_fallback"]
    assert result["session_reset_fallback_reason"] == "costmap_state_untrusted"
    assert len(updates) == 1 and updates[0][1]["force_full"] is True


def test_v6_compatible_reset_still_restores_base_map(monkeypatch):
    session = _bare_session()
    session.ctx = SimpleNamespace(
        hospital_map=SimpleNamespace(occupancy=np.array([[0, 100], [0, 0]], dtype=np.int8)),
    )
    updates = []
    monkeypatch.setattr(
        session, "update_local_mask",
        lambda mask, **kwargs: updates.append((np.asarray(mask).copy(), kwargs)) or {},
    )
    result = session.reset_query_state("q09")
    assert result["query_session_reset_mode"] == "v6_full"
    assert len(updates) == 1
    assert updates[0][0].tolist() == [[True, False], [True, True]]
    assert updates[0][1]["force_full"] is True


def test_delta_patch_coordinates_flip_and_bounds_are_correct():
    session = object.__new__(fixed.legacy.SmacSession)
    session.ctx = SimpleNamespace(hospital_map=SimpleNamespace(
        height=3, width=4,
        occupancy=np.zeros((3, 4), dtype=np.int8),
    ))
    mask = np.zeros((3, 4), dtype=bool)
    mask[0, 1] = True
    _mask, expected = session._grid_for_mask(mask)
    assert expected[2, 1] == 0
    assert expected[0, 1] == 100
    previous = np.full((3, 4), 100, dtype=np.int8)
    rectangles = fixed.legacy.delta_patch_rectangles(previous, expected)
    assert rectangles == [(1, 2, 1, 1)]
    assert all(x >= 0 and y >= 0 and x + w <= 4 and y + h <= 3 for x, y, w, h in rectangles)


def test_delta_switch_closes_old_window_and_matches_full_grid():
    lethal = np.full((8, 10), 100, dtype=np.int8)
    first = lethal.copy()
    first[1:3, 1:4] = 0
    second = lethal.copy()
    second[5:7, 7:9] = 0
    first_rectangles = fixed.legacy.delta_patch_rectangles(lethal, first)
    current = fixed.legacy.apply_delta_rectangles(lethal, first, first_rectangles)
    second_rectangles = fixed.legacy.delta_patch_rectangles(current, second)
    current = fixed.legacy.apply_delta_rectangles(current, second, second_rectangles)
    assert np.array_equal(current, second)
    assert np.all(current[1:3, 1:4] == 100)
    assert np.all(current[5:7, 7:9] == 0)


def test_delta_switch_keeps_distant_regions_as_separate_patches():
    lethal = np.full((12, 16), 100, dtype=np.int8)
    expected = lethal.copy()
    expected[1:3, 1:4] = 0
    expected[8:10, 12:15] = 0
    rectangles = fixed.legacy.delta_patch_rectangles(lethal, expected)
    assert rectangles == [(1, 1, 3, 2), (12, 8, 3, 2)]
    assert np.array_equal(fixed.legacy.apply_delta_rectangles(lethal, expected, rectangles), expected)


def test_delta_update_exception_falls_back_to_full(monkeypatch):
    session = object.__new__(fixed.legacy.SmacSession)
    session.ctx = SimpleNamespace(hospital_map=SimpleNamespace(
        height=2, width=3, occupancy=np.zeros((2, 3), dtype=np.int8),
    ))
    session.supports_local_mask = True
    session._local_update_publisher = object()
    session._local_map_publisher = object()
    session.client = object()
    session.local_map_update_strategy = "delta"
    session._force_full_next_update = False
    session._costmap_state_trusted = True
    session._current_allowed_mask = np.zeros((2, 3), dtype=bool)
    session._current_grid = np.full((2, 3), 100, dtype=np.int8)
    monkeypatch.setattr(
        session, "_publish_delta_updates",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("injected delta failure")),
    )
    full_updates = []
    monkeypatch.setattr(session, "_publish_full_grid", lambda values: full_updates.append(values.copy()) or 0.5)
    result = session.update_local_mask(np.ones((2, 3), dtype=bool))
    assert result["local_map_update_mode"] == "full_fallback"
    assert result["local_map_update_fallback"]
    assert "injected delta failure" in result["local_map_update_fallback_reason"]
    assert len(full_updates) == 1
    assert result["expected_mask_hash"] == result["applied_mask_hash"]


def test_path_outside_mask_is_rejected_after_full_fallback(monkeypatch):
    class FakeMap:
        resolution = 1.0
        width = 3
        height = 2
        occupancy = np.zeros((2, 3), dtype=np.int8)

        @staticmethod
        def world_to_cell(x, y):
            row, column = int(round(y)), int(round(x))
            return (row, column) if 0 <= row < 2 and 0 <= column < 3 else None

    class FakeClient:
        def __init__(self):
            self.calls = 0

        def plan(self, *_args, **_kwargs):
            self.calls += 1
            return "SUCCEEDED", "SUCCEEDED", 2.0, None, [
                {"x": 0.0, "y": 0.0, "yaw": 0.0},
                {"x": 2.0, "y": 0.0, "yaw": 0.0},
            ], None

    session = object.__new__(fixed.legacy.SmacSession)
    session.client = FakeClient()
    session.ctx = SimpleNamespace(hospital_map=FakeMap())
    session.planner_pid = 1
    session.stack_pids = []
    session.local_map_update_strategy = "delta"
    session._action_in_progress = False
    session._costmap_state_trusted = True
    updates = []

    def update(_mask, **kwargs):
        updates.append(kwargs)
        return {
            "local_map_update_ms": 1.0,
            "local_map_update_messages": 1,
            "local_map_update_cells": 1,
            "local_map_update_bytes": 1,
            "local_map_update_fallback": bool(kwargs.get("force_full")),
        }

    monkeypatch.setattr(session, "update_local_mask", update)
    allowed = np.zeros((2, 3), dtype=bool)
    allowed[0, 0] = True
    spec = fixed.legacy.BackendSpec("hybrid_astar", "smac", "test", True, "")
    result = session.plan(
        fixed.Query("q", [0.0, 0.0, 0.0], [2.0, 0.0, 0.0]),
        spec, allowed_mask=allowed,
    )
    assert session.client.calls == 2
    assert updates[-1]["force_full"] is True
    assert result.failure_code == "L3_PATH_OUTSIDE_LOCAL_MASK"
    assert not result.planner_success
    assert result.diagnostics["backend_call_count"] == 2
