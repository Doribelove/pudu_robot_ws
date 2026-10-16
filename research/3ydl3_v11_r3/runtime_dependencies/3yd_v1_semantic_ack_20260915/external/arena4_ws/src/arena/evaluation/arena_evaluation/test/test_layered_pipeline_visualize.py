from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from arena_evaluation import layered_pipeline_visualize as visualize


def _point(x: float, *, source: str = "grid", yaw: float = 0.0):
    return {
        "x": x,
        "y": 0.0,
        "yaw": yaw,
        "source": source,
        "motion_direction": "forward",
        "steering": 0.0,
        "velocity": 0.0,
        "planner_backend": "test",
        "backend_version": "test",
    }


def _artifact(tmp_path):
    image_path = tmp_path / "map.pgm"
    yaml_path = tmp_path / "map.yaml"
    image_path.write_bytes(b"P5\n1 1\n255\n\xff")
    yaml_path.write_text("image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n")
    hospital_map = SimpleNamespace(image_path=image_path, yaml_path=yaml_path)
    return SimpleNamespace(
        hospital_map=hospital_map,
        free_mask=np.ones((1, 1), dtype=bool),
        distance_m=np.ones((1, 1), dtype=float),
    )


def _task():
    return visualize.BenchmarkTask(
        map_id="map",
        query_id="A2B-01",
        label="test",
        start=(0.0, 0.0, 0.0),
        goal=(1.0, 0.0, 0.0),
        preference="center",
        preference_side="none",
        feature_tags=("test",),
    )


def test_visualizer_reuses_one_query_level_smac_session(monkeypatch, tmp_path):
    session = SimpleNamespace(stack_shutdown_time_ms=4.0, close_calls=0)

    def close():
        session.close_calls += 1

    session.close = close
    build_calls = []
    repair_sessions = []

    monkeypatch.setattr(
        visualize.layered_runtime,
        "backend_availability",
        lambda: {"hybrid_astar": visualize.layered_runtime.BackendSpec(
            "hybrid_astar", "Nav2 SmacPlannerHybrid DUBIN", "test", True, "",
        )},
    )
    monkeypatch.setattr(visualize.fixed, "_violation_groups", lambda _points: [[2]])
    monkeypatch.setattr(
        visualize.fixed,
        "_merged_window_ranges",
        lambda *_args, **_kwargs: [{
            "window_start_index": 0,
            "window_end_index": 4,
            "group_start_index": 2,
            "group_end_index": 2,
        }],
    )

    def build_session(*_args, **_kwargs):
        build_calls.append(True)
        return session, object(), {"l3_local_map_build_ms": 1.0, "l3_stack_startup_ms": 3.0}

    def repair(_ctx, _query, _l2, spec, _output, _commit, _timeout, smac_session=None):
        repair_sessions.append(smac_session)
        points = [_point(index * 0.25, source="l3_hybrid_smac", yaw=0.1) for index in range(5)]
        visualize.fixed._enrich(points, "commit")
        result = visualize.layered_runtime.PlanResult(
            planner_success=True,
            points=points,
            planner_backend=spec.backend,
            backend_version=spec.version,
            source="layered_l1_l2_l3_smac",
            diagnostics={
                "l3_attempted": True,
                "l3_backend_call_count": 1,
                "repair_window_count": 1,
                "l3_planning_time_ms": 2.0,
                "l3_process_overhead_ms": 1.0,
                "l3_action_wall_ms": 3.0,
                "stitch_validation_time_ms": 0.5,
            },
        )
        calls = [{"stage": "L3", "called": True, "planner_success": True}]
        windows = [{
            "window_index": 0,
            "attempt_index": 0,
            "radius_m": 2.0,
            "window_start_index": 0,
            "window_end_index": 4,
            "selected_candidate": True,
        }]
        return result, calls, windows

    monkeypatch.setattr(visualize.fixed, "_build_query_smac_session", build_session)
    monkeypatch.setattr(visualize.fixed, "repair_all_windows", repair)

    points = [_point(index * 0.25) for index in range(5)]
    result, windows, calls, diagnostics = visualize._l3_plan(
        _artifact(tmp_path), _task(), points, tmp_path,
    )

    assert len(build_calls) == 1
    assert repair_sessions == [session]
    assert session.close_calls == 1
    assert diagnostics["l3_stack_shutdown_ms"] == 4.0
    assert diagnostics["l3_backend_call_count"] == 1
    assert diagnostics["query_level_smac_context_reuse"] is True
    assert len(calls) == 1
    assert len(windows) == 1
    assert all(point["source"] == "l3_hybrid_smac" for point in result)


def test_visualizer_records_query_session_start_failure(monkeypatch, tmp_path):
    monkeypatch.setattr(
        visualize.layered_runtime,
        "backend_availability",
        lambda: {"hybrid_astar": visualize.layered_runtime.BackendSpec(
            "hybrid_astar", "Nav2 SmacPlannerHybrid DUBIN", "test", True, "",
        )},
    )
    monkeypatch.setattr(visualize.fixed, "_violation_groups", lambda _points: [[1]])
    monkeypatch.setattr(
        visualize.fixed,
        "_merged_window_ranges",
        lambda *_args, **_kwargs: [{
            "window_start_index": 0,
            "window_end_index": 2,
            "group_start_index": 1,
            "group_end_index": 1,
        }],
    )
    monkeypatch.setattr(
        visualize.fixed,
        "_build_query_smac_session",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("lifecycle failed")),
    )

    result, windows, calls, diagnostics = visualize._l3_plan(
        _artifact(tmp_path), _task(), [_point(0.0), _point(0.5), _point(1.0)], tmp_path,
    )

    assert result == []
    assert diagnostics["l3_failure_code"] == "L3_STACK_START_FAILED"
    assert diagnostics["l3_triggered"] is True
    assert diagnostics["l3_called"] is False
    assert len(windows) == 1
    assert calls[-1]["called"] is False


def test_failed_post_pass_retains_last_diagnostic_smac_candidate(monkeypatch, tmp_path):
    session = SimpleNamespace(stack_shutdown_time_ms=1.0, close=lambda: None)
    monkeypatch.setattr(
        visualize.layered_runtime,
        "backend_availability",
        lambda: {"hybrid_astar": visualize.layered_runtime.BackendSpec(
            "hybrid_astar", "Nav2 SmacPlannerHybrid DUBIN", "test", True, "",
        )},
    )
    monkeypatch.setattr(visualize.fixed, "_violation_groups", lambda _points: [[1]])
    monkeypatch.setattr(
        visualize.fixed,
        "_merged_window_ranges",
        lambda *_args, **_kwargs: [{
            "window_start_index": 0,
            "window_end_index": 2,
            "group_start_index": 1,
            "group_end_index": 1,
        }],
    )
    monkeypatch.setattr(
        visualize.fixed,
        "_build_query_smac_session",
        lambda *_args, **_kwargs: (session, object(), {}),
    )
    repair_calls = {"count": 0}

    def repair(*_args, **_kwargs):
        repair_calls["count"] += 1
        if repair_calls["count"] == 1:
            candidate = [_point(0.0, source="l3_hybrid_smac"), _point(0.5, source="l3_hybrid_smac"), _point(1.0, source="l3_hybrid_smac")]
            visualize.fixed._enrich(candidate, "commit")
            return visualize.layered_runtime.PlanResult(
                planner_success=False,
                points=candidate,
                failure_code="L3_FINAL_VALIDATION_FAILED",
                diagnostics={"l3_backend_call_count": 1, "l3_attempted": True},
            ), [{"stage": "L3", "called": True}], []
        return visualize.layered_runtime.PlanResult(
            planner_success=False,
            points=None,
            failure_code="L3_LOCAL_KINEMATIC_VALIDATION_FAILED",
            diagnostics={"l3_backend_call_count": 1, "l3_attempted": True},
        ), [{"stage": "L3", "called": True}], []

    monkeypatch.setattr(visualize.fixed, "repair_all_windows", repair)

    final_points, _windows, _calls, diagnostics = visualize._l3_plan(
        _artifact(tmp_path), _task(), [_point(0.0), _point(0.5), _point(1.0)], tmp_path,
    )

    assert final_points == []
    assert len(diagnostics["l3_diagnostic_points"]) == 3
    assert all(point["source"] == "l3_hybrid_smac" for point in diagnostics["l3_diagnostic_points"])


def test_yaw_arrows_select_only_actual_smac_output():
    points = [
        _point(0.0, source="grid", yaw=0.0),
        _point(0.5, source="l3_hybrid_smac", yaw=0.3),
        _point(1.0, source="grid", yaw=0.0),
    ]

    selected = visualize._smac_yaw_points(points)

    assert len(selected) == 1
    assert selected[0]["yaw"] == 0.3


def test_logical_window_prefers_accepted_early_retry():
    rows = [
        {"window_index": 0, "attempt_index": 0, "radius_m": 2.0, "selected_candidate": True},
        {"window_index": 0, "attempt_index": 1, "radius_m": 4.0, "selected_candidate": False},
    ]

    selected = visualize._logical_repair_windows(rows)

    assert len(selected) == 1
    assert selected[0]["radius_m"] == 2.0


def test_a2b_query_id_is_safe_for_ros_client_node(monkeypatch, tmp_path):
    captured = {}

    class FakeContext:
        def ok(self):
            return False

    class FakeRclpy:
        @staticmethod
        def init(context=None):
            captured["context"] = context

    class FakeStack:
        def __init__(self, **_kwargs):
            pass

    class FakeClient:
        pass

    monkeypatch.setattr(
        visualize.layered_runtime,
        "_strict_smac_config_path",
        lambda: tmp_path / "config.yaml",
    )
    (tmp_path / "config.yaml").write_text("GridBased: {}\n")
    monkeypatch.setattr(
        "arena_evaluation.planner_benchmark.config.load_yaml",
        lambda _path: {"GridBased": {}},
    )
    monkeypatch.setattr(
        "arena_evaluation.planner_benchmark.config.stack_parameters",
        lambda **_kwargs: {},
    )
    monkeypatch.setattr(
        "arena_evaluation.planner_benchmark.runner.BenchmarkStack",
        FakeStack,
    )
    monkeypatch.setattr(
        "arena_evaluation.planner_benchmark.runner.ComputePathClient",
        FakeClient,
    )
    monkeypatch.setattr("rclpy.init", FakeRclpy.init)
    monkeypatch.setattr("rclpy.context.Context", FakeContext)
    hospital_map = SimpleNamespace(width=1, height=1, origin=(0.0, 0.0, 0.0))
    context = visualize.layered_runtime.MapContext(
        "map", hospital_map, np.ones((1, 1), dtype=bool), np.ones((1, 1)), "", "", tmp_path / "map.yaml",
    )

    session = visualize.layered_runtime.SmacSession(
        context, tmp_path, map_yaml=tmp_path / "map.yaml", log_tag="layered_map_A2B-01",
    )

    assert session.client_node_name.startswith("planner_benchmark_client_layered_map_A2B_01_")
    assert "-" not in session.client_node_name
