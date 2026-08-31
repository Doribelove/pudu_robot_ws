from pathlib import Path
from types import SimpleNamespace

from arena_evaluation import layered_architecture_paired_benchmark as paired
from arena_evaluation import unified_four_backends_smoke as legacy
from arena_evaluation.planner_benchmark.models import Query


def test_mentor_benchmark_json_csv_are_identical_and_ordered():
    queries, metadata = paired._load_tasks()
    assert [query.query_id for query in queries] == list(paired.TASK_IDS)
    assert metadata["json_task_count"] == metadata["csv_task_count"] == 20
    assert metadata["resolution_m"] == 0.05
    assert metadata["dynamic_obstacles"] is False


def test_4x_map_scenario_json_matches_public_benchmark(monkeypatch):
    paired._configure_map(paired.FOUR_X_MAP_ID)
    try:
        queries, metadata = paired._load_tasks()
        assert [query.query_id for query in queries] == list(paired.TASK_IDS)
        assert metadata["map_id"] == paired.FOUR_X_MAP_ID
        assert metadata["scenario_sha256"]
        assert metadata["resolution_m"] == 0.05
    finally:
        paired._configure_map(paired.DEFAULT_MAP_ID)


def test_two_layer_parser_requires_static_map_and_exposes_worker_mode():
    args = paired.build_parser().parse_args(["--no-dynamic-obstacles"])
    assert args.worker_architecture is None
    worker = paired.build_parser().parse_args([
        "--no-dynamic-obstacles", "--worker-architecture", "two_layer",
        "--topology-cache-dir", "/tmp/cache", "--ros-domain-id", "101",
    ])
    assert worker.worker_architecture == "two_layer"
    assert worker.ros_domain_id == 101


def test_two_layer_runner_never_calls_grid_astar(monkeypatch, tmp_path):
    monkeypatch.setattr(legacy, "plan_grid_astar", lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("L2 called")))
    points = [{
        "x": 0.0, "y": 0.0, "yaw": 0.0, "source": "l3_prime_corridor_hybrid",
        "motion_direction": "forward", "steering": 0.0,
        "planner_backend": "smac", "backend_version": "test",
    }, {
        "x": 0.1, "y": 0.0, "yaw": 0.0, "source": "l3_prime_corridor_hybrid",
        "motion_direction": "forward", "steering": 0.0,
        "planner_backend": "smac", "backend_version": "test",
    }]
    result = legacy.PlanResult(planner_success=True, points=points, planner_backend="smac", backend_version="test")
    monkeypatch.setattr(paired.two_layer, "plan_l1_l3_corridor_hybrid", lambda *args, **kwargs: (result, {
        "l1_route_selected": True, "l3_prime_called": True, "l3_prime_call_count": 1,
        "corridor_padding_m": 2.0, "corridor_semantics": paired.TWO_LAYER_SEMANTICS,
        "corridor_mask_hash": "mask", "allowed_grid_cells": 10, "total_free_grid_cells": 100,
        "corridor_area_ratio": 0.1, "hybrid_planning_time_ms": 1.0,
    }))
    monkeypatch.setattr(legacy, "validate_path", lambda *args, **kwargs: {
        "static_footprint_valid": True, "kinematic_valid": True, "final_valid_success": True,
        "failure_code": "", "failure_detail": "", "path_length_m": 0.1,
        "minimum_clearance_m": 1.0, "maximum_curvature": 0.0, "curvature_p95": 0.0,
        "heading_discontinuity_count": 0, "position_discontinuity_count": 0,
        "steering_jump_count": 0, "reverse_distance_m": 0.0, "in_place_rotation_count": 0,
        "start_position_error_m": 0.0, "goal_position_error_m": 0.0,
        "start_yaw_error_rad": 0.0, "goal_yaw_error_rad": 0.0,
    })
    ctx = SimpleNamespace(map_id=paired.MAP_ID, map_sha256="map", map_yaml_sha256="yaml")
    query = Query("A2B-01", [0.0, 0.0, 0.0], [0.1, 0.0, 0.0])
    topology_info = {"topology_cache_key": "key", "topology_cache_hit": True}
    spec = SimpleNamespace(backend="smac", version="test")
    row, calls, _extra = paired._run_two_layer(ctx, SimpleNamespace(), topology_info, query, "measured", 1, None, spec, tmp_path, "source")
    assert row["l2_called"] is False
    assert row["l2_call_count"] == 0
    assert row["l3_prime_call_count"] == 1
    assert calls[0]["l2_call_count"] == 0
    assert calls[0]["corridor_profile"] == paired.TWO_LAYER_PROFILE


def test_two_layer_profile_is_fixed_raw_map_smac_aligned_2m():
    assert paired.TWO_LAYER_PROFILE == "raw_map_smac_aligned_2m"
    assert paired.TWO_LAYER_SEMANTICS == "raw_map_smac_aligned"
    assert paired.TWO_LAYER_PADDING_M == 2.0


def test_source_manifest_contains_both_benchmark_files():
    files, digest = paired._source_manifest()
    assert str(paired.BENCHMARK_JSON) in files
    assert str(paired.BENCHMARK_CSV) in files
    assert digest


def test_backend_call_rows_receive_query_level_l1_diagnostics():
    calls = paired._annotate_call_rows([{"stage": "L2", "physical_backend_call_count": 1, "failure_code": "ATTEMPT_FAILED"}], {
        "map_id": paired.MAP_ID,
        "cache_mode": "optimized",
        "l1_backend": "skeleton_distance_transform_v1",
        "l1_total_time_ms": 1.25,
        "topology_cache_key": "key",
        "topology_cache_hit": True,
        "topology_adjacency_cache_hit": True,
        "endpoint_spatial_index_cache_hit": True,
        "endpoint_candidate_cache_hit": True,
        "route_cache_hit": False,
        "final_valid_success": False,
        "failure_code": "CORRIDOR_NO_PATH",
    })
    assert calls[0]["l1_total_time_ms"] == 1.25
    assert calls[0]["topology_adjacency_cache_hit"] is True
    assert calls[0]["failure_code"] == "ATTEMPT_FAILED"
    assert calls[0]["query_failure_code"] == "CORRIDOR_NO_PATH"
