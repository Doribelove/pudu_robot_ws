"""Formal static benchmark for the independent 2D-V1 architecture.

2D-V1 uses the original 2A-V0 static skeleton topology and replaces only
the topology-level Graph A* search with Graph D* Lite.  L2 is disabled and
L3 is the real corridor-constrained Smac Hybrid DUBIN planner.  This runner
owns the frozen-task protocol and audit files; it does not alter any older
architecture runner or historical result directory.
"""

from __future__ import annotations

import argparse
import collections
import csv
import datetime as dt
import hashlib
import json
import math
import os
import resource
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import layered_2d_v0_pipeline as v0
from . import layered_2d_v1_pipeline as v1
from . import l1_l3_corridor_hybrid_validity as validity
from . import unified_four_backends_smoke as legacy
from .dynamic_snapshot import DynamicSnapshot
from .planner_benchmark.map_utils import sha256_file


ROOT = Path("/home/robot/pudu_robot_ws")
MAP_ID = "mentor_map_20260825_005"
WORLD = ROOT / "external/arena4_ws/src/arena/simulation-setup/worlds" / MAP_ID
MAP_YAML = WORLD / "map/map.yaml"
SCENARIO_JSON = WORLD / "scenarios/a2b_benchmark_20.json"
BENCHMARK_JSON = ROOT / "benchmarks/arena_a2b_20/arena_a2b_benchmark_20.json"
BENCHMARK_CSV = ROOT / "benchmarks/arena_a2b_20/arena_a2b_benchmark_20.csv"
TASK_IDS = tuple(f"A2B-{index:02d}" for index in range(1, 21))
ARCHITECTURE_ID = "2D-V1"
IMPLEMENTATION_REVISION = "r1"
PARENT_ARCHITECTURE = "2A-V0"
PROTOCOL_VERSION = "PLN-02-EXP-V1"
EXPERIMENT_KIND = "static_formal"
QUERY_SET_ID = "arena_a2b_benchmark_20"
WARMUPS = 3
REPETITIONS = 5
SEED = 0
ROS_DOMAIN_ID = 237
DEFAULT_OUTPUT = ROOT / "experiments/layered_planner_benchmark/2d_v1_mentor_map_20260825_005_20_performance_v1"
DEFAULT_CACHE_ROOT = ROOT / "experiments/layered_planner_benchmark/2d_v1_mentor_map_20260825_005_20_performance_v1_cache"
CORRIDOR_PROFILE = "bounded_corridor_expansion_full_update"
CORRIDOR_SEMANTICS = "raw_map_smac_aligned"
PADDING_SCHEDULE = (2.0, 4.0, 6.0)
SMAC_PARAMETER_PROFILE = "baseline"
OPTIMIZATION_PROFILE = "v7_candidate"
OPTIMIZATION_STAGE = "step3_delta_map"


def _json_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _numeric(value: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _truth(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _percentile(rows: Sequence[Mapping[str, Any]], field: str, p: float) -> Optional[float]:
    values = [_numeric(row.get(field)) for row in rows]
    values = [value for value in values if value is not None]
    return float(np.percentile(values, p)) if values else None


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    materialized = list(rows)
    fields: List[str] = []
    for row in materialized:
        for key in row:
            if key not in fields:
                fields.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or ["empty"])
        writer.writeheader()
        for row in materialized:
            writer.writerow({key: json.dumps(value, sort_keys=True, default=str) if isinstance(value, (dict, list, tuple)) else value for key, value in row.items()})


def _refuse_nonempty(path: Path) -> None:
    if path.exists() and any(path.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {path}")


def _query_hash(query: Any) -> str:
    return _json_hash({"query_id": query.query_id, "start": list(query.start), "goal": list(query.goal)})


def _load_tasks() -> Tuple[List[Any], Dict[str, Any]]:
    """Load and cross-check the immutable benchmark JSON, CSV and scenario."""
    from .planner_benchmark.models import Query

    payload = json.loads(BENCHMARK_JSON.read_text(encoding="utf-8"))
    entry = (payload.get("maps") or {}).get(MAP_ID)
    json_tasks = list((entry or {}).get("tasks") or [])
    if [str(item.get("id")) for item in json_tasks] != list(TASK_IDS):
        raise ValueError("benchmark JSON does not contain ordered mentor A2B-01..A2B-20")
    with BENCHMARK_CSV.open(newline="", encoding="utf-8") as stream:
        csv_tasks = [row for row in csv.DictReader(stream) if row.get("world") == MAP_ID]
    if [str(item.get("task_id")) for item in csv_tasks] != list(TASK_IDS):
        raise ValueError("benchmark CSV does not contain ordered mentor A2B-01..A2B-20")
    scenario = json.loads(SCENARIO_JSON.read_text(encoding="utf-8"))
    scenario_tasks = list(scenario.get("tasks") or [])
    if scenario.get("world") != MAP_ID or [str(item.get("id")) for item in scenario_tasks] != list(TASK_IDS):
        raise ValueError("scenario does not contain ordered mentor A2B-01..A2B-20")
    queries = []
    for item, csv_item, scenario_item in zip(json_tasks, csv_tasks, scenario_tasks):
        start = [float(value) for value in item["start"]]
        goal = [float(value) for value in item["goal"]]
        csv_start = [float(csv_item[key]) for key in ("start_x_m", "start_y_m", "start_yaw_rad")]
        csv_goal = [float(csv_item[key]) for key in ("goal_x_m", "goal_y_m", "goal_yaw_rad")]
        if (not np.allclose(start, csv_start, rtol=0.0, atol=1e-9)
                or not np.allclose(goal, csv_goal, rtol=0.0, atol=1e-9)
                or not np.allclose(start, scenario_item["start"], rtol=0.0, atol=1e-9)
                or not np.allclose(goal, scenario_item["goal"], rtol=0.0, atol=1e-9)):
            raise ValueError(f"JSON/CSV/scenario pose mismatch for {item['id']}")
        queries.append(Query(str(item["id"]), start, goal, str(item.get("label", "")), 0, "UNVALIDATED"))
    return queries, {
        "map_id": MAP_ID, "task_ids": list(TASK_IDS), "query_set_id": QUERY_SET_ID,
        "benchmark_json_sha256": sha256_file(BENCHMARK_JSON), "benchmark_csv_sha256": sha256_file(BENCHMARK_CSV),
        "scenario_sha256": sha256_file(SCENARIO_JSON), "json_task_count": len(json_tasks),
        "csv_task_count": len(csv_tasks), "scenario_task_count": len(scenario_tasks),
        "resolution_m": float(payload.get("resolution_m", 0.0)), "dynamic_obstacles": False,
    }


def _context() -> Any:
    hospital_map = legacy.HospitalMap.load(MAP_YAML)
    if not math.isclose(float(hospital_map.resolution), 0.05, abs_tol=1e-12):
        raise ValueError(f"map resolution must be 0.05, got {hospital_map.resolution}")
    _occupied, free_mask, distance_m, _ = v0.topology.preprocess_static_map(
        hospital_map, legacy.FOOTPRINT, padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    return legacy.MapContext(MAP_ID, hospital_map, free_mask, distance_m,
                             sha256_file(hospital_map.image_path), sha256_file(hospital_map.yaml_path), MAP_YAML)


def _source_manifest() -> Tuple[Dict[str, str], str]:
    files = [Path(v1.__file__).resolve(), Path(v0.__file__).resolve(), Path(__file__).resolve(),
             Path(v1.__file__).resolve().parent / "topology.py", Path(v1.__file__).resolve().parents[1] / "setup.py",
             MAP_YAML, MAP_YAML.parent / "map.pgm", BENCHMARK_JSON, BENCHMARK_CSV, SCENARIO_JSON,
             legacy._strict_smac_config_path()]
    hashes = {str(path): sha256_file(path) for path in files if path.exists()}
    return hashes, _json_hash(hashes)


def _directory_bytes(path: Path) -> int:
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file()) if path.exists() else 0


def _pss_bytes() -> Optional[int]:
    try:
        for line in Path("/proc/self/smaps_rollup").read_text(encoding="utf-8").splitlines():
            if line.startswith("Pss:"):
                return int(line.split()[1]) * 1024
    except (OSError, ValueError, IndexError):
        return None
    return None


def _path_quality(output: Path, row: Mapping[str, Any], query: Any) -> Dict[str, Any]:
    path_file = str(row.get("path_file") or "")
    points: List[Mapping[str, Any]] = []
    if path_file:
        try:
            points = list(json.loads((output / path_file).read_text(encoding="utf-8")))
        except (OSError, ValueError, TypeError):
            points = []
    length = _numeric(row.get("path_length_m"))
    euclidean = math.hypot(float(query.goal[0]) - float(query.start[0]), float(query.goal[1]) - float(query.start[1]))
    turns = [abs(legacy._delta(float(b.get("yaw", 0.0)), float(a.get("yaw", 0.0)))) for a, b in zip(points, points[1:])]
    return {
        "path_point_count": len(points), "euclidean_ratio": length / euclidean if length is not None and euclidean > 1e-9 else None,
        "reference_ratio": "not_available", "mean_clearance_m": "not_available",
        "heading_change_rate_p95": "not_available", "total_heading_change_rad": sum(turns),
        "large_turn_count": sum(value > math.radians(45.0) for value in turns),
    }


def _collision_diagnostics(ctx: Any, points: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    """Locate static footprint collisions without changing validation policy."""
    if not points:
        return {"collision_count": 0, "collision_segment_indices": [], "collision_positions": []}
    positions: List[List[float]] = []
    segments: List[int] = []
    for segment_index, (first, second) in enumerate(zip(points, points[1:])):
        dx = float(second["x"]) - float(first["x"])
        dy = float(second["y"]) - float(first["y"])
        dyaw = legacy._delta(float(second["yaw"]), float(first["yaw"]))
        steps = max(1, int(math.ceil(math.hypot(dx, dy) / legacy.COLLISION_SAMPLE_SPACING_M)), int(math.ceil(abs(dyaw) / legacy.COLLISION_YAW_SAMPLE_STEP_RAD)))
        for step in range(steps + 1):
            fraction = step / steps
            pose = (
                float(first["x"]) + fraction * dx,
                float(first["y"]) + fraction * dy,
                legacy._wrap(float(first["yaw"]) + fraction * dyaw),
            )
            if ctx.hospital_map.footprint_collision(pose, legacy.FOOTPRINT, unknown_is_collision=True):
                positions.append([float(pose[0]), float(pose[1]), float(pose[2])])
                segments.append(int(segment_index))
    return {
        "collision_count": len(positions),
        "collision_segment_indices": sorted(set(segments)),
        "collision_positions": positions[:64],
    }


def _run_one(ctx: Any, graph: Any, topology_info: Mapping[str, Any], query: Any, run_mode: str,
             repetition: int, session: Any, spec: Any, adapter: Any, pipeline: Any,
             output: Path, snapshot: DynamicSnapshot, source_commit: str) -> Tuple[Dict[str, Any], Dict[str, Any], Dict[str, Any]]:
    run_id = f"{MAP_ID}_{query.query_id}_2d_v1_{run_mode}_{repetition}"
    started_ns = time.monotonic_ns()
    cpu_before = resource.getrusage(resource.RUSAGE_SELF)
    # r1 validity-first policy: each serial query starts from a complete base
    # map reset so inflation state from the preceding request cannot leak into
    # this request. The Smac/ROS session itself is still reused.
    reset_info = session.reset_query_state(query.query_id, restore_base_map=True)
    calls_before = int(adapter.calls)
    result = pipeline.plan_initial(query, snapshot)
    query_calls = max(0, int(adapter.calls) - calls_before)
    diagnostics = dict(result.diagnostics or {})
    points = [dict(point) for point in (result.points or [])]
    path_hash = ""
    path_file = ""
    if points:
        if not points[0].get("path_hash"):
            path_hash = v0._enrich_path(points, source_commit)
        else:
            path_hash = str(points[0].get("path_hash"))
        path_file = f"paths/{run_id}.json"
        (output / path_file).write_text(json.dumps(points, indent=2, sort_keys=True), encoding="utf-8")
    validation = legacy.validate_path(ctx, query, points) if points else {
        "static_footprint_valid": False, "kinematic_valid": False, "path_length_m": None,
        "minimum_clearance_m": None, "maximum_curvature": None, "curvature_p95": None,
        "heading_discontinuity_count": 0, "position_discontinuity_count": 0, "steering_jump_count": 0,
        "reverse_distance_m": 0.0, "in_place_rotation_count": 0, "failure_code": result.failure_code or "EMPTY_PATH",
        "failure_detail": result.failure_code or "EMPTY_PATH",
    }
    collision_diag = _collision_diagnostics(ctx, points)
    final_valid = bool(result.success and validation.get("static_footprint_valid") and validation.get("kinematic_valid"))
    failure_code = "" if final_valid else str(validation.get("failure_code") or diagnostics.get("failure_code") or result.failure_code or "L3_PLANNER_FAILED")
    wall_ms = (time.monotonic_ns() - started_ns) / 1e6
    cpu_after = resource.getrusage(resource.RUSAGE_SELF)
    cpu_ms = max(0.0, (cpu_after.ru_utime - cpu_before.ru_utime + cpu_after.ru_stime - cpu_before.ru_stime) * 1000.0)
    row: Dict[str, Any] = {
        "run_id": run_id, "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(), "architecture": ARCHITECTURE_ID,
        "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_version": PROTOCOL_VERSION, "experiment_kind": EXPERIMENT_KIND, "map_id": MAP_ID,
        "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256, "query_id": query.query_id,
        "query_hash": _query_hash(query), "query_sha256": _query_hash(query), "run_mode": run_mode, "repetition": repetition,
        "start_x": query.start[0], "start_y": query.start[1], "start_yaw": query.start[2],
        "goal_x": query.goal[0], "goal_y": query.goal[1], "goal_yaw": query.goal[2],
        "l1_backend": v1.L1_BACKEND, "l1_state_type": "original_topology_node_id", "topology_representation": "2a_v0_static_skeleton_graph",
        "topology_refinement_enabled": False, "topology_node_count": graph.node_count, "topology_edge_count": graph.edge_count,
        "topology_cache_hit": bool(topology_info.get("topology_cache_hit")), "topology_cache_key": topology_info.get("topology_cache_key", ""),
        "topology_cache_load_time_ms": topology_info.get("topology_load_time_ms", 0.0), "topology_build_time_ms": topology_info.get("topology_build_time_ms", 0.0),
        "topology_build_count": topology_info.get("topology_build_count", 0), "topology_load_count": topology_info.get("topology_load_count", 0),
        "l1_success": bool(diagnostics.get("topology_node_ids")), "l1_call_count": 1 if diagnostics.get("topology_node_ids") else 0,
        "l1_graph_search_ms": diagnostics.get("l1_dstar_initial_time_ms", 0.0), "l1_dstar_initial_time_ms": diagnostics.get("l1_dstar_initial_time_ms", 0.0),
        "l1_attachment_lookup_ms": diagnostics.get("attachment_lookup_time_ms", 0.0),
        "l1_candidate_collision_check_ms": diagnostics.get("candidate_collision_check_time_ms", 0.0),
        "l1_adjacency_build_ms": diagnostics.get("adjacency_build_time_ms", 0.0),
        "l1_route_search_ms": diagnostics.get("l1_dstar_initial_time_ms", 0.0),
        "l1_route_construction_ms": diagnostics.get("route_construction_time_ms", 0.0),
        "l1_dstar_incremental_time_ms": diagnostics.get("l1_dstar_incremental_time_ms", 0.0),
        "dstar_expanded_nodes": diagnostics.get("dstar_expanded_nodes", 0), "dstar_generated_nodes": diagnostics.get("dstar_generated_nodes", 0),
        "dstar_queue_pops": diagnostics.get("dstar_queue_pops", 0), "dstar_queue_pushes": diagnostics.get("dstar_queue_pushes", 0),
        "dstar_g_start": diagnostics.get("dstar_g_start", "not_available"), "dstar_rhs_start": diagnostics.get("dstar_rhs_start", "not_available"),
        "dstar_timeout_triggered": diagnostics.get("dstar_timeout_triggered", False), "dstar_no_path": diagnostics.get("dstar_no_path", False),
        "dstar_queue_size": diagnostics.get("dstar_queue_size", 0),
        "l2_called": False, "l2_call_count": 0, "l3_backend": v1.L3_BACKEND, "l3_call_count": query_calls,
        "l3_call_count_total": int(adapter.calls), "l3_prime_call_count": query_calls, "l3_retry_count": max(0, query_calls - 1),
        "corridor_semantics": CORRIDOR_SEMANTICS, "corridor_profile": CORRIDOR_PROFILE,
        "corridor_initial_padding_m": diagnostics.get("corridor_initial_padding_m", 2.0), "corridor_padding_m": diagnostics.get("corridor_padding_m", 2.0),
        "corridor_extra_margin_m": diagnostics.get("corridor_extra_margin_m", 0.0),
        "corridor_retry_paddings_m": diagnostics.get("corridor_retry_paddings_m", []), "corridor_fallback_used": diagnostics.get("corridor_fallback_used", False),
        "corridor_fallback_attempt_count": diagnostics.get("corridor_fallback_attempt_count", 0), "corridor_mask_hash": diagnostics.get("corridor_mask_hash", ""),
        "corridor_allowed_cells": diagnostics.get("corridor_allowed_cells", 0), "corridor_total_free_cells": diagnostics.get("corridor_total_free_cells", 0),
        "corridor_area_ratio": diagnostics.get("corridor_area_ratio", 0.0), "corridor_min_clearance_m": diagnostics.get("corridor_min_clearance_m", "not_available"),
        "topology_node_ids": diagnostics.get("topology_node_ids", []),
        "topology_edge_ids": diagnostics.get("topology_edge_ids", []),
        "route_polyline": diagnostics.get("route_polyline", []),
        "selected_start_attachment": diagnostics.get("selected_start_attachment", "not_available"),
        "selected_goal_attachment": diagnostics.get("selected_goal_attachment", "not_available"),
        "selected_connection_edges": diagnostics.get("selected_connection_edges", []),
        "start_attachment_candidates": diagnostics.get("start_attachment_candidates", []),
        "goal_attachment_candidates": diagnostics.get("goal_attachment_candidates", []),
        "endpoint_attachment_policy": diagnostics.get("endpoint_attachment_policy", "not_available"),
        "attachment_rank_penalty_m": diagnostics.get("attachment_rank_penalty_m", "not_available"),
        "dstar_state_snapshot": diagnostics.get("dstar_state_snapshot", {}),
        "dstar_g_start": diagnostics.get("dstar_g_start", "not_available"),
        "dstar_rhs_start": diagnostics.get("dstar_rhs_start", "not_available"),
        "dstar_timeout_triggered": diagnostics.get("dstar_timeout_triggered", False),
        "dstar_no_path": diagnostics.get("dstar_no_path", False),
        "dstar_queue_size": diagnostics.get("dstar_queue_size", 0),
        "pipeline_wall_time_ms": wall_ms, "online_wall_ms": wall_ms, "pipeline_cpu_total_ms": cpu_ms, "cpu_ms": cpu_ms,
        "peak_rss": int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024, "RSS": int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024,
        "peak_pss": _pss_bytes() if _pss_bytes() is not None else "not_available", "PSS": _pss_bytes() if _pss_bytes() is not None else "not_available",
        "hybrid_planning_time_ms": diagnostics.get("planning_time_ms", diagnostics.get("planner_wall_time_ms", "not_available")),
        "l3_time_ms": diagnostics.get("planning_time_ms", diagnostics.get("planner_wall_time_ms", "not_available")),
        "action_status": diagnostics.get("action_status", "not_available"), "action_result_code": diagnostics.get("action_result_code", "not_available"),
        "planner_search_started": diagnostics.get("planner_search_started", "not_available"), "smac_failure_code": diagnostics.get("smac_failure_code", ""),
        "smac_log_excerpt": diagnostics.get("smac_log_excerpt", ""),
        "attempts": diagnostics.get("attempts", []),
        "static_footprint_valid": validation.get("static_footprint_valid", "not_available"), "kinematic_valid": validation.get("kinematic_valid", "not_available"),
        "path_length_m": validation.get("path_length_m"), "minimum_clearance_m": validation.get("minimum_clearance_m"),
        "maximum_curvature": validation.get("maximum_curvature"), "curvature_p95": validation.get("curvature_p95"),
        "heading_discontinuity_count": validation.get("heading_discontinuity_count", 0), "position_discontinuity_count": validation.get("position_discontinuity_count", 0),
        "steering_jump_count": validation.get("steering_jump_count", 0), "reverse_distance_m": validation.get("reverse_distance_m", 0.0),
        "in_place_rotation_count": validation.get("in_place_rotation_count", 0), "start_yaw_error_rad": validation.get("start_yaw_error_rad"), "goal_yaw_error_rad": validation.get("goal_yaw_error_rad"),
        "final_valid_success": final_valid, "action_success": bool(result.success), "result_code": "SUCCEEDED" if result.success else str(diagnostics.get("action_status") or failure_code),
        "failure_code": failure_code, "failure_detail": validation.get("failure_detail", diagnostics.get("failure_detail", "")), "path_hash": path_hash, "path_file": path_file,
        "snapshot_id": snapshot.snapshot_id, "snapshot_hash": snapshot.snapshot_hash, "query_session_reset_ms": reset_info.get("query_session_reset_ms", "not_available"),
        "query_session_reset_mode": reset_info.get("query_session_reset_mode", "not_available"), "source_commit": source_commit,
        "query_session_reset_fallback": reset_info.get("session_reset_fallback", False),
        "query_session_reset_fallback_reason": reset_info.get("session_reset_fallback_reason", ""),
        "start_attachment_candidates": diagnostics.get("start_attachment_candidates", []),
        "goal_attachment_candidates": diagnostics.get("goal_attachment_candidates", []),
        "selected_connection_edges": diagnostics.get("selected_connection_edges", []),
        "route_polyline": diagnostics.get("route_polyline", []),
        "start_inflated_cost": diagnostics.get("start_inflated_cost", "not_available"),
        "goal_inflated_cost": diagnostics.get("goal_inflated_cost", "not_available"),
        "start_raw_map_cost": diagnostics.get("start_raw_map_cost", "not_available"),
        "goal_raw_map_cost": diagnostics.get("goal_raw_map_cost", "not_available"),
        "smac_start_cost": diagnostics.get("smac_start_cost", "not_available"),
        "smac_goal_cost": diagnostics.get("smac_goal_cost", "not_available"),
        "start_full_footprint_valid": diagnostics.get("start_full_footprint_valid", "not_available"),
        "goal_full_footprint_valid": diagnostics.get("goal_full_footprint_valid", "not_available"),
        "start_is_lethal": diagnostics.get("start_is_lethal", "not_available"),
        "goal_is_lethal": diagnostics.get("goal_is_lethal", "not_available"),
        "start_in_corridor": diagnostics.get("start_in_corridor", "not_available"),
        "goal_in_corridor": diagnostics.get("goal_in_corridor", "not_available"),
        "collision_count": collision_diag["collision_count"],
        "collision_segment_indices": collision_diag["collision_segment_indices"],
        "collision_positions": collision_diag["collision_positions"],
    }
    row["l1_total_time_ms"] = sum(float(row.get(field) or 0.0) for field in ("l1_attachment_lookup_ms", "l1_candidate_collision_check_ms", "l1_adjacency_build_ms", "l1_route_search_ms", "l1_route_construction_ms"))
    row["l3_call_count_total"] = int(adapter.calls)
    row.update(_path_quality(output, row, query))
    call = {
        "run_id": run_id, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "query_id": query.query_id, "query_hash": row["query_hash"], "run_mode": run_mode, "repetition": repetition,
        "stage": "L3", "role": "l3_prime_full_corridor_hybrid", "planner_backend": v1.L3_BACKEND,
        "called": bool(query_calls), "physical_backend_call_count": query_calls, "l3_call_count": query_calls,
        "l2_called": False, "l2_call_count": 0, "final_valid_success": final_valid, "failure_code": failure_code,
        "planner_search_started": row["planner_search_started"], "corridor_mask_hash": row["corridor_mask_hash"],
        "corridor_padding_m": row["corridor_padding_m"], "attempt_count": diagnostics.get("corridor_fallback_attempt_count", 0) + (1 if query_calls else 0),
        "attempts": diagnostics.get("attempts", []), "topology_refinement_enabled": False,
    }
    metric = {"run_id": run_id, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
              "query_id": query.query_id, "query_hash": row["query_hash"], "path_hash": path_hash,
              "final_valid_success": final_valid, "static_footprint_valid": row["static_footprint_valid"], "kinematic_valid": row["kinematic_valid"],
              "path_length_m": row["path_length_m"], "minimum_clearance_m": row["minimum_clearance_m"], "maximum_curvature": row["maximum_curvature"],
              "curvature_p95": row["curvature_p95"], "heading_discontinuity_count": row["heading_discontinuity_count"],
              "position_discontinuity_count": row["position_discontinuity_count"], "steering_jump_count": row["steering_jump_count"],
              "reverse_distance_m": row["reverse_distance_m"], "in_place_rotation_count": row["in_place_rotation_count"],
              "large_turn_count": row["large_turn_count"], "heading_change_rate_p95": "not_available"}
    return row, call, metric


def _report(output: Path, rows: Sequence[Mapping[str, Any]], topology_info: Mapping[str, Any], session_info: Mapping[str, Any], source_hash: str) -> Dict[str, Any]:
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    valid = sum(_truth(row.get("final_valid_success")) for row in measured)
    query_ids = sorted({str(row.get("query_id")) for row in measured})
    failures = dict(collections.Counter(str(row.get("failure_code")) for row in measured if row.get("failure_code")))
    success = [row for row in measured if _truth(row.get("final_valid_success"))]
    fmt = lambda value, digits=2: "not_available" if value is None else f"{float(value):.{digits}f}"
    summary = {
        "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "measured_count": len(measured), "final_valid_count": valid, "final_valid_rate": valid / len(measured) if measured else 0.0,
        "query_any_valid": sum(any(_truth(row.get("final_valid_success")) for row in measured if row.get("query_id") == query_id) for query_id in query_ids),
        "query_all_repeat_valid": sum(all(_truth(row.get("final_valid_success")) for row in measured if row.get("query_id") == query_id) for query_id in query_ids),
        "online_wall_p50_ms": _percentile(measured, "online_wall_ms", 50), "online_wall_p95_ms": _percentile(measured, "online_wall_ms", 95), "online_wall_p99_ms": _percentile(measured, "online_wall_ms", 99),
        "success_online_wall_p50_ms": _percentile(success, "online_wall_ms", 50), "success_online_wall_p95_ms": _percentile(success, "online_wall_ms", 95), "success_online_wall_p99_ms": _percentile(success, "online_wall_ms", 99),
        "cpu_p50_ms": _percentile(measured, "cpu_ms", 50), "cpu_p95_ms": _percentile(measured, "cpu_ms", 95), "cpu_p99_ms": _percentile(measured, "cpu_ms", 99),
        "rss_p50_bytes": _percentile(measured, "RSS", 50), "rss_p95_bytes": _percentile(measured, "RSS", 95), "rss_p99_bytes": _percentile(measured, "RSS", 99),
        "pss_p50_bytes": _percentile(measured, "PSS", 50), "pss_p95_bytes": _percentile(measured, "PSS", 95), "pss_p99_bytes": _percentile(measured, "PSS", 99),
        "l1_call_count": sum(int(_numeric(row.get("l1_call_count"), 0) or 0) for row in measured), "l2_call_count": 0,
        "l3_call_count": sum(int(_numeric(row.get("l3_call_count"), 0) or 0) for row in measured), "l3_retry_count": sum(int(_numeric(row.get("l3_retry_count"), 0) or 0) for row in measured),
        "fallback_count": sum(int(_numeric(row.get("corridor_fallback_attempt_count"), 0) or 0) for row in measured), "failure_counts": failures,
        "dstar_initial_expanded_nodes": sum(int(_numeric(row.get("dstar_expanded_nodes"), 0) or 0) for row in measured),
        "dstar_initial_generated_nodes": sum(int(_numeric(row.get("dstar_generated_nodes"), 0) or 0) for row in measured),
        "gate_passed": valid == len(measured),
    }
    path_length = [row.get("path_length_m") for row in success if _numeric(row.get("path_length_m")) is not None]
    clearance = [row.get("minimum_clearance_m") for row in success if _numeric(row.get("minimum_clearance_m")) is not None]
    max_curvature = [row.get("maximum_curvature") for row in success if _numeric(row.get("maximum_curvature")) is not None]
    report = [
        f"# {ARCHITECTURE_ID}-{IMPLEMENTATION_REVISION} formal experiment", "",
        "独立静态 20-query 实验；本结果不代表多地图结论。", "",
        f"- 条件：地图 `{MAP_ID}`；A2B-01..A2B-20；每组 {WARMUPS} warmup + {REPETITIONS} measured；measured={len(measured)}；分辨率=0.05 m/cell；dynamic_obstacles=false。",
        f"- 架构：L1 原始 2A-V0 静态骨架拓扑 + Graph D* Lite；L2 关闭；L3' 全程走廊内真实 Smac Hybrid DUBIN。未使用 refined topology。",
        f"- 约束：Rmin=0.40 m；最大曲率=2.50 1/m；禁止倒车和原地旋转；走廊语义={CORRIDOR_SEMANTICS}；fallback={PADDING_SCHEDULE} m。",
        f"- 缓存：topology hit={bool(topology_info.get('topology_cache_hit'))}；formal build/load={topology_info.get('topology_build_count', 0)}/{topology_info.get('topology_load_count', 0)}；cache bytes={_directory_bytes(Path(topology_info.get('topology_cache_directory', ''))) or 'not_available'}；成本不计入在线 query 耗时。",
        f"- Smac session start/close/restart={session_info.get('session_start_count', 0)}/{session_info.get('session_close_count', 0)}/{session_info.get('session_restart_count', 0)}；L2=0；RRTstar/SST=0/0。",
        "", "## 结果", "",
        f"- Final-valid：**{valid}/{len(measured)} ({100.0 * valid / len(measured) if measured else 0.0:.1f}%)**；query-any-valid={summary['query_any_valid']}/{len(query_ids)}；query-all-repeat-valid={summary['query_all_repeat_valid']}/{len(query_ids)}。",
        f"- Online wall P50/P95/P99：{fmt(summary['online_wall_p50_ms'])}/{fmt(summary['online_wall_p95_ms'])}/{fmt(summary['online_wall_p99_ms'])} ms；成功样本：{fmt(summary['success_online_wall_p50_ms'])}/{fmt(summary['success_online_wall_p95_ms'])}/{fmt(summary['success_online_wall_p99_ms'])} ms。",
        f"- CPU P50/P95/P99：{fmt(summary['cpu_p50_ms'])}/{fmt(summary['cpu_p95_ms'])}/{fmt(summary['cpu_p99_ms'])} ms；RSS：{fmt(summary['rss_p50_bytes'], 0)}/{fmt(summary['rss_p95_bytes'], 0)}/{fmt(summary['rss_p99_bytes'], 0)} bytes；PSS：{fmt(summary['pss_p50_bytes'], 0)}/{fmt(summary['pss_p95_bytes'], 0)}/{fmt(summary['pss_p99_bytes'], 0)} bytes。",
        f"- 调用：L1/L2/L3'={summary['l1_call_count']}/0/{summary['l3_call_count']}；L3 retry={summary['l3_retry_count']}；corridor fallback attempts={summary['fallback_count']}。",
        f"- D* Lite measured 总展开/生成节点：{summary['dstar_initial_expanded_nodes']}/{summary['dstar_initial_generated_nodes']}；本轮无动态障碍更新，不能宣称增量收益。",
        f"- 成功路径质量：长度均值={fmt(np.mean([float(value) for value in path_length]) if path_length else None)} m；最小净空均值={fmt(np.mean([float(value) for value in clearance]) if clearance else None)} m；最大曲率={fmt(max([float(value) for value in max_curvature], default=None), 4)} 1/m；heading-rate P95=not_available。",
        f"- 硬约束：静态碰撞={sum(not _truth(row.get('static_footprint_valid')) and _truth(row.get('action_success')) for row in measured)}；运动学违规={sum(not _truth(row.get('kinematic_valid')) and _truth(row.get('action_success')) for row in measured)}；倒车距离={fmt(sum(_numeric(row.get('reverse_distance_m'), 0.0) or 0.0 for row in measured))} m；原地旋转={sum(int(_numeric(row.get('in_place_rotation_count'), 0) or 0) for row in measured)}。",
        f"- 失败分布：`{failures}`。",
        "", "## 解释", "",
        "2D-V1 的唯一架构变化是将原始拓扑图上的 Graph A* 替换为 Graph D* Lite；静态初次规划每次都需要建立当前请求的虚拟端点状态，本轮没有动态边代价变化，因此不能将 D* Lite 解释为已获得增量重规划收益。",
        "与 2A-V0 的比较必须同时核对缓存、Smac session、costmap reset、fallback、源码版本和统计口径；本报告保留所有失败样本，不以提前失败样本制造速度优势。",
        f"- 失败原因必须以 `runs.csv` 与 `backend_call_log.csv` 的 query 级真实调用记录为准；source_hash=`{source_hash}`。",
    ]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    return summary


def run_formal(output: Path = DEFAULT_OUTPUT, *, topology_cache_dir: Path = DEFAULT_CACHE_ROOT,
               warmups: int = WARMUPS, repetitions: int = REPETITIONS, ros_domain_id: int = ROS_DOMAIN_ID,
               query_ids: Optional[Sequence[str]] = None) -> Path:
    _refuse_nonempty(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir()
    queries, metadata = _load_tasks()
    if query_ids:
        selected = set(query_ids)
        if not selected.issubset(set(TASK_IDS)):
            raise ValueError("query_ids must be a subset of A2B-01..A2B-20")
        queries = [query for query in queries if query.query_id in selected]
    ctx = _context()
    cache_root = Path(topology_cache_dir).resolve()
    # One explicit preflight build followed by a cache load makes cache cost
    # auditable without charging topology construction to online planning.
    v0.prepare_static_topology(ctx.hospital_map, legacy.FOOTPRINT, cache_root, padding_m=0.05, safety_margin_m=0.05, allow_unknown=False)
    artifact, topology_info = v0.prepare_static_topology(ctx.hospital_map, legacy.FOOTPRINT, cache_root, padding_m=0.05, safety_margin_m=0.05, allow_unknown=False)
    topology_info = dict(topology_info)
    topology_info["topology_build_count_total"] = 1 + int(topology_info.get("topology_build_count", 0) or 0)
    graph = v1.build_static_topology_view(artifact)
    spec = legacy.backend_availability()["hybrid_astar"]
    source_files, source_hash = _source_manifest()
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    session = legacy.SmacSession(ctx, output, map_yaml=MAP_YAML, log_tag=f"formal_2d_v1_{MAP_ID}", local_mask_updates=True, optimization_profile=OPTIMIZATION_PROFILE, smac_parameter_profile=SMAC_PARAMETER_PROFILE, optimization_stage=OPTIMIZATION_STAGE)
    session.start()
    # The validity-first r1 runner requests a complete static-layer update for
    # every corridor attempt. This keeps costmap inflation deterministic while
    # retaining one map-level Smac session.
    adapter = v0.SmacHybridAdapter(session, spec, footprint=legacy.FOOTPRINT, source_commit=legacy._source_commit(), force_full_update=True)
    pipeline = v1.Layered2DV1Pipeline(graph, footprint=legacy.FOOTPRINT, l3_planner=adapter, corridor_padding_m=2.0, corridor_profile="padding", corridor_fallback_policy="bounded", validator=lambda _map, query, points: legacy.validate_path(ctx, query, points))
    snapshot = DynamicSnapshot.empty(snapshot_id="static_empty_v1", map_shape=artifact.free_mask.shape)
    rows: List[Dict[str, Any]] = []; calls: List[Dict[str, Any]] = []; metrics: List[Dict[str, Any]] = []
    try:
        for run_mode, count in (("warmup", warmups), ("measured", repetitions)):
            for repetition in range(1, int(count) + 1):
                for query in queries:
                    row, call, metric = _run_one(ctx, graph, topology_info, query, run_mode, repetition, session, spec, adapter, pipeline, output, snapshot, legacy._source_commit() or "unknown")
                    row["source_hash"] = source_hash
                    call["source_hash"] = source_hash
                    metric["source_hash"] = source_hash
                    rows.append(row); calls.append(call); metrics.append(metric)
    finally:
        session.close()
    session_info = {"architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "session_start_count": session.session_start_count, "session_close_count": session.session_close_count, "session_restart_count": session.session_restart_count, "session_startup_time_ms": session.stack_startup_time_ms, "session_shutdown_time_ms": session.stack_shutdown_time_ms, "topology_build_count": topology_info.get("topology_build_count", 0), "topology_load_count": topology_info.get("topology_load_count", 0), "topology_build_count_total": topology_info.get("topology_build_count_total", 1), "topology_cache_hit": topology_info.get("topology_cache_hit", False), "l3_call_count_total": int(adapter.calls)}
    _write_csv(output / "runs.csv", rows); _write_csv(output / "backend_call_log.csv", calls); _write_csv(output / "path_metrics.csv", metrics); _write_csv(output / "session_timing.csv", [session_info])
    failure_counts = collections.Counter(str(row.get("failure_code")) for row in rows if row.get("run_mode") == "measured" and row.get("failure_code"))
    _write_csv(output / "failure_summary.csv", [{"failure_code": code, "count": count} for code, count in sorted(failure_counts.items())])
    source_manifest = {"experiment_id": output.name, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "parent_architecture": PARENT_ARCHITECTURE, "protocol_version": PROTOCOL_VERSION, "source_hash": source_hash, "source_files": source_files, "map_id": MAP_ID, "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256, "query_sha256": {query.query_id: _query_hash(query) for query in queries}, "footprint_hash": v0._footprint_hash(legacy.FOOTPRINT)}
    (output / "source_manifest.yaml").write_text(yaml.safe_dump(source_manifest, sort_keys=False), encoding="utf-8")
    protocol = {"experiment_id": output.name, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "parent_architecture": PARENT_ARCHITECTURE, "protocol_version": PROTOCOL_VERSION, "map_id": MAP_ID, "query_set_id": QUERY_SET_ID, "query_ids": [query.query_id for query in queries], "warmups": warmups, "repetitions": repetitions, "resolution_m": 0.05, "dynamic_obstacles": False, "minimum_turning_radius_m": 0.40, "maximum_curvature_1pm": 2.50, "allow_reverse": False, "allow_in_place_rotation": False, "layers": {"L1": "2A-V0 original static skeleton topology + Graph D* Lite", "L2": "disabled", "L3_prime": "full selected topology corridor Smac Hybrid DUBIN"}, "topology_refinement_enabled": False, "corridor_profile": CORRIDOR_PROFILE, "corridor_semantics": CORRIDOR_SEMANTICS, "padding_schedule_m": list(PADDING_SCHEDULE), "cache_mode": "optimized", "smac_parameter_profile": SMAC_PARAMETER_PROFILE, "optimization_profile": OPTIMIZATION_PROFILE, "optimization_stage": OPTIMIZATION_STAGE, "l2_called": False, "l2_call_count": 0, "rrtstar_call_count": 0, "sst_call_count": 0, "metric_availability": {"expanded_generated_states": "Graph D* Lite counters recorded; Smac counters not exposed", "mean_clearance_m": "not_available: validator exposes minimum only", "reference_ratio": "not_available: no approved reference path", "heading_change_rate_p95": "not_available: no temporal sampling", "peak_pss": "recorded from /proc/self/smaps_rollup when available"}}
    (output / "protocol.yaml").write_text(yaml.safe_dump(protocol, sort_keys=False), encoding="utf-8")
    metric_availability = {"smac_expanded_states": "not_available", "smac_generated_states": "not_available", "mean_clearance_m": "not_available", "reference_ratio": "not_available", "heading_change_rate_p95": "not_available", "peak_pss": "available_from_smaps_rollup_or_not_available", "dynamic_collision_count": "not_applicable_static_empty_snapshot"}
    (output / "metric_availability.yaml").write_text(yaml.safe_dump(metric_availability, sort_keys=False), encoding="utf-8")
    summary = _report(output, rows, topology_info, session_info, source_hash)
    manifest = {"experiment_id": output.name, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "parent_architecture": PARENT_ARCHITECTURE, "protocol_version": PROTOCOL_VERSION, "experiment_kind": EXPERIMENT_KIND, "map_id": MAP_ID, "query_set_id": QUERY_SET_ID, "query_ids": [query.query_id for query in queries], "warmup_count": warmups, "measured_repetitions": repetitions, "run_count": len(rows), "topology_representation": "2a_v0_static_skeleton_graph", "topology_refinement_enabled": False, "topology_cache_hit": topology_info.get("topology_cache_hit", False), "topology_build_count": topology_info.get("topology_build_count", 0), "topology_load_count": topology_info.get("topology_load_count", 0), "topology_build_count_total": topology_info.get("topology_build_count_total", 1), "topology_load_time_ms": topology_info.get("topology_load_time_ms", 0.0), "topology_cache_key": topology_info.get("topology_cache_key", ""), "topology_cache_bytes": _directory_bytes(Path(topology_info.get("topology_cache_directory", ""))) or "not_available", "session_start_count": session_info["session_start_count"], "session_close_count": session_info["session_close_count"], "session_restart_count": session_info["session_restart_count"], "l2_called": False, "l2_call_count": 0, "rrtstar_call_count": 0, "sst_call_count": 0, "smac_session_l3_call_count_total": int(adapter.calls), "source_hash": source_hash, "metric_availability": "see protocol.yaml", **summary}
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the 2D-V1 original-topology Graph D* Lite + Smac formal benchmark")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--topology-cache-dir", default=str(DEFAULT_CACHE_ROOT))
    parser.add_argument("--warmups", type=int, default=WARMUPS)
    parser.add_argument("--repetitions", type=int, default=REPETITIONS)
    parser.add_argument("--query-id", action="append", dest="query_ids")
    parser.add_argument("--ros-domain-id", type=int, default=ROS_DOMAIN_ID)
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        path = run_formal(Path(args.output_dir).resolve(), topology_cache_dir=Path(args.topology_cache_dir).resolve(), warmups=args.warmups, repetitions=args.repetitions, ros_domain_id=args.ros_domain_id, query_ids=args.query_ids)
    except Exception as exc:
        print(f"2d_v1_formal_benchmark: ERROR: {exc}")
        return 2
    print(f"2D-V1-r1 output: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
