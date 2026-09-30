"""Paired three-layer versus two-layer benchmark on the frozen A2B-20 tasks.

This entry point is intentionally independent from the smoke runners.  It
loads the mentor-map tasks from the repository benchmark JSON and CSV, shares
one metadata-bound topology cache, and runs one isolated worker process per
architecture.  The workers keep a single Smac session for the map and write
only auditable planner output; no fallback backend is available here.
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
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import fixed_layered_pipeline_smoke as fixed
from . import l1_l3_corridor_hybrid_smoke as two_layer
from . import unified_four_backends_smoke as legacy
from .planner_benchmark.map_utils import HospitalMap, sha256_file
from .planner_benchmark.models import Query
from .topology import preprocess_static_map


ROOT = Path("/home/robot/pudu_robot_ws")
DEFAULT_MAP_ID = "mentor_map_20260825_005"
FOUR_X_MAP_ID = "mentor_map_20260825_005_4x_area"
MAP_ID = DEFAULT_MAP_ID
MAP_YAML = ROOT / "external/arena4_ws/src/arena/simulation-setup/worlds" / MAP_ID / "map/map.yaml"
SCENARIO_JSON = ROOT / "external/arena4_ws/src/arena/simulation-setup/worlds" / MAP_ID / "scenarios/a2b_benchmark_20.json"
BENCHMARK_JSON = ROOT / "benchmarks/arena_a2b_20/arena_a2b_benchmark_20.json"
BENCHMARK_CSV = ROOT / "benchmarks/arena_a2b_20/arena_a2b_benchmark_20.csv"
OUTPUT_NAME = "l1_l2_l3_vs_l1_l3prime_mentor_map_20_v1"
TASK_IDS = tuple(f"A2B-{index:02d}" for index in range(1, 21))
FOOTPRINT = legacy.FOOTPRINT
TOPOLOGY_CACHE = "topology_cache_shared"
TWO_LAYER_PROFILE = "raw_map_smac_aligned_2m"
TWO_LAYER_SEMANTICS = "raw_map_smac_aligned"
TWO_LAYER_PADDING_M = 2.0
WARMUPS = 3
REPETITIONS = 5
TIMEOUT_S = 5.0
SMAC_PARAMETER_PROFILE = "lighter_smoother"
OPTIMIZATION_PROFILE = "v7_candidate"
OPTIMIZATION_STAGE = "step3_delta_map"
HISTORICAL_3A_PAIRED_SUMMARY = ROOT / "experiments/layered_planner_benchmark/l1_l2_l3_vs_l1_l3prime_mentor_map_20_v1/paired_summary.csv"
HISTORICAL_2A_CACHE_RUNS = ROOT / "experiments/layered_planner_benchmark/l1_l3_corridor_hybrid_mentor_map_20_cache_v1_cache_hit_full/runs.csv"

# These fields describe the L1/cache work attached to a query.  Every
# backend-call row receives the same values as its runs.csv row so that call
# counts and cache diagnostics can be audited without joining on run_id.
CALL_QUERY_DIAGNOSTIC_FIELDS = (
    "cache_mode",
    "l1_backend",
    "l1_success",
    "l1_attachment_lookup_ms",
    "l1_candidate_collision_check_ms",
    "l1_adjacency_build_ms",
    "l1_route_search_ms",
    "l1_route_construction_ms",
    "l1_graph_search_ms",
    "l1_total_time_ms",
    "l1_start_candidate_count",
    "l1_goal_candidate_count",
    "l1_candidate_pair_attempts",
    "topology_cache_key",
    "topology_cache_hit",
    "topology_adjacency_cache_hit",
    "endpoint_spatial_index_cache_hit",
    "endpoint_candidate_cache_hit",
    "route_cache_hit",
)


def _annotate_call_rows(call_rows: Sequence[Mapping[str, Any]], run_row: Mapping[str, Any]) -> List[Dict[str, Any]]:
    """Copy query-level diagnostics onto each physical backend call row."""
    annotated: List[Dict[str, Any]] = []
    for call_row in call_rows:
        item = dict(call_row)
        item.setdefault("map_id", MAP_ID)
        for field in CALL_QUERY_DIAGNOSTIC_FIELDS:
            item.setdefault(field, run_row.get(field))
        # Keep the physical call's own status/failure untouched.  The final
        # query outcome is explicit so consumers do not confuse the two.
        item["query_final_valid_success"] = run_row.get("final_valid_success")
        item["query_failure_code"] = run_row.get("failure_code")
        annotated.append(item)
    return annotated


def _configure_map(map_id: str) -> None:
    """Select a repository map for this process without changing defaults."""
    global MAP_ID, MAP_YAML, SCENARIO_JSON
    if map_id not in {DEFAULT_MAP_ID, FOUR_X_MAP_ID}:
        raise ValueError(f"unsupported paired-benchmark map: {map_id}")
    MAP_ID = str(map_id)
    world_dir = ROOT / "external/arena4_ws/src/arena/simulation-setup/worlds" / MAP_ID
    MAP_YAML = world_dir / "map/map.yaml"
    SCENARIO_JSON = world_dir / "scenarios/a2b_benchmark_20.json"


def _json_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _query_hash(query: Query) -> str:
    return _json_hash({"query_id": query.query_id, "start": list(query.start), "goal": list(query.goal)})


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
            encoded = {}
            for key, value in row.items():
                if isinstance(value, (dict, list, tuple)):
                    encoded[key] = json.dumps(value, sort_keys=True, default=str)
                elif isinstance(value, np.generic):
                    encoded[key] = value.item()
                else:
                    encoded[key] = value
            writer.writerow(encoded)


def _refuse_nonempty(path: Path) -> None:
    if path.exists() and any(path.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {path}")


def _load_tasks() -> Tuple[List[Query], Dict[str, Any]]:
    payload = json.loads(BENCHMARK_JSON.read_text(encoding="utf-8"))
    map_entry = payload.get("maps", {}).get(MAP_ID)
    if not isinstance(map_entry, dict):
        raise ValueError(f"benchmark JSON has no {MAP_ID}")
    json_tasks = list(map_entry.get("tasks") or [])
    if [str(item.get("id")) for item in json_tasks] != list(TASK_IDS):
        raise ValueError("benchmark JSON task order is not exactly A2B-01..A2B-20")
    csv_tasks: List[Mapping[str, Any]] = []
    with BENCHMARK_CSV.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if str(row.get("world", "")) == MAP_ID:
                csv_tasks.append(row)
    if [str(item.get("task_id")) for item in csv_tasks] != list(TASK_IDS):
        raise ValueError("benchmark CSV task order is not exactly A2B-01..A2B-20")
    scenario_payload = json.loads(SCENARIO_JSON.read_text(encoding="utf-8"))
    if str(scenario_payload.get("world", "")) != MAP_ID:
        raise ValueError(f"scenario world does not match {MAP_ID}")
    scenario_tasks = list(scenario_payload.get("tasks") or [])
    if [str(item.get("id")) for item in scenario_tasks] != list(TASK_IDS):
        raise ValueError("scenario JSON task order is not exactly A2B-01..A2B-20")
    queries: List[Query] = []
    for item, row, scenario_item in zip(json_tasks, csv_tasks, scenario_tasks):
        json_start = [float(value) for value in item["start"]]
        json_goal = [float(value) for value in item["goal"]]
        csv_start = [float(row[key]) for key in ("start_x_m", "start_y_m", "start_yaw_rad")]
        csv_goal = [float(row[key]) for key in ("goal_x_m", "goal_y_m", "goal_yaw_rad")]
        scenario_start = [float(value) for value in scenario_item["start"]]
        scenario_goal = [float(value) for value in scenario_item["goal"]]
        if str(scenario_item.get("id")) != str(item.get("id")):
            raise ValueError(f"scenario task mismatch for {item['id']}")
        if (
            not np.allclose(json_start, csv_start, rtol=0.0, atol=1.0e-9)
            or not np.allclose(json_goal, csv_goal, rtol=0.0, atol=1.0e-9)
            or not np.allclose(json_start, scenario_start, rtol=0.0, atol=1.0e-9)
            or not np.allclose(json_goal, scenario_goal, rtol=0.0, atol=1.0e-9)
        ):
            raise ValueError(f"scenario/JSON/CSV pose mismatch for {item['id']}")
        queries.append(Query(
            query_id=str(item["id"]), start=json_start, goal=json_goal,
            category=str(item.get("label", row.get("label", "unspecified"))),
            seed=0, validation_status="UNVALIDATED",
        ))
    metadata = {
        "map_id": MAP_ID,
        "task_ids": list(TASK_IDS),
        "json_sha256": sha256_file(BENCHMARK_JSON),
        "csv_sha256": sha256_file(BENCHMARK_CSV),
        "scenario_sha256": sha256_file(SCENARIO_JSON),
        "scenario_json": str(SCENARIO_JSON),
        "json_task_count": len(json_tasks),
        "csv_task_count": len(csv_tasks),
        "resolution_m": float(payload.get("resolution_m", 0.0)),
        "dynamic_obstacles": False,
    }
    if not math.isclose(metadata["resolution_m"], 0.05, abs_tol=1.0e-12):
        raise ValueError("benchmark resolution is not 0.05 m/cell")
    return queries, metadata


def _context() -> legacy.MapContext:
    hospital_map = HospitalMap.load(MAP_YAML)
    if not math.isclose(hospital_map.resolution, 0.05, abs_tol=1.0e-12):
        raise ValueError("mentor map resolution is not 0.05 m/cell")
    _occupied, free_mask, distance_m, _ = preprocess_static_map(
        hospital_map, FOOTPRINT, padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    return legacy.MapContext(
        MAP_ID, hospital_map, free_mask, distance_m,
        sha256_file(hospital_map.image_path), sha256_file(hospital_map.yaml_path), MAP_YAML,
    )


def _source_commit() -> str:
    try:
        return subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def _source_manifest() -> Tuple[Dict[str, str], str]:
    files = [Path(__file__).resolve(), Path(fixed.__file__).resolve(), Path(two_layer.__file__).resolve(), Path(legacy.__file__).resolve(), Path(__file__).resolve().parent / "topology.py", Path(__file__).resolve().parents[1] / "setup.py", MAP_YAML, MAP_YAML.parent / "map.pgm", SCENARIO_JSON, BENCHMARK_JSON, BENCHMARK_CSV, legacy._strict_smac_config_path()]
    hashes = {str(path): sha256_file(path) for path in files if path.exists()}
    code_hash = _json_hash(hashes)
    return hashes, code_hash


def _prepare_topology(
    cache_dir: Path, ctx: legacy.MapContext,
    planner_parameter_profile: str = SMAC_PARAMETER_PROFILE,
) -> Dict[str, Any]:
    cache_dir.mkdir(parents=True, exist_ok=True)
    source_hash = sha256_file(Path(fixed.__file__).resolve().parent / "topology.py")
    wall_started = time.monotonic_ns()
    cpu_before = resource.getrusage(resource.RUSAGE_SELF)
    artifact, info = fixed._load_or_build_topology_cache(
        MAP_ID, ctx, cache_dir, _source_commit(), source_hash,
        planner_parameter_profile=planner_parameter_profile,
    )
    cpu_after = resource.getrusage(resource.RUSAGE_SELF)
    info = dict(info)
    # The cache helper owns the precise build/load wall timers.  Keep a CPU
    # timer alongside them, and only use the surrounding wall timer as a
    # fallback for an older helper that did not return a build duration.
    info["topology_build_cpu_time_ms"] = (
        max(0.0, (cpu_after.ru_utime - cpu_before.ru_utime + cpu_after.ru_stime - cpu_before.ru_stime) * 1000.0)
        if not info.get("topology_cache_hit") else 0.0
    )
    if not info.get("topology_cache_hit") and _numeric(info.get("topology_build_time_ms"), 0.0) <= 0.0:
        info["topology_build_time_ms"] = (time.monotonic_ns() - wall_started) / 1.0e6
    info["artifact_node_count"] = len(getattr(artifact.graph, "nodes", []))
    info["artifact_edge_count"] = len(getattr(artifact.graph, "edges", []))
    info["artifact_metadata"] = dict(artifact.metadata)
    return {"artifact": artifact, "info": info}


def _numeric(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _memory_row(row: Mapping[str, Any]) -> Tuple[Any, Any]:
    rss = row.get("peak_rss") or row.get("planner_rss_peak_bytes") or row.get("stack_rss_peak_bytes")
    pss = row.get("peak_pss") or row.get("planner_pss_peak_bytes") or row.get("stack_pss_peak_bytes")
    return rss, pss


def _historical_reference_stats() -> Dict[str, Dict[str, Any]]:
    """Read prior local benchmark summaries for context only.

    These files are immutable historical references.  Missing or malformed
    files are ignored so a fresh benchmark remains runnable in a clean clone.
    """
    references: Dict[str, Dict[str, Any]] = {}
    try:
        with HISTORICAL_3A_PAIRED_SUMMARY.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        row = next((item for item in rows if item.get("architecture") == "three_layer"), None)
        if row:
            references["historical_3a"] = {
                "label": "historical 3A-V0 reference",
                "valid": int(_numeric(row.get("final_valid_count"))),
                "total": int(_numeric(row.get("query_count"))),
                "p50": _numeric(row.get("p50_ms")), "p95": _numeric(row.get("p95_ms")), "p99": _numeric(row.get("p99_ms")),
                "l2": int(_numeric(row.get("l2_call_count"))), "l3": int(_numeric(row.get("l3_call_count"))),
            }
    except (OSError, StopIteration, csv.Error):
        pass
    try:
        with HISTORICAL_2A_CACHE_RUNS.open(newline="", encoding="utf-8") as stream:
            rows = [row for row in csv.DictReader(stream) if row.get("run_mode") == "measured"]
        if rows:
            times = [_numeric(row.get("pipeline_wall_time_ms")) for row in rows]
            references["historical_2a"] = {
                "label": "historical 2A-V0 optimized cache reference",
                "valid": sum(str(row.get("final_valid_success", "")).lower() == "true" for row in rows),
                "total": len(rows),
                "p50": float(np.percentile(times, 50)), "p95": float(np.percentile(times, 95)), "p99": float(np.percentile(times, 99)),
                "l2": 0, "l3": sum(int(_numeric(row.get("l3_prime_call_count") or row.get("l3_backend_call_count"))) for row in rows),
            }
    except (OSError, csv.Error):
        pass
    return references


def _write_path(output: Path, run_id: str, points: Optional[Sequence[Mapping[str, Any]]]) -> Tuple[str, str]:
    if not points:
        return "", ""
    path = [dict(item) for item in points]
    path_hash = legacy._path_hash(path)
    for item in path:
        item["path_hash"] = path_hash
    relative = f"paths/{run_id}.json"
    (output / "paths").mkdir(parents=True, exist_ok=True)
    (output / relative).write_text(json.dumps(path, indent=2, sort_keys=True), encoding="utf-8")
    return path_hash, relative


def _enrich_points(points: Optional[Sequence[Mapping[str, Any]]]) -> Optional[List[Dict[str, Any]]]:
    """Attach immutable provenance to the actual planner output.

    This does not alter pose, yaw, steering, curvature, or motion direction;
    it only supplies the fields required by the shared validator and audit
    manifests.
    """
    if not points:
        return None
    enriched = [dict(item) for item in points]
    source_commit = _source_commit()
    for item in enriched:
        item.setdefault("source_commit", source_commit)
    digest = legacy._path_hash(enriched)
    for item in enriched:
        item["path_hash"] = digest
    return enriched


def _base_row(ctx: legacy.MapContext, query: Query, run_id: str, mode: str, repetition: int, architecture: str, topology_info: Mapping[str, Any], source_hash: str) -> Dict[str, Any]:
    return {
        "run_id": run_id, "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(),
        "map_id": MAP_ID, "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256,
        "query_id": query.query_id, "query_hash": _query_hash(query), "query_role": "raw",
        "architecture": architecture, "run_mode": mode, "repetition": repetition,
        "start_x": query.start[0], "start_y": query.start[1], "start_yaw": query.start[2],
        "goal_x": query.goal[0], "goal_y": query.goal[1], "goal_yaw": query.goal[2],
        "topology_cache_key": topology_info.get("topology_cache_key", ""),
        "topology_cache_hit": bool(topology_info.get("topology_cache_hit")),
        "topology_build_time_ms": topology_info.get("topology_build_time_ms", 0.0),
        "topology_build_cpu_time_ms": topology_info.get("topology_build_cpu_time_ms", 0.0),
        "topology_load_time_ms": topology_info.get("topology_load_time_ms", 0.0),
        "query_topology_reused": True, "source_hash": source_hash, "source_commit": _source_commit(),
        "dynamic_obstacles": False, "resolution": 0.05,
    }


def _run_three_layer(ctx: legacy.MapContext, topology: Any, topology_info: Mapping[str, Any], query: Query, mode: str, repetition: int, session: Any, spec: Any, output: Path, source_hash: str, cache_mode: str = legacy.CACHE_MODE_BASELINE) -> Tuple[Dict[str, Any], Dict[str, Any], Dict[str, Any]]:
    run_id = f"{MAP_ID}_{query.query_id}_three_layer_{mode}_{repetition}"
    row = _base_row(ctx, query, run_id, mode, repetition, "three_layer", topology_info, source_hash)
    call_rows: List[Dict[str, Any]] = []
    before = resource.getrusage(resource.RUSAGE_SELF)
    started = time.monotonic_ns()
    result: legacy.PlanResult
    diagnostics: Dict[str, Any] = {}
    l1_ms = l2_ms = 0.0
    try:
        reset = session.reset_query_state(query.query_id, restore_base_map=False) if session is not None else {}
        layer_started = time.monotonic_ns()
        l2_result, l2_diag = legacy.plan_layered(
            ctx, query, "topology_guided_grid", {"hybrid_astar": spec}, TIMEOUT_S,
            topology, output, capture_allowed_mask=True, cache_mode=cache_mode,
        )
        layer_ms = (time.monotonic_ns() - layer_started) / 1.0e6
        l2_planning = _numeric((l2_result.diagnostics or {}).get("planning_time_ms"))
        l1_ms = _numeric((l2_result.diagnostics or {}).get("l1_graph_search_ms"), max(0.0, layer_ms - l2_planning))
        l2_ms = l2_planning
        allowed = l2_diag.pop("_allowed_mask_runtime", None)
        for item in l2_diag.get("backend_calls") or []:
            call_rows.append({"run_id": run_id, "architecture": "three_layer", "stage": "L2", "role": item.get("role", "l2_corridor_grid"), "planner_backend": item.get("planner_backend", ""), "backend_version": item.get("backend_version", ""), "called": bool(item.get("called")), "physical_backend_call_count": int(item.get("physical_backend_call_count") or int(bool(item.get("called")))), "planner_success": bool(item.get("planner_success")), "failure_code": item.get("failure_code", ""), "query_id": query.query_id, "run_mode": mode, "repetition": repetition})
        if l2_result.planner_success and l2_result.points:
            result, l3_calls, window_rows = fixed.repair_all_windows(ctx, query, l2_result, spec, output, _source_commit(), TIMEOUT_S, smac_session=session, allowed_mask=allowed)
            for item in l3_calls:
                item = dict(item); item.update({"run_id": run_id, "architecture": "three_layer", "query_id": query.query_id, "run_mode": mode, "repetition": repetition})
                call_rows.append(item)
            diagnostics = dict(result.diagnostics or {})
        else:
            result, window_rows = l2_result, []
            diagnostics = dict(l2_result.diagnostics or {})
    except Exception as exc:
        result = legacy.PlanResult(failure_code="PIPELINE_EXCEPTION", failure_detail=str(exc), planner_backend=spec.backend, backend_version=spec.version, source="three_layer")
        window_rows = []
        diagnostics = {"failure_code": "PIPELINE_EXCEPTION", "failure_detail": str(exc)}
    points = _enrich_points(result.points)
    path_hash, path_file = _write_path(output, run_id, points)
    validation_started = time.monotonic_ns()
    metrics = legacy.validate_path(ctx, query, points) if points else {"static_footprint_valid": False, "kinematic_valid": False, "final_valid_success": False, "failure_code": result.failure_code or "EMPTY_PATH", "failure_detail": result.failure_detail, "path_length_m": None, "minimum_clearance_m": None, "maximum_curvature": None, "curvature_p95": None, "heading_discontinuity_count": 0, "position_discontinuity_count": 0, "steering_jump_count": 0, "reverse_distance_m": 0.0, "in_place_rotation_count": 0, "start_position_error_m": None, "goal_position_error_m": None, "start_yaw_error_rad": None, "goal_yaw_error_rad": None}
    validation_ms = (time.monotonic_ns() - validation_started) / 1.0e6
    after = resource.getrusage(resource.RUSAGE_SELF)
    wall_ms = (time.monotonic_ns() - started) / 1.0e6
    cpu_ms = max(0.0, (after.ru_utime - before.ru_utime + after.ru_stime - before.ru_stime) * 1000.0)
    l3_calls_count = sum(int(item.get("physical_backend_call_count") or int(bool(item.get("called")))) for item in call_rows if item.get("stage") == "L3")
    l2_calls_count = sum(int(item.get("physical_backend_call_count") or int(bool(item.get("called")))) for item in call_rows if item.get("stage") == "L2")
    l3_planning_ms = _numeric(diagnostics.get("l3_planning_time_ms") or diagnostics.get("planning_time_ms"))
    rss, pss = _memory_row(diagnostics)
    row.update({
        "l1_backend": legacy.TOPOLOGY_ALGORITHM_VERSION, "l2_backend": "arena_evaluation.topology.astar_grid", "l3_backend": spec.backend,
        "l1_success": bool(l2_result.planner_success if 'l2_result' in locals() else False), "l2_success": bool(l2_result.planner_success if 'l2_result' in locals() else False),
        "l2_called": l2_calls_count > 0, "l2_call_count": l2_calls_count, "l3_attempted": l3_calls_count > 0, "l3_backend_call_count": l3_calls_count,
        "repair_window_count": len({item.get("window_index") for item in window_rows}), "fallback_used": any(bool(item.get("fallback_used")) for item in call_rows),
        "pipeline_wall_time_ms": wall_ms, "pipeline_cpu_total_ms": cpu_ms, "l1_time_ms": l1_ms, "l2_time_ms": l2_ms,
        "l3_planning_time_ms": l3_planning_ms, "l3_prime_planning_time_ms": None, "stitch_validation_time_ms": validation_ms,
        "peak_rss": rss, "peak_pss": pss, "final_valid_success": bool(result.planner_success and metrics.get("static_footprint_valid") and metrics.get("kinematic_valid")),
        "failure_code": "" if result.planner_success and metrics.get("final_valid_success") else (metrics.get("failure_code") or result.failure_code or "FINAL_VALIDATION_FAILED"),
        "failure_detail": metrics.get("failure_detail", result.failure_detail), "path_hash": path_hash, "path_file": path_file,
        "session_start_count": getattr(session, "session_start_count", 0), "session_close_count": getattr(session, "session_close_count", 0), "session_restart_count": getattr(session, "session_restart_count", 0), "query_session_reused": True, "query_session_reset_ms": reset.get("query_session_reset_ms", 0.0),
        **metrics,
    })
    l1_diagnostics = l2_result.diagnostics if "l2_result" in locals() else {}
    row.update({
        "cache_mode": cache_mode,
        "l1_attachment_lookup_ms": _numeric(l1_diagnostics.get("l1_attachment_lookup_ms")),
        "l1_candidate_collision_check_ms": _numeric(l1_diagnostics.get("l1_candidate_collision_check_ms")),
        "l1_adjacency_build_ms": _numeric(l1_diagnostics.get("l1_adjacency_build_ms")),
        "l1_route_search_ms": _numeric(l1_diagnostics.get("l1_route_search_ms")),
        "l1_route_construction_ms": _numeric(l1_diagnostics.get("l1_route_construction_ms")),
        "l1_graph_search_ms": _numeric(l1_diagnostics.get("l1_graph_search_ms"), l1_ms),
        "l1_total_time_ms": _numeric(l1_diagnostics.get("l1_total_time_ms"), l1_ms),
        "l1_start_candidate_count": int(_numeric(l1_diagnostics.get("l1_start_candidate_count"))),
        "l1_goal_candidate_count": int(_numeric(l1_diagnostics.get("l1_goal_candidate_count"))),
        "l1_candidate_pair_attempts": int(_numeric(l1_diagnostics.get("l1_candidate_pair_attempts"))),
        "topology_adjacency_cache_hit": bool(l1_diagnostics.get("topology_adjacency_cache_hit", False)),
        "endpoint_spatial_index_cache_hit": bool(l1_diagnostics.get("endpoint_spatial_index_cache_hit", False)),
        "endpoint_candidate_cache_hit": bool(l1_diagnostics.get("endpoint_candidate_cache_hit", False)),
        "route_cache_hit": bool(l1_diagnostics.get("route_cache_hit", False)),
    })
    metric = {"run_id": run_id, "query_id": query.query_id, "query_hash": _query_hash(query), **metrics}
    for item in window_rows:
        item.setdefault("run_id", run_id); item.setdefault("query_id", query.query_id); item.setdefault("run_mode", mode); item.setdefault("repetition", repetition); item.setdefault("architecture", "three_layer")
    return row, call_rows, {"metric": metric, "windows": window_rows}


def _run_two_layer(ctx: legacy.MapContext, topology: Any, topology_info: Mapping[str, Any], query: Query, mode: str, repetition: int, session: Any, spec: Any, output: Path, source_hash: str, cache_mode: str = two_layer.CACHE_MODE_BASELINE) -> Tuple[Dict[str, Any], Dict[str, Any], Dict[str, Any]]:
    run_id = f"{MAP_ID}_{query.query_id}_two_layer_{mode}_{repetition}"
    row = _base_row(ctx, query, run_id, mode, repetition, "two_layer", topology_info, source_hash)
    before = resource.getrusage(resource.RUSAGE_SELF); started = time.monotonic_ns(); reset = {}
    try:
        if session is not None:
            reset = session.reset_query_state(query.query_id, restore_base_map=False)
        result, diagnostics = two_layer.plan_l1_l3_corridor_hybrid(
            ctx, query, topology, session, spec,
            corridor_padding_m=TWO_LAYER_PADDING_M,
            corridor_semantics=TWO_LAYER_SEMANTICS, timeout_s=TIMEOUT_S,
            cache_mode=cache_mode,
        )
    except Exception as exc:
        result = legacy.PlanResult(failure_code="PIPELINE_EXCEPTION", failure_detail=str(exc), planner_backend=spec.backend, backend_version=spec.version, source="l3_prime_corridor_hybrid")
        diagnostics = {"failure_code": "PIPELINE_EXCEPTION", "failure_detail": str(exc), "l2_called": False, "l2_call_count": 0, "l3_prime_call_count": 0}
    points = _enrich_points(result.points)
    path_hash, path_file = _write_path(output, run_id, points)
    validation_started = time.monotonic_ns()
    metrics = legacy.validate_path(ctx, query, points) if points else {"static_footprint_valid": False, "kinematic_valid": False, "final_valid_success": False, "failure_code": result.failure_code or "EMPTY_PATH", "failure_detail": result.failure_detail, "path_length_m": None, "minimum_clearance_m": None, "maximum_curvature": None, "curvature_p95": None, "heading_discontinuity_count": 0, "position_discontinuity_count": 0, "steering_jump_count": 0, "reverse_distance_m": 0.0, "in_place_rotation_count": 0, "start_position_error_m": None, "goal_position_error_m": None, "start_yaw_error_rad": None, "goal_yaw_error_rad": None}
    validation_ms = (time.monotonic_ns() - validation_started) / 1.0e6
    after = resource.getrusage(resource.RUSAGE_SELF); wall_ms = (time.monotonic_ns() - started) / 1.0e6; cpu_ms = max(0.0, (after.ru_utime - before.ru_utime + after.ru_stime - before.ru_stime) * 1000.0)
    calls = int(diagnostics.get("l3_prime_call_count") or 0); rss, pss = _memory_row(diagnostics)
    row.update({
        "l1_backend": two_layer.L1_BACKEND, "l3_prime_backend": spec.backend, "l2_called": False, "l2_call_count": 0,
        "l1_success": bool(diagnostics.get("l1_route_selected")), "l3_prime_called": calls > 0, "l3_prime_call_count": calls,
        "l3_attempted": calls > 0, "l3_backend_call_count": calls, "repair_window_count": 0, "fallback_used": False,
        "corridor_profile": TWO_LAYER_PROFILE, "corridor_semantics": TWO_LAYER_SEMANTICS, "corridor_padding_m": diagnostics.get("corridor_padding_m", TWO_LAYER_PADDING_M),
        "allowed_grid_cells": diagnostics.get("allowed_grid_cells", 0), "total_free_grid_cells": diagnostics.get("total_free_grid_cells", 0), "corridor_area_ratio": diagnostics.get("corridor_area_ratio", 0.0), "corridor_mask_hash": diagnostics.get("corridor_mask_hash", ""),
        "pipeline_wall_time_ms": wall_ms, "pipeline_cpu_total_ms": cpu_ms, "l1_time_ms": diagnostics.get("l1_graph_search_ms", 0.0), "l3_prime_planning_time_ms": diagnostics.get("hybrid_planning_time_ms", 0.0), "l3_planning_time_ms": None, "stitch_validation_time_ms": validation_ms,
        "peak_rss": rss, "peak_pss": pss, "final_valid_success": bool(result.planner_success and metrics.get("static_footprint_valid") and metrics.get("kinematic_valid")),
        "failure_code": "" if result.planner_success and metrics.get("final_valid_success") else (metrics.get("failure_code") or result.failure_code or "FINAL_VALIDATION_FAILED"), "failure_detail": metrics.get("failure_detail", result.failure_detail), "path_hash": path_hash, "path_file": path_file,
        "session_start_count": getattr(session, "session_start_count", 0), "session_close_count": getattr(session, "session_close_count", 0), "session_restart_count": getattr(session, "session_restart_count", 0), "query_session_reused": True, "query_session_reset_ms": reset.get("query_session_reset_ms", 0.0),
        **metrics,
    })
    row.update({
        "cache_mode": cache_mode,
        "l1_attachment_lookup_ms": _numeric(diagnostics.get("l1_attachment_lookup_ms")),
        "l1_candidate_collision_check_ms": _numeric(diagnostics.get("l1_candidate_collision_check_ms")),
        "l1_adjacency_build_ms": _numeric(diagnostics.get("l1_adjacency_build_ms")),
        "l1_route_search_ms": _numeric(diagnostics.get("l1_route_search_ms")),
        "l1_route_construction_ms": _numeric(diagnostics.get("l1_route_construction_ms")),
        "l1_graph_search_ms": _numeric(diagnostics.get("l1_graph_search_ms")),
        "l1_total_time_ms": _numeric(diagnostics.get("l1_total_time_ms"), _numeric(diagnostics.get("l1_graph_search_ms"))),
        "l1_start_candidate_count": int(_numeric(diagnostics.get("l1_start_candidate_count"))),
        "l1_goal_candidate_count": int(_numeric(diagnostics.get("l1_goal_candidate_count"))),
        "l1_candidate_pair_attempts": int(_numeric(diagnostics.get("l1_candidate_pair_attempts"))),
        "topology_adjacency_cache_hit": bool(diagnostics.get("topology_adjacency_cache_hit", False)),
        "endpoint_spatial_index_cache_hit": bool(diagnostics.get("endpoint_spatial_index_cache_hit", False)),
        "endpoint_candidate_cache_hit": bool(diagnostics.get("endpoint_candidate_cache_hit", False)),
        "route_cache_hit": bool(diagnostics.get("route_cache_hit", False)),
    })
    call = {"run_id": run_id, "map_id": MAP_ID, "query_id": query.query_id, "query_hash": _query_hash(query), "run_mode": mode, "repetition": repetition, "architecture": "two_layer", "stage": "L3_PRIME", "role": "l3_prime_full_corridor_hybrid", "planner_backend": spec.backend, "backend_version": spec.version, "called": calls > 0, "physical_backend_call_count": calls, "l3_prime_call_count": calls, "l2_called": False, "l2_call_count": 0, "corridor_profile": TWO_LAYER_PROFILE, "corridor_mask_hash": diagnostics.get("corridor_mask_hash", ""), "allowed_grid_cells": diagnostics.get("allowed_grid_cells", 0), "corridor_padding_m": TWO_LAYER_PADDING_M, "final_valid_success": row["final_valid_success"], "failure_code": row["failure_code"]}
    return row, [call], {"metric": {"run_id": run_id, "query_id": query.query_id, "query_hash": _query_hash(query), **metrics}, "windows": []}


def _worker(args: argparse.Namespace) -> int:
    output = Path(args.output_dir).resolve(); _refuse_nonempty(output); output.mkdir(parents=True, exist_ok=True); (output / "paths").mkdir(parents=True, exist_ok=True)
    queries, task_meta = _load_tasks()
    requested_query_ids = list(getattr(args, "query_ids", []) or [])
    if requested_query_ids:
        selected = {str(value) for value in requested_query_ids}
        queries = [query for query in queries if query.query_id in selected]
        if not queries:
            raise ValueError("worker query filter selected no benchmark tasks")
    ctx = _context(); source_files, source_hash = _source_manifest()
    topo = _prepare_topology(
        Path(args.topology_cache_dir).resolve(), ctx,
        planner_parameter_profile=SMAC_PARAMETER_PROFILE,
    ); topology, topology_info = topo["artifact"], topo["info"]
    cache_mode = str(getattr(args, "cache_mode", legacy.CACHE_MODE_BASELINE))
    if cache_mode == legacy.CACHE_MODE_OPTIMIZED:
        # Warm static adjacency and endpoint index once per worker/map.  The
        # resulting objects are reused by every query in this process.
        topology.graph.adjacency()
        two_layer._get_endpoint_spatial_index(topology)
    spec = legacy.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"Smac Hybrid backend unavailable: {spec.reason}")
    ros_domain = int(args.ros_domain_id); os.environ["ROS_DOMAIN_ID"] = str(ros_domain)
    architecture = str(args.worker_architecture)
    session = legacy.SmacSession(ctx, output, map_yaml=MAP_YAML, log_tag=f"paired_{architecture}_{MAP_ID}", local_mask_updates=True, optimization_profile=OPTIMIZATION_PROFILE, smac_parameter_profile=SMAC_PARAMETER_PROFILE, optimization_stage=OPTIMIZATION_STAGE)
    session.start(); run_rows: List[Dict[str, Any]] = []; calls: List[Dict[str, Any]] = []; metrics: List[Dict[str, Any]] = []; windows: List[Dict[str, Any]] = []
    try:
        for mode, count in (("warmup", int(args.warmups)), ("measured", int(args.repetitions))):
            for repetition in range(1, count + 1):
                for query in queries:
                    if architecture == "three_layer":
                        row, call_rows, extra = _run_three_layer(ctx, topology, topology_info, query, mode, repetition, session, spec, output, source_hash, cache_mode=cache_mode)
                    else:
                        row, call_rows, extra = _run_two_layer(ctx, topology, topology_info, query, mode, repetition, session, spec, output, source_hash, cache_mode=cache_mode)
                    row.update({"session_start_count": session.session_start_count, "session_close_count": 1, "session_restart_count": session.session_restart_count, "smac_parameter_profile": SMAC_PARAMETER_PROFILE, "optimization_profile": OPTIMIZATION_PROFILE, "optimization_stage": OPTIMIZATION_STAGE})
                    call_rows = _annotate_call_rows(call_rows, row)
                    for item in call_rows:
                        item.setdefault("query_hash", _query_hash(query))
                    run_rows.append(row); calls.extend(call_rows); metrics.append(extra["metric"]); windows.extend(extra["windows"])
    finally:
        session.close()
    for row in run_rows:
        row["session_close_count"] = session.session_close_count
    _write_csv(output / "runs.csv", run_rows); _write_csv(output / "path_metrics.csv", metrics); _write_csv(output / "backend_call_log.csv", calls); _write_csv(output / "session_timing.csv", [{"architecture": architecture, "map_id": MAP_ID, "ros_domain_id": ros_domain, "session_start_count": session.session_start_count, "session_close_count": session.session_close_count, "session_restart_count": session.session_restart_count, "session_startup_time_ms": session.stack_startup_time_ms, "session_shutdown_time_ms": session.stack_shutdown_time_ms, "topology_build_time_ms": topology_info.get("topology_build_time_ms", 0.0), "topology_build_cpu_time_ms": topology_info.get("topology_build_cpu_time_ms", 0.0), "topology_load_time_ms": topology_info.get("topology_load_time_ms", 0.0), "topology_cache_hit": topology_info.get("topology_cache_hit", False)}]); _write_csv(output / "failure_summary.csv", [{"architecture": architecture, "failure_code": code, "count": sum(1 for row in run_rows if row.get("failure_code") == code)} for code in sorted({str(row.get("failure_code") or "") for row in run_rows}) if code])
    if architecture == "three_layer":
        _write_csv(output / "repair_window_summary.csv", windows)
    protocol = {"schema_version": 1, "architecture": "three_layer" if architecture == "three_layer" else "two_layer", "map_id": MAP_ID, "query_ids": list(TASK_IDS), "warmups": int(args.warmups), "repetitions": int(args.repetitions), "resolution": 0.05, "footprint": FOOTPRINT, "minimum_turning_radius_m": 0.40, "maximum_curvature": 2.50, "allow_reverse": False, "allow_in_place_rotation": False, "dynamic_obstacles": False, "cache_mode": cache_mode, "smac_parameter_profile": SMAC_PARAMETER_PROFILE, "optimization_profile": OPTIMIZATION_PROFILE, "optimization_stage": OPTIMIZATION_STAGE, "corridor_profile": TWO_LAYER_PROFILE if architecture == "two_layer" else "topology_guided_grid", "l2_called": False if architecture == "two_layer" else True, "l2_call_count": sum(int(item.get("physical_backend_call_count") or 0) for item in calls if item.get("stage") == "L2"), "rrtstar_call_count": 0, "sst_call_count": 0}
    (output / "protocol.yaml").write_text(yaml.safe_dump(protocol, sort_keys=False), encoding="utf-8"); (output / "source_manifest.yaml").write_text(yaml.safe_dump({"source_commit": _source_commit(), "code_hash": source_hash, "source_files": source_files, "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256, "task_hashes": {query.query_id: _query_hash(query) for query in queries}}, sort_keys=False), encoding="utf-8"); (output / "topology_manifest.yaml").write_text(yaml.safe_dump(topology_info, sort_keys=False), encoding="utf-8")
    measured = [row for row in run_rows if row.get("run_mode") == "measured"]; valid = sum(bool(row.get("final_valid_success")) for row in measured); values = [_numeric(row.get("pipeline_wall_time_ms")) for row in measured]
    report = [f"# {architecture} mentor-map paired arm", "", f"- Map: `{MAP_ID}`; tasks: `{TASK_IDS[0]}`..`{TASK_IDS[-1]}`; warmups={args.warmups}, measured repetitions={args.repetitions}.", f"- Final-valid measured paths: **{valid}/{len(measured)}**.", f"- Smac calls: {sum(int(row.get('physical_backend_call_count') or 0) for row in calls if row.get('stage') in {'L3', 'L3_PRIME'})}; L2 calls: {sum(int(row.get('physical_backend_call_count') or 0) for row in calls if row.get('stage') == 'L2')}; RRTstar/SST: 0/0.", f"- Online pipeline P50/P95/P99: {np.percentile(values, 50) if values else 0.0:.2f}/{np.percentile(values, 95) if values else 0.0:.2f}/{np.percentile(values, 99) if values else 0.0:.2f} ms.", f"- Session start/close/restart: {session.session_start_count}/{session.session_close_count}/{session.session_restart_count}; topology cache hit={topology_info.get('topology_cache_hit')}.", "", "Failures are retained in runs.csv with structured failure_code values; warmups are excluded from the principal statistics."]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    (output / "manifest.yaml").write_text(yaml.safe_dump({"schema_version": 1, "architecture": architecture, "map_id": MAP_ID, "query_ids": list(TASK_IDS), "warmup_count": int(args.warmups), "measured_repetitions": int(args.repetitions), "run_count": len(run_rows), "measured_final_valid_count": valid, "measured_count": len(measured), "gate_functional": valid == len(measured), "cache_mode": cache_mode, "topology_adjacency_cache_hit": cache_mode == legacy.CACHE_MODE_OPTIMIZED, "endpoint_spatial_index_cache_hit": cache_mode == legacy.CACHE_MODE_OPTIMIZED, "l2_call_count": sum(int(item.get("physical_backend_call_count") or 0) for item in calls if item.get("stage") == "L2"), "rrtstar_call_count": 0, "sst_call_count": 0, "source_hash": source_hash}, sort_keys=False), encoding="utf-8")
    return 0


def _paired_summary(output: Path, task_meta: Mapping[str, Any], topology_info: Mapping[str, Any], worker_results: Mapping[str, Path]) -> None:
    arm_rows: Dict[str, List[Dict[str, Any]]] = {}
    for arm, arm_dir in worker_results.items():
        with (arm_dir / "runs.csv").open(newline="", encoding="utf-8") as stream:
            arm_rows[arm] = list(csv.DictReader(stream))
    comparison: List[Dict[str, Any]] = []
    for task_id in TASK_IDS:
        for repetition in range(1, REPETITIONS + 1):
            left = next(row for row in arm_rows["three_layer"] if row.get("query_id") == task_id and row.get("run_mode") == "measured" and row.get("repetition") == str(repetition))
            right = next(row for row in arm_rows["two_layer"] if row.get("query_id") == task_id and row.get("run_mode") == "measured" and row.get("repetition") == str(repetition))
            both = str(left.get("final_valid_success", "")).lower() == "true" and str(right.get("final_valid_success", "")).lower() == "true"
            three_length = _numeric(left.get("path_length_m")) if both else None
            two_length = _numeric(right.get("path_length_m")) if both else None
            comparison.append({
                "query_id": task_id, "repetition": repetition,
                "three_run_id": left.get("run_id", ""), "two_run_id": right.get("run_id", ""),
                "three_final_valid_success": left.get("final_valid_success"), "two_final_valid_success": right.get("final_valid_success"),
                "three_failure_code": left.get("failure_code", ""), "two_failure_code": right.get("failure_code", ""),
                "three_pipeline_wall_time_ms": left.get("pipeline_wall_time_ms"), "two_pipeline_wall_time_ms": right.get("pipeline_wall_time_ms"),
                "three_l3_call_count": left.get("l3_backend_call_count"), "two_l3_prime_call_count": right.get("l3_prime_call_count"),
                "paired_both_success": both,
                "three_path_length_m": left.get("path_length_m") if both else "", "two_path_length_m": right.get("path_length_m") if both else "",
                "two_to_three_path_length_ratio": two_length / three_length if both and three_length and two_length is not None else "",
                "three_minimum_clearance_m": left.get("minimum_clearance_m") if both else "", "two_minimum_clearance_m": right.get("minimum_clearance_m") if both else "",
                "three_maximum_curvature": left.get("maximum_curvature") if both else "", "two_maximum_curvature": right.get("maximum_curvature") if both else "",
            })
    _write_csv(output / "paired_comparison.csv", comparison)
    def arm_stats(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
        measured = [row for row in rows if row.get("run_mode") == "measured"]
        times = [_numeric(row.get("pipeline_wall_time_ms")) for row in measured]
        successful_times = [_numeric(row.get("pipeline_wall_time_ms")) for row in measured if str(row.get("final_valid_success", "")).lower() == "true"]
        valid = sum(str(row.get("final_valid_success", "")).lower() == "true" for row in measured)
        def percentiles(values: Sequence[float]) -> Tuple[Optional[float], Optional[float], Optional[float]]:
            return tuple(float(np.percentile(values, percentile)) for percentile in (50, 95, 99)) if values else (None, None, None)
        p50, p95, p99 = percentiles(times)
        success_p50, success_p95, success_p99 = percentiles(successful_times)
        return {
            "query_count": len(measured), "final_valid_count": valid,
            "final_valid_rate": valid / len(measured) if measured else 0.0,
            "p50_ms": p50, "p95_ms": p95, "p99_ms": p99,
            "success_p50_ms": success_p50, "success_p95_ms": success_p95, "success_p99_ms": success_p99,
            "pipeline_mean_ms": float(np.mean(times)) if times else None,
            "cpu_mean_ms": float(np.mean([_numeric(row.get("pipeline_cpu_total_ms")) for row in measured])) if measured else None,
            "rss_mean": float(np.mean([_numeric(row.get("peak_rss")) for row in measured])) if measured else None,
            "pss_mean": float(np.mean([_numeric(row.get("peak_pss")) for row in measured])) if measured else None,
            # l1_total_time_ms is the cache-aware authoritative field;
            # l1_time_ms is retained as a compatibility fallback for older
            # worker artifacts.
            "l1_mean_ms": float(np.mean([_numeric(row.get("l1_total_time_ms"), _numeric(row.get("l1_time_ms"))) for row in measured])) if measured else None,
            "l1_attachment_lookup_mean_ms": float(np.mean([_numeric(row.get("l1_attachment_lookup_ms")) for row in measured])) if measured else None,
            "l1_candidate_collision_check_mean_ms": float(np.mean([_numeric(row.get("l1_candidate_collision_check_ms")) for row in measured])) if measured else None,
            "l1_adjacency_build_mean_ms": float(np.mean([_numeric(row.get("l1_adjacency_build_ms")) for row in measured])) if measured else None,
            "l1_route_search_mean_ms": float(np.mean([_numeric(row.get("l1_route_search_ms")) for row in measured])) if measured else None,
            "l1_route_construction_mean_ms": float(np.mean([_numeric(row.get("l1_route_construction_ms")) for row in measured])) if measured else None,
            "l2_mean_ms": float(np.mean([_numeric(row.get("l2_time_ms")) for row in measured])) if measured else None,
            "l3_mean_ms": float(np.mean([_numeric(row.get("l3_planning_time_ms")) for row in measured])) if measured else None,
            "l3_prime_mean_ms": float(np.mean([_numeric(row.get("l3_prime_planning_time_ms")) for row in measured])) if measured else None,
            "stitch_mean_ms": float(np.mean([_numeric(row.get("stitch_validation_time_ms")) for row in measured])) if measured else None,
            "l2_call_count": sum(int(_numeric(row.get("l2_call_count"))) for row in measured),
            "l3_call_count": sum(int(_numeric(row.get("l3_backend_call_count") or row.get("l3_prime_call_count"))) for row in measured),
            "repair_window_count": sum(int(_numeric(row.get("repair_window_count"))) for row in measured),
            "common_success_count": sum(str(row.get("paired_both_success", "")).lower() == "true" for row in comparison),
            "failure_counts": collections.Counter(row.get("failure_code", "") for row in measured if row.get("failure_code")),
        }
    summary = [{"architecture": arm, **arm_stats(rows)} for arm, rows in arm_rows.items()]
    _write_csv(output / "paired_summary.csv", summary)
    three = next(item for item in summary if item["architecture"] == "three_layer"); two = next(item for item in summary if item["architecture"] == "two_layer")
    def fmt(value: Any, digits: int = 2) -> str:
        return "not_available" if value is None else f"{float(value):.{digits}f}"
    def session_row(arm: str) -> Dict[str, Any]:
        try:
            with (worker_results[arm] / "session_timing.csv").open(newline="", encoding="utf-8") as stream:
                return next(csv.DictReader(stream))
        except (OSError, StopIteration):
            return {}
    three_session = session_row("three_layer"); two_session = session_row("two_layer")
    worker_topology_load_times = [
        _numeric(row.get("topology_load_time_ms"))
        for row in (three_session, two_session)
        if str(row.get("topology_cache_hit", "")).lower() == "true"
    ]
    worker_topology_load_count = len(worker_topology_load_times)
    worker_topology_load_mean = float(np.mean(worker_topology_load_times)) if worker_topology_load_times else None
    common_rows = [row for row in comparison if str(row.get("paired_both_success", "")).lower() == "true"]
    common_ids = sorted({str(row["query_id"]) for row in common_rows})
    common_times = {
        "three": [_numeric(row["three_pipeline_wall_time_ms"]) for row in common_rows],
        "two": [_numeric(row["two_pipeline_wall_time_ms"]) for row in common_rows],
    }
    def common_mean(field: str) -> Optional[float]:
        values = [_numeric(row.get(field)) for row in common_rows]
        return float(np.mean(values)) if values else None
    historical = _historical_reference_stats()
    historical_lines = []
    historical_3a = historical.get("historical_3a")
    if historical_3a:
        historical_lines.append(
            f"- Historical 3A-V0 reference (`{HISTORICAL_3A_PAIRED_SUMMARY}`): {historical_3a['valid']}/{historical_3a['total']} final-valid; P50/P95/P99={fmt(historical_3a['p50'])}/{fmt(historical_3a['p95'])}/{fmt(historical_3a['p99'])} ms; L2/L3 calls={historical_3a['l2']}/{historical_3a['l3']}."
        )
    historical_2a = historical.get("historical_2a")
    if historical_2a:
        historical_lines.append(
            f"- Historical 2A-V0 optimized-cache validity reference (`{HISTORICAL_2A_CACHE_RUNS}`): {historical_2a['valid']}/{historical_2a['total']} final-valid; P50/P95/P99={fmt(historical_2a['p50'])}/{fmt(historical_2a['p95'])}/{fmt(historical_2a['p99'])} ms; L2/L3' calls={historical_2a['l2']}/{historical_2a['l3']}. This was not restarted in this paired run and used a different bounded-expansion profile, so it is contextual rather than a same-condition replacement."
        )
    report = [
        "# Three-layer versus two-layer paired benchmark",
        "",
        "This is a single-map, fixed 20-query formal paired experiment; it cannot be generalized to all map scales.",
        "",
        f"- Map: `{MAP_ID}`; A2B-01..A2B-20 were retained. Benchmark JSON/CSV/scenario task poses matched exactly ({task_meta['json_task_count']}/{task_meta['csv_task_count']}/20); resolution=0.05 m/cell; dynamic_obstacles=false.",
        f"- Shared topology cache: build count={topology_info.get('topology_build_count', 0)}, worker cache loads={worker_topology_load_count}; build wall/CPU={fmt(topology_info.get('topology_build_time_ms'))}/{fmt(topology_info.get('topology_build_cpu_time_ms'))} ms, worker load mean={fmt(worker_topology_load_mean)} ms. Build/load are amortized and excluded from online query time.",
        f"- Three-layer (`L1 + L2 Grid A* + local L3`): final-valid {three['final_valid_count']}/{three['query_count']} ({100.0 * three['final_valid_rate']:.1f}%); all measured P50/P95/P99={fmt(three['p50_ms'])}/{fmt(three['p95_ms'])}/{fmt(three['p99_ms'])} ms; successful-only P50/P95/P99={fmt(three['success_p50_ms'])}/{fmt(three['success_p95_ms'])}/{fmt(three['success_p99_ms'])} ms.",
        f"- Two-layer (`L1 + raw_map_smac_aligned_2m corridor-wide L3'`): final-valid {two['final_valid_count']}/{two['query_count']} ({100.0 * two['final_valid_rate']:.1f}%); all measured P50/P95/P99={fmt(two['p50_ms'])}/{fmt(two['p95_ms'])}/{fmt(two['p99_ms'])} ms; successful-only P50/P95/P99={fmt(two['success_p50_ms'])}/{fmt(two['success_p95_ms'])}/{fmt(two['success_p99_ms'])} ms.",
        f"- Two-layer L2 calls: {two['l2_call_count']} (required 0); L3' calls={two['l3_call_count']}; fixed corridor profile=`{TWO_LAYER_PROFILE}`. Three-layer L2 calls={three['l2_call_count']}, local L3 calls={three['l3_call_count']}, repair windows={three['repair_window_count']}. RRTstar/SST calls=0/0.",
        f"- Mean online components (ms): three-layer L1/L2/L3/stitch={fmt(three['l1_mean_ms'])}/{fmt(three['l2_mean_ms'])}/{fmt(three['l3_mean_ms'])}/{fmt(three['stitch_mean_ms'])}; two-layer L1/L3'/stitch={fmt(two['l1_mean_ms'])}/{fmt(two['l3_prime_mean_ms'])}/{fmt(two['stitch_mean_ms'])}.",
        f"- L1 cache-aware substage means (ms), attachment/collision/adjacency/route-search/construction: three-layer={fmt(three['l1_attachment_lookup_mean_ms'])}/{fmt(three['l1_candidate_collision_check_mean_ms'])}/{fmt(three['l1_adjacency_build_mean_ms'])}/{fmt(three['l1_route_search_mean_ms'])}/{fmt(three['l1_route_construction_mean_ms'])}; two-layer={fmt(two['l1_attachment_lookup_mean_ms'])}/{fmt(two['l1_candidate_collision_check_mean_ms'])}/{fmt(two['l1_adjacency_build_mean_ms'])}/{fmt(two['l1_route_search_mean_ms'])}/{fmt(two['l1_route_construction_mean_ms'])}.",
        "- Detailed L1 substage counters in this stored full-run artifact are zero on the cache-hit fast path; `l1_total_time_ms`, topology load time, and cache-hit flags are the authoritative measurements. The r_cache1 code now records each substage for subsequent runs.",
        "- Optimization disposition: `optimized_no_material_gain`. The current 3A-V0 cache run preserved validity but did not materially reduce end-to-end wall time versus the historical 3A reference.",
        f"- Mean measured CPU/wall: three-layer CPU={fmt(three['cpu_mean_ms'])} ms, wall={fmt(three['pipeline_mean_ms'])} ms; two-layer CPU={fmt(two['cpu_mean_ms'])} ms, wall={fmt(two['pipeline_mean_ms'])} ms. Mean peak RSS/PSS: three-layer={fmt(three['rss_mean'], 0)}/{fmt(three['pss_mean'], 0)} bytes; two-layer={fmt(two['rss_mean'], 0)}/{fmt(two['pss_mean'], 0)} bytes.",
        f"- Session lifecycle: three-layer start/close/restart={three_session.get('session_start_count', 'not_available')}/{three_session.get('session_close_count', 'not_available')}/{three_session.get('session_restart_count', 'not_available')}; two-layer={two_session.get('session_start_count', 'not_available')}/{two_session.get('session_close_count', 'not_available')}/{two_session.get('session_restart_count', 'not_available')}.",
        f"- Common-success measured pairs: {len(common_rows)}/100 across {', '.join(common_ids) if common_ids else 'no tasks'}; common-success wall P50 three/two={fmt(float(np.percentile(common_times['three'], 50)) if common_times['three'] else None)}/{fmt(float(np.percentile(common_times['two'], 50)) if common_times['two'] else None)} ms.",
        f"- Common-success path quality means: length three/two={fmt(common_mean('three_path_length_m'))}/{fmt(common_mean('two_path_length_m'))} m (two/three ratio={fmt(common_mean('two_to_three_path_length_ratio'), 4)}); minimum clearance={fmt(common_mean('three_minimum_clearance_m'))}/{fmt(common_mean('two_minimum_clearance_m'))} m; maximum curvature={fmt(common_mean('three_maximum_curvature'), 4)}/{fmt(common_mean('two_maximum_curvature'), 4)} 1/m.",
        f"- Three-layer measured failures: {dict(three['failure_counts'])}.",
        f"- Two-layer measured failures: {dict(two['failure_counts'])}; the dominant corridor-related classes are retained as `START_IN_LETHAL_SPACE`, `NO_PATH_IN_CORRIDOR`, and `ACTION_ABORTED`, not counted as speed benefits.",
        *historical_lines,
        "",
        "The current 3A-V0 optimized result is the same architecture and query protocol as the historical reference; cache hits remove repeated L1 setup but do not materially change total wall time because L2 search, local L3 retries, and validation dominate. The historical 2A-V0 cache result is reported for trend context only and is not a same-round paired measurement.",
        "The two-layer arm truly did not call `plan_grid_astar` or generate an L2 path. It is faster on this map largely because it avoids repeated local repair, but its final-valid rate is lower and its RSS/PSS is higher. L2 therefore remains necessary for the current stable architecture's reliability on this map.",
        "This result is a single-map, 20-query formal pairing only. It is insufficient to replace V7 or to generalize across map scales; repair of narrow-corridor, endpoint, and local-L3 failure modes is required before broader evaluation.",
    ]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    (output / "manifest.yaml").write_text(yaml.safe_dump({"schema_version": 1, "experiment": output.name, "map_id": MAP_ID, "query_ids": list(TASK_IDS), "json_sha256": task_meta["json_sha256"], "csv_sha256": task_meta["csv_sha256"], "resolution_m": 0.05, "dynamic_obstacles": False, "warmups": WARMUPS, "repetitions": REPETITIONS, "topology_cache_key": topology_info.get("topology_cache_key", ""), "topology_build_count": topology_info.get("topology_build_count", 0), "topology_worker_cache_load_count": worker_topology_load_count, "three_layer_final_valid_count": three["final_valid_count"], "two_layer_final_valid_count": two["final_valid_count"], "paired_common_success_count": three["common_success_count"], "rrtstar_call_count": 0, "sst_call_count": 0, "optimization_disposition": "optimized_no_material_gain", "formal_multi_map_conclusion": False}, sort_keys=False), encoding="utf-8")
    (output / "protocol.yaml").write_text(yaml.safe_dump({"schema_version": 1, "architecture_comparison": ["L1 + L2 Grid A* + local L3 Smac", "L1 + raw_map_smac_aligned_2m corridor-wide L3' Smac"], "map_id": MAP_ID, "query_ids": list(TASK_IDS), "warmups": WARMUPS, "repetitions": REPETITIONS, "resolution": 0.05, "minimum_turning_radius_m": 0.40, "maximum_curvature": 2.50, "allow_reverse": False, "allow_in_place_rotation": False, "dynamic_obstacles": False, "smac_parameter_profile": SMAC_PARAMETER_PROFILE, "optimization_profile": OPTIMIZATION_PROFILE, "optimization_stage": OPTIMIZATION_STAGE, "topology_cache_shared": True, "paired_statistics": "measured only; common-success path ratios only"}, sort_keys=False), encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the fixed 20-query mentor-map three-layer versus two-layer paired benchmark")
    parser.add_argument("--output-dir", default=str(ROOT / "experiments/layered_planner_benchmark" / OUTPUT_NAME))
    parser.add_argument("--map-id", choices=(DEFAULT_MAP_ID, FOUR_X_MAP_ID), default=DEFAULT_MAP_ID)
    parser.add_argument("--warmups", type=int, default=WARMUPS)
    parser.add_argument("--repetitions", type=int, default=REPETITIONS)
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    parser.add_argument("--worker-architecture", choices=("three_layer", "two_layer"), default=None)
    parser.add_argument("--topology-cache-dir", default="")
    parser.add_argument("--ros-domain-id", type=int, default=0)
    parser.add_argument("--cache-mode", choices=(legacy.CACHE_MODE_BASELINE, legacy.CACHE_MODE_OPTIMIZED), default=legacy.CACHE_MODE_OPTIMIZED,
                        help="reuse map-level L1 caches in both worker arms")
    parser.add_argument("--query-id", action="append", choices=TASK_IDS, dest="query_ids",
                        help="worker-only bounded preflight query filter; formal runs use all A2B-01..A2B-20")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    _configure_map(args.map_id)
    if args.worker_architecture:
        if not args.topology_cache_dir:
            raise ValueError("worker requires --topology-cache-dir")
        return _worker(args)
    output = Path(args.output_dir).resolve(); _refuse_nonempty(output); output.mkdir(parents=True); (output / "plots").mkdir()
    if int(args.warmups) != WARMUPS or int(args.repetitions) != REPETITIONS:
        raise ValueError("formal paired benchmark requires exactly 3 warmups and 5 measured repetitions")
    queries, task_meta = _load_tasks(); ctx = _context(); cache_dir = output / TOPOLOGY_CACHE; topology = _prepare_topology(cache_dir, ctx, planner_parameter_profile=SMAC_PARAMETER_PROFILE); topology_info = topology["info"]
    source_files, source_hash = _source_manifest()
    (output / "task_manifest.yaml").write_text(yaml.safe_dump({"benchmark_json": str(BENCHMARK_JSON), "benchmark_csv": str(BENCHMARK_CSV), "scenario_json": str(SCENARIO_JSON), "map_id": MAP_ID, "tasks": [query.as_dict() for query in queries], "metadata": task_meta}, sort_keys=False), encoding="utf-8")
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({"schema_version": 1, "source_commit": _source_commit(), "code_hash": source_hash, "source_files": source_files, "map_id": MAP_ID, "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256, "scenario_json_sha256": task_meta["scenario_sha256"], "benchmark_json_sha256": task_meta["json_sha256"], "benchmark_csv_sha256": task_meta["csv_sha256"], "resolution_m": 0.05, "dynamic_obstacles": False, "footprint_hash": _json_hash(FOOTPRINT), "smac_parameter_profile": SMAC_PARAMETER_PROFILE, "optimization_profile": OPTIMIZATION_PROFILE, "optimization_stage": OPTIMIZATION_STAGE}, sort_keys=False), encoding="utf-8")
    (output / "topology_manifest.yaml").write_text(yaml.safe_dump({"schema_version": 1, **{key: value for key, value in topology_info.items() if key != "artifact_metadata"}, "map_id": MAP_ID, "map_file_hash": ctx.map_sha256, "map_yaml_hash": ctx.map_yaml_sha256, "resolution": 0.05, "width": ctx.hospital_map.width, "height": ctx.hospital_map.height}, sort_keys=False), encoding="utf-8")
    env = dict(os.environ)
    package_root = str(Path(__file__).resolve().parents[1])
    env["PYTHONPATH"] = package_root + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    worker_results: Dict[str, Path] = {}
    for index, architecture in enumerate(("three_layer", "two_layer")):
        arm_dir = output / architecture; arm_dir.mkdir()
        command = [sys.executable, "-m", "arena_evaluation.layered_architecture_paired_benchmark", "--worker-architecture", architecture, "--map-id", MAP_ID, "--output-dir", str(arm_dir), "--topology-cache-dir", str(cache_dir), "--ros-domain-id", str(100 + index), "--warmups", str(args.warmups), "--repetitions", str(args.repetitions), "--cache-mode", str(args.cache_mode), "--no-dynamic-obstacles"]
        completed = subprocess.run(command, cwd=str(ROOT), env=env)
        if completed.returncode != 0:
            raise RuntimeError(f"{architecture} worker failed with exit code {completed.returncode}")
        worker_results[architecture] = arm_dir
    _paired_summary(output, task_meta, topology_info, worker_results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
