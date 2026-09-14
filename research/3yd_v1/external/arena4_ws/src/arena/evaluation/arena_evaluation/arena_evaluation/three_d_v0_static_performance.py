"""Static initial-planning benchmark for the independent ``3D-V0`` arm.

The benchmark is intentionally separate from the frozen ``3A-V0``/V7 and
``2A-V0`` runners.  It exercises the real L1 Graph A* + persistent L2 D* Lite
+ L3 Smac Hybrid composition on the mentor A2B-20 task set with one immutable
empty dynamic snapshot.  Topology construction and persistence are performed
once during setup and are excluded from per-query online timing.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import os
import resource
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import fixed_layered_pipeline_smoke as fixed
from . import layered_architecture_paired_benchmark as paired
from . import unified_four_backends_smoke as legacy
from .dynamic_snapshot import DynamicSnapshot
from .layered_dynamic_pipeline import LayeredDynamicPipeline, SmacHybridAdapter
from .planner_benchmark.map_utils import HospitalMap, sha256_file
from .planner_benchmark.models import Query
from .topology import load_topology, preprocess_static_map


ROOT = Path("/home/robot/pudu_robot_ws")
MAP_ID = "mentor_map_20260825_005"
WORLD = ROOT / "external/arena4_ws/src/arena/simulation-setup/worlds" / MAP_ID
MAP_YAML = WORLD / "map/map.yaml"
SCENARIO_JSON = WORLD / "scenarios/a2b_benchmark_20.json"
BENCHMARK_JSON = ROOT / "benchmarks/arena_a2b_20/arena_a2b_benchmark_20.json"
BENCHMARK_CSV = ROOT / "benchmarks/arena_a2b_20/arena_a2b_benchmark_20.csv"
TASK_IDS = tuple(f"A2B-{index:02d}" for index in range(1, 21))
ARCHITECTURE_ID = "3D-V0"
IMPLEMENTATION_REVISION = "r1"
OUTPUT_NAME = "3d_v0_mentor_map_20_performance_v1"
WARMUPS = 3
REPETITIONS = 5
TIMEOUT_S = 7.0
TOPOLOGY_PADDING_M = 0.05
TOPOLOGY_SAFETY_MARGIN_M = 0.05
CORRIDOR_PADDING_M = 2.0
SNAPSHOT_ID = "static_empty_v1"
SMAC_PARAMETER_PROFILE = "lighter_smoother"
OPTIMIZATION_PROFILE = "v7_candidate"
OPTIMIZATION_STAGE = "step3_delta_map"


def _json_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    ).hexdigest()


def _query_hash(query: Query) -> str:
    return _json_hash({"query_id": query.query_id, "start": list(query.start), "goal": list(query.goal)})


def _numeric(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


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
            encoded: Dict[str, Any] = {}
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


def _source_commit() -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(ROOT), "rev-parse", "HEAD"],
            text=True, stderr=subprocess.DEVNULL,
        ).strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def _load_tasks() -> Tuple[List[Query], Dict[str, Any]]:
    """Load and cross-check the three immutable A2B-20 task sources."""
    payload = json.loads(BENCHMARK_JSON.read_text(encoding="utf-8"))
    map_entry = (payload.get("maps") or {}).get(MAP_ID)
    json_tasks = list((map_entry or {}).get("tasks") or [])
    if [str(item.get("id")) for item in json_tasks] != list(TASK_IDS):
        raise ValueError("benchmark JSON task order is not exactly A2B-01..A2B-20")
    with BENCHMARK_CSV.open(newline="", encoding="utf-8") as stream:
        csv_tasks = [row for row in csv.DictReader(stream) if row.get("world") == MAP_ID]
    if [str(item.get("task_id")) for item in csv_tasks] != list(TASK_IDS):
        raise ValueError("benchmark CSV task order is not exactly A2B-01..A2B-20")
    scenario = json.loads(SCENARIO_JSON.read_text(encoding="utf-8"))
    scenario_tasks = list(scenario.get("tasks") or [])
    if scenario.get("world") != MAP_ID or [str(item.get("id")) for item in scenario_tasks] != list(TASK_IDS):
        raise ValueError("scenario does not contain ordered mentor A2B-01..A2B-20")
    queries: List[Query] = []
    for item, csv_item, scenario_item in zip(json_tasks, csv_tasks, scenario_tasks):
        start = [float(value) for value in item["start"]]
        goal = [float(value) for value in item["goal"]]
        csv_start = [float(csv_item[key]) for key in ("start_x_m", "start_y_m", "start_yaw_rad")]
        csv_goal = [float(csv_item[key]) for key in ("goal_x_m", "goal_y_m", "goal_yaw_rad")]
        if (
            not np.allclose(start, csv_start, rtol=0.0, atol=1.0e-9)
            or not np.allclose(goal, csv_goal, rtol=0.0, atol=1.0e-9)
            or not np.allclose(start, scenario_item["start"], rtol=0.0, atol=1.0e-9)
            or not np.allclose(goal, scenario_item["goal"], rtol=0.0, atol=1.0e-9)
        ):
            raise ValueError(f"JSON/CSV/scenario pose mismatch for {item['id']}")
        queries.append(Query(
            str(item["id"]), start, goal,
            str(item.get("label", csv_item.get("label", "unspecified"))), 0, "UNVALIDATED",
        ))
    metadata = {
        "map_id": MAP_ID,
        "task_ids": list(TASK_IDS),
        "benchmark_json_sha256": sha256_file(BENCHMARK_JSON),
        "benchmark_csv_sha256": sha256_file(BENCHMARK_CSV),
        "scenario_sha256": sha256_file(SCENARIO_JSON),
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
        hospital_map, legacy.FOOTPRINT,
        padding_m=TOPOLOGY_PADDING_M, safety_margin_m=TOPOLOGY_SAFETY_MARGIN_M,
        allow_unknown=False,
    )
    return legacy.MapContext(
        MAP_ID, hospital_map, free_mask, distance_m,
        sha256_file(hospital_map.image_path), sha256_file(hospital_map.yaml_path), MAP_YAML,
    )


def _prepare_topology(output: Path, ctx: legacy.MapContext) -> Tuple[Any, Dict[str, Any]]:
    """Build, persist, and load one immutable artifact exactly once each."""
    cache_root = output / "topology_cache" / MAP_ID
    cache_root.mkdir(parents=True, exist_ok=True)
    source_hash = sha256_file(Path(__file__).resolve().parent / "topology.py")
    expected = fixed._topology_cache_expected(MAP_ID, ctx, _source_commit(), source_hash)
    cache_key = hashlib.sha256(
        json.dumps(expected, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    cache_dir = cache_root / cache_key
    build_started = time.monotonic_ns()
    cpu_before = resource.getrusage(resource.RUSAGE_SELF)
    artifact_built = legacy.build_topology(
        ctx.hospital_map, legacy.FOOTPRINT,
        padding_m=TOPOLOGY_PADDING_M, safety_margin_m=TOPOLOGY_SAFETY_MARGIN_M,
        allow_unknown=False,
    )
    build_ms = (time.monotonic_ns() - build_started) / 1.0e6
    cpu_after = resource.getrusage(resource.RUSAGE_SELF)
    build_cpu_ms = max(
        0.0,
        (cpu_after.ru_utime - cpu_before.ru_utime + cpu_after.ru_stime - cpu_before.ru_stime) * 1000.0,
    )
    persist_started = time.monotonic_ns()
    legacy.save_topology(artifact_built, cache_dir)
    (cache_dir / "cache_manifest.yaml").write_text(yaml.safe_dump({
        "schema_version": 1, "cache_key": cache_key, "metadata": expected,
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
    }, sort_keys=False), encoding="utf-8")
    persist_ms = (time.monotonic_ns() - persist_started) / 1.0e6
    load_started = time.monotonic_ns()
    artifact = load_topology(
        cache_dir, ctx.hospital_map, legacy.FOOTPRINT,
        padding_m=TOPOLOGY_PADDING_M, safety_margin_m=TOPOLOGY_SAFETY_MARGIN_M,
        allow_unknown=False,
    )
    load_ms = (time.monotonic_ns() - load_started) / 1.0e6
    info = {
        **expected,
        "topology_cache_key": cache_key,
        "cache_directory": str(cache_dir),
        "topology_cache_hit": False,
        "topology_build_count": 1,
        "topology_load_count": 1,
        "topology_build_time_ms": build_ms,
        "topology_build_cpu_time_ms": build_cpu_ms,
        "topology_persist_time_ms": persist_ms,
        "topology_load_time_ms": load_ms,
        "graph_nodes": len(artifact.graph.nodes),
        "graph_edges": len(artifact.graph.edges),
        "graph_components": artifact.graph.components,
        "skeleton_backend": artifact.metadata.get("skeleton_backend", "unknown"),
    }
    return artifact, info


def _read_pss_bytes() -> Optional[int]:
    try:
        for line in Path("/proc/self/smaps_rollup").read_text(encoding="utf-8").splitlines():
            if line.startswith("Pss:"):
                return int(line.split()[1]) * 1024
    except (OSError, ValueError, IndexError):
        return None
    return None


def _path_with_hash(points: Optional[Sequence[Mapping[str, Any]]]) -> Tuple[Optional[List[Dict[str, Any]]], str]:
    if not points:
        return None, ""
    enriched = [dict(item) for item in points]
    enriched = [dict(item, source_commit=_source_commit()) if "source_commit" not in item else item for item in enriched]
    digest = legacy._path_hash(enriched)
    for point in enriched:
        point["path_hash"] = digest
    return enriched, digest


def _empty_metrics(result: Any) -> Dict[str, Any]:
    return {
        "static_footprint_valid": False, "kinematic_valid": False,
        "final_valid_success": False, "failure_code": result.failure_code or "EMPTY_PATH",
        "failure_detail": result.failure_code or "path is empty", "path_length_m": None,
        "minimum_clearance_m": None, "maximum_curvature": None, "curvature_p95": None,
        "heading_discontinuity_count": 0, "position_discontinuity_count": 0,
        "steering_jump_count": 0, "reverse_distance_m": 0.0,
        "in_place_rotation_count": 0, "start_position_error_m": None,
        "goal_position_error_m": None, "start_yaw_error_rad": None,
        "goal_yaw_error_rad": None,
    }


def _memory_values(diagnostics: Mapping[str, Any], ru_before: Any, ru_after: Any) -> Tuple[int, Optional[int]]:
    process_rss = max(int(getattr(ru_before, "ru_maxrss", 0) or 0), int(getattr(ru_after, "ru_maxrss", 0) or 0)) * 1024
    rss_candidates = [process_rss]
    for key in ("planner_rss_peak_bytes", "stack_rss_peak_bytes"):
        value = diagnostics.get(key)
        if value is not None:
            try:
                rss_candidates.append(int(value))
            except (TypeError, ValueError):
                pass
    pss_candidates = []
    for key in ("planner_pss_peak_bytes", "stack_pss_peak_bytes"):
        value = diagnostics.get(key)
        if value is not None:
            try:
                pss_candidates.append(int(value))
            except (TypeError, ValueError):
                pass
    local_pss = _read_pss_bytes()
    if local_pss is not None:
        pss_candidates.append(local_pss)
    return max(rss_candidates), (max(pss_candidates) if pss_candidates else None)


def _historical_summary(path: Path) -> Dict[str, Any]:
    result: Dict[str, Any] = {"path": str(path), "available": False}
    if not path.exists():
        return result
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        measured = [row for row in rows if str(row.get("run_mode", "")).lower() == "measured"]
        values = [_numeric(row.get("pipeline_wall_time_ms") or row.get("wall_time_ms"), float("nan")) for row in measured]
        values = [value for value in values if math.isfinite(value)]
        valid = sum(str(row.get("final_valid_success", "")).lower() == "true" for row in measured)
        result.update({
            "available": True, "measured_count": len(measured),
            "final_valid_count": valid, "final_valid_rate": valid / len(measured) if measured else None,
            "wall_p50_ms": float(np.percentile(values, 50)) if values else None,
            "wall_p95_ms": float(np.percentile(values, 95)) if values else None,
            "wall_p99_ms": float(np.percentile(values, 99)) if values else None,
        })
    except (OSError, csv.Error, ValueError):
        pass
    return result


def run_benchmark(
    output: Path,
    *,
    query_ids: Optional[Sequence[str]] = None,
    warmups: int = WARMUPS,
    repetitions: int = REPETITIONS,
    timeout_s: float = TIMEOUT_S,
    ros_domain_id: Optional[int] = None,
    enforce_formal_protocol: bool = True,
) -> Path:
    if warmups < 0 or repetitions <= 0:
        raise ValueError("warmups must be >= 0 and repetitions must be > 0")
    if enforce_formal_protocol and (warmups != WARMUPS or repetitions != REPETITIONS):
        raise ValueError("formal 3D-V0 performance requires exactly 3 warmups and 5 measured repetitions")
    _refuse_nonempty(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir()
    queries, task_meta = _load_tasks()
    selected_ids = list(query_ids or TASK_IDS)
    if selected_ids != list(TASK_IDS) and any(item not in TASK_IDS for item in selected_ids):
        raise ValueError("query_ids must be a subset of A2B-01..A2B-20")
    selected = [query for query in queries if query.query_id in selected_ids]
    if len(selected) != len(selected_ids):
        raise ValueError("one or more requested query IDs are missing")
    ctx = _context()
    artifact, topology_info = _prepare_topology(output, ctx)
    # Keep the raw occupancy layer alongside the footprint-safe L1/L2 mask;
    # Smac must apply static inflation exactly once inside the shared session.
    artifact.raw_free_mask = np.asarray(ctx.hospital_map.occupancy == 0, dtype=bool)
    source_paths = [
        Path(__file__).resolve(), Path(fixed.__file__).resolve(), Path(legacy.__file__).resolve(),
        Path(LayeredDynamicPipeline.__module__.replace(".", "/") + ".py"),
        Path(__file__).resolve().parent / "layered_dynamic_pipeline.py",
        Path(__file__).resolve().parent / "l2_dstar.py",
        Path(__file__).resolve().parent / "dstar_lite.py",
        Path(__file__).resolve().parent / "dynamic_snapshot.py",
        Path(__file__).resolve().parent / "topology.py",
        Path(__file__).resolve().parents[1] / "setup.py",
        MAP_YAML, MAP_YAML.parent / "map.pgm", BENCHMARK_JSON, BENCHMARK_CSV, SCENARIO_JSON,
        legacy._strict_smac_config_path(),
    ]
    source_files = {str(path.resolve()): sha256_file(path) for path in source_paths if path.exists()}
    source_hash = _json_hash(source_files)
    snapshot = DynamicSnapshot.empty(
        SNAPSHOT_ID, timestamp=0.0, map_version=ctx.map_sha256,
        map_shape=tuple(artifact.free_mask.shape),
    )
    (output / "dynamic_snapshot.yaml").write_text(yaml.safe_dump(snapshot.as_dict(), sort_keys=False), encoding="utf-8")
    (output / "task_manifest.yaml").write_text(yaml.safe_dump({"map_id": MAP_ID, "tasks": [query.__dict__ for query in queries], "metadata": task_meta}, sort_keys=False), encoding="utf-8")
    (output / "topology_manifest.yaml").write_text(yaml.safe_dump(topology_info, sort_keys=False), encoding="utf-8")
    spec = legacy.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"Smac Hybrid backend unavailable: {spec.reason}")
    # Fast-DDS reserves domain IDs through 232; stay below that limit while
    # remaining isolated from the user's default visualization domain.
    ros_domain = int(ros_domain_id if ros_domain_id is not None else 220 + (os.getpid() % 10))
    os.environ["ROS_DOMAIN_ID"] = str(ros_domain)
    session = legacy.SmacSession(
        ctx, output, map_yaml=MAP_YAML, log_tag=f"3d_v0_{MAP_ID}", local_mask_updates=True,
        optimization_profile=OPTIMIZATION_PROFILE,
        smac_parameter_profile=SMAC_PARAMETER_PROFILE,
        optimization_stage=OPTIMIZATION_STAGE,
    )
    session.start()
    adapter = SmacHybridAdapter(session, spec)
    pipeline = LayeredDynamicPipeline(
        artifact, footprint=legacy.FOOTPRINT, l3_planner=adapter,
        corridor_padding_m=CORRIDOR_PADDING_M,
        corridor_padding_schedule_m=(2.0, 4.0, 6.0),
    )
    run_rows: List[Dict[str, Any]] = []
    metric_rows: List[Dict[str, Any]] = []
    call_rows: List[Dict[str, Any]] = []
    try:
        sequence = (
            [(query, "warmup", index) for index in range(1, warmups + 1) for query in selected]
            + [(query, "measured", index) for index in range(1, repetitions + 1) for query in selected]
        )
        for query, run_mode, repetition in sequence:
            run_id = f"{MAP_ID}_{query.query_id}_3d_v0_{run_mode}_{repetition}"
            before_resource = resource.getrusage(resource.RUSAGE_SELF)
            started_ns = time.monotonic_ns()
            reset = session.reset_query_state(query.query_id, restore_base_map=False)
            reset_ms = _numeric(reset.get("query_session_reset_ms"))
            result = pipeline.plan_initial(query, snapshot, timeout_s=timeout_s)
            pipeline_wall_ms = (time.monotonic_ns() - started_ns) / 1.0e6
            after_resource = resource.getrusage(resource.RUSAGE_SELF)
            cpu_ms = max(0.0, (after_resource.ru_utime - before_resource.ru_utime + after_resource.ru_stime - before_resource.ru_stime) * 1000.0)
            diagnostics = dict(result.diagnostics or {})
            l3_diag = dict(diagnostics.get("l3") or {})
            points, path_hash = _path_with_hash(result.points)
            path_file = ""
            if points:
                path_file = f"paths/{run_id}.json"
                (output / path_file).write_text(json.dumps(points, indent=2, sort_keys=True), encoding="utf-8")
            validation_started = time.monotonic_ns()
            metrics = legacy.validate_path(ctx, query, points) if points else _empty_metrics(result)
            validation_ms = (time.monotonic_ns() - validation_started) / 1.0e6
            final_valid = bool(result.success and metrics.get("static_footprint_valid") and metrics.get("kinematic_valid"))
            if not final_valid:
                failure_code = str(metrics.get("failure_code") or result.failure_code or "FINAL_VALIDATION_FAILED")
            else:
                failure_code = ""
            l2 = diagnostics.get("l2") or {}
            l3_call_count = int(l3_diag.get("backend_call_count") or 0)
            l2_attempts = list(diagnostics.get("l2_attempts") or [])
            l2_called = bool(l2_attempts)
            l2_call_count = len(l2_attempts)
            l3_called = l3_call_count > 0
            rss, pss = _memory_values(l3_diag, before_resource, after_resource)
            l1_route_ms = _numeric(diagnostics.get("l1_route_search_ms"))
            l1_attach_ms = _numeric(diagnostics.get("l1_attachment_lookup_ms"))
            l1_total_ms = max(l1_route_ms, l1_attach_ms)
            l2_init_ms = _numeric(diagnostics.get("l2_initialization_ms"))
            l2_compute_ms = _numeric(diagnostics.get("l2_search_time_ms"))
            l2_extract_ms = _numeric(diagnostics.get("l2_extract_path_ms"))
            l2_total_ms = l2_init_ms + l2_compute_ms + l2_extract_ms
            l3_plan_ms = _numeric(l3_diag.get("l3_planning_time_ms") or l3_diag.get("planning_time_ms") or result.diagnostics.get("l3_planner_time_ms"))
            l3_action_ms = _numeric(l3_diag.get("l3_action_wall_ms") or l3_diag.get("wall_time_ms"))
            l3_overhead_ms = _numeric(l3_diag.get("l3_process_overhead_ms"))
            row: Dict[str, Any] = {
                "run_id": run_id, "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(),
                "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
                "map_id": MAP_ID, "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256,
                "query_id": query.query_id, "query_hash": _query_hash(query), "query_role": "raw",
                "run_mode": run_mode, "repetition": repetition,
                "start_x": query.start[0], "start_y": query.start[1], "start_yaw": query.start[2],
                "goal_x": query.goal[0], "goal_y": query.goal[1], "goal_yaw": query.goal[2],
                "dynamic_obstacles": False, "dynamic_snapshot_id": snapshot.snapshot_id,
                "dynamic_snapshot_hash": snapshot.snapshot_hash, "dynamic_update_count": 0,
                "l1_reroute_count": int(pipeline.l1_reroute_count), "l2_reset_count": int(pipeline.l2_reset_count),
                "dstar_initial_search": True, "dstar_incremental_update": False,
                "topology_cache_key": topology_info["topology_cache_key"], "topology_cache_hit": topology_info["topology_cache_hit"],
                "topology_build_count": topology_info["topology_build_count"], "topology_load_count": topology_info["topology_load_count"],
                "topology_build_time_ms": topology_info["topology_build_time_ms"], "topology_load_time_ms": topology_info["topology_load_time_ms"],
                "topology_adjacency_cache_hit": bool(diagnostics.get("topology_adjacency_cache_hit", False)),
                "endpoint_spatial_index_cache_hit": True, "route_cache_hit": bool(diagnostics.get("route_cache_hit", False)),
                "dstar_state_cache_hit": False,
                "l1_attachment_lookup_ms": l1_attach_ms, "l1_route_search_ms": l1_route_ms, "l1_total_time_ms": l1_total_ms,
                "l2_dstar_initialization_ms": l2_init_ms, "l2_dstar_compute_shortest_path_ms": l2_compute_ms,
                "l2_dstar_extract_path_ms": l2_extract_ms, "l2_total_time_ms": l2_total_ms,
                "l3_planning_time_ms": l3_plan_ms, "l3_action_wall_ms": l3_action_ms,
                "l3_process_overhead_ms": l3_overhead_ms, "stitch_validation_time_ms": validation_ms,
                "query_session_reset_ms": reset_ms, "pipeline_wall_time_ms": pipeline_wall_ms,
                "pipeline_cpu_total_ms": cpu_ms, "peak_rss": rss, "peak_pss": pss,
                "l1_success": bool(diagnostics.get("l1_success", diagnostics.get("selected_goal_node") is not None)),
                "l2_called": l2_called, "l2_call_count": l2_call_count,
                "l2_success": bool(result.grid_path), "l3_called": l3_called,
                "l3_attempted": l3_called, "l3_call_count": l3_call_count,
                "l3_backend_call_count": l3_call_count, "repair_window_count": 0,
                "rrtstar_call_count": 0, "sst_call_count": 0,
                "planner_success": bool(result.success), "action_success": bool(l3_called and result.success),
                "final_valid_success": final_valid,
                "failure_code": failure_code, "failure_detail": str(metrics.get("failure_detail") or result.failure_code or ""),
                "path_hash": path_hash, "path_file": path_file,
                "dstar_expanded_states": int(diagnostics.get("l2_expanded_nodes") or 0),
                "dstar_generated_states": int(diagnostics.get("l2_generated_nodes") or 0),
                "dstar_update_vertex_count": int(diagnostics.get("dstar_update_vertex_count") or 0),
                "dstar_queue_push_count": int(diagnostics.get("dstar_queue_push_count") or 0),
                "dstar_queue_pop_count": int(diagnostics.get("dstar_queue_pop_count") or 0),
                "dstar_initial_queue_size": int(diagnostics.get("dstar_initial_queue_size") or 0),
                "dstar_final_queue_size": int(diagnostics.get("dstar_final_queue_size") or 0),
                "dstar_state_reset_count": 0,
                "session_start_count": session.session_start_count, "session_close_count": 0,
                "session_restart_count": session.session_restart_count, "ros_domain_id": ros_domain,
                "source_hash": source_hash, "source_commit": _source_commit(),
                **metrics,
            }
            run_rows.append(row)
            metric_rows.append({"run_id": run_id, "architecture_id": ARCHITECTURE_ID, "map_id": MAP_ID, "query_id": query.query_id, "query_hash": _query_hash(query), **metrics})
            call_rows.extend([
                {
                    "run_id": run_id, "architecture_id": ARCHITECTURE_ID, "map_id": MAP_ID, "query_id": query.query_id,
                    "run_mode": run_mode, "repetition": repetition, "stage": "L2", "role": "dstar_lite_initial_search",
                    "planner_backend": "arena_evaluation.dstar_lite.DStarLite", "called": l2_called,
                    "physical_backend_call_count": l2_call_count, "planner_success": bool(result.grid_path),
                    "failure_code": str((l2 or {}).get("failure_code") or ""), "dstar_initial_search": True,
                },
                {
                    "run_id": run_id, "architecture_id": ARCHITECTURE_ID, "map_id": MAP_ID, "query_id": query.query_id,
                    "run_mode": run_mode, "repetition": repetition, "stage": "L3", "role": "smac_hybrid_initial_corridor",
                    "planner_backend": spec.backend, "backend_version": spec.version, "called": l3_called,
                    "physical_backend_call_count": l3_call_count, "planner_success": bool(result.success),
                    "failure_code": str(result.failure_code or ""), "dynamic_snapshot_id": snapshot.snapshot_id,
                },
            ])
    finally:
        session.close()
    for row in run_rows:
        row["session_close_count"] = session.session_close_count
    _write_csv(output / "runs.csv", run_rows)
    _write_csv(output / "path_metrics.csv", metric_rows)
    _write_csv(output / "backend_call_log.csv", call_rows)
    _write_csv(output / "session_timing.csv", [{
        "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "map_id": MAP_ID, "ros_domain_id": ros_domain,
        "session_start_count": session.session_start_count, "session_close_count": session.session_close_count,
        "session_restart_count": session.session_restart_count,
        "session_startup_time_ms": session.stack_startup_time_ms, "session_shutdown_time_ms": session.stack_shutdown_time_ms,
        **topology_info,
    }])
    measured = [row for row in run_rows if row["run_mode"] == "measured"]
    values = [_numeric(row.get("pipeline_wall_time_ms"), float("nan")) for row in measured]
    values = [value for value in values if math.isfinite(value)]
    valid = sum(bool(row.get("final_valid_success")) for row in measured)
    failure_counts: Dict[str, int] = {}
    for row in measured:
        code = str(row.get("failure_code") or "")
        if code:
            failure_counts[code] = failure_counts.get(code, 0) + 1
    _write_csv(output / "failure_summary.csv", [{"architecture_id": ARCHITECTURE_ID, "failure_code": code, "count": count} for code, count in sorted(failure_counts.items())])
    comparison = {
        "3A-V0": _historical_summary(ROOT / "experiments/layered_planner_benchmark/fixed_layered_pipeline_v7_online_efficiency_postfix5_final/runs.csv"),
        "2A-V0": _historical_summary(ROOT / "experiments/layered_planner_benchmark/l1_l3_corridor_hybrid_mentor_map_20_validity_v3/runs.csv"),
        "3D-V0": {"available": True, "measured_count": len(measured), "final_valid_count": valid,
                  "final_valid_rate": valid / len(measured) if measured else None,
                  "wall_p50_ms": float(np.percentile(values, 50)) if values else None,
                  "wall_p95_ms": float(np.percentile(values, 95)) if values else None,
                  "wall_p99_ms": float(np.percentile(values, 99)) if values else None},
    }
    (output / "architecture_comparison.yaml").write_text(yaml.safe_dump(comparison, sort_keys=False), encoding="utf-8")
    manifest = {
        "schema_version": 1, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "map_id": MAP_ID, "query_ids": selected_ids, "warmup_count": warmups,
        "measured_repetitions": repetitions, "run_count": len(run_rows), "measured_count": len(measured),
        "measured_final_valid_count": valid, "final_valid_rate": valid / len(measured) if measured else None,
        "dynamic_obstacles": False, "dynamic_snapshot_id": snapshot.snapshot_id,
        "dynamic_snapshot_hash": snapshot.snapshot_hash, "dynamic_update_count": 0,
        "l1_reroute_count": 0, "l2_reset_count": 0, "topology_build_count": 1, "topology_load_count": 1,
        "session_start_count": session.session_start_count, "session_close_count": session.session_close_count,
        "session_restart_count": session.session_restart_count, "l2_grid_astar_call_count": 0,
        "rrtstar_call_count": 0, "sst_call_count": 0,
        "smac_call_count": sum(int(row.get("l3_backend_call_count") or 0) for row in measured),
        "source_hash": source_hash, "source_commit": _source_commit(),
        "static_constraints": {"resolution_m": 0.05, "minimum_turning_radius_m": 0.40, "maximum_curvature": 2.50, "allow_reverse": False, "allow_in_place_rotation": False},
        "formal_protocol": bool(enforce_formal_protocol),
        "formal_dynamic_multi_map_unlocked": False,
    }
    (output / "protocol.yaml").write_text(yaml.safe_dump({
        "schema_version": 1, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "test_name": "static_initial_global_planning_performance", "map_id": MAP_ID,
        "query_ids": list(TASK_IDS), "warmups": warmups, "repetitions": repetitions,
        "dynamic_obstacles": False, "dynamic_snapshot": snapshot.as_dict(),
        "topology_build_load_required": "1/1", "smac_session_lifecycle_required": "1/1/0",
        "corridor_padding_m": CORRIDOR_PADDING_M, "smac_parameter_profile": SMAC_PARAMETER_PROFILE,
        "optimization_profile": OPTIMIZATION_PROFILE, "optimization_stage": OPTIMIZATION_STAGE,
        "constraints": manifest["static_constraints"],
    }, sort_keys=False), encoding="utf-8")
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({
        "source_commit": _source_commit(), "code_hash": source_hash, "source_files": source_files,
        "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256,
        "task_hashes": {query.query_id: _query_hash(query) for query in queries},
    }, sort_keys=False), encoding="utf-8")
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    report = [
        f"# {ARCHITECTURE_ID} 静态初次全局规划性能测试",
        "",
        f"- 架构：L1 Graph A* + L2 持久化 D* Lite + L3 真实 Smac Hybrid A*；实现：`{IMPLEMENTATION_REVISION}`。",
        f"- 地图：`{MAP_ID}`；query：`A2B-01..A2B-20`；每个 query `{warmups} warmup + {repetitions} measured`，measured={len(measured)}。",
        f"- 空动态快照：`{snapshot.snapshot_id}`，动态更新=0，L1 重路由=0，L2 reset=0。",
        f"- Final-valid：**{valid}/{len(measured)} ({100.0 * valid / len(measured) if measured else 0.0:.1f}%)**。",
        f"- Online pipeline P50/P95/P99：**{np.percentile(values, 50) if values else 0.0:.2f}/{np.percentile(values, 95) if values else 0.0:.2f}/{np.percentile(values, 99) if values else 0.0:.2f} ms**。",
        f"- 分层耗时中位数（详见 runs.csv）：L1={np.median([_numeric(r.get('l1_total_time_ms')) for r in measured]) if measured else 0.0:.2f} ms，L2 D* Lite={np.median([_numeric(r.get('l2_total_time_ms')) for r in measured]) if measured else 0.0:.2f} ms，L3 action={np.median([_numeric(r.get('l3_action_wall_ms')) for r in measured]) if measured else 0.0:.2f} ms。",
        f"- D* Lite 初次展开/生成节点总计：{sum(int(r.get('dstar_expanded_states') or 0) for r in measured)}/{sum(int(r.get('dstar_generated_states') or 0) for r in measured)}；不含任何 Grid A* 调用。",
        f"- 拓扑生命周期：build/load=1/1，构建={topology_info['topology_build_time_ms']:.2f} ms，持久化={topology_info['topology_persist_time_ms']:.2f} ms，加载={topology_info['topology_load_time_ms']:.2f} ms；这些不计入 online pipeline wall time。",
        f"- Smac session：start/close/restart={session.session_start_count}/{session.session_close_count}/{session.session_restart_count}；RRTstar/SST=0/0。",
        "",
        "## 口径",
        "",
        "本轮只验证静态地图下的初次全局规划成本；`dstar_incremental_update=false`。因此不能把本轮结果宣称为动态障碍增量更新收益，也不能与 V7/两层历史结果做不同地图或不同重复次数的直接优劣结论。",
        "",
        "## 失败与路径质量",
        "",
        "失败 query 按结构化原因码写入 `failure_summary.csv`；成功路径的静态 footprint、运动学、倒车、原地旋转、航向/位置/转向连续性指标写入 `path_metrics.csv`。",
        "",
        "## 对比",
        "",
        "`architecture_comparison.yaml` 仅读取现有独立历史产物，给出可追溯的 3A-V0 / 2A-V0 默认实现参考统计；没有重复运行历史实验，且本轮不解锁动态多地图实验。",
    ]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the independent 3D-V0 static initial-planning performance benchmark")
    parser.add_argument("--output-dir", default=str(ROOT / "experiments/layered_planner_benchmark" / OUTPUT_NAME))
    parser.add_argument("--query-id", action="append", dest="query_ids")
    parser.add_argument("--warmups", type=int, default=WARMUPS)
    parser.add_argument("--repetitions", type=int, default=REPETITIONS)
    parser.add_argument("--timeout-s", type=float, default=TIMEOUT_S)
    parser.add_argument("--ros-domain-id", type=int, default=None)
    parser.add_argument("--preflight", action="store_true", help="allow a small non-formal real-Smac preflight run")
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output = run_benchmark(
            Path(args.output_dir).resolve(), query_ids=args.query_ids,
            warmups=int(args.warmups), repetitions=int(args.repetitions),
            timeout_s=float(args.timeout_s), ros_domain_id=args.ros_domain_id,
            enforce_formal_protocol=not bool(args.preflight),
        )
    except (OSError, RuntimeError, ValueError, KeyError) as exc:
        print(f"3d_v0_static_performance: ERROR: {exc}")
        return 2
    print(f"3D-V0 output: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
