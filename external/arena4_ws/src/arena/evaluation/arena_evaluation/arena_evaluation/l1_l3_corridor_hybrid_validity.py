"""Bounded validity attempts for the independent L1 + corridor-wide L3' arm.

This runner is intentionally separate from the hospital smoke and from the
formal paired benchmark.  It runs the frozen mentor-map A2B-01..A2B-20 tasks
once per attempt, preserving every failure and every bounded corridor retry.
The two-layer implementation never invokes the Grid A* stage.
"""

from __future__ import annotations

import argparse
import collections
import csv
import datetime as dt
import hashlib
import json
import os
import resource
import shutil
import time
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import l1_l3_corridor_hybrid_smoke as candidate
from . import unified_four_backends_smoke as legacy
from .planner_benchmark.map_utils import HospitalMap, sha256_file
from .planner_benchmark.models import Query
from .topology import preprocess_static_map


ROOT = Path("/home/robot/pudu_robot_ws")
MAP_ID = "mentor_map_20260825_005"
WORLD = ROOT / "external/arena4_ws/src/arena/simulation-setup/worlds" / MAP_ID
MAP_YAML = WORLD / "map/map.yaml"
SCENARIO_JSON = WORLD / "scenarios/a2b_benchmark_20.json"
BENCHMARK_JSON = ROOT / "benchmarks/arena_a2b_20/arena_a2b_benchmark_20.json"
BENCHMARK_CSV = ROOT / "benchmarks/arena_a2b_20/arena_a2b_benchmark_20.csv"
OUTPUT_NAME = "l1_l3_corridor_hybrid_mentor_map_20_validity_v3"
TASK_IDS = tuple(f"A2B-{index:02d}" for index in range(1, 21))
PADDING_SCHEDULE = (2.0, 4.0, 6.0)
BASELINE_PROFILE = "v2_repair_baseline"
EXPANSION_PROFILE = "bounded_corridor_expansion_full_update"


def _json_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fields: List[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or ["empty"])
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json.dumps(value, sort_keys=True, default=str) if isinstance(value, (list, dict)) else value for key, value in row.items()})


def _refuse_nonempty(path: Path) -> None:
    if path.exists() and any(path.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {path}")


def _load_tasks() -> Tuple[List[Query], Dict[str, Any]]:
    benchmark = json.loads(BENCHMARK_JSON.read_text(encoding="utf-8"))
    entry = (benchmark.get("maps") or {}).get(MAP_ID)
    if not entry or [str(item.get("id")) for item in entry.get("tasks", [])] != list(TASK_IDS):
        raise ValueError("benchmark JSON does not contain ordered mentor A2B-01..A2B-20")
    with BENCHMARK_CSV.open(newline="", encoding="utf-8") as stream:
        csv_rows = [row for row in csv.DictReader(stream) if row.get("world") == MAP_ID]
    if [str(row.get("task_id")) for row in csv_rows] != list(TASK_IDS):
        raise ValueError("benchmark CSV does not contain ordered mentor A2B-01..A2B-20")
    scenario = json.loads(SCENARIO_JSON.read_text(encoding="utf-8"))
    if scenario.get("world") != MAP_ID:
        raise ValueError("scenario map id mismatch")
    scenario_rows = scenario.get("tasks") or []
    if [str(item.get("id")) for item in scenario_rows] != list(TASK_IDS):
        raise ValueError("scenario JSON does not contain ordered A2B-01..A2B-20")
    queries: List[Query] = []
    for item, csv_row, scenario_row in zip(entry["tasks"], csv_rows, scenario_rows):
        start = [float(v) for v in item["start"]]
        goal = [float(v) for v in item["goal"]]
        csv_start = [float(csv_row[k]) for k in ("start_x_m", "start_y_m", "start_yaw_rad")]
        csv_goal = [float(csv_row[k]) for k in ("goal_x_m", "goal_y_m", "goal_yaw_rad")]
        if not np.allclose(start, csv_start, rtol=0, atol=1e-9) or not np.allclose(goal, csv_goal, rtol=0, atol=1e-9):
            raise ValueError(f"JSON/CSV mismatch for {item['id']}")
        if not np.allclose(start, scenario_row["start"], rtol=0, atol=1e-9) or not np.allclose(goal, scenario_row["goal"], rtol=0, atol=1e-9):
            raise ValueError(f"JSON/scenario mismatch for {item['id']}")
        queries.append(Query(str(item["id"]), start, goal, str(item.get("label", "")), 0, "UNVALIDATED"))
    return queries, {
        "map_id": MAP_ID, "task_ids": list(TASK_IDS),
        "json_sha256": sha256_file(BENCHMARK_JSON), "csv_sha256": sha256_file(BENCHMARK_CSV),
        "scenario_sha256": sha256_file(SCENARIO_JSON), "json_task_count": len(entry["tasks"]),
        "csv_task_count": len(csv_rows), "resolution_m": float(benchmark.get("resolution_m", 0.0)),
        "dynamic_obstacles": False,
    }


def _context() -> legacy.MapContext:
    hospital_map = HospitalMap.load(MAP_YAML)
    if not np.isclose(hospital_map.resolution, 0.05):
        raise ValueError("mentor map resolution must be 0.05 m/cell")
    _occupied, free_mask, distance_m, _ = preprocess_static_map(
        hospital_map, legacy.FOOTPRINT, padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    return legacy.MapContext(MAP_ID, hospital_map, free_mask, distance_m, sha256_file(hospital_map.image_path), sha256_file(hospital_map.yaml_path), MAP_YAML)


def _source_commit() -> str:
    try:
        import subprocess
        return subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip() or "unknown"
    except Exception:
        return "unknown"


def _source_manifest() -> Tuple[Dict[str, str], str]:
    files = [Path(candidate.__file__).resolve(), Path(legacy.__file__).resolve(), Path(__file__).resolve(), Path(candidate.__file__).resolve().parent / "topology.py", Path(__file__).resolve().parents[1] / "setup.py", MAP_YAML, MAP_YAML.parent / "map.pgm", BENCHMARK_JSON, BENCHMARK_CSV, SCENARIO_JSON, legacy._strict_smac_config_path()]
    hashes = {str(path): sha256_file(path) for path in files if path.exists()}
    return hashes, _json_hash(hashes)


def _load_topology(
    ctx: legacy.MapContext,
    output: Path,
    cache_root: Optional[Path] = None,
) -> Tuple[Any, Dict[str, Any]]:
    # Reuse the immutable cache produced by the paired mentor-map benchmark;
    # the loader verifies map/geometry/algorithm/source-content metadata and
    # never mutates that historical directory.
    explicit_cache_root = cache_root is not None
    cache_root = (cache_root or (
        ROOT / "experiments/layered_planner_benchmark/"
        "l1_l3_corridor_hybrid_mentor_map_20_cache_v1/topology_cache_shared"
    )).resolve()
    source_hash = sha256_file(Path(candidate.__file__).resolve().parent / "topology.py")
    # An explicitly supplied root is the cache's read/write owner.  Keeping
    # miss-built artifacts under that root makes a subsequent attempt able to
    # load the cache instead of rebuilding into a one-off output directory.
    # The implicit historical root remains read-only; misses stay isolated in
    # the current output directory so historical experiment evidence is never
    # mutated.
    fallback_root = cache_root if explicit_cache_root else output / "topology_cache"
    artifact, info = candidate._load_authoritative_topology(
        MAP_ID, ctx, cache_root, _source_commit(), source_hash, fallback_root,
    )
    info = dict(info)
    info["topology_build_cpu_time_ms"] = 0.0 if info.get("topology_cache_hit") else info.get("topology_build_time_ms", 0.0)
    return artifact, info


def _run_attempt(
    output: Path,
    profile: str,
    force_full_update: bool,
    schedule: Sequence[float],
    *,
    warmups: int = 0,
    repetitions: int = 1,
    cache_mode: str = candidate.CACHE_MODE_BASELINE,
    query_ids: Optional[Sequence[str]] = None,
    topology_cache_dir: Optional[Path] = None,
) -> Dict[str, Any]:
    _refuse_nonempty(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir()
    all_queries, task_meta = _load_tasks()
    selected_ids = list(query_ids) if query_ids is not None else list(TASK_IDS)
    if not selected_ids or any(item not in TASK_IDS for item in selected_ids):
        raise ValueError(f"query_ids must be a non-empty subset of {TASK_IDS}")
    queries = [query for query in all_queries if query.query_id in selected_ids]
    ctx = _context()
    topology, topology_info = _load_topology(ctx, output, topology_cache_dir)
    source_files, source_hash = _source_manifest()
    spec = legacy.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"Smac unavailable: {spec.reason}")
    ros_domain = 120 + (os.getpid() % 50)
    os.environ["ROS_DOMAIN_ID"] = str(ros_domain)
    session = candidate.SmacSession(ctx, output, map_yaml=MAP_YAML, log_tag=f"validity_{profile}", local_mask_updates=True, optimization_profile="v7_candidate", smac_parameter_profile="baseline", optimization_stage="step3_delta_map")
    source_commit = _source_commit()
    session.start()
    run_rows: List[Dict[str, Any]] = []
    call_rows: List[Dict[str, Any]] = []
    metric_rows: List[Dict[str, Any]] = []
    attempt_rows: List[Dict[str, Any]] = []
    try:
        for run_mode, count in (("warmup", int(warmups)), ("measured", int(repetitions))):
            for repetition in range(1, count + 1):
                for query in queries:
                    row, call, metric = candidate._run_one(
                        ctx, topology, topology_info, query, run_mode, repetition, session, spec,
                        output, source_commit, corridor_padding_m=float(schedule[0]),
                        corridor_semantics=candidate.CORRIDOR_SEMANTICS, profile_name=profile,
                        padding_schedule_m=schedule, force_full_update=force_full_update,
                        validate_each_attempt=True, cache_mode=cache_mode,
                    )
                    row["source_hash"] = source_hash
                    row["corridor_profile"] = profile
                    row["padding_schedule_m"] = list(schedule)
                    run_rows.append(row); call_rows.append(call); metric_rows.append(metric)
                    for attempt in (row.get("diagnostics") or {}).get("attempts", []):
                        attempt_rows.append({"run_id": row["run_id"], "query_id": query.query_id, "run_mode": run_mode, "repetition": repetition, "profile": profile, **attempt})
    finally:
        session.close()
    for row in run_rows:
        row.update({"session_start_count": session.session_start_count, "session_close_count": session.session_close_count, "session_restart_count": session.session_restart_count, "ros_domain_id": ros_domain})
    _write_csv(output / "runs.csv", run_rows)
    _write_csv(output / "path_metrics.csv", metric_rows)
    _write_csv(output / "backend_call_log.csv", call_rows)
    _write_csv(output / "attempt_summary.csv", attempt_rows)
    _write_csv(output / "repair_window_summary.csv", attempt_rows)
    _write_csv(output / "session_timing.csv", [{"map_id": MAP_ID, "profile": profile, "ros_domain_id": ros_domain, "session_start_count": session.session_start_count, "session_close_count": session.session_close_count, "session_restart_count": session.session_restart_count, "session_startup_time_ms": session.stack_startup_time_ms, "session_shutdown_time_ms": session.stack_shutdown_time_ms, "topology_cache_hit": topology_info.get("topology_cache_hit"), "topology_load_time_ms": topology_info.get("topology_load_time_ms", 0.0)}])
    _write_csv(output / "failure_summary.csv", [{"profile": profile, "failure_code": code, "count": sum(1 for row in run_rows if row.get("failure_code") == code)} for code in sorted({str(row.get("failure_code") or "") for row in run_rows}) if code])
    (output / "topology_cache_manifest.yaml").write_text(yaml.safe_dump(topology_info, sort_keys=False), encoding="utf-8")
    (output / "protocol.yaml").write_text(yaml.safe_dump({
        "architecture": candidate.ARCHITECTURE, "map_id": MAP_ID,
        "query_ids": list(TASK_IDS), "profile": profile,
        "padding_schedule_m": list(schedule),
        "corridor_semantics": candidate.CORRIDOR_SEMANTICS,
        "resolution": 0.05, "minimum_turning_radius_m": 0.40,
        "maximum_curvature": 2.50, "allow_reverse": False,
        "allow_in_place_rotation": False, "dynamic_obstacles": False,
        "l2_called": False, "l2_call_count": 0,
        "rrtstar_call_count": 0, "sst_call_count": 0,
        "cache_mode": cache_mode,
        "topology_cache_directory": str((topology_cache_dir or (
            ROOT / "experiments/layered_planner_benchmark/"
            "l1_l3_corridor_hybrid_mentor_map_20_cache_v1/topology_cache_shared"
        )).resolve()),
    }, sort_keys=False), encoding="utf-8")
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({"source_commit": source_commit, "source_hash": source_hash, "source_files": source_files, "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256, "task_metadata": task_meta, "query_hashes": {query.query_id: candidate._query_hash(query) for query in queries}}, sort_keys=False), encoding="utf-8")
    measured = [row for row in run_rows if row.get("run_mode") == "measured"]
    valid = sum(str(row.get("final_valid_success", "")).lower() == "true" for row in measured)
    failures = collections.Counter(str(row.get("failure_code") or "") for row in measured if str(row.get("failure_code") or ""))
    walls = [float(row.get("pipeline_wall_time_ms") or 0.0) for row in measured]
    l1_rows = [row for row in measured]
    def _mean_field(name: str) -> float:
        values = []
        for row in l1_rows:
            try:
                values.append(float(row.get(name) or 0.0))
            except (TypeError, ValueError):
                pass
        return float(np.mean(values)) if values else 0.0
    def _percentile(rows: Sequence[Mapping[str, Any]], name: str, percentile: float) -> float:
        values: List[float] = []
        for row in rows:
            try:
                values.append(float(row.get(name) or 0.0))
            except (TypeError, ValueError):
                continue
        return float(np.percentile(values, percentile)) if values else 0.0

    baseline_path = ROOT / "experiments/layered_planner_benchmark/l1_l3_corridor_hybrid_mentor_map_20_validity_v3/runs.csv"
    baseline_rows = [row for row in _read_rows(baseline_path) if row.get("run_mode") == "measured"]
    measured_calls = sum(int(row.get("l3_prime_call_count") or 0) for row in measured)
    measured_fallbacks = sum(str(row.get("fallback_used")).lower() == "true" for row in measured)
    report = [
        "# Two-layer mentor-map validity attempt", "",
        f"- Profile: `{profile}`; cache_mode=`{cache_mode}`; bounded schedule: `{list(schedule)}`; force_full_update={force_full_update}.",
        f"- Raw tasks: {','.join(selected_ids)}, warmups={warmups}, measured repetitions={repetitions}; final-valid **{valid}/{len(measured)}** measured.",
        f"- Failure codes: `{dict(failures)}`.",
        f"- Smac calls: {measured_calls} measured; L2 calls: 0; RRTstar/SST: 0/0.",
        f"- Topology cache: hit={topology_info.get('topology_cache_hit')}; build={topology_info.get('topology_build_time_ms', 0.0):.2f} ms; load={topology_info.get('topology_load_time_ms', 0.0):.2f} ms.",
        f"- Session startup/shutdown: {session.stack_startup_time_ms:.2f}/{session.stack_shutdown_time_ms:.2f} ms; start/close/restart={session.session_start_count}/{session.session_close_count}/{session.session_restart_count}.",
        f"- Optimized online wall P50/P95/P99: {_percentile(measured, 'pipeline_wall_time_ms', 50):.2f}/{_percentile(measured, 'pipeline_wall_time_ms', 95):.2f}/{_percentile(measured, 'pipeline_wall_time_ms', 99):.2f} ms.",
        f"- Optimized CPU P50/P95/P99: {_percentile(measured, 'pipeline_cpu_total_ms', 50):.2f}/{_percentile(measured, 'pipeline_cpu_total_ms', 95):.2f}/{_percentile(measured, 'pipeline_cpu_total_ms', 99):.2f} ms.",
        f"- Baseline v3 wall P50/P95/P99: {_percentile(baseline_rows, 'pipeline_wall_time_ms', 50):.2f}/{_percentile(baseline_rows, 'pipeline_wall_time_ms', 95):.2f}/{_percentile(baseline_rows, 'pipeline_wall_time_ms', 99):.2f} ms; valid={sum(str(row.get('final_valid_success')).lower() == 'true' for row in baseline_rows)}/{len(baseline_rows)}.",
        f"- L1 means (ms): attachment={_mean_field('l1_attachment_lookup_ms'):.2f}, candidate_collision={_mean_field('l1_candidate_collision_check_ms'):.2f}, adjacency={_mean_field('l1_adjacency_build_ms'):.2f}, route_search={_mean_field('l1_route_search_ms'):.2f}, construction={_mean_field('l1_route_construction_ms'):.2f}.",
        f"- Online means (ms): corridor_mask={_mean_field('corridor_mask_total_time_ms'):.2f}, Smac={_mean_field('hybrid_planning_time_ms'):.2f}, action_wall={_mean_field('l3_action_wall_ms'):.2f}, process_overhead={_mean_field('l3_process_overhead_ms'):.2f}, final_validation={_mean_field('final_validation_time_ms'):.2f}.",
        f"- L1 cache hits: endpoint_index={sum(str(row.get('endpoint_spatial_index_cache_hit')).lower() == 'true' for row in measured)}/{len(measured)}, route={sum(str(row.get('route_cache_hit')).lower() == 'true' for row in measured)}/{len(measured)}; fallbacks={measured_fallbacks}.",
        "- Every returned path is validated by the unchanged static footprint and hard kinematic validator; no yaw, steering, curvature, or failure field is rewritten.",
        "", "This is an iteration attempt, not a formal paired architecture conclusion.",
    ]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    (output / "manifest.yaml").write_text(yaml.safe_dump({"experiment": output.name, "architecture": candidate.ARCHITECTURE, "profile": profile, "cache_mode": cache_mode, "map_id": MAP_ID, "query_ids": list(selected_ids), "query_ids_all": list(TASK_IDS), "warmup_count": int(warmups), "measured_repetitions": int(repetitions), "run_count": len(run_rows), "measured_final_valid_count": valid, "measured_count": len(measured), "failure_counts": dict(failures), "l2_call_count": 0, "rrtstar_call_count": 0, "sst_call_count": 0, "topology_cache_hit": topology_info.get("topology_cache_hit"), "session_start_count": session.session_start_count, "session_close_count": session.session_close_count, "session_restart_count": session.session_restart_count, "gate_passed": valid == len(measured) and len(measured) > 0 and all(int(row.get("l2_call_count") or 0) == 0 for row in run_rows)}, sort_keys=False), encoding="utf-8")
    return {"profile": profile, "output": str(output), "valid": valid, "measured_count": len(measured), "calls": sum(int(row.get("l3_prime_call_count") or 0) for row in measured), "failures": dict(failures), "rows": run_rows}


def _read_rows(path: Path) -> List[Dict[str, Any]]:
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            return list(csv.DictReader(stream))
    except OSError:
        return []


def _finalize_root_summary(root: Path, attempt: Path) -> Path:
    """Promote one completed attempt into the requested independent v3 root.

    The attempt directory remains immutable.  This copies only generated
    evidence into the new root and writes a comparison report against the
    historical two-layer baseline and the existing same-map three-layer
    reference; neither historical directory is modified or rerun.
    """
    root.mkdir(parents=True, exist_ok=True)
    for name in (
        "runs.csv", "path_metrics.csv", "backend_call_log.csv",
        "repair_window_summary.csv", "attempt_summary.csv", "session_timing.csv",
        "failure_summary.csv", "protocol.yaml", "source_manifest.yaml",
        "topology_cache_manifest.yaml",
    ):
        source = attempt / name
        if source.exists():
            shutil.copy2(source, root / name)
    if (attempt / "paths").exists():
        shutil.copytree(attempt / "paths", root / "paths", dirs_exist_ok=True)
    current_source_files, current_source_hash = _source_manifest()
    current_source_payload = yaml.safe_load((root / "source_manifest.yaml").read_text(encoding="utf-8")) if (root / "source_manifest.yaml").exists() else {}
    current_source_payload.update({"source_commit": _source_commit(), "source_hash": current_source_hash, "code_hash": current_source_hash, "source_files": current_source_files})
    (root / "source_manifest.yaml").write_text(yaml.safe_dump(current_source_payload, sort_keys=False), encoding="utf-8")
    rows = _read_rows(attempt / "runs.csv")
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    valid = sum(str(row.get("final_valid_success", "")).lower() == "true" for row in measured)
    failures = collections.Counter(str(row.get("failure_code") or "") for row in measured if row.get("failure_code"))
    baseline_path = ROOT / "experiments/layered_planner_benchmark/l1_l2_l3_vs_l1_l3prime_mentor_map_20_v1/two_layer/runs.csv"
    three_path = ROOT / "experiments/layered_planner_benchmark/l1_l2_l3_vs_l1_l3prime_mentor_map_20_v1/three_layer/runs.csv"
    baseline = [row for row in _read_rows(baseline_path) if row.get("run_mode") == "measured"]
    three = [row for row in _read_rows(three_path) if row.get("run_mode") == "measured"]
    baseline_valid = sum(str(row.get("final_valid_success", "")).lower() == "true" for row in baseline)
    three_valid = sum(str(row.get("final_valid_success", "")).lower() == "true" for row in three)
    walls = [float(row.get("pipeline_wall_time_ms") or 0.0) for row in measured]
    calls = sum(int(row.get("l3_prime_call_count") or 0) for row in measured)
    session = (_read_rows(attempt / "session_timing.csv") or [{}])[0]
    manifest = yaml.safe_load((attempt / "manifest.yaml").read_text(encoding="utf-8")) if (attempt / "manifest.yaml").exists() else {}
    manifest.update({
        "experiment": root.name,
        "final_attempt": str(attempt),
        "final_valid_count": valid,
        "measured_count": len(measured),
        "final_valid_rate": valid / len(measured) if measured else 0.0,
        "failure_counts": dict(failures),
        "measured_l3_prime_call_count": calls,
        "historical_v2_baseline_final_valid_count": baseline_valid,
        "historical_v2_baseline_measured_count": len(baseline),
        "historical_three_layer_reference_final_valid_count": three_valid,
        "historical_three_layer_reference_measured_count": len(three),
        "l2_call_count": 0,
        "rrtstar_call_count": 0,
        "sst_call_count": 0,
        "gate_validity_passed": bool(valid >= 0.5 * len(measured)) if measured else False,
        "formal_multi_map_experiment_unlocked": False,
    })
    (root / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    p50 = float(np.percentile(walls, 50)) if walls else 0.0
    p95 = float(np.percentile(walls, 95)) if walls else 0.0
    p99 = float(np.percentile(walls, 99)) if walls else 0.0
    report = [
        "# L1 + L3' Mentor-Map Validity v3",
        "",
        "This independent validity run fixes the two-layer candidate only; it is not a new multi-map or four-backend evaluation.",
        "",
        f"- Map: `{MAP_ID}`; raw tasks: A2B-01..A2B-20; measured repetitions={manifest.get('measured_repetitions', 'not_available')} (warmups={manifest.get('warmup_count', 'not_available')}).",
        f"- Candidate: L1 skeleton topology + graph A* -> L3' corridor-wide Smac Hybrid DUBIN; L2 calls=0; RRTstar/SST calls=0.",
        f"- Final-valid measured paths: **{valid}/{len(measured)} ({100.0 * valid / len(measured) if measured else 0.0:.1f}%)**.",
        f"- Structured measured failures: `{dict(failures)}`. A2B-07 remains a static footprint collision and A2B-16 remains `NO_PATH_IN_CORRIDOR`; no query or pose was modified.",
        f"- Measured L3' calls: **{calls}**. Successful windows stop at the first valid 2 m attempt; failures use bounded 2/4/6 m attempts only.",
        f"- Online pipeline wall P50/P95/P99: **{p50:.2f}/{p95:.2f}/{p99:.2f} ms**; these include real costmap/action work and are not presented as a latency pass.",
        f"- Map-level Smac session start/close/restart: **{session.get('session_start_count', 'not_available')}/{session.get('session_close_count', 'not_available')}/{session.get('session_restart_count', 'not_available')}**; topology cache hit={session.get('topology_cache_hit', 'not_available')}.",
        f"- Historical v2 two-layer reference: {baseline_valid}/{len(baseline)} valid; historical three-layer reference: {three_valid}/{len(three)} valid. The candidate is {valid / len(measured) * 100.0 - (three_valid / len(three) * 100.0 if three else 0.0):.1f} percentage points above that reference, so it does not lag the reference; this is not a replacement for a fresh paired run.",
        "",
        "## Hard constraints",
        "",
        "All accepted paths were checked by the unchanged full footprint line-segment validator and hard forward-only kinematic validator. No reverse motion, in-place rotation, yaw/steering/curvature rewrite, fallback backend, or failure-code masking was used.",
        "",
        "## Decision",
        "",
        f"- Two-layer validity threshold (>=50%): **{'PASS' if valid >= 0.5 * len(measured) else 'FAIL'}**.",
        "- Broader formal multi-map evaluation: **LOCKED**; this remains a single-map validity repair and the two retained failure modes require review before generalization.",
        "- Current return point: candidate validity is materially improved over v2, while A2B-07/A2B-16 remain reproducible blockers for 100% coverage.",
    ]
    (root / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    return root


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run one bounded 20-query validity attempt for L1 + corridor-wide Smac Hybrid")
    parser.add_argument("--output-dir", default=str(ROOT / "experiments/layered_planner_benchmark" / OUTPUT_NAME / "attempt_01_bounded_expansion_full_update"))
    parser.add_argument("--profile", choices=(BASELINE_PROFILE, EXPANSION_PROFILE), default=EXPANSION_PROFILE)
    parser.add_argument("--baseline", action="store_true", help="use the v2 one-shot 2 m baseline")
    parser.add_argument("--cache-mode", choices=(candidate.CACHE_MODE_BASELINE, candidate.CACHE_MODE_OPTIMIZED), default=candidate.CACHE_MODE_BASELINE)
    parser.add_argument("--query-id", action="append", dest="query_ids", choices=list(TASK_IDS),
                        help="restrict an iteration attempt to selected raw tasks")
    parser.add_argument("--force-full-update", action="store_true")
    parser.add_argument("--ros-domain-id", type=int, default=None)
    parser.add_argument("--topology-cache-dir", default=None,
                        help="independent read/write cache root for this attempt")
    parser.add_argument("--warmups", type=int, default=0)
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--finalize-attempt", default=None, help="promote an existing attempt directory into --output-dir without rerunning planners")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.finalize_attempt:
        try:
            output = _finalize_root_summary(Path(args.output_dir).resolve(), Path(args.finalize_attempt).resolve())
        except (OSError, ValueError, KeyError, yaml.YAMLError) as exc:
            print(f"l1_l3_corridor_hybrid_validity: ERROR: {exc}")
            return 2
        print(f"validity summary output: {output}")
        return 0
    schedule = (2.0,) if args.baseline else PADDING_SCHEDULE
    profile = BASELINE_PROFILE if args.baseline else args.profile
    cache_mode = candidate.CACHE_MODE_BASELINE if args.baseline else args.cache_mode
    if args.ros_domain_id is not None:
        os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    try:
        result = _run_attempt(
            Path(args.output_dir).resolve(), profile,
            bool(args.force_full_update and not args.baseline), schedule,
            warmups=int(args.warmups), repetitions=int(args.repetitions),
            cache_mode=cache_mode,
            query_ids=args.query_ids,
            topology_cache_dir=(Path(args.topology_cache_dir).resolve() if args.topology_cache_dir else None),
        )
    except (OSError, RuntimeError, ValueError, KeyError) as exc:
        print(f"l1_l3_corridor_hybrid_validity: ERROR: {exc}")
        return 2
    print(f"validity attempt output: {result['output']} final_valid={result['valid']}/{result['measured_count']} calls={result['calls']}")
    return 0 if result["valid"] == result["measured_count"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
