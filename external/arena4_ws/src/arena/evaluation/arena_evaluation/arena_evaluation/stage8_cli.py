"""Stage 8A/8B static benchmark command.

8A consumes frozen Stage 6 paths and validates/repairs them with a single
static Nav2 Smac Hybrid REEDS_SHEPP stack.  8B is a deterministic offline
L2 preference scan; it never changes the static free-space mask.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import gzip
import json
import math
import os
import shutil
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import pandas as pd
import yaml

from .planner_benchmark.config import load_protocol, load_queries, resolve_path, stack_parameters
from .planner_benchmark.models import Query
from .planner_benchmark.map_utils import HospitalMap, sha256_file
from .planner_benchmark.runner import BenchmarkStack, ComputePathClient
from .stage8 import (
    HardDiagnostics,
    HardRadiusConfig,
    RepairWindow,
    arc_repair_windows,
    classify_segments,
    diagnose_hard_path,
    distance,
    merge_repair_windows,
    repair_window_schedule,
    stitch_errors,
    trigger_indices,
)

DEFAULT_STAGE6 = Path("experiments/layered_planner_benchmark/hospital_005/stage6_l1_l2")
DEFAULT_STAGE7 = Path("experiments/layered_planner_benchmark/hospital_005/stage7_l3_kinematic")
DEFAULT_SMAC = Path("experiments/planner_benchmark/hospital_005/stage5_smac_normalized")
DEFAULT_MAP = Path("experiments/maps/hospital_005/map.yaml")
DEFAULT_QUERIES = Path("experiments/planner_benchmark/hospital_005/queries_v2.yaml")
FOOTPRINT = [[0.255, 0.215], [0.255, -0.215], [-0.255, -0.215], [-0.255, 0.215]]
Q04_FAILURE = "STATIC_SEMANTICS_CONSERVATIVE_INFLATION_MISMATCH"


def _read_points(path: Path) -> List[Dict[str, float]]:
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        payload = json.load(stream)
    return [{"x": float(p["x"]), "y": float(p["y"]), "yaw": float(p["yaw"])} for p in payload]


def _save_points(path: Path, points: Sequence[Dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(path, "wt", encoding="utf-8") as stream:
        json.dump(list(points), stream)


def _write_rows(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: List[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader(); writer.writerows(rows)


def _path_from_row(directory: Path, row: pd.Series) -> Optional[List[Dict[str, float]]]:
    value = row.get("path_file")
    if value is None or str(value) in {"", "nan", "NaN"}:
        return None
    path = Path(str(value))
    if not path.is_absolute():
        path = directory / path
    return _read_points(path) if path.exists() else None


def _stage8_config() -> HardRadiusConfig:
    return HardRadiusConfig()


def _protocol_payload(protocol: Dict[str, object], map_path: Path, config: HardRadiusConfig) -> Dict[str, object]:
    payload = dict(protocol)
    payload.update({
        "schema_version": 8,
        "experiment": "hospital_stage8_hard_radius_l3",
        "map_yaml": str(map_path),
        "map_sha256": sha256_file(map_path.parent / yaml.safe_load(map_path.read_text())["image"]),
        "resolution": float(HospitalMap.load(map_path).resolution),
        "footprint": FOOTPRINT,
        "allow_in_place_rotation": False,
        "minimum_turning_radius": config.minimum_turning_radius,
        "maximum_curvature": config.maximum_curvature,
        "allow_reverse": True,
        "reverse_penalty": config.reverse_penalty,
        "motion_model": config.motion_model,
        "local_smac_angle_quantization_bins": 360,
        "local_smac_smooth_path": False,
        "local_smac_footprint_padding_m": 0.10,
        "dynamic_obstacles": False,
        "curvature_sample_spacing_m": config.curvature_sample_spacing_m,
        "curvature_evaluation_window_m": config.curvature_evaluation_window_m,
        "collision_sample_spacing_m": config.collision_sample_spacing_m,
        "numerical_tolerance": config.numerical_tolerance,
        "stitch_position_tolerance_m": config.stitch_position_tolerance_m,
        "stitch_yaw_tolerance_deg": config.stitch_yaw_tolerance_deg,
        "initial_repair_window_m": config.initial_repair_window_m,
        "expanded_repair_window_m": config.expanded_repair_window_m,
    })
    return payload


class StaticSmacBackend:
    """One planner stack and one action client reused for all local windows."""

    def __init__(self, map_path: Path, protocol: Dict[str, object], output: Path, timeout: float):
        self.map_path = map_path
        self.protocol = protocol
        self.output = output
        self.timeout = timeout
        self.stack: Optional[BenchmarkStack] = None
        self.client: Optional[ComputePathClient] = None
        self.planner_pid = 0
        self.stack_pids: List[int] = []
        self.params_file = output / "logs" / "stage8_smac_reeds_shepp_params.yaml"

    def start(self) -> None:
        import rclpy
        config_path = Path(__file__).resolve().parents[1] / "config" / "planner_benchmark_normalized_smac_hybrid.yaml"
        planner_config = yaml.safe_load(config_path.read_text()) or {}
        planner_config["motion_model_for_search"] = "REEDS_SHEPP"
        planner_config["minimum_turning_radius"] = 0.40
        planner_config["reverse_penalty"] = 2.0
        # Keep the planner's primitive geometry visible to the validator. The
        # Nav2 smoother can otherwise introduce a sub-radius chord after the
        # REEDS_SHEPP search has satisfied its primitive constraint.
        planner_config["smooth_path"] = False
        if isinstance(planner_config.get("GridBased"), dict):
            planner_config["GridBased"]["motion_model_for_search"] = "REEDS_SHEPP"
            planner_config["GridBased"]["minimum_turning_radius"] = 0.40
            planner_config["GridBased"]["reverse_penalty"] = 2.0
            planner_config["GridBased"]["smooth_path"] = False
            planner_config["GridBased"]["angle_quantization_bins"] = 360
        params = stack_parameters(protocol=self.protocol, planner_config=planner_config)
        params["global_costmap"]["global_costmap"]["ros__parameters"]["footprint_padding"] = 0.10
        self.params_file.parent.mkdir(parents=True, exist_ok=True)
        self.params_file.write_text(yaml.safe_dump(params, sort_keys=False))
        self.stack = BenchmarkStack(map_yaml=self.map_path, params_file=self.params_file, log_file=self.output / "logs" / "stage8_smac_stack.log")
        self.stack.start()
        self.planner_pid, self.stack_pids, error = self.stack.pids()
        if error:
            self.stack.stop(); raise RuntimeError(error)
        self.client = ComputePathClient(timeout=self.timeout)
        self._rclpy = rclpy

    def stop(self) -> None:
        if self.client is not None:
            self.client.close(); self.client = None
        if self.stack is not None:
            self.stack.stop(); self.stack = None

    def plan(self, start: Point, goal: Point) -> Tuple[Optional[List[Point]], Dict[str, object]]:
        if self.client is None:
            return None, {"action_status": "UNAVAILABLE", "result_code": "LOCAL_HYBRID_NO_PATH"}
        query = Query("local_window", [start["x"], start["y"], start["yaw"]], [goal["x"], goal["y"], goal["yaw"]], "stage8_local", 20260821, "VALID")
        started = time.monotonic_ns()
        status, code, wall, measurement, points, result = self.client.plan(query, planner_pid=self.planner_pid, stack_pids=self.stack_pids, sample_interval_ms=10.0)
        detail = {
            "action_status": status, "result_code": code, "wall_time_ms": wall,
            "planning_time_ms": (float(getattr(result.planning_time, "sec", 0)) * 1000.0 + float(getattr(result.planning_time, "nanosec", 0)) / 1e6) if result is not None and getattr(result, "planning_time", None) is not None else None,
            "cpu_total_ms": getattr(measurement, "planner_cpu_total_ms", None) if measurement else None,
            "rss_peak_bytes": getattr(measurement, "planner_rss_peak_bytes", None) if measurement else None,
            "pss_peak_bytes": getattr(measurement, "planner_pss_peak_bytes", None) if measurement else None,
            "elapsed_ms": (time.monotonic_ns() - started) / 1e6,
        }
        return points, detail


def _within_window(candidate: Sequence[Dict[str, float]], original: Sequence[Point], window: RepairWindow, radius: float) -> bool:
    anchors = original[window.start_index:window.end_index + 1]
    if not anchors:
        return False
    allowance = max(0.30, radius * 0.5)
    for point in candidate:
        if min(distance(point, anchor) for anchor in anchors) > allowance:
            return False
    return True


def _replace_window(points: Sequence[Point], window: RepairWindow, candidate: Sequence[Point]) -> List[Point]:
    return list(points[:window.start_index]) + list(candidate) + list(points[window.end_index + 1:])


def _expand_window(points: Sequence[Point], window: RepairWindow, extra_m: float) -> RepairWindow:
    start = window.start_index; travelled = 0.0
    while start > 0 and travelled < extra_m:
        travelled += distance(points[start - 1], points[start]); start -= 1
    end = window.end_index; travelled = 0.0
    while end + 1 < len(points) and travelled < extra_m:
        travelled += distance(points[end], points[end + 1]); end += 1
    return RepairWindow(start, end, window.center_index, window.reason)


def repair_with_backend(points: Sequence[Point], hospital_map: HospitalMap, config: HardRadiusConfig, backend: StaticSmacBackend, topology_edges: Sequence[int], grid_mode: str) -> Dict[str, object]:
    before = diagnose_hard_path(points, hospital_map, FOOTPRINT, config)
    triggers = trigger_indices(points, before, config)
    windows = arc_repair_windows(points, triggers, config.initial_repair_window_m)
    if not windows:
        return {"success": before.hard_kinematic_valid, "points": list(points), "before": before, "after": before, "windows": [], "hybrid_calls": 0, "hybrid_success": 0, "failure_reason": "" if before.hard_kinematic_valid else "KINEMATIC_REPAIR_FAILED", "planning_time_ms": 0.0, "cpu_total_ms": 0.0, "rss_peak_bytes": None, "pss_peak_bytes": None}
    all_points = list(points)
    calls = 0; successes = 0; planning_ms = 0.0; cpu_ms = 0.0; rss = []; pss = []; used_windows = []; attempts = []; last_reason = "LOCAL_HYBRID_NO_PATH"
    # Work backwards so replacing a later subpath cannot invalidate the
    # original indices of an earlier repair window.
    for window in reversed(windows):
        repaired = False
        for radius in repair_window_schedule(config):
            expanded = window if math.isclose(radius, config.initial_repair_window_m) else _expand_window(all_points, window, radius - config.initial_repair_window_m)
            calls += 1
            start = all_points[expanded.start_index]; goal = all_points[expanded.end_index]
            candidate, measurement = backend.plan(start, goal)
            planning_ms += float(measurement.get("planning_time_ms") or 0.0)
            cpu_ms += float(measurement.get("cpu_total_ms") or 0.0)
            if measurement.get("rss_peak_bytes") is not None: rss.append(int(measurement["rss_peak_bytes"]))
            if measurement.get("pss_peak_bytes") is not None: pss.append(int(measurement["pss_peak_bytes"]))
            attempt = {"center_index": window.center_index, "window_start_index": expanded.start_index, "window_end_index": expanded.end_index, "window_radius_m": radius, **measurement}
            if not candidate or measurement.get("result_code") != "SUCCEEDED":
                last_reason = "LOCAL_HYBRID_NO_PATH"; attempt["rejection_reason"] = last_reason; attempts.append(attempt)
                continue
            if not _within_window(candidate, all_points, expanded, radius):
                last_reason = "LOCAL_HYBRID_LEFT_REPAIR_WINDOW"; attempt["rejection_reason"] = last_reason; attempts.append(attempt)
                continue
            candidate_diag = diagnose_hard_path(candidate, hospital_map, FOOTPRINT, config)
            if not candidate_diag.hard_kinematic_valid:
                if candidate_diag.static_collision_count:
                    last_reason = "LOCAL_HYBRID_STATIC_COLLISION"
                elif candidate_diag.hard_radius_violation_count:
                    last_reason = "MINIMUM_TURNING_RADIUS_VIOLATION"
                elif candidate_diag.zero_displacement_yaw_changes:
                    last_reason = "IN_PLACE_ROTATION_FORBIDDEN"
                else:
                    last_reason = candidate_diag.failure_codes[0] if candidate_diag.failure_codes else "KINEMATIC_REPAIR_FAILED"
                attempt.update({"rejection_reason": last_reason, **candidate_diag.as_dict()}); attempts.append(attempt)
                continue
            first_pos, first_yaw, first_ok = stitch_errors(start, candidate[0], config)
            last_pos, last_yaw, last_ok = stitch_errors(goal, candidate[-1], config)
            if not (first_ok and last_ok):
                last_reason = "STITCH_POSITION_DISCONTINUITY" if first_pos > config.stitch_position_tolerance_m or last_pos > config.stitch_position_tolerance_m else "STITCH_YAW_DISCONTINUITY"
                attempt.update({"rejection_reason": last_reason, "stitch_position_error_m": max(first_pos, last_pos), "stitch_yaw_error_deg": max(first_yaw, last_yaw)}); attempts.append(attempt)
                continue
            all_points = _replace_window(all_points, expanded, candidate)
            used_windows.append({"window": expanded, "padding_m": radius, "length_m": sum(distance(a, b) for a, b in zip(candidate, candidate[1:])), "stitch_position_error_m": max(first_pos, last_pos), "stitch_yaw_error_deg": max(first_yaw, last_yaw), "repair_reason": "IN_PLACE_ROTATION_FORBIDDEN_OR_HARD_RADIUS", "segments": classify_segments(candidate, grid_mode=grid_mode, topology_edge_ids=topology_edges, source="kinematic", planner="smac_hybrid_reeds_shepp", repair_reason="IN_PLACE_ROTATION_FORBIDDEN_OR_HARD_RADIUS")})
            successes += 1; repaired = True; attempt["rejection_reason"] = ""; attempts.append(attempt); break
        if not repaired:
            return {"success": False, "points": [], "before": before, "after": None, "windows": windows, "used_windows": used_windows, "attempts": attempts, "hybrid_calls": calls, "hybrid_success": successes, "failure_reason": "KINEMATIC_REPAIR_FAILED", "hybrid_failure_reason": last_reason, "planning_time_ms": planning_ms, "cpu_total_ms": cpu_ms, "rss_peak_bytes": max(rss) if rss else None, "pss_peak_bytes": max(pss) if pss else None}
    after = diagnose_hard_path(all_points, hospital_map, FOOTPRINT, config)
    return {"success": after.hard_kinematic_valid, "points": all_points if after.hard_kinematic_valid else [], "before": before, "after": after, "windows": windows, "used_windows": used_windows, "attempts": attempts, "hybrid_calls": calls, "hybrid_success": successes, "failure_reason": "" if after.hard_kinematic_valid else "KINEMATIC_REPAIR_FAILED", "hybrid_failure_reason": "" if after.hard_kinematic_valid else "KINEMATIC_REPAIR_FAILED", "planning_time_ms": planning_ms, "cpu_total_ms": cpu_ms, "rss_peak_bytes": max(rss) if rss else None, "pss_peak_bytes": max(pss) if pss else None}


def _load_stage6(stage6: Path) -> pd.DataFrame:
    manifest = yaml.safe_load((stage6 / "manifest.yaml").read_text()) or {}
    if bool(manifest.get("dynamic_obstacles", False)):
        raise ValueError("dynamic_obstacles=true in Stage 6 input")
    frame = pd.read_csv(stage6 / "query_runs.csv")
    frame = frame[frame["mode"].eq("topology_guided_grid_fallback")].copy()
    if "run_mode" in frame.columns:
        frame = frame[frame["run_mode"].eq("measured")]
    if len(frame) != 50:
        raise ValueError(f"Stage 8 requires 50 Stage 6 measured fallback rows, found {len(frame)}")
    return frame


def _run_row(row: pd.Series, stage6: Path, hospital_map: HospitalMap, config: HardRadiusConfig, backend: Optional[StaticSmacBackend], output: Path) -> Tuple[Dict[str, object], Optional[List[Point]]]:
    run_id = f"{row.query_id}_layered_hard_radius_l3_measured_{int(row.repetition)}_{time.time_ns()}"
    points = _path_from_row(stage6, row)
    base = {"run_id": run_id, "query_id": row.query_id, "repetition": int(row.repetition), "mode": "layered_hard_radius_l3", "source_stage6_run_id": row.run_id, "q04_diagnostic": Q04_FAILURE if row.query_id == "q04" else "", "dynamic_obstacles": False, "allow_in_place_rotation": False, "minimum_turning_radius_m": 0.40, "maximum_curvature": 2.50, "source": "grid", "grid_mode": row.get("grid_mode", ""), "topology_edge_ids": row.get("topology_edge_ids", "[]"), "action_success": bool(points), "static_footprint_valid": False, "final_valid_success": False, "hard_kinematic_valid": False, "turning_radius_preference_satisfied": False, "failure_code": "", "hybrid_calls": 0, "hybrid_success": 0, "repair_window_count": 0, "repair_window_padding_m": None, "repair_length_m": 0.0, "repair_length_ratio": 0.0, "rotate_in_place_count": 0, "stitch_position_error_m": None, "stitch_yaw_error_deg": None, "l3_planning_time_ms": 0.0, "l3_cpu_total_ms": 0.0, "l3_rss_peak_bytes": None, "l3_pss_peak_bytes": None, "stage6_online_time_ms": row.get("total_online_time_ms"), "composed_online_time_ms": None, "composed_online_time_is_estimate": True}
    if row.query_id == "q04" or points is None:
        base["failure_code"] = Q04_FAILURE if row.query_id == "q04" else "EMPTY_PATH"
        return base, None
    before = diagnose_hard_path(points, hospital_map, FOOTPRINT, config)
    base.update({"heading_jump_count_before": before.heading_jump_count, "hard_radius_violation_count_before": before.hard_radius_violation_count, "static_footprint_collision_count_before": before.static_collision_count, "trigger_count": len(trigger_indices(points, before, config))})
    if backend is None:
        result = {"success": before.hard_kinematic_valid, "points": points if before.hard_kinematic_valid else [], "before": before, "after": before if before.hard_kinematic_valid else None, "windows": [], "hybrid_calls": 0, "hybrid_success": 0, "failure_reason": "" if before.hard_kinematic_valid else "KINEMATIC_REPAIR_FAILED", "planning_time_ms": 0.0, "cpu_total_ms": 0.0, "rss_peak_bytes": None, "pss_peak_bytes": None, "used_windows": []}
    else:
        result = repair_with_backend(points, hospital_map, config, backend, [], str(row.get("grid_mode", "")))
    after = result.get("after")
    base.update({"hybrid_calls": result.get("hybrid_calls", 0), "hybrid_success": result.get("hybrid_success", 0), "hybrid_failure_reason": result.get("hybrid_failure_reason", ""), "repair_window_count": len(result.get("windows", [])), "l3_planning_time_ms": result.get("planning_time_ms", 0.0), "l3_cpu_total_ms": result.get("cpu_total_ms", 0.0), "l3_rss_peak_bytes": result.get("rss_peak_bytes"), "l3_pss_peak_bytes": result.get("pss_peak_bytes"), "failure_code": result.get("failure_reason", ""), "hard_kinematic_valid": bool(after and after.hard_kinematic_valid), "turning_radius_preference_satisfied": bool(after and after.turning_radius_preference_satisfied), "static_footprint_valid": bool(after and after.static_collision_count == 0), "final_valid_success": bool(result.get("success", False)), "rotate_in_place_count": 0, "hybrid_attempts": json.dumps(result.get("attempts", []))})
    if result.get("used_windows"):
        base["repair_window_padding_m"] = max(item["padding_m"] for item in result["used_windows"])
        base["repair_length_m"] = sum(float(item["length_m"]) for item in result["used_windows"])
        base["repair_length_ratio"] = base["repair_length_m"] / max(1.0e-9, float(row.get("final_path_length_m", 0.0) or 0.0))
        base["stitch_position_error_m"] = max(float(item["stitch_position_error_m"]) for item in result["used_windows"])
        base["stitch_yaw_error_deg"] = max(float(item["stitch_yaw_error_deg"]) for item in result["used_windows"])
    if after:
        base.update(after.as_dict())
    final = result.get("points")
    if final:
        path_file = Path("paths") / f"{run_id}.json.gz"; _save_points(output / path_file, final); base["path_file"] = str(path_file); base["path_point_count"] = len(final)
        base["composed_online_time_ms"] = float(row.get("total_online_time_ms", 0.0) or 0.0) + float(base["l3_planning_time_ms"] or 0.0)
        repair_segments = [segment for item in result.get("used_windows", []) for segment in item.get("segments", [])]
        repair_length = sum(float(item.get("length_m", 0.0)) for item in repair_segments)
        total_length = sum(distance(a, b) for a, b in zip(final, final[1:]))
        base["segments"] = ([{"source": "grid", "planner": "grid_astar", "direction": "mixed", "length_m": max(0.0, total_length - repair_length), "grid_mode": str(row.get("grid_mode", "")), "repair_reason": "", "topology_edge_ids": []}] if total_length > repair_length else []) + repair_segments
        return base, final
    return base, None


def _reference_run(mode: str, source_row: pd.Series, points: Optional[Sequence[Point]], hospital_map: HospitalMap, config: HardRadiusConfig, output: Path, *, source_run_id: str, result_code: str = "SUCCEEDED", reference_only: bool = False) -> Dict[str, object]:
    run_id = f"{source_row.query_id}_{mode}_measured_{int(source_row.repetition)}_{time.time_ns()}"
    diagnostics = diagnose_hard_path(points or [], hospital_map, FOOTPRINT, config) if points else None
    q04 = str(source_row.query_id) == "q04"
    action_success = bool(points) and result_code == "SUCCEEDED"
    static_valid = bool(diagnostics and diagnostics.static_collision_count == 0)
    hard_valid = bool(diagnostics and diagnostics.hard_kinematic_valid)
    failure = Q04_FAILURE if q04 else ("" if action_success and static_valid and hard_valid else (diagnostics.failure_codes[0] if diagnostics and diagnostics.failure_codes else result_code))
    row = {
        "run_id": run_id, "query_id": source_row.query_id, "repetition": int(source_row.repetition), "mode": mode,
        "source_stage6_run_id": source_run_id, "reference_only": reference_only, "q04_diagnostic": Q04_FAILURE if q04 else "",
        "dynamic_obstacles": False, "allow_in_place_rotation": mode == "stage7_with_rotation",
        "minimum_turning_radius_m": 0.40, "maximum_curvature": 2.50,
        "action_success": action_success, "static_footprint_valid": static_valid,
        "hard_kinematic_valid": hard_valid, "turning_radius_preference_satisfied": bool(diagnostics and diagnostics.turning_radius_preference_satisfied),
        "final_valid_success": bool(action_success and static_valid and hard_valid and not q04),
        "failure_code": failure, "hybrid_calls": 0, "hybrid_success": 0,
        "rotate_in_place_count": diagnostics.zero_displacement_yaw_changes if diagnostics else 0,
        "static_footprint_collision_count": diagnostics.static_collision_count if diagnostics else 0,
        "hard_radius_violation_count": diagnostics.hard_radius_violation_count if diagnostics else None,
        "minimum_radius_observed_m": diagnostics.minimum_radius_m if diagnostics else None,
        "maximum_curvature_observed": diagnostics.maximum_curvature if diagnostics else None,
        "reverse_distance_m": diagnostics.direction_distance["reverse"] if diagnostics else None,
        "direction_switch_count": diagnostics.direction_switch_count if diagnostics else None,
        "l3_planning_time_ms": 0.0, "l3_cpu_total_ms": 0.0,
    }
    if points:
        path_file = Path("paths") / f"{run_id}.json.gz"; _save_points(output / path_file, points)
        row["path_file"] = str(path_file); row["path_point_count"] = len(points)
        source = "kinematic" if mode in {"stage7_with_rotation", "full_smac_normalized"} else "grid"
        planner = "stage7_explicit_rotation" if mode == "stage7_with_rotation" else ("smac_hybrid_reeds_shepp" if mode == "full_smac_normalized" else "grid_astar")
        row["segments"] = classify_segments(points, grid_mode=str(source_row.get("grid_mode", "reference")), topology_edge_ids=[], source=source, planner=planner)
    return row


def run_stage8a(stage6_path: Path, stage7_path: Path, smac_path: Path, map_path: Path, protocol_path: Path, output_path: Path, query_ids: Optional[Sequence[str]], repetitions: int, timeout: float, validate_only: bool = False, use_hybrid: bool = True) -> Path:
    protocol_file, protocol = load_protocol(protocol_path)
    if bool(protocol.get("dynamic_obstacles", False)):
        raise ValueError("dynamic_obstacles must be false")
    hospital_map = HospitalMap.load(map_path)
    frame = _load_stage6(stage6_path)
    if query_ids:
        frame = frame[frame.query_id.isin(set(query_ids))]
    if repetitions != 5:
        frame = frame[frame.repetition <= repetitions]
    output_path = output_path.resolve()
    if output_path.exists() and any(output_path.iterdir()):
        raise ValueError(f"refusing to overwrite existing Stage 8 output: {output_path}")
    output_path.mkdir(parents=True, exist_ok=True)
    config = _stage8_config()
    payload = _protocol_payload(protocol, map_path, config); payload["protocol_file"] = str(protocol_file)
    (output_path / "protocol.yaml").write_text(yaml.safe_dump(payload, sort_keys=False))
    (output_path / "manifest.yaml").write_text(yaml.safe_dump({"schema_version": 8, "experiment": "hospital_stage8a_hard_radius_l3", "dynamic_obstacles": False, "source_stage6": str(stage6_path), "queries": str(payload.get("queries_file", "")), "modes": ["grid_raw", "stage7_with_rotation", "layered_hard_radius_l3", "full_smac_normalized"]}, sort_keys=False))
    if validate_only:
        _write_rows(output_path / "kinematic_validation.csv", [{"query_id": q, "validation_status": "VALID" if q != "q04" else "INVALID_STATIC_SEMANTICS", "q04_diagnostic": Q04_FAILURE if q == "q04" else ""} for q in sorted(frame.query_id.unique())]); return output_path
    backend: Optional[StaticSmacBackend] = None
    stage7_runs = pd.read_csv(stage7_path / "kinematic_runs.csv")
    stage7_runs = stage7_runs[stage7_runs["mode"].eq("grid_with_rotation")]
    smac_runs = pd.read_csv(smac_path / "planner_runs.csv")
    smac_runs = smac_runs[smac_runs["run_mode"].eq("measured")]
    runs: List[Dict[str, object]] = []
    try:
        if use_hybrid:
            import rclpy
            rclpy.init(); backend = StaticSmacBackend(map_path, payload, output_path, timeout); backend.start()
        for _, row in frame.iterrows():
            raw_points = _path_from_row(stage6_path, row)
            runs.append(_reference_run("grid_raw", row, raw_points, hospital_map, config, output_path, source_run_id=str(row.run_id)))
            old = stage7_runs[(stage7_runs.query_id == row.query_id) & (stage7_runs.repetition == row.repetition)]
            old_row = old.iloc[0] if not old.empty else row
            old_points = _path_from_row(stage7_path, old_row) if not old.empty else None
            old_reference = _reference_run("stage7_with_rotation", row, old_points, hospital_map, config, output_path, source_run_id=str(old_row.get("run_id", "")), reference_only=True)
            old_reference["rotate_in_place_count"] = int(float(old_row.get("rotate_in_place_count", old_reference["rotate_in_place_count"]) or 0))
            runs.append(old_reference)
            run, points = _run_row(row, stage6_path, hospital_map, config, backend, output_path)
            runs.append(run)
            reference = smac_runs[(smac_runs.query_id == row.query_id) & (smac_runs.repetition == row.repetition)]
            ref_row = reference.iloc[0] if not reference.empty else row
            ref_points = _path_from_row(smac_path, ref_row) if not reference.empty else None
            ref_code = str(ref_row.get("result_code", "EMPTY_PATH"))
            runs.append(_reference_run("full_smac_normalized", row, ref_points, hospital_map, config, output_path, source_run_id=str(ref_row.get("run_id", "")), result_code=ref_code, reference_only=True))
    finally:
        if backend is not None:
            backend.stop()
            try:
                import rclpy
                rclpy.shutdown()
            except Exception:
                pass
    validation = [{"query_id": q, "validation_status": "VALID" if q != "q04" else "INVALID_STATIC_SEMANTICS", "q04_diagnostic": Q04_FAILURE if q == "q04" else ""} for q in sorted(frame.query_id.unique())]
    _write_rows(output_path / "kinematic_validation.csv", validation)
    _write_rows(output_path / "failure_summary.csv", [{"failure_code": code, "count": int(sum(str(r.get("failure_code", "")) == code for r in runs))} for code in sorted(set(str(r.get("failure_code", "")) for r in runs if r.get("failure_code")))])
    segment_rows = []
    for run in runs:
        for index, segment in enumerate(run.pop("segments", [])):
            segment_rows.append({"run_id": run["run_id"], "query_id": run["query_id"], "mode": run["mode"], "segment_index": index, **segment})
    _write_rows(output_path / "kinematic_runs.csv", runs)
    _write_rows(output_path / "segment_metrics.csv", segment_rows)
    _write_rows(output_path / "path_metrics.csv", [{key: row.get(key) for key in ("run_id", "query_id", "repetition", "mode", "action_success", "static_footprint_valid", "hard_kinematic_valid", "final_valid_success", "static_footprint_collision_count", "hard_radius_violation_count", "minimum_radius_observed_m", "maximum_curvature_observed", "reverse_distance_m", "direction_switch_count", "l3_planning_time_ms", "l3_cpu_total_ms")} for row in runs])
    measured = pd.DataFrame(runs)
    if not measured.empty:
        summary = []
        for mode, group in measured.groupby("mode"):
            valid = group[group["final_valid_success"].astype(bool)]
            reachable = valid[valid.query_id != "q04"]
            reachable_total = group[group.query_id != "q04"]
            summary.append({"mode": mode, "count": len(group), "success_count": len(valid), "all_query_success_rate": len(valid) / max(1, len(group)), "reachable_query_success_rate": len(reachable) / max(1, len(reachable_total)), "hybrid_calls": int(group.hybrid_calls.sum()), "hybrid_success": int(group.hybrid_success.sum()), "rotate_in_place_count": int(group.rotate_in_place_count.sum())})
        _write_rows(output_path / "summary_by_mode.csv", summary)
        by_query = []
        for (mode, query_id), group in measured.groupby(["mode", "query_id"]):
            by_query.append({"mode": mode, "query_id": query_id, "count": len(group), "success_count": int(group.final_valid_success.astype(bool).sum()), "success_rate": float(group.final_valid_success.astype(bool).mean()), "action_success_count": int(group.action_success.astype(bool).sum()), "static_valid_count": int(group.static_footprint_valid.astype(bool).sum()), "hard_kinematic_valid_count": int(group.hard_kinematic_valid.astype(bool).sum()), "hybrid_calls": int(group.hybrid_calls.sum()), "hybrid_success": int(group.hybrid_success.sum())})
        _write_rows(output_path / "summary_by_query.csv", by_query)
        candidate = measured[measured["mode"].eq("layered_hard_radius_l3")]
        successful = candidate[candidate.final_valid_success.astype(bool)]
        _write_rows(output_path / "stage8_acceptance_summary.csv", [{
            "candidate_count": len(candidate), "candidate_success_count": len(successful),
            "all_query_success_rate": len(successful) / max(1, len(candidate)),
            "reachable_query_success_rate": len(successful[successful.query_id != "q04"]) / max(1, len(candidate[candidate.query_id != "q04"])),
            "successful_static_collision_count": int(pd.to_numeric(successful.get("static_footprint_collision_count", 0), errors="coerce").fillna(0).sum()),
            "successful_rotate_in_place_count": int(pd.to_numeric(successful.get("rotate_in_place_count", 0), errors="coerce").fillna(0).sum()),
            "successful_hard_radius_violation_count": int(pd.to_numeric(successful.get("hard_radius_violation_count", 0), errors="coerce").fillna(0).sum()),
            "hybrid_calls": int(candidate.hybrid_calls.sum()), "hybrid_success": int(candidate.hybrid_success.sum()),
            "q04_failure_code": Q04_FAILURE,
            "action_success_static_invalid_count": int((candidate.action_success.astype(bool) & ~candidate.static_footprint_valid.astype(bool)).sum()),
        }])
    return output_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stage 8 static hard-radius L3 benchmark")
    parser.add_argument("--stage6", default=str(DEFAULT_STAGE6)); parser.add_argument("--stage7", default=str(DEFAULT_STAGE7)); parser.add_argument("--smac-reference", default=str(DEFAULT_SMAC)); parser.add_argument("--map", dest="map_path", default=str(DEFAULT_MAP)); parser.add_argument("--protocol", default=str(DEFAULT_QUERIES.parent / "topology_protocol_v2.yaml")); parser.add_argument("--queries", default=str(DEFAULT_QUERIES)); parser.add_argument("--output-dir", required=True); parser.add_argument("--query-id", action="append"); parser.add_argument("--repetitions", type=int, default=5); parser.add_argument("--timeout", type=float, default=5.0); parser.add_argument("--validate-only", action="store_true"); parser.add_argument("--no-hybrid", action="store_true", help="offline validation only; never claims a repair"); parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output = run_stage8a(Path(args.stage6), Path(args.stage7), Path(args.smac_reference), Path(args.map_path), Path(args.protocol), Path(args.output_dir), args.query_id, args.repetitions, args.timeout, args.validate_only, not args.no_hybrid)
        print(f"stage8a output: {output}"); return 0
    except Exception as exc:
        print(f"stage8a failed: {exc}", file=sys.stderr); return 2


if __name__ == "__main__":
    raise SystemExit(main())
