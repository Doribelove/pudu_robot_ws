"""Stage 7 static validator and on-demand L3 benchmark.

The command consumes frozen Stage 6 grid paths and frozen Stage 5 Smac result
files.  It never starts ROS; a local Smac adapter is deliberately optional so
an unsafe repair is reported instead of being silently replaced.
"""

from __future__ import annotations

import argparse
import datetime as dt
import gzip
import hashlib
import math
import os
import resource
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

import numpy as np
import pandas as pd
import yaml

from .kinematic import (
    KinematicConfig,
    build_segments,
    diagnose_path,
    insert_safe_rotations,
    repair_path,
)
from .planner_benchmark.config import load_queries
from .planner_benchmark.map_utils import HospitalMap, sha256_file
from .planner_benchmark.resources import read_snapshot
from .topology import map_input_hash, static_collision_count


DEFAULT_STAGE6 = Path("experiments/layered_planner_benchmark/hospital_005/stage6_l1_l2")
DEFAULT_MAP = Path("experiments/maps/hospital_005/map.yaml")
DEFAULT_QUERIES = Path("experiments/planner_benchmark/hospital_005/queries_v2.yaml")
DEFAULT_SMAC = Path("experiments/planner_benchmark/hospital_005/stage5_smac_normalized")
FOOTPRINT = [[0.255, 0.215], [0.255, -0.215], [-0.255, -0.215], [-0.255, 0.215]]
MODES = ["grid_raw", "grid_with_rotation", "layered_on_demand_l3", "full_smac_normalized"]
STATIC_FAILURE = "STATIC_SEMANTICS_CONSERVATIVE_INFLATION_MISMATCH"


def _sha256_payload(value: object) -> str:
    return hashlib.sha256(yaml.safe_dump(value, sort_keys=True).encode()).hexdigest()


def _load_points(path: Path) -> List[Dict[str, float]]:
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) if path.suffix == ".yaml" else None
    if payload is not None:
        return payload
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        import json
        return [
            {"x": float(point["x"]), "y": float(point["y"]), "yaw": float(point["yaw"])}
            for point in json.load(stream)
        ]


def _save_points(path: Path, points: Sequence[Dict[str, float]]) -> None:
    import json
    path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(path, "wt", encoding="utf-8") as stream:
        json.dump(list(points), stream)


def _protocol(config: KinematicConfig, hospital_map: HospitalMap) -> Dict[str, object]:
    return {
        "schema_version": 1,
        "experiment": "hospital_stage7_on_demand_l3",
        "map_id": hospital_map.map_id,
        "map_yaml": str(hospital_map.yaml_path),
        "map_sha256": map_input_hash(hospital_map.yaml_path, hospital_map.image_path),
        "resolution": hospital_map.resolution,
        "origin": list(hospital_map.origin),
        "dynamic_obstacles": False,
        "footprint": FOOTPRINT,
        "allow_in_place_rotation": config.allow_in_place_rotation,
        "allow_reverse": config.allow_reverse,
        "reverse_penalty": config.reverse_penalty,
        "preferred_minimum_turning_radius": config.preferred_minimum_turning_radius,
        "heading_jump_trigger_deg": config.heading_jump_trigger_deg,
        "rotation_collision_sample_deg": config.rotation_collision_sample_deg,
        "stitch_position_tolerance_m": config.stitch_position_tolerance_m,
        "stitch_yaw_tolerance_deg": config.stitch_yaw_tolerance_deg,
        "initial_repair_window_m": config.initial_repair_window_m,
        "expanded_repair_window_m": config.expanded_repair_window_m,
        "motion_model": config.motion_model,
    }


def _read_stage6(stage6_dir: Path) -> pd.DataFrame:
    manifest = yaml.safe_load((stage6_dir / "manifest.yaml").read_text()) or {}
    if bool(manifest.get("dynamic_obstacles", False)):
        raise ValueError("Stage 6 input has dynamic_obstacles=true")
    frame = pd.read_csv(stage6_dir / "query_runs.csv")
    frame = frame[frame["mode"].eq("topology_guided_grid_fallback")].copy()
    if frame.empty:
        raise ValueError("Stage 6 fallback-mode measured input is empty")
    return frame


def _read_smac_reference(directory: Path) -> tuple[pd.DataFrame, pd.DataFrame]:
    runs = pd.read_csv(directory / "planner_runs.csv")
    metrics = pd.read_csv(directory / "path_metrics.csv")
    runs = runs[runs["run_mode"].eq("measured")].copy()
    return runs, metrics


def _resource_before() -> object:
    return read_snapshot(os.getpid())


def _resource_after(before: object, started_ns: int, *, planning_ms: float) -> Dict[str, object]:
    after = read_snapshot(os.getpid())
    elapsed = (time.monotonic_ns() - started_ns) / 1e6
    if before is None or after is None:
        return {
            "l3_cpu_user_ms": None, "l3_cpu_system_ms": None, "l3_cpu_total_ms": None,
            "l3_cpu_percent": None, "l3_rss_before_bytes": None, "l3_rss_peak_bytes": None,
            "l3_pss_before_bytes": None, "l3_pss_peak_bytes": None,
            "l3_resource_error": "local /proc snapshot unavailable", "l3_wall_time_ms": elapsed,
        }
    user = max(0.0, after.cpu_user_ms - before.cpu_user_ms)
    system = max(0.0, after.cpu_system_ms - before.cpu_system_ms)
    return {
        "l3_cpu_user_ms": user, "l3_cpu_system_ms": system, "l3_cpu_total_ms": user + system,
        "l3_cpu_percent": (user + system) / planning_ms * 100.0 if planning_ms > 0 else None,
        "l3_rss_before_bytes": before.rss_bytes, "l3_rss_peak_bytes": max(before.rss_bytes or 0, after.rss_bytes or 0) or None,
        "l3_pss_before_bytes": before.pss_bytes, "l3_pss_peak_bytes": max(before.pss_bytes or 0, after.pss_bytes or 0) or None,
        "l3_resource_error": "", "l3_wall_time_ms": elapsed,
    }


def _path_length(points: Sequence[Dict[str, float]]) -> float:
    return sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(points, points[1:]))


def _path_row(
    *,
    run_id: str,
    source_row: pd.Series,
    mode: str,
    points: Sequence[Dict[str, float]],
    hospital_map: HospitalMap,
    config: KinematicConfig,
    topology_edges: Sequence[int],
    stage6_path_exists: bool,
    result_code: str = "SUCCEEDED",
    l3_applicable: bool = True,
    reference_only: bool = False,
    stage6_online_ms: Optional[float] = None,
    l3_data: Optional[Dict[str, object]] = None,
) -> tuple[Dict[str, object], Dict[str, object], List[Dict[str, object]]]:
    l3_data = l3_data or {}
    diagnostics = diagnose_path(points, hospital_map, FOOTPRINT, config) if points else None
    pre_diagnostics = l3_data.get("diagnostics") or diagnostics
    static_collision = diagnostics.static_collision_count if diagnostics else 0
    hard_valid = bool(diagnostics and diagnostics.hard_kinematic_valid)
    static_valid = bool(points and static_collision == 0)
    final_valid = bool(stage6_path_exists and static_valid and (hard_valid if mode != "full_smac_normalized" else static_valid))
    if str(source_row["query_id"]) == "q04" and mode == "full_smac_normalized":
        # The frozen Smac reference may contain an action result, but q04 has
        # no Stage 6 geometry and is excluded from the Stage 7 L3 success
        # denominator by protocol.
        final_valid = False
        result_code = STATIC_FAILURE
    if result_code != "SUCCEEDED":
        final_valid = False
    segments = build_segments(
        points,
        grid_mode=str(source_row.get("grid_mode", "reference")),
        topology_edge_ids=topology_edges,
        rotation_centers=[item.get("center_index", -1) for item in l3_data.get("rotation_segments", [])],
    ) if points else []
    if reference_only:
        for segment in segments:
            segment["source"] = "smac_reference"
            segment["planner"] = "smac_hybrid_reeds_shepp"
    rotation_segments = l3_data.get("rotation_segments", [])
    rotation_angle = sum(abs(float(item.get("rotation_angle_rad", 0.0))) for item in rotation_segments)
    original_length = float(source_row.get("final_path_length_m", 0.0) or 0.0)
    final_length = _path_length(points) if points else None
    l3_planning_ms = float(l3_data.get("l3_planning_time_ms", 0.0) or 0.0)
    run = {
        "run_id": run_id, "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(),
        "query_id": source_row["query_id"], "repetition": int(source_row["repetition"]),
        "mode": mode, "source_stage6_run_id": source_row["run_id"],
        "result_code": result_code, "stage6_result_code": source_row.get("result_code", ""),
        "q04_diagnostic": STATIC_FAILURE if str(source_row["query_id"]) == "q04" else "",
        "reference_only": reference_only, "l3_applicable": l3_applicable,
        "action_success": bool(stage6_path_exists if mode != "full_smac_normalized" else points),
        "static_footprint_valid": static_valid, "final_valid_success": final_valid,
        "hard_kinematic_valid": hard_valid if mode != "full_smac_normalized" else None,
        "turning_radius_preference_satisfied": diagnostics.turning_radius_preference_satisfied if diagnostics else None,
        "static_footprint_collision_count": static_collision,
        "heading_jump_count_before": pre_diagnostics.heading_jump_count if pre_diagnostics else 0,
        "heading_jump_max_rad_before": pre_diagnostics.heading_jump_max_rad if pre_diagnostics else None,
        "curvature_preference_violation_count_before": pre_diagnostics.curvature_preference_violation_count if pre_diagnostics else None,
        "curvature_preference_violation_count_after": l3_data.get("post_curvature_preference_violation_count", diagnostics.curvature_preference_violation_count if diagnostics else None),
        "l3_triggered": bool(l3_data.get("l3_triggered", False)),
        "trigger_count": len(pre_diagnostics.trigger_indices) if pre_diagnostics else 0,
        "repair_window_count": int(l3_data.get("repair_window_count", len(pre_diagnostics.repair_windows) if pre_diagnostics else 0)),
        "rotate_in_place_count": len(rotation_segments),
        "rotation_angle_total_rad": rotation_angle,
        "rotation_sweep_collision_count": pre_diagnostics.rotation_collision_count if pre_diagnostics else None,
        "hybrid_calls": int(l3_data.get("hybrid_calls", 0)),
        "hybrid_success": bool(l3_data.get("hybrid_success", False)),
        "hybrid_failure_reason": l3_data.get("hybrid_failure_reason", ""),
        "repair_window_padding_m": l3_data.get("repair_window_padding_m"),
        "repair_length_m": max(0.0, (final_length or 0.0) - original_length) if final_length is not None else None,
        "repair_length_ratio": (max(0.0, (final_length or 0.0) - original_length) / original_length) if original_length else None,
        "reverse_distance_m": diagnostics.direction_distance["reverse"] if diagnostics else None,
        "reverse_ratio": (diagnostics.direction_distance["reverse"] / max(1e-9, sum(diagnostics.direction_distance.values())) if diagnostics else None),
        "direction_switch_count": diagnostics.direction_switch_count if diagnostics else None,
        "stitch_position_error_m": float(l3_data.get("stitch_position_error_m", 0.0)),
        "stitch_yaw_error_deg": float(l3_data.get("stitch_yaw_error_deg", 0.0)),
        "stitch_valid": bool(l3_data.get("stitch_valid", True)),
        "stage6_online_time_ms": stage6_online_ms,
        "l3_planning_time_ms": l3_planning_ms,
        "composed_online_time_ms": (float(stage6_online_ms) + l3_planning_ms) if stage6_online_ms is not None else None,
        "composed_online_time_is_estimate": True,
        "final_path_length_m": final_length,
        "minimum_clearance_m": float(source_row.get("minimum_clearance_m", 0.0)) if points else None,
        "path_point_count": len(points),
        "path_file": "",
        "grid_mode": source_row.get("grid_mode", "reference"),
        "topology_edge_ids": list(topology_edges),
        "topology_guided_source": "grid" if mode != "full_smac_normalized" else "smac_reference",
    }
    run.update({key: value for key, value in l3_data.items() if key.startswith("l3_") and key not in {"l3_planning_time_ms"}})
    total_length = sum(float(segment.get("length_m", 0.0)) for segment in segments)
    segment_rows = []
    for index, segment in enumerate(segments):
        segment_rows.append({
            "run_id": run_id, "segment_index": index, "source": segment.get("source"),
            "direction": segment.get("direction"), "planner": segment.get("planner"),
            "repair_reason": segment.get("repair_reason", ""),
            "length_m": segment.get("length_m", 0.0),
            "length_ratio": (float(segment.get("length_m", 0.0)) / total_length if total_length else 0.0),
            "rotation_angle_rad": segment.get("rotation_angle_rad", 0.0),
            "grid_mode": segment.get("grid_mode", run["grid_mode"]),
            "topology_edge_ids": segment.get("topology_edge_ids", list(topology_edges)),
        })
    metrics = {
        "run_id": run_id, "query_id": source_row["query_id"], "repetition": int(source_row["repetition"]),
        "mode": mode, "final_path_length_m": final_length,
        "length_over_stage6_grid": final_length / original_length if final_length is not None and original_length else None,
        "minimum_clearance_m": run["minimum_clearance_m"],
        "static_footprint_collision_count": static_collision,
        "hard_kinematic_valid": hard_valid if mode != "full_smac_normalized" else None,
        "turning_radius_preference_satisfied": run["turning_radius_preference_satisfied"],
        "repair_length_m": run["repair_length_m"], "repair_length_ratio": run["repair_length_ratio"],
        "reverse_distance_m": run["reverse_distance_m"], "reverse_ratio": run["reverse_ratio"],
        "direction_switch_count": run["direction_switch_count"],
        "l3_planning_time_ms": l3_planning_ms, "composed_online_time_ms": run["composed_online_time_ms"],
        "composed_online_time_is_estimate": True, "action_success": run["action_success"],
        "static_footprint_valid": static_valid, "final_valid_success": final_valid,
    }
    return run, metrics, segment_rows


def _summary(output: Path, runs: List[Dict[str, object]], metrics: List[Dict[str, object]], segments: List[Dict[str, object]]) -> None:
    frame = pd.DataFrame(runs)
    metric_frame = pd.DataFrame(metrics)
    pd.DataFrame(runs).to_csv(output / "kinematic_runs.csv", index=False)
    metric_frame.to_csv(output / "path_metrics.csv", index=False)
    pd.DataFrame(segments).to_csv(output / "segment_metrics.csv", index=False)
    failure = frame[~frame["final_valid_success"].astype(bool)]
    if failure.empty:
        pd.DataFrame(columns=["mode", "result_code", "count"]).to_csv(output / "failure_summary.csv", index=False)
    else:
        failure.groupby(["mode", "result_code"], dropna=False).size().rename("count").reset_index().to_csv(output / "failure_summary.csv", index=False)
    summary_rows = []
    for mode, group in frame.groupby("mode"):
        valid = group["final_valid_success"].astype(bool)
        all_queries = int(group["query_id"].nunique())
        reachable = set(frame.loc[frame["query_id"].ne("q04") & frame["mode"].ne("full_smac_normalized"), "query_id"].astype(str))
        for field in ["l3_planning_time_ms", "composed_online_time_ms", "final_path_length_m", "reverse_ratio", "repair_length_m", "l3_rss_peak_bytes", "l3_pss_peak_bytes"]:
            values = pd.to_numeric(group.loc[valid, field], errors="coerce").dropna()
            summary_rows.append({
                "mode": mode, "metric": field, "count": len(values),
                "success_count": int(valid.sum()), "all_query_count": all_queries,
                "action_success_rate": float(group["action_success"].astype(bool).mean()),
                "static_footprint_valid_rate": float(group["static_footprint_valid"].astype(bool).mean()),
                "final_valid_success_rate": float(valid.mean()),
                "all_query_success_rate": float(group.loc[valid, "query_id"].astype(str).nunique() / all_queries) if all_queries else None,
                "reachable_query_count": len(reachable),
                "reachable_query_success_rate": float(group.loc[valid & group["query_id"].astype(str).isin(reachable), "query_id"].astype(str).nunique() / len(reachable)) if reachable else None,
                "P50": float(values.quantile(0.5)) if len(values) else None,
                "P95": float(values.quantile(0.95)) if len(values) else None,
                "P99": float(values.quantile(0.99)) if len(values) else None,
                "mean": float(values.mean()) if len(values) else None,
                "std": float(values.std(ddof=1)) if len(values) > 1 else (0.0 if len(values) else None),
            })
    pd.DataFrame(summary_rows).to_csv(output / "summary_by_mode.csv", index=False)
    query_rows = []
    for (query_id, mode), group in frame.groupby(["query_id", "mode"]):
        valid = group["final_valid_success"].astype(bool)
        query_rows.append({
            "query_id": query_id, "mode": mode, "count": len(group),
            "success_count": int(valid.sum()), "query_success": bool(valid.all()),
            "triggered_count": int(group["l3_triggered"].astype(bool).sum()),
            "rotation_count": int(group["rotate_in_place_count"].sum()),
            "hybrid_calls": int(group["hybrid_calls"].sum()),
            "mean_l3_planning_time_ms": float(pd.to_numeric(group["l3_planning_time_ms"], errors="coerce").mean()),
            "mean_composed_online_time_ms": float(pd.to_numeric(group["composed_online_time_ms"], errors="coerce").mean()),
        })
    pd.DataFrame(query_rows).to_csv(output / "summary_by_query.csv", index=False)
    acceptance = []
    for mode, group in frame.groupby("mode"):
        valid = group["final_valid_success"].astype(bool)
        reachable = set(group.loc[group["query_id"].ne("q04"), "query_id"].astype(str))
        acceptance.append({
            "mode": mode, "query_count": int(group["query_id"].nunique()),
            "action_success_rate": float(group["action_success"].astype(bool).mean()),
            "static_footprint_valid_rate": float(group["static_footprint_valid"].astype(bool).mean()),
            "final_valid_success_rate": float(valid.mean()),
            "all_query_success_rate": float(group.loc[valid, "query_id"].astype(str).nunique() / group["query_id"].nunique()),
            "reachable_query_count": len(reachable),
            "reachable_query_success_rate": float(group.loc[valid & group["query_id"].astype(str).isin(reachable), "query_id"].astype(str).nunique() / len(reachable)) if reachable else None,
            "l3_triggered_paths": int(group["l3_triggered"].astype(bool).groupby(group["query_id"]).any().sum()),
            "rotation_segments": int(group["rotate_in_place_count"].sum()),
            "hybrid_calls": int(group["hybrid_calls"].sum()),
            "repair_success_rate": float(group.loc[group["l3_triggered"].astype(bool), "final_valid_success"].mean()) if group["l3_triggered"].astype(bool).any() else None,
            "static_footprint_collision_max": int(pd.to_numeric(group["static_footprint_collision_count"], errors="coerce").fillna(0).max()),
        })
    pd.DataFrame(acceptance).to_csv(output / "stage7_acceptance_summary.csv", index=False)
    # Paired reference comparison is restricted to rows that are statically
    # valid in both modes.  Composed time remains explicitly an estimate.
    comparison_rows = []
    smac = frame[frame["mode"].eq("full_smac_normalized")].set_index(["query_id", "repetition"])
    for mode in ("grid_raw", "grid_with_rotation", "layered_on_demand_l3"):
        current = frame[frame["mode"].eq(mode)].set_index(["query_id", "repetition"])
        joined = current.join(smac, lsuffix="", rsuffix="_smac", how="inner")
        valid_pair = joined["final_valid_success"] & joined["final_valid_success_smac"]
        if not valid_pair.any():
            continue
        time_ratio = joined.loc[valid_pair, "composed_online_time_ms"] / joined.loc[valid_pair, "composed_online_time_ms_smac"]
        length_ratio = joined.loc[valid_pair, "final_path_length_m"] / joined.loc[valid_pair, "final_path_length_m_smac"]
        comparison_rows.append({
            "mode": mode, "paired_valid_count": int(valid_pair.sum()),
            "composed_time_ratio_vs_full_smac_mean": float(time_ratio.mean()),
            "composed_time_ratio_vs_full_smac_P95": float(time_ratio.quantile(0.95)),
            "path_length_ratio_vs_full_smac_mean": float(length_ratio.mean()),
            "path_length_ratio_vs_full_smac_P95": float(length_ratio.quantile(0.95)),
            "comparison_time_is_composed_estimate": True,
        })
    pd.DataFrame(comparison_rows).to_csv(output / "stage7_comparison.csv", index=False)
    _plots(output / "plots", frame)


def _plots(directory: Path, frame: pd.DataFrame) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    for field, filename, title, ylabel in [
        ("l3_planning_time_ms", "l3_time_by_mode.png", "L3 validation/repair time", "ms"),
        ("composed_online_time_ms", "composed_time_by_mode.png", "Composed online estimate", "ms"),
        ("final_path_length_m", "l3_path_length_by_mode.png", "Final path length", "m"),
        ("repair_length_ratio", "repair_length_ratio.png", "Repair length ratio", "ratio"),
    ]:
        groups, labels = [], []
        for mode, group in frame.groupby("mode"):
            values = pd.to_numeric(group.loc[group["final_valid_success"].astype(bool), field], errors="coerce").dropna()
            if len(values):
                groups.append(values.to_numpy()); labels.append(mode)
        if not groups:
            continue
        fig, axis = plt.subplots(figsize=(10, 5))
        axis.boxplot(groups, tick_labels=labels)
        axis.tick_params(axis="x", rotation=25)
        axis.set_title(title); axis.set_ylabel(ylabel); axis.grid(True, alpha=0.25)
        fig.tight_layout(); fig.savefig(directory / filename, dpi=140); plt.close(fig)


def run_stage7(
    *, stage6_dir: Path, map_yaml: Path, queries_yaml: Path, smac_dir: Path,
    output_dir: Path, modes: Sequence[str], query_ids: Optional[Sequence[str]],
    max_repetitions: Optional[int], validate_only: bool,
) -> Path:
    output_dir = output_dir.resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError(f"refusing to overwrite existing Stage 7 output: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    hospital_map = HospitalMap.load(map_yaml)
    config = KinematicConfig()
    stage6 = _read_stage6(stage6_dir)
    if query_ids:
        stage6 = stage6[stage6["query_id"].astype(str).isin(set(query_ids))]
    if max_repetitions is not None:
        stage6 = stage6[stage6["repetition"].astype(int) <= int(max_repetitions)]
    if stage6.empty:
        raise ValueError("no Stage 6 fallback rows selected")
    _, queries = load_queries(queries_yaml)
    query_by_id = {query.query_id: query for query in queries}
    smac_runs, smac_metrics = _read_smac_reference(smac_dir)
    protocol = _protocol(config, hospital_map)
    (output_dir / "protocol.yaml").write_text(yaml.safe_dump(protocol, sort_keys=False))
    (output_dir / "manifest.yaml").write_text(yaml.safe_dump({
        "schema_version": 1, "experiment": "hospital_stage7_on_demand_l3",
        "stage6_input": str(stage6_dir.resolve()), "smac_reference": str(smac_dir.resolve()),
        "map_yaml": str(map_yaml.resolve()), "queries_yaml": str(queries_yaml.resolve()),
        "map_sha256": protocol["map_sha256"], "dynamic_obstacles": False,
        "modes": list(modes), "input_rows": len(stage6), "reference_is_frozen": True,
    }, sort_keys=False))
    validation_rows: List[Dict[str, object]] = []
    runs: List[Dict[str, object]] = []
    metrics: List[Dict[str, object]] = []
    segments: List[Dict[str, object]] = []
    stage6_root = stage6_dir.resolve()
    for _, source in stage6.iterrows():
        query = query_by_id.get(str(source["query_id"]))
        if query is None:
            raise ValueError(f"query is missing from fixed query set: {source['query_id']}")
        path_exists = bool(str(source.get("path_file", ""))) and str(source.get("path_file", "")) != "nan"
        points = _load_points(stage6_root / str(source["path_file"])) if path_exists else []
        pre = diagnose_path(points, hospital_map, FOOTPRINT, config) if points else None
        validation = {
            "query_id": query.query_id, "repetition": int(source["repetition"]),
            "stage6_run_id": source["run_id"], "stage6_result_code": source.get("result_code", ""),
            "validation_status": "VALID" if points and bool(source.get("static_footprint_valid", False)) else str(source.get("result_code", "INVALID")),
            "l3_applicable": bool(points and query.query_id != "q04"),
            "heading_jump_count": pre.heading_jump_count if pre else 0,
            "trigger_count": len(pre.trigger_indices) if pre else 0,
            "repair_window_count": len(pre.repair_windows) if pre else 0,
            "rotation_collision_count": pre.rotation_collision_count if pre else 0,
            "static_collision_count": pre.static_collision_count if pre else 0,
            "turning_radius_preference_satisfied": pre.turning_radius_preference_satisfied if pre else None,
            "hard_kinematic_valid_raw": pre.hard_kinematic_valid if pre else False,
            "diagnostic_code": STATIC_FAILURE if query.query_id == "q04" else "",
        }
        validation_rows.append(validation)
        if validate_only:
            continue
        topology_edges = []
        try:
            topology_edges = yaml.safe_load(str(source.get("topology_edge_ids", "[]"))) or []
        except Exception:
            topology_edges = []
        for mode in modes:
            run_id = f"{query.query_id}_{mode}_measured_{int(source['repetition'])}_{time.time_ns()}"
            if mode == "full_smac_normalized":
                ref = smac_runs[(smac_runs["query_id"].astype(str) == query.query_id) & (smac_runs["repetition"].astype(int) == int(source["repetition"]))]
                if ref.empty:
                    ref = smac_runs[smac_runs["query_id"].astype(str) == query.query_id].head(1)
                refrow = ref.iloc[0] if not ref.empty else None
                refmetric = smac_metrics[smac_metrics["run_id"].astype(str) == str(refrow["run_id"])] if refrow is not None else pd.DataFrame()
                ref_points = _load_points(smac_dir / str(refrow["path_file"])) if refrow is not None and str(refrow["path_file"]) != "nan" else []
                row, metric, segment = _path_row(
                    run_id=run_id, source_row=source, mode=mode, points=ref_points,
                    hospital_map=hospital_map, config=config, topology_edges=[],
                    stage6_path_exists=bool(refrow is not None), result_code=(STATIC_FAILURE if query.query_id == "q04" else ("SUCCEEDED" if refrow is not None else "REFERENCE_MISSING")),
                    l3_applicable=False, reference_only=True, stage6_online_ms=float(refrow["wall_time_ms"]) if refrow is not None else None,
                    l3_data={"l3_planning_time_ms": 0.0},
                )
                if refrow is not None:
                    row["reference_planning_time_ms"] = float(refrow["planning_time_ms"])
                    row["reference_wall_time_ms"] = float(refrow["wall_time_ms"])
                    row["reference_cpu_total_ms"] = float(refrow["cpu_total_ms"])
                    row["reference_static_footprint_collision_count"] = int(refmetric.iloc[0]["footprint_collision_count"]) if not refmetric.empty else None
                runs.append(row); metrics.append(metric); segments.extend(segment)
                if ref_points:
                    rel = Path("paths") / f"{run_id}.json.gz"; _save_points(output_dir / rel, ref_points); row["path_file"] = str(rel)
                continue
            if query.query_id == "q04" or not points:
                row, metric, segment = _path_row(
                    run_id=run_id, source_row=source, mode=mode, points=[], hospital_map=hospital_map,
                    config=config, topology_edges=topology_edges, stage6_path_exists=False,
                    result_code=STATIC_FAILURE if query.query_id == "q04" else str(source.get("result_code", "EMPTY_PATH")),
                    l3_applicable=False, stage6_online_ms=float(source.get("total_online_time_ms", 0.0)),
                )
                runs.append(row); metrics.append(metric); segments.extend(segment); continue
            started = time.monotonic_ns(); before = _resource_before()
            if mode == "grid_raw":
                result = {"points": points, "success": True, "failure_code": "", "rotation_segments": [], "hybrid_calls": 0, "l3_triggered": bool(pre and pre.trigger_indices), "l3_planning_time_ms": 0.0, "diagnostics": pre}
            elif mode == "grid_with_rotation":
                repaired, rotations, status = insert_safe_rotations(pre, hospital_map, FOOTPRINT, config)
                result = {"points": repaired, "success": bool(status.get("success")), "failure_code": status.get("failure_code", ""), "rotation_segments": rotations, "hybrid_calls": 0, "l3_triggered": bool(pre.trigger_indices), "repair_window_count": len(pre.repair_windows), "repair_window_padding_m": config.initial_repair_window_m, "l3_planning_time_ms": 0.0, "diagnostics": pre}
                if not result["success"]: result["hybrid_failure_reason"] = status.get("hybrid_failure_reason", "")
            else:
                result = repair_path(points, hospital_map, FOOTPRINT, config)
                result["l3_triggered"] = bool(pre and pre.trigger_indices)
                result["l3_planning_time_ms"] = 0.0
            elapsed = (time.monotonic_ns() - started) / 1e6
            result["l3_planning_time_ms"] = elapsed
            result.update(_resource_after(before, started, planning_ms=elapsed))
            if result.get("success"):
                post = diagnose_path(result["points"], hospital_map, FOOTPRINT, config)
                result["post_curvature_preference_violation_count"] = post.curvature_preference_violation_count
            row, metric, segment = _path_row(
                run_id=run_id, source_row=source, mode=mode, points=result.get("points", []),
                hospital_map=hospital_map, config=config, topology_edges=topology_edges,
                stage6_path_exists=True, result_code="SUCCEEDED" if result.get("success") else result.get("failure_code", "KINEMATIC_REPAIR_FAILED"),
                l3_applicable=True, stage6_online_ms=float(source.get("total_online_time_ms", 0.0)), l3_data=result,
            )
            runs.append(row); metrics.append(metric); segments.extend(segment)
            if result.get("points"):
                rel = Path("paths") / f"{run_id}.json.gz"; _save_points(output_dir / rel, result["points"]); row["path_file"] = str(rel)
    pd.DataFrame(validation_rows).to_csv(output_dir / "kinematic_validation.csv", index=False)
    if validate_only:
        return output_dir
    _summary(output_dir, runs, metrics, segments)
    return output_dir


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate Stage 6 static paths and run on-demand L3 checks")
    parser.add_argument("--stage6-dir", default=str(DEFAULT_STAGE6))
    parser.add_argument("--map", dest="map_yaml", default=str(DEFAULT_MAP))
    parser.add_argument("--queries", default=str(DEFAULT_QUERIES))
    parser.add_argument("--smac-reference", default=str(DEFAULT_SMAC))
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--mode", choices=["all", *MODES], default="all")
    parser.add_argument("--query-id", action="append", dest="query_ids")
    parser.add_argument("--max-repetitions", type=int, default=None)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(list(sys.argv[1:] if argv is None else argv))
    if not args.no_dynamic_obstacles:
        raise ValueError("--no-dynamic-obstacles is required")
    modes = MODES if args.mode == "all" else [args.mode]
    query_ids = []
    for value in args.query_ids or []:
        query_ids.extend(part for part in value.split(",") if part)
    try:
        output = run_stage7(
            stage6_dir=Path(args.stage6_dir), map_yaml=Path(args.map_yaml), queries_yaml=Path(args.queries),
            smac_dir=Path(args.smac_reference), output_dir=Path(args.output_dir), modes=modes,
            query_ids=query_ids or None, max_repetitions=args.max_repetitions, validate_only=args.validate_only,
        )
    except (ValueError, OSError, KeyError) as exc:
        print(f"kinematic_benchmark: ERROR: {exc}", file=sys.stderr)
        return 2
    print(f"stage7 kinematic output: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
