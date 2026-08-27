"""Static A2B scale baseline for the fixed Hospital-derived map family.

This command is intentionally a thin orchestration layer around the existing
topology/grid and hard-radius L3 code. It creates versioned query/protocol
inputs and writes a read-only cross-scale report. It never starts Gazebo or
accepts dynamic-obstacle input.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import gzip
import hashlib
import json
import math
import os
import resource
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd
import yaml
from PIL import Image

from .planner_benchmark.map_utils import HospitalMap, sha256_file
from .planner_benchmark.config import load_queries
from .topology import build_topology, save_topology
from .topology_cli import run_topology_benchmark
from .stage8 import HardRadiusConfig, diagnose_hard_path, distance, trigger_indices
from .stage8_cli import FOOTPRINT, StaticSmacBackend, repair_with_backend


WORKSPACE = Path("/home/robot/pudu_robot_ws")
DEFAULT_ROOT = WORKSPACE / "experiments/scale_benchmark/hospital_static_v1"
SOURCE_QUERIES = WORKSPACE / "experiments/planner_benchmark/hospital_005/queries_v2.yaml"
MAP_SPECS: Dict[str, Dict[str, Any]] = {
    "hospital_005": {"scale": 1.0, "map_yaml": WORKSPACE / "experiments/maps/hospital_005/map.yaml"},
    "hospital_100x100_005": {"scale": 1.25, "map_yaml": WORKSPACE / "experiments/maps/hospital_100x100_005/map.yaml"},
    "hospital_200x200_005": {"scale": 2.5, "map_yaml": WORKSPACE / "experiments/maps/hospital_200x200_005/map.yaml"},
    "hospital_400x400_005": {"scale": 5.0, "map_yaml": WORKSPACE / "experiments/maps/hospital_400x400_005/map.yaml"},
}
TIMEOUT_BY_MAP_SECONDS = {
    "hospital_005": 5.0,
    "hospital_100x100_005": 5.0,
    "hospital_200x200_005": 15.0,
    "hospital_400x400_005": 60.0,
}
Q04_FAILURE = "STATIC_SEMANTICS_CONSERVATIVE_INFLATION_MISMATCH"
SCHEMA_VERSION = 1


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _read_yaml(path: Path) -> Dict[str, Any]:
    return yaml.safe_load(path.read_text()) or {}


def _write_yaml(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False))


def _source_queries() -> Dict[str, Any]:
    payload = _read_yaml(SOURCE_QUERIES)
    if len(payload.get("queries", [])) != 10:
        raise ValueError("the frozen Hospital query set must contain exactly 10 queries")
    return payload


def _map_info(map_id: str) -> Dict[str, Any]:
    if map_id not in MAP_SPECS:
        raise ValueError(f"unknown scale map: {map_id}")
    map_yaml = Path(MAP_SPECS[map_id]["map_yaml"]).resolve()
    config = _read_yaml(map_yaml)
    image = Path(config["image"])
    if not image.is_absolute():
        image = map_yaml.parent / image
    with Image.open(image) as handle:
        width, height = handle.size
    resolution = float(config["resolution"])
    origin = [float(value) for value in config["origin"]]
    return {
        "map_id": map_id,
        "map_yaml": str(map_yaml),
        "map_yaml_sha256": sha256_file(map_yaml),
        "map_sha256": sha256_file(image),
        "image": str(image.resolve()),
        "width_cells": int(width),
        "height_cells": int(height),
        "grid_cells": int(width * height),
        "resolution": resolution,
        "origin": origin,
        "physical_width_m": float(width * resolution),
        "physical_height_m": float(height * resolution),
        "physical_area_m2": float(width * height * resolution * resolution),
        "scale": float(MAP_SPECS[map_id]["scale"]),
    }


def _scaled_queries(map_id: str) -> Dict[str, Any]:
    source = _source_queries()
    scale = float(MAP_SPECS[map_id]["scale"])
    queries = []
    for item in source["queries"]:
        start = [float(item["start"][0]) * scale, float(item["start"][1]) * scale, float(item["start"][2])]
        goal = [float(item["goal"][0]) * scale, float(item["goal"][1]) * scale, float(item["goal"][2])]
        queries.append({
            "query_id": str(item["query_id"]),
            "category": str(item.get("category", "unspecified")),
            "seed": int(item.get("seed", source.get("seed", 20260821))),
            "validation_status": "UNVALIDATED",
            "start": start,
            "goal": goal,
            "source_query_id": str(item["query_id"]),
            "scale_factor": scale,
        })
    info = _map_info(map_id)
    return {
        "schema_version": 1,
        "query_set_id": f"{map_id}_scaled_from_hospital_005_queries_v2",
        "map": map_id,
        "resolution": info["resolution"],
        "scale_factor": scale,
        "source_queries": str(SOURCE_QUERIES),
        "transformation": "x_new=scale*x_old; y_new=scale*y_old; yaw_new=yaw_old",
        "seed": int(source.get("seed", 20260821)),
        "queries": queries,
    }


def _protocol(map_id: str) -> Dict[str, Any]:
    info = _map_info(map_id)
    return {
        "schema_version": SCHEMA_VERSION,
        "experiment": "hospital_static_scale_baseline_v1",
        "map": map_id,
        "map_yaml": info["map_yaml"],
        "map_sha256": info["map_sha256"],
        "map_yaml_sha256": info["map_yaml_sha256"],
        "resolution": info["resolution"],
        "width_cells": info["width_cells"],
        "height_cells": info["height_cells"],
        "grid_cells": info["grid_cells"],
        "physical_extent_m": [info["physical_width_m"], info["physical_height_m"]],
        "physical_area_m2": info["physical_area_m2"],
        "origin": info["origin"],
        "scale_factor": info["scale"],
        "random_seed": 20260821,
        "warmup_runs": 3,
        "measured_runs": 5,
        "external_timeout_seconds": TIMEOUT_BY_MAP_SECONDS[map_id],
        "dynamic_obstacles": False,
        "allow_unknown": False,
        "allow_in_place_rotation": False,
        "allow_reverse": True,
        "preferred_minimum_turning_radius": 0.40,
        "reverse_penalty": 2.0,
        "minimum_endpoint_clearance_m": 0.5,
        "variants": {
            "product": {
                "allow_unknown": True,
                "tolerance": 0.5,
                "inflation_radius": 0.55,
                "cost_scaling_factor": 3.0,
            },
            "normalized": {
                "allow_unknown": False,
                "tolerance": 0.25,
                "inflation_radius": 0.55,
                "cost_scaling_factor": 3.0,
            },
        },
        "footprint_padding_m": 0.05,
        "additional_safety_margin_m": 0.05,
        "corridor_padding_sequence_m": [1.0, 2.0, 4.0],
        "attach_radius_m": 5.0,
        "footprint": FOOTPRINT,
        "vehicle_model": {
            "allow_in_place_rotation": False,
            "minimum_turning_radius_m": 0.40,
            "maximum_curvature_per_m": 2.50,
            "allow_reverse": True,
            "reverse_penalty": 2.0,
            "motion_model": "REEDS_SHEPP",
        },
        "modes": ["full_grid", "topology_guided_grid_fallback", "layered_hard_radius_l3"],
        "query_transform": "uniform_geometry_scale",
    }


def prepare_inputs(root: Path, map_ids: Sequence[str]) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    manifest = {
        "schema_version": 1,
        "experiment": "hospital_static_scale_baseline_v1",
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "dynamic_obstacles": False,
        "source_queries": str(SOURCE_QUERIES),
        "query_transform": "x_new=scale*x_old; y_new=scale*y_old; yaw_new=yaw_old",
        "map_ids": list(map_ids),
        "modes": ["full_grid", "topology_guided_grid_fallback", "layered_hard_radius_l3"],
    }
    for map_id in map_ids:
        info = _map_info(map_id)
        directory = root / map_id
        directory.mkdir(parents=True, exist_ok=True)
        _write_yaml(directory / "protocol.yaml", _protocol(map_id))
        _write_yaml(directory / "queries.yaml", _scaled_queries(map_id))
        manifest.setdefault("maps", []).append(info)
    _write_yaml(root / "manifest.yaml", manifest)
    return root


def _query_objects(path: Path) -> List[Dict[str, Any]]:
    return list((_read_yaml(path).get("queries", [])))


def validate_inputs(root: Path, map_ids: Sequence[str]) -> pd.DataFrame:
    rows: List[Dict[str, Any]] = []
    for map_id in map_ids:
        directory = root / map_id
        protocol = _read_yaml(directory / "protocol.yaml")
        map_path = Path(protocol["map_yaml"])
        hospital_map = HospitalMap.load(map_path)
        if hospital_map.width != int(protocol["width_cells"]) or hospital_map.height != int(protocol["height_cells"]):
            raise ValueError(f"map dimensions do not match protocol for {map_id}")
        if not math.isclose(hospital_map.resolution, float(protocol["resolution"]), abs_tol=1e-12):
            raise ValueError(f"map resolution does not match protocol for {map_id}")
        if bool(protocol.get("dynamic_obstacles", True)):
            raise ValueError("dynamic_obstacles must be false")
        for item in _query_objects(directory / "queries.yaml"):
            from .planner_benchmark.models import Query
            query = Query(
                query_id=str(item["query_id"]), start=[float(v) for v in item["start"]],
                goal=[float(v) for v in item["goal"]], category=str(item.get("category", "")),
                seed=int(item.get("seed", 20260821)), validation_status="UNVALIDATED",
            )
            checked = hospital_map.validate_query(query, FOOTPRINT, 0.0, allow_unknown=False)
            start_status = checked.start_status
            goal_status = checked.goal_status
            validation = "VALID" if checked.validation_status == "VALID" else (
                "INVALID_START" if start_status != "VALID" else "INVALID_GOAL"
            )
            rows.append({
                "map_id": map_id, "query_id": query.query_id,
                "validation_status": validation, "start_status": start_status,
                "goal_status": goal_status, "connected_raw": checked.connected,
                "start_clearance_m": checked.start_clearance_m,
                "goal_clearance_m": checked.goal_clearance_m,
                "reason": checked.reason, "dynamic_obstacles": False,
            })
        frame = pd.DataFrame([row for row in rows if row["map_id"] == map_id])
        frame.to_csv(directory / "query_validation.csv", index=False)
    result = pd.DataFrame(rows)
    result.to_csv(root / "query_validation_all.csv", index=False)
    return result


def _read_points(path: Path) -> List[Dict[str, float]]:
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        return [{"x": float(p["x"]), "y": float(p["y"]), "yaw": float(p["yaw"])} for p in json.load(stream)]


def _write_points(path: Path, points: Sequence[Dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(path, "wt", encoding="utf-8") as stream:
        json.dump(list(points), stream)


def _path_length(points: Sequence[Dict[str, float]]) -> float:
    return sum(distance(a, b) for a, b in zip(points, points[1:]))


def _resource_peak() -> int:
    return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024


def _run_l3(
    topology_dir: Path,
    protocol_path: Path,
    map_id: str,
    output_dir: Path,
    *,
    with_hybrid: bool,
    timeout: float,
) -> Path:
    """Evaluate fallback paths with the existing hard-radius L3 adapter."""
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError(f"refusing to overwrite scale L3 output: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    protocol = _read_yaml(protocol_path)
    map_path = Path(protocol["map_yaml"])
    hospital_map = HospitalMap.load(map_path)
    frame = pd.read_csv(topology_dir / "query_runs.csv")
    frame = frame[(frame["mode"] == "topology_guided_grid_fallback") & (frame["run_mode"] == "measured")].copy()
    if len(frame) != 50:
        raise ValueError(f"expected 50 measured fallback rows for {map_id}, found {len(frame)}")
    config = HardRadiusConfig(allow_in_place_rotation=False)
    output_protocol = dict(protocol)
    output_protocol.update({
        "experiment": "hospital_static_scale_layered_hard_radius_l3",
        "l3_execution_mode": "static_smac_reeds_shepp_on_demand" if with_hybrid else "validator_only_no_hybrid",
        "dynamic_obstacles": False,
        "minimum_turning_radius_m": 0.40,
        "maximum_curvature_per_m": 2.50,
        "allow_in_place_rotation": False,
    })
    _write_yaml(output_dir / "protocol.yaml", output_protocol)
    backend: Optional[StaticSmacBackend] = None
    runs: List[Dict[str, Any]] = []
    metrics: List[Dict[str, Any]] = []
    segments: List[Dict[str, Any]] = []
    full_baselines = {}
    full = pd.read_csv(topology_dir / "query_runs.csv")
    for _, base in full[(full["mode"] == "full_grid") & (full["run_mode"] == "measured")].iterrows():
        full_baselines[(str(base.query_id), int(base.repetition))] = base.get("final_path_length_m")
    try:
        for _, row in frame.iterrows():
            run_id = f"{map_id}_{row.query_id}_layered_hard_radius_l3_measured_{int(row.repetition)}_{time.time_ns()}"
            path_value = row.get("path_file")
            points = None if pd.isna(path_value) or not str(path_value) else _read_points(topology_dir / str(path_value))
            result: Dict[str, Any] = {
                "run_id": run_id, "map_id": map_id, "query_id": str(row.query_id),
                "repetition": int(row.repetition), "run_mode": "measured",
                "mode": "layered_hard_radius_l3", "source_stage6_run_id": str(row.run_id),
                "dynamic_obstacles": False, "allow_in_place_rotation": False,
                "minimum_turning_radius_m": 0.40, "maximum_curvature": 2.50,
                "stage6_online_time_ms": row.get("total_online_time_ms"),
                "l3_execution_mode": "static_smac_reeds_shepp_on_demand" if with_hybrid else "validator_only_no_hybrid",
                "hybrid_calls": 0, "hybrid_success": 0, "repair_window_count": 0,
                "repair_window_padding_m": None, "l3_planning_time_ms": 0.0,
                "l3_cpu_total_ms": 0.0, "l3_rss_peak_bytes": None, "l3_pss_peak_bytes": None,
                "composed_online_time_is_estimate": True, "source": "grid",
                "grid_mode": str(row.get("grid_mode", "full_grid_fallback")),
                "topology_edge_ids": str(row.get("topology_edge_ids", "[]")),
                "q04_diagnostic": Q04_FAILURE if str(row.query_id) == "q04" and pd.isna(row.get("path_file")) else "",
            }
            if points is None:
                result.update({"action_success": False, "static_footprint_valid": False,
                               "hard_kinematic_valid": False, "final_valid_success": False,
                               "failure_code": str(row.get("result_code", "EMPTY_PATH")) if str(row.get("result_code", "")) else (Q04_FAILURE if str(row.query_id) == "q04" else "EMPTY_PATH"),
                               "l3_triggered": False, "path_file": ""})
                runs.append(result)
                metrics.append({**result, "final_path_length_m": None, "length_over_full_grid": None})
                continue
            before = diagnose_hard_path(points, hospital_map, FOOTPRINT, config)
            triggers = trigger_indices(points, before, config)
            repaired = {"success": before.hard_kinematic_valid, "points": points, "before": before,
                        "after": before, "hybrid_calls": 0, "hybrid_success": 0,
                        "planning_time_ms": 0.0, "cpu_total_ms": 0.0, "rss_peak_bytes": None,
                        "pss_peak_bytes": None, "windows": [], "used_windows": [],
                        "failure_reason": "" if before.hard_kinematic_valid else "KINEMATIC_REPAIR_FAILED"}
            if triggers and with_hybrid:
                if backend is None:
                    import rclpy
                    rclpy.init()
                    backend = StaticSmacBackend(map_path, output_protocol, output_dir, timeout)
                    backend.start()
                try:
                    edge_ids = yaml.safe_load(str(row.get("topology_edge_ids", "[]"))) or []
                except Exception:
                    edge_ids = []
                repaired = repair_with_backend(points, hospital_map, config, backend, edge_ids, str(row.get("grid_mode", "full_grid_fallback")))
            after = repaired.get("after")
            final_points = repaired.get("points") if repaired.get("success") else None
            l3_ms = float(repaired.get("planning_time_ms", 0.0) or 0.0)
            result.update({
                "action_success": bool(final_points),
                "static_footprint_valid": bool(after and after.static_collision_count == 0),
                "hard_kinematic_valid": bool(after and after.hard_kinematic_valid),
                "turning_radius_preference_satisfied": bool(after and after.turning_radius_preference_satisfied),
                "final_valid_success": bool(repaired.get("success", False)),
                "failure_code": str(repaired.get("failure_reason", "")),
                "hybrid_calls": int(repaired.get("hybrid_calls", 0)),
                "hybrid_success": int(repaired.get("hybrid_success", 0)),
                "hybrid_failure_reason": str(repaired.get("hybrid_failure_reason", "")),
                "l3_triggered": bool(triggers), "trigger_count": len(triggers),
                "repair_window_count": len(repaired.get("windows", [])),
                "repair_window_padding_m": max((float(item.get("padding_m", 0.0)) for item in repaired.get("used_windows", [])), default=None),
                "repair_length_m": sum(float(item.get("length_m", 0.0)) for item in repaired.get("used_windows", [])),
                "l3_planning_time_ms": l3_ms,
                "l3_cpu_total_ms": float(repaired.get("cpu_total_ms", 0.0) or 0.0),
                "l3_rss_peak_bytes": repaired.get("rss_peak_bytes"),
                "l3_pss_peak_bytes": repaired.get("pss_peak_bytes"),
            })
            result["composed_online_time_ms"] = float(row.get("total_online_time_ms", 0.0) or 0.0) + l3_ms
            if final_points:
                rel = Path("paths") / f"{run_id}.json.gz"
                _write_points(output_dir / rel, final_points)
                result["path_file"] = str(rel); result["path_point_count"] = len(final_points)
                result["final_path_length_m"] = _path_length(final_points)
                result["length_over_full_grid"] = (result["final_path_length_m"] / float(full_baselines[(str(row.query_id), int(row.repetition))])
                                                     if full_baselines.get((str(row.query_id), int(row.repetition))) else None)
                for index, item in enumerate(repaired.get("used_windows", [])):
                    for segment in item.get("segments", []):
                        segments.append({"run_id": run_id, "query_id": row.query_id, "segment_index": index, **segment})
            else:
                result["path_file"] = ""; result["final_path_length_m"] = None; result["length_over_full_grid"] = None
            runs.append(result)
            metrics.append({**result, "final_path_length_m": result.get("final_path_length_m"), "length_over_full_grid": result.get("length_over_full_grid")})
    finally:
        if backend is not None:
            backend.stop()
            try:
                import rclpy
                rclpy.shutdown()
            except Exception:
                pass
    run_frame = pd.DataFrame(runs)
    run_frame.to_csv(output_dir / "kinematic_runs.csv", index=False)
    pd.DataFrame(metrics).to_csv(output_dir / "path_metrics.csv", index=False)
    pd.DataFrame(segments).to_csv(output_dir / "segment_metrics.csv", index=False)
    failures = run_frame[~run_frame["final_valid_success"].astype(bool)] if not run_frame.empty else run_frame
    failures.groupby(["failure_code"], dropna=False).size().rename("count").reset_index().to_csv(output_dir / "failure_summary.csv", index=False)
    summary = []
    if not run_frame.empty:
        for mode, group in run_frame.groupby("mode"):
            valid = group["final_valid_success"].astype(bool)
            reachable = group[group["query_id"] != "q04"]
            summary.append({"mode": mode, "count": len(group), "success_count": int(valid.sum()),
                            "all_query_success_rate": float(valid.mean()),
                            "reachable_query_success_rate": float(reachable["final_valid_success"].astype(bool).mean()) if len(reachable) else None,
                            "l3_triggered_paths": int(group["l3_triggered"].astype(bool).sum()),
                            "hybrid_calls": int(group["hybrid_calls"].sum()), "hybrid_success": int(group["hybrid_success"].sum()),
                            "l3_execution_mode": str(group["l3_execution_mode"].iloc[0])})
    pd.DataFrame(summary).to_csv(output_dir / "summary_by_mode.csv", index=False)
    (output_dir / "manifest.yaml").write_text(yaml.safe_dump({
        "schema_version": 1, "experiment": "hospital_static_scale_layered_hard_radius_l3",
        "map": map_id, "topology_input": str(topology_dir), "dynamic_obstacles": False,
        "with_hybrid": with_hybrid, "input_rows": len(frame), "modes": ["layered_hard_radius_l3"],
    }, sort_keys=False))
    return output_dir


def _quantile(series: pd.Series, q: float) -> Optional[float]:
    values = pd.to_numeric(series, errors="coerce").dropna()
    return float(values.quantile(q)) if len(values) else None


def _mode_rows(map_id: str, mode: str, frame: pd.DataFrame, reachable_ids: Optional[Sequence[str]] = None) -> Dict[str, Any]:
    measured = frame[frame.get("run_mode", "measured").astype(str).eq("measured")] if "run_mode" in frame else frame
    valid = measured.get("final_valid_success", pd.Series(dtype=bool)).astype(bool)
    queries = measured.get("query_id", pd.Series(dtype=str)).astype(str)
    reachable_ids = set(str(value) for value in (reachable_ids if reachable_ids is not None else queries[valid]))
    scalar_map = {
        "planning_time_ms": "planning_time_ms" if "planning_time_ms" in measured else "search_time_ms",
        "wall_time_ms": "total_online_time_ms" if "total_online_time_ms" in measured else "composed_online_time_ms",
        "cpu_total_ms": "cpu_total_ms" if "cpu_total_ms" in measured else "l3_cpu_total_ms",
        "planner_rss_peak_bytes": "query_rss_peak_bytes" if "query_rss_peak_bytes" in measured else "l3_rss_peak_bytes",
        "planner_pss_peak_bytes": "query_pss_peak_bytes" if "query_pss_peak_bytes" in measured else "l3_pss_peak_bytes",
        "expanded_nodes": "expanded_nodes",
        "search_space_ratio": "search_space_ratio",
        "path_length_m": "final_path_length_m",
        "length_over_full_grid": "length_over_full_grid",
    }
    result: Dict[str, Any] = {"map_id": map_id, "mode": mode, "run_mode": "measured", "count": len(measured),
                              "success_count": int(valid.sum()), "query_count": int(queries.nunique()),
                              "success_rate": float(valid.mean()) if len(valid) else None,
                              "reachable_query_count": len(reachable_ids),
                              "reachable_query_success_rate": float(valid[queries.isin(reachable_ids)].mean()) if reachable_ids else None}
    if mode == "layered_hard_radius_l3":
        scalar_map["planning_time_ms"] = "l3_planning_time_ms"
    for label, field in scalar_map.items():
        if field not in measured:
            for suffix, q in (("P50", .5), ("P95", .95), ("P99", .99)):
                result[f"{label}_{suffix}"] = None
            continue
        for suffix, q in (("P50", .5), ("P95", .95), ("P99", .99)):
            result[f"{label}_{suffix}"] = _quantile(measured.loc[valid, field], q)
    return result


def report_scale(root: Path, map_ids: Sequence[str]) -> Path:
    report_dir = root / "stage_summary_v3"
    if report_dir.exists() and any(report_dir.iterdir()):
        raise ValueError(f"refusing to overwrite scale summary: {report_dir}")
    report_dir.mkdir(parents=True, exist_ok=True)
    summary: List[Dict[str, Any]] = []
    precompute: List[Dict[str, Any]] = []
    for map_id in map_ids:
        directory = root / map_id
        info = _map_info(map_id)
        topology_dir = directory / "topology_benchmark"
        pre = pd.read_csv(topology_dir / "precompute_metrics.csv").iloc[0].to_dict()
        pre.update({"map_id": map_id, "grid_cells": info["grid_cells"], "physical_area_m2": info["physical_area_m2"], "free_grid_cells": int(pd.to_numeric(pd.read_csv(topology_dir / "query_runs.csv")["total_free_grid_cells"], errors="coerce").dropna().iloc[0])})
        precompute.append(pre)
        topo = pd.read_csv(topology_dir / "query_runs.csv")
        topo_measured = topo[topo["run_mode"].eq("measured")] if "run_mode" in topo else topo
        full_measured = topo_measured[topo_measured["mode"].eq("full_grid")]
        full_success = full_measured[full_measured["final_valid_success"].astype(bool)]
        reachable_ids = sorted(set(full_success["query_id"].astype(str)))
        for mode in ("full_grid", "topology_guided_grid_fallback"):
            row = _mode_rows(map_id, mode, topo[topo["mode"].eq(mode)], reachable_ids)
            row.update({"grid_cells": info["grid_cells"], "physical_area_m2": info["physical_area_m2"], "resolution": info["resolution"], "map_sha256": info["map_sha256"]})
            summary.append(row)
        l3_path = directory / "layered_hard_radius_l3"
        if (l3_path / "kinematic_runs.csv").exists():
            l3 = _enrich_l3_output(l3_path)
            row = _mode_rows(map_id, "layered_hard_radius_l3", l3, reachable_ids)
            row.update({"grid_cells": info["grid_cells"], "physical_area_m2": info["physical_area_m2"], "resolution": info["resolution"], "map_sha256": info["map_sha256"], "l3_execution_mode": str(l3.get("l3_execution_mode", pd.Series(["unknown"])).iloc[0])})
            summary.append(row)
    summary_frame = pd.DataFrame(summary)
    summary_frame.to_csv(report_dir / "scale_summary.csv", index=False)
    summary_frame.to_csv(report_dir / "scale_by_map.csv", index=False)
    pd.DataFrame(precompute).to_csv(report_dir / "topology_precompute_by_map.csv", index=False)
    query_rows: List[Dict[str, Any]] = []
    failure_rows: List[Dict[str, Any]] = []
    for map_id in map_ids:
        directory = root / map_id
        topo = pd.read_csv(directory / "topology_benchmark" / "query_runs.csv")
        measured = topo[topo["run_mode"].eq("measured")] if "run_mode" in topo else topo
        for mode, group in measured.groupby("mode"):
            for query_id, subset in group.groupby("query_id"):
                query_rows.append({"map_id": map_id, "mode": mode, "query_id": query_id,
                                   "count": len(subset), "success_count": int(subset["final_valid_success"].astype(bool).sum()),
                                   "success_rate": float(subset["final_valid_success"].astype(bool).mean())})
        failure_rows.extend(measured.loc[~measured["final_valid_success"].astype(bool),
                                         ["query_id", "mode", "result_code"]].assign(map_id=map_id).to_dict("records"))
        l3_file = directory / "layered_hard_radius_l3" / "kinematic_runs.csv"
        if l3_file.exists():
            l3 = pd.read_csv(l3_file)
            for query_id, subset in l3.groupby("query_id"):
                query_rows.append({"map_id": map_id, "mode": "layered_hard_radius_l3", "query_id": query_id,
                                   "count": len(subset), "success_count": int(subset["final_valid_success"].astype(bool).sum()),
                                   "success_rate": float(subset["final_valid_success"].astype(bool).mean())})
            failure_rows.extend(l3.loc[~l3["final_valid_success"].astype(bool),
                                       ["query_id", "mode", "failure_code"]].rename(columns={"failure_code": "result_code"}).assign(map_id=map_id).to_dict("records"))
    pd.DataFrame(query_rows).to_csv(report_dir / "scale_by_query.csv", index=False)
    pd.DataFrame(failure_rows).to_csv(report_dir / "scale_failure_summary.csv", index=False)
    amortized = []
    for row in precompute:
        build_ms = float(row.get("topology_build_wall_time_ms", 0.0) or 0.0)
        for query_count in (10, 100, 1000):
            amortized.append({"map_id": row["map_id"], "query_count": query_count, "topology_build_wall_time_ms": build_ms, "amortized_topology_cost_ms": build_ms / query_count})
    pd.DataFrame(amortized).to_csv(report_dir / "scale_amortization.csv", index=False)
    _write_scale_plots(report_dir / "plots", summary_frame)
    _write_precompute_plots(report_dir / "plots", pd.DataFrame(precompute))
    return report_dir


def _write_scale_plots(directory: Path, frame: pd.DataFrame) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    x_fields = [("grid_cells", "grid cells"), ("physical_area_m2", "physical area (m2)")]
    metrics = [("wall_time_ms_P50", "Wall time P50 (ms)", "wall_time_vs_scale"),
               ("wall_time_ms_P95", "Wall time P95 (ms)", "wall_time_p95_vs_scale"),
               ("cpu_total_ms_P50", "CPU time P50 (ms)", "cpu_time_vs_scale"),
               ("planner_rss_peak_bytes_P50", "Planner RSS P50 (bytes)", "planner_rss_vs_scale"),
               ("planner_pss_peak_bytes_P50", "Planner PSS P50 (bytes)", "planner_pss_vs_scale"),
               ("expanded_nodes_P50", "A* expanded nodes P50", "expanded_nodes_vs_scale"),
               ("success_rate", "Final valid success rate", "success_rate_vs_scale")]
    for x_field, xlabel in x_fields:
        for field, ylabel, name in metrics:
            if field not in frame:
                continue
            fig, axis = plt.subplots(figsize=(9, 5))
            for mode, group in frame.groupby("mode"):
                values = group.sort_values(x_field)
                y = pd.to_numeric(values[field], errors="coerce")
                axis.plot(values[x_field], y, marker="o", label=mode)
            axis.set_xlabel(xlabel); axis.set_ylabel(ylabel); axis.set_title(f"{ylabel} vs {xlabel}"); axis.grid(True, alpha=.25); axis.legend()
            fig.tight_layout(); fig.savefig(directory / f"{name}_by_{x_field}.png", dpi=140); plt.close(fig)
    pre = frame.iloc[0:0]


def _write_precompute_plots(directory: Path, frame: pd.DataFrame) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    for field, title, name in [
        ("topology_build_wall_time_ms", "Topology build wall time", "topology_build_time"),
        ("topology_build_peak_rss_bytes", "Topology build peak RSS", "topology_build_rss"),
        ("topology_file_size_bytes", "Persisted topology size", "topology_file_size"),
    ]:
        if field not in frame:
            continue
        values = frame.sort_values("grid_cells")
        fig, axis = plt.subplots(figsize=(9, 5))
        axis.plot(values["grid_cells"], pd.to_numeric(values[field], errors="coerce"), marker="o")
        axis.set_xlabel("grid cells"); axis.set_ylabel(field); axis.set_title(f"{title} vs grid cells"); axis.grid(True, alpha=.25)
        fig.tight_layout(); fig.savefig(directory / f"{name}_vs_grid_cells.png", dpi=140); plt.close(fig)


def _enrich_l3_output(directory: Path) -> pd.DataFrame:
    """Fill diagnostics from saved paths without rerunning the planner."""
    path = directory / "kinematic_runs.csv"
    frame = pd.read_csv(path)
    protocol = _read_yaml(directory.parent / "protocol.yaml")
    hospital_map = HospitalMap.load(Path(protocol["map_yaml"]))
    config = HardRadiusConfig(allow_in_place_rotation=False)
    for index, row in frame.iterrows():
        value = row.get("path_file")
        if value is None or str(value) in {"", "nan", "NaN"}:
            continue
        path_file = Path(str(value))
        if not path_file.is_absolute():
            path_file = directory / path_file
        if not path_file.exists():
            continue
        diagnostics = diagnose_hard_path(_read_points(path_file), hospital_map, FOOTPRINT, config)
        updates = diagnostics.as_dict()
        updates["failure_codes"] = json.dumps(diagnostics.failure_codes)
        updates["final_path_length_m"] = _path_length(diagnostics.points)
        for key, value in updates.items():
            frame.at[index, key] = value
    frame.to_csv(path, index=False)
    frame.to_csv(directory / "kinematic_runs_enriched.csv", index=False)
    return frame


def run_maps(root: Path, map_ids: Sequence[str], *, repetitions: int, warmups: int, with_hybrid: bool, timeout: float, query_ids: Optional[Sequence[str]] = None) -> None:
    for map_id in map_ids:
        directory = root / map_id
        protocol = directory / "protocol.yaml"
        queries = directory / "queries.yaml"
        topology_output = directory / "topology_benchmark"
        if topology_output.exists() and any(topology_output.iterdir()):
            raise ValueError(f"refusing to overwrite existing scale topology output: {topology_output}")
        run_topology_benchmark(
            map_name="hospital_005", protocol_path=protocol, queries_path=queries,
            output_dir=topology_output, topology_dir=None,
            modes=["full_grid", "topology_guided_grid_fallback"], query_ids=query_ids,
            repetitions=repetitions, warmups=warmups, build_only=False,
            corridor_padding_m=1.0, attach_radius_m=5.0,
        )
        _run_l3(topology_output, protocol, map_id, directory / "layered_hard_radius_l3", with_hybrid=with_hybrid, timeout=timeout)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the static Hospital scale baseline across 80/100/200/400 m maps")
    parser.add_argument("--root", default=str(DEFAULT_ROOT))
    parser.add_argument("--stage", choices=["prepare", "validate", "run", "report", "all"], default="prepare")
    parser.add_argument("--map-id", action="append", dest="map_ids", choices=list(MAP_SPECS))
    parser.add_argument("--repetitions", type=int, default=8, help="3 warmups + 5 measured by default")
    parser.add_argument("--warmups", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--query-id", action="append", dest="query_ids")
    parser.add_argument("--no-hybrid", action="store_true", help="L3 validator-only smoke; formal runs should omit this")
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    map_ids = args.map_ids or list(MAP_SPECS)
    root = Path(args.root).resolve()
    try:
        if args.stage in {"prepare", "all"}:
            prepare_inputs(root, map_ids)
        if args.stage in {"validate", "all"}:
            validate_inputs(root, map_ids)
        if args.stage in {"run", "all"}:
            run_maps(root, map_ids, repetitions=args.repetitions, warmups=args.warmups,
                     with_hybrid=not args.no_hybrid, timeout=args.timeout, query_ids=args.query_ids)
        if args.stage in {"report", "all"}:
            report_scale(root, map_ids)
        print(f"scale benchmark {args.stage} output: {root}")
        return 0
    except Exception as exc:
        print(f"scale_benchmark: ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
