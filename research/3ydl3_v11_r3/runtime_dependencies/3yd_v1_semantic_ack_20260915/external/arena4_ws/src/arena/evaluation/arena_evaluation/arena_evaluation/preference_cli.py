"""Stage 8B static L2 preference scan and selected hard-radius L3."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import resource
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import pandas as pd
import yaml

from .planner_benchmark.config import load_protocol, load_queries
from .planner_benchmark.map_utils import HospitalMap
from .preference import build_preference_geometry, path_preference_metrics, preference_astar
from .stage8 import HardRadiusConfig, diagnose_hard_path, distance
from .stage8_cli import FOOTPRINT, Q04_FAILURE, StaticSmacBackend, _protocol_payload, _save_points, repair_with_backend
from .topology import attach_pose, cells_to_poses, corridor_mask, load_topology, search_topology
from .topology_cli import corridor_padding_schedule

DEFAULT_MAP = Path("experiments/maps/hospital_005/map.yaml")
DEFAULT_PROTOCOL = Path("experiments/planner_benchmark/hospital_005/topology_protocol_v2.yaml")
DEFAULT_QUERIES = Path("experiments/planner_benchmark/hospital_005/queries_v2.yaml")
DEFAULT_TOPOLOGY = Path("experiments/topology_benchmark/hospital_005/stage5_full_v2/topology")
WEIGHTS = (0.0, 0.25, 0.5, 1.0)


def _write(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    fields: List[str] = []
    for row in rows:
        for key in row:
            if key not in fields: fields.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(rows)


def _run_l2(artifact, query, mode: str, weight: float, output: Path) -> Tuple[Dict[str, object], Optional[List[Dict[str, float]]]]:
    started = time.monotonic_ns(); cpu_started = time.process_time_ns(); rss_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
    start = artifact.hospital_map.world_to_cell(query.start[0], query.start[1]); goal = artifact.hospital_map.world_to_cell(query.goal[0], query.goal[1])
    row = {"query_id": query.query_id, "preference_mode": mode, "preference_weight": weight, "dynamic_obstacles": False, "right_wall_target_m": 0.40, "narrow_width_threshold_m": 1.23, "fallback_used": False, "fallback_reason": "", "corridor_padding_used_m": None, "corridor_attempts": 0, "result_code": ""}
    if query.query_id == "q04" or start is None or goal is None or not artifact.free_mask[start] or not artifact.free_mask[goal]:
        row.update({"result_code": Q04_FAILURE if query.query_id == "q04" else "INVALID_ENDPOINT", "final_valid_success": False}); return row, None
    start_attach = attach_pose(artifact, query.start, FOOTPRINT, max_radius_m=5.0, allow_unknown=False); goal_attach = attach_pose(artifact, query.goal, FOOTPRINT, max_radius_m=5.0, allow_unknown=False)
    if start_attach is None or goal_attach is None:
        row.update({"result_code": "PREFERENCE_GEOMETRY_UNAVAILABLE", "final_valid_success": False}); return row, None
    route = search_topology(artifact, start_attach.node_id, goal_attach.node_id)
    if route is None:
        row.update({"result_code": "PREFERENCE_GEOMETRY_UNAVAILABLE", "final_valid_success": False}); return row, None
    geometry = build_preference_geometry(artifact, route, mode)
    if geometry.failure_code:
        row.update({"result_code": geometry.failure_code, "final_valid_success": False}); return row, None
    attempts = []; result = None; grid_mode = "corridor"
    for padding in corridor_padding_schedule(1.0):
        mask = corridor_mask(artifact, route, start, goal, padding)
        result = preference_astar(artifact.free_mask, start, goal, mask, geometry.penalty if mode != "none" else None, weight, artifact.hospital_map.resolution)
        attempts.append(result); row["corridor_attempts"] += 1
        if result.path:
            row["corridor_padding_used_m"] = padding; grid_mode = "corridor" if padding == 1.0 else "expanded_corridor"; break
    if result is None or result.path is None:
        row["fallback_used"] = True; row["fallback_reason"] = "CORRIDOR_NO_PATH"
        result = preference_astar(artifact.free_mask, start, goal, None, geometry.penalty if mode != "none" else None, weight, artifact.hospital_map.resolution)
        attempts.append(result); grid_mode = "full_grid_fallback"
    if result.path is None:
        code = "CENTER_PREFERENCE_NO_PATH" if mode == "center" else ("RIGHT_EDGE_PREFERENCE_NO_PATH" if mode == "right_edge" else "FULL_GRID_FAILED")
        row.update({"result_code": code, "final_valid_success": False}); return row, None
    poses = cells_to_poses(artifact, result.path, query.start[2], query.goal[2]); diagnostics = diagnose_hard_path(poses, artifact.hospital_map, FOOTPRINT, HardRadiusConfig())
    run_id = f"{query.query_id}_{mode}_{weight:g}_{time.time_ns()}"; path_file = Path("paths") / f"{run_id}.json.gz"; _save_points(output / path_file, poses)
    total_expanded = sum(item.expanded_nodes for item in attempts); total_generated = sum(item.generated_nodes for item in attempts)
    row.update({"run_id": run_id, "result_code": "SUCCEEDED" if diagnostics.static_collision_count == 0 else "STATIC_FOOTPRINT_COLLISION", "action_success": True, "static_footprint_valid": diagnostics.static_collision_count == 0, "final_valid_success": diagnostics.static_collision_count == 0, "static_footprint_collision_count": diagnostics.static_collision_count, "grid_mode": grid_mode, "path_file": str(path_file), "path_point_count": len(poses), "path_length_m": sum(distance(a, b) for a, b in zip(poses, poses[1:])), "minimum_clearance_m": min(artifact.hospital_map.clearance(p["x"], p["y"]) or 0.0 for p in poses), "expanded_nodes": total_expanded, "generated_nodes": total_generated, "max_open_set_size": max(item.max_open_set_size for item in attempts), "search_space_ratio": result.search_space_ratio, "search_time_ms": sum(item.search_time_ms for item in attempts), "online_time_ms": (time.monotonic_ns() - started) / 1e6, "cpu_time_ms": (time.process_time_ns() - cpu_started) / 1e6, "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024, "rss_before_bytes": rss_before, "topology_edge_ids": json.dumps(route.edge_ids), **path_preference_metrics(result.path, geometry, artifact)})
    if mode != "none" and float(row.get("preference_active_ratio", 0.0)) == 0.0:
        row["preference_status"] = "PREFERENCE_INACTIVE_NARROW"
    else:
        row["preference_status"] = "ACTIVE" if mode != "none" else "NONE"
    return row, poses


def _select_weights(frame: pd.DataFrame) -> Dict[str, float]:
    baseline = frame[frame.preference_mode.eq("none")]
    baseline_success = float(baseline.final_valid_success.astype(bool).mean())
    baseline_lengths = baseline.set_index("query_id").path_length_m
    selected = {}
    for mode, error_field in (("center", "center_deviation_p50_m"), ("right_edge", "right_wall_error_p50_m")):
        choices = []
        for weight, group in frame[frame.preference_mode.eq(mode)].groupby("preference_weight"):
            success = float(group.final_valid_success.astype(bool).mean())
            valid = group[group.final_valid_success.astype(bool)].copy()
            ratios = [float(row.path_length_m) / float(baseline_lengths[row.query_id]) for _, row in valid.iterrows() if row.query_id in baseline_lengths and float(baseline_lengths[row.query_id]) > 0]
            p95 = float(pd.Series(ratios).quantile(0.95)) if ratios else float("inf")
            collision = int(pd.to_numeric(group.static_footprint_collision_count, errors="coerce").fillna(0).sum())
            error = float(pd.to_numeric(valid.get(error_field), errors="coerce").dropna().mean()) if error_field in valid else float("inf")
            if collision == 0 and success >= baseline_success and p95 <= 1.15:
                choices.append((error, float(weight)))
        if not choices:
            raise RuntimeError(f"no {mode} weight satisfies the frozen selection rules")
        selected[mode] = min(choices)[1]
    return selected


def _run_selected_l3(frame: pd.DataFrame, paths: Dict[str, List[Dict[str, float]]], selected: Dict[str, float], hospital_map: HospitalMap, protocol: Dict[str, object], output: Path, repetitions: int, timeout: float) -> List[Dict[str, object]]:
    import rclpy
    rows = []; config = HardRadiusConfig(); rclpy.init(); backend = StaticSmacBackend(hospital_map.yaml_path, _protocol_payload(protocol, hospital_map.yaml_path, config), output, timeout); backend.start()
    try:
        for mode, weight in selected.items():
            chosen = frame[(frame.preference_mode == mode) & (frame.preference_weight == weight)]
            for _, source in chosen.iterrows():
                for repetition in range(1, repetitions + 1):
                    base = {"query_id": source.query_id, "preference_mode": mode, "preference_weight": weight, "repetition": repetition, "source_l2_run_id": source.get("run_id", ""), "dynamic_obstacles": False, "result_code": "", "final_valid_success": False, "rotate_in_place_count": 0}
                    if source.query_id == "q04" or not bool(source.final_valid_success):
                        base["result_code"] = Q04_FAILURE if source.query_id == "q04" else str(source.result_code); rows.append(base); continue
                    points = paths[str(source.run_id)]; result = repair_with_backend(points, hospital_map, config, backend, [], str(source.grid_mode)); after = result.get("after")
                    base.update({"hybrid_calls": result.get("hybrid_calls", 0), "hybrid_success": result.get("hybrid_success", 0), "hybrid_failure_reason": result.get("hybrid_failure_reason", ""), "result_code": "SUCCEEDED" if result.get("success") else "KINEMATIC_REPAIR_FAILED", "static_footprint_valid": bool(after and after.static_collision_count == 0), "hard_kinematic_valid": bool(after and after.hard_kinematic_valid), "final_valid_success": bool(result.get("success")), "hard_radius_violation_count": after.hard_radius_violation_count if after else None, "static_footprint_collision_count": after.static_collision_count if after else None, "minimum_radius_observed_m": after.minimum_radius_m if after else None, "maximum_curvature_observed": after.maximum_curvature if after else None, "l3_planning_time_ms": result.get("planning_time_ms", 0.0), "l3_cpu_total_ms": result.get("cpu_total_ms", 0.0), "l3_rss_peak_bytes": result.get("rss_peak_bytes"), "l3_pss_peak_bytes": result.get("pss_peak_bytes")})
                    used = result.get("used_windows", []); base["repair_window_count"] = len(result.get("windows", [])); base["repair_length_m"] = sum(float(item.get("length_m", 0.0)) for item in used); base["repair_window_padding_m"] = max((float(item.get("padding_m", 0.0)) for item in used), default=None)
                    final = result.get("points")
                    if final:
                        run_id = f"{source.query_id}_{mode}_hard_l3_{repetition}_{time.time_ns()}"; path_file = Path("paths") / f"{run_id}.json.gz"; _save_points(output / path_file, final); base.update({"run_id": run_id, "path_file": str(path_file), "path_length_m": sum(distance(a, b) for a, b in zip(final, final[1:]))})
                        repair_segments = [segment for item in used for segment in item.get("segments", [])]; repair_length = sum(float(item.get("length_m", 0.0)) for item in repair_segments); total_length = float(base["path_length_m"])
                        base["segments"] = ([{"source": "grid", "planner": "grid_astar", "direction": "mixed", "length_m": max(0.0, total_length - repair_length), "grid_mode": str(source.grid_mode), "repair_reason": ""}] if total_length > repair_length else []) + repair_segments
                    rows.append(base)
    finally:
        backend.stop(); rclpy.shutdown()
    return rows


def run(output: Path, map_path: Path, protocol_path: Path, queries_path: Path, topology_path: Path, l3_repetitions: int, timeout: float, query_ids: Optional[Sequence[str]] = None) -> Path:
    if output.exists() and any(output.iterdir()): raise ValueError(f"refusing to overwrite Stage 8B output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    protocol_file, protocol = load_protocol(protocol_path)
    if bool(protocol.get("dynamic_obstacles", False)): raise ValueError("dynamic_obstacles must be false")
    hospital_map = HospitalMap.load(map_path); _, queries = load_queries(queries_path)
    if query_ids:
        queries = [query for query in queries if query.query_id in set(query_ids)]
        if not queries: raise ValueError("no requested query IDs exist")
    artifact = load_topology(topology_path, hospital_map, FOOTPRINT, padding_m=float(protocol.get("footprint_padding_m", 0.05)), safety_margin_m=float(protocol.get("additional_safety_margin_m", 0.05)), allow_unknown=False)
    manifest = {"schema_version": 8, "experiment": "hospital_stage8b_lateral_preference", "map_yaml": str(map_path), "protocol": str(protocol_file), "queries": str(queries_path), "topology": str(topology_path), "dynamic_obstacles": False, "preference_weights": list(WEIGHTS), "right_wall_target_m": 0.40, "narrow_width_threshold_m": 1.23}
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
    protocol_copy = dict(protocol); protocol_copy.update({"stage8b_preference_weights": list(WEIGHTS), "right_wall_target_m": 0.40, "narrow_width_threshold_m": 1.23, "allow_in_place_rotation": False, "minimum_turning_radius": 0.40, "maximum_curvature": 2.50, "allow_reverse": True, "reverse_penalty": 2.0, "motion_model": "REEDS_SHEPP", "dynamic_obstacles": False})
    (output / "protocol.yaml").write_text(yaml.safe_dump(protocol_copy, sort_keys=False))
    scan = []; paths = {}
    for query in queries:
        row, points = _run_l2(artifact, query, "none", 0.0, output); scan.append(row)
        if points: paths[str(row["run_id"])] = points
        for mode in ("center", "right_edge"):
            for weight in WEIGHTS:
                row, points = _run_l2(artifact, query, mode, weight, output); scan.append(row)
                if points: paths[str(row["run_id"])] = points
    _write(output / "weight_scan.csv", scan); frame = pd.DataFrame(scan); selected = _select_weights(frame)
    (output / "selected_weights.yaml").write_text(yaml.safe_dump(selected, sort_keys=False))
    l3_rows = _run_selected_l3(frame, paths, selected, hospital_map, protocol, output, l3_repetitions, timeout)
    segment_rows=[]
    for row in l3_rows:
        for index,segment in enumerate(row.pop("segments", [])): segment_rows.append({"run_id":row.get("run_id",""),"query_id":row["query_id"],"preference_mode":row["preference_mode"],"segment_index":index,**segment})
    _write(output / "kinematic_runs.csv", l3_rows); _write(output / "segment_metrics.csv",segment_rows)
    _write(output / "path_metrics.csv", [{key: row.get(key) for key in ("run_id","query_id","preference_mode","preference_weight","result_code","path_length_m","minimum_clearance_m","static_footprint_collision_count","expanded_nodes","generated_nodes","search_space_ratio","online_time_ms","cpu_time_ms","peak_rss_bytes","center_deviation_p50_m","center_deviation_p95_m","right_wall_error_p50_m","right_wall_error_p95_m","correct_side_ratio","preference_active_ratio")} for row in scan])
    failure_codes=sorted(set(str(row.get("result_code","")) for row in scan+l3_rows if row.get("result_code") and row.get("result_code")!="SUCCEEDED")); _write(output/"failure_summary.csv",[{"failure_code":code,"count":sum(str(row.get("result_code",""))==code for row in scan+l3_rows)} for code in failure_codes])
    summaries = []
    for (mode, weight), group in frame.groupby(["preference_mode", "preference_weight"]):
        valid = group[group.final_valid_success.astype(bool)]; summaries.append({"preference_mode": mode, "preference_weight": weight, "count": len(group), "success_count": len(valid), "success_rate": len(valid) / max(1, len(group)), "path_length_mean_m": pd.to_numeric(valid.path_length_m, errors="coerce").mean(), "expanded_nodes_mean": pd.to_numeric(valid.expanded_nodes, errors="coerce").mean(), "online_time_mean_ms": pd.to_numeric(valid.online_time_ms, errors="coerce").mean(), "center_deviation_p50_mean_m": pd.to_numeric(valid.get("center_deviation_p50_m"), errors="coerce").mean() if "center_deviation_p50_m" in valid else None, "right_wall_error_p50_mean_m": pd.to_numeric(valid.get("right_wall_error_p50_m"), errors="coerce").mean() if "right_wall_error_p50_m" in valid else None})
    _write(output / "summary_by_weight.csv", summaries)
    _write(output / "summary_by_query.csv", [{"preference_mode":mode,"query_id":query_id,"count":len(group),"success_count":int(group.final_valid_success.astype(bool).sum()),"success_rate":float(group.final_valid_success.astype(bool).mean())} for (mode,query_id),group in frame.groupby(["preference_mode","query_id"])])
    l3 = pd.DataFrame(l3_rows); acceptance = []
    for mode, group in l3.groupby("preference_mode"):
        valid = group[group.final_valid_success.astype(bool)]; acceptance.append({"preference_mode": mode, "selected_weight": selected[mode], "count": len(group), "success_count": len(valid), "all_query_success_rate": len(valid) / max(1, len(group)), "reachable_query_success_rate": len(valid[valid.query_id != "q04"]) / max(1, len(group[group.query_id != "q04"])), "successful_collision_count": int(pd.to_numeric(valid.static_footprint_collision_count, errors="coerce").fillna(0).sum()), "successful_hard_radius_violation_count": int(pd.to_numeric(valid.hard_radius_violation_count, errors="coerce").fillna(0).sum()), "hybrid_calls": int(pd.to_numeric(group.hybrid_calls, errors="coerce").fillna(0).sum()), "hybrid_success": int(pd.to_numeric(group.hybrid_success, errors="coerce").fillna(0).sum())})
    _write(output / "stage8b_acceptance_summary.csv", acceptance)
    _write_plots(output / "plots", frame, selected)
    return output


def _write_plots(directory: Path, frame: pd.DataFrame, selected: Dict[str, float]) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    for field, filename, ylabel in (("path_length_m", "preference_path_length.png", "path length (m)"), ("online_time_ms", "preference_online_time.png", "online time (ms)"), ("expanded_nodes", "preference_expanded_nodes.png", "expanded nodes")):
        fig, ax = plt.subplots(figsize=(9, 5)); groups=[]; labels=[]
        for (mode, weight), group in frame.groupby(["preference_mode", "preference_weight"]):
            values=pd.to_numeric(group[field],errors="coerce").dropna()
            if len(values): groups.append(values); labels.append(f"{mode}\n{weight:g}")
        if groups: ax.boxplot(groups,tick_labels=labels); ax.tick_params(axis="x",rotation=30)
        ax.set_ylabel(ylabel); ax.grid(True,alpha=.25); fig.tight_layout(); fig.savefig(directory/filename,dpi=140); plt.close(fig)


def build_parser() -> argparse.ArgumentParser:
    parser=argparse.ArgumentParser(description="Stage 8B static lateral preference benchmark"); parser.add_argument("--map",default=str(DEFAULT_MAP)); parser.add_argument("--protocol",default=str(DEFAULT_PROTOCOL)); parser.add_argument("--queries",default=str(DEFAULT_QUERIES)); parser.add_argument("--topology",default=str(DEFAULT_TOPOLOGY)); parser.add_argument("--output-dir",required=True); parser.add_argument("--query-id",action="append"); parser.add_argument("--l3-repetitions",type=int,default=5); parser.add_argument("--timeout",type=float,default=5.0); parser.add_argument("--no-dynamic-obstacles",action="store_true",required=True); return parser


def main(argv=None) -> int:
    args=build_parser().parse_args(argv)
    try:
        output=run(Path(args.output_dir).resolve(),Path(args.map).resolve(),Path(args.protocol).resolve(),Path(args.queries).resolve(),Path(args.topology).resolve(),args.l3_repetitions,args.timeout,args.query_id); print(f"stage8b output: {output}"); return 0
    except Exception as exc:
        print(f"stage8b failed: {exc}",file=sys.stderr); return 2


if __name__ == "__main__": raise SystemExit(main())
