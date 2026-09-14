"""Read-only Stage 5 aggregation for the fixed 0.05 m Hospital benchmark."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

import numpy as np
import pandas as pd
import yaml

from .planner_benchmark.config import load_protocol, load_queries, resolve_path
from .planner_benchmark.map_utils import HospitalMap
from .topology import astar_grid, attach_pose, load_topology, search_topology


RUN_METRICS = [
    "planning_time_ms", "wall_time_ms", "cpu_total_ms", "cpu_percent",
    "planner_rss_peak_bytes", "planner_pss_peak_bytes",
    "stack_rss_peak_bytes", "stack_pss_peak_bytes",
]
PATH_METRICS = [
    "path_length_m", "euclidean_distance_m", "length_over_euclidean",
    "length_over_navfn", "length_over_shortest_observed_valid",
    "minimum_clearance_m", "clearance_p05_m", "clearance_p50_m",
    "footprint_collision_count", "heading_change_p95_rad",
    "curvature_p95_per_m", "preferred_radius_violation_count",
    "in_place_rotation_count", "reverse_distance_m", "reverse_ratio",
]
PLANNER_DIRECTORIES = (
    "stage5_navfn_product", "stage5_navfn_normalized",
    "stage5_smac_product", "stage5_smac_normalized",
)


def validity_columns(frame: pd.DataFrame) -> pd.DataFrame:
    frame = frame.copy()
    collision = pd.to_numeric(frame.get("footprint_collision_count"), errors="coerce")
    frame["action_success"] = frame["result_code"].eq("SUCCEEDED")
    frame["static_footprint_valid"] = frame["action_success"] & collision.fillna(1).eq(0)
    frame["final_valid_success"] = frame["action_success"] & frame["static_footprint_valid"]
    return frame


def _load_planner_runs(root: Path) -> pd.DataFrame:
    frames: List[pd.DataFrame] = []
    for name in PLANNER_DIRECTORIES:
        directory = root / name
        runs_path = directory / "planner_runs.csv"
        metrics_path = directory / "path_metrics.csv"
        if not runs_path.exists():
            raise ValueError(f"missing formal planner result: {runs_path}")
        runs = pd.read_csv(runs_path)
        runs = runs[runs["run_mode"].eq("measured")].copy()
        if len(runs) != 50 or runs["query_id"].nunique() != 10:
            raise ValueError(f"incomplete measured result in {directory}: rows={len(runs)}")
        metrics = pd.read_csv(metrics_path) if metrics_path.exists() and metrics_path.stat().st_size else pd.DataFrame()
        if not metrics.empty:
            metrics = metrics.drop(columns=["planner_id", "config_variant"], errors="ignore")
            runs = runs.merge(metrics, on=["run_id", "query_id"], how="left", validate="one_to_one")
        runs["planner"] = np.where(runs["planner_id"].str.startswith("navfn"), "navfn", "smac_hybrid")
        frames.append(runs)
    combined = validity_columns(pd.concat(frames, ignore_index=True, sort=False))
    return _recompute_path_ratios(combined)


def _recompute_path_ratios(frame: pd.DataFrame) -> pd.DataFrame:
    frame = frame.copy()
    length = pd.to_numeric(frame.get("path_length_m"), errors="coerce")
    valid = frame["final_valid_success"] & length.notna() & length.gt(0)
    frame["length_over_navfn"] = np.nan
    frame["length_over_shortest_observed_valid"] = np.nan
    navfn = frame.loc[valid & frame["planner"].eq("navfn"), ["query_id", "config_variant"]].copy()
    navfn["length"] = length.loc[navfn.index]
    navfn_reference = navfn.groupby(["query_id", "config_variant"])["length"].median()
    shortest = frame.loc[valid, ["query_id", "config_variant"]].copy()
    shortest["length"] = length.loc[shortest.index]
    shortest_reference = shortest.groupby(["query_id", "config_variant"])["length"].min()
    keys = pd.MultiIndex.from_frame(frame[["query_id", "config_variant"]])
    navfn_values = navfn_reference.reindex(keys).to_numpy()
    shortest_values = shortest_reference.reindex(keys).to_numpy()
    navfn_available = valid & pd.notna(navfn_values)
    frame.loc[navfn_available, "length_over_navfn"] = length.loc[navfn_available] / navfn_values[navfn_available]
    frame.loc[valid, "length_over_shortest_observed_valid"] = length.loc[valid] / shortest_values[valid]
    return frame


def _stat_rows(frame: pd.DataFrame, groups: Sequence[str], metrics: Iterable[str]) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    for keys, group in frame.groupby(list(groups), dropna=False):
        keys = keys if isinstance(keys, tuple) else (keys,)
        base = dict(zip(groups, keys))
        for metric in metrics:
            if metric not in group:
                continue
            values = pd.to_numeric(group[metric], errors="coerce").dropna()
            rows.append({
                **base, "metric": metric, "count": int(values.count()),
                "total_runs": int(len(group)),
                "action_success_count": int(group["action_success"].sum()),
                "action_success_rate": float(group["action_success"].mean()),
                "static_footprint_valid_count": int(group["static_footprint_valid"].sum()),
                "static_footprint_valid_rate": float(group["static_footprint_valid"].mean()),
                "final_valid_success_count": int(group["final_valid_success"].sum()),
                "final_valid_success_rate": float(group["final_valid_success"].mean()),
                "collision_path_count": int((pd.to_numeric(group.get("footprint_collision_count"), errors="coerce").fillna(0) > 0).sum()),
                "mean": float(values.mean()) if len(values) else None,
                "std": float(values.std(ddof=1)) if len(values) > 1 else (0.0 if len(values) else None),
                "P50": float(values.quantile(0.50)) if len(values) else None,
                "P95": float(values.quantile(0.95)) if len(values) else None,
                "P99": float(values.quantile(0.99)) if len(values) else None,
                "min": float(values.min()) if len(values) else None,
                "max": float(values.max()) if len(values) else None,
            })
    return rows


def _endpoint_diagnostics(
    protocol_path: Path,
    queries_path: Path,
    topology_dir: Path,
    planner_frame: pd.DataFrame,
) -> pd.DataFrame:
    protocol_file, protocol = load_protocol(protocol_path)
    _, queries = load_queries(queries_path)
    hospital_map = HospitalMap.load(resolve_path(protocol["map_yaml"], base=protocol_file.parent))
    footprint = protocol["footprint"]
    artifact = load_topology(
        topology_dir, hospital_map, footprint,
        padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    navfn = planner_frame[
        planner_frame["planner"].eq("navfn") & planner_frame["config_variant"].eq("product")
    ]
    rows = []
    for query in queries:
        validation = hospital_map.validate_query(query, footprint, 0.5, allow_unknown=False)
        start_cell = hospital_map.world_to_cell(query.start[0], query.start[1])
        goal_cell = hospital_map.world_to_cell(query.goal[0], query.goal[1])
        full_path = astar_grid(artifact.free_mask, start_cell, goal_cell) if start_cell and goal_cell else None
        start_attach = attach_pose(artifact, query.start, footprint, max_radius_m=5.0, allow_unknown=False)
        goal_attach = attach_pose(artifact, query.goal, footprint, max_radius_m=5.0, allow_unknown=False)
        topology_route = None
        topology_result = "TOPOLOGY_COMPONENT_MISMATCH"
        if not start_attach:
            topology_result = "TOPOLOGY_START_NOT_ATTACHABLE"
        elif not goal_attach:
            topology_result = "TOPOLOGY_GOAL_NOT_ATTACHABLE"
        elif start_attach.component_id == goal_attach.component_id:
            topology_route = search_topology(artifact, start_attach.node_id, goal_attach.node_id)
            topology_result = "SUCCEEDED" if topology_route else "TOPOLOGY_NO_ROUTE"
        navfn_success = bool(navfn[navfn["query_id"].eq(query.query_id)]["action_success"].any())
        if navfn_success and full_path is None:
            failure = "STATIC_SEMANTICS_CONSERVATIVE_INFLATION_MISMATCH"
        elif full_path is None:
            failure = "FULL_GRID_FAILED"
        elif topology_route is None:
            failure = topology_result
        else:
            failure = ""
        rows.append({
            "query_id": query.query_id,
            "start_clearance": validation.start_clearance_m,
            "goal_clearance": validation.goal_clearance_m,
            "start_valid": validation.start_status == "VALID",
            "goal_valid": validation.goal_status == "VALID",
            "start_component": start_attach.component_id if start_attach else None,
            "goal_component": goal_attach.component_id if goal_attach else None,
            "full_grid_result": "SUCCEEDED" if full_path else "FULL_GRID_FAILED",
            "topology_result": topology_result,
            "navfn_action_success": navfn_success,
            "failure_code": failure,
            "map_resolution": hospital_map.resolution,
            "inflation_radius": float(protocol["variants"]["product"]["inflation_radius"]),
            "topology_footprint_padding_m": 0.05,
            "topology_safety_margin_m": 0.05,
            "allow_unknown": False,
        })
    return pd.DataFrame(rows)


def _topology_summary(topology_run_dir: Path) -> pd.DataFrame:
    runs = pd.read_csv(topology_run_dir / "query_runs.csv")
    rows = []
    for mode, group in runs.groupby("mode"):
        rows.append({
            "mode": mode, "count": len(group),
            "success_count": int(group["final_success"].sum()),
            "success_rate": float(group["final_success"].mean()),
            "collision_path_count": int((group["static_footprint_collision_count"] > 0).sum()),
            "fallback_count": int(group["fallback_used"].sum()),
            "corridor_expanded_count": int(group["topology_status"].eq("CORRIDOR_EXPANDED").sum()),
            "P50_total_query_time_ms": float(group["total_topology_query_time_ms"].quantile(0.5)),
            "P95_total_query_time_ms": float(group["total_topology_query_time_ms"].quantile(0.95)),
            "P99_total_query_time_ms": float(group["total_topology_query_time_ms"].quantile(0.99)),
        })
    return pd.DataFrame(rows)


def _resolution_comparison(stage3_summary: Path, current_summary: pd.DataFrame) -> pd.DataFrame:
    if not stage3_summary.exists():
        return pd.DataFrame()
    old = pd.read_csv(stage3_summary)
    new = current_summary.copy()
    rows = []
    for _, row in new.iterrows():
        match = old[
            old["planner"].eq(row["planner"])
            & old["config_variant"].eq(row["config_variant"])
            & old["metric"].eq(row["metric"])
        ]
        if match.empty:
            continue
        previous = match.iloc[0]
        rows.append({
            "planner": row["planner"], "config_variant": row["config_variant"],
            "metric": row["metric"], "hospital_010_P50": previous.get("P50"),
            "hospital_010_P95": previous.get("P95"), "hospital_010_P99": previous.get("P99"),
            "hospital_005_v2_P50": row.get("P50"), "hospital_005_v2_P95": row.get("P95"),
            "hospital_005_v2_P99": row.get("P99"),
            "strict_query_set_comparison": False,
            "comparison_note": "descriptive_only_stage3_uses_frozen_v1_queries_v2_requires_010_rerun",
        })
    return pd.DataFrame(rows)


def _plots(output: Path, frame: pd.DataFrame, topology: pd.DataFrame) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plots = output / "plots"
    plots.mkdir(parents=True, exist_ok=True)
    frame = frame.copy()
    frame["label"] = frame["planner"] + "/" + frame["config_variant"]
    for fields, filename, titles in [
        (["planning_time_ms", "wall_time_ms", "cpu_total_ms"], "planner_time_comparison.png", ["Planning", "Wall", "CPU"]),
        (["planner_rss_peak_bytes", "stack_rss_peak_bytes"], "planner_memory_comparison.png", ["Planner RSS", "Stack RSS"]),
        (["path_length_m", "minimum_clearance_m", "curvature_p95_per_m"], "path_quality_comparison.png", ["Length", "Clearance", "Curvature P95"]),
    ]:
        figure, axes = plt.subplots(1, len(fields), figsize=(6 * len(fields), 5), squeeze=False)
        for axis, field, title in zip(axes[0], fields, titles):
            data, labels = [], []
            for label, group in frame.groupby("label"):
                values = pd.to_numeric(group.get(field), errors="coerce").dropna()
                if len(values): data.append(values); labels.append(label)
            if data: axis.boxplot(data, tick_labels=labels); axis.tick_params(axis="x", rotation=25)
            axis.set_title(title); axis.grid(True, alpha=0.25)
        figure.tight_layout(); figure.savefig(plots / filename, dpi=140); plt.close(figure)
    rates = frame.groupby("label")[["action_success", "static_footprint_valid", "final_valid_success"]].mean()
    axis = rates.plot(kind="bar", figsize=(10, 5)); axis.set_ylim(0, 1); axis.set_ylabel("rate")
    axis.figure.tight_layout(); axis.figure.savefig(plots / "validity_rates.png", dpi=140); plt.close(axis.figure)
    axis = topology.set_index("mode")[["success_rate"]].plot(kind="bar", figsize=(9, 5), legend=False)
    axis.set_ylim(0, 1); axis.set_ylabel("success rate"); axis.figure.tight_layout()
    axis.figure.savefig(plots / "topology_success.png", dpi=140); plt.close(axis.figure)


def build_stage5_report(
    planner_root: str | Path,
    topology_run_dir: str | Path,
    protocol: str | Path,
    queries: str | Path,
    output: str | Path,
    stage3_summary: str | Path | None = None,
) -> Path:
    planner_root = Path(planner_root).resolve()
    topology_run_dir = Path(topology_run_dir).resolve()
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite Stage 5 report: {output}")
    output.mkdir(parents=True, exist_ok=True)
    combined = _load_planner_runs(planner_root)
    combined.to_csv(output / "planner_measured_enriched.csv", index=False)
    summary = pd.DataFrame(_stat_rows(combined, ["planner", "config_variant"], RUN_METRICS + PATH_METRICS))
    summary.to_csv(output / "baseline_summary_v2.csv", index=False)
    pd.DataFrame(_stat_rows(combined, ["planner", "config_variant", "query_id"], RUN_METRICS + PATH_METRICS)).to_csv(output / "baseline_by_query_v2.csv", index=False)
    failures = combined[~combined["final_valid_success"]].copy()
    failures.groupby(["planner", "config_variant", "query_id", "result_code", "action_success", "static_footprint_valid"], dropna=False).size().rename("count").reset_index().to_csv(output / "failure_summary_v2.csv", index=False)
    diagnostics = _endpoint_diagnostics(Path(protocol), Path(queries), topology_run_dir / "topology", combined)
    diagnostics.to_csv(output / "endpoint_diagnostics.csv", index=False)
    topology = _topology_summary(topology_run_dir)
    topology.to_csv(output / "topology_summary_v2.csv", index=False)
    pd.read_csv(topology_run_dir / "fallback_summary.csv").to_csv(output / "topology_fallback_summary_v2.csv", index=False)
    pd.read_csv(topology_run_dir / "precompute_metrics.csv").to_csv(output / "topology_precompute_v2.csv", index=False)
    comparison = _resolution_comparison(Path(stage3_summary), summary) if stage3_summary else pd.DataFrame()
    comparison.to_csv(output / "resolution_comparison_descriptive.csv", index=False)
    _, report_protocol = load_protocol(protocol)
    manifest = {
        "schema_version": 2, "map": str(report_protocol["map"]),
        "resolution": float(report_protocol["resolution"]),
        "dynamic_obstacles": False, "query_set": str(Path(queries).resolve()),
        "strict_010_005_query_comparison_available": False,
        "strict_comparison_requirement": "rerun queries_v2.yaml on hospital 0.1 m",
        "planner_directories": list(PLANNER_DIRECTORIES),
        "topology_run_dir": str(topology_run_dir),
    }
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
    _plots(output, combined, topology)
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build the fixed-resolution Hospital Stage 5 report")
    parser.add_argument("--planner-root", required=True)
    parser.add_argument("--topology-dir", required=True)
    parser.add_argument("--protocol", required=True)
    parser.add_argument("--queries", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--stage3-summary", default=None)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    output = build_stage5_report(
        args.planner_root, args.topology_dir, args.protocol, args.queries,
        args.output, args.stage3_summary,
    )
    print(f"Stage 5 report: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
