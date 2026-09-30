"""Strict same-query comparison of the 0.1 m and 0.05 m Stage 5 reruns."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List

import numpy as np
import pandas as pd
import yaml


METRICS = [
    "planning_time_ms", "wall_time_ms", "cpu_total_ms", "cpu_percent",
    "planner_rss_peak_bytes", "planner_pss_peak_bytes",
    "stack_rss_peak_bytes", "stack_pss_peak_bytes", "path_length_m",
    "length_over_euclidean", "length_over_navfn",
    "length_over_shortest_observed_valid", "minimum_clearance_m",
    "curvature_p95_per_m", "heading_change_p95_rad", "reverse_distance_m",
    "reverse_ratio",
]


def _summary(frame: pd.DataFrame, groups: List[str]) -> pd.DataFrame:
    rows: List[Dict[str, object]] = []
    for keys, group in frame.groupby(groups, dropna=False):
        keys = keys if isinstance(keys, tuple) else (keys,)
        base = dict(zip(groups, keys))
        for metric in METRICS:
            values = pd.to_numeric(group.get(metric), errors="coerce").dropna()
            rows.append({
                **base, "metric": metric, "count": int(values.count()),
                "total_runs": len(group),
                "action_success_count": int(group["action_success"].sum()),
                "action_success_rate": float(group["action_success"].mean()),
                "static_footprint_valid_count": int(group["static_footprint_valid"].sum()),
                "static_footprint_valid_rate": float(group["static_footprint_valid"].mean()),
                "final_valid_success_count": int(group["final_valid_success"].sum()),
                "final_valid_success_rate": float(group["final_valid_success"].mean()),
                "collision_path_count": int((pd.to_numeric(group.get("footprint_collision_count"), errors="coerce").fillna(0) > 0).sum()),
                "mean": float(values.mean()) if len(values) else None,
                "std": float(values.std(ddof=1)) if len(values) > 1 else (0.0 if len(values) else None),
                "P50": float(values.quantile(0.5)) if len(values) else None,
                "P95": float(values.quantile(0.95)) if len(values) else None,
                "P99": float(values.quantile(0.99)) if len(values) else None,
                "min": float(values.min()) if len(values) else None,
                "max": float(values.max()) if len(values) else None,
            })
    return pd.DataFrame(rows)


def _topology_frame(directory: Path, resolution: float) -> pd.DataFrame:
    precompute = pd.read_csv(directory / "precompute_metrics.csv").iloc[0]
    runs = pd.read_csv(directory / "query_runs.csv")
    rows = []
    for mode, group in runs.groupby("mode"):
        rows.append({
            "resolution": resolution, "mode": mode, "count": len(group),
            "success_count": int(group["final_success"].sum()),
            "success_rate": float(group["final_success"].mean()),
            "fallback_count": int(group["fallback_used"].sum()),
            "corridor_expanded_count": int(group["topology_status"].eq("CORRIDOR_EXPANDED").sum()),
            "collision_path_count": int((group["static_footprint_collision_count"] > 0).sum()),
            "online_time_P50_ms": float(group["total_topology_query_time_ms"].quantile(0.5)),
            "online_time_P95_ms": float(group["total_topology_query_time_ms"].quantile(0.95)),
            "topology_build_wall_time_ms": float(precompute["topology_build_wall_time_ms"]),
            "topology_build_cpu_time_ms": float(precompute["topology_build_cpu_time_ms"]),
            "topology_build_peak_rss_bytes": int(precompute["topology_build_peak_rss_bytes"]),
            "topology_file_size_bytes": int(precompute["topology_file_size_bytes"]),
            "topology_graph_nodes": int(precompute["topology_graph_nodes"]),
            "topology_graph_edges": int(precompute["topology_graph_edges"]),
            "topology_graph_components": int(precompute["topology_graph_components"]),
        })
    return pd.DataFrame(rows)


def _delta_table(summary: pd.DataFrame) -> pd.DataFrame:
    index = ["planner", "config_variant", "metric"]
    fields = ["P50", "P95", "P99", "mean", "action_success_rate", "final_valid_success_rate"]
    low = summary[summary["resolution"].eq(0.1)].set_index(index)
    high = summary[summary["resolution"].eq(0.05)].set_index(index)
    rows = []
    for key in sorted(set(low.index) & set(high.index)):
        old, new = low.loc[key], high.loc[key]
        row = dict(zip(index, key))
        for field in fields:
            before, after = float(old[field]), float(new[field])
            row[f"resolution_010_{field}"] = before
            row[f"resolution_005_{field}"] = after
            row[f"ratio_005_over_010_{field}"] = after / before if before != 0 else None
        rows.append(row)
    return pd.DataFrame(rows)


def _map_comparison(metadata: Path) -> pd.DataFrame:
    data = yaml.safe_load(metadata.read_text())
    return pd.DataFrame([
        {"map_id": "hospital_010_v2", "resolution": 0.1, "width_cells": data["source_size"][0],
         "height_cells": data["source_size"][1], "grid_cells": np.prod(data["source_size"]),
         "occupied_cells": data["occupied_cell_count"], "free_cells": data["free_cell_count"],
         "unknown_cells": data["unknown_cell_count"], "map_sha256": data["source_map_sha256"]},
        {"map_id": "hospital_005", "resolution": 0.05, "width_cells": data["target_size"][0],
         "height_cells": data["target_size"][1], "grid_cells": np.prod(data["target_size"]),
         "occupied_cells": data["derived_occupied_cell_count"], "free_cells": data["derived_free_cell_count"],
         "unknown_cells": data["derived_unknown_cell_count"], "map_sha256": data["derived_map_sha256"]},
    ])


def _plots(output: Path, frame: pd.DataFrame, topology: pd.DataFrame) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plots = output / "plots"; plots.mkdir(parents=True, exist_ok=True)
    frame = frame.copy()
    frame["label"] = frame["planner"] + "/" + frame["config_variant"] + "/" + frame["resolution"].astype(str)
    for fields, filename in [
        (["planning_time_ms", "wall_time_ms"], "strict_time_comparison.png"),
        (["planner_rss_peak_bytes", "stack_rss_peak_bytes"], "strict_memory_comparison.png"),
        (["path_length_m", "minimum_clearance_m", "curvature_p95_per_m"], "strict_path_quality.png"),
    ]:
        fig, axes = plt.subplots(1, len(fields), figsize=(6 * len(fields), 5), squeeze=False)
        for axis, field in zip(axes[0], fields):
            values, labels = [], []
            for label, group in frame.groupby("label"):
                data = pd.to_numeric(group.get(field), errors="coerce").dropna()
                if len(data): values.append(data); labels.append(label)
            if values: axis.boxplot(values, tick_labels=labels); axis.tick_params(axis="x", rotation=30)
            axis.set_title(field); axis.grid(True, alpha=0.25)
        fig.tight_layout(); fig.savefig(plots / filename, dpi=140); plt.close(fig)
    rates = frame.groupby(["resolution", "planner", "config_variant"])[["action_success", "final_valid_success"]].mean()
    axis = rates.plot(kind="bar", figsize=(12, 5)); axis.set_ylim(0, 1); axis.set_ylabel("rate")
    axis.figure.tight_layout(); axis.figure.savefig(plots / "strict_validity_rates.png", dpi=140); plt.close(axis.figure)
    pivot = topology.pivot(index="mode", columns="resolution", values="success_rate")
    axis = pivot.plot(kind="bar", figsize=(10, 5)); axis.set_ylim(0, 1); axis.set_ylabel("success rate")
    axis.figure.tight_layout(); axis.figure.savefig(plots / "strict_topology_success.png", dpi=140); plt.close(axis.figure)


def build_resolution_comparison(
    report_010: str | Path, report_005: str | Path,
    topology_010: str | Path, topology_005: str | Path,
    map_metadata: str | Path, output: str | Path,
) -> Path:
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite resolution comparison: {output}")
    output.mkdir(parents=True, exist_ok=True)
    frames = []
    for report, resolution in ((Path(report_010), 0.1), (Path(report_005), 0.05)):
        frame = pd.read_csv(report / "planner_measured_enriched.csv")
        frame["resolution"] = resolution; frames.append(frame)
    combined = pd.concat(frames, ignore_index=True, sort=False)
    combined.to_csv(output / "strict_measured_runs.csv", index=False)
    summary = _summary(combined, ["resolution", "planner", "config_variant"])
    summary.to_csv(output / "strict_resolution_summary.csv", index=False)
    _summary(combined, ["resolution", "planner", "config_variant", "query_id"]).to_csv(output / "strict_resolution_by_query.csv", index=False)
    _delta_table(summary).to_csv(output / "strict_resolution_deltas.csv", index=False)
    topology = pd.concat([
        _topology_frame(Path(topology_010), 0.1), _topology_frame(Path(topology_005), 0.05),
    ], ignore_index=True)
    topology.to_csv(output / "strict_topology_comparison.csv", index=False)
    _map_comparison(Path(map_metadata)).to_csv(output / "strict_map_comparison.csv", index=False)
    manifest = {
        "schema_version": 2, "query_set_identical": True,
        "query_set": "hospital_005_queries_v2", "dynamic_obstacles": False,
        "resolutions": [0.1, 0.05], "run_mode": "measured",
    }
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
    _plots(output, combined, topology)
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Strict fixed-query 0.1 m / 0.05 m comparison")
    parser.add_argument("--report-010", required=True)
    parser.add_argument("--report-005", required=True)
    parser.add_argument("--topology-010", required=True)
    parser.add_argument("--topology-005", required=True)
    parser.add_argument("--map-metadata", required=True)
    parser.add_argument("--output", required=True)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    output = build_resolution_comparison(
        args.report_010, args.report_005, args.topology_010, args.topology_005,
        args.map_metadata, args.output,
    )
    print(f"Strict resolution comparison: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
