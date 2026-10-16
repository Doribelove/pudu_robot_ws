from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable, List, Sequence

import pandas as pd

from .path_metrics import load_path


STAT_COLUMNS = ["count", "success_count", "success_rate", "P50", "P95", "P99", "mean", "std", "min", "max"]


def generate_report(run_dir: str | Path) -> Path:
    run_dir = Path(run_dir).resolve()
    plots_dir = run_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    runs = pd.read_csv(run_dir / "planner_runs.csv")
    measured_runs = runs[runs["run_mode"].eq("measured")].copy() if "run_mode" in runs.columns else runs
    metrics_path = run_dir / "path_metrics.csv"
    metrics = pd.read_csv(metrics_path) if metrics_path.exists() and metrics_path.stat().st_size else pd.DataFrame()
    if not metrics.empty and "run_id" in metrics.columns:
        combined = measured_runs.merge(metrics, on=["run_id", "query_id", "planner_id", "config_variant"], how="left", suffixes=("", "_path"))
    else:
        combined = measured_runs
    _write_summary(run_dir / "summary_by_planner.csv", combined, ["planner_id", "config_variant"])
    _write_summary(run_dir / "summary_by_query.csv", combined, ["query_id", "planner_id", "config_variant"])
    _plot_reports(plots_dir, run_dir, measured_runs, metrics)
    return run_dir


def _write_summary(path: Path, runs: pd.DataFrame, groups: Sequence[str]) -> None:
    numeric = [
        "planning_time_ms", "wall_time_ms", "cpu_total_ms", "cpu_percent",
        "planner_rss_peak_bytes", "planner_pss_peak_bytes", "stack_rss_peak_bytes", "stack_pss_peak_bytes",
    ]
    numeric.extend(column for column in runs.columns if column not in {"run_id", *groups, "result_code"} and pd.api.types.is_numeric_dtype(runs[column]))
    numeric = list(dict.fromkeys(numeric))
    rows = []
    if runs.empty:
        path.write_text("")
        return
    for keys, group in runs.groupby(list(groups), dropna=False):
        if not isinstance(keys, tuple):
            keys = (keys,)
        base = dict(zip(groups, keys))
        run_success = group["result_code"].eq("SUCCEEDED")
        for metric in numeric:
            source = group[metric] if metric in group.columns else pd.Series(dtype=float)
            values = pd.to_numeric(source, errors="coerce").dropna()
            row = dict(base)
            row["metric"] = metric
            row["count"] = int(values.count())
            row["success_count"] = int(run_success.sum())
            row["success_rate"] = float(run_success.mean()) if len(group) else 0.0
            row["P50"] = float(values.quantile(0.50)) if len(values) else None
            row["P95"] = float(values.quantile(0.95)) if len(values) else None
            row["P99"] = float(values.quantile(0.99)) if len(values) else None
            row["mean"] = float(values.mean()) if len(values) else None
            row["std"] = float(values.std(ddof=1)) if len(values) > 1 else 0.0 if len(values) else None
            row["min"] = float(values.min()) if len(values) else None
            row["max"] = float(values.max()) if len(values) else None
            rows.append(row)
    pd.DataFrame(rows).to_csv(path, index=False)


def _plot_reports(plots_dir: Path, run_dir: Path, runs: pd.DataFrame, metrics: pd.DataFrame) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return

    def finish(name: str, title: str) -> tuple:
        figure, axis = plt.subplots(figsize=(8, 5))
        axis.set_title(title)
        axis.grid(True, alpha=0.25)
        return figure, axis

    figure, axis = finish("", "Planning time distribution")
    _box_by_group(axis, runs, "planning_time_ms")
    axis.set_ylabel("ms")
    figure.tight_layout(); figure.savefig(plots_dir / "planning_time_distribution.png", dpi=140); plt.close(figure)

    figure, axis = finish("", "Wall time vs server planning time")
    if not runs.empty:
        for name, group in runs.groupby(["planner_id", "config_variant"]):
            axis.scatter(group["planning_time_ms"], group["wall_time_ms"], label="/".join(name), alpha=0.8)
        axis.legend(fontsize=8)
    axis.set_xlabel("planning_time_ms"); axis.set_ylabel("wall_time_ms")
    figure.tight_layout(); figure.savefig(plots_dir / "wall_vs_planning_time.png", dpi=140); plt.close(figure)

    for field, filename, title, ylabel in [
        ("cpu_total_ms", "cpu_time_distribution.png", "Planner CPU time", "ms"),
        ("planner_rss_peak_bytes", "planner_peak_memory.png", "Planner peak RSS", "bytes"),
        ("stack_rss_peak_bytes", "stack_peak_memory.png", "Planning stack peak RSS", "bytes"),
    ]:
        figure, axis = finish("", title)
        _box_by_group(axis, runs, field)
        axis.set_ylabel(ylabel)
        figure.tight_layout(); figure.savefig(plots_dir / filename, dpi=140); plt.close(figure)

    figure, axis = finish("", "Success rate")
    if not runs.empty:
        rates = runs.assign(success=runs["result_code"].eq("SUCCEEDED")).groupby(["planner_id", "config_variant"])["success"].mean()
        rates.plot(kind="bar", ax=axis)
        axis.set_ylim(0, 1)
    axis.set_ylabel("rate")
    figure.tight_layout(); figure.savefig(plots_dir / "success_rate.png", dpi=140); plt.close(figure)

    for field, filename, title in [
        ("length_over_euclidean", "path_length_ratio.png", "Path length / Euclidean distance"),
        ("minimum_clearance_m", "minimum_clearance.png", "Minimum clearance"),
        ("curvature_p95_per_m", "curvature_distribution.png", "P95 curvature"),
    ]:
        figure, axis = finish("", title)
        _box_by_group(axis, metrics, field)
        figure.tight_layout(); figure.savefig(plots_dir / filename, dpi=140); plt.close(figure)

    _plot_paths(plots_dir / "hospital_paths_overlay.png", run_dir, runs, title="Hospital paths")
    _plot_paths(plots_dir / "navfn_vs_smac_paths.png", run_dir, runs, title="NavFn vs Smac paths", planners_only=True)


def _box_by_group(axis, frame: pd.DataFrame, field: str) -> None:
    if frame.empty or field not in frame.columns:
        return
    values = []
    labels = []
    for name, group in frame.groupby(["planner_id", "config_variant"]):
        series = pd.to_numeric(group[field], errors="coerce").dropna()
        if len(series):
            values.append(series.to_numpy())
            labels.append("/".join(name))
    if values:
        axis.boxplot(values, labels=labels, patch_artist=True)
        axis.tick_params(axis="x", rotation=30)


def _plot_paths(path: Path, run_dir: Path, runs: pd.DataFrame, *, title: str, planners_only: bool = False) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    figure, axis = plt.subplots(figsize=(8, 8))
    try:
        import yaml
        from PIL import Image
        map_row = pd.read_csv(run_dir / "maps.csv").iloc[0]
        map_yaml = Path(map_row["map_yaml"])
        config = yaml.safe_load(map_yaml.read_text())
        image_path = map_yaml.parent / config["image"]
        image = Image.open(image_path).convert("L")
        origin = config["origin"]
        resolution = float(config["resolution"])
        axis.imshow(image, cmap="gray", origin="upper", extent=[origin[0], origin[0] + image.width * resolution, origin[1], origin[1] + image.height * resolution], alpha=0.35)
    except (OSError, KeyError, TypeError, ValueError, IndexError):
        pass
    if not runs.empty:
        successful = runs[runs["result_code"] == "SUCCEEDED"]
        if planners_only:
            successful = successful[successful["planner_id"].str.contains("navfn|smac", case=False, regex=True)]
        for _, row in successful.iterrows():
            path_file = row.get("path_file", "")
            if not path_file:
                continue
            try:
                points = load_path(run_dir / path_file)
            except (OSError, ValueError, EOFError, KeyError):
                continue
            label = f"{row['planner_id']}/{row['config_variant']}"
            axis.plot([point["x"] for point in points], [point["y"] for point in points], alpha=0.55, label=label)
    axis.set_title(title); axis.set_aspect("equal", adjustable="box"); axis.set_xlabel("x (m)"); axis.set_ylabel("y (m)")
    handles, labels = axis.get_legend_handles_labels()
    if handles:
        unique = dict(zip(labels, handles)); axis.legend(unique.values(), unique.keys(), fontsize=7)
    figure.tight_layout(); figure.savefig(path, dpi=140); plt.close(figure)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Aggregate planner benchmark CSV and path outputs")
    parser.add_argument("--dir", required=True)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    print(f"benchmark report: {generate_report(args.dir)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
