"""Read-only aggregation of independent Hospital planner benchmark runs."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

import pandas as pd


RUN_METRICS = [
    "planning_time_ms",
    "wall_time_ms",
    "cpu_total_ms",
    "cpu_percent",
    "planner_rss_peak_bytes",
    "planner_pss_peak_bytes",
    "stack_rss_peak_bytes",
    "stack_pss_peak_bytes",
]
PATH_METRICS = [
    "path_length_m",
    "euclidean_distance_m",
    "length_over_euclidean",
    "length_over_navfn",
    "length_over_shortest_observed_valid",
    "minimum_clearance_m",
    "footprint_collision_count",
    "heading_change_p95_rad",
    "curvature_p95_per_m",
    "preferred_radius_violation_count",
    "in_place_rotation_count",
    "reverse_distance_m",
    "reverse_ratio",
]
GROUP_KEYS = ["planner", "config_variant"]
QUERY_KEYS = ["planner", "config_variant", "query_id"]
SUMMARY_FIELDS = ["count", "success_count", "success_rate", "mean", "std", "P50", "P95", "P99", "min", "max"]


def build_cross_report(root: str | Path, output: str | Path | None = None) -> Path:
    root = Path(root).resolve()
    output = Path(output).resolve() if output else root / "stage3_summary"
    output.mkdir(parents=True, exist_ok=True)
    frames: List[pd.DataFrame] = []
    for directory in _stage_directories(root):
        runs_path = directory / "planner_runs.csv"
        metrics_path = directory / "path_metrics.csv"
        if not runs_path.exists():
            continue
        runs = pd.read_csv(runs_path)
        runs = runs[runs["run_mode"].eq("measured")].copy()
        runs["planner"] = runs["planner_id"].map(_canonical_planner)
        if metrics_path.exists() and metrics_path.stat().st_size:
            metrics = pd.read_csv(metrics_path)
            metrics = metrics.merge(
                runs[["run_id", "query_id", "planner_id", "config_variant"]],
                on=["run_id", "query_id", "planner_id", "config_variant"],
                how="inner",
            )
            metrics["planner"] = metrics["planner_id"].map(_canonical_planner)
            runs = runs.merge(
                metrics.drop(columns=["planner_id", "config_variant"], errors="ignore"),
                on=["run_id", "query_id", "planner"],
                how="left",
                suffixes=("", "_path"),
            )
        frames.append(runs)
    if not frames:
        raise ValueError(f"no stage3_* planner_runs.csv files found under {root}")
    combined = pd.concat(frames, ignore_index=True, sort=False)
    combined = _recompute_cross_planner_ratios(combined)
    _write_summary(output / "baseline_summary.csv", combined, GROUP_KEYS, RUN_METRICS + PATH_METRICS)
    _write_summary(output / "baseline_by_query.csv", combined, QUERY_KEYS, RUN_METRICS + PATH_METRICS)
    _write_failures(output / "baseline_failure_summary.csv", root)
    _plot_comparison(output, combined)
    return output


def _stage_directories(root: Path) -> List[Path]:
    return sorted(path for path in root.glob("stage3_*") if path.is_dir() and path.name != "stage3_summary")


def _canonical_planner(value: str) -> str:
    return "navfn" if str(value).startswith("navfn") else "smac_hybrid"


def _recompute_cross_planner_ratios(frame: pd.DataFrame) -> pd.DataFrame:
    """Calculate ratios that cannot be defined inside one planner's run directory."""
    frame = frame.copy()
    lengths = pd.to_numeric(
        frame["path_length_m"] if "path_length_m" in frame else pd.Series(index=frame.index, dtype=float),
        errors="coerce",
    )
    collisions = pd.to_numeric(
        frame["footprint_collision_count"]
        if "footprint_collision_count" in frame
        else pd.Series(0, index=frame.index, dtype=float),
        errors="coerce",
    )
    successful = frame["result_code"].eq("SUCCEEDED") & lengths.notna() & lengths.gt(0)
    valid = successful & collisions.fillna(0).eq(0)
    frame["length_over_navfn"] = float("nan")
    frame["length_over_shortest_observed_valid"] = float("nan")
    if not valid.any():
        return frame

    reference = frame.loc[successful, ["query_id", "config_variant", "planner"]].copy()
    reference["path_length_m"] = lengths.loc[successful]
    navfn_reference = (
        reference[reference["planner"].eq("navfn") & collisions.loc[reference.index].fillna(0).eq(0)]
        .groupby(["query_id", "config_variant"])["path_length_m"]
        .median()
    )
    shortest_reference = frame.loc[valid, ["query_id", "config_variant"]].copy()
    shortest_reference["path_length_m"] = lengths.loc[valid]
    shortest_reference = shortest_reference.groupby(["query_id", "config_variant"])["path_length_m"].min()
    keys = pd.MultiIndex.from_frame(frame[["query_id", "config_variant"]])
    navfn_lengths = navfn_reference.reindex(keys).to_numpy()
    shortest_lengths = shortest_reference.reindex(keys).to_numpy()
    frame.loc[successful, "length_over_navfn"] = (
        lengths.loc[successful] / navfn_lengths[successful.to_numpy()]
    )
    frame.loc[valid, "length_over_shortest_observed_valid"] = (
        lengths.loc[valid] / shortest_lengths[valid.to_numpy()]
    )
    return frame


def _write_summary(path: Path, frame: pd.DataFrame, groups: Sequence[str], metrics: Sequence[str]) -> None:
    rows: List[Dict[str, object]] = []
    for keys, group in frame.groupby(list(groups), dropna=False):
        if not isinstance(keys, tuple):
            keys = (keys,)
        base = dict(zip(groups, keys))
        success = group["result_code"].eq("SUCCEEDED")
        for metric in metrics:
            if metric not in group.columns:
                continue
            values = pd.to_numeric(group[metric], errors="coerce").dropna()
            row: Dict[str, object] = dict(base)
            row["metric"] = metric
            row["count"] = int(values.count())
            row["success_count"] = int(success.sum())
            row["success_rate"] = float(success.mean()) if len(group) else 0.0
            row["mean"] = float(values.mean()) if len(values) else None
            row["std"] = float(values.std(ddof=1)) if len(values) > 1 else (0.0 if len(values) else None)
            row["P50"] = float(values.quantile(0.50)) if len(values) else None
            row["P95"] = float(values.quantile(0.95)) if len(values) else None
            row["P99"] = float(values.quantile(0.99)) if len(values) else None
            row["min"] = float(values.min()) if len(values) else None
            row["max"] = float(values.max()) if len(values) else None
            rows.append(row)
    pd.DataFrame(rows).to_csv(path, index=False)


def _write_failures(path: Path, root: Path) -> None:
    rows: List[Dict[str, object]] = []
    for directory in _stage_directories(root):
        runs_path = directory / "planner_runs.csv"
        if not runs_path.exists():
            continue
        runs = pd.read_csv(runs_path)
        failures = runs[~runs["result_code"].eq("SUCCEEDED")].copy()
        if failures.empty:
            continue
        failures["planner"] = failures["planner_id"].map(_canonical_planner)
        rows.append(failures[["planner", "config_variant", "query_id", "run_mode", "result_code"]])
    if rows:
        summary = (
            pd.concat(rows, ignore_index=True)
            .groupby(["planner", "config_variant", "query_id", "run_mode", "result_code"], dropna=False)
            .size()
            .rename("count")
            .reset_index()
        )
        summary.to_csv(path, index=False)
    else:
        pd.DataFrame(
            columns=["planner", "config_variant", "query_id", "run_mode", "result_code", "count"]
        ).to_csv(path, index=False)


def _plot_comparison(output: Path, frame: pd.DataFrame) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return

    frame = frame.copy()
    frame["label"] = frame["planner"] + "/" + frame["config_variant"]
    order = sorted(frame["label"].dropna().unique())

    def comparison_plot(filename: str, columns: Sequence[str], titles: Sequence[str], ylabels: Sequence[str]) -> None:
        figure, axes = plt.subplots(1, len(columns), figsize=(6 * len(columns), 5), squeeze=False)
        for axis, column, title, ylabel in zip(axes[0], columns, titles, ylabels):
            values, labels = [], []
            if column in frame:
                for label in order:
                    subset = pd.to_numeric(
                        frame.loc[frame["label"].eq(label), column], errors="coerce"
                    ).dropna()
                    if len(subset):
                        values.append(subset.to_numpy())
                        labels.append(label)
            if values:
                # Matplotlib 3.6 uses ``labels``; newer releases renamed the
                # keyword to ``tick_labels``.  The positional data and labels
                # are otherwise identical, so use the compatible spelling.
                axis.boxplot(values, labels=labels, patch_artist=True)
                axis.tick_params(axis="x", rotation=30)
            axis.set_title(title)
            axis.set_ylabel(ylabel)
            axis.grid(True, alpha=0.25)
        figure.tight_layout()
        figure.savefig(output / filename, dpi=140)
        plt.close(figure)

    comparison_plot(
        "baseline_comparison.png",
        ["planning_time_ms", "wall_time_ms"],
        ["Server planning time", "Client wall time"],
        ["milliseconds", "milliseconds"],
    )
    comparison_plot(
        "baseline_memory_comparison.png",
        ["planner_rss_peak_bytes", "stack_rss_peak_bytes"],
        ["Planner peak RSS", "Planning stack peak RSS"],
        ["bytes", "bytes"],
    )
    comparison_plot(
        "baseline_path_quality.png",
        ["path_length_m", "minimum_clearance_m", "curvature_p95_per_m"],
        ["Path length", "Minimum clearance", "P95 curvature"],
        ["meters", "meters", "1/meters"],
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Aggregate stage 3 Hospital planner benchmark results")
    parser.add_argument("--root", required=True, help="Hospital benchmark root")
    parser.add_argument("--output", default=None, help="Output directory; defaults to <root>/stage3_summary")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    print(f"stage3 summary: {build_cross_report(args.root, args.output)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
