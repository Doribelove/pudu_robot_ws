"""Strict, read-only plots for the PLN-02 single-planner benchmark."""

from __future__ import annotations

import argparse
from pathlib import Path
import pandas as pd


MAP_ORDER = [
    "hospital_005",
    "hospital_boundary_100x100_005",
    "hospital_boundary_200x200_005",
    "hospital_boundary_400x400_005",
]
MAP_LABELS = {
    "hospital_005": "80x80 m",
    "hospital_boundary_100x100_005": "100x100 m",
    "hospital_boundary_200x200_005": "200x200 m",
    "hospital_boundary_400x400_005": "400x400 m",
}
ALGORITHM_LABELS = {
    "astar": "8-neighbor A*",
    "hybrid_astar": "SE(2) Hybrid A*",
    "rrt_star_dubins_surrogate": "RRT* Dubins surrogate",
    "kinodynamic_rrt_star_bicycle": "Kinodynamic RRT* bicycle",
}


def _as_bool(values: pd.Series) -> pd.Series:
    """Parse CSV booleans without treating the string ``False`` as truthy."""
    if values.dtype == bool:
        return values.fillna(False)
    return values.map(
        lambda value: str(value).strip().lower() in {"1", "true", "yes", "y", "t"}
    )


def _load(directory: Path) -> pd.DataFrame:
    runs = pd.read_csv(directory / "runs.csv")
    metrics_path = directory / "path_metrics.csv"
    metrics = pd.read_csv(metrics_path) if metrics_path.exists() else pd.DataFrame()
    measured = runs[runs["run_mode"].astype(str).eq("measured")].copy()
    if not metrics.empty:
        join = ["run_id", "map_id", "query_id", "algorithm", "run_mode"]
        fields = [field for field in metrics.columns if field not in join]
        measured = measured.merge(metrics[join + fields], on=join, how="left", suffixes=("", "_metric"))
    # Older path_metrics.csv files do not contain derived path ratios.  Keep
    # the raw files untouched and derive the ratio for plotting here.
    if "length_over_euclidean" not in measured.columns:
        if {"path_length_m", "euclidean_distance_m"}.issubset(measured.columns):
            length = pd.to_numeric(measured["path_length_m"], errors="coerce")
            euclidean = pd.to_numeric(measured["euclidean_distance_m"], errors="coerce")
            valid = euclidean > 1e-9
            measured["length_over_euclidean"] = length.where(valid) / euclidean.where(valid)
        else:
            measured["length_over_euclidean"] = float("nan")
    return measured


def _maps(directory: Path, frame: pd.DataFrame) -> pd.DataFrame:
    maps = pd.read_csv(directory / "maps.csv")
    maps["grid_cells"] = pd.to_numeric(maps["width_cells"], errors="coerce") * pd.to_numeric(maps["height_cells"], errors="coerce")
    maps["physical_area_m2"] = pd.to_numeric(maps["physical_area_m2"], errors="coerce")
    return frame.merge(maps[["map_id", "grid_cells", "physical_area_m2"]], on="map_id", how="left")


def _save(fig, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    import matplotlib.pyplot as plt
    plt.close(fig)


def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def _quantile_table(frame: pd.DataFrame, field: str) -> pd.DataFrame:
    columns = ["map_id", "algorithm", "P50", "P95", "P99"]
    if field not in frame.columns:
        return pd.DataFrame(columns=columns)
    rows = []
    for (map_id, algorithm), group in frame.groupby(["map_id", "algorithm"], sort=False):
        values = pd.to_numeric(group[field], errors="coerce").dropna()
        rows.append({
            "map_id": map_id,
            "algorithm": algorithm,
            "P50": float(values.quantile(.50)) if len(values) else None,
            "P95": float(values.quantile(.95)) if len(values) else None,
            "P99": float(values.quantile(.99)) if len(values) else None,
        })
    return pd.DataFrame(rows, columns=columns)


def _scale_lines(frame: pd.DataFrame, field: str, ylabel: str, title: str, path: Path, *, unit_scale: float = 1.0) -> None:
    plt = _mpl()
    table = _quantile_table(frame, field)
    fig, axes = plt.subplots(1, 3, figsize=(16, 5), sharex=True)
    scale = frame[["map_id", "grid_cells", "physical_area_m2"]].drop_duplicates().set_index("map_id").reindex(MAP_ORDER)
    for axis, quantile in zip(axes, ("P50", "P95", "P99")):
        for algorithm, group in table.groupby("algorithm", sort=False):
            group = group.set_index("map_id").reindex(MAP_ORDER)
            y = pd.to_numeric(group[quantile], errors="coerce") * unit_scale
            x_values = pd.to_numeric(scale["grid_cells"], errors="coerce")
            valid = y.notna() & x_values.notna()
            if valid.any():
                axis.plot(x_values.loc[valid], y.loc[valid], marker="o",
                          label=ALGORITHM_LABELS.get(algorithm, algorithm))
        axis.set_title(quantile)
        axis.set_xlabel("grid cells")
        axis.set_xscale("log")
        axis.grid(alpha=.25)
    axes[0].set_ylabel(ylabel)
    if axes[-1].get_legend_handles_labels()[0]:
        axes[-1].legend(fontsize=8)
    fig.suptitle(title)
    _save(fig, path)


def _success_plot(frame: pd.DataFrame, path: Path) -> None:
    plt = _mpl()
    rows = []
    for (map_id, algorithm), group in frame.groupby(["map_id", "algorithm"], sort=False):
        rows.append({
            "map_id": map_id, "algorithm": algorithm,
            "planner_success": _as_bool(group["planner_success"]).mean(),
            "static_footprint_valid": _as_bool(group["static_footprint_valid"]).mean(),
            "kinematic_valid": _as_bool(group["kinematic_valid"]).mean(),
            "final_valid_success": _as_bool(group["final_valid_success"]).mean(),
        })
    table = pd.DataFrame(rows)
    scale = frame[["map_id", "grid_cells"]].drop_duplicates().set_index("map_id").reindex(MAP_ORDER)
    fig, axes = plt.subplots(1, 4, figsize=(18, 5), sharex=True, sharey=True)
    for axis, algorithm in zip(axes, sorted(table.algorithm.unique())):
        group = table[table.algorithm == algorithm].set_index("map_id").reindex(MAP_ORDER)
        for field in ("planner_success", "static_footprint_valid", "kinematic_valid", "final_valid_success"):
            axis.plot(scale["grid_cells"], group[field], marker="o", label=field)
        axis.set_title(ALGORITHM_LABELS.get(algorithm, algorithm))
        axis.set_xscale("log"); axis.set_ylim(-.02, 1.02); axis.grid(alpha=.25); axis.set_xlabel("grid cells")
    axes[0].set_ylabel("rate")
    axes[-1].legend(fontsize=7, loc="lower left")
    fig.suptitle("Measured success stages vs map scale")
    _save(fig, path)


def _path_quality(frame: pd.DataFrame, path: Path) -> None:
    plt = _mpl()
    fields = [
        ("path_length_m", "path length (m)"),
        ("length_over_euclidean", "length / Euclidean"),
        ("minimum_clearance_m", "minimum clearance (m)"),
        ("curvature_p95_per_m", "curvature P95 (1/m)"),
        ("heading_change_p95_rad", "heading change P95 (rad)"),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    for axis, (field, ylabel) in zip(axes.flat, fields):
        eligible = frame[_as_bool(frame["planner_success"])].copy()
        values = []
        labels = []
        for algorithm, group in eligible.groupby("algorithm", sort=False):
            if field not in group.columns:
                continue
            data = pd.to_numeric(group[field], errors="coerce").dropna()
            if len(data):
                values.append(data.to_numpy())
                labels.append(ALGORITHM_LABELS.get(algorithm, algorithm))
        if any(len(value) for value in values):
            axis.boxplot(values, tick_labels=labels, showfliers=True)
        else:
            axis.text(.5, .5, "No planner-success paths", ha="center", va="center", transform=axis.transAxes)
        axis.set_title(field + " (planner-success candidates)")
        axis.set_ylabel(ylabel); axis.tick_params(axis="x", rotation=22); axis.grid(alpha=.25)
    axes.flat[-1].axis("off")
    fig.suptitle("Four-algorithm path quality; final-valid paths are tracked separately")
    _save(fig, path)


def _boundary_plot(frame: pd.DataFrame, directory: Path) -> None:
    plt = _mpl()
    status_fields = ["planner_success", "static_footprint_valid", "kinematic_valid", "final_valid_success"]
    for field in status_fields:
        frame[field] = _as_bool(frame[field])
    rates = frame.groupby("algorithm")[status_fields].mean()
    rates.index = [ALGORITHM_LABELS.get(value, value) for value in rates.index]
    fig, ax = plt.subplots(figsize=(12, 6))
    rates.plot(kind="bar", ax=ax)
    ax.set_ylim(0, 1); ax.set_ylabel("rate"); ax.set_title("Boundary stress measured success stages (100 m, 20 queries/algorithm)")
    ax.grid(axis="y", alpha=.25); ax.legend(fontsize=8)
    _save(fig, directory / "boundary_stress_success_stages.png")

    timing = _quantile_table(frame, "wall_time_ms")
    fig, ax = plt.subplots(figsize=(12, 6))
    timing["algorithm"] = timing["algorithm"].map(lambda value: ALGORITHM_LABELS.get(value, value))
    timing.set_index("algorithm")[["P50", "P95", "P99"]].plot(kind="bar", ax=ax)
    ax.set_ylabel("wall time (ms)"); ax.set_title("Boundary stress wall time quantiles")
    ax.grid(axis="y", alpha=.25); ax.legend()
    _save(fig, directory / "boundary_stress_wall_time_quantiles.png")

    failures = frame.loc[~_as_bool(frame["final_valid_success"])].groupby(["algorithm", "failure_code"]).size().unstack(fill_value=0)
    failures.index = [ALGORITHM_LABELS.get(value, value) for value in failures.index]
    fig, ax = plt.subplots(figsize=(12, 6))
    if not failures.empty:
        failures.plot(kind="bar", stacked=True, ax=ax)
    ax.set_ylabel("run count"); ax.set_title("Boundary stress structured failures")
    ax.grid(axis="y", alpha=.25)
    _save(fig, directory / "boundary_stress_failure_codes.png")


def generate(directory: Path, *, output_name: str = "plots_strict_v1") -> Path:
    directory = directory.resolve()
    frame = _maps(directory, _load(directory))
    output = directory / output_name
    output.mkdir(parents=True, exist_ok=True)
    _scale_lines(frame, "wall_time_ms", "wall time (ms)", "Measured wall time vs grid scale", output / "planning_time_scale_p50_p95_p99.png")
    _scale_lines(frame, "planner_rss_peak_bytes", "planner RSS (bytes)", "Measured planner RSS vs grid scale", output / "memory_scale_p50_p95_p99.png")
    _scale_lines(frame, "planner_pss_peak_bytes", "planner PSS (bytes)", "Measured planner PSS vs grid scale", output / "pss_memory_scale_p50_p95_p99.png")
    _scale_lines(frame, "cpu_total_ms", "CPU time (ms)", "Measured CPU time vs grid scale", output / "cpu_scale_p50_p95_p99.png")
    _scale_lines(frame, "path_length_m", "path length (m)", "Returned path length vs grid scale (planner-success candidates)", output / "path_length_scale_p50_p95_p99.png")
    _scale_lines(frame, "length_over_euclidean", "length / Euclidean", "Path length ratio vs grid scale (planner-success candidates)", output / "path_length_ratio_scale_p50_p95_p99.png")
    _success_plot(frame, output / "success_scale_stages.png")
    _path_quality(frame, output / "path_quality_candidates.png")
    # Explicit empty-valid output prevents a candidate-path plot from being
    # mistaken for an Ackermann-valid quality result.
    valid = frame[_as_bool(frame["final_valid_success"])]
    (output / "valid_path_count.txt").write_text(f"final_valid_rows={len(valid)}\n")
    boundary = directory / "boundary_stress"
    if boundary.exists() and (boundary / "runs.csv").exists():
        boundary_frame = _load(boundary)
        _boundary_plot(boundary_frame, output)
        _path_quality(boundary_frame, output / "boundary_stress_path_quality.png")
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate strict read-only PLN-02 benchmark plots")
    parser.add_argument("--dir", required=True)
    parser.add_argument("--output-name", default="plots_strict_v1")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    output = generate(Path(args.dir), output_name=args.output_name)
    print(f"strict plots: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
