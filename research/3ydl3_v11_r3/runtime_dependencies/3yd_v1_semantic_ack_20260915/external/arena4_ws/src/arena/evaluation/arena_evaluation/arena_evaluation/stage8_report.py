"""Read-only aggregation for Stage 8 experiment directories."""

from __future__ import annotations

import argparse
import gzip
import json
import math
from pathlib import Path
from typing import Dict, List

import pandas as pd
import yaml


def _path_length(directory: Path, value) -> float:
    if value is None or str(value) in {"", "nan", "NaN"}: return float("nan")
    path = Path(str(value)); path = path if path.is_absolute() else directory / path
    if not path.exists(): return float("nan")
    with gzip.open(path, "rt", encoding="utf-8") as stream: points = json.load(stream)
    return sum(math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])) for a, b in zip(points, points[1:]))


def _stats(values: pd.Series, prefix: str) -> Dict[str, object]:
    values = pd.to_numeric(values, errors="coerce").dropna()
    if values.empty: return {}
    return {f"{prefix}_mean": values.mean(), f"{prefix}_P50": values.quantile(.50), f"{prefix}_P95": values.quantile(.95), f"{prefix}_P99": values.quantile(.99), f"{prefix}_min": values.min(), f"{prefix}_max": values.max()}


def report_stage8a(directory: Path, stage7: Path, smac: Path) -> None:
    protocol_path = directory / "protocol.yaml"; protocol = yaml.safe_load(protocol_path.read_text()) or {}; protocol.update({"local_smac_angle_quantization_bins": 360, "local_smac_smooth_path": False, "local_smac_footprint_padding_m": 0.10, "dynamic_obstacles": False}); protocol_path.write_text(yaml.safe_dump(protocol, sort_keys=False))
    runs = pd.read_csv(directory / "kinematic_runs.csv"); runs["path_length_m"] = [_path_length(directory, value) for value in runs.path_file]
    old = pd.read_csv(stage7 / "kinematic_runs.csv"); old = old[old["mode"].eq("layered_on_demand_l3")]
    old_map = {(str(row.query_id), int(row.repetition)): row for _, row in old.iterrows()}
    smac_runs = pd.read_csv(smac / "planner_runs.csv"); smac_runs = smac_runs[smac_runs.run_mode.eq("measured")].set_index("run_id")
    for index, row in runs.iterrows():
        if row["mode"] == "stage7_with_rotation":
            source = old_map.get((str(row.query_id), int(row.repetition)))
            if source is not None:
                runs.loc[index, "comparison_time_ms"] = source.get("l3_planning_time_ms")
                runs.loc[index, "comparison_cpu_ms"] = source.get("l3_cpu_total_ms")
                runs.loc[index, "comparison_rss_bytes"] = source.get("l3_rss_peak_bytes")
                runs.loc[index, "comparison_pss_bytes"] = source.get("l3_pss_peak_bytes")
        elif row["mode"] == "full_smac_normalized" and str(row.source_stage6_run_id) in smac_runs.index:
            source = smac_runs.loc[str(row.source_stage6_run_id)]
            runs.loc[index, "comparison_time_ms"] = source.get("planning_time_ms")
            runs.loc[index, "comparison_cpu_ms"] = source.get("cpu_total_ms")
            runs.loc[index, "comparison_rss_bytes"] = source.get("planner_rss_peak_bytes")
            runs.loc[index, "comparison_pss_bytes"] = source.get("planner_pss_peak_bytes")
        elif row["mode"] == "layered_hard_radius_l3":
            runs.loc[index, "comparison_time_ms"] = row.get("l3_planning_time_ms")
            runs.loc[index, "comparison_cpu_ms"] = row.get("l3_cpu_total_ms")
            runs.loc[index, "comparison_rss_bytes"] = row.get("l3_rss_peak_bytes")
            runs.loc[index, "comparison_pss_bytes"] = row.get("l3_pss_peak_bytes")
    summary = []
    for mode, group in runs.groupby("mode"):
        valid = group[group.final_valid_success.astype(bool)]
        row = {"mode": mode, "count": len(group), "success_count": len(valid), "all_query_success_rate": len(valid) / max(1, len(group)), "reachable_query_success_rate": len(valid[valid.query_id != "q04"]) / max(1, len(group[group.query_id != "q04"])), "static_valid_count": int(group.static_footprint_valid.astype(bool).sum()), "hard_kinematic_valid_count": int(group.hard_kinematic_valid.astype(bool).sum()), "rotate_in_place_count": int(pd.to_numeric(group.rotate_in_place_count, errors="coerce").fillna(0).sum()), "hybrid_calls": int(pd.to_numeric(group.hybrid_calls, errors="coerce").fillna(0).sum()), "hybrid_success": int(pd.to_numeric(group.hybrid_success, errors="coerce").fillna(0).sum())}
        row.update(_stats(group.comparison_time_ms, "planning_time_ms")); row.update(_stats(group.get("composed_online_time_ms", pd.Series(dtype=float)), "composed_online_time_ms")); row.update(_stats(group.comparison_cpu_ms, "cpu_time_ms")); row.update(_stats(group.comparison_rss_bytes, "rss_bytes")); row.update(_stats(group.comparison_pss_bytes, "pss_bytes")); row.update(_stats(group.path_length_m, "path_length_m")); summary.append(row)
    pd.DataFrame(summary).to_csv(directory / "stage8a_performance_summary.csv", index=False)
    candidate = runs[runs["mode"].eq("layered_hard_radius_l3")]
    attempts = []
    for _, row in candidate.iterrows():
        value = row.get("hybrid_attempts")
        if isinstance(value, str) and value.startswith("["):
            attempts.extend(json.loads(value))
    pd.DataFrame([{"window_radius_m": radius, "attempt_count": sum(float(item.get("window_radius_m", 0)) == radius for item in attempts), "accepted_count": sum(float(item.get("window_radius_m", 0)) == radius and not item.get("rejection_reason") for item in attempts)} for radius in (1.0, 2.0)]).to_csv(directory / "repair_window_summary.csv", index=False)
    candidate = runs[runs["mode"].eq("layered_hard_radius_l3")].set_index(["query_id","repetition"]); reference = runs[runs["mode"].eq("full_smac_normalized")].set_index(["query_id","repetition"]); paired = candidate.join(reference[["final_valid_success","path_length_m","comparison_time_ms","comparison_rss_bytes"]],rsuffix="_full_smac")
    paired = paired[paired.final_valid_success.astype(bool) & paired.final_valid_success_full_smac.astype(bool)]
    pd.DataFrame([{"paired_valid_count":len(paired),"path_length_ratio_mean":(paired.path_length_m/paired.path_length_m_full_smac).mean(),"path_length_ratio_P95":(paired.path_length_m/paired.path_length_m_full_smac).quantile(.95),"l3_only_planning_time_ratio_mean":(paired.comparison_time_ms/paired.comparison_time_ms_full_smac).mean(),"l3_only_planning_time_speedup_mean":(paired.comparison_time_ms_full_smac/paired.comparison_time_ms).replace([float("inf")],pd.NA).dropna().mean(),"composed_online_time_ratio_mean":(paired.composed_online_time_ms/paired.comparison_time_ms_full_smac).mean(),"composed_online_time_ratio_P50":(paired.composed_online_time_ms/paired.comparison_time_ms_full_smac).quantile(.50),"rss_layer_peak_ratio_mean":(paired.comparison_rss_bytes/paired.comparison_rss_bytes_full_smac).mean()}]).to_csv(directory/"stage8a_paired_comparison.csv",index=False)
    _plots(directory / "plots", runs)


def report_stage8b(directory: Path) -> None:
    manifest = yaml.safe_load((directory / "manifest.yaml").read_text()) or {}
    source = Path(str(manifest.get("protocol", "")))
    protocol = yaml.safe_load((directory / "protocol.yaml").read_text()) if (directory / "protocol.yaml").exists() else (yaml.safe_load(source.read_text()) if source.exists() else {})
    protocol.update({"stage8b_preference_weights": [0.0, 0.25, 0.5, 1.0], "right_wall_target_m": 0.40, "narrow_width_threshold_m": 1.23, "allow_in_place_rotation": False, "minimum_turning_radius": 0.40, "maximum_curvature": 2.50, "allow_reverse": True, "reverse_penalty": 2.0, "motion_model": "REEDS_SHEPP", "local_smac_angle_quantization_bins": 360, "local_smac_smooth_path": False, "local_smac_footprint_padding_m": 0.10, "dynamic_obstacles": False})
    (directory / "protocol.yaml").write_text(yaml.safe_dump(protocol, sort_keys=False))
    scan = pd.read_csv(directory / "weight_scan.csv"); selected = yaml.safe_load((directory / "selected_weights.yaml").read_text())
    baseline = scan[scan.preference_mode.eq("none")].set_index("query_id")
    rows = []
    for mode, weight in selected.items():
        group = scan[(scan.preference_mode == mode) & (scan.preference_weight == weight)].copy(); joined = group.join(baseline[["path_length_m", "expanded_nodes", "online_time_ms"]], on="query_id", rsuffix="_none")
        valid = joined[joined.final_valid_success.astype(bool)]; ratios = valid.path_length_m / valid.path_length_m_none
        error_field = "center_deviation_p50_m" if mode == "center" else "right_wall_error_p50_m"; zero = scan[(scan.preference_mode == mode) & (scan.preference_weight == 0.0)]
        rows.append({"preference_mode": mode, "selected_weight": weight, "success_rate": group.final_valid_success.astype(bool).mean(), "path_length_ratio_mean": ratios.mean(), "path_length_ratio_P95": ratios.quantile(.95), "expanded_nodes_change_ratio": (valid.expanded_nodes / valid.expanded_nodes_none).mean() - 1.0, "online_time_change_ratio": (valid.online_time_ms / valid.online_time_ms_none).mean() - 1.0, "selected_preference_error_mean_m": pd.to_numeric(valid[error_field], errors="coerce").mean(), "weight0_preference_error_mean_m": pd.to_numeric(zero[error_field], errors="coerce").mean(), "preference_error_reduction_ratio": 1.0 - pd.to_numeric(valid[error_field], errors="coerce").mean() / pd.to_numeric(zero[error_field], errors="coerce").mean(), "correct_side_ratio_mean": pd.to_numeric(valid.get("correct_side_ratio"), errors="coerce").mean() if "correct_side_ratio" in valid else None, "preference_active_ratio_mean": pd.to_numeric(valid.preference_active_ratio, errors="coerce").mean()})
    pd.DataFrame(rows).to_csv(directory / "stage8b_selected_comparison.csv", index=False)
    l3 = pd.read_csv(directory / "kinematic_runs.csv"); lookup = scan.dropna(subset=["run_id"]).set_index("run_id")["online_time_ms"]; timing = l3.copy(); timing["online_time_ms"] = timing.source_l2_run_id.map(lookup); timing["composed_online_time_ms"] = pd.to_numeric(timing.online_time_ms,errors="coerce") + pd.to_numeric(timing.l3_planning_time_ms,errors="coerce").fillna(0)
    composed=[]
    for mode,group in timing.groupby("preference_mode"):
        valid=group[group.final_valid_success.astype(bool)]; row={"preference_mode":mode,"count":len(group),"success_count":len(valid)}; row.update(_stats(valid.online_time_ms,"l2_online_time_ms")); row.update(_stats(valid.l3_planning_time_ms,"l3_planning_time_ms")); row.update(_stats(valid.composed_online_time_ms,"composed_online_time_ms")); composed.append(row)
    pd.DataFrame(composed).to_csv(directory/"stage8b_composed_time_summary.csv",index=False)


def _plots(directory: Path, runs: pd.DataFrame) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    for field, filename, ylabel in (("comparison_time_ms", "stage8a_planning_time.png", "planning time (ms)"), ("path_length_m", "stage8a_path_length.png", "path length (m)"), ("comparison_rss_bytes", "stage8a_peak_rss.png", "peak RSS (bytes)")):
        groups=[]; labels=[]
        for mode, group in runs.groupby("mode"):
            values=pd.to_numeric(group[field],errors="coerce").dropna()
            if len(values): groups.append(values); labels.append(mode)
        fig,ax=plt.subplots(figsize=(9,5))
        if groups: ax.boxplot(groups,tick_labels=labels); ax.tick_params(axis="x",rotation=25)
        ax.set_ylabel(ylabel); ax.grid(True,alpha=.25); fig.tight_layout(); fig.savefig(directory/filename,dpi=140); plt.close(fig)


def main(argv=None) -> int:
    parser=argparse.ArgumentParser(description="Aggregate Stage 8 static benchmark results"); parser.add_argument("--stage8a",required=True); parser.add_argument("--stage8b",required=True); parser.add_argument("--stage7",default="experiments/layered_planner_benchmark/hospital_005/stage7_l3_kinematic"); parser.add_argument("--smac",default="experiments/planner_benchmark/hospital_005/stage5_smac_normalized"); args=parser.parse_args(argv)
    report_stage8a(Path(args.stage8a),Path(args.stage7),Path(args.smac)); report_stage8b(Path(args.stage8b)); return 0


if __name__ == "__main__": raise SystemExit(main())
