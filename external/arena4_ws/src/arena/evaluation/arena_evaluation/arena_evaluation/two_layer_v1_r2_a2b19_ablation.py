"""Single-parameter A2B-19 Smac ablations with internal instrumentation."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np
import yaml

from . import two_layer_v1_formal_benchmark as parent
from . import two_layer_v1_r2_roi_pathaudit_benchmark as r2


DEFAULT_OUTPUT = parent.ROOT / "experiments/layered_planner_benchmark/2a_v1_r2_a2b19_ablation_v1"
PROFILES: Sequence[tuple[str, Dict[str, Any]]] = (
    ("baseline", {}),
    ("obstacle_heuristic_cache", {"cache_obstacle_heuristic": True}),
    ("angle_bins_48", {"angle_quantization_bins": 48}),
    ("downsample_2", {"downsample_costmap": True, "downsampling_factor": 2}),
    ("analytic_ratio_2", {"analytic_expansion_ratio": 2.0}),
    ("analytic_max_length_4", {"analytic_expansion_max_length": 4.0}),
    ("smoother_iterations_50", {"smoother": {"max_iterations": 50}}),
)


def _p50(rows: Sequence[Dict[str, str]], field: str) -> Optional[float]:
    values = []
    for row in rows:
        try:
            values.append(float(row[field]))
        except (KeyError, TypeError, ValueError):
            pass
    return float(np.percentile(values, 50)) if values else None


def run(output: Path, *, warmups: int, repetitions: int, topology_cache_dir: Path, ros_domain_id: int) -> Path:
    parent._refuse_nonempty(output)
    output.mkdir(parents=True)
    summary: List[Dict[str, Any]] = []
    for index, (name, override) in enumerate(PROFILES):
        profile_output = output / name
        effective = {"benchmark_instrumentation": True, **override}
        r2.run_experiment(
            profile_output, costmap_mode="roi_ack", endpoint_mode="baseline",
            warmups=warmups, repetitions=repetitions, query_ids=("A2B-19",),
            ros_domain_id=ros_domain_id + index,
            topology_cache_dir=topology_cache_dir,
            planner_parameter_overrides=effective,
        )
        with (profile_output / "runs.csv").open(newline="", encoding="utf-8") as stream:
            rows = [row for row in csv.DictReader(stream) if row.get("run_mode") == "measured"]
        valid = [row for row in rows if r2._truth(row.get("final_valid_success"))]
        maximum_curvature = max((float(row.get("maximum_curvature") or 0.0) for row in valid), default=None)
        minimum_clearance = min((float(row.get("minimum_clearance_m") or 0.0) for row in valid), default=None)
        summary.append({
            "profile": name, "single_parameter_override": override,
            "measured_count": len(rows), "final_valid_count": len(valid),
            "online_p50_ms": _p50(valid, "online_wall_ms"),
            "costmap_update_p50_ms": _p50(valid, "costmap_update_ms"),
            "smac_action_p50_ms": _p50(valid, "l3_action_wall_ms"),
            "smac_planning_p50_ms": _p50(valid, "hybrid_planning_time_ms"),
            "smac_search_p50_ms": _p50(valid, "smac_search_ms"),
            "smac_smoothing_p50_ms": _p50(valid, "smac_smoothing_ms"),
            "expanded_states_p50": _p50(valid, "expanded_states"),
            "generated_states_p50": _p50(valid, "generated_states"),
            "heuristic_reset_p50_ms": _p50(valid, "smac_heuristic_reset_ms"),
            "heuristic_eval_p50_ms": _p50(valid, "smac_heuristic_eval_ms"),
            "analytic_expansion_p50_ms": _p50(valid, "smac_analytic_expansion_ms"),
            "analytic_attempts_p50": _p50(valid, "smac_analytic_attempts"),
            "analytic_successes_p50": _p50(valid, "smac_analytic_successes"),
            "maximum_curvature": maximum_curvature,
            "minimum_clearance_m": minimum_clearance,
            "maximum_reverse_distance_m": max((float(row.get("reverse_distance_m") or 0.0) for row in valid), default=None),
            "maximum_in_place_rotation_count": max((int(float(row.get("in_place_rotation_count") or 0)) for row in valid), default=None),
            "ack_failure_count": sum(not r2._truth(row.get("costmap_update_acknowledged")) for row in rows),
            "failure_codes": sorted({str(row.get("failure_code") or "") for row in rows if row.get("failure_code")}),
        })
    r2._write_csv(output / "ablation_summary.csv", summary)
    source_files, source_hash = r2._source_manifest()
    source_files[str(Path(__file__).resolve())] = parent.sha256_file(Path(__file__).resolve())
    source_hash = r2._json_hash(source_files)
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({
        "source_hash": source_hash, "source_files": source_files,
    }, sort_keys=False), encoding="utf-8")
    (output / "protocol.yaml").write_text(yaml.safe_dump({
        "diagnostic_only": True, "query_id": "A2B-19",
        "one_parameter_changed_per_profile": True,
        "base_profile": "lighter_smoother", "profiles": dict(PROFILES),
        "internal_instrumentation": True, "warmups": warmups,
        "repetitions": repetitions, "formal_constraints_unchanged": True,
    }, sort_keys=False), encoding="utf-8")
    valid_candidates = [
        row for row in summary
        if row["final_valid_count"] == repetitions
        and float(row["maximum_curvature"] or 999.0) <= 2.501
        and float(row["maximum_reverse_distance_m"] or 0.0) <= 1.0e-6
        and int(row["maximum_in_place_rotation_count"] or 0) == 0
        and row["ack_failure_count"] == 0
    ]
    selected = min(valid_candidates, key=lambda row: float(row["smac_action_p50_ms"] or float("inf"))) if valid_candidates else None
    (output / "manifest.yaml").write_text(yaml.safe_dump({
        "experiment_id": output.name, "source_hash": source_hash,
        "profile_count": len(PROFILES), "selected_profile": selected["profile"] if selected else "none",
        "selection_rule": "lowest valid Smac action P50; formal adoption still requires four-query preflight",
    }, sort_keys=False), encoding="utf-8")
    lines = [
        "# A2B-19 instrumented single-parameter ablation", "",
        "| Profile | Valid | Action P50 ms | Search P50 ms | Smooth P50 ms | Expanded P50 | Generated P50 | Heuristic eval P50 ms | Analytic P50 ms | Max curvature | Min clearance m |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary:
        lines.append(
            f"| {row['profile']} | {row['final_valid_count']}/{row['measured_count']} | "
            f"{row['smac_action_p50_ms']} | {row['smac_search_p50_ms']} | {row['smac_smoothing_p50_ms']} | "
            f"{row['expanded_states_p50']} | {row['generated_states_p50']} | {row['heuristic_eval_p50_ms']} | "
            f"{row['analytic_expansion_p50_ms']} | {row['maximum_curvature']} | {row['minimum_clearance_m']} |"
        )
    lines.extend(["", f"Diagnostic winner: `{selected['profile'] if selected else 'none'}`. Formal adoption requires the separate 01/07/16/19 gate.", ""])
    (output / "final_report.md").write_text("\n".join(lines), encoding="utf-8")
    return output


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--topology-cache-dir", default=str(r2.DEFAULT_TOPOLOGY_CACHE))
    parser.add_argument("--warmups", type=int, default=1)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--ros-domain-id", type=int, default=100)
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    args = parser.parse_args(argv)
    try:
        result = run(
            Path(args.output_dir).resolve(), warmups=args.warmups,
            repetitions=args.repetitions,
            topology_cache_dir=Path(args.topology_cache_dir).resolve(),
            ros_domain_id=args.ros_domain_id,
        )
    except Exception as exc:
        print(f"two_layer_v1_r2_a2b19_ablation: ERROR: {exc}")
        return 2
    print(f"A2B-19 ablation output: {result}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
