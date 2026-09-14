"""Profile-controlled efficiency smoke for the fixed three-layer pipeline."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import shutil
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import fixed_layered_pipeline_smoke as fixed


OUTPUT_NAME = "fixed_layered_pipeline_smoke_v5_efficiency"
DEFAULT_OUTPUT = fixed.ROOT / "experiments/layered_planner_benchmark" / OUTPUT_NAME
V6_OUTPUT_NAME = "fixed_layered_pipeline_smoke_v6_window_efficiency"
V6_DEFAULT_OUTPUT = fixed.ROOT / "experiments/layered_planner_benchmark" / V6_OUTPUT_NAME
V7_ROOT = fixed.ROOT / "experiments/layered_planner_benchmark/fixed_layered_pipeline_v7_online_efficiency"
DEFAULT_CACHE = fixed.ROOT / "experiments/layered_planner_benchmark/topology_cache_v1"
V7_STAGE_SPECS: Tuple[Tuple[str, str, str, str], ...] = (
    ("baseline_v6_compatible", "v6_compatible", "baseline", "baseline"),
    ("step1_skip_simplification", "v7_candidate", "step1_skip_simplification", "baseline"),
    ("step2_light_reset", "v7_candidate", "step2_light_reset", "baseline"),
    ("step3_delta_map", "v7_candidate", "step3_delta_map", "baseline"),
    ("smac_angle_bins_48", "v7_candidate", "step3_delta_map", "angle_bins_48"),
    ("smac_downsample_2", "v7_candidate", "step3_delta_map", "downsample_2"),
    ("smac_lighter_smoother", "v7_candidate", "step3_delta_map", "lighter_smoother"),
    ("smac_bounded_search", "v7_candidate", "step3_delta_map", "bounded_search"),
)


def _read_csv(path: Path) -> List[Dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _truth(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _number(value: Any) -> float:
    try:
        return float(value or 0.0)
    except (TypeError, ValueError):
        return 0.0


def _measured_rows(directory: Path) -> List[Dict[str, str]]:
    return [
        row for row in _read_csv(directory / "runs.csv")
        if row.get("run_mode") == "measured" and row.get("query_role") == "raw"
    ]


def _experiment_summary(directory: Path) -> Dict[str, Any]:
    rows = _measured_rows(directory)
    manifest = yaml.safe_load((directory / "manifest.yaml").read_text(encoding="utf-8")) or {}
    online = [_number(row.get("online_pipeline_wall_time_ms")) for row in rows]
    resets = [_number(row.get("query_session_reset_ms")) for row in rows]
    simplification = [_number(row.get("simplification_total_time_ms")) for row in rows]
    local_map = [_number(row.get("l3_local_map_update_ms")) for row in rows]
    planner = [_number(row.get("l3_planner_wall_ms")) for row in rows]
    hard_valid = all(
        _truth(row.get("final_valid_success"))
        and _truth(row.get("static_footprint_valid"))
        and _truth(row.get("kinematic_valid"))
        and _number(row.get("maximum_curvature")) <= fixed.MAX_CURVATURE + fixed.CURVATURE_NUMERICAL_TOLERANCE
        and _number(row.get("reverse_distance_m")) <= 1.0e-9
        and int(_number(row.get("in_place_rotation_count"))) == 0
        and int(_number(row.get("heading_discontinuity_count"))) == 0
        and int(_number(row.get("position_discontinuity_count"))) == 0
        and int(_number(row.get("steering_jump_count"))) == 0
        for row in rows
    )
    closure = all(
        abs(_number(row.get("unaccounted_time_ms")))
        <= 0.05 * max(1.0, _number(row.get("online_pipeline_wall_time_ms")))
        for row in rows
    )
    return {
        "experiment": directory.name,
        "directory": str(directory),
        "optimization_profile": manifest.get("optimization_profile", ""),
        "optimization_stage": manifest.get("optimization_stage", ""),
        "smac_parameter_profile": manifest.get("smac_parameter_profile", ""),
        "source_hash": manifest.get("source_hash", ""),
        "config_hash": manifest.get("config_hash", ""),
        "measured_count": len(rows),
        "final_valid_count": sum(_truth(row.get("final_valid_success")) for row in rows),
        "functional_gate": len(rows) == 12 and hard_valid,
        "timing_closed": closure,
        "online_p50_ms": float(np.percentile(online, 50)) if online else float("inf"),
        "online_p95_ms": float(np.percentile(online, 95)) if online else float("inf"),
        "online_p99_ms": float(np.percentile(online, 99)) if online else float("inf"),
        "simplification_p50_ms": float(np.percentile(simplification, 50)) if simplification else 0.0,
        "query_reset_p50_ms": float(np.percentile(resets, 50)) if resets else 0.0,
        "local_map_update_p50_ms": float(np.percentile(local_map, 50)) if local_map else 0.0,
        "l3_planner_p50_ms": float(np.percentile(planner, 50)) if planner else 0.0,
        "l3_backend_call_count": sum(int(_number(row.get("l3_backend_call_count"))) for row in rows),
        "l3_window_count": sum(int(_number(row.get("repair_window_count"))) for row in rows),
        "local_map_update_messages": sum(int(_number(row.get("l3_local_map_update_messages"))) for row in rows),
        "local_map_update_cells": sum(int(_number(row.get("l3_local_map_update_cells"))) for row in rows),
        "local_map_update_bytes": sum(int(_number(row.get("l3_local_map_update_bytes"))) for row in rows),
        "delta_fallback_count": sum(int(_number(row.get("l3_local_map_update_fallback_count"))) for row in rows),
        "unexpected_delta_retry_count": sum(
            max(0, int(_number(row.get("l3_backend_call_count"))) - int(_number(row.get("repair_window_count"))))
            for row in rows
        ),
        "session_reset_fallback_count": sum(_truth(row.get("session_reset_fallback")) for row in rows),
        "low_window_skip_count": sum(row.get("simplification_skip_reason") == "low_l3_window_count" for row in rows),
        "session_start_count": int(manifest.get("session_start_count") or 0),
        "session_close_count": int(manifest.get("session_close_count") or 0),
        "session_restart_count": int(manifest.get("session_restart_count") or 0),
        "rrtstar_call_count": int(manifest.get("rrtstar_call_count") or 0),
        "sst_call_count": int(manifest.get("sst_call_count") or 0),
    }


def _quality_regressions(baseline_dir: Path, candidate_dir: Path) -> List[str]:
    baseline = {
        (row.get("query_id"), row.get("repetition")): row
        for row in _measured_rows(baseline_dir)
    }
    reasons: List[str] = []
    for row in _measured_rows(candidate_dir):
        key = (row.get("query_id"), row.get("repetition"))
        reference = baseline.get(key)
        if reference is None:
            reasons.append(f"missing_baseline:{key[0]}:{key[1]}")
            continue
        reference_length = _number(reference.get("path_length_m"))
        if reference_length and _number(row.get("path_length_m")) > reference_length * 1.02 + 1.0e-9:
            reasons.append(f"path_length_gt_2pct:{key[0]}:{key[1]}")
        if _number(row.get("minimum_clearance_m")) + 0.01 + 1.0e-9 < _number(reference.get("minimum_clearance_m")):
            reasons.append(f"clearance_drop_gt_0.01m:{key[0]}:{key[1]}")
    return reasons


def _acceptance(
    baseline_dir: Path, candidate_dir: Path,
    baseline: Mapping[str, Any], candidate: Mapping[str, Any],
) -> Tuple[bool, List[str]]:
    reasons: List[str] = []
    if not candidate.get("functional_gate"):
        reasons.append("functional_or_hard_constraint_gate_failed")
    if candidate.get("rrtstar_call_count") or candidate.get("sst_call_count"):
        reasons.append("forbidden_backend_called")
    if (
        candidate.get("session_start_count"), candidate.get("session_close_count"),
        candidate.get("session_restart_count"),
    ) != (1, 1, 0):
        reasons.append("session_lifecycle_not_1_1_0")
    p50_reduction = 1.0 - float(candidate["online_p50_ms"]) / float(baseline["online_p50_ms"])
    p95_reduction = 1.0 - float(candidate["online_p95_ms"]) / float(baseline["online_p95_ms"])
    if p50_reduction < 0.10:
        reasons.append("p50_improvement_below_10pct")
    if p95_reduction < 0.10:
        reasons.append("p95_improvement_below_10pct")
    if float(candidate["online_p50_ms"]) > 600.0:
        reasons.append("p50_above_600ms")
    if float(candidate["online_p95_ms"]) > 900.0:
        reasons.append("p95_above_900ms")
    if not candidate.get("timing_closed"):
        reasons.append("timing_not_closed_within_5pct")
    if int(candidate.get("delta_fallback_count") or 0):
        reasons.append("delta_map_fallback_observed")
    if (
        candidate.get("optimization_stage") == "step3_delta_map"
        and int(candidate.get("unexpected_delta_retry_count") or 0)
    ):
        reasons.append("unexplained_delta_retry_observed")
    reasons.extend(_quality_regressions(baseline_dir, candidate_dir))
    return not reasons, sorted(set(reasons))


def _resolve_final_profile(
    provisional_profile: str, provisional_smac: str,
    rerun_accepted: bool, rerun_reasons: Sequence[str],
) -> Dict[str, Any]:
    fallback = provisional_profile == "v7_candidate" and not rerun_accepted
    return {
        "profile": "v6_compatible" if fallback else provisional_profile,
        "smac_profile": "baseline" if fallback else provisional_smac,
        "optimization_stage": "baseline" if fallback else (
            "step3_delta_map" if provisional_profile == "v7_candidate" else "baseline"
        ),
        "fallback_applied": fallback,
        "reasons": list(rerun_reasons),
    }


def _run_stage(
    directory: Path, *, optimization_profile: str, optimization_stage: str,
    smac_parameter_profile: str, cache_dir: Path, selected_final_profile: str,
) -> Path:
    return fixed.run_smoke(
        directory,
        map_ids=("hospital_005",), query_ids=fixed.RAW_SMOKE_QUERY_IDS,
        include_diagnostics=False, diagnostic_query_ids=(), context_scope="map",
        warmups=1, repetitions=3, topology_cache_dir=cache_dir,
        simplify_l2=True, extra_source_files=(Path(__file__).resolve(),),
        efficiency_profile="v6", efficiency_baseline_runs=fixed.V5_BASELINE_RUNS,
        optimization_profile=optimization_profile, optimization_stage=optimization_stage,
        smac_parameter_profile=smac_parameter_profile,
        selected_final_profile=selected_final_profile,
    )


def _prepare_rollback_bundle(root: Path) -> None:
    """Copy the immutable v6 rollback bundle into a fresh A/B root.

    Historical v7 roots are intentionally never reused for a new run.  A
    caller may provide an empty sibling directory; in that case the latest
    completed v7 result is used only as the read-only source of the rollback
    archive.
    """
    root.mkdir(parents=True, exist_ok=True)
    existing = {item.name for item in root.iterdir()}
    if existing - {"rollback", "rollback_manifest.yaml"}:
        raise ValueError(f"refusing to overwrite non-empty A/B root: {root}")
    has_rollback_dir = (root / "rollback").is_dir()
    has_rollback_manifest = (root / "rollback_manifest.yaml").is_file()
    if has_rollback_dir != has_rollback_manifest:
        raise ValueError(f"rollback bundle is incomplete: {root}")
    if has_rollback_dir and has_rollback_manifest:
        return
    rollback_sources = (
        root,
        V7_ROOT,
        fixed.ROOT / "experiments/layered_planner_benchmark/"
        "fixed_layered_pipeline_v7_online_efficiency_postfix5_final",
        fixed.ROOT / "experiments/layered_planner_benchmark/"
        "fixed_layered_pipeline_v7_online_efficiency_postfix4_final",
    )
    source_root = next(
        (
            candidate for candidate in rollback_sources
            if (candidate / "rollback").is_dir()
            and (candidate / "rollback_manifest.yaml").is_file()
        ),
        None,
    )
    if source_root is None:
        raise RuntimeError("v6 rollback baseline is unavailable")
    shutil.copy2(source_root / "rollback_manifest.yaml", root / "rollback_manifest.yaml")
    shutil.copytree(source_root / "rollback", root / "rollback")


def run_v7_ab(root: Path = V7_ROOT) -> Path:
    root = root.resolve()
    _prepare_rollback_bundle(root)
    cache_dir = root / "topology_cache"
    stage_summaries: List[Dict[str, Any]] = []
    for name, profile, stage, smac_profile in V7_STAGE_SPECS:
        directory = root / name
        _run_stage(
            directory, optimization_profile=profile, optimization_stage=stage,
            smac_parameter_profile=smac_profile, cache_dir=cache_dir,
            selected_final_profile="v6_compatible",
        )
        stage_summaries.append(_experiment_summary(directory))

    baseline_dir = root / "baseline_v6_compatible"
    baseline = stage_summaries[0]
    for summary in stage_summaries:
        summary["p50_reduction_vs_baseline"] = (
            1.0 - float(summary["online_p50_ms"]) / float(baseline["online_p50_ms"])
        )
        summary["p95_reduction_vs_baseline"] = (
            1.0 - float(summary["online_p95_ms"]) / float(baseline["online_p95_ms"])
        )
        accepted, reasons = _acceptance(
            baseline_dir, Path(summary["directory"]), baseline, summary,
        ) if summary["optimization_profile"] == "v7_candidate" else (False, ["baseline_reference"])
        summary["accepted"] = accepted
        summary["rejection_reasons"] = ";".join(reasons)

    eligible_names = {
        "step3_delta_map", "smac_angle_bins_48", "smac_downsample_2",
        "smac_lighter_smoother", "smac_bounded_search",
    }
    eligible = [
        summary for summary in stage_summaries
        if summary["experiment"] in eligible_names and summary["accepted"]
    ]
    winner = min(
        eligible, key=lambda item: (float(item["online_p95_ms"]), float(item["online_p50_ms"])),
    ) if eligible else None
    provisional_profile = "v7_candidate" if winner is not None else "v6_compatible"
    provisional_smac = str(winner["smac_parameter_profile"]) if winner else "baseline"
    provisional_stage = "step3_delta_map" if winner else "baseline"
    selected_attempt_dir = root / "selected_v7_candidate"
    _run_stage(
        selected_attempt_dir, optimization_profile=provisional_profile,
        optimization_stage=provisional_stage, smac_parameter_profile=provisional_smac,
        cache_dir=cache_dir, selected_final_profile=provisional_profile,
    )
    selected_attempt_summary = _experiment_summary(selected_attempt_dir)
    selected_accepted, selected_reasons = (
        _acceptance(baseline_dir, selected_attempt_dir, baseline, selected_attempt_summary)
        if provisional_profile == "v7_candidate" else (
            False, ["no_v7_candidate_met_all_acceptance_rules"]
        )
    )
    selection = _resolve_final_profile(
        provisional_profile, provisional_smac, selected_accepted, selected_reasons,
    )
    selected_profile = str(selection["profile"])
    selected_smac = str(selection["smac_profile"])
    selected_stage = str(selection["optimization_stage"])
    automatic_fallback_applied = bool(selection["fallback_applied"])
    selected_dir = baseline_dir if automatic_fallback_applied else selected_attempt_dir
    selected_summary = baseline if automatic_fallback_applied else selected_attempt_summary

    comparison_rows = stage_summaries + [{
        **selected_attempt_summary,
        "accepted": selected_accepted,
        "rejection_reasons": ";".join(selected_reasons),
        "p50_reduction_vs_baseline": 1.0 - float(selected_attempt_summary["online_p50_ms"]) / float(baseline["online_p50_ms"]),
        "p95_reduction_vs_baseline": 1.0 - float(selected_attempt_summary["online_p95_ms"]) / float(baseline["online_p95_ms"]),
    }]
    if automatic_fallback_applied:
        comparison_rows.append({
            **baseline,
            "experiment": "selected_v6_fallback",
            "accepted": False,
            "rejection_reasons": "selected_v7_rerun_rejected:" + ";".join(selected_reasons),
            "p50_reduction_vs_baseline": 0.0,
            "p95_reduction_vs_baseline": 0.0,
        })
    fixed._write_csv(root / "ab_comparison.csv", comparison_rows)
    smac_names = {name for name, _profile, _stage, smac in V7_STAGE_SPECS if smac != "baseline"}
    fixed._write_csv(
        root / "smac_profile_comparison.csv",
        [summary for summary in stage_summaries if summary["experiment"] in smac_names],
    )

    selected_files = (
        "runs.csv", "path_metrics.csv", "backend_call_log.csv", "session_timing.csv",
        "repair_window_summary.csv", "map_update_summary.csv", "simplification_summary.csv",
        "protocol.yaml", "topology_cache_manifest.yaml",
    )
    for name in selected_files:
        shutil.copy2(selected_dir / name, root / name)
    shutil.copytree(selected_dir / "paths", root / "paths")

    source_manifest = yaml.safe_load(
        (selected_dir / "source_manifest.yaml").read_text(encoding="utf-8")
    ) or {}
    source_manifest.update({
        "fallback_profile": "v6_compatible",
        "selected_final_profile": selected_profile,
        "selected_smac_parameter_profile": selected_smac,
        "automatic_fallback_applied": automatic_fallback_applied,
        "ab_comparison_sha256": hashlib.sha256((root / "ab_comparison.csv").read_bytes()).hexdigest(),
        "rollback_manifest_sha256": hashlib.sha256((root / "rollback_manifest.yaml").read_bytes()).hexdigest(),
    })
    (root / "source_manifest.yaml").write_text(
        yaml.safe_dump(source_manifest, sort_keys=False), encoding="utf-8",
    )
    latency_gate = bool(
        selected_accepted and float(selected_summary["online_p50_ms"]) <= 500.0
        and float(selected_summary["online_p95_ms"]) <= 1000.0
    )
    manifest = {
        "schema_version": 7,
        "experiment": root.name,
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "optimization_profile": "profile_controlled_ab",
        "fallback_profile": "v6_compatible",
        "provisional_selected_profile": provisional_profile,
        "provisional_selected_smac_parameter_profile": provisional_smac,
        "selected_final_profile": selected_profile,
        "selected_smac_parameter_profile": selected_smac,
        "selected_source_directory": str(selected_dir),
        "selected_attempt_directory": str(selected_attempt_dir),
        "automatic_fallback_applied": automatic_fallback_applied,
        "functional_gate_passed": bool(selected_summary["functional_gate"]),
        "v7_acceptance_passed": selected_accepted,
        "latency_gate_passed": latency_gate,
        "formal_scale_benchmark_unlocked": latency_gate,
        "selection_reasons": selected_reasons,
        "source_hash": selected_summary["source_hash"],
        "config_hash": selected_summary["config_hash"],
        "rrtstar_call_count": selected_summary["rrtstar_call_count"],
        "sst_call_count": selected_summary["sst_call_count"],
    }
    (root / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")

    by_name = {summary["experiment"]: summary for summary in stage_summaries}
    step1 = by_name["step1_skip_simplification"]
    step2 = by_name["step2_light_reset"]
    step3 = by_name["step3_delta_map"]
    smac_best = min(
        (summary for summary in stage_summaries if summary["experiment"] in smac_names and summary["functional_gate"]),
        key=lambda item: float(item["l3_planner_p50_ms"]), default=None,
    )
    report = (
        "# Fixed layered pipeline v7 online-efficiency A/B report\n\n"
        "This is a bounded `hospital_005` profile A/B smoke, not a formal multi-map experiment.\n\n"
        f"- Final selection: `{selected_profile}` with Smac profile `{selected_smac}`; fallback profile is always `v6_compatible`.\n"
        f"- Automatic fallback after the independent selected-v7 rerun: {automatic_fallback_applied}; reasons: {', '.join(selected_reasons) if selected_reasons else 'none'}.\n"
        f"- Baseline online P50/P95/P99: {baseline['online_p50_ms']:.2f}/{baseline['online_p95_ms']:.2f}/{baseline['online_p99_ms']:.2f} ms.\n"
        f"- Selected online P50/P95/P99: {selected_summary['online_p50_ms']:.2f}/{selected_summary['online_p95_ms']:.2f}/{selected_summary['online_p99_ms']:.2f} ms.\n"
        f"- Skip-simplification saving at P50: {baseline['simplification_p50_ms'] - step1['simplification_p50_ms']:.2f} ms; candidate P50={step1['simplification_p50_ms']:.2f} ms; low-window skips={step1['low_window_skip_count']}/12.\n"
        f"- Light-reset saving at P50: {step1['query_reset_p50_ms'] - step2['query_reset_p50_ms']:.2f} ms; reset P50={step2['query_reset_p50_ms']:.2f} ms; fallbacks={step2['session_reset_fallback_count']}.\n"
        f"- Delta-map cells/bytes: baseline={baseline['local_map_update_cells']}/{baseline['local_map_update_bytes']}; step3={step3['local_map_update_cells']}/{step3['local_map_update_bytes']}; update P50 {baseline['local_map_update_p50_ms']:.2f}->{step3['local_map_update_p50_ms']:.2f} ms; fallbacks={step3['delta_fallback_count']}.\n"
        f"- Best functional single-variable Smac planner P50: `{smac_best['smac_parameter_profile'] if smac_best else 'none'}` at {smac_best['l3_planner_p50_ms'] if smac_best else 0.0:.2f} ms; baseline={baseline['l3_planner_p50_ms']:.2f} ms.\n"
        f"- Selected final validity: {selected_summary['final_valid_count']}/12; path-quality regressions: {'none' if not _quality_regressions(baseline_dir, selected_dir) else ';'.join(_quality_regressions(baseline_dir, selected_dir))}.\n"
        f"- Session start/close/restart: {selected_summary['session_start_count']}/{selected_summary['session_close_count']}/{selected_summary['session_restart_count']}; RRTstar/SST calls={selected_summary['rrtstar_call_count']}/{selected_summary['sst_call_count']}.\n"
        f"- v7 acceptance: {selected_accepted}; latency gate: {latency_gate}; timing closed within 5%: {selected_summary['timing_closed']}.\n"
        f"- Rejection/fallback reasons: {', '.join(selected_reasons) if selected_reasons else 'none'}.\n"
        f"- Allowed to enter the next formal multi-map stage: {latency_gate}. No formal multi-map run was started here.\n"
    )
    (root / "final_report.md").write_text(report, encoding="utf-8")
    return root


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a profile-controlled map-cached fixed layered efficiency smoke",
    )
    parser.add_argument("--profile", choices=("v5", "v6"), default=None, help="legacy v5/v6 experiment selector")
    parser.add_argument(
        "--optimization-profile", choices=("v6_compatible", "v7_candidate"),
        default="v6_compatible",
    )
    parser.add_argument(
        "--smac-parameter-profile", choices=tuple(fixed.legacy.SMAC_PARAMETER_PROFILES),
        default="baseline",
    )
    parser.add_argument(
        "--optimization-stage",
        choices=("baseline", "step1_skip_simplification", "step2_light_reset", "step3_delta_map"),
        default="step3_delta_map",
    )
    parser.add_argument(
        "--selected-final-profile", choices=("v6_compatible", "v7_candidate"),
        default="v6_compatible",
    )
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--topology-cache-dir", default=str(DEFAULT_CACHE))
    parser.add_argument(
        "--run-v7-ab", action="store_true",
        help="run the complete same-round v6/v7 staged A/B and automatic selector",
    )
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.run_v7_ab:
        try:
            output = run_v7_ab(Path(args.output_dir).resolve() if args.output_dir else V7_ROOT)
            print(output)
            return 0
        except (OSError, RuntimeError, ValueError) as exc:
            print(f"fixed_layered_pipeline_efficiency_smoke: ERROR: {exc}")
            return 2
    legacy_profile = args.profile
    efficiency_profile = legacy_profile or "v6"
    if args.output_dir:
        output_dir = Path(args.output_dir).resolve()
    elif legacy_profile:
        output_dir = V6_DEFAULT_OUTPUT if legacy_profile == "v6" else DEFAULT_OUTPUT
    else:
        output_dir = V7_ROOT / (
            f"manual_{args.optimization_profile}_{args.smac_parameter_profile}"
        )
    try:
        output = fixed.run_smoke(
            output_dir,
            map_ids=("hospital_005",),
            query_ids=fixed.RAW_SMOKE_QUERY_IDS,
            include_diagnostics=legacy_profile == "v5",
            diagnostic_query_ids=(("q00",) if legacy_profile == "v5" else ()),
            context_scope="map",
            warmups=1,
            repetitions=3,
            topology_cache_dir=Path(args.topology_cache_dir).resolve(),
            simplify_l2=True,
            extra_source_files=(Path(__file__).resolve(),),
            efficiency_profile=efficiency_profile,
            efficiency_baseline_runs=(
                fixed.V5_BASELINE_RUNS if efficiency_profile == "v6" else fixed.V4_BASELINE_RUNS
            ),
            optimization_profile=args.optimization_profile,
            optimization_stage=args.optimization_stage,
            smac_parameter_profile=args.smac_parameter_profile,
            selected_final_profile=args.selected_final_profile,
        )
        print(output)
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"fixed_layered_pipeline_efficiency_smoke: ERROR: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
