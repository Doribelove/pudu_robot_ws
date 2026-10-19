"""Formal static runner for the independent 2D-V2 candidate."""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import statistics
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence

import numpy as np
import yaml

from . import layered_2d_v2_pipeline as v2
from . import l1_l3_corridor_hybrid_smoke as candidate
from . import two_layer_2d_v1_r2_formal_benchmark as d1r2
from . import two_layer_v1_formal_benchmark as base
from . import two_layer_v1_r2_roi_pathaudit_benchmark as a2r2
from .planner_benchmark.map_utils import sha256_file


ROOT = Path("/home/robot/pudu_robot_ws")
MAP_ID = "mentor_map_20260825_005"
ARCHITECTURE_ID = v2.ARCHITECTURE_ID
IMPLEMENTATION_REVISION = v2.IMPLEMENTATION_REVISION
PARENT_ARCHITECTURE = v2.PARENT_ARCHITECTURE
PROTOCOL_VERSION = v2.PROTOCOL_VERSION
ROS_DOMAIN_ID = 118
WARMUPS = 3
REPETITIONS = 5
TOPOLOGY_CACHE = d1r2.DEFAULT_CACHE_ROOT
FROZEN_2A_R2 = ROOT / "experiments/layered_planner_benchmark/2a_v1_mentor_map_20260825_005_20_r2_roi_pathaudit_v1"
FROZEN_2D_R2 = ROOT / "experiments/layered_planner_benchmark/2d_v1_mentor_map_20260825_005_20_performance_r2_20260903_1147"
FROZEN_2D_R3 = ROOT / "experiments/layered_planner_benchmark/2d_v1_dynamic_incremental_value_v1_20260903_134619"
CONFIG = Path(__file__).resolve().parents[1] / "config/two_layer_2d_v2_r0_enhanced.yaml"


def _default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return ROOT / "experiments/layered_planner_benchmark" / f"2d_v2_static_mentor_map_005_r0_{stamp}"


def _tree_hash(path: Path) -> str:
    records = []
    for item in sorted(candidate_path for candidate_path in path.rglob("*") if candidate_path.is_file()):
        records.append([str(item.relative_to(path)), sha256_file(item)])
    return v2.stable_hash(records)


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    a2r2._write_csv(path, rows)


def _number(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if np.isfinite(result) else None


def _truth(value: Any) -> bool:
    return value is True or str(value).strip().lower() in {"1", "true", "yes"}


def _stats(rows: Sequence[Mapping[str, Any]], field: str) -> Dict[str, Any]:
    values = [value for row in rows if (value := _number(row.get(field))) is not None]
    return {
        "count": len(values), "p50": float(np.percentile(values, 50)) if values else None,
        "p95": float(np.percentile(values, 95)) if values else None,
        "p99": float(np.percentile(values, 99)) if values else None,
        "mean": statistics.fmean(values) if values else None,
    }


def _source_manifest() -> Dict[str, str]:
    files, _old_hash = a2r2._source_manifest()
    sources = [Path(__file__).resolve(), Path(v2.__file__).resolve(), CONFIG,
               Path(__file__).resolve().parents[1] / "setup.py"]
    test_root = Path(__file__).resolve().parents[1] / "test"
    sources.extend(sorted(test_root.glob("test_*2d_v2*.py")))
    for source in sources:
        if source.is_file():
            files[str(source)] = sha256_file(source)
    return dict(sorted(files.items()))


def _snapshot_sources(output: Path, sources: Mapping[str, str]) -> Dict[str, Any]:
    directory = output / "source_snapshot"
    directory.mkdir()
    records = []
    for index, (name, digest) in enumerate(sorted(sources.items())):
        source = Path(name)
        if not source.is_file() or sha256_file(source) != digest:
            raise RuntimeError(f"source changed before snapshot: {source}")
        target = directory / f"{index:03d}_{source.name}"
        shutil.copyfile(source, target)
        records.append({"source": str(source), "snapshot": str(target.relative_to(output)),
                        "sha256": digest, "bytes": target.stat().st_size})
    payload = {"schema_version": 1, "file_count": len(records), "files": records,
               "combined_hash": v2.stable_hash([[row["snapshot"], row["sha256"]] for row in records])}
    (output / "source_snapshot_manifest.yaml").write_text(
        yaml.safe_dump(payload, sort_keys=False), encoding="utf-8",
    )
    return payload


def _reference_rows() -> Dict[str, List[Dict[str, str]]]:
    result: Dict[str, List[Dict[str, str]]] = {}
    with (FROZEN_2D_R2 / "runs.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row.get("run_mode") == "measured":
                result.setdefault(str(row["query_id"]), []).append(row)
    return result


def _phase_summary(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    fields = (
        "online_wall_ms", "l1_graph_search_ms", "l1_attachment_lookup_ms",
        "l1_route_search_ms", "total_corridor_mask_online_ms", "costmap_update_ms",
        "costmap_ack_wait_ms", "l3_action_wall_ms", "hybrid_planning_time_ms",
        "canonical_path_audit_ms", "final_validation_time_ms",
        "path_postprocess_ms", "path_validation_ms",
    )
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    result = []
    for group, selected in (("overall", measured),
                            ("success", [row for row in measured if _truth(row.get("final_valid_success"))])):
        for field in fields:
            result.append({"group": group, "phase": field, **_stats(selected, field)})
    return result


def _artifact_validation(output: Path) -> Dict[str, Any]:
    required = (
        "final_report.md", "protocol.yaml", "manifest.yaml", "verification.yaml",
        "runs.csv", "per_query_results.csv", "phase_timing_summary.csv",
        "correctness_oracle.csv", "cache_diagnostics.csv", "ack_diagnostics.csv",
        "break_even_curve_absolute.csv", "break_even_curve_ratio.csv",
        "memory_summary.csv", "source_snapshot_manifest.yaml", "stdout.log",
        "stderr.log", "reproduction_command.txt",
    )
    missing = [name for name in required if not (output / name).is_file()]
    source_manifest = yaml.safe_load((output / "source_snapshot_manifest.yaml").read_text()) if not missing else {}
    bad = []
    for row in (source_manifest or {}).get("files", []):
        path = output / row["snapshot"]
        if not path.is_file() or sha256_file(path) != row["sha256"]:
            bad.append(row["snapshot"])
    return {"required_artifact_count": len(required), "missing": missing,
            "bad_source_snapshot_files": bad, "passed": not missing and not bad}


def _write_report(output: Path, summary: Mapping[str, Any], rows: Sequence[Mapping[str, Any]],
                  per_query: Sequence[Mapping[str, Any]], cache: Any,
                  frozen_hashes: Mapping[str, str]) -> None:
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    success = [row for row in measured if _truth(row.get("final_valid_success"))]
    index = {row["query_id"]: row for row in per_query}
    status = "PASS" if summary["static_gate_pass"] else "FAIL"
    fmt = lambda value: "not_available" if value is None else f"{float(value):.3f}"
    lines = [
        "# 2D-V2 r0 static formal experiment", "",
        f"- Candidate gate: **{status}**; promotion is not implied by this static result alone.",
        f"- Final-valid: **{summary['final_valid_count']}/{summary['measured_count']}**.",
        f"- Success online P50/P95/P99: {fmt(summary['success_online']['p50'])}/"
        f"{fmt(summary['success_online']['p95'])}/{fmt(summary['success_online']['p99'])} ms.",
        f"- Overall online P50/P95/P99: {fmt(summary['overall_online']['p50'])}/"
        f"{fmt(summary['overall_online']['p95'])}/{fmt(summary['overall_online']['p99'])} ms.",
        f"- ACK: {summary['ack_success_count']}/{summary['l3_call_count']} successful; "
        f"mismatch cells={summary['ack_mismatch_cells']}; repair={summary['repair_count']}; "
        f"full fallback={summary['full_fallback_count']}.",
        f"- Adaptive mask cache hit/miss={cache.route_hits}/{cache.route_misses}; "
        f"offline build={cache.offline_build_ms:.3f} ms; edge bytes={cache.edge_cache_bytes}.",
        "", "## Required query controls", "",
        "| Query | valid | failure | online P50 ms | route/mask/path note |", "|---|---:|---|---:|---|",
    ]
    for query_id in ("A2B-07", "A2B-16", "A2B-19"):
        if query_id not in index:
            continue
        row = index[query_id]
        lines.append(f"| {query_id} | {row['final_valid_count']}/{row['measured_count']} | "
                     f"{row['failure_codes'] or '-'} | {row['online_p50_ms']} | {row['hash_summary']} |")
    lines.extend([
        "", "## Architecture and attribution", "",
        "- L1 uses the frozen original skeleton graph and persistent Graph D* Lite. L2 is disabled.",
        "- The downstream runtime is the shared 2A-r2 implementation: 2/4 m adaptive masks, old/new dirty ROI, server content ACK, bounded repair/one full fallback, zero fixed settle after ACK, and a shared canonical PathAudit result.",
        "- ROI, ACK, 48 bins, adaptive corridors and PathAudit gains are engineering-runtime gains, not D* Lite gains.",
        "- Static runs use one L3 call in the normal path; A2B-16 remains an L1 no-route control.",
        "", "## Frozen inputs", "",
    ])
    for name, digest in frozen_hashes.items():
        lines.append(f"- `{name}`: `{digest}` (before=after).")
    lines.extend(["", "## Reproduction", "", "```bash",
        "source /opt/ros/humble/setup.bash",
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash",
        "v2_out=/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v2_static_mentor_map_005_r0_$(date +%Y%m%d_%H%M%S)",
        f"ROS_DOMAIN_ID={ROS_DOMAIN_ID} ros2 run arena_evaluation two_layer_2d_v2_static_benchmark --output-dir \"$v2_out\" --warmups 3 --repetitions 5 --ros-domain-id {ROS_DOMAIN_ID} --no-dynamic-obstacles",
        "```", "",
    ])
    (output / "final_report.md").write_text("\n".join(lines), encoding="utf-8")


def run_formal(output: Path, *, warmups: int = WARMUPS, repetitions: int = REPETITIONS,
               ros_domain_id: int = ROS_DOMAIN_ID,
               query_ids: Optional[Sequence[str]] = None,
               angle_bins: int = v2.ANGLE_QUANTIZATION_BINS,
               corridor_mode: str = "adaptive_2m_4m") -> Path:
    output = output.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    if warmups < 0 or repetitions < 1:
        raise ValueError("warmups must be nonnegative and repetitions positive")
    frozen = {str(path): _tree_hash(path) for path in (FROZEN_2A_R2, FROZEN_2D_R2, FROZEN_2D_R3)}
    output.mkdir(parents=True)
    (output / "paths").mkdir()
    queries, task_metadata = base._load_tasks()
    if query_ids:
        wanted = set(query_ids)
        queries = [query for query in queries if query.query_id in wanted]
        if len(queries) != len(wanted):
            raise ValueError("unknown query id")
    ctx = base._context()
    artifact, topology_info, source_audit = d1r2._load_frozen_r1_topology(ctx, TOPOLOGY_CACHE)
    sources = _source_manifest()
    source_hash = v2.stable_hash(sources)
    selector = v2.PersistentDStarRouteSelector(ctx, artifact, topology_info, source_hash=source_hash)
    cache = v2.V2AdaptiveRouteMaskCache(ctx, artifact, source_hash, output / "offline_mask_cache",
                                        selector=selector, corridor_mode=corridor_mode)
    cache_manifest = cache.prepare(queries)
    auditor = v2.PathAuditor(ctx, source_commit=base.validity._source_commit() or "unknown")
    spec = base.legacy.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"BACKEND_UNAVAILABLE: {spec.reason}")
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    session = v2.SmacSession(
        ctx, output, map_yaml=base.validity.MAP_YAML,
        log_tag=f"formal_2d_v2_r0_{MAP_ID}", local_mask_updates=True,
        optimization_profile=a2r2.r1.OPTIMIZATION_PROFILE,
        smac_parameter_profile=a2r2.r1.SMAC_PARAMETER_PROFILE,
        optimization_stage=a2r2.r1.OPTIMIZATION_STAGE,
        enable_mask_reuse_noop=True,
        planner_parameter_overrides={"angle_quantization_bins": int(angle_bins)},
        costmap_ack_timeout_s=0.8,
    )
    session.local_map_update_strategy = "roi_ack"
    session.full_grid_settle_cycles = 0
    rows: List[Dict[str, Any]] = []
    calls: List[Dict[str, Any]] = []
    metrics: List[Dict[str, Any]] = []
    session.start()
    try:
        for run_mode, count in (("warmup", warmups), ("measured", repetitions)):
            for repetition in range(1, count + 1):
                for query in queries:
                    row, call, metric = candidate._run_one(
                        ctx, artifact, topology_info, query, run_mode, repetition,
                        session, spec, output, base.validity._source_commit(),
                        corridor_padding_m=v2.BASE_PADDING_M,
                        corridor_semantics=v2.CORRIDOR_SEMANTICS,
                        profile_name=v2.CORRIDOR_PROFILE,
                        padding_schedule_m=(v2.BASE_PADDING_M,),
                        validate_each_attempt=True,
                        cache_mode=candidate.CACHE_MODE_OPTIMIZED,
                        corridor_mask_builder=cache.builder,
                        route_selector=cache.route_selector,
                        canonical_path_auditor=auditor.audit,
                        skip_session_path_mask_validation=True,
                        baseline_fallback_decider=lambda *_args: False,
                    )
                    annotated = cache_v1_annotate(output, row, query, task_metadata, topology_info)
                    annotated.update({"architecture_id": ARCHITECTURE_ID,
                                      "implementation_revision": IMPLEMENTATION_REVISION,
                                      "parent_architecture": PARENT_ARCHITECTURE,
                                      "source_hash": source_hash,
                                      "angle_quantization_bins": int(angle_bins),
                                      "dynamic_obstacles": False})
                    annotated["path_postprocess_ms"] = sum(float(annotated.get(field) or 0.0) for field in
                        ("ros_path_conversion_ms", "point_annotation_ms", "canonical_path_audit_ms"))
                    annotated["path_validation_ms"] = sum(float(annotated.get(field) or 0.0) for field in
                        ("footprint_validation_ms", "kinematic_validation_ms", "path_within_mask_ms"))
                    call = {**dict(call), "architecture_id": ARCHITECTURE_ID,
                            "implementation_revision": IMPLEMENTATION_REVISION,
                            "snapshot_id": "static-v2-s0"}
                    metric = {**dict(metric), "architecture_id": ARCHITECTURE_ID,
                              "implementation_revision": IMPLEMENTATION_REVISION}
                    rows.append(annotated); calls.append(call); metrics.append(metric)
    finally:
        session.close()

    sources_after = _source_manifest()
    if sources_after != sources:
        raise RuntimeError("source changed during formal run")
    frozen_after = {name: _tree_hash(Path(name)) for name in frozen}
    if frozen_after != frozen:
        raise RuntimeError("a frozen experiment directory changed during the run")
    _write_csv(output / "runs.csv", rows)
    _write_csv(output / "backend_call_log.csv", calls)
    _write_csv(output / "path_metrics.csv", metrics)
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    success = [row for row in measured if _truth(row.get("final_valid_success"))]
    reference = _reference_rows()
    per_query = []
    correctness = []
    for query in queries:
        selected = [row for row in measured if row.get("query_id") == query.query_id]
        valid_count = sum(_truth(row.get("final_valid_success")) for row in selected)
        failures = sorted({str(row.get("failure_code") or "") for row in selected if row.get("failure_code")})
        route_hashes = sorted({str(row.get("route_signature") or "") for row in selected})
        mask_hashes = sorted({str(row.get("corridor_mask_hash") or "") for row in selected})
        path_hashes = sorted({str(row.get("path_hash") or "") for row in selected if row.get("path_hash")})
        old = reference.get(query.query_id, [])
        old_failures = sorted({str(row.get("failure_code") or "") for row in old if row.get("failure_code")})
        no_new_failure_type = set(failures).issubset(set(old_failures) | {"L1_NO_ROUTE"})
        safe = all((not _truth(row.get("final_valid_success"))) or (
            _truth(row.get("static_footprint_valid")) and _truth(row.get("kinematic_valid"))
            and float(row.get("reverse_distance_m") or 0.0) <= 1e-9
            and int(float(row.get("in_place_rotation_count") or 0)) == 0
            and float(row.get("maximum_curvature") or 0.0) <= 2.5 + 1e-6
        ) for row in selected)
        per_query.append({"query_id": query.query_id, "measured_count": len(selected),
                          "final_valid_count": valid_count, "failure_codes": ",".join(failures),
                          "online_p50_ms": _stats(selected, "online_wall_ms")["p50"],
                          "route_hashes": route_hashes, "mask_hashes": mask_hashes,
                          "path_hashes": path_hashes,
                          "hash_summary": f"route={len(route_hashes)},mask={len(mask_hashes)},path={len(path_hashes)}"})
        correctness.append({"query_id": query.query_id, "accepted_path_safe": safe,
                            "no_new_failure_type": no_new_failure_type,
                            "v1_r2_failure_codes": old_failures, "v2_failure_codes": failures,
                            "route_hashes": route_hashes, "mask_hashes": mask_hashes,
                            "path_hashes": path_hashes})
    _write_csv(output / "per_query_results.csv", per_query)
    _write_csv(output / "correctness_oracle.csv", correctness)
    _write_csv(output / "phase_timing_summary.csv", _phase_summary(rows))
    ack_rows = []
    for row in measured:
        if int(float(row.get("l3_call_count") or 0)):
            ack_rows.append({"query_id": row["query_id"], "repetition": row["repetition"],
                             "ack_status": row.get("costmap_ack_status"),
                             "acknowledged": row.get("costmap_update_acknowledged"),
                             "mismatch_cells": int(float(row.get("costmap_ack_mismatch_cells") or 0)),
                             "ack_sequence": row.get("costmap_ack_sequence"),
                             "repair_attempted": row.get("costmap_repair_attempted", False),
                             "repair_count": int(float(row.get("costmap_repair_count") or 0)),
                             "full_fallback": row.get("costmap_update_fallback", False),
                             "route_signature": row.get("route_signature", ""),
                             "mask_hash": row.get("corridor_mask_hash", "")})
    _write_csv(output / "ack_diagnostics.csv", ack_rows)
    cache_rows = [
        {"cache": "adaptive_route_mask", "hits": cache.route_hits, "misses": cache.route_misses,
         "hit_rate": cache.route_hits / max(1, cache.route_hits + cache.route_misses),
         "cold_build_ms": cache.offline_build_ms,
         "memory_bytes": sum(mask.nbytes for mask, _diag in cache.route_masks.values())},
        {"cache": "edge_masks", "hits": cache.edge_hits, "misses": cache.edge_misses,
         "hit_rate": cache.edge_hits / max(1, cache.edge_hits + cache.edge_misses),
         "cold_build_ms": cache.edge_build_ms, "memory_bytes": cache.edge_cache_bytes},
        {"cache": "endpoint_exact_pose", "hits": selector.pipeline.endpoint_cache_hits,
         "misses": selector.pipeline.endpoint_cache_misses,
         "hit_rate": selector.pipeline.endpoint_cache_hits / max(1, selector.pipeline.endpoint_cache_hits + selector.pipeline.endpoint_cache_misses),
         "cold_build_ms": selector.pipeline.endpoint_cache_build_time_ms,
         "memory_bytes": selector.pipeline.endpoint_cache_memory_bytes},
    ]
    _write_csv(output / "cache_diagnostics.csv", cache_rows)
    _write_csv(output / "memory_summary.csv", [
        {"component": row["cache"], "memory_bytes": row["memory_bytes"]} for row in cache_rows
    ] + [{"component": "persistent_dstar_all_queries",
          "memory_bytes": sum(v2.incremental._dstar_memory_bytes(state.planner)
                              for state in selector.states.values())}])
    _write_csv(output / "break_even_curve_absolute.csv", [{"status": "NOT_APPLICABLE_STATIC"}])
    _write_csv(output / "break_even_curve_ratio.csv", [{"status": "NOT_APPLICABLE_STATIC"}])
    _write_csv(output / "session_timing.csv", [{
        "session_start_count": session.session_start_count,
        "session_close_count": session.session_close_count,
        "session_restart_count": session.session_restart_count,
        "startup_ms": session.stack_startup_time_ms, "shutdown_ms": session.stack_shutdown_time_ms,
        "fixed_settle_cycles": session.full_grid_settle_cycles,
    }])
    ack_success = sum(_truth(row.get("acknowledged")) for row in ack_rows)
    mismatch = sum(int(row["mismatch_cells"]) for row in ack_rows)
    repair = sum(int(row["repair_count"]) for row in ack_rows)
    fallback = sum(_truth(row["full_fallback"]) for row in ack_rows)
    failure_queries = {row["query_id"] for row in per_query if row["final_valid_count"] < row["measured_count"]}
    safe_pass = all(row["accepted_path_safe"] for row in correctness)
    overall_online = _stats(measured, "online_wall_ms")
    success_online = _stats(success, "online_wall_ms")
    gates = {
        "final_valid_pass": len(success) >= 90 if len(measured) == 100 else True,
        "no_new_failure_query_pass": failure_queries.issubset({"A2B-07", "A2B-16"}),
        "success_p50_pass": success_online["p50"] is not None and success_online["p50"] <= 500.0,
        "success_p95_pass": success_online["p95"] is not None and success_online["p95"] <= 1041.39,
        "ack_pass": ack_success == len(ack_rows) and mismatch == 0,
        "path_safety_pass": safe_pass,
        "a2b16_truthful_failure": next((row["final_valid_count"] == 0 and row["failure_codes"] == "L1_NO_ROUTE" for row in per_query if row["query_id"] == "A2B-16"), False),
    }
    gates["static_gate_pass"] = all(gates.values())
    summary = {"measured_count": len(measured), "final_valid_count": len(success),
               "overall_online": overall_online, "success_online": success_online,
               "l3_call_count": len(ack_rows), "ack_success_count": ack_success,
               "ack_mismatch_cells": mismatch, "repair_count": repair,
               "full_fallback_count": fallback, **gates}
    source_snapshot = _snapshot_sources(output, sources)
    protocol = {
        "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "parent_architecture": PARENT_ARCHITECTURE, "status": "candidate",
        "protocol_version": PROTOCOL_VERSION, "experiment_kind": "static_formal",
        "map_id": MAP_ID, "query_ids": [query.query_id for query in queries],
        "warmups": warmups, "repetitions": repetitions, "resolution_m": 0.05,
        "footprint_m": [0.51, 0.43], "minimum_turning_radius_m": 0.4,
        "maximum_curvature_1pm": 2.5, "allow_reverse": False,
        "allow_in_place_rotation": False, "dynamic_obstacles": False,
        "l1": "original_static_skeleton_persistent_graph_dstar_lite", "l2": "disabled",
        "l3_prime": "one_nav2_smac_planner_hybrid_dubin",
        "corridor_profile": v2.CORRIDOR_PROFILE, "corridor_padding_m": [2.0, 4.0],
        "angle_quantization_bins": int(angle_bins), "roi_content_ack": True,
        "corridor_mode": corridor_mode,
        "roi_payload_max_bytes": v2.ROI_MAX_MESSAGE_BYTES,
        "fixed_settle_cycles_after_ack": 0, "canonical_path_audit": True,
        "frozen_directory_hashes": frozen,
        "controlled_protocol_extension": "dynamic support is user-authorized for isolated 2D-V2 and does not alter static defaults",
    }
    (output / "protocol.yaml").write_text(yaml.safe_dump(protocol, sort_keys=False), encoding="utf-8")
    reproduction = (
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        "v2_out=/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/"
        "2d_v2_static_mentor_map_005_r0_$(date +%Y%m%d_%H%M%S)\n"
        f"ROS_DOMAIN_ID={ros_domain_id} ros2 run arena_evaluation two_layer_2d_v2_static_benchmark "
        f"--output-dir \"$v2_out\" --warmups {warmups} --repetitions {repetitions} "
        f"--ros-domain-id {ros_domain_id} --no-dynamic-obstacles\n"
    )
    (output / "reproduction_command.txt").write_text(reproduction, encoding="utf-8")
    (output / "stdout.log").write_text(f"output={output}\nstatic_gate_pass={gates['static_gate_pass']}\n", encoding="utf-8")
    (output / "stderr.log").write_text("", encoding="utf-8")
    _write_report(output, summary, rows, per_query, cache, frozen)
    manifest = {
        "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "parent_architecture": PARENT_ARCHITECTURE, "status": "candidate",
        "formal": True, "source_hash": source_hash,
        "source_snapshot_hash": source_snapshot["combined_hash"],
        "frozen_directory_hashes_before": frozen, "frozen_directory_hashes_after": frozen_after,
        "frozen_directories_unchanged": frozen == frozen_after,
        "topology_cache": topology_info, "source_audit": source_audit,
        "cache_manifest": cache_manifest, "summary": summary,
    }
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    validation = _artifact_validation(output)
    verification = {"formal_run_complete": True, "static_gates": gates,
                    "frozen_directories_unchanged": frozen == frozen_after,
                    "artifact_validation": validation,
                    "post_run_tests": "pending"}
    (output / "verification.yaml").write_text(yaml.safe_dump(verification, sort_keys=False), encoding="utf-8")
    validation = _artifact_validation(output)
    if not validation["passed"]:
        raise RuntimeError(f"artifact validation failed: {validation}")
    verification["artifact_validation"] = validation
    (output / "verification.yaml").write_text(yaml.safe_dump(verification, sort_keys=False), encoding="utf-8")
    return output


def cache_v1_annotate(output: Path, row: Mapping[str, Any], query: Any,
                      metadata: Mapping[str, Any], topology_info: Mapping[str, Any]) -> Dict[str, Any]:
    from . import two_layer_v1_r1_cache_benchmark as cache_v1
    return cache_v1._annotate_row(
        output, row, query, metadata, topology_info, candidate.CACHE_MODE_OPTIMIZED,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the formal 2D-V2 r0 enhanced static benchmark")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--warmups", type=int, default=WARMUPS)
    parser.add_argument("--repetitions", type=int, default=REPETITIONS)
    parser.add_argument("--query-id", action="append", dest="query_ids")
    parser.add_argument("--ros-domain-id", type=int, default=ROS_DOMAIN_ID)
    parser.add_argument("--angle-bins", type=int, choices=(48, 64), default=48)
    parser.add_argument("--corridor-mode", choices=("adaptive_2m_4m", "uniform_2m"),
                        default="adaptive_2m_4m")
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output = run_formal(args.output_dir or _default_output(), warmups=args.warmups,
                            repetitions=args.repetitions, ros_domain_id=args.ros_domain_id,
                            query_ids=args.query_ids, angle_bins=args.angle_bins,
                            corridor_mode=args.corridor_mode)
    except Exception as exc:
        print(f"two_layer_2d_v2_static_benchmark: ERROR: {exc}")
        return 2
    print(f"2D-V2 static output: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
