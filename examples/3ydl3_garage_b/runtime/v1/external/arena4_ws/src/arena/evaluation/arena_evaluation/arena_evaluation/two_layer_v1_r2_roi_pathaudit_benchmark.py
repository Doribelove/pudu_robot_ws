"""2A-V1-r2: ACKed ROI costmaps and one canonical path audit.

The implementation keeps the frozen 2A-V1 protocol.  Full-grid and ROI/ACK
modes share the same route/mask/path pipeline so preflight A/B results are
directly comparable.  Every output directory is write-once.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

from . import endpoint_heading
from . import l1_l3_corridor_hybrid_smoke as candidate
from . import path_audit
from . import two_layer_v1_formal_benchmark as parent
from . import two_layer_v1_r1_cache_benchmark as r1


ROOT = parent.ROOT
MAP_ID = parent.MAP_ID
ARCHITECTURE_ID = "2A-V1"
IMPLEMENTATION_REVISION = "r2-roi-pathaudit-v1"
PARENT_ARCHITECTURE = "2A-V1-r1-cache-v7"
PROTOCOL_VERSION = parent.PROTOCOL_VERSION
DEFAULT_OUTPUT = ROOT / "experiments/layered_planner_benchmark/2a_v1_mentor_map_20260825_005_20_r2_roi_pathaudit_v1"
DEFAULT_TOPOLOGY_CACHE = ROOT / "experiments/layered_planner_benchmark/2a_v1_r2_topology_cache_v1"
WARMUPS = parent.WARMUPS
REPETITIONS = parent.REPETITIONS
ROS_DOMAIN_ID = 87


def _json_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False, default=str).encode("utf-8")
    ).hexdigest()


def _grid_hash(mask: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(np.asarray(mask, dtype=np.uint8)).tobytes()).hexdigest()


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    materialized = list(rows)
    fields: List[str] = []
    for row in materialized:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or ["empty"])
        writer.writeheader()
        for row in materialized:
            writer.writerow({
                key: json.dumps(value, sort_keys=True, default=str)
                if isinstance(value, (dict, list, tuple)) else value
                for key, value in row.items()
            })


def _truth(value: Any) -> bool:
    return value is True or str(value).strip().lower() in {"1", "true", "yes"}


def _number(row: Mapping[str, Any], field: str) -> Optional[float]:
    try:
        value = float(row.get(field))
    except (TypeError, ValueError):
        return None
    return value if np.isfinite(value) else None


def _percentiles(rows: Sequence[Mapping[str, Any]], field: str) -> Dict[str, Optional[float]]:
    values = [value for row in rows if (value := _number(row, field)) is not None]
    return {
        f"p{percentile}": float(np.percentile(values, percentile)) if values else None
        for percentile in (50, 95, 99)
    }


def _source_manifest() -> Tuple[Dict[str, str], str]:
    files, _ = r1._source_manifest()
    for module in (candidate, endpoint_heading, path_audit):
        source = Path(module.__file__).resolve()
        files[str(source)] = parent.sha256_file(source)
    source = Path(__file__).resolve()
    files[str(source)] = parent.sha256_file(source)
    for source in (
        ROOT / "external/arena4_ws/src/deps/nav2/navigation2/nav2_smac_planner/include/nav2_smac_planner/a_star.hpp",
        ROOT / "external/arena4_ws/src/deps/nav2/navigation2/nav2_smac_planner/include/nav2_smac_planner/smac_planner_hybrid.hpp",
        ROOT / "external/arena4_ws/src/deps/nav2/navigation2/nav2_smac_planner/src/a_star.cpp",
        ROOT / "external/arena4_ws/src/deps/nav2/navigation2/nav2_smac_planner/src/smac_planner_hybrid.cpp",
        ROOT / "external/arena4_ws/install/nav2_smac_planner/lib/libnav2_smac_planner.so",
    ):
        if source.exists():
            files[str(source.resolve())] = parent.sha256_file(source.resolve())
    return files, _json_hash(files)


class R2RouteMaskCache(r1.RouteMaskCache):
    """Cache masks bound to endpoint pose, Dubins attachment and map hash."""

    def __init__(self, ctx: Any, topology: Any, source_hash: str, cache_root: Path, *, endpoint_mode: str):
        super().__init__(ctx, topology, source_hash, cache_root)
        if endpoint_mode not in {"baseline", "yaw_dubins"}:
            raise ValueError("endpoint_mode must be baseline or yaw_dubins")
        self.endpoint_mode = endpoint_mode
        self.selector = endpoint_heading.YawAwareEndpointSelector(ctx.map_sha256, top_k=8, rmin_m=0.4)
        self.endpoint_envelope_cells = 0
        self.endpoint_variant_count = 0

    def route_selector(self, topology: Any, query: Any, *, cache_mode: str, timing: Optional[Dict[str, Any]] = None):
        if self.endpoint_mode == "baseline":
            return candidate._select_route_with_endpoint_attach(
                topology, query, cache_mode=cache_mode, timing=timing,
            )
        return self.selector(topology, query, cache_mode=cache_mode, timing=timing)

    def key(self, route: Any, query: Any, start_cell: Any, goal_cell: Any) -> str:
        return _json_hash({
            "r1_key": super().key(route, query, start_cell, goal_cell),
            "endpoint_variant": str(getattr(route, "r2_endpoint_cache_key", "baseline")),
            "minimum_turning_radius_m": 0.4,
            "complete_start_pose": [float(value) for value in query.start],
            "complete_goal_pose": [float(value) for value in query.goal],
        })

    def _endpoint_envelope_mask(self, selection: Any) -> np.ndarray:
        centerline = np.zeros((self.ctx.hospital_map.height, self.ctx.hospital_map.width), dtype=np.uint8)
        poses = [selection.start_envelope, selection.goal_envelope]
        for envelope in poses:
            for first, second in zip(envelope, envelope[1:]):
                candidate._draw_world_segment(self.ctx, centerline, first, second)
            for pose in envelope:
                cell = self.ctx.hospital_map.world_to_cell(float(pose[0]), float(pose[1]))
                if cell is not None:
                    centerline[cell] = 1
        kernel, _radius = r1._kernel(self.ctx, r1.BASE_CORRIDOR_PADDING_M)
        expanded = cv2.dilate(centerline, kernel, iterations=1).astype(bool)
        return expanded & candidate._raw_free_mask(self.ctx)

    def prepare(self, queries: Sequence[Any]) -> Dict[str, Any]:
        started_ns = time.monotonic_ns()
        manifest = super().prepare(queries)
        if self.endpoint_mode == "yaw_dubins":
            for query in queries:
                timing: Dict[str, Any] = {}
                _start, _goal, route, _reason = self.selector(
                    self.topology, query, cache_mode=candidate.CACHE_MODE_OPTIMIZED,
                    timing=timing,
                )
                selection = self.selector.selection(query)
                if route is None or selection is None:
                    continue
                start_cell, goal_cell = candidate._endpoint_cells(self.ctx, query)
                key = self.key(route, query, start_cell, goal_cell)
                base_mask, diagnostics = parent.build_adaptive_corridor_mask(
                    self.ctx, self.topology, route, query, start_cell, goal_cell,
                    r1.BASE_CORRIDOR_PADDING_M, r1.CORRIDOR_SEMANTICS,
                )
                envelope = self._endpoint_envelope_mask(selection)
                merged = np.asarray(base_mask, dtype=bool) | envelope
                envelope_added = int(np.count_nonzero(merged & ~np.asarray(base_mask, dtype=bool)))
                diagnostics = dict(diagnostics)
                diagnostics.update({
                    "precomputed_mask_hash": _grid_hash(merged),
                    "precomputed_allowed_cells": int(np.count_nonzero(merged)),
                    "route_signature": self.route_signature(route),
                    "endpoint_envelope_mask_hash": _grid_hash(envelope),
                    "endpoint_envelope_added_cells": envelope_added,
                    "endpoint_envelope_in_mask": True,
                    "endpoint_cache_key": self.selector.key(query),
                    "endpoint_start_dubins_word": selection.start_dubins_word,
                    "endpoint_goal_dubins_word": selection.goal_dubins_word,
                    "endpoint_start_yaw_error_rad": selection.start_yaw_error_rad,
                    "endpoint_goal_yaw_error_rad": selection.goal_yaw_error_rad,
                    "endpoint_selected_start_node_id": int(selection.start.node_id),
                    "endpoint_selected_goal_node_id": int(selection.goal.node_id),
                })
                self.route_masks[key] = (merged, diagnostics)
                self.route_analysis[key] = dict(diagnostics)
                self.endpoint_envelope_cells += envelope_added
                self.endpoint_variant_count += 1
        self.offline_build_ms = (time.monotonic_ns() - started_ns) / 1.0e6
        manifest = {
            **manifest,
            "cache_version": "2a-v1-r2-yaw-roi-pathaudit-v1",
            "endpoint_mode": self.endpoint_mode,
            "endpoint_top_k": 8,
            "minimum_turning_radius_m": 0.4,
            "complete_pose_in_cache_key": True,
            "endpoint_variant_count": self.endpoint_variant_count,
            "endpoint_envelope_added_cells": self.endpoint_envelope_cells,
            "route_count": len(self.route_masks),
            "offline_build_ms": self.offline_build_ms,
        }
        self.cache_root.mkdir(parents=True, exist_ok=True)
        (self.cache_root / "mask_cache_manifest.yaml").write_text(
            yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8",
        )
        return manifest


def _load_parity(path: Optional[Path], expected_source_hash: str) -> Dict[str, Any]:
    if path is None:
        return {}
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema") != "2a-v1-r2-deterministic-failure-parity-v1":
        raise ValueError("invalid deterministic failure parity evidence")
    if str(payload.get("source_hash") or "") != str(expected_source_hash):
        raise ValueError("deterministic failure parity source hash does not match this runner")
    return dict(payload.get("queries") or {})


def _failure_decider(parity: Mapping[str, Any]):
    def decide(query: Any, metrics: Mapping[str, Any], diagnostics: Mapping[str, Any], result: Any) -> bool:
        evidence = parity.get(query.query_id)
        if not isinstance(evidence, Mapping):
            return True
        code = str(metrics.get("failure_code") or getattr(result, "failure_code", ""))
        if code != str(evidence.get("failure_code", "")):
            return True
        if diagnostics.get("costmap_update_acknowledged") is not True:
            return True
        expected_mask = str(evidence.get("corridor_mask_hash") or "")
        actual_mask = str(diagnostics.get("corridor_mask_hash") or "")
        if not expected_mask or expected_mask != actual_mask:
            return True
        expected_route = str(evidence.get("route_signature") or "")
        actual_route = str(diagnostics.get("route_signature") or "")
        if not expected_route or expected_route != actual_route:
            return True
        if code == "STATIC_FOOTPRINT_COLLISION":
            expected_path = str(evidence.get("path_hash") or "")
            actual_path = str(diagnostics.get("canonical_path_hash") or "")
            if not expected_path or expected_path != actual_path:
                return True
        return False
    return decide


def _summarize(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    success = [row for row in measured if _truth(row.get("final_valid_success"))]
    failure = [row for row in measured if not _truth(row.get("final_valid_success"))]
    for row in measured:
        row["costmap_update_plus_smac_action_ms"] = (
            float(row.get("costmap_update_ms") or 0.0) + float(row.get("l3_action_wall_ms") or 0.0)
        )
        row["path_postprocess_ms"] = sum(float(row.get(field) or 0.0) for field in (
            "ros_path_conversion_ms", "point_annotation_ms", "canonical_path_audit_ms",
        ))
        row["path_validation_ms"] = sum(float(row.get(field) or 0.0) for field in (
            "footprint_validation_ms", "kinematic_validation_ms", "path_within_mask_ms",
        ))
    return {
        "measured_request_count": len(measured),
        "final_valid_count": len(success),
        "final_valid_rate": (len(success) / len(measured)) if measured else 0.0,
        "all_online_wall_ms": _percentiles(measured, "online_wall_ms"),
        "successful_online_wall_ms": _percentiles(success, "online_wall_ms"),
        "failed_detection_wall_ms": _percentiles(failure, "online_wall_ms"),
        "successful_costmap_update_plus_smac_action_ms": _percentiles(success, "costmap_update_plus_smac_action_ms"),
        "successful_costmap_update_ms": _percentiles(success, "costmap_update_ms"),
        "successful_smac_action_ms": _percentiles(success, "l3_action_wall_ms"),
        "successful_path_postprocess_ms": _percentiles(success, "path_postprocess_ms"),
        "successful_path_validation_ms": _percentiles(success, "path_validation_ms"),
        "ack_failure_count": sum(row.get("costmap_update_acknowledged") is not True for row in measured),
        "start_in_lethal_count": sum(str(row.get("failure_code")) == "START_IN_LETHAL_SPACE" for row in measured),
        "unexplained_full_fallback_count": sum(
            _truth(row.get("costmap_update_fallback")) and not str(row.get("costmap_update_fallback_reason") or "")
            for row in measured
        ),
    }


def _write_report(output: Path, rows: Sequence[Mapping[str, Any]], summary: Mapping[str, Any], *, costmap_mode: str, endpoint_mode: str) -> None:
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    by_query: Dict[str, List[Mapping[str, Any]]] = {}
    for row in measured:
        by_query.setdefault(str(row.get("query_id")), []).append(row)
    lines = [
        "# 2A-V1 r2 ROI/PathAudit report", "",
        f"- output: `{output}`",
        f"- costmap mode: `{costmap_mode}`",
        f"- endpoint mode: `{endpoint_mode}`",
        f"- final-valid: {summary['final_valid_count']}/{summary['measured_request_count']}",
        f"- all online P50/P95/P99 ms: {summary['all_online_wall_ms']}",
        f"- successful online P50/P95/P99 ms: {summary['successful_online_wall_ms']}",
        f"- failed detection P50/P95/P99 ms: {summary['failed_detection_wall_ms']}",
        f"- successful costmap+action P50/P95/P99 ms: {summary['successful_costmap_update_plus_smac_action_ms']}",
        f"- successful path postprocess P50/P95/P99 ms: {summary['successful_path_postprocess_ms']}",
        f"- successful validation P50/P95/P99 ms: {summary['successful_path_validation_ms']}",
        f"- ACK failures: {summary['ack_failure_count']}",
        f"- START_IN_LETHAL_SPACE: {summary['start_in_lethal_count']}", "",
        "## Per query", "",
        "| Query | Valid | Failure codes | Online P50 ms | Costmap+action P50 ms |", "|---|---:|---|---:|---:|",
    ]
    for query_id, values in sorted(by_query.items()):
        valid = sum(_truth(row.get("final_valid_success")) for row in values)
        codes = sorted({str(row.get("failure_code") or "") for row in values if row.get("failure_code")})
        lines.append(
            f"| {query_id} | {valid}/{len(values)} | {', '.join(codes) or '-'} | "
            f"{_percentiles(values, 'online_wall_ms')['p50']!s} | "
            f"{_percentiles(values, 'costmap_update_plus_smac_action_ms')['p50']!s} |"
        )
    lines.extend([
        "", "## Smac internal metric availability", "",
        "`ComputePathToPose` on the installed Nav2 Humble build exposes total `planning_time`, but does not expose expanded/generated states or separate heuristic-build, analytic-expansion, search, and smoothing timers. These fields are therefore not reported as measured values.", "",
    ])
    (output / "final_report.md").write_text("\n".join(lines), encoding="utf-8")


def run_experiment(
    output: Path, *, costmap_mode: str, endpoint_mode: str,
    warmups: int, repetitions: int, query_ids: Optional[Sequence[str]],
    ros_domain_id: int, topology_cache_dir: Path,
    failure_parity_file: Optional[Path] = None,
    planner_parameter_overrides: Optional[Mapping[str, Any]] = None,
) -> Path:
    if costmap_mode not in {"full", "roi_ack"}:
        raise ValueError("costmap_mode must be full or roi_ack")
    parent._refuse_nonempty(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir()
    queries, metadata = parent._load_tasks()
    selected = list(query_ids or [query.query_id for query in queries])
    query_map = {query.query_id: query for query in queries}
    if any(query_id not in query_map for query_id in selected):
        raise ValueError("query_ids must be A2B-01..A2B-20")
    queries = [query_map[query_id] for query_id in selected]
    ctx = parent._context()
    topology, topology_info = parent._load_or_build_topology(ctx, output, topology_cache_dir.resolve())
    source_files, source_hash = _source_manifest()
    cache = R2RouteMaskCache(ctx, topology, source_hash, output / "offline_mask_cache", endpoint_mode=endpoint_mode)
    cache_manifest = cache.prepare(queries)
    auditor = path_audit.PathAuditor(ctx, source_commit=parent.validity._source_commit() or "unknown")
    spec = parent.legacy.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"BACKEND_UNAVAILABLE: {spec.reason}")
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    session = candidate.SmacSession(
        ctx, output, map_yaml=parent.validity.MAP_YAML,
        log_tag=f"formal_2a_v1_r2_{costmap_mode}_{MAP_ID}", local_mask_updates=True,
        optimization_profile=r1.OPTIMIZATION_PROFILE,
        smac_parameter_profile=r1.SMAC_PARAMETER_PROFILE,
        optimization_stage=r1.OPTIMIZATION_STAGE,
        enable_mask_reuse_noop=True,
        planner_parameter_overrides=planner_parameter_overrides,
        costmap_ack_timeout_s=0.8,
    )
    session.local_map_update_strategy = "roi_ack" if costmap_mode == "roi_ack" else "v6_full"
    session.full_grid_settle_cycles = 0 if costmap_mode == "roi_ack" else 20
    session.start()
    rows: List[Dict[str, Any]] = []
    calls: List[Dict[str, Any]] = []
    metrics: List[Dict[str, Any]] = []
    parity = _load_parity(failure_parity_file, source_hash)
    fallback_decider = _failure_decider(parity) if parity else None
    try:
        for run_mode, count in (("warmup", warmups), ("measured", repetitions)):
            for repetition in range(1, int(count) + 1):
                for query in queries:
                    row, call, metric = candidate._run_one(
                        ctx, topology, topology_info, query, run_mode, repetition,
                        session, spec, output, parent.validity._source_commit(),
                        corridor_padding_m=r1.BASE_CORRIDOR_PADDING_M,
                        corridor_semantics=r1.CORRIDOR_SEMANTICS,
                        profile_name=r1.CORRIDOR_PROFILE,
                        padding_schedule_m=(r1.BASE_CORRIDOR_PADDING_M,),
                        validate_each_attempt=True,
                        cache_mode=candidate.CACHE_MODE_OPTIMIZED,
                        corridor_mask_builder=cache.builder,
                        route_selector=cache.route_selector,
                        canonical_path_auditor=auditor.audit,
                        skip_session_path_mask_validation=True,
                        baseline_fallback_decider=fallback_decider,
                    )
                    annotated = r1._annotate_row(output, row, query, metadata, topology_info, candidate.CACHE_MODE_OPTIMIZED)
                    annotated.update({
                        "source_hash": source_hash,
                        "implementation_revision": IMPLEMENTATION_REVISION,
                        "architecture_id": ARCHITECTURE_ID,
                        "costmap_mode": costmap_mode,
                        "endpoint_mode": endpoint_mode,
                    })
                    annotated["costmap_update_plus_smac_action_ms"] = (
                        float(annotated.get("costmap_update_ms") or 0.0)
                        + float(annotated.get("l3_action_wall_ms") or 0.0)
                    )
                    annotated["path_postprocess_ms"] = sum(float(annotated.get(field) or 0.0) for field in (
                        "ros_path_conversion_ms", "point_annotation_ms", "canonical_path_audit_ms",
                    ))
                    annotated["path_validation_ms"] = sum(float(annotated.get(field) or 0.0) for field in (
                        "footprint_validation_ms", "kinematic_validation_ms", "path_within_mask_ms",
                    ))
                    call = dict(call)
                    call.update({"architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION})
                    metric = dict(metric)
                    metric.update({
                        "experiment_id": output.name, "architecture_id": ARCHITECTURE_ID,
                        "implementation_revision": IMPLEMENTATION_REVISION,
                        "query_sha256": annotated.get("query_hash", ""),
                        "path_hash": annotated.get("path_hash", ""),
                    })
                    rows.append(annotated)
                    calls.append(call)
                    metrics.append(metric)
    finally:
        session.close()
    _write_csv(output / "runs.csv", rows)
    _write_csv(output / "backend_call_log.csv", calls)
    _write_csv(output / "path_metrics.csv", metrics)
    session_info = {
        "experiment_id": output.name, "ros_domain_id": ros_domain_id,
        "session_start_count": session.session_start_count,
        "session_close_count": session.session_close_count,
        "session_restart_count": session.session_restart_count,
        "session_startup_time_ms": session.stack_startup_time_ms,
        "session_shutdown_time_ms": session.stack_shutdown_time_ms,
        "topology_cache_hit": topology_info.get("topology_cache_hit", False),
        "topology_build_time_ms": topology_info.get("topology_build_time_ms", 0.0),
        "topology_load_time_ms": topology_info.get("topology_load_time_ms", 0.0),
        "mask_cache_offline_build_ms": cache.offline_build_ms,
    }
    _write_csv(output / "session_timing.csv", [session_info])
    _write_csv(output / "cache_diagnostics.csv", [
        {"metric": "route_cache_hits", "value": cache.route_hits},
        {"metric": "route_cache_misses", "value": cache.route_misses},
        {"metric": "endpoint_variant_count", "value": cache.endpoint_variant_count},
        {"metric": "endpoint_envelope_added_cells", "value": cache.endpoint_envelope_cells},
        {"metric": "offline_build_ms", "value": cache.offline_build_ms},
    ])
    summary = _summarize(rows)
    _write_csv(output / "per_query_summary.csv", [
        {
            "query_id": query_id,
            "measured_count": len(values),
            "final_valid_count": sum(_truth(row.get("final_valid_success")) for row in values),
            "failure_codes": sorted({str(row.get("failure_code") or "") for row in values if row.get("failure_code")}),
            "online_wall_p50_ms": _percentiles(values, "online_wall_ms")["p50"],
            "online_wall_p95_ms": _percentiles(values, "online_wall_ms")["p95"],
            "costmap_action_p50_ms": _percentiles(values, "costmap_update_plus_smac_action_ms")["p50"],
        }
        for query_id in selected
        for values in [[row for row in rows if row.get("run_mode") == "measured" and row.get("query_id") == query_id]]
    ])
    manifest = {
        "experiment_id": output.name, "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "parent_architecture": PARENT_ARCHITECTURE,
        "protocol_version": PROTOCOL_VERSION, "map_id": MAP_ID,
        "query_ids": selected, "warmup_count": warmups,
        "measured_repetitions": repetitions, "run_count": len(rows),
        "costmap_mode": costmap_mode, "endpoint_mode": endpoint_mode,
        "source_hash": source_hash, "cache_manifest": cache_manifest,
        "failure_parity_file": str(failure_parity_file) if failure_parity_file else "",
        **summary,
    }
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({
        "experiment_id": output.name, "source_hash": source_hash,
        "source_files": source_files, "map_id": MAP_ID,
        "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256,
    }, sort_keys=False), encoding="utf-8")
    (output / "protocol.yaml").write_text(yaml.safe_dump({
        "experiment_id": output.name, "protocol_version": PROTOCOL_VERSION,
        "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "map_id": MAP_ID, "query_ids": selected, "warmups": warmups,
        "repetitions": repetitions, "resolution_m": 0.05,
        "footprint": candidate.FOOTPRINT, "minimum_turning_radius_m": 0.4,
        "maximum_curvature_1pm": 2.5, "allow_reverse": False,
        "allow_in_place_rotation": False, "dynamic_obstacles": False,
        "costmap_mode": costmap_mode, "costmap_ack": costmap_mode == "roi_ack",
        "fixed_settle_cycles": 0 if costmap_mode == "roi_ack" else 20,
        "endpoint_mode": endpoint_mode, "endpoint_top_k": 8,
        "canonical_path_audit": True, "collision_sample_spacing_m": 0.05,
        "corridor_sample_spacing_m": 0.025,
        "planner_parameter_overrides": dict(planner_parameter_overrides or {}),
        "metric_availability": {
            "planning_time": "measured_from_ComputePathToPose_result",
            "expanded_generated_states": "unavailable_in_installed_Nav2_Humble_ComputePathToPose_API",
            "heuristic_analytic_smoothing_breakdown": "unavailable_in_installed_Nav2_Humble_ComputePathToPose_API",
        },
    }, sort_keys=False), encoding="utf-8")
    _write_report(output, rows, summary, costmap_mode=costmap_mode, endpoint_mode=endpoint_mode)
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run 2A-V1 r2 ROI/ACK PathAudit benchmark")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--topology-cache-dir", default=str(DEFAULT_TOPOLOGY_CACHE))
    parser.add_argument("--costmap-mode", choices=("full", "roi_ack"), default="roi_ack")
    parser.add_argument("--endpoint-mode", choices=("baseline", "yaw_dubins"), default="baseline")
    parser.add_argument("--warmups", type=int, default=WARMUPS)
    parser.add_argument("--repetitions", type=int, default=REPETITIONS)
    parser.add_argument("--query-id", action="append", dest="query_ids")
    parser.add_argument("--ros-domain-id", type=int, default=ROS_DOMAIN_ID)
    parser.add_argument("--failure-parity-file")
    parser.add_argument("--planner-overrides-json", default="{}")
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        overrides = json.loads(args.planner_overrides_json)
        if not isinstance(overrides, dict):
            raise ValueError("planner overrides must be a JSON object")
        output = run_experiment(
            Path(args.output_dir).resolve(), costmap_mode=args.costmap_mode,
            endpoint_mode=args.endpoint_mode, warmups=args.warmups,
            repetitions=args.repetitions, query_ids=args.query_ids,
            ros_domain_id=args.ros_domain_id,
            topology_cache_dir=Path(args.topology_cache_dir).resolve(),
            failure_parity_file=Path(args.failure_parity_file).resolve() if args.failure_parity_file else None,
            planner_parameter_overrides=overrides,
        )
    except Exception as exc:
        print(f"two_layer_v1_r2_roi_pathaudit_benchmark: ERROR: {exc}")
        return 2
    print(f"2A-V1-r2 output: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
