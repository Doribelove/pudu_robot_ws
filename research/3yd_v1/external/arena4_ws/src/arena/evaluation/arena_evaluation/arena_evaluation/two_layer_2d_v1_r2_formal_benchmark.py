"""Formal, independently reproducible 2D-V1-r2 latency benchmark."""

from __future__ import annotations

import argparse
import collections
import csv
import datetime as dt
import hashlib
import json
import os
import resource
import shutil
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import canonical_path_validation as canonical
from . import dynamic_snapshot as dynamic_snapshot_module
from . import graph_dstar_lite as dstar_module
from . import layered_2d_v0_pipeline as v0
from . import layered_2d_v1_pipeline as r1_pipeline
from . import layered_2d_v1_r2_pipeline as r2_pipeline
from . import topology
from . import two_layer_2d_v1_formal_benchmark as r1_benchmark
from . import unified_four_backends_smoke as legacy
from .dynamic_snapshot import DynamicSnapshot
from .planner_benchmark.map_utils import sha256_file
from .planner_benchmark import map_utils as map_utils_module
from .planner_benchmark import runner as benchmark_runner


ROOT = r1_benchmark.ROOT
MAP_ID = r1_benchmark.MAP_ID
MAP_YAML = r1_benchmark.MAP_YAML
TASK_IDS = r1_benchmark.TASK_IDS
ARCHITECTURE_ID = "2D-V1"
IMPLEMENTATION_REVISION = "r2"
PARENT_ARCHITECTURE = "2D-V1-r1"
PROTOCOL_VERSION = r1_benchmark.PROTOCOL_VERSION
EXPERIMENT_KIND = "static_formal"
QUERY_SET_ID = r1_benchmark.QUERY_SET_ID
WARMUPS = 3
REPETITIONS = 5
ROS_DOMAIN_ID = 224
R1_EXPERIMENT = ROOT / "experiments/layered_planner_benchmark/2d_v1_mentor_map_20260825_005_20_performance_r1_maskaligned_v2"
R1_TOPOLOGY_CACHE_KEY = "af45cba4e2772b5d8209efdc171ad4672a48b3f01697d60f5d421ac821d42b4c"
DEFAULT_CACHE_ROOT = ROOT / "experiments/layered_planner_benchmark/2d_v1_mentor_map_20260825_005_20_performance_v1_cache"
DEFAULT_OUTPUT = ROOT / "experiments/layered_planner_benchmark/2d_v1_mentor_map_20260825_005_20_performance_r2"
R2_CONFIG = Path(__file__).resolve().parents[1] / "config/two_layer_2d_v1_r2_latency.yaml"
CORRIDOR_SEMANTICS = r1_benchmark.CORRIDOR_SEMANTICS
CORRIDOR_PROFILE = r1_benchmark.CORRIDOR_PROFILE
PADDING_SCHEDULE = r1_benchmark.PADDING_SCHEDULE
SMAC_PARAMETER_PROFILE = r1_benchmark.SMAC_PARAMETER_PROFILE
OPTIMIZATION_PROFILE = r1_benchmark.OPTIMIZATION_PROFILE
OPTIMIZATION_STAGE = r1_benchmark.OPTIMIZATION_STAGE


PHASE_FIELDS = (
    "query_session_reset_ms",
    "query_session_reset_local_map_generation_ms",
    "query_session_reset_local_map_serialization_ms",
    "query_session_reset_local_map_publication_ms",
    "query_session_reset_costmap_clear_ms",
    "query_session_reset_costmap_settle_ms",
    "attachment_node_lookup_ms",
    "edge_projection_candidate_lookup_ms",
    "projection_connection_collision_filter_ms",
    "attachment_candidate_ranking_ms",
    "endpoint_cache_lookup_ms",
    "endpoint_cache_store_ms",
    "attachment_diagnostics_serialization_ms",
    "graph_construction_ms",
    "base_graph_construction_ms",
    "temporary_connection_edge_construction_ms",
    "dstar_graph_initialization_ms",
    "dstar_lite_search_ms",
    "route_extraction_ms",
    "route_edge_resolution_ms",
    "route_polyline_construction_ms",
    "corridor_mask_rasterization_ms",
    "corridor_mask_dilation_ms",
    "corridor_mask_hash_cell_diagnostics_ms",
    "corridor_mask_online_ms",
    "endpoint_diagnostics_ms",
    "local_map_generation_ms",
    "local_map_serialization_ms",
    "local_map_publication_ms",
    "local_costmap_clear_ms",
    "costmap_settle_ms",
    "local_map_update_ms",
    "action_wall_ms",
    "nav2_reported_planning_time_ms",
    "client_process_overhead_ms",
    "path_within_mask_check_ms",
    "pipeline_canonical_validation_ms",
    "dynamic_collision_diagnostics_ms",
    "serialization_write_ms",
    "benchmark_side_validation_diagnostics_ms",
    "ros_map_costmap_overhead_ms",
    "online_accounted_ms",
    "online_unaccounted_ms",
)


def _json_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    ).hexdigest()


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    materialized = list(rows)
    fields: List[str] = []
    for row in materialized:
        for key in row:
            if key not in fields:
                fields.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or ["empty"])
        writer.writeheader()
        for row in materialized:
            writer.writerow({
                key: json.dumps(value, sort_keys=True, default=str)
                if isinstance(value, (dict, list, tuple)) else value
                for key, value in row.items()
            })


def _numeric(value: Any) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _truth(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _percentile(rows: Sequence[Mapping[str, Any]], field: str, percentile: float) -> Optional[float]:
    values = [_numeric(row.get(field)) for row in rows]
    materialized = [value for value in values if value is not None]
    return float(np.percentile(materialized, percentile)) if materialized else None


def _pss_bytes() -> Any:
    return r1_benchmark._pss_bytes() or "not_available"


def _refuse_nonempty(path: Path) -> None:
    r1_benchmark._refuse_nonempty(path)


def _online_accounted_ms(
    reset_info: Mapping[str, Any],
    diagnostics: Mapping[str, Any],
    serialization_write_ms: float,
    benchmark_diagnostics_ms: float,
) -> float:
    """Sum only non-overlapping online aggregates.

    Leaf diagnostics such as attachment lookup, D* Lite, dilation, costmap
    clear and Nav2 planning are reported separately but are already contained
    by the aggregate phases below and therefore must not be added here.
    """
    return sum((
        float(reset_info.get("query_session_reset_ms") or 0.0),
        float(diagnostics.get("l1_total_time_ms") or 0.0),
        float(diagnostics.get("route_polyline_construction_time_ms") or 0.0),
        float(diagnostics.get("corridor_mask_total_time_ms") or 0.0),
        float(diagnostics.get("endpoint_diagnostics_time_ms") or 0.0),
        float(diagnostics.get("local_map_update_ms") or 0.0),
        float(diagnostics.get("l3_action_wall_ms") or 0.0),
        float(diagnostics.get("path_within_mask_check_ms") or 0.0),
        float(diagnostics.get("pipeline_validation_time_ms") or 0.0),
        float(diagnostics.get("dynamic_collision_diagnostics_time_ms") or 0.0),
        float(serialization_write_ms),
        float(benchmark_diagnostics_ms),
    ))


def _audit_current_r1_sources(source_manifest: Mapping[str, Any]) -> Dict[str, Any]:
    expected = dict(source_manifest.get("source_files") or {})
    unchanged: List[str] = []
    mismatched: List[Dict[str, str]] = []
    missing: List[str] = []
    for name, expected_hash in expected.items():
        path = Path(str(name))
        if not path.is_file():
            missing.append(str(path))
            continue
        current_hash = sha256_file(path)
        if current_hash == str(expected_hash):
            unchanged.append(str(path))
        else:
            mismatched.append({
                "path": str(path), "r1_hash": str(expected_hash),
                "current_hash": current_hash,
            })
    return {
        "r1_manifest_file_count": len(expected),
        "r1_manifest_current_match_count": len(unchanged),
        "r1_manifest_current_mismatches": mismatched,
        "r1_manifest_current_missing": missing,
    }


def _load_r2_config() -> Dict[str, Any]:
    config = yaml.safe_load(R2_CONFIG.read_text(encoding="utf-8")) or {}
    required = {
        ("architecture_id",): ARCHITECTURE_ID,
        ("implementation_revision",): IMPLEMENTATION_REVISION,
        ("topology", "representation"): "2a_v0_static_skeleton_graph",
        ("topology", "search"): "graph_dstar_lite",
        ("topology", "refined_topology"): False,
        ("topology", "frozen_r1_cache_key"): R1_TOPOLOGY_CACHE_KEY,
        ("layers", "l2_enabled"): False,
        ("layers", "l3_backend"): "nav2_smac_planner_hybrid",
        ("layers", "motion_model"): "DUBIN",
        ("map_and_motion", "resolution_m"): 0.05,
        ("map_and_motion", "minimum_turning_radius_m"): 0.40,
        ("map_and_motion", "maximum_curvature_1pm"): 2.50,
        ("map_and_motion", "allow_reverse"): False,
        ("map_and_motion", "allow_in_place_rotation"): False,
        ("formal_experiment", "dynamic_obstacles"): False,
        ("formal_experiment", "restore_base_map_each_query"): True,
        ("formal_experiment", "force_full_corridor_update"): True,
        ("formal_experiment", "smac_parameters_changed_from_r1"): False,
    }
    for path, expected in required.items():
        value: Any = config
        for key in path:
            value = value[key]
        if value != expected:
            raise ValueError(
                f"r2 config contract mismatch for {'.'.join(path)}: "
                f"{value!r} != {expected!r}"
            )
    return config


def _load_frozen_r1_topology(
    ctx: Any, cache_root: Path,
) -> Tuple[Any, Dict[str, Any], Dict[str, Any]]:
    """Load the exact numpy_zhang_suen artifact used by the formal r1 run."""
    started_ns = time.monotonic_ns()
    source_manifest = yaml.safe_load(
        (R1_EXPERIMENT / "source_manifest.yaml").read_text(encoding="utf-8")
    ) or {}
    r1_manifest = yaml.safe_load(
        (R1_EXPERIMENT / "manifest.yaml").read_text(encoding="utf-8")
    ) or {}
    expected_key = str(r1_manifest.get("topology_cache_key") or "")
    if expected_key != R1_TOPOLOGY_CACHE_KEY:
        raise ValueError(f"r1 topology key changed: {expected_key!r}")
    directory = Path(cache_root).resolve() / MAP_ID / expected_key
    cache_manifest = yaml.safe_load(
        (directory / "topology_cache_manifest.yaml").read_text(encoding="utf-8")
    ) or {}
    metadata = dict(cache_manifest.get("metadata") or {})
    if cache_manifest.get("cache_key") != expected_key:
        raise ValueError("frozen r1 topology cache manifest key mismatch")
    if metadata.get("skeleton_backend") != "numpy_zhang_suen":
        raise ValueError("frozen r1 topology must use numpy_zhang_suen")
    artifact = topology.load_topology(
        directory, ctx.hospital_map, legacy.FOOTPRINT,
        padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    if len(artifact.graph.nodes) != 2114 or len(artifact.graph.edges) != 2172:
        raise ValueError("frozen r1 topology graph size mismatch")
    elapsed_ms = (time.monotonic_ns() - started_ns) / 1.0e6
    info = {
        "topology_cache_hit": True,
        "topology_cache_key": expected_key,
        "topology_load_time_ms": elapsed_ms,
        "topology_build_time_ms": 0.0,
        "topology_build_count": 0,
        "topology_load_count": 1,
        "topology_cache_directory": str(directory),
        "topology_cache_bytes": r1_benchmark._directory_bytes(directory),
        "topology_source_hash": metadata.get("source_hash", ""),
        "skeleton_backend": metadata.get("skeleton_backend"),
    }
    audit = {
        "r1_source_manifest_hash": source_manifest.get("source_hash", ""),
        "source_manifest_files_match_before_r2": True,
        "source_manifest_preimplementation_match_count": len(
            source_manifest.get("source_files") or {}
        ),
        "source_manifest_preimplementation_audit": (
            "All r1 source_manifest files were hash-checked before r2 edits."
        ),
        "frozen_topology_cache_key": expected_key,
        "frozen_skeleton_backend": metadata.get("skeleton_backend"),
        "current_auto_selected_skeleton_backend": (
            "scikit-image" if topology._has_skimage() else "numpy_zhang_suen"
        ),
        "dependency_backend_difference_detected": bool(topology._has_skimage()),
        "resolution_m": float(ctx.hospital_map.resolution),
        "graph_nodes": len(artifact.graph.nodes),
        "graph_edges": len(artifact.graph.edges),
        **_audit_current_r1_sources(source_manifest),
    }
    return artifact, info, audit


def _source_manifest() -> Tuple[Dict[str, str], str]:
    files = [
        Path(__file__).resolve(), Path(r2_pipeline.__file__).resolve(),
        Path(canonical.__file__).resolve(), Path(r1_pipeline.__file__).resolve(),
        Path(v0.__file__).resolve(), Path(legacy.__file__).resolve(),
        Path(topology.__file__).resolve(), Path(dynamic_snapshot_module.__file__).resolve(),
        Path(dstar_module.__file__).resolve(), Path(benchmark_runner.__file__).resolve(),
        Path(map_utils_module.__file__).resolve(),
        Path(r1_pipeline.__file__).resolve().parents[1] / "setup.py",
        MAP_YAML, MAP_YAML.parent / "map.pgm", r1_benchmark.BENCHMARK_JSON,
        r1_benchmark.BENCHMARK_CSV, r1_benchmark.SCENARIO_JSON,
        legacy._strict_smac_config_path(),
        R2_CONFIG,
    ]
    hashes = {str(path): sha256_file(path) for path in files}
    return hashes, _json_hash(hashes)


def _snapshot_sources(
    output: Path, source_files: Mapping[str, str], source_hash: str,
) -> Dict[str, Any]:
    """Preserve the exact executable inputs used by a no-commit experiment."""
    snapshot_dir = output / "source_snapshot"
    snapshot_dir.mkdir()
    entries: List[Dict[str, str]] = []
    for index, (name, expected_hash) in enumerate(source_files.items()):
        source = Path(name)
        target = snapshot_dir / f"{index:02d}_{source.name}"
        shutil.copyfile(source, target)
        snapshot_hash = sha256_file(target)
        if snapshot_hash != expected_hash:
            raise RuntimeError(f"source changed while snapshotting: {source}")
        entries.append({
            "original_path": str(source),
            "snapshot_path": str(target.relative_to(output)),
            "sha256": snapshot_hash,
        })
    manifest = {
        "source_hash": source_hash,
        "file_count": len(entries),
        "files": entries,
    }
    (output / "source_snapshot_manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8",
    )
    return manifest


def _empty_validation(failure_code: str) -> Dict[str, Any]:
    return {
        "static_footprint_valid": False, "kinematic_valid": False,
        "final_valid_success": False, "path_length_m": None,
        "minimum_clearance_m": None, "maximum_curvature": None,
        "curvature_p95": None, "heading_discontinuity_count": 0,
        "position_discontinuity_count": 0, "steering_jump_count": 0,
        "reverse_distance_m": 0.0, "in_place_rotation_count": 0,
        "collision_count": 0, "collision_segment_indices": [],
        "collision_positions": [], "failure_code": failure_code or "EMPTY_PATH",
        "failure_detail": failure_code or "EMPTY_PATH", "path_point_count": 0,
        "euclidean_ratio": None, "total_heading_change_rad": 0.0,
        "large_turn_count": 0,
        "canonical_validation_version": canonical.CANONICAL_VALIDATION_VERSION,
        "canonical_validation_time_ms": 0.0,
    }


def _run_one(
    ctx: Any,
    graph: Any,
    topology_info: Mapping[str, Any],
    query: Any,
    run_mode: str,
    repetition: int,
    session: Any,
    adapter: Any,
    pipeline: r2_pipeline.Layered2DV1R2Pipeline,
    output: Path,
    snapshot: DynamicSnapshot,
    source_commit: str,
) -> Tuple[Dict[str, Any], Dict[str, Any], Dict[str, Any], Dict[str, Any]]:
    run_id = f"{MAP_ID}_{query.query_id}_2d_v1_r2_{run_mode}_{repetition}"
    started_ns = time.monotonic_ns()
    cpu_before = resource.getrusage(resource.RUSAGE_SELF)
    reset_info = session.reset_query_state(query.query_id, restore_base_map=True)
    calls_before = int(adapter.calls)
    result = pipeline.plan_initial(query, snapshot)
    query_calls = max(0, int(adapter.calls) - calls_before)
    diagnostics = dict(result.diagnostics or {})

    serialization_started_ns = time.monotonic_ns()
    points = [dict(point) for point in (result.points or [])]
    path_hash = str(points[0].get("path_hash", "")) if points else ""
    if points and not path_hash:
        path_hash = v0._enrich_path(points, source_commit)
    path_file = ""
    if points:
        path_file = f"paths/{run_id}.json"
        (output / path_file).write_text(
            json.dumps(points, indent=2, sort_keys=True), encoding="utf-8",
        )
    serialization_write_ms = (time.monotonic_ns() - serialization_started_ns) / 1.0e6

    benchmark_diag_started_ns = time.monotonic_ns()
    if points:
        if diagnostics.get("canonical_validation_version") != canonical.CANONICAL_VALIDATION_VERSION:
            raise RuntimeError("pipeline did not return the canonical validation result")
        validation = diagnostics
    else:
        validation = _empty_validation(result.failure_code)
    final_valid = bool(result.success and validation.get("final_valid_success"))
    failure_code = "" if final_valid else str(
        validation.get("failure_code") or diagnostics.get("failure_code")
        or result.failure_code or "L3_PLANNER_FAILED"
    )
    benchmark_diag_ms = (time.monotonic_ns() - benchmark_diag_started_ns) / 1.0e6
    wall_ms = (time.monotonic_ns() - started_ns) / 1.0e6
    cpu_after = resource.getrusage(resource.RUSAGE_SELF)
    cpu_ms = max(
        0.0,
        (cpu_after.ru_utime - cpu_before.ru_utime
         + cpu_after.ru_stime - cpu_before.ru_stime) * 1000.0,
    )

    action_wall_ms = float(diagnostics.get("l3_action_wall_ms") or 0.0)
    nav2_planning_ms = float(diagnostics.get("planning_time_ms") or 0.0)
    local_update_ms = float(diagnostics.get("local_map_update_ms") or 0.0)
    pipeline_validation_ms = float(diagnostics.get("pipeline_validation_time_ms") or 0.0)
    accounted_ms = _online_accounted_ms(
        reset_info, diagnostics, serialization_write_ms, benchmark_diag_ms,
    )
    pss = _pss_bytes()
    row: Dict[str, Any] = {
        "run_id": run_id, "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(),
        "architecture": ARCHITECTURE_ID, "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "parent_architecture": PARENT_ARCHITECTURE,
        "protocol_version": PROTOCOL_VERSION, "experiment_kind": EXPERIMENT_KIND,
        "map_id": MAP_ID, "map_sha256": ctx.map_sha256,
        "map_yaml_sha256": ctx.map_yaml_sha256, "query_id": query.query_id,
        "query_hash": r1_benchmark._query_hash(query),
        "query_sha256": r1_benchmark._query_hash(query),
        "run_mode": run_mode, "repetition": repetition,
        "start_x": query.start[0], "start_y": query.start[1], "start_yaw": query.start[2],
        "goal_x": query.goal[0], "goal_y": query.goal[1], "goal_yaw": query.goal[2],
        "l1_backend": r2_pipeline.L1_BACKEND,
        "l1_state_type": "original_topology_node_id",
        "topology_representation": "2a_v0_static_skeleton_graph",
        "topology_refinement_enabled": False,
        "topology_node_count": graph.node_count, "topology_edge_count": graph.edge_count,
        "topology_cache_hit": True,
        "topology_cache_key": topology_info["topology_cache_key"],
        "topology_cache_load_time_ms": topology_info["topology_load_time_ms"],
        "topology_build_time_ms": 0.0, "topology_build_count": 0,
        "topology_load_count": 1,
        "l1_success": bool(diagnostics.get("topology_node_ids")), "l1_call_count": 1,
        "l1_attachment_lookup_ms": diagnostics.get("attachment_lookup_time_ms", 0.0),
        "l1_candidate_collision_check_ms": diagnostics.get("candidate_collision_check_time_ms", 0.0),
        "l1_graph_search_ms": diagnostics.get("dstar_lite_search_time_ms", 0.0),
        "l1_route_search_ms": diagnostics.get("dstar_lite_search_time_ms", 0.0),
        "l1_route_construction_ms": diagnostics.get("route_construction_time_ms", 0.0),
        "l1_total_time_ms": diagnostics.get("l1_total_time_ms", 0.0),
        "l1_timing_accounting_version": "non_overlapping_leaf_sum_v1",
        "l1_dstar_initial_time_ms": diagnostics.get("l1_dstar_initial_time_ms", 0.0),
        "l1_dstar_incremental_time_ms": diagnostics.get("l1_dstar_incremental_time_ms", 0.0),
        "dstar_expanded_nodes": diagnostics.get("dstar_expanded_nodes", 0),
        "dstar_generated_nodes": diagnostics.get("dstar_generated_nodes", 0),
        "dstar_queue_pops": diagnostics.get("dstar_queue_pops", 0),
        "dstar_queue_pushes": diagnostics.get("dstar_queue_pushes", 0),
        "dstar_timeout_triggered": diagnostics.get("dstar_timeout_triggered", False),
        "dstar_no_path": diagnostics.get("dstar_no_path", False),
        "l2_called": False, "l2_call_count": 0,
        "l3_backend": r2_pipeline.L3_BACKEND, "l3_call_count": query_calls,
        "l3_call_count_total": int(adapter.calls), "l3_prime_call_count": query_calls,
        "l3_retry_count": max(0, query_calls - 1),
        "corridor_semantics": CORRIDOR_SEMANTICS, "corridor_profile": CORRIDOR_PROFILE,
        "corridor_initial_padding_m": diagnostics.get("corridor_initial_padding_m", 2.0),
        "corridor_padding_m": diagnostics.get("corridor_padding_m", 2.0),
        "corridor_extra_margin_m": diagnostics.get("corridor_extra_margin_m", 0.2),
        "corridor_retry_paddings_m": diagnostics.get("corridor_retry_paddings_m", []),
        "corridor_fallback_used": diagnostics.get("corridor_fallback_used", False),
        "corridor_fallback_attempt_count": diagnostics.get("corridor_fallback_attempt_count", 0),
        "corridor_mask_hash": diagnostics.get("corridor_mask_hash", ""),
        "corridor_allowed_cells": diagnostics.get("corridor_allowed_cells", 0),
        "corridor_total_free_cells": diagnostics.get("corridor_total_free_cells", 0),
        "corridor_area_ratio": diagnostics.get("corridor_area_ratio", 0.0),
        "corridor_min_clearance_m": diagnostics.get("corridor_min_clearance_m", "not_available"),
        "corridor_cache_key": diagnostics.get("corridor_cache_key", ""),
        "corridor_route_signature": diagnostics.get("corridor_route_signature", ""),
        "corridor_cache_hit": diagnostics.get("corridor_cache_hit", False),
        "corridor_cache_memory_bytes": diagnostics.get("corridor_cache_memory_bytes", pipeline.corridor_cache.memory_bytes),
        "corridor_cache_build_time_ms": diagnostics.get("corridor_cache_build_time_ms", 0.0),
        "corridor_cache_lookup_time_ms": diagnostics.get("corridor_cache_lookup_time_ms", 0.0),
        "endpoint_cache_hits": diagnostics.get("endpoint_cache_hits", pipeline.endpoint_cache_hits),
        "endpoint_cache_misses": diagnostics.get("endpoint_cache_misses", pipeline.endpoint_cache_misses),
        "start_endpoint_cache_hit": diagnostics.get("start_endpoint_cache_hit", False),
        "goal_endpoint_cache_hit": diagnostics.get("goal_endpoint_cache_hit", False),
        "endpoint_cache_memory_bytes": diagnostics.get("endpoint_cache_memory_bytes", pipeline.endpoint_cache_memory_bytes),
        "edge_segment_index_build_time_ms": pipeline.edge_segment_index.build_time_ms,
        "edge_segment_index_memory_bytes": pipeline.edge_segment_index.memory_bytes,
        "edge_projection_segments_scanned": diagnostics.get("edge_projection_segments_scanned", 0),
        "edge_projection_segments_total": diagnostics.get("edge_projection_segments_total", 0),
        "topology_node_ids": diagnostics.get("topology_node_ids", []),
        "topology_edge_ids": diagnostics.get("topology_edge_ids", []),
        "route_polyline": diagnostics.get("route_polyline", []),
        "selected_start_attachment": diagnostics.get("selected_start_attachment", "not_available"),
        "selected_goal_attachment": diagnostics.get("selected_goal_attachment", "not_available"),
        "selected_connection_edges": diagnostics.get("selected_connection_edges", []),
        "start_attachment_candidates": diagnostics.get("start_attachment_candidates", []),
        "goal_attachment_candidates": diagnostics.get("goal_attachment_candidates", []),
        "endpoint_attachment_policy": diagnostics.get("endpoint_attachment_policy", "not_available"),
        "attachment_rank_penalty_m": diagnostics.get("attachment_rank_penalty_m", "not_available"),
        "pipeline_wall_time_ms": wall_ms, "online_wall_ms": wall_ms,
        "pipeline_cpu_total_ms": cpu_ms, "cpu_ms": cpu_ms,
        "peak_rss": int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024,
        "RSS": int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024,
        "peak_pss": pss, "PSS": pss,
        "hybrid_planning_time_ms": nav2_planning_ms, "l3_time_ms": nav2_planning_ms,
        "action_status": diagnostics.get("action_status", "not_available"),
        "action_result_code": diagnostics.get("action_result_code", "not_available"),
        "planner_search_started": diagnostics.get("planner_search_started", "not_available"),
        "smac_failure_code": diagnostics.get("smac_failure_code", ""),
        "attempts": diagnostics.get("attempts", []),
        "static_footprint_valid": validation.get("static_footprint_valid", False),
        "kinematic_valid": validation.get("kinematic_valid", False),
        "path_length_m": validation.get("path_length_m"),
        "minimum_clearance_m": validation.get("minimum_clearance_m"),
        "maximum_curvature": validation.get("maximum_curvature"),
        "curvature_p95": validation.get("curvature_p95"),
        "heading_discontinuity_count": validation.get("heading_discontinuity_count", 0),
        "position_discontinuity_count": validation.get("position_discontinuity_count", 0),
        "steering_jump_count": validation.get("steering_jump_count", 0),
        "reverse_distance_m": validation.get("reverse_distance_m", 0.0),
        "in_place_rotation_count": validation.get("in_place_rotation_count", 0),
        "start_yaw_error_rad": validation.get("start_yaw_error_rad"),
        "goal_yaw_error_rad": validation.get("goal_yaw_error_rad"),
        "collision_count": validation.get("collision_count", 0),
        "collision_segment_indices": validation.get("collision_segment_indices", []),
        "collision_positions": validation.get("collision_positions", []),
        "canonical_validation_version": validation.get("canonical_validation_version"),
        "canonical_validation_reused": True,
        "final_valid_success": final_valid,
        "action_success": bool(points and diagnostics.get("action_result_code") == "SUCCEEDED"),
        "result_code": "SUCCEEDED" if final_valid else failure_code,
        "failure_code": failure_code,
        "failure_detail": validation.get("failure_detail", diagnostics.get("failure_detail", "")),
        "path_hash": path_hash, "path_file": path_file,
        "path_point_count": validation.get("path_point_count", len(points)),
        "euclidean_ratio": validation.get("euclidean_ratio"),
        "reference_ratio": "not_available", "mean_clearance_m": "not_available",
        "heading_change_rate_p95": "not_available",
        "total_heading_change_rad": validation.get("total_heading_change_rad", 0.0),
        "large_turn_count": validation.get("large_turn_count", 0),
        "snapshot_id": snapshot.snapshot_id, "snapshot_hash": snapshot.snapshot_hash,
        "query_session_reset_mode": reset_info.get("query_session_reset_mode"),
        "query_session_reset_fallback": reset_info.get("session_reset_fallback", False),
        "query_session_reset_fallback_reason": reset_info.get("session_reset_fallback_reason", ""),
        "source_commit": source_commit,
        "query_session_reset_ms": reset_info.get("query_session_reset_ms", 0.0),
        "query_session_reset_local_map_generation_ms": reset_info.get("query_session_reset_local_map_generation_ms", 0.0),
        "query_session_reset_local_map_serialization_ms": reset_info.get("query_session_reset_local_map_serialization_ms", 0.0),
        "query_session_reset_local_map_publication_ms": reset_info.get("query_session_reset_local_map_publication_ms", 0.0),
        "query_session_reset_costmap_clear_ms": reset_info.get("query_session_reset_costmap_clear_ms", 0.0),
        "query_session_reset_costmap_settle_ms": reset_info.get("query_session_reset_costmap_settle_ms", 0.0),
        "attachment_node_lookup_ms": diagnostics.get("attachment_node_lookup_time_ms", 0.0),
        "edge_projection_candidate_lookup_ms": diagnostics.get("edge_projection_candidate_lookup_time_ms", 0.0),
        "projection_connection_collision_filter_ms": diagnostics.get("projection_connection_collision_filter_time_ms", 0.0),
        "attachment_candidate_ranking_ms": diagnostics.get("attachment_candidate_ranking_time_ms", 0.0),
        "endpoint_cache_lookup_ms": diagnostics.get("endpoint_cache_lookup_time_ms", 0.0),
        "endpoint_cache_store_ms": diagnostics.get("endpoint_cache_store_time_ms", 0.0),
        "attachment_diagnostics_serialization_ms": diagnostics.get("attachment_diagnostics_serialization_time_ms", 0.0),
        "graph_construction_ms": diagnostics.get("graph_construction_time_ms", 0.0),
        "base_graph_construction_ms": diagnostics.get("base_graph_construction_time_ms", 0.0),
        "temporary_connection_edge_construction_ms": diagnostics.get("temporary_connection_edge_construction_time_ms", 0.0),
        "dstar_graph_initialization_ms": diagnostics.get("dstar_graph_initialization_time_ms", 0.0),
        "dstar_lite_search_ms": diagnostics.get("dstar_lite_search_time_ms", 0.0),
        "route_extraction_ms": diagnostics.get("route_extraction_time_ms", 0.0),
        "route_edge_resolution_ms": diagnostics.get("route_edge_resolution_time_ms", 0.0),
        "route_polyline_construction_ms": diagnostics.get("route_polyline_construction_time_ms", 0.0),
        "corridor_mask_rasterization_ms": diagnostics.get("corridor_mask_rasterization_ms", 0.0),
        "corridor_mask_dilation_ms": diagnostics.get("corridor_mask_dilation_ms", 0.0),
        "corridor_mask_hash_cell_diagnostics_ms": float(diagnostics.get("corridor_mask_hash_diagnostics_ms") or 0.0) + float(diagnostics.get("corridor_mask_cell_count_diagnostics_ms") or 0.0),
        "corridor_mask_online_ms": diagnostics.get("corridor_mask_total_time_ms", 0.0),
        "endpoint_diagnostics_ms": diagnostics.get("endpoint_diagnostics_time_ms", 0.0),
        "local_map_generation_ms": diagnostics.get("local_map_generation_ms", 0.0),
        "local_map_serialization_ms": diagnostics.get("local_map_serialization_ms", 0.0),
        "local_map_publication_ms": diagnostics.get("local_map_publication_ms", 0.0),
        "local_costmap_clear_ms": diagnostics.get("local_costmap_clear_ms", 0.0),
        "costmap_settle_ms": diagnostics.get("costmap_settle_ms", 0.0),
        "local_map_update_ms": local_update_ms,
        "action_wall_ms": action_wall_ms,
        "nav2_reported_planning_time_ms": nav2_planning_ms,
        "client_process_overhead_ms": max(0.0, action_wall_ms - nav2_planning_ms),
        "path_within_mask_check_ms": diagnostics.get("path_within_mask_check_ms", 0.0),
        "pipeline_canonical_validation_ms": pipeline_validation_ms,
        "canonical_validation_time_ms": diagnostics.get("canonical_validation_time_ms", 0.0),
        "dynamic_collision_diagnostics_ms": diagnostics.get("dynamic_collision_diagnostics_time_ms", 0.0),
        "serialization_write_ms": serialization_write_ms,
        "benchmark_side_validation_diagnostics_ms": benchmark_diag_ms,
        "online_accounted_ms": accounted_ms,
        "online_unaccounted_ms": wall_ms - accounted_ms,
        "ros_map_costmap_overhead_ms": (
            float(reset_info.get("query_session_reset_ms") or 0.0)
            + local_update_ms + max(0.0, action_wall_ms - nav2_planning_ms)
        ),
    }
    call = {
        "run_id": run_id, "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION, "query_id": query.query_id,
        "query_hash": row["query_hash"], "run_mode": run_mode,
        "repetition": repetition, "stage": "L3",
        "role": "l3_prime_full_corridor_hybrid",
        "planner_backend": r2_pipeline.L3_BACKEND, "called": bool(query_calls),
        "physical_backend_call_count": query_calls, "l3_call_count": query_calls,
        "l2_called": False, "l2_call_count": 0,
        "final_valid_success": final_valid, "failure_code": failure_code,
        "planner_search_started": row["planner_search_started"],
        "corridor_mask_hash": row["corridor_mask_hash"],
        "corridor_padding_m": row["corridor_padding_m"],
        "attempt_count": len(diagnostics.get("attempts", [])),
        "attempts": diagnostics.get("attempts", []),
        "topology_refinement_enabled": False,
    }
    metric = {
        "run_id": run_id, "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "query_id": query.query_id, "query_hash": row["query_hash"],
        "path_hash": path_hash, "final_valid_success": final_valid,
        "static_footprint_valid": row["static_footprint_valid"],
        "kinematic_valid": row["kinematic_valid"],
        "path_length_m": row["path_length_m"],
        "minimum_clearance_m": row["minimum_clearance_m"],
        "maximum_curvature": row["maximum_curvature"],
        "curvature_p95": row["curvature_p95"],
        "heading_discontinuity_count": row["heading_discontinuity_count"],
        "position_discontinuity_count": row["position_discontinuity_count"],
        "steering_jump_count": row["steering_jump_count"],
        "reverse_distance_m": row["reverse_distance_m"],
        "in_place_rotation_count": row["in_place_rotation_count"],
        "collision_count": row["collision_count"],
        "collision_segment_indices": row["collision_segment_indices"],
        "collision_positions": row["collision_positions"],
    }
    phase = {"run_id": run_id, "query_id": query.query_id, "run_mode": run_mode,
             "repetition": repetition, "final_valid_success": final_valid}
    phase.update({field: row.get(field) for field in PHASE_FIELDS})
    return row, call, metric, phase


def _normal_json(value: Any) -> Any:
    if isinstance(value, str):
        try:
            return json.loads(value)
        except (ValueError, TypeError):
            return value
    return value


def _write_correctness_parity(output: Path, rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    with (R1_EXPERIMENT / "runs.csv").open(newline="", encoding="utf-8") as stream:
        reference = {
            (row["query_id"], int(row["repetition"])): row
            for row in csv.DictReader(stream) if row["run_mode"] == "measured"
        }
    comparisons = []
    for row in rows:
        if row.get("run_mode") != "measured":
            continue
        old = reference[(str(row["query_id"]), int(row["repetition"]))]
        old_path_file = str(old.get("path_file") or "")
        new_path_file = str(row.get("path_file") or "")
        old_points = (
            json.loads((R1_EXPERIMENT / old_path_file).read_text(encoding="utf-8"))
            if old_path_file else []
        )
        new_points = (
            json.loads((output / new_path_file).read_text(encoding="utf-8"))
            if new_path_file else []
        )
        ignored_provenance_fields = {"path_hash", "source_commit"}
        old_geometry = [
            {key: value for key, value in point.items() if key not in ignored_provenance_fields}
            for point in old_points
        ]
        new_geometry = [
            {key: value for key, value in point.items() if key not in ignored_provenance_fields}
            for point in new_points
        ]
        geometry_equal = old_geometry == new_geometry
        raw_path_hash_equal = str(old["path_hash"]) == str(row["path_hash"])
        comparisons.append({
            "query_id": row["query_id"], "repetition": row["repetition"],
            "final_valid_equal": _truth(old["final_valid_success"]) == bool(row["final_valid_success"]),
            "failure_code_equal": str(old["failure_code"]) == str(row["failure_code"]),
            "route_node_ids_equal": _normal_json(old["topology_node_ids"]) == row["topology_node_ids"],
            "route_edge_ids_equal": _normal_json(old["topology_edge_ids"]) == row["topology_edge_ids"],
            "start_attachment_candidates_equal": _normal_json(old["start_attachment_candidates"]) == row["start_attachment_candidates"],
            "goal_attachment_candidates_equal": _normal_json(old["goal_attachment_candidates"]) == row["goal_attachment_candidates"],
            "selected_start_attachment_equal": _normal_json(old["selected_start_attachment"]) == row["selected_start_attachment"],
            "selected_goal_attachment_equal": _normal_json(old["selected_goal_attachment"]) == row["selected_goal_attachment"],
            "selected_connection_edges_equal": _normal_json(old["selected_connection_edges"]) == row["selected_connection_edges"],
            "mask_hash_equal": str(old["corridor_mask_hash"]) == str(row["corridor_mask_hash"]),
            "allowed_cells_equal": int(float(old["corridor_allowed_cells"] or 0)) == int(row["corridor_allowed_cells"] or 0),
            "path_hash_equal": raw_path_hash_equal,
            "path_geometry_motion_equal": geometry_equal,
            "path_hash_provenance_only_difference": bool(
                old_points and geometry_equal and not raw_path_hash_equal
            ),
            "collision_count_equal": int(float(old["collision_count"] or 0)) == int(row["collision_count"] or 0),
            "collision_positions_equal": _normal_json(old["collision_positions"]) == row["collision_positions"],
            "path_length_equal": _numeric(old["path_length_m"]) == _numeric(row["path_length_m"]),
            "minimum_clearance_equal": _numeric(old["minimum_clearance_m"]) == _numeric(row["minimum_clearance_m"]),
            "maximum_curvature_equal": _numeric(old["maximum_curvature"]) == _numeric(row["maximum_curvature"]),
            "old_path_hash": old["path_hash"], "new_path_hash": row["path_hash"],
            "old_source_commit": old_points[0].get("source_commit", "") if old_points else "",
            "new_source_commit": new_points[0].get("source_commit", "") if new_points else "",
            "old_failure_code": old["failure_code"], "new_failure_code": row["failure_code"],
        })
    _write_csv(output / "correctness_parity.csv", comparisons)
    fields = [
        "final_valid_equal", "failure_code_equal", "route_node_ids_equal",
        "route_edge_ids_equal", "start_attachment_candidates_equal",
        "goal_attachment_candidates_equal", "mask_hash_equal",
        "selected_start_attachment_equal", "selected_goal_attachment_equal",
        "selected_connection_edges_equal", "allowed_cells_equal",
        "path_hash_equal", "path_geometry_motion_equal", "collision_count_equal",
        "collision_positions_equal", "path_length_equal",
        "minimum_clearance_equal", "maximum_curvature_equal",
    ]
    summary = {
        f"{field}_count": sum(bool(row[field]) for row in comparisons)
        for field in fields
    }
    summary["comparison_count"] = len(comparisons)
    summary["nonempty_path_count"] = sum(
        bool(row.get("old_path_hash") or row.get("new_path_hash")) for row in comparisons
    )
    summary["nonempty_path_hash_equal_count"] = sum(
        bool(row["path_hash_equal"] and (row.get("old_path_hash") or row.get("new_path_hash")))
        for row in comparisons
    )
    summary["path_hash_provenance_only_difference_count"] = sum(
        bool(row["path_hash_provenance_only_difference"]) for row in comparisons
    )
    summary["all_valid_paths_safe"] = all(
        (not row.get("final_valid_success"))
        or (row.get("static_footprint_valid") and row.get("kinematic_valid")
            and float(row.get("reverse_distance_m") or 0.0) <= 1.0e-6
            and int(row.get("in_place_rotation_count") or 0) == 0)
        for row in rows if row.get("run_mode") == "measured"
    )
    (output / "correctness_parity.yaml").write_text(
        yaml.safe_dump(summary, sort_keys=False), encoding="utf-8",
    )
    report = ["# r1/r2 correctness parity", ""] + [
        f"- {field}: {summary[f'{field}_count']}/{summary['comparison_count']}"
        for field in fields
    ] + [
        f"- nonempty_path_hash_equal: {summary['nonempty_path_hash_equal_count']}/{summary['nonempty_path_count']}",
        "- raw path hash includes source_commit; path_geometry_motion_equal excludes only path_hash/source_commit provenance fields.",
        f"- path_hash_provenance_only_difference: {summary['path_hash_provenance_only_difference_count']}/{summary['nonempty_path_count']}",
        f"- all_valid_paths_safe: {summary['all_valid_paths_safe']}",
    ]
    (output / "correctness_parity.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    return summary


def _phase_summary(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    return [
        {
            "phase": field,
            "p50_ms": _percentile(measured, field, 50),
            "p95_ms": _percentile(measured, field, 95),
            "p99_ms": _percentile(measured, field, 99),
        }
        for field in PHASE_FIELDS
    ]


def _stage_results() -> List[Dict[str, Any]]:
    names = [
        ("r1_exact", "2d_v1_r2_stage_baseline_r1_exact_20260903"),
        ("stage0_timing", "2d_v1_r2_stage0_timing_20260903"),
        ("stage1_corridor_cache", "2d_v1_r2_stage1_corridor_cache_20260903"),
        ("stage2_canonical_validation", "2d_v1_r2_stage2_canonical_validation_20260903"),
        ("stage3_attachment", "2d_v1_r2_stage3_attachment_20260903"),
    ]
    results = []
    parent = ROOT / "experiments/layered_planner_benchmark"
    previous = None
    for stage, name in names:
        path = parent / name / "runs.csv"
        if not path.is_file():
            continue
        with path.open(newline="", encoding="utf-8") as stream:
            rows = [row for row in csv.DictReader(stream) if row.get("run_mode") == "measured"]
        p50 = _percentile(rows, "online_wall_ms", 50)
        results.append({
            "stage": stage, "experiment": str(path.parent), "measured_count": len(rows),
            "online_p50_ms": p50,
            "incremental_saving_ms": (previous - p50) if previous is not None and p50 is not None else None,
        })
        previous = p50
    return results


def _report(
    output: Path,
    rows: Sequence[Mapping[str, Any]],
    phase_summary: Sequence[Mapping[str, Any]],
    parity: Mapping[str, Any],
    pipeline: r2_pipeline.Layered2DV1R2Pipeline,
    source_hash: str,
) -> Dict[str, Any]:
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    success = [row for row in measured if row.get("final_valid_success")]
    valid_count = len(success)
    failures = dict(collections.Counter(
        str(row.get("failure_code")) for row in measured if row.get("failure_code")
    ))
    retry_count = sum(int(row.get("l3_retry_count") or 0) for row in measured)
    fallback_count = sum(bool(row.get("corridor_fallback_used")) for row in measured)
    parity_fields = (
        "final_valid_equal", "failure_code_equal", "route_node_ids_equal",
        "route_edge_ids_equal", "start_attachment_candidates_equal",
        "goal_attachment_candidates_equal", "selected_start_attachment_equal",
        "selected_goal_attachment_equal", "selected_connection_edges_equal",
        "mask_hash_equal", "allowed_cells_equal", "path_geometry_motion_equal",
        "collision_count_equal", "collision_positions_equal", "path_length_equal",
        "minimum_clearance_equal", "maximum_curvature_equal",
    )
    parity_count = int(parity.get("comparison_count") or 0)
    parity_passed = bool(
        parity_count == len(measured)
        and all(
            int(parity.get(f"{field}_count") or 0) == parity_count
            for field in parity_fields
        )
        and parity.get("all_valid_paths_safe") is True
    )
    summary = {
        "measured_count": len(measured), "final_valid_count": valid_count,
        "final_valid_rate": valid_count / max(1, len(measured)),
        "online_wall_p50_ms": _percentile(measured, "online_wall_ms", 50),
        "online_wall_p95_ms": _percentile(measured, "online_wall_ms", 95),
        "online_wall_p99_ms": _percentile(measured, "online_wall_ms", 99),
        "success_online_wall_p50_ms": _percentile(success, "online_wall_ms", 50),
        "success_online_wall_p95_ms": _percentile(success, "online_wall_ms", 95),
        "success_online_wall_p99_ms": _percentile(success, "online_wall_ms", 99),
        "failure_counts": failures, "retry_count": retry_count,
        "fallback_count": fallback_count, "parity_gate_passed": parity_passed,
        "correctness_gate_passed": bool(
            valid_count >= 90
            and failures == {
                "STATIC_FOOTPRINT_COLLISION": 5, "L1_NO_ROUTE": 5,
            }
            and retry_count == 0 and fallback_count == 0 and parity_passed
        ),
    }
    summary["performance_gate_passed"] = bool(
        summary["success_online_wall_p50_ms"] is not None
        and summary["success_online_wall_p50_ms"] <= 1000.0
        and summary["success_online_wall_p95_ms"] <= 1714.44739605 * 1.05
        and summary["success_online_wall_p99_ms"] <= 1946.17641856 * 1.05
    )
    fmt = lambda value: "not_available" if value is None else f"{float(value):.2f}"
    phase_lines = [
        f"| {row['phase']} | {fmt(row['p50_ms'])} | {fmt(row['p95_ms'])} | {fmt(row['p99_ms'])} |"
        for row in phase_summary
    ]
    report = [
        "# 2D-V1-r2 formal experiment", "",
        "独立静态 20-query 实验；使用冻结的 r1 numpy_zhang_suen 拓扑，未启用 refined topology。", "",
        f"- Final-valid: **{valid_count}/{len(measured)}**; failures=`{failures}`.",
        f"- Overall online P50/P95/P99: {fmt(summary['online_wall_p50_ms'])}/{fmt(summary['online_wall_p95_ms'])}/{fmt(summary['online_wall_p99_ms'])} ms.",
        f"- Success online P50/P95/P99: {fmt(summary['success_online_wall_p50_ms'])}/{fmt(summary['success_online_wall_p95_ms'])}/{fmt(summary['success_online_wall_p99_ms'])} ms.",
        f"- Correctness/performance gates: {summary['correctness_gate_passed']}/{summary['performance_gate_passed']}.",
        f"- Measured retries/fallbacks: {retry_count}/{fallback_count}; parity gate={parity_passed}.",
        f"- Corridor cache hits/misses: {pipeline.corridor_cache.hits}/{pipeline.corridor_cache.misses}; build={pipeline.corridor_cache.build_time_ms:.2f} ms; payload={pipeline.corridor_cache.memory_bytes} bytes.",
        f"- Endpoint cache hits/misses: {pipeline.endpoint_cache_hits}/{pipeline.endpoint_cache_misses}; payload={pipeline.endpoint_cache_memory_bytes} bytes; edge index={pipeline.edge_segment_index.memory_bytes} bytes/{pipeline.edge_segment_index.build_time_ms:.2f} ms.",
        f"- r1 parity route/mask/path-geometry/failure: {parity.get('route_edge_ids_equal_count', 0)}/{parity.get('mask_hash_equal_count', 0)}/{parity.get('path_geometry_motion_equal_count', 0)}/{parity.get('failure_code_equal_count', 0)} of {parity.get('comparison_count', 0)}.",
        f"- Raw non-empty path hash parity: {parity.get('nonempty_path_hash_equal_count', 0)}/{parity.get('nonempty_path_count', 0)}; all differences are source_commit-only provenance changes for {parity.get('path_hash_provenance_only_difference_count', 0)} paths.",
        "- SmacPlannerHybrid/DUBIN parameters, full base reset, full corridor publication, footprint and validation thresholds are unchanged.",
        f"- source_hash=`{source_hash}`.", "", "## Phase timing", "",
        "| Phase | P50 ms | P95 ms | P99 ms |", "|---|---:|---:|---:|",
        *phase_lines,
    ]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    return summary


def run_formal(
    output: Path = DEFAULT_OUTPUT,
    *,
    topology_cache_dir: Path = DEFAULT_CACHE_ROOT,
    warmups: int = WARMUPS,
    repetitions: int = REPETITIONS,
    ros_domain_id: int = ROS_DOMAIN_ID,
    query_ids: Optional[Sequence[str]] = None,
) -> Path:
    _refuse_nonempty(output)
    r2_config = _load_r2_config()
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir()
    queries, task_metadata = r1_benchmark._load_tasks()
    if query_ids:
        selected = set(query_ids)
        if not selected.issubset(set(TASK_IDS)):
            raise ValueError("query_ids must be a subset of A2B-01..A2B-20")
        queries = [query for query in queries if query.query_id in selected]
    ctx = r1_benchmark._context()
    artifact, topology_info, source_audit = _load_frozen_r1_topology(
        ctx, Path(topology_cache_dir),
    )
    graph_started_ns = time.monotonic_ns()
    graph = r1_pipeline.build_static_topology_view(artifact)
    topology_view_build_ms = (time.monotonic_ns() - graph_started_ns) / 1.0e6
    graph.metadata["topology_cache_key"] = topology_info["topology_cache_key"]
    source_files, source_hash = _source_manifest()
    source_snapshot = _snapshot_sources(output, source_files, source_hash)
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    spec = legacy.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"Smac Hybrid backend unavailable: {spec.reason}")
    session = legacy.SmacSession(
        ctx, output, map_yaml=MAP_YAML, log_tag=f"formal_2d_v1_r2_{MAP_ID}",
        local_mask_updates=True, optimization_profile=OPTIMIZATION_PROFILE,
        smac_parameter_profile=SMAC_PARAMETER_PROFILE,
        optimization_stage=OPTIMIZATION_STAGE,
    )
    session.start()
    adapter = v0.SmacHybridAdapter(
        session, spec, footprint=legacy.FOOTPRINT,
        source_commit=legacy._source_commit(), force_full_update=True,
    )
    pipeline = r2_pipeline.Layered2DV1R2Pipeline(
        graph, footprint=legacy.FOOTPRINT, l3_planner=adapter,
        corridor_padding_m=2.0, corridor_profile="padding",
        corridor_fallback_policy="bounded",
        validator=lambda _map, query, points: canonical.canonical_validate_path(
            ctx, query, points,
        ),
        base_map_hash=ctx.map_sha256,
        topology_cache_key=topology_info["topology_cache_key"],
        topology_source_hash=topology_info["topology_source_hash"],
        corridor_semantics=CORRIDOR_SEMANTICS,
    )
    snapshot = DynamicSnapshot.empty(
        snapshot_id="static_empty_v1_r2", timestamp=0.0,
        map_version=ctx.map_sha256, map_shape=artifact.free_mask.shape,
    )
    rows: List[Dict[str, Any]] = []
    calls: List[Dict[str, Any]] = []
    metrics: List[Dict[str, Any]] = []
    phases: List[Dict[str, Any]] = []
    try:
        for run_mode, count in (("warmup", warmups), ("measured", repetitions)):
            for repetition in range(1, int(count) + 1):
                for query in queries:
                    row, call, metric, phase = _run_one(
                        ctx, graph, topology_info, query, run_mode, repetition,
                        session, adapter, pipeline, output, snapshot,
                        legacy._source_commit() or "unknown",
                    )
                    row["source_hash"] = source_hash
                    call["source_hash"] = source_hash
                    metric["source_hash"] = source_hash
                    phase["source_hash"] = source_hash
                    rows.append(row); calls.append(call); metrics.append(metric); phases.append(phase)
    finally:
        session.close()

    source_files_after, source_hash_after = _source_manifest()
    if source_hash_after != source_hash or source_files_after != source_files:
        raise RuntimeError(
            "benchmark source files changed while the formal experiment was running; "
            "discard this output and rerun with a stable workspace"
        )

    session_info = {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "ros_domain_id": int(ros_domain_id),
        "session_start_count": session.session_start_count,
        "session_close_count": session.session_close_count,
        "session_restart_count": session.session_restart_count,
        "session_startup_time_ms": session.stack_startup_time_ms,
        "session_shutdown_time_ms": session.stack_shutdown_time_ms,
        "topology_load_time_ms": topology_info["topology_load_time_ms"],
        "topology_view_build_time_ms": topology_view_build_ms,
        "edge_segment_index_build_time_ms": pipeline.edge_segment_index.build_time_ms,
        "corridor_cache_cold_build_time_ms": pipeline.corridor_cache.build_time_ms,
    }
    _write_csv(output / "runs.csv", rows)
    _write_csv(output / "backend_call_log.csv", calls)
    _write_csv(output / "path_metrics.csv", metrics)
    _write_csv(output / "phase_timing.csv", phases)
    _write_csv(output / "session_timing.csv", [session_info])
    failures = collections.Counter(
        str(row.get("failure_code")) for row in rows
        if row.get("run_mode") == "measured" and row.get("failure_code")
    )
    _write_csv(output / "failure_summary.csv", [
        {"failure_code": code, "count": count}
        for code, count in sorted(failures.items())
    ])
    cache_diagnostics = [
        {"cache": "corridor", "hits": pipeline.corridor_cache.hits,
         "misses": pipeline.corridor_cache.misses,
         "hit_rate": pipeline.corridor_cache.hits / max(1, pipeline.corridor_cache.hits + pipeline.corridor_cache.misses),
         "cold_build_time_ms": pipeline.corridor_cache.build_time_ms,
         "lookup_time_ms": pipeline.corridor_cache.lookup_time_ms,
         "memory_bytes": pipeline.corridor_cache.memory_bytes},
        {"cache": "endpoint", "hits": pipeline.endpoint_cache_hits,
         "misses": pipeline.endpoint_cache_misses,
         "hit_rate": pipeline.endpoint_cache_hits / max(1, pipeline.endpoint_cache_hits + pipeline.endpoint_cache_misses),
         "cold_build_time_ms": pipeline.endpoint_cache_build_time_ms,
         "lookup_time_ms": pipeline.endpoint_cache_lookup_time_ms,
         "memory_bytes": pipeline.endpoint_cache_memory_bytes},
        {"cache": "edge_segment_index", "hits": "not_applicable", "misses": "not_applicable",
         "hit_rate": "not_applicable", "cold_build_time_ms": pipeline.edge_segment_index.build_time_ms,
         "lookup_time_ms": "included_in_attachment_miss", "memory_bytes": pipeline.edge_segment_index.memory_bytes},
    ]
    _write_csv(output / "cache_diagnostics.csv", cache_diagnostics)
    phase_summary = _phase_summary(rows)
    _write_csv(output / "phase_timing_summary.csv", phase_summary)
    stage_results = _stage_results()
    _write_csv(output / "optimization_stage_results.csv", stage_results)
    parity = _write_correctness_parity(output, rows) if len(queries) == 20 and repetitions == 5 else {}
    summary = _report(output, rows, phase_summary, parity, pipeline, source_hash)

    source_audit.update({
        "source_hash": source_hash,
        "r1_experiment": str(R1_EXPERIMENT),
        "r2_source_file_count": len(source_files),
        "r2_source_snapshot_file_count": source_snapshot["file_count"],
        "r2_source_snapshot_manifest": str(output / "source_snapshot_manifest.yaml"),
        "frozen_topology_selected_to_reproduce_r1": True,
    })
    (output / "source_audit.yaml").write_text(
        yaml.safe_dump(source_audit, sort_keys=False), encoding="utf-8",
    )
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({
        "experiment_id": output.name, "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "parent_architecture": PARENT_ARCHITECTURE,
        "protocol_version": PROTOCOL_VERSION, "source_hash": source_hash,
        "source_files": source_files, "map_id": MAP_ID,
        "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256,
        "query_sha256": {query.query_id: r1_benchmark._query_hash(query) for query in queries},
        "footprint_hash": v0._footprint_hash(legacy.FOOTPRINT),
        "frozen_topology_cache_key": topology_info["topology_cache_key"],
        "frozen_topology_source_hash": topology_info["topology_source_hash"],
        "source_snapshot_manifest": "source_snapshot_manifest.yaml",
    }, sort_keys=False), encoding="utf-8")
    protocol = {
        "experiment_id": output.name, "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "parent_architecture": PARENT_ARCHITECTURE,
        "protocol_version": PROTOCOL_VERSION, "map_id": MAP_ID,
        "query_set_id": QUERY_SET_ID, "query_ids": [query.query_id for query in queries],
        "warmups": warmups, "repetitions": repetitions,
        "resolution_m": 0.05, "dynamic_obstacles": False,
        "minimum_turning_radius_m": 0.40, "maximum_curvature_1pm": 2.50,
        "allow_reverse": False, "allow_in_place_rotation": False,
        "layers": {"L1": "2A-V0 original static skeleton topology + Graph D* Lite",
                   "L2": "disabled", "L3_prime": "full corridor Nav2 SmacPlannerHybrid DUBIN"},
        "topology_refinement_enabled": False,
        "corridor_profile": CORRIDOR_PROFILE,
        "corridor_semantics": CORRIDOR_SEMANTICS,
        "padding_schedule_m": list(PADDING_SCHEDULE),
        "fallback_policy_unchanged": True,
        "smac_parameter_profile": SMAC_PARAMETER_PROFILE,
        "smac_parameters_changed_from_r1": False,
        "query_reset_restore_base_map": True,
        "corridor_force_full_update": True,
        "costmap_reuse_noop": False,
        "phase_accounting": "online wall uses non-overlapping aggregate phases; nested diagnostics are not re-added",
        "offline_costs": ["topology load", "topology view build", "edge segment index build", "corridor cold cache build"],
        "task_metadata": task_metadata,
        "r2_config": str(R2_CONFIG),
        "r2_config_sha256": sha256_file(R2_CONFIG),
        "r2_config_contract": r2_config,
    }
    (output / "protocol.yaml").write_text(
        yaml.safe_dump(protocol, sort_keys=False), encoding="utf-8",
    )
    manifest = {
        "experiment_id": output.name, "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "parent_architecture": PARENT_ARCHITECTURE,
        "protocol_version": PROTOCOL_VERSION, "experiment_kind": EXPERIMENT_KIND,
        "map_id": MAP_ID, "query_set_id": QUERY_SET_ID,
        "query_ids": [query.query_id for query in queries],
        "warmup_count": warmups, "measured_repetitions": repetitions,
        "run_count": len(rows), "topology_representation": "2a_v0_static_skeleton_graph",
        "topology_refinement_enabled": False,
        "topology_cache_hit": True,
        "topology_cache_key": topology_info["topology_cache_key"],
        "topology_cache_bytes": topology_info["topology_cache_bytes"],
        "corridor_cache_hits": pipeline.corridor_cache.hits,
        "corridor_cache_misses": pipeline.corridor_cache.misses,
        "corridor_cache_memory_bytes": pipeline.corridor_cache.memory_bytes,
        "corridor_cache_cold_build_time_ms": pipeline.corridor_cache.build_time_ms,
        "endpoint_cache_hits": pipeline.endpoint_cache_hits,
        "endpoint_cache_misses": pipeline.endpoint_cache_misses,
        "endpoint_cache_memory_bytes": pipeline.endpoint_cache_memory_bytes,
        "edge_segment_index_memory_bytes": pipeline.edge_segment_index.memory_bytes,
        "edge_segment_index_build_time_ms": pipeline.edge_segment_index.build_time_ms,
        "session_start_count": session_info["session_start_count"],
        "session_close_count": session_info["session_close_count"],
        "session_restart_count": session_info["session_restart_count"],
        "l2_called": False, "l2_call_count": 0,
        "source_hash": source_hash, "correctness_parity": parity,
        **summary,
    }
    (output / "manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8",
    )
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run independent 2D-V1-r2 latency-optimized formal benchmark",
    )
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--topology-cache-dir", default=str(DEFAULT_CACHE_ROOT))
    parser.add_argument("--warmups", type=int, default=WARMUPS)
    parser.add_argument("--repetitions", type=int, default=REPETITIONS)
    parser.add_argument("--query-id", action="append", dest="query_ids")
    parser.add_argument("--ros-domain-id", type=int, default=ROS_DOMAIN_ID)
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        path = run_formal(
            Path(args.output_dir).resolve(),
            topology_cache_dir=Path(args.topology_cache_dir).resolve(),
            warmups=args.warmups, repetitions=args.repetitions,
            ros_domain_id=args.ros_domain_id, query_ids=args.query_ids,
        )
    except Exception as exc:
        print(f"2d_v1_r2_formal_benchmark: ERROR: {exc}")
        return 2
    print(f"2D-V1-r2 output: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
