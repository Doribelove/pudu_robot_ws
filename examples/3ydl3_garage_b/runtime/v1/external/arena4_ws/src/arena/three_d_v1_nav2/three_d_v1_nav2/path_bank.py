"""Generate a SHA-bound bank of canonical L3 paths through frozen 3D-V1-r1."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import resource
import shutil
import subprocess
import time
import zipfile
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping

import numpy as np
import yaml

from arena_3d_v1.production_l1 import DeterministicGraphAStarL1
from arena_3d_v1.r1_pipeline import Layered3DV1R1Controller
from arena_evaluation import l1_l3_corridor_hybrid_smoke as production
from arena_evaluation import path_audit
from arena_evaluation import topology
from arena_evaluation import two_layer_v1_r1_cache_benchmark as runtime_profile
from arena_evaluation import two_layer_v2_semantic_benchmark as semantic_runtime
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.pdmap_semantic_converter import convert_pdmap
from arena_evaluation.semantic_query_defaults import load_query_set

from . import ARCHITECTURE_ID, TASK_ID
from .contracts import (
    EXPECTED_MAP_SHA256, EXPECTED_QUERY_IDS, PDMAP, QUERY_SET, sha256_file,
    verify_frozen_sources, verify_static_inputs,
)


AUTHORITATIVE_TOPOLOGY = Path(
    "/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/"
    "query_selection_r2_selected8_validation_v2/topology_cache"
)
INDEX_FIELDS = (
    "query_id", "start_x", "start_y", "start_yaw", "goal_x", "goal_y", "goal_yaw",
    "path_file", "path_sha256", "canonical_path_hash", "route_edge_ids", "l2_binding_hash",
    "l2_backend", "final_audit_passed", "l1_algorithm", "l3_algorithm", "angle_bins",
    "motion_model", "reverse_allowed", "rotate_in_place_allowed", "min_turning_radius_m",
    "max_curvature_1pm", "roi_ack_mismatches",
)


def _jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "item"):
        return value.item()
    return value


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]], fields=None) -> None:
    materialized = list(rows)
    names = list(fields or [])
    for row in materialized:
        for key in row:
            if key not in names:
                names.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=names or ["empty"])
        writer.writeheader()
        writer.writerows(materialized)


def _extract_map(pdmap: Path, destination: Path) -> tuple[Path, Path]:
    destination.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(pdmap) as archive:
        names = set(archive.namelist())
        for member in ("optemap.pgm", "optemap.yaml"):
            if member not in names:
                raise RuntimeError(f"pdmap is missing {member}")
            target = destination / member
            if target.exists():
                raise FileExistsError(f"refusing to overwrite derived map: {target}")
            target.write_bytes(archive.read(member))
    return destination / "optemap.pgm", destination / "optemap.yaml"


def _copy_topology(destination: Path) -> None:
    if destination.exists() and any(destination.iterdir()):
        raise FileExistsError(f"refusing to overwrite topology destination: {destination}")
    destination.mkdir(parents=True, exist_ok=True)
    for name in ("topology_arrays.npz", "topology_graph.json", "topology_metadata.yaml"):
        source = AUTHORITATIVE_TOPOLOGY / name
        if not source.is_file():
            raise FileNotFoundError(source)
        shutil.copy2(source, destination / name)


def _git_head() -> str:
    completed = subprocess.run(
        ["git", "-C", "/home/robot/pudu_robot_ws", "rev-parse", "HEAD"],
        check=True, text=True, stdout=subprocess.PIPE,
    )
    return completed.stdout.strip()


def run(output: Path, *, ros_domain_id: int = 218, query_id: str = "") -> Path:
    output = output.resolve()
    path_bank_dir = output / "path_bank"
    derived = output / "derived_map"
    for directory in (path_bank_dir, derived):
        if directory.exists() and any(directory.iterdir()):
            raise FileExistsError(f"refusing to overwrite non-empty directory: {directory}")
        directory.mkdir(parents=True, exist_ok=True)

    frozen_source_hashes = verify_frozen_sources()
    extracted = derived / "extracted"
    map_pgm, map_yaml = _extract_map(PDMAP, extracted)
    semantic_map, _ = convert_pdmap(
        pdmap=PDMAP, output_dir=derived / "semantic_conversion", overwrite=False,
    )
    inputs = verify_static_inputs(
        map_pgm=map_pgm, map_yaml=map_yaml,
        semantic_map_hash=semantic_map.semantic_map_hash,
    )
    topology_dir = derived / "topology_cache"
    _copy_topology(topology_dir)

    queries, _intents, query_metadata = load_query_set(
        QUERY_SET, actual_map_hash=EXPECTED_MAP_SHA256,
        actual_semantic_map_hash=semantic_map.semantic_map_hash,
        require_default_contract=True,
    )
    if query_id:
        queries = [item for item in queries if item.query_id == query_id]
        if len(queries) != 1:
            raise ValueError(f"unknown frozen query ID: {query_id}")
    elif tuple(item.query_id for item in queries) != EXPECTED_QUERY_IDS:
        raise RuntimeError("frozen eight-query order drift")

    ctx = semantic_runtime._context(map_yaml)
    artifact = topology.load_topology(
        topology_dir, ctx.hospital_map, runtime.FOOTPRINT,
        padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    l1 = DeterministicGraphAStarL1(
        ctx, artifact, map_hash=ctx.map_sha256,
        topology_hash=sha256_file(topology_dir / "topology_graph.json"),
    )
    spec = runtime.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"Smac Hybrid backend unavailable: {spec.reason}")

    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    runtime_output = path_bank_dir / "smac_runtime"
    runtime_output.mkdir()
    session = production.SmacSession(
        ctx, runtime_output, map_yaml=map_yaml, log_tag="nav2_3d_v1_r1_seq8_path_bank",
        local_mask_updates=True,
        optimization_profile=runtime_profile.OPTIMIZATION_PROFILE,
        smac_parameter_profile=runtime_profile.SMAC_PARAMETER_PROFILE,
        optimization_stage=runtime_profile.OPTIMIZATION_STAGE,
        enable_mask_reuse_noop=True,
        planner_parameter_overrides={"angle_quantization_bins": 48},
        costmap_ack_timeout_s=3.0,
    )
    session.local_map_update_strategy = "roi_ack"
    session.full_grid_settle_cycles = 0
    auditor = path_audit.PathAuditor(ctx, source_commit=_git_head())
    l2_cache = path_bank_dir / "l2_state_cache"
    path_dir = path_bank_dir / "paths"
    trace_dir = path_bank_dir / "traces"
    path_dir.mkdir()
    trace_dir.mkdir()
    index_rows: list[Dict[str, Any]] = []
    trace_rows: list[Dict[str, Any]] = []
    audit_rows: list[Dict[str, Any]] = []
    failures: list[Dict[str, Any]] = []
    session_started = False
    try:
        session.start()
        session_started = True
        for sequence, query in enumerate(queries, start=1):
            total_started = time.monotonic_ns()
            try:
                reset_info = session.reset_query_state(query.query_id, restore_base_map=False)
                l1_started = time.monotonic_ns()
                plan = l1.plan(query)
                l1_ms = (time.monotonic_ns() - l1_started) / 1.0e6
                if plan is None:
                    raise RuntimeError("L1_NO_ROUTE")
                controller = Layered3DV1R1Controller(
                    plan, cache_root=l2_cache, max_active_states=1,
                    dynamic_inflation_radius_cells=7, dstar_wall_budget_ms=500.0,
                    dstar_max_expansions=20_000, dstar_attempt_max_changed_cells=2,
                    verify_l2_oracle=True,
                )
                l2 = controller.initial_l2_result
                if not l2.success or not controller.l2.path_global:
                    raise RuntimeError(l2.failure_code or "L2_INITIAL_NO_PATH")
                if l2.partial_dstar_result_returned:
                    raise RuntimeError("PARTIAL_DSTAR_FORBIDDEN")
                target_mask = controller._target_mask()
                l3_started = time.monotonic_ns()
                result = session.plan(
                    query, spec, source="3d_v1_r1_l3_smac_hybrid",
                    allowed_mask=target_mask, window_start_index=0, window_end_index=-1,
                    window_path_length_m=0.0, skip_path_mask_validation=True,
                )
                l3_wall_ms = (time.monotonic_ns() - l3_started) / 1.0e6
                diagnostics = dict(result.diagnostics or {})
                if diagnostics.get("costmap_update_acknowledged") is not True:
                    raise RuntimeError("COSTMAP_CONTENT_ACK_FAILED")
                if int(diagnostics.get("costmap_ack_mismatch_cells") or 0) != 0:
                    raise RuntimeError("COSTMAP_CONTENT_ACK_MISMATCH")
                if not result.planner_success or not result.points:
                    raise RuntimeError(result.failure_code or "L3_NO_PATH")
                audit = auditor.audit(query, result.points, target_mask)
                result.path_audit = audit
                if not audit.final_valid_success:
                    raise RuntimeError(str(audit.metrics.get("failure_code") or "FINAL_PATH_AUDIT_FAILED"))
                if float(audit.metrics.get("reverse_distance_m") or 0.0) != 0.0:
                    raise RuntimeError("L3_REVERSE_PATH")
                if int(audit.metrics.get("in_place_rotation_count") or 0) != 0:
                    raise RuntimeError("L3_ROTATE_IN_PLACE_PATH")
                maximum_curvature = float(audit.metrics.get("maximum_curvature") or 0.0)
                if maximum_curvature > 2.50 + 1.0e-3:
                    raise RuntimeError("L3_CURVATURE_EXCEEDED")

                path_file = path_dir / f"{sequence:02d}_{query.query_id}.csv"
                _write_csv(path_file, ({
                    "x": format(float(point["x"]), ".17g"),
                    "y": format(float(point["y"]), ".17g"),
                    "yaw": format(float(point["yaw"]), ".17g"),
                } for point in result.points), fields=("x", "y", "yaw"))
                trace_payload = {
                    "architecture_id": ARCHITECTURE_ID,
                    "task_id": TASK_ID,
                    "query_id": query.query_id,
                    "sequence": sequence,
                    "runtime_contract": dict(controller.runtime_contract),
                    "l1": {**dict(plan.diagnostics), "route_edge_ids": list(plan.route_edge_ids)},
                    "l2": {
                        "selected_backend": l2.selected_backend,
                        "success": l2.success,
                        "partial_dstar_result_returned": l2.partial_dstar_result_returned,
                        "response_ms": l2.response_ms,
                        "binding_hash": controller.l2.binding_hash,
                        "diagnostics": l2.diagnostics,
                    },
                    "l3": {"algorithm": "smac_hybrid_astar", "diagnostics": diagnostics},
                    "canonical_path_audit_reused": True,
                    "audit": {"metrics": audit.metrics, "diagnostics": audit.diagnostics()},
                    "reset": reset_info,
                }
                trace_file = trace_dir / f"{sequence:02d}_{query.query_id}.json"
                trace_file.write_text(
                    json.dumps(_jsonable(trace_payload), indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
                index_rows.append({
                    "query_id": query.query_id,
                    "start_x": format(float(query.start[0]), ".17g"),
                    "start_y": format(float(query.start[1]), ".17g"),
                    "start_yaw": format(float(query.start[2]), ".17g"),
                    "goal_x": format(float(query.goal[0]), ".17g"),
                    "goal_y": format(float(query.goal[1]), ".17g"),
                    "goal_yaw": format(float(query.goal[2]), ".17g"),
                    "path_file": str(path_file.relative_to(path_bank_dir)),
                    "path_sha256": sha256_file(path_file),
                    "canonical_path_hash": audit.path_hash,
                    "route_edge_ids": ";".join(plan.route_edge_ids),
                    "l2_binding_hash": controller.l2.binding_hash,
                    "l2_backend": l2.selected_backend,
                    "final_audit_passed": "true", "l1_algorithm": "deterministic_graph_astar",
                    "l3_algorithm": "smac_hybrid_astar", "angle_bins": "48",
                    "motion_model": "DUBIN", "reverse_allowed": "false",
                    "rotate_in_place_allowed": "false", "min_turning_radius_m": "0.40",
                    "max_curvature_1pm": "2.50", "roi_ack_mismatches": "0",
                })
                trace_rows.append({
                    "sequence": sequence, "query_id": query.query_id,
                    "l1_algorithm": "deterministic_graph_astar", "l1_ms": l1_ms,
                    "route_edge_ids": ";".join(plan.route_edge_ids),
                    "corridor_cells": int(np.count_nonzero(plan.corridor_mask)),
                    "l2_backend": l2.selected_backend, "l2_ms": l2.response_ms,
                    "l2_binding_hash": controller.l2.binding_hash,
                    "l2_partial_result": False, "l3_algorithm": "smac_hybrid_astar",
                    "l3_wall_ms": l3_wall_ms,
                    "l3_planning_ms": diagnostics.get("planning_time_ms"),
                    "angle_bins": 48, "motion_model": "DUBIN",
                    "roi_message_count": diagnostics.get("roi_message_count"),
                    "roi_max_message_bytes": diagnostics.get("roi_max_message_bytes"),
                    "roi_acknowledged": diagnostics.get("costmap_update_acknowledged"),
                    "roi_ack_mismatch_cells": diagnostics.get("costmap_ack_mismatch_cells"),
                    "fixed_settle_cycles": 0,
                    "canonical_path_hash": audit.path_hash,
                    "canonical_path_audit_reused": True,
                    "total_ms": (time.monotonic_ns() - total_started) / 1.0e6,
                })
                audit_rows.append({
                    "sequence": sequence, "query_id": query.query_id,
                    "path_sha256": sha256_file(path_file), "canonical_path_hash": audit.path_hash,
                    "final_valid_success": audit.final_valid_success,
                    "static_footprint_valid": audit.metrics.get("static_footprint_valid"),
                    "kinematic_valid": audit.metrics.get("kinematic_valid"),
                    "within_corridor": audit.within_mask,
                    "path_length_m": audit.metrics.get("path_length_m"),
                    "minimum_clearance_m": audit.metrics.get("minimum_clearance_m"),
                    "maximum_curvature_1pm": audit.metrics.get("maximum_curvature"),
                    "reverse_distance_m": audit.metrics.get("reverse_distance_m"),
                    "in_place_rotation_count": audit.metrics.get("in_place_rotation_count"),
                    "audit_ms": audit.timings.get("canonical_path_audit_ms"),
                })
                controller.lifecycle.clear()
            except Exception as error:
                failures.append({
                    "stage": "PATH_BANK", "sequence": sequence,
                    "query_id": query.query_id, "failure_code": type(error).__name__,
                    "failure_detail": str(error), "wall_time_ns": time.time_ns(),
                })
                raise
    finally:
        if session_started:
            session.close()
        _write_csv(path_bank_dir / "global_layer_trace.csv", trace_rows)
        _write_csv(path_bank_dir / "path_audit.csv", audit_rows)
        _write_csv(path_bank_dir / "failures.csv", failures)

    index_path = path_bank_dir / "index.csv"
    _write_csv(index_path, index_rows, INDEX_FIELDS)
    manifest = {
        "task_id": TASK_ID, "architecture_id": ARCHITECTURE_ID,
        "complete": len(index_rows) == len(queries),
        "query_set": query_metadata, "inputs": inputs,
        "frozen_source_hashes": frozen_source_hashes,
        "topology_files": {
            name: sha256_file(topology_dir / name)
            for name in ("topology_arrays.npz", "topology_graph.json", "topology_metadata.yaml")
        },
        "index_sha256": sha256_file(index_path),
        "path_count": len(index_rows), "ros_domain_id": ros_domain_id,
        "smac_session_start_count": session.session_start_count,
        "smac_session_restart_count": session.session_restart_count,
        "smac_session_close_count": session.session_close_count,
        "max_rss_bytes": int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024,
    }
    (path_bank_dir / "manifest.yaml").write_text(
        yaml.safe_dump(_jsonable(manifest), sort_keys=False), encoding="utf-8",
    )
    return index_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--ros-domain-id", type=int, default=218)
    parser.add_argument("--query-id", default="")
    args = parser.parse_args()
    print(run(args.output_dir, ros_domain_id=args.ros_domain_id, query_id=args.query_id))
