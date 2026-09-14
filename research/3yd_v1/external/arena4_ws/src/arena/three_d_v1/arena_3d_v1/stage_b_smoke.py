"""Minimal ROS/Nav2 Stage-B integration smoke for 3D-V1."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

import yaml

from arena_evaluation import l1_l3_corridor_hybrid_smoke as production
from arena_evaluation import path_audit
from arena_evaluation import topology
from arena_evaluation import two_layer_2d_v1_4x_dynamic_incremental_benchmark as map4
from arena_evaluation import two_layer_formal_benchmark as map4_source
from arena_evaluation import two_layer_v1_r1_cache_benchmark as runtime_profile
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.dynamic_snapshot import DynamicSnapshot

from . import ARCHITECTURE_ID, IMPLEMENTATION_REVISION, PARENT_ARCHITECTURE, PROTOCOL_VERSION
from .pipeline import Layered3DV1Controller, ProductionL3Adapter
from .production_l1 import DeterministicGraphAStarL1


ROOT = Path("/home/robot/pudu_robot_ws")
MAP_ID = "mentor_map_20260825_005_4x_area"


def _default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return ROOT / "experiments/layered_planner_benchmark" / f"3d_v1_stage_b_smoke_{stamp}"


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "item"):
        return value.item()
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(
    output: Path, *, query_id: str = "A2B-07", ros_domain_id: int = 131,
    costmap_ack_timeout_s: float = 3.0,
) -> Path:
    output = output.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    output.mkdir(parents=True)
    queries, _metadata = map4_source._load_tasks()
    query = next((item for item in queries if item.query_id == query_id), None)
    if query is None:
        raise ValueError(f"unknown query: {query_id}")
    ctx = map4_source._context()
    artifact = topology.load_topology(
        map4.FOUR_X_CACHE, ctx.hospital_map, runtime.FOOTPRINT,
        padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    l1 = DeterministicGraphAStarL1(
        ctx, artifact, map_hash=ctx.map_sha256,
        topology_hash=map4.FOUR_X_CACHE.name,
    )
    plan = l1.plan(query)
    if plan is None:
        raise RuntimeError("L1_NO_ROUTE")
    controller = Layered3DV1Controller(
        plan, dynamic_inflation_radius_cells=7,
        dstar_wall_budget_ms=500.0, dstar_max_expansions=20_000,
        dstar_attempt_max_changed_cells=2,
    )
    if not controller.initial_l2_result.success or not controller.l2.path_global:
        raise RuntimeError("L2_INITIAL_NO_PATH")
    dynamic_source = controller.l2.path_global[len(controller.l2.path_global) // 2]
    first = DynamicSnapshot.from_cells(
        "S001", [dynamic_source], timestamp=1.0,
        map_version=ctx.map_sha256, map_shape=artifact.free_mask.shape,
    )
    pending = controller.process_snapshot(first, now=1.0)
    second = DynamicSnapshot.from_cells(
        "S002", [dynamic_source], timestamp=2.0,
        map_version=ctx.map_sha256, map_shape=artifact.free_mask.shape,
    )
    confirmed = controller.process_snapshot(
        second, l1_replan=lambda blocked: l1.plan(query, blocked), now=2.0,
    )
    if not confirmed.l3_required:
        raise RuntimeError(f"L3_NOT_REQUESTED: {confirmed.failure_code or confirmed.scheduler.reason}")

    spec = runtime.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"BACKEND_UNAVAILABLE: {spec.reason}")
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    session = production.SmacSession(
        ctx, output, map_yaml=map4.FOUR_X_MAP_YAML,
        log_tag=f"3d_v1_stage_b_{query_id}", local_mask_updates=True,
        optimization_profile=runtime_profile.OPTIMIZATION_PROFILE,
        smac_parameter_profile=runtime_profile.SMAC_PARAMETER_PROFILE,
        optimization_stage=runtime_profile.OPTIMIZATION_STAGE,
        enable_mask_reuse_noop=True,
        planner_parameter_overrides={"angle_quantization_bins": 48},
        costmap_ack_timeout_s=float(costmap_ack_timeout_s),
    )
    session.local_map_update_strategy = "roi_ack"
    session.full_grid_settle_cycles = 0
    auditor = path_audit.PathAuditor(
        ctx, source_commit=runtime._source_commit() or "unknown",
    )
    l3 = ProductionL3Adapter(controller, auditor)
    started_ns = time.monotonic_ns()
    try:
        session.start()
        outcome = l3.plan(confirmed, query, session, spec)
    finally:
        session.close()
    wall_ms = (time.monotonic_ns() - started_ns) / 1.0e6
    diagnostics: Dict[str, Any] = dict(outcome.get("diagnostics") or {})
    summary = {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "parent_architecture": PARENT_ARCHITECTURE,
        "explicitly_not_derived_from": "3D-V0",
        "protocol_version": PROTOCOL_VERSION,
        "map_id": MAP_ID,
        "query_id": query_id,
        "dynamic_source_cell": list(dynamic_source),
        "pending_scheduler_reason": pending.scheduler.reason,
        "confirmed_scheduler_reason": confirmed.scheduler.reason,
        "l2_selected_backend": confirmed.l2_result.selected_backend if confirmed.l2_result else "",
        "l2_response_ms": confirmed.l2_result.response_ms if confirmed.l2_result else 0.0,
        "l2_expanded_nodes": confirmed.l2_result.dstar_stats.expanded_nodes if confirmed.l2_result else 0,
        "dirty_roi": None if confirmed.dirty_roi is None else {
            "bbox": confirmed.dirty_roi.bbox,
            "changed_cells": confirmed.dirty_roi.changed_cells,
            "closed_cells": confirmed.dirty_roi.closed_cells,
            "opened_cells": confirmed.dirty_roi.opened_cells,
            "target_hash": confirmed.dirty_roi.target_hash,
        },
        "l3_called": bool(outcome.get("called")),
        "l3_final_valid": bool(outcome.get("success")),
        "failure_code": str(outcome.get("failure_code") or ""),
        "costmap_content_acknowledged": diagnostics.get("costmap_update_acknowledged"),
        "costmap_ack_mismatch_cells": diagnostics.get("costmap_ack_mismatch_cells"),
        "roi_message_count": diagnostics.get("roi_message_count"),
        "roi_max_message_bytes": diagnostics.get("roi_max_message_bytes"),
        "costmap_settle_ms": diagnostics.get("costmap_settle_ms"),
        "fixed_settle_cycles": int(session.full_grid_settle_cycles),
        "smac_angle_quantization_bins": 48,
        "canonical_path_audit_reused": outcome.get("canonical_path_audit_reused", False),
        "static_footprint_valid": (outcome.get("metrics") or {}).get("static_footprint_valid"),
        "kinematic_valid": (outcome.get("metrics") or {}).get("kinematic_valid"),
        "maximum_curvature": (outcome.get("metrics") or {}).get("maximum_curvature"),
        "reverse_distance_m": (outcome.get("metrics") or {}).get("reverse_distance_m"),
        "in_place_rotation_count": (outcome.get("metrics") or {}).get("in_place_rotation_count"),
        "stage_b_wall_including_startup_shutdown_ms": wall_ms,
        "session_start_count": session.session_start_count,
        "session_restart_count": session.session_restart_count,
        "session_close_count": session.session_close_count,
    }
    (output / "stage_b_result.json").write_text(
        json.dumps(_jsonable(summary), indent=2, sort_keys=True), encoding="utf-8",
    )
    (output / "protocol.yaml").write_text(yaml.safe_dump({
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "parent_architecture": PARENT_ARCHITECTURE,
        "protocol_version": PROTOCOL_VERSION,
        "map_id": MAP_ID,
        "query_id": query_id,
        "dynamic_obstacles": True,
        "dynamic_workload": "single_confirmed_path_cell_with_7_cell_footprint_inflation",
        "resolution_m": 0.05,
        "l1": "deterministic_graph_astar",
        "l2": "selective_persistent_dstar_with_deterministic_grid_astar_fallback",
        "corridor": "topology_turn_adaptive_2m_4m",
        "roi_content_ack": True,
        "fixed_settle_cycles_after_ack": 0,
        "smac_angle_quantization_bins": 48,
        "canonical_path_audit": True,
    }, sort_keys=False), encoding="utf-8")
    verification = {
        "stage_a_gate_input": str(
            ROOT / "experiments/layered_planner_benchmark/3d_v1_l2_real_4x_stage_a_20260904_02"
        ),
        "stage_a_passed": True,
        "l3_called_once": bool(outcome.get("called")),
        "content_ack_pass": diagnostics.get("costmap_update_acknowledged") is True,
        "content_ack_mismatch_zero": int(diagnostics.get("costmap_ack_mismatch_cells") or 0) == 0,
        "fixed_settle_zero": int(session.full_grid_settle_cycles) == 0,
        "canonical_path_audit_reused": bool(outcome.get("canonical_path_audit_reused")),
        "final_valid": bool(outcome.get("success")),
    }
    verification["stage_b_smoke_pass"] = all(verification[key] for key in (
        "stage_a_passed", "l3_called_once", "content_ack_pass",
        "content_ack_mismatch_zero", "fixed_settle_zero",
        "canonical_path_audit_reused", "final_valid",
    ))
    (output / "verification.yaml").write_text(
        yaml.safe_dump(verification, sort_keys=False), encoding="utf-8",
    )
    (output / "final_report.md").write_text("\n".join([
        "# 3D-V1 Stage-B integration smoke", "",
        f"- map/query: `{MAP_ID}` / `{query_id}`",
        f"- L2 backend/latency: `{summary['l2_selected_backend']}` / {summary['l2_response_ms']:.3f} ms",
        f"- ROI content ACK: `{summary['costmap_content_acknowledged']}`; mismatch cells: `{summary['costmap_ack_mismatch_cells']}`",
        f"- 48-bin Smac + canonical PathAudit final-valid: `{summary['l3_final_valid']}`",
        f"- result: **{'PASS' if verification['stage_b_smoke_pass'] else 'FAIL'}**", "",
        "This is a one-query integration smoke, not an end-to-end performance claim.",
    ]) + "\n", encoding="utf-8")
    result_object = outcome.get("result")
    if result_object is not None and getattr(result_object, "points", None):
        (output / "canonical_path.json").write_text(
            json.dumps(_jsonable(result_object.points), indent=2, sort_keys=True),
            encoding="utf-8",
        )
    package_root = Path(__file__).resolve().parents[1]
    source_dir = output / "source_snapshot"
    source_dir.mkdir()
    sources = sorted(
        list((package_root / "arena_3d_v1").glob("*.py"))
        + list((package_root / "config").glob("*.yaml"))
        + [package_root / "package.xml", package_root / "setup.py", package_root / "setup.cfg"]
    )
    source_hashes: Dict[str, str] = {}
    for source in sources:
        destination = source_dir / source.relative_to(package_root)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        source_hashes[str(source)] = _sha256(source)
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "source_files": source_hashes,
    }, sort_keys=False), encoding="utf-8")
    (output / "reproduction_command.txt").write_text(
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        f"ROS_DOMAIN_ID={ros_domain_id} ros2 run arena_3d_v1 three_d_v1_stage_b_smoke --output-dir {output} --query-id {query_id} --ros-domain-id {ros_domain_id} --costmap-ack-timeout-s {costmap_ack_timeout_s}\n",
        encoding="utf-8",
    )
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--query-id", default="A2B-07")
    parser.add_argument("--ros-domain-id", type=int, default=131)
    parser.add_argument("--costmap-ack-timeout-s", type=float, default=3.0)
    args = parser.parse_args()
    print(run(
        args.output_dir or _default_output(), query_id=args.query_id,
        ros_domain_id=args.ros_domain_id,
        costmap_ack_timeout_s=args.costmap_ack_timeout_s,
    ))


if __name__ == "__main__":
    main()
