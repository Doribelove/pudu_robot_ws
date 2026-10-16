"""Formal dynamic-obstacle evidence for the frozen 3D-V1-r1 integration."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import subprocess
import time
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Sequence

import numpy as np

from arena_3d_v1.pipeline import ProductionL3Adapter
from arena_3d_v1.production_l1 import DeterministicGraphAStarL1
from arena_3d_v1.r1_pipeline import Layered3DV1R1Controller
from arena_3d_v1.r1_stage_a import _barrier_sources
from arena_evaluation import l1_l3_corridor_hybrid_smoke as production
from arena_evaluation import path_audit, topology
from arena_evaluation import two_layer_v1_r1_cache_benchmark as runtime_profile
from arena_evaluation import two_layer_v2_semantic_benchmark as semantic_runtime
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.dynamic_snapshot import DynamicSnapshot
from arena_evaluation.semantic_query_defaults import load_query_set

from . import ARCHITECTURE_ID, TASK_ID
from .contracts import (
    EXPECTED_MAP_SHA256,
    EXPECTED_SEMANTIC_MAP_HASH,
    QUERY_SET,
    sha256_file,
    verify_frozen_sources,
)


QUERY_ID = "cmp2-04-multi-junction"


def _jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_jsonable(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if hasattr(value, "item"):
        return value.item()
    return value


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    values = list(rows)
    fields: list[str] = []
    for row in values:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or ["status"])
        writer.writeheader()
        for row in values:
            writer.writerow({
                key: json.dumps(_jsonable(value), sort_keys=True)
                if isinstance(value, (dict, list, tuple, set)) else value
                for key, value in row.items()
            })


def _stable_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(
        _jsonable(value), sort_keys=True, separators=(",", ":"),
    ).encode("utf-8")).hexdigest()


def _step_trace(step: Any) -> Dict[str, Any]:
    """Serialize planner evidence without embedding multi-million-cell masks."""
    update = step.snapshot_update
    l2 = step.l2_result
    path = [] if l2 is None or l2.path is None else list(l2.path)
    return {
        "snapshot_id": update.snapshot_id,
        "snapshot_hash": update.snapshot_hash,
        "snapshot_accepted": update.accepted,
        "snapshot_rejection_reason": update.rejection_reason,
        "newly_blocked_source_count": len(update.newly_blocked_sources),
        "newly_freed_source_count": len(update.newly_freed_sources),
        "blocked_cell_count": len(update.blocked_cells),
        "blocked_cells_hash": _stable_hash(update.blocked_cells),
        "effective_changed_cell_count": len(update.effective_changed_cells),
        "scheduler_invoke_l2": step.scheduler.invoke_l2,
        "scheduler_reason": step.scheduler.reason,
        "scheduler_path_intersection_count": len(step.scheduler.path_intersections),
        "l1_graph_astar_called": step.l1_graph_astar_called,
        "l1_reroute_succeeded": step.l1_reroute_succeeded,
        "l3_required": step.l3_required,
        "failure_code": step.failure_code,
        "route_signature": step.route_signature,
        "dirty_roi": step.dirty_roi,
        "l2": None if l2 is None else {
            "success": l2.success,
            "failure_code": l2.failure_code,
            "selected_backend": l2.selected_backend,
            "response_ms": l2.response_ms,
            "partial_dstar_result_returned": l2.partial_dstar_result_returned,
            "path_cell_count": len(path),
            "path_hash": _stable_hash(path),
            "diagnostics": l2.diagnostics,
        },
        "diagnostics": step.diagnostics,
    }


def _snapshot(
    name: str,
    index: int,
    cells: Sequence[tuple[int, int]],
    *,
    map_hash: str,
    shape: Sequence[int],
) -> DynamicSnapshot:
    return DynamicSnapshot.from_cells(
        f"{name}-{index}", cells, timestamp=float(index),
        map_version=map_hash, map_shape=(int(shape[0]), int(shape[1])),
    )


def _git_head() -> str:
    return subprocess.run(
        ["git", "-C", "/home/robot/pudu_robot_ws", "rev-parse", "HEAD"],
        check=True, text=True, stdout=subprocess.PIPE,
    ).stdout.strip()


def _new_controller(plan: Any, cache_root: Path) -> Layered3DV1R1Controller:
    controller = Layered3DV1R1Controller(
        plan, cache_root=cache_root, max_active_states=1,
        dynamic_inflation_radius_cells=7, confidence_threshold=0.60,
        dstar_wall_budget_ms=500.0, dstar_max_expansions=20_000,
        dstar_attempt_max_changed_cells=2, verify_l2_oracle=True,
    )
    if not controller.initial_l2_result.success or not controller.l2.path_global:
        raise RuntimeError("L2_INITIAL_NO_PATH")
    if controller.initial_l2_result.partial_dstar_result_returned:
        raise RuntimeError("PARTIAL_DSTAR_FORBIDDEN")
    return controller


def _confirmed_step(
    controller: Layered3DV1R1Controller,
    name: str,
    cells: Sequence[tuple[int, int]],
    *,
    map_hash: str,
    shape: Sequence[int],
    l1_replan=None,
) -> tuple[Any, Any]:
    pending = controller.process_snapshot(
        _snapshot(name, 1, cells, map_hash=map_hash, shape=shape),
        l1_replan=l1_replan, now=1.0,
    )
    confirmed = controller.process_snapshot(
        _snapshot(name, 2, cells, map_hash=map_hash, shape=shape),
        l1_replan=l1_replan, now=2.0,
    )
    if pending.scheduler.invoke_l2:
        raise RuntimeError(f"{name}: first observation bypassed confirmation")
    if not confirmed.scheduler.invoke_l2:
        raise RuntimeError(f"{name}: confirmed obstacle did not invoke L2")
    return pending, confirmed


def _run_l3(
    *,
    name: str,
    controller: Layered3DV1R1Controller,
    step: Any,
    query: Any,
    session: Any,
    spec: Any,
    auditor: Any,
) -> Dict[str, Any]:
    started_ns = time.monotonic_ns()
    result = ProductionL3Adapter(controller, auditor).plan(step, query, session, spec)
    result["wall_ms"] = (time.monotonic_ns() - started_ns) / 1.0e6
    diagnostics = dict(result.get("diagnostics") or {})
    metrics = dict(result.get("metrics") or {})
    if result.get("called") is not True or result.get("success") is not True:
        raise RuntimeError(f"{name}: {result.get('failure_code') or 'L3_FAILED'}")
    if diagnostics.get("costmap_update_acknowledged") is not True:
        raise RuntimeError(f"{name}: COSTMAP_CONTENT_ACK_FAILED")
    if int(diagnostics.get("costmap_ack_mismatch_cells") or 0) != 0:
        raise RuntimeError(f"{name}: COSTMAP_CONTENT_ACK_MISMATCH")
    if not metrics.get("final_valid_success"):
        raise RuntimeError(f"{name}: CANONICAL_PATH_AUDIT_FAILED")
    if float(metrics.get("reverse_distance_m") or 0.0) != 0.0:
        raise RuntimeError(f"{name}: REVERSE_PATH")
    if int(metrics.get("in_place_rotation_count") or 0) != 0:
        raise RuntimeError(f"{name}: ROTATE_IN_PLACE_PATH")
    if float(metrics.get("maximum_curvature") or 0.0) > 2.501:
        raise RuntimeError(f"{name}: CURVATURE_EXCEEDED")
    return result


def _write_l3_path(output: Path, name: str, result: Mapping[str, Any]) -> Path:
    path_dir = output / "paths"
    path_dir.mkdir(exist_ok=True)
    destination = path_dir / f"{name}.csv"
    points = list(getattr(result["result"], "points", ()) or ())
    _write_csv(destination, ({
        "index": index,
        "x": format(float(point["x"]), ".17g"),
        "y": format(float(point["y"]), ".17g"),
        "yaw": format(float(point["yaw"]), ".17g"),
    } for index, point in enumerate(points)))
    return destination


def _wide_barrier(ctx: Any) -> list[tuple[int, int]]:
    # Perpendicular to the middle of topology edge 202. At 0.40 m spacing,
    # the frozen 0.35 m inflation discs overlap and close the 2 m corridor,
    # while the parallel edge 203 remains outside the obstacle support.
    result: list[tuple[int, int]] = []
    x = 5.8
    while x <= 10.2 + 1.0e-9:
        cell = ctx.hospital_map.world_to_cell(x, 54.116896)
        if cell is None:
            raise RuntimeError("wide barrier left the map")
        result.append((int(cell[0]), int(cell[1])))
        x += 0.4
    return result


def run(output: Path, *, run_root: Path, ros_domain_id: int = 218) -> Path:
    output = output.resolve()
    run_root = run_root.resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to overwrite non-empty output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    verify_frozen_sources()

    map_yaml = run_root / "derived_map/extracted/optemap.yaml"
    topology_dir = run_root / "derived_map/topology_cache"
    ctx = semantic_runtime._context(map_yaml)
    artifact = topology.load_topology(
        topology_dir, ctx.hospital_map, runtime.FOOTPRINT,
        padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    queries, _intents, metadata = load_query_set(
        QUERY_SET, actual_map_hash=EXPECTED_MAP_SHA256,
        actual_semantic_map_hash=EXPECTED_SEMANTIC_MAP_HASH,
        require_default_contract=True,
    )
    if metadata.get("query_hash") != (
        "7e2a5ddb7a91b175779c0cfc1063dad77bf1c926ee52be94c350203204bac43e"
    ):
        raise RuntimeError("frozen query hash mismatch")
    query = next(item for item in queries if item.query_id == QUERY_ID)
    l1 = DeterministicGraphAStarL1(
        ctx, artifact, map_hash=ctx.map_sha256,
        topology_hash=sha256_file(topology_dir / "topology_graph.json"),
    )
    initial_plan = l1.plan(query)
    if initial_plan is None:
        raise RuntimeError("initial L1 route unavailable")

    spec = runtime.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"Smac Hybrid unavailable: {spec.reason}")
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    session = production.SmacSession(
        ctx, output / "smac_runtime", map_yaml=map_yaml,
        log_tag="nav2_3d_v1_r1_dynamic_acceptance", local_mask_updates=True,
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
    rows: list[Dict[str, Any]] = []
    audit_rows: list[Dict[str, Any]] = []
    trace: Dict[str, Any] = {
        "task_id": TASK_ID,
        "architecture_id": ARCHITECTURE_ID,
        "query_id": QUERY_ID,
        "map_sha256": ctx.map_sha256,
        "query_hash": metadata["query_hash"],
        "angle_bins": 48,
        "motion_model": "DUBIN",
        "allow_reverse": False,
        "allow_rotate_in_place": False,
        "minimum_turning_radius_m": 0.40,
        "maximum_curvature_1pm": 2.50,
    }
    controllers: list[Layered3DV1R1Controller] = []
    try:
        session.start()

        # Scenario 1: one confirmed obstacle on the current L2 path. The
        # compact persistent D* state must repair inside the same corridor.
        session.reset_query_state("small-corridor-repair", restore_base_map=False)
        small = _new_controller(initial_plan, output / "l2_cache_small")
        controllers.append(small)
        initial_l2_path = list(small.l2.path_global or ())
        obstacle = [initial_l2_path[len(initial_l2_path) * 2 // 5]]
        initial_binding = small.l2.binding_hash
        initial_route = tuple(small.plan.route_edge_ids)
        pending, repaired = _confirmed_step(
            small, "small", obstacle, map_hash=ctx.map_sha256,
            shape=artifact.free_mask.shape,
        )
        l2_result = repaired.l2_result
        if l2_result is None or not l2_result.success:
            raise RuntimeError("small: L2 repair failed")
        if l2_result.selected_backend != "compact_persistent_dstar":
            raise RuntimeError(f"small: unexpected backend {l2_result.selected_backend}")
        if l2_result.partial_dstar_result_returned:
            raise RuntimeError("small: partial D* result returned")
        repaired_path = list(small.l2.path_global or ())
        if repaired_path == initial_l2_path:
            raise RuntimeError("small: obstacle did not change the L2 path")
        if repaired.l1_graph_astar_called or small.l2.binding_hash != initial_binding:
            raise RuntimeError("small: repair escaped the original L1 corridor/binding")
        small_l3 = _run_l3(
            name="small", controller=small, step=repaired, query=query,
            session=session, spec=spec, auditor=auditor,
        )
        small_path = _write_l3_path(output, "small_obstacle_corridor_repair", small_l3)
        rows.append({
            "scenario": "small_obstacle_corridor_repair", "success": True,
            "source_count": len(obstacle),
            "confirmed_blocked_cells": len(repaired.snapshot_update.blocked_cells),
            "scheduler": repaired.scheduler.reason,
            "l1_called": repaired.l1_graph_astar_called,
            "route_changed": tuple(small.plan.route_edge_ids) != initial_route,
            "binding_changed": small.l2.binding_hash != initial_binding,
            "l2_backend": l2_result.selected_backend,
            "l2_response_ms": l2_result.response_ms,
            "l2_path_changed": repaired_path != initial_l2_path,
            "partial_dstar": l2_result.partial_dstar_result_returned,
            "l3_success": small_l3["success"], "l3_wall_ms": small_l3["wall_ms"],
            "roi_ack": small_l3["diagnostics"].get("costmap_update_acknowledged"),
            "roi_ack_mismatch_cells": small_l3["diagnostics"].get("costmap_ack_mismatch_cells"),
            "l3_path_file": str(small_path.relative_to(output)),
            "l3_path_sha256": sha256_file(small_path),
        })
        audit_rows.append({"scenario": "small_obstacle_corridor_repair", **small_l3["metrics"]})
        trace["small_obstacle_corridor_repair"] = {
            "obstacle_sources": obstacle,
            "pending": _step_trace(pending),
            "confirmed": _step_trace(repaired),
            "initial_l2_path_hash": _stable_hash(initial_l2_path),
            "repaired_l2_path_hash": _stable_hash(repaired_path),
            "initial_route_edge_ids": initial_route,
            "final_route_edge_ids": small.plan.route_edge_ids,
            "initial_l2_binding": initial_binding,
            "final_l2_binding": small.l2.binding_hash,
            "l3": {key: value for key, value in small_l3.items() if key != "result"},
        }

        # Scenario 2: a wide obstacle closes edge 202. L2 no-route is the only
        # condition that invokes L1, which excludes edge 202 and selects the
        # independent 203/205 channel before rebuilding L2 and calling L3.
        session.reset_query_state("wide-cross-channel-reroute", restore_base_map=True)
        wide = _new_controller(initial_plan, output / "l2_cache_wide")
        controllers.append(wide)
        wide_initial_route = tuple(wide.plan.route_edge_ids)
        wide_initial_binding = wide.l2.binding_hash
        barrier = _wide_barrier(ctx)
        pending, rerouted = _confirmed_step(
            wide, "wide", barrier, map_hash=ctx.map_sha256,
            shape=artifact.free_mask.shape,
            l1_replan=lambda blocked: l1.plan(query, blocked),
        )
        l2_result = rerouted.l2_result
        blocked_edges = tuple(sorted(l1.blocked_edges(rerouted.snapshot_update.blocked_cells)))
        if not rerouted.l1_graph_astar_called or not rerouted.l1_reroute_succeeded:
            raise RuntimeError("wide: L2 no-route did not cause successful L1 reroute")
        if l2_result is None or not l2_result.success or l2_result.partial_dstar_result_returned:
            raise RuntimeError("wide: rebound L2 did not return a complete path")
        if tuple(wide.plan.route_edge_ids) == wide_initial_route:
            raise RuntimeError("wide: route_edge_ids did not change")
        if wide.l2.binding_hash == wide_initial_binding:
            raise RuntimeError("wide: L2 binding did not change")
        if 202 not in blocked_edges or "203" not in wide.plan.route_edge_ids or "205" not in wide.plan.route_edge_ids:
            raise RuntimeError("wide: expected edge-202 exclusion / 203-205 alternate route missing")
        wide_l3 = _run_l3(
            name="wide", controller=wide, step=rerouted, query=query,
            session=session, spec=spec, auditor=auditor,
        )
        wide_path = _write_l3_path(output, "wide_obstacle_cross_channel_reroute", wide_l3)
        rows.append({
            "scenario": "wide_obstacle_cross_channel_reroute", "success": True,
            "source_count": len(barrier),
            "confirmed_blocked_cells": len(rerouted.snapshot_update.blocked_cells),
            "scheduler": rerouted.scheduler.reason,
            "l1_called": rerouted.l1_graph_astar_called,
            "l1_reroute_succeeded": rerouted.l1_reroute_succeeded,
            "blocked_edge_ids": blocked_edges,
            "route_changed": tuple(wide.plan.route_edge_ids) != wide_initial_route,
            "binding_changed": wide.l2.binding_hash != wide_initial_binding,
            "l2_backend": l2_result.selected_backend,
            "partial_dstar": l2_result.partial_dstar_result_returned,
            "l3_success": wide_l3["success"], "l3_wall_ms": wide_l3["wall_ms"],
            "roi_ack": wide_l3["diagnostics"].get("costmap_update_acknowledged"),
            "roi_ack_mismatch_cells": wide_l3["diagnostics"].get("costmap_ack_mismatch_cells"),
            "l3_path_file": str(wide_path.relative_to(output)),
            "l3_path_sha256": sha256_file(wide_path),
        })
        audit_rows.append({"scenario": "wide_obstacle_cross_channel_reroute", **wide_l3["metrics"]})
        trace["wide_obstacle_cross_channel_reroute"] = {
            "obstacle_sources": barrier,
            "pending": _step_trace(pending),
            "confirmed": _step_trace(rerouted),
            "blocked_edge_ids": blocked_edges,
            "initial_route_edge_ids": wide_initial_route,
            "final_route_edge_ids": wide.plan.route_edge_ids,
            "initial_l2_binding": wide_initial_binding,
            "final_l2_binding": wide.l2.binding_hash,
            "l3": {key: value for key, value in wide_l3.items() if key != "result"},
        }

        # Safety subcase: a map-spanning separator removes every alternate
        # channel. It must fail closed at L1 and can recover only after the
        # second clear observation confirms removal.
        session.reset_query_state("no-alternate-fail-closed", restore_base_map=True)
        no_alt = _new_controller(initial_plan, output / "l2_cache_no_alternate")
        controllers.append(no_alt)
        no_alt_barrier = sorted(_barrier_sources(
            no_alt.plan.start_cell, no_alt.plan.goal_cell, artifact.free_mask.shape,
        ))
        _pending, halted = _confirmed_step(
            no_alt, "no-alternate", no_alt_barrier,
            map_hash=ctx.map_sha256, shape=artifact.free_mask.shape,
            l1_replan=lambda blocked: l1.plan(query, blocked),
        )
        if halted.failure_code != "L1_NO_ROUTE" or not halted.l1_graph_astar_called:
            raise RuntimeError("no-alternate: did not halt with L1_NO_ROUTE")
        if halted.l3_required:
            raise RuntimeError("no-alternate: L3 was called while L1 had no route")
        recovering = no_alt.process_snapshot(
            _snapshot("clear", 3, (), map_hash=ctx.map_sha256, shape=artifact.free_mask.shape),
            l1_replan=lambda blocked: l1.plan(query, blocked), now=3.0,
        )
        if recovering.scheduler.invoke_l2:
            raise RuntimeError("no-alternate: recovered before the second clear observation")
        recovered = no_alt.process_snapshot(
            _snapshot("clear", 4, (), map_hash=ctx.map_sha256, shape=artifact.free_mask.shape),
            l1_replan=lambda blocked: l1.plan(query, blocked), now=4.0,
        )
        if recovered.l2_result is None or not recovered.l2_result.success or not recovered.l3_required:
            raise RuntimeError("no-alternate: recovery failed after confirmed clear")
        recovery_l3 = _run_l3(
            name="recovery", controller=no_alt, step=recovered, query=query,
            session=session, spec=spec, auditor=auditor,
        )
        recovery_path = _write_l3_path(output, "no_alternate_recovery", recovery_l3)
        rows.append({
            "scenario": "no_alternate_fail_closed_and_recovery", "success": True,
            "source_count": len(no_alt_barrier),
            "confirmed_blocked_cells": len(halted.snapshot_update.blocked_cells),
            "scheduler": halted.scheduler.reason,
            "failure_code": halted.failure_code,
            "l1_called": halted.l1_graph_astar_called,
            "l1_reroute_succeeded": halted.l1_reroute_succeeded,
            "l3_called_while_blocked": halted.l3_required,
            "first_clear_scheduler": recovering.scheduler.reason,
            "first_clear_invoked_l2": recovering.scheduler.invoke_l2,
            "second_clear_scheduler": recovered.scheduler.reason,
            "recovery_l2_backend": recovered.l2_result.selected_backend,
            "recovery_success": recovery_l3["success"],
            "recovery_l3_wall_ms": recovery_l3["wall_ms"],
            "roi_ack": recovery_l3["diagnostics"].get("costmap_update_acknowledged"),
            "roi_ack_mismatch_cells": recovery_l3["diagnostics"].get("costmap_ack_mismatch_cells"),
            "l3_path_file": str(recovery_path.relative_to(output)),
            "l3_path_sha256": sha256_file(recovery_path),
        })
        audit_rows.append({"scenario": "no_alternate_recovery", **recovery_l3["metrics"]})
        trace["no_alternate_fail_closed_and_recovery"] = {
            "barrier_source_count": len(no_alt_barrier),
            "halted": _step_trace(halted),
            "first_clear": _step_trace(recovering),
            "second_clear": _step_trace(recovered),
            "l3": {key: value for key, value in recovery_l3.items() if key != "result"},
        }
    finally:
        for controller in controllers:
            controller.lifecycle.clear()
        session.close()

    trace["all_scenarios_passed"] = len(rows) == 3 and all(row["success"] for row in rows)
    trace["session_start_count"] = session.session_start_count
    trace["session_close_count"] = session.session_close_count
    (output / "dynamic_trace.json").write_text(
        json.dumps(_jsonable(trace), indent=2, sort_keys=True) + "\n", encoding="utf-8",
    )
    _write_csv(output / "dynamic_scenarios.csv", rows)
    _write_csv(output / "dynamic_path_audit.csv", audit_rows)
    _write_csv(output / "failures.csv", [])
    return output


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--ros-domain-id", type=int, default=218)
    args = parser.parse_args(argv)
    result = run(args.output, run_root=args.run_root, ros_domain_id=args.ros_domain_id)
    print(result)
    return 0


__all__ = ["QUERY_ID", "main", "run"]
