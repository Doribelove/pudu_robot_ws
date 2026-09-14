"""Three-arm calibration/held-out Stage A for 3D-V1/r2 acceptance."""

from __future__ import annotations

import argparse
import csv
import gc
import hashlib
import json
import math
import statistics
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

import numpy as np
import yaml

from .l2_incremental import Cell, CorridorROI
from .l2_state_lifecycle import L2StateLifecycleManager, _rss_bytes
from .pipeline import L1Plan
from .production_l1 import DeterministicGraphAStarL1
from .r1_pipeline import Layered3DV1R1Controller
from .r1_stage_a import (
    CLASSIFICATION_QUERIES,
    FROZEN_R0_STATE_BYTES,
    _barrier_sources,
    _classify_diagnostics,
    _far_off_corridor_cell,
    _local_path,
    _off_path_corridor_cell,
    _path_cost,
    _path_hash,
    _select_nonblocking_eligible_sources,
    _sha256,
    _source_snapshot,
    _stable_hash,
    _summary,
    _write_csv,
)
from .r2_pipeline import Layered3DV1R2Controller
from .r2_state_lifecycle import (
    ALGORITHM_VERSION,
    ARCHITECTURE_ID,
    FORMAT_VERSION,
    PROTOCOL_ID,
    REVISION_ID,
    R2L2StateLifecycleManager,
    STATIC_MASK_SCHEMA,
)
from .real_stage_a_benchmark import MAP_ID, ROOT, _load_inputs, _snapshot, _tree_hash


CALIBRATION_QUERIES = ("A2B-02", "A2B-07", "A2B-11", "A2B-15")
HELDOUT_QUERIES = (
    "A2B-01", "A2B-03", "A2B-04", "A2B-05",
    "A2B-06", "A2B-08", "A2B-09", "A2B-10",
    "A2B-12", "A2B-13", "A2B-17", "A2B-18",
)
WORKLOAD_GENERATION_VERSION = "r2-reversed-anchor-realistic-synthetic-v1"
WORKLOAD_SEED = 20260904
DEFAULT_FROZEN_CONFIG = (
    Path(__file__).resolve().parents[1] / "config/three_d_v1_r2_production_acceptance.yaml"
)
FROZEN_BASELINES = (
    ROOT / "experiments/layered_planner_benchmark/3d_v1_l2_stage_a_selective_preflight_20260904_01",
    ROOT / "experiments/layered_planner_benchmark/3d_v1_l2_real_4x_stage_a_20260904_02",
    ROOT / "experiments/layered_planner_benchmark/3d_v1_stage_b_smoke_20260904_03",
    ROOT / "experiments/layered_planner_benchmark/3d_v1_r1_heldout_20260904_01",
    ROOT / "experiments/layered_planner_benchmark/3d_v1_r1_soak_20260904_01",
)


def _load_frozen_config(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.resolve().read_text(encoding="utf-8")) or {}
    if value.get("architecture_id") != ARCHITECTURE_ID:
        raise ValueError("frozen config architecture mismatch")
    if value.get("revision_id") != REVISION_ID or value.get("protocol_id") != PROTOCOL_ID:
        raise ValueError("frozen config revision/protocol mismatch")
    workload = value.get("workload") or {}
    if workload.get("generation_version") != WORKLOAD_GENERATION_VERSION:
        raise ValueError("frozen workload generation mismatch")
    if int(workload.get("seed", -1)) != WORKLOAD_SEED:
        raise ValueError("frozen workload seed mismatch")
    policy = value.get("policy") or {}
    expected = {
        "algorithm_version": ALGORITHM_VERSION,
        "static_mask_schema": STATIC_MASK_SCHEMA,
        "format_version": FORMAT_VERSION,
        "online_synchronous_dstar_build": False,
    }
    for key, expected_value in expected.items():
        if policy.get(key) != expected_value:
            raise ValueError(f"frozen policy mismatch: {key}")
    freeze = value.get("freeze") or {}
    calibration = Path(str(freeze.get("calibration_directory", "")))
    for filename, hash_key in (
        ("gate_results.yaml", "calibration_gate_results_sha256"),
        ("manifest.yaml", "calibration_manifest_sha256"),
    ):
        artifact = calibration / filename
        if not artifact.is_file() or _sha256(artifact) != freeze.get(hash_key):
            raise ValueError(f"frozen calibration evidence mismatch: {filename}")
    gates = yaml.safe_load((calibration / "gate_results.yaml").read_text(encoding="utf-8")) or {}
    if gates.get("stage_a_pass") is not True:
        raise ValueError("frozen calibration did not pass Stage A")
    package = Path(__file__).resolve().parent
    for filename, hash_key in (
        ("r2_state_lifecycle.py", "r2_state_lifecycle_sha256"),
        ("r2_pipeline.py", "r2_pipeline_sha256"),
        ("r2_stage_a.py", "r2_stage_a_pre_freeze_sha256"),
    ):
        if _sha256(package / filename) != freeze.get(hash_key):
            raise ValueError(f"post-freeze implementation changed: {filename}")
    return value


def _phase_fields(step: Any) -> Dict[str, Any]:
    diagnostics = step.diagnostics
    return {
        "confirmation_wall_ms": diagnostics.get("confirmation_wall_ms", 0.0),
        "scheduler_wall_ms": diagnostics.get("scheduler_wall_ms", 0.0),
        "target_mask_wall_ms": diagnostics.get("target_mask_wall_ms", 0.0),
        "l2_dispatch_wall_ms": diagnostics.get("l2_dispatch_wall_ms", 0.0),
        "l1_replan_wall_ms": diagnostics.get("l1_replan_wall_ms", 0.0),
        "dirty_roi_wall_ms": diagnostics.get("dirty_roi_wall_ms", 0.0),
        "pipeline_response_ms": diagnostics.get("pipeline_response_ms", 0.0),
        "end_to_end_pre_l3_ms": diagnostics.get("end_to_end_pre_l3_ms", 0.0),
    }


def _run_event(
    *,
    query_id: str,
    repetition: int,
    snapshot_index: int,
    category: str,
    occupied: Set[Cell],
    map_hash: str,
    map_shape: Sequence[int],
    roi: CorridorROI,
    arm_a: Layered3DV1R2Controller,
    arm_b: Layered3DV1R1Controller,
    arm_c: Layered3DV1R2Controller,
    rows: List[Dict[str, Any]],
) -> Tuple[Any, Any, Any]:
    snapshot = _snapshot(
        snapshot_index, sorted(occupied), map_hash=map_hash, shape=map_shape,
    )
    steps = (
        ("A_cold_grid_astar", arm_a.process_snapshot(snapshot, now=float(snapshot_index))),
        ("B_r1_selective", arm_b.process_snapshot(snapshot, now=float(snapshot_index))),
        ("C_r2_acceptance", arm_c.process_snapshot(snapshot, now=float(snapshot_index))),
    )
    reference_step = steps[0][1]
    for arm, step in steps[1:]:
        if step.snapshot_update.blocked_cells != reference_step.snapshot_update.blocked_cells:
            raise AssertionError(f"{arm} dynamic confirmation policy diverged")
        if (
            step.scheduler.invoke_l2 != reference_step.scheduler.invoke_l2
            or step.scheduler.reason != reference_step.scheduler.reason
        ):
            raise AssertionError(f"{arm} scheduler diverged")
    base = {
        "query_id": query_id,
        "repetition": repetition,
        "snapshot_index": snapshot_index,
        "category": category,
        "snapshot_id": snapshot.snapshot_id,
        "snapshot_hash": snapshot.snapshot_hash,
        "occupied_source_count": len(occupied),
        "scheduler_reason": reference_step.scheduler.reason,
        "scheduler_invoke_l2": reference_step.scheduler.invoke_l2,
        "scheduler_parity": True,
        "effective_changed_cells": len(reference_step.snapshot_update.effective_changed_cells),
        "newly_blocked_sources": len(reference_step.snapshot_update.newly_blocked_sources),
        "newly_freed_sources": len(reference_step.snapshot_update.newly_freed_sources),
        "workload_generation_version": WORKLOAD_GENERATION_VERSION,
        "workload_seed": WORKLOAD_SEED,
    }
    if not reference_step.scheduler.invoke_l2:
        for arm, step in steps:
            manager = arm_c.lifecycle if arm.startswith("C_") else None
            rows.append({
                **base, "arm": arm, "backend": "scheduler_skip",
                "l2_response_ms": 0.0, "search_ms": 0.0, "fallback_ms": 0.0,
                "expanded": 0, "heap_pops": 0, "update_vertex": 0,
                "predecessor_visits": 0, "reachable": True, "cost": 0.0,
                "cost_error_raw": 0.0, "cost_error": 0.0,
                "path_cell_parity": True, "blocked_or_recovering_in_path": 0,
                "partial_dstar": False, "hidden_reinitialize": 0,
                "all_correct": True,
                "resident_bytes": 0 if manager is None else manager.resident_bytes,
                "rss_bytes": _rss_bytes(),
                "synchronous_dstar_build_count": 0 if manager is None else manager.synchronous_dstar_build_count,
                "static_mask_storage_bytes": 0,
                "static_mask_logical_bytes": 0,
                "static_mask_materialize_ms": 0.0,
                **_phase_fields(step),
            })
        return tuple(item[1] for item in steps)  # type: ignore[return-value]

    baseline_result = reference_step.l2_result
    if baseline_result is None:
        raise AssertionError("cold A* arm produced no invoked result")
    oracle_path = _local_path(roi, baseline_result.path)
    oracle_cost = _path_cost(oracle_path)
    blocked_local = {
        roi.to_local(cell)
        for cell in reference_step.snapshot_update.blocked_cells
        if roi.contains_global(cell)
    }
    for arm, step in steps:
        result = step.l2_result
        if result is None:
            raise AssertionError(f"{arm} invoked L2 produced no result")
        local_path = _local_path(roi, result.path)
        reachable_parity = (local_path is None) == (oracle_path is None)
        raw_error = 0.0 if local_path is None and oracle_path is None else (
            math.inf if local_path is None or oracle_path is None
            else abs(_path_cost(local_path) - oracle_cost)
        )
        canonical_error = 0.0 if raw_error <= 1.0e-9 else raw_error
        blocked_count = len(set(local_path or ()).intersection(blocked_local))
        correct = bool(
            reachable_parity and canonical_error == 0.0 and blocked_count == 0
            and not result.partial_dstar_result_returned
        )
        static_mask = result.diagnostics.get("static_mask", {})
        rows.append({
            **base, "arm": arm, "backend": result.selected_backend,
            "l2_response_ms": result.response_ms,
            "search_ms": result.dstar_stats.search_time_ms,
            "fallback_ms": 0.0 if result.fallback_stats is None else result.fallback_stats.search_time_ms,
            "expanded": result.dstar_stats.expanded_nodes,
            "heap_pops": result.dstar_stats.queue_pops,
            "update_vertex": result.dstar_stats.update_vertex_count,
            "predecessor_visits": result.diagnostics.get("predecessor_visits_total", 0),
            "reachable": result.path is not None,
            "cost": _path_cost(local_path),
            "cost_error_raw": raw_error,
            "cost_error": canonical_error,
            "path_cell_parity": local_path == oracle_path,
            "path_hash": _path_hash(local_path),
            "blocked_or_recovering_in_path": blocked_count,
            "partial_dstar": result.partial_dstar_result_returned,
            "hidden_reinitialize": int(result.diagnostics.get("reinitialize_count", 0)),
            "all_correct": correct,
            "resident_bytes": (
                arm_c.lifecycle.resident_bytes if arm.startswith("C_")
                else int(result.diagnostics.get("state_memory_bytes", 0))
            ),
            "rss_bytes": _rss_bytes(),
            "synchronous_dstar_build_count": (
                arm_c.lifecycle.synchronous_dstar_build_count if arm.startswith("C_") else 0
            ),
            "static_mask_storage_bytes": static_mask.get("storage_bytes", 0),
            "static_mask_logical_bytes": static_mask.get("logical_bytes", 0),
            "static_mask_materialize_ms": static_mask.get("last_materialize_ms", 0.0),
            **_phase_fields(step),
        })
    return tuple(item[1] for item in steps)  # type: ignore[return-value]


def _write_timing_summary(path: Path, rows: Sequence[Mapping[str, Any]], query_ids: Sequence[str]) -> List[Dict[str, Any]]:
    invoked = [row for row in rows if row["scheduler_invoke_l2"]]
    result: List[Dict[str, Any]] = []
    groups = (
        ("all_invoked", invoked),
        ("eligible", [row for row in invoked if "eligible" in str(row["category"])]),
        ("fallback", [row for row in invoked if "fallback" in str(row["category"])]),
        ("no_route", [row for row in invoked if row["category"] == "no_route_confirmed"]),
        ("recovery", [row for row in invoked if row["category"] == "recovery_confirmed"]),
    )
    arms = ("A_cold_grid_astar", "B_r1_selective", "C_r2_acceptance")
    for metric in ("l2_response_ms", "pipeline_response_ms"):
        for group, selected in groups:
            for arm in arms:
                values = [float(row[metric]) for row in selected if row["arm"] == arm]
                result.append({"metric": metric, "group": group, "arm": arm, **_summary(values)})
        for query_id in query_ids:
            for arm in arms:
                values = [
                    float(row[metric]) for row in invoked
                    if row["query_id"] == query_id and row["arm"] == arm
                ]
                result.append({"metric": metric, "group": f"query:{query_id}", "arm": arm, **_summary(values)})
    _write_csv(path, result)
    return result


def run(
    output: Path,
    *,
    mode: str,
    query_ids: Sequence[str],
    repetitions: int,
    dstar_budget_ms: float = 500.0,
    max_active_states: int = 1,
    frozen_config_path: Path = DEFAULT_FROZEN_CONFIG,
) -> Path:
    output = output.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    if mode not in {"calibration", "heldout"}:
        raise ValueError("mode must be calibration or heldout")
    if repetitions < 1:
        raise ValueError("repetitions must be positive")
    if mode == "heldout" and (repetitions < 10 or len(query_ids) < 12):
        raise ValueError("heldout requires at least 12 queries and 10 repetitions")
    frozen_config: Optional[Mapping[str, Any]] = None
    if mode == "heldout":
        frozen_config = _load_frozen_config(frozen_config_path)
        workload = frozen_config["workload"]
        policy = frozen_config["policy"]
        if tuple(query_ids) != tuple(workload["heldout_queries"]):
            raise ValueError("heldout query set/order differs from frozen config")
        if repetitions != int(workload["paired_repetitions_per_heldout_query"]):
            raise ValueError("heldout repetitions differ from frozen config")
        if float(dstar_budget_ms) != float(policy["dstar_wall_budget_ms"]):
            raise ValueError("heldout D* budget differs from frozen config")
        if int(max_active_states) != int(policy["max_active_states_default"]):
            raise ValueError("heldout LRU size differs from frozen config")
    output.mkdir(parents=True)
    frozen_before = {str(path): _tree_hash(path) for path in FROZEN_BASELINES}
    experiment_started = time.monotonic_ns()
    rows: List[Dict[str, Any]] = []
    initialization_rows: List[Dict[str, Any]] = []
    query_rows: List[Dict[str, Any]] = []
    ctx, queries, artifact, cache_manifest = _load_inputs()
    query_by_id = {item.query_id: item for item in queries}
    unknown = sorted(set(query_ids) - set(query_by_id))
    if unknown:
        raise ValueError(f"unknown queries: {unknown}")
    l1 = DeterministicGraphAStarL1(
        ctx, artifact, map_hash=ctx.map_sha256,
        topology_hash=str(cache_manifest.get("cache_key") or ""),
    )
    classification_rows = _classify_diagnostics(l1, queries)
    r1_cache_root = output / "verified_r1_baseline_cache"
    r2_cache_root = output / "verified_r2_cache"
    r1_lifecycle = L2StateLifecycleManager(
        r1_cache_root, max_active_states=max_active_states,
        dstar_wall_budget_ms=dstar_budget_ms, dstar_max_expansions=20_000,
    )
    r2_lifecycle = R2L2StateLifecycleManager(
        r2_cache_root, max_active_states=max_active_states,
        dstar_wall_budget_ms=dstar_budget_ms, dstar_max_expansions=20_000,
    )
    print(f"START mode={mode} queries={len(query_ids)} repetitions={repetitions}", flush=True)

    for query_id in query_ids:
        query = query_by_id[query_id]
        l1_started = time.monotonic_ns()
        plan = l1.plan(query)
        l1_ms = (time.monotonic_ns() - l1_started) / 1.0e6
        if plan is None:
            raise RuntimeError(f"production L1 found no route for {query_id}")
        roi = CorridorROI.from_global(
            plan.static_safe_free, plan.corridor_mask,
            plan.start_cell, plan.goal_cell,
            binding_fields=plan.binding_fields(),
        )

        r1_lifecycle.clear()
        started = time.monotonic_ns()
        r1_prebuild = Layered3DV1R1Controller(
            plan, cache_root=r1_cache_root, lifecycle_manager=r1_lifecycle,
            max_active_states=max_active_states,
            dynamic_inflation_radius_cells=7,
            dstar_wall_budget_ms=dstar_budget_ms,
            dstar_max_expansions=20_000,
            dstar_attempt_max_changed_cells=2,
        )
        r1_cold_ms = (time.monotonic_ns() - started) / 1.0e6
        r1_activation = r1_lifecycle.last_activation
        if not r1_prebuild.initial_l2_result.success or r1_activation is None:
            raise RuntimeError(f"r1 prebuild failed for {query_id}")
        r1_lifecycle.save_active()
        r1_lifecycle.clear()
        del r1_prebuild
        gc.collect()

        r2_prebuild = r2_lifecycle.prebuild(roi)
        if not r2_prebuild.success:
            raise RuntimeError(f"r2 prebuild failed for {query_id}")
        query_rows.append({
            "query_id": query_id,
            "l1_ms": l1_ms,
            "route_edge_count": len(plan.route_edge_ids),
            "route_length_m": plan.diagnostics.get("corridor_route_length_m", 0.0),
            "turn_count": plan.diagnostics.get("corner_count", 0),
            "corridor_cells": int(np.count_nonzero(plan.corridor_mask)),
            "roi_shape": list(roi.shape),
            "roi_array_cells": int(roi.base_free.size),
            "safe_state_cells": int(np.count_nonzero(roi.base_free)),
            "r1_cold_build_and_serialize_ms": r1_cold_ms,
            "r1_state_bytes": r1_activation.resident_bytes,
            "r2_offline_prebuild_ms": r2_prebuild.total_ms,
            "r2_first_solve_ms": r2_prebuild.first_solve_ms,
            "r2_state_bytes": r2_prebuild.resident_bytes,
        })
        print(
            f"PREBUILT {query_id} r1_ms={r1_cold_ms:.1f} "
            f"r2_ms={r2_prebuild.total_ms:.1f} r2_bytes={r2_prebuild.resident_bytes}",
            flush=True,
        )

        for repetition in range(1, repetitions + 1):
            r1_lifecycle.clear()
            r2_lifecycle.clear()
            arm_a = Layered3DV1R2Controller(
                plan, cache_root=output / "arm_a_empty_cache",
                dynamic_inflation_radius_cells=7,
                dstar_wall_budget_ms=dstar_budget_ms,
                dstar_max_expansions=20_000,
                dstar_attempt_max_changed_cells=2,
            )
            started = time.monotonic_ns()
            arm_b = Layered3DV1R1Controller(
                plan, cache_root=r1_cache_root, lifecycle_manager=r1_lifecycle,
                max_active_states=max_active_states,
                dynamic_inflation_radius_cells=7,
                dstar_wall_budget_ms=dstar_budget_ms,
                dstar_max_expansions=20_000,
                dstar_attempt_max_changed_cells=2,
            )
            r1_activate_ms = (time.monotonic_ns() - started) / 1.0e6
            started = time.monotonic_ns()
            arm_c = Layered3DV1R2Controller(
                plan, cache_root=r2_cache_root, lifecycle_manager=r2_lifecycle,
                max_active_states=max_active_states,
                dynamic_inflation_radius_cells=7,
                dstar_wall_budget_ms=dstar_budget_ms,
                dstar_max_expansions=20_000,
                dstar_attempt_max_changed_cells=2,
            )
            r2_activate_ms = (time.monotonic_ns() - started) / 1.0e6
            r1_warm = r1_lifecycle.last_activation
            r2_warm = r2_lifecycle.last_activation
            if r1_warm is None or r2_warm is None:
                raise AssertionError("missing warm activation telemetry")
            if not arm_b.initial_l2_result.success or not arm_c.initial_l2_result.success:
                raise RuntimeError(f"warm activation failed for {query_id}")
            initialization_rows.append({
                "query_id": query_id,
                "repetition": repetition,
                "r1_warm_activate_ms": r1_activate_ms,
                "r1_geometry_cache_hit": r1_warm.geometry_cache.hit,
                "r1_state_cache_hit": r1_warm.state_cache.hit,
                "r1_resident_bytes": r1_warm.resident_bytes,
                "r2_warm_activate_ms": r2_activate_ms,
                "r2_geometry_cache_hit": r2_warm.geometry_cache.hit,
                "r2_geometry_restore_ms": r2_warm.geometry_cache.wall_ms,
                "r2_state_cache_hit": r2_warm.state_cache.hit,
                "r2_state_restore_ms": r2_warm.state_cache.wall_ms,
                "r2_cache_decision_ms": r2_warm.cache_decision_ms,
                "r2_static_mask_pack_ms": r2_warm.static_mask_pack_ms,
                "r2_active_state_count": r2_warm.active_state_count,
                "r2_resident_bytes": r2_warm.resident_bytes,
                "r2_synchronous_dstar_build": r2_warm.synchronous_dstar_build_performed,
                "r2_rss_before_bytes": r2_warm.rss_before_bytes,
                "r2_rss_after_bytes": r2_warm.rss_after_bytes,
            })

            occupied: Set[Cell] = set()
            initial_paths = (
                list(arm_a.l2.path_global or ()),
                list(arm_b.l2.path_global or ()),
                list(arm_c.l2.path_global or ()),
            )
            workload_paths = tuple(list(reversed(path)) for path in initial_paths)
            one = _select_nonblocking_eligible_sources(
                paths=workload_paths, count=1, existing_sources=set(),
                plan=plan, map_shape=artifact.free_mask.shape,
            )
            off_corridor = _far_off_corridor_cell(plan)
            off_path = _off_path_corridor_cell(roi, initial_paths[0])
            barrier = _barrier_sources(
                plan.start_cell, plan.goal_cell, artifact.free_mask.shape,
            )
            event_index = 0

            def event(category: str, sources: Set[Cell]) -> Tuple[Any, Any, Any]:
                nonlocal event_index
                event_index += 1
                return _run_event(
                    query_id=query_id, repetition=repetition,
                    snapshot_index=event_index, category=category,
                    occupied=sources, map_hash=ctx.map_sha256,
                    map_shape=artifact.free_mask.shape, roi=roi,
                    arm_a=arm_a, arm_b=arm_b, arm_c=arm_c, rows=rows,
                )

            event("unconfirmed", one)
            event("one_cell_eligible", one)
            occupied |= one
            event("duplicate", occupied)
            event("off_corridor_pending", occupied | {off_corridor})
            event("off_corridor_confirmed", occupied | {off_corridor})
            occupied.add(off_corridor)
            event("off_path_pending", occupied | {off_path})
            event("off_path_confirmed", occupied | {off_path})
            occupied.add(off_path)
            current_paths = (
                list(arm_a.l2.path_global or initial_paths[0]),
                list(arm_b.l2.path_global or initial_paths[1]),
                list(arm_c.l2.path_global or initial_paths[2]),
            )
            two = _select_nonblocking_eligible_sources(
                paths=tuple(list(reversed(path)) for path in current_paths),
                count=2, existing_sources=occupied, plan=plan,
                map_shape=artifact.free_mask.shape,
            )
            event("two_cell_pending", occupied | two)
            event("two_cell_eligible", occupied | two)
            occupied |= two
            current_paths = (
                list(arm_a.l2.path_global or current_paths[0]),
                list(arm_b.l2.path_global or current_paths[1]),
                list(arm_c.l2.path_global or current_paths[2]),
            )
            large = _select_nonblocking_eligible_sources(
                paths=tuple(list(reversed(path)) for path in current_paths),
                count=5, existing_sources=occupied, plan=plan,
                map_shape=artifact.free_mask.shape,
            )
            event("large_change_pending", occupied | large)
            event("large_change_fallback", occupied | large)
            occupied |= large
            event("no_route_pending", occupied | barrier)
            no_route = event("no_route_confirmed", occupied | barrier)
            if any(step.l2_result is None or step.l2_result.success for step in no_route):
                raise AssertionError(f"barrier failed to produce no-route for {query_id}")
            event("recovery_pending", set())
            recovery = event("recovery_confirmed", set())
            if any(step.l2_result is None or not step.l2_result.success for step in recovery):
                raise AssertionError(f"recovery failed for {query_id}")

            r1_lifecycle.clear()
            r2_lifecycle.clear()
            del arm_a, arm_b, arm_c
            gc.collect()
            print(f"RUN {query_id} repetition={repetition}/{repetitions}", flush=True)
        gc.collect()

    frozen_after = {str(path): _tree_hash(path) for path in FROZEN_BASELINES}
    if frozen_before != frozen_after:
        raise RuntimeError("a frozen r0/r1 baseline changed during r2 Stage A")
    _write_csv(output / "runs.csv", rows)
    with (output / "runs.jsonl").open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, sort_keys=True) + "\n")
    _write_csv(output / "initialization.csv", initialization_rows)
    _write_csv(output / "query_stratification.csv", query_rows)
    _write_csv(output / "classification_diagnostics.csv", classification_rows)
    timing_rows = _write_timing_summary(output / "timing_summary.csv", rows, query_ids)

    def timing(metric: str, group: str, arm: str) -> Mapping[str, Any]:
        return next(row for row in timing_rows if row["metric"] == metric and row["group"] == group and row["arm"] == arm)

    l2_a = timing("l2_response_ms", "all_invoked", "A_cold_grid_astar")
    l2_b = timing("l2_response_ms", "all_invoked", "B_r1_selective")
    l2_c = timing("l2_response_ms", "all_invoked", "C_r2_acceptance")
    pipeline_a = timing("pipeline_response_ms", "all_invoked", "A_cold_grid_astar")
    pipeline_c = timing("pipeline_response_ms", "all_invoked", "C_r2_acceptance")
    eligible_b = timing("l2_response_ms", "eligible", "B_r1_selective")
    eligible_c = timing("l2_response_ms", "eligible", "C_r2_acceptance")
    candidate_rows = [row for row in rows if row["arm"] == "C_r2_acceptance"]
    correctness_rows = [row for row in rows if row["arm"] in {"B_r1_selective", "C_r2_acceptance"}]
    warm_values = [float(row["r2_warm_activate_ms"]) for row in initialization_rows]
    resident_values = [float(row["r2_resident_bytes"]) for row in initialization_rows] + [
        float(row["resident_bytes"]) for row in candidate_rows
    ]
    telemetry_fields = (
        "confirmation_wall_ms", "scheduler_wall_ms", "target_mask_wall_ms",
        "l2_dispatch_wall_ms", "dirty_roi_wall_ms", "pipeline_response_ms",
        "end_to_end_pre_l3_ms",
    )
    telemetry_complete = all(all(field in row for field in telemetry_fields) for row in rows)
    gates_config = (frozen_config or {}).get("gates", {})
    thresholds = {
        "p50_reduction": float(gates_config.get("r2_p50_reduction_vs_cold_astar_min", 0.20)),
        "p95_ratio": float(gates_config.get("r2_pipeline_p95_ratio_vs_cold_astar_max", 1.05)),
        "p99_ratio": float(gates_config.get("r2_pipeline_p99_ratio_vs_cold_astar_max", 1.10)),
        "warm_ms": float(gates_config.get("warm_activation_p95_ms_max", 250.0)),
        "resident_bytes": int(gates_config.get("resident_bytes_target_max", 30_000_000)),
        "eligible_regression": float(gates_config.get("eligible_p95_regression_vs_r1_max", 0.05)),
    }
    p50_reduction = 1.0 - float(l2_c["p50"]) / float(l2_a["p50"])
    p95_ratio = float(pipeline_c["p95"]) / float(pipeline_a["p95"])
    p99_ratio = float(pipeline_c["p99"]) / float(pipeline_a["p99"])
    eligible_regression = float(eligible_c["p95"]) / float(eligible_b["p95"]) - 1.0
    gates: Dict[str, Any] = {
        "mode": mode,
        "workload_classification": "realistic_synthetic_workload_on_real_4x_map",
        "reliable_real_dynamic_log_found": False,
        "workload_generation_version": WORKLOAD_GENERATION_VERSION,
        "workload_seed": WORKLOAD_SEED,
        "query_count": len(query_ids),
        "paired_repetitions_per_query": repetitions,
        "correctness_rows": len(correctness_rows),
        "correctness_failures": sum(not bool(row["all_correct"]) for row in correctness_rows),
        "oracle_parity_pass": all(bool(row["all_correct"]) for row in correctness_rows),
        "canonical_cost_error_max": max(float(row["cost_error"]) for row in correctness_rows),
        "max_raw_cost_error": max(float(row["cost_error_raw"]) for row in correctness_rows),
        "blocked_or_recovering_in_path": sum(int(row["blocked_or_recovering_in_path"]) for row in correctness_rows),
        "partial_dstar_results": sum(bool(row["partial_dstar"]) for row in correctness_rows),
        "hidden_reinitialize_count": sum(int(row["hidden_reinitialize"]) for row in candidate_rows),
        "scheduler_parity_pass": all(bool(row["scheduler_parity"]) for row in rows),
        "recovery_pass": all(bool(row["all_correct"]) for row in correctness_rows if row["category"] == "recovery_confirmed"),
        "path_cell_parity": {
            arm: {
                "matching_rows": sum(bool(row["path_cell_parity"]) for row in correctness_rows if row["arm"] == arm),
                "total_rows": sum(1 for row in correctness_rows if row["arm"] == arm),
            }
            for arm in ("B_r1_selective", "C_r2_acceptance")
        },
        "cold_astar_l2_all_invoked": dict(l2_a),
        "r1_l2_all_invoked": dict(l2_b),
        "r2_l2_all_invoked": dict(l2_c),
        "cold_astar_pipeline_all_invoked": dict(pipeline_a),
        "r2_pipeline_all_invoked": dict(pipeline_c),
        "r2_l2_p50_reduction_vs_astar": p50_reduction,
        "r2_pipeline_p95_ratio_vs_astar": p95_ratio,
        "r2_pipeline_p99_ratio_vs_astar": p99_ratio,
        "eligible_p95_regression_vs_r1": eligible_regression,
        "warm_activation_ms": _summary(warm_values),
        "resident_bytes": _summary(resident_values),
        "resident_reduction_vs_r0": 1.0 - max(resident_values) / FROZEN_R0_STATE_BYTES,
        "lru_peak_active_state_count": r2_lifecycle.peak_active_state_count,
        "lru_hard_limit": r2_lifecycle.HARD_MAX_ACTIVE_STATES,
        "cache_hits": sum(bool(row["r2_state_cache_hit"] and row["r2_geometry_cache_hit"]) for row in initialization_rows),
        "cache_misses": sum(not bool(row["r2_state_cache_hit"] and row["r2_geometry_cache_hit"]) for row in initialization_rows),
        "synchronous_dstar_build_count": r2_lifecycle.synchronous_dstar_build_count,
        "telemetry_complete": telemetry_complete,
        "frozen_baselines_unchanged": frozen_before == frozen_after,
        "frozen_thresholds": thresholds,
    }
    gates.update({
        "p50_gate_pass": p50_reduction >= thresholds["p50_reduction"],
        "p95_gate_pass": p95_ratio <= thresholds["p95_ratio"],
        "p99_gate_pass": p99_ratio <= thresholds["p99_ratio"],
        "warm_activation_gate_pass": gates["warm_activation_ms"]["p95"] <= thresholds["warm_ms"],
        "resident_target_pass": gates["resident_bytes"]["max"] <= thresholds["resident_bytes"],
        "resident_reduction_gate_pass": gates["resident_reduction_vs_r0"] >= 0.30,
        "eligible_regression_gate_pass": eligible_regression <= thresholds["eligible_regression"],
        "lru_bound_pass": r2_lifecycle.peak_active_state_count <= max_active_states <= 2,
        "cache_hit_pass": gates["cache_hits"] == len(initialization_rows),
        "no_sync_build_pass": gates["synchronous_dstar_build_count"] == 0,
        "telemetry_complete_pass": telemetry_complete,
    })
    gates["stage_a_pass"] = all(bool(gates[key]) for key in (
        "oracle_parity_pass", "scheduler_parity_pass", "recovery_pass",
        "p50_gate_pass", "p95_gate_pass", "p99_gate_pass",
        "warm_activation_gate_pass", "resident_target_pass",
        "resident_reduction_gate_pass", "eligible_regression_gate_pass",
        "lru_bound_pass", "cache_hit_pass", "no_sync_build_pass",
        "telemetry_complete_pass", "frozen_baselines_unchanged",
    )) and all(int(gates[key]) == 0 for key in (
        "blocked_or_recovering_in_path", "partial_dstar_results",
        "hidden_reinitialize_count",
    ))
    (output / "gate_results.yaml").write_text(yaml.safe_dump(gates, sort_keys=False), encoding="utf-8")

    workload_manifest = {
        "classification": "realistic_synthetic_workload_on_real_4x_map",
        "reliable_real_dynamic_logs_found": False,
        "generation_version": WORKLOAD_GENERATION_VERSION,
        "seed": WORKLOAD_SEED,
        "query_ids": list(query_ids),
        "query_set_hash": _stable_hash(list(query_ids)),
        "snapshot_stream_hash": _stable_hash([
            (row["query_id"], row["repetition"], row["snapshot_index"], row["snapshot_hash"])
            for row in rows if row["arm"] == "A_cold_grid_astar"
        ]),
        "r1_heldout_reused_for_promotion": False,
        "notes": "Routes are retained for map coverage; reversed anchor selection creates a new frozen event stream. r1 held-out remains regression-only.",
    }
    (output / "workload_manifest.yaml").write_text(
        yaml.safe_dump(workload_manifest, sort_keys=False), encoding="utf-8",
    )
    source_hashes = _source_snapshot(output)
    manifest = {
        "architecture_id": ARCHITECTURE_ID,
        "revision_id": REVISION_ID,
        "protocol_id": PROTOCOL_ID,
        "mode": mode,
        "map_id": MAP_ID,
        "map_hash": ctx.map_sha256,
        "resolution_m": float(ctx.hospital_map.resolution),
        "query_ids": list(query_ids),
        "repetitions": repetitions,
        "dstar_budget_ms": dstar_budget_ms,
        "max_active_states": max_active_states,
        "frozen_config": str(frozen_config_path.resolve()) if mode == "heldout" else "",
        "frozen_config_sha256": _sha256(frozen_config_path) if mode == "heldout" else "",
        "frozen_baseline_tree_hashes": frozen_before,
        "source_files": source_hashes,
    }
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    command_queries = ",".join(query_ids)
    (output / "reproduction_command.txt").write_text(
        "cd /home/robot/pudu_robot_ws\n"
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        "PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/three_d_v1:"
        "/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation "
        f"/usr/bin/python3 -m arena_3d_v1.r2_stage_a --mode {mode} --output-dir {output} "
        f"--query-ids {command_queries} --repetitions {repetitions} "
        f"--dstar-budget-ms {dstar_budget_ms} --max-active-states {max_active_states} "
        f"--frozen-config {frozen_config_path.resolve()}\n",
        encoding="utf-8",
    )
    verification = {
        "required_artifacts_present": True,
        "frozen_baselines_unchanged": frozen_before == frozen_after,
        "three_arms_present": sorted({row["arm"] for row in rows}) == [
            "A_cold_grid_astar", "B_r1_selective", "C_r2_acceptance",
        ],
        "strict_snapshot_hash_pairing": all(
            len({row["snapshot_hash"] for row in rows if (
                row["query_id"], row["repetition"], row["snapshot_index"]
            ) == key}) == 1
            for key in {(row["query_id"], row["repetition"], row["snapshot_index"]) for row in rows}
        ),
        "source_snapshot_count": len(source_hashes),
        "stage_a_gate_pass": gates["stage_a_pass"],
        "r1_heldout_used_as_promotion_data": False,
    }
    (output / "verification.yaml").write_text(yaml.safe_dump(verification, sort_keys=False), encoding="utf-8")
    report = [
        f"# 3D-V1-r2 {mode} Stage A", "",
        f"- real map: `{MAP_ID}`; workload: **realistic synthetic**, not a measured real-world distribution",
        f"- new event stream: `{WORKLOAD_GENERATION_VERSION}`, seed `{WORKLOAD_SEED}`",
        f"- queries/repetitions: `{len(query_ids)}` / `{repetitions}`",
        f"- correctness failures: `{gates['correctness_failures']}/{gates['correctness_rows']}`",
        f"- r2 L2 vs cold A* P50 reduction: `{p50_reduction:.2%}`",
        f"- r2/A* pipeline P95/P99 ratios: `{p95_ratio:.3f}` / `{p99_ratio:.3f}`",
        f"- r2 eligible P95 regression vs r1: `{eligible_regression:.2%}`",
        f"- warm activation P95: `{gates['warm_activation_ms']['p95']:.3f} ms`",
        f"- max resident: `{gates['resident_bytes']['max']:.0f} B`",
        f"- synchronous online D* builds: `{gates['synchronous_dstar_build_count']}`",
        f"- Stage-A gate: **{'PASS' if gates['stage_a_pass'] else 'FAIL'}**", "",
        "A2B-16/A2B-19 remain classification-only. r1 held-out is regression evidence only and is not reused for this promotion decision.",
    ]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    elapsed_ms = (time.monotonic_ns() - experiment_started) / 1.0e6
    summary = f"PASS output={output} stage_a_gate={gates['stage_a_pass']} elapsed_ms={elapsed_ms:.1f}"
    (output / "stdout.log").write_text(summary + "\n", encoding="utf-8")
    (output / "stderr.log").write_text("", encoding="utf-8")
    print(summary, flush=True)
    return output


def _default_output(mode: str) -> Path:
    return ROOT / "experiments/layered_planner_benchmark" / (
        f"3d_v1_r2_{mode}_{time.strftime('%Y%m%d_%H%M%S')}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("calibration", "heldout"), required=True)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--query-ids", default=None)
    parser.add_argument("--repetitions", type=int, default=None)
    parser.add_argument("--dstar-budget-ms", type=float, default=500.0)
    parser.add_argument("--max-active-states", type=int, default=1)
    parser.add_argument("--frozen-config", type=Path, default=DEFAULT_FROZEN_CONFIG)
    args = parser.parse_args()
    default_queries = CALIBRATION_QUERIES if args.mode == "calibration" else HELDOUT_QUERIES
    query_ids = tuple(
        item.strip() for item in (args.query_ids or ",".join(default_queries)).split(",")
        if item.strip()
    )
    repetitions = args.repetitions or (3 if args.mode == "calibration" else 10)
    try:
        run(
            args.output_dir or _default_output(args.mode), mode=args.mode,
            query_ids=query_ids, repetitions=repetitions,
            dstar_budget_ms=args.dstar_budget_ms,
            max_active_states=args.max_active_states,
            frozen_config_path=args.frozen_config,
        )
    except Exception as exc:
        if args.output_dir is not None:
            args.output_dir.mkdir(parents=True, exist_ok=True)
            (args.output_dir / "INTERRUPTED_RUN.md").write_text(
                f"# Interrupted run\n\n`{type(exc).__name__}: {exc}`\n", encoding="utf-8",
            )
            (args.output_dir / "stderr.log").write_text(
                f"{type(exc).__name__}: {exc}\n", encoding="utf-8",
            )
        raise


if __name__ == "__main__":
    main()
