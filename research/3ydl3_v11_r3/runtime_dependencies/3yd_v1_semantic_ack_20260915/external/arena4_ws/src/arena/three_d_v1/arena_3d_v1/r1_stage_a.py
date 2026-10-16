"""Three-arm calibration and held-out Stage-A runner for 3D-V1/r1."""

from __future__ import annotations

import argparse
import copy
import csv
import gc
import hashlib
import json
import math
import shutil
import statistics
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

import numpy as np
import yaml

from .dynamic_policy import DynamicGridConfirmation, RelevanceScheduler, _inflate
from .l2_incremental import Cell, CorridorROI, deterministic_grid_astar
from .l2_state_lifecycle import (
    ADJACENCY_RULE,
    ALGORITHM_VERSION,
    ARCHITECTURE_ID,
    GEOMETRY_SCHEMA,
    L2StateLifecycleManager,
    PROTOCOL_ID,
    REVISION_ID,
    STATE_SCHEMA,
    _rss_bytes,
)
from .pipeline import Layered3DV1Controller
from .production_l1 import DeterministicGraphAStarL1
from .r1_pipeline import Layered3DV1R1Controller
from .real_stage_a_benchmark import MAP_ID, ROOT, _load_inputs, _snapshot, _tree_hash


CALIBRATION_QUERIES = ("A2B-02", "A2B-07", "A2B-11", "A2B-15")
HELDOUT_QUERIES = (
    "A2B-01", "A2B-03", "A2B-04", "A2B-05",
    "A2B-06", "A2B-08", "A2B-09", "A2B-10",
    "A2B-12", "A2B-13", "A2B-17", "A2B-18",
)
CLASSIFICATION_QUERIES = ("A2B-16", "A2B-19")
FROZEN_R0_STATE_BYTES = 57_974_008
FROZEN_BASELINES = (
    ROOT / "experiments/layered_planner_benchmark/3d_v1_l2_stage_a_selective_preflight_20260904_01",
    ROOT / "experiments/layered_planner_benchmark/3d_v1_l2_real_4x_stage_a_20260904_02",
    ROOT / "experiments/layered_planner_benchmark/3d_v1_stage_b_smoke_20260904_03",
)
DEFAULT_FROZEN_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "config/three_d_v1_r1_l2_lifecycle.yaml"
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_frozen_config(path: Path) -> Mapping[str, Any]:
    path = path.resolve()
    value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if value.get("architecture_id") != ARCHITECTURE_ID:
        raise ValueError("frozen config architecture mismatch")
    if value.get("revision_id") != REVISION_ID or value.get("protocol_id") != PROTOCOL_ID:
        raise ValueError("frozen config revision/protocol mismatch")
    policy = value.get("policy") or {}
    expected_policy = {
        "geometry_schema": GEOMETRY_SCHEMA,
        "state_schema": STATE_SCHEMA,
        "algorithm_version": ALGORITHM_VERSION,
        "adjacency_rule": ADJACENCY_RULE,
    }
    for key, expected in expected_policy.items():
        if policy.get(key) != expected:
            raise ValueError(f"frozen config policy mismatch: {key}")
    freeze = value.get("freeze") or {}
    calibration = Path(str(freeze.get("calibration_directory", "")))
    for filename, hash_key in (
        ("gate_results.yaml", "calibration_gate_results_sha256"),
        ("manifest.yaml", "calibration_manifest_sha256"),
    ):
        artifact = calibration / filename
        if not artifact.is_file() or _sha256(artifact) != freeze.get(hash_key):
            raise ValueError(f"frozen calibration evidence mismatch: {filename}")
    calibration_gates = yaml.safe_load(
        (calibration / "gate_results.yaml").read_text(encoding="utf-8")
    ) or {}
    if calibration_gates.get("stage_a_pass") is not True:
        raise ValueError("frozen calibration did not pass Stage A")
    package = Path(__file__).resolve().parent
    for filename, hash_key in (
        ("l2_state_lifecycle.py", "l2_state_lifecycle_sha256"),
        ("r1_pipeline.py", "r1_pipeline_sha256"),
    ):
        if _sha256(package / filename) != freeze.get(hash_key):
            raise ValueError(f"post-freeze implementation changed: {filename}")
    return value


def _stable_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), default=str,
    ).encode("utf-8")).hexdigest()


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    values = list(rows)
    fields: List[str] = []
    for row in values:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or ["status"])
        writer.writeheader()
        for row in values:
            writer.writerow({
                key: json.dumps(value, sort_keys=True)
                if isinstance(value, (list, tuple, dict)) else value
                for key, value in row.items()
            })


def _summary(values: Sequence[float]) -> Dict[str, Any]:
    return {
        "count": len(values),
        "p50": float(np.percentile(values, 50)) if values else math.nan,
        "p95": float(np.percentile(values, 95)) if values else math.nan,
        "p99": float(np.percentile(values, 99)) if values else math.nan,
        "mean": statistics.fmean(values) if values else math.nan,
        "max": max(values) if values else math.nan,
    }


def _path_cost(path: Optional[Sequence[Cell]]) -> float:
    if not path:
        return INF
    return float(sum(
        math.sqrt(2.0) if first[0] != second[0] and first[1] != second[1] else 1.0
        for first, second in zip(path, path[1:])
    ))


INF = float("inf")


def _path_hash(path: Optional[Sequence[Cell]]) -> str:
    return _stable_hash([] if path is None else [list(cell) for cell in path])


def _select_path_sources(
    path: Sequence[Cell], count: int, excluded: Set[Cell],
) -> Set[Cell]:
    selected: Set[Cell] = set()
    if len(path) < 3:
        raise RuntimeError("path too short for dynamic workload")
    for index in np.linspace(len(path) * 0.15, len(path) * 0.85, count * 12, dtype=int):
        cell = tuple(path[min(len(path) - 2, max(1, int(index)))])
        if cell not in excluded:
            selected.add(cell)
        if len(selected) == count:
            return selected
    raise RuntimeError(f"could not select {count} distinct path cells")


def _select_common_path_sources(
    paths: Sequence[Sequence[Cell]], count: int, excluded: Set[Cell],
) -> Set[Cell]:
    """Select deterministic sources that affect every arm's current path."""
    if not paths or any(len(path) < 3 for path in paths):
        raise RuntimeError("paths too short for shared dynamic workload")
    common = set(paths[0])
    for path in paths[1:]:
        common.intersection_update(path)
    ordered = [cell for cell in paths[0][1:-1] if cell in common and cell not in excluded]
    if len(ordered) < count:
        raise RuntimeError(
            f"only {len(ordered)} common path cells are available; need {count}"
        )
    selected: Set[Cell] = set()
    for index in np.linspace(0, len(ordered) - 1, count * 12, dtype=int):
        selected.add(ordered[int(index)])
        if len(selected) == count:
            return selected
    raise RuntimeError(f"could not select {count} distinct common path cells")


def _path_valid_under_blocked(path: Sequence[Cell], blocked: Set[Cell]) -> bool:
    if not path or any(cell in blocked for cell in path):
        return False
    for first, second in zip(path, path[1:]):
        if first[0] != second[0] and first[1] != second[1]:
            if (first[0], second[1]) in blocked or (second[0], first[1]) in blocked:
                return False
    return True


def _path_support(path: Sequence[Cell], shape: Sequence[int]) -> Set[Cell]:
    height, width = int(shape[0]), int(shape[1])
    return {
        (row + drow, column + dcolumn)
        for row, column in path
        for drow in (-1, 0, 1)
        for dcolumn in (-1, 0, 1)
        if 0 <= row + drow < height and 0 <= column + dcolumn < width
    }


def _select_nonblocking_eligible_sources(
    *,
    paths: Sequence[Sequence[Cell]],
    count: int,
    existing_sources: Set[Cell],
    plan: Any,
    map_shape: Sequence[int],
    inflation_radius: int = 7,
) -> Set[Cell]:
    """Touch every path's scheduler support without invalidating any path."""
    if not paths or any(len(path) < 3 for path in paths):
        raise RuntimeError("paths too short for nonblocking eligible workload")
    common = set(paths[0])
    for path in paths[1:]:
        common.intersection_update(path)
    anchors = [cell for cell in paths[0][1:-1] if cell in common]
    if not anchors:
        raise RuntimeError("no common path anchors for eligible workload")
    height, width = int(map_shape[0]), int(map_shape[1])
    supports = [_path_support(path, map_shape) for path in paths]
    existing_blocked = _inflate(existing_sources, map_shape, inflation_radius)
    selected: Set[Cell] = set()
    distance = inflation_radius + 1
    offsets = ((-distance, 0), (0, -distance), (0, distance), (distance, 0))
    anchor_indices = np.linspace(0, len(anchors) - 1, min(len(anchors), count * 64), dtype=int)
    for index in anchor_indices:
        anchor = anchors[int(index)]
        for drow, dcolumn in offsets:
            candidate = (anchor[0] + drow, anchor[1] + dcolumn)
            if (
                candidate in existing_sources or candidate in selected
                or not (0 <= candidate[0] < height and 0 <= candidate[1] < width)
            ):
                continue
            candidate_blocked = _inflate(selected | {candidate}, map_shape, inflation_radius)
            newly_blocked = candidate_blocked - existing_blocked
            relevant = {cell for cell in newly_blocked if plan.corridor_mask[cell]}
            if not relevant or any(not relevant.intersection(support) for support in supports):
                continue
            combined_blocked = existing_blocked | candidate_blocked
            if any(not _path_valid_under_blocked(path, combined_blocked) for path in paths):
                continue
            selected.add(candidate)
            if len(selected) == count:
                return selected
    raise RuntimeError(
        f"could not select {count} shared nonblocking eligible sources"
    )


def _far_off_corridor_cell(plan: Any, margin: int = 9) -> Cell:
    free = plan.static_safe_free
    corridor = plan.corridor_mask
    height, width = free.shape
    for row in range(0, height, 16):
        for column in range(0, width, 16):
            if not free[row, column] or corridor[row, column]:
                continue
            r0, r1 = max(0, row - margin), min(height, row + margin + 1)
            c0, c1 = max(0, column - margin), min(width, column + margin + 1)
            if not np.any(corridor[r0:r1, c0:c1]):
                return row, column
    raise RuntimeError("could not find a dynamic source safely outside the active corridor")


def _off_path_corridor_cell(roi: CorridorROI, path: Sequence[Cell], margin: int = 10) -> Cell:
    support: Set[Cell] = {
        (row + drow, column + dcolumn)
        for row, column in path
        for drow in range(-margin, margin + 1)
        for dcolumn in range(-margin, margin + 1)
    }
    rows, columns = np.nonzero(roi.base_free)
    stride = max(1, len(rows) // 2048)
    for index in range(0, len(rows), stride):
        candidate = roi.to_global((int(rows[index]), int(columns[index])))
        if candidate not in support:
            return candidate
    raise RuntimeError("could not find a corridor cell outside current path support")


def _barrier_sources(
    start: Cell, goal: Cell, map_shape: Sequence[int], spacing: int = 5,
) -> Set[Cell]:
    """Build a map-spanning separator so no alternate corridor can bypass it."""
    height, width = int(map_shape[0]), int(map_shape[1])
    result: Set[Cell] = set()
    if abs(start[1] - goal[1]) >= abs(start[0] - goal[0]):
        column = (start[1] + goal[1]) // 2
        for row in range(0, height, spacing):
            result.add((row, column))
        result.add((height - 1, column))
    else:
        row = (start[0] + goal[0]) // 2
        for column in range(0, width, spacing):
            result.add((row, column))
        result.add((row, width - 1))
    return result


def _clone_r0_controller(template: Layered3DV1Controller) -> Layered3DV1Controller:
    result = Layered3DV1Controller.__new__(Layered3DV1Controller)
    result.dynamic_inflation_radius_cells = template.dynamic_inflation_radius_cells
    result.dstar_wall_budget_ms = template.dstar_wall_budget_ms
    result.dstar_max_expansions = template.dstar_max_expansions
    result.dstar_attempt_max_changed_cells = template.dstar_attempt_max_changed_cells
    result.verify_l2_oracle = False
    result.confirmation = DynamicGridConfirmation(
        map_version=template.plan.map_hash,
        map_shape=template.plan.corridor_mask.shape,
        inflation_radius_cells=template.dynamic_inflation_radius_cells,
        confidence_threshold=template.confirmation.confidence_threshold,
    )
    result.plan = template.plan
    result.scheduler = RelevanceScheduler(template.plan.corridor_mask)
    result.l2 = copy.deepcopy(template.l2)
    result.l1_rebind_count = 0
    result.server_l3_mask = np.asarray(template.plan.corridor_mask, dtype=bool).copy()
    result.pending_l3_mask = result.server_l3_mask.copy()
    result._pending_l3_hash = ""
    result.initial_l2_result = template.initial_l2_result
    return result


def _oracle_for_update(roi: CorridorROI, blocked_global: Sequence[Cell]) -> GridAStarResult:
    free = roi.base_free.copy()
    for raw in blocked_global:
        cell = (int(raw[0]), int(raw[1]))
        if roi.contains_global(cell):
            local = roi.to_local(cell)
            if roi.base_free[local]:
                free[local] = False
    return deterministic_grid_astar(free, roi.start_local, roi.goal_local)


def _local_path(roi: CorridorROI, path: Optional[Sequence[Cell]]) -> Optional[List[Cell]]:
    return None if path is None else [roi.to_local(cell) for cell in path]


def _run_event(
    *,
    query_id: str,
    repetition: int,
    snapshot_index: int,
    category: str,
    occupied: Set[Cell],
    map_hash: str,
    map_shape: Sequence[int],
    r0: Layered3DV1Controller,
    r1: Layered3DV1R1Controller,
    rows: List[Dict[str, Any]],
) -> Tuple[Any, Any]:
    snapshot = _snapshot(
        snapshot_index, sorted(occupied), map_hash=map_hash, shape=map_shape,
    )
    r0_step = r0.process_snapshot(snapshot, now=float(snapshot_index))
    r1_step = r1.process_snapshot(snapshot, now=float(snapshot_index))
    scheduler_parity = bool(
        r0_step.scheduler.invoke_l2 == r1_step.scheduler.invoke_l2
        and r0_step.scheduler.reason == r1_step.scheduler.reason
    )
    if r0_step.snapshot_update.blocked_cells != r1_step.snapshot_update.blocked_cells:
        raise AssertionError("r0/r1 dynamic confirmation policy diverged")
    if not scheduler_parity:
        raise AssertionError(
            "r0/r1 scheduler diverged for shared workload: "
            f"r0={r0_step.scheduler.reason}/{r0_step.scheduler.invoke_l2}, "
            f"r1={r1_step.scheduler.reason}/{r1_step.scheduler.invoke_l2}, "
            f"query={query_id}, repetition={repetition}, snapshot={snapshot_index}"
        )

    base = {
        "query_id": query_id,
        "repetition": repetition,
        "snapshot_index": snapshot_index,
        "category": category,
        "snapshot_id": snapshot.snapshot_id,
        "snapshot_hash": snapshot.snapshot_hash,
        "occupied_source_count": len(occupied),
        "scheduler_reason": r0_step.scheduler.reason,
        "scheduler_invoke_l2": r0_step.scheduler.invoke_l2,
        "scheduler_parity": scheduler_parity,
        "r0_scheduler_reason": r0_step.scheduler.reason,
        "r1_scheduler_reason": r1_step.scheduler.reason,
        "effective_changed_cells": len(r0_step.snapshot_update.effective_changed_cells),
        "newly_blocked_sources": len(r0_step.snapshot_update.newly_blocked_sources),
        "newly_freed_sources": len(r0_step.snapshot_update.newly_freed_sources),
    }
    if not r0_step.scheduler.invoke_l2:
        for arm in ("A_cold_grid_astar", "B_r0_selective", "C_r1_optimized"):
            rows.append({
                **base, "arm": arm, "backend": "scheduler_skip",
                "response_ms": 0.0, "expanded": 0, "heap_pops": 0,
                "update_vertex": 0, "predecessor_visits": 0,
                "reachable": True, "cost": 0.0, "cost_error_raw": 0.0,
                "cost_error": 0.0, "path_cell_parity": True,
                "blocked_or_recovering_in_path": 0, "partial_dstar": False,
                "hidden_reinitialize": 0, "all_correct": True,
                "resident_bytes": r1.lifecycle.resident_bytes if arm.startswith("C_") else 0,
                "rss_bytes": _rss_bytes(),
            })
        return r0_step, r1_step

    oracle = _oracle_for_update(r0.l2.roi, r0_step.snapshot_update.blocked_cells)
    oracle_path = oracle.path
    rows.append({
        **base, "arm": "A_cold_grid_astar", "backend": "deterministic_grid_astar",
        "response_ms": oracle.search_time_ms, "expanded": oracle.expanded_nodes,
        "heap_pops": 0, "update_vertex": 0, "predecessor_visits": 0,
        "reachable": oracle_path is not None, "cost": oracle.cost,
        "cost_error_raw": 0.0, "cost_error": 0.0, "path_cell_parity": True,
        "blocked_or_recovering_in_path": 0, "partial_dstar": False,
        "hidden_reinitialize": 0, "all_correct": True,
        "resident_bytes": 0, "rss_bytes": _rss_bytes(),
    })
    blocked_local = {
        r0.l2.roi.to_local(cell)
        for cell in r0_step.snapshot_update.blocked_cells
        if r0.l2.roi.contains_global(cell)
    }
    for arm, step in (("B_r0_selective", r0_step), ("C_r1_optimized", r1_step)):
        result = step.l2_result
        if result is None:
            raise AssertionError("invoked L2 produced no result")
        local_path = _local_path(r0.l2.roi, result.path)
        reachable_parity = (local_path is None) == (oracle_path is None)
        raw_error = 0.0 if oracle_path is None and local_path is None else (
            INF if oracle_path is None or local_path is None
            else abs(_path_cost(local_path) - oracle.cost)
        )
        canonical_error = 0.0 if raw_error <= 1.0e-9 else raw_error
        blocked_count = len(set(local_path or ()).intersection(blocked_local))
        correct = bool(
            reachable_parity and canonical_error == 0.0 and blocked_count == 0
            and not result.partial_dstar_result_returned
        )
        rows.append({
            **base, "arm": arm, "backend": result.selected_backend,
            "response_ms": result.response_ms,
            "search_ms": result.dstar_stats.search_time_ms,
            "fallback_ms": 0.0 if result.fallback_stats is None else result.fallback_stats.search_time_ms,
            "expanded": result.dstar_stats.expanded_nodes,
            "heap_pops": result.dstar_stats.queue_pops,
            "update_vertex": result.dstar_stats.update_vertex_count,
            "predecessor_visits": result.diagnostics.get("predecessor_visits_total", 0),
            "reachable": result.path is not None, "cost": _path_cost(local_path),
            "cost_error_raw": raw_error, "cost_error": canonical_error,
            "path_cell_parity": local_path == oracle_path,
            "path_hash": _path_hash(local_path),
            "blocked_or_recovering_in_path": blocked_count,
            "partial_dstar": result.partial_dstar_result_returned,
            "hidden_reinitialize": int(result.diagnostics.get("reinitialize_count", 0)),
            "all_correct": correct,
            "resident_bytes": (
                r1.lifecycle.resident_bytes if arm.startswith("C_")
                else int(result.diagnostics.get("state_memory_bytes", 0))
            ),
            "rss_bytes": _rss_bytes(),
        })
    return r0_step, r1_step


def _source_snapshot(output: Path) -> Dict[str, str]:
    package_root = Path(__file__).resolve().parents[1]
    destination_root = output / "source_snapshot"
    destination_root.mkdir()
    sources = sorted(
        list((package_root / "arena_3d_v1").glob("*.py"))
        + list((package_root / "config").glob("*.yaml"))
        + list((package_root / "test").glob("test_*.py"))
        + [package_root / "package.xml", package_root / "setup.py", package_root / "setup.cfg"]
    )
    result: Dict[str, str] = {}
    for source in sources:
        destination = destination_root / source.relative_to(package_root)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        result[str(source)] = _sha256(source)
    return result


def _classify_diagnostics(
    l1: DeterministicGraphAStarL1,
    queries: Sequence[Any],
) -> List[Dict[str, Any]]:
    result: List[Dict[str, Any]] = []
    for query_id in CLASSIFICATION_QUERIES:
        query = next(item for item in queries if item.query_id == query_id)
        started = time.monotonic_ns()
        plan = l1.plan(query)
        l1_ms = (time.monotonic_ns() - started) / 1.0e6
        result.append({
            "query_id": query_id,
            "excluded_from_success_performance_aggregate": True,
            "known_classification": (
                "FULL_MAP_ALL_VARIANTS_FAILED_INVESTIGATE_MAP_OR_SMAC"
                if query_id == "A2B-16" else "KNOWN_L3_SEARCH_LONG_TAIL"
            ),
            "l1_route_available": plan is not None,
            "l1_ms": l1_ms,
            "route_edge_count": 0 if plan is None else len(plan.route_edge_ids),
            "corridor_cells": 0 if plan is None else int(np.count_nonzero(plan.corridor_mask)),
        })
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
    if mode == "heldout" and repetitions < 10:
        raise ValueError("heldout requires at least 10 paired repetitions")
    if mode == "heldout" and len(query_ids) < 12:
        raise ValueError("heldout requires at least 12 queries")
    frozen_config: Optional[Mapping[str, Any]] = None
    if mode == "heldout":
        frozen_config = _load_frozen_config(frozen_config_path)
        frozen_workload = frozen_config["workload"]
        frozen_policy = frozen_config["policy"]
        if tuple(query_ids) != tuple(frozen_workload["heldout_queries"]):
            raise ValueError("heldout query set/order differs from frozen config")
        if repetitions != int(frozen_workload["paired_repetitions_per_heldout_query"]):
            raise ValueError("heldout repetitions differ from frozen config")
        if float(dstar_budget_ms) != float(frozen_policy["dstar_wall_budget_ms"]):
            raise ValueError("heldout D* budget differs from frozen config")
        if int(max_active_states) != int(frozen_policy["max_active_states_default"]):
            raise ValueError("heldout LRU size differs from frozen config")
    output.mkdir(parents=True)
    frozen_before = {str(path): _tree_hash(path) for path in FROZEN_BASELINES}
    rows: List[Dict[str, Any]] = []
    initialization_rows: List[Dict[str, Any]] = []
    query_rows: List[Dict[str, Any]] = []
    cache_root = output / "verified_lifecycle_cache"
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
    lifecycle = L2StateLifecycleManager(
        cache_root,
        max_active_states=max_active_states,
        dstar_wall_budget_ms=dstar_budget_ms,
        dstar_max_expansions=20_000,
    )
    experiment_started = time.monotonic_ns()
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

        r0_started = time.monotonic_ns()
        r0_template = Layered3DV1Controller(
            plan,
            dynamic_inflation_radius_cells=7,
            dstar_wall_budget_ms=dstar_budget_ms,
            dstar_max_expansions=20_000,
            dstar_attempt_max_changed_cells=2,
        )
        r0_build_ms = (time.monotonic_ns() - r0_started) / 1.0e6
        if not r0_template.initial_l2_result.success:
            raise RuntimeError(f"r0 initial L2 path unavailable for {query_id}")

        lifecycle.clear()
        r1_started = time.monotonic_ns()
        r1_prebuild = Layered3DV1R1Controller(
            plan,
            cache_root=cache_root,
            max_active_states=max_active_states,
            dynamic_inflation_radius_cells=7,
            dstar_wall_budget_ms=dstar_budget_ms,
            dstar_max_expansions=20_000,
            dstar_attempt_max_changed_cells=2,
            lifecycle_manager=lifecycle,
        )
        r1_build_ms = (time.monotonic_ns() - r1_started) / 1.0e6
        if not r1_prebuild.initial_l2_result.success:
            raise RuntimeError(f"r1 initial L2 path unavailable for {query_id}")
        r1_cold_activation = lifecycle.last_activation
        lifecycle.save_active()
        lifecycle.clear()
        del r1_prebuild
        gc.collect()

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
            "r0_cold_build_ms": r0_build_ms,
            "r0_state_bytes": r0_template.l2.state_memory_bytes(),
            "r1_cold_build_and_serialize_ms": r1_build_ms,
            "r1_state_bytes": r1_cold_activation.resident_bytes if r1_cold_activation else 0,
            "r1_geometry_cache_bytes": 0 if r1_cold_activation is None else r1_cold_activation.geometry_cache.bytes_on_disk,
            "r1_state_cache_bytes": 0 if r1_cold_activation is None else r1_cold_activation.state_cache.bytes_on_disk,
        })
        print(
            f"PREBUILT {query_id} r0_ms={r0_build_ms:.1f} r1_ms={r1_build_ms:.1f}",
            flush=True,
        )

        for repetition in range(1, repetitions + 1):
            clone_started = time.monotonic_ns()
            r0 = _clone_r0_controller(r0_template)
            r0_activate_ms = (time.monotonic_ns() - clone_started) / 1.0e6
            lifecycle.clear()
            r1_activate_started = time.monotonic_ns()
            r1 = Layered3DV1R1Controller(
                plan,
                cache_root=cache_root,
                max_active_states=max_active_states,
                dynamic_inflation_radius_cells=7,
                dstar_wall_budget_ms=dstar_budget_ms,
                dstar_max_expansions=20_000,
                dstar_attempt_max_changed_cells=2,
                lifecycle_manager=lifecycle,
            )
            r1_activate_ms = (time.monotonic_ns() - r1_activate_started) / 1.0e6
            activation = lifecycle.last_activation
            initialization_rows.append({
                "query_id": query_id,
                "repetition": repetition,
                "r0_template_clone_ms": r0_activate_ms,
                "r1_warm_activate_ms": r1_activate_ms,
                "r1_geometry_cache_hit": activation.geometry_cache.hit,
                "r1_geometry_restore_ms": activation.geometry_cache.wall_ms,
                "r1_state_cache_hit": activation.state_cache.hit,
                "r1_state_restore_ms": activation.state_cache.wall_ms,
                "r1_active_state_count": activation.active_state_count,
                "r1_resident_bytes": activation.resident_bytes,
                "r1_rss_before_bytes": activation.rss_before_bytes,
                "r1_rss_after_bytes": activation.rss_after_bytes,
            })
            if not r1.initial_l2_result.success:
                raise RuntimeError(f"warm r1 activation failed for {query_id}")

            occupied: Set[Cell] = set()
            initial_path = list(r0.l2.path_global or ())
            initial_r1_path = list(r1.l2.path_global or ())
            one = _select_nonblocking_eligible_sources(
                paths=(initial_path, initial_r1_path), count=1,
                existing_sources=set(), plan=plan,
                map_shape=artifact.free_mask.shape,
            )
            off_corridor = _far_off_corridor_cell(plan)
            off_path = _off_path_corridor_cell(roi, initial_path)
            barrier = _barrier_sources(
                plan.start_cell, plan.goal_cell, artifact.free_mask.shape,
            )
            event_index = 0

            def event(category: str, sources: Set[Cell]) -> Tuple[Any, Any]:
                nonlocal event_index
                event_index += 1
                return _run_event(
                    query_id=query_id,
                    repetition=repetition,
                    snapshot_index=event_index,
                    category=category,
                    occupied=sources,
                    map_hash=ctx.map_sha256,
                    map_shape=artifact.free_mask.shape,
                    r0=r0,
                    r1=r1,
                    rows=rows,
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
            current_path = list(r0.l2.path_global or initial_path)
            current_r1_path = list(r1.l2.path_global or initial_r1_path)
            two = _select_nonblocking_eligible_sources(
                paths=(current_path, current_r1_path), count=2,
                existing_sources=occupied, plan=plan,
                map_shape=artifact.free_mask.shape,
            )
            event("two_cell_pending", occupied | two)
            event("two_cell_eligible", occupied | two)
            occupied |= two
            current_path = list(r0.l2.path_global or current_path)
            current_r1_path = list(r1.l2.path_global or current_r1_path)
            large = _select_nonblocking_eligible_sources(
                paths=(current_path, current_r1_path), count=5,
                existing_sources=occupied, plan=plan,
                map_shape=artifact.free_mask.shape,
            )
            event("large_change_pending", occupied | large)
            event("large_change_fallback", occupied | large)
            occupied |= large
            event("no_route_pending", occupied | barrier)
            no_route_r0, no_route_r1 = event("no_route_confirmed", occupied | barrier)
            if (
                no_route_r0.l2_result is None or no_route_r0.l2_result.success
                or no_route_r1.l2_result is None or no_route_r1.l2_result.success
            ):
                raise AssertionError(
                    f"barrier failed to produce no-route for {query_id}: "
                    f"r0_scheduler={no_route_r0.scheduler.reason}, "
                    f"r0_backend={getattr(no_route_r0.l2_result, 'selected_backend', None)}, "
                    f"r0_success={getattr(no_route_r0.l2_result, 'success', None)}, "
                    f"r1_scheduler={no_route_r1.scheduler.reason}, "
                    f"r1_backend={getattr(no_route_r1.l2_result, 'selected_backend', None)}, "
                    f"r1_success={getattr(no_route_r1.l2_result, 'success', None)}"
                )
            event("recovery_pending", set())
            recovery_r0, recovery_r1 = event("recovery_confirmed", set())
            if (
                recovery_r0.l2_result is None or not recovery_r0.l2_result.success
                or recovery_r1.l2_result is None or not recovery_r1.l2_result.success
            ):
                raise AssertionError(f"recovery failed for {query_id}")

            lifecycle.clear()
            del r0, r1
            gc.collect()
            print(f"RUN {query_id} repetition={repetition}/{repetitions}", flush=True)
        del r0_template
        gc.collect()

    frozen_after = {str(path): _tree_hash(path) for path in FROZEN_BASELINES}
    if frozen_before != frozen_after:
        raise RuntimeError("a frozen r0 baseline changed during r1 Stage A")
    _write_csv(output / "runs.csv", rows)
    with (output / "runs.jsonl").open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, sort_keys=True) + "\n")
    _write_csv(output / "initialization.csv", initialization_rows)
    _write_csv(output / "query_stratification.csv", query_rows)
    _write_csv(output / "classification_diagnostics.csv", classification_rows)

    invoked = [row for row in rows if row["scheduler_invoke_l2"]]
    timing_rows: List[Dict[str, Any]] = []
    for group_name, selected in (
        ("all_invoked", invoked),
        ("eligible", [row for row in invoked if "eligible" in row["category"]]),
        ("fallback", [row for row in invoked if "fallback" in row["category"]]),
        ("no_route", [row for row in invoked if row["category"] == "no_route_confirmed"]),
        ("recovery", [row for row in invoked if row["category"] == "recovery_confirmed"]),
    ):
        for arm in ("A_cold_grid_astar", "B_r0_selective", "C_r1_optimized"):
            values = [float(row["response_ms"]) for row in selected if row["arm"] == arm]
            timing_rows.append({"group": group_name, "arm": arm, **_summary(values)})
    for query_id in query_ids:
        for arm in ("A_cold_grid_astar", "B_r0_selective", "C_r1_optimized"):
            values = [
                float(row["response_ms"]) for row in invoked
                if row["query_id"] == query_id and row["arm"] == arm
            ]
            timing_rows.append({"group": f"query:{query_id}", "arm": arm, **_summary(values)})
    _write_csv(output / "timing_summary.csv", timing_rows)

    def group_summary(group: str, arm: str) -> Mapping[str, Any]:
        return next(row for row in timing_rows if row["group"] == group and row["arm"] == arm)

    baseline = group_summary("all_invoked", "A_cold_grid_astar")
    candidate = group_summary("all_invoked", "C_r1_optimized")
    restores = [float(row["r1_warm_activate_ms"]) for row in initialization_rows]
    resident = [int(row["r1_resident_bytes"]) for row in initialization_rows]
    p50_reduction = 1.0 - candidate["p50"] / baseline["p50"]
    p95_ratio = candidate["p95"] / baseline["p95"]
    p99_ratio = candidate["p99"] / baseline["p99"]
    correctness_rows = [row for row in rows if row["arm"] in {"B_r0_selective", "C_r1_optimized"}]
    gates: Dict[str, Any] = {
        "mode": mode,
        "workload_classification": "realistic_synthetic_workload_on_real_4x_map",
        "reliable_real_dynamic_log_found": False,
        "query_count": len(query_ids),
        "paired_repetitions_per_query": repetitions,
        "correctness_rows": len(correctness_rows),
        "correctness_failures": sum(not row["all_correct"] for row in correctness_rows),
        "oracle_parity_pass": all(row["all_correct"] for row in correctness_rows),
        "max_raw_cost_error": max(float(row["cost_error_raw"]) for row in correctness_rows),
        "canonical_cost_error_max": max(float(row["cost_error"]) for row in correctness_rows),
        "blocked_or_recovering_in_path": sum(int(row["blocked_or_recovering_in_path"]) for row in correctness_rows),
        "partial_dstar_results": sum(bool(row["partial_dstar"]) for row in correctness_rows),
        "hidden_reinitialize_count": sum(int(row["hidden_reinitialize"]) for row in correctness_rows if row["arm"] == "C_r1_optimized"),
        "scheduler_parity_pass": True,
        "path_cell_parity": {
            arm: {
                "matching_rows": sum(
                    bool(row["path_cell_parity"])
                    for row in correctness_rows if row["arm"] == arm
                ),
                "total_rows": sum(1 for row in correctness_rows if row["arm"] == arm),
            }
            for arm in ("B_r0_selective", "C_r1_optimized")
        },
        "recovery_pass": all(
            row["all_correct"] for row in correctness_rows
            if row["category"] == "recovery_confirmed"
        ),
        "r1_all_invoked": dict(candidate),
        "cold_astar_all_invoked": dict(baseline),
        "r1_p50_reduction_vs_astar": p50_reduction,
        "r1_p95_ratio_vs_astar": p95_ratio,
        "r1_p99_ratio_vs_astar": p99_ratio,
        "warm_activation_ms": _summary(restores),
        "resident_bytes": _summary([float(value) for value in resident]),
        "resident_reduction_vs_r0": 1.0 - max(resident) / FROZEN_R0_STATE_BYTES,
        "lru_peak_active_state_count": lifecycle.peak_active_state_count,
        "lru_hard_limit": lifecycle.HARD_MAX_ACTIVE_STATES,
        "cache_hits": sum(bool(row["r1_state_cache_hit"]) for row in initialization_rows),
        "cache_misses": sum(not bool(row["r1_state_cache_hit"]) for row in initialization_rows),
        "frozen_baselines_unchanged": frozen_before == frozen_after,
    }
    frozen_gates = (frozen_config or {}).get("gates", {})
    thresholds = {
        "p50_reduction": float(frozen_gates.get("r1_p50_reduction_vs_cold_astar_min", 0.20)),
        "p95_ratio": float(frozen_gates.get("r1_p95_ratio_vs_cold_astar_max", 1.05)),
        "p99_ratio": float(frozen_gates.get("r1_p99_ratio_vs_cold_astar_max", 1.10)),
        "p95_target": float(frozen_gates.get("target_p95_ratio_vs_cold_astar_max", 1.0)),
        "p99_target": float(frozen_gates.get("target_p99_ratio_vs_cold_astar_max", 1.0)),
        "warm_ms": float(frozen_gates.get("warm_activation_p95_ms_max", 1000.0)),
        "resident_reduction": float(frozen_gates.get("resident_reduction_vs_57974008_min", 0.30)),
        "resident_bytes": int(frozen_gates.get("resident_bytes_target_max", 35_000_000)),
    }
    gates["frozen_thresholds"] = thresholds
    gates.update({
        "p50_gate_pass": p50_reduction >= thresholds["p50_reduction"],
        "p95_gate_pass": p95_ratio <= thresholds["p95_ratio"],
        "p99_gate_pass": p99_ratio <= thresholds["p99_ratio"],
        "p95_target_pass": p95_ratio <= thresholds["p95_target"],
        "p99_target_pass": p99_ratio <= thresholds["p99_target"],
        "warm_activation_gate_pass": gates["warm_activation_ms"]["p95"] <= thresholds["warm_ms"],
        "resident_reduction_gate_pass": gates["resident_reduction_vs_r0"] >= thresholds["resident_reduction"],
        "resident_target_pass": gates["resident_bytes"]["max"] <= thresholds["resident_bytes"],
        "lru_bound_pass": lifecycle.peak_active_state_count <= max_active_states <= 2,
        "cache_hit_pass": gates["cache_hits"] == len(initialization_rows),
    })
    gates["stage_a_pass"] = all(bool(gates[key]) for key in (
        "oracle_parity_pass", "scheduler_parity_pass", "recovery_pass",
        "p50_gate_pass", "p95_gate_pass", "p99_gate_pass",
        "warm_activation_gate_pass", "resident_reduction_gate_pass",
        "resident_target_pass", "lru_bound_pass", "cache_hit_pass",
        "frozen_baselines_unchanged",
    )) and all(int(gates[key]) == 0 for key in (
        "blocked_or_recovering_in_path", "partial_dstar_results",
        "hidden_reinitialize_count",
    ))
    (output / "gate_results.yaml").write_text(
        yaml.safe_dump(gates, sort_keys=False), encoding="utf-8",
    )
    workload_manifest = {
        "classification": "realistic_synthetic_workload",
        "search_scope": [str(ROOT), "/home/robot/文档"],
        "real_cleaning_dynamic_logs_found": False,
        "excluded_files": "planner stdout logs and dependency test bags lack versioned 4x occupancy observations",
        "snapshot_categories": [
            "unconfirmed", "duplicate", "off_corridor", "off_path_cost_increase",
            "one_cell_eligible", "two_cell_eligible", "large_change_fallback",
            "no_route", "recovery",
        ],
        "seed": 0,
        "query_ids": list(query_ids),
        "query_set_hash": _stable_hash(list(query_ids)),
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
    (output / "manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8",
    )
    command_queries = ",".join(query_ids)
    (output / "reproduction_command.txt").write_text(
        "cd /home/robot/pudu_robot_ws\n"
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        "PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/three_d_v1:/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation "
        f"/usr/bin/python3 -m arena_3d_v1.r1_stage_a --mode {mode} --output-dir {output} --query-ids {command_queries} --repetitions {repetitions} --dstar-budget-ms {dstar_budget_ms} --max-active-states {max_active_states} --frozen-config {frozen_config_path.resolve()}\n",
        encoding="utf-8",
    )
    verification = {
        "required_artifacts_present": True,
        "frozen_baselines_unchanged": frozen_before == frozen_after,
        "three_arms_present": sorted({row["arm"] for row in rows}) == [
            "A_cold_grid_astar", "B_r0_selective", "C_r1_optimized",
        ],
        "strict_snapshot_hash_pairing": all(
            len({row["snapshot_hash"] for row in rows if (
                row["query_id"], row["repetition"], row["snapshot_index"]
            ) == key}) == 1
            for key in {
                (row["query_id"], row["repetition"], row["snapshot_index"])
                for row in rows
            }
        ),
        "source_snapshot_count": len(source_hashes),
        "stage_a_gate_pass": gates["stage_a_pass"],
    }
    (output / "verification.yaml").write_text(
        yaml.safe_dump(verification, sort_keys=False), encoding="utf-8",
    )
    report = [
        f"# 3D-V1-r1 {mode} Stage A", "",
        f"- real map: `{MAP_ID}`; workload: **realistic synthetic**, not a measured real-world distribution",
        f"- queries/repetitions: `{len(query_ids)}` / `{repetitions}`",
        f"- correctness failures: `{gates['correctness_failures']}/{gates['correctness_rows']}`",
        "- exact path-cell parity is reported separately; equal-cost deterministic "
        "D* alternatives do not relax the zero-cost-error gate",
        f"- r1 vs cold A* P50 reduction: `{p50_reduction:.2%}`",
        f"- r1/A* P95/P99 ratios: `{p95_ratio:.3f}` / `{p99_ratio:.3f}`",
        f"- warm activation P95: `{gates['warm_activation_ms']['p95']:.3f} ms`",
        f"- max resident bytes: `{gates['resident_bytes']['max']:.0f}`; reduction vs r0: `{gates['resident_reduction_vs_r0']:.2%}`",
        f"- Stage-A gate: **{'PASS' if gates['stage_a_pass'] else 'FAIL'}**", "",
        "A2B-16 and A2B-19 are reported only in `classification_diagnostics.csv` and are excluded from successful-query performance aggregates.",
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
        f"3d_v1_r1_{mode}_{time.strftime('%Y%m%d_%H%M%S')}"
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
            args.output_dir or _default_output(args.mode),
            mode=args.mode,
            query_ids=query_ids,
            repetitions=repetitions,
            dstar_budget_ms=args.dstar_budget_ms,
            max_active_states=args.max_active_states,
            frozen_config_path=args.frozen_config,
        )
    except Exception as exc:
        output = args.output_dir
        if output is not None:
            output.mkdir(parents=True, exist_ok=True)
            (output / "INTERRUPTED_RUN.md").write_text(
                f"# Interrupted run\n\n`{type(exc).__name__}: {exc}`\n",
                encoding="utf-8",
            )
            (output / "stderr.log").write_text(
                f"{type(exc).__name__}: {exc}\n", encoding="utf-8",
            )
        raise


if __name__ == "__main__":
    main()
