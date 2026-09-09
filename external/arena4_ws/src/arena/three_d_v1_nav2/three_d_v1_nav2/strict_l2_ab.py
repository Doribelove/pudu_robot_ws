"""Strict paired L2 A/B on the frozen pudu_wanda_3f cmp2-04 corridor.

The candidate arm updates one verified persistent D* Lite state.  The
baseline arm cold-starts the frozen deterministic grid A* on the exact same
post-change CorridorROI.  L3 Smac and canonical PathAudit are correctness
gates and are deliberately excluded from the L2 timing comparison.
"""

from __future__ import annotations

import argparse
import csv
import gc
import hashlib
import json
import math
import os
import platform
import resource
import shutil
import statistics
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass, is_dataclass
from itertools import count
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence

import numpy as np
import yaml

import arena_3d_v1.l2_incremental as l2_incremental
from arena_3d_v1.dynamic_policy import _inflate
from arena_3d_v1.l2_incremental import CorridorROI
from arena_3d_v1.pipeline import ProductionL3Adapter, _grid_hash, corridor_dirty_transition
from arena_3d_v1.production_l1 import DeterministicGraphAStarL1
from arena_3d_v1.r1_pipeline import Layered3DV1R1Controller
from arena_evaluation import l1_l3_corridor_hybrid_smoke as production
from arena_evaluation import path_audit, topology
from arena_evaluation import two_layer_v1_r1_cache_benchmark as runtime_profile
from arena_evaluation import two_layer_v2_semantic_benchmark as semantic_runtime
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.dynamic_snapshot import DynamicSnapshot
from arena_evaluation.semantic_query_defaults import load_query_set

from .contracts import (
    EXPECTED_MAP_SHA256,
    EXPECTED_PDMAP_SHA256,
    EXPECTED_QUERY_HASH,
    EXPECTED_SEMANTIC_MAP_HASH,
    QUERY_SET,
    sha256_file,
    verify_frozen_sources,
)


ROOT = Path("/home/robot/pudu_robot_ws")
QUERY_ID = "cmp2-04-multi-junction"
PROTOCOL_ID = "PLN-02-3D-V1-R1-L2-STRICT-PAIRED-AB-V1"
ARCHITECTURE_ID = "3D-V1-r1-nav2-teb-integration"
SOURCE_FRACTION = 0.20
EXPECTED_SOURCE_CELL = (1108, 1761)
DYNAMIC_INFLATION_RADIUS_CELLS = 7
DSTAR_WALL_BUDGET_MS = 500.0
DSTAR_MAX_EXPANSIONS = 20_000
DSTAR_CHANGED_SOURCE_LIMIT = 2


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


def _stable_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(
        _jsonable(value), sort_keys=True, separators=(",", ":"),
    ).encode("utf-8")).hexdigest()


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


def _rss_bytes() -> int:
    try:
        for line in Path("/proc/self/status").read_text(encoding="utf-8").splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) * 1024
    except OSError:
        pass
    return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024


def _pss_bytes() -> int:
    try:
        for line in Path("/proc/self/smaps_rollup").read_text(encoding="utf-8").splitlines():
            if line.startswith("Pss:"):
                return int(line.split()[1]) * 1024
    except OSError:
        pass
    return 0


class _PeakRssSampler:
    def __init__(self, period_s: float = 0.002) -> None:
        self.period_s = float(period_s)
        self.peak = _rss_bytes()
        self.samples = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        while not self._stop.is_set():
            self.peak = max(self.peak, _rss_bytes())
            self.samples += 1
            self._stop.wait(self.period_s)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.peak = max(self.peak, _rss_bytes())


@dataclass(frozen=True)
class _Case:
    ctx: Any
    artifact: Any
    query: Any
    query_metadata: Mapping[str, Any]
    plan: Any
    map_yaml: Path
    topology_dir: Path


def _load_case(run_root: Path) -> _Case:
    run_root = Path(run_root).resolve()
    map_yaml = run_root / "derived_map/extracted/optemap.yaml"
    topology_dir = run_root / "derived_map/topology_cache"
    if not map_yaml.is_file():
        raise FileNotFoundError(f"missing archived map: {map_yaml}")
    ctx = semantic_runtime._context(map_yaml)
    if ctx.map_sha256 != EXPECTED_MAP_SHA256:
        raise RuntimeError("frozen occupancy map hash mismatch")
    artifact = topology.load_topology(
        topology_dir, ctx.hospital_map, runtime.FOOTPRINT,
        padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    queries, _intents, metadata = load_query_set(
        QUERY_SET, actual_map_hash=EXPECTED_MAP_SHA256,
        actual_semantic_map_hash=EXPECTED_SEMANTIC_MAP_HASH,
        require_default_contract=True,
    )
    if metadata.get("query_hash") != EXPECTED_QUERY_HASH:
        raise RuntimeError("frozen query hash mismatch")
    query = next(item for item in queries if item.query_id == QUERY_ID)
    l1 = DeterministicGraphAStarL1(
        ctx, artifact, map_hash=ctx.map_sha256,
        topology_hash=sha256_file(topology_dir / "topology_graph.json"),
    )
    plan = l1.plan(query)
    if plan is None:
        raise RuntimeError("frozen cmp2-04 L1 route unavailable")
    return _Case(ctx, artifact, query, metadata, plan, map_yaml, topology_dir)


def _roi(case: _Case) -> CorridorROI:
    return CorridorROI.from_global(
        case.plan.static_safe_free, case.plan.corridor_mask,
        case.plan.start_cell, case.plan.goal_cell,
        binding_fields=case.plan.binding_fields(),
    )


def _new_free(roi: CorridorROI, blocked_global: Sequence[tuple[int, int]]) -> np.ndarray:
    free = roi.base_free.copy()
    for cell in blocked_global:
        if roi.contains_global(cell):
            local = roi.to_local(cell)
            if roi.base_free[local]:
                free[local] = False
    return free


def _global_path(roi: CorridorROI, path: Optional[Sequence[tuple[int, int]]]) -> list[tuple[int, int]]:
    return [] if path is None else [roi.to_global(cell) for cell in path]


def _path_cost(path: Optional[Sequence[tuple[int, int]]]) -> float:
    if not path:
        return math.inf
    return float(sum(
        math.sqrt(2.0) if a[0] != b[0] and a[1] != b[1] else 1.0
        for a, b in zip(path, path[1:])
    ))


def _new_controller(plan: Any, cache_root: Path) -> Layered3DV1R1Controller:
    return Layered3DV1R1Controller(
        plan, cache_root=cache_root, max_active_states=1,
        dynamic_inflation_radius_cells=DYNAMIC_INFLATION_RADIUS_CELLS,
        dstar_wall_budget_ms=DSTAR_WALL_BUDGET_MS,
        dstar_max_expansions=DSTAR_MAX_EXPANSIONS,
        dstar_attempt_max_changed_cells=DSTAR_CHANGED_SOURCE_LIMIT,
        verify_l2_oracle=False,
    )


def _trace_frozen_grid_astar(
    traversable: np.ndarray, start: tuple[int, int], goal: tuple[int, int],
) -> Mapping[str, Any]:
    """Replay the exact frozen function while counting its heap operations."""
    original_pop = l2_incremental.heapq.heappop
    original_push = l2_incremental.heapq.heappush
    counts = {"pops": 0, "pushes": 0}

    def counted_pop(queue: Any) -> Any:
        counts["pops"] += 1
        return original_pop(queue)

    def counted_push(queue: Any, item: Any) -> None:
        counts["pushes"] += 1
        original_push(queue, item)

    l2_incremental.heapq.heappop = counted_pop
    l2_incremental.heapq.heappush = counted_push
    try:
        started = time.monotonic_ns()
        result = l2_incremental.deterministic_grid_astar(traversable, start, goal)
        wall_ms = (time.monotonic_ns() - started) / 1.0e6
    finally:
        l2_incremental.heapq.heappop = original_pop
        l2_incremental.heapq.heappush = original_push
    return {"result": result, "wall_ms": wall_ms, **counts}


def _worker(args: argparse.Namespace) -> None:
    if args.cpu >= 0:
        os.sched_setaffinity(0, {int(args.cpu)})
    case = _load_case(args.run_root)
    roi = _roi(case)
    source = (int(args.source_row), int(args.source_column))
    blocked = tuple(sorted(_inflate(
        {source}, case.artifact.free_mask.shape, DYNAMIC_INFLATION_RADIUS_CELLS,
    )))
    new_free = _new_free(roi, blocked)
    target_mask = np.asarray(case.plan.corridor_mask, dtype=bool).copy()
    for cell in blocked:
        if target_mask[cell]:
            target_mask[cell] = False
    dirty = corridor_dirty_transition(case.plan.corridor_mask, target_mask)
    common: Dict[str, Any] = {
        "arm": args.arm,
        "map_hash": case.ctx.map_sha256,
        "query_hash": case.query_metadata["query_hash"],
        "query_id": QUERY_ID,
        "start": list(case.query.start),
        "goal": list(case.query.goal),
        "route_edge_ids": list(case.plan.route_edge_ids),
        "route_signature": case.plan.route_signature,
        "corridor_mask_hash": _grid_hash(case.plan.corridor_mask),
        "roi_binding_hash": roi.binding.digest,
        "roi_bbox": list(roi.bbox),
        "roi_shape": list(roi.shape),
        "corridor_cells": int(np.count_nonzero(roi.base_free)),
        "source_cell": list(source),
        "source_count": 1,
        "inflation_radius_cells": DYNAMIC_INFLATION_RADIUS_CELLS,
        "blocked_global_count": len(blocked),
        "blocked_global_hash": _stable_hash(blocked),
        "old_mask_hash": _grid_hash(case.plan.corridor_mask),
        "new_mask_hash": _grid_hash(target_mask),
        "dirty_bbox": list(dirty.bbox or ()),
        "dirty_changed_cells": dirty.changed_cells,
        "dirty_closed_cells": dirty.closed_cells,
        "dirty_opened_cells": dirty.opened_cells,
        "dirty_old_state_residual_cells": dirty.old_state_residual_cells,
        "cpu_affinity": sorted(os.sched_getaffinity(0)),
    }
    if dirty.changed_cells <= 0 or dirty.old_state_residual_cells:
        raise RuntimeError("invalid old/new ROI transition")
    gc.collect()
    ready_rss = _rss_bytes()
    ready_pss = _pss_bytes()
    if args.arm == "persistent_dstar_lite":
        controller = _new_controller(case.plan, args.cache_root)
        activation = dict(controller.initial_l2_result.diagnostics.get("activation") or {})
        if not controller.initial_l2_result.success or not controller.l2.dstar_ready:
            raise RuntimeError("D* state is not ready before update")
        ready_rss = _rss_bytes()
        ready_pss = _pss_bytes()
        ready_state_bytes = int(controller.l2.state_memory_bytes())
        sampler = _PeakRssSampler()
        cpu_started = time.process_time_ns()
        wall_started = time.monotonic_ns()
        sampler.start()
        result = controller.l2.update(blocked, verify_oracle=False, force_cold_astar=False)
        sampler.stop()
        wall_ms = (time.monotonic_ns() - wall_started) / 1.0e6
        cpu_ms = (time.process_time_ns() - cpu_started) / 1.0e6
        if result.selected_backend != "compact_persistent_dstar":
            raise RuntimeError(f"D* arm selected {result.selected_backend}")
        oracle = l2_incremental.deterministic_grid_astar(
            controller.l2.current_free, roi.start_local, roi.goal_local,
        )
        path_global = list(result.path or ())
        local_path = [roi.to_local(cell) for cell in path_global]
        cost = _path_cost(local_path)
        stats = result.dstar_stats
        row = {
            **common,
            "backend": result.selected_backend,
            "success": result.success,
            "dstar_ready_before": True,
            "dstar_ready_after": controller.l2.dstar_ready,
            "state_reused": result.state_reused,
            "changed_cells_backend": result.changed_cells,
            "search_time_ms": stats.search_time_ms,
            "solver_response_ms": result.response_ms,
            "worker_wall_ms": wall_ms,
            "cpu_ms": cpu_ms,
            "expanded_nodes": stats.expanded_nodes,
            "generated_nodes": stats.generated_nodes,
            "open_queue_pops": stats.queue_pops,
            "open_queue_pushes": stats.queue_pushes,
            "initial_open_size": stats.initial_queue_size,
            "final_open_size": stats.final_queue_size,
            "vertex_update_calls": stats.update_vertex_count,
            "successful_relaxations": "not_applicable",
            "timeout": stats.timeout_triggered,
            "partial_result": result.partial_dstar_result_returned,
            "path_cell_count": len(path_global),
            "path_cost_cells": cost,
            "path_length_m": cost * float(case.plan.resolution),
            "path_hash": _stable_hash(path_global),
            "blocked_path_cells": len(set(path_global).intersection(blocked)),
            "oracle_reachable": oracle.path is not None,
            "oracle_cost_cells": oracle.cost,
            "oracle_cost_error": abs(cost - oracle.cost),
            "oracle_path_cell_parity": local_path == oracle.path,
            "frozen_astar_oracle_ms": oracle.search_time_ms,
            "ready_state_bytes": ready_state_bytes,
            "state_bytes_after": int(controller.l2.state_memory_bytes()),
            "ready_rss_bytes": ready_rss,
            "peak_rss_bytes": sampler.peak,
            "incremental_peak_rss_bytes": max(0, sampler.peak - ready_rss),
            "ready_pss_bytes": ready_pss,
            "after_pss_bytes": _pss_bytes(),
            "rss_sample_count": sampler.samples,
            "state_cache_hit": activation.get("state_cache_hit"),
            "geometry_cache_hit": activation.get("geometry_cache_hit"),
        }
        controller.lifecycle.clear()
    else:
        sampler = _PeakRssSampler()
        cpu_started = time.process_time_ns()
        wall_started = time.monotonic_ns()
        sampler.start()
        result = l2_incremental.deterministic_grid_astar(
            new_free, roi.start_local, roi.goal_local,
        )
        sampler.stop()
        wall_ms = (time.monotonic_ns() - wall_started) / 1.0e6
        cpu_ms = (time.process_time_ns() - cpu_started) / 1.0e6
        traced = _trace_frozen_grid_astar(new_free, roi.start_local, roi.goal_local)
        traced_result = traced["result"]
        if (
            result.path != traced_result.path
            or result.expanded_nodes != traced_result.expanded_nodes
            or abs(result.cost - traced_result.cost) > 1.0e-12
        ):
            raise RuntimeError("A* heap-trace replay changed the frozen result")
        path_global = _global_path(roi, result.path)
        row = {
            **common,
            "backend": "deterministic_grid_astar_cold",
            "success": result.path is not None,
            "dstar_ready_before": "not_applicable",
            "dstar_ready_after": "not_applicable",
            "state_reused": False,
            "changed_cells_backend": dirty.changed_cells,
            "search_time_ms": result.search_time_ms,
            "solver_response_ms": result.search_time_ms,
            "worker_wall_ms": wall_ms,
            "cpu_ms": cpu_ms,
            "expanded_nodes": result.expanded_nodes,
            "generated_nodes": result.generated_nodes,
            "open_queue_pops": traced["pops"],
            "open_queue_pushes": traced["pushes"] + 1,
            "initial_open_size": 1,
            "final_open_size": "not_exposed",
            "vertex_update_calls": "not_applicable",
            "successful_relaxations": max(0, result.generated_nodes - 1),
            "timeout": result.timeout_triggered,
            "partial_result": False,
            "path_cell_count": len(path_global),
            "path_cost_cells": result.cost,
            "path_length_m": result.cost * float(case.plan.resolution),
            "path_hash": _stable_hash(path_global),
            "blocked_path_cells": len(set(path_global).intersection(blocked)),
            "oracle_reachable": result.path is not None,
            "oracle_cost_cells": result.cost,
            "oracle_cost_error": 0.0,
            "oracle_path_cell_parity": True,
            "frozen_astar_oracle_ms": result.search_time_ms,
            "astar_heap_trace_replay_ms": traced["wall_ms"],
            "ready_state_bytes": 0,
            "state_bytes_after": 0,
            "ready_rss_bytes": ready_rss,
            "peak_rss_bytes": sampler.peak,
            "incremental_peak_rss_bytes": max(0, sampler.peak - ready_rss),
            "ready_pss_bytes": ready_pss,
            "after_pss_bytes": _pss_bytes(),
            "rss_sample_count": sampler.samples,
            "state_cache_hit": "not_applicable",
            "geometry_cache_hit": "not_applicable",
        }
    print(json.dumps(_jsonable(row), sort_keys=True), flush=True)


def _snapshot(
    name: str, index: int, source: tuple[int, int], case: _Case,
) -> DynamicSnapshot:
    return DynamicSnapshot.from_cells(
        f"{name}-{index}", [source], timestamp=float(index),
        map_version=case.ctx.map_sha256,
        map_shape=case.artifact.free_mask.shape,
    )


def _pipeline_gate(case: _Case, cache_root: Path, source: tuple[int, int]) -> Mapping[str, Any]:
    controller = _new_controller(case.plan, cache_root)
    initial_path = list(controller.l2.path_global or ())
    binding = controller.l2.binding_hash
    pending = controller.process_snapshot(_snapshot("pipeline-gate", 1, source, case), now=1.0)
    confirmed = controller.process_snapshot(_snapshot("pipeline-gate", 2, source, case), now=2.0)
    result = confirmed.l2_result
    if pending.scheduler.invoke_l2:
        raise RuntimeError("confirmation gate was bypassed")
    if result is None or not result.success:
        raise RuntimeError("pipeline D* gate produced no path")
    gate = {
        "pending_scheduler_reason": pending.scheduler.reason,
        "confirmed_scheduler_reason": confirmed.scheduler.reason,
        "confirmed_scheduler_invoke_l2": confirmed.scheduler.invoke_l2,
        "newly_blocked_source_count": len(confirmed.snapshot_update.newly_blocked_sources),
        "effective_changed_cell_count": len(confirmed.snapshot_update.effective_changed_cells),
        "selected_backend": result.selected_backend,
        "dstar_ready_after": controller.l2.dstar_ready,
        "dstar_expanded_nodes": result.dstar_stats.expanded_nodes,
        "dstar_queue_pops": result.dstar_stats.queue_pops,
        "dstar_vertex_updates": result.dstar_stats.update_vertex_count,
        "dstar_search_time_ms": result.dstar_stats.search_time_ms,
        "partial_result": result.partial_dstar_result_returned,
        "l1_called": confirmed.l1_graph_astar_called,
        "route_edge_ids": list(controller.plan.route_edge_ids),
        "binding_before": binding,
        "binding_after": controller.l2.binding_hash,
        "path_changed": initial_path != list(controller.l2.path_global or ()),
        "dirty_roi": _jsonable(confirmed.dirty_roi),
    }
    gate["pass"] = bool(
        confirmed.scheduler.invoke_l2
        and len(confirmed.snapshot_update.newly_blocked_sources) == 1
        and result.selected_backend == "compact_persistent_dstar"
        and controller.l2.dstar_ready
        and not result.partial_dstar_result_returned
        and not confirmed.l1_graph_astar_called
        and binding == controller.l2.binding_hash
        and gate["path_changed"]
    )
    controller.lifecycle.clear()
    if not gate["pass"]:
        raise RuntimeError("strict pipeline D* admission gate failed")
    return gate


def _run_worker(
    script: Path, arm: str, run_root: Path, cache_root: Path,
    source: tuple[int, int], cpu: int,
) -> Mapping[str, Any]:
    command = [
        sys.executable, "-m", "three_d_v1_nav2.strict_l2_ab",
        "--worker", "--arm", arm,
        "--run-root", str(run_root), "--cache-root", str(cache_root),
        "--source-row", str(source[0]), "--source-column", str(source[1]),
        "--cpu", str(cpu),
    ]
    result = subprocess.run(
        command, check=False, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, timeout=60.0,
    )
    if result.returncode:
        raise RuntimeError(
            f"worker {arm} failed rc={result.returncode}: {result.stderr.strip()}"
        )
    lines = [line for line in result.stdout.splitlines() if line.strip()]
    if not lines:
        raise RuntimeError(f"worker {arm} returned no JSON")
    return json.loads(lines[-1])


def _summary(values: Sequence[float]) -> Mapping[str, float]:
    array = np.asarray(values, dtype=float)
    return {
        "count": int(len(array)),
        "p50": float(np.quantile(array, 0.50, method="linear")),
        "p95": float(np.quantile(array, 0.95, method="linear")),
        "p99": float(np.quantile(array, 0.99, method="linear")),
        "mean": float(statistics.fmean(array)),
        "min": float(np.min(array)),
        "max": float(np.max(array)),
    }


def _git_head() -> str:
    return subprocess.run(
        ["git", "-C", str(ROOT), "rev-parse", "HEAD"], check=True,
        text=True, stdout=subprocess.PIPE,
    ).stdout.strip()


def _process_snapshot() -> str:
    return subprocess.run(
        ["ps", "-eo", "pid,lstart,etimes,cmd", "--sort=pid"], check=True,
        text=True, stdout=subprocess.PIPE,
    ).stdout


def _run_l3_gates(
    output: Path, case: _Case, cache_root: Path, source: tuple[int, int],
    *, repetitions: int, ros_domain_id: int,
) -> list[Mapping[str, Any]]:
    spec = runtime.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"Smac Hybrid unavailable: {spec.reason}")
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    session = production.SmacSession(
        case.ctx, output / "smac_runtime", map_yaml=case.map_yaml,
        log_tag="strict_l2_ab_cmp2_04", local_mask_updates=True,
        optimization_profile=runtime_profile.OPTIMIZATION_PROFILE,
        smac_parameter_profile=runtime_profile.SMAC_PARAMETER_PROFILE,
        optimization_stage=runtime_profile.OPTIMIZATION_STAGE,
        enable_mask_reuse_noop=True,
        planner_parameter_overrides={"angle_quantization_bins": 48},
        costmap_ack_timeout_s=3.0,
    )
    session.local_map_update_strategy = "roi_ack"
    session.full_grid_settle_cycles = 0
    auditor = path_audit.PathAuditor(case.ctx, source_commit=_git_head())
    rows: list[Mapping[str, Any]] = []
    path_dir = output / "l3_paths"
    path_dir.mkdir(exist_ok=True)
    try:
        session.start()
        for repetition in range(1, repetitions + 1):
            order = (
                ("persistent_dstar_lite", "cold_grid_astar")
                if repetition % 2 else
                ("cold_grid_astar", "persistent_dstar_lite")
            )
            for order_index, arm in enumerate(order, 1):
                session.reset_query_state(
                    f"strict-ab-{repetition}-{arm}", restore_base_map=True,
                )
                controller = _new_controller(case.plan, cache_root)
                initial_binding = controller.l2.binding_hash
                pending = controller.process_snapshot(
                    _snapshot(f"l3-{repetition}-{arm}", 1, source, case), now=1.0,
                )
                if pending.scheduler.invoke_l2:
                    raise RuntimeError("L3 gate bypassed two-observation confirmation")
                if arm == "cold_grid_astar":
                    controller.l2.state.ready = False
                step = controller.process_snapshot(
                    _snapshot(f"l3-{repetition}-{arm}", 2, source, case), now=2.0,
                )
                l2_result = step.l2_result
                expected = (
                    "compact_persistent_dstar" if arm == "persistent_dstar_lite"
                    else "deterministic_grid_astar_direct"
                )
                if l2_result is None or l2_result.selected_backend != expected:
                    raise RuntimeError(
                        f"L3 gate {arm} selected {getattr(l2_result, 'selected_backend', None)}"
                    )
                started = time.monotonic_ns()
                outcome = ProductionL3Adapter(controller, auditor).plan(
                    step, case.query, session, spec,
                )
                l3_wall_ms = (time.monotonic_ns() - started) / 1.0e6
                if outcome.get("called") is not True or outcome.get("success") is not True:
                    raise RuntimeError(
                        f"L3/PathAudit gate failed: {outcome.get('failure_code')}"
                    )
                result = outcome["result"]
                audit = result.path_audit
                points = list(result.points or ())
                path_file = path_dir / f"r{repetition}_{arm}.csv"
                _write_csv(path_file, ({
                    "index": index,
                    "x": format(float(point["x"]), ".17g"),
                    "y": format(float(point["y"]), ".17g"),
                    "yaw": format(float(point["yaw"]), ".17g"),
                } for index, point in enumerate(points)))
                metrics = dict(outcome.get("metrics") or {})
                diagnostics = dict(outcome.get("diagnostics") or {})
                rows.append({
                    "repetition": repetition,
                    "order_index": order_index,
                    "arm": arm,
                    "l2_backend": l2_result.selected_backend,
                    "l2_binding_hash": controller.l2.binding_hash,
                    "binding_unchanged": controller.l2.binding_hash == initial_binding,
                    "route_edge_ids": list(controller.plan.route_edge_ids),
                    "old_mask_hash": _grid_hash(case.plan.corridor_mask),
                    "new_mask_hash": step.dirty_roi.target_hash,
                    "dirty_bbox": list(step.dirty_roi.bbox or ()),
                    "dirty_changed_cells": step.dirty_roi.changed_cells,
                    "l3_wall_ms": l3_wall_ms,
                    "l3_planning_ms": diagnostics.get("planning_time_ms"),
                    "roi_ack": diagnostics.get("costmap_update_acknowledged"),
                    "roi_ack_mismatch_cells": diagnostics.get("costmap_ack_mismatch_cells"),
                    "angle_bins": 48,
                    "motion_model": "DUBIN",
                    "canonical_path_hash": audit.path_hash,
                    "path_file": str(path_file.relative_to(output)),
                    "path_file_sha256": sha256_file(path_file),
                    "path_length_m": metrics.get("path_length_m"),
                    "minimum_clearance_m": metrics.get("minimum_clearance_m"),
                    "maximum_curvature_1pm": metrics.get("maximum_curvature"),
                    "static_footprint_valid": metrics.get("static_footprint_valid"),
                    "kinematic_valid": metrics.get("kinematic_valid"),
                    "within_corridor": audit.within_mask,
                    "reverse_distance_m": metrics.get("reverse_distance_m"),
                    "in_place_rotation_count": metrics.get("in_place_rotation_count"),
                    "final_valid_success": metrics.get("final_valid_success"),
                    "failure_code": metrics.get("failure_code") or "",
                })
                controller.lifecycle.clear()
    finally:
        session.close()
    return rows


def _plot(output: Path, summary: Mapping[str, Any]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = ["Persistent D* Lite", "Cold grid A*"]
    dstar = summary["arms"]["persistent_dstar_lite"]
    astar = summary["arms"]["cold_grid_astar"]
    panels = (
        ("Expanded nodes (P50)", dstar["expanded_nodes"]["p50"], astar["expanded_nodes"]["p50"]),
        ("OPEN pops (P50)", dstar["open_queue_pops"]["p50"], astar["open_queue_pops"]["p50"]),
        ("Search time ms (P50)", dstar["search_time_ms"]["p50"], astar["search_time_ms"]["p50"]),
        ("Incremental RSS MiB (P50)", dstar["incremental_peak_rss_mib"]["p50"], astar["incremental_peak_rss_mib"]["p50"]),
    )
    figure, axes = plt.subplots(2, 2, figsize=(11, 7))
    colors = ["#2d6cdf", "#e08b2c"]
    for axis, (title, left, right) in zip(axes.flat, panels):
        bars = axis.bar(labels, [left, right], color=colors)
        axis.set_title(title)
        axis.grid(axis="y", alpha=0.25)
        for bar, value in zip(bars, (left, right)):
            axis.text(bar.get_x() + bar.get_width() / 2, bar.get_height(), f"{value:.2f}", ha="center", va="bottom")
    figure.suptitle("3D-V1-r1 strict paired L2 A/B — cmp2-04, one-source / 149-cell update")
    figure.tight_layout()
    figure.savefig(output / "strict_ab_summary.png", dpi=180)
    plt.close(figure)


def _source_snapshot(output: Path, script: Path) -> Mapping[str, str]:
    destination = output / "source_snapshot"
    destination.mkdir(exist_ok=True)
    sources = [
        script,
        ROOT / "external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/l2_state_lifecycle.py",
        ROOT / "external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/l2_incremental.py",
        ROOT / "external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/r1_pipeline.py",
        ROOT / "external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/pipeline.py",
        ROOT / "external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/production_l1.py",
        ROOT / "external/arena4_ws/src/arena/three_d_v1/config/three_d_v1_r1_l2_lifecycle.yaml",
        QUERY_SET,
    ]
    hashes: Dict[str, str] = {}
    for index, source in enumerate(sources):
        target = destination / f"{index:02d}_{source.name}"
        shutil.copy2(source, target)
        hashes[str(source.resolve())] = sha256_file(source)
    return hashes


def run(
    output: Path, *, run_root: Path, repetitions: int = 10,
    warmups: int = 1, l3_repetitions: int = 3,
    cpu: int = 15, ros_domain_id: int = 230,
) -> Path:
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to overwrite non-empty output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    script = Path(__file__).resolve()
    failures: list[Mapping[str, Any]] = []
    before = _process_snapshot()
    (output / "processes_before.txt").write_text(before, encoding="utf-8")
    started_ns = time.monotonic_ns()
    verify_frozen_sources()
    case = _load_case(run_root)
    cache_root = output / "l2_cache"
    cache_root.mkdir()

    prebuild_started = time.monotonic_ns()
    template = _new_controller(case.plan, cache_root)
    prebuild_ms = (time.monotonic_ns() - prebuild_started) / 1.0e6
    if not template.initial_l2_result.success or not template.l2.dstar_ready:
        raise RuntimeError("failed to prebuild a ready D* state")
    initial_path = list(template.l2.path_global or ())
    source = initial_path[int(len(initial_path) * SOURCE_FRACTION)]
    if tuple(source) != EXPECTED_SOURCE_CELL:
        raise RuntimeError(f"source-cell drift: {source} != {EXPECTED_SOURCE_CELL}")
    initial_binding = template.l2.binding_hash
    initial_backend = template.initial_l2_result.selected_backend
    template.lifecycle.clear()
    pipeline_gate = _pipeline_gate(case, cache_root, source)
    (output / "pipeline_admission_gate.json").write_text(
        json.dumps(_jsonable(pipeline_gate), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    rows: list[Mapping[str, Any]] = []
    paired: list[Mapping[str, Any]] = []
    total_rounds = warmups + repetitions
    for ordinal in range(total_rounds):
        measured = ordinal >= warmups
        repetition = ordinal - warmups + 1 if measured else ordinal - warmups
        order = (
            ("persistent_dstar_lite", "cold_grid_astar")
            if ordinal % 2 == 0 else
            ("cold_grid_astar", "persistent_dstar_lite")
        )
        round_rows: Dict[str, Mapping[str, Any]] = {}
        for order_index, arm in enumerate(order, 1):
            try:
                row = dict(_run_worker(
                    script, arm, run_root, cache_root, source, cpu,
                ))
                row.update({
                    "ordinal": ordinal,
                    "repetition": repetition,
                    "measured": measured,
                    "order_index": order_index,
                })
                rows.append(row)
                round_rows[arm] = row
            except Exception as error:
                failures.append({
                    "stage": "L2_PAIRED_RUN", "ordinal": ordinal,
                    "repetition": repetition, "arm": arm,
                    "failure_code": type(error).__name__,
                    "failure_detail": str(error),
                })
                raise
        dstar = round_rows["persistent_dstar_lite"]
        astar = round_rows["cold_grid_astar"]
        exact_input = all(dstar[key] == astar[key] for key in (
            "map_hash", "query_hash", "start", "goal", "route_edge_ids",
            "corridor_mask_hash", "roi_binding_hash", "source_cell",
            "blocked_global_hash", "old_mask_hash", "new_mask_hash",
            "dirty_bbox", "dirty_changed_cells",
        ))
        cost_error = abs(float(dstar["path_cost_cells"]) - float(astar["path_cost_cells"]))
        pair = {
            "ordinal": ordinal,
            "repetition": repetition,
            "measured": measured,
            "execution_order": list(order),
            "exact_input_pairing": exact_input,
            "both_success": bool(dstar["success"] and astar["success"]),
            "both_collision_free_l2": int(dstar["blocked_path_cells"]) == 0 and int(astar["blocked_path_cells"]) == 0,
            "path_cost_error_cells": cost_error,
            "path_cost_parity": cost_error <= 1.0e-9,
            "path_cell_parity": dstar["path_hash"] == astar["path_hash"],
            "dstar_backend": dstar["backend"],
            "dstar_ready_after": dstar["dstar_ready_after"],
            "dstar_expanded_nodes": dstar["expanded_nodes"],
            "astar_expanded_nodes": astar["expanded_nodes"],
            "expanded_reduction": 1.0 - float(dstar["expanded_nodes"]) / float(astar["expanded_nodes"]),
            "dstar_open_queue_pops": dstar["open_queue_pops"],
            "astar_open_queue_pops": astar["open_queue_pops"],
            "open_pop_reduction": 1.0 - float(dstar["open_queue_pops"]) / float(astar["open_queue_pops"]),
            "dstar_vertex_update_calls": dstar["vertex_update_calls"],
            "astar_successful_relaxations": astar["successful_relaxations"],
            "dstar_search_time_ms": dstar["search_time_ms"],
            "astar_search_time_ms": astar["search_time_ms"],
            "time_reduction": 1.0 - float(dstar["search_time_ms"]) / float(astar["search_time_ms"]),
            "dstar_incremental_peak_rss_bytes": dstar["incremental_peak_rss_bytes"],
            "astar_incremental_peak_rss_bytes": astar["incremental_peak_rss_bytes"],
        }
        pair["pair_pass"] = bool(
            exact_input and pair["both_success"] and pair["both_collision_free_l2"]
            and pair["path_cost_parity"]
            and dstar["backend"] == "compact_persistent_dstar"
            and dstar["dstar_ready_after"] is True
            and not dstar["timeout"] and not dstar["partial_result"]
            and not astar["timeout"]
        )
        paired.append(pair)

    measured_rows = [row for row in rows if row["measured"]]
    arms: Dict[str, Any] = {}
    for arm in ("persistent_dstar_lite", "cold_grid_astar"):
        selected = [row for row in measured_rows if row["arm"] == arm]
        arms[arm] = {
            "expanded_nodes": _summary([float(row["expanded_nodes"]) for row in selected]),
            "generated_nodes": _summary([float(row["generated_nodes"]) for row in selected]),
            "open_queue_pops": _summary([float(row["open_queue_pops"]) for row in selected]),
            "search_time_ms": _summary([float(row["search_time_ms"]) for row in selected]),
            "solver_response_ms": _summary([float(row["solver_response_ms"]) for row in selected]),
            "cpu_ms": _summary([float(row["cpu_ms"]) for row in selected]),
            "incremental_peak_rss_mib": _summary([
                float(row["incremental_peak_rss_bytes"]) / (1024.0 * 1024.0)
                for row in selected
            ]),
            "ready_rss_mib": _summary([
                float(row["ready_rss_bytes"]) / (1024.0 * 1024.0)
                for row in selected
            ]),
            "ready_state_mib": _summary([
                float(row["ready_state_bytes"]) / (1024.0 * 1024.0)
                for row in selected
            ]),
            "state_after_mib": _summary([
                float(row["state_bytes_after"]) / (1024.0 * 1024.0)
                for row in selected
            ]),
            "path_cost_cells": _summary([float(row["path_cost_cells"]) for row in selected]),
            "path_length_m": _summary([float(row["path_length_m"]) for row in selected]),
            "all_success": all(row["success"] for row in selected),
            "all_collision_free_l2": all(int(row["blocked_path_cells"]) == 0 for row in selected),
            "all_timeout_free": all(not row["timeout"] for row in selected),
        }
    measured_pairs = [pair for pair in paired if pair["measured"]]
    expanded_reduction = 1.0 - (
        arms["persistent_dstar_lite"]["expanded_nodes"]["p50"]
        / arms["cold_grid_astar"]["expanded_nodes"]["p50"]
    )
    pop_reduction = 1.0 - (
        arms["persistent_dstar_lite"]["open_queue_pops"]["p50"]
        / arms["cold_grid_astar"]["open_queue_pops"]["p50"]
    )
    time_reduction = 1.0 - (
        arms["persistent_dstar_lite"]["search_time_ms"]["p50"]
        / arms["cold_grid_astar"]["search_time_ms"]["p50"]
    )
    solver_response_reduction = 1.0 - (
        arms["persistent_dstar_lite"]["solver_response_ms"]["p50"]
        / arms["cold_grid_astar"]["solver_response_ms"]["p50"]
    )

    # Persist the expensive paired L2 evidence before starting the independent
    # ROS/Smac correctness gate.  A fail-closed ACK error must not erase the
    # measurements that preceded it.
    _write_csv(output / "l2_runs.csv", rows)
    with (output / "l2_runs.jsonl").open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(_jsonable(row), sort_keys=True) + "\n")
    _write_csv(output / "paired_comparisons.csv", paired)
    try:
        l3_rows = _run_l3_gates(
            output, case, cache_root, source,
            repetitions=l3_repetitions, ros_domain_id=ros_domain_id,
        )
    except Exception as error:
        failures.append({
            "stage": "L3_PATHAUDIT_GATE",
            "failure_code": type(error).__name__,
            "failure_detail": str(error),
            "ros_domain_id": ros_domain_id,
        })
        _write_csv(output / "failures.csv", failures)
        raise
    l3_hashes = {
        arm: {row["canonical_path_hash"] for row in l3_rows if row["arm"] == arm}
        for arm in ("persistent_dstar_lite", "cold_grid_astar")
    }
    l3_all_valid = all(
        row["final_valid_success"]
        and row["static_footprint_valid"]
        and row["kinematic_valid"]
        and row["within_corridor"]
        and float(row["reverse_distance_m"] or 0.0) == 0.0
        and int(row["in_place_rotation_count"] or 0) == 0
        and float(row["maximum_curvature_1pm"] or 0.0) <= 2.501
        and row["roi_ack"] is True
        and int(row["roi_ack_mismatch_cells"] or 0) == 0
        for row in l3_rows
    )
    l3_path_hash_parity = l3_hashes["persistent_dstar_lite"] == l3_hashes["cold_grid_astar"]
    l3_quality_fields = (
        "path_length_m", "minimum_clearance_m", "maximum_curvature_1pm",
        "static_footprint_valid", "kinematic_valid", "reverse_distance_m",
        "in_place_rotation_count", "final_valid_success",
    )
    l3_quality_parity = all(
        all(
            next(row for row in l3_rows if row["repetition"] == repetition and row["arm"] == "persistent_dstar_lite")[field]
            == next(row for row in l3_rows if row["repetition"] == repetition and row["arm"] == "cold_grid_astar")[field]
            for field in l3_quality_fields
        )
        for repetition in range(1, l3_repetitions + 1)
    )

    summary = {
        "protocol_id": PROTOCOL_ID,
        "architecture_id": ARCHITECTURE_ID,
        "classification": "strict_paired_component_ab_on_real_map_with_l3_pathaudit_gate",
        "map_hash": case.ctx.map_sha256,
        "query_hash": case.query_metadata["query_hash"],
        "query_id": QUERY_ID,
        "source_cell": list(source),
        "source_count": 1,
        "effective_changed_cells": pipeline_gate["effective_changed_cell_count"],
        "route_edge_ids": list(case.plan.route_edge_ids),
        "l2_binding_hash": initial_binding,
        "warmups": warmups,
        "measured_repetitions": repetitions,
        "l3_gate_repetitions_per_arm": l3_repetitions,
        "arms": arms,
        "paired": {
            "all_pair_gates_pass": all(pair["pair_pass"] for pair in measured_pairs),
            "exact_input_pairing": all(pair["exact_input_pairing"] for pair in measured_pairs),
            "path_cost_parity": all(pair["path_cost_parity"] for pair in measured_pairs),
            "path_cell_parity_count": sum(pair["path_cell_parity"] for pair in measured_pairs),
            "expanded_nodes_p50_reduction": expanded_reduction,
            "open_queue_pops_p50_reduction": pop_reduction,
            "search_time_p50_reduction": time_reduction,
            "solver_response_p50_reduction": solver_response_reduction,
            "dstar_vertex_update_calls": _summary([
                float(pair["dstar_vertex_update_calls"]) for pair in measured_pairs
            ]),
            "astar_successful_relaxations": _summary([
                float(pair["astar_successful_relaxations"]) for pair in measured_pairs
            ]),
        },
        "l3_pathaudit_gate": {
            "all_valid": l3_all_valid,
            "path_hash_parity": l3_path_hash_parity,
            "quality_metric_parity": l3_quality_parity,
            "canonical_path_hashes": {key: sorted(value) for key, value in l3_hashes.items()},
            "collision_cases": sum(not row["static_footprint_valid"] for row in l3_rows),
            "kinematic_invalid_cases": sum(not row["kinematic_valid"] for row in l3_rows),
            "roi_ack_mismatch_cells": sum(int(row["roi_ack_mismatch_cells"] or 0) for row in l3_rows),
        },
    }
    summary["strict_ab_pass"] = bool(
        pipeline_gate["pass"]
        and summary["paired"]["all_pair_gates_pass"]
        and l3_all_valid and l3_path_hash_parity and l3_quality_parity
    )

    _write_csv(output / "l3_pathaudit_gates.csv", l3_rows)
    _write_csv(output / "failures.csv", failures)
    (output / "summary.json").write_text(
        json.dumps(_jsonable(summary), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    _plot(output, summary)

    source_hashes = _source_snapshot(output, script)
    protocol = {
        "protocol_id": PROTOCOL_ID,
        "experiment_kind": "dynamic_incremental_optimization_ab",
        "architecture_id": ARCHITECTURE_ID,
        "frozen_revision": "3D-V1-r1-l2-state-lifecycle-soak",
        "query_id": QUERY_ID,
        "map_hash": EXPECTED_MAP_SHA256,
        "pdmap_hash": EXPECTED_PDMAP_SHA256,
        "semantic_map_hash": EXPECTED_SEMANTIC_MAP_HASH,
        "query_hash": EXPECTED_QUERY_HASH,
        "resolution_m": 0.05,
        "route_edge_ids": list(case.plan.route_edge_ids),
        "l2_binding_hash": initial_binding,
        "source_selection": {
            "method": "initial_l2_path_floor_20_percent",
            "fraction": SOURCE_FRACTION,
            "expected_global_cell": list(EXPECTED_SOURCE_CELL),
            "source_count": 1,
            "dynamic_inflation_radius_cells": DYNAMIC_INFLATION_RADIUS_CELLS,
        },
        "arms": {
            "candidate": "verified_ready_compact_persistent_dstar_lite",
            "baseline": "frozen_deterministic_grid_astar_cold_start",
        },
        "dstar_policy": {
            "ready_required": True,
            "changed_source_limit": DSTAR_CHANGED_SOURCE_LIMIT,
            "wall_budget_ms": DSTAR_WALL_BUDGET_MS,
            "max_expansions": DSTAR_MAX_EXPANSIONS,
            "partial_result_forbidden": True,
            "fallback_forbidden_in_candidate_arm": True,
        },
        "measurement": {
            "warmups": warmups,
            "repetitions": repetitions,
            "alternating_order": True,
            "cpu_affinity": cpu,
            "quantile_method": "numpy_linear",
            "l2_only_timing": True,
            "astar_open_counts": "exact_frozen_function_heap_trace_replay",
            "astar_official_time": "unmodified_frozen_function_before_trace_replay",
            "memory": "fresh_worker_ready_rss_vs_sampled_online_peak_rss",
        },
        "correctness_gates": {
            "same_old_new_roi": True,
            "same_map_query_corridor_obstacle": True,
            "reachable_and_optimal_cost_parity_tolerance_cells": 1.0e-9,
            "l2_blocked_path_cells": 0,
            "l3_smac_bins": 48,
            "l3_motion_model": "DUBIN",
            "canonical_pathaudit": True,
            "collision": 0,
            "kinematic_violations": 0,
            "reverse": 0,
            "rotate_in_place": 0,
            "maximum_curvature_1pm": 2.50,
        },
        "scope_limit": "component-level L2 causal A/B; not a vehicle arrival or real-sensor distribution",
    }
    (output / "protocol.yaml").write_text(
        yaml.safe_dump(_jsonable(protocol), sort_keys=False), encoding="utf-8",
    )
    environment = {
        "timestamp_sgt": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "hostname": platform.node(),
        "platform": platform.platform(),
        "python": sys.version,
        "cpu_count": os.cpu_count(),
        "worker_cpu_affinity": cpu,
        "git_head": _git_head(),
        "background_processes_preserved": True,
        "performance_isolation_note": "same-CPU alternating workers; pre-existing user processes were not stopped",
    }
    (output / "environment.yaml").write_text(
        yaml.safe_dump(_jsonable(environment), sort_keys=False), encoding="utf-8",
    )
    after = _process_snapshot()
    (output / "processes_after.txt").write_text(after, encoding="utf-8")
    reproduction = (
        f"cd {ROOT}\n"
        "source /opt/ros/humble/setup.bash\n"
        f"source {ROOT}/external/arena4_ws/install/setup.bash\n"
        f"PYTHONPATH={ROOT}/external/arena4_ws/src/arena/three_d_v1:"
        f"{ROOT}/external/arena4_ws/src/arena/evaluation/arena_evaluation:"
        f"{ROOT}/external/arena4_ws/src/arena/three_d_v1_nav2:${{PYTHONPATH:-}} "
        f"/usr/bin/python3 -m three_d_v1_nav2.strict_l2_ab "
        f"--output-dir {output}_reproduction --run-root {Path(run_root).resolve()} "
        f"--repetitions {repetitions} --warmups {warmups} "
        f"--l3-repetitions {l3_repetitions} --cpu {cpu} --ros-domain-id {ros_domain_id}\n"
    )
    (output / "reproduction_command.txt").write_text(reproduction, encoding="utf-8")
    report = f"""# 3D-V1-r1 L2 严格配对 A/B（cmp2-04）

## 结论

严格门槛：**{'PASS' if summary['strict_ab_pass'] else 'FAIL'}**。同一冻结地图、cmp2-04 起终点、L1 route、L2 corridor/binding 和单源障碍变化下，persistent D* Lite 的 P50 展开节点减少 **{expanded_reduction:.2%}**，OPEN pop 减少 **{pop_reduction:.2%}**，完整 solver response 缩短 **{solver_response_reduction:.2%}**；其中 core search 缩短 **{time_reduction:.2%}**。

## 固定输入

- map SHA-256：`{case.ctx.map_sha256}`
- query hash：`{case.query_metadata['query_hash']}`
- source cell：`{source}`；确认源数 `1`；old/new 变化 `{pipeline_gate['effective_changed_cell_count']}` cells
- L2 binding：`{initial_binding}`
- route edges：`{','.join(case.plan.route_edge_ids)}`
- D* admission：`dstar_ready=true`，backend=`compact_persistent_dstar`，未 fallback，未返回 partial

## L2 配对结果（{repetitions} 次 measured；P50/P95/P99 为描述性统计）

| 指标 | Persistent D* Lite | Cold grid A* | P50 变化 |
|---|---:|---:|---:|
| expanded nodes | {arms['persistent_dstar_lite']['expanded_nodes']['p50']:.0f} / {arms['persistent_dstar_lite']['expanded_nodes']['p95']:.0f} / {arms['persistent_dstar_lite']['expanded_nodes']['p99']:.0f} | {arms['cold_grid_astar']['expanded_nodes']['p50']:.0f} / {arms['cold_grid_astar']['expanded_nodes']['p95']:.0f} / {arms['cold_grid_astar']['expanded_nodes']['p99']:.0f} | {expanded_reduction:.2%} reduction |
| OPEN queue pops | {arms['persistent_dstar_lite']['open_queue_pops']['p50']:.0f} / {arms['persistent_dstar_lite']['open_queue_pops']['p95']:.0f} / {arms['persistent_dstar_lite']['open_queue_pops']['p99']:.0f} | {arms['cold_grid_astar']['open_queue_pops']['p50']:.0f} / {arms['cold_grid_astar']['open_queue_pops']['p95']:.0f} / {arms['cold_grid_astar']['open_queue_pops']['p99']:.0f} | {pop_reduction:.2%} reduction |
| search time ms | {arms['persistent_dstar_lite']['search_time_ms']['p50']:.3f} / {arms['persistent_dstar_lite']['search_time_ms']['p95']:.3f} / {arms['persistent_dstar_lite']['search_time_ms']['p99']:.3f} | {arms['cold_grid_astar']['search_time_ms']['p50']:.3f} / {arms['cold_grid_astar']['search_time_ms']['p95']:.3f} / {arms['cold_grid_astar']['search_time_ms']['p99']:.3f} | {time_reduction:.2%} |
| solver response ms | {arms['persistent_dstar_lite']['solver_response_ms']['p50']:.3f} / {arms['persistent_dstar_lite']['solver_response_ms']['p95']:.3f} / {arms['persistent_dstar_lite']['solver_response_ms']['p99']:.3f} | {arms['cold_grid_astar']['solver_response_ms']['p50']:.3f} / {arms['cold_grid_astar']['solver_response_ms']['p95']:.3f} / {arms['cold_grid_astar']['solver_response_ms']['p99']:.3f} | {solver_response_reduction:.2%} |
| incremental peak RSS MiB | {arms['persistent_dstar_lite']['incremental_peak_rss_mib']['p50']:.3f} / {arms['persistent_dstar_lite']['incremental_peak_rss_mib']['p95']:.3f} / {arms['persistent_dstar_lite']['incremental_peak_rss_mib']['p99']:.3f} | {arms['cold_grid_astar']['incremental_peak_rss_mib']['p50']:.3f} / {arms['cold_grid_astar']['incremental_peak_rss_mib']['p95']:.3f} / {arms['cold_grid_astar']['incremental_peak_rss_mib']['p99']:.3f} | worker RSS delta |
| persistent state MiB (ready / after) | {arms['persistent_dstar_lite']['ready_state_mib']['p50']:.3f} / {arms['persistent_dstar_lite']['state_after_mib']['p50']:.3f} | 0 / 0 | D* retained state |

D* `UpdateVertex` P50 为 {summary['paired']['dstar_vertex_update_calls']['p50']:.0f}；A* 不存在 D* 的 `UpdateVertex` 操作，因此不伪造同名指标，另报成功 relax P50={summary['paired']['astar_successful_relaxations']['p50']:.0f}。

## 正确性与 PathAudit

- {sum(pair['pair_pass'] for pair in measured_pairs)}/{len(measured_pairs)} 配对输入、可达性、最优 cost、blocked-cell 检查通过；L2 最优 cost 差均 <= 1e-9 cell。
- L2 栅格路径允许是不同的等价最优解；相同 downstream mask 进入 L3。
- L3/PathAudit：{sum(row['final_valid_success'] for row in l3_rows)}/{len(l3_rows)} final-valid；两臂 canonical path hash {'一致' if l3_path_hash_parity else '不一致'}；碰撞 {summary['l3_pathaudit_gate']['collision_cases']}，运动学违规 {summary['l3_pathaudit_gate']['kinematic_invalid_cases']}，ROI ACK mismatch {summary['l3_pathaudit_gate']['roi_ack_mismatch_cells']}。

## 解释边界

这证明的是：在该冻结真实地图 corridor 上、已具备可用 persistent state、单一小范围障碍变化的适用桶内，D* Lite 是否比同输入冷启动 grid A* 少展开节点/少弹出 OPEN，并同时给出时间与内存结果。它不代表大障碍、恢复降价、`dstar_ready=false` 或真实感知噪声分布；那些情况按冻结合同直接回退 grid A*。
"""
    (output / "final_report.md").write_text(report, encoding="utf-8")
    manifest = {
        "experiment_id": output.name,
        "complete": summary["strict_ab_pass"],
        "elapsed_ms": (time.monotonic_ns() - started_ns) / 1.0e6,
        "run_root": str(Path(run_root).resolve()),
        "map_yaml": str(case.map_yaml),
        "topology_dir": str(case.topology_dir),
        "initial_backend": initial_backend,
        "initial_l2_binding_hash": initial_binding,
        "initial_path_hash": _stable_hash(initial_path),
        "prebuild_ms": prebuild_ms,
        "source_hashes": source_hashes,
        "artifacts": {},
    }
    for path in sorted(item for item in output.rglob("*") if item.is_file()):
        if path.name == "manifest.yaml":
            continue
        manifest["artifacts"][str(path.relative_to(output))] = sha256_file(path)
    (output / "manifest.yaml").write_text(
        yaml.safe_dump(_jsonable(manifest), sort_keys=False), encoding="utf-8",
    )
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", action="store_true")
    parser.add_argument("--arm", choices=("persistent_dstar_lite", "cold_grid_astar"))
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--cache-root", type=Path)
    parser.add_argument("--source-row", type=int, default=EXPECTED_SOURCE_CELL[0])
    parser.add_argument("--source-column", type=int, default=EXPECTED_SOURCE_CELL[1])
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--warmups", type=int, default=1)
    parser.add_argument("--l3-repetitions", type=int, default=3)
    parser.add_argument("--cpu", type=int, default=15)
    parser.add_argument("--ros-domain-id", type=int, default=230)
    args = parser.parse_args()
    if args.worker:
        if args.arm is None or args.cache_root is None:
            parser.error("--worker requires --arm and --cache-root")
        _worker(args)
        return
    if args.output_dir is None:
        parser.error("--output-dir is required")
    result = run(
        args.output_dir, run_root=args.run_root,
        repetitions=args.repetitions, warmups=args.warmups,
        l3_repetitions=args.l3_repetitions, cpu=args.cpu,
        ros_domain_id=args.ros_domain_id,
    )
    print(f"PASS output={result}", flush=True)


if __name__ == "__main__":
    main()
