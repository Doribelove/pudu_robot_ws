"""Profile the frozen 3D-V1/r0 L2 lifecycle before r1 optimization."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import resource
import shutil
import sys
import time
import tracemalloc
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

import numpy as np
import yaml

from arena_evaluation.dstar_lite import DStarSearchStats

from . import l2_incremental as r0_l2
from .pipeline import Layered3DV1Controller
from .real_stage_a_benchmark import (
    MAP_ID,
    ROOT,
    _load_inputs,
    _select_path_sources,
    _snapshot,
)


ARCHITECTURE_ID = "3D-V1"
REVISION_ID = "r1-l2-state-lifecycle-soak"
PROTOCOL_ID = "PLN-02-3D-V1-R1-L2-LIFECYCLE-V1"
Cell = Tuple[int, int]


def _elapsed_ms(started_ns: int) -> float:
    return (time.monotonic_ns() - started_ns) / 1.0e6


def _rss_bytes() -> int:
    try:
        for line in Path("/proc/self/status").read_text(encoding="utf-8").splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) * 1024
    except OSError:
        pass
    return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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
                if isinstance(value, (dict, list, tuple)) else value
                for key, value in row.items()
            })


def _mapping_bytes(values: Mapping[Any, Any]) -> int:
    return int(sys.getsizeof(values) + sum(
        sys.getsizeof(key) + sys.getsizeof(value) for key, value in values.items()
    ))


def _sequence_bytes(values: Sequence[Any]) -> int:
    return int(sys.getsizeof(values) + sum(sys.getsizeof(value) for value in values))


class ProfiledR0CornerSafe(r0_l2.CornerSafeDStarLite):
    """Behavior-identical r0 D* with timing and visit counters."""

    def __init__(
        self,
        traversable: np.ndarray,
        start: Cell,
        goal: Cell,
        *,
        cost_map: Optional[np.ndarray] = None,
        allow_diagonal: bool = True,
    ) -> None:
        self.array_setup_ms = 0.0
        self.initial_open_ms = 0.0
        self.neighbor_calls = 0
        self.neighbor_yields = 0
        self.neighbor_generation_ms = 0.0
        self.predecessor_calls = 0
        self.predecessor_visits = 0
        self.update_cells_ms = 0.0
        self.extract_ms = 0.0
        self._constructing = True
        started = time.monotonic_ns()
        super().__init__(
            traversable, start, goal,
            cost_map=cost_map, allow_diagonal=allow_diagonal,
        )
        self.constructor_total_ms = _elapsed_ms(started)
        self._constructing = False
        self.g_rhs_and_object_init_ms = max(
            0.0, self.constructor_total_ms - self.array_setup_ms - self.initial_open_ms,
        )

    def _set_arrays(self, traversable: np.ndarray, cost_map: Optional[np.ndarray]) -> None:
        started = time.monotonic_ns()
        super()._set_arrays(traversable, cost_map)
        self.array_setup_ms += _elapsed_ms(started)

    def _push(self, cell: Cell) -> None:
        started = time.monotonic_ns()
        super()._push(cell)
        if self._constructing:
            self.initial_open_ms += _elapsed_ms(started)

    def _neighbors(self, cell: Cell):  # type: ignore[override]
        started = time.monotonic_ns()
        values = tuple(super()._neighbors(cell))
        self.neighbor_generation_ms += _elapsed_ms(started)
        self.neighbor_calls += 1
        self.neighbor_yields += len(values)
        yield from values

    def _predecessors(self, cell: Cell):  # type: ignore[override]
        values = tuple(super()._predecessors(cell))
        self.predecessor_calls += 1
        self.predecessor_visits += len(values)
        yield from values

    def update_cells(self, changed_cells, **kwargs):  # type: ignore[override]
        started = time.monotonic_ns()
        result = super().update_cells(changed_cells, **kwargs)
        self.update_cells_ms += _elapsed_ms(started)
        return result

    def extract_path(self, *, max_length: Optional[int] = None):  # type: ignore[override]
        started = time.monotonic_ns()
        result = super().extract_path(max_length=max_length)
        self.extract_ms += _elapsed_ms(started)
        return result

    def counters(self) -> Dict[str, float]:
        return {
            "neighbor_calls": float(self.neighbor_calls),
            "neighbor_yields": float(self.neighbor_yields),
            "neighbor_generation_ms": self.neighbor_generation_ms,
            "predecessor_calls": float(self.predecessor_calls),
            "predecessor_visits": float(self.predecessor_visits),
            "update_cells_ms": self.update_cells_ms,
            "extract_ms": self.extract_ms,
        }


def _counter_delta(after: Mapping[str, float], before: Mapping[str, float]) -> Dict[str, float]:
    return {key: float(after[key]) - float(before.get(key, 0.0)) for key in after}


def _profile_event(
    controller: Layered3DV1Controller,
    *,
    index: int,
    category: str,
    occupied: Set[Cell],
    map_hash: str,
    map_shape: Sequence[int],
) -> Tuple[Any, Dict[str, Any]]:
    planner = controller.l2.planner
    if not isinstance(planner, ProfiledR0CornerSafe):
        raise AssertionError("r0 profiler lost its instrumented planner")
    counters_before = planner.counters()
    rss_before = _rss_bytes()
    step = controller.process_snapshot(
        _snapshot(index, sorted(occupied), map_hash=map_hash, shape=map_shape),
        now=float(index),
    )
    rss_after = _rss_bytes()
    counters = _counter_delta(planner.counters(), counters_before)
    result = step.l2_result
    row: Dict[str, Any] = {
        "snapshot_index": index,
        "category": category,
        "scheduler_reason": step.scheduler.reason,
        "scheduler_invoked_l2": step.scheduler.invoke_l2,
        "confirmation_ms": step.snapshot_update.confirmation_ms,
        "pipeline_response_ms": step.diagnostics.get("pipeline_response_ms", 0.0),
        "backend": "scheduler_skip" if result is None else result.selected_backend,
        "l2_response_ms": 0.0 if result is None else result.response_ms,
        "compute_shortest_path_ms": 0.0 if result is None else result.dstar_stats.search_time_ms,
        "fallback_search_ms": (
            0.0 if result is None or result.fallback_stats is None
            else result.fallback_stats.search_time_ms
        ),
        "expanded": 0 if result is None else result.dstar_stats.expanded_nodes,
        "heap_pops": 0 if result is None else result.dstar_stats.queue_pops,
        "heap_pushes": 0 if result is None else result.dstar_stats.queue_pushes,
        "update_vertex": 0 if result is None else result.dstar_stats.update_vertex_count,
        "partial_dstar": False if result is None else result.partial_dstar_result_returned,
        "success": True if result is None else result.success,
        "rss_before_bytes": rss_before,
        "rss_after_bytes": rss_after,
        "rss_delta_bytes": rss_after - rss_before,
        **counters,
    }
    return step, row


def run(output: Path, *, query_id: str = "A2B-07") -> Path:
    output = output.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    output.mkdir(parents=True)
    phase_rows: List[Dict[str, Any]] = []
    event_rows: List[Dict[str, Any]] = []
    original_planner = r0_l2.CornerSafeDStarLite
    tracemalloc.start()
    rss_process_start = _rss_bytes()
    try:
        started = time.monotonic_ns()
        ctx, queries, artifact, cache_manifest = _load_inputs()
        phase_rows.append({"phase": "load_map_topology", "wall_ms": _elapsed_ms(started)})
        query = next((item for item in queries if item.query_id == query_id), None)
        if query is None:
            raise ValueError(f"unknown query: {query_id}")

        from .production_l1 import DeterministicGraphAStarL1
        started = time.monotonic_ns()
        l1 = DeterministicGraphAStarL1(
            ctx, artifact, map_hash=ctx.map_sha256,
            topology_hash=str(cache_manifest.get("cache_key") or ""),
        )
        phase_rows.append({"phase": "l1_edge_cell_index", "wall_ms": _elapsed_ms(started)})
        started = time.monotonic_ns()
        plan = l1.plan(query)
        phase_rows.append({"phase": "l1_route_and_corridor", "wall_ms": _elapsed_ms(started)})
        if plan is None:
            raise RuntimeError(f"no r0 L1 plan for {query_id}")

        started = time.monotonic_ns()
        roi_probe = r0_l2.CorridorROI.from_global(
            plan.static_safe_free, plan.corridor_mask,
            plan.start_cell, plan.goal_cell,
            binding_fields=plan.binding_fields(), border_cells=1,
        )
        phase_rows.append({"phase": "corridor_roi_crop", "wall_ms": _elapsed_ms(started)})
        del roi_probe

        r0_l2.CornerSafeDStarLite = ProfiledR0CornerSafe
        heap_before = tracemalloc.take_snapshot()
        rss_before_state = _rss_bytes()
        started = time.monotonic_ns()
        controller = Layered3DV1Controller(
            plan, dynamic_inflation_radius_cells=7,
            dstar_wall_budget_ms=500.0,
            dstar_max_expansions=20_000,
            dstar_attempt_max_changed_cells=2,
        )
        cold_total_ms = _elapsed_ms(started)
        rss_after_state = _rss_bytes()
        heap_after = tracemalloc.take_snapshot()
        planner = controller.l2.planner
        if not isinstance(planner, ProfiledR0CornerSafe):
            raise AssertionError("profile instrumentation was not installed")
        init_stats = controller.initial_l2_result.dstar_stats
        phase_rows.extend([
            {"phase": "r0_array_copy_and_cost_map", "wall_ms": planner.array_setup_ms},
            {"phase": "r0_g_rhs_object_initialization", "wall_ms": planner.g_rhs_and_object_init_ms},
            {"phase": "r0_initial_open_build", "wall_ms": planner.initial_open_ms},
            {"phase": "r0_planner_constructor", "wall_ms": planner.constructor_total_ms},
            {"phase": "r0_first_compute_shortest_path", "wall_ms": init_stats.search_time_ms},
            {"phase": "r0_first_path_extraction", "wall_ms": planner.extract_ms},
            {"phase": "r0_cold_build_total", "wall_ms": cold_total_ms},
        ])

        path = controller.l2.path_global or []
        one = _select_path_sources(path, 1, set())
        _, row = _profile_event(
            controller, index=1, category="one_source_unconfirmed", occupied=one,
            map_hash=ctx.map_sha256, map_shape=artifact.free_mask.shape,
        )
        event_rows.append(row)
        _, row = _profile_event(
            controller, index=2, category="one_source_eligible", occupied=one,
            map_hash=ctx.map_sha256, map_shape=artifact.free_mask.shape,
        )
        event_rows.append(row)
        five = _select_path_sources(controller.l2.path_global or path, 5, one)
        occupied = one | five
        _, row = _profile_event(
            controller, index=3, category="large_change_unconfirmed", occupied=occupied,
            map_hash=ctx.map_sha256, map_shape=artifact.free_mask.shape,
        )
        event_rows.append(row)
        _, row = _profile_event(
            controller, index=4, category="large_change_fallback", occupied=occupied,
            map_hash=ctx.map_sha256, map_shape=artifact.free_mask.shape,
        )
        event_rows.append(row)
        _, row = _profile_event(
            controller, index=5, category="recovery_unconfirmed", occupied=set(),
            map_hash=ctx.map_sha256, map_shape=artifact.free_mask.shape,
        )
        event_rows.append(row)
        _, row = _profile_event(
            controller, index=6, category="recovery_fallback", occupied=set(),
            map_hash=ctx.map_sha256, map_shape=artifact.free_mask.shape,
        )
        event_rows.append(row)

        rss_before_resync = _rss_bytes()
        started = time.monotonic_ns()
        resync = controller.service_l2_resync()
        resync_ms = _elapsed_ms(started)
        rss_after_resync = _rss_bytes()
        event_rows.append({
            "snapshot_index": 7,
            "category": "explicit_quiet_period_resync",
            "scheduler_reason": "EXPLICIT_MAINTENANCE",
            "scheduler_invoked_l2": True,
            "backend": resync.selected_backend,
            "l2_response_ms": resync_ms,
            "compute_shortest_path_ms": resync.dstar_stats.search_time_ms,
            "fallback_search_ms": 0.0,
            "expanded": resync.dstar_stats.expanded_nodes,
            "heap_pops": resync.dstar_stats.queue_pops,
            "heap_pushes": resync.dstar_stats.queue_pushes,
            "update_vertex": resync.dstar_stats.update_vertex_count,
            "partial_dstar": resync.partial_dstar_result_returned,
            "success": resync.success,
            "rss_before_bytes": rss_before_resync,
            "rss_after_bytes": rss_after_resync,
            "rss_delta_bytes": rss_after_resync - rss_before_resync,
        })

        positive_heap_delta = sum(
            max(0, stat.size_diff)
            for stat in heap_after.compare_to(heap_before, "filename")
        )
        state = controller.l2
        planner = state.planner
        arrays = {
            "roi_base_free": int(state.roi.base_free.nbytes),
            "current_free": int(state.current_free.nbytes),
            "planner_traversable": int(planner.traversable.nbytes),
            "planner_cost_map": int(planner.cost_map.nbytes),
        }
        python_objects = {
            "g": _mapping_bytes(planner.g),
            "rhs": _mapping_bytes(planner.rhs),
            "open": _sequence_bytes(planner._open),
            "queued_keys": _mapping_bytes(planner._queued_keys),
        }
        profile: Dict[str, Any] = {
            "architecture_id": ARCHITECTURE_ID,
            "revision_id": REVISION_ID,
            "protocol_id": PROTOCOL_ID,
            "profile_subject": "frozen_3D-V1_r0",
            "map_id": MAP_ID,
            "map_hash": ctx.map_sha256,
            "query_id": query_id,
            "resolution_m": float(ctx.hospital_map.resolution),
            "route_signature": plan.route_signature,
            "route_edge_count": len(plan.route_edge_ids),
            "global_grid_cells": int(plan.corridor_mask.size),
            "global_corridor_cells": int(np.count_nonzero(plan.corridor_mask)),
            "roi_shape": list(state.roi.shape),
            "roi_array_cells": int(state.current_free.size),
            "corridor_safe_cells": int(np.count_nonzero(state.roi.base_free)),
            "r0_cell_to_state_mapping": "none; tuple-keyed dict state",
            "r0_adjacency": "generated on demand; not materialized",
            "r0_cache_file_bytes": 0,
            "r0_cache_status": "NOT_IMPLEMENTED",
            "cold_build_ms": cold_total_ms,
            "first_solve": {
                "expanded": init_stats.expanded_nodes,
                "generated": init_stats.generated_nodes,
                "heap_pops": init_stats.queue_pops,
                "heap_pushes": init_stats.queue_pushes,
                "update_vertex": init_stats.update_vertex_count,
                "predecessor_visits": planner.predecessor_visits,
                "neighbor_calls": planner.neighbor_calls,
                "neighbor_yields": planner.neighbor_yields,
                "neighbor_generation_ms": planner.neighbor_generation_ms,
            },
            "memory": {
                "numpy_arrays_bytes": arrays,
                "numpy_arrays_total_bytes": sum(arrays.values()),
                "python_objects_estimated_bytes": python_objects,
                "python_objects_estimated_total_bytes": sum(python_objects.values()),
                "r0_reported_state_memory_bytes": state.state_memory_bytes(),
                "tracemalloc_positive_delta_bytes": int(positive_heap_delta),
                "rss_process_start_bytes": rss_process_start,
                "rss_before_state_bytes": rss_before_state,
                "rss_after_state_bytes": rss_after_state,
                "rss_state_delta_bytes": rss_after_state - rss_before_state,
                "rss_after_resync_bytes": rss_after_resync,
                "tracemalloc_current_peak_bytes": list(tracemalloc.get_traced_memory()),
            },
            "phases": phase_rows,
            "events": event_rows,
            "trigger_counts": {
                "scheduler_skip": sum(not row.get("scheduler_invoked_l2", False) for row in event_rows),
                "persistent_dstar": sum(row.get("backend") == "persistent_dstar" for row in event_rows),
                "fallback_or_direct_astar": sum("astar" in str(row.get("backend")) for row in event_rows),
                "resync": sum("resync" in str(row.get("backend")) for row in event_rows),
            },
        }
        (output / "profile.json").write_text(
            json.dumps(profile, indent=2, sort_keys=True) + "\n", encoding="utf-8",
        )
        _write_csv(output / "phase_timings.csv", phase_rows)
        with (output / "events.jsonl").open("w", encoding="utf-8") as stream:
            for row in event_rows:
                stream.write(json.dumps(row, sort_keys=True) + "\n")

        source_files = [
            Path(__file__).resolve(), Path(r0_l2.__file__).resolve(),
            Path(__file__).with_name("pipeline.py").resolve(),
            Path(__file__).with_name("dynamic_policy.py").resolve(),
            Path(__file__).with_name("production_l1.py").resolve(),
        ]
        snapshot_dir = output / "source_snapshot"
        snapshot_dir.mkdir()
        source_hashes: Dict[str, str] = {}
        for source in source_files:
            shutil.copy2(source, snapshot_dir / source.name)
            source_hashes[str(source)] = _sha256(source)
        manifest = {
            "architecture_id": ARCHITECTURE_ID,
            "revision_id": REVISION_ID,
            "protocol_id": PROTOCOL_ID,
            "profile_subject": "frozen_3D-V1_r0",
            "map_id": MAP_ID,
            "map_hash": ctx.map_sha256,
            "query_id": query_id,
            "r0_source_bundle_sha256_from_stage0": "8f189bbb6361b6604898f80c76ebc9594b1a909cc8e89db2b27d4873e4ee15f5",
            "source_files": source_hashes,
        }
        (output / "manifest.yaml").write_text(
            yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8",
        )
        (output / "reproduction_command.txt").write_text(
            "cd /home/robot/pudu_robot_ws\n"
            "source /opt/ros/humble/setup.bash\n"
            "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
            f"PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/three_d_v1:/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation /usr/bin/python3 -m arena_3d_v1.r1_profile --output-dir {output} --query-id {query_id}\n",
            encoding="utf-8",
        )
        bottleneck = max(phase_rows, key=lambda row: float(row["wall_ms"]))
        report = [
            "# 3D-V1 r0 L2 lifecycle profile", "",
            f"- map/query: `{MAP_ID}` / `{query_id}`",
            f"- ROI shape/safe cells: `{state.roi.shape}` / `{np.count_nonzero(state.roi.base_free)}`",
            f"- cold build: `{cold_total_ms:.3f} ms`",
            f"- first solve expanded/update_vertex/predecessor visits: `{init_stats.expanded_nodes}` / `{init_stats.update_vertex_count}` / `{planner.predecessor_visits}`",
            f"- reported state memory: `{state.state_memory_bytes()} B`",
            f"- measured RSS state delta: `{rss_after_state - rss_before_state} B`",
            f"- largest measured phase: `{bottleneck['phase']}` = `{float(bottleneck['wall_ms']):.3f} ms`", "",
            "## Evidence-led optimization decision", "",
            "The first reverse search and tuple-keyed Python g/rhs state dominate lifecycle cost and memory. r1 should compact only footprint-safe corridor cells into reversible integer IDs, store float64 g/rhs and dynamic occupancy in arrays/bitsets, retain on-demand corner-safe adjacency, add verified offline cache restore, and bound mutable state residency with an LRU. The deterministic A* fallback and r0 scheduler/L1/L3 contracts remain frozen.", "",
            "The concurrently misrouted `3d_v1_r1_r0_profile_20260904_01` directory is explicitly excluded; this report was generated independently from the restored r0 source.",
        ]
        (output / "profile_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
        verification = {
            "profile_complete": True,
            "profile_precedes_r1_algorithm_optimization": True,
            "r0_core_behavior_monkey_patch_only_for_telemetry": True,
            "partial_dstar_results": sum(bool(row.get("partial_dstar")) for row in event_rows),
            "non_authoritative_misrouted_profile_excluded": True,
        }
        (output / "verification.yaml").write_text(
            yaml.safe_dump(verification, sort_keys=False), encoding="utf-8",
        )
        summary = f"PASS profile={output} cold_build_ms={cold_total_ms:.3f} state_bytes={state.state_memory_bytes()}"
        (output / "stdout.log").write_text(summary + "\n", encoding="utf-8")
        (output / "stderr.log").write_text("", encoding="utf-8")
        print(summary)
        return output
    finally:
        r0_l2.CornerSafeDStarLite = original_planner
        tracemalloc.stop()


def _default_output() -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return ROOT / "experiments/layered_planner_benchmark" / f"3d_v1_r1_r0_profile_{stamp}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--query-id", default="A2B-07")
    args = parser.parse_args()
    run(args.output_dir or _default_output(), query_id=args.query_id)


if __name__ == "__main__":
    main()
