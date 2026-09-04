"""Real 4x-map Stage-A gate for the 3D-V1 L2 policy."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import statistics
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

import numpy as np
import yaml

from arena_evaluation import topology
from arena_evaluation import two_layer_2d_v1_4x_dynamic_incremental_benchmark as map4
from arena_evaluation import two_layer_formal_benchmark as map4_source
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.dynamic_snapshot import DynamicSnapshot

from . import ARCHITECTURE_ID, IMPLEMENTATION_REVISION, PARENT_ARCHITECTURE, PROTOCOL_VERSION
from .dynamic_policy import _inflate
from .l2_incremental import Cell, deterministic_grid_astar
from .pipeline import Layered3DV1Controller
from .production_l1 import DeterministicGraphAStarL1


ROOT = Path("/home/robot/pudu_robot_ws")
MAP_ID = "mentor_map_20260825_005_4x_area"
FROZEN_BASELINES = (
    ROOT / "experiments/layered_planner_benchmark/2a_v1_mentor_map_20260825_005_20_r2_roi_pathaudit_v1",
    ROOT / "experiments/layered_planner_benchmark/2d_v2_static_mentor_map_005_r0_20260903_154754",
    ROOT / "experiments/layered_planner_benchmark/2d_v2_r1_dstar_tail_heldout_20260904_100713",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _tree_hash(directory: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(item for item in directory.rglob("*") if item.is_file()):
        digest.update(str(path.relative_to(directory)).encode("utf-8"))
        digest.update(b"\0")
        digest.update(bytes.fromhex(_sha256(path)))
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
                key: json.dumps(value, sort_keys=True) if isinstance(value, (list, tuple, dict)) else value
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
        return math.inf
    return float(sum(
        math.sqrt(2.0) if first[0] != second[0] and first[1] != second[1] else 1.0
        for first, second in zip(path, path[1:])
    ))


def _select_path_sources(path: Sequence[Cell], count: int, excluded: Set[Cell]) -> Set[Cell]:
    selected: Set[Cell] = set()
    for index in np.linspace(len(path) * 0.20, len(path) * 0.80, count * 5, dtype=int):
        cell = tuple(path[min(len(path) - 2, max(1, int(index)))])
        if cell not in excluded:
            selected.add(cell)
        if len(selected) == count:
            break
    if len(selected) != count:
        raise RuntimeError(f"could not select {count} distinct dynamic source cells")
    return selected


def _load_inputs() -> Tuple[Any, Sequence[Any], Any, Mapping[str, Any]]:
    queries, _metadata = map4_source._load_tasks()
    ctx = map4_source._context()
    artifact = topology.load_topology(
        map4.FOUR_X_CACHE, ctx.hospital_map, runtime.FOOTPRINT,
        padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    if tuple(artifact.free_mask.shape) != (3024, 6574):
        raise ValueError("unexpected 4x production-map shape")
    if abs(float(ctx.hospital_map.resolution) - 0.05) > 1.0e-12:
        raise ValueError("real Stage A requires 0.05 m/cell")
    cache_manifest = yaml.safe_load(
        (map4.FOUR_X_CACHE / "cache_manifest.yaml").read_text(encoding="utf-8")
    ) or {}
    return ctx, queries, artifact, cache_manifest


def _snapshot(
    index: int, occupied: Sequence[Cell], *, map_hash: str, shape: Sequence[int],
) -> DynamicSnapshot:
    return DynamicSnapshot.from_cells(
        f"S{index:03d}", occupied, timestamp=float(index),
        map_version=map_hash, map_shape=(int(shape[0]), int(shape[1])),
    )


def run(
    output: Path, *, query_id: str = "A2B-07", repetitions: int = 3,
    dstar_budget_ms: float = 500.0,
) -> Path:
    output = output.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    if repetitions < 1:
        raise ValueError("repetitions must be positive")
    missing = [str(path) for path in FROZEN_BASELINES if not path.is_dir()]
    if missing:
        raise FileNotFoundError(f"missing frozen baselines: {missing}")
    frozen_before = {str(path): _tree_hash(path) for path in FROZEN_BASELINES}
    output.mkdir(parents=True)
    ctx, queries, artifact, cache_manifest = _load_inputs()
    query = next((item for item in queries if item.query_id == query_id), None)
    if query is None:
        raise ValueError(f"unknown query: {query_id}")
    topology_source_mismatch = (
        str((cache_manifest.get("metadata") or {}).get("source_hash", ""))
        != _sha256(Path(topology.__file__).resolve())
    )
    l1 = DeterministicGraphAStarL1(
        ctx, artifact, map_hash=ctx.map_sha256,
        topology_hash=str(cache_manifest.get("cache_key") or map4.FOUR_X_CACHE.name),
    )
    rows: List[Dict[str, Any]] = []
    initialization_rows: List[Dict[str, Any]] = []
    for repetition in range(1, repetitions + 1):
        l1_started = time.monotonic_ns()
        plan = l1.plan(query)
        l1_ms = (time.monotonic_ns() - l1_started) / 1.0e6
        if plan is None:
            raise RuntimeError(f"production L1 found no route for {query_id}")
        init_started = time.monotonic_ns()
        controller = Layered3DV1Controller(
            plan, dynamic_inflation_radius_cells=7,
            dstar_wall_budget_ms=dstar_budget_ms,
            dstar_max_expansions=20_000,
            dstar_attempt_max_changed_cells=2,
        )
        init_ms = (time.monotonic_ns() - init_started) / 1.0e6
        if not controller.initial_l2_result.success:
            raise RuntimeError("production-map initial L2 path is unavailable")
        initialization_rows.append({
            "repetition": repetition, "query_id": query_id,
            "l1_graph_astar_ms": l1_ms,
            "l2_initialization_ms": init_ms,
            "l2_initial_expanded_nodes": controller.initial_l2_result.dstar_stats.expanded_nodes,
            "l2_state_memory_bytes": controller.initial_l2_result.diagnostics["state_memory_bytes"],
            "l2_roi_shape": controller.initial_l2_result.diagnostics["roi_shape"],
            "corridor_cells": controller.initial_l2_result.diagnostics["corridor_cells"],
            "route_edge_count": len(plan.route_edge_ids),
            "route_signature": plan.route_signature,
        })
        occupied_sources: Set[Cell] = set()
        snapshot_index = 0

        def observe(category: str, sources: Set[Cell]) -> Any:
            nonlocal snapshot_index
            snapshot_index += 1
            step = controller.process_snapshot(
                _snapshot(snapshot_index, sorted(sources), map_hash=ctx.map_sha256,
                          shape=artifact.free_mask.shape),
                l1_replan=lambda blocked: l1.plan(query, blocked),
                now=float(snapshot_index),
            )
            row: Dict[str, Any] = {
                "architecture_id": ARCHITECTURE_ID,
                "implementation_revision": IMPLEMENTATION_REVISION,
                "parent_architecture": PARENT_ARCHITECTURE,
                "protocol_version": PROTOCOL_VERSION,
                "map_id": MAP_ID,
                "query_id": query_id,
                "repetition": repetition,
                "snapshot_index": snapshot_index,
                "category": category,
                "snapshot_accepted": step.snapshot_update.accepted,
                "source_status_change_count": len(step.snapshot_update.source_status_changes),
                "newly_blocked_source_count": len(step.snapshot_update.newly_blocked_sources),
                "newly_freed_source_count": len(step.snapshot_update.newly_freed_sources),
                "effective_changed_cells": len(step.snapshot_update.effective_changed_cells),
                "scheduler_skip": not step.scheduler.invoke_l2,
                "scheduler_reason": step.scheduler.reason,
                "l1_graph_astar_called": step.l1_graph_astar_called,
                "l3_required": step.l3_required,
                "failure_code": step.failure_code,
                "pipeline_response_ms": step.diagnostics.get("pipeline_response_ms", 0.0),
            }
            if step.l2_result is not None:
                result = step.l2_result
                oracle = deterministic_grid_astar(
                    controller.l2.current_free,
                    controller.l2.roi.start_local,
                    controller.l2.roi.goal_local,
                )
                local_path = None if result.path is None else [
                    controller.l2.roi.to_local(cell) for cell in result.path
                ]
                reachable_parity = (local_path is None) == (oracle.path is None)
                cost_error = 0.0 if oracle.path is None else abs(
                    _path_cost(local_path) - oracle.cost
                )
                blocked_absent = not set(local_path or ()).intersection(
                    controller.l2.dynamic_blocked_local
                )
                row.update({
                    "l2_selected_backend": result.selected_backend,
                    "l2_response_ms": result.response_ms,
                    "l2_search_ms": result.dstar_stats.search_time_ms,
                    "l2_expanded_nodes": result.dstar_stats.expanded_nodes,
                    "l2_fallback_search_ms": (
                        result.fallback_stats.search_time_ms if result.fallback_stats else 0.0
                    ),
                    "cold_grid_astar_ms": oracle.search_time_ms,
                    "cold_grid_astar_expanded_nodes": oracle.expanded_nodes,
                    "reachable_parity": reachable_parity,
                    "path_cost_error": cost_error,
                    "blocked_cell_absent": blocked_absent,
                    "partial_dstar_result_returned": result.partial_dstar_result_returned,
                    "all_correct": bool(
                        reachable_parity and cost_error <= 1.0e-9 and blocked_absent
                        and not result.partial_dstar_result_returned
                    ),
                })
            else:
                row.update({
                    "l2_selected_backend": "scheduler_skip",
                    "l2_response_ms": 0.0,
                    "cold_grid_astar_ms": 0.0,
                    "reachable_parity": True,
                    "path_cost_error": 0.0,
                    "blocked_cell_absent": True,
                    "partial_dstar_result_returned": False,
                    "all_correct": True,
                })
            rows.append(row)
            return step

        first = _select_path_sources(controller.l2.path_global or (), 1, set())
        observe("one_source_pending", first)
        observe("one_source_confirmed", first)
        occupied_sources |= first

        second = _select_path_sources(
            controller.l2.path_global or (), 1, occupied_sources,
        )
        observe("second_source_pending", occupied_sources | second)
        observe("second_source_confirmed", occupied_sources | second)
        occupied_sources |= second

        five = _select_path_sources(
            controller.l2.path_global or (), 5, occupied_sources,
        )
        observe("five_sources_pending", occupied_sources | five)
        observe("five_sources_confirmed", occupied_sources | five)
        occupied_sources |= five

        observe("recovery_pending", set())
        observe("recovery_confirmed", set())

    frozen_after = {str(path): _tree_hash(path) for path in FROZEN_BASELINES}
    if frozen_before != frozen_after:
        raise RuntimeError("a frozen baseline changed during real Stage A")
    _write_csv(output / "runs.csv", rows)
    _write_csv(output / "initialization.csv", initialization_rows)
    invoked = [row for row in rows if not row["scheduler_skip"]]
    eligible = [row for row in invoked if row["l2_selected_backend"] == "persistent_dstar"]
    candidate = _summary([float(row["l2_response_ms"]) for row in invoked])
    baseline = _summary([float(row["cold_grid_astar_ms"]) for row in invoked])
    eligible_candidate = _summary([float(row["l2_response_ms"]) for row in eligible])
    eligible_baseline = _summary([float(row["cold_grid_astar_ms"]) for row in eligible])
    eligible_reduction = (
        1.0 - eligible_candidate["p50"] / eligible_baseline["p50"]
        if eligible_baseline["p50"] > 0 else -math.inf
    )
    p95_ratio = candidate["p95"] / baseline["p95"] if baseline["p95"] > 0 else math.inf
    p99_ratio = candidate["p99"] / baseline["p99"] if baseline["p99"] > 0 else math.inf
    gates = {
        "scope": "real_4x_map_l2_stage_a",
        "map_id": MAP_ID,
        "query_id": query_id,
        "correctness_rows": len(rows),
        "correctness_failures": sum(not row["all_correct"] for row in rows),
        "oracle_parity_pass": all(row["all_correct"] for row in rows),
        "scheduler_skip_count": sum(row["scheduler_skip"] for row in rows),
        "l2_invoked_count": len(invoked),
        "dstar_eligible_count": len(eligible),
        "candidate_invoked_response_ms": candidate,
        "cold_astar_invoked_response_ms": baseline,
        "eligible_dstar_response_ms": eligible_candidate,
        "eligible_cold_astar_response_ms": eligible_baseline,
        "eligible_p50_reduction": eligible_reduction,
        "all_invoked_p95_ratio": p95_ratio,
        "all_invoked_p99_ratio": p99_ratio,
        "eligible_p50_gate_pass": eligible_reduction >= 0.20,
        "p95_gate_pass": p95_ratio <= 1.05,
        "p99_gate_pass": p99_ratio <= 1.10,
        "recovery_pass": all(
            row["all_correct"] for row in rows if row["category"] == "recovery_confirmed"
        ),
        "partial_dstar_results": sum(row["partial_dstar_result_returned"] for row in rows),
        "topology_cache_source_hash_differs_from_current_workspace": topology_source_mismatch,
        "cache_treated_as_frozen_input_with_actual_source_recorded": True,
    }
    gates["real_stage_a_pass"] = all(bool(gates[key]) for key in (
        "oracle_parity_pass", "eligible_p50_gate_pass", "p95_gate_pass",
        "p99_gate_pass", "recovery_pass",
    )) and gates["partial_dstar_results"] == 0
    (output / "gate_results.yaml").write_text(
        yaml.safe_dump(gates, sort_keys=False), encoding="utf-8",
    )
    _write_csv(output / "correctness_oracle.csv", [{
        key: row.get(key) for key in (
            "repetition", "snapshot_index", "category", "l2_selected_backend",
            "reachable_parity", "path_cost_error", "blocked_cell_absent",
            "partial_dstar_result_returned", "all_correct",
        )
    } for row in rows])
    _write_csv(output / "timing_summary.csv", [
        {"group": "all_invoked", "arm": "selective_dstar_astar", **candidate},
        {"group": "all_invoked", "arm": "cold_grid_astar", **baseline},
        {"group": "dstar_eligible", "arm": "persistent_dstar", **eligible_candidate},
        {"group": "dstar_eligible", "arm": "cold_grid_astar", **eligible_baseline},
    ])

    package_root = Path(__file__).resolve().parents[1]
    source_dir = output / "source_snapshot"
    source_dir.mkdir()
    own_sources = sorted(
        list((package_root / "arena_3d_v1").glob("*.py"))
        + list((package_root / "config").glob("*.yaml"))
        + [package_root / "package.xml", package_root / "setup.py", package_root / "setup.cfg"]
    )
    # Keep the dependency list explicit without importing historical 3D-V0.
    from arena_evaluation import l1_l3_corridor_hybrid_smoke, two_layer_v1_formal_benchmark
    dependency_sources = [
        Path(topology.__file__).resolve(), Path(runtime.__file__).resolve(),
        Path(l1_l3_corridor_hybrid_smoke.__file__).resolve(),
        Path(two_layer_v1_formal_benchmark.__file__).resolve(),
    ]
    source_manifest: Dict[str, str] = {}
    for source in own_sources:
        destination = source_dir / "arena_3d_v1" / source.relative_to(package_root)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        source_manifest[str(source)] = _sha256(source)
    for source in dependency_sources:
        destination = source_dir / "production_dependencies" / source.name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        source_manifest[str(source)] = _sha256(source)
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "parent_architecture": PARENT_ARCHITECTURE,
        "explicitly_not_derived_from": "3D-V0",
        "protocol_version": PROTOCOL_VERSION,
        "map_id": MAP_ID,
        "map_hash": ctx.map_sha256,
        "topology_cache": str(map4.FOUR_X_CACHE),
        "topology_cache_manifest_source_hash": str(
            (cache_manifest.get("metadata") or {}).get("source_hash", "")
        ),
        "actual_topology_source_hash": _sha256(Path(topology.__file__).resolve()),
        "frozen_baseline_tree_hashes": frozen_before,
        "source_files": source_manifest,
    }, sort_keys=False), encoding="utf-8")
    (output / "reproduction_command.txt").write_text(
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        f"ROS_DOMAIN_ID=131 ros2 run arena_3d_v1 three_d_v1_real_stage_a --output-dir {output} --query-id {query_id} --repetitions {repetitions} --dstar-budget-ms {dstar_budget_ms}\n",
        encoding="utf-8",
    )
    report = [
        "# 3D-V1 real 4x-map L2 Stage-A report", "",
        f"- map/query: `{MAP_ID}` / `{query_id}`",
        f"- correctness: {len(rows) - gates['correctness_failures']}/{len(rows)}",
        f"- scheduler skips: {gates['scheduler_skip_count']}/{len(rows)}",
        f"- eligible D* P50: {eligible_candidate['p50']:.6f} ms; paired A*: {eligible_baseline['p50']:.6f} ms; reduction {eligible_reduction:.2%}",
        f"- all invoked candidate P95/P99 ratios vs A*: {p95_ratio:.3f}/{p99_ratio:.3f}",
        f"- real Stage-A gate: **{'PASS' if gates['real_stage_a_pass'] else 'FAIL'}**", "",
        "Stage B (ROS/Nav2/Smac) is permitted only when this gate passes. This run does not claim Stage-B end-to-end value.",
        (
            "The frozen topology cache source hash differs from the current workspace; both hashes are preserved instead of rewriting the cache."
            if topology_source_mismatch else
            "The frozen topology cache source hash matches the current topology source; both values are still recorded."
        ),
    ]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    (output / "verification.yaml").write_text(yaml.safe_dump({
        "required_artifacts_present": True,
        "frozen_baselines_unchanged": frozen_before == frozen_after,
        "source_snapshot_count": len(source_manifest),
        "stage_b": "PERMITTED" if gates["real_stage_a_pass"] else "NOT_RUN_L2_GATE_FAILED",
        "ros_processes_started": False,
        "gazebo_nav2_smac_started": False,
    }, sort_keys=False), encoding="utf-8")
    return output


def _default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return ROOT / "experiments/layered_planner_benchmark" / f"3d_v1_l2_real_4x_stage_a_{stamp}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--query-id", default="A2B-07")
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--dstar-budget-ms", type=float, default=500.0)
    args = parser.parse_args()
    print(run(
        args.output_dir or _default_output(), query_id=args.query_id,
        repetitions=args.repetitions, dstar_budget_ms=args.dstar_budget_ms,
    ))


if __name__ == "__main__":
    main()
