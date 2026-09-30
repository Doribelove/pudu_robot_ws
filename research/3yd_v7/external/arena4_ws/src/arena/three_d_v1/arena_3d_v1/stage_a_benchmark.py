"""Independent Stage-A value gate for the 3D-V1 L2 implementation."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import statistics
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import numpy as np
import yaml

from . import ARCHITECTURE_ID, IMPLEMENTATION_REVISION, PARENT_ARCHITECTURE, PROTOCOL_VERSION
from .l2_incremental import (
    Cell,
    CorridorROI,
    PersistentCorridorDStar,
    deterministic_grid_astar,
)


ROOT = Path("/home/robot/pudu_robot_ws")
DEFAULT_REPETITIONS = 10


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return ROOT / "experiments/layered_planner_benchmark" / f"3d_v1_l2_stage_a_preflight_{stamp}"


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    values = list(rows)
    keys: List[str] = []
    for row in values:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys or ["status"])
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


def _path_cost(path: Sequence[Cell] | None) -> float:
    if not path:
        return math.inf
    return float(sum(
        math.sqrt(2.0) if first[0] != second[0] and first[1] != second[1] else 1.0
        for first, second in zip(path, path[1:])
    ))


def _synthetic_roi() -> CorridorROI:
    """Deterministic full-resolution maze; not a production-map claim."""
    free = np.ones((121, 181), dtype=bool)
    # Alternating door positions force a long path while keeping local
    # detours available for the small-change measurements.
    for column, gap_row in ((35, 25), (70, 95), (105, 25), (140, 95)):
        free[:, column:column + 2] = False
        free[max(1, gap_row - 7):min(120, gap_row + 8), column:column + 2] = True
    corridor = np.ones_like(free)
    start, goal = (110, 10), (10, 170)
    return CorridorROI.from_global(
        free, corridor, start, goal,
        binding_fields={
            "map_hash": "synthetic-stage-a-v1",
            "map_origin": (0.0, 0.0, 0.0),
            "resolution": 0.05,
            "topology_hash": "synthetic-topology-v1",
            "route_edge_ids": ("synthetic-route",),
            "footprint_hash": "jackal-0.51x0.43",
        },
    )


def _sequence(initial_path: Sequence[Cell], shape: Sequence[int]) -> List[Tuple[str, str, Set[Cell]]]:
    path = list(initial_path)
    safe_indices = [
        max(2, min(len(path) - 3, int(len(path) * fraction)))
        for fraction in (0.30, 0.45, 0.60, 0.72, 0.82)
    ]
    block_1 = {path[safe_indices[0]]}
    block_2 = {path[index] for index in safe_indices[:2]}
    block_5 = {path[index] for index in safe_indices}
    # Local clusters represent people/carts rather than arbitrary graph-edge
    # counts.  They remain deterministic and affect the current route.
    cluster_20: Set[Cell] = set(block_5)
    for row, column in list(block_5):
        for drow, dcolumn in ((-1, 0), (1, 0), (0, -1)):
            cluster_20.add((row + drow, column + dcolumn))
    cluster_20 = set(sorted(cluster_20)[:20])
    barrier_row = int(shape[0]) // 2
    barrier = {(barrier_row, column) for column in range(int(shape[1]))}
    return [
        ("DYN-01", "one_cell_path_affected", block_1),
        ("DYN-02", "two_cell_moving", block_2),
        ("DYN-05", "five_cell_path_affected", block_5),
        ("DYN-20", "twenty_cell_cluster", cluster_20),
        ("DYN-NO-ROUTE", "no_route", barrier),
        ("DYN-RECOVERY", "recovery", block_1),
        ("DYN-CLEAR", "full_recovery", set()),
    ]


def run(output: Path, *, repetitions: int = DEFAULT_REPETITIONS) -> Path:
    output = output.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    if repetitions < 1:
        raise ValueError("repetitions must be positive")
    output.mkdir(parents=True)
    rows: List[Dict[str, Any]] = []
    for repetition in range(1, int(repetitions) + 1):
        roi = _synthetic_roi()
        pure = PersistentCorridorDStar(
            roi, dstar_wall_budget_ms=10_000.0,
            dstar_max_expansions=roi.base_free.size * 2,
        )
        hybrid = PersistentCorridorDStar(
            roi, dstar_wall_budget_ms=20.0,
            dstar_max_expansions=20_000,
        )
        initial = pure.initialize(verify_oracle=False)
        hybrid_initial = hybrid.initialize(verify_oracle=False)
        if not initial.success or initial.path is None:
            raise AssertionError("synthetic Stage-A initial path is unreachable")
        if not hybrid_initial.success:
            raise AssertionError("hybrid arm initial path is unreachable")
        initial_local = [roi.to_local(cell) for cell in initial.path]
        previous_blocked: Set[Cell] = set()
        for snapshot_index, (scenario_id, category, blocked) in enumerate(
            _sequence(initial_local, roi.shape), start=1,
        ):
            global_blocked = [roi.to_global(cell) for cell in blocked]
            result = pure.update(global_blocked, verify_oracle=False)
            changed_count = len(previous_blocked.symmetric_difference(blocked))
            force_cold = (
                changed_count > 2
                or category in {"recovery", "full_recovery"}
                or not hybrid.dstar_ready
            )
            hybrid_result = hybrid.update(
                global_blocked, verify_oracle=False, force_cold_astar=force_cold,
            )
            previous_blocked = set(blocked)
            oracle = deterministic_grid_astar(
                pure.current_free, roi.start_local, roi.goal_local,
            )
            result_local = None if result.path is None else [roi.to_local(cell) for cell in result.path]
            hybrid_local = None if hybrid_result.path is None else [
                roi.to_local(cell) for cell in hybrid_result.path
            ]
            reachable_parity = (result_local is None) == (oracle.path is None)
            cost_error = 0.0 if oracle.path is None else abs(
                _path_cost(result_local) - oracle.cost
            )
            hybrid_reachable_parity = (hybrid_local is None) == (oracle.path is None)
            hybrid_cost_error = 0.0 if oracle.path is None else abs(
                _path_cost(hybrid_local) - oracle.cost
            )
            blocked_absent = not set(result_local or ()).intersection(pure.dynamic_blocked_local)
            hybrid_blocked_absent = not set(hybrid_local or ()).intersection(hybrid.dynamic_blocked_local)
            rows.append({
                "architecture_id": ARCHITECTURE_ID,
                "implementation_revision": IMPLEMENTATION_REVISION,
                "parent_architecture": PARENT_ARCHITECTURE,
                "protocol_version": PROTOCOL_VERSION,
                "experiment_scope": "synthetic_stage_a_preflight_not_production_claim",
                "repetition": repetition,
                "snapshot_index": snapshot_index,
                "scenario_id": scenario_id,
                "category": category,
                "changed_cells": result.changed_cells,
                "blocked_cells": len(blocked),
                "selected_backend": result.selected_backend,
                "dstar_response_ms": result.response_ms,
                "dstar_search_ms": result.dstar_stats.search_time_ms,
                "dstar_expanded_nodes": result.dstar_stats.expanded_nodes,
                "dstar_update_vertex_count": result.dstar_stats.update_vertex_count,
                "dstar_queue_pops": result.dstar_stats.queue_pops,
                "astar_response_ms": oracle.search_time_ms,
                "astar_expanded_nodes": oracle.expanded_nodes,
                "hybrid_response_ms": hybrid_result.response_ms,
                "hybrid_selected_backend": hybrid_result.selected_backend,
                "hybrid_dstar_search_ms": hybrid_result.dstar_stats.search_time_ms,
                "hybrid_expanded_nodes": (
                    hybrid_result.dstar_stats.expanded_nodes
                    if hybrid_result.selected_backend == "persistent_dstar"
                    else int(hybrid_result.fallback_stats.expanded_nodes)
                    if hybrid_result.fallback_stats is not None else 0
                ),
                "hybrid_dstar_ready": hybrid.dstar_ready,
                "hybrid_reachable": hybrid_result.success,
                "hybrid_reachable_parity": hybrid_reachable_parity,
                "hybrid_path_cost_error": hybrid_cost_error,
                "hybrid_blocked_cell_absent": hybrid_blocked_absent,
                "hybrid_partial_dstar_result_returned": hybrid_result.partial_dstar_result_returned,
                "dstar_reachable": result.success,
                "astar_reachable": oracle.path is not None,
                "reachable_parity": reachable_parity,
                "path_cost_error": cost_error,
                "blocked_cell_absent": blocked_absent,
                "partial_dstar_result_returned": result.partial_dstar_result_returned,
                "state_reused": result.state_reused,
                "dstar_ready": result.diagnostics.get("dstar_ready"),
                "state_memory_bytes": result.diagnostics.get("state_memory_bytes"),
                "all_correct": bool(
                    reachable_parity and cost_error <= 1.0e-9 and blocked_absent
                    and not result.partial_dstar_result_returned
                    and hybrid_reachable_parity and hybrid_cost_error <= 1.0e-9
                    and hybrid_blocked_absent
                    and not hybrid_result.partial_dstar_result_returned
                ),
            })

    _write_csv(output / "runs.csv", rows)
    dynamic_rows = [row for row in rows if row["category"] not in {"no_route", "recovery", "full_recovery"}]
    dstar = [float(row["dstar_response_ms"]) for row in dynamic_rows]
    hybrid = [float(row["hybrid_response_ms"]) for row in dynamic_rows]
    astar = [float(row["astar_response_ms"]) for row in dynamic_rows]
    dstar_expanded = [float(row["dstar_expanded_nodes"]) for row in dynamic_rows]
    hybrid_expanded = [float(row["hybrid_expanded_nodes"]) for row in dynamic_rows]
    astar_expanded = [float(row["astar_expanded_nodes"]) for row in dynamic_rows]
    ds, hs, ast = _summary(dstar), _summary(hybrid), _summary(astar)
    de, he, ae = _summary(dstar_expanded), _summary(hybrid_expanded), _summary(astar_expanded)
    p50_reduction = 1.0 - hs["p50"] / ast["p50"] if ast["p50"] > 0 else -math.inf
    p95_ratio = hs["p95"] / ast["p95"] if ast["p95"] > 0 else math.inf
    p99_ratio = hs["p99"] / ast["p99"] if ast["p99"] > 0 else math.inf
    expanded_reduction = 1.0 - he["p50"] / ae["p50"] if ae["p50"] > 0 else -math.inf
    no_route = [row for row in rows if row["category"] == "no_route"]
    recovery = [row for row in rows if row["category"] in {"recovery", "full_recovery"}]
    gates = {
        "scope": "synthetic_stage_a_preflight",
        "correctness_rows": len(rows),
        "correctness_failures": sum(not row["all_correct"] for row in rows),
        "oracle_parity_pass": all(row["all_correct"] for row in rows),
        "dstar_dynamic_response_ms": ds,
        "hybrid_dynamic_response_ms": hs,
        "astar_dynamic_response_ms": ast,
        "dstar_dynamic_expanded_nodes": de,
        "hybrid_dynamic_expanded_nodes": he,
        "astar_dynamic_expanded_nodes": ae,
        "p50_reduction": p50_reduction,
        "p95_ratio": p95_ratio,
        "p99_ratio": p99_ratio,
        "expanded_p50_reduction": expanded_reduction,
        "p50_gate_pass": p50_reduction >= 0.20,
        "p95_gate_pass": p95_ratio <= 1.05,
        "p99_gate_pass": p99_ratio <= 1.10,
        "expanded_gate_pass": expanded_reduction >= 0.50,
        "no_route_pass": bool(no_route) and all(row["all_correct"] and not row["hybrid_reachable"] for row in no_route),
        "recovery_pass": bool(recovery) and all(row["all_correct"] and row["hybrid_reachable"] for row in recovery),
    }
    gates["stage_a_preflight_pass"] = all(bool(gates[key]) for key in (
        "oracle_parity_pass", "p50_gate_pass", "p95_gate_pass", "p99_gate_pass",
        "expanded_gate_pass", "no_route_pass", "recovery_pass",
    ))
    (output / "gate_results.yaml").write_text(
        yaml.safe_dump(gates, sort_keys=False), encoding="utf-8",
    )
    timing_rows = []
    for category in sorted({str(row["category"]) for row in rows}):
        selected = [row for row in rows if row["category"] == category]
        for arm, field in (
            ("pure_persistent_dstar", "dstar_response_ms"),
            ("selective_dstar_with_cold_astar", "hybrid_response_ms"),
            ("cold_grid_astar", "astar_response_ms"),
        ):
            timing_rows.append({"category": category, "arm": arm, **_summary([
                float(row[field]) for row in selected
            ])})
    _write_csv(output / "timing_summary.csv", timing_rows)
    _write_csv(output / "correctness_oracle.csv", [{
        key: row[key] for key in (
            "repetition", "snapshot_index", "scenario_id", "category",
            "reachable_parity", "path_cost_error", "blocked_cell_absent",
            "partial_dstar_result_returned", "hybrid_reachable_parity",
            "hybrid_path_cost_error", "hybrid_blocked_cell_absent",
            "hybrid_partial_dstar_result_returned", "all_correct",
        )
    } for row in rows])

    package_root = Path(__file__).resolve().parents[1]
    source_dir = output / "source_snapshot"
    source_dir.mkdir()
    source_files = sorted(
        list((package_root / "arena_3d_v1").glob("*.py"))
        + list((package_root / "config").glob("*.yaml"))
        + [package_root / "package.xml", package_root / "setup.py", package_root / "setup.cfg"]
    )
    manifest = {}
    for source in source_files:
        destination = source_dir / source.relative_to(package_root)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        manifest[str(source)] = _sha256(source)
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "parent_architecture": PARENT_ARCHITECTURE,
        "explicitly_not_derived_from": "3D-V0",
        "protocol_version": PROTOCOL_VERSION,
        "source_files": manifest,
    }, sort_keys=False), encoding="utf-8")
    command = (
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        f"ros2 run arena_3d_v1 three_d_v1_stage_a --output-dir {output} --repetitions {repetitions}\n"
    )
    (output / "reproduction_command.txt").write_text(command, encoding="utf-8")
    report = [
        "# 3D-V1 L2 Stage-A preflight", "",
        "This is an independent synthetic dynamic-extension preflight, not a PLN-02 static production claim.", "",
        f"- architecture: `{ARCHITECTURE_ID}` / `{IMPLEMENTATION_REVISION}`",
        f"- parent substrate: `{PARENT_ARCHITECTURE}`; explicitly not derived from `3D-V0`",
        f"- correctness: {len(rows) - gates['correctness_failures']}/{len(rows)}",
        f"- D* response P50/P95/P99: {ds['p50']:.6f}/{ds['p95']:.6f}/{ds['p99']:.6f} ms",
        f"- selective D*/A* response P50/P95/P99: {hs['p50']:.6f}/{hs['p95']:.6f}/{hs['p99']:.6f} ms",
        f"- A* response P50/P95/P99: {ast['p50']:.6f}/{ast['p95']:.6f}/{ast['p99']:.6f} ms",
        f"- P50 reduction: {p50_reduction:.2%}",
        f"- P95/P99 ratios: {p95_ratio:.3f}/{p99_ratio:.3f}",
        f"- expanded-node P50 reduction: {expanded_reduction:.2%}",
        f"- preflight gate: **{'PASS' if gates['stage_a_preflight_pass'] else 'FAIL'}**", "",
        "A PASS permits a separately frozen production-map Stage A. It does not by itself permit ROS/Nav2 Stage B.",
    ]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    (output / "verification.yaml").write_text(yaml.safe_dump({
        "required_files_present": True,
        "source_snapshot_count": len(source_files),
        "source_snapshot_hash_match": all(
            _sha256(source_dir / Path(source).relative_to(package_root)) == digest
            for source, digest in manifest.items()
        ),
        "frozen_experiment_modified": False,
        "stage_b": "NOT_RUN_SYNTHETIC_PREFLIGHT_ONLY",
    }, sort_keys=False), encoding="utf-8")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--repetitions", type=int, default=DEFAULT_REPETITIONS)
    args = parser.parse_args()
    print(run(args.output_dir or _default_output(), repetitions=args.repetitions))


if __name__ == "__main__":
    main()
