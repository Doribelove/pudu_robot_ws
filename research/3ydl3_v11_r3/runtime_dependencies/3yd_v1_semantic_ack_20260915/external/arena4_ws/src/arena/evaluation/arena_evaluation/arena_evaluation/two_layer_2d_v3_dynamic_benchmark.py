"""Calibration, held-out L1 evaluation, and soak replay for 2D-V3."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import resource
import shutil
import statistics
import time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import dynamic_cleaning_replay as cleaning
from . import dynamic_incremental_value as dynamic
from . import hybrid_l1_router as hybrid
from . import layered_2d_v1_pipeline as v1
from . import layered_2d_v3_pipeline as v3
from . import two_layer_2d_v1_4x_dynamic_incremental_benchmark as r4
from . import two_layer_2d_v1_dynamic_incremental_benchmark as r3
from .graph_dstar_lite import GraphDStarLite
from .planner_benchmark.map_utils import sha256_file


ROOT = Path("/home/robot/pudu_robot_ws")
MAP_ID = "mentor_map_20260825_005_4x_area"
SEED = 20260903
DEFAULT_WARMUPS = 3
DEFAULT_REPETITIONS = 20
DEFAULT_SOAK_CYCLES = 50
DEFAULT_ROS_DOMAIN_ID = 103

FROZEN_BASELINES = (
    ROOT / "experiments/layered_planner_benchmark/2d_v2_static_mentor_map_005_r0_20260903_154754",
    ROOT / "experiments/layered_planner_benchmark/2d_v2_dynamic_4x_area_r0_20260903_154947",
    ROOT / "experiments/layered_planner_benchmark/2d_v1_dynamic_incremental_value_v1_20260903_134619",
    ROOT / "experiments/layered_planner_benchmark/2d_v1_dynamic_incremental_4x_area_v1_20260903_150321",
    ROOT / "experiments/layered_planner_benchmark/2a_v1_mentor_map_20260825_005_20_r2_roi_pathaudit_v1",
)

BUDGET_CANDIDATES = (
    hybrid.BudgetConfig(0.5, 64, 1024, 4096, 2048, 2, 0.00092, 1, 1),
    hybrid.BudgetConfig(1.0, 128, 2048, 6144, 3072, 5, 0.00230, 1, 1),
    hybrid.BudgetConfig(2.0, 256, 4096, 8192, 4096, 5, 0.00230, 1, 1),
    hybrid.BudgetConfig(4.0, 512, 8192, 12288, 8192, 20, 0.00921, 1, 1),
    hybrid.BudgetConfig(6.0, 768, 12288, 16384, 12288, 20, 0.00921, 2, 1),
    hybrid.BudgetConfig(8.0, 1024, 16384, 24576, 16384, 100, 0.04604, 2, 1),
)


def _default_output(mode: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = {
        "calibrate": "2d_v3_calibration_4x_area_r0_",
        "formal": "2d_v3_dynamic_4x_area_r0_",
        "soak": "2d_v3_cleaning_replay_r0_",
        "ratio": "2d_v3_ratio_break_even_4x_r0_",
    }[mode]
    return ROOT / "experiments/layered_planner_benchmark" / f"{prefix}{stamp}"


def _refuse_nonempty(path: Path) -> None:
    if path.exists() and any(path.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {path}")


def _tree_hash(directory: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(item for item in directory.rglob("*") if item.is_file()):
        digest.update(str(path.relative_to(directory)).encode("utf-8"))
        digest.update(b"\0")
        digest.update(bytes.fromhex(sha256_file(path)))
    return digest.hexdigest()


def _frozen_hashes() -> Dict[str, str]:
    missing = [str(path) for path in FROZEN_BASELINES if not path.is_dir()]
    if missing:
        raise FileNotFoundError(f"missing frozen baselines: {missing}")
    return {str(path): _tree_hash(path) for path in FROZEN_BASELINES}


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    values = list(rows)
    keys: List[str] = []
    for row in values:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys or ["status"])
        writer.writeheader()
        for row in values:
            writer.writerow({key: json.dumps(value, sort_keys=True) if isinstance(value, (list, dict, tuple)) else value
                             for key, value in row.items()})


def _percentile(values: Sequence[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=float), q * 100.0)) if values else float("nan")


def _summary(values: Sequence[float]) -> Dict[str, Any]:
    return {
        "count": len(values), "p50": _percentile(values, 0.50),
        "p95": _percentile(values, 0.95), "p99": _percentile(values, 0.99),
        "mean": statistics.fmean(values) if values else float("nan"),
        "max": max(values) if values else float("nan"),
    }


def _load_inputs() -> Tuple[Any, Any, Any, Any, Any, Dict[str, Any], Dict[str, Any], Dict[str, Tuple[Tuple[int, int], ...]], List[cleaning.CleaningWorkload]]:
    ctx, queries, task_metadata, artifact, topology_info = r4._load_4x_inputs()
    reference = r4._load_static_reference()
    graph_view = v1.build_static_topology_view(artifact)
    graph_view.metadata["topology_cache_key"] = topology_info["topology_cache_key"]
    query_graphs, endpoint_pipeline = r4._build_query_graphs(
        graph_view, topology_info, ctx, queries, reference,
    )
    edge_cells = r3._edge_cells(graph_view)
    exclusive, any_witness, _reverse = r3._witness_maps(edge_cells)
    workloads = cleaning.build_workloads(
        query_graphs, exclusive, any_witness, seed=SEED,
        minimum_cut=r3._minimum_static_cut,
    )
    return (ctx, queries, task_metadata, artifact, topology_info, query_graphs,
            reference, edge_cells, workloads)


def _run_episode(
    graph: Any, workload: cleaning.CleaningWorkload, payloads: Sequence[str],
    edge_cells: Mapping[str, Sequence[Tuple[int, int]]], *, map_version: str,
    map_shape: Sequence[int], budget: hybrid.BudgetConfig, run_mode: str,
    repetition: int,
) -> List[Dict[str, Any]]:
    overlays = {
        arm: dynamic.DynamicEdgeOverlay(edge_cells, map_version=map_version, map_shape=map_shape)
        for arm in hybrid.ARMS
    }
    routers = {
        arm: hybrid.HybridL1Router(
            arm, graph.template, topology_edge_count=len(edge_cells), budget=budget,
        ) for arm in hybrid.ARMS
    }
    rows: List[Dict[str, Any]] = []
    initial_static_hash = row_static_hash(graph)
    oracle_cache: Dict[str, dynamic.GraphAStarResult] = {}
    for snapshot_index, payload in enumerate(payloads):
        per_snapshot: List[Dict[str, Any]] = []
        order = list(hybrid.ARMS)
        offset = (snapshot_index + repetition) % len(order)
        order = order[offset:] + order[:offset]
        for arm in order:
            prepared = overlays[arm].consume_json(payload)
            status_hash = dynamic.stable_hash(prepared.statuses)
            cpu_started_ns = time.process_time_ns()
            result = routers[arm].step(prepared)
            response_cpu_ms = (time.process_time_ns() - cpu_started_ns) / 1.0e6
            # Response has already become available. Service the explicitly
            # accounted low-priority state resync before the next snapshot.
            resync = routers[arm].service_resync()
            result.update(resync)
            result["full_l1_accounted_ms"] = float(result.get("response_l1_ms", 0.0)) + float(resync.get("resync_cpu_ms", 0.0))
            oracle = oracle_cache.get(prepared.input_hash)
            if oracle is None:
                oracle = dynamic.deterministic_graph_astar(graph.template, prepared.statuses)
                oracle_cache[prepared.input_hash] = oracle
            reachable = bool(result.get("reachable"))
            oracle_reachable = oracle.node_path is not None
            if reachable and oracle_reachable:
                cost_error = abs(float(result["path_cost"]) - float(oracle.cost))
            else:
                cost_error = 0.0
            blocked = {
                edge for edge, status in prepared.statuses.items()
                if status in {GraphDStarLite.BLOCKED, GraphDStarLite.RECOVERING}
            }
            row = {
                "architecture_id": v3.ARCHITECTURE_ID,
                "implementation_revision": v3.IMPLEMENTATION_REVISION,
                "parent_architecture": v3.PARENT_ARCHITECTURE,
                "protocol_version": v3.PROTOCOL_VERSION,
                "map_id": MAP_ID, "scenario_id": workload.scenario_id,
                "split": workload.split, "category": workload.category,
                "scale_family": workload.scale_family,
                "requested_changed_edges": workload.requested_changed_edges,
                "requested_changed_ratio": workload.requested_changed_ratio,
                "query_id": workload.query_id, "run_mode": run_mode,
                "repetition": repetition, "snapshot_index": snapshot_index,
                "snapshot_id": prepared.snapshot.snapshot_id,
                "snapshot_hash": prepared.snapshot.snapshot_hash,
                "arm": arm, "snapshot_accepted": prepared.accepted,
                "snapshot_rejection_reason": prepared.rejection_reason,
                "snapshot_parse_ms": prepared.parse_time_ms,
                "changed_cell_to_edge_mapping_ms": prepared.mapping_time_ms,
                "edge_state_transition_ms": prepared.state_transition_time_ms,
                "changed_cells_count": len(prepared.changed_cells),
                "changed_edge_count": len(prepared.changed_edges),
                "changed_edge_ratio": len(prepared.changed_edges) / max(1, len(edge_cells)),
                "changed_edge_ids": list(prepared.changed_edges),
                "edge_status_hash": status_hash,
                "topology_static_hash": graph.template.static_hash,
                "input_hash": prepared.input_hash,
                "arm_response_cpu_ms": response_cpu_ms,
                "process_peak_rss_bytes": int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024,
                "oracle_reachable": oracle_reachable,
                "oracle_failure_code": "" if oracle_reachable else "L1_NO_ROUTE",
                "oracle_path_cost": float(oracle.cost),
                "oracle_path_edge_ids": list(oracle.edge_path),
                "oracle_expanded_nodes": int(oracle.expanded_nodes),
                "oracle_search_ms_diagnostic_not_timed_in_arm": float(oracle.search_time_ms),
                "reachable_parity": reachable == oracle_reachable,
                "failure_code_parity": str(result.get("failure_code", "")) == ("" if oracle_reachable else "L1_NO_ROUTE"),
                "path_cost_error": cost_error,
                "path_cost_parity": cost_error == 0.0,
                "route_edge_ids_equal": list(result.get("path_edge_ids", ())) == list(oracle.edge_path),
                "blocked_edge_absent": not blocked.intersection(result.get("path_edge_ids", ())),
                "static_topology_immutable": graph.template.static_hash == initial_static_hash,
                **result,
            }
            row["all_correct"] = all(bool(row[field]) for field in (
                "reachable_parity", "failure_code_parity", "path_cost_parity",
                "route_edge_ids_equal", "blocked_edge_absent", "static_topology_immutable",
            )) and not bool(row.get("partial_dstar_result_returned"))
            per_snapshot.append(row)
            rows.append(row)
        if len({row["input_hash"] for row in per_snapshot}) != 1:
            raise AssertionError("three-arm input hash mismatch")
        if len({row["edge_status_hash"] for row in per_snapshot}) != 1:
            raise AssertionError("three-arm edge-state mismatch")
    return rows


def row_static_hash(graph: Any) -> str:
    # The template is frozen and no arm receives a mutable reference to its
    # edge list; recomputing from a temporary D* catches accidental mutation.
    return dynamic.GraphTemplate.from_dstar(graph.template.new_dstar({})).static_hash


def _summaries(rows: Sequence[Mapping[str, Any]]) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    measured = [row for row in rows if row["run_mode"] == "measured"]
    phases = (
        "snapshot_parse_ms", "changed_cell_to_edge_mapping_ms",
        "edge_state_transition_ms", "update_edges_ms", "dstar_attempt_ms",
        "fallback_astar_ms", "resync_cpu_ms", "compute_shortest_path_ms",
        "route_extraction_ms", "response_l1_ms", "full_l1_accounted_ms",
        "arm_response_cpu_ms",
    )
    timing: List[Dict[str, Any]] = []
    groups = {
        "s0_initial": lambda row: bool(row.get("initial_plan")),
        "dynamic_all": lambda row: bool(row.get("dynamic_update")),
        "dynamic_l1_invoked": lambda row: bool(row.get("dynamic_update")) and bool(row.get("l1_invoked")),
        "scheduler_skip": lambda row: bool(row.get("scheduler_skip")),
        "no_route": lambda row: not bool(row.get("reachable")),
        "recovery": lambda row: row["category"] == "obstacle_disappearance_recovery" and bool(row.get("dynamic_update")),
    }
    for group_name, predicate in groups.items():
        for arm in hybrid.ARMS:
            selected = [row for row in measured if row["arm"] == arm and predicate(row)]
            for phase in phases:
                timing.append({"group": group_name, "arm": arm, "metric": phase,
                               **_summary([float(row.get(phase) or 0.0) for row in selected])})
    expanded = []
    for group_name, predicate in groups.items():
        for arm in hybrid.ARMS:
            selected = [row for row in measured if row["arm"] == arm and predicate(row)]
            expanded.append({"group": group_name, "arm": arm, "metric": "expanded_nodes",
                             **_summary([float(row.get("expanded_nodes") or 0.0) for row in selected])})
    return timing, expanded


def _correctness(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    fields = (
        "scenario_id", "query_id", "category", "repetition", "snapshot_index",
        "snapshot_id", "arm", "input_hash", "edge_status_hash", "reachable",
        "oracle_reachable", "failure_code", "oracle_failure_code", "path_cost",
        "oracle_path_cost", "path_cost_error", "reachable_parity",
        "failure_code_parity", "path_cost_parity", "route_edge_ids_equal",
        "blocked_edge_absent", "partial_dstar_result_returned", "all_correct",
    )
    return [{field: row.get(field) for field in fields} for row in rows if row["run_mode"] == "measured"]


def _gates(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    measured = [row for row in rows if row["run_mode"] == "measured"]
    correct = all(bool(row["all_correct"]) for row in measured)
    selected = [row for row in measured if bool(row.get("dynamic_update")) and bool(row.get("l1_invoked"))]
    astar = [float(row["response_l1_ms"]) for row in selected if row["arm"] == hybrid.COLD_GRAPH_ASTAR]
    mixed = [float(row["response_l1_ms"]) for row in selected if row["arm"] == hybrid.HYBRID]
    astar_s, mixed_s = _summary(astar), _summary(mixed)
    p50_reduction = 1.0 - mixed_s["p50"] / astar_s["p50"] if astar_s["p50"] > 0 else -math.inf
    p95_ratio = mixed_s["p95"] / astar_s["p95"] if astar_s["p95"] > 0 else math.inf
    p99_ratio = mixed_s["p99"] / astar_s["p99"] if astar_s["p99"] > 0 else math.inf
    hybrid_rows = [row for row in measured if row["arm"] == hybrid.HYBRID]
    no_route_correct = all(row["all_correct"] for row in hybrid_rows if not row["oracle_reachable"])
    recovery_rows = [row for row in hybrid_rows if row["category"] == "obstacle_disappearance_recovery"]
    recovery_correct = bool(recovery_rows) and all(row["all_correct"] for row in recovery_rows)
    return {
        "correctness_rows": len(measured),
        "correctness_failures": sum(not bool(row["all_correct"]) for row in measured),
        "oracle_parity_pass": correct,
        "maximum_path_cost_error": max((float(row["path_cost_error"]) for row in measured), default=0.0),
        "blocked_edges_in_returned_paths": sum(not bool(row["blocked_edge_absent"]) for row in measured),
        "partial_dstar_results_returned": sum(bool(row.get("partial_dstar_result_returned")) for row in measured),
        "no_route_pass": no_route_correct, "recovery_pass": recovery_correct,
        "graph_astar_response_l1": astar_s, "hybrid_response_l1": mixed_s,
        "hybrid_p50_reduction": p50_reduction,
        "hybrid_p95_ratio": p95_ratio, "hybrid_p99_ratio": p99_ratio,
        "p50_gate_pass": p50_reduction >= 0.10,
        "p95_gate_pass": p95_ratio <= 1.05,
        "p99_gate_pass": p99_ratio <= 1.10,
        "stage4_pass": bool(correct and no_route_correct and recovery_correct
                            and p50_reduction >= 0.10 and p95_ratio <= 1.05 and p99_ratio <= 1.10),
        "response_latency_excludes_resync": True,
        "resync_cpu_fully_reported_separately": True,
        "hybrid_accounted_compute": _summary([
            float(row["full_l1_accounted_ms"]) for row in selected if row["arm"] == hybrid.HYBRID
        ]),
    }


def _break_even(rows: Sequence[Mapping[str, Any]], *, ratio: bool) -> List[Dict[str, Any]]:
    measured = [
        row for row in rows
        if row["run_mode"] == "measured" and row.get("l1_invoked")
        and row.get("dynamic_update")
    ]
    groups: Dict[Any, List[Mapping[str, Any]]] = {}
    for row in measured:
        key = (round(float(row.get("requested_changed_ratio") or row["changed_edge_ratio"]), 6)
               if ratio else int(row["changed_edge_count"]))
        groups.setdefault(key, []).append(row)
    result = []
    for key, values in sorted(groups.items()):
        astar = [float(row["response_l1_ms"]) for row in values if row["arm"] == hybrid.COLD_GRAPH_ASTAR]
        dstar = [float(row["response_l1_ms"]) for row in values if row["arm"] == hybrid.PURE_PERSISTENT_DSTAR]
        mixed = [float(row["response_l1_ms"]) for row in values if row["arm"] == hybrid.HYBRID]
        astar_expanded = [float(row["expanded_nodes"]) for row in values if row["arm"] == hybrid.COLD_GRAPH_ASTAR]
        dstar_expanded = [float(row["expanded_nodes"]) for row in values if row["arm"] == hybrid.PURE_PERSISTENT_DSTAR]
        if not astar or not dstar or not mixed:
            continue
        result.append({
            "changed_edge_ratio" if ratio else "changed_edge_count": key,
            "sample_count_per_arm": min(len(astar), len(dstar), len(mixed)),
            "graph_astar_p50_ms": _percentile(astar, 0.50),
            "pure_dstar_p50_ms": _percentile(dstar, 0.50),
            "hybrid_p50_ms": _percentile(mixed, 0.50),
            "dstar_over_astar_wall_ratio": _percentile(dstar, 0.50) / _percentile(astar, 0.50),
            "hybrid_over_astar_wall_ratio": _percentile(mixed, 0.50) / _percentile(astar, 0.50),
            "dstar_over_astar_expanded_ratio": _percentile(dstar_expanded, 0.50) / max(1.0, _percentile(astar_expanded, 0.50)),
        })
    return result


def _workload_characterization(rows: Sequence[Mapping[str, Any]], workloads: Sequence[cleaning.CleaningWorkload]) -> List[Dict[str, Any]]:
    measured = [row for row in rows if row["run_mode"] == "measured" and row["arm"] == hybrid.COLD_GRAPH_ASTAR]
    result = []
    for workload in workloads:
        selected = [row for row in measured if row["scenario_id"] == workload.scenario_id]
        result.append({
            **{key: value for key, value in asdict(workload).items() if key not in {"witness_cells"}},
            "snapshot_count": len(selected),
            "changed_cells": _summary([float(row["changed_cells_count"]) for row in selected]),
            "changed_edges": _summary([float(row["changed_edge_count"]) for row in selected]),
            "changed_edge_ratio": _summary([float(row["changed_edge_ratio"]) for row in selected]),
            "path_affected_rate": statistics.fmean(float(bool(row.get("route_intersection"))) for row in selected) if selected else 0.0,
            "no_route_rate": statistics.fmean(float(not bool(row.get("oracle_reachable"))) for row in selected) if selected else 0.0,
        })
    return result


def _calibrate(rows_by_budget: Sequence[Tuple[hybrid.BudgetConfig, List[Dict[str, Any]]]]) -> Tuple[hybrid.BudgetConfig, List[Dict[str, Any]]]:
    candidates = []
    for budget, rows in rows_by_budget:
        gates = _gates(rows)
        hybrid_rows = [row for row in rows if row["run_mode"] == "measured" and row["arm"] == hybrid.HYBRID]
        candidate = {
            **budget.as_dict(), **gates,
            "dstar_attempt_rate": statistics.fmean(float(row.get("selected_policy") == hybrid.PURE_PERSISTENT_DSTAR) for row in hybrid_rows),
            "budget_fallback_rate": statistics.fmean(float(bool(row.get("budget_triggered"))) for row in hybrid_rows),
            "resync_cpu_total_ms": sum(float(row.get("resync_cpu_ms") or 0.0) for row in hybrid_rows),
        }
        candidates.append(candidate)
    passing = [item for item in candidates if item["stage4_pass"]]
    if passing:
        chosen_row = max(passing, key=lambda item: (item["hybrid_p50_reduction"], -item["hybrid_p95_ratio"], -item["wall_ms"]))
    else:
        chosen_row = min(candidates, key=lambda item: (item["hybrid_p95_ratio"], item["hybrid_p99_ratio"], -item["hybrid_p50_reduction"]))
    chosen = next(budget for budget, _rows in rows_by_budget if budget.wall_ms == chosen_row["wall_ms"])
    for item in candidates:
        item["selected"] = item["wall_ms"] == chosen.wall_ms
        item["selection_rule"] = "highest P50 reduction among all-gate candidates; otherwise best P95/P99"
    return chosen, candidates


def _source_snapshot(output: Path, event_paths: Sequence[Path]) -> Dict[str, Any]:
    source_dir = output / "source_snapshot"
    source_dir.mkdir()
    files = [
        Path(__file__).resolve(), Path(cleaning.__file__).resolve(),
        Path(hybrid.__file__).resolve(), Path(v3.__file__).resolve(),
        Path(dynamic.__file__).resolve(), Path(dynamic.__file__).resolve().with_name("graph_dstar_lite.py"),
        Path(r4.__file__).resolve(), Path(r3.__file__).resolve(),
        Path(__file__).resolve().parents[1] / "config/two_layer_2d_v3_r0_hybrid.yaml",
        Path(__file__).resolve().parents[1] / "test/test_two_layer_2d_v3_dynamic_benchmark.py",
        *event_paths,
    ]
    records = []
    for index, source in enumerate(files):
        if not source.is_file():
            raise FileNotFoundError(source)
        target = source_dir / f"{index:03d}_{source.name}"
        shutil.copyfile(source, target)
        records.append({"source": str(source), "snapshot": str(target.relative_to(output)),
                        "sha256": sha256_file(target), "bytes": target.stat().st_size})
    payload = {"schema_version": 1, "files": records,
               "combined_hash": dynamic.stable_hash([[row["snapshot"], row["sha256"]] for row in records])}
    (output / "source_snapshot_manifest.yaml").write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return payload


REQUIRED = (
    "final_report.md", "protocol.yaml", "manifest.yaml", "verification.yaml",
    "runs.csv", "per_scenario_results.csv", "phase_timing_summary.csv",
    "correctness_oracle.csv", "selector_decisions.csv", "dstar_queue_diagnostics.csv",
    "fallback_resync.csv", "break_even_curve_absolute.csv",
    "break_even_curve_ratio.csv", "cache_memory_timeline.csv",
    "workload_characterization.csv", "source_snapshot_manifest.yaml",
    "stdout.log", "stderr.log", "reproduction_command.txt",
)


def _validate(output: Path) -> Dict[str, Any]:
    missing = [name for name in REQUIRED if not (output / name).is_file()]
    bad_sources = []
    manifest = yaml.safe_load((output / "source_snapshot_manifest.yaml").read_text()) if (output / "source_snapshot_manifest.yaml").is_file() else {}
    for row in manifest.get("files", []):
        path = output / row["snapshot"]
        if not path.is_file() or sha256_file(path) != row["sha256"]:
            bad_sources.append(row["snapshot"])
    return {"required_files": len(REQUIRED), "missing": missing,
            "bad_source_hashes": bad_sources, "passed": not missing and not bad_sources}


def _report(output: Path, mode: str, gates: Mapping[str, Any], budget: hybrid.BudgetConfig,
            rows: Sequence[Mapping[str, Any]], stage6_status: str) -> None:
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    hrows = [row for row in measured if row.get("arm") == hybrid.HYBRID]
    l1 = [row for row in hrows if row.get("l1_invoked")]
    attempts = [row for row in hrows if row.get("selected_policy") == hybrid.PURE_PERSISTENT_DSTAR]
    fallback = [row for row in hrows if row.get("actual_algorithm") == hybrid.COLD_GRAPH_ASTAR]
    scheduler = [row for row in hrows if row.get("scheduler_skip")]
    lines = [
        "# 2D-V3 hybrid tail-bounded dynamic experiment", "",
        f"- Mode: `{mode}`; workload: **realistic synthetic cleaning workload** (not real cleaning data).",
        f"- Stage-4 gate: **{gates.get('stage4_pass', False)}**; Stage 6: `{stage6_status}`.",
        f"- Correctness: {gates.get('correctness_rows', 0) - gates.get('correctness_failures', 0)}/{gates.get('correctness_rows', 0)}; max cost error {gates.get('maximum_path_cost_error', 0)}.",
        f"- Frozen budget: wall={budget.wall_ms} ms, pop={budget.max_queue_pops}, update_vertex={budget.max_update_vertex}, OPEN={budget.max_open_size}, inconsistent={budget.max_inconsistent_states}.",
        f"- Graph A* response P50/P95/P99: {gates.get('graph_astar_response_l1', {}).get('p50', float('nan')):.4f}/{gates.get('graph_astar_response_l1', {}).get('p95', float('nan')):.4f}/{gates.get('graph_astar_response_l1', {}).get('p99', float('nan')):.4f} ms.",
        f"- Hybrid response P50/P95/P99: {gates.get('hybrid_response_l1', {}).get('p50', float('nan')):.4f}/{gates.get('hybrid_response_l1', {}).get('p95', float('nan')):.4f}/{gates.get('hybrid_response_l1', {}).get('p99', float('nan')):.4f} ms.",
        f"- Hybrid P50 reduction={100*float(gates.get('hybrid_p50_reduction', 0)):.2f}%; P95/P99 ratios={float(gates.get('hybrid_p95_ratio', float('nan'))):.3f}/{float(gates.get('hybrid_p99_ratio', float('nan'))):.3f}.",
        f"- Scheduler skip rate={len(scheduler)/max(1,len(hrows)):.3f}; bounded-D* attempt rate={len(attempts)/max(1,len(l1)):.3f}; A* fallback/direct rate={len(fallback)/max(1,len(l1)):.3f}.",
        f"- Resync CPU total={sum(float(row.get('resync_cpu_ms') or 0.0) for row in hrows):.3f} ms; it is excluded from route-response latency but included in accounted compute and CPU reports.",
        "", "## Attribution", "",
        "- Route-unaffected reuse is scheduler value and is never attributed to D*.",
        "- D* value is measured only on L1-invoked rows against the identically filtered cold Graph A* arm.",
        "- ROI/ACK, 48 bins, Smac and PathAudit are unchanged V2 substrate capabilities and are not attributed to D*.",
        "", "## Reproduction", "", "```bash",
        (output / "reproduction_command.txt").read_text().strip(), "```", "",
    ]
    (output / "final_report.md").write_text("\n".join(lines), encoding="utf-8")


def run(
    output: Path, *, mode: str, frozen_budget_path: Optional[Path] = None,
    warmups: int = DEFAULT_WARMUPS, repetitions: int = DEFAULT_REPETITIONS,
    soak_cycles: int = DEFAULT_SOAK_CYCLES,
) -> Path:
    output = output.resolve(); _refuse_nonempty(output)
    if mode in {"formal", "ratio"} and (warmups < 3 or repetitions < 20):
        raise ValueError("formal held-out run requires >=3 warmups and >=20 repetitions")
    frozen_before = _frozen_hashes()
    (ctx, queries, task_metadata, artifact, topology_info, query_graphs,
     reference, edge_cells, workloads) = _load_inputs()
    if mode == "ratio":
        exclusive, _any_witness, _reverse = r3._witness_maps(edge_cells)
        chosen_workloads = cleaning.build_ratio_matched_workloads(
            query_graphs, exclusive, seed=SEED, topology_edge_count=len(edge_cells),
            select_alternate=r3._select_alternate_edge,
            blocked_oracle=r3._blocked_oracle,
        )
    else:
        chosen_workloads = [item for item in workloads if item.split == ("calibration" if mode == "calibrate" else "held_out")]
    output.mkdir(parents=True)
    (output / "stdout.log").write_text("2D-V3 runner writes structured results to this directory.\n", encoding="utf-8")
    (output / "stderr.log").write_text("", encoding="utf-8")
    events = output / "dynamic_event_streams"; events.mkdir()
    payloads: Dict[str, List[str]] = {}; event_paths = []
    for workload in chosen_workloads:
        path, values = cleaning.write_event_stream(
            events, workload, map_version=ctx.map_sha256,
            map_shape=artifact.free_mask.shape, seed=SEED,
        )
        event_paths.append(path); payloads[workload.scenario_id] = values
    rows: List[Dict[str, Any]] = []
    calibration_rows: List[Dict[str, Any]] = []
    if mode == "calibrate":
        runs_by_budget = []
        for candidate_index, budget in enumerate(BUDGET_CANDIDATES):
            candidate_rows: List[Dict[str, Any]] = []
            for workload in chosen_workloads:
                candidate_rows.extend(_run_episode(
                    query_graphs[workload.query_id], workload, payloads[workload.scenario_id],
                    edge_cells, map_version=ctx.map_sha256, map_shape=artifact.free_mask.shape,
                    budget=budget, run_mode="measured", repetition=candidate_index + 1,
                ))
            for row in candidate_rows:
                row["calibration_budget_wall_ms"] = budget.wall_ms
                row["calibration_budget_hash"] = dynamic.stable_hash(budget.as_dict())
            runs_by_budget.append((budget, candidate_rows))
            rows.extend(candidate_rows)
        budget, calibration_rows = _calibrate(runs_by_budget)
        (output / "frozen_budget.yaml").write_text(yaml.safe_dump({
            "architecture_id": v3.ARCHITECTURE_ID,
            "implementation_revision": v3.IMPLEMENTATION_REVISION,
            "calibration_output": str(output), "budget": budget.as_dict(),
            "selection_candidates": calibration_rows,
        }, sort_keys=False), encoding="utf-8")
        _write_csv(output / "calibration_candidates.csv", calibration_rows)
        analysis_rows = [
            row for row in rows
            if float(row.get("calibration_budget_wall_ms", -1.0)) == budget.wall_ms
        ]
    else:
        if frozen_budget_path is None:
            raise ValueError("formal/soak mode requires --frozen-budget")
        frozen_payload = yaml.safe_load(frozen_budget_path.read_text()) or {}
        budget = hybrid.BudgetConfig(**dict(frozen_payload["budget"]))
        iteration_plan = ((("warmup", warmups), ("measured", repetitions))
                          if mode in {"formal", "ratio"} else (("measured", soak_cycles),))
        for run_mode, count in iteration_plan:
            for repetition in range(1, count + 1):
                for workload in chosen_workloads:
                    rows.extend(_run_episode(
                        query_graphs[workload.query_id], workload, payloads[workload.scenario_id],
                        edge_cells, map_version=ctx.map_sha256, map_shape=artifact.free_mask.shape,
                        budget=budget, run_mode=run_mode, repetition=repetition,
                    ))
        analysis_rows = rows
    gates = _gates(analysis_rows)
    timing, expanded = _summaries(analysis_rows)
    correctness = _correctness(rows)
    measured = [row for row in analysis_rows if row["run_mode"] == "measured"]
    per_scenario = []
    for scenario_id in sorted({row["scenario_id"] for row in measured}):
        values = [row for row in measured if row["scenario_id"] == scenario_id and row["arm"] == hybrid.HYBRID]
        per_scenario.append({
            "scenario_id": scenario_id, "query_id": values[0]["query_id"],
            "category": values[0]["category"], "correct": all(row["all_correct"] for row in values),
            "response_l1": _summary([float(row["response_l1_ms"]) for row in values if row.get("l1_invoked")]),
            "scheduler_skip_rate": statistics.fmean(float(row.get("scheduler_skip")) for row in values),
            "fallback_rate": statistics.fmean(float(row.get("actual_algorithm") == hybrid.COLD_GRAPH_ASTAR) for row in values),
        })
    _write_csv(output / "runs.csv", rows)
    _write_csv(output / "per_scenario_results.csv", per_scenario)
    _write_csv(output / "phase_timing_summary.csv", timing)
    _write_csv(output / "expanded_nodes_summary.csv", expanded)
    _write_csv(output / "correctness_oracle.csv", correctness)
    _write_csv(output / "selector_decisions.csv", [{key: row.get(key) for key in (
        "scenario_id", "category", "repetition", "snapshot_index", "snapshot_id",
        "arm", "changed_edge_count", "changed_edge_ratio", "route_intersection",
        "corridor_intersection", "critical_bridge_intersection", "open_size_before",
        "inconsistent_states_before", "selected_policy", "actual_algorithm",
        "selector_reason", "scheduler_skip", "budget_triggered", "budget_reason",
    )} for row in measured])
    _write_csv(output / "dstar_queue_diagnostics.csv", [{key: row.get(key) for key in (
        "scenario_id", "repetition", "snapshot_index", "arm", "expanded_nodes",
        "generated_nodes", "queue_pushes", "queue_pops", "stale_queue_entries",
        "open_peak", "g_changed_nodes", "rhs_changed_nodes",
        "predecessor_propagations", "update_vertex_count", "state_memory_bytes",
    )} for row in measured if row["arm"] != hybrid.COLD_GRAPH_ASTAR])
    _write_csv(output / "fallback_resync.csv", [{key: row.get(key) for key in (
        "scenario_id", "repetition", "snapshot_index", "selector_reason",
        "budget_triggered", "budget_reason", "dstar_attempt_ms", "fallback_astar_ms",
        "resync_ran", "resync_ms", "resync_cpu_ms", "resync_snapshot_id",
        "dstar_ready_after", "partial_dstar_result_returned",
    )} for row in measured if row["arm"] == hybrid.HYBRID])
    _write_csv(output / "break_even_curve_absolute.csv", _break_even(analysis_rows, ratio=False))
    _write_csv(output / "break_even_curve_ratio.csv", _break_even(analysis_rows, ratio=True))
    _write_csv(output / "workload_characterization.csv", _workload_characterization(analysis_rows, chosen_workloads))
    _write_csv(output / "cache_memory_timeline.csv", [{
        "scenario_id": row["scenario_id"], "repetition": row["repetition"],
        "snapshot_index": row["snapshot_index"], "arm": row["arm"],
        "dstar_state_memory_bytes": row.get("state_memory_bytes", 0),
        "process_peak_rss_bytes": row.get("process_peak_rss_bytes", 0),
        "route_mask_cache_status": "not_materialized_in_pure_l1_stage",
        "background_resync_cpu_ms": row.get("resync_cpu_ms", 0.0),
    } for row in measured])
    memory_rows = [
        {"arm": arm, **_summary([float(row.get("state_memory_bytes") or 0.0) for row in measured if row["arm"] == arm])}
        for arm in hybrid.ARMS
    ]
    memory_rows.append({
        "arm": "benchmark_process_peak_rss",
        **_summary([float(row.get("process_peak_rss_bytes") or 0.0) for row in measured]),
        "note": "includes three isolated cell-to-edge indexes and oracle harness; not algorithm state",
    })
    _write_csv(output / "memory_summary.csv", memory_rows)
    _write_csv(output / "workload_source_audit.csv", [{
        "candidate": "arena simulation example pedestrian/factory scenarios",
        "aligned_map": False, "real_cleaning_trace": False,
        "decision": "rejected_as_noncompliant_reference_only",
    }, {
        "candidate": "workspace rosbag/db3 search", "aligned_map": False,
        "real_cleaning_trace": False, "decision": "no_compliant_trace_found",
    }])
    reproduction = (
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        f"ROS_DOMAIN_ID={DEFAULT_ROS_DOMAIN_ID} ros2 run arena_evaluation two_layer_2d_v3_dynamic_benchmark "
        f"--mode {mode} --output-dir /tmp/REPLACE_WITH_NEW_WRITE_ONCE_DIR"
        + ("" if mode == "calibrate" else f" --frozen-budget {frozen_budget_path}")
        + f" --warmups {warmups} --repetitions {repetitions} --soak-cycles {soak_cycles}\n"
    )
    (output / "reproduction_command.txt").write_text(reproduction, encoding="utf-8")
    source_manifest = _source_snapshot(output, event_paths)
    frozen_after = _frozen_hashes()
    if frozen_after != frozen_before:
        raise AssertionError("frozen baseline changed during V3 experiment")
    stage6_status = "ELIGIBLE_AFTER_STATIC_GATE" if gates["stage4_pass"] and mode == "formal" else "NOT_RUN_STAGE4_FAILED" if mode == "formal" else "NOT_APPLICABLE"
    protocol = {
        "architecture_id": v3.ARCHITECTURE_ID,
        "implementation_revision": v3.IMPLEMENTATION_REVISION,
        "parent_architecture": v3.PARENT_ARCHITECTURE,
        "protocol_version": v3.PROTOCOL_VERSION,
        "mode": mode, "map_id": MAP_ID, "workload_source": "realistic_synthetic_cleaning_workload",
        "real_cleaning_data_available": False,
        "arms": list(hybrid.ARMS), "budget": budget.as_dict(),
        "calibration_and_held_out_separated": True,
        "dynamic_state_machine": ["AVAILABLE", "BLOCKED_PENDING", "BLOCKED", "RECOVERING", "AVAILABLE"],
        "recovering_cost": "INF", "blocked_cost": "INF",
        "stage4_gates": {"p50_reduction": 0.10, "p95_ratio_max": 1.05, "p99_ratio_max": 1.10,
                         "oracle_parity": 1.0, "max_path_cost_error": 0.0},
        "result": gates, "stage6_status": stage6_status,
    }
    (output / "protocol.yaml").write_text(yaml.safe_dump(protocol, sort_keys=False), encoding="utf-8")
    manifest = {
        "experiment_id": output.name, "created_at": datetime.now().astimezone().isoformat(),
        "architecture_id": v3.ARCHITECTURE_ID, "map_id": MAP_ID,
        "map_hash": ctx.map_sha256, "map_shape": list(artifact.free_mask.shape),
        "topology_hash": topology_info["topology_cache_key"],
        "topology_nodes": len(artifact.graph.nodes), "topology_edges": len(artifact.graph.edges),
        "workloads": len(chosen_workloads), "warmups": warmups,
        "repetitions": repetitions if mode != "soak" else soak_cycles,
        "seed": SEED, "frozen_baseline_hashes_before": frozen_before,
        "frozen_baseline_hashes_after": frozen_after,
        "source_snapshot_combined_hash": source_manifest["combined_hash"],
        "task_metadata": task_metadata,
    }
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    # Create the report before machine validation because it is itself required.
    _report(output, mode, gates, budget, analysis_rows, stage6_status)
    (output / "verification.yaml").write_text("pending: true\n", encoding="utf-8")
    verification = _validate(output)
    verification.update({"frozen_baselines_unchanged": frozen_before == frozen_after,
                         "gates": gates})
    (output / "verification.yaml").write_text(yaml.safe_dump(verification, sort_keys=False), encoding="utf-8")
    if not verification["passed"]:
        raise AssertionError(f"formal artifact validation failed: {verification}")
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("calibrate", "formal", "soak", "ratio"), default="formal")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--frozen-budget", type=Path)
    parser.add_argument("--warmups", type=int, default=DEFAULT_WARMUPS)
    parser.add_argument("--repetitions", type=int, default=DEFAULT_REPETITIONS)
    parser.add_argument("--soak-cycles", type=int, default=DEFAULT_SOAK_CYCLES)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    output = args.output_dir or _default_output(args.mode)
    run(output, mode=args.mode, frozen_budget_path=args.frozen_budget,
        warmups=args.warmups, repetitions=args.repetitions,
        soak_cycles=args.soak_cycles)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
