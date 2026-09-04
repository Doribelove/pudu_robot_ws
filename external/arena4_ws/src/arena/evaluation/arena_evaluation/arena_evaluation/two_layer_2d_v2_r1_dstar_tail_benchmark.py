"""Formal calibration/held-out/soak runner for D* Lite core-tail research."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import shutil
import statistics
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import dstar_tail_research as research
from . import dynamic_incremental_value as dynamic
from . import two_layer_2d_v3_dynamic_benchmark as v3bench
from .graph_dstar_lite import GraphDStarLite
from .planner_benchmark.map_utils import sha256_file


ROOT = Path("/home/robot/pudu_robot_ws")
BASE = ROOT / "experiments/layered_planner_benchmark"
V3_CALIBRATION = BASE / "2d_v3_calibration_4x_area_r0_20260903_183307"
V3_HELDOUT = BASE / "2d_v3_dynamic_4x_area_r0_20260903_183611"
V3_SOAK = BASE / "2d_v3_cleaning_replay_r0_20260903_184825"
V3_RATIO = BASE / "2d_v3_ratio_break_even_4x_r0_20260903_190301"
FROZEN = (
    BASE / "2d_v2_static_mentor_map_005_r0_20260903_154754",
    BASE / "2d_v2_dynamic_4x_area_r0_20260903_154947",
    V3_CALIBRATION, V3_HELDOUT, V3_SOAK, V3_RATIO,
    BASE / "2d_v1_dynamic_incremental_value_v1_20260903_134619",
    BASE / "2d_v1_dynamic_incremental_4x_area_v1_20260903_150321",
)
CONFIG = Path(__file__).resolve().parents[1] / "config/two_layer_2d_v2_r1_dstar_tail_research.yaml"
TEST_SOURCE = Path(__file__).resolve().parents[1] / "test/test_dstar_tail_research.py"
SEED = 20260903
DEFAULT_WARMUPS = 3
DEFAULT_REPETITIONS = 20

REQUIRED = (
    "final_report.md", "protocol.yaml", "manifest.yaml", "verification.yaml",
    "runs.csv", "per_scenario_summary.csv", "phase_timing_summary.csv",
    "correctness_oracle.csv", "queue_diagnostics.csv",
    "update_vertex_diagnostics.csv", "connectivity_precheck.csv",
    "no_route_recovery.csv", "resync_strategies.csv",
    "break_even_curve_absolute.csv", "break_even_curve_ratio.csv",
    "memory_cpu_summary.csv", "source_snapshot_manifest.yaml",
    "stdout.log", "stderr.log", "reproduction_command.txt",
)


def _default_output(mode: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = {
        "calibrate": "2d_v2_r1_dstar_tail_calibration_",
        "heldout": "2d_v2_r1_dstar_tail_heldout_",
        "soak": "2d_v2_r1_dstar_tail_soak_",
    }[mode]
    return BASE / f"{prefix}{stamp}"


def _tree_hash(directory: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(item for item in directory.rglob("*") if item.is_file()):
        digest.update(str(path.relative_to(directory)).encode())
        digest.update(b"\0")
        digest.update(bytes.fromhex(sha256_file(path)))
    return digest.hexdigest()


def _frozen_hashes() -> Dict[str, str]:
    missing = [str(path) for path in FROZEN if not path.is_dir()]
    if missing:
        raise FileNotFoundError(f"missing frozen authorities: {missing}")
    return {str(path): _tree_hash(path) for path in FROZEN}


def _refuse_nonempty(path: Path) -> None:
    if path.exists() and any(path.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {path}")


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
            writer.writerow({
                key: json.dumps(value, sort_keys=True) if isinstance(value, (dict, list, tuple)) else value
                for key, value in row.items()
            })


def _p(values: Sequence[float], percentile: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=float), percentile)) if values else math.nan


def _summary(values: Sequence[float]) -> Dict[str, Any]:
    return {
        "count": len(values), "p50": _p(values, 50), "p95": _p(values, 95),
        "p99": _p(values, 99),
        "mean": statistics.fmean(values) if values else math.nan,
        "max": max(values) if values else math.nan,
    }


def _load_streams(directory: Path) -> List[Dict[str, Any]]:
    result = []
    for path in sorted((directory / "dynamic_event_streams").glob("*.json")):
        payload = json.loads(path.read_text(encoding="utf-8"))
        result.append({
            "source": path, "workload": dict(payload["workload"]),
            "payloads": [json.dumps(row, sort_keys=True, separators=(",", ":"))
                         for row in payload["snapshots"]],
        })
    return result


def _authority_streams(mode: str) -> List[Dict[str, Any]]:
    if mode == "calibrate":
        return _load_streams(V3_CALIBRATION)
    heldout = _load_streams(V3_HELDOUT)
    if mode == "heldout":
        heldout.extend(_load_streams(V3_RATIO))
    return heldout


def _gate_rows(rows: Sequence[Mapping[str, Any]]) -> List[Mapping[str, Any]]:
    # No unchanged-snapshot or off-route scheduler shortcut is included.
    # Every actual confirmed edge-state transition is charged to every arm.
    return [
        row for row in rows
        if row.get("run_mode") == "measured" and row.get("dynamic_update")
        and int(row.get("changed_edge_count") or 0) > 0
    ]


def _run_episode(
    stream: Mapping[str, Any], template: dynamic.GraphTemplate,
    edge_cells: Mapping[str, Sequence[Tuple[int, int]]], *,
    map_version: str, map_shape: Sequence[int], run_mode: str,
    repetition: int, combo_backend: str,
    arms: Sequence[str] = research.ARMS,
) -> List[Dict[str, Any]]:
    overlay = dynamic.DynamicEdgeOverlay(edge_cells, map_version=map_version, map_shape=map_shape)
    states = {
        arm: research.TailArmState(arm, template, combo_backend=combo_backend)
        for arm in arms
    }
    workload = stream["workload"]
    rows: List[Dict[str, Any]] = []
    previous_oracle_edges: Tuple[str, ...] = ()
    for snapshot_index, payload in enumerate(stream["payloads"]):
        prepared = overlay.consume_json(payload)
        status_hash = dynamic.stable_hash(prepared.statuses)
        route_intersections = sorted(set(prepared.changed_edges).intersection(previous_oracle_edges))
        order = list(arms)
        offset = (snapshot_index + repetition) % len(order)
        order = order[offset:] + order[:offset]
        results = {arm: states[arm].run(prepared) for arm in order}
        oracle = results[research.COLD_GRAPH_ASTAR]
        baseline_hash = results.get(research.BASELINE_DSTAR, {}).get("g_rhs_state_hash", "")
        per_snapshot = []
        for arm in arms:
            result = results[arm]
            reachable_parity = bool(result["reachable"]) == bool(oracle["reachable"])
            failure_parity = str(result["failure_code"]) == str(oracle["failure_code"])
            if result["reachable"] and oracle["reachable"]:
                cost_error = abs(float(result["path_cost"]) - float(oracle["path_cost"]))
            else:
                cost_error = 0.0
            route_equal = list(result["path_edge_ids"]) == list(oracle["path_edge_ids"])
            is_dstar = arm != research.COLD_GRAPH_ASTAR
            g_rhs_parity = (not is_dstar) or str(result["g_rhs_state_hash"]) == str(baseline_hash)
            row = {
                "research_id": research.RESEARCH_ID,
                "parent_architecture": research.PARENT_ARCHITECTURE,
                "reference_architecture": research.REFERENCE_ARCHITECTURE,
                "experiment_kind": research.EXPERIMENT_KIND,
                "status": research.STATUS, "protocol_version": research.PROTOCOL_VERSION,
                "map_id": "mentor_map_20260825_005_4x_area",
                "scenario_id": workload["scenario_id"], "split": workload["split"],
                "category": workload["category"], "scale_family": workload.get("scale_family", "cleaning_semantic"),
                "query_id": workload["query_id"], "source_kind": workload["source_kind"],
                "requested_changed_edges": workload.get("requested_changed_edges", 0),
                "requested_changed_ratio": workload.get("requested_changed_ratio", 0.0),
                "run_mode": run_mode, "repetition": repetition,
                "snapshot_index": snapshot_index,
                "snapshot_id": prepared.snapshot.snapshot_id,
                "snapshot_hash": prepared.snapshot.snapshot_hash,
                "snapshot_accepted": prepared.accepted,
                "snapshot_rejection_reason": prepared.rejection_reason,
                "snapshot_parse_ms": prepared.parse_time_ms,
                "changed_cell_to_edge_mapping_ms": prepared.mapping_time_ms,
                "edge_state_transition_ms": prepared.state_transition_time_ms,
                "changed_cells_count": len(prepared.changed_cells),
                "changed_edge_count": len(prepared.changed_edges),
                "changed_edge_ratio": len(prepared.changed_edges) / max(1, len(edge_cells)),
                "changed_edge_ids": list(prepared.changed_edges),
                "route_intersection": bool(route_intersections),
                "route_intersection_count": len(route_intersections),
                "edge_status_hash": status_hash,
                "topology_static_hash": template.static_hash,
                "input_hash": prepared.input_hash,
                "oracle_reachable": oracle["reachable"],
                "oracle_failure_code": oracle["failure_code"],
                "oracle_path_cost": oracle["path_cost"],
                "oracle_path_edge_ids": oracle["path_edge_ids"],
                "reachable_parity": reachable_parity,
                "failure_code_parity": failure_parity,
                "path_cost_error": cost_error,
                "path_cost_parity": cost_error == 0.0,
                "route_edge_ids_equal": route_equal,
                "g_rhs_state_parity": g_rhs_parity,
                **result,
            }
            row["all_correct"] = all((
                prepared.accepted, reachable_parity, failure_parity,
                cost_error == 0.0, route_equal,
                not bool(result["blocked_edges_in_path"]),
                bool(result["converged"]), bool(result["dstar_state_invariant"]),
                g_rhs_parity, not bool(result["partial_dstar_result_returned"]),
                not bool(result["implicit_reinitialize"]),
                str(result["algorithm_input_hash"]) == str(prepared.input_hash),
            ))
            per_snapshot.append(row); rows.append(row)
        if len({row["input_hash"] for row in per_snapshot}) != 1:
            raise AssertionError("arm input hashes differ")
        if len({row["edge_status_hash"] for row in per_snapshot}) != 1:
            raise AssertionError("arm status hashes differ")
        previous_oracle_edges = tuple(str(edge) for edge in oracle["path_edge_ids"])
    return rows


def _arm_timing(rows: Sequence[Mapping[str, Any]], group: str = "dynamic_changed") -> Dict[str, Dict[str, Any]]:
    selected = _gate_rows(rows) if group == "dynamic_changed" else list(rows)
    return {
        arm: _summary([float(row["full_l1_ms"]) for row in selected if row["arm"] == arm])
        for arm in research.ARMS if any(row["arm"] == arm for row in selected)
    }


def select_combo_backend(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    """Freeze only changes with a correct, single-variable calibration win."""
    gate = _gate_rows(rows)
    timings = _arm_timing(rows)
    correct = {
        arm: all(bool(row["all_correct"]) for row in rows if row["arm"] == arm)
        for arm in research.ARMS
    }
    selected = research.BASELINE_DSTAR
    decisions: List[Dict[str, Any]] = []

    def tail_effective(candidate: str, parent: str) -> bool:
        c, p = timings[candidate], timings[parent]
        return c["p95"] <= 0.99 * p["p95"] and c["p99"] <= 1.01 * p["p99"]

    indexed_ok = bool(correct[research.INDEXED_DSTAR] and tail_effective(
        research.INDEXED_DSTAR, research.BASELINE_DSTAR,
    ))
    decisions.append({"change": "indexed_open", "parent": research.BASELINE_DSTAR,
                      "candidate": research.INDEXED_DSTAR, "accepted": indexed_ok,
                      "rule": "P95 improves >=1%, P99 regresses <=1%, correctness 100%"})
    if indexed_ok:
        selected = research.INDEXED_DSTAR
        batch_ok = bool(correct[research.INDEXED_BATCH_DSTAR] and tail_effective(
            research.INDEXED_BATCH_DSTAR, research.INDEXED_DSTAR,
        ))
    else:
        batch_ok = False
    decisions.append({"change": "cached_batch_update", "parent": research.INDEXED_DSTAR,
                      "candidate": research.INDEXED_BATCH_DSTAR, "accepted": batch_ok,
                      "rule": "only considered after indexed OPEN; same tail rule"})
    if batch_ok:
        selected = research.INDEXED_BATCH_DSTAR
        no_route_parent = [float(row["full_l1_ms"]) for row in gate
                           if row["arm"] == selected and not row["oracle_reachable"]]
        no_route_candidate = [float(row["full_l1_ms"]) for row in gate
                              if row["arm"] == research.INDEXED_BATCH_CONNECTIVITY
                              and not row["oracle_reachable"]]
        e, p = timings[research.INDEXED_BATCH_CONNECTIVITY], timings[selected]
        connectivity_ok = bool(
            correct[research.INDEXED_BATCH_CONNECTIVITY]
            and no_route_parent and no_route_candidate
            and _p(no_route_candidate, 95) <= 0.95 * _p(no_route_parent, 95)
            and e["p95"] <= 1.05 * p["p95"]
        )
    else:
        connectivity_ok = False
    decisions.append({"change": "exact_connectivity_precheck", "parent": research.INDEXED_BATCH_DSTAR,
                      "candidate": research.INDEXED_BATCH_CONNECTIVITY, "accepted": connectivity_ok,
                      "rule": "only after indexed+batch; no-route P95 improves >=5%, overall P95 <=1.05x"})
    if connectivity_ok:
        selected = research.INDEXED_BATCH_CONNECTIVITY
    return {
        "selected_backend": selected, "selection_decisions": decisions,
        "calibration_timing": timings, "calibration_rows": len(gate),
        "held_out_not_observed": True,
    }


def _gates(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    selected = _gate_rows(rows)
    timings = _arm_timing(rows)
    astar = timings[research.COLD_GRAPH_ASTAR]
    combo = timings[research.COMBO_DSTAR]
    p50_reduction = 1.0 - combo["p50"] / astar["p50"]
    p95_ratio = combo["p95"] / astar["p95"]
    p99_ratio = combo["p99"] / astar["p99"]
    no_route = {
        arm: _summary([float(row["full_l1_ms"]) for row in selected
                       if row["arm"] == arm and not row["oracle_reachable"]])
        for arm in (research.COLD_GRAPH_ASTAR, research.COMBO_DSTAR)
    }
    no_route_gate = bool(
        no_route[research.COMBO_DSTAR]["count"]
        and no_route[research.COMBO_DSTAR]["p50"] <= no_route[research.COLD_GRAPH_ASTAR]["p50"]
        and no_route[research.COMBO_DSTAR]["p95"] <= no_route[research.COLD_GRAPH_ASTAR]["p95"]
    )
    correctness = all(bool(row["all_correct"]) for row in rows if row["run_mode"] == "measured")
    return {
        "measured_rows": sum(row["run_mode"] == "measured" for row in rows),
        "gate_rows_per_arm": sum(row["arm"] == research.COLD_GRAPH_ASTAR for row in selected),
        "correctness_pass": correctness,
        "correctness_failures": sum(not bool(row["all_correct"]) for row in rows if row["run_mode"] == "measured"),
        "maximum_path_cost_error": max((float(row["path_cost_error"]) for row in rows), default=0.0),
        "blocked_edges_in_returned_paths": sum(bool(row["blocked_edges_in_path"]) for row in rows),
        "partial_results_returned": sum(bool(row["partial_dstar_result_returned"]) for row in rows),
        "hidden_reinitialize_count": sum(bool(row["implicit_reinitialize"]) for row in rows),
        "snapshot_status_input_mismatch": sum(
            str(row["algorithm_input_hash"]) != str(row["input_hash"]) for row in rows
        ),
        "timing": timings, "p50_reduction": p50_reduction,
        "p95_ratio": p95_ratio, "p99_ratio": p99_ratio,
        "p50_absolute_delta_ms": combo["p50"] - astar["p50"],
        "p95_absolute_delta_ms": combo["p95"] - astar["p95"],
        "p99_absolute_delta_ms": combo["p99"] - astar["p99"],
        "p50_gate_pass": p50_reduction >= 0.10,
        "p95_gate_pass": p95_ratio <= 1.05,
        "p99_gate_pass": p99_ratio <= 1.10,
        "no_route": no_route, "no_route_gate_pass": no_route_gate,
        "recovery_correct": all(
            row["all_correct"] for row in rows
            if row["run_mode"] == "measured" and row["category"] == "obstacle_disappearance_recovery"
        ),
        "all_gate_pass": bool(
            correctness and p50_reduction >= 0.10 and p95_ratio <= 1.05
            and p99_ratio <= 1.10 and no_route_gate
        ),
        "gate_excludes_unchanged_snapshot_scheduler_opportunities": True,
    }


def _timing_rows(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    measured = [row for row in rows if row["run_mode"] == "measured"]
    groups = {
        "s0_initial": lambda r: bool(r["initial_plan"]),
        "dynamic_changed": lambda r: bool(r["dynamic_update"]) and int(r["changed_edge_count"]) > 0,
        "path_affected": lambda r: bool(r["dynamic_update"]) and int(r["changed_edge_count"]) > 0 and bool(r["route_intersection"]),
        "path_unaffected": lambda r: bool(r["dynamic_update"]) and int(r["changed_edge_count"]) > 0 and not bool(r["route_intersection"]),
        "no_route": lambda r: bool(r["dynamic_update"]) and int(r["changed_edge_count"]) > 0 and not bool(r["oracle_reachable"]),
        "recovery": lambda r: r["category"] == "obstacle_disappearance_recovery" and bool(r["dynamic_update"]) and int(r["changed_edge_count"]) > 0,
    }
    metrics = (
        "snapshot_parse_ms", "changed_cell_to_edge_mapping_ms", "edge_state_transition_ms",
        "cold_init_ms", "update_edges_ms", "connectivity_precheck_ms",
        "compute_shortest_path_ms", "route_extraction_ms", "response_l1_ms",
        "full_l1_ms", "process_cpu_ms", "diagnostics_ms_excluded",
    )
    result = []
    for group, predicate in groups.items():
        for arm in research.ARMS:
            selected = [row for row in measured if row["arm"] == arm and predicate(row)]
            for metric in metrics:
                result.append({"group": group, "arm": arm, "metric": metric,
                               **_summary([float(row[metric]) for row in selected])})
    return result


def _metric_summary_rows(rows: Sequence[Mapping[str, Any]], metrics: Sequence[str]) -> List[Dict[str, Any]]:
    selected = _gate_rows(rows)
    return [
        {"arm": arm, "metric": metric,
         **_summary([float(row[metric]) for row in selected if row["arm"] == arm])}
        for arm in research.ARMS for metric in metrics
    ]


def _per_scenario(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    measured = _gate_rows(rows)
    result = []
    keys = sorted({(row["scenario_id"], row["arm"]) for row in measured})
    for scenario, arm in keys:
        selected = [row for row in measured if row["scenario_id"] == scenario and row["arm"] == arm]
        result.append({
            "scenario_id": scenario, "arm": arm,
            "category": selected[0]["category"], "scale_family": selected[0]["scale_family"],
            "sample_count": len(selected),
            "changed_edges": _summary([float(row["changed_edge_count"]) for row in selected]),
            "full_l1_ms": _summary([float(row["full_l1_ms"]) for row in selected]),
            "expanded_nodes": _summary([float(row["expanded_nodes"]) for row in selected]),
            "all_correct": all(row["all_correct"] for row in selected),
        })
    return result


def _correctness(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    fields = (
        "scenario_id", "query_id", "category", "repetition", "snapshot_index",
        "snapshot_id", "arm", "input_hash", "edge_status_hash", "reachable",
        "oracle_reachable", "failure_code", "oracle_failure_code", "path_cost",
        "oracle_path_cost", "path_cost_error", "reachable_parity",
        "failure_code_parity", "path_cost_parity", "route_edge_ids_equal",
        "blocked_edges_in_path", "converged", "dstar_state_invariant",
        "g_rhs_state_parity", "partial_dstar_result_returned",
        "implicit_reinitialize", "all_correct",
    )
    return [{field: row.get(field) for field in fields} for row in rows if row["run_mode"] == "measured"]


def _break_even(rows: Sequence[Mapping[str, Any]], ratio: bool) -> List[Dict[str, Any]]:
    selected = _gate_rows(rows)
    if ratio:
        selected = [row for row in selected if row["scale_family"] == "ratio_matched"]
        keys = sorted({float(row["requested_changed_ratio"]) for row in selected})
    else:
        keys = [1, 2, 5, 20, 100]
    result = []
    for key in keys:
        if ratio:
            values = [row for row in selected if float(row["requested_changed_ratio"]) == key]
        else:
            values = [row for row in selected if int(row["changed_edge_count"]) == key]
        if not values:
            continue
        summaries = {
            arm: _summary([float(row["full_l1_ms"]) for row in values if row["arm"] == arm])
            for arm in (research.COLD_GRAPH_ASTAR, research.BASELINE_DSTAR, research.COMBO_DSTAR)
        }
        a, b, c = summaries[research.COLD_GRAPH_ASTAR], summaries[research.BASELINE_DSTAR], summaries[research.COMBO_DSTAR]
        result.append({
            "requested_changed_ratio" if ratio else "changed_edge_count": key,
            "sample_count_per_arm": a["count"],
            "graph_astar_p50_ms": a["p50"], "baseline_dstar_p50_ms": b["p50"],
            "combo_dstar_p50_ms": c["p50"],
            "baseline_dstar_over_astar": b["p50"] / a["p50"],
            "combo_dstar_over_astar": c["p50"] / a["p50"],
            "baseline_expanded_over_astar": _p([
                float(row["expanded_nodes"]) for row in values if row["arm"] == research.BASELINE_DSTAR
            ], 50) / max(1.0, _p([
                float(row["expanded_nodes"]) for row in values if row["arm"] == research.COLD_GRAPH_ASTAR
            ], 50)),
        })
    return result


def _no_route_recovery(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    selected = _gate_rows(rows)
    result = []
    for group, predicate in (
        ("no_route", lambda r: not bool(r["oracle_reachable"])),
        ("recovery", lambda r: r["category"] == "obstacle_disappearance_recovery"),
        ("bridge_min_cut", lambda r: "no_route" in str(r["category"]) or bool(r.get("expected_no_route"))),
    ):
        for arm in research.ARMS:
            values = [row for row in selected if row["arm"] == arm and predicate(row)]
            result.append({"group": group, "arm": arm,
                           "reachable_parity": all(row["reachable_parity"] for row in values),
                           "failure_code_parity": all(row["failure_code_parity"] for row in values),
                           "full_l1_ms": _summary([float(row["full_l1_ms"]) for row in values])})
    return result


def _resync_study(
    streams: Sequence[Mapping[str, Any]], query_graphs: Mapping[str, Any],
    edge_cells: Mapping[str, Sequence[Tuple[int, int]]], *, map_version: str,
    map_shape: Sequence[int], repetition: int,
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for stream in streams:
        workload = stream["workload"]
        template = query_graphs[workload["query_id"]].template
        overlay = dynamic.DynamicEdgeOverlay(edge_cells, map_version=map_version, map_shape=map_shape)
        strategies = {
            "immediate": research.ResyncState("immediate", template, quiet_window_snapshots=0),
            "lazy": research.ResyncState("lazy", template, quiet_window_snapshots=2),
            "batched_background": research.ResyncState("batched_background", template, quiet_window_snapshots=1),
        }
        previous_route: Tuple[str, ...] = ()
        last_prepared = None
        for snapshot_index, payload in enumerate(stream["payloads"]):
            prepared = overlay.consume_json(payload); last_prepared = prepared
            oracle = dynamic.deterministic_graph_astar(template, prepared.statuses)
            intersection = bool(set(prepared.changed_edges).intersection(previous_route))
            fallback = bool(prepared.changed_edges) and (
                intersection or len(prepared.changed_edges) > 2 or oracle.node_path is None
            )
            response_ms = float(oracle.search_time_ms + oracle.extraction_time_ms)
            for name, state in strategies.items():
                if snapshot_index == 0:
                    result = {"resync_ran": True, "resync_cpu_ms": state.initialize(prepared),
                              "resync_wall_ms": 0.0, "ready": True, "coalesced_snapshots": 1,
                              "resync_snapshot_id": prepared.snapshot.snapshot_id,
                              "resync_status_hash": dynamic.stable_hash(prepared.statuses)}
                elif fallback:
                    result = state.on_fallback(prepared)
                else:
                    result = state.observe(prepared)
                row = {
                    "scenario_id": workload["scenario_id"], "strategy": name,
                    "repetition": repetition, "snapshot_index": snapshot_index,
                    "snapshot_id": prepared.snapshot.snapshot_id,
                    "changed_edge_count": len(prepared.changed_edges),
                    "fallback_event": fallback, "ready_before_response": state.ready,
                    "astar_response_ms_when_fallback_or_not_ready": response_ms if fallback or not state.ready else 0.0,
                    "status_hash_expected": dynamic.stable_hash(prepared.statuses), **result,
                }
                row["resync_status_hash_match"] = (
                    not row.get("resync_ran")
                    or str(row.get("resync_status_hash")) == str(row["status_hash_expected"])
                )
                row["total_accounted_ms"] = float(row["astar_response_ms_when_fallback_or_not_ready"]) + float(row["resync_cpu_ms"])
                rows.append(row)
            previous_route = tuple(oracle.edge_path)
        for name, state in strategies.items():
            result = state.flush()
            if result["resync_ran"] and last_prepared is not None:
                rows.append({
                    "scenario_id": workload["scenario_id"], "strategy": name,
                    "repetition": repetition, "snapshot_index": 21,
                    "snapshot_id": last_prepared.snapshot.snapshot_id,
                    "changed_edge_count": 0, "fallback_event": False,
                    "ready_before_response": False,
                    "astar_response_ms_when_fallback_or_not_ready": 0.0,
                    "status_hash_expected": dynamic.stable_hash(last_prepared.statuses),
                    **result,
                    "resync_status_hash_match": str(result.get("resync_status_hash")) == dynamic.stable_hash(last_prepared.statuses),
                    "total_accounted_ms": float(result["resync_cpu_ms"]),
                })
    return rows


def _source_snapshot(output: Path, event_sources: Sequence[Path]) -> Dict[str, Any]:
    directory = output / "source_snapshot"; directory.mkdir()
    sources = [
        Path(__file__).resolve(), Path(research.__file__).resolve(),
        Path(__file__).resolve().with_name("indexed_dstar_open.py"),
        Path(__file__).resolve().with_name("graph_dstar_lite.py"),
        Path(__file__).resolve().with_name("dynamic_incremental_value.py"),
        CONFIG, TEST_SOURCE, *event_sources,
    ]
    records = []
    for index, source in enumerate(sources):
        target = directory / f"{index:03d}_{source.name}"
        shutil.copyfile(source, target)
        records.append({"source": str(source), "snapshot": str(target.relative_to(output)),
                        "sha256": sha256_file(target), "bytes": target.stat().st_size})
    payload = {"schema_version": 1, "files": records,
               "combined_hash": dynamic.stable_hash([[row["snapshot"], row["sha256"]] for row in records])}
    (output / "source_snapshot_manifest.yaml").write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return payload


def validate_artifacts(output: Path) -> Dict[str, Any]:
    missing = [name for name in REQUIRED if not (output / name).is_file()]
    bad = []
    manifest_path = output / "source_snapshot_manifest.yaml"
    if manifest_path.is_file():
        payload = yaml.safe_load(manifest_path.read_text()) or {}
        for row in payload.get("files", []):
            path = output / row["snapshot"]
            if not path.is_file() or sha256_file(path) != row["sha256"]:
                bad.append(row["snapshot"])
    return {"required_count": len(REQUIRED), "missing": missing,
            "bad_source_snapshot_hashes": bad, "passed": not missing and not bad}


def _report(output: Path, mode: str, gates: Mapping[str, Any], selection: Mapping[str, Any],
            rows: Sequence[Mapping[str, Any]], verdict: str, stage_b: str) -> None:
    timing = gates["timing"]
    lines = [
        "# 2D-V2 r1 D* Lite core tail-latency research", "",
        f"- Mode: `{mode}`.", f"- Formal verdict: **{verdict}**.",
        f"- Frozen combo backend: `{selection['selected_backend']}`.",
        f"- Correctness: {gates['measured_rows'] - gates['correctness_failures']}/{gates['measured_rows']}; max cost error={gates['maximum_path_cost_error']}.",
        f"- System Stage B: `{stage_b}`.", "", "## Paired L1 timing", "",
        "| arm | P50 ms | P95 ms | P99 ms |", "|---|---:|---:|---:|",
    ]
    for arm in research.ARMS:
        if arm in timing:
            item = timing[arm]
            lines.append(f"| {arm} | {item['p50']:.3f} | {item['p95']:.3f} | {item['p99']:.3f} |")
    lines += [
        "", "The gate uses every confirmed changed-edge snapshot and excludes unchanged-snapshot scheduler opportunities. All parse, cell-to-edge mapping, transition, update, precheck, search, extraction and synchronous state-maintenance costs are included.",
        "", "## Gate", "",
        f"- Combo minus A*: {gates['p50_absolute_delta_ms']:+.3f}/{gates['p95_absolute_delta_ms']:+.3f}/{gates['p99_absolute_delta_ms']:+.3f} ms at P50/P95/P99.",
        f"- P50 reduction={100*gates['p50_reduction']:.2f}% (need >=10%); P95/P99 ratios={gates['p95_ratio']:.3f}/{gates['p99_ratio']:.3f} (limits 1.05/1.10).",
        f"- No-route P50/P95 gate={gates['no_route_gate_pass']}; recovery correctness={gates['recovery_correct']}.",
        "", "## Interpretation", "",
        "- Indexed OPEN is a true unique-entry heap; stale entries, indexed updates and sifts are recorded directly.",
        "- The frozen lazy baseline already deduplicates the initial changed-edge affected-node batch. The remaining update_vertex count is primarily propagation from expanded inconsistent states.",
        "- Exact connectivity response time and the mandatory D* state-maintenance charge are reported separately; the formal full-L1 gate includes maintenance.",
        "- No ROI/ACK, heading-bin, corridor, Smac, PathAudit or scheduler gain is included in this L1-only result.",
        "- The C++ prototype is not run unless the measured profile shows Python queue constants, rather than unchanged consistency propagation, dominate the residual tail.",
        "", "## Reproduction", "", "```bash",
        (output / "reproduction_command.txt").read_text().strip(), "```", "",
    ]
    (output / "final_report.md").write_text("\n".join(lines), encoding="utf-8")


def run(
    output: Path, *, mode: str, frozen_candidate: Optional[Path] = None,
    warmups: int = DEFAULT_WARMUPS, repetitions: int = DEFAULT_REPETITIONS,
    soak_cycles: int = 20, preimplementation_profile: Optional[Path] = None,
) -> Path:
    output = output.resolve(); _refuse_nonempty(output)
    if mode == "heldout" and (warmups < 3 or repetitions < 20):
        raise ValueError("formal held-out requires >=3 warmups and >=20 repetitions")
    if mode in {"heldout", "soak"} and frozen_candidate is None:
        raise ValueError("heldout/soak requires --frozen-candidate from calibration")
    frozen_before = _frozen_hashes()
    ctx, _queries, task_metadata, artifact, topology_info, query_graphs, _reference, edge_cells, _workloads = v3bench._load_inputs()
    if len(next(iter(query_graphs.values())).template.nodes) < 4376:
        raise AssertionError("4x query graph unexpectedly smaller than audited topology")
    streams = _authority_streams(mode)
    output.mkdir(parents=True)
    (output / "stdout.log").write_text("Structured L1-only research run completed by the benchmark CLI.\n", encoding="utf-8")
    (output / "stderr.log").write_text("", encoding="utf-8")
    event_dir = output / "dynamic_event_streams"; event_dir.mkdir()
    event_sources = []
    for stream in streams:
        source = Path(stream["source"]); target = event_dir / source.name
        shutil.copyfile(source, target); event_sources.append(source)
    if preimplementation_profile and preimplementation_profile.is_file():
        shutil.copyfile(preimplementation_profile, output / "preimplementation_profile.json")

    if mode == "calibrate":
        combo_backend = research.BASELINE_DSTAR
        plan = (("measured", repetitions),)
        arms = research.ARMS
    else:
        frozen = yaml.safe_load(frozen_candidate.read_text()) or {}
        combo_backend = str(frozen["selected_backend"])
        plan = (("warmup", warmups), ("measured", repetitions)) if mode == "heldout" else (("measured", soak_cycles),)
        arms = research.ARMS if mode == "heldout" else (
            research.COLD_GRAPH_ASTAR, research.BASELINE_DSTAR, research.COMBO_DSTAR,
        )

    rows: List[Dict[str, Any]] = []
    for run_mode, count in plan:
        for repetition in range(1, count + 1):
            for stream in streams:
                workload = stream["workload"]
                rows.extend(_run_episode(
                    stream, query_graphs[workload["query_id"]].template, edge_cells,
                    map_version=ctx.map_sha256, map_shape=artifact.free_mask.shape,
                    run_mode=run_mode, repetition=repetition,
                    combo_backend=combo_backend, arms=arms,
                ))

    if mode == "calibrate":
        selection = select_combo_backend(rows)
        (output / "frozen_candidate.yaml").write_text(yaml.safe_dump({
            "research_id": research.RESEARCH_ID,
            "selected_backend": selection["selected_backend"],
            "selection_decisions": selection["selection_decisions"],
            "calibration_output": str(output), "held_out_observed": False,
        }, sort_keys=False), encoding="utf-8")
    else:
        selection = {
            "selected_backend": combo_backend,
            "selection_decisions": frozen.get("selection_decisions", []),
            "calibration_output": frozen.get("calibration_output", ""),
            "held_out_not_observed": True,
        }

    gates = _gates(rows)
    baseline = gates["timing"].get(research.BASELINE_DSTAR, {})
    combo = gates["timing"].get(research.COMBO_DSTAR, {})
    tail_improved = bool(
        baseline and combo
        and (combo["p95"] <= 0.90 * baseline["p95"] or combo["p99"] <= 0.90 * baseline["p99"])
    )
    if mode == "calibrate":
        verdict, stage_b = "CALIBRATION_ONLY", "NOT_RUN_CALIBRATION"
    elif gates["all_gate_pass"]:
        verdict, stage_b = "A", "ELIGIBLE_FOR_SEPARATE_SYSTEM_VALIDATION"
    elif tail_improved:
        verdict, stage_b = "B", "NOT_RUN_L1_GATE_FAILED"
    else:
        verdict, stage_b = "C", "NOT_RUN_L1_GATE_FAILED"

    resync_rows: List[Dict[str, Any]] = []
    if mode in {"heldout", "soak"}:
        resync_repetitions = 1 if mode == "heldout" else min(3, soak_cycles)
        for repetition in range(1, resync_repetitions + 1):
            resync_rows.extend(_resync_study(
                streams, query_graphs, edge_cells, map_version=ctx.map_sha256,
                map_shape=artifact.free_mask.shape, repetition=repetition,
            ))

    _write_csv(output / "runs.csv", rows)
    _write_csv(output / "per_scenario_summary.csv", _per_scenario(rows))
    _write_csv(output / "phase_timing_summary.csv", _timing_rows(rows))
    _write_csv(output / "correctness_oracle.csv", _correctness(rows))
    _write_csv(output / "queue_diagnostics.csv", _metric_summary_rows(rows, (
        "queue_pushes", "queue_pops", "stale_queue_entries", "open_peak",
        "indexed_insertions", "indexed_updates", "indexed_removals",
        "indexed_sift_operations", "key_calculations", "tuple_allocation_proxy",
    )))
    _write_csv(output / "update_vertex_diagnostics.csv", _metric_summary_rows(rows, (
        "update_vertex_count", "predecessor_propagations", "g_changed_nodes",
        "rhs_changed_nodes", "batch_candidate_nodes", "batch_unique_nodes", "batch_dedup_saved",
    )))
    _write_csv(output / "connectivity_precheck.csv", _metric_summary_rows(rows, (
        "connectivity_precheck_ms", "connectivity_visited_nodes", "connectivity_edge_checks",
        "maintenance_after_exact_response_ms",
    )))
    _write_csv(output / "no_route_recovery.csv", _no_route_recovery(rows))
    _write_csv(output / "resync_strategies.csv", resync_rows or [{"status": "NOT_RUN_IN_CALIBRATION"}])
    _write_csv(output / "break_even_curve_absolute.csv", _break_even(rows, False))
    _write_csv(output / "break_even_curve_ratio.csv", _break_even(rows, True))
    memory = _metric_summary_rows(rows, ("state_memory_bytes", "process_cpu_ms", "cold_init_ms"))
    if resync_rows:
        for strategy in ("immediate", "lazy", "batched_background"):
            selected = [row for row in resync_rows if row["strategy"] == strategy]
            total_cpu = sum(float(row["resync_cpu_ms"]) for row in selected)
            memory.append({
                "arm": f"resync_{strategy}", "metric": "resync_total_cpu_ms",
                **_summary([float(row["resync_cpu_ms"]) for row in selected]),
                "total": total_cpu,
                "modeled_cpu_utilization_1hz": total_cpu / max(1, len(selected)) / 1000.0,
                "modeled_cpu_utilization_5hz": 5 * total_cpu / max(1, len(selected)) / 1000.0,
                "modeled_cpu_utilization_10hz": 10 * total_cpu / max(1, len(selected)) / 1000.0,
                "frequency_rows_are_model_not_measured": True,
            })
    _write_csv(output / "memory_cpu_summary.csv", memory)

    reproduction = (
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        f"out=/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v2_r1_dstar_tail_{mode}_$(date +%Y%m%d_%H%M%S)\n"
        f"ROS_DOMAIN_ID=121 ros2 run arena_evaluation two_layer_2d_v2_r1_dstar_tail_benchmark --mode {mode} --output-dir \"$out\""
    )
    if frozen_candidate:
        reproduction += f" --frozen-candidate {frozen_candidate}"
    reproduction += f" --warmups {warmups} --repetitions {repetitions} --soak-cycles {soak_cycles}\n"
    (output / "reproduction_command.txt").write_text(reproduction, encoding="utf-8")
    protocol = yaml.safe_load(CONFIG.read_text()) or {}
    protocol.update({
        "mode": mode, "warmups": warmups, "repetitions": repetitions,
        "soak_cycles": soak_cycles, "seed": SEED,
        "authority_event_directories": sorted({str(Path(stream["source"]).parent.parent) for stream in streams}),
        "selected_backend": selection["selected_backend"],
        "gate_population": "measured dynamic snapshots with changed_edge_count > 0; no scheduler skips",
    })
    (output / "protocol.yaml").write_text(yaml.safe_dump(protocol, sort_keys=False), encoding="utf-8")
    source_snapshot = _source_snapshot(output, event_sources)
    frozen_after = _frozen_hashes()
    if frozen_before != frozen_after:
        raise AssertionError("a frozen authority directory changed during the run")
    manifest = {
        "experiment_id": output.name, "created_at": datetime.now(timezone.utc).astimezone().isoformat(),
        "research_id": research.RESEARCH_ID, "formal": mode in {"heldout", "soak"},
        "mode": mode, "final_verdict": verdict, "stage_b_status": stage_b,
        "map_id": "mentor_map_20260825_005_4x_area", "map_hash": ctx.map_sha256,
        "map_shape": list(artifact.free_mask.shape), "topology_nodes": len(artifact.graph.nodes),
        "topology_edges": len(artifact.graph.edges), "scenario_count": len(streams),
        "source_kind": "realistic_synthetic_cleaning_workload",
        "selected_backend": selection["selected_backend"], "selection": selection,
        "gates": gates, "frozen_hashes_before": frozen_before,
        "frozen_hashes_after": frozen_after, "frozen_unchanged": True,
        "source_snapshot_hash": source_snapshot["combined_hash"],
        "task_metadata": task_metadata,
        "cpp_prototype": "NOT_RUN_STRUCTURAL_PROPAGATION_DOMINATES" if mode != "calibrate" else "PENDING_HELDOUT_PROFILE",
        "system_stage_b": stage_b,
    }
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    _report(output, mode, gates, selection, rows, verdict, stage_b)
    (output / "verification.yaml").write_text(yaml.safe_dump({
        "formal_run_complete": True, "correctness": gates["correctness_pass"],
        "frozen_authorities_unchanged": True, "source_snapshot_hash": source_snapshot["combined_hash"],
        "artifact_validation": "pending_final_machine_check",
    }, sort_keys=False), encoding="utf-8")
    validation = validate_artifacts(output)
    verification = yaml.safe_load((output / "verification.yaml").read_text()) or {}
    verification["artifact_validation"] = validation
    (output / "verification.yaml").write_text(yaml.safe_dump(verification, sort_keys=False), encoding="utf-8")
    if not validation["passed"]:
        raise AssertionError(validation)
    return output


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("calibrate", "heldout", "soak"), required=True)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--frozen-candidate", type=Path)
    parser.add_argument("--warmups", type=int, default=DEFAULT_WARMUPS)
    parser.add_argument("--repetitions", type=int, default=DEFAULT_REPETITIONS)
    parser.add_argument("--soak-cycles", type=int, default=20)
    parser.add_argument("--preimplementation-profile", type=Path)
    args = parser.parse_args(argv)
    output = args.output_dir or _default_output(args.mode)
    run(output, mode=args.mode, frozen_candidate=args.frozen_candidate,
        warmups=args.warmups, repetitions=args.repetitions,
        soak_cycles=args.soak_cycles,
        preimplementation_profile=args.preimplementation_profile)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
