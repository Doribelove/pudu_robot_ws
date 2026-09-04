"""Formal 4x-area scale extension of the 2D-V1 dynamic L1 experiment."""

from __future__ import annotations

import argparse
import csv
import json
import os
import random
import shutil
import statistics
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import dynamic_incremental_value as value
from . import layered_2d_v1_pipeline as r1_pipeline
from . import layered_2d_v1_r2_pipeline as r2_pipeline
from . import topology
from . import two_layer_2d_v1_dynamic_incremental_benchmark as parent
from . import two_layer_formal_benchmark as four_x_source
from . import unified_four_backends_smoke as legacy
from .dynamic_snapshot import DynamicSnapshot
from .graph_dstar_lite import GraphDStarLite
from .planner_benchmark.map_utils import sha256_file


ROOT = Path("/home/robot/pudu_robot_ws")
MAP_ID = "mentor_map_20260825_005_4x_area"
ARCHITECTURE_ID = "2D-V1"
IMPLEMENTATION_REVISION = "r4"
PARENT_ARCHITECTURE = "2D-V1-r3"
EXPERIMENT_KIND = "dynamic_incremental_scale_extension"
PROTOCOL_VERSION = "PLN-02-EXP-V1"
DEFAULT_SEED = 20260903
DEFAULT_WARMUPS = 3
DEFAULT_REPETITIONS = 20
DEFAULT_MAIN_QUERY_COUNT = 10
DEFAULT_ROS_REPETITIONS = 3
DEFAULT_ROS_DOMAIN_ID = 99
ABSOLUTE_POINTS = (1, 2, 5, 20, 100)
ONE_X_EDGE_COUNT = 2172
RATIO_TARGETS = tuple(value / ONE_X_EDGE_COUNT for value in ABSOLUTE_POINTS)
ONE_X_EXPERIMENT = ROOT / "experiments/layered_planner_benchmark/2d_v1_dynamic_incremental_value_v1_20260903_134619"
FROZEN_R2 = ROOT / "experiments/layered_planner_benchmark/2d_v1_mentor_map_20260825_005_20_performance_r2_20260903_1147"
FOUR_X_STATIC = ROOT / "experiments/layered_planner_benchmark/2a_v0_mentor_map_20260825_005_4x_area_20_r3_v1"
FOUR_X_WORLD = ROOT / "external/arena4_ws/src/arena/simulation-setup/worlds" / MAP_ID
FOUR_X_MAP_YAML = FOUR_X_WORLD / "map/map.yaml"
FOUR_X_MAP_PGM = FOUR_X_WORLD / "map/map.pgm"
FOUR_X_SCENARIO = FOUR_X_WORLD / "scenarios/a2b_benchmark_20.json"
FOUR_X_CACHE = ROOT / (
    "experiments/layered_planner_benchmark/"
    "2a_v0_mentor_map_20260825_005_4x_area_20_r3_v1_cache/"
    "mentor_map_20260825_005_4x_area/"
    "1ae9b33645cf8623bdacab2065b479e6c54420ca8dfa63c04fbee80e73a35b71"
)
PRIMARY_ARMS = (value.INCREMENTAL_DSTAR, value.COLD_GRAPH_ASTAR)


def _default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return ROOT / "experiments/layered_planner_benchmark" / f"2d_v1_dynamic_incremental_4x_area_v1_{stamp}"


def _refuse_nonempty(path: Path) -> None:
    if path.exists() and any(path.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {path}")


def _percentile(values: Sequence[float], q: float) -> float:
    return parent._percentile(values, q)


def _summary(values: Sequence[float]) -> Dict[str, float]:
    return parent._summary(values)


def ratio_change_points(edge_count: int) -> List[Dict[str, Any]]:
    result: List[Dict[str, Any]] = []
    seen = set()
    for target in RATIO_TARGETS:
        count = max(1, int(round(float(edge_count) * target)))
        if count in seen:
            continue
        seen.add(count)
        result.append({
            "target_ratio": target,
            "target_percent": target * 100.0,
            "changed_edge_count": count,
            "actual_ratio": count / max(1, int(edge_count)),
        })
    return result


def _load_4x_inputs() -> Tuple[Any, List[Any], Dict[str, Any], Any, Dict[str, Any]]:
    required = (FOUR_X_MAP_YAML, FOUR_X_MAP_PGM, FOUR_X_SCENARIO,
                FOUR_X_CACHE / "cache_manifest.yaml",
                FOUR_X_CACHE / "topology_metadata.yaml",
                FOUR_X_CACHE / "topology_graph.json",
                FOUR_X_CACHE / "topology_arrays.npz")
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"missing mandatory 4x inputs: {missing}")
    queries, task_metadata = four_x_source._load_tasks()
    ctx = four_x_source._context()
    cache_manifest = yaml.safe_load(
        (FOUR_X_CACHE / "cache_manifest.yaml").read_text(encoding="utf-8")
    ) or {}
    metadata = dict(cache_manifest.get("metadata") or {})
    checks = {
        "map_id": MAP_ID,
        "map_file_hash": sha256_file(FOUR_X_MAP_PGM),
        "map_yaml_hash": sha256_file(FOUR_X_MAP_YAML),
        "resolution": 0.05,
        "width": 6574,
        "height": 3024,
        "source_hash": sha256_file(Path(topology.__file__).resolve()),
        "skeleton_backend": "numpy_zhang_suen",
    }
    for key, expected in checks.items():
        if metadata.get(key) != expected:
            raise ValueError(f"4x topology cache mismatch for {key}: {metadata.get(key)!r} != {expected!r}")
    artifact = topology.load_topology(
        FOUR_X_CACHE, ctx.hospital_map, legacy.FOOTPRINT,
        padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    source_key = str(cache_manifest.get("cache_key") or FOUR_X_CACHE.name)
    binding_payload = {
        "map_hash": sha256_file(FOUR_X_MAP_PGM),
        "map_yaml_hash": sha256_file(FOUR_X_MAP_YAML),
        "combined_map_hash": ctx.map_sha256,
        "resolution": ctx.hospital_map.resolution,
        "origin": list(ctx.hospital_map.origin),
        "shape": list(artifact.free_mask.shape),
        "source_topology_cache_key": source_key,
        "topology_static_hash": parent.value.stable_hash(metadata),
        "topology_parameters": {
            "padding_m": 0.05, "safety_margin_m": 0.05,
            "allow_unknown": False, "algorithm": metadata.get("topology_algorithm_version"),
            "backend": metadata.get("skeleton_backend"),
        },
        "implementation_revision": IMPLEMENTATION_REVISION,
    }
    info = {
        "topology_cache_hit": True,
        "source_topology_cache_key": source_key,
        "topology_cache_key": value.stable_hash(binding_payload),
        "topology_source_hash": metadata.get("source_hash", ""),
        "topology_cache_bytes": four_x_source._directory_bytes(FOUR_X_CACHE),
        "topology_cache_directory": str(FOUR_X_CACHE),
        "skeleton_backend": metadata.get("skeleton_backend"),
        "cache_binding": binding_payload,
    }
    return ctx, queries, task_metadata, artifact, info


def _load_static_reference() -> Dict[str, Dict[str, Any]]:
    result: Dict[str, Dict[str, Any]] = {}
    with (FOUR_X_STATIC / "runs.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("run_mode") != "measured" or str(row.get("repetition")) != "1":
                continue
            result[str(row["query_id"])] = {
                "final_valid": str(row.get("final_valid_success", "")).lower() == "true",
                "failure_code": str(row.get("failure_code", "")),
                "path_hash": str(row.get("path_hash", "")),
            }
    if len(result) != 20:
        raise ValueError(f"4x static reference has {len(result)} queries, expected 20")
    return result


def _build_query_graphs(
    graph_view: Any, topology_info: Mapping[str, Any], ctx: Any,
    queries: Sequence[Any], static_reference: Mapping[str, Mapping[str, Any]],
) -> Tuple[Dict[str, parent.QueryGraph], r2_pipeline.Layered2DV1R2Pipeline]:
    snapshot = DynamicSnapshot.empty(
        snapshot_id="S0", timestamp=1.0, map_version=ctx.map_sha256,
        map_shape=graph_view.artifact.free_mask.shape,
    )
    pipeline = r2_pipeline.Layered2DV1R2Pipeline(
        graph_view, footprint=legacy.FOOTPRINT, l3_planner=None,
        corridor_padding_m=2.0, corridor_profile="padding",
        corridor_fallback_policy="bounded", base_map_hash=ctx.map_sha256,
        topology_cache_key=str(topology_info["topology_cache_key"]),
        topology_source_hash=str(topology_info["topology_source_hash"]),
        corridor_semantics="raw_map_smac_aligned",
    )
    result: Dict[str, parent.QueryGraph] = {}
    for query in queries:
        starts, goals, diagnostics = pipeline._attach(query, snapshot)
        planner, _start, _goal, _positions = pipeline._make_graph(starts, goals, snapshot)
        template = value.GraphTemplate.from_dstar(planner)
        oracle = value.deterministic_graph_astar(template, {})
        edges = tuple(edge for edge in oracle.edge_path if edge.startswith("topology_"))
        frozen = dict(static_reference.get(query.query_id, {}))
        frozen.update({"topology_edge_ids": list(edges), "graph_reachable": oracle.node_path is not None})
        result[query.query_id] = parent.QueryGraph(
            query, template, frozen, edges, float(oracle.cost), diagnostics,
        )
    return result, pipeline


@dataclass
class ScenarioSpec:
    scenario: parent.Scenario
    scale_family: str
    requested_changed_edges: int
    requested_changed_ratio: float


def _quantile_queries(query_graphs: Mapping[str, parent.QueryGraph], count: int) -> List[parent.QueryGraph]:
    pool = [
        graph for query_id, graph in sorted(query_graphs.items())
        if graph.baseline_edge_ids and query_id not in {"A2B-07", "A2B-16", "A2B-19"}
    ]
    pool.sort(key=lambda graph: (len(graph.baseline_edge_ids), graph.query.query_id))
    if len(pool) < count:
        raise RuntimeError(f"only {len(pool)} 4x L1-reachable non-control queries")
    indices = []
    for index in range(count):
        candidate = round(index * (len(pool) - 1) / max(1, count - 1))
        if candidate not in indices:
            indices.append(candidate)
    selected = [pool[index] for index in indices]
    for graph in pool:
        if len(selected) >= count:
            break
        if graph not in selected:
            selected.append(graph)
    return selected[:count]


def _reachable_scale_targets(
    graph: parent.QueryGraph, count: int,
    exclusive: Mapping[str, Tuple[int, int]], seed: int,
) -> Tuple[str, ...]:
    alternate = parent._select_alternate_edge(graph, exclusive)
    if alternate is None:
        raise RuntimeError(f"{graph.query.query_id} lacks alternate route edge")
    alternate_route = parent._blocked_oracle(graph, (alternate,))
    protected = set(alternate_route.edge_path)
    baseline = set(graph.baseline_edge_ids)
    pool = [
        edge for edge in sorted(exclusive)
        if edge != alternate and edge not in protected and edge not in baseline
    ]
    random.Random(seed).shuffle(pool)
    if len(pool) < count - 1:
        raise RuntimeError(f"need {count - 1} filler edges, have {len(pool)}")
    targets = tuple(sorted((alternate, *pool[:count - 1])))
    if parent._blocked_oracle(graph, targets).node_path is None:
        raise RuntimeError(f"scale target unexpectedly disconnects {graph.query.query_id}")
    return targets


def _build_scenarios(
    query_graphs: Mapping[str, parent.QueryGraph],
    exclusive: Mapping[str, Tuple[int, int]], any_witness: Mapping[str, Tuple[int, int]],
    *, edge_count: int, seed: int, main_query_count: int,
) -> Tuple[List[ScenarioSpec], List[str]]:
    selected = _quantile_queries(query_graphs, main_query_count)
    specs: List[ScenarioSpec] = []

    def add(sid: str, graph: parent.QueryGraph, category: str, targets: Tuple[str, ...],
            family: str = "semantic", min_cut: int = 0) -> None:
        witnesses = {edge: (exclusive.get(edge) or any_witness[edge]) for edge in targets}
        specs.append(ScenarioSpec(parent.Scenario(
            sid, graph.query.query_id, category, "main", targets, witnesses,
            min_cut_size=min_cut,
        ), family, len(targets), len(targets) / edge_count))

    outside_graph, alternate_graph, moving_graph, recovery_graph, cut_graph = selected[:5]
    outside = next(edge for edge in sorted(exclusive) if edge not in set(outside_graph.baseline_edge_ids))
    add("SEM-OUTSIDE", outside_graph, "outside_path", (outside,))
    alt = parent._select_alternate_edge(alternate_graph, exclusive)
    if alt is None:
        raise RuntimeError("no alternate route for semantic scenario")
    add("SEM-ALTERNATE", alternate_graph, "path_nonbridge_alternate", (alt,))
    route_cells = [edge for edge in moving_graph.baseline_edge_ids[3:-3] if edge in exclusive]
    if len(route_cells) < 3:
        raise RuntimeError("moving scenario lacks three exclusive route edges")
    moving = tuple(dict.fromkeys((route_cells[len(route_cells)//4], route_cells[len(route_cells)//2], route_cells[3*len(route_cells)//4])))
    if len(moving) != 3:
        moving = tuple(route_cells[:3])
    add("SEM-MOVING", moving_graph, "moving_e1_e2_e3", moving)
    recovery = next(edge for edge in recovery_graph.baseline_edge_ids[3:-3] if edge in exclusive)
    add("SEM-RECOVERY", recovery_graph, "disappearance_recovery", (recovery,))
    cut = parent._minimum_static_cut(cut_graph)
    add("SEM-NO-ROUTE", cut_graph, "bridge_or_min_cut_no_route", cut, min_cut=len(cut))

    for index, count in enumerate(ABSOLUTE_POINTS):
        graph = selected[index % len(selected)]
        targets = _reachable_scale_targets(graph, count, exclusive, seed + 1000 + count)
        add(f"ABS-{count:04d}", graph, f"absolute_changed_edges_{count}", targets, "absolute")
    ratio_points = ratio_change_points(edge_count)
    for index, point in enumerate(ratio_points):
        graph = selected[(index + 5) % len(selected)]
        count = int(point["changed_edge_count"])
        targets = _reachable_scale_targets(graph, count, exclusive, seed + 2000 + count)
        add(f"RATIO-{index + 1:02d}", graph, f"ratio_changed_edges_{count}", targets, "ratio")
        specs[-1].requested_changed_ratio = float(point["target_ratio"])

    for query_id, group in (("A2B-07", "negative_control"),
                            ("A2B-16", "negative_control"),
                            ("A2B-19", "smac_long_tail_control")):
        graph = query_graphs[query_id]
        off_path = [edge for edge in sorted(exclusive) if edge not in set(graph.baseline_edge_ids)]
        target = off_path[(seed + int(query_id[-2:])) % len(off_path)]
        scenario = parent.Scenario(
            f"CTRL-{query_id}", query_id, group, group, (target,),
            {target: exclusive[target]},
            frozen_failure_code=str(graph.frozen.get("failure_code", "")),
        )
        specs.append(ScenarioSpec(scenario, "control", 1, 1 / edge_count))
    main_queries = sorted({spec.scenario.query_id for spec in specs if spec.scenario.analysis_group == "main"})
    if len(main_queries) != main_query_count:
        raise RuntimeError(f"expected {main_query_count} distinct main queries, got {main_queries}")
    return specs, main_queries


def _correctness_rows(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    grouped: Dict[Tuple[str, str, int, int], Dict[str, Mapping[str, Any]]] = {}
    for row in rows:
        if row.get("run_mode") != "measured":
            continue
        key = (str(row["scenario_id"]), str(row["query_id"]),
               int(row["repetition"]), int(row["snapshot_index"]))
        grouped.setdefault(key, {})[str(row["arm"])] = row
    result = []
    for key, arms in sorted(grouped.items()):
        incremental, oracle = arms[value.INCREMENTAL_DSTAR], arms[value.COLD_GRAPH_ASTAR]
        error = abs(float(incremental["path_cost"]) - float(oracle["path_cost"])) if oracle["reachable"] else 0.0
        item = {
            "scenario_id": key[0], "query_id": key[1], "repetition": key[2],
            "snapshot_index": key[3], "snapshot_id": oracle["snapshot_id"],
            "analysis_group": oracle["analysis_group"],
            "input_hash_match": incremental["algorithm_input_hash"] == oracle["algorithm_input_hash"],
            "edge_status_hash_match": incremental["edge_status_hash"] == oracle["edge_status_hash"],
            "edge_cost_hash_match": incremental["edge_cost_hash"] == oracle["edge_cost_hash"],
            "reachable_parity": incremental["reachable"] == oracle["reachable"],
            "incremental_cost_error": error, "cost_parity": error == 0.0,
            "route_edge_ids_equal": incremental["path_edge_ids"] == oracle["path_edge_ids"],
            "blocked_edge_absent": not incremental["blocked_edges_in_path"] and not oracle["blocked_edges_in_path"],
            "no_route_classification_match": all(
                (arm["failure_code"] == "L1_NO_ROUTE") == (not bool(oracle["reachable"]))
                for arm in (incremental, oracle)
            ),
            "incremental_no_reinitialize": (
                int(incremental["reinitialize_call_count"]) == 0
                and not bool(incremental["implicit_reinitialize"])
                and (key[3] == 0 or bool(incremental["planner_identity_stable"]))
            ),
            "topology_immutable": all(
                not bool(arm["static_topology_mutated"]) and not bool(arm["static_map_mutated"])
                for arm in (incremental, oracle)
            ),
        }
        item["all_correct"] = all(bool(item[field]) for field in (
            "input_hash_match", "edge_status_hash_match", "edge_cost_hash_match",
            "reachable_parity", "cost_parity", "route_edge_ids_equal",
            "blocked_edge_absent", "no_route_classification_match",
            "incremental_no_reinitialize", "topology_immutable",
        ))
        result.append(item)
    return result


def _timing_summaries(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    phases = ("snapshot_parse_ms", "changed_cell_to_edge_mapping_ms", "edge_state_transition_ms",
              "update_edges_ms", "compute_shortest_path_ms", "route_extraction_ms",
              "algorithm_wall_ms", "full_incremental_l1_ms")
    filters = {
        "main_initial": lambda row: row["analysis_group"] == "main" and row["initial_plan"],
        "main_dynamic": lambda row: row["analysis_group"] == "main" and row["dynamic_update"],
        "path_affected": lambda row: row["analysis_group"] == "main" and row["dynamic_update"] and row["path_intersection"],
        "path_unaffected": lambda row: row["analysis_group"] == "main" and row["dynamic_update"] and not row["path_intersection"],
        "no_route": lambda row: row["analysis_group"] == "main" and row["dynamic_update"] and row["reachable"] is False,
        "recovery": lambda row: row["analysis_group"] == "main" and row["dynamic_update"] and row["scenario_category"] == "disappearance_recovery",
    }
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    result = []
    for group, predicate in filters.items():
        for arm in PRIMARY_ARMS:
            selected = [row for row in measured if row["arm"] == arm and predicate(row)]
            for phase in phases:
                result.append({"group": group, "arm": arm, "metric": phase,
                               **_summary([float(row[phase]) for row in selected])})
    return result


def _expanded_summaries(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    groups = {
        "main_dynamic": lambda row: row["analysis_group"] == "main" and row["dynamic_update"],
        "path_affected": lambda row: row["analysis_group"] == "main" and row["dynamic_update"] and row["path_intersection"],
        "path_unaffected": lambda row: row["analysis_group"] == "main" and row["dynamic_update"] and not row["path_intersection"],
        "no_route": lambda row: row["analysis_group"] == "main" and row["dynamic_update"] and row["reachable"] is False,
        "recovery": lambda row: row["analysis_group"] == "main" and row["dynamic_update"] and row["scenario_category"] == "disappearance_recovery",
    }
    return [
        {"group": group, "arm": arm, "metric": "expanded_nodes",
         **_summary([float(row["expanded_nodes"]) for row in measured if row["arm"] == arm and predicate(row)])}
        for group, predicate in groups.items() for arm in PRIMARY_ARMS
    ]


def _break_even(rows: Sequence[Mapping[str, Any]], family: str) -> List[Dict[str, Any]]:
    selected = [
        row for row in rows if row.get("run_mode") == "measured"
        and row["analysis_group"] == "main" and row["dynamic_update"]
        and row.get("scale_family") == family
    ]
    result = []
    for requested in sorted({int(row["requested_changed_edges"]) for row in selected}):
        bucket = [row for row in selected if int(row["requested_changed_edges"]) == requested]
        inc = [row for row in bucket if row["arm"] == value.INCREMENTAL_DSTAR]
        astar = [row for row in bucket if row["arm"] == value.COLD_GRAPH_ASTAR]
        inc_wall = _percentile([float(row["full_incremental_l1_ms"]) for row in inc], 0.50)
        astar_wall = _percentile([float(row["full_incremental_l1_ms"]) for row in astar], 0.50)
        inc_exp = _percentile([float(row["expanded_nodes"]) for row in inc], 0.50)
        astar_exp = _percentile([float(row["expanded_nodes"]) for row in astar], 0.50)
        result.append({
            "scale_family": family, "requested_changed_edges": requested,
            "observed_changed_edges_p50": _percentile([float(row["changed_edge_count"]) for row in inc], 0.50),
            "topology_edge_count": int(bucket[0]["topology_edge_count"]),
            "requested_changed_edges_ratio": float(bucket[0]["requested_changed_ratio"]),
            "actual_changed_edges_ratio": requested / int(bucket[0]["topology_edge_count"]),
            "incremental_dstar_wall_p50_ms": inc_wall,
            "cold_graph_astar_wall_p50_ms": astar_wall,
            "astar_over_dstar_wall_speedup": astar_wall / inc_wall if inc_wall else float("inf"),
            "incremental_dstar_expanded_p50": inc_exp,
            "cold_graph_astar_expanded_p50": astar_exp,
            "expanded_node_ratio": inc_exp / astar_exp if astar_exp else 0.0,
        })
    return result


def _stage_a_gates(rows: Sequence[Mapping[str, Any]], correctness: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    main_correct = [row for row in correctness if row["analysis_group"] == "main"]
    measured = [row for row in rows if row.get("run_mode") == "measured" and row["analysis_group"] == "main" and row["dynamic_update"]]
    inc = [row for row in measured if row["arm"] == value.INCREMENTAL_DSTAR]
    astar = [row for row in measured if row["arm"] == value.COLD_GRAPH_ASTAR]
    affected_inc = [float(row["expanded_nodes"]) for row in inc if row["path_intersection"]]
    affected_astar = [float(row["expanded_nodes"]) for row in astar if row["path_intersection"]]
    inc_p50 = _percentile([float(row["full_incremental_l1_ms"]) for row in inc], 0.50)
    astar_p50 = _percentile([float(row["full_incremental_l1_ms"]) for row in astar], 0.50)
    inc_p95 = _percentile([float(row["full_incremental_l1_ms"]) for row in inc], 0.95)
    astar_p95 = _percentile([float(row["full_incremental_l1_ms"]) for row in astar], 0.95)
    inc_exp = _percentile(affected_inc, 0.50)
    astar_exp = _percentile(affected_astar, 0.50)
    expanded_reduction = 1.0 - inc_exp / astar_exp
    wall_reduction = 1.0 - inc_p50 / astar_p50
    correctness_pass = bool(main_correct) and all(row["all_correct"] for row in main_correct)
    no_reinit = all(int(row["reinitialize_call_count"]) == 0 and not row["implicit_reinitialize"] for row in inc)
    result = {
        "correctness_pass": correctness_pass, "correctness_rows": len(main_correct),
        "correctness_failures": sum(not row["all_correct"] for row in main_correct),
        "oracle_rows_total": len(correctness),
        "oracle_failures_total": sum(not row["all_correct"] for row in correctness),
        "max_cost_error": max((float(row["incremental_cost_error"]) for row in main_correct), default=float("nan")),
        "incremental_no_reinitialize": no_reinit,
        "path_affected_incremental_expanded_p50": inc_exp,
        "path_affected_cold_astar_expanded_p50": astar_exp,
        "expanded_nodes_p50_reduction": expanded_reduction,
        "expanded_nodes_gate_pass": expanded_reduction >= 0.50,
        "incremental_full_l1_p50_ms": inc_p50,
        "cold_graph_astar_full_l1_p50_ms": astar_p50,
        "full_l1_p50_reduction": wall_reduction,
        "full_l1_p50_gate_pass": wall_reduction >= 0.30,
        "incremental_full_l1_p95_ms": inc_p95,
        "cold_graph_astar_full_l1_p95_ms": astar_p95,
        "p95_gate_pass": inc_p95 <= astar_p95,
    }
    result["stage_a_pass"] = all((correctness_pass, no_reinit,
        result["expanded_nodes_gate_pass"], result["full_l1_p50_gate_pass"], result["p95_gate_pass"]))
    return result


def _stage_b_allowed(gates: Mapping[str, Any], stage_a_only: bool) -> bool:
    """Single admission point: ROS is impossible after any Stage-A failure."""
    return bool(gates.get("stage_a_pass")) and not bool(stage_a_only)


def _per_scenario(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    measured = [row for row in rows if row.get("run_mode") == "measured" and row["dynamic_update"]]
    result = []
    for scenario_id in sorted({str(row["scenario_id"]) for row in measured}):
        for arm in PRIMARY_ARMS:
            selected = [row for row in measured if row["scenario_id"] == scenario_id and row["arm"] == arm]
            result.append({
                "scenario_id": scenario_id, "query_id": selected[0]["query_id"],
                "category": selected[0]["scenario_category"], "scale_family": selected[0]["scale_family"],
                "arm": arm, **_summary([float(row["full_incremental_l1_ms"]) for row in selected]),
                "expanded_p50": _percentile([float(row["expanded_nodes"]) for row in selected], 0.50),
                "path_intersection_rate": statistics.fmean(bool(row["path_intersection"]) for row in selected),
                "route_changed_rate": statistics.fmean(bool(row["route_changed"]) for row in selected),
                "no_route_count": sum(row["reachable"] is False for row in selected),
            })
    return result


def _memory_summary(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    result = []
    for group, predicate in (("S0_initial", lambda row: row["initial_plan"]),
                             ("dynamic", lambda row: row["dynamic_update"])):
        for arm in PRIMARY_ARMS:
            values = [float(row["state_memory_bytes"]) for row in measured if row["arm"] == arm and predicate(row)]
            result.append({"group": group, "arm": arm, **_summary(values), "peak": max(values)})
    return result


def _one_x_scale_row() -> Dict[str, Any]:
    ctx = parent.task_source._context()
    artifact, info, _audit = parent.r2_benchmark._load_frozen_r1_topology(ctx, parent.DEFAULT_CACHE_ROOT)
    occupancy = np.asarray(ctx.hospital_map.occupancy)
    route_counts = []
    with (FROZEN_R2 / "runs.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("run_mode") == "measured" and row.get("repetition") == "1":
                edges = json.loads(row.get("topology_edge_ids") or "[]")
                if edges:
                    route_counts.append(len(edges))
    cache_rows = {row["component"]: row for row in csv.DictReader((ONE_X_EXPERIMENT / "cache_and_state_diagnostics.csv").open())}
    return {
        "scale": "1x", "map_id": "mentor_map_20260825_005",
        "width": ctx.hospital_map.width, "height": ctx.hospital_map.height,
        "total_cells": int(occupancy.size), "free_cells": int((occupancy == 0).sum()),
        "occupied_cells": int((occupancy == 100).sum()), "unknown_cells": int((occupancy < 0).sum()),
        "raw_free_area_m2": float((occupancy == 0).sum()) * 0.0025,
        "traversable_cells": int(artifact.free_mask.sum()),
        "traversable_area_m2": float(artifact.free_mask.sum()) * 0.0025,
        "topology_nodes": len(artifact.graph.nodes), "topology_edges": len(artifact.graph.edges),
        "topology_components": artifact.graph.components,
        "route_edges_mean": statistics.fmean(route_counts),
        "route_edges_p50": _percentile(route_counts, 0.50), "route_edges_p95": _percentile(route_counts, 0.95),
        "topology_cache_bytes": int(cache_rows["frozen_topology_cache"]["memory_bytes"]),
        "endpoint_cache_bytes": int(cache_rows["r2_endpoint_attachment_cache"]["memory_bytes"]),
        "cell_to_edge_index_bytes": int(cache_rows["cell_to_edge_index"]["memory_bytes"]),
        "topology_cache_key": info["topology_cache_key"], "map_sha256": ctx.map_sha256,
    }


def _four_x_scale_row(ctx: Any, artifact: Any, topology_info: Mapping[str, Any],
                      query_graphs: Mapping[str, parent.QueryGraph], pipeline: Any,
                      index: value.CellToEdgeIndex) -> Dict[str, Any]:
    occupancy = np.asarray(ctx.hospital_map.occupancy)
    route_counts = [len(graph.baseline_edge_ids) for graph in query_graphs.values() if graph.baseline_edge_ids]
    return {
        "scale": "4x_area", "map_id": MAP_ID, "width": ctx.hospital_map.width,
        "height": ctx.hospital_map.height, "total_cells": int(occupancy.size),
        "free_cells": int((occupancy == 0).sum()), "occupied_cells": int((occupancy == 100).sum()),
        "unknown_cells": int((occupancy < 0).sum()),
        "raw_free_area_m2": float((occupancy == 0).sum()) * 0.0025,
        "traversable_cells": int(artifact.free_mask.sum()),
        "traversable_area_m2": float(artifact.free_mask.sum()) * 0.0025,
        "topology_nodes": len(artifact.graph.nodes), "topology_edges": len(artifact.graph.edges),
        "topology_components": artifact.graph.components,
        "route_edges_mean": statistics.fmean(route_counts),
        "route_edges_p50": _percentile(route_counts, 0.50), "route_edges_p95": _percentile(route_counts, 0.95),
        "topology_cache_bytes": int(topology_info["topology_cache_bytes"]),
        "endpoint_cache_bytes": int(pipeline.endpoint_cache_memory_bytes),
        "cell_to_edge_index_bytes": int(index.memory_bytes),
        "topology_cache_key": topology_info["topology_cache_key"], "map_sha256": ctx.map_sha256,
    }


def _snapshot_sources(output: Path, event_paths: Sequence[Path]) -> Dict[str, Any]:
    source_dir = output / "source_snapshot"
    source_dir.mkdir()
    files = [
        Path(__file__).resolve(), Path(value.__file__).resolve(), Path(parent.__file__).resolve(),
        Path(value.__file__).resolve().with_name("graph_dstar_lite.py"),
        Path(value.__file__).resolve().with_name("dynamic_snapshot.py"),
        Path(r1_pipeline.__file__).resolve(), Path(r2_pipeline.__file__).resolve(),
        Path(topology.__file__).resolve(), Path(legacy.__file__).resolve(),
        Path(__file__).resolve().parents[1] / "setup.py",
        Path(__file__).resolve().parents[1] / "config/two_layer_2d_v1_r4_dynamic_incremental_4x_area.yaml",
        Path(__file__).resolve().parents[1] / "test/test_two_layer_2d_v1_4x_dynamic_incremental_benchmark.py",
        FOUR_X_MAP_YAML, FOUR_X_MAP_PGM, FOUR_X_SCENARIO,
        FOUR_X_CACHE / "cache_manifest.yaml", FOUR_X_CACHE / "topology_metadata.yaml",
        FOUR_X_CACHE / "topology_graph.json", FOUR_X_CACHE / "topology_arrays.npz",
        ONE_X_EXPERIMENT / "final_report.md", ONE_X_EXPERIMENT / "manifest.yaml",
        ONE_X_EXPERIMENT / "break_even_curve.csv", *event_paths,
    ]
    records = []
    for index, source in enumerate(files):
        if not source.is_file():
            raise FileNotFoundError(source)
        target = source_dir / f"{index:02d}_{source.name}"
        shutil.copyfile(source, target)
        records.append({"source": str(source), "snapshot": str(target.relative_to(output)),
                        "sha256": sha256_file(target), "bytes": target.stat().st_size})
    combined = value.stable_hash([[row["snapshot"], row["sha256"]] for row in records])
    manifest = {"schema_version": 1, "file_count": len(records), "files": records, "combined_hash": combined}
    (output / "source_snapshot_manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    return manifest


REQUIRED = (
    "final_report.md", "protocol.yaml", "manifest.yaml", "verification.yaml", "runs.csv",
    "timing_summary.csv", "expanded_nodes_summary.csv", "break_even_curve_absolute.csv",
    "break_even_curve_ratio.csv", "correctness_oracle.csv", "per_scenario_summary.csv",
    "topology_scale_comparison.csv", "memory_summary.csv", "source_snapshot_manifest.yaml",
    "stdout.log", "stderr.log", "reproduction_command.txt",
)


def _validate_artifacts(output: Path) -> Dict[str, Any]:
    missing = [name for name in REQUIRED if not (output / name).is_file()]
    bad_sources = []
    manifest_path = output / "source_snapshot_manifest.yaml"
    source_count = 0
    if manifest_path.is_file():
        payload = yaml.safe_load(manifest_path.read_text()) or {}
        source_count = len(payload.get("files") or [])
        for row in payload.get("files") or []:
            path = output / row["snapshot"]
            if not path.is_file() or sha256_file(path) != row["sha256"]:
                bad_sources.append(str(row["snapshot"]))
    streams = list((output / "dynamic_event_streams").glob("*.json"))
    bad_streams = []
    for path in streams:
        payload = json.loads(path.read_text())
        if len(payload.get("snapshots") or []) != 21:
            bad_streams.append(path.name)
    return {"required_count": len(REQUIRED), "missing": missing,
            "event_stream_count": len(streams), "bad_event_streams": bad_streams,
            "source_snapshot_file_count": source_count, "bad_source_hashes": bad_sources,
            "passed": not missing and not bad_sources and not bad_streams and bool(streams)}


def _report(output: Path, gates: Mapping[str, Any], timing: Sequence[Mapping[str, Any]],
            expanded: Sequence[Mapping[str, Any]], absolute: Sequence[Mapping[str, Any]],
            ratio: Sequence[Mapping[str, Any]], scale: Sequence[Mapping[str, Any]],
            stage_b_status: str, main_queries: Sequence[str]) -> str:
    timing_index = {(row["group"], row["arm"], row["metric"]): row for row in timing}
    expanded_index = {(row["group"], row["arm"]): row for row in expanded}
    def tv(group: str, arm: str, metric: str, p: str) -> float:
        return float(timing_index[(group, arm, metric)][p])
    verdict = "C" if not gates["stage_a_pass"] else ("A" if stage_b_status == "PASSED_ENGINEERING_GATE" else "B")
    lines = [
        "# 2D-V1 persistent D* Lite 4x-area scale experiment", "",
        f"- Final verdict: **{verdict}**.",
        f"- Stage A correctness: {gates['correctness_rows'] - gates['correctness_failures']}/{gates['correctness_rows']}; pass={gates['correctness_pass']}.",
        f"- All groups (including controls): {gates['oracle_rows_total'] - gates['oracle_failures_total']}/{gates['oracle_rows_total']} oracle rows.",
        f"- Stage A value gate: pass={gates['stage_a_pass']}; Stage B: `{stage_b_status}`.",
        f"- Main query set ({len(main_queries)}): {', '.join(main_queries)}.", "",
        "## Scale audit", "",
        "| scale | grid | free/traversable area m² | topology nodes/edges/components | route edges mean/P50/P95 | topology/endpoint/cell-index bytes |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in scale:
        lines.append(f"| {row['scale']} | {row['width']}×{row['height']} | {row['raw_free_area_m2']:.2f}/{row['traversable_area_m2']:.2f} | {row['topology_nodes']}/{row['topology_edges']}/{row['topology_components']} | {row['route_edges_mean']:.2f}/{row['route_edges_p50']:.1f}/{row['route_edges_p95']:.1f} | {row['topology_cache_bytes']}/{row['endpoint_cache_bytes']}/{row['cell_to_edge_index_bytes']} |")
    lines.extend(["", "## Stage A timing", "",
                  "| arm | S0 full L1 P50/P95/P99 ms | dynamic full L1 P50/P95/P99 ms | search P50 ms | expanded P50 |",
                  "|---|---:|---:|---:|---:|"])
    for arm in PRIMARY_ARMS:
        lines.append(
            f"| `{arm}` | {tv('main_initial',arm,'full_incremental_l1_ms','p50'):.4f}/{tv('main_initial',arm,'full_incremental_l1_ms','p95'):.4f}/{tv('main_initial',arm,'full_incremental_l1_ms','p99'):.4f} | "
            f"{tv('main_dynamic',arm,'full_incremental_l1_ms','p50'):.4f}/{tv('main_dynamic',arm,'full_incremental_l1_ms','p95'):.4f}/{tv('main_dynamic',arm,'full_incremental_l1_ms','p99'):.4f} | "
            f"{tv('main_dynamic',arm,'compute_shortest_path_ms','p50'):.4f} | {expanded_index[('main_dynamic',arm)]['p50']:.1f} |"
        )
    lines.extend(["", "## Frozen gates", "",
        f"- Path-affected expanded P50 reduction: {100*gates['expanded_nodes_p50_reduction']:.2f}% (pass={gates['expanded_nodes_gate_pass']}).",
        f"- Full L1 P50 reduction: {100*gates['full_l1_p50_reduction']:.2f}% (pass={gates['full_l1_p50_gate_pass']}).",
        f"- P95 incremental/cold A*: {gates['incremental_full_l1_p95_ms']:.4f}/{gates['cold_graph_astar_full_l1_p95_ms']:.4f} ms (pass={gates['p95_gate_pass']}).",
        "- Complete L1 includes parse, cell-to-edge mapping, state transition, edge update, search, and extraction.",
        "", "## Absolute break-even", "",
        "| edges | ratio | A*/D* speedup | D*/A* expanded ratio |", "|---:|---:|---:|---:|",
    ])
    for row in absolute:
        lines.append(f"| {row['requested_changed_edges']} | {row['actual_changed_edges_ratio']:.6f} | {row['astar_over_dstar_wall_speedup']:.3f}x | {row['expanded_node_ratio']:.3f} |")
    lines.extend(["", "## Ratio-matched break-even", "",
                  "| target ratio | actual edges/ratio | A*/D* speedup | D*/A* expanded ratio |",
                  "|---:|---:|---:|---:|"])
    for row in ratio:
        lines.append(f"| {100*row['requested_changed_edges_ratio']:.3f}% | {row['requested_changed_edges']}/{row['actual_changed_edges_ratio']:.6f} | {row['astar_over_dstar_wall_speedup']:.3f}x | {row['expanded_node_ratio']:.3f} |")
    lines.extend(["", "## Interpretation", "",
        "- Cold Graph A* rebuilds search state per snapshot but shares the same immutable adjacency as persistent D*.",
        "- Persistent D* reuses g/rhs/OPEN/km and receives changed edges only; implicit reinitialize is forbidden.",
        "- Path-unaffected scheduler skips, if Stage B runs, are not attributed to D*.",
        "- Conclusions are limited to this map, topology generator, attachment policy, and dynamic scenario distribution.",
        "", "## Reproduction", "", "```bash",
        "source /opt/ros/humble/setup.bash",
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash",
        "pln02_out=/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v1_dynamic_incremental_4x_area_v1_$(date +%Y%m%d_%H%M%S)",
        f"ROS_DOMAIN_ID={DEFAULT_ROS_DOMAIN_ID} ros2 run arena_evaluation two_layer_2d_v1_4x_dynamic_incremental_benchmark --output-dir \"$pln02_out\" --warmups {DEFAULT_WARMUPS} --repetitions {DEFAULT_REPETITIONS} --main-query-count {DEFAULT_MAIN_QUERY_COUNT} --seed {DEFAULT_SEED} --ros-repetitions {DEFAULT_ROS_REPETITIONS} --ros-domain-id {DEFAULT_ROS_DOMAIN_ID}",
        "```",
    ])
    (output / "final_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return verdict


def run_formal(output: Path, *, warmups: int = DEFAULT_WARMUPS,
               repetitions: int = DEFAULT_REPETITIONS,
               main_query_count: int = DEFAULT_MAIN_QUERY_COUNT,
               seed: int = DEFAULT_SEED, ros_domain_id: int = DEFAULT_ROS_DOMAIN_ID,
               ros_repetitions: int = DEFAULT_ROS_REPETITIONS,
               stage_a_only: bool = False) -> Path:
    output = output.resolve()
    _refuse_nonempty(output)
    if warmups < 3 or repetitions < 20:
        raise ValueError("formal run requires at least 3 warmups and 20 repetitions")
    if not 8 <= main_query_count <= 10:
        raise ValueError("main_query_count must be 8..10")
    one_x_hash_before, r2_hash_before = parent._tree_hash(ONE_X_EXPERIMENT), parent._tree_hash(FROZEN_R2)
    one_x_scale = _one_x_scale_row()
    ctx, queries, task_metadata, artifact, topology_info = _load_4x_inputs()
    static_reference = _load_static_reference()
    graph_view = r1_pipeline.build_static_topology_view(artifact)
    graph_view.metadata["topology_cache_key"] = topology_info["topology_cache_key"]
    query_graphs, pipeline = _build_query_graphs(graph_view, topology_info, ctx, queries, static_reference)
    edge_cells = parent._edge_cells(graph_view)
    exclusive, any_witness, _reverse = parent._witness_maps(edge_cells)
    specs, main_queries = _build_scenarios(
        query_graphs, exclusive, any_witness, edge_count=len(edge_cells),
        seed=seed, main_query_count=main_query_count,
    )
    output.mkdir(parents=True)
    events_dir = output / "dynamic_event_streams"
    events_dir.mkdir()
    event_payloads: Dict[str, List[str]] = {}
    event_paths = []
    scenario_rows = []
    for spec in specs:
        scenario = spec.scenario
        path, payloads = parent._write_event_stream(
            events_dir, scenario, map_version=ctx.map_sha256,
            map_shape=artifact.free_mask.shape, seed=seed,
        )
        event_paths.append(path); event_payloads[scenario.scenario_id] = payloads
        graph = query_graphs[scenario.query_id]
        scenario_rows.append({**asdict(scenario), "scale_family": spec.scale_family,
            "requested_changed_edges": spec.requested_changed_edges,
            "requested_changed_ratio": spec.requested_changed_ratio,
            "static_reference_final_valid": static_reference[scenario.query_id]["final_valid"],
            "r4_l1_static_reachable": bool(graph.baseline_edge_ids),
            "static_route_edge_count": len(graph.baseline_edge_ids), "static_route_cost": graph.baseline_cost,
            "event_stream": str(path.relative_to(output)), "snapshot_count": 21, "seed": seed})
    parent._write_csv(output / "scenario_manifest.csv", scenario_rows)

    all_rows: List[Dict[str, Any]] = []
    spec_by_id = {spec.scenario.scenario_id: spec for spec in specs}
    for run_mode, count in (("warmup", warmups), ("measured", repetitions)):
        for repetition in range(1, count + 1):
            for scenario_index, spec in enumerate(specs):
                scenario = spec.scenario
                order = list(PRIMARY_ARMS)
                if (repetition + scenario_index) % 2:
                    order.reverse()
                rows = value.run_paired_episode(
                    query_graphs[scenario.query_id].template,
                    event_payloads[scenario.scenario_id], edge_cells,
                    map_version=ctx.map_sha256, map_shape=artifact.free_mask.shape,
                    arm_order=order,
                )
                parent._annotate_rows(rows, scenario, query_graphs[scenario.query_id],
                                      run_mode=run_mode, repetition=repetition,
                                      topology_edge_count=len(edge_cells))
                for row in rows:
                    row.update({"implementation_revision": IMPLEMENTATION_REVISION,
                        "parent_architecture": PARENT_ARCHITECTURE,
                        "experiment_kind": EXPERIMENT_KIND,
                        "scale_family": spec.scale_family,
                        "requested_changed_edges": spec.requested_changed_edges,
                        "requested_changed_ratio": spec.requested_changed_ratio})
                all_rows.extend(rows)
    parent._write_csv(output / "runs.csv", all_rows)
    correctness = _correctness_rows(all_rows)
    timing = _timing_summaries(all_rows)
    expanded = _expanded_summaries(all_rows)
    absolute = _break_even(all_rows, "absolute")
    ratio = _break_even(all_rows, "ratio")
    gates = _stage_a_gates(all_rows, correctness)
    per_scenario = _per_scenario(all_rows)
    memory = _memory_summary(all_rows)
    index = value.CellToEdgeIndex(edge_cells)
    scale_rows = [one_x_scale, _four_x_scale_row(ctx, artifact, topology_info, query_graphs, pipeline, index)]
    parent._write_csv(output / "correctness_oracle.csv", correctness)
    parent._write_csv(output / "timing_summary.csv", timing)
    parent._write_csv(output / "expanded_nodes_summary.csv", expanded)
    parent._write_csv(output / "break_even_curve_absolute.csv", absolute)
    parent._write_csv(output / "break_even_curve_ratio.csv", ratio)
    parent._write_csv(output / "per_scenario_summary.csv", per_scenario)
    parent._write_csv(output / "topology_scale_comparison.csv", scale_rows)
    parent._write_csv(output / "memory_summary.csv", memory)

    ros_rows: List[Dict[str, Any]] = []
    stage_b_summary: Dict[str, Any] = {}
    if not gates["stage_a_pass"]:
        stage_b_status = "NOT_RUN_STAGE_A_FAILED"
    elif stage_a_only:
        stage_b_status = "NOT_RUN_STAGE_A_ONLY_REQUESTED"
    elif _stage_b_allowed(gates, stage_a_only):
        old_map_yaml, old_map_id = parent.r2_benchmark.MAP_YAML, parent.MAP_ID
        try:
            parent.r2_benchmark.MAP_YAML = FOUR_X_MAP_YAML
            parent.MAP_ID = MAP_ID
            ros_rows, stage_b_status, stage_b_summary, ros_timing = parent._run_ros_stage_b(
                output, [spec.scenario for spec in specs], query_graphs, event_payloads,
                edge_cells, graph_view, topology_info, ctx,
                ros_domain_id=ros_domain_id, repetitions=ros_repetitions,
            )
            for row in ros_rows:
                row.update({"implementation_revision": IMPLEMENTATION_REVISION,
                            "parent_architecture": PARENT_ARCHITECTURE,
                            "experiment_kind": EXPERIMENT_KIND, "map_id": MAP_ID})
            timing.extend(ros_timing)
            parent._write_csv(output / "timing_summary.csv", timing)
        finally:
            parent.r2_benchmark.MAP_YAML, parent.MAP_ID = old_map_yaml, old_map_id
    parent._write_csv(output / "paired_ros_runs.csv", ros_rows or [{"status": stage_b_status,
        "reason": "Stage A did not satisfy all frozen gates" if not gates["stage_a_pass"] else "stage-a-only requested"}])

    protocol = {
        "experiment_id": output.name, "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION, "parent_architecture": PARENT_ARCHITECTURE,
        "experiment_kind": EXPERIMENT_KIND, "protocol_version": PROTOCOL_VERSION,
        "map_id": MAP_ID, "resolution_m": 0.05, "dynamic_obstacles": True,
        "primary_arms": list(PRIMARY_ARMS), "diagnostic_cold_dstar": "not_run",
        "dynamic_state_machine": ["AVAILABLE", "BLOCKED_PENDING", "BLOCKED", "RECOVERING", "AVAILABLE"],
        "blocked_cost": "INF", "static_map_mutation": False, "static_topology_mutation": False,
        "absolute_change_points": list(ABSOLUTE_POINTS), "ratio_change_points": ratio_change_points(len(edge_cells)),
        "episode": {"initial_snapshot": "S0", "dynamic_snapshots": "S1..S20", "pipeline_rebuild_within_episode": False},
        "warmups": warmups, "repetitions": repetitions, "seed": seed,
        "stage_a_frozen_gates": {"correctness": 1.0, "path_affected_expanded_reduction": 0.50,
            "full_l1_p50_reduction": 0.30, "p95_no_regression": True, "implicit_reinitialize_allowed": False},
        "stage_a_result": gates, "stage_b_status": stage_b_status,
        "layers": {"L1": "persistent D* Lite vs deterministic cold Graph A*", "L2": "disabled",
                   "L3_prime": "Nav2 SmacPlannerHybrid/DUBIN only if Stage A passes"},
        "motion": {"minimum_turning_radius_m": 0.4, "maximum_curvature_1pm": 2.5,
                   "allow_reverse": False, "allow_in_place_rotation": False},
        "cache_binding": topology_info["cache_binding"], "task_metadata": task_metadata,
        "metric_availability": {"queue_peak_size": "not_available: GraphDStarLite does not retain a reliable per-search peak",
                                "stale_queue_entries": "not_available: not separately counted by the frozen core"},
        "ros_domain_id": ros_domain_id,
    }
    (output / "protocol.yaml").write_text(yaml.safe_dump(protocol, sort_keys=False), encoding="utf-8")
    verdict = _report(output, gates, timing, expanded, absolute, ratio, scale_rows,
                      stage_b_status, main_queries)
    reproduction = (
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        "pln02_out=/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/"
        "2d_v1_dynamic_incremental_4x_area_v1_$(date +%Y%m%d_%H%M%S)\n"
        f"ROS_DOMAIN_ID={ros_domain_id} ros2 run arena_evaluation two_layer_2d_v1_4x_dynamic_incremental_benchmark "
        f"--output-dir \"$pln02_out\" --warmups {warmups} --repetitions {repetitions} "
        f"--main-query-count {main_query_count} --seed {seed} --ros-repetitions {ros_repetitions} --ros-domain-id {ros_domain_id}\n"
    )
    (output / "reproduction_command.txt").write_text(reproduction, encoding="utf-8")
    (output / "stdout.log").write_text(
        f"output={output}\nverdict={verdict}\nstage_a_pass={gates['stage_a_pass']}\nstage_b_status={stage_b_status}\n",
        encoding="utf-8")
    (output / "stderr.log").write_text("", encoding="utf-8")
    source_snapshot = _snapshot_sources(output, event_paths)
    one_x_hash_after, r2_hash_after = parent._tree_hash(ONE_X_EXPERIMENT), parent._tree_hash(FROZEN_R2)
    if one_x_hash_after != one_x_hash_before or r2_hash_after != r2_hash_before:
        raise RuntimeError("a frozen reference directory changed during the 4x experiment")
    verification = {
        "formal_run_complete": True, "stage_a": gates, "stage_b_status": stage_b_status,
        "all_correctness_rows_pass": all(row["all_correct"] for row in correctness),
        "source_snapshot_hash": source_snapshot["combined_hash"],
        "one_x_tree_hash_before": one_x_hash_before, "one_x_tree_hash_after": one_x_hash_after,
        "frozen_r2_tree_hash_before": r2_hash_before, "frozen_r2_tree_hash_after": r2_hash_after,
        "frozen_references_unchanged": one_x_hash_before == one_x_hash_after and r2_hash_before == r2_hash_after,
        "post_run_commands": "pending",
    }
    (output / "verification.yaml").write_text(yaml.safe_dump(verification, sort_keys=False), encoding="utf-8")
    manifest = {
        "experiment_id": output.name, "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION, "parent_architecture": PARENT_ARCHITECTURE,
        "experiment_kind": EXPERIMENT_KIND, "protocol_version": PROTOCOL_VERSION,
        "formal": True, "final_verdict": verdict, "stage_a": gates,
        "stage_b_status": stage_b_status, "stage_b": stage_b_summary,
        "map_id": MAP_ID, "map_pgm_sha256": sha256_file(FOUR_X_MAP_PGM),
        "map_yaml_sha256": sha256_file(FOUR_X_MAP_YAML), "map_combined_sha256": ctx.map_sha256,
        "query_sha256": sha256_file(FOUR_X_SCENARIO), "topology_cache": topology_info,
        "topology_nodes": len(graph_view.nodes), "topology_edges": len(graph_view.edges),
        "main_query_ids": main_queries, "scenario_count": len(specs),
        "snapshots_per_episode": 21, "warmups": warmups, "measured_repetitions": repetitions,
        "algorithm_row_count": len(all_rows), "correctness_row_count": len(correctness),
        "one_x_reference": str(ONE_X_EXPERIMENT), "frozen_r2_reference": str(FROZEN_R2),
        "one_x_tree_hash_before": one_x_hash_before, "one_x_tree_hash_after": one_x_hash_after,
        "frozen_r2_tree_hash_before": r2_hash_before, "frozen_r2_tree_hash_after": r2_hash_after,
        "source_snapshot_file_count": source_snapshot["file_count"],
        "source_snapshot_hash": source_snapshot["combined_hash"],
    }
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    validation = _validate_artifacts(output)
    if not validation["passed"]:
        raise RuntimeError(f"formal artifact validation failed: {validation}")
    manifest["artifact_validation"] = validation
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the formal 2D-V1-r4 4x-area dynamic incremental scale experiment")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--warmups", type=int, default=DEFAULT_WARMUPS)
    parser.add_argument("--repetitions", type=int, default=DEFAULT_REPETITIONS)
    parser.add_argument("--main-query-count", type=int, default=DEFAULT_MAIN_QUERY_COUNT)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--ros-domain-id", type=int, default=DEFAULT_ROS_DOMAIN_ID)
    parser.add_argument("--ros-repetitions", type=int, default=DEFAULT_ROS_REPETITIONS)
    parser.add_argument("--stage-a-only", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output = run_formal(args.output_dir or _default_output(), warmups=args.warmups,
                            repetitions=args.repetitions, main_query_count=args.main_query_count,
                            seed=args.seed, ros_domain_id=args.ros_domain_id,
                            ros_repetitions=args.ros_repetitions, stage_a_only=args.stage_a_only)
    except Exception as exc:
        print(f"two_layer_2d_v1_4x_dynamic_incremental_benchmark: ERROR: {exc}")
        return 2
    print(f"2D-V1 4x dynamic incremental output: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
