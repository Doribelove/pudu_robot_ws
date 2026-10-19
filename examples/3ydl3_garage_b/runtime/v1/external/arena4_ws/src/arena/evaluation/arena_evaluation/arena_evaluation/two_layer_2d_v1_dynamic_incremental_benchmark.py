"""Formal paired dynamic-incremental value experiment for 2D-V1-r3.

Stage A is deliberately ROS-free.  It compares a persistent Graph D* Lite,
fresh Graph D* Lite, and fresh deterministic Graph A* on identical dynamic
edge overlays.  Stage B is admitted only when every Stage-A correctness gate
and both algorithm-value gates pass.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import random
import resource
import shutil
import statistics
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

import yaml

from . import dynamic_incremental_value as value
from . import layered_2d_v0_pipeline as v0
from . import layered_2d_v1_pipeline as r1_pipeline
from . import layered_2d_v1_r2_pipeline as r2_pipeline
from . import canonical_path_validation as canonical
from . import two_layer_2d_v1_r2_formal_benchmark as r2_benchmark
from . import two_layer_v1_formal_benchmark as task_source
from . import unified_four_backends_smoke as legacy
from .dynamic_snapshot import DynamicSnapshot
from .graph_dstar_lite import GraphDStarLite
from .planner_benchmark.map_utils import sha256_file


ROOT = Path("/home/robot/pudu_robot_ws")
FROZEN_R2 = ROOT / "experiments/layered_planner_benchmark/2d_v1_mentor_map_20260825_005_20_performance_r2_20260903_1147"
DEFAULT_CACHE_ROOT = ROOT / "experiments/layered_planner_benchmark/2d_v1_mentor_map_20260825_005_20_performance_v1_cache"
DEFAULT_OUTPUT_ROOT = ROOT / "experiments/layered_planner_benchmark"
MAP_ID = "mentor_map_20260825_005"
FROZEN_TOPOLOGY_KEY = "af45cba4e2772b5d8209efdc171ad4672a48b3f01697d60f5d421ac821d42b4c"
DEFAULT_SEED = 20260903
DEFAULT_WARMUPS = 3
DEFAULT_REPETITIONS = 20
DEFAULT_MAIN_QUERY_COUNT = 8
DEFAULT_ROS_REPETITIONS = 3
EVALUATOR_DEADLINE_MS = 2500.0


def _elapsed_ms(started_ns: int) -> float:
    return (time.monotonic_ns() - started_ns) / 1.0e6


def _default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return DEFAULT_OUTPUT_ROOT / f"2d_v1_dynamic_incremental_value_v1_{stamp}"


def _refuse_nonempty(path: Path) -> None:
    if path.resolve() == FROZEN_R2.resolve():
        raise ValueError("the frozen 2D-V1-r2 baseline is read-only")
    if path.exists() and any(path.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output directory: {path}")


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]], fieldnames: Optional[Sequence[str]] = None) -> None:
    normalized: List[Dict[str, Any]] = []
    keys: List[str] = list(fieldnames or ())
    for row in rows:
        converted: Dict[str, Any] = {}
        for key, item in row.items():
            converted[str(key)] = (
                json.dumps(item, sort_keys=True, separators=(",", ":"), allow_nan=False)
                if isinstance(item, (dict, list, tuple)) else item
            )
            if str(key) not in keys:
                keys.append(str(key))
        normalized.append(converted)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(normalized)


def _tree_hash(directory: Path) -> str:
    payload = []
    for path in sorted(item for item in directory.rglob("*") if item.is_file()):
        payload.append([str(path.relative_to(directory)), sha256_file(path)])
    return value.stable_hash(payload)


def _baseline_source_audit() -> Dict[str, Any]:
    pairs = {
        "layered_2d_v1_r2_pipeline.py": "01_layered_2d_v1_r2_pipeline.py",
        "layered_2d_v1_pipeline.py": "03_layered_2d_v1_pipeline.py",
        "layered_2d_v0_pipeline.py": "04_layered_2d_v0_pipeline.py",
        "unified_four_backends_smoke.py": "05_unified_four_backends_smoke.py",
        "topology.py": "06_topology.py",
        "dynamic_snapshot.py": "07_dynamic_snapshot.py",
        "graph_dstar_lite.py": "08_graph_dstar_lite.py",
    }
    rows = []
    for current_name, frozen_name in pairs.items():
        current = Path(__file__).with_name(current_name)
        frozen = FROZEN_R2 / "source_snapshot" / frozen_name
        current_hash, frozen_hash = sha256_file(current), sha256_file(frozen)
        rows.append({
            "file": current_name, "current_sha256": current_hash,
            "frozen_r2_sha256": frozen_hash, "match": current_hash == frozen_hash,
        })
    return {
        "files": rows,
        "matching_files": sum(bool(row["match"]) for row in rows),
        "different_files": [row["file"] for row in rows if not row["match"]],
        "baseline_choice": (
            "frozen r2 topology, map and query artifacts; current r2 planner sources "
            "only where hash-identical; current ROS session source is snapshotted and its "
            "pre-existing workspace difference is disclosed"
        ),
    }


def _percentile(values: Sequence[float], q: float) -> float:
    ordered = sorted(float(item) for item in values if math.isfinite(float(item)))
    if not ordered:
        return float("nan")
    position = (len(ordered) - 1) * float(q)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _summary(values: Sequence[float]) -> Dict[str, float]:
    return {
        "count": len(values),
        "p50": _percentile(values, 0.50),
        "p95": _percentile(values, 0.95),
        "p99": _percentile(values, 0.99),
        "mean": statistics.fmean(values) if values else float("nan"),
    }


def _load_frozen_validity() -> Dict[str, Dict[str, Any]]:
    rows: Dict[str, Dict[str, Any]] = {}
    with (FROZEN_R2 / "runs.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("run_mode") != "measured" or row.get("repetition") != "1":
                continue
            rows[str(row["query_id"])] = {
                "final_valid": str(row.get("final_valid_success", "")).lower() == "true",
                "failure_code": str(row.get("failure_code", "")),
                "path_hash": str(row.get("path_hash", "")),
                "topology_edge_ids": json.loads(row.get("topology_edge_ids") or "[]"),
                "topology_node_ids": json.loads(row.get("topology_node_ids") or "[]"),
            }
    return rows


@dataclass
class QueryGraph:
    query: Any
    template: value.GraphTemplate
    frozen: Mapping[str, Any]
    baseline_edge_ids: Tuple[str, ...]
    baseline_cost: float
    attachment_diagnostics: Mapping[str, Any]


def _build_query_graphs(
    graph_view: Any,
    topology_info: Mapping[str, Any],
    ctx: Any,
    queries: Sequence[Any],
    frozen: Mapping[str, Mapping[str, Any]],
) -> Dict[str, QueryGraph]:
    snapshot = DynamicSnapshot.empty(
        snapshot_id="S0", timestamp=1.0, map_version=ctx.map_sha256,
        map_shape=graph_view.artifact.free_mask.shape,
    )
    pipeline = r2_pipeline.Layered2DV1R2Pipeline(
        graph_view, footprint=legacy.FOOTPRINT, l3_planner=None,
        corridor_padding_m=2.0, corridor_profile="padding",
        corridor_fallback_policy="bounded",
        base_map_hash=ctx.map_sha256,
        topology_cache_key=str(topology_info["topology_cache_key"]),
        topology_source_hash=str(topology_info["topology_source_hash"]),
        corridor_semantics="raw_map_smac_aligned",
    )
    result: Dict[str, QueryGraph] = {}
    for query in queries:
        starts, goals, diagnostics = pipeline._attach(query, snapshot)
        planner, _start, _goal, _positions = pipeline._make_graph(starts, goals, snapshot)
        template = value.GraphTemplate.from_dstar(planner)
        oracle = value.deterministic_graph_astar(template, {})
        edge_ids = tuple(
            edge_id for edge_id in oracle.edge_path if edge_id.startswith("topology_")
        )
        frozen_edges = tuple(frozen.get(query.query_id, {}).get("topology_edge_ids", ()))
        if edge_ids != frozen_edges:
            raise RuntimeError(
                f"{query.query_id} baseline route differs from frozen r2: "
                f"{len(edge_ids)} edges versus {len(frozen_edges)}"
            )
        result[query.query_id] = QueryGraph(
            query, template, frozen.get(query.query_id, {}), edge_ids,
            float(oracle.cost), diagnostics,
        )
    return result


def _edge_cells(graph_view: Any) -> Dict[str, Tuple[Tuple[int, int], ...]]:
    return {
        str(edge_id): tuple(tuple(cell) for cell in edge.edge_cells)
        for edge_id, edge in sorted(graph_view.edges.items())
    }


def _witness_maps(
    edge_cells: Mapping[str, Sequence[Tuple[int, int]]],
) -> Tuple[Dict[str, Tuple[int, int]], Dict[str, Tuple[int, int]], Dict[Tuple[int, int], Tuple[str, ...]]]:
    reverse: Dict[Tuple[int, int], List[str]] = {}
    for edge_id, cells in edge_cells.items():
        for cell in cells:
            reverse.setdefault(tuple(cell), []).append(str(edge_id))
    cell_edges = {cell: tuple(sorted(set(edges))) for cell, edges in reverse.items()}
    exclusive: Dict[str, Tuple[int, int]] = {}
    any_witness: Dict[str, Tuple[int, int]] = {}
    for edge_id, cells in sorted(edge_cells.items()):
        if cells:
            any_witness[edge_id] = tuple(cells[len(cells) // 2])
        candidates = [cell for cell in cells if cell_edges.get(tuple(cell)) == (edge_id,)]
        if candidates:
            exclusive[edge_id] = tuple(candidates[len(candidates) // 2])
    return exclusive, any_witness, cell_edges


def _blocked_oracle(query_graph: QueryGraph, edge_ids: Iterable[str]) -> value.GraphAStarResult:
    statuses = {str(edge_id): GraphDStarLite.BLOCKED for edge_id in edge_ids}
    return value.deterministic_graph_astar(query_graph.template, statuses)


def _minimum_static_cut(query_graph: QueryGraph) -> Tuple[str, ...]:
    """Compute an exact unit-capacity static-edge min cut using Dinic flow."""
    nodes = tuple(query_graph.template.nodes)
    index = {node: offset for offset, node in enumerate(nodes)}
    residual: List[List[List[Any]]] = [[] for _ in nodes]

    def add_arc(first: int, second: int, capacity: int, edge_id: str) -> None:
        forward = [index[second], int(capacity), len(residual[index[second]]), edge_id]
        backward = [index[first], 0, len(residual[index[first]]), edge_id]
        residual[index[first]].append(forward)
        residual[index[second]].append(backward)

    big = 10000
    for edge in query_graph.template.edges:
        capacity = 1 if str(edge.edge_id).startswith("topology_") else big
        add_arc(int(edge.source), int(edge.target), capacity, str(edge.edge_id))
        if edge.bidirectional:
            add_arc(int(edge.target), int(edge.source), capacity, str(edge.edge_id))
    source, sink = index[query_graph.template.start], index[query_graph.template.goal]
    flow = 0
    while flow < big:
        level = [-1] * len(nodes)
        level[source] = 0
        queue = [source]
        for current in queue:
            for target, capacity, _reverse, _edge_id in residual[current]:
                if capacity > 0 and level[target] < 0:
                    level[target] = level[current] + 1
                    queue.append(target)
        if level[sink] < 0:
            break
        cursor = [0] * len(nodes)

        def send(current: int, available: int) -> int:
            if current == sink:
                return available
            while cursor[current] < len(residual[current]):
                arc = residual[current][cursor[current]]
                target, capacity, reverse_index, _edge_id = arc
                if capacity > 0 and level[target] == level[current] + 1:
                    pushed = send(target, min(available, capacity))
                    if pushed:
                        arc[1] -= pushed
                        residual[target][reverse_index][1] += pushed
                        return pushed
                cursor[current] += 1
            return 0

        while flow < big:
            pushed = send(source, big - flow)
            if not pushed:
                break
            flow += pushed
    reachable = {source}
    queue = [source]
    for current in queue:
        for target, capacity, _reverse, _edge_id in residual[current]:
            if capacity > 0 and target not in reachable:
                reachable.add(target)
                queue.append(target)
    cut = {
        str(edge.edge_id) for edge in query_graph.template.edges
        if str(edge.edge_id).startswith("topology_")
        and ((index[edge.source] in reachable) != (index[edge.target] in reachable))
    }
    if not cut or _blocked_oracle(query_graph, cut).node_path is not None:
        raise RuntimeError(f"failed to construct verified static min cut for {query_graph.query.query_id}")
    return tuple(sorted(cut))


def _select_alternate_edge(
    query_graph: QueryGraph, exclusive: Mapping[str, Tuple[int, int]],
) -> Optional[str]:
    route = list(query_graph.baseline_edge_ids)
    ordered = route[len(route) // 4: 3 * len(route) // 4] + route[:len(route) // 4]
    for edge_id in ordered:
        if edge_id not in exclusive:
            continue
        candidate = _blocked_oracle(query_graph, (edge_id,))
        if candidate.node_path is not None and tuple(candidate.edge_path) != tuple(query_graph.baseline_edge_ids):
            return edge_id
    return None


@dataclass
class Scenario:
    scenario_id: str
    query_id: str
    category: str
    analysis_group: str
    target_edges: Tuple[str, ...]
    witness_cells: Mapping[str, Tuple[int, int]]
    min_cut_size: int = 0
    frozen_failure_code: str = ""


def _choose_group(
    candidates: Sequence[str], count: int, exclusive: Mapping[str, Tuple[int, int]], seed: int,
) -> Tuple[str, ...]:
    pool = [edge_id for edge_id in candidates if edge_id in exclusive]
    random.Random(seed).shuffle(pool)
    if len(pool) < count:
        raise RuntimeError(f"only {len(pool)} exclusive edge witnesses available, need {count}")
    return tuple(sorted(pool[:count]))


def _build_scenarios(
    query_graphs: Mapping[str, QueryGraph],
    frozen: Mapping[str, Mapping[str, Any]],
    exclusive: Mapping[str, Tuple[int, int]],
    any_witness: Mapping[str, Tuple[int, int]],
    *, seed: int, main_query_count: int,
) -> List[Scenario]:
    eligible = [
        graph for query_id, graph in sorted(query_graphs.items())
        if frozen.get(query_id, {}).get("final_valid")
        and query_id not in {"A2B-07", "A2B-16", "A2B-19"}
    ]
    if len(eligible) < main_query_count:
        raise RuntimeError("not enough frozen static-valid queries for dynamic selection")
    eligible.sort(key=lambda graph: (len(graph.baseline_edge_ids), graph.query.query_id))
    quantile_indices = sorted({
        round(index * (len(eligible) - 1) / max(1, main_query_count - 1))
        for index in range(main_query_count)
    })
    selected = [eligible[index] for index in quantile_indices]
    for graph in eligible:
        if len(selected) >= main_query_count:
            break
        if graph not in selected:
            selected.append(graph)
    selected = selected[:main_query_count]

    categories = [
        "outside_path", "path_nonbridge_alternate", "moving_e1_e2_e3",
        "disappearance_recovery", "changed_edges_5", "changed_edges_20",
        "alternate_channel_100", "bridge_or_min_cut_no_route",
    ][:main_query_count]
    scenarios: List[Scenario] = []
    all_exclusive = sorted(exclusive)
    for offset, (graph, category) in enumerate(zip(selected, categories)):
        route = list(graph.baseline_edge_ids)
        route_set = set(route)
        if category == "outside_path":
            targets = _choose_group(
                [edge for edge in all_exclusive if edge not in route_set], 1,
                exclusive, seed + offset,
            )
        elif category == "path_nonbridge_alternate":
            alternate = _select_alternate_edge(graph, exclusive)
            if alternate is None:
                replacement = next(
                    (candidate for candidate in eligible if candidate not in selected
                     and _select_alternate_edge(candidate, exclusive) is not None), None,
                )
                if replacement is None:
                    raise RuntimeError("no static-valid query has a blockable non-bridge route edge")
                graph = replacement
                selected[offset] = replacement
                alternate = _select_alternate_edge(graph, exclusive)
            targets = (str(alternate),)
        elif category == "moving_e1_e2_e3":
            route_witnesses = [edge for edge in route[3:-3] if edge in exclusive]
            if len(route_witnesses) < 3:
                raise RuntimeError(f"{graph.query.query_id} lacks three movable edge witnesses")
            picks = [
                route_witnesses[len(route_witnesses) // 4],
                route_witnesses[len(route_witnesses) // 2],
                route_witnesses[3 * len(route_witnesses) // 4],
            ]
            targets = tuple(dict.fromkeys(picks))
            if len(targets) != 3:
                targets = tuple(route_witnesses[:3])
        elif category == "disappearance_recovery":
            targets = _choose_group(route[3:-3], 1, exclusive, seed + offset)
        elif category == "changed_edges_5":
            targets = _choose_group(all_exclusive, 5, exclusive, seed + offset)
        elif category == "changed_edges_20":
            targets = _choose_group(all_exclusive, 20, exclusive, seed + offset)
        elif category == "alternate_channel_100":
            alternate = _select_alternate_edge(graph, exclusive)
            if alternate is None:
                raise RuntimeError("100-edge scenario requires a route edge with an alternate channel")
            alternate_route = _blocked_oracle(graph, (alternate,))
            protected = set(alternate_route.edge_path)
            fill_pool = [
                edge for edge in all_exclusive
                if edge != alternate and edge not in protected and edge not in route_set
            ]
            fill = _choose_group(fill_pool, 99, exclusive, seed + offset)
            targets = tuple(sorted((alternate, *fill)))
            if len(targets) != 100 or _blocked_oracle(graph, targets).node_path is None:
                raise RuntimeError("could not preserve an alternate channel in the 100-edge scenario")
        elif category == "bridge_or_min_cut_no_route":
            cut = _minimum_static_cut(graph)
            targets = cut
            missing = [edge_id for edge_id in targets if edge_id not in any_witness]
            if missing:
                raise RuntimeError(f"min-cut edges have no cell witness: {missing}")
            scenarios.append(Scenario(
                f"DYN-{offset + 1:02d}", graph.query.query_id, category, "main",
                targets, {edge_id: any_witness[edge_id] for edge_id in targets},
                min_cut_size=len(targets),
            ))
            continue
        else:
            raise AssertionError(category)
        scenarios.append(Scenario(
            f"DYN-{offset + 1:02d}", graph.query.query_id, category, "main",
            targets, {edge_id: exclusive[edge_id] for edge_id in targets},
        ))

    for query_id, group in (("A2B-07", "negative_control"),
                            ("A2B-16", "negative_control"),
                            ("A2B-19", "smac_long_tail_control")):
        if query_id in query_graphs:
            graph = query_graphs[query_id]
            off_path = [edge for edge in all_exclusive if edge not in set(graph.baseline_edge_ids)]
            targets = _choose_group(off_path, 1, exclusive, seed + int(query_id[-2:]))
            scenarios.append(Scenario(
                f"CTRL-{query_id}", query_id, group, group, targets,
                {targets[0]: exclusive[targets[0]]},
                frozen_failure_code=str(frozen.get(query_id, {}).get("failure_code", "")),
            ))
    main_queries = {scenario.query_id for scenario in scenarios if scenario.analysis_group == "main"}
    if len(main_queries) != main_query_count:
        raise RuntimeError("scenario selection did not produce distinct main queries")
    return scenarios


def _occupied_edge_sequence(scenario: Scenario) -> List[Tuple[str, ...]]:
    empty: Tuple[str, ...] = ()
    targets = tuple(scenario.target_edges)
    sequence: List[Tuple[str, ...]] = [empty]
    if scenario.category == "moving_e1_e2_e3":
        first, second, third = targets
        pattern = [
            (first,), (first,), (second,), (second,), (third,), (third,),
            empty, empty, (first,), (first,), (second,), (second,),
            (third,), (third,), empty, empty, (first,), (first,), empty, empty,
        ]
    else:
        pattern = []
        for _cycle in range(5):
            pattern.extend((targets, targets, empty, empty))
    sequence.extend(pattern)
    if len(sequence) != 21:
        raise AssertionError("every episode must contain S0 plus S1..S20")
    return sequence


def _write_event_stream(
    directory: Path, scenario: Scenario, *, map_version: str, map_shape: Sequence[int], seed: int,
) -> Tuple[Path, List[str]]:
    snapshots = []
    payloads = []
    for index, occupied_edges in enumerate(_occupied_edge_sequence(scenario)):
        cells = sorted({scenario.witness_cells[edge_id] for edge_id in occupied_edges})
        raw = {
            "snapshot_id": f"{scenario.scenario_id}-S{index}",
            "timestamp": float(index + 1),
            "occupied_cells": [list(cell) for cell in cells],
            "obstacle_confidence": {"generator": 1.0},
            "ttl": None,
            "map_version": map_version,
            "map_shape": [int(map_shape[0]), int(map_shape[1])],
            "generator_seed": seed,
            "target_occupied_edge_ids": list(occupied_edges),
        }
        # DynamicSnapshot owns the canonical snapshot hash; save it explicitly
        # so each arm receives byte-for-byte identical input.
        snap = DynamicSnapshot(
            raw["snapshot_id"], raw["timestamp"], tuple(cells),
            raw["obstacle_confidence"], None, map_version, tuple(map_shape),
        )
        raw["snapshot_hash"] = snap.snapshot_hash
        snapshots.append(raw)
        payloads.append(json.dumps(raw, sort_keys=True, separators=(",", ":")))
    path = directory / f"{scenario.scenario_id}.json"
    path.write_text(json.dumps({
        "schema_version": 1, "scenario_id": scenario.scenario_id,
        "query_id": scenario.query_id, "category": scenario.category,
        "analysis_group": scenario.analysis_group, "snapshots": snapshots,
    }, indent=2, sort_keys=True), encoding="utf-8")
    return path, payloads


def _annotate_rows(
    rows: List[Dict[str, Any]], scenario: Scenario, query_graph: QueryGraph,
    *, run_mode: str, repetition: int, topology_edge_count: int,
) -> None:
    prior_paths: Dict[str, Set[str]] = {arm: set() for arm in value.ARMS}
    for row in sorted(rows, key=lambda item: (int(item["snapshot_index"]), str(item["arm"]))):
        arm = str(row["arm"])
        path = set(row.get("path_edge_ids") or ())
        occupied = set(row.get("occupied_edge_ids") or ())
        changed = set(row.get("changed_edge_ids") or ())
        previous = prior_paths[arm]
        row.update({
            "architecture_id": value.ARCHITECTURE_ID,
            "implementation_revision": value.IMPLEMENTATION_REVISION,
            "parent_architecture": value.PARENT_ARCHITECTURE,
            "experiment_kind": value.EXPERIMENT_KIND,
            "protocol_version": value.PROTOCOL_VERSION,
            "scenario_id": scenario.scenario_id,
            "query_id": scenario.query_id,
            "scenario_category": scenario.category,
            "analysis_group": scenario.analysis_group,
            "run_mode": run_mode, "repetition": repetition,
            "topology_static_hash": query_graph.template.static_hash,
            "topology_edge_count": topology_edge_count,
            "changed_edges_ratio": float(row["changed_edge_count"]) / max(1, topology_edge_count),
            "path_intersection": bool(previous.intersection(occupied | changed)),
            "route_changed": bool(previous and path != previous),
            "route_changed_ratio": (
                len(previous.symmetric_difference(path)) / max(1, len(previous.union(path)))
                if previous else 0.0
            ),
            "initial_plan": int(row["snapshot_index"]) == 0,
            "dynamic_update": int(row["snapshot_index"]) > 0,
            "blocked_cost_semantics": "INF",
            "static_map_mutated": False,
            "static_topology_mutated": False,
            "l2_called": False,
            "l3_called": False,
        })
        if row.get("reachable") is True:
            prior_paths[arm] = path


def _correctness_rows(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    grouped: Dict[Tuple[str, str, int, int], Dict[str, Mapping[str, Any]]] = {}
    for row in rows:
        if row.get("run_mode") != "measured":
            continue
        key = (
            str(row["scenario_id"]), str(row["query_id"]),
            int(row["repetition"]), int(row["snapshot_index"]),
        )
        grouped.setdefault(key, {})[str(row["arm"])] = row
    result = []
    for key, arms in sorted(grouped.items()):
        oracle = arms[value.COLD_GRAPH_ASTAR]
        incremental = arms[value.INCREMENTAL_DSTAR]
        cold_dstar = arms[value.COLD_DSTAR]
        reachable_parity = (
            incremental["reachable"] == oracle["reachable"]
            and cold_dstar["reachable"] == oracle["reachable"]
        )
        inc_error = (
            abs(float(incremental["path_cost"]) - float(oracle["path_cost"]))
            if oracle["reachable"] else 0.0
        )
        cold_error = (
            abs(float(cold_dstar["path_cost"]) - float(oracle["path_cost"]))
            if oracle["reachable"] else 0.0
        )
        row = {
            "scenario_id": key[0], "query_id": key[1],
            "repetition": key[2], "snapshot_index": key[3],
            "snapshot_id": oracle["snapshot_id"],
            "analysis_group": oracle["analysis_group"],
            "input_hash_match": len({arm["algorithm_input_hash"] for arm in arms.values()}) == 1,
            "edge_status_hash_match": len({arm["edge_status_hash"] for arm in arms.values()}) == 1,
            "edge_cost_hash_match": len({arm["edge_cost_hash"] for arm in arms.values()}) == 1,
            "reachable_parity": reachable_parity,
            "incremental_cost_error": inc_error,
            "cold_dstar_cost_error": cold_error,
            "cost_parity": inc_error <= 1.0e-9 and cold_error <= 1.0e-9,
            "route_edge_ids_equal": (
                incremental["path_edge_ids"] == oracle["path_edge_ids"]
                and cold_dstar["path_edge_ids"] == oracle["path_edge_ids"]
            ),
            "blocked_edge_absent": all(not arm["blocked_edges_in_path"] for arm in arms.values()),
            "no_route_classification_match": all(
                (arm["failure_code"] == "L1_NO_ROUTE") == (not bool(oracle["reachable"]))
                for arm in arms.values()
            ),
            "incremental_no_reinitialize": (
                int(incremental["reinitialize_call_count"]) == 0
                and not bool(incremental["implicit_reinitialize"])
                and (key[3] == 0 or bool(incremental["planner_identity_stable"]))
            ),
            "topology_immutable": all(
                not bool(arm["static_topology_mutated"]) and not bool(arm["static_map_mutated"])
                for arm in arms.values()
            ),
        }
        row["all_correct"] = all(bool(row[field]) for field in (
            "input_hash_match", "edge_status_hash_match", "edge_cost_hash_match",
            "reachable_parity", "cost_parity", "blocked_edge_absent",
            "route_edge_ids_equal",
            "no_route_classification_match", "incremental_no_reinitialize",
            "topology_immutable",
        ))
        result.append(row)
    return result


def _timing_summaries(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    result: List[Dict[str, Any]] = []
    phases = (
        "snapshot_parse_ms", "changed_cell_to_edge_mapping_ms",
        "edge_state_transition_ms", "update_edges_ms",
        "compute_shortest_path_ms", "route_extraction_ms",
        "algorithm_wall_ms", "full_incremental_l1_ms",
    )
    filters = {
        "main_initial": lambda row: row["analysis_group"] == "main" and row["initial_plan"],
        "main_dynamic": lambda row: row["analysis_group"] == "main" and row["dynamic_update"],
        "main_path_affected": lambda row: row["analysis_group"] == "main" and row["dynamic_update"] and row["path_intersection"],
        "main_path_outside": lambda row: row["analysis_group"] == "main" and row["dynamic_update"] and not row["path_intersection"],
        "smac_long_tail_control": lambda row: row["analysis_group"] == "smac_long_tail_control" and row["dynamic_update"],
    }
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    for group_name, predicate in filters.items():
        for arm in value.ARMS:
            selected = [row for row in measured if row["arm"] == arm and predicate(row)]
            for phase in phases:
                stats = _summary([float(row[phase]) for row in selected])
                result.append({"group": group_name, "arm": arm, "metric": phase, **stats})
    return result


def _expanded_summaries(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    result = []
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    for group_name, predicate in (
        ("main_dynamic", lambda row: row["analysis_group"] == "main" and row["dynamic_update"]),
        ("main_path_affected", lambda row: row["analysis_group"] == "main" and row["dynamic_update"] and row["path_intersection"]),
        ("main_path_outside", lambda row: row["analysis_group"] == "main" and row["dynamic_update"] and not row["path_intersection"]),
    ):
        for arm in value.ARMS:
            selected = [float(row["expanded_nodes"]) for row in measured if row["arm"] == arm and predicate(row)]
            result.append({"group": group_name, "arm": arm, "metric": "expanded_nodes", **_summary(selected)})
    return result


def _break_even(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    grouped: Dict[Tuple[int, int], Dict[str, List[Mapping[str, Any]]]] = {}
    for row in rows:
        if row.get("run_mode") != "measured" or row["analysis_group"] != "main" or not row["dynamic_update"]:
            continue
        key = (int(row["changed_edge_count"]), int(row["topology_edge_count"]))
        grouped.setdefault(key, {}).setdefault(str(row["arm"]), []).append(row)
    result = []
    for (changed, total), arms in sorted(grouped.items()):
        inc = arms.get(value.INCREMENTAL_DSTAR, [])
        astar = arms.get(value.COLD_GRAPH_ASTAR, [])
        if not inc or not astar:
            continue
        inc_wall = _percentile([float(row["full_incremental_l1_ms"]) for row in inc], 0.50)
        astar_wall = _percentile([float(row["full_incremental_l1_ms"]) for row in astar], 0.50)
        inc_exp = _percentile([float(row["expanded_nodes"]) for row in inc], 0.50)
        astar_exp = _percentile([float(row["expanded_nodes"]) for row in astar], 0.50)
        result.append({
            "changed_edge_count": changed, "topology_edge_count": total,
            "changed_edges_ratio": changed / max(1, total),
            "incremental_dstar_wall_p50_ms": inc_wall,
            "cold_graph_astar_wall_p50_ms": astar_wall,
            "dstar_over_astar_wall_ratio": inc_wall / astar_wall if astar_wall else float("inf"),
            "astar_over_dstar_wall_speedup": astar_wall / inc_wall if inc_wall else float("inf"),
            "incremental_dstar_expanded_p50": inc_exp,
            "cold_graph_astar_expanded_p50": astar_exp,
            "expanded_node_ratio": inc_exp / astar_exp if astar_exp else 0.0,
        })
    return result


def _stage_a_gates(
    rows: Sequence[Mapping[str, Any]], correctness: Sequence[Mapping[str, Any]],
) -> Dict[str, Any]:
    main_correct = [row for row in correctness if row["analysis_group"] == "main"]
    correctness_pass = bool(main_correct) and all(bool(row["all_correct"]) for row in main_correct)
    measured = [
        row for row in rows if row.get("run_mode") == "measured"
        and row["analysis_group"] == "main" and row["dynamic_update"]
    ]
    affected_inc = [
        float(row["expanded_nodes"]) for row in measured
        if row["arm"] == value.INCREMENTAL_DSTAR and row["path_intersection"]
    ]
    affected_astar = [
        float(row["expanded_nodes"]) for row in measured
        if row["arm"] == value.COLD_GRAPH_ASTAR and row["path_intersection"]
    ]
    inc_wall = [float(row["full_incremental_l1_ms"]) for row in measured if row["arm"] == value.INCREMENTAL_DSTAR]
    astar_wall = [float(row["full_incremental_l1_ms"]) for row in measured if row["arm"] == value.COLD_GRAPH_ASTAR]
    inc_exp_p50 = _percentile(affected_inc, 0.50)
    astar_exp_p50 = _percentile(affected_astar, 0.50)
    inc_wall_p50, astar_wall_p50 = _percentile(inc_wall, 0.50), _percentile(astar_wall, 0.50)
    inc_wall_p95, astar_wall_p95 = _percentile(inc_wall, 0.95), _percentile(astar_wall, 0.95)
    expanded_reduction = 1.0 - inc_exp_p50 / astar_exp_p50 if astar_exp_p50 else 0.0
    wall_reduction = 1.0 - inc_wall_p50 / astar_wall_p50 if astar_wall_p50 else 0.0
    return {
        "correctness_pass": correctness_pass,
        "correctness_rows": len(main_correct),
        "correctness_failures": sum(not bool(row["all_correct"]) for row in main_correct),
        "local_path_affected_incremental_expanded_p50": inc_exp_p50,
        "local_path_affected_cold_astar_expanded_p50": astar_exp_p50,
        "expanded_nodes_p50_reduction": expanded_reduction,
        "expanded_nodes_gate_pass": expanded_reduction >= 0.50,
        "incremental_full_l1_p50_ms": inc_wall_p50,
        "cold_graph_astar_full_l1_p50_ms": astar_wall_p50,
        "full_l1_p50_reduction": wall_reduction,
        "full_l1_p50_gate_pass": wall_reduction >= 0.30,
        "incremental_full_l1_p95_ms": inc_wall_p95,
        "cold_graph_astar_full_l1_p95_ms": astar_wall_p95,
        "p95_gate_pass": inc_wall_p95 <= astar_wall_p95,
        "stage_a_pass": bool(
            correctness_pass and expanded_reduction >= 0.50
            and wall_reduction >= 0.30 and inc_wall_p95 <= astar_wall_p95
        ),
    }


def _cache_state_rows(
    graph_view: Any, topology_info: Mapping[str, Any], query_graphs: Mapping[str, QueryGraph],
    rows: Sequence[Mapping[str, Any]], cell_index: value.CellToEdgeIndex,
    stage_b_summary: Optional[Mapping[str, Any]] = None,
) -> List[Dict[str, Any]]:
    measured_inc = [
        row for row in rows if row.get("run_mode") == "measured"
        and row["arm"] == value.INCREMENTAL_DSTAR
    ]
    result = [
        {"component": "frozen_topology_cache", "build_count": 0, "hit_count": 1,
         "miss_count": 0, "build_time_ms": 0.0,
         "lookup_time_ms": topology_info["topology_load_time_ms"],
         "memory_bytes": topology_info["topology_cache_bytes"],
         "hit_rate": 1.0,
         "detail": topology_info["topology_cache_key"]},
        {"component": "cell_to_edge_index", "build_count": 1, "hit_count": len(measured_inc),
         "miss_count": 0, "build_time_ms": cell_index.build_time_ms,
         "lookup_time_ms": sum(float(row["changed_cell_to_edge_mapping_ms"]) for row in measured_inc),
         "memory_bytes": cell_index.memory_bytes,
         "hit_rate": 1.0,
         "detail": f"{len(cell_index.cell_to_edges)} indexed cells"},
        {"component": "r2_endpoint_attachment_cache", "build_count": len(query_graphs) * 2,
         "hit_count": 0, "miss_count": len(query_graphs) * 2,
         "build_time_ms": sum(float(graph.attachment_diagnostics.get("attachment_lookup_time_ms", 0.0)) for graph in query_graphs.values()),
         "lookup_time_ms": 0.0,
         "memory_bytes": max((int(graph.attachment_diagnostics.get("endpoint_cache_memory_bytes", 0)) for graph in query_graphs.values()), default=0),
         "hit_rate": 0.0,
         "detail": "built once per endpoint before paired episodes; identical graph template shared"},
        {"component": "incremental_dstar_state", "build_count": len({(row['scenario_id'], row['run_mode'], row['repetition']) for row in measured_inc}),
         "hit_count": sum(bool(row["g_reused"] and row["rhs_reused"] and row["open_reused"] and row["km_reused"]) for row in measured_inc),
         "miss_count": sum(bool(row["initial_plan"]) for row in measured_inc),
         "build_time_ms": sum(float(row["graph_initialization_ms"]) for row in measured_inc if row["initial_plan"]),
         "lookup_time_ms": 0.0,
         "memory_bytes": max((int(row["state_memory_bytes"]) for row in measured_inc), default=0),
         "hit_rate": (sum(bool(row["g_reused"] and row["rhs_reused"] and row["open_reused"] and row["km_reused"]) for row in measured_inc) / max(1, len(measured_inc))),
         "detail": "g/rhs/OPEN/km state retained; reinitialize_call_count=0"},
    ]
    for arm, cache in dict((stage_b_summary or {}).get("corridor_cache_by_arm") or {}).items():
        hits, misses = int(cache["hits"]), int(cache["misses"])
        result.append({
            "component": f"r2_corridor_cache_{arm}",
            "build_count": misses, "hit_count": hits, "miss_count": misses,
            "build_time_ms": float(cache["cold_build_time_ms"]),
            "lookup_time_ms": float(cache["lookup_time_ms"]),
            "memory_bytes": int(cache["memory_bytes"]),
            "hit_rate": hits / max(1, hits + misses),
            "detail": "independent equal-condition cache per ROS arm",
        })
    return result


def _process_pss_bytes() -> int:
    try:
        for line in Path("/proc/self/smaps_rollup").read_text(encoding="utf-8").splitlines():
            if line.startswith("Pss:"):
                return int(line.split()[1]) * 1024
    except (OSError, ValueError, IndexError):
        pass
    return 0


def _snapshot_from_payload(payload: str) -> DynamicSnapshot:
    raw = json.loads(payload)
    return DynamicSnapshot(
        str(raw["snapshot_id"]), float(raw["timestamp"]),
        tuple(tuple(cell) for cell in raw.get("occupied_cells", ())),
        dict(raw.get("obstacle_confidence") or {}), raw.get("ttl"),
        str(raw.get("map_version", "")),
        None if raw.get("map_shape") is None else tuple(raw["map_shape"]),
        str(raw.get("snapshot_hash", "")),
    )


def _ros_stage_b_summary(rows: Sequence[Mapping[str, Any]]) -> Tuple[str, Dict[str, Any], List[Dict[str, Any]]]:
    main = [
        row for row in rows if row["analysis_group"] == "main"
        and int(row["snapshot_index"]) > 0
    ]
    summaries: List[Dict[str, Any]] = []
    for group_name, predicate in (
        ("all_dynamic", lambda row: True),
        ("actual_l3_replan", lambda row: bool(row["l3_called"])),
        ("scheduler_no_l3_replan", lambda row: bool(row["scheduler_no_l3_replan"])),
    ):
        for arm in (value.INCREMENTAL_DSTAR, value.COLD_GRAPH_ASTAR):
            selected = [row for row in main if row["arm"] == arm and predicate(row)]
            for metric in (
                "snapshot_to_new_l1_route_ms", "snapshot_to_final_valid_path_ms",
                "l3_wall_ms", "cpu_ms",
            ):
                summaries.append({
                    "group": f"ros_{group_name}", "arm": arm, "metric": metric,
                    **_summary([float(row[metric]) for row in selected]),
                })
    tail = [
        row for row in rows if row["analysis_group"] == "smac_long_tail_control"
        and bool(row["l3_called"])
    ]
    tail_summary: Dict[str, Any] = {}
    for arm in (value.INCREMENTAL_DSTAR, value.COLD_GRAPH_ASTAR):
        selected = [row for row in tail if row["arm"] == arm]
        stats = _summary([float(row["snapshot_to_final_valid_path_ms"]) for row in selected])
        tail_summary[arm] = stats
        summaries.append({
            "group": "ros_smac_long_tail_control", "arm": arm,
            "metric": "snapshot_to_final_valid_path_ms", **stats,
        })
    replan = [row for row in main if row["l3_called"]]
    inc = [row for row in replan if row["arm"] == value.INCREMENTAL_DSTAR]
    astar = [row for row in replan if row["arm"] == value.COLD_GRAPH_ASTAR]
    inc_p50 = _percentile([float(row["snapshot_to_final_valid_path_ms"]) for row in inc], 0.50)
    astar_p50 = _percentile([float(row["snapshot_to_final_valid_path_ms"]) for row in astar], 0.50)
    inc_p95 = _percentile([float(row["snapshot_to_final_valid_path_ms"]) for row in inc], 0.95)
    astar_p95 = _percentile([float(row["snapshot_to_final_valid_path_ms"]) for row in astar], 0.95)
    improvement = 1.0 - inc_p50 / astar_p50 if astar_p50 else 0.0
    inc_deadline = sum(bool(row["deadline_miss"]) for row in inc)
    astar_deadline = sum(bool(row["deadline_miss"]) for row in astar)
    deadline_reduction = (
        1.0 - (inc_deadline / max(1, len(inc))) / (astar_deadline / max(1, len(astar)))
        if astar_deadline else 0.0
    )
    valid_paths = [row for row in main if row["final_valid_success"]]
    safety_pass = all(
        bool(row["static_footprint_valid"]) and bool(row["kinematic_valid"])
        and int(row["dynamic_collision_count"]) == 0
        for row in valid_paths
    )
    paired: Dict[Tuple[str, int, int], Dict[str, Mapping[str, Any]]] = {}
    for row in main:
        key = (str(row["scenario_id"]), int(row["ros_repetition"]), int(row["snapshot_index"]))
        paired.setdefault(key, {})[str(row["arm"])] = row
    pair_rows = [item for item in paired.values() if len(item) == 2]
    paired_reachability = all(
        pair[value.INCREMENTAL_DSTAR]["l1_reachable"]
        == pair[value.COLD_GRAPH_ASTAR]["l1_reachable"] for pair in pair_rows
    )
    paired_cost = all(
        (not pair[value.COLD_GRAPH_ASTAR]["l1_reachable"])
        or abs(float(pair[value.INCREMENTAL_DSTAR]["l1_path_cost"])
               - float(pair[value.COLD_GRAPH_ASTAR]["l1_path_cost"])) <= 1.0e-9
        for pair in pair_rows
    )
    final_valid_equal = all(
        pair[value.INCREMENTAL_DSTAR]["final_valid_success"]
        == pair[value.COLD_GRAPH_ASTAR]["final_valid_success"] for pair in pair_rows
    )
    inc_smac_failures = sum(bool(row["smac_failure_code"]) for row in inc)
    astar_smac_failures = sum(bool(row["smac_failure_code"]) for row in astar)
    no_smac_regression = inc_smac_failures <= astar_smac_failures
    performance_pass = improvement >= 0.05 or deadline_reduction >= 0.20
    p95_pass = inc_p95 <= astar_p95 * 1.05
    engineering_pass = all((
        safety_pass, paired_reachability, paired_cost, final_valid_equal,
        no_smac_regression, performance_pass, p95_pass,
    ))
    result = {
        "safety_pass": safety_pass,
        "paired_reachability_pass": paired_reachability,
        "paired_cost_pass": paired_cost,
        "final_valid_equal": final_valid_equal,
        "incremental_final_valid_count": sum(bool(row["final_valid_success"]) for row in main if row["arm"] == value.INCREMENTAL_DSTAR),
        "cold_astar_final_valid_count": sum(bool(row["final_valid_success"]) for row in main if row["arm"] == value.COLD_GRAPH_ASTAR),
        "incremental_smac_failure_count": inc_smac_failures,
        "cold_astar_smac_failure_count": astar_smac_failures,
        "no_smac_failure_regression": no_smac_regression,
        "incremental_e2e_replan_p50_ms": inc_p50,
        "cold_astar_e2e_replan_p50_ms": astar_p50,
        "e2e_p50_improvement": improvement,
        "e2e_p50_gate_pass": improvement >= 0.05,
        "incremental_e2e_replan_p95_ms": inc_p95,
        "cold_astar_e2e_replan_p95_ms": astar_p95,
        "p95_gate_pass": p95_pass,
        "incremental_deadline_miss_count": inc_deadline,
        "cold_astar_deadline_miss_count": astar_deadline,
        "deadline_miss_reduction": deadline_reduction,
        "deadline_gate_pass": deadline_reduction >= 0.20,
        "engineering_value_pass": engineering_pass,
        "scheduler_no_l3_replan_count": sum(bool(row["scheduler_no_l3_replan"]) for row in main) // 2,
        "actual_l3_replan_pair_count": len(replan) // 2,
        "a2b19_long_tail": tail_summary,
    }
    return (
        "PASSED_ENGINEERING_GATE" if engineering_pass else "FAILED_ENGINEERING_GATE",
        result, summaries,
    )


def _run_ros_stage_b(
    output: Path,
    scenarios: Sequence[Scenario],
    query_graphs: Mapping[str, QueryGraph],
    event_payloads: Mapping[str, Sequence[str]],
    edge_cells: Mapping[str, Iterable[Sequence[int]]],
    graph_view: Any,
    topology_info: Mapping[str, Any],
    ctx: Any,
    *, ros_domain_id: int, repetitions: int,
) -> Tuple[List[Dict[str, Any]], str, Dict[str, Any], List[Dict[str, Any]]]:
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    spec = legacy.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"Smac Hybrid backend unavailable: {spec.reason}")
    session = legacy.SmacSession(
        ctx, output, map_yaml=r2_benchmark.MAP_YAML,
        log_tag=f"dynamic_2d_v1_r3_{MAP_ID}", local_mask_updates=True,
        optimization_profile=r2_benchmark.OPTIMIZATION_PROFILE,
        smac_parameter_profile=r2_benchmark.SMAC_PARAMETER_PROFILE,
        optimization_stage=r2_benchmark.OPTIMIZATION_STAGE,
    )
    session.start()
    adapter = v0.SmacHybridAdapter(
        session, spec, footprint=legacy.FOOTPRINT,
        source_commit=legacy._source_commit(), force_full_update=True,
    )
    pipelines = {
        arm: r2_pipeline.Layered2DV1R2Pipeline(
            graph_view, footprint=legacy.FOOTPRINT, l3_planner=adapter,
            corridor_padding_m=2.0, corridor_profile="padding",
            corridor_fallback_policy="bounded",
            validator=lambda _map, query, points: canonical.canonical_validate_path(
                ctx, query, points,
            ),
            base_map_hash=ctx.map_sha256,
            topology_cache_key=str(topology_info["topology_cache_key"]),
            topology_source_hash=str(topology_info["topology_source_hash"]),
            corridor_semantics="raw_map_smac_aligned",
        )
        for arm in (value.INCREMENTAL_DSTAR, value.COLD_GRAPH_ASTAR)
    }
    rows: List[Dict[str, Any]] = []
    ros_scenarios = [
        scenario for scenario in scenarios
        if scenario.analysis_group in {"main", "smac_long_tail_control"}
    ]
    try:
        for repetition in range(1, int(repetitions) + 1):
            for scenario_index, scenario in enumerate(ros_scenarios):
                reset_info = session.reset_query_state(
                    scenario.query_id, restore_base_map=True,
                )
                query_graph = query_graphs[scenario.query_id]
                overlays = {
                    arm: value.DynamicEdgeOverlay(
                        edge_cells, map_version=ctx.map_sha256,
                        map_shape=graph_view.artifact.free_mask.shape,
                    ) for arm in (value.INCREMENTAL_DSTAR, value.COLD_GRAPH_ASTAR)
                }
                arm_states = {
                    arm: value.ArmState(arm, query_graph.template)
                    for arm in (value.INCREMENTAL_DSTAR, value.COLD_GRAPH_ASTAR)
                }
                previous_route: Dict[str, Tuple[str, ...]] = {
                    value.INCREMENTAL_DSTAR: (), value.COLD_GRAPH_ASTAR: (),
                }
                previous_points: Dict[str, List[Dict[str, Any]]] = {
                    value.INCREMENTAL_DSTAR: [], value.COLD_GRAPH_ASTAR: [],
                }
                previous_valid = {
                    value.INCREMENTAL_DSTAR: False, value.COLD_GRAPH_ASTAR: False,
                }
                for snapshot_index, payload in enumerate(event_payloads[scenario.scenario_id]):
                    arms = [value.INCREMENTAL_DSTAR, value.COLD_GRAPH_ASTAR]
                    if (repetition + scenario_index + snapshot_index) % 2:
                        arms.reverse()
                    pair_input_hashes = set()
                    for arm in arms:
                        wall_started_ns = time.monotonic_ns()
                        cpu_started_ns = time.process_time_ns()
                        prepared = overlays[arm].consume_json(payload)
                        l1_result = arm_states[arm].run(prepared)
                        pair_input_hashes.add(l1_result["algorithm_input_hash"])
                        snapshot = prepared.snapshot
                        node_path = list(l1_result.get("path_node_ids") or ())
                        route_edges = tuple(
                            edge_id for edge_id in l1_result.get("path_edge_ids", ())
                            if str(edge_id).startswith("topology_")
                        )
                        old_route = previous_route[arm]
                        route_changed = bool(old_route and route_edges != old_route)
                        dynamic_risk = (
                            v0.dynamic_collision_count(
                                ctx.hospital_map, previous_points[arm], legacy.FOOTPRINT,
                                snapshot,
                            ) if previous_points[arm] else 0
                        )
                        l3_needed = bool(
                            l1_result.get("reachable") is True
                            and (snapshot_index == 0 or route_changed or dynamic_risk
                                 or not previous_valid[arm])
                        )
                        scheduler_noop = bool(
                            l1_result.get("reachable") is True and not l3_needed
                        )
                        pipeline = pipelines[arm]
                        l3_diag: Dict[str, Any] = {}
                        points: List[Dict[str, Any]] = []
                        final_valid = False
                        failure_code = str(l1_result.get("failure_code") or "")
                        l3_wall_ms = 0.0
                        if l1_result.get("reachable") is True and l3_needed:
                            pipeline._virtual_positions = {
                                int(node): tuple(position)
                                for node, position in query_graph.template.node_positions.items()
                                if int(node) < 0
                            }
                            pipeline._route_edge_ids = list(route_edges)
                            l3_started_ns = time.monotonic_ns()
                            l3_result = pipeline._run_l3(
                                query_graph.query, snapshot, node_path, validate=True,
                            )
                            l3_wall_ms = _elapsed_ms(l3_started_ns)
                            l3_diag = dict(l3_result.diagnostics or {})
                            points = [dict(point) for point in (l3_result.points or ())]
                            final_valid = bool(
                                l3_result.success
                                and l3_diag.get("final_valid_success", True)
                                and int(l3_diag.get("dynamic_collision_count", 0) or 0) == 0
                            )
                            failure_code = "" if final_valid else str(
                                l3_diag.get("failure_code") or l3_result.failure_code
                                or "L3_PLANNER_FAILED"
                            )
                        elif scheduler_noop:
                            points = [dict(point) for point in previous_points[arm]]
                            final_valid = bool(previous_valid[arm] and dynamic_risk == 0)
                            failure_code = "" if final_valid else "DYNAMIC_FOOTPRINT_COLLISION"
                            l3_diag = {
                                "static_footprint_valid": final_valid,
                                "kinematic_valid": final_valid,
                                "dynamic_collision_count": dynamic_risk,
                                "canonical_validation_reused": True,
                            }
                        if l1_result.get("reachable") is True:
                            previous_route[arm] = route_edges
                        if final_valid:
                            previous_points[arm] = points
                            previous_valid[arm] = True
                        elif not scheduler_noop:
                            previous_valid[arm] = False
                        total_ms = _elapsed_ms(wall_started_ns)
                        cpu_ms = (time.process_time_ns() - cpu_started_ns) / 1.0e6
                        rss_bytes = int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024
                        dynamic_collision = int(l3_diag.get("dynamic_collision_count", 0) or 0)
                        row = {
                            "architecture_id": value.ARCHITECTURE_ID,
                            "implementation_revision": value.IMPLEMENTATION_REVISION,
                            "parent_architecture": value.PARENT_ARCHITECTURE,
                            "experiment_kind": value.EXPERIMENT_KIND,
                            "protocol_version": value.PROTOCOL_VERSION,
                            "scenario_id": scenario.scenario_id,
                            "query_id": scenario.query_id,
                            "scenario_category": scenario.category,
                            "analysis_group": scenario.analysis_group,
                            "ros_repetition": repetition,
                            "snapshot_index": snapshot_index,
                            "snapshot_id": snapshot.snapshot_id,
                            "l1_snapshot_id": snapshot.snapshot_id,
                            "corridor_snapshot_id": snapshot.snapshot_id,
                            "l3_snapshot_id": snapshot.snapshot_id,
                            "snapshot_hash": snapshot.snapshot_hash,
                            "arm": arm,
                            "algorithm_input_hash": l1_result["algorithm_input_hash"],
                            "changed_cells_count": len(prepared.changed_cells),
                            "changed_edge_count": len(prepared.changed_edges),
                            "changed_edge_ids": list(prepared.changed_edges),
                            "changed_edges_ratio": len(prepared.changed_edges) / max(1, len(edge_cells)),
                            "blocked_edge_ids": sorted(
                                edge_id for edge_id, status in prepared.statuses.items()
                                if status == GraphDStarLite.BLOCKED
                            ),
                            "path_intersection": bool(dynamic_risk),
                            "route_changed": route_changed,
                            "route_changed_ratio": (
                                len(set(old_route).symmetric_difference(route_edges))
                                / max(1, len(set(old_route).union(route_edges)))
                                if old_route else 0.0
                            ),
                            "l1_reachable": l1_result.get("reachable"),
                            "l1_path_cost": l1_result.get("path_cost"),
                            "l1_route_edge_ids": list(route_edges),
                            "l1_path_hash": l1_result.get("path_hash", ""),
                            "dstar_expanded_nodes": (
                                l1_result.get("expanded_nodes", 0)
                                if arm == value.INCREMENTAL_DSTAR else 0
                            ),
                            "expanded_nodes": l1_result.get("expanded_nodes", 0),
                            "generated_nodes": l1_result.get("generated_nodes", 0),
                            "initial_plan_ms": total_ms if snapshot_index == 0 else 0.0,
                            "incremental_update_ms": (
                                l1_result.get("full_incremental_l1_ms", 0.0)
                                if arm == value.INCREMENTAL_DSTAR and snapshot_index > 0 else 0.0
                            ),
                            "full_replan_ms": (
                                l1_result.get("full_incremental_l1_ms", 0.0)
                                if arm == value.COLD_GRAPH_ASTAR and snapshot_index > 0 else 0.0
                            ),
                            "snapshot_to_new_l1_route_ms": l1_result.get("full_incremental_l1_ms", 0.0),
                            "snapshot_to_final_valid_path_ms": total_ms,
                            "l3_wall_ms": l3_wall_ms,
                            "l3_called": l3_needed,
                            "l3_call_count": 1 if l3_needed else 0,
                            "scheduler_no_l3_replan": scheduler_noop,
                            "scheduler_benefit_not_attributed_to_dstar": scheduler_noop,
                            "final_valid_success": final_valid,
                            "static_footprint_valid": bool(l3_diag.get("static_footprint_valid", final_valid)),
                            "kinematic_valid": bool(l3_diag.get("kinematic_valid", final_valid)),
                            "dynamic_collision_count": dynamic_collision,
                            "failure_code": failure_code,
                            "smac_failure_code": str(l3_diag.get("smac_failure_code", "")),
                            "l3_retry_count": 0,
                            "wrong_channel_switch": False,
                            "action_wall_ms": float(l3_diag.get("action_wall_time_ms", l3_diag.get("planner_wall_time_ms", 0.0)) or 0.0),
                            "nav2_reported_planning_time_ms": float(l3_diag.get("nav2_reported_planning_time_ms", l3_diag.get("planning_time_ms", 0.0)) or 0.0),
                            "local_map_update_ms": float(l3_diag.get("local_map_update_ms", 0.0) or 0.0),
                            "costmap_clear_ms": float(l3_diag.get("local_costmap_clear_ms", 0.0) or 0.0),
                            "costmap_settle_ms": float(l3_diag.get("costmap_settle_ms", 0.0) or 0.0),
                            "corridor_cache_hit": bool(l3_diag.get("corridor_cache_hit", False)),
                            "corridor_mask_hash": str(l3_diag.get("corridor_mask_hash", "")),
                            "path_hash": str(l3_diag.get("path_hash", "")),
                            "cpu_ms": cpu_ms, "RSS": rss_bytes,
                            "PSS": _process_pss_bytes(),
                            "deadline_ms": EVALUATOR_DEADLINE_MS,
                            "deadline_miss": total_ms > EVALUATOR_DEADLINE_MS,
                            "query_session_reset_ms": float(reset_info.get("query_session_reset_ms", 0.0)) if snapshot_index == 0 else 0.0,
                            "topology_static_hash": query_graph.template.static_hash,
                            "static_map_mutated": False,
                            "static_topology_mutated": False,
                            "l2_called": False,
                        }
                        rows.append(row)
                    if len(pair_input_hashes) != 1:
                        raise AssertionError(
                            f"ROS paired input mismatch for {scenario.scenario_id} S{snapshot_index}"
                        )
    finally:
        session.close()
    status, summary, timing = _ros_stage_b_summary(rows)
    summary.update({
        "session_start_count": session.session_start_count,
        "session_close_count": session.session_close_count,
        "session_restart_count": session.session_restart_count,
        "adapter_l3_call_count": adapter.calls,
        "corridor_cache_by_arm": {
            arm: {
                "hits": pipeline.corridor_cache.hits,
                "misses": pipeline.corridor_cache.misses,
                "cold_build_time_ms": pipeline.corridor_cache.build_time_ms,
                "lookup_time_ms": pipeline.corridor_cache.lookup_time_ms,
                "memory_bytes": pipeline.corridor_cache.memory_bytes,
            }
            for arm, pipeline in pipelines.items()
        },
    })
    return rows, status, summary, timing


def _snapshot_sources(output: Path, extra_files: Sequence[Path]) -> Dict[str, Any]:
    source_dir = output / "source_snapshot"
    source_dir.mkdir()
    files = [
        Path(__file__).resolve(), Path(value.__file__).resolve(),
        Path(r2_pipeline.__file__).resolve(), Path(r1_pipeline.__file__).resolve(),
        Path(v0.__file__).resolve(),
        Path(__file__).with_name("graph_dstar_lite.py").resolve(),
        Path(__file__).with_name("dynamic_snapshot.py").resolve(),
        Path(__file__).with_name("topology.py").resolve(),
        Path(legacy.__file__).resolve(), Path(task_source.__file__).resolve(),
        Path(r2_benchmark.__file__).resolve(),
        Path(__file__).resolve().parents[1] / "setup.py",
        Path(__file__).resolve().parents[1] / "config" / "two_layer_2d_v1_r3_dynamic_incremental.yaml",
        Path(__file__).resolve().parents[1] / "test" / "test_dynamic_incremental_value.py",
        Path(__file__).resolve().parents[1] / "test" / "test_two_layer_2d_v1_dynamic_incremental_benchmark.py",
        FROZEN_R2 / "source_snapshot" / "12_map.yaml",
        FROZEN_R2 / "source_snapshot" / "13_map.pgm",
        FROZEN_R2 / "source_snapshot" / "14_arena_a2b_benchmark_20.json",
        FROZEN_R2 / "source_snapshot" / "15_arena_a2b_benchmark_20.csv",
        *[Path(item).resolve() for item in extra_files],
    ]
    unique = []
    seen = set()
    for path in files:
        if path in seen:
            continue
        seen.add(path)
        unique.append(path)
    manifest_rows = []
    for index, path in enumerate(unique):
        target = source_dir / f"{index:02d}_{path.name}"
        shutil.copy2(path, target)
        manifest_rows.append({
            "source": str(path), "snapshot": str(target.relative_to(output)),
            "sha256": sha256_file(target), "bytes": target.stat().st_size,
        })
    manifest = {
        "schema_version": 1, "file_count": len(manifest_rows),
        "files": manifest_rows,
        "combined_hash": value.stable_hash(manifest_rows),
    }
    (output / "source_snapshot_manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8",
    )
    return manifest


REQUIRED_ARTIFACTS = (
    "protocol.yaml", "manifest.yaml", "scenario_manifest.csv",
    "paired_algorithm_runs.csv", "paired_ros_runs.csv", "correctness_oracle.csv",
    "failure_summary.csv", "timing_summary.csv", "expanded_nodes_summary.csv",
    "break_even_curve.csv", "cache_and_state_diagnostics.csv",
    "source_snapshot_manifest.yaml", "final_report.md",
)


def _validate_formal_artifacts(output: Path) -> Dict[str, Any]:
    missing = [name for name in REQUIRED_ARTIFACTS if not (output / name).is_file()]
    empty = [
        name for name in REQUIRED_ARTIFACTS
        if (output / name).is_file() and (output / name).stat().st_size == 0
    ]
    stream_files = sorted((output / "dynamic_event_streams").glob("*.json"))
    bad_streams = []
    for path in stream_files:
        payload = json.loads(path.read_text(encoding="utf-8"))
        snapshots = payload.get("snapshots") or []
        if len(snapshots) != 21 or not str(snapshots[0].get("snapshot_id", "")).endswith("S0"):
            bad_streams.append(path.name)
    source_manifest = yaml.safe_load(
        (output / "source_snapshot_manifest.yaml").read_text(encoding="utf-8")
    ) or {}
    bad_source_hashes = []
    for item in source_manifest.get("files") or []:
        path = output / item["snapshot"]
        if not path.is_file() or sha256_file(path) != item["sha256"]:
            bad_source_hashes.append(str(item.get("snapshot")))
    result = {
        "required_artifact_count": len(REQUIRED_ARTIFACTS),
        "missing_artifacts": missing, "empty_artifacts": empty,
        "event_stream_count": len(stream_files), "bad_event_streams": bad_streams,
        "source_snapshot_file_count": len(source_manifest.get("files") or []),
        "bad_source_snapshot_hashes": bad_source_hashes,
    }
    result["passed"] = not any((missing, empty, bad_streams, bad_source_hashes)) and bool(stream_files)
    return result


def _report(
    output: Path, gates: Mapping[str, Any], correctness: Sequence[Mapping[str, Any]],
    timing: Sequence[Mapping[str, Any]], expanded: Sequence[Mapping[str, Any]],
    break_even: Sequence[Mapping[str, Any]], scenarios: Sequence[Scenario],
    *, stage_b_status: str, stage_b_summary: Mapping[str, Any],
) -> str:
    timing_index = {
        (str(row["group"]), str(row["arm"]), str(row["metric"])): row
        for row in timing
    }

    def timing_value(group: str, arm: str, metric: str, quantile: str) -> float:
        return float(timing_index.get((group, arm, metric), {}).get(quantile, float("nan")))

    correct_count = sum(bool(row["all_correct"]) for row in correctness if row["analysis_group"] == "main")
    correct_total = sum(row["analysis_group"] == "main" for row in correctness)
    if not gates["stage_a_pass"]:
        verdict = "C"
        conclusion = "D* Lite 在 2D-V1 的 L1 层没有达到预设价值门槛；恢复 Graph A*，将 D* 保留给 3D-V0 的 L2 栅格层。"
    elif stage_b_status == "PASSED_ENGINEERING_GATE":
        verdict = "A"
        conclusion = "D* Lite 同时具有算法价值与端到端工程价值。"
    else:
        verdict = "B"
        conclusion = "D* Lite 有算法收益，但端到端收益不足。"
    lines = [
        "# 2D-V1 D* Lite dynamic incremental value experiment",
        "",
        f"- Final verdict: **{verdict}** — {conclusion}",
        f"- Architecture/revision: `{value.ARCHITECTURE_ID}` / `{value.IMPLEMENTATION_REVISION}`; parent `{value.PARENT_ARCHITECTURE}`.",
        f"- Protocol/kind: `{value.PROTOCOL_VERSION}` / `{value.EXPERIMENT_KIND}`.",
        f"- Stage A correctness: {correct_count}/{correct_total}; pass={gates['correctness_pass']}.",
        f"- Stage A value gate: pass={gates['stage_a_pass']}; Stage B: `{stage_b_status}`.",
        "",
        "## Stage A gates",
        "",
        f"- Local path-affected expanded-node P50: incremental {gates['local_path_affected_incremental_expanded_p50']:.3f}, cold Graph A* {gates['local_path_affected_cold_astar_expanded_p50']:.3f}; reduction {100*gates['expanded_nodes_p50_reduction']:.2f}% (target >=50%).",
        f"- Full incremental L1 wall P50: incremental {gates['incremental_full_l1_p50_ms']:.4f} ms, cold Graph A* {gates['cold_graph_astar_full_l1_p50_ms']:.4f} ms; reduction {100*gates['full_l1_p50_reduction']:.2f}% (target >=30%).",
        f"- Full incremental L1 wall P95: incremental {gates['incremental_full_l1_p95_ms']:.4f} ms, cold Graph A* {gates['cold_graph_astar_full_l1_p95_ms']:.4f} ms; no-regression pass={gates['p95_gate_pass']}.",
        "- Full L1 timing includes JSON snapshot parsing, changed-cell→edge mapping, edge-state transition, changed-edge update, search, and route extraction.",
        "- `BLOCKED` is represented as true infinity; dynamic data remains an overlay and does not mutate the static map/topology.",
        "",
        "### Three-arm timing (main dynamic snapshots)",
        "",
        "| arm | full L1 P50/P95/P99 ms | pure search P50 ms |",
        "|---|---:|---:|",
    ]
    for arm in value.ARMS:
        lines.append(
            f"| `{arm}` | {timing_value('main_dynamic', arm, 'full_incremental_l1_ms', 'p50'):.4f} / "
            f"{timing_value('main_dynamic', arm, 'full_incremental_l1_ms', 'p95'):.4f} / "
            f"{timing_value('main_dynamic', arm, 'full_incremental_l1_ms', 'p99'):.4f} | "
            f"{timing_value('main_dynamic', arm, 'compute_shortest_path_ms', 'p50'):.4f} |"
        )
    lines.extend([
        "",
        "## Scenario coverage",
        "",
    ])
    for scenario in scenarios:
        lines.append(
            f"- `{scenario.scenario_id}` / `{scenario.query_id}`: {scenario.category}; "
            f"group={scenario.analysis_group}; target edges={len(scenario.target_edges)}."
        )
    lines.extend([
        "",
        "## Break-even curve",
        "",
        "| changed edges | ratio | A*/D* wall speedup | expanded ratio D*/A* |",
        "|---:|---:|---:|---:|",
    ])
    for row in break_even:
        lines.append(
            f"| {row['changed_edge_count']} | {row['changed_edges_ratio']:.6f} | "
            f"{row['astar_over_dstar_wall_speedup']:.3f}x | {row['expanded_node_ratio']:.3f} |"
        )
    lines.extend([
        "",
        "## Interpretation",
        "",
        "- Path-outside no-replan benefits, if observed in Stage B, are scheduler benefits and are not credited to D* Lite.",
        "- A2B-07/A2B-16 are negative controls. A2B-19 is excluded from the primary D* conclusion and reserved as a Smac long-tail control.",
        "- No Gazebo, Nav2, or Smac process is started unless all Stage-A gates pass.",
    ])
    if stage_b_summary:
        tail = dict(stage_b_summary.get("a2b19_long_tail") or {})
        tail_inc = dict(tail.get(value.INCREMENTAL_DSTAR) or {})
        tail_astar = dict(tail.get(value.COLD_GRAPH_ASTAR) or {})
        lines.extend([
            "",
            "## Stage B engineering result",
            "",
            f"- Actual-L3-replan E2E P50: incremental {stage_b_summary.get('incremental_e2e_replan_p50_ms', float('nan')):.3f} ms, cold Graph A* {stage_b_summary.get('cold_astar_e2e_replan_p50_ms', float('nan')):.3f} ms; improvement {100*stage_b_summary.get('e2e_p50_improvement', 0.0):.2f}%.",
            f"- P95: incremental {stage_b_summary.get('incremental_e2e_replan_p95_ms', float('nan')):.3f} ms, cold Graph A* {stage_b_summary.get('cold_astar_e2e_replan_p95_ms', float('nan')):.3f} ms; gate={stage_b_summary.get('p95_gate_pass', False)}.",
            f"- Deadline misses incremental/A*: {stage_b_summary.get('incremental_deadline_miss_count', 0)}/{stage_b_summary.get('cold_astar_deadline_miss_count', 0)}; deadline reduction {100*stage_b_summary.get('deadline_miss_reduction', 0.0):.2f}%.",
            f"- Safety={stage_b_summary.get('safety_pass', False)}, final-valid parity={stage_b_summary.get('final_valid_equal', False)}, engineering gate={stage_b_summary.get('engineering_value_pass', False)}.",
            f"- Scheduler-only L3 no-op pairs: {stage_b_summary.get('scheduler_no_l3_replan_count', 0)}; these samples are excluded from the D* E2E value gate.",
            f"- A2B-19 long-tail control E2E P50: incremental {tail_inc.get('p50', float('nan')):.3f} ms, cold Graph A* {tail_astar.get('p50', float('nan')):.3f} ms; excluded from the primary conclusion.",
            "- Corridor caches are independent per arm and start under identical conditions; hit/miss/build/memory values are recorded in `cache_and_state_diagnostics.csv`.",
        ])
    lines.extend([
        "",
        "## Reproduction",
        "",
        "```bash",
        "source /opt/ros/humble/setup.bash",
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash",
        f"ROS_DOMAIN_ID=97 ros2 run arena_evaluation two_layer_2d_v1_dynamic_incremental_benchmark --output-dir {output} --warmups {DEFAULT_WARMUPS} --repetitions {DEFAULT_REPETITIONS} --ros-repetitions {DEFAULT_ROS_REPETITIONS} --ros-domain-id 97",
        "```",
    ])
    (output / "final_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return verdict


def run_formal(
    output: Path,
    *, warmups: int = DEFAULT_WARMUPS,
    repetitions: int = DEFAULT_REPETITIONS,
    main_query_count: int = DEFAULT_MAIN_QUERY_COUNT,
    seed: int = DEFAULT_SEED,
    ros_domain_id: int = 97,
    ros_repetitions: int = DEFAULT_ROS_REPETITIONS,
    stage_a_only: bool = False,
) -> Path:
    output = output.resolve()
    _refuse_nonempty(output)
    if not 8 <= int(main_query_count) <= 10:
        raise ValueError("main_query_count must be between 8 and 10")
    if not 20 <= int(repetitions) <= 30:
        raise ValueError("formal repetitions must be between 20 and 30")
    frozen_hash_before = _tree_hash(FROZEN_R2)
    baseline_source_audit = _baseline_source_audit()
    output.mkdir(parents=True)
    events_dir = output / "dynamic_event_streams"
    events_dir.mkdir()

    frozen = _load_frozen_validity()
    queries, task_metadata = task_source._load_tasks()
    ctx = task_source._context()
    artifact, topology_info, source_audit = r2_benchmark._load_frozen_r1_topology(
        ctx, DEFAULT_CACHE_ROOT,
    )
    if topology_info["topology_cache_key"] != FROZEN_TOPOLOGY_KEY:
        raise RuntimeError("frozen topology cache key mismatch")
    graph_view = r1_pipeline.build_static_topology_view(artifact)
    graph_view.metadata["topology_cache_key"] = topology_info["topology_cache_key"]
    query_graphs = _build_query_graphs(
        graph_view, topology_info, ctx, queries, frozen,
    )
    edge_cells = _edge_cells(graph_view)
    exclusive, any_witness, _cell_edges = _witness_maps(edge_cells)
    scenarios = _build_scenarios(
        query_graphs, frozen, exclusive, any_witness,
        seed=seed, main_query_count=main_query_count,
    )
    event_payloads: Dict[str, List[str]] = {}
    event_paths: List[Path] = []
    for scenario in scenarios:
        path, payloads = _write_event_stream(
            events_dir, scenario, map_version=ctx.map_sha256,
            map_shape=artifact.free_mask.shape, seed=seed,
        )
        event_paths.append(path)
        event_payloads[scenario.scenario_id] = payloads

    scenario_rows = []
    for scenario in scenarios:
        query_graph = query_graphs[scenario.query_id]
        scenario_rows.append({
            **asdict(scenario),
            "static_final_valid": frozen.get(scenario.query_id, {}).get("final_valid"),
            "static_path_hash": frozen.get(scenario.query_id, {}).get("path_hash", ""),
            "static_route_edge_count": len(query_graph.baseline_edge_ids),
            "static_route_cost": query_graph.baseline_cost,
            "topology_static_hash": query_graph.template.static_hash,
            "event_stream": f"dynamic_event_streams/{scenario.scenario_id}.json",
            "seed": seed, "snapshot_count": 21,
            "included_in_primary_dstar_conclusion": scenario.analysis_group == "main",
        })
    _write_csv(output / "scenario_manifest.csv", scenario_rows)

    all_rows: List[Dict[str, Any]] = []
    for run_mode, count in (("warmup", warmups), ("measured", repetitions)):
        for repetition in range(1, int(count) + 1):
            for scenario_index, scenario in enumerate(scenarios):
                graph = query_graphs[scenario.query_id]
                order = list(value.ARMS)
                rotation = (repetition + scenario_index) % len(order)
                order = order[rotation:] + order[:rotation]
                rows = value.run_paired_episode(
                    graph.template, event_payloads[scenario.scenario_id], edge_cells,
                    map_version=ctx.map_sha256, map_shape=artifact.free_mask.shape,
                    arm_order=order,
                )
                _annotate_rows(
                    rows, scenario, graph, run_mode=run_mode,
                    repetition=repetition, topology_edge_count=len(edge_cells),
                )
                all_rows.extend(rows)
    _write_csv(output / "paired_algorithm_runs.csv", all_rows)

    correctness = _correctness_rows(all_rows)
    timing = _timing_summaries(all_rows)
    expanded = _expanded_summaries(all_rows)
    break_even = _break_even(all_rows)
    gates = _stage_a_gates(all_rows, correctness)
    _write_csv(output / "correctness_oracle.csv", correctness)
    _write_csv(output / "expanded_nodes_summary.csv", expanded)
    _write_csv(output / "break_even_curve.csv", break_even)

    ros_rows: List[Dict[str, Any]] = []
    stage_b_summary: Dict[str, Any] = {}
    if not gates["stage_a_pass"]:
        stage_b_status = "NOT_RUN_STAGE_A_FAILED"
    elif stage_a_only:
        stage_b_status = "NOT_RUN_STAGE_A_ONLY_REQUESTED"
    else:
        ros_rows, stage_b_status, stage_b_summary, ros_timing = _run_ros_stage_b(
            output, scenarios, query_graphs, event_payloads, edge_cells,
            graph_view, topology_info, ctx, ros_domain_id=ros_domain_id,
            repetitions=ros_repetitions,
        )
        timing.extend(ros_timing)
    if ros_rows:
        _write_csv(output / "paired_ros_runs.csv", ros_rows)
    else:
        _write_csv(output / "paired_ros_runs.csv", [{
            "status": stage_b_status, "reason": (
                "Stage A did not satisfy all algorithm-value gates"
                if not gates["stage_a_pass"] else "stage-a-only mode"
            ),
        }])
    _write_csv(output / "timing_summary.csv", timing)

    failure_rows = []
    for field in (
        "input_hash_match", "edge_status_hash_match", "edge_cost_hash_match",
        "reachable_parity", "cost_parity", "blocked_edge_absent",
        "route_edge_ids_equal",
        "no_route_classification_match", "incremental_no_reinitialize",
        "topology_immutable", "all_correct",
    ):
        failure_rows.append({
            "check": field,
            "failure_count": sum(not bool(row[field]) for row in correctness),
            "total_count": len(correctness),
        })
    for check, passed in sorted(stage_b_summary.items()):
        if str(check).endswith("_pass") or check in {"final_valid_equal"}:
            failure_rows.append({
                "check": f"stage_b_{check}",
                "failure_count": 0 if bool(passed) else 1,
                "total_count": 1,
            })
    _write_csv(output / "failure_summary.csv", failure_rows)
    index_for_diagnostics = value.CellToEdgeIndex(edge_cells)
    cache_rows = _cache_state_rows(
        graph_view, topology_info, query_graphs, all_rows, index_for_diagnostics,
        stage_b_summary,
    )
    _write_csv(output / "cache_and_state_diagnostics.csv", cache_rows)

    protocol = {
        "experiment_id": output.name,
        "architecture_id": value.ARCHITECTURE_ID,
        "implementation_revision": value.IMPLEMENTATION_REVISION,
        "parent_architecture": value.PARENT_ARCHITECTURE,
        "experiment_kind": value.EXPERIMENT_KIND,
        "protocol_version": value.PROTOCOL_VERSION,
        "map_id": MAP_ID, "resolution_m": 0.05,
        "dynamic_obstacles": True,
        "layers": {"L1": "2A-V0 static skeleton topology + paired graph search",
                   "L2": "disabled", "L3_prime": "unchanged r2 Smac Hybrid DUBIN"},
        "arms": {
            value.INCREMENTAL_DSTAR: "one initial plan; retain g/rhs/OPEN/km; update changed edges only",
            value.COLD_DSTAR: "fresh Graph D* Lite for every snapshot",
            value.COLD_GRAPH_ASTAR: "fresh deterministic reverse Graph A* oracle for every snapshot",
        },
        "dynamic_state_machine": [
            "AVAILABLE", "BLOCKED_PENDING", "BLOCKED", "RECOVERING", "AVAILABLE",
        ],
        "blocked_cost": "INF", "dynamic_layer": "M_dynamic/edge-cost overlay only",
        "static_map_mutation": False, "static_topology_mutation": False,
        "episode": {"initial_snapshot": "S0", "dynamic_snapshots": "S1..S20",
                    "pipeline_rebuild_within_episode": False},
        "warmups": warmups, "repetitions": repetitions, "seed": seed,
        "minimum_turning_radius_m": 0.40, "maximum_curvature_1pm": 2.50,
        "allow_reverse": False, "allow_in_place_rotation": False,
        "smac_motion_model": "DUBIN", "topology_refinement_enabled": False,
        "stage_b_admission": gates,
        "ros_domain_id": ros_domain_id,
        "task_metadata": task_metadata,
    }
    (output / "protocol.yaml").write_text(
        yaml.safe_dump(protocol, sort_keys=False), encoding="utf-8",
    )

    source_snapshot = _snapshot_sources(output, event_paths)
    frozen_hash_after = _tree_hash(FROZEN_R2)
    if frozen_hash_after != frozen_hash_before:
        raise RuntimeError("frozen r2 directory changed during the dynamic experiment")
    manifest = {
        "experiment_id": output.name,
        "architecture_id": value.ARCHITECTURE_ID,
        "implementation_revision": value.IMPLEMENTATION_REVISION,
        "parent_architecture": value.PARENT_ARCHITECTURE,
        "experiment_kind": value.EXPERIMENT_KIND,
        "protocol_version": value.PROTOCOL_VERSION,
        "formal": True, "stage_a": gates, "stage_b_status": stage_b_status,
        "stage_b": stage_b_summary,
        "main_query_count": main_query_count,
        "scenario_count": len(scenarios), "snapshots_per_episode": 21,
        "warmups": warmups, "measured_repetitions": repetitions,
        "algorithm_row_count": len(all_rows),
        "correctness_row_count": len(correctness),
        "topology_cache_key": topology_info["topology_cache_key"],
        "topology_nodes": len(graph_view.nodes), "topology_edges": len(graph_view.edges),
        "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256,
        "frozen_r2_directory": str(FROZEN_R2),
        "frozen_r2_tree_hash_before": frozen_hash_before,
        "frozen_r2_tree_hash_after": frozen_hash_after,
        "frozen_r2_unchanged": frozen_hash_before == frozen_hash_after,
        "source_snapshot_file_count": source_snapshot["file_count"],
        "source_snapshot_hash": source_snapshot["combined_hash"],
        "source_audit": source_audit,
        "frozen_r2_source_audit": baseline_source_audit,
    }
    verdict = _report(
        output, gates, correctness, timing, expanded, break_even, scenarios,
        stage_b_status=stage_b_status, stage_b_summary=stage_b_summary,
    )
    manifest["final_verdict"] = verdict
    (output / "manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8",
    )
    artifact_validation = _validate_formal_artifacts(output)
    if not artifact_validation["passed"]:
        raise RuntimeError(f"formal artifact validation failed: {artifact_validation}")
    manifest["artifact_validation"] = artifact_validation
    (output / "manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8",
    )
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the formal 2D-V1-r3 D* Lite dynamic incremental value experiment",
    )
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--warmups", type=int, default=DEFAULT_WARMUPS)
    parser.add_argument("--repetitions", type=int, default=DEFAULT_REPETITIONS)
    parser.add_argument("--main-query-count", type=int, default=DEFAULT_MAIN_QUERY_COUNT)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--ros-domain-id", type=int, default=97)
    parser.add_argument("--ros-repetitions", type=int, default=DEFAULT_ROS_REPETITIONS)
    parser.add_argument("--stage-a-only", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    output = args.output_dir or _default_output()
    try:
        path = run_formal(
            output, warmups=args.warmups, repetitions=args.repetitions,
            main_query_count=args.main_query_count, seed=args.seed,
            ros_domain_id=args.ros_domain_id, ros_repetitions=args.ros_repetitions,
            stage_a_only=args.stage_a_only,
        )
    except Exception as exc:
        print(f"two_layer_2d_v1_dynamic_incremental_benchmark: ERROR: {exc}")
        return 2
    print(f"2D-V1 dynamic incremental output: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["build_parser", "main", "run_formal"]
