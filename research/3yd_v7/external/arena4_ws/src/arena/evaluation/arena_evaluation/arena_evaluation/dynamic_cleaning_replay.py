"""Deterministic realistic-synthetic cleaning workload for 2D-V3.

No compliant real cleaning trace is present in the workspace.  This generator
therefore models cleaning-floor obstacle motifs using topology-edge witness
cells.  It never labels its output as real data and records the exact synthetic
parameters needed to replay every snapshot byte-for-byte.
"""

from __future__ import annotations

import json
import random
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Sequence, Tuple

from .dynamic_snapshot import DynamicSnapshot
from .graph_dstar_lite import GraphDStarLite


@dataclass(frozen=True)
class CleaningWorkload:
    scenario_id: str
    split: str
    query_id: str
    category: str
    target_edges: Tuple[str, ...]
    witness_cells: Mapping[str, Tuple[int, int]]
    obstacle_count: int
    nominal_speed_mps: float
    observation_hz: float
    duration_s: float
    expected_path_affected: bool
    expected_no_route: bool = False
    min_cut_size: int = 0
    source_kind: str = "realistic_synthetic_cleaning_workload"
    scale_family: str = "cleaning_semantic"
    requested_changed_edges: int = 0
    requested_changed_ratio: float = 0.0


CATEGORIES = (
    "single_person_crossing",
    "multiple_people_crossing",
    "crowd_gathering",
    "obstacle_moving_along_corridor",
    "slow_cart_lane_occupation",
    "doorway_persistent_block",
    "multiple_channels_changing",
    "high_frequency_transient_obstacle",
    "obstacle_jitter_false_positive",
    "obstacle_disappearance_recovery",
    "bridge_min_cut_no_route",
    "many_obstacles_outside_path",
    "few_obstacles_inside_path",
    "many_obstacles_inside_path",
)


_PROFILE = {
    "single_person_crossing": (1, 1.20),
    "multiple_people_crossing": (5, 1.10),
    "crowd_gathering": (20, 0.35),
    "obstacle_moving_along_corridor": (3, 0.80),
    "slow_cart_lane_occupation": (1, 0.25),
    "doorway_persistent_block": (2, 0.00),
    "multiple_channels_changing": (20, 0.55),
    "high_frequency_transient_obstacle": (1, 1.50),
    "obstacle_jitter_false_positive": (1, 0.05),
    "obstacle_disappearance_recovery": (1, 0.70),
    "bridge_min_cut_no_route": (2, 0.00),
    "many_obstacles_outside_path": (100, 0.60),
    "few_obstacles_inside_path": (2, 1.00),
    "many_obstacles_inside_path": (100, 0.45),
}


def _pick(pool: Sequence[str], count: int, seed: int) -> Tuple[str, ...]:
    values = list(dict.fromkeys(str(edge) for edge in pool))
    random.Random(seed).shuffle(values)
    if len(values) < count:
        raise RuntimeError(f"only {len(values)} edge witnesses available, need {count}")
    return tuple(sorted(values[:count]))


def build_workloads(
    query_graphs: Mapping[str, Any],
    exclusive: Mapping[str, Tuple[int, int]],
    any_witness: Mapping[str, Tuple[int, int]],
    *, seed: int,
    minimum_cut: Any,
) -> List[CleaningWorkload]:
    """Build paired calibration/held-out workloads for every cleaning motif."""
    eligible = [
        graph for query_id, graph in sorted(query_graphs.items())
        if graph.baseline_edge_ids and query_id not in {"A2B-07", "A2B-16", "A2B-19"}
    ]
    eligible.sort(key=lambda graph: (len(graph.baseline_edge_ids), graph.query.query_id))
    if len(eligible) < 10:
        raise RuntimeError("2D-V3 requires at least ten L1-reachable non-control queries")
    quantiles = [round(index * (len(eligible) - 1) / 9) for index in range(10)]
    selected = [eligible[index] for index in quantiles]
    all_exclusive = tuple(sorted(exclusive))
    workloads: List[CleaningWorkload] = []
    for split_index, split in enumerate(("calibration", "held_out")):
        for category_index, category in enumerate(CATEGORIES):
            graph = selected[(category_index + split_index * 3) % len(selected)]
            route = tuple(edge for edge in graph.baseline_edge_ids if edge in exclusive)
            route_set = set(graph.baseline_edge_ids)
            outside = tuple(edge for edge in all_exclusive if edge not in route_set)
            requested_count, speed = _PROFILE[category]
            expected_no_route = False
            min_cut_size = 0
            local_seed = seed + split_index * 10000 + category_index * 101
            if category in {"doorway_persistent_block", "bridge_min_cut_no_route"}:
                cut = tuple(str(edge) for edge in minimum_cut(graph))
                targets = cut
                expected_no_route = True
                min_cut_size = len(cut)
                witnesses = {
                    edge: tuple(any_witness[edge]) for edge in targets
                }
            elif category == "many_obstacles_outside_path":
                targets = _pick(outside, 100, local_seed)
                witnesses = {edge: tuple(exclusive[edge]) for edge in targets}
            elif category == "obstacle_jitter_false_positive":
                targets = _pick(outside, 1, local_seed)
                witnesses = {edge: tuple(exclusive[edge]) for edge in targets}
            elif category == "many_obstacles_inside_path":
                on_route_count = min(len(route), 12)
                on_route = _pick(route, on_route_count, local_seed)
                fill = _pick(
                    tuple(edge for edge in outside if edge not in on_route),
                    100 - len(on_route), local_seed + 1,
                )
                targets = tuple(sorted((*on_route, *fill)))
                witnesses = {edge: tuple(exclusive[edge]) for edge in targets}
            else:
                count = requested_count
                on_route_count = min(len(route), count)
                if category in {
                    "single_person_crossing", "slow_cart_lane_occupation",
                    "high_frequency_transient_obstacle",
                    "obstacle_disappearance_recovery",
                }:
                    on_route_count = 1
                elif category == "obstacle_moving_along_corridor":
                    on_route_count = min(3, len(route))
                elif category == "few_obstacles_inside_path":
                    on_route_count = min(2, len(route))
                else:
                    on_route_count = min(max(1, count // 4), len(route))
                on_route = _pick(route, on_route_count, local_seed)
                fill = _pick(outside, count - len(on_route), local_seed + 1)
                targets = tuple(sorted((*on_route, *fill)))
                witnesses = {edge: tuple(exclusive[edge]) for edge in targets}
            scenario_id = f"{split[:3].upper()}-{category_index + 1:02d}"
            obstacle_count = max(requested_count, len(targets))
            workloads.append(CleaningWorkload(
                scenario_id=scenario_id, split=split,
                query_id=str(graph.query.query_id), category=category,
                target_edges=targets, witness_cells=witnesses,
                obstacle_count=obstacle_count, nominal_speed_mps=speed,
                observation_hz=2.0, duration_s=10.0,
                expected_path_affected=category not in {
                    "obstacle_jitter_false_positive", "many_obstacles_outside_path",
                },
                expected_no_route=expected_no_route,
                min_cut_size=min_cut_size,
                requested_changed_edges=len(targets),
                requested_changed_ratio=len(targets) / max(1, len(all_exclusive)),
            ))
    return workloads


def build_ratio_matched_workloads(
    query_graphs: Mapping[str, Any],
    exclusive: Mapping[str, Tuple[int, int]],
    *, seed: int, topology_edge_count: int,
    select_alternate: Any, blocked_oracle: Any,
) -> List[CleaningWorkload]:
    """Reproduce the 1x-normalized 0.046/0.092/0.230/0.921/4.604% points."""
    targets = ((0.00046, 2), (0.00092, 4), (0.00230, 11),
               (0.00921, 42), (0.04604, 210))
    eligible = [
        graph for query_id, graph in sorted(query_graphs.items())
        if graph.baseline_edge_ids and query_id not in {"A2B-07", "A2B-16", "A2B-19"}
        and select_alternate(graph, exclusive) is not None
    ]
    if len(eligible) < len(targets):
        raise RuntimeError("not enough alternate-route queries for ratio-matched workload")
    result = []
    all_edges = tuple(sorted(exclusive))
    for index, (nominal_ratio, count) in enumerate(targets):
        graph = eligible[index]
        alternate = str(select_alternate(graph, exclusive))
        alternate_route = blocked_oracle(graph, (alternate,))
        protected = set(alternate_route.edge_path)
        baseline = set(graph.baseline_edge_ids)
        pool = [edge for edge in all_edges
                if edge != alternate and edge not in protected and edge not in baseline]
        fill = _pick(pool, count - 1, seed + 50000 + index)
        changed = tuple(sorted((alternate, *fill)))
        if blocked_oracle(graph, changed).node_path is None:
            raise RuntimeError(f"ratio point {count} unexpectedly removes all routes")
        result.append(CleaningWorkload(
            scenario_id=f"RATIO-{index + 1:02d}", split="ratio_matched",
            query_id=str(graph.query.query_id),
            category=f"ratio_matched_{nominal_ratio * 100:.3f}pct",
            target_edges=changed,
            witness_cells={edge: tuple(exclusive[edge]) for edge in changed},
            obstacle_count=count, nominal_speed_mps=0.5,
            observation_hz=2.0, duration_s=10.0,
            expected_path_affected=True, source_kind="realistic_synthetic_scale_ablation",
            scale_family="ratio_matched", requested_changed_edges=count,
            requested_changed_ratio=nominal_ratio,
        ))
    return result


def occupied_edge_sequence(workload: CleaningWorkload) -> List[Tuple[str, ...]]:
    empty: Tuple[str, ...] = ()
    targets = tuple(workload.target_edges)
    sequence: List[Tuple[str, ...]] = [empty]
    category = workload.category
    if category == "obstacle_moving_along_corridor":
        moving = targets[:3]
        if len(moving) < 3:
            moving = tuple((targets * 3)[:3])
        e1, e2, e3 = moving
        sequence.extend([
            (e1,), (e1,), (e2,), (e2,), (e3,), (e3,), empty, empty,
            (e1,), (e1,), (e2,), (e2,), (e3,), (e3,), empty, empty,
            (e1,), (e1,), empty, empty,
        ])
    elif category in {"slow_cart_lane_occupation", "doorway_persistent_block"}:
        sequence.extend([targets] * 10 + [empty] * 2 + [targets] * 4 + [empty] * 4)
    elif category in {"high_frequency_transient_obstacle", "obstacle_jitter_false_positive"}:
        sequence.extend([targets if index % 2 == 0 else empty for index in range(20)])
    elif category == "crowd_gathering":
        half = targets[: max(1, len(targets) // 2)]
        sequence.extend([half, targets, targets, targets, half, empty, empty, empty] * 2)
        sequence.extend([targets, targets, empty, empty])
    elif category == "multiple_channels_changing":
        half = targets[: max(1, len(targets) // 2)]
        other = targets[max(1, len(targets) // 2):]
        sequence.extend([half, half, other, other, targets, targets, empty, empty] * 2)
        sequence.extend([targets, targets, empty, empty])
    else:
        for _cycle in range(5):
            sequence.extend((targets, targets, empty, empty))
    if len(sequence) != 21:
        raise AssertionError(f"{workload.scenario_id} generated {len(sequence)} snapshots")
    return sequence


def write_event_stream(
    directory: Path, workload: CleaningWorkload, *, map_version: str,
    map_shape: Sequence[int], seed: int,
) -> Tuple[Path, List[str]]:
    snapshots: List[Dict[str, Any]] = []
    payloads: List[str] = []
    for index, occupied_edges in enumerate(occupied_edge_sequence(workload)):
        cells = sorted({workload.witness_cells[edge] for edge in occupied_edges})
        raw: Dict[str, Any] = {
            "snapshot_id": f"{workload.scenario_id}-S{index}",
            "timestamp": float(index) / workload.observation_hz + 1.0,
            "occupied_cells": [list(cell) for cell in cells],
            "obstacle_confidence": {
                "synthetic": 1.0, "obstacle_count": float(workload.obstacle_count),
            },
            "ttl": None, "map_version": str(map_version),
            "map_shape": [int(map_shape[0]), int(map_shape[1])],
            "generator_seed": int(seed), "source_kind": workload.source_kind,
            "category": workload.category,
            "target_occupied_edge_ids": list(occupied_edges),
        }
        snapshot = DynamicSnapshot(
            raw["snapshot_id"], raw["timestamp"], tuple(cells),
            raw["obstacle_confidence"], None, str(map_version), tuple(map_shape),
        )
        raw["snapshot_hash"] = snapshot.snapshot_hash
        snapshots.append(raw)
        payloads.append(json.dumps(raw, sort_keys=True, separators=(",", ":")))
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{workload.scenario_id}.json"
    path.write_text(json.dumps({
        "schema_version": 1,
        "source_kind": workload.source_kind,
        "workload": asdict(workload),
        "snapshots": snapshots,
    }, indent=2, sort_keys=True), encoding="utf-8")
    return path, payloads


def invalid_snapshot_payloads(
    valid_payload: str,
) -> Dict[str, str]:
    """Create deterministic protocol-negative inputs for regression tests."""
    raw = json.loads(valid_payload)
    expired = dict(raw)
    expired.update({"snapshot_id": "expired", "timestamp": -100.0, "ttl": 0.1})
    expired.pop("snapshot_hash", None)
    out_of_order = dict(raw)
    out_of_order.update({"snapshot_id": "out-of-order", "timestamp": raw["timestamp"] - 1.0})
    out_of_order.pop("snapshot_hash", None)
    return {
        "expired": json.dumps(expired, sort_keys=True),
        "out_of_order": json.dumps(out_of_order, sort_keys=True),
    }


__all__ = [
    "CATEGORIES", "CleaningWorkload", "build_workloads",
    "build_ratio_matched_workloads",
    "occupied_edge_sequence", "write_event_stream", "invalid_snapshot_payloads",
]
