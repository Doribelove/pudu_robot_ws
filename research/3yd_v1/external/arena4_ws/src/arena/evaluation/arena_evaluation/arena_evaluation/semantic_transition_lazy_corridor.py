"""On-demand directed SE(2) corridor search for 2A-V3.

Candidate states are identical to the ordered-corridor preflight.  Unlike the
eager oracle, this implementation solves and footprint-validates Dubins edges
only when a live search label expands.  A bounded, deterministic successor
selection ranks all analytically feasible target states before sampling and
retains a configured number of valid edges per future station layer.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import heapq
import math
import time
from typing import Any

import numpy as np

from .semantic_constraint_core import dubins_choices, dubins_edge_from_parameters
from .semantic_transition_ordered_corridor import (
    CorridorEdge,
    OrderedCorridorPolicy,
    OrientedRoute,
    _edge_semantics,
    _progressive_edge,
    build_candidate_layers,
    evaluate_controls,
)


@dataclass(frozen=True)
class LazySearchPolicy:
    progress_priority_per_m: float = 55.0
    live_labels_per_state: int = 4
    valid_successors_per_target_layer: int = 6
    maximum_expanded_labels: int = 5000
    maximum_goal_candidates: int = 32

    def __post_init__(self) -> None:
        if self.progress_priority_per_m < 0.0:
            raise ValueError("progress priority cannot be negative")
        if min(
            self.live_labels_per_state,
            self.valid_successors_per_target_layer,
            self.maximum_expanded_labels,
            self.maximum_goal_candidates,
        ) < 1:
            raise ValueError("lazy search bounds must be positive")


@dataclass(frozen=True)
class LazyLabel:
    label_id: int
    node: int
    semantic_score: float
    length_m: float
    wrong_side_m: float
    outside_target_m: float
    error_integral_m2: float
    parent: int
    edge: CorridorEdge | None
    signature: tuple[int, ...]


class LazyEdgeFactory:
    def __init__(
        self, world: Any, route: OrientedRoute, ordered: OrderedCorridorPolicy,
        *, candidate_cell_provider=None,
        valid_successors_per_target_layer: int,
    ) -> None:
        self.world, self.route, self.ordered = world, route, ordered
        self.lane_label, self.states, self.layers, candidate_diagnostics = build_candidate_layers(
            world, route, ordered, candidate_cell_provider=candidate_cell_provider,
        )
        self.valid_successors_per_target_layer = int(valid_successors_per_target_layer)
        self.cache: dict[int, tuple[CorridorEdge, ...]] = {}
        self.rejected = Counter()
        self.counters = Counter()
        self.candidate_diagnostics = candidate_diagnostics
        self.started = time.monotonic()

    def _candidate_specs(self, source, targets):
        specs = []
        for target_id in targets:
            target = self.states[target_id]
            direct = math.dist(source.pose[:2], target.pose[:2])
            if direct <= self.ordered.projection_epsilon_m or direct > self.ordered.maximum_local_edge_length_m:
                self.rejected["EDGE_DISTANCE_BOUND"] += 1
                continue
            bound = min(
                self.ordered.maximum_local_edge_length_m,
                self.ordered.maximum_local_edge_ratio * direct + 0.05,
            )
            choices = dubins_choices(source.pose, target.pose, self.ordered.turning_radius_m)
            within = [
                (choice, word, params, self.ordered.turning_radius_m * sum(params))
                for choice, (word, params) in enumerate(choices)
                if self.ordered.turning_radius_m * sum(params) <= bound
            ]
            self.rejected["DUBINS_LENGTH_BOUND"] += len(choices) - len(within)
            if not within:
                continue
            minimum_length = min(item[3] for item in within)
            error = target.error_m if math.isfinite(target.error_m) else 0.0
            proxy = (
                minimum_length
                + 100.0 * (not target.correct_side)
                + 50.0 * (not target.target_band)
                + 2.0 * error
            )
            specs.append((
                proxy, not target.target_band, not target.correct_side, error,
                minimum_length, target.yaw_bin, target.state_id, within,
            ))
        return sorted(specs)

    def _validated_edge(self, source, target, within):
        accepted = []
        for choice, word, params, _analytic_length in within:
            control = dubins_edge_from_parameters(
                source.pose, target.pose, self.ordered.turning_radius_m,
                word, params,
            )
            progressive, _ = _progressive_edge(
                self.route, control, source, target,
                self.ordered.projection_epsilon_m, dense=False,
            )
            if not progressive:
                self.rejected["NON_MONOTONE_OR_OVERSHOOT_EDGE"] += 1
                continue
            rows, cols, inside = self.world.cells(control.samples)
            if not np.all(inside) or np.any(
                self.world.grids["labels"][rows, cols] != self.lane_label
            ):
                self.rejected["LANE_INSTANCE_EDGE_ESCAPE"] += 1
                continue
            if not self.world.validate_edge(control, dense=True):
                self.rejected["FULL_FOOTPRINT_OR_HARD_EDGE"] += 1
                continue
            n, correct, target_count, error_integral = _edge_semantics(self.world, control)
            accepted.append((
                50.0 * max(0, n - correct)
                + 20.0 * max(0, n - target_count)
                + error_integral + control.length,
                choice, control, n, correct, target_count, error_integral,
            ))
        if not accepted:
            return None
        _, choice, control, n, correct, target_count, error_integral = min(
            accepted, key=lambda item: (item[0], item[2].length, item[1], item[2].word),
        )
        canonical_id = ((source.state_id * len(self.states) + target.state_id) * 6 + choice)
        return CorridorEdge(
            edge_id=canonical_id,
            source=source.state_id,
            target=target.state_id,
            source_layer=source.layer_index,
            target_layer=target.layer_index,
            dubins_choice=choice,
            length_m=float(control.length),
            lane_samples=n,
            correct_samples=correct,
            target_samples=target_count,
            error_integral_m2=error_integral,
            control=control,
        )

    def successors(self, source_id: int) -> tuple[CorridorEdge, ...]:
        if source_id in self.cache:
            self.counters["successor_cache_hits"] += 1
            return self.cache[source_id]
        self.counters["expanded_source_states"] += 1
        source = self.states[source_id]
        result = []
        last_layer = min(
            len(self.layers),
            source.layer_index + 1 + self.ordered.maximum_station_skip,
        )
        for target_layer in range(source.layer_index + 1, last_layer):
            valid_count = 0
            specs = self._candidate_specs(source, self.layers[target_layer])
            self.counters["analytically_ranked_state_pairs"] += len(specs)
            for *_rank, target_id, within in specs:
                self.counters["sampled_state_pairs"] += 1
                edge = self._validated_edge(source, self.states[target_id], within)
                if edge is None:
                    continue
                result.append(edge)
                valid_count += 1
                if valid_count >= self.valid_successors_per_target_layer:
                    break
        result.sort(key=lambda edge: (
            edge.target_layer, edge.target, edge.length_m, edge.dubins_choice,
        ))
        self.cache[source_id] = tuple(result)
        self.counters["generated_valid_edges"] += len(result)
        return self.cache[source_id]

    def diagnostics(self) -> dict[str, Any]:
        return {
            **self.candidate_diagnostics,
            "edge_generation": "on_demand_bounded_successors",
            "materialized_edge_count": int(self.counters["generated_valid_edges"]),
            "edge_rejection_counts": dict(sorted(self.rejected.items())),
            "expanded_source_state_count": int(self.counters["expanded_source_states"]),
            "analytically_ranked_state_pair_count": int(self.counters["analytically_ranked_state_pairs"]),
            "sampled_state_pair_count": int(self.counters["sampled_state_pairs"]),
            "successor_cache_hit_count": int(self.counters["successor_cache_hits"]),
            "lazy_graph_wall_s": time.monotonic() - self.started,
        }


def _label_key(label: LazyLabel):
    return (
        label.semantic_score, label.length_m, label.wrong_side_m,
        label.outside_target_m, label.signature,
    )


def _controls(label: LazyLabel, labels: list[LazyLabel]):
    result = []
    current = label
    while current.parent >= 0:
        result.append(current.edge.control)
        current = labels[current.parent]
    result.reverse()
    return result


def search_lazy(
    factory: LazyEdgeFactory, world: Any, semantic_map: Any,
    policy: LazySearchPolicy,
) -> tuple[dict[str, Any] | None, list[dict[str, Any]], dict[str, Any]]:
    labels: list[LazyLabel] = []
    active: dict[int, list[int]] = {state.state_id: [] for state in factory.states}
    heap = []
    counters = Counter()

    def add(node, score, length, wrong, outside, error, parent, edge, signature):
        label = LazyLabel(
            len(labels), node, score, length, wrong, outside, error,
            parent, edge, signature,
        )
        labels.append(label)
        bucket = active[node]
        bucket.append(label.label_id)
        bucket.sort(key=lambda index: _label_key(labels[index]))
        del bucket[policy.live_labels_per_state:]
        if label.label_id not in bucket:
            counters["dominated_on_admission"] += 1
            return
        state = factory.states[node]
        remaining = max(0.0, factory.route.length_m - state.station_m)
        priority = score + policy.progress_priority_per_m * remaining
        heapq.heappush(heap, (
            priority, *_label_key(label), label.label_id,
        ))

    start = factory.layers[0][0]
    goal = factory.layers[-1][0]
    add(start, 0.0, 0.0, 0.0, 0.0, 0.0, -1, None, ())
    evaluations = []
    witness = None
    while heap and counters["expanded_labels"] < policy.maximum_expanded_labels:
        *_priority, label_id = heapq.heappop(heap)
        label = labels[label_id]
        if label_id not in active[label.node]:
            counters["stale_heap_labels"] += 1
            continue
        if label.node == goal:
            counters["goal_candidates"] += 1
            controls = _controls(label, labels)
            evaluated = evaluate_controls(world, factory.route, controls, semantic_map)
            lane = evaluated.get("semantics", {}).get("active_window", {}).get("classes", {}).get("lane", {})
            evaluations.append({
                "rank": len(evaluations),
                "semantic_score": label.semantic_score,
                "path_length_m": label.length_m,
                "correct_side_ratio": lane.get("correct_side_ratio"),
                "target_band_ratio": lane.get("target_band_ratio"),
                "lateral_error_p50_m": lane.get("lateral_error_p50_m"),
                "failure_codes": evaluated["failure_codes"],
                "gate_passed": evaluated["gate_passed"],
            })
            if evaluated["gate_passed"]:
                witness = evaluated
                break
            if counters["goal_candidates"] >= policy.maximum_goal_candidates:
                break
            continue
        counters["expanded_labels"] += 1
        for edge in factory.successors(label.node):
            wrong = label.wrong_side_m + edge.wrong_side_m
            outside = label.outside_target_m + edge.outside_target_m
            error = label.error_integral_m2 + edge.error_integral_m2
            length = label.length_m + edge.length_m
            score = length + 100.0 * wrong + 50.0 * outside + 2.0 * error
            add(
                edge.target, score, length, wrong, outside, error,
                label.label_id, edge, label.signature + (edge.edge_id,),
            )
    diagnostics = {
        "expanded_label_count": int(counters["expanded_labels"]),
        "generated_label_count": len(labels),
        "active_label_count": sum(len(bucket) for bucket in active.values()),
        "dominated_on_admission_count": int(counters["dominated_on_admission"]),
        "stale_heap_label_count": int(counters["stale_heap_labels"]),
        "goal_candidate_count": int(counters["goal_candidates"]),
        "remaining_heap_count": len(heap),
        "resource_limit_reached": bool(
            witness is None and counters["expanded_labels"] >= policy.maximum_expanded_labels
        ),
    }
    return witness, evaluations, diagnostics
