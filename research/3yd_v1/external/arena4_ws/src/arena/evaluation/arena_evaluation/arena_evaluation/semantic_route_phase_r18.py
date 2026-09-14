"""Fail-closed one-layer gap bridging for the 2A-V3 r18 research arm.

The frozen r17 policy deliberately permits only adjacent guide layers because
unconditional two-layer connections produced a geometric revisit on the first
parking query.  A different failure mode appears when an adjacent layer has no
valid Dubins/footprint edge even though the following layer is locally
reachable.  This adapter tries the following layer *only when* the adjacent
layer has zero valid successors.  All original distance, Dubins, monotonic
projection, semantic phase, footprint and hard-cost checks remain unchanged.
"""
from __future__ import annotations

import math

from .semantic_constraint_core import dubins_choices
from .semantic_route_phase_v3 import Edge, LazyRoutePhaseSearch


METHOD_ID = "adjacent_first_semantic_boundary_gap_bridge_r18_v2"


class AdjacentFirstGapBridgeSearchR18(LazyRoutePhaseSearch):
    """Use a bounded bridge only at a dead end or semantic phase boundary."""

    def _edges_to_layer(self, source_id: int, target_layer: int) -> list[Edge]:
        source = self.states[source_id]
        specs = []
        for target_id in self.layers[target_layer]:
            target = self.states[target_id]
            direct = math.dist(source.pose[:2], target.pose[:2])
            if (
                direct <= self.policy.projection_epsilon_m
                or direct > self.policy.maximum_local_edge_length_m
            ):
                self.rejected["EDGE_DISTANCE_BOUND"] += 1
                continue
            bound = min(
                self.policy.maximum_local_edge_length_m,
                self.policy.maximum_local_edge_ratio * direct + 0.05,
            )
            for choice, (word, params) in enumerate(
                dubins_choices(
                    source.pose, target.pose, self.policy.turning_radius_m
                )
            ):
                length = self.policy.turning_radius_m * sum(params)
                if length <= bound:
                    specs.append(
                        (
                            100.0 * (not target.semantic_correct)
                            + 50.0 * (not target.semantic_target)
                            + 2.0 * target.semantic_error
                            + length,
                            not target.semantic_target,
                            target.semantic_error,
                            length,
                            target.yaw_bin,
                            target.state_id,
                            choice,
                            word,
                            params,
                        )
                    )
                else:
                    self.rejected["DUBINS_LENGTH_BOUND"] += 1
        result: list[Edge] = []
        for *_rank, target_id, choice, word, params in sorted(specs):
            self.counters["sampled_state_pairs"] += 1
            edge = self._edge(
                source, self.states[target_id], choice, word, params
            )
            if edge is None:
                continue
            result.append(edge)
            if len(result) >= self.policy.valid_successors_per_target_layer:
                break
        return result

    def successors(self, source_id: int) -> tuple[Edge, ...]:
        if source_id in self.cache:
            self.counters["cache_hit"] += 1
            return self.cache[source_id]
        source = self.states[source_id]
        adjacent = source.layer_index + 1
        result: list[Edge] = []
        if adjacent < len(self.layers):
            result = self._edges_to_layer(source_id, adjacent)
        bridge_layer = adjacent + 1
        boundary = False
        if bridge_layer < len(self.layers):
            source_key = (source.phase_kind, source.phase_instance)
            adjacent_state = self.states[self.layers[adjacent][0]]
            bridge_state = self.states[self.layers[bridge_layer][0]]
            adjacent_key = (
                adjacent_state.phase_kind,
                adjacent_state.phase_instance,
            )
            bridge_key = (bridge_state.phase_kind, bridge_state.phase_instance)
            boundary = source_key != adjacent_key or adjacent_key != bridge_key
        if bridge_layer < len(self.layers) and (not result or boundary):
            reason = "semantic_boundary" if boundary else "empty_adjacent"
            self.counters[f"{reason}_bridge_attempts"] += 1
            bridged = self._edges_to_layer(source_id, bridge_layer)
            result.extend(bridged)
            if bridged:
                self.counters[f"{reason}_bridge_successes"] += 1
        result.sort(
            key=lambda edge: (
                edge.target_layer,
                edge.target,
                edge.length_m,
                edge.dubins_choice,
            )
        )
        self.cache[source_id] = tuple(result)
        self.counters["valid_edges"] += len(result)
        return self.cache[source_id]


__all__ = ["METHOD_ID", "AdjacentFirstGapBridgeSearchR18"]
