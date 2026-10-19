"""Phase-owned primitive selection for the 2A-V3 r16 research arm.

The frozen r13 route-phase lattice ranks a Dubins connection before the
primitive has been replayed.  The ranking therefore knows whether the target
state lies in the semantic band, but not whether the *edge* stays in it.  A
small per-layer successor cap can discard a centre-following parking edge in
favour of an arc whose endpoint is centred but whose interior is not.

R16 keeps the same states, 48 yaw bins, forward-only Dubins primitives,
footprint checks, hard semantics and final audits.  It changes only the
bounded successor interface: a finite number of primitives is materialised
first, then ranked by their measured whole-edge semantic coverage.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from .semantic_constraint_core import dubins_choices
from .semantic_route_phase_v3 import Edge, LazyRoutePhaseSearch


METHOD_ID = "phase_owned_whole_primitive_semantic_ranking_r16_v1"


@dataclass(frozen=True)
class WholePrimitiveSelectionPolicyR16:
    """Independent resource bounds for r16 primitive selection."""

    maximum_materialized_edges_per_target_layer: int = 96
    retained_edges_per_target_layer: int = 12

    def __post_init__(self) -> None:
        if self.maximum_materialized_edges_per_target_layer < 1:
            raise ValueError("r16 materialized-edge bound must be positive")
        if self.retained_edges_per_target_layer < 1:
            raise ValueError("r16 retained-edge bound must be positive")
        if (
            self.retained_edges_per_target_layer
            > self.maximum_materialized_edges_per_target_layer
        ):
            raise ValueError("r16 cannot retain more edges than it materializes")


def whole_edge_semantic_rank(edge: Edge) -> tuple[float | int, ...]:
    """Deterministic rank using the replayed primitive, not just its endpoint."""

    lane_wrong_ratio = edge.lane_wrong_m / max(edge.length_m, 1.0e-12)
    lane_outside_ratio = edge.lane_outside_m / max(edge.length_m, 1.0e-12)
    parking_outside_ratio = edge.parking_outside_m / max(edge.length_m, 1.0e-12)
    return (
        lane_wrong_ratio,
        lane_outside_ratio,
        parking_outside_ratio,
        edge.semantic_error_integral / max(edge.length_m, 1.0e-12),
        edge.length_m,
        edge.target_layer,
        edge.target,
        edge.dubins_choice,
        edge.edge_id,
    )


class WholePrimitiveRoutePhaseSearchR16(LazyRoutePhaseSearch):
    """R13 graph and audits with bounded whole-primitive successor ranking."""

    def __init__(self, world, route, policy, selection_policy=None):
        super().__init__(world, route, policy)
        self.selection_policy = (
            selection_policy or WholePrimitiveSelectionPolicyR16()
        )

    def successors(self, source_id: int) -> tuple[Edge, ...]:
        if source_id in self.cache:
            self.counters["cache_hit"] += 1
            return self.cache[source_id]
        source = self.states[source_id]
        result: list[Edge] = []
        for target_layer in range(
            source.layer_index + 1,
            min(
                len(self.layers),
                source.layer_index + 1 + self.policy.maximum_station_skip,
            ),
        ):
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
                                not target.semantic_target,
                                not target.semantic_correct,
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

            materialized: list[Edge] = []
            for *_rank, target_id, choice, word, params in sorted(specs):
                self.counters["sampled_state_pairs"] += 1
                edge = self._edge(
                    source, self.states[target_id], choice, word, params
                )
                if edge is None:
                    continue
                materialized.append(edge)
                if (
                    len(materialized)
                    >= self.selection_policy.maximum_materialized_edges_per_target_layer
                ):
                    self.counters["materialization_cap_hits"] += 1
                    break

            ranked = sorted(materialized, key=whole_edge_semantic_rank)
            retained = ranked[
                : self.selection_policy.retained_edges_per_target_layer
            ]
            self.counters["materialized_before_semantic_rank"] += len(materialized)
            self.counters["whole_edge_rank_pruned"] += len(materialized) - len(retained)
            result.extend(retained)

        result.sort(
            key=lambda edge: (
                edge.target_layer,
                *whole_edge_semantic_rank(edge),
            )
        )
        self.cache[source_id] = tuple(result)
        self.counters["valid_edges"] += len(result)
        return self.cache[source_id]


__all__ = [
    "METHOD_ID",
    "WholePrimitiveRoutePhaseSearchR16",
    "WholePrimitiveSelectionPolicyR16",
    "whole_edge_semantic_rank",
]
