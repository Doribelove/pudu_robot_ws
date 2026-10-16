"""Bounded reverse reachability in the request's forward-only SE(2) DAG.

Reverse traversal is an analysis order; every tested vehicle primitive still
runs forward from an earlier route station to a later one. No negative graph
result is a proof of continuous-space or semantic query infeasibility.
"""
from __future__ import annotations

import math
import time
from dataclasses import asdict

from .semantic_constraint_core import dubins_choices, dubins_edge_from_parameters
from .semantic_route_phase_v3 import Edge, LazyRoutePhaseSearch

METHOD_ID = "goal_coreachable_forward_dubins_dag_r19_v1"


def edge_objective(edge, policy):
    return (
        edge.length_m + policy.lane_wrong_weight * edge.lane_wrong_m
        + policy.lane_outside_target_weight * edge.lane_outside_m
        + policy.parking_outside_center_weight * edge.parking_outside_m
        + policy.semantic_error_weight * edge.semantic_error_integral
    )


def require_resolved_naturalness(candidate):
    """Do not let constrained station projection silently dismiss a raw alert.

    A nearest-route projection can be ambiguous, so an alert is not proof of
    an unnecessary detour. It is nevertheless unresolved evidence, not a pass.
    An independent justification would need a separately frozen validator.
    """
    raw = candidate.get("naturalness")
    candidate["parent_strict_gate_passed"] = candidate["gate_passed"]
    if not isinstance(raw, dict) or "audit_passed" not in raw:
        failure = "R19_NATURALNESS_AUDIT_MISSING"
    elif not raw["audit_passed"] or raw.get("requires_detour_justification", False):
        failure = "R19_NATURALNESS_REVIEW_REQUIRED"
    else:
        failure = ""
    candidate["r19_detour_review"] = {
        "passed": not failure, "failure_code": failure,
        "independent_detour_justification_provided": False,
        "alert_is_continuous_space_infeasibility_proof": False,
    }
    if failure:
        candidate["gate_passed"] = False
        candidate["failure_codes"] = list(dict.fromkeys([*candidate["failure_codes"], failure]))


class GoalReachableSearchR19(LazyRoutePhaseSearch):
    """Exact weighted suffix on a finite, uncapped-successor state DAG.

Unlike the parent endpoint-first successor cap, a source is discarded only
after all locally admissible edges to goal-coreachable states are checked.
The state set, station skip, edge length/ratio and all edge audits are inherited.
One best suffix per state bounds memory; it does not claim to optimise every
non-additive whole-path acceptance metric. The complete audit decides success.
"""

    def solve(self, semantic_map, *, wall_budget_s=30.0, terminal_attachment=False,
              lower_bound_pruning=True, semantic_cost_prefilter=True):
        started = time.monotonic()
        goal = self.layers[-1][0]
        distance = {goal: 0.0}
        best_edges = {}
        layer_records = []
        status = "COMPLETE"
        try:
            for layer_index in range(len(self.layers) - 2, -1, -1):
                record = {"layer": layer_index, "state_count": len(self.layers[layer_index]),
                          "goal_coreachable_count": 0, "tested_primitive_count": 0}
                checked_before = self.counters["sampled_state_pairs"]
                for source_id in self.layers[layer_index]:
                    if time.monotonic() - started >= wall_budget_s:
                        raise TimeoutError("GRAPH_WALL_BUDGET_EXCEEDED")
                    if self.counters["reverse_expanded_states"] >= self.policy.maximum_expanded_labels:
                        raise TimeoutError("GRAPH_STATE_BUDGET_EXCEEDED")
                    self.counters["reverse_expanded_states"] += 1
                    source = self.states[source_id]
                    specs = []
                    target_layers = list(range(
                        layer_index + 1,
                        min(len(self.layers), layer_index + 1 + self.policy.maximum_station_skip),
                    ))
                    # The endpoint has an exact query yaw rather than an
                    # ordinary sampled guide tangent. Bind its attachment by
                    # the unchanged metric edge-length budget, not a rounding
                    # accident in the number of remaining station samples.
                    if (terminal_attachment and len(self.layers)-1 not in target_layers
                            and self.route.length_m-source.station_m
                            <= self.policy.maximum_local_edge_length_m):
                        target_layers.append(len(self.layers)-1)
                        self.counters["terminal_attachment_sources"] += 1
                    for target_layer in target_layers:
                        for target_id in self.layers[target_layer]:
                            if target_id not in distance:
                                self.counters["unreachable_target_pruned"] += 1
                                continue
                            target = self.states[target_id]
                            direct = math.dist(source.pose[:2], target.pose[:2])
                            if not self.policy.projection_epsilon_m < direct <= self.policy.maximum_local_edge_length_m:
                                self.rejected["EDGE_DISTANCE_BOUND"] += 1
                                continue
                            bound = min(self.policy.maximum_local_edge_length_m,
                                        self.policy.maximum_local_edge_ratio * direct + .05)
                            for choice, (word, params) in enumerate(dubins_choices(
                                source.pose, target.pose, self.policy.turning_radius_m,
                            )):
                                primitive_length = self.policy.turning_radius_m * sum(params)
                                if primitive_length > bound:
                                    self.rejected["DUBINS_LENGTH_BOUND"] += 1
                                    continue
                                specs.append((primitive_length + distance[target_id],
                                              target_id, choice, word, params))
                    best = None
                    for lower, target_id, choice, word, params in sorted(specs):
                        if time.monotonic() - started >= wall_budget_s:
                            raise TimeoutError("GRAPH_WALL_BUDGET_EXCEEDED")
                        # All semantic additions are nonnegative. Include a
                        # conservative roundoff margin; equal-cost edges are
                        # still tested so deterministic tie-breaks are kept.
                        if lower_bound_pruning and best and lower > best[0] + 1e-9:
                            self.counters["objective_lower_bound_pruned"] += 1
                            continue
                        if semantic_cost_prefilter and best:
                            # Evaluate exactly the parent's additive semantic
                            # statistic before expensive projection/footprint
                            # tests. This can only reject a cost-dominated
                            # edge. Every retained edge still calls _edge.
                            preliminary = dubins_edge_from_parameters(
                                source.pose, self.states[target_id].pose,
                                self.policy.turning_radius_m, word, params)
                            stats = self.world.edge_statistics(preliminary.samples)
                            scored = Edge(-1, source_id, target_id, layer_index,
                                          self.states[target_id].layer_index, choice,
                                          float(preliminary.length), *stats, preliminary)
                            bound = edge_objective(scored, self.policy) + distance[target_id]
                            if bound > best[0] + 1e-9:
                                self.counters["semantic_cost_dominated_pruned"] += 1
                                continue
                        self.counters["sampled_state_pairs"] += 1
                        edge = self._edge(source, self.states[target_id], choice, word, params)
                        if edge is None:
                            continue
                        added = edge_objective(edge, self.policy)
                        if not math.isfinite(added) or added < edge.length_m - 1e-9:
                            raise ValueError("INVALID_NONNEGATIVE_EDGE_OBJECTIVE")
                        score = added + distance[target_id]
                        option = (score, edge.target, edge.dubins_choice, edge.edge_id, edge)
                        if best is None or option[:-1] < best[:-1]:
                            best = option
                    if best:
                        score, *_tie, edge = best
                        distance[source_id], best_edges[source_id] = score, edge
                        record["goal_coreachable_count"] += 1
                record["tested_primitive_count"] = self.counters["sampled_state_pairs"] - checked_before
                layer_records.append(record)
        except TimeoutError as error:
            status = str(error)
        diagnostics = {
            **self.layer_diagnostics, "method_id": METHOD_ID, "policy": asdict(self.policy),
            "graph_status": status, "graph_ms": (time.monotonic()-started)*1000.0,
            "goal_coreachable_state_count": len(distance),
            "start_goal_connected": self.layers[0][0] in distance,
            "continuous_space_infeasibility_proof": False,
            "semantic_applicability_inferred_from_failure": False,
            "reverse_layer_records": layer_records,
            "counters": dict(self.counters), "edge_rejection_counts": dict(self.rejected),
            "retained_control_count": len(best_edges),
            "terminal_attachment_enabled": terminal_attachment,
            "objective_lower_bound_pruning": lower_bound_pruning,
            "exact_semantic_cost_prefilter": semantic_cost_prefilter,
        }
        if status != "COMPLETE":
            return None, diagnostics
        node = self.layers[0][0]
        if node not in distance:
            diagnostics["failure_code"] = "NO_ROUTE_IN_BOUND_SE2_GRAPH"
            return None, diagnostics
        controls = []
        while node != goal:
            edge = best_edges[node]
            controls.append(edge.control)
            node = edge.target
        audit_started = time.monotonic()
        candidate = self.world.audit(controls, self.route, semantic_map, self.policy)
        # Preserve the unmodified strict audit. The parent's applicability
        # decoration must never turn a semantic miss into a strict witness.
        require_resolved_naturalness(candidate)
        candidate["planner_method_id"] = METHOD_ID
        candidate["semantic_metric_revision"] = "parking-route-local-aisle-normalized-r3-research"
        diagnostics["audit_ms"] = (time.monotonic()-audit_started)*1000.0
        diagnostics["objective"] = distance[self.layers[0][0]]
        diagnostics["failure_code"] = "" if candidate["gate_passed"] else "STRICT_PATH_AUDIT_FAILED"
        diagnostics["strict_failure_codes"] = list(candidate["failure_codes"])
        return candidate, diagnostics
