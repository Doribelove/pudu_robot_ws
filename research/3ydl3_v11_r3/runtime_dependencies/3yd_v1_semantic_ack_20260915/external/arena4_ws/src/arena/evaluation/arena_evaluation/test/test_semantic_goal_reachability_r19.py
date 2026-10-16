from dataclasses import replace
from types import SimpleNamespace

import pytest

from arena_evaluation.semantic_goal_reachability_r19 import GoalReachableSearchR19
from arena_evaluation.semantic_route_phase_v3 import RoutePhasePolicy
from arena_evaluation.two_layer_v3_semantic_r19 import load


def fake_search(monkeypatch, edges, layers, *, audit_pass=True, policy=None):
    from collections import Counter
    from arena_evaluation import semantic_goal_reachability_r19 as module
    search = object.__new__(GoalReachableSearchR19)
    search.policy = policy or RoutePhasePolicy(maximum_station_skip=2)
    search.layers = layers
    search.states = {}
    for layer, nodes in enumerate(layers):
        for node in nodes:
            search.states[node] = SimpleNamespace(
                state_id=node, station_m=layer*.5, layer_index=layer,
                pose=(layer * .5, node * .01, 0.0))
    search.counters, search.rejected, search.layer_diagnostics = Counter(), Counter(), {}
    search.route = SimpleNamespace(length_m=(len(layers)-1)*.5)
    search.world = SimpleNamespace(audit=lambda *_args: {
        "gate_passed": audit_pass,
        "failure_codes": [] if audit_pass else ["R2_SEMANTIC_GATE"],
        "controls": _args[0],
        "naturalness": {"audit_passed": True, "requires_detour_justification": False},
    })
    # Unit graph oracle uses abstract edge weights, not raster statistics.
    original_solve = search.solve
    search.solve = lambda *args, **kwargs: original_solve(
        *args, **{"semantic_cost_prefilter": False, **kwargs})
    monkeypatch.setattr(module, "dubins_choices", lambda *_args: [("SSS", (.5, 0., 0.))])

    def edge(source, target, *_args):
        pair = source.state_id, target.state_id
        if pair not in edges:
            return None
        return SimpleNamespace(
            length_m=edges[pair], lane_wrong_m=0., lane_outside_m=0.,
            parking_outside_m=0., semantic_error_integral=0., target=pair[1],
            dubins_choice=0, edge_id=pair[0]*100+pair[1], control=pair)
    search._edge = edge
    return search


def test_backward_pruning_keeps_a_more_expensive_route_around_dead_end(monkeypatch):
    # Forward greedy picks 0->1, but 1 is a dead end. Reverse DP must choose 2.
    search = fake_search(monkeypatch, {(0, 1): .1, (0, 2): 1., (2, 3): .5},
                         [[0], [1, 2], [3]])
    candidate, result = search.solve(None)
    assert candidate["gate_passed"]
    assert result["start_goal_connected"]
    assert result["objective"] == pytest.approx(1.5)
    assert result["goal_coreachable_state_count"] == 3


def test_two_station_skip_includes_second_layer(monkeypatch):
    search = fake_search(monkeypatch, {(0, 2): 1.}, [[0], [1], [2]])
    candidate, result = search.solve(None)
    assert candidate is not None
    assert result["start_goal_connected"]


def test_graph_miss_is_not_semantic_inapplicability(monkeypatch):
    search = fake_search(monkeypatch, {}, [[0], [1]])
    candidate, result = search.solve(None)
    assert candidate is None
    assert result["failure_code"] == "NO_ROUTE_IN_BOUND_SE2_GRAPH"
    assert result["continuous_space_infeasibility_proof"] is False
    assert result["semantic_applicability_inferred_from_failure"] is False


def test_audit_semantic_failure_cannot_be_decorated_as_success(monkeypatch):
    search = fake_search(monkeypatch, {(0, 1): .5}, [[0], [1]], audit_pass=False)
    candidate, result = search.solve(None)
    assert candidate["gate_passed"] is False
    assert result["failure_code"] == "STRICT_PATH_AUDIT_FAILED"


@pytest.mark.parametrize("raw,code", [
    (None, "R19_NATURALNESS_AUDIT_MISSING"),
    ({"audit_passed": False, "requires_detour_justification": True}, "R19_NATURALNESS_REVIEW_REQUIRED"),
    ({"audit_passed": True, "requires_detour_justification": True}, "R19_NATURALNESS_REVIEW_REQUIRED"),
])
def test_ordered_progress_cannot_silently_clear_raw_detour_alert(raw, code):
    from arena_evaluation.semantic_goal_reachability_r19 import require_resolved_naturalness
    candidate = {"gate_passed": True, "failure_codes": [], "naturalness": raw,
                 "ordered_progress": {"ordered_progress_gate_passed": True}}
    require_resolved_naturalness(candidate)
    assert candidate["parent_strict_gate_passed"] is True
    assert candidate["gate_passed"] is False
    assert code in candidate["failure_codes"]
    assert candidate["r19_detour_review"]["alert_is_continuous_space_infeasibility_proof"] is False


def test_resource_limit_returns_no_partial_result(monkeypatch):
    policy = replace(RoutePhasePolicy(), maximum_expanded_labels=1)
    search = fake_search(monkeypatch, {(0, 1): .5, (1, 2): .5},
                         [[0], [1], [2]], policy=policy)
    candidate, result = search.solve(None)
    assert candidate is None
    assert result["graph_status"] == "GRAPH_STATE_BUDGET_EXCEEDED"


def test_equal_cost_graph_has_deterministic_result(monkeypatch):
    graph = {(0, 1): 1., (0, 2): 1., (1, 3): .5, (2, 3): .5}
    one = fake_search(monkeypatch, graph, [[0], [1, 2], [3]]).solve(None)[1]
    two = fake_search(monkeypatch, graph, [[0], [1, 2], [3]]).solve(None)[1]
    assert one["objective"] == two["objective"]
    assert one["counters"] == two["counters"]


def test_parent_bindings_and_existing_r17_bounds_are_retained():
    config, parents = load()
    assert config["maximum_expanded_states"] == 12000
    assert parents[0]["se2_local_connection"]["maximum_local_edge_length_m"] == 1.75
    assert parents[0]["frozen_bindings"]["minimum_turning_radius_m"] == .40


def test_terminal_attachment_uses_metric_window_without_moving_pose(monkeypatch):
    search = fake_search(monkeypatch, {(0, 3): 1.5}, [[0], [1], [2], [3]])
    search.route.length_m = 1.5
    before = search.states[3].pose
    candidate, diagnostics = search.solve(None, terminal_attachment=True)
    assert candidate["gate_passed"]
    assert search.states[3].pose == before
    assert diagnostics["counters"]["terminal_attachment_sources"] == 1


def test_terminal_attachment_does_not_expand_past_metric_window(monkeypatch):
    policy = RoutePhasePolicy(maximum_station_skip=2, maximum_local_edge_length_m=1.75)
    search = fake_search(monkeypatch, {(0, 4): 2.0}, [[0], [1], [2], [3], [4]], policy=policy)
    candidate, diagnostics = search.solve(None, terminal_attachment=True)
    assert candidate is None
    assert diagnostics["start_goal_connected"] is False


def test_branch_bound_matches_exhaustive_graph_on_random_dags(monkeypatch):
    import random
    rng = random.Random(1909)
    layers = [[0], [1, 2, 3], [4, 5, 6], [7, 8], [9]]
    for _ in range(40):
        graph = {}
        for index, nodes in enumerate(layers[:-1]):
            for later in layers[index+1:index+3]:
                for source in nodes:
                    for target in later:
                        if rng.random() < .6:
                            # Fake Dubins length lower bound is .2005 m.
                            graph[source, target] = rng.uniform(.21, 4.)
        one, a = fake_search(monkeypatch, graph, layers).solve(None, lower_bound_pruning=True)
        two, b = fake_search(monkeypatch, graph, layers).solve(None, lower_bound_pruning=False)
        assert a["start_goal_connected"] == b["start_goal_connected"]
        if one is not None:
            assert a["objective"] == pytest.approx(b["objective"])
            assert one["controls"] == two["controls"]


def test_semantic_prefilter_matches_full_edge_evaluation(monkeypatch):
    import numpy as np
    from arena_evaluation import semantic_constraint_core as core
    from arena_evaluation import semantic_goal_reachability_r19 as module
    from arena_evaluation.semantic_route_phase_v3 import Edge

    def build():
        search = fake_search(monkeypatch, {}, [[0], [1, 2, 3], [4, 5], [6]])
        monkeypatch.setattr(module, "dubins_choices", core.dubins_choices)
        def statistics(samples):
            n = len(samples)
            target = int(np.count_nonzero(samples[:, 1] < .035))
            return n, target, target, 0, 0, float(np.abs(samples[:, 1]).sum()*.025)
        search.world.edge_statistics = statistics
        def edge(source, target, choice, word, params):
            if (source.state_id, target.state_id) == (0, 1):
                return None
            control = core.dubins_edge_from_parameters(
                source.pose, target.pose, search.policy.turning_radius_m, word, params)
            return Edge(source.state_id*100+target.state_id, source.state_id, target.state_id,
                        source.layer_index, target.layer_index, choice, control.length,
                        *statistics(control.samples), (source.state_id,target.state_id,choice))
        search._edge = edge
        return search
    candidate_a, a = build().solve(None, semantic_cost_prefilter=True)
    candidate_b, b = build().solve(None, semantic_cost_prefilter=False)
    assert a["start_goal_connected"] == b["start_goal_connected"]
    assert a["objective"] == pytest.approx(b["objective"])
    assert candidate_a["controls"] == candidate_b["controls"]
