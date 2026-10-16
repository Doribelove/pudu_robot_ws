from collections import Counter
from types import SimpleNamespace

from arena_evaluation.semantic_route_phase_r18 import (
    METHOD_ID,
    AdjacentFirstGapBridgeSearchR18,
)


def _edge(layer: int, target: int):
    return SimpleNamespace(
        target_layer=layer,
        target=target,
        length_m=0.5,
        dubins_choice=0,
    )


def _search(phases, responses):
    search = object.__new__(AdjacentFirstGapBridgeSearchR18)
    search.states = [
        SimpleNamespace(
            state_id=index,
            layer_index=index,
            phase_kind=kind,
            phase_instance=instance,
        )
        for index, (kind, instance) in enumerate(phases)
    ]
    search.layers = [[index] for index in range(len(phases))]
    search.cache = {}
    search.counters = Counter()
    search.rejected = Counter()
    search._edges_to_layer = lambda _source, layer: list(responses.get(layer, ()))
    return search


def test_same_phase_with_adjacent_edge_does_not_bridge():
    search = _search(
        [("parking", 1), ("parking", 1), ("parking", 1)],
        {1: [_edge(1, 1)], 2: [_edge(2, 2)]},
    )
    result = search.successors(0)
    assert [edge.target_layer for edge in result] == [1]
    assert search.counters["semantic_boundary_bridge_attempts"] == 0


def test_semantic_boundary_may_add_one_bounded_bridge():
    search = _search(
        [("parking", 1), ("lane", 2), ("lane", 2)],
        {1: [_edge(1, 1)], 2: [_edge(2, 2)]},
    )
    result = search.successors(0)
    assert [edge.target_layer for edge in result] == [1, 2]
    assert search.counters["semantic_boundary_bridge_attempts"] == 1
    assert search.counters["semantic_boundary_bridge_successes"] == 1


def test_empty_adjacent_layer_bridges_once_even_inside_same_phase():
    search = _search(
        [("lane", 3), ("lane", 3), ("lane", 3)],
        {1: [], 2: [_edge(2, 2)]},
    )
    result = search.successors(0)
    assert [edge.target_layer for edge in result] == [2]
    assert search.counters["empty_adjacent_bridge_attempts"] == 1
    assert search.counters["empty_adjacent_bridge_successes"] == 1


def test_r18_method_name_records_rejected_diagnostic_mechanism():
    assert METHOD_ID == "adjacent_first_semantic_boundary_gap_bridge_r18_v2"
