from __future__ import annotations

from types import SimpleNamespace

import pytest

from arena_evaluation import two_layer_pipeline_visualize as visualize
from arena_evaluation.topology import TopologyEdge, TopologyGraph, TopologyNode


def _artifact_with_route():
    nodes = [
        TopologyNode(1, 0.0, 0.0, 0, 0, 2, 1.0, 2.0, 0),
        TopologyNode(2, 1.0, 0.0, 1, 0, 2, 1.0, 2.0, 0),
        TopologyNode(3, 2.0, 1.0, 2, 1, 1, 0.8, 1.6, 0),
    ]
    edges = [
        TopologyEdge(10, 1, 2, 1.0, 1.0, 1.0, 2.0, 2, [[0.0, 0.0], [1.0, 0.0]]),
        TopologyEdge(11, 2, 3, 1.414, 0.8, 0.9, 1.6, 2, [[1.0, 0.0], [2.0, 1.0]]),
    ]
    return SimpleNamespace(graph=TopologyGraph(nodes=nodes, edges=edges))


def test_parser_requires_static_map_flag():
    with pytest.raises(SystemExit):
        visualize.main([])


def test_frozen_architecture_identity_and_l2_disabled():
    assert visualize.ARCHITECTURE_ID == "2A-V0"
    assert visualize.IMPLEMENTATION_REVISION == "r3"
    assert visualize.L2_STATUS == "disabled"


def test_route_reconstruction_uses_recorded_node_and_edge_order():
    artifact = _artifact_with_route()
    route = visualize._route_from_row(
        artifact,
        {"topology_node_ids": "[1, 2, 3]", "topology_edge_ids": "[10, 11]"},
    )
    assert route.node_ids == [1, 2, 3]
    assert route.edge_ids == [10, 11]
    assert route.polyline == [[0.0, 0.0], [1.0, 0.0], [2.0, 1.0]]
    assert route.min_width_m == 1.6


def test_route_reconstruction_handles_reversed_recorded_edge():
    artifact = _artifact_with_route()
    route = visualize._route_from_row(
        artifact,
        {"topology_node_ids": "[3, 2, 1]", "topology_edge_ids": "[11, 10]"},
    )
    assert route.polyline == [[2.0, 1.0], [1.0, 0.0], [0.0, 0.0]]
