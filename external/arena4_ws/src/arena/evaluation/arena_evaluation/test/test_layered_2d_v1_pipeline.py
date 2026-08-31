import numpy as np

from arena_evaluation import topology
from arena_evaluation.dynamic_snapshot import DynamicSnapshot
from arena_evaluation.graph_dstar_lite import GraphDStarLite, GraphEdge
from arena_evaluation.layered_2d_v1_pipeline import (
    ARCHITECTURE_ID,
    IMPLEMENTATION_REVISION,
    Layered2DV1Pipeline,
    build_static_topology_view,
    _demo,
)


class _Map:
    resolution = 1.0
    occupancy = np.zeros((8, 12), dtype=np.int8)

    def world_to_cell(self, x, y):
        return int(round(y)), int(round(x))

    def footprint_collision(self, pose, footprint, unknown_is_collision=True):
        del pose, footprint, unknown_is_collision
        return False


def _artifact():
    nodes = [
        topology.TopologyNode(0, 1.0, 4.0, 4, 1, 2, 1.0, 2.0, 1),
        topology.TopologyNode(1, 5.0, 4.0, 4, 5, 2, 1.0, 2.0, 1),
        topology.TopologyNode(2, 9.0, 4.0, 4, 9, 2, 1.0, 2.0, 1),
    ]
    edges = [
        topology.TopologyEdge(10, 0, 1, 4.0, 1.0, 1.0, 2.0, 2, [[1.0, 4.0], [5.0, 4.0]]),
        topology.TopologyEdge(11, 1, 2, 4.0, 1.0, 1.0, 2.0, 2, [[5.0, 4.0], [9.0, 4.0]]),
    ]
    return topology.TopologyArtifact(
        _Map(), np.ones((8, 12), dtype=bool), np.ones((8, 12), dtype=bool),
        np.ones((8, 12), dtype=np.float32), np.ones((8, 12), dtype=np.int32),
        topology.TopologyGraph(nodes, edges), {"map_sha256": "map", "algorithm": topology.TOPOLOGY_ALGORITHM_VERSION},
    )


def test_architecture_identity_and_no_refinement():
    view = build_static_topology_view(_artifact())
    assert ARCHITECTURE_ID == "2D-V1"
    assert IMPLEMENTATION_REVISION == "r1"
    assert view.metadata["refinement_enabled"] is False
    assert view.metadata["topology_representation"] == "2a_v0_static_skeleton_graph"
    assert view.node_count == 3
    assert view.edge_count == 2
    assert set(view.nodes) == {0, 1, 2}
    assert set(view.edges) == {"topology_10", "topology_11"}


def test_dstar_states_are_original_topology_ids():
    view = build_static_topology_view(_artifact())
    graph = GraphDStarLite(view.nodes, [
        GraphEdge(edge.edge_id, edge.source_node, edge.target_node, edge.length_m, edge.static_cost)
        for edge in view.edges.values()
    ], 0, 2)
    result = graph.compute_shortest_path()
    assert result.no_path is False
    assert graph.extract_path() == [0, 1, 2]
    assert all(node in view.nodes for node in graph.extract_path())


def test_demo_switches_route_after_graph_edge_block():
    result = _demo()
    assert result["architecture_id"] == "2D-V1"
    assert result["topology_refinement_enabled"] is False
    assert result["initial_path"] == [0, 1, 3]
    assert result["updated_path"] == [0, 2, 3]
    assert result["l2_called"] is False


def test_pipeline_identity_does_not_inherit_v0_labels():
    view = build_static_topology_view(_artifact())
    pipeline = Layered2DV1Pipeline(view, footprint=[])
    result = pipeline.plan_initial(type("Query", (), {"start": [1.0, 4.0, 0.0], "goal": [9.0, 4.0, 0.0]})())
    assert result.diagnostics["architecture_id"] == "2D-V1"
    assert result.diagnostics["implementation_revision"] == "r1"
    assert result.diagnostics["topology_refinement_enabled"] is False


def test_virtual_endpoint_edges_are_forward_only():
    view = build_static_topology_view(_artifact())
    pipeline = Layered2DV1Pipeline(view, footprint=[])
    starts = pipeline._attach(type("Query", (), {"start": [1.0, 4.0, 0.0], "goal": [9.0, 4.0, 0.0]})(), DynamicSnapshot.empty(map_shape=view.artifact.free_mask.shape))[0]
    goals = pipeline._attach(type("Query", (), {"start": [1.0, 4.0, 0.0], "goal": [9.0, 4.0, 0.0]})(), DynamicSnapshot.empty(map_shape=view.artifact.free_mask.shape))[1]
    graph, start_root, goal_root, _ = pipeline._make_graph(starts, goals, DynamicSnapshot.empty(map_shape=view.artifact.free_mask.shape))
    assert all(edge.bidirectional for edge in graph.edges.values() if edge.edge_id.startswith("topology_"))
    assert all(not edge.bidirectional for edge in graph.edges.values() if not edge.edge_id.startswith("topology_"))
    assert all(target != start_root for target, _edge, _reverse in graph.adjacency.get(start_root, []))
    assert all(source != goal_root for source, _edge, _reverse in graph._predecessors.get(goal_root, []))


def test_node_attachment_heading_error_uses_incident_tangent():
    view = build_static_topology_view(_artifact())
    pose = [1.0, 4.0, 1.5707963267948966]
    pipeline = Layered2DV1Pipeline(view, footprint=[])
    starts = pipeline._attach(type("Query", (), {"start": pose, "goal": [9.0, 4.0, 0.0]})(), DynamicSnapshot.empty(map_shape=view.artifact.free_mask.shape))[0]
    assert starts
    assert any(candidate.heading_error_rad > 0.1 for candidate in starts)
