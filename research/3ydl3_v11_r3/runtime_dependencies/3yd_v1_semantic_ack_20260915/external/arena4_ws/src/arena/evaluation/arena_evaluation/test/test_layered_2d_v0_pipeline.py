from types import SimpleNamespace
from pathlib import Path
import os
import subprocess
import sys

import numpy as np

from arena_evaluation.dynamic_snapshot import DynamicSnapshot
from arena_evaluation.graph_dstar_lite import GraphDStarLite, GraphEdge
from arena_evaluation.layered_2d_v0_pipeline import (
    ARCHITECTURE_ID,
    IMPLEMENTATION_REVISION,
    Layered2DV0Pipeline,
    RefinedNodeSpatialIndex,
    build_refined_topology,
    corridor_mask_for_route,
    prepare_refined_topology,
    _classify_smac_failure,
)
from arena_evaluation.topology import TopologyArtifact, TopologyEdge, TopologyGraph, TopologyNode


class FakeMap:
    map_id = "2d-test"
    resolution = 1.0
    width = 30
    height = 20
    occupancy = np.zeros((height, width), dtype=np.int8)

    def world_to_cell(self, x, y):
        row, col = int(round(float(y))), int(round(float(x)))
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None

    def footprint_collision(self, *args, **kwargs):
        del args, kwargs
        return False


def _artifact():
    nodes = [
        TopologyNode(0, 2.0, 10.0, 10, 2, 2, 2.0, 4.0, 1),
        TopologyNode(1, 12.0, 10.0, 10, 12, 3, 2.0, 4.0, 1),
        TopologyNode(2, 22.0, 10.0, 10, 22, 2, 2.0, 4.0, 1),
        TopologyNode(3, 12.0, 4.0, 4, 12, 2, 2.0, 4.0, 1),
    ]
    edges = [
        TopologyEdge(0, 0, 1, 10.0, 2.0, 2.0, 4.0, 2, [[2.0, 10.0], [12.0, 10.0]]),
        TopologyEdge(1, 1, 2, 10.0, 2.0, 2.0, 4.0, 2, [[12.0, 10.0], [22.0, 10.0]]),
        TopologyEdge(2, 1, 3, 6.0, 2.0, 2.0, 4.0, 2, [[12.0, 10.0], [12.0, 4.0]]),
    ]
    free = np.ones((20, 30), dtype=bool)
    return TopologyArtifact(
        FakeMap(), free, free.copy(), np.ones_like(free, dtype=np.float32),
        np.ones_like(free, dtype=np.int32), TopologyGraph(nodes, edges),
        {"map_sha256": "map-hash", "algorithm": "skeleton_distance_transform_v1", "skeleton_backend": "numpy_zhang_suen"},
    )


def test_refinement_preserves_topology_and_is_not_a_grid_graph():
    refined = build_refined_topology(_artifact(), spacing_m=4.0)
    assert refined.node_count < refined.artifact.free_mask.size
    assert refined.edge_count >= 3
    assert all(edge.corridor_mask_id for edge in refined.edges.values())
    assert set(refined.adjacency) == set(refined.nodes)


def test_refined_edge_and_corridor_keep_source_bend_geometry():
    artifact = _artifact()
    curved = TopologyEdge(
        9, 0, 2, 20.0, 2.0, 2.0, 4.0, 2,
        [[2.0, 10.0], [6.0, 10.0], [10.0, 8.0], [16.0, 10.0], [22.0, 10.0]],
    )
    artifact.graph = TopologyGraph(list(artifact.graph.nodes), list(artifact.graph.edges) + [curved])
    refined = build_refined_topology(artifact, spacing_m=20.0)
    curved_edges = [edge for edge in refined.edges.values() if edge.source_edge_id == 9]
    assert curved_edges
    assert any(len(edge.polyline) > 2 for edge in curved_edges)
    mask = corridor_mask_for_route(refined, [0, 4, 5, 2], [2.0, 10.0, 0.0], [22.0, 10.0, 0.0], padding_m=0.25)
    # The bent route occupies cells away from the straight chord.
    bend_cell = artifact.hospital_map.world_to_cell(10.0, 8.0)
    assert bend_cell is not None and bool(mask[bend_cell])


def test_graph_dstar_lite_uses_topology_nodes_and_repairs_changed_edge():
    graph = GraphDStarLite(
        nodes=[0, 1, 2, 3],
        edges=[
            GraphEdge("direct-a", 0, 1, 1.0), GraphEdge("direct-b", 1, 3, 1.0),
            GraphEdge("detour-a", 0, 2, 1.1), GraphEdge("detour-b", 2, 3, 1.1),
        ], start=0, goal=3,
    )
    initial = graph.compute_shortest_path()
    assert graph.extract_path() == [0, 1, 3]
    graph.update_edges(["direct-a", "direct-b"], statuses={"direct-a": GraphDStarLite.BLOCKED, "direct-b": GraphDStarLite.BLOCKED})
    repaired = graph.compute_shortest_path()
    assert graph.extract_path() == [0, 2, 3]
    assert repaired.expanded_nodes >= 0
    snapshot = graph.state_snapshot()
    assert snapshot["start_node"] == 0
    assert snapshot["goal_node"] == 3
    assert "g" in snapshot and "rhs" in snapshot and "OPEN" in snapshot


def test_pipeline_l2_is_closed_and_l3_receives_corridor_mask():
    refined = build_refined_topology(_artifact(), spacing_m=4.0)

    class FakeL3:
        def __init__(self):
            self.calls = 0
            self.masks = []

        def plan(self, query, corridor_mask, snapshot, *, topology_artifact):
            self.calls += 1
            self.masks.append(np.asarray(corridor_mask).copy())
            return SimpleNamespace(
                planner_success=True,
                points=[
                    {"x": query.start[0], "y": query.start[1], "yaw": query.start[2]},
                    {"x": query.goal[0], "y": query.goal[1], "yaw": query.goal[2]},
                ],
                failure_code="",
                diagnostics={"planner_search_started": True},
            ), {"planner_search_started": True}

    l3 = FakeL3()
    query = SimpleNamespace(start=[2.0, 10.0, 0.0], goal=[22.0, 10.0, 0.0])
    pipeline = Layered2DV0Pipeline(refined, footprint=[[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]], l3_planner=l3)
    result = pipeline.plan_initial(query)
    assert result.success
    assert result.diagnostics["l2_called"] is False
    assert result.diagnostics["l2_call_count"] == 0
    assert l3.calls == 1
    assert l3.masks[0].shape == refined.artifact.free_mask.shape
    assert result.diagnostics["topology_edge_ids"]


def test_corridor_failure_uses_bounded_profile_fallback_without_l2():
    refined = build_refined_topology(_artifact(), spacing_m=4.0)

    class RetryL3:
        def __init__(self):
            self.calls = []

        def plan(self, query, corridor_mask, snapshot, *, topology_artifact):
            del snapshot, topology_artifact
            self.calls.append(int(np.count_nonzero(corridor_mask)))
            if len(self.calls) < 2:
                return SimpleNamespace(
                    planner_success=False, points=None, failure_code="NO_PATH_IN_CORRIDOR",
                    diagnostics={"planner_search_started": True},
                ), {"failure_code": "NO_PATH_IN_CORRIDOR", "planner_search_started": True}
            return SimpleNamespace(
                planner_success=True,
                points=[
                    {"x": query.start[0], "y": query.start[1], "yaw": query.start[2]},
                    {"x": query.goal[0], "y": query.goal[1], "yaw": query.goal[2]},
                ], failure_code="", diagnostics={"planner_search_started": True},
            ), {"planner_search_started": True}

    l3 = RetryL3()
    query = SimpleNamespace(start=[2.0, 10.0, 0.0], goal=[22.0, 10.0, 0.0])
    pipeline = Layered2DV0Pipeline(
        refined,
        footprint=[[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]],
        l3_planner=l3,
        corridor_fallback_policy="bounded",
    )
    result = pipeline.plan_initial(query)
    assert result.success
    assert len(l3.calls) == 2
    assert result.diagnostics["corridor_fallback_used"] is True
    assert result.diagnostics["corridor_retry_paddings_m"] == [4.0]
    assert result.diagnostics["l2_call_count"] == 0


def test_corridor_mask_includes_virtual_endpoint_connection_segments():
    refined = build_refined_topology(_artifact(), spacing_m=4.0)
    mask_without = corridor_mask_for_route(
        refined, [-1000000, -110, 0, 1, -2100, -2000000],
        [2.0, 10.0, 0.0], [22.0, 10.0, 0.0], padding_m=0.25,
    )
    mask_with = corridor_mask_for_route(
        refined, [-1000000, -110, 0, 1, -2100, -2000000],
        [2.0, 10.0, 0.0], [22.0, 10.0, 0.0], padding_m=0.25,
        virtual_positions={-110: (4.0, 9.0), -2100: (20.0, 11.0)},
    )
    assert int(np.count_nonzero(mask_with)) >= int(np.count_nonzero(mask_without))


def test_dynamic_update_reuses_dstar_graph_and_replans():
    refined = build_refined_topology(_artifact(), spacing_m=4.0)

    class FakeL3:
        def __init__(self):
            self.calls = 0

        def plan(self, query, corridor_mask, snapshot, *, topology_artifact):
            del corridor_mask, snapshot, topology_artifact
            self.calls += 1
            return SimpleNamespace(
                planner_success=True,
                points=[
                    {"x": query.start[0], "y": query.start[1], "yaw": query.start[2]},
                    {"x": query.goal[0], "y": query.goal[1], "yaw": query.goal[2]},
                ],
                failure_code="",
                diagnostics={"planner_search_started": True},
            ), {"planner_search_started": True}

    l3 = FakeL3()
    query = SimpleNamespace(start=[2.0, 10.0, 0.0], goal=[22.0, 10.0, 0.0])
    pipeline = Layered2DV0Pipeline(refined, footprint=[[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]], l3_planner=l3)
    initial = pipeline.plan_initial(query)
    assert initial.success
    dstar_before = pipeline.dstar
    updated = pipeline.update_dynamic(query, DynamicSnapshot.from_cells("snap-2", [(10, 12)], map_shape=(20, 30)))
    assert updated.diagnostics["dynamic_replan_triggered"] is True
    assert pipeline.dstar is dstar_before
    assert l3.calls >= 2
    assert updated.diagnostics["l2_call_count"] == 0


def test_dynamic_snapshot_updates_overlay_without_mutating_static_map():
    refined = build_refined_topology(_artifact(), spacing_m=4.0)
    class FakeL3:
        def plan(self, query, corridor_mask, snapshot, *, topology_artifact):
            return SimpleNamespace(planner_success=True, points=None, failure_code="", diagnostics={}), {"planner_search_started": True}
    query = SimpleNamespace(start=[2.0, 10.0, 0.0], goal=[22.0, 10.0, 0.0])
    static_before = refined.artifact.hospital_map.occupancy.copy()
    pipeline = Layered2DV0Pipeline(refined, footprint=[[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]], l3_planner=FakeL3())
    pipeline.plan_initial(query)
    update = pipeline.update_dynamic(query, DynamicSnapshot.from_cells("snap-1", [(10, 12)], map_shape=(20, 30)))
    assert update.diagnostics["l2_called"] is False
    assert update.diagnostics["changed_edge_count"] >= 1
    assert np.array_equal(refined.artifact.hospital_map.occupancy, static_before)


def test_refined_cache_metadata_rejects_stale_parameters(tmp_path):
    refined = build_refined_topology(_artifact(), spacing_m=4.0)
    loaded, info = prepare_refined_topology(_artifact(), [[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]], tmp_path, spacing_m=4.0, planner_parameter_profile="test")
    assert loaded.node_count == refined.node_count
    assert info["topology_cache_hit"] is False
    assert info["cache_state"] == "cache_rebuild"
    loaded_again, info_again = prepare_refined_topology(_artifact(), [[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]], tmp_path, spacing_m=4.0, planner_parameter_profile="test")
    assert loaded_again.node_count == loaded.node_count
    assert info_again["topology_cache_hit"] is True
    assert info_again["cache_state"] == "cache_hit"
    assert info_again["skeleton_backend"] == "numpy_zhang_suen"
    assert isinstance(loaded_again.attachment_index, RefinedNodeSpatialIndex)
    assert loaded_again.attachment_index.query(2.0, 10.0, 2.0)
    assert ARCHITECTURE_ID == "2D-V0"
    assert IMPLEMENTATION_REVISION == "r4"


def test_refined_cache_invalidation_records_parameter_change(tmp_path):
    artifact = _artifact()
    footprint = [[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]]
    _refined, first = prepare_refined_topology(artifact, footprint, tmp_path, spacing_m=4.0, planner_parameter_profile="test")
    assert first["cache_miss"] is True
    _refined, changed = prepare_refined_topology(artifact, footprint, tmp_path, spacing_m=3.0, planner_parameter_profile="test")
    assert changed["topology_cache_hit"] is False
    assert changed["cache_invalidated"] is True
    assert changed["cache_rebuild"] is True


def test_refined_cache_partial_payload_is_not_a_hit(tmp_path):
    artifact = _artifact()
    footprint = [[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]]
    refined, first = prepare_refined_topology(artifact, footprint, tmp_path, spacing_m=4.0, planner_parameter_profile="test")
    cache_dir = Path(first["refined_topology_cache_directory"])
    (cache_dir / "refined_topology.json").unlink()
    _refined, second = prepare_refined_topology(artifact, footprint, tmp_path, spacing_m=4.0, planner_parameter_profile="test")
    assert second["topology_cache_hit"] is False
    assert second["cache_miss_reason"] == "metadata_mismatch"


def test_refined_cache_is_hit_by_a_second_process(tmp_path):
    artifact = _artifact()
    footprint = [[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]]
    _refined, first = prepare_refined_topology(artifact, footprint, tmp_path, spacing_m=4.0, planner_parameter_profile="process")
    assert first["cache_state"] == "cache_rebuild"
    test_dir = str(Path(__file__).resolve().parent)
    package_root = str(Path(__file__).resolve().parents[1])
    code = (
        "from test_layered_2d_v0_pipeline import _artifact; "
        "from arena_evaluation.layered_2d_v0_pipeline import prepare_refined_topology; "
        f"a=_artifact(); f=[[0.2,0.2],[0.2,-0.2],[-0.2,-0.2],[-0.2,0.2]]; "
        f"_,i=prepare_refined_topology(a,f,{str(tmp_path)!r},spacing_m=4.0,planner_parameter_profile='process'); "
        "print(i['cache_state'], i['topology_cache_hit'])"
    )
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([package_root, test_dir, env.get("PYTHONPATH", "")])
    output = subprocess.check_output([sys.executable, "-c", code], cwd=test_dir, env=env, text=True)
    assert output.strip() == "cache_hit True"


def test_smac_failure_classifier_preserves_search_start_semantics():
    assert _classify_smac_failure("ACTION_ABORTED", {}, "Starting point in lethal space")[:2] == ("START_IN_LETHAL_SPACE", False)
    assert _classify_smac_failure("ACTION_ABORTED", {}, "failed to create plan, no valid path found")[:2] == ("NO_PATH_IN_CORRIDOR", True)
    assert _classify_smac_failure("ACTION_ABORTED", {}, "failed to create plan, exceeded maximum iterations")[:2] == ("SMAC_MAX_ITERATIONS", True)
    assert _classify_smac_failure("CLIENT_TIMEOUT", {}, "")[:2] == ("PLANNER_TIMEOUT", "not_available")
    assert _classify_smac_failure("SERVER_UNAVAILABLE", {}, "")[:2] == ("BACKEND_UNAVAILABLE", False)
    assert _classify_smac_failure("ACTION_ABORTED", {}, "unexplained abort")[:2] == ("ACTION_ABORTED_UNKNOWN", "not_available")
