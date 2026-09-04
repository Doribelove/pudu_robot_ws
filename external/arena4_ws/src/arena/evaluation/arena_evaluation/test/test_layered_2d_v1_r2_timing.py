from types import SimpleNamespace

import numpy as np

from arena_evaluation import layered_2d_v0_pipeline as v0
from arena_evaluation import layered_2d_v1_pipeline as r1
from arena_evaluation import layered_2d_v1_r2_pipeline as r2
from arena_evaluation import topology
from arena_evaluation.dynamic_snapshot import DynamicSnapshot


class _Map:
    resolution = 1.0
    occupancy = np.zeros((8, 12), dtype=np.int8)

    def world_to_cell(self, x, y):
        cell = int(round(y)), int(round(x))
        if 0 <= cell[0] < self.occupancy.shape[0] and 0 <= cell[1] < self.occupancy.shape[1]:
            return cell
        return None

    def footprint_collision(self, pose, footprint, unknown_is_collision=True):
        del pose, footprint, unknown_is_collision
        return False

    def clearance(self, x, y):
        del x, y
        return 1.0


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
        topology.TopologyGraph(nodes, edges),
        {"map_sha256": "map", "algorithm": topology.TOPOLOGY_ALGORITHM_VERSION},
    )


def _large_artifact():
    hospital_map = _Map()
    hospital_map.occupancy = np.zeros((100, 120), dtype=np.int8)
    hospital_map.height = 100
    hospital_map.width = 120
    nodes = [
        topology.TopologyNode(0, 0.0, 1.0, 1, 0, 1, 1.0, 2.0, 1),
        topology.TopologyNode(1, 8.0, 1.0, 1, 8, 1, 1.0, 2.0, 1),
        topology.TopologyNode(2, 50.0, 50.0, 50, 50, 1, 1.0, 2.0, 2),
        topology.TopologyNode(3, 60.0, 50.0, 50, 60, 1, 1.0, 2.0, 2),
    ]
    edges = [
        topology.TopologyEdge(10, 0, 1, 8.0, 1.0, 1.0, 2.0, 2, [[0.0, 1.0], [8.0, 1.0]]),
        topology.TopologyEdge(11, 2, 3, 10.0, 1.0, 1.0, 2.0, 2, [[50.0, 50.0], [60.0, 50.0]]),
    ]
    return topology.TopologyArtifact(
        hospital_map, np.ones((100, 120), dtype=bool), np.ones((100, 120), dtype=bool),
        np.ones((100, 120), dtype=np.float32), np.ones((100, 120), dtype=np.int32),
        topology.TopologyGraph(nodes, edges),
        {"map_sha256": "large-map", "source_hash": "topology-source", "algorithm": topology.TOPOLOGY_ALGORITHM_VERSION},
    )


def test_stage0_timed_mask_is_bitwise_identical_to_r1():
    refined = r1.build_static_topology_view(_artifact())
    route = [-1000000, -1000100, 0, 1, 2, -2000100, -2000000]
    virtual = {-1000100: (1.0, 4.0), -2000100: (9.0, 4.0)}
    expected = v0.corridor_mask_for_route(
        refined, route, [1.0, 4.0, 0.0], [9.0, 4.0, 0.0],
        padding_m=2.2, virtual_positions=virtual,
    )
    actual, timing = r2._legacy_corridor_mask_timed(
        refined, route, [1.0, 4.0, 0.0], [9.0, 4.0, 0.0],
        padding_m=2.2, virtual_positions=virtual,
    )
    assert np.array_equal(actual, expected)
    assert timing["corridor_mask_rasterization_ms"] >= 0.0
    assert timing["corridor_mask_dilation_ms"] >= 0.0


def test_l1_total_is_sum_of_non_overlapping_leaf_phases():
    refined = r1.build_static_topology_view(_artifact())
    pipeline = r2.Layered2DV1R2Pipeline(refined, footprint=[])
    query = SimpleNamespace(
        query_id="q", start=[1.0, 4.0, 0.0], goal=[9.0, 4.0, 0.0],
    )
    _path, timing = pipeline._plan_route(
        query, DynamicSnapshot.empty(timestamp=0.0, map_shape=(8, 12)),
    )
    leaf_fields = (
        "attachment_lookup_time_ms", "graph_construction_time_ms",
        "dstar_lite_search_time_ms", "route_extraction_time_ms",
        "route_edge_resolution_time_ms",
    )
    assert abs(timing["l1_total_time_ms"] - sum(timing[field] for field in leaf_fields)) < 1.0e-9
    # The aggregate attachment and route-construction values are compatibility
    # fields; adding them again would reproduce the r1 double count.
    assert timing["route_construction_time_ms"] == (
        timing["route_extraction_time_ms"] + timing["route_edge_resolution_time_ms"]
    )


def test_r2_identity_is_independent_from_r1():
    assert r2.ARCHITECTURE_ID == r1.ARCHITECTURE_ID == "2D-V1"
    assert r1.IMPLEMENTATION_REVISION == "r1"
    assert r2.IMPLEMENTATION_REVISION == "r2"


def test_roi_cache_hit_is_bitwise_equivalent_at_map_boundary():
    refined = r1.build_static_topology_view(_large_artifact())
    route = [-1000000, -1000100, 0, 1, -2000100, -2000000]
    virtual = {-1000100: (0.0, 1.0), -2000100: (8.0, 1.0)}
    start = [0.0, 1.0, 0.0]
    goal = [8.0, 1.0, 0.0]
    snapshot = DynamicSnapshot.empty(
        snapshot_id="static", timestamp=0.0, map_version="map-v1", map_shape=(100, 120),
    )
    expected = v0.corridor_mask_for_route(
        refined, route, start, goal, padding_m=2.2, virtual_positions=virtual,
    )
    cache = r2.CorridorMaskCache(
        refined, [], base_map_hash="large-map", topology_cache_key="topology-key",
        topology_source_hash="topology-source", corridor_semantics="raw_map_smac_aligned",
        corridor_profile="padding",
    )
    first, first_diag = cache.get_or_build(
        route, ["topology_10"], start, goal, padding_m=2.0, extra_margin_m=0.2,
        virtual_positions=virtual, snapshot=snapshot,
    )
    second, second_diag = cache.get_or_build(
        route, ["topology_10"], start, goal, padding_m=2.0, extra_margin_m=0.2,
        virtual_positions=virtual, snapshot=snapshot,
    )
    assert np.array_equal(first, expected)
    assert np.array_equal(second, expected)
    assert first_diag["corridor_cache_hit"] is False
    assert second_diag["corridor_cache_hit"] is True
    assert second_diag["corridor_mask_hash"] == first_diag["corridor_mask_hash"]
    assert second_diag["corridor_allowed_cells"] == int(np.count_nonzero(expected))
    assert first_diag["corridor_mask_roi_height_cells"] < expected.shape[0]
    assert first_diag["corridor_mask_roi_width_cells"] < expected.shape[1]
    assert cache.memory_bytes < expected.nbytes


def test_corridor_cache_key_invalidates_all_safety_inputs():
    refined = r1.build_static_topology_view(_large_artifact())
    route = [0, 1]
    start = [0.0, 1.0, 0.0]
    goal = [8.0, 1.0, 0.0]
    first_snapshot = DynamicSnapshot.empty(
        snapshot_id="s1", timestamp=1.0, map_version="v1", map_shape=(100, 120),
    )
    second_snapshot = DynamicSnapshot.from_cells(
        "s2", [(2, 2)], timestamp=2.0, map_version="v2", map_shape=(100, 120),
    )
    cache = r2.CorridorMaskCache(
        refined, [[0.0, 0.0]], base_map_hash="map", topology_cache_key="topology",
        topology_source_hash="source", corridor_semantics="semantics", corridor_profile="profile",
    )
    first_key = cache._key(route, ["topology_10"], start, goal, 2.0, 0.2, first_snapshot)[0]
    assert first_key != cache._key(route, ["topology_10"], start, goal, 3.0, 0.2, first_snapshot)[0]
    assert first_key != cache._key(route, ["topology_10"], start, goal, 2.0, 0.3, first_snapshot)[0]
    assert first_key != cache._key(route, ["topology_10"], start, goal, 2.0, 0.2, second_snapshot)[0]
    different_footprint = r2.CorridorMaskCache(
        refined, [[1.0, 0.0]], base_map_hash="map", topology_cache_key="topology",
        topology_source_hash="source", corridor_semantics="semantics", corridor_profile="profile",
    )
    assert first_key != different_footprint._key(
        route, ["topology_10"], start, goal, 2.0, 0.2, first_snapshot,
    )[0]
    different_context = r2.CorridorMaskCache(
        refined, [[0.0, 0.0]], base_map_hash="other-map",
        topology_cache_key="other-topology", topology_source_hash="other-source",
        corridor_semantics="other-semantics", corridor_profile="other-profile",
    )
    assert first_key != different_context._key(
        route, ["topology_10"], start, goal, 2.0, 0.2, first_snapshot,
    )[0]


def test_dynamic_cache_lookups_reject_expired_snapshots():
    refined = r1.build_static_topology_view(_large_artifact())
    snapshot = DynamicSnapshot.from_cells(
        "expired", [(2, 2)], timestamp=0.0, ttl=0.0,
        map_version="v1", map_shape=(100, 120),
    )
    corridor = r2.CorridorMaskCache(
        refined, [], base_map_hash="map", topology_cache_key="topology",
        topology_source_hash="source", corridor_semantics="semantics",
        corridor_profile="profile",
    )
    with np.testing.assert_raises_regex(ValueError, "expired dynamic snapshot"):
        corridor.get_or_build(
            [0, 1], ["topology_10"], [0.0, 1.0, 0.0], [8.0, 1.0, 0.0],
            padding_m=2.0, extra_margin_m=0.2, virtual_positions={},
            snapshot=snapshot,
        )
    pipeline = r2.Layered2DV1R2Pipeline(refined, footprint=[])
    with np.testing.assert_raises_regex(ValueError, "expired dynamic snapshot"):
        pipeline._optimized_attachment_candidates([0.0, 1.0, 0.0], snapshot)


def test_spatial_attachment_and_cache_preserve_r1_candidates():
    refined = r1.build_static_topology_view(_large_artifact())
    pose = [0.0, 1.0, 0.0]
    snapshot = DynamicSnapshot.empty(
        snapshot_id="static", timestamp=0.0, map_version="v1", map_shape=(100, 120),
    )
    expected = v0.attachment_candidates(
        refined, pose, [], radius_m=8.0, limit=16, snapshot=snapshot,
    )
    expected = [r1.Layered2DV1Pipeline._nearest_edge_endpoint(item, refined) for item in expected]
    expected.sort(key=lambda candidate: (
        0 if candidate.role == "original" else 1,
        candidate.distance_m, candidate.heading_error_rad, candidate.candidate_id,
    ))
    pipeline = r2.Layered2DV1R2Pipeline(refined, footprint=[])
    first, first_timing = pipeline._optimized_attachment_candidates(pose, snapshot)
    second, second_timing = pipeline._optimized_attachment_candidates(pose, snapshot)
    assert [r2._serialize_candidate(item) for item in first] == [
        r2._serialize_candidate(item) for item in expected
    ]
    assert [r2._serialize_candidate(item) for item in second] == [
        r2._serialize_candidate(item) for item in expected
    ]
    assert first_timing["endpoint_cache_hit"] is False
    assert second_timing["endpoint_cache_hit"] is True
    assert first_timing["edge_projection_segments_scanned"] < first_timing[
        "edge_projection_segments_total"
    ]


def test_endpoint_cache_invalidates_dynamic_snapshot_exactly():
    refined = r1.build_static_topology_view(_large_artifact())
    pipeline = r2.Layered2DV1R2Pipeline(refined, footprint=[])
    pose = [0.0, 1.0, 0.0]
    first = DynamicSnapshot.empty(
        snapshot_id="one", timestamp=1.0, map_version="v1", map_shape=(100, 120),
    )
    second = DynamicSnapshot.from_cells(
        "two", [(1, 1)], timestamp=2.0, map_version="v2", map_shape=(100, 120),
    )
    pipeline._optimized_attachment_candidates(pose, first)
    _candidates, timing = pipeline._optimized_attachment_candidates(pose, second)
    assert timing["endpoint_cache_hit"] is False
    assert pipeline.endpoint_cache_misses == 2


def test_endpoint_cache_key_changes_with_topology_and_footprint_context():
    refined = r1.build_static_topology_view(_large_artifact())
    snapshot = DynamicSnapshot.empty(
        snapshot_id="static", timestamp=0.0, map_version="v1", map_shape=(100, 120),
    )
    first = r2.Layered2DV1R2Pipeline(
        refined, footprint=[[0.0, 0.0]], topology_cache_key="topology-a",
        topology_source_hash="source-a",
    )
    second = r2.Layered2DV1R2Pipeline(
        refined, footprint=[[1.0, 0.0]], topology_cache_key="topology-b",
        topology_source_hash="source-b",
    )
    assert first._endpoint_cache_key([0.0, 1.0, 0.0], snapshot) != second._endpoint_cache_key(
        [0.0, 1.0, 0.0], snapshot,
    )
