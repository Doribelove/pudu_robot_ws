from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy import ndimage

from arena_evaluation.edge_semantic_annotator import (
    EdgeSemanticAnnotation, EdgeSemanticAnnotator, SemanticEdgeRouter,
    semantic_edge_cache_key, topology_graph_hash,
)
from arena_evaluation.pdmap_semantic_converter import convert_payload
from arena_evaluation.planner_benchmark.map_utils import HospitalMap
from arena_evaluation.regional_preference import RegionalPreferenceBuilder
from arena_evaluation.semantic_costmap_composer import (
    LETHAL_OBSTACLE, SemanticCostmapAckVerifier, SemanticCostmapComposer,
    internal_soft_to_occupancy, occupancy_to_static_layer,
)
from arena_evaluation.semantic_map import SemanticFeature, SemanticMapV1
from arena_evaluation.semantic_path_audit import SemanticPathAuditor
from arena_evaluation.semantic_rasterizer import CLASS_CODES, SemanticRasterizer
from arena_evaluation.semantic_relaxation import PreferenceRelaxationController
from arena_evaluation.topology import TopologyEdge, TopologyGraph, TopologyNode


FOOTPRINT = [[0.255, 0.215], [0.255, -0.215], [-0.255, -0.215], [-0.255, 0.215]]


def fixture():
    height, width, resolution = 100, 180, 0.05
    occupancy = np.zeros((height, width), dtype=np.int8)
    occupancy[[0, -1], :] = 100
    occupancy[:, [0, -1]] = 100
    occupancy[50:53, 20:23] = 100
    distance = ndimage.distance_transform_edt(occupancy == 0, sampling=resolution).astype(np.float32)
    hospital_map = HospitalMap(
        Path("synthetic.yaml"), Path("synthetic.pgm"), resolution,
        (0.0, 0.0, 0.0), width, height, occupancy, distance,
    )
    data = (
        ("lane", "lane", [[0.5, 1.0], [8.4, 1.0], [8.4, 4.0], [0.5, 4.0], [0.5, 1.0]], False, True, False, 60),
        ("junction", "junction_area", [[3.8, 1.0], [4.7, 1.0], [4.7, 4.0], [3.8, 4.0], [3.8, 1.0]], False, True, False, 80),
        ("parking", "parking_area", [[7.0, 1.2], [8.3, 1.2], [8.3, 3.8], [7.0, 3.8], [7.0, 1.2]], False, True, False, 70),
        ("forbidden", "forbidden", [[5.1, 2.0], [5.7, 2.0], [5.7, 3.0], [5.1, 3.0], [5.1, 2.0]], True, False, False, 100),
        ("no-stop", "no_stopping", [[6.1, 1.2], [6.8, 1.2], [6.8, 1.8], [6.1, 1.8], [6.1, 1.2]], False, False, True, 90),
        ("bump", "speed_bumps", [[2.5, 1.0], [2.8, 1.0], [2.8, 4.0], [2.5, 4.0], [2.5, 1.0]], False, True, False, 40),
    )
    features = [
        SemanticFeature(
            semantic_id, semantic_class, "polygon", points,
            hard=hard, soft=soft, non_stopping=non_stopping,
            direction_rule="route_tangent_right" if semantic_class == "lane" else "none",
            priority=priority, source_field=f"fixture.{semantic_id}",
        )
        for semantic_id, semantic_class, points, hard, soft, non_stopping, priority in data
    ]
    semantic_map = SemanticMapV1(
        "map", resolution, (0.0, 0.0, 0.0), width, height, "synthetic-hash",
        features=features, traffic_rules={"right_hand_drive": True},
    )
    raster = SemanticRasterizer(footprint=FOOTPRINT, safety_margin_m=0.05).rasterize(
        semantic_map, hospital_map=hospital_map,
    )
    return hospital_map, semantic_map, raster


def route_and_fields(level="R0", policy=None, goal=(8.0, 2.5, 0.0)):
    hospital_map, semantic_map, raster = fixture()
    route = [[0.8, 2.5], [3.5, 2.5], [5.0, 2.5], [8.0, 2.5]]
    builder = RegionalPreferenceBuilder(hospital_map, raster, policy=policy)
    forward = builder.build(route, goal=goal, allowed_mask=hospital_map.occupancy == 0, relaxation_level=level)
    reverse = builder.build(list(reversed(route)), goal=route[0], allowed_mask=hospital_map.occupancy == 0, relaxation_level=level)
    return hospital_map, semantic_map, raster, route, forward, reverse


def cell(hospital_map, x, y):
    value = hospital_map.world_to_cell(x, y)
    assert value is not None
    return value


def test_straight_lane_forward_keeps_query_right():
    hospital_map, _, _, _, forward, _ = route_and_fields()
    assert forward.cost[cell(hospital_map, 2.0, 1.4)] < forward.cost[cell(hospital_map, 2.0, 3.6)]
    assert abs(float(forward.lane_distance_to_right_m[cell(hospital_map, 2.0, 1.4)]) - 0.4) < 0.15


def test_reverse_query_automatically_flips_right_side():
    hospital_map, _, _, _, forward, reverse = route_and_fields()
    south, north = cell(hospital_map, 2.0, 1.4), cell(hospital_map, 2.0, 3.6)
    assert forward.cost[south] < forward.cost[north]
    assert reverse.cost[north] < reverse.cost[south]
    assert forward.route_hash != reverse.route_hash


def test_lane_preference_smoothly_decays_entering_junction():
    hospital_map, _, _, _, forward, _ = route_and_fields()
    far = forward.junction_transition_factor[cell(hospital_map, 3.0, 1.5)]
    near = forward.junction_transition_factor[cell(hospital_map, 3.6, 1.5)]
    inside = forward.junction_transition_factor[cell(hospital_map, 4.1, 1.5)]
    assert 1.0 >= far > near > inside == 0.0


def test_lane_preference_restores_after_junction():
    hospital_map, _, _, _, forward, _ = route_and_fields()
    inside = forward.junction_transition_factor[cell(hospital_map, 4.4, 1.5)]
    near_exit = forward.junction_transition_factor[cell(hospital_map, 4.9, 1.5)]
    far_exit = forward.junction_transition_factor[cell(hospital_map, 5.8, 1.5)]
    assert inside == 0.0 < near_exit < far_exit == 1.0


def test_parking_area_prefers_medial_center():
    hospital_map, _, _, _, forward, _ = route_and_fields(goal=(8.0, 2.5, 0.0))
    center_value = forward.parking_normalized_deviation[cell(hospital_map, 7.5, 2.5)]
    edge_value = forward.parking_normalized_deviation[cell(hospital_map, 7.1, 2.5)]
    assert center_value < edge_value


def test_parking_endpoint_taper_allows_edge_goal():
    hospital_map, semantic_map, raster = fixture()
    route = [[6.8, 2.5], [8.15, 1.35]]
    builder = RegionalPreferenceBuilder(hospital_map, raster)
    field = builder.build(route, goal=(8.15, 1.35, 0.0), allowed_mask=hospital_map.occupancy == 0)
    goal_cell = cell(hospital_map, 8.15, 1.35)
    assert field.cost[goal_cell] == 0
    assert field.parking_normalized_deviation[goal_cell] > 0.0


def test_forbidden_area_is_lethal_and_blocks_l1_edge():
    hospital_map, semantic_map, raster = fixture()
    edge = TopologyEdge(0, 0, 1, 2.0, 1.0, 1.0, 2.0, 2, [[4.8, 2.5], [6.0, 2.5]])
    graph = TopologyGraph(
        [TopologyNode(0, 4.8, 2.5, 96, 49, 1, 1.0, 2.0, 1), TopologyNode(1, 6.0, 2.5, 120, 49, 1, 1.0, 2.0, 1)],
        [edge],
    )
    topology = SimpleNamespace(graph=graph)
    annotator = EdgeSemanticAnnotator(
        hospital_map, semantic_map, raster, base_map_hash="base",
        topology_hash=topology_graph_hash(topology),
    )
    annotation = annotator.annotate(edge)
    assert annotation.blocked
    assert annotation.total_cost == float("inf")
    assert raster.hard_mask[cell(hospital_map, 5.4, 2.5)]


def test_no_stopping_is_traversable_but_goal_is_rejected():
    hospital_map, semantic_map, raster, _, field, _ = route_and_fields()
    composer = SemanticCostmapComposer()
    composed = composer.compose(hospital_map.occupancy, raster, field, semantics_enabled=True)
    goal = cell(hospital_map, 6.4, 1.5)
    assert composed.internal_cost[goal] < LETHAL_OBSTACLE
    auditor = SemanticPathAuditor(hospital_map, semantic_map, raster)
    result = auditor.audit(
        [{"x": 5.9, "y": 1.5, "yaw": 0.0, "source": "kinematic"},
         {"x": 6.4, "y": 1.5, "yaw": 0.0, "source": "kinematic"}],
        field, relaxation_level="R0",
        canonical_metrics={"static_footprint_valid": True, "kinematic_valid": True},
    )
    assert result.failure_code == "NO_STOPPING_GOAL_VIOLATION"


def test_narrow_channel_r2_disables_only_soft_preference():
    hospital_map, _, raster, _, r2, _ = route_and_fields(level="R2", policy={"narrow_channel_width_m": 4.0})
    lane_point = cell(hospital_map, 2.0, 1.5)
    assert r2.cost[lane_point] == 0
    assert not r2.active_lateral_mask[lane_point]
    assert not raster.hard_footprint_mask[lane_point]


def test_r4_never_relaxes_hard_forbidden():
    hospital_map, _, raster, _, r4, _ = route_and_fields(level="R4")
    composed = SemanticCostmapComposer().compose(
        hospital_map.occupancy, raster, r4,
        allowed_mask=hospital_map.occupancy == 0, semantics_enabled=True,
    )
    assert composed.internal_cost[cell(hospital_map, 5.4, 2.5)] == LETHAL_OBSTACLE


def test_static_obstacle_is_never_cleared_by_semantics():
    hospital_map, _, raster, _, field, _ = route_and_fields()
    composed = SemanticCostmapComposer().compose(hospital_map.occupancy, raster, field)
    assert composed.internal_cost[51, 21] == LETHAL_OBSTACLE
    assert composed.diagnostics["static_obstacle_cells_after"] >= composed.diagnostics["static_obstacle_cells_before"]


def test_base_nontrinary_occupancy_is_converted_exactly_once():
    values = np.asarray([[-1, 0, 1, 50, 99, 100]], dtype=np.int16)
    actual = SemanticCostmapComposer._base_internal(values)
    expected = occupancy_to_static_layer(values)
    assert np.array_equal(actual, expected)
    assert actual.tolist() == [[255, 0, 2, 127, 251, 254]]


def test_soft_cost_survives_nontrinary_static_layer_mapping():
    for cost in (1, 32, 120, 252):
        occupancy = internal_soft_to_occupancy(cost)
        actual = occupancy_to_static_layer(occupancy)
        assert int(occupancy) in range(1, 100)
        assert 0 < int(actual) < int(LETHAL_OBSTACLE)


def test_semantic_roi_ack_checks_hard_and_soft_values():
    hospital_map, _, raster, _, field, _ = route_and_fields()
    composed = SemanticCostmapComposer().compose(hospital_map.occupancy, raster, field)
    verifier = SemanticCostmapAckVerifier()
    good = verifier.verify(composed, composed.expected_master_cost, roi_sequence=7, received_costmap_timestamp_ns=123)
    assert good.acknowledged and good.roi_sequence == 7
    assert good.publication_version == "2A-V2-semantic-costmap-v1"
    bad_grid = composed.expected_master_cost.copy()
    soft_cell = tuple(np.argwhere((composed.soft_cost > 0) & ~composed.hard_semantic_mask)[0])
    bad_grid[soft_cell] = max(0, int(bad_grid[soft_cell]) - 1)
    bad = verifier.verify(composed, bad_grid, roi_sequence=8, received_costmap_timestamp_ns=124)
    assert not bad.acknowledged and bad.soft_mismatch_cells == 1


def test_pdmap_world_to_pgm_y_axis_is_inverted():
    atlas = {"map": {"zones": [{
        "id": "lane", "type": "lane", "nodes": [[0.0, 0.0], [1.0, 0.0], [1.0, 0.5], [0.0, 0.5]],
    }], "elements": []}}
    semantic_map = convert_payload(
        atlas, {"resolution": 0.05, "origin": [0.0, 0.0, 0.0]}, (40, 40),
        source_pdmap_hash="hash",
    )
    hospital_map = HospitalMap(
        Path("map.yaml"), Path("map.pgm"), 0.05, (0.0, 0.0, 0.0), 40, 40,
        np.zeros((40, 40), np.int8), np.ones((40, 40), np.float32),
    )
    raster = SemanticRasterizer(footprint=FOOTPRINT).rasterize(semantic_map, hospital_map=hospital_map)
    assert raster.masks["lane"][hospital_map.world_to_cell(0.5, 0.25)]
    assert not raster.masks["lane"][hospital_map.world_to_cell(0.5, 1.5)]


def test_polygon_overlap_priority_and_speed_bump_additivity():
    hospital_map, _, raster = fixture()
    overlap = cell(hospital_map, 4.2, 2.5)
    assert raster.class_grid[overlap] == CLASS_CODES["junction_area"]
    bump = cell(hospital_map, 2.65, 2.5)
    assert raster.masks["lane"][bump] and raster.masks["speed_bumps"][bump]


def test_cache_key_changes_with_semantics_policy_and_direction():
    base = dict(base_map_hash="a", semantic_map_hash="b", policy_hash="c", topology_hash="d")
    forward = semantic_edge_cache_key(**base, direction_signature="forward")
    assert forward != semantic_edge_cache_key(**base, direction_signature="reverse")
    assert forward != semantic_edge_cache_key(**{**base, "semantic_map_hash": "x"}, direction_signature="forward")
    assert forward != semantic_edge_cache_key(**{**base, "policy_hash": "x"}, direction_signature="forward")


def test_semantic_router_uses_one_multi_source_search_for_attachment_candidates():
    nodes = [
        TopologyNode(index, float(index), 0.0, index, 0, 1, 1.0, 2.0, 1)
        for index in range(4)
    ]
    edges = [
        TopologyEdge(0, 0, 2, 1.0, 1.0, 1.0, 2.0, 2, [[0.0, 0.0], [2.0, 0.0]]),
        TopologyEdge(1, 1, 2, 1.0, 1.0, 1.0, 2.0, 2, [[1.0, 0.0], [2.0, 0.0]]),
        TopologyEdge(2, 2, 3, 1.0, 1.0, 1.0, 2.0, 2, [[2.0, 0.0], [3.0, 0.0]]),
    ]

    class StubAnnotator:
        policy_hash = "stub"

        def annotate(self, edge, *, reversed_traversal=False):
            blocked = edge.edge_id == 0
            return EdgeSemanticAnnotation(
                edge_id=edge.edge_id, traversal_reversed=reversed_traversal,
                base_length_m=edge.length_m,
                total_cost=float("inf") if blocked else edge.length_m,
                blocked=blocked,
            )

    router = SemanticEdgeRouter(SimpleNamespace(graph=TopologyGraph(nodes, edges)), StubAnnotator())
    selected = router.search_any([0, 1], [3])
    assert selected is not None
    route, start_id, goal_id = selected
    assert (start_id, goal_id) == (1, 3)
    assert route.edge_ids == [1, 2]
    assert router.search_any([0, 1], [3]) is selected


def test_semantics_disabled_matches_r2_base_occupancy():
    hospital_map, _, raster, _, field, _ = route_and_fields()
    composed = SemanticCostmapComposer().compose(
        hospital_map.occupancy, raster, field, semantics_enabled=False,
    )
    assert np.all(composed.internal_cost[hospital_map.occupancy == 0] == 0)
    assert np.all(composed.internal_cost[hospital_map.occupancy == 100] == LETHAL_OBSTACLE)
    assert not np.any(composed.hard_semantic_mask)


def test_converter_does_not_infer_direction_from_polygon_order():
    zones = [
        {"id": "a", "type": "lane", "nodes": [[0, 0], [1, 0], [1, 1], [0, 1]]},
        {"id": "b", "type": "lane", "nodes": [[0, 1], [1, 1], [1, 0], [0, 0]]},
    ]
    semantic_map = convert_payload(
        {"map": {"zones": zones, "elements": [{"type": "traffic_rule", "right_hand_drive": True}]}},
        {"resolution": 0.05, "origin": [0.0, 0.0, 0.0]}, (40, 40), source_pdmap_hash="hash",
    )
    assert [feature.direction_rule for feature in semantic_map.features] == ["route_tangent_right"] * 2
    assert all("explicit_direction" not in feature.properties for feature in semantic_map.features)


def test_relaxation_records_trigger_stage_and_preserves_invariants():
    calls = []
    controller = PreferenceRelaxationController()

    def attempt(level, params):
        calls.append((level, params))
        return level == "R2", level, "NO_PATH_IN_CORRIDOR", "L3", True

    result = controller.run(attempt)
    assert result.success and result.relaxation_level == "R2"
    assert [item.relaxation_level for item in result.attempts] == ["R0", "R1", "R2"]
    assert result.attempts[1].trigger_reason == "NO_PATH_IN_CORRIDOR"
    assert all(item.hard_constraints_held for item in result.attempts)


def test_self_intersection_is_reported_and_rejected():
    semantic_map = convert_payload(
        {"map": {"zones": [{
            "id": "bow", "type": "lane", "nodes": [[0, 0], [1, 1], [0, 1], [1, 0]],
        }], "elements": []}},
        {"resolution": 0.05, "origin": [0.0, 0.0, 0.0]}, (40, 40), source_pdmap_hash="hash",
    )
    assert not semantic_map.features
    assert any(item.code == "SELF_INTERSECTING_POLYGON" for item in semantic_map.diagnostics)
