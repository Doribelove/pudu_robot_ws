from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy import ndimage

from arena_evaluation.edge_semantic_annotator import EdgeSemanticAnnotator, SemanticEdgeRouter, topology_graph_hash
from arena_evaluation.planner_benchmark.map_utils import HospitalMap
from arena_evaluation.regional_preference_r1 import RegionalPreferenceBuilderR1, expand_roi_to_route_lane_instances, orient_route_for_query
from arena_evaluation.semantic_costmap_composer import LETHAL_OBSTACLE, SemanticCostmap, SemanticCostmapComposer
from arena_evaluation.semantic_map import SemanticFeature, SemanticMapV1
from arena_evaluation.semantic_path_audit import SemanticPathAuditor
from arena_evaluation.semantic_rasterizer import SemanticRasterizer
from arena_evaluation.semantic_smac_session import SemanticSmacSession
from arena_evaluation.topology import TopologyEdge, TopologyGraph, TopologyNode, TopologyRoute
from arena_evaluation.two_layer_v2_semantic_r1_benchmark import ArmSwitches, _paired
from arena_evaluation.unified_four_backends_smoke import SmacSession


FOOTPRINT = [[0.255, 0.215], [0.255, -0.215], [-0.255, -0.215], [-0.255, 0.215]]


def fixture():
    height, width, resolution = 100, 180, 0.05
    occupancy = np.zeros((height, width), dtype=np.int8)
    occupancy[[0, -1], :] = 100
    occupancy[:, [0, -1]] = 100
    distance = ndimage.distance_transform_edt(occupancy == 0, sampling=resolution).astype(np.float32)
    hospital_map = HospitalMap(
        Path("synthetic.yaml"), Path("synthetic.pgm"), resolution,
        (0.0, 0.0, 0.0), width, height, occupancy, distance,
    )
    features = [
        SemanticFeature(
            "lane", "lane", "polygon",
            [[0.5, 1.0], [8.4, 1.0], [8.4, 4.0], [0.5, 4.0], [0.5, 1.0]],
            soft=True, direction_rule="route_tangent_right", priority=60, source_field="test.lane",
        ),
        SemanticFeature(
            "parking", "parking_area", "polygon",
            [[7.0, 1.2], [8.3, 1.2], [8.3, 3.8], [7.0, 3.8], [7.0, 1.2]],
            soft=True, priority=70, source_field="test.parking",
        ),
        SemanticFeature(
            "forbidden", "forbidden", "polygon",
            [[5.1, 2.0], [5.7, 2.0], [5.7, 3.0], [5.1, 3.0], [5.1, 2.0]],
            hard=True, priority=100, source_field="test.forbidden",
        ),
        SemanticFeature(
            "bump", "speed_bumps", "polygon",
            [[2.5, 1.0], [2.8, 1.0], [2.8, 4.0], [2.5, 4.0], [2.5, 1.0]],
            soft=True, priority=40, source_field="test.bump",
        ),
    ]
    semantic_map = SemanticMapV1(
        "map", resolution, (0.0, 0.0, 0.0), width, height, "synthetic-r1",
        features=features, traffic_rules={"right_hand_drive": True},
    )
    raster = SemanticRasterizer(footprint=FOOTPRINT, safety_margin_m=0.05).rasterize(
        semantic_map, hospital_map=hospital_map,
    )
    return hospital_map, semantic_map, raster


def cell(hospital_map, x, y):
    result = hospital_map.world_to_cell(x, y)
    assert result is not None
    return result


def fields():
    hospital_map, semantic_map, raster = fixture()
    route = [[0.8, 2.5], [3.8, 2.5], [5.0, 2.5], [8.0, 2.5]]
    builder = RegionalPreferenceBuilderR1(hospital_map, raster)
    allowed = hospital_map.occupancy == 0
    forward = builder.build(route, goal=route[-1], allowed_mask=allowed)
    reverse = builder.build(list(reversed(route)), goal=route[0], allowed_mask=allowed)
    return hospital_map, semantic_map, raster, route, forward, reverse


def test_r1_forward_reverse_target_flips_and_uses_boundary_pair():
    hospital_map, _, _, _, forward, reverse = fields()
    south = cell(hospital_map, 2.0, 1.4)
    north = cell(hospital_map, 2.0, 3.6)
    assert forward.cost[south] < forward.cost[north]
    assert reverse.cost[north] < reverse.cost[south]
    assert forward.lane_correct_side[south]
    assert reverse.lane_correct_side[north]
    assert not forward.lane_correct_side[north]
    assert not reverse.lane_correct_side[south]
    assert forward.lane_distance_to_right_m[south] < forward.lane_distance_to_left_m[south]
    assert reverse.lane_distance_to_right_m[north] < reverse.lane_distance_to_left_m[north]
    assert forward.diagnostics["correct_side_definition"] == "distance_right_le_distance_left"


def test_r1_cost_has_tolerance_plateau_and_stays_below_comfort_cap():
    hospital_map, _, _, _, forward, _ = fields()
    near_a = cell(hospital_map, 2.0, 1.35)
    near_b = cell(hospital_map, 2.0, 1.45)
    assert forward.cost[near_a] <= 2
    assert forward.cost[near_b] <= 2
    assert int(forward.cost.max()) <= 64
    assert forward.diagnostics["soft_cost_saturation_ratio"] < 0.5
    assert np.count_nonzero(forward.lane_error_m <= 0.5) > 0


def test_r1_r2_recompose_reuses_r0_geometry_exactly():
    hospital_map, _, raster = fixture()
    route = [[0.8, 2.5], [8.0, 2.5]]
    allowed = hospital_map.occupancy == 0
    builder = RegionalPreferenceBuilderR1(hospital_map, raster)
    base = builder.build(route, goal=route[-1], allowed_mask=allowed, relaxation_level="R0")
    for level in ("R1", "R2"):
        direct = builder.build(route, goal=route[-1], allowed_mask=allowed, relaxation_level=level)
        reused = builder.derive_relaxation(
            base, goal=route[-1], allowed_mask=allowed, relaxation_level=level,
        )
        assert np.array_equal(reused.cost, direct.cost)
        assert np.array_equal(reused.active_lateral_mask, direct.active_lateral_mask)
        assert reused.diagnostics["geometry_cache_hit"] is True


def test_audit_only_field_reports_lane_metrics_without_affecting_planning_cost():
    hospital_map, semantic_map, raster = fixture()
    route = [[0.8, 2.5], [8.0, 2.5]]
    field = RegionalPreferenceBuilderR1(hospital_map, raster).build(
        route, goal=route[-1], allowed_mask=hospital_map.occupancy == 0,
        planning_preference_enabled=False,
    )
    assert not np.any(field.cost)
    result = SemanticPathAuditor(hospital_map, semantic_map, raster).audit(
        [{"x": 1.0, "y": 1.4, "yaw": 0.0}, {"x": 3.0, "y": 1.4, "yaw": 0.0}],
        field, relaxation_level="R0",
        canonical_metrics={"static_footprint_valid": True, "kinematic_valid": True},
    )
    assert result.metrics["lane_correct_side_ratio"] == 1.0
    assert result.metrics["base_center_to_right_boundary_error_p50_m"] is not None
    assert result.metrics["path_vs_route_tangent_agreement_p50"] > 0.99


def test_route_orientation_reverses_copy_and_annotations_not_cached_source():
    route = TopologyRoute([1, 2], [9], 1.0, 2.0, [[9.0, 0.0], [1.0, 0.0]])
    setattr(route, "semantic_edge_annotations", [{"edge_id": 9, "traversal_reversed": False}])
    query = SimpleNamespace(start=(1.0, 0.0, 0.0), goal=(9.0, 0.0, 0.0))
    oriented, diagnostics = orient_route_for_query(route, query)
    assert diagnostics["route_reversed_for_query"]
    assert oriented.polyline == [[1.0, 0.0], [9.0, 0.0]]
    assert oriented.node_ids == [2, 1]
    assert oriented.semantic_edge_annotations[0]["traversal_reversed"] is True
    assert route.polyline == [[9.0, 0.0], [1.0, 0.0]]


def test_composer_orthogonal_switches_match_declared_behavior():
    hospital_map, _, raster, _, field, _ = fields()
    composer = SemanticCostmapComposer(policy={
        "soft_cost_max": 80,
        "class_soft_cost": {"speed_bumps": 72, "parking_area": 4},
    })
    allowed = hospital_map.occupancy == 0
    neutral = composer.compose(
        hospital_map.occupancy, raster, None, allowed_mask=allowed,
        hard_semantics_enabled=False, soft_class_costs_enabled=False,
        regional_preference_enabled=False,
    )
    forbidden = cell(hospital_map, 5.4, 2.5)
    assert neutral.internal_cost[forbidden] < LETHAL_OBSTACLE
    hard = composer.compose(
        hospital_map.occupancy, raster, None, allowed_mask=allowed,
        hard_semantics_enabled=True, soft_class_costs_enabled=False,
        regional_preference_enabled=False, hard_semantics_use_footprint=True,
    )
    assert hard.internal_cost[forbidden] == LETHAL_OBSTACLE
    footprint_buffer = cell(hospital_map, 4.85, 2.5)
    assert not raster.hard_mask[footprint_buffer]
    assert raster.hard_footprint_mask[footprint_buffer]
    assert hard.internal_cost[footprint_buffer] == LETHAL_OBSTACLE
    classes = composer.compose(
        hospital_map.occupancy, raster, None, allowed_mask=allowed,
        hard_semantics_enabled=False, soft_class_costs_enabled=True,
        regional_preference_enabled=False,
    )
    assert classes.soft_cost[cell(hospital_map, 2.65, 1.5)] == 72
    regional = composer.compose(
        hospital_map.occupancy, raster, field, allowed_mask=allowed,
        hard_semantics_enabled=False, soft_class_costs_enabled=False,
        regional_preference_enabled=True,
    )
    assert np.any(regional.soft_cost)
    assert int(regional.soft_cost.max()) <= 64


def test_l1_hard_and_cost_switches_are_independent():
    hospital_map, semantic_map, raster = fixture()
    edge = TopologyEdge(0, 0, 1, 1.2, 1.0, 1.0, 2.0, 2, [[4.8, 2.5], [6.0, 2.5]])
    graph = TopologyGraph(
        [TopologyNode(0, 4.8, 2.5, 96, 49, 1, 1.0, 2.0, 1), TopologyNode(1, 6.0, 2.5, 120, 49, 1, 1.0, 2.0, 1)],
        [edge],
    )
    topology = SimpleNamespace(graph=graph)
    no_hard = EdgeSemanticAnnotator(
        hospital_map, semantic_map, raster, base_map_hash="base",
        topology_hash=topology_graph_hash(topology),
        policy={"semantic_costs_enabled": True, "hard_semantics_enabled": False},
    ).annotate(edge)
    assert not no_hard.blocked and no_hard.total_cost > no_hard.base_length_m
    hard_no_cost = EdgeSemanticAnnotator(
        hospital_map, semantic_map, raster, base_map_hash="base",
        topology_hash=topology_graph_hash(topology),
        policy={"semantic_costs_enabled": False, "hard_semantics_enabled": True},
    ).annotate(edge)
    assert hard_no_cost.blocked and hard_no_cost.semantic_integral == 0.0


def test_arm_switch_validation_rejects_audit_off_and_labels_are_hashable():
    values = {
        "route_selector": "legacy", "l1_semantic_costs": False,
        "l1_hard_semantics": False, "l3_hard_semantics": False,
        "l3_soft_class_costs": False, "regional_preference": False,
        "semantic_audit": True, "relaxation": "strict",
    }
    parsed = ArmSwitches.parse(values)
    assert parsed.hash and parsed.route_selector == "legacy"
    with pytest.raises(ValueError, match="semantic_audit"):
        ArmSwitches.parse({**values, "semantic_audit": False})


def test_paired_comparison_never_mixes_success_sets_or_queries():
    rows = [
        {"arm": "E0", "query_id": "q1", "repetition": 1, "run_mode": "measured", "final_valid_success": True, "path_length_m": 10.0},
        {"arm": "E4", "query_id": "q1", "repetition": 1, "run_mode": "measured", "final_valid_success": True, "path_length_m": 12.0},
        {"arm": "E4", "query_id": "q2", "repetition": 1, "run_mode": "measured", "final_valid_success": True, "path_length_m": 2.0},
    ]
    result = _paired(rows)
    q1 = next(item for item in result if item["query_id"] == "q1")
    q2 = next(item for item in result if item["query_id"] == "q2")
    assert q1["paired_valid"] and q1["delta_path_length_m"] == 2.0
    assert not q2["paired_valid"] and q2["delta_path_length_m"] is None


def test_lane_instance_roi_expansion_excludes_adjacent_feature():
    hospital_map, semantic_map, _ = fixture()
    semantic_map.features.append(SemanticFeature(
        "adjacent", "lane", "polygon",
        [[0.5, 4.1], [8.4, 4.1], [8.4, 4.7], [0.5, 4.7], [0.5, 4.1]],
        soft=True, direction_rule="route_tangent_right", priority=60,
        source_field="test.adjacent",
    ))
    raster = SemanticRasterizer(footprint=FOOTPRINT, safety_margin_m=0.05).rasterize(
        semantic_map, hospital_map=hospital_map,
    )
    base = np.zeros(hospital_map.occupancy.shape, dtype=bool)
    base[cell(hospital_map, 1.0, 2.5)] = True
    expanded, diagnostics = expand_roi_to_route_lane_instances(
        hospital_map, raster, semantic_map, [[0.8, 2.5], [8.0, 2.5]], base,
        free_mask=hospital_map.occupancy == 0, route_probe_radius_m=0.2,
    )
    assert expanded[cell(hospital_map, 2.0, 2.0)]
    assert not expanded[cell(hospital_map, 2.0, 4.4)]
    assert diagnostics["selected_lane_instance_count"] == 1
    assert diagnostics["lane_instance_adjacent_instances_excluded"] == 1


def _endpoint_cost_topology():
    nodes = [
        TopologyNode(0, 0.0, 0.0, 0, 0, 1, 1.0, 2.0, 1),
        TopologyNode(1, 0.0, 1.0, 0, 1, 1, 1.0, 2.0, 1),
        TopologyNode(2, 10.0, 0.0, 10, 0, 1, 1.0, 2.0, 1),
        TopologyNode(3, 1.0, 1.0, 1, 1, 1, 1.0, 2.0, 1),
    ]
    edges = [
        TopologyEdge(10, 0, 2, 10.0, 1.0, 1.0, 2.0, 2, [[0.0, 0.0], [10.0, 0.0]]),
        TopologyEdge(11, 1, 3, 1.0, 1.0, 1.0, 2.0, 2, [[0.0, 1.0], [1.0, 1.0]]),
    ]
    return SimpleNamespace(graph=TopologyGraph(nodes, edges))


def test_semantic_multi_source_uses_endpoint_costs_without_changing_edge_policy():
    topology = _endpoint_cost_topology()

    class NeutralAnnotator:
        policy_hash = "neutral"

        @staticmethod
        def annotate(edge, reversed_traversal=False):
            return SimpleNamespace(
                blocked=False, total_cost=edge.length_m,
                to_dict=lambda: {
                    "edge_id": edge.edge_id,
                    "traversal_reversed": reversed_traversal,
                },
            )

    selected = SemanticEdgeRouter(topology, NeutralAnnotator()).search_any(
        [0, 1], [2, 3],
        start_costs={0: 0.0, 1: 8.0}, goal_costs={2: 0.0, 3: 8.0},
    )
    assert selected is not None
    route, start, goal = selected
    assert (start, goal) == (0, 2)
    assert route.length_m == pytest.approx(10.0)


def test_semantic_ack_noop_reuses_prior_verified_counts(monkeypatch):
    grid = np.zeros((2, 2), dtype=np.uint8)
    semantic = SemanticCostmap(
        internal_cost=grid, occupancy_grid=grid.astype(np.int8),
        expected_master_cost=grid, soft_cost=grid,
        hard_semantic_mask=grid.astype(bool), affected_mask=grid.astype(bool),
        policy_hash="policy", semantic_map_hash="map", preference_field_hash="field",
        semantics_enabled=True,
    )
    session = SemanticSmacSession.__new__(SemanticSmacSession)
    session._semantic_costmap = semantic
    key = session._semantic_ack_key(semantic)
    session._semantic_ack_cache = {key: {
        "costmap_ack_hard_checked_cells": 7,
        "costmap_ack_hard_mismatch_cells": 0,
        "costmap_ack_soft_checked_cells": 11,
        "costmap_ack_soft_mismatch_cells": 0,
        "costmap_ack_soft_exact_mismatch_cells": 3,
        "costmap_ack_soft_exact_mismatch_ratio": 3 / 11,
        "costmap_ack_semantics": "interval_not_exact",
    }}
    monkeypatch.setattr(
        SmacSession, "update_local_mask",
        lambda self, allowed_mask, **kwargs: {"local_map_update_mode": "reuse_noop"},
    )
    result = SemanticSmacSession.update_local_mask(
        session, np.ones((2, 2), dtype=bool),
    )
    assert result["costmap_ack_status"] == "reused_server_verified_semantic_state"
    assert result["costmap_ack_soft_checked_cells"] == 11
    assert result["costmap_ack_soft_exact_mismatch_cells"] == 3
