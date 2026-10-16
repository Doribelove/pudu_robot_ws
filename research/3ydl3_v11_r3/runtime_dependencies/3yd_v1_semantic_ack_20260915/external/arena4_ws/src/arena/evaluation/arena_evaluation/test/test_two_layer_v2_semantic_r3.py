from __future__ import annotations

import math

import numpy as np
from scipy import ndimage

from arena_evaluation.regional_preference_r3 import RegionalPreferenceBuilderR3
from arena_evaluation.regional_preference_r2 import RegionalPreferenceBuilderR2
from test_two_layer_v2_semantic_r1 import cell, fixture


TEST_POLICY = {
    "guide_station_spacing_m": 0.50,
    "guide_lateral_sample_spacing_m": 0.10,
    "guide_max_lateral_probe_m": 2.0,
    "guide_beam_width": 160,
    "guide_static_clearance_m": 0.20,
}


def _curvature(first, second, third):
    a = math.dist(first, second)
    b = math.dist(second, third)
    c = math.dist(first, third)
    denominator = a * b * c
    if denominator <= 1.0e-12:
        return float("inf")
    twice_area = abs(
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    )
    return 2.0 * twice_area / denominator


def test_r3_forward_reverse_builds_mirrored_curvature_feasible_guides():
    hospital_map, semantic_map, raster = fixture()
    route = [[0.8, 2.5], [8.0, 2.5]]
    allowed = hospital_map.occupancy == 0
    builder = RegionalPreferenceBuilderR3(
        hospital_map, raster, semantic_map=semantic_map, policy=TEST_POLICY,
    )
    forward = builder.build(route, goal=route[-1], allowed_mask=allowed)
    reverse = builder.build(list(reversed(route)), goal=route[0], allowed_mask=allowed)

    for field in (forward, reverse):
        assert field.diagnostics["guide_status"] == "BUILT"
        assert field.diagnostics["viability_gate_passed"] is True
        assert field.diagnostics["viability_classification"] == "DIRECTED_TARGET_VIABLE"
        assert field.diagnostics["viability_full_path_correct_side_station_ratio"] >= 0.80
        assert field.diagnostics["viability_full_path_target_station_ratio"] > 0.50
        assert field.diagnostics["guide_applied_to_planning"] is True
        assert field.diagnostics["guide_soft_only"] is True
        assert field.diagnostics["guide_max_curvature_1pm"] <= 2.50 + 1.0e-9
        assert field.diagnostics["guide_correct_side_ratio"] >= 0.80
        assert field.diagnostics["guide_target_error_p50_m"] <= 0.50
        assert int(field.cost.max()) <= 64

    forward_south = cell(hospital_map, 4.0, 1.4)
    forward_north = cell(hospital_map, 4.0, 3.6)
    reverse_south = cell(hospital_map, 4.0, 1.4)
    reverse_north = cell(hospital_map, 4.0, 3.6)
    assert forward.cost[forward_south] < forward.cost[forward_north]
    assert reverse.cost[reverse_north] < reverse.cost[reverse_south]
    assert (
        forward.diagnostics["viability_certificate_hash"]
        != reverse.diagnostics["viability_certificate_hash"]
    )
    assert forward.diagnostics["viability_route_hash"] != reverse.diagnostics["viability_route_hash"]


def test_r3_guide_routes_continuously_around_fragmented_target_band():
    hospital_map, semantic_map, raster = fixture()
    # Break the preferred south-side band without blocking the complete lane.
    obstacle = np.zeros_like(hospital_map.occupancy, dtype=bool)
    for x in np.arange(3.9, 4.16, 0.05):
        for y in np.arange(1.0, 1.90, 0.05):
            obstacle[cell(hospital_map, float(x), float(y))] = True
    hospital_map.occupancy[obstacle] = 100
    hospital_map.distance_m = ndimage.distance_transform_edt(
        hospital_map.occupancy != 100, sampling=hospital_map.resolution,
    ).astype(np.float32)
    allowed = hospital_map.occupancy == 0
    route = [[0.8, 2.5], [8.0, 2.5]]
    field = RegionalPreferenceBuilderR3(
        hospital_map, raster, semantic_map=semantic_map,
        policy={**TEST_POLICY, "guide_static_clearance_m": 0.25},
    ).build(route, goal=route[-1], allowed_mask=allowed)

    assert field.diagnostics["guide_status"] == "BUILT"
    points = field.diagnostics["guide_polyline_world"]
    assert len(points) >= 8
    assert field.diagnostics["guide_max_curvature_1pm"] <= 2.50 + 1.0e-9
    assert all(
        hospital_map.distance_m[cell(hospital_map, point[0], point[1])] >= 0.25
        for point in points
    )
    curvatures = [
        _curvature(first, second, third)
        for first, second, third in zip(points, points[1:], points[2:])
    ]
    assert max(curvatures, default=0.0) <= 2.50 + 1.0e-9


def test_r3_audit_only_and_relaxed_attempts_do_not_apply_the_guide():
    hospital_map, semantic_map, raster = fixture()
    route = [[0.8, 2.5], [8.0, 2.5]]
    allowed = hospital_map.occupancy == 0
    builder = RegionalPreferenceBuilderR3(
        hospital_map, raster, semantic_map=semantic_map, policy=TEST_POLICY,
    )
    audit_only = builder.build(
        route, goal=route[-1], allowed_mask=allowed,
        planning_preference_enabled=False,
    )
    relaxed = builder.build(
        route, goal=route[-1], allowed_mask=allowed, relaxation_level="R1",
    )
    assert audit_only.diagnostics["guide_status"] == "BUILT"
    assert audit_only.diagnostics["guide_applied_to_planning"] is False
    assert not np.any(audit_only.cost)
    assert relaxed.diagnostics["guide_applied_to_planning"] is False


def test_r3_guide_never_crosses_an_unselected_lane_instance():
    hospital_map, semantic_map, raster = fixture()
    route = [[0.8, 2.5], [8.0, 2.5]]
    allowed = hospital_map.occupancy == 0
    field = RegionalPreferenceBuilderR3(
        hospital_map, raster, semantic_map=semantic_map, policy=TEST_POLICY,
    ).build(route, goal=route[-1], allowed_mask=allowed)
    selected = set(field.diagnostics["guide_lane_instance_ids"])
    assert selected
    for point in field.diagnostics["guide_polyline_world"]:
        label = int(field.lane_instance_id[cell(hospital_map, point[0], point[1])])
        assert label in selected


def test_r3_semantically_rejected_guide_fails_closed_to_r2_cost():
    hospital_map, semantic_map, raster = fixture()
    # Make the desired south band unavailable over most of the route while
    # leaving a continuous, safe path near the center/north of the lane.
    obstacle = np.zeros_like(hospital_map.occupancy, dtype=bool)
    for x in np.arange(1.4, 7.41, 0.05):
        for y in np.arange(1.0, 2.20, 0.05):
            obstacle[cell(hospital_map, float(x), float(y))] = True
    hospital_map.occupancy[obstacle] = 100
    hospital_map.distance_m = ndimage.distance_transform_edt(
        hospital_map.occupancy != 100, sampling=hospital_map.resolution,
    ).astype(np.float32)
    allowed = hospital_map.occupancy == 0
    route = [[0.8, 2.5], [8.0, 2.5]]
    policy = {**TEST_POLICY, "guide_static_clearance_m": 0.10}
    r2 = RegionalPreferenceBuilderR2(
        hospital_map, raster, semantic_map=semantic_map, policy=policy,
    ).build(route, goal=route[-1], allowed_mask=allowed)
    r3 = RegionalPreferenceBuilderR3(
        hospital_map, raster, semantic_map=semantic_map, policy=policy,
    ).build(route, goal=route[-1], allowed_mask=allowed)
    assert r3.diagnostics["guide_status"] == "NO_FEASIBLE_GUIDE"
    assert r3.diagnostics["viability_gate_passed"] is False
    assert r3.diagnostics["viability_classification"] in {
        "DIRECTED_TARGET_LOCALLY_UNAVAILABLE",
        "DIRECTED_ONLINE_METRIC_PAIR_INFEASIBLE",
        "DIRECTED_LANE_CURVATURE_DISCONNECTED",
    }
    assert r3.diagnostics["guide_acceptance_gate_passed"] is False
    assert r3.diagnostics["guide_applied_to_planning"] is False
    assert np.array_equal(r3.cost, r2.cost)


def test_r3_certificate_is_bound_to_roi_and_same_lane_source_instance():
    hospital_map, semantic_map, raster = fixture()
    route = [[0.8, 2.5], [8.0, 2.5]]
    full = hospital_map.occupancy == 0
    clipped = full.copy()
    clipped[:, :20] = False
    builder = RegionalPreferenceBuilderR3(
        hospital_map, raster, semantic_map=semantic_map, policy=TEST_POLICY,
    )
    first = builder.build(route, goal=route[-1], allowed_mask=full)
    second = builder.build(route, goal=route[-1], allowed_mask=clipped)

    assert first.diagnostics["viability_schema_version"].endswith("certificate-v1")
    assert first.diagnostics["viability_roi_hash"] != second.diagnostics["viability_roi_hash"]
    assert first.diagnostics["viability_certificate_hash"] != second.diagnostics["viability_certificate_hash"]
    assert first.diagnostics["viability_lane_instance_ids"] == ["lane"]
    assert all(
        record["lane_semantic_id"] == "lane"
        for record in first.diagnostics["viability_lane_runs"]
    )
