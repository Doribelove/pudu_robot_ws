import math
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml
from PIL import Image

from arena_evaluation.planner_benchmark.map_utils import HospitalMap
from arena_evaluation.stage8 import (
    HardRadiusConfig,
    arc_repair_windows,
    classify_segments,
    diagnose_hard_path,
    menger_curvature,
    repair_window_schedule,
    resample_by_arc,
    stitch_errors,
)
from arena_evaluation.stage8_cli import _within_window, _reference_run, _run_row, build_parser as stage8a_parser
from arena_evaluation.preference_cli import build_parser as stage8b_parser, run as run_preference
from arena_evaluation.preference import build_preference_geometry, preference_astar
from arena_evaluation.topology import TopologyArtifact, TopologyGraph, TopologyRoute, astar_grid


FOOTPRINT = [[0.10, 0.08], [0.10, -0.08], [-0.10, -0.08], [-0.10, 0.08]]


def _map(tmp_path: Path) -> HospitalMap:
    Image.fromarray(np.full((200, 200), 254, dtype=np.uint8)).save(tmp_path / "map.pgm")
    (tmp_path / "map.yaml").write_text(yaml.safe_dump({
        "image": "map.pgm", "resolution": 0.05, "origin": [-5.0, -5.0, 0.0],
        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196,
    }))
    return HospitalMap.load(tmp_path / "map.yaml")


def _arc(radius: float, start: float = 0.0, end: float = math.pi / 2.0):
    count = max(4, int(math.ceil(abs(end - start) * radius / 0.025)))
    return [
        {"x": radius * math.cos(a), "y": radius * math.sin(a), "yaw": a + math.pi / 2.0}
        for a in np.linspace(start, end, count)
    ]


def test_zero_displacement_yaw_change_is_forbidden(tmp_path):
    diagnostics = diagnose_hard_path(
        [{"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": 0.0, "y": 0.0, "yaw": 0.5}],
        _map(tmp_path), FOOTPRINT, HardRadiusConfig(),
    )
    assert diagnostics.zero_displacement_yaw_changes == 1
    assert "IN_PLACE_ROTATION_FORBIDDEN" in diagnostics.failure_codes
    assert not diagnostics.hard_kinematic_valid


def test_hard_radius_boundary_and_violation(tmp_path):
    valid = diagnose_hard_path(_arc(0.40), _map(tmp_path), FOOTPRINT, HardRadiusConfig())
    invalid = diagnose_hard_path(_arc(0.30), _map(tmp_path), FOOTPRINT, HardRadiusConfig())
    assert valid.hard_radius_violation_count == 0
    assert invalid.hard_radius_violation_count > 0


def test_menger_curvature_is_stable_after_fixed_arc_resampling():
    points = resample_by_arc(_arc(1.0), 0.05)
    values = [menger_curvature(points[i - 1], points[i], points[i + 1]) for i in range(1, len(points) - 1)]
    assert np.median(values) == pytest.approx(1.0, rel=0.03)


def test_forward_reverse_and_cusp_are_explicit(tmp_path):
    points = [
        {"x": -0.5, "y": 0.0, "yaw": 0.0},
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
        {"x": -0.5, "y": 0.0, "yaw": 0.0},
    ]
    diagnostics = diagnose_hard_path(points, _map(tmp_path), FOOTPRINT, HardRadiusConfig())
    assert diagnostics.direction_distance["forward"] > 0.0
    assert diagnostics.direction_distance["reverse"] > 0.0
    assert diagnostics.direction_switch_count == 1


def test_reverse_curve_obeys_same_hard_radius(tmp_path):
    points = _arc(0.30)
    for point in points:
        point["yaw"] += math.pi
    diagnostics = diagnose_hard_path(points, _map(tmp_path), FOOTPRINT, HardRadiusConfig())
    assert diagnostics.direction_distance["reverse"] > 0.0
    assert diagnostics.hard_radius_violation_count > 0


def test_repair_windows_use_one_then_two_metres():
    config = HardRadiusConfig()
    assert repair_window_schedule(config) == (1.0, 2.0)
    points = [{"x": i * 0.05, "y": 0.0, "yaw": 0.0} for i in range(101)]
    windows = arc_repair_windows(points, [50], 1.0)
    assert len(windows) == 1
    assert points[50]["x"] - points[windows[0].start_index]["x"] >= 1.0 - 1e-9
    assert points[windows[0].end_index]["x"] - points[50]["x"] >= 1.0 - 1e-9


def test_local_smac_candidate_cannot_leave_window():
    original = [{"x": i * 0.05, "y": 0.0, "yaw": 0.0} for i in range(41)]
    window = arc_repair_windows(original, [20], 1.0)[0]
    assert _within_window(original, original, window, 1.0)
    escaped = list(original) + [{"x": 1.0, "y": 2.0, "yaw": 0.0}]
    assert not _within_window(escaped, original, window, 1.0)


def test_stitch_position_and_yaw_tolerances():
    config = HardRadiusConfig()
    _, _, valid = stitch_errors(
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
        {"x": 0.05, "y": 0.0, "yaw": math.radians(10.0)}, config,
    )
    assert valid
    assert not stitch_errors(
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
        {"x": 0.051, "y": 0.0, "yaw": 0.0}, config,
    )[2]


def test_segment_source_has_no_rotate_in_place():
    segments = classify_segments(
        [{"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": 0.5, "y": 0.0, "yaw": 0.0}],
        grid_mode="corridor", topology_edge_ids=[1], source="kinematic",
        planner="smac_hybrid_reeds_shepp", repair_reason="MINIMUM_TURNING_RADIUS_VIOLATION",
    )
    assert segments[0]["source"] == "kinematic"
    assert segments[0]["direction"] in {"forward", "reverse"}
    assert segments[0]["direction"] != "rotate_in_place"


def _artifact(hospital_map: HospitalMap) -> TopologyArtifact:
    free = hospital_map.occupancy == 0
    return TopologyArtifact(hospital_map, free, np.zeros_like(free), hospital_map.distance_m, np.ones_like(free, dtype=np.int32), TopologyGraph(), {})


def _bounded_map(tmp_path: Path) -> HospitalMap:
    pixels = np.full((200, 200), 254, dtype=np.uint8)
    pixels[[0, -1], :] = 0; pixels[:, [0, -1]] = 0
    Image.fromarray(pixels).save(tmp_path / "map.pgm")
    (tmp_path / "map.yaml").write_text(yaml.safe_dump({"image": "map.pgm", "resolution": 0.05, "origin": [-5.0, -5.0, 0.0], "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196}))
    return HospitalMap.load(tmp_path / "map.yaml")


def test_none_preference_matches_stage6_astar():
    free = np.ones((20, 20), dtype=bool)
    free[8:12, 10] = False
    expected = astar_grid(free, (10, 2), (10, 17), return_stats=True)
    actual = preference_astar(free, (10, 2), (10, 17), None, None, 0.0, 1.0)
    assert actual.path == expected.path
    assert actual.expanded_nodes == expected.expanded_nodes


def test_center_cost_prefers_route_center(tmp_path):
    artifact = _artifact(_bounded_map(tmp_path))
    route = TopologyRoute([], [], 4.0, 4.0, [[-2.0, 0.0], [2.0, 0.0]])
    geometry = build_preference_geometry(artifact, route, "center")
    center = artifact.hospital_map.world_to_cell(0.0, 0.0)
    off_center = artifact.hospital_map.world_to_cell(0.0, 1.0)
    assert geometry.penalty[center] < geometry.penalty[off_center]


def test_right_edge_uses_directed_signed_lateral_offset(tmp_path):
    artifact = _artifact(_bounded_map(tmp_path))
    route = TopologyRoute([], [], 4.0, 4.0, [[-2.0, 0.0], [2.0, 0.0]])
    geometry = build_preference_geometry(artifact, route, "right_edge")
    right = artifact.hospital_map.world_to_cell(0.0, -1.0)
    left = artifact.hospital_map.world_to_cell(0.0, 1.0)
    assert geometry.lateral_deviation_m[right] < 0.0
    assert geometry.lateral_deviation_m[left] > 0.0
    assert geometry.right_wall_distance_m[right] < geometry.right_wall_distance_m[left]
    near_target = artifact.hospital_map.world_to_cell(0.0, -4.5)
    center = artifact.hospital_map.world_to_cell(0.0, 0.0)
    assert abs(float(geometry.right_wall_distance_m[near_target]) - 0.40) < abs(float(geometry.right_wall_distance_m[center]) - 0.40)


def test_narrow_channel_disables_preference(tmp_path):
    pixels = np.full((200, 200), 254, dtype=np.uint8)
    pixels[90, :] = 0; pixels[110, :] = 0
    Image.fromarray(pixels).save(tmp_path / "map.pgm")
    (tmp_path / "map.yaml").write_text(yaml.safe_dump({"image": "map.pgm", "resolution": 0.05, "origin": [-5.0, -5.0, 0.0], "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196}))
    artifact = _artifact(HospitalMap.load(tmp_path / "map.yaml"))
    route = TopologyRoute([], [], 4.0, 1.0, [[-2.0, 0.0], [2.0, 0.0]])
    geometry = build_preference_geometry(artifact, route, "right_edge")
    center = artifact.hospital_map.world_to_cell(0.0, 0.0)
    assert not geometry.active[center]
    assert geometry.penalty[center] == 0.0


def test_action_success_is_separate_from_static_and_hard_valid(tmp_path):
    pixels = np.full((200, 200), 254, dtype=np.uint8); pixels[100, 100] = 0
    Image.fromarray(pixels).save(tmp_path / "map.pgm")
    (tmp_path / "map.yaml").write_text(yaml.safe_dump({"image": "map.pgm", "resolution": 0.05, "origin": [-5.0, -5.0, 0.0], "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196}))
    source = pd.Series({"query_id": "qx", "repetition": 1, "grid_mode": "corridor"})
    points = [{"x": -0.05, "y": -0.05, "yaw": 0.0}, {"x": 0.05, "y": -0.05, "yaw": 0.0}]
    row = _reference_run("full_smac_normalized", source, points, HospitalMap.load(tmp_path / "map.yaml"), HardRadiusConfig(), tmp_path, source_run_id="reference")
    assert row["action_success"] is True
    assert row["static_footprint_valid"] is False
    assert row["final_valid_success"] is False


def test_q04_does_not_enter_l3(tmp_path):
    row = pd.Series({"query_id": "q04", "repetition": 1, "run_id": "q04_source", "path_file": np.nan})
    result, points = _run_row(row, tmp_path, _map(tmp_path), HardRadiusConfig(), None, tmp_path / "out")
    assert points is None
    assert result["failure_code"] == "STATIC_SEMANTICS_CONSERVATIVE_INFLATION_MISMATCH"
    assert result["hybrid_calls"] == 0


def test_dynamic_static_marker_is_required_by_stage8_clis():
    with pytest.raises(SystemExit): stage8a_parser().parse_args(["--output-dir", "out"])
    with pytest.raises(SystemExit): stage8b_parser().parse_args(["--output-dir", "out"])


def test_stage8b_refuses_nonempty_output(tmp_path):
    output = tmp_path / "old_stage"; output.mkdir(); (output / "frozen.csv").write_text("x\n")
    with pytest.raises(ValueError, match="refusing to overwrite"):
        run_preference(output, tmp_path / "map.yaml", tmp_path / "protocol.yaml", tmp_path / "queries.yaml", tmp_path / "topology", 1, 5.0)
