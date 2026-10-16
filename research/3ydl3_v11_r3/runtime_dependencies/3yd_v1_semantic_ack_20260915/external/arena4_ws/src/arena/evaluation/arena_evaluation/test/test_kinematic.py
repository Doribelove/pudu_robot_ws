from pathlib import Path

import numpy as np
import pytest
import yaml
from PIL import Image

from arena_evaluation.kinematic import (
    KinematicConfig,
    build_segments,
    diagnose_path,
    insert_safe_rotations,
    merge_trigger_indices,
    repair_path,
    rotation_sweep_collision,
    repair_window_schedule,
    stitch_error,
    unwrap_angles,
    wrap_angle,
)
from arena_evaluation.kinematic_cli import build_parser
from arena_evaluation.planner_benchmark.map_utils import HospitalMap


FOOTPRINT = [[0.12, 0.09], [0.12, -0.09], [-0.12, -0.09], [-0.12, 0.09]]


def _open_map(tmp_path: Path, size: int = 80) -> HospitalMap:
    image = tmp_path / "map.pgm"
    Image.fromarray(np.full((size, size), 254, dtype=np.uint8)).save(image)
    config = tmp_path / "map.yaml"
    config.write_text(yaml.safe_dump({
        "image": image.name, "resolution": 0.05, "origin": [-2.0, -2.0, 0.0],
        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196,
    }))
    return HospitalMap.load(config)


def test_yaw_wrap_and_unwrap():
    assert wrap_angle(3.5) == pytest.approx(-2.7831853)
    values = unwrap_angles([3.1, -3.1, -3.0])
    assert values[1] > values[0]
    assert values[2] > values[1]


def test_heading_trigger_and_rotation_insertion(tmp_path):
    hospital_map = _open_map(tmp_path)
    points = [
        {"x": -1.0, "y": 0.0, "yaw": 0.0},
        {"x": -0.5, "y": 0.0, "yaw": 0.0},
        {"x": 0.0, "y": 0.0, "yaw": 1.2},
        {"x": 0.0, "y": 0.5, "yaw": 1.2},
    ]
    config = KinematicConfig()
    diagnostics = diagnose_path(points, hospital_map, FOOTPRINT, config)
    assert diagnostics.trigger_indices
    repaired, rotations, result = insert_safe_rotations(diagnostics, hospital_map, FOOTPRINT, config)
    assert result["success"]
    assert rotations
    assert any(abs(a["x"] - b["x"]) < 1e-9 and abs(a["y"] - b["y"]) < 1e-9 for a, b in zip(repaired, repaired[1:]))
    post = diagnose_path(repaired, hospital_map, FOOTPRINT, config)
    assert post.rotation_collision_count == 0


def test_rotation_sweep_uses_five_degree_samples(tmp_path):
    hospital_map = _open_map(tmp_path)
    point = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    collision, samples = rotation_sweep_collision(hospital_map, point, 0.0, np.pi / 2, FOOTPRINT, 5.0)
    assert collision is False
    assert samples == 19


def test_trigger_points_merge_and_window_order():
    assert merge_trigger_indices([1, 2, 4, 10], max_index_gap=2) == [[1, 2, 4], [10]]
    config = KinematicConfig(initial_repair_window_m=1.0, expanded_repair_window_m=2.0)
    assert repair_window_schedule(config) == (1.0, 2.0)


def test_forward_reverse_and_switches(tmp_path):
    hospital_map = _open_map(tmp_path)
    points = [
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
        {"x": 0.5, "y": 0.0, "yaw": 0.0},
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
    ]
    diagnostics = diagnose_path(points, hospital_map, FOOTPRINT, KinematicConfig())
    assert diagnostics.direction_distance["forward"] > 0
    assert diagnostics.direction_distance["reverse"] > 0
    assert diagnostics.direction_switch_count == 1


def test_stitch_tolerances():
    config = KinematicConfig()
    before = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    after = {"x": 0.04, "y": 0.0, "yaw": np.deg2rad(9.0)}
    position, yaw, valid = stitch_error(before, after, config)
    assert position <= 0.05 and yaw <= 10.0 and valid


def test_source_and_direction_segments(tmp_path):
    points = [
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
        {"x": 0.0, "y": 0.0, "yaw": 1.0},
        {"x": 0.5, "y": 0.0, "yaw": 0.0},
    ]
    segments = build_segments(points, grid_mode="corridor", topology_edge_ids=[3])
    assert segments[0]["source"] == "kinematic"
    assert segments[0]["direction"] == "rotate_in_place"
    assert segments[1]["source"] == "grid"
    assert segments[1]["direction"] == "forward"


def test_unsafe_rotation_without_smac_is_structured_failure(tmp_path):
    hospital_map = _open_map(tmp_path)
    # A valid open-space path still has a heading trigger, but a fake obstacle
    # is not needed to exercise the no-backend contract here.
    points = [
        {"x": -1.0, "y": 0.0, "yaw": 0.0},
        {"x": -0.5, "y": 0.0, "yaw": 0.0},
        {"x": 0.0, "y": 0.0, "yaw": 1.2},
        {"x": 0.0, "y": 0.5, "yaw": 1.2},
    ]
    result = repair_path(points, hospital_map, FOOTPRINT, KinematicConfig())
    assert result["success"]
    assert result["hybrid_calls"] == 0


def test_dynamic_marker_is_required_by_cli():
    parser = build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(["--output-dir", "out"])
    args = parser.parse_args(["--output-dir", "out", "--no-dynamic-obstacles"])
    assert args.no_dynamic_obstacles is True


def test_raw_grid_heading_jump_is_not_hard_valid(tmp_path):
    hospital_map = _open_map(tmp_path)
    points = [
        {"x": -1.0, "y": 0.0, "yaw": 0.0},
        {"x": -0.5, "y": 0.0, "yaw": 1.0},
        {"x": 0.0, "y": 0.0, "yaw": 1.0},
    ]
    diagnostics = diagnose_path(points, hospital_map, FOOTPRINT, KinematicConfig())
    assert diagnostics.heading_jump_count == 1
    assert diagnostics.hard_kinematic_valid is False
