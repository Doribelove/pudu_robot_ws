from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from PIL import Image

from arena_evaluation.planner_benchmark.map_utils import HospitalMap
from arena_evaluation.topology import (
    TOPOLOGY_FAILURE_CODES,
    TopologyArtifact,
    astar_grid,
    attach_pose,
    build_graph,
    build_topology,
    corridor_mask,
    extract_skeleton,
    footprint_hash,
    load_topology,
    preprocess_static_map,
    save_topology,
    search_topology,
    static_collision_count,
)
from arena_evaluation.topology_cli import _assert_static_cli
from arena_evaluation.topology_cli import corridor_padding_schedule


FOOTPRINT = [[0.12, 0.09], [0.12, -0.09], [-0.12, -0.09], [-0.12, 0.09]]


def _write_map(tmp_path: Path, pixels: np.ndarray, resolution: float = 0.1) -> HospitalMap:
    image = tmp_path / "map.pgm"
    Image.fromarray(pixels.astype(np.uint8)).save(image)
    (tmp_path / "map.yaml").write_text(yaml.safe_dump({
        "image": image.name, "resolution": resolution, "origin": [-2.5, -2.5, 0.0],
        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196,
    }))
    return HospitalMap.load(tmp_path / "map.yaml")


def _open_map(tmp_path: Path, size: int = 50) -> HospitalMap:
    return _write_map(tmp_path, np.full((size, size), 254, dtype=np.uint8))


def _line_skeleton(size: int = 50) -> np.ndarray:
    skeleton = np.zeros((size, size), dtype=bool)
    skeleton[size // 2, 5:size - 5] = True
    return skeleton


def test_map_inflation_blocks_cells_near_obstacle(tmp_path):
    pixels = np.full((50, 50), 254, dtype=np.uint8)
    pixels[25, 25] = 0
    hospital_map = _write_map(tmp_path, pixels)
    inflated, free, _, _ = preprocess_static_map(hospital_map, FOOTPRINT, padding_m=0.1, safety_margin_m=0.1)
    assert inflated[25, 25]
    assert not free[25, 25]
    assert not free[25, 23]


def test_unknown_policy_changes_free_mask(tmp_path):
    pixels = np.full((20, 20), 254, dtype=np.uint8)
    pixels[10, 10] = 205
    hospital_map = _write_map(tmp_path, pixels)
    _, strict_free, _, _ = preprocess_static_map(hospital_map, FOOTPRINT, allow_unknown=False)
    _, permissive_free, _, _ = preprocess_static_map(hospital_map, FOOTPRINT, allow_unknown=True)
    assert not strict_free[10, 10]
    assert permissive_free[10, 10]


def test_distance_transform_is_zero_at_inflated_obstacle(tmp_path):
    pixels = np.full((30, 30), 254, dtype=np.uint8)
    pixels[15, 15] = 0
    hospital_map = _write_map(tmp_path, pixels)
    _, free, distance, _ = preprocess_static_map(hospital_map, FOOTPRINT, padding_m=0.0, safety_margin_m=0.0)
    assert distance[15, 15] == 0
    assert distance[0, 0] > 0
    assert not free[15, 15]


def test_skeleton_extracts_only_free_pixels(tmp_path):
    hospital_map = _open_map(tmp_path)
    _, free, _, _ = preprocess_static_map(hospital_map, FOOTPRINT, padding_m=0.0, safety_margin_m=0.0)
    skeleton = extract_skeleton(free)
    assert skeleton.any()
    assert np.all(~skeleton | free)


def test_endpoint_nodes_are_identified(tmp_path):
    hospital_map = _open_map(tmp_path)
    skeleton = _line_skeleton()
    graph = build_graph(skeleton, np.ones_like(skeleton, dtype=np.float32), 0.1, hospital_map)
    assert len(graph.nodes) >= 2
    assert sum(node.degree == 1 for node in graph.nodes) >= 2


def test_branch_nodes_are_identified(tmp_path):
    hospital_map = _open_map(tmp_path)
    skeleton = _line_skeleton()
    skeleton[10:26, 25] = True
    graph = build_graph(skeleton, np.ones_like(skeleton, dtype=np.float32), 0.1, hospital_map)
    assert any(node.degree >= 3 for node in graph.nodes)


def test_degree_two_pixels_are_compressed_into_edge(tmp_path):
    hospital_map = _open_map(tmp_path)
    graph = build_graph(_line_skeleton(), np.ones((50, 50), dtype=np.float32), 0.1, hospital_map)
    assert graph.edges
    assert max(edge.pixel_count for edge in graph.edges) > 2


def test_graph_edge_length_uses_resolution(tmp_path):
    hospital_map = _open_map(tmp_path)
    graph = build_graph(_line_skeleton(), np.ones((50, 50), dtype=np.float32), 0.1, hospital_map)
    assert sum(edge.length_m for edge in graph.edges) == pytest.approx(4.0, abs=0.5)


def test_graph_node_clearance_and_width(tmp_path):
    hospital_map = _open_map(tmp_path)
    distance = np.full((50, 50), 0.7, dtype=np.float32)
    graph = build_graph(_line_skeleton(), distance, 0.1, hospital_map)
    assert graph.nodes
    assert graph.nodes[0].clearance_m == pytest.approx(0.7)
    assert graph.nodes[0].channel_width_m == pytest.approx(1.4)


def test_start_attachment_returns_nearest_safe_node(tmp_path):
    hospital_map = _open_map(tmp_path)
    artifact = build_topology(hospital_map, FOOTPRINT, padding_m=0.0, safety_margin_m=0.0)
    attachment = attach_pose(artifact, [-1.5, 0.0, 0.0], FOOTPRINT, max_radius_m=5.0)
    assert attachment is not None
    assert attachment.distance_m <= 5.0


def test_goal_attachment_returns_nearest_safe_node(tmp_path):
    hospital_map = _open_map(tmp_path)
    artifact = build_topology(hospital_map, FOOTPRINT, padding_m=0.0, safety_margin_m=0.0)
    attachment = attach_pose(artifact, [1.5, 0.0, 0.0], FOOTPRINT, max_radius_m=5.0)
    assert attachment is not None


def test_component_labels_separate_disconnected_skeleton(tmp_path):
    hospital_map = _open_map(tmp_path)
    skeleton = np.zeros((50, 50), dtype=bool)
    skeleton[10, 5:20] = True
    skeleton[35, 30:45] = True
    graph = build_graph(skeleton, np.ones_like(skeleton, dtype=np.float32), 0.1, hospital_map)
    assert graph.components == 2


def test_topology_no_route_is_structured(tmp_path):
    hospital_map = _open_map(tmp_path)
    skeleton = np.zeros((50, 50), dtype=bool)
    skeleton[10, 5:20] = True
    skeleton[35, 30:45] = True
    artifact = TopologyArtifact(
        hospital_map, np.ones((50, 50), dtype=bool), skeleton,
        np.ones((50, 50), dtype=np.float32), np.ones((50, 50), dtype=np.int32),
        build_graph(skeleton, np.ones((50, 50), dtype=np.float32), 0.1, hospital_map), {},
    )
    assert search_topology(artifact, 0, 2) is None


def test_corridor_mask_contains_endpoints_and_free_route(tmp_path):
    hospital_map = _open_map(tmp_path)
    artifact = build_topology(hospital_map, FOOTPRINT, padding_m=0.0, safety_margin_m=0.0)
    start = hospital_map.world_to_cell(-1.5, 0.0)
    goal = hospital_map.world_to_cell(1.5, 0.0)
    start_attachment = attach_pose(artifact, [-1.5, 0.0, 0.0], FOOTPRINT)
    goal_attachment = attach_pose(artifact, [1.5, 0.0, 0.0], FOOTPRINT)
    route = search_topology(artifact, start_attachment.node_id, goal_attachment.node_id)
    mask = corridor_mask(artifact, route, start, goal, 0.5)
    assert mask[start] and mask[goal]
    assert np.all(mask <= artifact.free_mask)


def test_corridor_failure_is_distinct_from_full_grid(tmp_path):
    free = np.zeros((20, 20), dtype=bool)
    free[10, 1:19] = True
    start, goal = (10, 1), (10, 18)
    restricted = np.zeros_like(free)
    assert astar_grid(free, start, goal) is not None
    assert astar_grid(free, start, goal, restricted) is None


def test_astar_stats_account_for_search_space_and_expansions():
    free = np.zeros((20, 20), dtype=bool)
    free[10, 1:19] = True
    allowed = np.zeros_like(free)
    allowed[10, 1:19] = True
    result = astar_grid(free, (10, 1), (10, 18), allowed, resolution=0.1, return_stats=True)
    assert result.path is not None
    assert result.expanded_nodes > 0
    assert result.generated_nodes >= result.expanded_nodes
    assert result.max_open_set_size > 0
    assert result.allowed_grid_cells <= result.total_free_grid_cells
    assert result.search_space_ratio <= 1.0
    assert result.path_cost == pytest.approx(1.7)
    assert result.failure_code == ""


def test_astar_stats_report_structured_failure():
    free = np.zeros((10, 10), dtype=bool)
    free[5, 1:9] = True
    result = astar_grid(free, (5, 1), (5, 8), np.zeros_like(free), return_stats=True)
    assert result.path is None
    assert result.failure_code == "ENDPOINT_OUTSIDE_ALLOWED"
    assert result.expanded_nodes == 0


def test_astar_deadline_is_checked_inside_search_loop():
    free = np.ones((200, 200), dtype=bool)
    result = astar_grid(free, (0, 0), (199, 199), resolution=0.05, return_stats=True, timeout_s=0.0)
    assert result.path is None
    assert result.failure_code == "TIMEOUT"
    assert result.timeout_triggered is True
    assert result.timeout_checks >= 1


def test_corridor_padding_schedule_is_frozen():
    assert corridor_padding_schedule(1.0) == (1.0, 2.0, 4.0)
    with pytest.raises(ValueError):
        corridor_padding_schedule(0.5)


def test_full_grid_astar_fallback_finds_path(tmp_path):
    free = np.zeros((20, 20), dtype=bool)
    free[10, 1:19] = True
    assert astar_grid(free, (10, 1), (10, 18)) is not None


def test_topology_metadata_hash_round_trip_and_stale_rejection(tmp_path):
    hospital_map = _open_map(tmp_path)
    artifact = build_topology(hospital_map, FOOTPRINT, padding_m=0.0, safety_margin_m=0.0)
    output = tmp_path / "topology"
    save_topology(artifact, output)
    loaded = load_topology(output, hospital_map, FOOTPRINT, padding_m=0.0, safety_margin_m=0.0)
    assert loaded.metadata["footprint_hash"] == footprint_hash(FOOTPRINT)
    with pytest.raises(ValueError, match="stale"):
        load_topology(output, hospital_map, FOOTPRINT, padding_m=0.1, safety_margin_m=0.0)


def test_dynamic_obstacle_inputs_are_rejected():
    with pytest.raises(ValueError):
        _assert_static_cli(["--output-dir", "x", "tm_obstacles:=scenario"])
    _assert_static_cli(["--output-dir", "x", "--no-dynamic-obstacles"])


def test_static_footprint_collision_is_counted(tmp_path):
    pixels = np.full((40, 40), 254, dtype=np.uint8)
    pixels[20, 20] = 0
    hospital_map = _write_map(tmp_path, pixels)
    artifact = build_topology(hospital_map, FOOTPRINT, padding_m=0.0, safety_margin_m=0.0)
    poses = [{"x": -0.45, "y": -0.45, "yaw": 0.0}, {"x": 0.0, "y": 0.0, "yaw": 0.0}]
    assert static_collision_count(artifact, poses, FOOTPRINT) >= 1


def test_topology_failure_codes_are_present():
    assert "TOPOLOGY_BUILD_FAILED" in TOPOLOGY_FAILURE_CODES
    assert "FULL_GRID_FALLBACK" in TOPOLOGY_FAILURE_CODES
    assert "STATIC_FOOTPRINT_COLLISION" in TOPOLOGY_FAILURE_CODES
