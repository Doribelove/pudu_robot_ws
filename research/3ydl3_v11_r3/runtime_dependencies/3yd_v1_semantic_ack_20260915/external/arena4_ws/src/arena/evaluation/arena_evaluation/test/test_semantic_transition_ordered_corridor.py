from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from arena_evaluation import semantic_transition_ordered_corridor as ordered
from arena_evaluation.semantic_constraint_core import dense_interpolate
from arena_evaluation.semantic_transition_ordered_corridor import (
    CorridorFailure,
    OrderedCorridorPolicy,
    OrientedRoute,
    YAW_BIN_COUNT,
    _candidate_cells,
    _controls_for_label,
    _write_json,
    audit_route_progress,
    build_candidate_layers,
    build_graph,
    search_complete_paths,
)
from arena_evaluation.semantic_transition_lazy_corridor import (
    LazyEdgeFactory, LazySearchPolicy,
)


class FakeMap:
    resolution = 0.25
    full_origin = (0.0, 0.0)
    full_height = 80
    row0 = 0
    col0 = 0
    height = 80
    width = 80

    def __init__(self):
        self.distance_m = np.full((self.height, self.width), 10.0, dtype=np.float32)

    def world_to_cell(self, x, y):
        col = int(np.floor(float(x) / self.resolution))
        row = self.full_height - 1 - int(np.floor(float(y) / self.resolution))
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None


class FakeWorld:
    def __init__(self, *, target=True):
        shape = (80, 80)
        self.map = FakeMap()
        self.start = (2.0, 10.0, 0.0)
        self.goal = (8.0, 10.0, 0.0)
        self.selected = [7]
        self.master = np.zeros(shape, dtype=np.uint8)
        self.grids = {
            "labels": np.full(shape, 7, dtype=np.int16),
            "allowed": np.ones(shape, dtype=bool),
            "hard": np.zeros(shape, dtype=bool),
            "correct": np.ones(shape, dtype=bool),
            "error": np.full(shape, 0.1 if target else 1.0, dtype=np.float32),
        }
        self.meta = {"route_polyline": [[2.0, 10.0], [5.0, 10.0], [8.0, 10.0]]}
        self.bound_length = 40.0

    def cells(self, samples):
        poses = np.asarray(samples)
        cols = np.floor(poses[:, 0] / self.map.resolution).astype(np.int64)
        rows = self.map.full_height - 1 - np.floor(poses[:, 1] / self.map.resolution).astype(np.int64)
        inside = (rows >= 0) & (rows < self.map.height) & (cols >= 0) & (cols < self.map.width)
        return rows, cols, inside

    def collision_free(self, samples):
        rows, cols, inside = self.cells(samples)
        return bool(np.all(inside) and np.all(self.master[rows, cols] < 253))

    def semantic_counts(self, samples):
        rows, cols, inside = self.cells(samples)
        if not np.all(inside) or np.any(self.grids["labels"][rows, cols] != 7):
            return None
        correct = self.grids["correct"][rows, cols]
        target = correct & (self.grids["error"][rows, cols] <= 0.5)
        return len(rows), int(correct.sum()), int(target.sum())

    def validate_edge(self, edge, *, dense=False):
        counts = self.semantic_counts(edge.samples)
        path = dense_interpolate(np.vstack((edge.start, edge.samples))) if dense else edge.samples
        if counts is None or not self.collision_free(path):
            return False
        edge.n, edge.correct, edge.target = counts
        return True


def small_policy(**changes):
    values = dict(
        station_spacing_m=1.0,
        lateral_sample_spacing_m=0.5,
        maximum_lateral_probe_m=0.5,
        maximum_cells_per_station=1,
        yaw_neighbor_bins=(0,),
        maximum_station_skip=1,
        maximum_local_edge_length_m=1.5,
        maximum_local_edge_ratio=2.0,
        maximum_labels_per_node=2,
    )
    values.update(changes)
    return OrderedCorridorPolicy(**values)


def test_oriented_route_reverses_and_attaches_exact_query_endpoints():
    route = OrientedRoute([[8.1, 10.0], [5.0, 10.0], [2.1, 10.0]], (2.0, 10.0), (8.0, 10.0))
    assert route.reversed_for_query
    assert np.array_equal(route.points[0], [2.0, 10.0])
    assert np.array_equal(route.points[-1], [8.0, 10.0])
    assert route.length_m == pytest.approx(6.0)


def test_route_binding_fails_closed_when_endpoint_is_not_attached():
    with pytest.raises(CorridorFailure, match="attachment") as error:
        OrientedRoute([[4.0, 10.0], [8.0, 10.0]], (2.0, 10.0), (8.0, 10.0), 0.75)
    assert error.value.code == "ROUTE_BINDING_FAILED"


def test_signed_projection_catches_goal_overshoot_and_return_without_intersection():
    route = OrientedRoute([[0.0, 0.0], [10.0, 0.0]], (0.0, 0.0), (10.0, 0.0))
    # Parallel-adjacent return geometry evades an exact segment-intersection test.
    path = [[0.0, 0.0, 0.0], [12.0, 0.0, 0.0], [12.0, 0.1, np.pi], [10.0, 0.1, np.pi]]
    result = audit_route_progress(route, path)
    assert not result["ordered_progress_gate_passed"]
    assert result["terminal_station_overshoot_m"] == pytest.approx(2.0)
    assert result["backward_route_progress_m"] == pytest.approx(2.0)
    assert result["failure_codes"] == ["TERMINAL_STATION_OVERSHOOT", "ROUTE_STATION_REGRESSION"]


def test_local_station_window_matches_global_on_non_self_near_route():
    route = OrientedRoute(
        [[0.0, 0.0], [2.0, 0.0], [4.0, 1.0], [6.0, 1.0]],
        [0.0, 0.0, 0.0], [6.0, 1.0, 0.0],
    )
    points = np.asarray([[2.1, 0.1], [3.0, 0.5], [4.1, 0.9]])
    global_projection = route.project(points)
    local_projection = route.project_station_window(points, 1.5, 5.0)
    assert local_projection[0] == pytest.approx(global_projection[0])
    assert local_projection[1] == pytest.approx(global_projection[1])


def test_progress_audit_accepts_precomputed_projection_without_reprojection(monkeypatch):
    route = OrientedRoute([[0.0, 0.0], [2.0, 0.0]], [0, 0, 0], [2, 0, 0])
    path = np.asarray([[0.2, 0.0], [1.0, 0.0]])
    projection = route.project(path[:, :2])
    monkeypatch.setattr(
        route, "project",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError),
    )
    result = audit_route_progress(route, path, projection=projection)
    assert result["ordered_progress_gate_passed"] is True


def test_candidate_layers_are_same_lane_ordered_and_use_48_bin_yaws():
    world = FakeWorld()
    route = OrientedRoute(world.meta["route_polyline"], world.start, world.goal)
    lane, states, layers, diagnostics = build_candidate_layers(world, route, small_policy())
    assert lane == 7
    assert diagnostics["target_candidate_layer_ratio_upper_bound"] == 1.0
    assert all(layers)
    assert all(state.lane_label == lane for state in states)
    assert all(0 <= state.yaw_bin < YAW_BIN_COUNT for state in states)
    assert [states[layer[0]].station_m for layer in layers] == sorted(
        states[layer[0]].station_m for layer in layers
    )


def test_candidate_pruning_keeps_lateral_transition_ladder_to_target():
    world = FakeWorld(target=False)
    # For a +x route the right-normal probe moves toward decreasing y.
    for row in range(world.map.height):
        y = (world.map.full_height - row - 0.5) * world.map.resolution
        if y <= 8.25:
            world.grids["error"][row] = 0.1
    route = OrientedRoute(world.meta["route_polyline"], world.start, world.goal)
    policy = small_policy(
        maximum_lateral_probe_m=3.0,
        lateral_sample_spacing_m=0.25,
        maximum_cells_per_station=5,
    )
    cells = _candidate_cells(world, route.sample(2.0), 7, policy)
    offsets = sorted(item["offset"] for item in cells)
    assert len(offsets) == 5
    assert min(abs(value) for value in offsets) <= 0.25
    assert max(offsets) >= 1.75
    assert max(np.diff(offsets)) <= 0.75


def test_no_target_state_returns_explicit_monotone_corridor_failure():
    world = FakeWorld(target=False)
    route = OrientedRoute(world.meta["route_polyline"], world.start, world.goal)
    with pytest.raises(CorridorFailure) as error:
        build_candidate_layers(world, route, small_policy())
    assert error.value.code == "NO_MONOTONE_TARGET_CORRIDOR"


def test_graph_edges_are_fully_validated_progressive_and_deterministic():
    world = FakeWorld()
    route = OrientedRoute(world.meta["route_polyline"], world.start, world.goal)
    first = build_graph(world, route, small_policy())
    second = build_graph(world, route, small_policy())
    first_summary = [(e.source, e.target, e.dubins_choice, e.length_m) for e in first.edges]
    second_summary = [(e.source, e.target, e.dubins_choice, e.length_m) for e in second.edges]
    assert first_summary == second_summary
    assert first.edges
    for edge in first.edges:
        assert edge.lane_samples > 0
        assert edge.correct_samples == edge.lane_samples
        assert edge.target_samples == edge.lane_samples
        dense = dense_interpolate(np.vstack((edge.control.start, edge.control.samples)))
        assert audit_route_progress(route, dense)["ordered_progress_gate_passed"]


def test_finite_dag_search_reaches_goal_and_reconstructs_exact_controls():
    world = FakeWorld()
    route = OrientedRoute(world.meta["route_polyline"], world.start, world.goal)
    graph = build_graph(world, route, small_policy())
    candidates, diagnostics = search_complete_paths(graph, small_policy())
    assert candidates
    label, labels = candidates[0]
    controls = _controls_for_label(label, labels, graph)
    assert controls[0].start == world.start
    assert controls[-1].goal == world.goal
    assert diagnostics["semantic_first"]["complete_label_count"] > 0
    assert diagnostics["semantic_first"]["stored_label_count"] <= (
        len(graph.states) * small_policy().maximum_labels_per_node
    )


def test_lazy_factory_materializes_only_bounded_deterministic_successors():
    world = FakeWorld()
    route = OrientedRoute(world.meta["route_polyline"], world.start, world.goal)
    policy = small_policy(maximum_station_skip=2)
    first = LazyEdgeFactory(
        world, route, policy, valid_successors_per_target_layer=2,
    )
    second = LazyEdgeFactory(
        world, route, policy, valid_successors_per_target_layer=2,
    )
    assert first.diagnostics()["materialized_edge_count"] == 0
    start = first.layers[0][0]
    a = first.successors(start)
    b = second.successors(second.layers[0][0])
    assert [(e.target_layer, e.target, e.dubins_choice) for e in a] == [
        (e.target_layer, e.target, e.dubins_choice) for e in b
    ]
    counts = {}
    for edge in a:
        counts[edge.target_layer] = counts.get(edge.target_layer, 0) + 1
    assert all(value <= 2 for value in counts.values())
    assert first.successors(start) is a
    assert first.diagnostics()["successor_cache_hit_count"] == 1


def test_lazy_policy_rejects_nonpositive_resource_bounds():
    with pytest.raises(ValueError):
        LazySearchPolicy(valid_successors_per_target_layer=0)


def test_write_once_json_refuses_overwrite(tmp_path: Path):
    target = tmp_path / "record.json"
    _write_json(target, {"first": True})
    with pytest.raises(FileExistsError):
        _write_json(target, {"second": True})


def test_source_contains_no_frozen_positive_coordinates_or_saved_path_loader():
    source = Path(__file__).parents[1] / "arena_evaluation/semantic_transition_ordered_corridor.py"
    text = source.read_text()
    assert "-25.750998" not in text
    assert "WITNESSES" not in text
    assert "saved_path" not in text
    assert '"used_historical_paths": False' in text


def test_final_evaluation_requires_goal_plane_naturalness_screen(monkeypatch):
    world = FakeWorld()
    route = OrientedRoute(world.meta["route_polyline"], world.start, world.goal)
    path = np.asarray([world.start, world.goal], dtype=float)
    world.audit = lambda controls: ({
        "canonical": {"final_valid_success": True},
        "padded_effective_master_collision_free": True,
        "exact_endpoint_xy_yaw": True,
        "edge_continuity": True,
        "trace_replay_exact": True,
        "same_lane_instance": True,
        "maximum_control_curvature_1pm": 0.0,
        "arc_length_m": 6.0,
    }, path)
    monkeypatch.setattr(ordered, "semantic_metrics", lambda world, points: {
        "semantic_gate_passed": True,
    })
    monkeypatch.setattr(ordered, "explicit_hard_audit", lambda world, points, semantic_map: {
        "hard_feature_gate_passed": True,
    })
    monkeypatch.setattr(ordered, "audit_revisits", lambda path: {
        "revisit_screen_passed": True,
    })
    monkeypatch.setattr(ordered, "audit_path_necessity", lambda *args, **kwargs: {
        "input_complete": True,
        "audit_passed": False,
        "failure_codes": ["GOAL_PLANE_OVERSHOOT"],
    })
    result = ordered.evaluate_controls(
        world,
        route,
        [SimpleNamespace(certificate=lambda: {})],
        SimpleNamespace(),
    )
    assert not result["gate_passed"]
    assert "NATURALNESS_SCREEN" in result["failure_codes"]
    assert result["naturalness"]["failure_codes"] == ["GOAL_PLANE_OVERSHOOT"]
