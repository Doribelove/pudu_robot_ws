from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from arena_evaluation import semantic_applicability_v3 as subject
from arena_evaluation import semantic_applicability_replay_v3 as replay
from arena_evaluation.semantic_transition_ordered_corridor import CorridorState


def _state(state_id, layer, *, target=False, station=None, error=1.0):
    return CorridorState(
        state_id=state_id,
        layer_index=layer,
        station_m=float(layer if station is None else station),
        x=float(layer),
        y=0.0,
        yaw=0.0,
        yaw_bin=0,
        lane_label=4,
        lateral_offset_m=0.0,
        error_m=float(error),
        correct_side=bool(target),
        target_band=bool(target),
        endpoint="start" if layer == 0 else ("goal" if layer == 3 else ""),
    )


def _edge(edge_id, source, target, source_layer, target_layer, length=1.0):
    return SimpleNamespace(
        edge_id=edge_id,
        source=source,
        target=target,
        source_layer=source_layer,
        target_layer=target_layer,
        length_m=float(length),
    )


def _graph(states, layers, edges, *, route_length=3.0):
    outgoing = {state.state_id: [] for state in states}
    for edge in edges:
        outgoing[edge.source].append(edge.edge_id)
    return SimpleNamespace(
        states=states,
        layers=layers,
        edges=edges,
        outgoing=outgoing,
        route=SimpleNamespace(length_m=float(route_length)),
    )


def test_target_chain_requires_start_reach_and_goal_coreach():
    states = [_state(0, 0), _state(1, 1, target=True), _state(2, 2), _state(3, 3)]
    graph = _graph(states, [[0], [1], [2], [3]], [
        _edge(0, 0, 1, 0, 1),
        _edge(1, 1, 2, 1, 2),
        _edge(2, 2, 3, 2, 3),
    ])
    edge_ids, target, diagnostics = subject._target_chain(graph)
    assert edge_ids == [0, 1, 2]
    assert target == 1
    assert diagnostics["reachable_coreachable_target_state_count"] == 1

    disconnected = _graph(states, [[0], [1], [2], [3]], [
        _edge(0, 0, 1, 0, 1),
        _edge(1, 2, 3, 2, 3),
    ])
    assert subject._target_chain(disconnected) is None


def test_target_chain_rejects_target_after_goal_plane():
    states = [_state(0, 0), _state(1, 1, target=True, station=3.2), _state(2, 3)]
    graph = _graph(states, [[0], [1], [2]], [
        _edge(0, 0, 1, 0, 1),
        _edge(1, 1, 2, 1, 2),
    ], route_length=3.0)
    assert subject._target_chain(graph) is None


def test_target_chain_tie_break_is_deterministic_and_midroute_first():
    states = [
        _state(0, 0),
        _state(1, 1, target=True, station=0.8, error=0.1),
        _state(2, 1, target=True, station=1.5, error=0.4),
        _state(3, 3),
    ]
    edges = [
        _edge(0, 0, 1, 0, 1), _edge(1, 1, 3, 1, 3),
        _edge(2, 0, 2, 0, 1), _edge(3, 2, 3, 1, 3),
    ]
    graph = _graph(states, [[0], [1, 2], [], [3]], edges)
    assert subject._target_chain(graph)[:2] == ([2, 3], 2)


def test_undirected_route_hash_is_direction_invariant():
    route = [[0.0, 0.0], [1.0, 0.5], [2.0, 1.0]]
    assert subject._undirected_route_hash(route) == subject._undirected_route_hash(list(reversed(route)))


def test_endpoint_yaws_use_route_not_direct_endpoint_chord():
    start, goal = subject._endpoint_yaws([[0.0, 0.0], [0.0, 1.0], [1.0, 1.0]])
    assert start == pytest.approx(0.5 * 3.141592653589793)
    assert goal == pytest.approx(0.0)


def test_frozen_pool_ranking_uses_only_permitted_geometry_fields():
    config, _ = subject._load_config(subject.DEFAULT_CONFIG)
    ranked = subject._ranked_pool(config)
    assert ranked
    assert ranked[0]["candidate_id"] == "mirror-candidate-006"
    assert set(ranked[0]) == {
        "candidate_id", "lane_label", "lane_semantic_id", "endpoint_node_ids",
        "minimum_endpoint_clearance_m", "endpoint_euclidean_separation_m", "directions",
    }
    assert set(ranked[0]["directions"][0]) == {"direction", "start_xy", "goal_xy"}


def test_legacy_positive_is_retained_not_applicable_and_not_success():
    config, _ = subject._load_config(subject.DEFAULT_CONFIG)
    result = subject._classify_legacy_positive(config)
    assert result["status"] == "NOT_APPLICABLE"
    assert result["counts_as_success"] is False
    assert result["retained_as_negative_regression"] is True
    assert result["reachable_target_before_goal_count"] == 0
    assert result["reachable_target_on_goal_plane_count"] == 0
    assert result["continuous_space_infeasibility_proof"] is False


def test_ordered_policy_keeps_immutable_vehicle_limits():
    config, _ = subject._load_config(subject.DEFAULT_CONFIG)
    policy = subject._ordered_policy(config)
    assert policy.turning_radius_m > 0.40
    assert policy.endpoint_attachment_limit_m == 2.0
    assert policy.yaw_neighbor_bins == (-1, 0, 1)


def test_map_cell_slab_keeps_high_clearance_raw_target():
    shape = (9, 9)

    class Map:
        resolution = 0.05
        full_origin = (0.0, 0.0)
        full_height = 9
        row0 = col0 = 0
        height = width = 9
        distance_m = np.ones(shape, dtype=float)

        @staticmethod
        def world_to_cell(x, y):
            col = int(np.floor(x / 0.05))
            row = 8 - int(np.floor(y / 0.05))
            return (row, col) if 0 <= row < 9 and 0 <= col < 9 else None

    labels = np.ones(shape, dtype=np.int32) * 4
    allowed = np.ones(shape, dtype=bool)
    hard = np.zeros(shape, dtype=bool)
    error = np.ones(shape, dtype=float)
    correct = np.zeros(shape, dtype=bool)
    error[4, 6] = 0.10
    error[4, 7] = 0.40
    correct[4, 6:8] = True
    Map.distance_m[4, 6] = 0.25
    Map.distance_m[4, 7] = 0.80
    world = SimpleNamespace(
        map=Map(),
        grids={"labels": labels, "allowed": allowed, "hard": hard,
               "error": error, "correct": correct},
        master=np.zeros(shape, dtype=np.uint8),
    )
    sample = SimpleNamespace(x=0.225, y=0.225, tangent_x=1.0, tangent_y=0.0)
    policy = subject.OrderedCorridorPolicy(
        maximum_lateral_probe_m=1.0,
        maximum_cells_per_station=3,
    )
    values = subject._map_cell_candidate_cells(
        world, sample, 4, policy, station_slab_half_width_m=0.2,
    )
    targets = [item for item in values if item["target"]]
    assert targets
    assert targets[0]["clearance"] == pytest.approx(0.80)


def test_write_json_is_write_once(tmp_path: Path):
    path = tmp_path / "value.json"
    subject._write_json(path, {"ok": True})
    with pytest.raises(FileExistsError):
        subject._write_json(path, {"ok": False})


def test_replay_deterministic_view_excludes_only_graph_timing():
    certificate = {
        "applicable": True,
        "graph": {"graph_build_wall_s": 12.0, "state_count": 4},
        "path_sha256": "abc",
    }
    view = replay._deterministic_view(certificate)
    assert view["graph"] == {"state_count": 4}
    assert certificate["graph"]["graph_build_wall_s"] == 12.0
