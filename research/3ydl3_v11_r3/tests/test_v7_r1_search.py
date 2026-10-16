"""Behavior parity and reachability tests for the new goal-stop kernel."""
import numpy as np
import pytest
from scipy.sparse.csgraph import dijkstra

from three_yd_v5.free_space_topology import FreeSpaceTopology
from three_yd_v5.goal_search import goal_dijkstra


def path(pred, src, dst):
    result = [dst]
    while result[-1] != src:
        result.append(int(pred[result[-1]]))
        assert result[-1] >= 0 and len(result) <= len(pred)
    return result[::-1]


@pytest.mark.parametrize('seed', range(24))
def test_same_distance_and_equal_cost_path_as_r0(seed):
    rng = np.random.default_rng(seed)
    free = rng.random((23, 31)) > (seed % 4) * .09
    graph = FreeSpaceTopology._compile_graph(free)
    vertices = np.flatnonzero(free)
    for src, dst in rng.choice(vertices, (12, 2)):
        original, old_pred = dijkstra(graph, indices=int(src), return_predecessors=True)
        distance, pred, settled = goal_dijkstra(graph, src, dst)
        assert distance == original[dst]
        assert settled <= int(np.isfinite(original).sum())
        if np.isfinite(distance):
            assert path(pred, src, dst) == path(old_pred, src, dst)


def test_near_goal_stops_before_full_component():
    graph = FreeSpaceTopology._compile_graph(np.ones((40, 40), bool))
    distance, pred, settled = goal_dijkstra(graph, 0, 1)
    assert distance == 1 and path(pred, 0, 1) == [0, 1]
    assert settled < 10


def test_disconnected_does_not_cut_diagonal():
    graph = FreeSpaceTopology._compile_graph(np.eye(3, dtype=bool))
    distance, _, settled = goal_dijkstra(graph, 0, 8)
    assert not np.isfinite(distance) and settled == 1


def test_bad_endpoints_rejected_without_native_array_read():
    graph = FreeSpaceTopology._compile_graph(np.ones((3, 3), bool))
    with pytest.raises(ValueError, match='INVALID_INPUT'):
        goal_dijkstra(graph, -1, 5)


def test_csr_type_is_checked():
    graph = FreeSpaceTopology._compile_graph(np.ones((3, 3), bool)).astype(np.float64)
    with pytest.raises(ValueError, match='FLOAT32_INT32'):
        goal_dijkstra(graph, 0, 5)
