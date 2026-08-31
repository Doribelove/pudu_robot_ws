import numpy as np

from arena_evaluation.dstar_lite import DStarLite


def test_dstar_lite_finds_path_and_repairs_local_change():
    free = np.ones((9, 9), dtype=bool)
    free[4, 1:8] = False
    free[4, 4] = True
    planner = DStarLite(free, (8, 0), (0, 8))
    first = planner.compute_shortest_path()
    path = planner.extract_path()
    assert path is not None
    assert first.no_path is False
    assert first.expanded_nodes > 0

    # Block the current route's gap.  D* Lite retains its state and updates
    # only that cell and its neighbours.
    free[4, 4] = False
    affected = planner.update_cells([(4, 4)], traversable=free)
    second = planner.compute_shortest_path()
    repaired = planner.extract_path()
    assert affected <= 9
    assert repaired is not None
    assert (4, 4) not in repaired
    # The detour can legitimately require more expansions than the original
    # straight-through gap; the important property is that the state repairs
    # without rebuilding the planner and returns a valid path.
    assert planner.update_count >= 1
    state = planner.state_snapshot()
    assert state["goal"] == [0, 8]
    assert "priority_queue_size" in state


def test_dstar_lite_dynamic_cell_can_be_reopened():
    free = np.ones((7, 7), dtype=bool)
    planner = DStarLite(free, (6, 0), (0, 6))
    planner.compute_shortest_path()
    free[3, 3] = False
    planner.update_cells([(3, 3)], traversable=free)
    planner.compute_shortest_path()
    free[3, 3] = True
    planner.update_cells([(3, 3)], traversable=free)
    planner.compute_shortest_path()
    assert planner.extract_path() is not None


def test_dstar_lite_reports_bounded_timeout():
    planner = DStarLite(np.ones((100, 100), dtype=bool), (99, 0), (0, 99))
    stats = planner.compute_shortest_path(max_expansions=1)
    assert stats.timeout_triggered is True
    assert stats.no_path is True
