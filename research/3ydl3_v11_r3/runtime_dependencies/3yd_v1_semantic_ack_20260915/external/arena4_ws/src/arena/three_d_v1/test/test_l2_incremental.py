import numpy as np

from arena_3d_v1.l2_incremental import (
    CorridorROI,
    PersistentCorridorDStar,
    deterministic_grid_astar,
)


def make_roi(free, start, goal):
    corridor = np.ones_like(free, dtype=bool)
    return CorridorROI.from_global(
        free, corridor, start, goal,
        binding_fields={
            "map_hash": "map-v1",
            "map_origin": (0.0, 0.0, 0.0),
            "resolution": 0.05,
            "topology_hash": "topology-v1",
            "route_edge_ids": ("1", "2"),
            "footprint_hash": "jackal",
        },
    )


def test_corner_cutting_is_forbidden_for_dstar_and_oracle():
    free = np.ones((3, 3), dtype=bool)
    free[1, 0] = False
    free[2, 1] = False
    roi = make_roi(free, (2, 0), (0, 2))
    planner = PersistentCorridorDStar(roi)
    dstar = planner.initialize(verify_oracle=True)
    astar = deterministic_grid_astar(free, (2, 0), (0, 2))
    assert not dstar.success
    assert astar.path is None


def test_persistent_dstar_repairs_in_place_and_matches_cold_astar():
    free = np.ones((31, 41), dtype=bool)
    free[15, :] = False
    free[15, 8:13] = True
    free[15, 28:33] = True
    roi = make_roi(free, (28, 4), (2, 36))
    planner = PersistentCorridorDStar(
        roi, dstar_wall_budget_ms=1000.0, dstar_max_expansions=100_000,
    )
    initial = planner.initialize(verify_oracle=True)
    assert initial.success
    identity = id(planner.planner)
    blocked = [roi.to_global((15, column)) for column in range(8, 13)]
    repaired = planner.update(blocked, verify_oracle=True)
    assert repaired.success
    assert repaired.selected_backend == "persistent_dstar"
    assert repaired.state_reused
    assert id(planner.planner) == identity
    assert not set(repaired.path or ()).intersection(blocked)
    assert repaired.oracle_cost_error == 0.0


def test_timeout_fallback_never_returns_partial_dstar_result():
    free = np.ones((61, 61), dtype=bool)
    roi = make_roi(free, (55, 5), (5, 55))
    planner = PersistentCorridorDStar(
        roi, dstar_wall_budget_ms=0.01, dstar_max_expansions=1,
    )
    assert planner.initialize().success
    path = planner.path_global
    assert path
    blocked = [path[len(path) // 2]]
    result = planner.update(blocked)
    assert result.success
    assert result.selected_backend == "deterministic_grid_astar_fallback"
    assert result.partial_dstar_result_returned is False
    assert planner.dstar_ready is False
    resync = planner.service_resync()
    assert resync.success
    assert planner.dstar_ready is True


def test_binding_changes_when_route_or_corridor_changes():
    free = np.ones((11, 11), dtype=bool)
    first = make_roi(free, (10, 0), (0, 10))
    corridor = np.ones_like(free)
    corridor[5, 0] = False
    second = CorridorROI.from_global(
        free, corridor, (10, 0), (0, 10),
        binding_fields={
            "map_hash": "map-v1", "map_origin": (0.0, 0.0, 0.0),
            "resolution": 0.05, "topology_hash": "topology-v1",
            "route_edge_ids": ("different",), "footprint_hash": "jackal",
        },
    )
    assert first.binding.digest != second.binding.digest
