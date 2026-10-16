import math

import numpy as np

from arena_evaluation.dynamic_snapshot import DynamicSnapshot
from arena_3d_v1.l2_incremental import CorridorROI
from arena_3d_v1.pipeline import L1Plan
from arena_3d_v1.r2_pipeline import Layered3DV1R2Controller
from arena_3d_v1.r2_state_lifecycle import R2L2StateLifecycleManager
from arena_3d_v1.stable_pipeline import Layered3DV1StableController


def make_plan(route="route-a", corridor=None):
    free = np.ones((17, 25), dtype=bool)
    if corridor is None:
        corridor = np.ones_like(free)
    return L1Plan(
        static_safe_free=free,
        corridor_mask=np.asarray(corridor, dtype=bool),
        start_cell=(14, 2),
        goal_cell=(2, 22),
        map_hash="map-v1",
        map_origin=(0.0, 0.0, 0.0),
        resolution=0.05,
        topology_hash="topology-v1",
        route_edge_ids=(route,),
        footprint_hash="jackal",
        route_signature=route,
    )


def roi(plan):
    return CorridorROI.from_global(
        plan.static_safe_free, plan.corridor_mask,
        plan.start_cell, plan.goal_cell, binding_fields=plan.binding_fields(),
    )


def snapshot(index, occupied):
    return DynamicSnapshot.from_cells(
        f"S{index}", occupied, timestamp=float(index),
        map_version="map-v1", map_shape=(17, 25),
    )


def path_cost(path):
    if not path:
        return math.inf
    return sum(
        math.sqrt(2.0) if a[0] != b[0] and a[1] != b[1] else 1.0
        for a, b in zip(path, path[1:])
    )


def test_stable_wrapper_is_behaviorally_equivalent_to_frozen_r2(tmp_path):
    plan = make_plan()
    first_cache = tmp_path / "r2"
    second_cache = tmp_path / "stable"
    first_manager = R2L2StateLifecycleManager(first_cache)
    second_manager = R2L2StateLifecycleManager(second_cache)
    assert first_manager.prebuild(roi(plan), verify_oracle=True).success
    assert second_manager.prebuild(roi(plan), verify_oracle=True).success
    frozen = Layered3DV1R2Controller(
        plan, cache_root=first_cache, lifecycle_manager=first_manager,
        dynamic_inflation_radius_cells=0, verify_l2_oracle=True,
    )
    stable = Layered3DV1StableController(
        plan, cache_root=second_cache, lifecycle_manager=second_manager,
        dynamic_inflation_radius_cells=0, verify_l2_oracle=True,
    )
    source = frozen.l2.path_global[len(frozen.l2.path_global) // 2]
    events = [
        snapshot(1, [source]),
        snapshot(2, [source]),
        snapshot(3, [source]),
        snapshot(4, []),
        snapshot(5, []),
    ]
    for event in events:
        old = frozen.process_snapshot(event, now=event.timestamp)
        new = stable.process_snapshot(event, now=event.timestamp)
        assert new.scheduler.reason == old.scheduler.reason
        assert new.scheduler.invoke_l2 == old.scheduler.invoke_l2
        assert new.failure_code == old.failure_code
        assert new.route_signature == old.route_signature
        if old.l2_result is None:
            assert new.l2_result is None
            assert new.diagnostics["selected_backend"] == "scheduler_skip"
            continue
        assert new.l2_result.selected_backend == old.l2_result.selected_backend
        assert new.l2_result.success == old.l2_result.success
        assert new.l2_result.partial_dstar_result_returned is False
        assert path_cost(new.l2_result.path) == path_cost(old.l2_result.path)
        assert new.l2_result.path == old.l2_result.path
        assert new.diagnostics["production_baseline_id"] == "3D-V1-r2-stable"
    assert stable.lifecycle.synchronous_dstar_build_count == 0


def _corridor(rows):
    mask = np.zeros((17, 25), dtype=bool)
    for first, second in zip(rows, rows[1:]):
        row, column = first
        target_row, target_column = second
        while (row, column) != (target_row, target_column):
            mask[row, column] = True
            row += (target_row > row) - (target_row < row)
            column += (target_column > column) - (target_column < column)
        mask[row, column] = True
    return mask


def test_l2_no_route_rebinds_changed_l1_route_or_returns_l1_no_route(tmp_path):
    first_mask = _corridor([(14, 2), (14, 22), (2, 22)])
    second_mask = _corridor([(14, 2), (2, 2), (2, 22)])
    first = make_plan("route-a", first_mask)
    second = make_plan("route-b", second_mask)
    controller = Layered3DV1StableController(
        first, cache_root=PathLikeMissing(tmp_path / "miss-a"),
        dynamic_inflation_radius_cells=0, verify_l2_oracle=True,
    )
    blocker = (14, 12)
    controller.process_snapshot(snapshot(1, [blocker]), now=1.0)
    rerouted = controller.process_snapshot(
        snapshot(2, [blocker]), l1_replan=lambda _blocked: second, now=2.0,
    )
    assert rerouted.l1_graph_astar_called
    assert rerouted.l1_reroute_succeeded
    assert rerouted.route_signature == "route-b"
    assert rerouted.l2_result.success
    assert controller.lifecycle.synchronous_dstar_build_count == 0

    failed = Layered3DV1StableController(
        first, cache_root=PathLikeMissing(tmp_path / "miss-b"),
        dynamic_inflation_radius_cells=0, verify_l2_oracle=True,
    )
    failed.process_snapshot(snapshot(1, [blocker]), now=1.0)
    no_route = failed.process_snapshot(
        snapshot(2, [blocker]), l1_replan=lambda _blocked: None, now=2.0,
    )
    assert no_route.l1_graph_astar_called
    assert no_route.failure_code == "L1_NO_ROUTE"
    assert not no_route.l3_required


def PathLikeMissing(path):
    # A normal empty directory exercises the same verified-cache miss path.
    return path
