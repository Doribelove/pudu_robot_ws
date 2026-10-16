import numpy as np

from arena_evaluation.dynamic_snapshot import DynamicSnapshot
from arena_3d_v1.dynamic_policy import DynamicGridConfirmation, RelevanceScheduler


def snapshot(index, occupied, *, map_version="map-v1"):
    return DynamicSnapshot.from_cells(
        f"S{index}", occupied, timestamp=float(index),
        map_version=map_version, map_shape=(20, 20),
    )


def test_two_observation_confirmation_and_recovery():
    guard = DynamicGridConfirmation(
        map_version="map-v1", map_shape=(20, 20), inflation_radius_cells=0,
    )
    first = guard.consume(snapshot(1, [(10, 10)]), now=1.0)
    assert first.effective_changed_cells == ()
    second = guard.consume(snapshot(2, [(10, 10)]), now=2.0)
    assert second.newly_blocked_cells == ((10, 10),)
    third = guard.consume(snapshot(3, []), now=3.0)
    assert third.effective_changed_cells == ()
    assert (10, 10) in third.blocked_cells
    fourth = guard.consume(snapshot(4, []), now=4.0)
    assert fourth.newly_freed_cells == ((10, 10),)
    assert (10, 10) not in fourth.blocked_cells


def test_rejected_snapshot_is_atomic():
    guard = DynamicGridConfirmation(
        map_version="map-v1", map_shape=(20, 20), inflation_radius_cells=0,
    )
    guard.consume(snapshot(1, [(5, 5)]), now=1.0)
    rejected = guard.consume(snapshot(1, [(6, 6)]), now=1.0)
    assert not rejected.accepted
    assert rejected.rejection_reason == "OUT_OF_ORDER_SNAPSHOT"
    assert rejected.diagnostics["state_mutated"] is False


def test_scheduler_skips_only_safe_cost_increases_and_runs_recovery():
    corridor = np.zeros((20, 20), dtype=bool)
    corridor[5:15, 2:18] = True
    scheduler = RelevanceScheduler(corridor)
    path = [(10, column) for column in range(3, 17)]
    guard = DynamicGridConfirmation(
        map_version="map-v1", map_shape=(20, 20), inflation_radius_cells=0,
    )
    guard.consume(snapshot(1, [(6, 4)]), now=1.0)
    off_path = guard.consume(snapshot(2, [(6, 4)]), now=2.0)
    decision = scheduler.decide(off_path, path)
    assert not decision.invoke_l2
    assert decision.reason == "OFF_PATH_COST_INCREASE"
    guard.consume(snapshot(3, []), now=3.0)
    recovered = guard.consume(snapshot(4, []), now=4.0)
    decision = scheduler.decide(recovered, path)
    assert decision.invoke_l2
    assert decision.reason == "RECOVERY_REQUIRES_OPTIMALITY_REPAIR"


def test_scheduler_detects_path_support_and_outside_corridor():
    corridor = np.zeros((20, 20), dtype=bool)
    corridor[5:15, 2:18] = True
    scheduler = RelevanceScheduler(corridor)
    path = [(10, column) for column in range(3, 17)]
    guard = DynamicGridConfirmation(
        map_version="map-v1", map_shape=(20, 20), inflation_radius_cells=0,
    )
    guard.consume(snapshot(1, [(10, 9)]), now=1.0)
    affected = guard.consume(snapshot(2, [(10, 9)]), now=2.0)
    assert scheduler.decide(affected, path).invoke_l2

    second = DynamicGridConfirmation(
        map_version="map-v1", map_shape=(20, 20), inflation_radius_cells=0,
    )
    second.consume(snapshot(1, [(1, 1)]), now=1.0)
    outside = second.consume(snapshot(2, [(1, 1)]), now=2.0)
    assert scheduler.decide(outside, path).reason == "CHANGE_OUTSIDE_ACTIVE_CORRIDOR"
