import numpy as np
import pytest

from arena_evaluation.dynamic_snapshot import DynamicSnapshot, apply_dynamic_snapshot, path_intersects_snapshot


def test_snapshot_is_deterministic_and_does_not_mutate_static_map():
    base = np.ones((8, 8), dtype=bool)
    snapshot = DynamicSnapshot.from_cells("s1", [(3, 3), (2, 2), (3, 3)], timestamp=10.0, map_shape=base.shape)
    free, costs, changed = apply_dynamic_snapshot(base, snapshot)
    assert bool(base[3, 3]) is True
    assert bool(free[3, 3]) is False
    assert costs.shape == base.shape
    assert changed == ((2, 2), (3, 3))
    assert snapshot.snapshot_hash == DynamicSnapshot.from_cells("s1", [(3, 3), (2, 2)], timestamp=10.0, map_shape=base.shape).snapshot_hash


def test_snapshot_ttl_and_inflation():
    snapshot = DynamicSnapshot.from_cells("s2", [(4, 4)], timestamp=10.0, ttl=2.0, map_shape=(10, 10))
    assert snapshot.is_expired(now=12.1)
    assert not snapshot.is_expired(now=11.9)
    cells = set(snapshot.inflated_cells(1))
    assert (4, 4) in cells and (3, 4) in cells and (4, 5) in cells


def test_snapshot_path_intersection_is_ahead_only():
    path = [(5, 0), (4, 0), (3, 0), (2, 0), (1, 0)]
    snapshot = DynamicSnapshot.from_cells("s3", [(4, 0)], map_shape=(6, 6))
    assert path_intersects_snapshot(path, snapshot)
    assert not path_intersects_snapshot(path, snapshot, ahead_from_index=2)


def test_snapshot_rejects_wrong_map_shape():
    with pytest.raises(ValueError):
        apply_dynamic_snapshot(np.ones((3, 3), dtype=bool), DynamicSnapshot.from_cells("bad", [(1, 1)], map_shape=(4, 4)))
