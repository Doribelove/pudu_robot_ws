import importlib

import numpy as np

from arena_evaluation.dstar_lite import DStarLite
from arena_evaluation.dynamic_snapshot import DynamicSnapshot


module = importlib.import_module("arena_evaluation.three_d_v0_static_performance")


def test_static_task_sources_are_consistent():
    queries, metadata = module._load_tasks()
    assert [query.query_id for query in queries] == list(module.TASK_IDS)
    assert metadata["dynamic_obstacles"] is False
    assert metadata["resolution_m"] == 0.05


def test_empty_snapshot_is_stable_and_map_bound():
    snapshot = DynamicSnapshot.empty(
        module.SNAPSHOT_ID, timestamp=0.0, map_version="map-hash", map_shape=(4, 5),
    )
    assert snapshot.snapshot_id == "static_empty_v1"
    assert snapshot.occupied_cells == ()
    assert snapshot.ttl is None
    assert len(snapshot.snapshot_hash) == 64


def test_dstar_initial_stats_expose_audit_counters():
    planner = DStarLite(np.ones((8, 8), dtype=bool), (7, 0), (0, 7))
    stats = planner.compute_shortest_path()
    assert stats.expanded_nodes > 0
    assert stats.queue_pops > 0
    assert stats.initial_queue_size >= 1
    assert stats.queue_pushes >= 0
    assert stats.update_vertex_count > 0
    assert stats.final_queue_size >= 0
