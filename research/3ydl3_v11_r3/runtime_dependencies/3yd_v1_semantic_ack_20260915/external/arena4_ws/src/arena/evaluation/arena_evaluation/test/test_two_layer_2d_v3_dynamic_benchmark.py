import json

import numpy as np
import pytest

from arena_evaluation import dynamic_incremental_value as dynamic
from arena_evaluation import hybrid_l1_router as hybrid
from arena_evaluation import layered_2d_v2_pipeline as v2
from arena_evaluation import layered_2d_v3_pipeline as v3
from arena_evaluation.dynamic_snapshot import DynamicSnapshot
from arena_evaluation.graph_dstar_lite import GraphDStarLite, GraphEdge


def _template():
    planner = GraphDStarLite(
        range(4),
        [
            GraphEdge("topology_01", 0, 1, 1.0),
            GraphEdge("topology_13", 1, 3, 1.0),
            GraphEdge("topology_02", 0, 2, 1.0),
            GraphEdge("topology_23", 2, 3, 1.0),
        ],
        0, 3,
    )
    return dynamic.GraphTemplate.from_dstar(planner)


def _payload(snapshot_id, timestamp, occupied=()):
    raw = {
        "snapshot_id": snapshot_id, "timestamp": float(timestamp),
        "occupied_cells": [list(cell) for cell in occupied],
        "obstacle_confidence": {}, "ttl": None,
        "map_version": "map", "map_shape": [10, 10],
    }
    snapshot = DynamicSnapshot(
        snapshot_id, float(timestamp), tuple(occupied), {}, None, "map", (10, 10),
    )
    raw["snapshot_hash"] = snapshot.snapshot_hash
    return json.dumps(raw, sort_keys=True)


def _overlay():
    cells = {
        "topology_01": ((1, 1),), "topology_13": ((1, 2),),
        "topology_02": ((2, 1),), "topology_23": ((2, 2),),
    }
    return dynamic.DynamicEdgeOverlay(cells, map_version="map", map_shape=(10, 10))


def test_bounded_search_never_exposes_partial_and_resyncs():
    overlay = _overlay()
    router = hybrid.HybridL1Router(
        hybrid.HYBRID, _template(), topology_edge_count=4,
        budget=hybrid.BudgetConfig(
            wall_ms=0.0, max_queue_pops=0, max_update_vertex=1,
            max_open_size=100, max_inconsistent_states=100,
            max_changed_edges=5, max_changed_ratio=1.0,
            max_route_intersections=2, recent_fallback_cooldown=0,
        ),
    )
    initial = router.step(overlay.consume_json(_payload("S0", 1.0)))
    assert initial["reachable"] and not initial["partial_dstar_result_returned"]
    router.service_resync()
    attempted = router.step(overlay.consume_json(_payload("S1", 2.0, ((1, 2),))))
    assert attempted["budget_triggered"]
    assert attempted["actual_algorithm"] == hybrid.COLD_GRAPH_ASTAR
    assert attempted["path_edge_ids"] == ["topology_02", "topology_23"]
    assert not attempted["partial_dstar_result_returned"]
    assert not attempted["dstar_ready_after"]
    resync = router.service_resync()
    assert resync["resync_ran"] and resync["resync_ready"]
    assert router.dstar_snapshot_id == "S1"


def test_recovering_remains_inf_until_available():
    overlay = _overlay()
    prepared = [
        overlay.consume_json(_payload("S0", 1.0)),
        overlay.consume_json(_payload("S1", 2.0, ((1, 2),))),
        overlay.consume_json(_payload("S2", 3.0, ((1, 2),))),
        overlay.consume_json(_payload("S3", 4.0)),
        overlay.consume_json(_payload("S4", 5.0)),
    ]
    assert prepared[2].statuses["topology_13"] == GraphDStarLite.BLOCKED
    assert prepared[3].statuses["topology_13"] == GraphDStarLite.RECOVERING
    edge = next(edge for edge in _template().edges if edge.edge_id == "topology_13")
    assert dynamic.effective_edge_cost(edge, prepared[3].statuses) == float("inf")
    assert prepared[4].statuses["topology_13"] == GraphDStarLite.AVAILABLE
    assert dynamic.effective_edge_cost(edge, prepared[4].statuses) == 1.0


def test_snapshot_arriving_before_resync_coalesces_to_latest():
    overlay = _overlay()
    router = hybrid.HybridL1Router(
        hybrid.HYBRID, _template(), topology_edge_count=4,
        budget=hybrid.BudgetConfig(wall_ms=0.0, max_queue_pops=0,
                                   max_changed_ratio=1.0, max_route_intersections=2),
    )
    router.step(overlay.consume_json(_payload("S0", 1.0)))
    router.step(overlay.consume_json(_payload("S1", 2.0, ((1, 2),))))
    router.step(overlay.consume_json(_payload("S2", 3.0, ((1, 2),))))
    # A recovery snapshot arrives while the old resync is still queued.
    router.step(overlay.consume_json(_payload("S3", 4.0)))
    resync = router.service_resync()
    assert resync["resync_snapshot_id"] == "S3"
    assert router.dstar_snapshot_id == "S3"


def test_expired_and_out_of_order_are_rejected_without_state_change():
    overlay = _overlay()
    accepted = overlay.consume_json(_payload("S1", 10.0))
    assert accepted.accepted
    rejected = overlay.consume_json(_payload("old", 9.0))
    assert not rejected.accepted and rejected.rejection_reason == "OUT_OF_ORDER_SNAPSHOT"
    assert overlay.guard.last_snapshot_id == "S1"


def test_dynamic_corridor_dirty_union_closes_old_cells():
    old = np.zeros((8, 8), dtype=bool); old[1:4, 1:4] = True
    new = np.zeros((8, 8), dtype=bool); new[3:7, 3:7] = True
    transition = v2.corridor_dirty_transition(old, new)
    assert transition["old_corridor_residual_cells"] == 0
    assert transition["closed_cells"] == int(np.count_nonzero(old & ~new))
    assert transition["opened_cells"] == int(np.count_nonzero(new & ~old))


def test_compressed_route_lru_equivalence_eviction_and_binding():
    binding = {"map_hash": "one", "shape": [12, 12], "resolution": 0.05}
    cache = v3.DynamicRouteMaskLRU(map_shape=(12, 12), binding=binding,
                                   memory_cap_bytes=350)
    first = np.zeros((12, 12), dtype=bool); first[1:3, 1:3] = True
    second = np.zeros((12, 12), dtype=bool); second[8:10, 8:10] = True
    cache.put_primitive("edge:e1", first)
    cache.put_primitive("edge:e2", second)
    combined, miss = cache.compose(route_edge_ids=("e1", "e2"), snapshot_id="S1")
    assert np.array_equal(combined, first | second)
    again, hit = cache.compose(route_edge_ids=("e1", "e2"), snapshot_id="S1")
    assert np.array_equal(again, combined) and hit["cache_hit"]
    with pytest.raises(ValueError):
        cache.validate_binding({"map_hash": "four", "shape": [12, 12], "resolution": 0.05})
    cache.invalidate({"map_hash": "four", "shape": [12, 12], "resolution": 0.05})
    assert cache.diagnostics()["memory_bytes"] == 0


def test_exact_bridge_detection_and_selector_direct_fallback():
    template = dynamic.GraphTemplate.from_dstar(GraphDStarLite(
        range(3), [GraphEdge("topology_a", 0, 1, 1.0),
                   GraphEdge("topology_b", 1, 2, 1.0)], 0, 2,
    ))
    assert hybrid.exact_undirected_bridges(template) == {"topology_a", "topology_b"}


def test_stable_no_route_result_is_reused_without_repeated_l1():
    template = dynamic.GraphTemplate.from_dstar(GraphDStarLite(
        range(3), [GraphEdge("topology_a", 0, 1, 1.0),
                   GraphEdge("topology_b", 1, 2, 1.0)], 0, 2,
    ))
    edge_cells = {"topology_a": ((1, 1),), "topology_b": ((1, 2),)}
    overlay = dynamic.DynamicEdgeOverlay(edge_cells, map_version="map", map_shape=(10, 10))
    router = hybrid.HybridL1Router(hybrid.HYBRID, template, topology_edge_count=2)
    router.step(overlay.consume_json(_payload("S0", 1.0)))
    router.step(overlay.consume_json(_payload("S1", 2.0, ((1, 1),))))
    blocked = router.step(overlay.consume_json(_payload("S2", 3.0, ((1, 1),))))
    assert not blocked["reachable"] and blocked["l1_invoked"]
    stable = router.step(overlay.consume_json(_payload("S3", 4.0, ((1, 1),))))
    assert not stable["reachable"]
    assert stable["scheduler_skip"] and not stable["l1_invoked"]
    assert stable["failure_code"] == "L1_NO_ROUTE"
