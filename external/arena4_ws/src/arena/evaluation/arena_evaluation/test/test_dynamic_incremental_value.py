import json

from arena_evaluation.dynamic_incremental_value import (
    ARMS,
    COLD_GRAPH_ASTAR,
    INCREMENTAL_DSTAR,
    DynamicEdgeOverlay,
    GraphTemplate,
    run_paired_episode,
)
from arena_evaluation.graph_dstar_lite import GraphDStarLite, GraphEdge


def _template():
    planner = GraphDStarLite(
        [0, 1, 2, 3],
        [
            GraphEdge("a", 0, 1, 1.0), GraphEdge("b", 1, 3, 1.0),
            GraphEdge("c", 0, 2, 1.2), GraphEdge("d", 2, 3, 1.2),
        ],
        0, 3,
    )
    planner.node_positions = {0: (0.0, 0.0), 1: (1.0, 0.0),
                              2: (1.0, 1.0), 3: (2.0, 0.0)}
    return GraphTemplate.from_dstar(planner)


def _payload(index, cells, *, timestamp=None, ttl=None, map_version="map-v1"):
    return json.dumps({
        "snapshot_id": f"S{index}", "timestamp": float(index + 1 if timestamp is None else timestamp),
        "occupied_cells": cells, "ttl": ttl, "map_version": map_version,
        "map_shape": [2, 3],
    }, sort_keys=True)


EDGE_CELLS = {"a": [(0, 1)], "b": [(0, 2)], "c": [(1, 1)], "d": [(1, 2)]}


def test_blocked_state_machine_uses_inf_semantics_and_recovers():
    overlay = DynamicEdgeOverlay(EDGE_CELLS, map_version="map-v1", map_shape=(2, 3))
    statuses = []
    for payload in (
        _payload(0, []), _payload(1, [[0, 1]]), _payload(2, [[0, 1]]),
        _payload(3, []), _payload(4, []),
    ):
        prepared = overlay.consume_json(payload)
        statuses.append(prepared.statuses["a"])
    assert statuses == ["AVAILABLE", "BLOCKED_PENDING", "BLOCKED", "RECOVERING", "AVAILABLE"]


def test_expired_and_out_of_order_snapshots_do_not_mutate_overlay():
    overlay = DynamicEdgeOverlay(EDGE_CELLS, map_version="map-v1", map_shape=(2, 3))
    first = overlay.consume_json(_payload(0, []))
    before = dict(overlay.statuses)
    old = overlay.consume_json(_payload(1, [[0, 1]], timestamp=0.5))
    expired = overlay.consume_json(_payload(2, [[0, 1]], timestamp=1.5, ttl=0.1), now=2.0)
    wrong_map = overlay.consume_json(_payload(3, [[0, 1]], map_version="other"))
    assert first.accepted
    assert old.rejection_reason == "OUT_OF_ORDER_SNAPSHOT"
    assert expired.rejection_reason == "EXPIRED_SNAPSHOT"
    assert wrong_map.rejection_reason == "MAP_VERSION_MISMATCH"
    assert overlay.statuses == before


def test_cell_to_edge_mapping_and_three_arm_dynamic_oracle_parity():
    payloads = [
        _payload(0, []), _payload(1, [[0, 1]]), _payload(2, [[0, 1]]),
        _payload(3, [[1, 1]]), _payload(4, [[1, 1]]),
        _payload(5, []), _payload(6, []),
    ]
    rows = run_paired_episode(
        _template(), payloads, EDGE_CELLS,
        map_version="map-v1", map_shape=(2, 3),
    )
    assert len(rows) == len(payloads) * len(ARMS)
    for index in range(len(payloads)):
        paired = [row for row in rows if row["snapshot_index"] == index]
        assert len({row["algorithm_input_hash"] for row in paired}) == 1
        assert len({row["reachable"] for row in paired}) == 1
        assert len({row["path_cost"] for row in paired}) == 1
        assert len({tuple(row["path_edge_ids"]) for row in paired}) == 1
        assert all(not row["blocked_edges_in_path"] for row in paired)


def test_incremental_state_is_reused_without_implicit_reinitialize():
    rows = run_paired_episode(
        _template(), [_payload(0, []), _payload(1, [[0, 1]]), _payload(2, [[0, 1]])],
        EDGE_CELLS, map_version="map-v1", map_shape=(2, 3),
    )
    incremental = [row for row in rows if row["arm"] == INCREMENTAL_DSTAR]
    assert incremental[0]["initialization_count"] == 1
    assert all(row["initialization_count"] == 1 for row in incremental)
    assert all(row["reinitialize_call_count"] == 0 for row in incremental)
    assert all(row["planner_identity_stable"] for row in incremental)
    assert all(row["g_reused"] and row["rhs_reused"] and row["open_reused"] and row["km_reused"] for row in incremental[1:])
    cold = [row for row in rows if row["arm"] == "cold_dstar"]
    assert [row["initialization_count"] for row in cold] == [1, 2, 3]
    assert [row["reinitialize_call_count"] for row in cold] == [0, 1, 2]
    assert all(not row["g_reused"] and not row["rhs_reused"] for row in cold)


def test_no_route_and_reappearance_parity_and_static_template_immutable():
    template = _template()
    static_hash = template.static_hash
    payloads = [
        _payload(0, []), _payload(1, [[0, 1], [1, 1]]),
        _payload(2, [[0, 1], [1, 1]]), _payload(3, []), _payload(4, []),
    ]
    rows = run_paired_episode(
        template, payloads, EDGE_CELLS,
        map_version="map-v1", map_shape=(2, 3),
    )
    blocked = [row for row in rows if row["snapshot_index"] == 2]
    recovered = [row for row in rows if row["snapshot_index"] == 4]
    assert all(row["reachable"] is False and row["failure_code"] == "L1_NO_ROUTE" for row in blocked)
    assert all(row["reachable"] is True for row in recovered)
    assert template.static_hash == static_hash
    assert any(row["arm"] == COLD_GRAPH_ASTAR for row in rows)


def test_virtual_endpoint_heuristic_uses_safe_zero_fallback():
    planner = GraphDStarLite([-1, 1], [GraphEdge("virtual", -1, 1, 0.1)], -1, 1)
    assert planner._heuristic(-1, 1) == 0.0
