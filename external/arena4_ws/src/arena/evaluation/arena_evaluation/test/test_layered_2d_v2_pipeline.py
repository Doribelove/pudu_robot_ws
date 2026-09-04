import json

import numpy as np

from arena_evaluation.dynamic_incremental_value import GraphTemplate
from arena_evaluation.graph_dstar_lite import GraphDStarLite, GraphEdge
from arena_evaluation.layered_2d_v2_pipeline import (
    ARCHITECTURE_ID,
    IMPLEMENTATION_REVISION,
    PersistentDynamicEpisode,
    corridor_dirty_transition,
)


def _payload(index, cells, *, timestamp=None, ttl=None):
    return json.dumps({
        "snapshot_id": f"S{index}",
        "timestamp": float(index + 1 if timestamp is None else timestamp),
        "occupied_cells": cells,
        "ttl": ttl,
        "map_version": "map-v2",
        "map_shape": [2, 3],
    }, sort_keys=True)


def _template():
    planner = GraphDStarLite(
        [0, 1, 2, 3],
        [GraphEdge("a", 0, 1, 1.0), GraphEdge("b", 1, 3, 1.0),
         GraphEdge("c", 0, 2, 1.2), GraphEdge("d", 2, 3, 1.2)],
        0, 3,
    )
    planner.node_positions = {0: (0.0, 0.0), 1: (1.0, 0.0),
                              2: (1.0, 1.0), 3: (2.0, 0.0)}
    return GraphTemplate.from_dstar(planner)


EDGE_CELLS = {"a": [(0, 1)], "b": [(0, 2)],
              "c": [(1, 1)], "d": [(1, 2)]}


def test_v2_identity_is_independent_candidate():
    assert ARCHITECTURE_ID == "2D-V2"
    assert IMPLEMENTATION_REVISION == "r0-enhanced-runtime-v1"


def test_old_new_corridor_dirty_union_closes_residue_and_opens_new():
    old = np.zeros((8, 9), dtype=bool)
    old[1:4, 1:3] = True
    new = np.zeros_like(old)
    new[3:7, 5:8] = True
    result = corridor_dirty_transition(old, new)
    assert result["dirty_cells"] == int(np.count_nonzero(old ^ new))
    assert result["closed_cells"] == int(np.count_nonzero(old & ~new))
    assert result["opened_cells"] == int(np.count_nonzero(new & ~old))
    assert result["old_corridor_residual_cells"] == 0


def test_multiple_route_switches_never_leave_old_corridor_residue():
    masks = []
    for column in (1, 4, 7, 2):
        mask = np.zeros((12, 12), dtype=bool)
        mask[2:10, column:column + 2] = True
        masks.append(mask)
    for old, new in zip(masks, masks[1:]):
        assert corridor_dirty_transition(old, new)["old_corridor_residual_cells"] == 0


def test_persistent_episode_oracle_parity_block_no_route_and_recovery_without_reinit():
    episode = PersistentDynamicEpisode(
        _template(), EDGE_CELLS, map_version="map-v2", map_shape=(2, 3),
    )
    rows = [episode.step(payload) for payload in (
        _payload(0, []),
        _payload(1, [[0, 1]]), _payload(2, [[0, 1]]),
        _payload(3, [[0, 1], [1, 1]]), _payload(4, [[0, 1], [1, 1]]),
        _payload(5, []), _payload(6, []),
    )]
    assert all(row["accepted"] for row in rows)
    assert rows[2]["reachable"] and "a" not in rows[2]["edge_path"]
    assert rows[4]["reachable"] is False
    assert rows[6]["reachable"] is True
    assert rows[0]["g_rhs_open_km_reused"] is False
    assert all(row["g_rhs_open_km_reused"] for row in rows[1:])
    assert all(row["reinitialize_count"] == 0 for row in rows)
    assert all(row["oracle_cost_error"] <= 1e-9 for row in rows)


def test_expired_and_out_of_order_snapshots_are_rejected_without_state_change():
    episode = PersistentDynamicEpisode(
        _template(), EDGE_CELLS, map_version="map-v2", map_shape=(2, 3),
    )
    first = episode.step(_payload(0, []))
    old = episode.step(_payload(1, [[0, 1]], timestamp=0.5))
    expired = episode.step(_payload(2, [[0, 1]], timestamp=1.5, ttl=0.1))
    assert first["accepted"]
    assert old["failure_code"] == "OUT_OF_ORDER_SNAPSHOT"
    assert expired["failure_code"] == "EXPIRED_SNAPSHOT"
    assert episode.snapshot_count == 1
