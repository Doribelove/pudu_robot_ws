import json
import random

from arena_evaluation import dynamic_incremental_value as dynamic
from arena_evaluation.dynamic_snapshot import DynamicSnapshot
from arena_evaluation.graph_dstar_lite import GraphDStarLite, GraphEdge
from arena_evaluation.indexed_dstar_open import (
    IndexedGraphDStarLite,
    IndexedLexicographicPriorityQueue,
    InstrumentedGraphDStarLite,
    exact_start_goal_connected,
)
from arena_evaluation import dstar_tail_research as research
from arena_evaluation import two_layer_2d_v2_r1_dstar_tail_benchmark as benchmark


def _edges():
    return [
        GraphEdge("a", 0, 1, 1.0), GraphEdge("b", 1, 4, 1.0),
        GraphEdge("c", 0, 2, 1.0), GraphEdge("d", 2, 3, 1.0),
        GraphEdge("e", 3, 4, 1.0), GraphEdge("f", 1, 2, 0.5),
    ]


def _path(planner):
    stats = planner.compute_shortest_path()
    assert stats.converged
    return planner.extract_path(), planner._value(planner.g, planner.start), stats


def test_indexed_open_random_operations_match_reference():
    rng = random.Random(20260903)
    queue = IndexedLexicographicPriorityQueue()
    reference = {}
    for _ in range(2000):
        node = rng.randrange(75)
        operation = rng.randrange(4)
        if operation < 2:
            key = (float(rng.randrange(12)), float(rng.randrange(12)))
            queue.insert_or_update(node, key)
            reference[node] = key
        elif operation == 2:
            assert queue.remove(node) == (node in reference)
            reference.pop(node, None)
        elif reference:
            expected_key = min(reference.values())
            key, popped = queue.pop_min()
            assert key == expected_key
            assert reference.pop(popped) == expected_key
        assert len(queue) == len(reference)
        assert set(queue._positions) == set(reference)


def test_lexicographic_update_remove_and_pop_min():
    queue = IndexedLexicographicPriorityQueue()
    queue.insert_or_update(-100, (2.0, 0.0))
    queue.insert_or_update(2, (1.0, 9.0))
    queue.insert_or_update(3, (1.0, 4.0))
    queue.insert_or_update(-100, (0.5, 8.0))
    assert queue.key_for(-100) == (0.5, 8.0)
    assert queue.pop_min() == ((0.5, 8.0), -100)
    assert queue.remove(2)
    assert queue.pop_min() == ((1.0, 4.0), 3)
    assert len(queue) == 0


def test_indexed_matches_lazy_for_virtual_endpoint_dynamic_sequence():
    edges = [GraphEdge("root_start", -1, 0, 0.1), *_edges(),
             GraphEdge("root_goal", 4, -2, 0.1)]
    nodes = [-2, -1, 0, 1, 2, 3, 4]
    lazy = InstrumentedGraphDStarLite(nodes, edges, -1, -2)
    indexed = IndexedGraphDStarLite(nodes, edges, -1, -2)
    for planner in (lazy, indexed):
        planner.node_positions = {-1: (0, 0), -2: (3, 0), 0: (0, 0),
                                  1: (1, 0), 2: (1, 1), 3: (2, 1), 4: (2, 0)}
    sequences = [
        {},
        {"b": GraphDStarLite.BLOCKED},
        {"b": GraphDStarLite.BLOCKED, "d": GraphDStarLite.BLOCKED},
        {"b": GraphDStarLite.RECOVERING},
        {"b": GraphDStarLite.AVAILABLE, "d": GraphDStarLite.AVAILABLE},
    ]
    old = {}
    for statuses in sequences:
        changed = sorted(set(old) | set(statuses))
        changed = [edge for edge in changed if old.get(edge) != statuses.get(edge)]
        for planner in (lazy, indexed):
            planner.update_edges(changed, statuses=statuses)
        lazy_path, lazy_cost, _ = _path(lazy)
        indexed_path, indexed_cost, stats = _path(indexed)
        assert indexed_path == lazy_path
        assert indexed_cost == lazy_cost
        assert stats.stale_queue_entries == 0
        old = dict(statuses)


def test_existing_update_edges_already_deduplicates_initial_batch():
    planner = InstrumentedGraphDStarLite(range(5), _edges(), 0, 4)
    _path(planner)
    affected = planner.update_edges(["a", "a", "f", "f"], statuses={"a": "BLOCKED"})
    assert affected == planner.update_batch_unique_count
    assert planner.update_batch_candidate_count > planner.update_batch_unique_count


def test_exact_connectivity_bridge_multicut_and_recovery():
    template = dynamic.GraphTemplate.from_dstar(GraphDStarLite(range(5), _edges(), 0, 4))
    connected = exact_start_goal_connected(template.nodes, template.adjacency, 0, 4, {})
    cut = {"a": GraphDStarLite.BLOCKED, "c": GraphDStarLite.BLOCKED}
    bridge_blocked = exact_start_goal_connected(template.nodes, template.adjacency, 0, 4, cut)
    recovering = exact_start_goal_connected(
        template.nodes, template.adjacency, 0, 4,
        {"a": GraphDStarLite.RECOVERING, "c": GraphDStarLite.BLOCKED},
    )
    restored = exact_start_goal_connected(template.nodes, template.adjacency, 0, 4, {})
    assert connected[0] and not bridge_blocked[0] and not recovering[0] and restored[0]


def _payload(snapshot_id, timestamp, occupied=()):
    snapshot = DynamicSnapshot(snapshot_id, timestamp, tuple(occupied), {}, None, "m", (8, 8))
    raw = {
        "snapshot_id": snapshot_id, "timestamp": timestamp,
        "occupied_cells": [list(cell) for cell in occupied],
        "obstacle_confidence": {}, "ttl": None, "map_version": "m",
        "map_shape": [8, 8], "snapshot_hash": snapshot.snapshot_hash,
    }
    return json.dumps(raw, sort_keys=True)


def test_no_route_and_recovery_oracle_parity_without_reinitialize():
    template = dynamic.GraphTemplate.from_dstar(GraphDStarLite(range(3), [
        GraphEdge("a", 0, 1, 1.0), GraphEdge("b", 1, 2, 1.0),
    ], 0, 2))
    overlay = dynamic.DynamicEdgeOverlay({"a": ((1, 1),), "b": ((1, 2),)},
                                         map_version="m", map_shape=(8, 8))
    planner = IndexedGraphDStarLite(template.nodes, template.edges, 0, 2)
    identity = id(planner)
    for index, cells in enumerate(((), ((1, 1),), ((1, 1),), (), ())):
        prepared = overlay.consume_json(_payload(f"S{index}", float(index + 1), cells))
        planner.update_edges(prepared.changed_edges, statuses=prepared.changed_statuses)
        path, cost, stats = _path(planner)
        oracle = dynamic.deterministic_graph_astar(template, prepared.statuses)
        assert (path is None) == (oracle.node_path is None)
        assert cost == oracle.cost
        assert stats.converged and id(planner) == identity


def test_all_research_arms_share_input_and_oracle_result():
    template = dynamic.GraphTemplate.from_dstar(GraphDStarLite(range(5), _edges(), 0, 4))
    overlay = dynamic.DynamicEdgeOverlay(
        {edge.edge_id: ((index + 1, 1),) for index, edge in enumerate(_edges())},
        map_version="m", map_shape=(8, 8),
    )
    states = {arm: research.TailArmState(arm, template) for arm in research.ARMS}
    for index, cells in enumerate(((), ((2, 1),), ((2, 1),), (), ())):
        prepared = overlay.consume_json(_payload(f"S{index}", float(index + 1), cells))
        results = [state.run(prepared) for state in states.values()]
        assert len({row["algorithm_input_hash"] for row in results}) == 1
        assert len({(row["reachable"], row["failure_code"], row["path_cost"])
                    for row in results}) == 1
        assert all(not row["blocked_edges_in_path"] for row in results)
        assert all(not row["partial_dstar_result_returned"] for row in results)


def test_budgeted_incomplete_search_cannot_be_extracted_by_research_arm():
    planner = IndexedGraphDStarLite(range(5), _edges(), 0, 4)
    stats = planner.compute_shortest_path(max_queue_pops=0)
    assert not stats.converged and stats.budget_triggered
    # TailArmState never passes a budget and asserts convergence before it
    # extracts.  The low-level partial state remains resumable.
    resumed = planner.compute_shortest_path()
    assert resumed.converged and planner.extract_path()


def test_lazy_resync_coalesces_and_validates_latest_snapshot():
    template = dynamic.GraphTemplate.from_dstar(GraphDStarLite(range(5), _edges(), 0, 4))
    overlay = dynamic.DynamicEdgeOverlay(
        {edge.edge_id: ((index + 1, 1),) for index, edge in enumerate(_edges())},
        map_version="m", map_shape=(8, 8),
    )
    s0 = overlay.consume_json(_payload("S0", 1.0))
    s1 = overlay.consume_json(_payload("S1", 2.0, ((2, 1),)))
    s2 = overlay.consume_json(_payload("S2", 3.0, ((2, 1),)))
    s3 = overlay.consume_json(_payload("S3", 4.0, ((2, 1),)))
    s4 = overlay.consume_json(_payload("S4", 5.0, ((2, 1),)))
    state = research.ResyncState("lazy", template, quiet_window_snapshots=2)
    state.initialize(s0)
    assert not state.on_fallback(s1)["resync_ran"]
    assert not state.observe(s2)["resync_ran"]
    assert not state.observe(s3)["resync_ran"]
    result = state.observe(s4)
    assert result["resync_ran"] and result["coalesced_snapshots"] == 4
    assert result["resync_snapshot_id"] == "S4"
    assert result["resync_status_hash"] == dynamic.stable_hash(s4.statuses)
    assert state.ready


def test_tail_arm_timer_accounting_has_no_duplicate_phase_sum():
    template = dynamic.GraphTemplate.from_dstar(GraphDStarLite(range(5), _edges(), 0, 4))
    overlay = dynamic.DynamicEdgeOverlay(
        {edge.edge_id: ((index + 1, 1),) for index, edge in enumerate(_edges())},
        map_version="m", map_shape=(8, 8),
    )
    prepared = overlay.consume_json(_payload("S0", 1.0))
    for arm in research.ARMS:
        result = research.TailArmState(arm, template).run(prepared)
        common = prepared.parse_time_ms + prepared.mapping_time_ms + prepared.state_transition_time_ms
        assert result["full_l1_ms"] >= common
        assert result["full_l1_ms"] + 1e-9 >= result["response_l1_ms"]
        # Diagnostics are explicitly outside both response and full-L1 wall.
        assert result["diagnostics_ms_excluded"] >= 0.0


def test_calibration_does_not_promote_slower_indexed_change():
    rows = []
    timings = {
        research.COLD_GRAPH_ASTAR: [10.0, 11.0, 12.0],
        research.BASELINE_DSTAR: [5.0, 50.0, 70.0],
        research.INDEXED_DSTAR: [7.0, 75.0, 90.0],
        research.INDEXED_BATCH_DSTAR: [7.0, 74.0, 89.0],
        research.INDEXED_BATCH_CONNECTIVITY: [9.0, 77.0, 92.0],
        research.COMBO_DSTAR: [5.0, 50.0, 70.0],
    }
    for arm, values in timings.items():
        for value in values:
            rows.append({
                "arm": arm, "run_mode": "measured", "dynamic_update": True,
                "changed_edge_count": 1, "full_l1_ms": value,
                "all_correct": True, "oracle_reachable": False,
            })
    selected = benchmark.select_combo_backend(rows)
    assert selected["selected_backend"] == research.BASELINE_DSTAR
    assert not any(row["accepted"] for row in selected["selection_decisions"])


def test_artifact_manifest_machine_validation(tmp_path):
    for name in benchmark.REQUIRED:
        (tmp_path / name).write_text("status\n", encoding="utf-8")
    source = tmp_path / "source_snapshot" / "x.py"
    source.parent.mkdir(); source.write_text("x=1\n", encoding="utf-8")
    manifest = {
        "files": [{"snapshot": "source_snapshot/x.py",
                   "sha256": benchmark.sha256_file(source)}],
    }
    (tmp_path / "source_snapshot_manifest.yaml").write_text(
        benchmark.yaml.safe_dump(manifest), encoding="utf-8",
    )
    assert benchmark.validate_artifacts(tmp_path)["passed"]
