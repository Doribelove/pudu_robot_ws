from types import SimpleNamespace

import numpy as np

from arena_evaluation import l1_l3_corridor_hybrid_smoke as candidate
from arena_evaluation import l1_l3_corridor_hybrid_validity as validity
from arena_evaluation.topology import TopologyEdge, TopologyGraph, TopologyNode


class _Map:
    occupancy = np.zeros((20, 20), dtype=np.int8)
    resolution = 0.05

    def footprint_collision(self, pose, footprint, unknown_is_collision=True):
        del pose, footprint, unknown_is_collision
        return False

    def world_to_cell(self, x, y):
        return int(round(float(y) / 0.05)) + 10, int(round(float(x) / 0.05)) + 10


def _topology():
    nodes = [
        TopologyNode(0, 0.0, 0.0, 10, 10, 1, 1.0, 1.0, 0),
        TopologyNode(1, 0.5, 0.0, 10, 11, 2, 1.0, 1.0, 0),
        TopologyNode(2, 1.0, 0.0, 10, 12, 1, 1.0, 1.0, 0),
        TopologyNode(3, 30.0, 30.0, 1, 1, 1, 1.0, 1.0, 1),
    ]
    edge0 = TopologyEdge(0, 0, 1, 0.5, 1.0, 1.0, 1.0, 2, [[0.0, 0.0], [0.5, 0.0]])
    edge1 = TopologyEdge(1, 1, 2, 0.5, 1.0, 1.0, 1.0, 2, [[0.5, 0.0], [1.0, 0.0]])
    return SimpleNamespace(
        graph=TopologyGraph(nodes, [edge0, edge1]),
        free_mask=np.ones((20, 20), dtype=bool),
        hospital_map=_Map(),
    )


def test_optimized_endpoint_index_and_candidate_cache_hit():
    topology = _topology()
    first_stats = {}
    first = candidate._attachment_candidates(
        topology, [0.0, 0.0, 0.0], cache_mode=candidate.CACHE_MODE_OPTIMIZED,
        timing=first_stats,
    )
    second_stats = {}
    second = candidate._attachment_candidates(
        topology, [0.0, 0.0, 0.0], cache_mode=candidate.CACHE_MODE_OPTIMIZED,
        timing=second_stats,
    )
    assert [item.node_id for item in first] == [item.node_id for item in second]
    assert first_stats["endpoint_candidate_cache_hit"] is False
    assert second_stats["endpoint_candidate_cache_hit"] is True
    assert second_stats["spatial_index_cache_hit"] is True
    assert second_stats["scanned_count"] == 0


def test_optimized_route_selection_uses_one_multi_source_search(monkeypatch):
    topology = _topology()
    route = SimpleNamespace(node_ids=[0, 1, 2], edge_ids=[0, 1], length_m=1.0, min_width_m=1.0)
    monkeypatch.setattr(
        candidate, "search_topology_multi_goal_timed",
        lambda *_args, **_kwargs: (route, 0, 2, {
            "adjacency_cache_hit": True, "adjacency_build_ms": 0.0,
            "route_search_ms": 0.1, "route_construction_ms": 0.01,
        }),
    )
    monkeypatch.setattr(
        candidate.legacy, "search_topology",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("pairwise search must not run")),
    )
    timing = {}
    start, goal, selected, reason = candidate._select_route_with_endpoint_attach(
        topology,
        SimpleNamespace(start=[0.0, 0.0, 0.0], goal=[1.0, 0.0, 0.0]),
        cache_mode=candidate.CACHE_MODE_OPTIMIZED,
        timing=timing,
    )
    assert selected is route
    assert start.node_id == 0
    assert goal.node_id == 2
    assert reason == "multi_source_route"
    assert timing["candidate_pair_attempts"] == 1
    assert timing["route_cache_hit"] is False


def test_optimized_route_selection_reuses_cached_route(monkeypatch):
    topology = _topology()
    route = SimpleNamespace(node_ids=[0, 1, 2], edge_ids=[0, 1], length_m=1.0, min_width_m=1.0)
    calls = {"count": 0}

    def search(*_args, **_kwargs):
        calls["count"] += 1
        return route, 0, 2, {
            "adjacency_cache_hit": True, "adjacency_build_ms": 0.0,
            "route_search_ms": 0.1, "route_construction_ms": 0.01,
        }

    monkeypatch.setattr(candidate, "search_topology_multi_goal_timed", search)
    query = SimpleNamespace(start=[0.0, 0.0, 0.0], goal=[1.0, 0.0, 0.0])
    first_timing = {}
    candidate._select_route_with_endpoint_attach(
        topology, query, cache_mode=candidate.CACHE_MODE_OPTIMIZED, timing=first_timing,
    )
    second_timing = {}
    start, goal, selected, reason = candidate._select_route_with_endpoint_attach(
        topology, query, cache_mode=candidate.CACHE_MODE_OPTIMIZED, timing=second_timing,
    )
    assert calls["count"] == 1
    assert (start.node_id, goal.node_id, selected) == (0, 2, route)
    assert reason == "route_cache_hit"
    assert second_timing["route_cache_hit"] is True
    assert second_timing["route_search_ms"] == 0.0


def test_explicit_topology_cache_root_is_write_owner(monkeypatch, tmp_path):
    calls = {}
    artifact = SimpleNamespace(metadata={"skeleton_backend": "numpy_zhang_suen"})

    def load(_map_id, _ctx, cache_root, _commit, _source_hash, fallback_root):
        calls["cache_root"] = cache_root
        calls["fallback_root"] = fallback_root
        return artifact, {"topology_cache_hit": False, "topology_build_time_ms": 1.0}

    monkeypatch.setattr(validity.candidate, "_load_authoritative_topology", load)
    monkeypatch.setattr(validity, "_source_commit", lambda: "test-commit")
    result, info = validity._load_topology(SimpleNamespace(), tmp_path, tmp_path / "shared-cache")
    assert result is artifact
    assert calls["cache_root"] == (tmp_path / "shared-cache").resolve()
    assert calls["fallback_root"] == (tmp_path / "shared-cache").resolve()
    assert info["topology_cache_hit"] is False


def test_implicit_topology_cache_root_keeps_miss_in_attempt(monkeypatch, tmp_path):
    calls = {}

    def load(_map_id, _ctx, cache_root, _commit, _source_hash, fallback_root):
        calls["cache_root"] = cache_root
        calls["fallback_root"] = fallback_root
        return SimpleNamespace(metadata={"skeleton_backend": "numpy_zhang_suen"}), {"topology_cache_hit": False}

    monkeypatch.setattr(validity.candidate, "_load_authoritative_topology", load)
    monkeypatch.setattr(validity, "_source_commit", lambda: "test-commit")
    validity._load_topology(SimpleNamespace(), tmp_path)
    assert calls["fallback_root"] == (tmp_path / "topology_cache").resolve()
    assert calls["fallback_root"] != calls["cache_root"]
