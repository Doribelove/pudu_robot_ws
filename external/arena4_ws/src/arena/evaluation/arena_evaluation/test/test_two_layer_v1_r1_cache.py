from types import SimpleNamespace

import numpy as np

from arena_evaluation import l1_l3_corridor_hybrid_smoke as candidate
from arena_evaluation import two_layer_v1_formal_benchmark as r0
from arena_evaluation import two_layer_v1_r1_cache_benchmark as r1
from arena_evaluation import unified_four_backends_smoke as legacy


class _Map:
    resolution = 0.05
    width = 40
    height = 40
    origin = [0.0, 0.0, 0.0]

    def __init__(self):
        self.occupancy = np.zeros((self.height, self.width), dtype=np.int8)

    def world_to_cell(self, x, y):
        row = int(round(float(y) / self.resolution))
        col = int(round(float(x) / self.resolution))
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None


def _ctx():
    return SimpleNamespace(
        hospital_map=_Map(), map_id="test-map", map_sha256="map", map_yaml_sha256="yaml",
    )


def _route():
    return SimpleNamespace(
        node_ids=[1, 2], edge_ids=[7], length_m=1.0,
        polyline=[[0.2, 0.2], [1.0, 0.2]],
    )


def test_route_cache_hit_preserves_exact_mask_and_hash(monkeypatch, tmp_path):
    ctx = _ctx()
    topology = SimpleNamespace(
        metadata={"topology_cache_key": "topology"},
        graph=SimpleNamespace(nodes=[], edges=[]),
    )
    route = _route()
    query = SimpleNamespace(start=[0.2, 0.2, 0.0], goal=[1.0, 0.2, 0.0])
    expected = np.zeros((40, 40), dtype=bool)
    expected[4:8, 4:24] = True
    calls = {"count": 0}

    def fake_builder(*_args):
        calls["count"] += 1
        return expected.copy(), {
            "corner_count": 0, "corner_node_ids": [], "corner_edge_ids": [],
            "corner_max_curvature_1pm": 0.0, "corner_support_length_m": 0.0,
            "base_corridor_padding_m": 2.0, "corner_corridor_padding_m": 4.0,
            "corner_widened_area_ratio": 0.0, "corner_corridor_mask_hash": r1._grid_hash(expected),
        }

    monkeypatch.setattr(r0, "build_adaptive_corridor_mask", fake_builder)
    cache = r1.RouteMaskCache(ctx, topology, "source", tmp_path)
    first, first_diag = cache.builder(ctx, topology, route, query, (4, 4), (4, 20), 2.0, r1.CORRIDOR_SEMANTICS)
    second, second_diag = cache.builder(ctx, topology, route, query, (4, 4), (4, 20), 2.0, r1.CORRIDOR_SEMANTICS)
    assert calls["count"] == 1
    assert np.array_equal(first, second)
    assert r1._grid_hash(first) == r1._grid_hash(second)
    assert first_diag["mask_cache_hit"] is False
    assert second_diag["mask_cache_hit"] is True
    assert second_diag["precomputed_mask_hash"] == r1._grid_hash(expected)
    assert second_diag["precomputed_allowed_cells"] == int(np.count_nonzero(expected))


def test_compact_edge_mask_uses_roi_not_full_map():
    ctx = _ctx()
    entry = r1._compact_dilated_mask(ctx, np.pad(np.ones((2, 3), dtype=np.uint8), ((10, 28), (12, 25))), 0.1)
    assert entry["shape"][0] < ctx.hospital_map.height
    assert entry["shape"][1] < ctx.hospital_map.width
    assert len(entry["packed"]) < ctx.hospital_map.height * ctx.hospital_map.width
    assert entry["allowed_cells"] > 0


def _session_for_update(ctx, current_grid):
    session = object.__new__(legacy.SmacSession)
    session.ctx = ctx
    session.supports_local_mask = True
    session._local_update_publisher = object()
    session._local_map_publisher = object()
    session.client = object()
    session.local_map_update_strategy = "delta"
    session._current_grid = current_grid.copy()
    session._current_allowed_mask = None
    session._costmap_state_trusted = True
    session._force_full_next_update = False
    session._last_update_had_fallback = False
    session.enable_mask_reuse_noop = True
    session._local_mask_info = {}
    session._publish_delta_updates = lambda *_args, **_kwargs: current_grid.copy()
    session._publish_full_grid = lambda *_args, **_kwargs: 0.0
    session._clear_global_costmap = lambda: 0.0
    return session


def test_costmap_reuse_noop_requires_identical_mask():
    ctx = _ctx()
    allowed = np.zeros((40, 40), dtype=bool)
    allowed[5:10, 5:20] = True
    values = np.where(allowed, ctx.hospital_map.occupancy, 100).astype(np.int8)
    session = _session_for_update(ctx, np.flipud(values))
    info = session.update_local_mask(allowed, force_full=False)
    assert info["local_map_update_mode"] == "reuse_noop"
    assert info["local_map_update_skipped"] is True
    assert info["local_map_update_messages"] == 0
    assert info["previous_mask_hash"] == info["expected_mask_hash"] == info["applied_mask_hash"]

    changed = allowed.copy()
    changed[30, 30] = True
    session._current_grid = np.flipud(values)
    info_changed = session.update_local_mask(changed, force_full=False)
    assert info_changed["local_map_update_mode"] != "reuse_noop"
    assert info_changed["local_map_update_skipped"] is False


def test_force_full_update_never_reuses_identical_mask():
    ctx = _ctx()
    allowed = np.zeros((40, 40), dtype=bool)
    allowed[5:10, 5:20] = True
    values = np.where(allowed, ctx.hospital_map.occupancy, 100).astype(np.int8)
    session = _session_for_update(ctx, np.flipud(values))
    info = session.update_local_mask(allowed, force_full=True)
    assert info["local_map_update_mode"] == "full_fallback"
    assert info["local_map_update_skipped"] is False
