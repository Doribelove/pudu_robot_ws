from types import SimpleNamespace

import numpy as np
import pytest

from arena_evaluation import two_layer_v1_formal_benchmark as v1


class _Map:
    resolution = 0.05

    def __init__(self, shape=(120, 120)):
        self.occupancy = np.zeros(shape, dtype=np.int8)

    def world_to_cell(self, x, y):
        row = int(round(float(y) / self.resolution))
        col = int(round(float(x) / self.resolution))
        if row < 0 or col < 0 or row >= self.occupancy.shape[0] or col >= self.occupancy.shape[1]:
            return None
        return row, col


def _context(shape=(120, 120)):
    hospital_map = _Map(shape)
    return SimpleNamespace(
        hospital_map=hospital_map,
        free_mask=np.ones(shape, dtype=bool),
    )


def _route(points, node_ids=None, edge_ids=None):
    return SimpleNamespace(
        polyline=points,
        node_ids=list(node_ids or []),
        edge_ids=list(edge_ids or []),
        length_m=float(sum(np.linalg.norm(np.asarray(b) - np.asarray(a)) for a, b in zip(points, points[1:]))),
    )


def test_straight_route_has_no_corner_and_uses_only_base_padding():
    route = _route([[0.5, 1.0], [4.0, 1.0]])
    analysis = v1.analyze_topology_route(route, SimpleNamespace(graph=SimpleNamespace(nodes=[], edges=[])))
    assert analysis["corner_count"] == 0
    assert analysis["corner_intervals_m"] == []

    ctx = _context()
    base = np.zeros_like(ctx.free_mask)
    base[20, 20] = True
    calls = []

    def fake_base(*_args):
        calls.append("base")
        return base

    def fake_dilate(*_args, **kwargs):
        calls.append(float(kwargs.get("padding_m", _args[-1])))
        return np.ones_like(ctx.free_mask)

    original_base = v1.candidate._build_corridor_mask
    original_dilate = v1._dilate_raw
    original_raw = v1.candidate._raw_free_mask
    try:
        v1.candidate._build_corridor_mask = fake_base
        v1._dilate_raw = fake_dilate
        v1.candidate._raw_free_mask = lambda _ctx: np.ones_like(ctx.free_mask)
        mask, diagnostics = v1.build_adaptive_corridor_mask(
            ctx, SimpleNamespace(graph=SimpleNamespace(nodes=[], edges=[])), route,
            SimpleNamespace(), (20, 10), (20, 80), 2.0, v1.CORRIDOR_SEMANTICS,
        )
    finally:
        v1.candidate._build_corridor_mask = original_base
        v1._dilate_raw = original_dilate
        v1.candidate._raw_free_mask = original_raw
    assert np.array_equal(mask, base)
    assert diagnostics["corner_count"] == 0
    assert diagnostics["base_corridor_padding_m"] == 2.0
    assert diagnostics["corner_corridor_padding_m"] == 4.0
    assert calls == ["base"]
    assert diagnostics["no_6m_padding"] is True


def test_arc_length_corner_gets_four_meter_support_interval():
    route = _route([[0.5, 1.0], [2.5, 1.0], [2.5, 3.0], [2.5, 5.0]])
    topology = SimpleNamespace(
        hospital_map=SimpleNamespace(resolution=0.05),
        graph=SimpleNamespace(nodes=[], edges=[]),
    )
    analysis = v1.analyze_topology_route(route, topology)
    assert analysis["corner_count"] >= 1
    assert analysis["corner_max_curvature_1pm"] >= v1.CORNER_CURVATURE_THRESHOLD
    start, end = analysis["corner_intervals_m"][0]
    # The turn is around s=2 m and the support interval includes one metre on
    # each side, clipped only by the route endpoints.
    assert start <= 1.0 + 1e-6
    assert end >= 3.0 - 1e-6


def test_adaptive_mask_dilates_only_corner_subset_and_intersects_raw_free():
    ctx = _context()
    raw = np.ones_like(ctx.free_mask)
    raw[30, 30] = False
    base = np.zeros_like(ctx.free_mask)
    base[20, 20] = True
    corner = np.zeros_like(ctx.free_mask)
    corner[40, 40] = True
    dilation_paddings = []

    def fake_dilate(_ctx, _centerline, padding_m):
        dilation_paddings.append(float(padding_m))
        return corner.copy() if float(padding_m) == 4.0 else np.zeros_like(raw)

    original_base = v1.candidate._build_corridor_mask
    original_dilate = v1._dilate_raw
    original_raw = v1.candidate._raw_free_mask
    try:
        v1.candidate._build_corridor_mask = lambda *_args: base.copy()
        v1._dilate_raw = fake_dilate
        v1.candidate._raw_free_mask = lambda _ctx: raw
        route = _route([[0.5, 1.0], [2.5, 1.0], [2.5, 3.0], [2.5, 5.0]])
        mask, diagnostics = v1.build_adaptive_corridor_mask(
            ctx, SimpleNamespace(graph=SimpleNamespace(nodes=[], edges=[])), route,
            SimpleNamespace(), (20, 10), (20, 80), 2.0, v1.CORRIDOR_SEMANTICS,
        )
    finally:
        v1.candidate._build_corridor_mask = original_base
        v1._dilate_raw = original_dilate
        v1.candidate._raw_free_mask = original_raw
    assert dilation_paddings == [4.0]
    assert mask[20, 20]
    assert mask[40, 40]
    assert not mask[30, 30]
    assert diagnostics["corner_count"] >= 1
    assert diagnostics["corner_corridor_mask_hash"] == v1._grid_hash(corner)
    assert diagnostics["corner_widened_area_ratio"] > 0.0


def test_builder_rejects_non_base_padding_including_six_meter():
    ctx = _context()
    route = _route([[0.5, 1.0], [4.0, 1.0]])
    with pytest.raises(ValueError):
        v1.build_adaptive_corridor_mask(
            ctx, SimpleNamespace(graph=SimpleNamespace(nodes=[], edges=[])), route,
            SimpleNamespace(), (20, 10), (20, 80), 6.0, v1.CORRIDOR_SEMANTICS,
        )


def test_v1_protocol_disables_l2_and_optional_backends():
    assert v1.ARCHITECTURE_ID == "2A-V1"
    assert v1.IMPLEMENTATION_REVISION == "r0"
    assert v1.PARENT_ARCHITECTURE == "2A-V0"
    assert v1.CORRIDOR_PROFILE == "topology_turn_adaptive_2m_4m"
    assert v1.NO_6M_PADDING is True
    assert v1.CORRIDOR_SEMANTICS == "raw_map_smac_aligned"
