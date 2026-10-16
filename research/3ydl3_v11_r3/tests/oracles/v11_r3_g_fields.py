"""Exact conversion of sparse float fields and immutable composed storage.

The original float-to-occupancy converter still processes every varying tile.
Only its constant background work is replaced by the same scalar conversion.
No original soft value, pre-bound policy byte, source obstacle or ACK changes.
"""
from dataclasses import replace
import math
import numpy as np
from arena_evaluation.semantic_costmap_composer import internal_soft_to_occupancy
from .r4_tiles import TiledSoft


def sparse_supported(source, soft, hard):
    if (type(soft) is not TiledSoft or not isinstance(source, np.ndarray)
            or source.ndim != 2 or source.dtype != np.dtype(np.int8)
            or not isinstance(hard, np.ndarray) or hard.dtype != np.dtype(bool)
            or hard.shape != source.shape or soft.shape != source.shape
            or not isinstance(soft.drivable, np.ndarray)
            or soft.drivable.shape != source.shape or soft.drivable.dtype != np.dtype(bool)
            or not math.isfinite(soft.cap)):
        return False
    h, w = source.shape
    for box, values in soft.pieces:
        if (len(box) != 4 or any(not isinstance(v, (int, np.integer)) for v in box)
                or not isinstance(values, np.ndarray) or values.dtype != np.dtype(np.float32)):
            return False
        a, b, c, d = box
        if not (0 <= a < b <= h and 0 <= c < d <= w) or values.shape != (b-a, d-c):
            return False
    return True


def sparse_occupancy(source, soft, hard, rows=96):
    """Return the original pre-bound occupancy, including hard/unknown cells."""
    from .v9_storage import occupancy_on_free_support
    if not sparse_supported(source, soft, hard):
        return occupancy_on_free_support(source, soft, hard, rows), {
            'r8_sparse_occupancy_used': False, 'r8_sparse_occupancy_fallback': 'unsupported_field'}
    # TiledSoft creates float32 backgrounds before the original converter
    # widens them to float64. Preserve that first rounding for arbitrary caps.
    base = np.int8(internal_soft_to_occupancy(np.float32(soft.cap)))
    outside = np.int8(internal_soft_to_occupancy(np.float32(200.)))
    result = np.empty(source.shape, np.int8)
    for a in range(0, len(source), rows):
        b = min(len(source), a+rows); raw = source[a:b]
        values = np.maximum(raw, base)
        excluded = ~soft.drivable[a:b]
        np.maximum(raw, outside, out=values, where=excluded)
        values[raw < 0] = -1
        values[hard[a:b]] = 100
        result[a:b] = values
    cells = 0
    # The original field applies pieces in this order; later overlaps win.
    # Quantize original float32 values with the unchanged original converter.
    for (a, b, c, d), values in soft.pieces:
        raw = source[a:b, c:d]
        converted = np.maximum(raw.astype(np.int16), internal_soft_to_occupancy(values)).astype(np.int8)
        excluded = ~soft.drivable[a:b, c:d]
        np.maximum(raw, outside, out=converted, where=excluded)
        converted[raw < 0] = -1
        converted[hard[a:b, c:d]] = 100
        result[a:b, c:d] = converted
        cells += values.size
    return result, {'r8_sparse_occupancy_used': True,
                    'r8_varying_soft_cells_converted': int(cells),
                    'r8_background_conversion': 'original converter on original float32 scalars',
                    'r8_full_prebound_occupancy_preserved': True}


def freeze_array(value):
    """Use immutable bytes; retain scalar broadcasts without a dense buffer."""
    from .v9_maps import immutable, seal
    from .compact_auxiliary import TYPES
    if type(value) in TYPES:return value
    if immutable(value):
        return value
    if value.size and all(stride == 0 for stride in value.strides):
        scalar = value[(0,)*value.ndim]
        backing = np.frombuffer(np.asarray(scalar, dtype=value.dtype).tobytes(), value.dtype)
        return np.broadcast_to(backing.reshape((1,)*value.ndim), value.shape)
    return seal(value)


def freeze_auxiliary(grid):
    """Seal only newly composed fields; live mutable external grids keep guards."""
    names = ('internal_cost', 'hard_semantic_mask', 'affected_mask')
    values = [getattr(grid, name) for name in names]
    from .compact_auxiliary import TYPES
    if any((not isinstance(v, np.ndarray) and type(v) not in TYPES) or v.dtype.hasobject for v in values):
        return grid
    soft = grid.soft_cost
    if type(soft) is TiledSoft:
        arrays = [soft.drivable, *(v for _, v in soft.pieces)]
        if any(not isinstance(v, np.ndarray) or v.dtype.hasobject for v in arrays):
            return grid
        soft = TiledSoft(soft.shape, soft.cap, freeze_array(soft.drivable),
                        [(tuple(box), freeze_array(v)) for box, v in soft.pieces])
    elif isinstance(soft, np.ndarray) and not soft.dtype.hasobject:
        soft = freeze_array(soft)
    else:
        return grid
    return replace(grid, **{name: freeze_array(value) for name, value in zip(names, values)},
                   soft_cost=soft,
                   diagnostics={**grid.diagnostics, 'r8_immutable_auxiliary_fields': True})


def freeze_owned_scope(mask):
    """Seal a newly generated owning planner mask at its construction boundary.

    This does not freeze dynamic/source map inputs or arbitrary alias views.
    Replacing a scope or changing a mutable dynamic mask still changes the
    original target cache key. No hash or live-field/ACK check is removed.
    """
    from .v9_r8_options import enabled
    from .v9_options import enabled as v9_enabled
    from .v9_maps import immutable, seal
    if not enabled('G') or not v9_enabled('A'):
        return mask
    if (type(mask) is not np.ndarray or mask.ndim != 2 or mask.dtype != np.dtype(bool)
            or not mask.flags.c_contiguous):
        return mask
    if immutable(mask):
        return mask
    if not mask.flags.owndata:
        return mask
    return seal(mask)
