"""Fresh exact boolean route-scope aggregation at the L2 data boundary.

This replaces only any-of-nine aggregation with false padding and a physical
bottom-left origin. Nine strided views avoid copying the complete fine grid
into a padded array. The original search and every unsupported input retain
the original aggregation path. No input, geometry, audit or ACK is cached.
"""
import numpy as np
from arena_3d_v1.multires_grid import aggregate as _original
from .v9_r9_options import enabled

_STATS = {'exact_scope_calls': 0, 'original_fallback_calls': 0, 'disabled_calls': 0,
          'fine_cells_processed': 0, 'logical_padded_bool_buffer_bytes_avoided': 0}


def aggregate_scope(mask, factor):
    if not enabled('X'):
        _STATS['disabled_calls'] += 1
        return _original(mask, factor)
    if (type(mask) is not np.ndarray or mask.ndim != 2
            or mask.dtype != np.dtype(bool) or not mask.size
            or not mask.flags.c_contiguous or type(factor) is not int or factor != 3):
        _STATS['original_fallback_calls'] += 1
        return _original(mask, factor)
    h, w = mask.shape
    rows, columns = (h+2)//3, (w+2)//3
    bottom = mask[::-1]
    # Missing fine cells in the final physical top/right blocks are false.
    out = np.zeros((rows, columns), dtype=bool)
    for i in range(3):
        for j in range(3):
            plane = bottom[i::3, j::3]
            target = out[:plane.shape[0], :plane.shape[1]]
            np.logical_or(target, plane, out=target)
    result = np.ascontiguousarray(out[::-1])
    _STATS['exact_scope_calls'] += 1
    _STATS['fine_cells_processed'] += int(mask.size)
    _STATS['logical_padded_bool_buffer_bytes_avoided'] += rows*3*columns*3
    return result


def scope_stats():
    return {**_STATS, 'mode_enabled': enabled('X'),
            'current_input_or_scope_result_cached': False,
            'audit_or_ACK_results_cached': False,
            'logical_padding_byte_counter_is_not_RSS': True,
            'aggregation': 'original bottom-left3x3 any, false padding'}
