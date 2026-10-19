"""Exact one-byte StaticLayer lookup, with the original unknown-layout path.

The table is still freshly produced by the original all256-value converter.
Every call reads the current source bytes and allocates a new uint8 result.
No source, geometry, policy, ACK or audit decisions are cached or changed.
"""
import numpy as np
import cv2
from .v9_r9_options import enabled

_COUNTS = {'native_LUT_calls': 0, 'original_fallback_calls': 0, 'disabled_calls': 0}


def exact_lookup(occupancy, table):
    if not enabled('L'):
        _COUNTS['disabled_calls'] += 1
        return None
    if (type(occupancy) is not np.ndarray or occupancy.dtype != np.dtype(np.int8)
            or occupancy.ndim != 2 or not min(occupancy.shape, default=0)
            or not occupancy.flags.c_contiguous
            or type(table) is not np.ndarray or table.dtype != np.dtype(np.uint8)
            or table.shape != (256,) or not table.flags.c_contiguous):
        _COUNTS['original_fallback_calls'] += 1
        return None
    try:
        result = cv2.LUT(occupancy.view(np.uint8), table)
    except cv2.error:
        _COUNTS['original_fallback_calls'] += 1
        return None
    if result is None or result.dtype != np.dtype(np.uint8) or result.shape != occupancy.shape:
        _COUNTS['original_fallback_calls'] += 1
        return None
    _COUNTS['native_LUT_calls'] += 1
    return result


def lookup_stats():
    return {**_COUNTS, 'mode_enabled': enabled('L'), 'input_content_cached': False,
            'OpenCV_threads': cv2.getNumThreads()}
