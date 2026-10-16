"""V10-r3 request storage and exact wide-label lookup experiments."""
import ctypes
import os
from pathlib import Path
import numpy as np

_LIB = None
_COUNTS = {'wide_calls': 0, 'wide_cells': 0, 'compact_states': 0}


def enabled(part):
    mode = os.environ.get('THREEYD_V10_R3_MODE', 'control')
    return mode != 'control' and part in mode


def wide_membership(labels, elements):
    """Return None for unsupported input; preserve original NumPy membership."""
    if (not enabled('B') or type(labels) is not np.ndarray or labels.ndim != 2
            or labels.dtype not in (np.dtype('int16'), np.dtype('uint16'))
            or not labels.flags.c_contiguous or not labels.dtype.isnative
            or type(elements) not in (list, tuple, np.ndarray)):
        return None
    try:
        values = np.asarray(elements)
        if values.size and values.dtype.kind not in 'iub':
            return None
        # The table covers every bit pattern of the source dtype. Use NumPy's
        # own comparison/conversion rules, including out-of-domain IDs.
        domain = np.arange(65536, dtype=np.uint16).view(labels.dtype)
        table = np.isin(domain, elements).astype(np.uint8)
    except (TypeError, ValueError, OverflowError, IndexError):
        return None
    global _LIB
    if _LIB is None:
        path = Path(__file__).resolve().parents[2] / 'native/librequest_membership.so'
        _LIB = ctypes.CDLL(str(path))
        _LIB.request_membership_u16.argtypes = [ctypes.c_void_p] * 3 + [ctypes.c_size_t]
        _LIB.request_membership_u16.restype = None
    result = np.empty(labels.shape, np.bool_)
    _LIB.request_membership_u16(labels.ctypes.data, table.ctypes.data,
                               result.ctypes.data, labels.size)
    _COUNTS['wide_calls'] += 1
    _COUNTS['wide_cells'] += labels.size
    return result


def initialize_compact_state(state, geometry, binding, potential, weight, directions, progress):
    from arena_3d_v1.semantic_l2 import WeightedDStarState
    from .firstplan import state_coordinates
    state.xy_direction = np.asarray(directions, float)
    state.progress = np.asarray(progress, np.int16)
    state.cells = state_coordinates(geometry)
    # Memoryviews return Python int/float just like the old lists. Compact
    # copies also preserve the old snapshots if source arrays later mutate.
    state._rows = memoryview(state.cells[:, 0].copy())
    state._cols = memoryview(state.cells[:, 1].copy())
    state._dx = memoryview(state.xy_direction[:, 0].copy())
    state._dy = memoryview(state.xy_direction[:, 1].copy())
    WeightedDStarState.__init__(state, geometry, binding, potential, weight)
    _COUNTS['compact_states'] += 1


def stats():
    from .request_interpolation import stats as interpolation_stats
    return {**_COUNTS, 'mode': os.environ.get('THREEYD_V10_R3_MODE', 'control'),
            'input_content_cached': False, 'table_cached': False,
            'interpolation':interpolation_stats()}


def request_digest(value):
    if enabled('H'):
        from .v9_maps import content_digest
        return content_digest(value)
    from arena_3d_v1.semantic_world import digest
    return digest(value)
