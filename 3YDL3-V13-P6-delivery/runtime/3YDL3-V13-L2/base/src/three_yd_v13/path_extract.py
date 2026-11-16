"""Equivalent read-only native extraction; unsupported models use the original."""
import ctypes
from pathlib import Path
import numpy as np
from arena_3d_v1.l2_state_lifecycle import OFFSETS, OFFSET_INDEX

_LIB = None
_SIDES = np.asarray([
    (OFFSET_INDEX[(0, dc)], OFFSET_INDEX[(dr, 0)]) if dr and dc else (-1, -1)
    for dr, dc in OFFSETS
], dtype=np.int64)


def supported(state):
    arrays = [(state.geometry.neighbor_deltas, np.int16),
              (state.edge_weights, np.float64), (state.g, np.float64),
              (state.blocked, np.bool_), (state._rows, np.int32),
              (state._cols, np.int32)]
    return not hasattr(state, 'approach_strength') and all(
        np.asarray(a).dtype == dtype and np.asarray(a).flags.c_contiguous
        for a, dtype in arrays)


def extract(state, max_length=None):
    global _LIB
    if state.invalid_extraction_injected:
        return None
    n = state.geometry.state_count
    limit = max_length or max(2, n * 2)
    # A simple path cannot contain more than n distinct nodes. If the legacy
    # limit exceeds n, the visited-set check still rejects the same cycle.
    capacity = min(n, max(1, limit))
    if _LIB is None:
        _LIB = ctypes.CDLL(str(Path(__file__).resolve().parents[2] / 'native/libv13_extract_path.so'))
        _LIB.extract_path.argtypes = [ctypes.c_int64] * 4 + [ctypes.c_void_p] * 8
        _LIB.extract_path.restype = ctypes.c_int64
    if limit < 1:
        return [state.start_id] if state.start_id == state.goal_id and np.isfinite(state.g[state.start_id]) else None
    path = np.empty(capacity, dtype=np.int64)
    length = _LIB.extract_path(n, state.start_id, state.goal_id, capacity,
        state.geometry.neighbor_deltas.ctypes.data, state.edge_weights.ctypes.data,
        state.g.ctypes.data, state.blocked.ctypes.data, _SIDES.ctypes.data,
        np.asarray(state._rows).ctypes.data, np.asarray(state._cols).ctypes.data,
        path.ctypes.data)
    if length < 0:
        raise ValueError('V13_NATIVE_EXTRACTION_INVALID_GRAPH')
    return path[:length].tolist() if length else None
