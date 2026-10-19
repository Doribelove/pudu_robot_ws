"""Compiled target-stop search with r0's CSR order and equal-cost choices."""
import ctypes
from pathlib import Path

import numpy as np

_LIB = None


def goal_dijkstra(graph, src, dst):
    global _LIB
    if _LIB is None:
        path = Path(__file__).resolve().parents[2] / 'native/libgoal_dijkstra.so'
        if not path.is_file():
            raise RuntimeError('V7_R1_NATIVE_MISSING: run tools/build_goal_search.py')
        _LIB = ctypes.CDLL(str(path))
        _LIB.goal_dijkstra.argtypes = [ctypes.c_int, ctypes.c_void_p,
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_int64),
            ctypes.POINTER(ctypes.c_double)]
        _LIB.goal_dijkstra.restype = ctypes.c_int
    if (graph.indptr.dtype != np.int32 or graph.indices.dtype != np.int32
            or graph.data.dtype != np.float32 or not graph.has_sorted_indices
            or graph.shape[0] != graph.shape[1]):
        raise ValueError('V7_R1_NATIVE_REQUIRES_SORTED_FLOAT32_INT32_CSR')
    pred = np.empty(graph.shape[0], np.int32)
    count, distance = ctypes.c_int64(), ctypes.c_double()
    result = _LIB.goal_dijkstra(graph.shape[0], graph.indptr.ctypes.data,
        graph.indices.ctypes.data, graph.data.ctypes.data, int(src), int(dst),
        pred.ctypes.data, ctypes.byref(count), ctypes.byref(distance))
    if result == -1:
        raise MemoryError('V7_R1_NATIVE_SEARCH_ALLOCATION_FAILED')
    if result not in (0, 1):
        raise ValueError('V7_R1_NATIVE_SEARCH_INVALID_INPUT')
    return distance.value, pred, count.value
