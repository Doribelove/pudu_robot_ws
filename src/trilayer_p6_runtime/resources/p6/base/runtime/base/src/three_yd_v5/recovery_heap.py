"""Return unused glibc heap pages after one-time map/service preparation.

This releases only allocator-owned free memory. Live map, topology, query,
controller, ACK and audit storage is neither traversed nor changed.
Platforms without malloc_trim retain their ordinary allocator behavior.
"""
import ctypes
import time


def release_preparation_heap():
    tick = time.monotonic()
    try:
        trim = ctypes.CDLL(None).malloc_trim
    except (AttributeError, OSError):
        return {'supported': False, 'released': False, 'wall_ms': 0.}
    trim.argtypes = [ctypes.c_size_t]
    trim.restype = ctypes.c_int
    released = bool(trim(0))
    return {'supported': True, 'released': released,
            'wall_ms': (time.monotonic()-tick)*1000}
