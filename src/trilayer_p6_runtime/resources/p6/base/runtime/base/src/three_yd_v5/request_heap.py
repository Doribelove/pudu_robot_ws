"""Bound allocator slack after request computations and before returning.

This runs inside the navigation request. It touches no live map or planner
object, retains a 32 MiB top reserve, and skips small free heaps. Other libc
implementations keep their original behavior. Native child allocators and
global allocator thresholds remain unchanged.
"""
import ctypes
import os
import time
from functools import lru_cache

FREE_THRESHOLD = 64 * 2**20
TOP_RESERVE = 32 * 2**20


class Mallinfo(ctypes.Structure):
    _fields_=[(name,ctypes.c_size_t) for name in
        ('arena','ordblks','smblks','hblks','hblkhd','usmblks','fsmblks','uordblks','fordblks','keepcost')]


@lru_cache(maxsize=1)
def backend():
    try:
        if not os.confstr('CS_GNU_LIBC_VERSION').startswith('glibc '):return None
        lib=ctypes.CDLL(None);info=lib.mallinfo2;trim=lib.malloc_trim
        info.argtypes=[];info.restype=Mallinfo
        trim.argtypes=[ctypes.c_size_t];trim.restype=ctypes.c_int
        return info,trim
    except (AttributeError,OSError,ValueError):return None


def release_retired_heap():
    tick=time.monotonic();api=backend()
    out={'supported':api is not None,'threshold_bytes':FREE_THRESHOLD,'top_reserve_bytes':TOP_RESERVE,
         'triggered':False,'released':False,'included_in_navigation':True}
    if api is not None:
        info,trim=api;before=info()
        out['free_before_bytes']=int(before.fordblks)
        out['arena_before_bytes']=int(before.arena)
        if before.fordblks>=FREE_THRESHOLD:
            out['triggered']=True;out['released']=bool(trim(TOP_RESERVE))
            after=info();out['free_after_bytes']=int(after.fordblks)
            out['arena_after_bytes']=int(after.arena)
    out['wall_ms']=(time.monotonic()-tick)*1000
    return out
