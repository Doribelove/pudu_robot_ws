"""Fuse full-dirty classes without changing any actual readback comparison.

No previous map means every cell is affected and stale.  The existing exact
native classifier sees the complete original hard/soft/expected arrays once;
the affected digest is then the same full expected byte sequence.
"""
import numpy as np


def initialize(plan, semantic, expected):
    from .firstcost import enabled, soft_positive_rows
    from .request_ack import lib, supported, ByteView, view
    from .r4_tiles import TiledSoft
    if not enabled('B') or expected.size < 2**20:
        return False
    master = np.flipud(semantic.expected_master_cost)
    hard = np.flipud(semantic.hard_semantic_mask)
    shape = expected.shape
    if (len(shape) != 2 or not min(shape)
            or not supported(expected, np.int8, shape)
            or not supported(master, np.uint8, shape)
            or not supported(hard, bool, shape)
            or type(semantic.soft_cost) not in (np.ndarray, TiledSoft)
            or semantic.soft_cost.shape != shape):
        return False
    dll = lib()
    if not dll.vector_ack_available():
        return False
    source = semantic.soft_cost
    if type(source) is TiledSoft and source.cap > 0:
        # Only retained tiles can override the positive default. Outside the
        # drivable mask the original value is 200, also positive. Read the
        # same exact values (including NaNs) within every retained tile.
        positive = np.ones(shape, bool)
        for (a, b, c, d), values in source.pieces:
            positive[a:b, c:d] = (values > 0) | ~source.drivable[a:b, c:d]
        soft = positive[::-1]
    else:
        soft = soft_positive_rows(source, 0, shape[0])[::-1]
    if not supported(soft, bool, shape):
        return False
    flags = np.empty(shape, np.uint8)
    counts = np.zeros(5, np.uint64)
    bbox = np.empty(4, np.int64)
    # changed/previous placeholders cannot affect the full-dirty result.
    arrays = (expected, hard, master, hard, soft, hard)
    descriptors = (ByteView * 6)(*(view(a) for a in arrays))
    dll.classify_ack(descriptors, flags.ctypes.data, *shape, 0,
                     counts.ctypes.data, bbox.ctypes.data)
    plan.master = master
    plan.flags = flags
    plan.counts = dict(zip(('affected', 'hard', 'soft', 'stale', 'ordinary'),
                           map(int, counts)))
    plan.dirty_bbox = [0, 0, shape[1], shape[0]]
    from .tail_options import enabled as tail_enabled
    from .v9_maps import flipped_digest
    from .r6_ack import grid_digest
    plan.expected_full_hash = (flipped_digest(semantic.expected_master_cost)
                              if tail_enabled('D') else grid_digest(master))
    plan.expected_affected_hash = plan.expected_full_hash
    return True


def actual_equal(server, master):
    """Compare every actual byte with the existing exact native comparator."""
    import ctypes
    from .request_ack import lib, supported, view
    if (not supported(server, np.uint8, getattr(master, 'shape', None))
            or not supported(master, np.uint8, server.shape)
            or server.size < 2**20):
        return None
    a, b = view(server), view(master)
    return bool(lib().equal_ack(ctypes.byref(a), ctypes.byref(b), *master.shape))
