"""Exact immutable storage sharing at two original initialization boundaries.

No map, obstacle, search, audit or ACK decision is reused. Static equality is
checked against both current arrays. The search scope is derived from the same
current priority that the original initialization copies, before any search.
"""
import numpy as np
from .v9_maps import immutable, seal
from .v9_options import enabled as maps_enabled
from .v9_r9_options import enabled

_STATS = {'static_shared_calls': 0, 'static_original_seal_calls': 0,
          'scope_shared_calls': 0, 'scope_original_copy_calls': 0,
          'static_duplicate_buffer_bytes_avoided': 0,
          'scope_duplicate_buffer_bytes_avoided': 0}


def active():
    return enabled('U') and maps_enabled('A')


def initial_static_hard(world_hard, static_hard):
    """Reuse only equal immutable current world bytes; otherwise original seal."""
    supported = (active() and type(world_hard) is np.ndarray
                 and type(static_hard) is np.ndarray and world_hard.ndim == 2
                 and world_hard.dtype == static_hard.dtype == np.dtype(bool)
                 and world_hard.shape == static_hard.shape and world_hard.size > 0
                 and world_hard.flags.c_contiguous and static_hard.flags.c_contiguous
                 and static_hard.flags.owndata and static_hard.flags.writeable
                 and immutable(world_hard))
    if supported and all(np.array_equal(world_hard[a:a+64], static_hard[a:a+64])
                         for a in range(0, len(world_hard), 64)):
        _STATS['static_shared_calls'] += 1
        _STATS['static_duplicate_buffer_bytes_avoided'] += int(static_hard.nbytes)
        return world_hard
    _STATS['static_original_seal_calls'] += 1
    return seal(static_hard)


def initial_search_scope(priority, generic):
    """One new immutable buffer for identical map-only priority/search scope."""
    supported = (active() and generic is True and type(priority) is np.ndarray
                 and priority.ndim == 2 and priority.dtype == np.dtype(bool)
                 and priority.size > 0 and priority.flags.c_contiguous
                 and priority.flags.owndata and priority.flags.writeable)
    if supported:
        shared = seal(priority)
        _STATS['scope_shared_calls'] += 1
        _STATS['scope_duplicate_buffer_bytes_avoided'] += int(priority.nbytes)
        return shared, shared
    _STATS['scope_original_copy_calls'] += 1
    return priority, priority.copy()


def buffer_stats():
    return {**_STATS, 'mode_enabled': enabled('U'),
            'static_or_scope_content_cached': False,
            'audit_or_ACK_results_cached': False,
            'byte_counters_are_logical_duplicate_storage_not_RSS': True}
