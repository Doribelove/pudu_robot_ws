"""Reuse hashes only for arrays backed by genuinely immutable bytes.

Readonly flags on an owning ndarray are insufficient: an alias can mutate its
storage or a caller can re-enable writes. Mutable inputs are always rehashed.
The small identity cache holds weak references and is released with the maps.
Actual server observations are never cached by this module.
"""
import hashlib
import weakref
import time
from dataclasses import replace,fields
import numpy as np
from .r6_ack import grid_digest
from .v9_options import enabled

_HASHES={}
_FLIPS={}

def immutable(a):
    current=a
    while isinstance(current,np.ndarray):
        if current.flags.writeable:return False
        current=current.base
    return isinstance(current,bytes)

def seal(a):
    a=np.asarray(a)
    if immutable(a):return a
    return np.frombuffer(np.ascontiguousarray(a).tobytes(),dtype=a.dtype).reshape(a.shape)

def content_digest(a):
    a=np.asarray(a)
    if not enabled('A'):
        from .v9_review import enabled as review_enabled
        if review_enabled('S'):return grid_digest(a)
        from arena_3d_v1.semantic_world import digest
        return digest(a)
    if not immutable(a):return grid_digest(a)
    key=id(a);old=_HASHES.get(key)
    if old is not None and old[0]() is a:return old[1]
    value=grid_digest(a)
    def remove(ref):
        entry=_HASHES.get(key)
        if entry is not None and entry[0] is ref:_HASHES.pop(key,None)
    _HASHES[key]=(weakref.ref(a,remove),value)
    return value

def seal_grid(grid):
    from .v9_review import enabled as review_enabled
    if enabled('A'):
        grid=replace(grid,occupancy_grid=seal(grid.occupancy_grid),
            expected_master_cost=seal(grid.expected_master_cost),
            diagnostics={**grid.diagnostics,'v9_immutable_composed_map':True})
        from .v9_r8_options import enabled as r8_enabled
        if r8_enabled('F'):
            from .v9_r8_fields import freeze_auxiliary
            grid=freeze_auxiliary(grid)
    if review_enabled('S'):
        from .v9_storage import HashSemanticCostmap
        grid=HashSemanticCostmap(**{f.name:getattr(grid,f.name) for f in fields(grid)})
    return grid

def prepare_world(w):
    if not enabled('A'):return
    tick=time.monotonic()
    w.map.occupancy=seal(w.map.occupancy)
    w.hard=seal(w.hard)
    if getattr(w,'v7_map_only',False):
        from .v9_r7_options import enabled as neutral_enabled
        replacement=None
        if neutral_enabled():
            from .v9_r7_neutral import neutral_labels
            replacement,w.v9_r7_neutral_diagnostics=neutral_labels(w)
        w._labels=seal(w.labels) if replacement is None else replacement
    else:w.labels=seal(w.labels)
    w.v9_preparation_ms=(time.monotonic()-tick)*1000

def flow_digest(a):
    return content_digest(a) if enabled("A") else grid_digest(a)

def flipped_digest(a):
    a=np.asarray(a)
    if not enabled('A') or not immutable(a):return grid_digest(np.flipud(a))
    key=id(a);old=_FLIPS.get(key)
    if old is not None and old[0]() is a:return old[1]
    value=grid_digest(np.flipud(a))
    def remove(ref):
        entry=_FLIPS.get(key)
        if entry is not None and entry[0] is ref:_FLIPS.pop(key,None)
    _FLIPS[key]=(weakref.ref(a,remove),value)
    return value

def stats():return {'immutable_hash_entries':len(_HASHES),'flipped_hash_entries':len(_FLIPS)}

def clear():_HASHES.clear();_FLIPS.clear()
