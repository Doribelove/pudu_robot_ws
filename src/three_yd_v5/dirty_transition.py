"""Exact mask transition without constructing a redundant applied-grid copy.

All input content is compared every call. No object-ID/content-cache shortcut
and no bypass of the session's effective costmap ACK. Changed cells determine
counts/bbox; the logical old/new update has zero residual by construction.
"""
import hashlib
import numpy as np
from arena_3d_v1.pipeline import DirtyROI,corridor_dirty_transition as original_transition
from .r2_options import enabled


def corridor_dirty_transition(old_mask,new_mask):
    if not enabled('B'):return original_transition(old_mask,new_mask)
    old=np.asarray(old_mask,dtype=bool);new=np.asarray(new_mask,dtype=bool)
    if old.shape!=new.shape:raise ValueError('old and new L3 masks must have identical shapes')
    target_hash=hashlib.sha256(memoryview(np.ascontiguousarray(new)).cast('B') if new.size else b'').hexdigest()
    dirty=old^new
    if not dirty.any():return DirtyROI(None,0,0,0,target_hash,0)
    from .firstplan import enabled as first_enabled
    if first_enabled("A"):
        rows=np.flatnonzero(dirty.any(axis=1));columns=np.flatnonzero(dirty.any(axis=0))
        changed=int(np.count_nonzero(dirty));opened=int(np.count_nonzero(new&dirty))
        bbox=(int(rows[0]),int(rows[-1])+1,int(columns[0]),int(columns[-1])+1)
        return DirtyROI(bbox,changed,changed-opened,opened,target_hash,0)
    rows,columns=np.nonzero(dirty)
    changed=len(rows);opened=int(np.count_nonzero(new[rows,columns]))
    bbox=(int(rows.min()),int(rows.max())+1,int(columns.min()),int(columns.max())+1)
    return DirtyROI(bbox,changed,changed-opened,opened,target_hash,0)
