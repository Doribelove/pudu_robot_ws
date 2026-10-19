"""Exact storage for the frozen map-only world's neutral label field.

No new semantics, map crop, changed input, or safety/algorithm change. A
readonly broadcast holds the same int16 zeros at every original map cell.
"""
import hashlib
import inspect
from functools import lru_cache
import numpy as np

LABEL_PROPERTY_SHA256='659ad3746bb9dcb81f4462717c19a46e404ac49ced18e49ea03b22d326dc1679'


@lru_cache(maxsize=8)
def original_property(function):
    try:
        return hashlib.sha256(inspect.getsource(function).encode()).hexdigest()==LABEL_PROPERTY_SHA256
    except (TypeError,OSError):
        return False


def neutral_labels(world):
    from .free_space_topology import MapOnlyWorld
    log={'candidate':'R7-neutral-int16-zero-storage','enabled':False}
    def reject(reason):
        return None,{**log,'rejection':reason,'fallback_original_storage':True}
    if type(world) is not MapOnlyWorld or not getattr(world,'v7_map_only',False):
        return reject('NOT_EXACT_ORIGINAL_MAP_ONLY_WORLD')
    descriptor=getattr(MapOnlyWorld,'labels',None)
    if not isinstance(descriptor,property) or not original_property(descriptor.fget):
        return reject('ORIGINAL_NEUTRAL_LABEL_PROPERTY_CHANGED')
    if world.features or world.rules:
        return reject('SEMANTIC_FEATURES_OR_RULES_PRESENT')
    shape=(world.map.height,world.map.width)
    if len(shape)!=2 or any(not isinstance(v,(int,np.integer)) or v<=0 for v in shape):
        return reject('INVALID_ORIGINAL_MAP_SHAPE')
    if not isinstance(world.hard,np.ndarray) or world.hard.shape!=shape:
        return reject('HARD_MAP_SHAPE_MISMATCH')
    old=world._labels
    if old is not None:
        if not isinstance(old,np.ndarray) or old.shape!=shape or old.dtype!=np.dtype(np.int16):
            return reject('ORIGINAL_LABEL_SHAPE_OR_DTYPE_MISMATCH')
        # Initialization is serial. Check every actual old cell; no sample
        # or inferred uniformity is used if the property has been evaluated.
        for a in range(0,shape[0],64):
            if np.any(old[a:a+64]!=0):
                return reject('NONZERO_ORIGINAL_LABELS')
    scalar=np.frombuffer(bytes(2),dtype=np.int16)
    labels=np.broadcast_to(scalar,shape)
    from .v9_maps import immutable
    assert immutable(labels) and labels.dtype==np.int16 and labels.shape==shape
    return labels,{**log,'enabled':True,'original_property_verified':True,
                   'original_existing_cells_all_zero_verified':old is not None,
                   'original_property_lazy_zero_contract_used':old is None,
                   'original_shape':list(shape),'dtype':'int16','logical_bytes':int(labels.nbytes),
                   'new_backing_buffer_bytes':2,'broadcast_readonly':True,
                   'storage_values_unchanged':True,'RSS_improvement_not_measured':True}
