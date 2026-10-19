"""Exact soft conversion on free cells, without dense float strip expansion."""
import numpy as np
from .v9_r8_fields import sparse_supported


def occupancy_on_free_cells(source, soft, hard, rows=96):
    from .v9_storage import occupancy_on_free_support
    from arena_evaluation.semantic_costmap_composer import internal_soft_to_occupancy
    if not sparse_supported(source, soft, hard):
        return occupancy_on_free_support(source, soft, hard, rows)
    result=np.full(source.shape,100,np.int8)
    base=np.int8(internal_soft_to_occupancy(np.float32(soft.cap)))
    outside=np.int8(internal_soft_to_occupancy(np.float32(200.)))
    for a in range(0,len(source),rows):
        b=min(len(source),a+rows)
        free=~hard[a:b]
        if not np.any(free):continue
        raw=source[a:b][free]
        converted=np.maximum(raw,base)
        excluded=~soft.drivable[a:b][free]
        np.maximum(raw,outside,out=converted,where=excluded)
        converted[raw<0]=-1
        result[a:b][free]=converted
    # Preserve TiledSoft overwrite order; never convert hard tile cells.
    for (a,b,c,d),tile in soft.pieces:
        free=~hard[a:b,c:d]
        if not np.any(free):continue
        raw=source[a:b,c:d][free]
        values=tile[free].copy()
        values[~soft.drivable[a:b,c:d][free]]=200.
        converted=np.maximum(raw.astype(np.int16),internal_soft_to_occupancy(values)).astype(np.int8)
        converted[raw<0]=-1
        result[a:b,c:d][free]=converted
    return result
