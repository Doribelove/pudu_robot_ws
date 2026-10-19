"""Bound temporary geometry work; omit absent semantic fields without changing values."""
import numpy as np
from three_yd_v4.r5_guidance import cross_section

def _road_fields_original(controller,view,bbox,chunk=65536):
    r0,r1,c0,c1=bbox;shape=(r1-r0,c1-c0)
    if controller.generic_mode:
        return (np.broadcast_to(np.array(np.nan,np.float32),shape),
                np.broadcast_to(np.array(False),shape),
                np.broadcast_to(np.array(0.,np.float32),(*shape,2)))
    normalized=np.full(shape,np.nan,np.float32);eligible=np.zeros(shape,bool)
    direction=np.zeros((*shape,2),np.float32)
    w=controller.world;top=controller.topology
    for begin in range(0,shape[0]*shape[1],chunk):
        ix=np.arange(begin,min(begin+chunk,shape[0]*shape[1]))
        xy=view.world_points(ix//shape[1]+r0,ix%shape[1]+c0);fr,fc=w.cells(xy)
        labels=np.zeros(len(ix),top.subregions.dtype)
        valid=(fr>=0)&(fc>=0)&(fr<w.map.height)&(fc<w.map.width)
        labels[valid]=top.subregions[fr[valid],fc[valid]]
        for rid,d in top.contexts[controller.topology_route.signature]['unique_roads']:
            selected=labels==rid
            direction.reshape(-1,2)[ix[selected]]=d
            if np.any(selected):
                norm,_,_=cross_section(top.router.polygons[rid],d,xy[selected])
                normalized.ravel()[ix[selected]]=norm
                eligible.ravel()[ix[selected]]=np.isfinite(norm)
    return normalized,eligible,direction

def resident(*arrays):
    seen=set();total=0
    for value in arrays:
        a=value
        while isinstance(a.base,np.ndarray):a=a.base
        if id(a) not in seen:seen.add(id(a));total+=a.nbytes
    return int(total)

def point_free(world,view,scope_cells,fine_hard,shape):
    # This field is ANDed with the scope in the original implementation.
    # Coordinates outside that scope therefore need not be constructed.
    xy=view.world_points(scope_cells[:,0],scope_cells[:,1]);r,c=world.cells(xy)
    valid=(r>=0)&(c>=0)&(r<world.map.height)&(c<world.map.width)
    result=np.zeros(shape,bool);rc=scope_cells[valid]
    result[rc[:,0],rc[:,1]]=~fine_hard[r[valid],c[valid]]
    return result


def road_fields(controller,view,bbox,chunk=65536):
    from .localfix_options import enabled
    if enabled('X') and not controller.generic_mode:
        from .local_road_fields import road_fields as exact_fields
        result=exact_fields(controller,view,bbox,chunk)
        if result is not None:return result
    return _road_fields_original(controller,view,bbox,chunk)
