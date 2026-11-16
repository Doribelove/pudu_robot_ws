"""Exact separable coordinate mapping for the original Cartesian grid APIs."""
import numpy as np
from arena_3d_v1.semantic_world import SemanticWorld
from arena_3d_v1.multires_grid import GridView
from three_yd_v4.r5_guidance import cross_section


def road_fields(controller,view,bbox,chunk=65536):
    w=controller.world
    if (getattr(w.cells,'__func__',None) is not SemanticWorld.cells or
        getattr(view.world_points,'__func__',None) is not GridView.world_points):
        return None
    r0,r1,c0,c1=bbox;shape=(r1-r0,c1-c0);top=controller.topology
    if not shape[0] or not shape[1]:return None
    rr=np.arange(r0,r1);cc=np.arange(c0,c1)
    fine_rows=w.cells(view.world_points(rr,np.full(rr.shape,c0)))[0]
    fine_cols=w.cells(view.world_points(np.full(cc.shape,r0),cc))[1]
    normalized=np.full(shape,np.nan,np.float32);eligible=np.zeros(shape,bool)
    direction=np.zeros((*shape,2),np.float32)
    valid_rows=(fine_rows>=0)&(fine_rows<w.map.height)
    valid_cols=(fine_cols>=0)&(fine_cols<w.map.width)
    if valid_rows.all() and valid_cols.all():
        labels=top.subregions[fine_rows[:,None],fine_cols[None,:]]
    else:
        labels=np.zeros(shape,top.subregions.dtype)
        rows=np.flatnonzero(valid_rows);cols=np.flatnonzero(valid_cols)
        labels[rows[:,None],cols[None,:]]=top.subregions[fine_rows[rows,None],fine_cols[None,cols]]
    for rid,d in top.contexts[controller.topology_route.signature]['unique_roads']:
        rows,cols=np.nonzero(labels==rid)
        direction[rows,cols]=d
        if len(rows):
            xy=view.world_points(rows+r0,cols+c0)
            norm,_,_=cross_section(top.router.polygons[rid],d,xy)
            normalized[rows,cols]=norm
            eligible[rows,cols]=np.isfinite(norm)
    return normalized,eligible,direction
