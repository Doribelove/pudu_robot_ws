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
    for begin in range(0,shape[0]*shape[1],chunk):
        ix=np.arange(begin,min(begin+chunk,shape[0]*shape[1]));rows=ix//shape[1];cols=ix%shape[1]
        fr,fc=fine_rows[rows],fine_cols[cols]
        labels=np.zeros(len(ix),top.subregions.dtype)
        valid=(fr>=0)&(fc>=0)&(fr<w.map.height)&(fc<w.map.width)
        labels[valid]=top.subregions[fr[valid],fc[valid]]
        for rid,d in top.contexts[controller.topology_route.signature]['unique_roads']:
            selected=labels==rid
            direction.reshape(-1,2)[ix[selected]]=d
            if np.any(selected):
                xy=view.world_points(rows[selected]+r0,cols[selected]+c0)
                norm,_,_=cross_section(top.router.polygons[rid],d,xy)
                normalized.ravel()[ix[selected]]=norm
                eligible.ravel()[ix[selected]]=np.isfinite(norm)
    return normalized,eligible,direction
