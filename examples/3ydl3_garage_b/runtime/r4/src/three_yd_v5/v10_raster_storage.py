"""Exact binary-mask storage adapter during serial map construction.

The pinned rasterizer writes only bytes 0/1. A bool view is value-identical
and avoids copying the full image after each small feature. No labels,
polygon rounding, fill policy or topology algorithm change. Hooks are scoped
and restored on failure; unknown upstream code uses the original function.
"""
from contextlib import contextmanager
import hashlib,inspect,os,threading
import cv2
import numpy as np
from arena_evaluation import semantic_rasterizer as raster
ORIGINAL=raster.rasterize_feature
PINNED_SHA256='9ce00a5653dcb4a86eda5f19b913e51b4d80df270ccf7aaa0d95bce64267175b'
_LOCK=threading.RLock()

def binary_view(semantic_map,feature):
    mask=np.zeros((semantic_map.height,semantic_map.width),dtype=np.uint8)
    points=raster._world_polygon_cells(semantic_map,feature)
    if feature.geometry_type=='polygon' and len(points)>=3:
        cv2.fillPoly(mask,[points],1,lineType=cv2.LINE_8)
    elif feature.geometry_type=='line' and len(points)>=2:
        cv2.polylines(mask,[points],False,1,thickness=1,lineType=cv2.LINE_8)
    elif feature.geometry_type=='point' and len(points):
        mask[points[0,1],points[0,0]]=1
    return mask.view(np.bool_)

@contextmanager
def construction_storage():
    from arena_3d_v1 import semantic_world
    from . import topology
    targets=(raster,semantic_world,topology)
    diagnostics={'enabled':False,'calls':0,'avoided_copy_bytes':0,'frozen_topology_source_changed':False}
    with _LOCK:
        known=hashlib.sha256(inspect.getsource(ORIGINAL).encode()).hexdigest()==PINNED_SHA256
        if os.environ.get('THREEYD_V10_RASTER_STORAGE','1')!='1' or not known or any(m.rasterize_feature is not ORIGINAL for m in targets):
            diagnostics['reason']='disabled_or_unrecognized_rasterizer';yield diagnostics;return
        def counted(s,f):
            value=binary_view(s,f);diagnostics['calls']+=1;diagnostics['avoided_copy_bytes']+=value.nbytes;return value
        diagnostics['enabled']=True
        for m in targets:m.rasterize_feature=counted
        try:yield diagnostics
        finally:
            for m in targets:m.rasterize_feature=ORIGINAL
