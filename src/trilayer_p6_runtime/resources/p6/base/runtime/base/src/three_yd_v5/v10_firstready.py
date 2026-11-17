"""Exact storage operations for the two pinned serial map constructors.

Small feature masks are passed as integer pixel indices to NumPy assignment.
Large features retain the parent's dense mask to bound temporary memory.
Only SemanticWorld's threshold-only EDT expression is evaluated as a lattice
predicate. This never supplies approximate distances to a caller. Frozen
constructors, priorities, graph search, geometry and auditing stay unchanged.
"""
from contextlib import contextmanager
import hashlib,inspect,math,os,threading
import cv2
import numpy as np
from scipy import ndimage
from arena_evaluation import semantic_rasterizer as raster
from .v10_raster_storage import binary_view,construction_storage,ORIGINAL,PINNED_SHA256

WORLD_INIT_SHA256='294328478ee50e2a06c0d5d85d16fe57a02c86f5b2f284873f3e8ab3aa6c63aa'
TOPOLOGY_INIT_SHA256='03dd02f66238201e0fe65ef366ce2abf561f8589ab053e999fb0bbb5d3e86239'
_LOCK=threading.RLock()
_EDT=ndimage.distance_transform_edt

class PixelIndex(tuple):
    """Pixel assignment index plus the pinned constructor's boolean OR."""
    def __new__(cls,rows,cols,shape):
        result=super().__new__(cls,(rows,cols));result.shape=tuple(shape);return result

    def __array_ufunc__(self,ufunc,method,*inputs,**kwargs):
        if ufunc is not np.bitwise_or or method!='__call__' or len(inputs)!=2:
            return NotImplemented
        arrays=[v for v in inputs if isinstance(v,np.ndarray)]
        if len(arrays)!=1 or arrays[0].shape!=self.shape or arrays[0].dtype!=np.dtype(bool):
            return NotImplemented
        array=arrays[0];out=kwargs.get('out')
        if set(kwargs)-{'out'} or (out is not None and (len(out)!=1 or out[0] is not array)):
            return NotImplemented
        result=array if out is not None else array.copy()
        result[tuple(self)]=True
        return result

def feature_index(semantic_map,feature,diagnostics=None):
    """Use the original rounded/clipped cells and the same OpenCV fill rule."""
    points=raster._world_polygon_cells(semantic_map,feature)
    shape=(semantic_map.height,semantic_map.width);cells=math.prod(shape)
    if not len(points):
        index=PixelIndex(np.empty(0,np.intp),np.empty(0,np.intp),shape);area=0
    else:
        col0,row0=points.min(axis=0);col1,row1=points.max(axis=0)+1
        area=int((row1-row0)*(col1-col0))
        if area>min(cells//16,262144):
            if diagnostics is not None:diagnostics['dense_features']+=1
            return binary_view(semantic_map,feature)
        local=np.zeros((row1-row0,col1-col0),np.uint8)
        xy=np.asarray(points-(col0,row0),np.int32)
        if feature.geometry_type=='polygon' and len(points)>=3:
            cv2.fillPoly(local,[xy],1,lineType=cv2.LINE_8)
        elif feature.geometry_type=='line' and len(points)>=2:
            cv2.polylines(local,[xy],False,1,thickness=1,lineType=cv2.LINE_8)
        elif feature.geometry_type=='point':local[xy[0,1],xy[0,0]]=1
        rows,cols=np.nonzero(local);rows+=row0;cols+=col0
        index=PixelIndex(rows,cols,shape)
    if diagnostics is not None:
        diagnostics['indexed_features']+=1
        diagnostics['local_raster_cells']+=area
        diagnostics['full_mask_cells_avoided']+=cells
        diagnostics['max_index_bytes']=max(diagnostics['max_index_bytes'],sum(v.nbytes for v in index))
    return index

def threshold_safe(free,resolution,cutoff,diagnostics=None):
    """Exactly evaluate (EDT(free) * resolution >= cutoff), guarded at ties."""
    free=np.asarray(free);resolution=float(resolution);cutoff=float(cutoff)
    reason=None
    if free.ndim!=2 or free.dtype!=np.dtype(bool) or not min(free.shape):reason='unsupported_input'
    elif not (math.isfinite(resolution) and resolution>0 and math.isfinite(cutoff) and cutoff>0):reason='unsupported_threshold'
    else:
        radius=int(math.ceil(cutoff/resolution))
        if radius<1 or radius>128:reason='kernel_limit'
        elif free.all():reason='all_free_original_virtual_site'
        else:
            seed=np.ones((2*radius+1,2*radius+1),bool);seed[radius,radius]=False
            distance=_EDT(seed)*resolution
            margin=float(np.min(abs(distance-cutoff)))
            if margin<=max(1e-10,128*np.finfo(float).eps*max(1.,cutoff)):reason='lattice_threshold_ambiguity'
    if reason:
        if diagnostics is not None:diagnostics.update(direct=False,fallback=reason)
        return _EDT(free)*resolution>=cutoff
    kernel=(distance<cutoff).astype(np.uint8);safe=np.empty(free.shape,bool)
    for begin in range(0,len(free),128):
        end=min(len(free),begin+128);lo=max(0,begin-radius);hi=min(len(free),end+radius)
        unsafe=np.logical_not(free[lo:hi]).view(np.uint8)
        expanded=cv2.dilate(unsafe,kernel,borderType=cv2.BORDER_CONSTANT,borderValue=0)
        safe[begin:end]=expanded[begin-lo:end-lo]==0
    if diagnostics is not None:diagnostics.update(direct=True,kernel_shape=list(kernel.shape),kernel_cells=int(kernel.sum()),lattice_margin_m=margin,scratch_rows=128+2*radius,global_float_EDT_built=False)
    return safe

class _ScaledDistance:
    def __init__(self,free,resolution,diagnostics):self.free=free;self.resolution=resolution;self.diagnostics=diagnostics
    def __ge__(self,cutoff):return threshold_safe(self.free,self.resolution,cutoff,self.diagnostics)

class _ThresholdDistance:
    def __init__(self,free,diagnostics):self.free=free;self.diagnostics=diagnostics
    def __mul__(self,resolution):return _ScaledDistance(self.free,resolution,self.diagnostics)

class _WorldNdimage:
    def __init__(self,diagnostics):self.diagnostics=diagnostics
    def __getattr__(self,name):return getattr(ndimage,name)
    def distance_transform_edt(self,free,*args,**kwargs):
        if args or kwargs:return _EDT(free,*args,**kwargs)
        return _ThresholdDistance(free,self.diagnostics)

def _source_hash(function):
    try:return hashlib.sha256(inspect.getsource(function).encode()).hexdigest()
    except (OSError,TypeError):return None

@contextmanager
def firstready_construction():
    from arena_3d_v1 import semantic_world
    from . import topology
    diagnostics={'enabled':False,'indexed_features':0,'dense_features':0,'local_raster_cells':0,'full_mask_cells_avoided':0,'max_index_bytes':0,'safe_predicate':{},'frozen_L1_source_changed':False}
    with _LOCK:
        known=(_source_hash(semantic_world.SemanticWorld.__init__)==WORLD_INIT_SHA256
               and _source_hash(topology.V5Topology.__init__)==TOPOLOGY_INIT_SHA256
               and _source_hash(ORIGINAL)==PINNED_SHA256
               and semantic_world.rasterize_feature is ORIGINAL
               and topology.rasterize_feature is ORIGINAL
               and semantic_world.ndimage is ndimage)
        if os.environ.get('THREEYD_V10_FIRSTREADY','1')!='1' or not known:
            with construction_storage() as parent:
                diagnostics.update(reason='disabled_or_unrecognized_constructor',parent=parent)
                yield diagnostics
            return
        diagnostics['enabled']=True
        def indexed(s,f):return feature_index(s,f,diagnostics)
        semantic_world.rasterize_feature=topology.rasterize_feature=indexed
        semantic_world.ndimage=_WorldNdimage(diagnostics['safe_predicate'])
        try:yield diagnostics
        finally:
            semantic_world.rasterize_feature=topology.rasterize_feature=ORIGINAL
            semantic_world.ndimage=ndimage
