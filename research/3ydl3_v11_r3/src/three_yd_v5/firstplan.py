"""Full-route first-query preparation; no search or acceptance policy changes."""
import os
import cv2
import numpy as np


def enabled(part):
    mode=os.environ.get('THREEYD_V8_FIRST_MODE','control')
    return mode != 'control' and part in mode


def sparse_dilate(source,kernel):
    """Exact zero-background dilation over its finite nonzero support.

    Odd centered kernels only. Padding by the complete kernel radius ensures
    that every output influenced by a source pixel is computed. Outside is 0,
    matching the original uint8 dilation's default border.
    """
    a=np.asarray(source,np.uint8);k=np.asarray(kernel,np.uint8)
    if a.ndim!=2 or k.ndim!=2 or any(v%2!=1 for v in k.shape):
        raise ValueError('SPARSE_DILATION_REQUIRES_ODD_2D_KERNEL')
    result=np.zeros_like(a)
    rows=np.flatnonzero(a.any(axis=1));cols=np.flatnonzero(a.any(axis=0))
    if not len(rows):return result
    ry,rx=k.shape[0]//2,k.shape[1]//2
    y0,y1=max(0,int(rows[0])-ry),min(len(a),int(rows[-1])+ry+1)
    x0,x1=max(0,int(cols[0])-rx),min(a.shape[1],int(cols[-1])+rx+1)
    result[y0:y1,x0:x1]=cv2.dilate(a[y0:y1,x0:x1],k,borderType=cv2.BORDER_CONSTANT,borderValue=0)
    return result


def state_coordinates(geometry):
    linear=np.asarray(geometry.state_cells_linear)
    return np.column_stack((linear//geometry.roi_shape[1]+geometry.bbox[0],
                            linear%geometry.roi_shape[1]+geometry.bbox[2])).astype(np.int32)
