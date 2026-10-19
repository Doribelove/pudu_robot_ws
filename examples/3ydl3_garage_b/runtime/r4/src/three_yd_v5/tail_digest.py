"""Reuse the source hash only for an exact, proven immutable publication view."""
import numpy as np
from .tail_options import enabled
from .v9_maps import immutable,flow_digest,flipped_digest


def publication_source_digest(source,expected):
    if enabled('D') and type(source) is np.ndarray and type(expected) is np.ndarray:
        flipped=np.flipud(source)
        if (immutable(source) and immutable(expected) and expected.shape==flipped.shape
            and expected.dtype==flipped.dtype and expected.strides==flipped.strides
            and expected.ctypes.data==flipped.ctypes.data):
            return flipped_digest(source)
    return flow_digest(expected)
