"""Exact current semantic-label membership; unknown inputs use np.isin.

Every supported call constructs a fresh table using the original np.isin
over the complete represented byte domain, reads current labels and IDs,
and allocates an independent bool output. No source or decision is cached.
This helper belongs only to mask preparation, never to the route audit.
"""
import cv2
import numpy as np
from .v9_r9_options import enabled

_COUNTS = {'native_membership_calls': 0, 'original_fallback_calls': 0,
           'disabled_calls': 0, 'lookup_cells': 0}
_DTYPES = {}


def _original(labels, elements):
    _COUNTS['original_fallback_calls'] += 1
    return np.isin(labels, elements)


def semantic_membership(labels, elements):
    if not enabled('M'):
        _COUNTS['disabled_calls'] += 1
        return np.isin(labels, elements)
    if (type(labels) is not np.ndarray or labels.ndim != 2
            or not min(labels.shape, default=0) or not labels.flags.c_contiguous
            or labels.dtype.kind not in 'iu' or not labels.dtype.isnative
            or type(elements) not in (list, tuple, np.ndarray)):
        return _original(labels, elements)
    try:
        values = np.asarray(elements)
    except (TypeError, ValueError, OverflowError):
        return _original(labels, elements)
    if values.size and values.dtype.kind not in 'iub':
        return _original(labels, elements)
    lower = -128 if labels.dtype == np.dtype(np.int8) else 0
    upper = 127 if labels.dtype == np.dtype(np.int8) else 255
    if values.size and (int(values.min()) < lower or int(values.max()) > upper):
        return _original(labels, elements)
    if labels.dtype.itemsize == 1:
        codes = labels.view(np.uint8)
        domain = np.arange(256, dtype=np.uint8).view(labels.dtype)
    else:
        if int(labels.min()) < 0 or int(labels.max()) > 255:
            return _original(labels, elements)
        codes = labels.astype(np.uint8)
        domain = np.arange(256, dtype=labels.dtype)
    try:
        table = np.isin(domain, elements).astype(np.uint8)
    except (TypeError, ValueError, OverflowError, IndexError):
        return _original(labels, elements)
    try:
        result = cv2.LUT(codes, table)
    except cv2.error:
        return _original(labels, elements)
    if (type(result) is not np.ndarray or result.dtype != np.dtype(np.uint8)
            or result.shape != labels.shape):
        return _original(labels, elements)
    _COUNTS['native_membership_calls'] += 1
    _COUNTS['lookup_cells'] += labels.size
    dtype = str(labels.dtype)
    _DTYPES[dtype] = _DTYPES.get(dtype, 0) + 1
    return result.view(np.bool_)


def membership_stats():
    return {**_COUNTS, 'source_dtypes': dict(_DTYPES), 'mode_enabled': enabled('M'),
            'input_content_cached': False, 'table_cached': False,
            'audit_or_ACK_results_cached': False, 'OpenCV_threads': cv2.getNumThreads()}
