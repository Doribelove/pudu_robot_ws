"""Batch array bookkeeping around the original per-point scalar norm."""
import numpy as np


def tangents(points, arc, before, after, offset):
    ids = np.arange(len(points))
    before = np.maximum(0, np.minimum(before, ids))
    after = np.minimum(len(points)-1, np.maximum(after, ids))
    delta = points[after] - points[before]
    # Keep the original np.linalg.norm on each 1-D two-element vector.
    # Axis reductions can round differently from its dot/sqrt implementation.
    norms = np.fromiter((float(np.linalg.norm(d)) for d in delta),
                        dtype=float, count=len(delta))
    for index in np.flatnonzero(norms <= 1e-9):
        delta[index] = points[min(index+1, len(points)-1)] - points[max(index-1, 0)]
        norms[index] = float(np.linalg.norm(delta[index]))
    if np.any(norms <= 1e-9):
        raise ValueError('V5_R8_OFFSET_TANGENT_UNDEFINED')
    direction = delta / norms[:, None]
    left = np.column_stack((-direction[:, 1], direction[:, 0]))
    target = points + float(offset)*left
    return direction, target
