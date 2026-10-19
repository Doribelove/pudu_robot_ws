"""Exact L2 ROI construction without a coordinate for every selected cell.

The frozen L1 implementation remains untouched. Unusual input types retain
the original helper; ordinary boolean arrays use identical bounds/bindings.
"""
import numpy as np
from arena_3d_v1.multires_grid import make_roi as original_make_roi
from arena_3d_v1.l2_incremental import CorridorROI, CorridorBinding
from arena_3d_v1.semantic_world import digest
from arena_evaluation.semantic_map import canonical_hash
from .r11_fields import bbox


def make_roi(view, mask, start, goal, route_signature):
    if not (type(mask) is np.ndarray and mask.ndim == 2 and mask.dtype == np.bool_
            and type(view.safe) is np.ndarray and view.safe.shape == mask.shape
            and view.safe.dtype == np.bool_):
        return original_make_roi(view, mask, start, goal, route_signature)
    for rc in (start, goal):
        if not (0 <= rc[0] < view.map.height and 0 <= rc[1] < view.map.width
                and view.safe[rc] and mask[rc]):
            raise ValueError('COARSE_ENDPOINT_UNREPRESENTABLE' if view.factor > 1 else 'FINE_ENDPOINT_UNSAFE')
    bounds = bbox(mask, halo=1)
    if bounds is None:
        return original_make_roi(view, mask, start, goal, route_signature)
    r0, r1, c0, c1 = bounds
    binding = CorridorBinding(view.key, mask.shape, view.map.origin, view.map.resolution,
        route_signature, (route_signature,), digest(mask), start, goal, canonical_hash(view.vehicle))
    return CorridorROI(bounds, np.ascontiguousarray(mask[r0:r1,c0:c1] & view.safe[r0:r1,c0:c1]),
        (start[0]-r0,start[1]-c0), (goal[0]-r0,goal[1]-c0), binding, int(mask.sum()))
