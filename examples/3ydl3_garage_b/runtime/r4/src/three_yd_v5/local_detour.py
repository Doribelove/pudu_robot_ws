"""Bounded map-only search-hint extension, never a physical clearance test."""
import math
import numpy as np


def extend_hint(extension, world, labels, center, radius):
    if radius not in (3., 5., 8.):
        raise ValueError('R6_LOCAL_DETOUR_RADIUS')
    center = np.asarray(center, float)
    if center.shape != (2,) or not np.isfinite(center).all():
        raise ValueError('R6_LOCAL_DETOUR_CENTER')
    rr0, cc0 = world.cells(center[None])
    rad = int(math.ceil(radius / world.map.resolution)) + 1
    ar, br = max(0, int(rr0[0])-rad), min(world.hard.shape[0], int(rr0[0])+rad+1)
    ac, bc = max(0, int(cc0[0])-rad), min(world.hard.shape[1], int(cc0[0])+rad+1)
    rr, cc = np.mgrid[ar:br, ac:bc]
    xy = world.world_points(rr.ravel(), cc.ravel())
    disk = (np.linalg.norm(xy-center, axis=1) <= radius).reshape(rr.shape)
    disk &= labels[ar:br, ac:bc] == 1
    extension[ar:br, ac:bc] |= disk
