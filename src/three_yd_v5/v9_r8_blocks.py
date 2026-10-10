"""Exact per-tile logical OR on contiguous boolean grids, without padding.

The original reduceat computes the OR of every cell in each tile. Reshaping
complete tiles into views permits the same Cartesian reduction with contiguous
row runs; partial last rows/columns are reduced separately. Unknown layouts
return None so the caller retains the original reduction. No distance domain,
tile order, neighborhood halo or inflation computation changes.
"""
import numpy as np


def tile_support(mask, tile):
    if (type(mask) is not np.ndarray or mask.ndim != 2 or mask.dtype != np.dtype(bool)
            or not mask.flags.c_contiguous or not isinstance(tile, (int, np.integer))
            or tile <= 0 or 0 in mask.shape):
        return None
    h, w = mask.shape; fy, fx = h//tile, w//tile
    ny, nx = (h+tile-1)//tile, (w+tile-1)//tile
    small = np.zeros((ny, nx), bool)
    if fy and fx:
        view = mask[:fy*tile, :fx*tile].reshape(fy, tile, fx, tile)
        small[:fy, :fx] = view.any(axis=(1, 3))
    if h % tile and fx:
        view = mask[fy*tile:, :fx*tile].reshape(h-fy*tile, fx, tile)
        small[fy, :fx] = view.any(axis=(0, 2))
    if w % tile and fy:
        view = mask[:fy*tile, fx*tile:].reshape(fy, tile, w-fx*tile)
        small[:fy, fx] = view.any(axis=(1, 2))
    if h % tile and w % tile:
        small[fy, fx] = mask[fy*tile:, fx*tile:].any()
    return small
