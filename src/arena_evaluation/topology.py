from __future__ import annotations
import math
from typing import Sequence
import cv2
import numpy as np

def _footprint_kernel(resolution: float, footprint: Sequence[Sequence[float]], padding_m: float, safety_margin_m: float, angle_count: int=24) -> np.ndarray:
    """Rasterize the complete rectangular footprint over all headings."""
    points = np.asarray(footprint, dtype=np.float64)
    radius = float(np.max(np.hypot(points[:, 0], points[:, 1]))) + padding_m + safety_margin_m
    half_cells = int(math.ceil(radius / resolution)) + 2
    size = 2 * half_cells + 1
    kernel = np.zeros((size, size), dtype=np.uint8)
    center = half_cells
    margin_cells = int(math.ceil((padding_m + safety_margin_m) / resolution))
    for angle in np.linspace(0.0, math.pi, max(4, angle_count), endpoint=False):
        (cos_a, sin_a) = (math.cos(float(angle)), math.sin(float(angle)))
        transformed = []
        for (px, py) in points:
            world_x = cos_a * px - sin_a * py
            world_y = sin_a * px + cos_a * py
            col = int(round(center + world_x / resolution))
            row = int(round(center - world_y / resolution))
            transformed.append([col, row])
        cv2.fillPoly(kernel, [np.asarray(transformed, dtype=np.int32)], 1)
    if margin_cells:
        diameter = 2 * margin_cells + 1
        disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (diameter, diameter))
        kernel = cv2.dilate(kernel, disk)
    kernel[center, center] = 1
    return kernel
