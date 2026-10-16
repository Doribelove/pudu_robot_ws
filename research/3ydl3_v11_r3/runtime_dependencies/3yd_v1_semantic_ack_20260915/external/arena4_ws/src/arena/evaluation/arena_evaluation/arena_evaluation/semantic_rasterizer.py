"""Rasterize ``SemanticMapV1`` without weakening the static obstacle map."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

import cv2
import numpy as np

from .semantic_map import SemanticFeature, SemanticMapV1, canonical_hash


CLASS_CODES = {
    "unlabelled": 0,
    "fence_area": 1,
    "speed_bumps": 2,
    "lane": 3,
    "parking_area": 4,
    "junction_area": 5,
    "no_stopping": 6,
    "forbidden": 7,
    "no_go": 7,
}


def grid_hash(value: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def _world_polygon_cells(semantic_map: SemanticMapV1, feature: SemanticFeature) -> np.ndarray:
    result = []
    for x, y in feature.coordinates:
        col = math.floor((float(x) - semantic_map.origin[0]) / semantic_map.resolution)
        row_from_bottom = math.floor((float(y) - semantic_map.origin[1]) / semantic_map.resolution)
        row = semantic_map.height - 1 - row_from_bottom
        result.append([
            int(np.clip(col, 0, semantic_map.width - 1)),
            int(np.clip(row, 0, semantic_map.height - 1)),
        ])
    return np.asarray(result, dtype=np.int32)


def rasterize_feature(semantic_map: SemanticMapV1, feature: SemanticFeature) -> np.ndarray:
    mask = np.zeros((semantic_map.height, semantic_map.width), dtype=np.uint8)
    points = _world_polygon_cells(semantic_map, feature)
    if feature.geometry_type == "polygon" and len(points) >= 3:
        cv2.fillPoly(mask, [points], 1, lineType=cv2.LINE_8)
    elif feature.geometry_type == "line" and len(points) >= 2:
        cv2.polylines(mask, [points], False, 1, thickness=1, lineType=cv2.LINE_8)
    elif feature.geometry_type == "point" and len(points):
        mask[points[0, 1], points[0, 0]] = 1
    return mask.astype(bool)


def footprint_kernel(
    resolution: float, footprint: Sequence[Sequence[float]], safety_margin_m: float,
    *, angle_count: int = 36,
) -> np.ndarray:
    """Rasterize the complete footprint over headings, plus safety margin."""
    points = np.asarray(footprint, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 3 or points.shape[1] != 2:
        raise ValueError("footprint must contain at least three [x,y] vertices")
    radius = float(np.max(np.hypot(points[:, 0], points[:, 1]))) + float(safety_margin_m)
    half = int(math.ceil(radius / float(resolution))) + 2
    kernel = np.zeros((2 * half + 1, 2 * half + 1), dtype=np.uint8)
    for yaw in np.linspace(0.0, math.pi, max(4, int(angle_count)), endpoint=False):
        cosine, sine = math.cos(float(yaw)), math.sin(float(yaw))
        cells = []
        for x, y in points:
            world_x = cosine * x - sine * y
            world_y = sine * x + cosine * y
            cells.append([
                int(round(half + world_x / resolution)),
                int(round(half - world_y / resolution)),
            ])
        cv2.fillPoly(kernel, [np.asarray(cells, dtype=np.int32)], 1)
    margin_cells = int(math.ceil(float(safety_margin_m) / float(resolution)))
    if margin_cells:
        disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * margin_cells + 1,) * 2)
        kernel = cv2.dilate(kernel, disk)
    kernel[half, half] = 1
    return kernel


@dataclass
class RasterizedSemantics:
    semantic_map_hash: str
    policy_hash: str
    resolution: float
    origin: Sequence[float]
    width: int
    height: int
    masks: Dict[str, np.ndarray]
    class_grid: np.ndarray
    priority_grid: np.ndarray
    hard_mask: np.ndarray
    hard_footprint_mask: np.ndarray
    no_stopping_mask: np.ndarray
    metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def raster_hash(self) -> str:
        hashes = {
            "class_grid": grid_hash(self.class_grid),
            "priority_grid": grid_hash(self.priority_grid),
            "hard_mask": grid_hash(self.hard_mask),
            "hard_footprint_mask": grid_hash(self.hard_footprint_mask),
            "no_stopping_mask": grid_hash(self.no_stopping_mask),
            "masks": {key: grid_hash(value) for key, value in sorted(self.masks.items())},
            "semantic_map_hash": self.semantic_map_hash,
            "policy_hash": self.policy_hash,
        }
        return canonical_hash(hashes)

    def save(self, path: str | Path, *, overwrite: bool = False) -> Path:
        target = Path(path)
        if target.exists() and not overwrite:
            raise FileExistsError(f"refusing to overwrite semantic raster: {target}")
        target.parent.mkdir(parents=True, exist_ok=True)
        arrays = {
            "class_grid": self.class_grid,
            "priority_grid": self.priority_grid,
            "hard_mask": self.hard_mask.astype(np.uint8),
            "hard_footprint_mask": self.hard_footprint_mask.astype(np.uint8),
            "no_stopping_mask": self.no_stopping_mask.astype(np.uint8),
        }
        arrays.update({f"mask__{key}": value.astype(np.uint8) for key, value in self.masks.items()})
        arrays["metadata_json"] = np.asarray(json.dumps({
            **self.metadata,
            "semantic_map_hash": self.semantic_map_hash,
            "policy_hash": self.policy_hash,
            "raster_hash": self.raster_hash,
            "resolution": self.resolution,
            "origin": list(self.origin),
            "width": self.width,
            "height": self.height,
        }, sort_keys=True))
        np.savez_compressed(target, **arrays)
        return target


class SemanticRasterizer:
    def __init__(
        self, *, footprint: Sequence[Sequence[float]], safety_margin_m: float = 0.05,
    ) -> None:
        self.footprint = [[float(x), float(y)] for x, y in footprint]
        self.safety_margin_m = float(safety_margin_m)

    def rasterize(
        self, semantic_map: SemanticMapV1, *, hospital_map: Optional[Any] = None,
    ) -> RasterizedSemantics:
        if hospital_map is not None:
            semantic_map.validate_against_map(hospital_map)
        shape = (semantic_map.height, semantic_map.width)
        masks = {
            semantic_class: np.zeros(shape, dtype=bool)
            for semantic_class in sorted(set(CLASS_CODES) - {"unlabelled"})
        }
        class_grid = np.zeros(shape, dtype=np.uint8)
        priority_grid = np.zeros(shape, dtype=np.int16)
        hard_mask = np.zeros(shape, dtype=bool)
        no_stopping = np.zeros(shape, dtype=bool)
        # Independent class masks preserve additive speed-bump cost even when
        # a higher-priority lane or junction overlaps the same cell.
        for feature in sorted(semantic_map.features, key=lambda item: (item.priority, item.semantic_id)):
            feature_mask = rasterize_feature(semantic_map, feature)
            masks.setdefault(feature.semantic_class, np.zeros(shape, dtype=bool))
            masks[feature.semantic_class] |= feature_mask
            replace = feature_mask & (int(feature.priority) >= priority_grid)
            class_grid[replace] = int(CLASS_CODES[feature.semantic_class])
            priority_grid[replace] = int(feature.priority)
            if feature.hard:
                hard_mask |= feature_mask
            if feature.non_stopping:
                no_stopping |= feature_mask
        kernel = footprint_kernel(
            semantic_map.resolution, self.footprint, self.safety_margin_m,
        )
        hard_footprint = cv2.dilate(hard_mask.astype(np.uint8), kernel, iterations=1).astype(bool)
        policy = {
            "footprint": self.footprint,
            "semantic_safety_margin_m": self.safety_margin_m,
            "overlap_priority": [
                "forbidden/no_go", "junction_area", "parking_area", "lane", "unlabelled",
            ],
            "fence_area_hard": False,
        }
        return RasterizedSemantics(
            semantic_map_hash=semantic_map.semantic_map_hash,
            policy_hash=canonical_hash(policy),
            resolution=semantic_map.resolution,
            origin=semantic_map.origin,
            width=semantic_map.width,
            height=semantic_map.height,
            masks=masks,
            class_grid=class_grid,
            priority_grid=priority_grid,
            hard_mask=hard_mask,
            hard_footprint_mask=hard_footprint,
            no_stopping_mask=no_stopping,
            metadata={
                "policy": policy,
                "class_cell_counts": {
                    key: int(np.count_nonzero(value)) for key, value in sorted(masks.items())
                },
                "hard_cell_count": int(np.count_nonzero(hard_mask)),
                "hard_footprint_cell_count": int(np.count_nonzero(hard_footprint)),
                "pgm_y_axis_inverted": True,
            },
        )


__all__ = [
    "CLASS_CODES", "RasterizedSemantics", "SemanticRasterizer",
    "rasterize_feature", "footprint_kernel", "grid_hash",
]
