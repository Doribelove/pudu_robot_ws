from __future__ import annotations
import hashlib
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Sequence
import cv2
import numpy as np
from .semantic_map import SemanticFeature, SemanticMapV1, canonical_hash

def grid_hash(value: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()

def _world_polygon_cells(semantic_map: SemanticMapV1, feature: SemanticFeature) -> np.ndarray:
    result = []
    for (x, y) in feature.coordinates:
        col = math.floor((float(x) - semantic_map.origin[0]) / semantic_map.resolution)
        row_from_bottom = math.floor((float(y) - semantic_map.origin[1]) / semantic_map.resolution)
        row = semantic_map.height - 1 - row_from_bottom
        result.append([int(np.clip(col, 0, semantic_map.width - 1)), int(np.clip(row, 0, semantic_map.height - 1))])
    return np.asarray(result, dtype=np.int32)

def rasterize_feature(semantic_map: SemanticMapV1, feature: SemanticFeature) -> np.ndarray:
    mask = np.zeros((semantic_map.height, semantic_map.width), dtype=np.uint8)
    points = _world_polygon_cells(semantic_map, feature)
    if feature.geometry_type == 'polygon' and len(points) >= 3:
        cv2.fillPoly(mask, [points], 1, lineType=cv2.LINE_8)
    elif feature.geometry_type == 'line' and len(points) >= 2:
        cv2.polylines(mask, [points], False, 1, thickness=1, lineType=cv2.LINE_8)
    elif feature.geometry_type == 'point' and len(points):
        mask[points[0, 1], points[0, 0]] = 1
    return mask.astype(bool)

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
        hashes = {'class_grid': grid_hash(self.class_grid), 'priority_grid': grid_hash(self.priority_grid), 'hard_mask': grid_hash(self.hard_mask), 'hard_footprint_mask': grid_hash(self.hard_footprint_mask), 'no_stopping_mask': grid_hash(self.no_stopping_mask), 'masks': {key: grid_hash(value) for (key, value) in sorted(self.masks.items())}, 'semantic_map_hash': self.semantic_map_hash, 'policy_hash': self.policy_hash}
        return canonical_hash(hashes)

    def save(self, path: str | Path, *, overwrite: bool=False) -> Path:
        target = Path(path)
        if target.exists() and (not overwrite):
            raise FileExistsError(f'refusing to overwrite semantic raster: {target}')
        target.parent.mkdir(parents=True, exist_ok=True)
        arrays = {'class_grid': self.class_grid, 'priority_grid': self.priority_grid, 'hard_mask': self.hard_mask.astype(np.uint8), 'hard_footprint_mask': self.hard_footprint_mask.astype(np.uint8), 'no_stopping_mask': self.no_stopping_mask.astype(np.uint8)}
        arrays.update({f'mask__{key}': value.astype(np.uint8) for (key, value) in self.masks.items()})
        arrays['metadata_json'] = np.asarray(json.dumps({**self.metadata, 'semantic_map_hash': self.semantic_map_hash, 'policy_hash': self.policy_hash, 'raster_hash': self.raster_hash, 'resolution': self.resolution, 'origin': list(self.origin), 'width': self.width, 'height': self.height}, sort_keys=True))
        np.savez_compressed(target, **arrays)
        return target
