from __future__ import annotations

import hashlib
import math
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml
from PIL import Image

from .models import Query, QueryValidation


Cell = Tuple[int, int]  # row, column in image coordinates


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


@dataclass
class HospitalMap:
    yaml_path: Path
    image_path: Path
    resolution: float
    origin: Tuple[float, float, float]
    width: int
    height: int
    occupancy: np.ndarray
    distance_m: np.ndarray
    _component_labels: dict = field(default_factory=dict, init=False, repr=False)

    @classmethod
    def load(cls, yaml_path: str | Path) -> "HospitalMap":
        yaml_path = Path(yaml_path).resolve()
        config = yaml.safe_load(yaml_path.read_text())
        image_path = Path(config["image"])
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        image = np.asarray(Image.open(image_path).convert("L"))
        if image.ndim != 2:
            raise ValueError(f"map image must be grayscale: {image_path}")
        resolution = float(config["resolution"])
        origin = tuple(float(value) for value in config["origin"])
        occupied_threshold = float(config.get("occupied_thresh", 0.65))
        free_threshold = float(config.get("free_thresh", 0.196))
        negate = bool(config.get("negate", 0))
        probability = image.astype(np.float32) / 255.0
        # map_server interprets a white pixel as free unless negate=1.
        probability = probability if negate else 1.0 - probability
        # ROS map_server uses 100 for occupied, 0 for free and -1 unknown.
        occupancy = np.full(image.shape, -1, dtype=np.int8)
        occupancy[probability > occupied_threshold] = 100
        occupancy[probability < free_threshold] = 0
        # Values exactly on free_thresh are intentionally kept unknown.
        distance_m = _distance_to_occupied(occupancy == 100, resolution)
        return cls(
            yaml_path=yaml_path,
            image_path=image_path.resolve(),
            resolution=resolution,
            origin=(origin[0], origin[1], origin[2]),
            width=int(image.shape[1]),
            height=int(image.shape[0]),
            occupancy=occupancy,
            distance_m=distance_m,
        )

    @property
    def map_id(self) -> str:
        return self.yaml_path.parent.parent.name if self.yaml_path.parent.name == "map" else self.yaml_path.parent.name

    @property
    def sha256(self) -> str:
        return sha256_file(self.image_path)

    def world_to_cell(self, x: float, y: float) -> Optional[Cell]:
        col = math.floor((float(x) - self.origin[0]) / self.resolution)
        row_from_bottom = math.floor((float(y) - self.origin[1]) / self.resolution)
        row = self.height - 1 - row_from_bottom
        if not (0 <= row < self.height and 0 <= col < self.width):
            return None
        return row, col

    def cell_to_world(self, cell: Cell) -> Tuple[float, float]:
        row, col = cell
        x = self.origin[0] + (col + 0.5) * self.resolution
        y = self.origin[1] + (self.height - row - 0.5) * self.resolution
        return x, y

    def clearance(self, x: float, y: float) -> Optional[float]:
        cell = self.world_to_cell(x, y)
        if cell is None:
            return None
        return float(self.distance_m[cell])

    def footprint_cells(self, pose: Sequence[float]) -> List[Cell]:
        x, y, yaw = (float(value) for value in pose[:3])
        # The caller supplies footprint points in robot coordinates.
        return []

    def footprint_collision(
        self,
        pose: Sequence[float],
        footprint: Sequence[Sequence[float]],
        *,
        unknown_is_collision: bool = False,
    ) -> bool:
        polygon = _transform_polygon(pose, footprint)
        if not polygon:
            return True
        min_x = min(point[0] for point in polygon) - self.resolution
        max_x = max(point[0] for point in polygon) + self.resolution
        min_y = min(point[1] for point in polygon) - self.resolution
        max_y = max(point[1] for point in polygon) + self.resolution
        min_cell = self.world_to_cell(min_x, min_y)
        max_cell = self.world_to_cell(max_x, max_y)
        if min_cell is None or max_cell is None:
            return True
        rows = range(min(min_cell[0], max_cell[0]), max(min_cell[0], max_cell[0]) + 1)
        cols = range(min(min_cell[1], max_cell[1]), max(min_cell[1], max_cell[1]) + 1)
        half_diagonal = math.sqrt(2.0) * self.resolution / 2.0
        for row in rows:
            for col in cols:
                if not (0 <= row < self.height and 0 <= col < self.width):
                    return True
                cell_value = int(self.occupancy[row, col])
                if cell_value == 100 or (unknown_is_collision and cell_value < 0):
                    center = self.cell_to_world((row, col))
                    if _point_in_polygon(center, polygon) or _distance_to_polygon(center, polygon) <= half_diagonal:
                        return True
        return False

    def validate_query(
        self,
        query: Query,
        footprint: Sequence[Sequence[float]],
        minimum_clearance_m: float,
        allow_unknown: bool,
    ) -> QueryValidation:
        start_status, start_reason = self._validate_pose(
            query.start, footprint, minimum_clearance_m, allow_unknown
        )
        goal_status, goal_reason = self._validate_pose(
            query.goal, footprint, minimum_clearance_m, allow_unknown
        )
        connected = False
        if start_status == "VALID" and goal_status == "VALID":
            start_cell = self.world_to_cell(query.start[0], query.start[1])
            goal_cell = self.world_to_cell(query.goal[0], query.goal[1])
            connected = self.connected(start_cell, goal_cell, allow_unknown=allow_unknown)
        status = "VALID" if start_status == "VALID" and goal_status == "VALID" and connected else "INVALID"
        reason_parts = [part for part in (start_reason, goal_reason) if part]
        if status == "INVALID" and not connected and start_status == goal_status == "VALID":
            reason_parts.append("start and goal are not connected under the selected unknown-space policy")
        return QueryValidation(
            query_id=query.query_id,
            config_variant="",
            validation_status=status,
            start_status=start_status,
            goal_status=goal_status,
            connected=connected,
            start_clearance_m=self.clearance(query.start[0], query.start[1]),
            goal_clearance_m=self.clearance(query.goal[0], query.goal[1]),
            reason="; ".join(reason_parts),
            suggested_start=(self.suggest_near(query.start, footprint, minimum_clearance_m, allow_unknown) if start_status != "VALID" else None),
            suggested_goal=(self.suggest_near(query.goal, footprint, minimum_clearance_m, allow_unknown) if goal_status != "VALID" else None),
        )

    def _validate_pose(
        self,
        pose: Sequence[float],
        footprint: Sequence[Sequence[float]],
        minimum_clearance_m: float,
        allow_unknown: bool,
    ) -> Tuple[str, str]:
        cell = self.world_to_cell(pose[0], pose[1])
        if cell is None:
            return "OUT_OF_BOUNDS", "pose is outside map bounds"
        if self.occupancy[cell] == 100:
            return "OCCUPIED", "pose cell is occupied"
        if self.occupancy[cell] < 0 and not allow_unknown:
            return "UNKNOWN", "pose cell is unknown while allow_unknown=false"
        if self.footprint_collision(pose, footprint, unknown_is_collision=not allow_unknown):
            return "FOOTPRINT_COLLISION", "rotated footprint overlaps occupied/unknown cells"
        clearance = self.clearance(pose[0], pose[1])
        if clearance is None or clearance < minimum_clearance_m:
            return "INSUFFICIENT_CLEARANCE", f"endpoint center clearance {clearance!r} m is below {minimum_clearance_m} m"
        return "VALID", ""

    def connected(self, start: Optional[Cell], goal: Optional[Cell], *, allow_unknown: bool) -> bool:
        if start is None or goal is None:
            return False
        labels = self._labels_for(allow_unknown)
        return labels[start] > 0 and labels[start] == labels[goal]

    def _labels_for(self, allow_unknown: bool) -> np.ndarray:
        key = bool(allow_unknown)
        if key in self._component_labels:
            return self._component_labels[key]
        traversable = self.occupancy == 0
        if allow_unknown:
            traversable |= self.occupancy < 0
        try:
            import cv2  # type: ignore

            _, labels = cv2.connectedComponents(traversable.astype(np.uint8), connectivity=8)
        except Exception:
            labels = np.zeros(traversable.shape, dtype=np.int32)
            component = 0
            for row, col in zip(*np.where(traversable)):
                if labels[row, col] != 0:
                    continue
                component += 1
                queue = deque([(int(row), int(col))])
                labels[row, col] = component
                while queue:
                    current_row, current_col = queue.popleft()
                    for next_row, next_col in ((current_row - 1, current_col), (current_row + 1, current_col), (current_row, current_col - 1), (current_row, current_col + 1), (current_row - 1, current_col - 1), (current_row - 1, current_col + 1), (current_row + 1, current_col - 1), (current_row + 1, current_col + 1)):
                        if 0 <= next_row < self.height and 0 <= next_col < self.width and traversable[next_row, next_col] and labels[next_row, next_col] == 0:
                            labels[next_row, next_col] = component
                            queue.append((next_row, next_col))
        self._component_labels[key] = labels
        return labels

    def suggest_near(
        self,
        pose: Sequence[float],
        footprint: Sequence[Sequence[float]],
        minimum_clearance_m: float,
        allow_unknown: bool,
    ) -> Optional[List[float]]:
        cell = self.world_to_cell(pose[0], pose[1])
        if cell is None:
            return None
        max_radius = max(self.width, self.height)
        for radius in range(1, max_radius):
            for row in range(cell[0] - radius, cell[0] + radius + 1):
                for col in (cell[1] - radius, cell[1] + radius):
                    candidate = (row, col)
                    if self._candidate_valid(candidate, pose[2], footprint, minimum_clearance_m, allow_unknown):
                        x, y = self.cell_to_world(candidate)
                        return [round(x, 3), round(y, 3), round(float(pose[2]), 3)]
            for col in range(cell[1] - radius + 1, cell[1] + radius):
                for row in (cell[0] - radius, cell[0] + radius):
                    candidate = (row, col)
                    if self._candidate_valid(candidate, pose[2], footprint, minimum_clearance_m, allow_unknown):
                        x, y = self.cell_to_world(candidate)
                        return [round(x, 3), round(y, 3), round(float(pose[2]), 3)]
        return None

    def _candidate_valid(self, cell: Cell, yaw: float, footprint: Sequence[Sequence[float]], clearance: float, allow_unknown: bool) -> bool:
        row, col = cell
        if not (0 <= row < self.height and 0 <= col < self.width):
            return False
        x, y = self.cell_to_world(cell)
        return (
            self.occupancy[cell] == 0
            and (allow_unknown or self.occupancy[cell] >= 0)
            and float(self.distance_m[cell]) >= clearance + _footprint_radius(footprint)
            and not self.footprint_collision((x, y, yaw), footprint, unknown_is_collision=not allow_unknown)
        )


def _distance_to_occupied(occupied: np.ndarray, resolution: float) -> np.ndarray:
    try:
        import cv2  # type: ignore

        source = (~occupied).astype(np.uint8)
        return cv2.distanceTransform(source, cv2.DIST_L2, 5) * float(resolution)
    except Exception:
        # A dependency-free fallback for test images. It is deliberately conservative.
        result = np.full(occupied.shape, np.inf, dtype=np.float32)
        queue = deque()
        for row, col in zip(*np.where(occupied)):
            result[row, col] = 0.0
            queue.append((row, col))
        while queue:
            row, col = queue.popleft()
            for next_row, next_col in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
                if 0 <= next_row < result.shape[0] and 0 <= next_col < result.shape[1]:
                    candidate = result[row, col] + resolution
                    if candidate < result[next_row, next_col]:
                        result[next_row, next_col] = candidate
                        queue.append((next_row, next_col))
        return result


def _transform_polygon(pose: Sequence[float], footprint: Sequence[Sequence[float]]) -> List[Tuple[float, float]]:
    x, y, yaw = (float(value) for value in pose[:3])
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return [
        (x + cos_yaw * float(px) - sin_yaw * float(py), y + sin_yaw * float(px) + cos_yaw * float(py))
        for px, py in footprint
    ]


def _footprint_radius(footprint: Sequence[Sequence[float]]) -> float:
    return max((math.hypot(float(x), float(y)) for x, y in footprint), default=0.0)




def _point_in_polygon(point: Tuple[float, float], polygon: Sequence[Tuple[float, float]]) -> bool:
    x, y = point
    inside = False
    for index, (x1, y1) in enumerate(polygon):
        x2, y2 = polygon[(index + 1) % len(polygon)]
        intersects = (y1 > y) != (y2 > y)
        if intersects and x < (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-12) + x1:
            inside = not inside
    return inside


def _distance_to_polygon(point: Tuple[float, float], polygon: Sequence[Tuple[float, float]]) -> float:
    return min(
        _distance_to_segment(point, polygon[index], polygon[(index + 1) % len(polygon)])
        for index in range(len(polygon))
    )


def _distance_to_segment(point: Tuple[float, float], first: Tuple[float, float], second: Tuple[float, float]) -> float:
    px, py = point
    x1, y1 = first
    x2, y2 = second
    dx, dy = x2 - x1, y2 - y1
    denominator = dx * dx + dy * dy
    if denominator <= 1e-12:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / denominator))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))
