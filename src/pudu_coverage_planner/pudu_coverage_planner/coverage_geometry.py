"""Grid-native Boustrophedon decomposition and sweep path generation."""

from dataclasses import dataclass
import heapq
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


GridPoint = Tuple[float, float]
Segment = Tuple[int, int, int]


@dataclass
class GridCoveragePlan:
    """Coverage paths expressed in the input mask's pixel coordinates."""

    sweep_rotation_deg: float
    cell_paths: List[List[GridPoint]]
    cell_segments: List[List[Tuple[GridPoint, GridPoint]]]


def elliptical_kernel(radius_cells: int) -> np.ndarray:
    radius = max(0, int(radius_cells))
    if radius == 0:
        return np.ones((1, 1), dtype=np.uint8)
    size = radius * 2 + 1
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))


def polygon_to_mask(
    shape: Tuple[int, int], points: Sequence[GridPoint]
) -> np.ndarray:
    mask = np.zeros(shape, dtype=np.uint8)
    if len(points) < 3:
        return mask.astype(bool)
    contour = np.rint(np.asarray(points, dtype=np.float64)).astype(np.int32)
    cv2.fillPoly(mask, [contour], 1)
    return mask.astype(bool)


def erode_mask(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    if radius_cells <= 0:
        return mask.astype(bool, copy=True)
    eroded = cv2.erode(
        mask.astype(np.uint8), elliptical_kernel(radius_cells), iterations=1
    )
    return eroded.astype(bool)


def dilate_mask(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    if radius_cells <= 0:
        return mask.astype(bool, copy=True)
    dilated = cv2.dilate(
        mask.astype(np.uint8), elliptical_kernel(radius_cells), iterations=1
    )
    return dilated.astype(bool)


def _row_intervals(row: np.ndarray) -> List[Tuple[int, int]]:
    indices = np.flatnonzero(row)
    if indices.size == 0:
        return []
    breaks = np.flatnonzero(np.diff(indices) > 1)
    starts = np.r_[indices[0], indices[breaks + 1]]
    ends = np.r_[indices[breaks], indices[-1]]
    return [(int(start), int(end)) for start, end in zip(starts, ends)]


def _rotation_transform(
    shape: Tuple[int, int], angle_deg: float
) -> Tuple[np.ndarray, np.ndarray, int]:
    height, width = shape
    side = int(math.ceil(math.hypot(width, height))) + 6
    center = ((side - 1) * 0.5, (side - 1) * 0.5)
    translate = np.array(
        [
            [1.0, 0.0, center[0] - (width - 1) * 0.5],
            [0.0, 1.0, center[1] - (height - 1) * 0.5],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    rotation = np.eye(3, dtype=np.float64)
    rotation[:2, :] = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
    forward = rotation @ translate
    inverse = np.linalg.inv(forward)
    return forward[:2, :], inverse[:2, :], side


def _rotate_mask(
    mask: np.ndarray, angle_deg: float
) -> Tuple[np.ndarray, np.ndarray]:
    forward, inverse, side = _rotation_transform(mask.shape, angle_deg)
    rotated = cv2.warpAffine(
        mask.astype(np.uint8),
        forward,
        (side, side),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    return rotated.astype(bool), inverse


def _sweep_cost(mask: np.ndarray, lane_spacing: int) -> Tuple[int, int]:
    rows = np.flatnonzero(np.any(mask, axis=1))
    if rows.size == 0:
        return (10**9, 10**9)
    interval_count = 0
    span = 0
    first = int(rows[0])
    last = int(rows[-1])
    for y in range(first, last + 1, max(1, lane_spacing)):
        intervals = _row_intervals(mask[y])
        interval_count += len(intervals)
        span += sum(end - start + 1 for start, end in intervals)
    # Turns dominate execution time; longer lane span is preferred as a tie-break.
    return (interval_count, -span)


def choose_sweep_rotation(mask: np.ndarray, lane_spacing: int) -> float:
    best_angle = 0.0
    best_cost = (10**9, 10**9)
    for angle in range(0, 180, 15):
        rotated, _ = _rotate_mask(mask, float(angle))
        cost = _sweep_cost(rotated, lane_spacing)
        if cost < best_cost:
            best_cost = cost
            best_angle = float(angle)
    return best_angle


def boustrophedon_cells(mask: np.ndarray) -> Dict[int, List[Segment]]:
    """Split a free-space mask whenever sweep-line connectivity changes."""
    cells: Dict[int, List[Segment]] = {}
    previous: List[Tuple[int, int, int]] = []
    next_id = 0

    for y in range(mask.shape[0]):
        current_intervals = _row_intervals(mask[y])
        overlaps: List[List[int]] = [[] for _ in current_intervals]
        previous_overlaps: List[List[int]] = [[] for _ in previous]

        for current_index, (x0, x1) in enumerate(current_intervals):
            for previous_index, (px0, px1, _) in enumerate(previous):
                if min(x1, px1) >= max(x0, px0):
                    overlaps[current_index].append(previous_index)
                    previous_overlaps[previous_index].append(current_index)

        current: List[Tuple[int, int, int]] = []
        for current_index, (x0, x1) in enumerate(current_intervals):
            parents = overlaps[current_index]
            if len(parents) == 1 and len(previous_overlaps[parents[0]]) == 1:
                cell_id = previous[parents[0]][2]
            else:
                cell_id = next_id
                next_id += 1
            cells.setdefault(cell_id, []).append((y, x0, x1))
            current.append((x0, x1, cell_id))
        previous = current

    return cells


def _line_is_free(mask: np.ndarray, start: GridPoint, goal: GridPoint) -> bool:
    distance = max(abs(goal[0] - start[0]), abs(goal[1] - start[1]))
    samples = max(2, int(math.ceil(distance)) + 1)
    for ratio in np.linspace(0.0, 1.0, samples):
        x = int(round(start[0] + ratio * (goal[0] - start[0])))
        y = int(round(start[1] + ratio * (goal[1] - start[1])))
        if y < 0 or y >= mask.shape[0] or x < 0 or x >= mask.shape[1]:
            return False
        if not mask[y, x]:
            return False
    return True


def _astar(
    mask: np.ndarray, start: GridPoint, goal: GridPoint
) -> Optional[List[GridPoint]]:
    source = (int(round(start[0])), int(round(start[1])))
    target = (int(round(goal[0])), int(round(goal[1])))
    if not (0 <= source[0] < mask.shape[1] and 0 <= source[1] < mask.shape[0]):
        return None
    if not (0 <= target[0] < mask.shape[1] and 0 <= target[1] < mask.shape[0]):
        return None
    if source == target:
        return [start]
    if not mask[source[1], source[0]] or not mask[target[1], target[0]]:
        return None

    neighbors = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    ]
    queue: List[Tuple[float, float, Tuple[int, int]]] = []
    heapq.heappush(queue, (0.0, 0.0, source))
    cost = {source: 0.0}
    parent: Dict[Tuple[int, int], Tuple[int, int]] = {}

    while queue:
        _, current_cost, current = heapq.heappop(queue)
        if current == target:
            path: List[GridPoint] = [target]
            while path[-1] != source:
                path.append(parent[path[-1]])
            path.reverse()
            return [(float(x), float(y)) for x, y in path]
        if current_cost > cost.get(current, float("inf")):
            continue

        for dx, dy, step_cost in neighbors:
            nx = current[0] + dx
            ny = current[1] + dy
            if nx < 0 or nx >= mask.shape[1] or ny < 0 or ny >= mask.shape[0]:
                continue
            if not mask[ny, nx]:
                continue
            if dx != 0 and dy != 0:
                if not mask[current[1], nx] or not mask[ny, current[0]]:
                    continue
            candidate = current_cost + step_cost
            point = (nx, ny)
            if candidate >= cost.get(point, float("inf")):
                continue
            cost[point] = candidate
            parent[point] = current
            heuristic = math.hypot(target[0] - nx, target[1] - ny)
            heapq.heappush(queue, (candidate + heuristic, candidate, point))
    return None


def _append_densified(
    output: List[GridPoint], points: Iterable[GridPoint], spacing: float
) -> None:
    for point in points:
        if not output:
            output.append(point)
            continue
        start = output[-1]
        distance = math.hypot(point[0] - start[0], point[1] - start[1])
        steps = max(1, int(math.ceil(distance / max(0.5, spacing))))
        for index in range(1, steps + 1):
            ratio = index / steps
            candidate = (
                start[0] + ratio * (point[0] - start[0]),
                start[1] + ratio * (point[1] - start[1]),
            )
            if math.hypot(candidate[0] - output[-1][0], candidate[1] - output[-1][1]) > 1e-6:
                output.append(candidate)


def _cell_lanes(
    mask: np.ndarray,
    segments: Sequence[Segment],
    lane_spacing: int,
    point_spacing: float,
) -> Tuple[List[GridPoint], List[Tuple[GridPoint, GridPoint]]]:
    by_row = {y: (x0, x1) for y, x0, x1 in segments}
    rows = sorted(by_row)
    if not rows:
        return [], []

    selected: List[int] = []
    target = rows[0]
    while target <= rows[-1]:
        row = min(rows, key=lambda value: abs(value - target))
        if row not in selected:
            selected.append(row)
        target += max(1, lane_spacing)
    if rows[-1] - selected[-1] >= max(1, lane_spacing // 2):
        selected.append(rows[-1])
    selected = sorted(set(selected))

    output: List[GridPoint] = []
    lane_segments: List[Tuple[GridPoint, GridPoint]] = []
    reverse = False
    for row in selected:
        x0, x1 = by_row[row]
        if x1 - x0 < 1:
            continue
        start = (float(x1), float(row)) if reverse else (float(x0), float(row))
        goal = (float(x0), float(row)) if reverse else (float(x1), float(row))
        lane_segments.append((start, goal))
        if output:
            if _line_is_free(mask, output[-1], start):
                connector = [start]
            else:
                connector = _astar(mask, output[-1], start)
            if connector is None:
                continue
            _append_densified(output, connector, point_spacing)
        _append_densified(output, [start, goal], point_spacing)
        reverse = not reverse
    return output, lane_segments


def _apply_affine(matrix: np.ndarray, point: GridPoint) -> GridPoint:
    vector = matrix @ np.array([point[0], point[1], 1.0], dtype=np.float64)
    return (float(vector[0]), float(vector[1]))


def _project_to_mask(mask: np.ndarray, point: GridPoint) -> Optional[GridPoint]:
    """Snap inverse-warp interpolation artifacts back into valid free cells."""
    center_x = int(round(point[0]))
    center_y = int(round(point[1]))
    candidates: List[Tuple[float, int, int]] = []
    for radius in range(0, 4):
        candidates.clear()
        for y in range(center_y - radius, center_y + radius + 1):
            for x in range(center_x - radius, center_x + radius + 1):
                if not (0 <= y < mask.shape[0] and 0 <= x < mask.shape[1]):
                    continue
                if not mask[y, x]:
                    continue
                candidates.append((math.hypot(point[0] - x, point[1] - y), x, y))
        if candidates:
            _, x, y = min(candidates)
            return (float(x), float(y))
    return None


def _restore_path(
    mask: np.ndarray,
    inverse: np.ndarray,
    path: Sequence[GridPoint],
    point_spacing: float,
) -> List[GridPoint]:
    """Transform a rotated path and guarantee every connector remains free."""
    output: List[GridPoint] = []
    for rotated_point in path:
        projected = _project_to_mask(mask, _apply_affine(inverse, rotated_point))
        if projected is None:
            continue
        if not output:
            output.append(projected)
            continue
        if _line_is_free(mask, output[-1], projected):
            connector = [projected]
        else:
            connector = _astar(mask, output[-1], projected)
        if connector is not None:
            _append_densified(output, connector, point_spacing)
    return output


def _order_paths(
    paths: List[List[GridPoint]], start: Optional[GridPoint]
) -> List[List[GridPoint]]:
    if not paths:
        return []
    remaining = list(paths)
    ordered: List[List[GridPoint]] = []
    current = start if start is not None else remaining[0][0]
    while remaining:
        best_index = 0
        best_reverse = False
        best_distance = float("inf")
        for index, path in enumerate(remaining):
            for reverse, endpoint in ((False, path[0]), (True, path[-1])):
                distance = math.hypot(endpoint[0] - current[0], endpoint[1] - current[1])
                if distance < best_distance:
                    best_index = index
                    best_reverse = reverse
                    best_distance = distance
        path = remaining.pop(best_index)
        if best_reverse:
            path = list(reversed(path))
        ordered.append(path)
        current = path[-1]
    return ordered


def grid_path_length(points: Sequence[GridPoint]) -> float:
    return sum(
        math.hypot(goal[0] - start[0], goal[1] - start[1])
        for start, goal in zip(points, points[1:]))


def split_path_at_turns(
    points: Sequence[GridPoint],
    turn_angle_rad: float,
    min_length_cells: float,
) -> List[List[GridPoint]]:
    """Split tight connectors while retaining continuous executable lanes."""
    if len(points) < 3:
        return [list(points)] if len(points) >= 2 else []
    pieces: List[List[GridPoint]] = []
    current = [points[0]]
    previous_heading: Optional[float] = None
    for index in range(1, len(points)):
        start = points[index - 1]
        goal = points[index]
        heading = math.atan2(goal[1] - start[1], goal[0] - start[0])
        if previous_heading is not None:
            delta = abs(math.atan2(
                math.sin(heading - previous_heading),
                math.cos(heading - previous_heading)))
            if delta >= max(0.0, turn_angle_rad) and len(current) >= 2:
                pieces.append(current)
                current = [start]
        current.append(goal)
        previous_heading = heading
    if len(current) >= 2:
        pieces.append(current)

    usable = [
        piece for piece in pieces
        if grid_path_length(piece) >= max(0.0, min_length_cells)]
    # Tiny isolated cells still need an executable path.
    return usable if usable else [list(points)]


def plan_coverage(
    navigable_mask: np.ndarray,
    lane_spacing_cells: int,
    point_spacing_cells: float,
    min_cell_area_cells: int = 1,
    start: Optional[GridPoint] = None,
) -> GridCoveragePlan:
    """Plan obstacle-aware, ordered sweep paths over a navigable-center mask."""
    mask = navigable_mask.astype(bool)
    if not np.any(mask):
        return GridCoveragePlan(0.0, [], [])

    lane_spacing = max(1, int(lane_spacing_cells))
    angle = choose_sweep_rotation(mask, lane_spacing)
    rotated, inverse = _rotate_mask(mask, angle)
    cells = boustrophedon_cells(rotated)

    paths: List[List[GridPoint]] = []
    visual_segments: List[List[Tuple[GridPoint, GridPoint]]] = []
    for segments in cells.values():
        area = sum(x1 - x0 + 1 for _, x0, x1 in segments)
        if area < max(1, min_cell_area_cells):
            continue
        path, lane_segments = _cell_lanes(
            rotated,
            segments,
            lane_spacing,
            point_spacing_cells,
        )
        if len(path) < 2:
            continue
        restored = _restore_path(mask, inverse, path, point_spacing_cells)
        if len(restored) < 2:
            continue
        paths.append(restored)
        visual_segments.append([
            (_apply_affine(inverse, segment[0]), _apply_affine(inverse, segment[1]))
            for segment in lane_segments
        ])

    ordered = _order_paths(paths, start)
    return GridCoveragePlan(angle, ordered, visual_segments)
