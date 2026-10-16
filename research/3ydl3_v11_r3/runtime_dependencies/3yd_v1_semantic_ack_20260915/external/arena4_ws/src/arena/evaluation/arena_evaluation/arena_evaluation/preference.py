"""Static L2 lateral preference costs for Stage 8B."""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .topology import AStarResult, Cell, TopologyArtifact, TopologyRoute, path_length_cells

_NEIGHBORS = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))


@dataclass
class PreferenceGeometry:
    mode: str
    penalty: np.ndarray
    active: np.ndarray
    lateral_deviation_m: np.ndarray
    right_wall_distance_m: np.ndarray
    region: np.ndarray
    failure_code: str = ""


def _cell_distance(a: Cell, b: Cell) -> float:
    return math.hypot(float(a[0] - b[0]), float(a[1] - b[1]))


def preference_astar(
    free_mask: np.ndarray,
    start: Cell,
    goal: Cell,
    allowed_mask: Optional[np.ndarray],
    penalty: Optional[np.ndarray],
    weight: float,
    resolution: float,
) -> AStarResult:
    """The Stage 6 A* transition rule plus a normalized nonnegative cost."""
    started = time.monotonic_ns()
    effective = free_mask if allowed_mask is None else (free_mask & allowed_mask)
    total_free = int(np.count_nonzero(free_mask)); allowed = int(np.count_nonzero(effective))
    expanded = generated = max_open = 0; path = None; failure = ""
    shape = free_mask.shape
    if not (0 <= start[0] < shape[0] and 0 <= start[1] < shape[1] and 0 <= goal[0] < shape[0] and 0 <= goal[1] < shape[1]):
        failure = "INVALID_ENDPOINT"
    elif not (free_mask[start] and free_mask[goal]):
        failure = "INVALID_ENDPOINT"
    elif allowed_mask is not None and not (allowed_mask[start] and allowed_mask[goal]):
        failure = "ENDPOINT_OUTSIDE_ALLOWED"
    else:
        queue = [(0.0, 0.0, start)]; came: Dict[Cell, Cell] = {}; costs = {start: 0.0}; generated = max_open = 1
        while queue:
            _, cost, current = heapq.heappop(queue)
            if cost != costs.get(current):
                continue
            expanded += 1
            if current == goal:
                path = [current]
                while path[-1] in came:
                    path.append(came[path[-1]])
                path.reverse(); break
            row, col = current
            for dr, dc in _NEIGHBORS:
                candidate = row + dr, col + dc
                if not (0 <= candidate[0] < shape[0] and 0 <= candidate[1] < shape[1]) or not effective[candidate]:
                    continue
                base = _cell_distance(current, candidate)
                cell_penalty = float(penalty[candidate]) if penalty is not None else 0.0
                new_cost = cost + base * (1.0 + float(weight) * max(0.0, min(1.0, cell_penalty)))
                if new_cost >= costs.get(candidate, float("inf")):
                    continue
                if candidate not in costs:
                    generated += 1
                costs[candidate] = new_cost; came[candidate] = current
                heapq.heappush(queue, (new_cost + _cell_distance(candidate, goal), new_cost, candidate)); max_open = max(max_open, len(queue))
        if path is None:
            failure = "NO_PATH"
    return AStarResult(path, expanded, generated, max_open, allowed, total_free, float(allowed / total_free) if total_free else 0.0, path_length_cells(path, resolution) if path else None, (time.monotonic_ns() - started) / 1e6, failure)


def build_preference_geometry(
    artifact: TopologyArtifact,
    route: TopologyRoute,
    mode: str,
    *,
    right_wall_target_m: float = 0.40,
    narrow_width_m: float = 1.23,
    clearance_cap_m: float = 1.50,
) -> PreferenceGeometry:
    if mode not in {"none", "center", "right_edge"}:
        raise ValueError(f"unknown preference mode: {mode}")
    shape = artifact.free_mask.shape
    zero = np.zeros(shape, dtype=np.float32)
    if mode == "none":
        return PreferenceGeometry(mode, zero, artifact.free_mask.copy(), zero.copy(), zero.copy(), np.zeros(shape, dtype=np.uint8))
    route_xy = np.asarray(route.polyline, dtype=np.float64)
    if len(route_xy) < 2:
        return PreferenceGeometry(mode, zero, np.zeros(shape, dtype=bool), zero.copy(), zero.copy(), np.zeros(shape, dtype=np.uint8), "PREFERENCE_GEOMETRY_UNAVAILABLE")
    from scipy.spatial import cKDTree
    tangents = np.empty_like(route_xy)
    tangents[0] = route_xy[1] - route_xy[0]; tangents[-1] = route_xy[-1] - route_xy[-2]
    tangents[1:-1] = route_xy[2:] - route_xy[:-2]
    norms = np.hypot(tangents[:, 0], tangents[:, 1]); tangents /= np.maximum(norms[:, None], 1.0e-9)
    cells = np.argwhere(artifact.free_mask)
    x = artifact.hospital_map.origin[0] + (cells[:, 1] + 0.5) * artifact.hospital_map.resolution
    y = artifact.hospital_map.origin[1] + (artifact.hospital_map.height - cells[:, 0] - 0.5) * artifact.hospital_map.resolution
    world = np.column_stack([x, y]); _, nearest = cKDTree(route_xy).query(world, workers=-1)
    offset = world - route_xy[nearest]
    signed = tangents[nearest, 0] * offset[:, 1] - tangents[nearest, 1] * offset[:, 0]
    center_deviation = np.abs(signed)
    route_cells = [artifact.hospital_map.world_to_cell(px, py) for px, py in route_xy]
    half_width_route = np.asarray([artifact.hospital_map.distance_m[cell] if cell is not None else 0.0 for cell in route_cells], dtype=np.float64)
    half_width = half_width_route[nearest]
    widths = 2.0 * half_width
    active_values = widths >= narrow_width_m
    region_values = np.where(widths < narrow_width_m, 0, np.where(widths < 3.0, 1, 2)).astype(np.uint8)
    raw_clearance = artifact.hospital_map.distance_m[cells[:, 0], cells[:, 1]].astype(np.float64)
    center_penalty = 0.5 * np.clip(center_deviation / np.maximum(half_width, 0.10), 0.0, 1.0) + 0.5 * (1.0 - np.clip(raw_clearance / clearance_cap_m, 0.0, 1.0))
    right_wall = half_width + signed
    right_penalty = np.clip(np.abs(right_wall - right_wall_target_m) / max(right_wall_target_m, 1.0e-6), 0.0, 1.0)
    values = center_penalty if mode == "center" else right_penalty
    values = np.where(active_values, values, 0.0)
    penalty = np.zeros(shape, dtype=np.float32); active = np.zeros(shape, dtype=bool); lateral = np.zeros(shape, dtype=np.float32); wall = np.zeros(shape, dtype=np.float32); region = np.zeros(shape, dtype=np.uint8)
    indices = cells[:, 0], cells[:, 1]
    penalty[indices] = values.astype(np.float32); active[indices] = active_values; lateral[indices] = signed.astype(np.float32); wall[indices] = right_wall.astype(np.float32); region[indices] = region_values
    return PreferenceGeometry(mode, penalty, active, lateral, wall, region)


def path_preference_metrics(path: Sequence[Cell], geometry: PreferenceGeometry, artifact: TopologyArtifact, target_m: float = 0.40) -> Dict[str, object]:
    if not path:
        return {}
    cells = tuple(zip(*path)); active = geometry.active[cells]; lateral = geometry.lateral_deviation_m[cells]; wall = geometry.right_wall_distance_m[cells]; regions = geometry.region[cells]
    enabled = int(np.count_nonzero(active)); total = len(path)
    result = {
        "preference_active_ratio": enabled / max(1, total),
        "preference_inactive_narrow_count": int(np.count_nonzero(regions == 0)),
        "region_corridor_count": int(np.count_nonzero(regions == 1)),
        "region_wide_count": int(np.count_nonzero(regions == 2)),
    }
    if geometry.mode == "center" and enabled:
        values = np.abs(lateral[active]); result.update({"center_deviation_p50_m": float(np.quantile(values, 0.50)), "center_deviation_p95_m": float(np.quantile(values, 0.95))})
    if geometry.mode == "right_edge" and enabled:
        error = np.abs(wall[active] - target_m); result.update({"right_wall_error_p50_m": float(np.quantile(error, 0.50)), "right_wall_error_p95_m": float(np.quantile(error, 0.95)), "correct_side_ratio": float(np.mean(lateral[active] <= 0.0))})
    clearances = [artifact.hospital_map.clearance(*artifact.hospital_map.cell_to_world(cell)) for cell in path]
    result["clearance_p50_m"] = float(np.quantile(clearances, 0.50)); result["clearance_p95_m"] = float(np.quantile(clearances, 0.95))
    return result

