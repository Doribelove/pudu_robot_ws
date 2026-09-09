"""Continuous parking-centre references for the 2A-V3 route-phase planner.

The r13 route-phase lattice sampled parking candidates on cross sections of
the frozen L1 polyline.  That is safe, but it is not a continuous reference:
when the L1 line runs along a parking boundary the independently best cell on
successive sections can jump between disconnected clearance ridges.  This
module constructs one deterministic, request-bound centre-biased path through
each parking phase and splices it into the oriented L1 route.  The resulting
polyline is still only a guide.  The 48-bin forward-only Dubins search and the
complete final audits remain authoritative.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import heapq
import math
from typing import Any, Iterable, Sequence

import cv2
import numpy as np

from .semantic_map import canonical_hash
from .semantic_route_phase_v3 import OrientedRoute, Phase, RoutePhaseWorld, _phase_near


METHOD_ID = "parking_component_continuous_centre_reference_v1"


@dataclass(frozen=True)
class ParkingReferencePolicy:
    station_spacing_m: float = 0.25
    search_resolution_m: float = 0.10
    route_tube_half_width_m: float = 8.0
    centre_cost_weight: float = 18.0
    outside_target_cost_weight: float = 6.0
    simplify_tolerance_m: float = 0.10
    smoothing_window: int = 9
    smoothing_passes: int = 3
    maximum_expanded_cells: int = 350_000
    minimum_parking_run_m: float = 0.50
    endpoint_snap_limit_m: float = 2.0
    target_deviation_max: float = 0.25
    maximum_reference_curvature_1pm: float = 2.50

    def __post_init__(self) -> None:
        if self.station_spacing_m <= 0 or self.search_resolution_m <= 0:
            raise ValueError("parking reference spacing must be positive")
        if self.route_tube_half_width_m <= 0 or self.centre_cost_weight <= 0:
            raise ValueError("parking reference weights and tube must be positive")
        if self.maximum_expanded_cells < 1 or self.smoothing_passes < 0:
            raise ValueError("parking reference resource bounds are invalid")
        if self.smoothing_window < 3 or self.smoothing_window % 2 == 0:
            raise ValueError("parking smoothing window must be odd and >=3")
        if not 0.0 < self.target_deviation_max < 1.0:
            raise ValueError("parking target deviation must be in (0,1)")
        if self.maximum_reference_curvature_1pm > 2.50:
            raise ValueError("parking reference cannot relax maximum curvature")


@dataclass(frozen=True)
class ParkingReferenceResult:
    gate_passed: bool
    failure_code: str
    polyline: tuple[tuple[float, float], ...]
    original_route_hash: str
    reference_route_hash: str
    original_length_m: float
    reference_length_m: float
    parking_run_count: int
    target_sample_ratio: float
    maximum_reference_curvature_1pm: float
    footprint_sample_count: int
    expanded_cells: int
    diagnostics: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["polyline"] = [list(point) for point in self.polyline]
        return value


def _deduplicate(points: Iterable[Sequence[float]]) -> list[tuple[float, float]]:
    result: list[tuple[float, float]] = []
    for point in points:
        value = float(point[0]), float(point[1])
        if not result or math.dist(result[-1], value) > 1.0e-9:
            result.append(value)
    return result


def _polyline_length(points: Sequence[Sequence[float]]) -> float:
    return float(sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:])))


def _resample(points: Sequence[Sequence[float]], spacing: float) -> np.ndarray:
    values = np.asarray(_deduplicate(points), dtype=np.float64)
    if len(values) < 2:
        return values
    lengths = np.hypot(*(np.diff(values, axis=0).T))
    station = np.r_[0.0, np.cumsum(lengths)]
    sample = np.arange(0.0, station[-1], max(float(spacing), 1.0e-3))
    if not len(sample) or station[-1] - sample[-1] > 1.0e-9:
        sample = np.r_[sample, station[-1]]
    return np.column_stack((
        np.interp(sample, station, values[:, 0]),
        np.interp(sample, station, values[:, 1]),
    ))


def _maximum_curvature(points: Sequence[Sequence[float]], spacing: float = 0.10) -> float:
    values = _resample(points, spacing)
    if len(values) < 3:
        return 0.0
    first, middle, last = values[:-2], values[1:-1], values[2:]
    a = np.hypot(*(middle-first).T)
    b = np.hypot(*(last-middle).T)
    c = np.hypot(*(last-first).T)
    cross = np.abs(
        (middle[:, 0]-first[:, 0])*(last[:, 1]-first[:, 1])
        - (middle[:, 1]-first[:, 1])*(last[:, 0]-first[:, 0])
    )
    denominator = a*b*c
    curvature = np.divide(
        2.0*cross, denominator,
        out=np.full_like(cross, np.inf), where=denominator > 1.0e-9,
    )
    return float(np.max(curvature)) if len(curvature) else 0.0


def _nearest(mask: np.ndarray, cell: tuple[int, int], limit_cells: int) -> tuple[int, int] | None:
    row, col = map(int, cell)
    r0, r1 = max(0, row-limit_cells), min(mask.shape[0], row+limit_cells+1)
    c0, c1 = max(0, col-limit_cells), min(mask.shape[1], col+limit_cells+1)
    values = np.argwhere(mask[r0:r1, c0:c1])
    if not len(values):
        return None
    values[:, 0] += r0
    values[:, 1] += c0
    distance = (values[:, 0]-row)**2 + (values[:, 1]-col)**2
    order = np.lexsort((values[:, 1], values[:, 0], distance))
    target = values[int(order[0])]
    return int(target[0]), int(target[1])


def _astar(
    traversable: np.ndarray, cost: np.ndarray, start: tuple[int, int], goal: tuple[int, int],
    maximum_expanded: int,
) -> tuple[list[tuple[int, int]] | None, int]:
    """Deterministic 8-connected weighted A* with corner-cut prevention."""
    height, width = traversable.shape
    start_id, goal_id = start[0]*width+start[1], goal[0]*width+goal[1]
    g = {start_id: 0.0}
    parent: dict[int, int] = {}
    heap = [(math.hypot(goal[0]-start[0], goal[1]-start[1]), 0.0, start_id)]
    closed: set[int] = set()
    moves = ((-1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), (1, 0, 1.0),
             (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
             (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)))
    while heap and len(closed) < int(maximum_expanded):
        _f, current_g, node = heapq.heappop(heap)
        if node in closed or current_g > g.get(node, math.inf)+1.0e-12:
            continue
        closed.add(node)
        if node == goal_id:
            path = []
            while True:
                path.append(divmod(node, width))
                if node == start_id:
                    break
                node = parent[node]
            path.reverse()
            return path, len(closed)
        row, col = divmod(node, width)
        for dr, dc, length in moves:
            rr, cc = row+dr, col+dc
            if not (0 <= rr < height and 0 <= cc < width and traversable[rr, cc]):
                continue
            if dr and dc and not (traversable[row, cc] and traversable[rr, col]):
                continue
            target = rr*width+cc
            step = length*(1.0+0.5*(float(cost[row, col])+float(cost[rr, cc])))
            candidate = current_g+step
            if candidate+1.0e-12 >= g.get(target, math.inf):
                continue
            g[target], parent[target] = candidate, node
            heuristic = math.hypot(goal[0]-rr, goal[1]-cc)
            heapq.heappush(heap, (candidate+heuristic, candidate, target))
    return None, len(closed)


def _route_tube(world: RoutePhaseWorld, points: Sequence[Sequence[float]], radius_m: float) -> np.ndarray:
    mask = np.zeros(world.master.shape, dtype=np.uint8)
    cells = [world.map.world_to_cell(float(point[0]), float(point[1])) for point in points]
    pixels = np.asarray([[cell[1], cell[0]] for cell in cells if cell is not None], dtype=np.int32)
    if len(pixels) >= 2:
        cv2.polylines(mask, [pixels], False, 1, thickness=1, lineType=cv2.LINE_8)
    elif len(pixels) == 1:
        mask[pixels[0, 1], pixels[0, 0]] = 1
    radius = max(1, int(math.ceil(float(radius_m)/world.map.resolution)))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*radius+1, 2*radius+1))
    return cv2.dilate(mask, kernel).astype(bool)


def _smooth_cells(
    cells: Sequence[tuple[int, int]], traversable: np.ndarray, passes: int, window: int,
) -> np.ndarray:
    values = np.asarray([[float(col), float(row)] for row, col in cells], dtype=np.float64)
    if len(values) < window:
        return values
    kernel = np.ones(window, dtype=np.float64)/float(window)
    result = values.copy()
    for _ in range(int(passes)):
        candidate = result.copy()
        half = window//2
        for axis in range(2):
            filtered = np.convolve(result[:, axis], kernel, mode="same")
            candidate[half:-half, axis] = filtered[half:-half]
        rounded = np.rint(candidate).astype(int)
        inside = (
            (rounded[:, 1] >= 0) & (rounded[:, 1] < traversable.shape[0])
            & (rounded[:, 0] >= 0) & (rounded[:, 0] < traversable.shape[1])
        )
        valid = np.zeros(len(result), dtype=bool)
        valid[inside] = traversable[rounded[inside, 1], rounded[inside, 0]]
        result[valid] = candidate[valid]
        result[0], result[-1] = values[0], values[-1]
    return result


def _cell_xy(world: RoutePhaseWorld, values: np.ndarray) -> np.ndarray:
    cols, rows = values[:, 0], values[:, 1]
    return np.column_stack((
        world.map.full_origin[0]+(cols+world.map.col0+.5)*world.map.resolution,
        world.map.full_origin[1]+(world.map.full_height-rows-world.map.row0-.5)*world.map.resolution,
    ))


class ContinuousParkingReferenceBuilder:
    def __init__(self, policy: ParkingReferencePolicy | None = None) -> None:
        self.policy = policy or ParkingReferencePolicy()

    def build(self, world: RoutePhaseWorld, route: OrientedRoute) -> ParkingReferenceResult:
        samples = route.stations(self.policy.station_spacing_m)
        phases = [_phase_near(world, item.x, item.y, .50) for item in samples]
        runs: list[tuple[int, int, Phase]] = []
        begin = 0
        for index in range(1, len(phases)+1):
            if index == len(phases) or phases[index] != phases[begin]:
                if phases[begin].kind == "parking":
                    runs.append((begin, index-1, phases[begin]))
                begin = index
        original = [[float(point[0]), float(point[1])] for point in route.points]
        if not runs:
            return self._result(True, "", original, route, 0, 0.0, 0, [], world)

        # Work on the regular route samples so parking run boundaries have an
        # exact, deterministic position in the stitched reference.
        stitched: list[tuple[float, float]] = []
        cursor = 0
        expanded_total = 0
        run_diagnostics = []
        target_samples = parking_samples = 0
        for start_index, end_index, phase in runs:
            if samples[end_index].station_m-samples[start_index].station_m < self.policy.minimum_parking_run_m:
                continue
            stitched.extend((samples[i].x, samples[i].y) for i in range(cursor, start_index))
            component = world.grids["parking_components"] == int(phase.instance)
            safe = (
                component & world.grids["allowed"].astype(bool)
                & ~world.grids["hard"].astype(bool) & (world.master < 253)
            )
            tube_points = [(samples[i].x, samples[i].y) for i in range(start_index, end_index+1)]
            safe &= _route_tube(world, tube_points, self.policy.route_tube_half_width_m)
            start_cell = world.map.world_to_cell(samples[start_index].x, samples[start_index].y)
            goal_cell = world.map.world_to_cell(samples[end_index].x, samples[end_index].y)
            snap = int(math.ceil(self.policy.endpoint_snap_limit_m/world.map.resolution))
            start_cell = None if start_cell is None else _nearest(safe, start_cell, snap)
            goal_cell = None if goal_cell is None else _nearest(safe, goal_cell, snap)
            if start_cell is None or goal_cell is None:
                return self._result(False, "PARKING_REFERENCE_ATTACH_FAILED", original, route,
                                    len(runs), 0.0, expanded_total, run_diagnostics, world)
            rows, cols = np.where(safe)
            margin = max(2, int(math.ceil(.50/world.map.resolution)))
            r0, r1 = max(0, int(rows.min())-margin), min(safe.shape[0], int(rows.max())+margin+1)
            c0, c1 = max(0, int(cols.min())-margin), min(safe.shape[1], int(cols.max())+margin+1)
            factor = max(1, int(round(self.policy.search_resolution_m/world.map.resolution)))
            local_safe = safe[r0:r1, c0:c1]
            deviation = np.asarray(world.grids["parking_deviation"][r0:r1, c0:c1], np.float32)
            if factor > 1:
                height = int(math.ceil(local_safe.shape[0]/factor))
                width = int(math.ceil(local_safe.shape[1]/factor))
                coarse_safe = cv2.resize(local_safe.astype(np.uint8), (width, height), interpolation=cv2.INTER_AREA) >= .999
                finite = np.where(np.isfinite(deviation), deviation, 1.0)
                coarse_deviation = cv2.resize(finite, (width, height), interpolation=cv2.INTER_AREA)
            else:
                coarse_safe = local_safe
                coarse_deviation = np.where(np.isfinite(deviation), deviation, 1.0)
            start = ((start_cell[0]-r0)//factor, (start_cell[1]-c0)//factor)
            goal = ((goal_cell[0]-r0)//factor, (goal_cell[1]-c0)//factor)
            start = _nearest(coarse_safe, start, max(2, snap//factor))
            goal = _nearest(coarse_safe, goal, max(2, snap//factor))
            if start is None or goal is None:
                return self._result(False, "PARKING_REFERENCE_COARSE_ATTACH_FAILED", original, route,
                                    len(runs), 0.0, expanded_total, run_diagnostics, world)
            target = coarse_deviation <= self.policy.target_deviation_max
            costs = (
                self.policy.centre_cost_weight*np.square(np.clip(coarse_deviation, 0.0, 1.0))
                + self.policy.outside_target_cost_weight*(~target)
            )
            path, expanded = _astar(
                coarse_safe, costs, start, goal, self.policy.maximum_expanded_cells,
            )
            expanded_total += expanded
            if path is None:
                return self._result(False, "PARKING_REFERENCE_NO_CONNECTED_ROUTE", original, route,
                                    len(runs), 0.0, expanded_total, run_diagnostics, world)
            full_cells = [
                (min(safe.shape[0]-1, r0+row*factor+factor//2),
                 min(safe.shape[1]-1, c0+col*factor+factor//2))
                for row, col in path
            ]
            full_cells[0], full_cells[-1] = start_cell, goal_cell
            smooth = _smooth_cells(
                full_cells, safe, self.policy.smoothing_passes, self.policy.smoothing_window,
            )
            xy = _cell_xy(world, smooth)
            if len(xy) > 2:
                simplified = cv2.approxPolyDP(
                    xy.astype(np.float32).reshape((-1, 1, 2)),
                    self.policy.simplify_tolerance_m, False,
                ).reshape((-1, 2)).astype(np.float64)
            else:
                simplified = xy
            stitched.extend(map(tuple, simplified))
            run_dev = []
            for point in _resample(simplified, self.policy.station_spacing_m):
                cell = world.map.world_to_cell(float(point[0]), float(point[1]))
                if cell is not None and component[cell]:
                    value = float(world.grids["parking_deviation"][cell])
                    if math.isfinite(value):
                        parking_samples += 1
                        target_samples += value <= self.policy.target_deviation_max
                        run_dev.append(value)
            run_diagnostics.append({
                "phase_instance": int(phase.instance),
                "start_station_m": float(samples[start_index].station_m),
                "end_station_m": float(samples[end_index].station_m),
                "start_snap_m": math.dist(
                    world.map.cell_to_world(start_cell),
                    (samples[start_index].x, samples[start_index].y),
                ),
                "goal_snap_m": math.dist(
                    world.map.cell_to_world(goal_cell),
                    (samples[end_index].x, samples[end_index].y),
                ),
                "expanded_cells": int(expanded),
                "coarse_path_cells": len(path),
                "reference_points": len(simplified),
                "target_ratio": float(np.mean(np.asarray(run_dev) <= self.policy.target_deviation_max)) if run_dev else 0.0,
                "deviation_p50": float(np.median(run_dev)) if run_dev else None,
            })
            cursor = end_index+1
        stitched.extend((samples[i].x, samples[i].y) for i in range(cursor, len(samples)))
        stitched = _deduplicate(stitched)
        if len(stitched) < 2:
            return self._result(False, "PARKING_REFERENCE_EMPTY", original, route,
                                len(runs), 0.0, expanded_total, run_diagnostics, world)
        curvature = _maximum_curvature(stitched)
        target_ratio = target_samples/max(1, parking_samples)
        # This centreline is a continuous geometric guide, not the executed
        # trajectory.  Its polygonal vertex curvature is diagnostic only.  A
        # reference is called curvature-feasible solely after the downstream
        # 48-bin Dubins lattice materialises and exactly replays a <=2.50 1/m
        # full-footprint path.  Rejecting the centreline here would conflate a
        # sampling corner with the vehicle trajectory it guides.
        result = self._result(
            True, "",
            stitched, route, len(runs), target_ratio, expanded_total, run_diagnostics, world,
            curvature=curvature,
        )
        return result

    def _result(
        self, passed: bool, code: str, points: Sequence[Sequence[float]],
        route: OrientedRoute, run_count: int, target_ratio: float, expanded: int,
        runs: Sequence[dict[str, Any]], world: RoutePhaseWorld, *, curvature: float | None = None,
    ) -> ParkingReferenceResult:
        values = tuple(_deduplicate(points))
        reference_hash = canonical_hash([list(value) for value in values])
        return ParkingReferenceResult(
            gate_passed=bool(passed), failure_code=str(code), polyline=values,
            original_route_hash=str(route.route_hash), reference_route_hash=reference_hash,
            original_length_m=float(route.length_m), reference_length_m=_polyline_length(values),
            parking_run_count=int(run_count), target_sample_ratio=float(target_ratio),
            maximum_reference_curvature_1pm=float(
                _maximum_curvature(values) if curvature is None else curvature
            ),
            footprint_sample_count=0, expanded_cells=int(expanded),
            diagnostics={
                "method_id": METHOD_ID, "policy": asdict(self.policy),
                "runs": list(runs), "map_resolution_m": float(world.map.resolution),
                "geometric_centerline_requires_se2_materialization": True,
                "binding_sha256": hashlib.sha256(repr((
                    world.meta.get("map_hash"), world.meta.get("semantic_map_hash"),
                    world.meta.get("query"), route.route_hash, asdict(self.policy),
                )).encode()).hexdigest(),
            },
        )


__all__ = [
    "METHOD_ID", "ParkingReferencePolicy", "ParkingReferenceResult",
    "ContinuousParkingReferenceBuilder", "_astar", "_maximum_curvature",
]
