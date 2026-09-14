"""Lexicographic parking-centre reference research for 2A-V3 r15.

This module is intentionally separate from the frozen r14 implementation.
It tests one narrow hypothesis: r14's weighted scalar grid objective can trade
away too much parking-centre coverage.  R15 first minimizes the number of
samples outside the approved centre band, then normalized deviation, and only
then path length.  The result remains a guide; the unchanged 48-bin,
forward-only Dubins search and full-footprint audit are still authoritative.
"""
from __future__ import annotations

from dataclasses import asdict
import heapq
import math
from typing import Any, Sequence

import cv2
import numpy as np

from .semantic_map import canonical_hash
from .semantic_parking_reference_v3 import (
    METHOD_ID as R14_METHOD_ID,
    ParkingReferencePolicy,
    ParkingReferenceResult,
    _cell_xy,
    _deduplicate,
    _maximum_curvature,
    _nearest,
    _polyline_length,
    _resample,
    _route_tube,
)
from .semantic_route_phase_v3 import OrientedRoute, Phase, RoutePhaseWorld, _phase_near


METHOD_ID = "parking_component_lexicographic_centre_reference_r15_v1"


def lexicographic_target_astar(
    traversable: np.ndarray,
    deviation: np.ndarray,
    target: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    maximum_expanded: int,
) -> tuple[list[tuple[int, int]] | None, int, tuple[float, float, float] | None]:
    """Deterministic lexicographic A* without corner cutting.

    The objective tuple is ``(outside_target_length, deviation_integral,
    geometric_length)``.  This prevents weights or units from allowing a
    shorter boundary-hugging path to defeat the approved centre-band goal.
    Euclidean distance is used only as a lower bound on the final tuple item,
    so it cannot alter the first two objective priorities.
    """
    free = np.asarray(traversable, dtype=bool)
    dev = np.where(np.isfinite(deviation), deviation, 1.0).astype(np.float64)
    centre = np.asarray(target, dtype=bool)
    height, width = free.shape
    start_id, goal_id = start[0] * width + start[1], goal[0] * width + goal[1]
    zero = (0.0, 0.0, 0.0)
    score: dict[int, tuple[float, float, float]] = {start_id: zero}
    parent: dict[int, int] = {}
    heuristic = math.hypot(goal[0] - start[0], goal[1] - start[1])
    heap = [(0.0, 0.0, heuristic, 0.0, start_id)]
    closed: set[int] = set()
    moves = (
        (-1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), (1, 0, 1.0),
        (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
    )
    while heap and len(closed) < int(maximum_expanded):
        outside, error, _estimated_length, length, node = heapq.heappop(heap)
        current = (outside, error, length)
        if node in closed or current != score.get(node):
            continue
        closed.add(node)
        if node == goal_id:
            path = []
            cursor = node
            while True:
                path.append(divmod(cursor, width))
                if cursor == start_id:
                    break
                cursor = parent[cursor]
            path.reverse()
            return path, len(closed), current
        row, col = divmod(node, width)
        for dr, dc, step_length in moves:
            rr, cc = row + dr, col + dc
            if not (0 <= rr < height and 0 <= cc < width and free[rr, cc]):
                continue
            if dr and dc and not (free[row, cc] and free[rr, col]):
                continue
            target_id = rr * width + cc
            step_outside = step_length * 0.5 * (
                float(not centre[row, col]) + float(not centre[rr, cc])
            )
            step_error = step_length * 0.5 * (float(dev[row, col]) + float(dev[rr, cc]))
            candidate = (
                outside + step_outside,
                error + step_error,
                length + step_length,
            )
            if candidate >= score.get(target_id, (math.inf, math.inf, math.inf)):
                continue
            score[target_id] = candidate
            parent[target_id] = node
            lower_length = candidate[2] + math.hypot(goal[0] - rr, goal[1] - cc)
            heapq.heappush(
                heap,
                (candidate[0], candidate[1], lower_length, candidate[2], target_id),
            )
    return None, len(closed), None


class LexicographicParkingReferenceBuilderR15:
    """Build a full-resolution, target-first parking reference."""

    def __init__(self, policy: ParkingReferencePolicy | None = None) -> None:
        self.policy = policy or ParkingReferencePolicy()

    def build(self, world: RoutePhaseWorld, route: OrientedRoute) -> ParkingReferenceResult:
        samples = route.stations(self.policy.station_spacing_m)
        phases = [_phase_near(world, item.x, item.y, 0.50) for item in samples]
        runs: list[tuple[int, int, Phase]] = []
        begin = 0
        for index in range(1, len(phases) + 1):
            if index == len(phases) or phases[index] != phases[begin]:
                if phases[begin].kind == "parking":
                    runs.append((begin, index - 1, phases[begin]))
                begin = index
        original = [[float(point[0]), float(point[1])] for point in route.points]
        if not runs:
            return self._result(True, "", original, route, 0, 0.0, 0, [], world)

        stitched: list[tuple[float, float]] = []
        cursor = 0
        expanded_total = 0
        run_diagnostics: list[dict[str, Any]] = []
        target_samples = 0
        parking_samples = 0
        for start_index, end_index, phase in runs:
            run_length = samples[end_index].station_m - samples[start_index].station_m
            if run_length < self.policy.minimum_parking_run_m:
                continue
            stitched.extend((samples[i].x, samples[i].y) for i in range(cursor, start_index))
            component = world.grids["parking_components"] == int(phase.instance)
            safe = (
                component
                & world.grids["allowed"].astype(bool)
                & ~world.grids["hard"].astype(bool)
                & (world.master < 253)
            )
            tube_points = [(samples[i].x, samples[i].y) for i in range(start_index, end_index + 1)]
            safe &= _route_tube(world, tube_points, self.policy.route_tube_half_width_m)
            start_cell = world.map.world_to_cell(samples[start_index].x, samples[start_index].y)
            goal_cell = world.map.world_to_cell(samples[end_index].x, samples[end_index].y)
            snap = int(math.ceil(self.policy.endpoint_snap_limit_m / world.map.resolution))
            start_cell = None if start_cell is None else _nearest(safe, start_cell, snap)
            goal_cell = None if goal_cell is None else _nearest(safe, goal_cell, snap)
            if start_cell is None or goal_cell is None:
                return self._result(
                    False, "PARKING_REFERENCE_ATTACH_FAILED", original, route,
                    len(runs), 0.0, expanded_total, run_diagnostics, world,
                )
            rows, cols = np.where(safe)
            margin = max(2, int(math.ceil(0.50 / world.map.resolution)))
            row0, row1 = max(0, int(rows.min()) - margin), min(safe.shape[0], int(rows.max()) + margin + 1)
            col0, col1 = max(0, int(cols.min()) - margin), min(safe.shape[1], int(cols.max()) + margin + 1)
            local_safe = safe[row0:row1, col0:col1]
            deviation = np.asarray(
                world.grids["parking_deviation"][row0:row1, col0:col1], np.float32,
            )
            start = (start_cell[0] - row0, start_cell[1] - col0)
            goal = (goal_cell[0] - row0, goal_cell[1] - col0)
            target = np.isfinite(deviation) & (deviation <= self.policy.target_deviation_max)
            path, expanded, objective = lexicographic_target_astar(
                local_safe, deviation, target, start, goal, self.policy.maximum_expanded_cells,
            )
            expanded_total += expanded
            if path is None:
                return self._result(
                    False, "PARKING_REFERENCE_NO_CONNECTED_ROUTE", original, route,
                    len(runs), 0.0, expanded_total, run_diagnostics, world,
                )
            full_cells = [(row0 + row, col0 + col) for row, col in path]
            xy = _cell_xy(
                world,
                np.asarray([[float(col), float(row)] for row, col in full_cells], dtype=np.float64),
            )
            # Preserve the target-first grid path.  Removing vertices across a
            # corner can cut through non-target or unsafe cells, so r15 only
            # removes exact duplicates before SE(2) materialization.
            simplified = np.asarray(_deduplicate(xy), dtype=np.float64)
            stitched.extend(map(tuple, simplified))
            run_dev = []
            for point in _resample(simplified, self.policy.station_spacing_m):
                cell = world.map.world_to_cell(float(point[0]), float(point[1]))
                if cell is not None and component[cell]:
                    value = float(world.grids["parking_deviation"][cell])
                    if math.isfinite(value):
                        parking_samples += 1
                        target_samples += int(value <= self.policy.target_deviation_max)
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
                "full_resolution_path_cells": len(path),
                "reference_points": len(simplified),
                "target_ratio": float(np.mean(np.asarray(run_dev) <= self.policy.target_deviation_max)) if run_dev else 0.0,
                "deviation_p50": float(np.median(run_dev)) if run_dev else None,
                "lexicographic_objective_cells": list(objective) if objective is not None else None,
                "target_band_safe_cells": int(np.count_nonzero(target & local_safe)),
            })
            cursor = end_index + 1
        stitched.extend((samples[i].x, samples[i].y) for i in range(cursor, len(samples)))
        stitched = _deduplicate(stitched)
        if len(stitched) < 2:
            return self._result(
                False, "PARKING_REFERENCE_EMPTY", original, route,
                len(runs), 0.0, expanded_total, run_diagnostics, world,
            )
        return self._result(
            True, "", stitched, route, len(runs),
            target_samples / max(1, parking_samples), expanded_total,
            run_diagnostics, world,
        )

    def _result(
        self,
        passed: bool,
        code: str,
        points: Sequence[Sequence[float]],
        route: OrientedRoute,
        run_count: int,
        target_ratio: float,
        expanded: int,
        runs: Sequence[dict[str, Any]],
        world: RoutePhaseWorld,
    ) -> ParkingReferenceResult:
        values = tuple(_deduplicate(points))
        return ParkingReferenceResult(
            gate_passed=bool(passed), failure_code=str(code), polyline=values,
            original_route_hash=str(route.route_hash),
            reference_route_hash=canonical_hash([list(value) for value in values]),
            original_length_m=float(route.length_m),
            reference_length_m=_polyline_length(values),
            parking_run_count=int(run_count), target_sample_ratio=float(target_ratio),
            maximum_reference_curvature_1pm=_maximum_curvature(values),
            footprint_sample_count=0, expanded_cells=int(expanded),
            diagnostics={
                "method_id": METHOD_ID,
                "parent_method_id": R14_METHOD_ID,
                "objective": "LEXICOGRAPHIC_OUTSIDE_TARGET_THEN_DEVIATION_THEN_LENGTH",
                "search_resolution_m": float(world.map.resolution),
                "policy": asdict(self.policy), "runs": list(runs),
                "map_resolution_m": float(world.map.resolution),
                "geometric_centerline_requires_se2_materialization": True,
                "binding_sha256": canonical_hash({
                    "map_hash": world.meta.get("map_hash"),
                    "semantic_map_hash": world.meta.get("semantic_map_hash"),
                    "query": world.meta.get("query"),
                    "route_hash": route.route_hash,
                    "policy": asdict(self.policy),
                    "method_id": METHOD_ID,
                }),
            },
        )


__all__ = [
    "METHOD_ID", "LexicographicParkingReferenceBuilderR15",
    "lexicographic_target_astar",
]
