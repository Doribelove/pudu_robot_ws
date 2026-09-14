"""Request-derived, finite ordered-corridor feasibility planner.

This module is deliberately independent of the earlier free-waypoint optimizer.
It builds every candidate from the current request's oriented L1 route and the
bound semantic/effective-master grids.  It never reads a saved path or control
certificate.  The finite graph is an offline feasibility family, not a claim of
continuous-space completeness and not an online Nav2 planner.

The graph is acyclic in route station.  Every accepted edge is an analytic,
forward-only Dubins edge, is checked with ``ConstraintWorld.validate_edge``
using dense full-footprint validation, remains in one semantic lane instance,
and has nondecreasing projection onto the request-oriented route.  Thus a path
cannot pass the goal station and return merely to improve endpoint-trimmed
semantic statistics.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import platform
import resource
import shutil
import time
from typing import Iterable, Sequence

import numpy as np

from .semantic_constraint_core import (
    ConstraintWorld,
    dense_interpolate,
    dubins_choices,
    dubins_edge_from_parameters,
    wrap_angle,
)
from .semantic_map import SemanticMapV1, canonical_hash, sha256_file
from .semantic_path_necessity_audit import audit_path_necessity
from .semantic_path_revisit_audit import audit_revisits
from .semantic_transition_contract import TransitionContractR2, resample_path
from .semantic_transition_r2_preflight import explicit_hard_audit, semantic_metrics


YAW_BIN_COUNT = 48
YAW_BIN_SIZE = 2.0 * math.pi / YAW_BIN_COUNT
METHOD_ID = "request_derived_ordered_lane_corridor_dubins_dag_v1"
PROTOCOL_ID = "PLN-02-ORDERED-CORRIDOR-OFFLINE-PREFLIGHT-V1"
CONTRACT_REVISION = "semantic-endpoint-transition-short-full-6m-r2"


class CorridorFailure(RuntimeError):
    """Expected fail-closed outcome carrying a stable machine-readable code."""

    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = code
        self.detail = detail


@dataclass(frozen=True)
class OrderedCorridorPolicy:
    station_spacing_m: float = 0.50
    lateral_sample_spacing_m: float = 0.25
    maximum_lateral_probe_m: float = 6.0
    maximum_cells_per_station: int = 5
    yaw_neighbor_bins: tuple[int, ...] = (-1, 0, 1)
    maximum_station_skip: int = 2
    maximum_local_edge_length_m: float = 2.50
    maximum_local_edge_ratio: float = 2.0
    maximum_labels_per_node: int = 8
    maximum_graph_edges: int = 100_000
    endpoint_attachment_limit_m: float = 0.75
    turning_radius_m: float = 0.401
    projection_epsilon_m: float = 1.0e-6

    def __post_init__(self) -> None:
        if self.station_spacing_m <= 0.0 or self.lateral_sample_spacing_m <= 0.0:
            raise ValueError("station and lateral spacing must be positive")
        if self.maximum_lateral_probe_m <= 0.0:
            raise ValueError("maximum lateral probe must be positive")
        if self.maximum_cells_per_station < 1 or self.maximum_station_skip < 1:
            raise ValueError("candidate and station-skip limits must be positive")
        if self.maximum_labels_per_node < 1 or self.maximum_graph_edges < 1:
            raise ValueError("finite graph limits must be positive")
        if self.endpoint_attachment_limit_m < 0.0:
            raise ValueError("endpoint attachment limit cannot be negative")
        if self.turning_radius_m < 0.40:
            raise ValueError("turning radius cannot relax the frozen 0.40 m limit")
        if not self.yaw_neighbor_bins:
            raise ValueError("at least one yaw-bin offset is required")
        if any(not isinstance(value, int) for value in self.yaw_neighbor_bins):
            raise ValueError("yaw offsets must be integer 48-bin offsets")


@dataclass(frozen=True)
class RouteSample:
    station_m: float
    x: float
    y: float
    tangent_x: float
    tangent_y: float


class OrientedRoute:
    """Exact-endpoint route with deterministic continuous station projection."""

    def __init__(
        self,
        polyline: Sequence[Sequence[float]],
        start: Sequence[float],
        goal: Sequence[float],
        endpoint_attachment_limit_m: float = 0.75,
    ) -> None:
        raw = np.asarray(polyline, dtype=np.float64)
        if raw.ndim != 2 or raw.shape[1] < 2 or len(raw) < 2:
            raise CorridorFailure("ROUTE_BINDING_FAILED", "route_polyline must contain at least two finite points")
        raw = raw[:, :2]
        if not np.all(np.isfinite(raw)):
            raise CorridorFailure("ROUTE_BINDING_FAILED", "route_polyline contains non-finite coordinates")
        keep = np.r_[True, np.linalg.norm(np.diff(raw, axis=0), axis=1) > 1.0e-9]
        raw = raw[keep]
        if len(raw) < 2:
            raise CorridorFailure("ROUTE_BINDING_FAILED", "route_polyline has no positive-length segment")
        start_xy = np.asarray(start[:2], dtype=np.float64)
        goal_xy = np.asarray(goal[:2], dtype=np.float64)
        normal = float(np.linalg.norm(raw[0] - start_xy) + np.linalg.norm(raw[-1] - goal_xy))
        reverse = float(np.linalg.norm(raw[-1] - start_xy) + np.linalg.norm(raw[0] - goal_xy))
        self.reversed_for_query = reverse < normal
        if self.reversed_for_query:
            raw = raw[::-1].copy()
            normal, reverse = reverse, normal
        self.normal_endpoint_sum_m = normal
        self.reversed_endpoint_sum_m = reverse
        start_distance = float(np.linalg.norm(raw[0] - start_xy))
        goal_distance = float(np.linalg.norm(raw[-1] - goal_xy))
        self.start_attachment_distance_m = start_distance
        self.goal_attachment_distance_m = goal_distance
        if max(start_distance, goal_distance) > endpoint_attachment_limit_m:
            raise CorridorFailure(
                "ROUTE_BINDING_FAILED",
                f"route endpoint attachment exceeds {endpoint_attachment_limit_m:.3f} m: "
                f"start={start_distance:.6f}, goal={goal_distance:.6f}",
            )
        # Endpoint attachment is part of the request binding, not an additional
        # route excursion.  Replacing the two bound samples also avoids a tiny
        # prepend/append backtrack when the raster route ends just beyond a pose.
        raw[0] = start_xy
        raw[-1] = goal_xy
        keep = np.r_[True, np.linalg.norm(np.diff(raw, axis=0), axis=1) > 1.0e-9]
        self.points = raw[keep]
        self.vectors = np.diff(self.points, axis=0)
        self.segment_lengths = np.linalg.norm(self.vectors, axis=1)
        self.tangents = self.vectors / self.segment_lengths[:, None]
        self.cumulative = np.r_[0.0, np.cumsum(self.segment_lengths)]
        self.length_m = float(self.cumulative[-1])
        self.route_hash = canonical_hash(self.points.tolist())

    def sample(self, station_m: float) -> RouteSample:
        station = min(max(float(station_m), 0.0), self.length_m)
        index = min(int(np.searchsorted(self.cumulative, station, side="right") - 1), len(self.vectors) - 1)
        distance = station - self.cumulative[index]
        point = self.points[index] + distance * self.tangents[index]
        tangent = self.tangents[index]
        return RouteSample(station, float(point[0]), float(point[1]), float(tangent[0]), float(tangent[1]))

    def stations(self, spacing_m: float) -> list[RouteSample]:
        values = np.arange(0.0, self.length_m, float(spacing_m), dtype=np.float64).tolist()
        if not values or self.length_m - values[-1] > 1.0e-9:
            values.append(self.length_m)
        return [self.sample(value) for value in values]

    def project(self, xy: Sequence[Sequence[float]]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Project poses to station, retaining signed endpoint-ray overshoot."""
        query = np.asarray(xy, dtype=np.float64)
        if query.ndim == 1:
            query = query[None, :]
        if query.ndim != 2 or query.shape[1] != 2 or not np.all(np.isfinite(query)):
            raise ValueError("route projection requires a finite Nx2 array")
        delta = query[:, None, :] - self.points[:-1][None, :, :]
        parameters = np.sum(delta * self.vectors[None, :, :], axis=2) / (self.segment_lengths[None, :] ** 2)
        clipped = np.clip(parameters, 0.0, 1.0)
        nearest = self.points[:-1][None, :, :] + clipped[:, :, None] * self.vectors[None, :, :]
        squared = np.sum((query[:, None, :] - nearest) ** 2, axis=2)
        segment = np.argmin(squared, axis=1)
        rows = np.arange(len(query))
        station = self.cumulative[segment] + clipped[rows, segment] * self.segment_lengths[segment]
        distance2 = squared[rows, segment]
        source = segment.astype(np.int32)

        start_parameter = (query - self.points[0]) @ self.tangents[0]
        start_valid = start_parameter < 0.0
        start_nearest = self.points[0] + start_parameter[:, None] * self.tangents[0]
        start_distance2 = np.sum((query - start_nearest) ** 2, axis=1)
        use_start = start_valid & (start_distance2 < distance2 - 1.0e-12)
        station[use_start] = start_parameter[use_start]
        distance2[use_start] = start_distance2[use_start]
        source[use_start] = -1

        goal_parameter = (query - self.points[-1]) @ self.tangents[-1]
        goal_valid = goal_parameter > 0.0
        goal_nearest = self.points[-1] + goal_parameter[:, None] * self.tangents[-1]
        goal_distance2 = np.sum((query - goal_nearest) ** 2, axis=1)
        use_goal = goal_valid & (goal_distance2 < distance2 - 1.0e-12)
        station[use_goal] = self.length_m + goal_parameter[use_goal]
        distance2[use_goal] = goal_distance2[use_goal]
        source[use_goal] = len(self.vectors)
        return station, np.sqrt(distance2), source

    def project_station_window(
        self, xy: Sequence[Sequence[float]], minimum_station_m: float,
        maximum_station_m: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Project against a bounded route interval for local edge filtering.

        This is only a permissive search prefilter.  The accepted complete path
        is still projected against every route segment by
        :func:`audit_route_progress`, so a self-near or crossing route cannot
        bypass the final ordered-progress gate.
        """
        query = np.asarray(xy, dtype=np.float64)
        if query.ndim == 1:
            query = query[None, :]
        if query.ndim != 2 or query.shape[1] != 2 or not np.all(np.isfinite(query)):
            raise ValueError("route projection requires a finite Nx2 array")
        low = max(0.0, float(minimum_station_m))
        high = min(self.length_m, float(maximum_station_m))
        first = max(0, int(np.searchsorted(self.cumulative, low, side="right") - 1))
        last = min(
            len(self.vectors),
            int(np.searchsorted(self.cumulative, high, side="left")) + 1,
        )
        last = max(first + 1, last)
        points = self.points[first:last]
        vectors = self.vectors[first:last]
        lengths = self.segment_lengths[first:last]
        delta = query[:, None, :] - points[None, :, :]
        parameters = np.sum(delta * vectors[None, :, :], axis=2) / (lengths[None, :] ** 2)
        clipped = np.clip(parameters, 0.0, 1.0)
        nearest = points[None, :, :] + clipped[:, :, None] * vectors[None, :, :]
        squared = np.sum((query[:, None, :] - nearest) ** 2, axis=2)
        local_segment = np.argmin(squared, axis=1)
        rows = np.arange(len(query))
        global_segment = local_segment + first
        station = (
            self.cumulative[global_segment]
            + clipped[rows, local_segment] * self.segment_lengths[global_segment]
        )
        return station, np.sqrt(squared[rows, local_segment]), global_segment.astype(np.int32)


def audit_route_progress(
    route: OrientedRoute,
    path: Sequence[Sequence[float]],
    *,
    epsilon_m: float = 1.0e-6,
    projection: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None,
) -> dict:
    poses = np.asarray(path, dtype=np.float64)
    if poses.ndim != 2 or poses.shape[1] < 2 or len(poses) < 2 or not np.all(np.isfinite(poses)):
        raise ValueError("progress audit requires a finite path with at least two poses")
    station, lateral, sources = (
        route.project(poses[:, :2]) if projection is None else projection
    )
    delta = np.diff(station)
    backward = float(np.maximum(-delta, 0.0).sum())
    overshoot = max(0.0, float(np.max(station) - route.length_m))
    undershoot = max(0.0, float(-np.min(station)))
    regression_indices = np.flatnonzero(delta < -epsilon_m)
    failures = []
    if overshoot > epsilon_m:
        failures.append("TERMINAL_STATION_OVERSHOOT")
    if undershoot > epsilon_m:
        failures.append("START_STATION_UNDERSHOOT")
    if len(regression_indices):
        failures.append("ROUTE_STATION_REGRESSION")
    return {
        "ordered_progress_gate_passed": not failures,
        "failure_codes": failures,
        "route_length_m": route.length_m,
        "minimum_projected_station_m": float(np.min(station)),
        "maximum_projected_station_m": float(np.max(station)),
        "terminal_station_overshoot_m": overshoot,
        "start_station_undershoot_m": undershoot,
        "backward_route_progress_m": backward,
        "regression_step_count": int(len(regression_indices)),
        "first_regression_pose_index": int(regression_indices[0] + 1) if len(regression_indices) else None,
        "maximum_route_lateral_distance_m": float(np.max(lateral)),
        "projection_source_hash": hashlib.sha256(np.ascontiguousarray(sources).tobytes()).hexdigest(),
        "station_hash": hashlib.sha256(np.ascontiguousarray(station).tobytes()).hexdigest(),
        "epsilon_m": float(epsilon_m),
        "scope": "oriented request route with signed endpoint-ray projection",
    }


@dataclass(frozen=True)
class CorridorState:
    state_id: int
    layer_index: int
    station_m: float
    x: float
    y: float
    yaw: float
    yaw_bin: int
    lane_label: int
    lateral_offset_m: float
    error_m: float
    correct_side: bool
    target_band: bool
    endpoint: str = ""

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw


@dataclass
class CorridorEdge:
    edge_id: int
    source: int
    target: int
    source_layer: int
    target_layer: int
    dubins_choice: int
    length_m: float
    lane_samples: int
    correct_samples: int
    target_samples: int
    error_integral_m2: float
    control: object

    @property
    def wrong_side_m(self) -> float:
        return max(0, self.lane_samples - self.correct_samples) * 0.025

    @property
    def outside_target_m(self) -> float:
        return max(0, self.lane_samples - self.target_samples) * 0.025


@dataclass
class CorridorGraph:
    route: OrientedRoute
    lane_label: int
    states: list[CorridorState]
    layers: list[list[int]]
    edges: list[CorridorEdge]
    outgoing: dict[int, list[int]]
    diagnostics: dict


def _cell(world, x: float, y: float):
    value = world.map.world_to_cell(float(x), float(y))
    return None if value is None else (int(value[0]), int(value[1]))


def _resolve_endpoint_lane(world, pose, radius_cells: int = 10) -> int:
    cell = _cell(world, pose[0], pose[1])
    if cell is None:
        return 0
    selected = set(map(int, world.selected))
    direct = int(world.grids["labels"][cell])
    if direct in selected:
        return direct
    row, col = cell
    labels = world.grids["labels"]
    row0, row1 = max(0, row - radius_cells), min(labels.shape[0], row + radius_cells + 1)
    col0, col1 = max(0, col - radius_cells), min(labels.shape[1], col + radius_cells + 1)
    choices = sorted(int(value) for value in np.unique(labels[row0:row1, col0:col1]) if int(value) in selected)
    return choices[0] if len(choices) == 1 else 0


def _candidate_cells(world, sample: RouteSample, lane_label: int, policy: OrderedCorridorPolicy):
    offsets = np.arange(
        -policy.maximum_lateral_probe_m,
        policy.maximum_lateral_probe_m + 0.5 * policy.lateral_sample_spacing_m,
        policy.lateral_sample_spacing_m,
        dtype=np.float64,
    )
    values = []
    seen = set()
    for offset in offsets:
        # Positive is the request-oriented right normal, matching r3.
        x = sample.x + float(offset) * sample.tangent_y
        y = sample.y - float(offset) * sample.tangent_x
        cell = _cell(world, x, y)
        if cell is None or cell in seen:
            continue
        seen.add(cell)
        row, col = cell
        if int(world.grids["labels"][cell]) != lane_label:
            continue
        if not bool(world.grids["allowed"][cell]) or bool(world.grids["hard"][cell]):
            continue
        if int(world.master[cell]) >= 253:
            continue
        error = float(world.grids["error"][cell])
        if not math.isfinite(error):
            continue
        correct = bool(world.grids["correct"][cell])
        clearance = float(world.map.distance_m[cell])
        values.append({
            "x": float(x), "y": float(y), "row": row, "col": col,
            "offset": float(offset), "error": error, "correct": correct,
            "target": bool(correct and error <= 0.50), "clearance": clearance,
        })
    if not values:
        return []
    # Preserve a ladder from the route attachment to the best semantic point.
    # Keeping only the centre and several near-identical target cells would
    # leave a multi-metre lateral gap and make an otherwise valid ordered
    # transition disconnected at 0.5 m station spacing.
    target_values = [item for item in values if item["target"]]
    correct_values = [item for item in values if item["correct"]]
    anchor_pool = target_values or correct_values or values
    semantic_anchor = min(
        anchor_pool,
        key=lambda item: (
            item["error"], -item["clearance"], abs(item["offset"]),
            item["row"], item["col"],
        ),
    )
    selected = []
    selected_cells = set()
    for fraction in np.linspace(0.0, 1.0, policy.maximum_cells_per_station):
        desired = float(fraction) * semantic_anchor["offset"]
        item = min(
            values,
            key=lambda value: (
                abs(value["offset"] - desired), not value["correct"],
                value["error"], -value["clearance"], value["row"], value["col"],
            ),
        )
        identity = item["row"], item["col"]
        if identity not in selected_cells:
            selected.append(item)
            selected_cells.add(identity)

    # Preserve transition geometry and semantic optima before filling any
    # remaining slots.  All tie breaks are explicit and deterministic.
    selectors = (
        lambda item: (abs(item["offset"]), item["row"], item["col"]),
        lambda item: (not item["target"], item["error"], -item["clearance"], abs(item["offset"]), item["row"], item["col"]),
        lambda item: (not item["correct"], item["error"], -item["clearance"], abs(item["offset"]), item["row"], item["col"]),
        lambda item: (-item["clearance"], abs(item["offset"]), item["row"], item["col"]),
    )
    for key in selectors:
        if len(selected) >= policy.maximum_cells_per_station:
            break
        item = min(values, key=key)
        identity = item["row"], item["col"]
        if identity not in selected_cells:
            selected.append(item)
            selected_cells.add(identity)
    ranked = sorted(
        values,
        key=lambda item: (
            not item["target"], not item["correct"], item["error"],
            -item["clearance"], abs(item["offset"]), item["row"], item["col"],
        ),
    )
    for item in ranked:
        if len(selected) >= policy.maximum_cells_per_station:
            break
        identity = item["row"], item["col"]
        if identity not in selected_cells:
            selected.append(item)
            selected_cells.add(identity)
    return selected[:policy.maximum_cells_per_station]


def build_candidate_layers(
    world, route: OrientedRoute, policy: OrderedCorridorPolicy,
    *, candidate_cell_provider=None,
):
    start_label = _resolve_endpoint_lane(world, world.start)
    goal_label = _resolve_endpoint_lane(world, world.goal)
    if start_label <= 0 or goal_label <= 0 or start_label != goal_label:
        raise CorridorFailure(
            "LANE_INSTANCE_MISMATCH",
            f"targeted ordered corridor requires one resolved endpoint lane: start={start_label}, goal={goal_label}",
        )
    lane_label = start_label
    states: list[CorridorState] = []
    layers: list[list[int]] = []

    def append_state(**kwargs) -> int:
        state_id = len(states)
        states.append(CorridorState(state_id=state_id, **kwargs))
        return state_id

    start_station = float(route.project([world.start[:2]])[0][0])
    layers.append([append_state(
        layer_index=0, station_m=start_station, x=float(world.start[0]), y=float(world.start[1]),
        yaw=float(world.start[2]), yaw_bin=round(float(world.start[2]) / YAW_BIN_SIZE) % YAW_BIN_COUNT,
        lane_label=lane_label, lateral_offset_m=0.0, error_m=float("nan"),
        correct_side=False, target_band=False, endpoint="start",
    )])

    provider = _candidate_cells if candidate_cell_provider is None else candidate_cell_provider
    sampled = route.stations(policy.station_spacing_m)
    interior = sampled[1:-1]
    per_layer_target = []
    for sample in interior:
        layer_index = len(layers)
        layer = []
        base_bin = round(math.atan2(sample.tangent_y, sample.tangent_x) / YAW_BIN_SIZE) % YAW_BIN_COUNT
        cells = provider(world, sample, lane_label, policy)
        for item in cells:
            for offset_bin in sorted(set(policy.yaw_neighbor_bins)):
                yaw_bin = (base_bin + offset_bin) % YAW_BIN_COUNT
                yaw = wrap_angle(yaw_bin * YAW_BIN_SIZE)
                pose = np.asarray([[item["x"], item["y"], yaw]], dtype=np.float64)
                if not world.collision_free(pose):
                    continue
                actual_station = float(route.project(pose[:, :2])[0][0])
                if abs(actual_station - sample.station_m) > 0.51 * policy.station_spacing_m:
                    continue
                layer.append(append_state(
                    layer_index=layer_index, station_m=actual_station,
                    x=item["x"], y=item["y"], yaw=yaw, yaw_bin=yaw_bin,
                    lane_label=lane_label, lateral_offset_m=item["offset"], error_m=item["error"],
                    correct_side=item["correct"], target_band=item["target"], endpoint="",
                ))
        layer.sort(key=lambda state_id: (
            states[state_id].lateral_offset_m, states[state_id].yaw_bin,
            states[state_id].x, states[state_id].y,
        ))
        layers.append(layer)
        per_layer_target.append(any(states[state_id].target_band for state_id in layer))

    goal_layer = len(layers)
    goal_station = float(route.project([world.goal[:2]])[0][0])
    layers.append([append_state(
        layer_index=goal_layer, station_m=goal_station, x=float(world.goal[0]), y=float(world.goal[1]),
        yaw=float(world.goal[2]), yaw_bin=round(float(world.goal[2]) / YAW_BIN_SIZE) % YAW_BIN_COUNT,
        lane_label=lane_label, lateral_offset_m=0.0, error_m=float("nan"),
        correct_side=False, target_band=False, endpoint="goal",
    )])
    if not any(per_layer_target):
        raise CorridorFailure(
            "NO_MONOTONE_TARGET_CORRIDOR",
            "no footprint-valid target-band state exists inside the request route station interval",
        )
    diagnostics = {
        "lane_label": lane_label,
        "layer_count": len(layers),
        "interior_layer_count": len(interior),
        "nonempty_interior_layer_count": sum(bool(layer) for layer in layers[1:-1]),
        "target_candidate_layer_count": sum(per_layer_target),
        "target_candidate_layer_ratio_upper_bound": float(sum(per_layer_target) / len(per_layer_target)) if per_layer_target else 0.0,
        "state_count": len(states),
        "yaw_bin_count": YAW_BIN_COUNT,
        "yaw_neighbor_bins": list(policy.yaw_neighbor_bins),
    }
    return lane_label, states, layers, diagnostics


def _edge_semantics(world, edge) -> tuple[int, int, int, float]:
    rows, cols, inside = world.cells(edge.samples)
    if not np.all(inside):
        return 0, 0, 0, float("inf")
    errors = np.asarray(world.grids["error"][rows, cols], dtype=np.float64)
    finite = np.isfinite(errors)
    integral = float(np.sum(errors[finite]) * 0.025) if np.any(finite) else float("inf")
    return int(edge.n), int(edge.correct), int(edge.target), integral


def _progressive_edge(
    route: OrientedRoute,
    edge,
    source: CorridorState,
    target: CorridorState,
    epsilon_m: float,
    *, dense: bool = True,
) -> tuple[bool, dict]:
    samples = np.vstack((edge.start, edge.samples))
    checked = dense_interpolate(samples) if dense else samples
    projection = route.project_station_window(
        checked[:, :2], source.station_m - 0.25, target.station_m + 0.25,
    )
    progress = audit_route_progress(
        route, checked, epsilon_m=epsilon_m, projection=projection,
    )
    station = projection[0]
    endpoint_ordered = bool(
        abs(station[0] - source.station_m) <= max(epsilon_m, 1.0e-5)
        and abs(station[-1] - target.station_m) <= max(epsilon_m, 1.0e-5)
        and target.station_m > source.station_m + epsilon_m
    )
    return bool(progress["ordered_progress_gate_passed"] and endpoint_ordered), progress


def build_graph(
    world, route: OrientedRoute, policy: OrderedCorridorPolicy,
    *, candidate_cell_provider=None,
) -> CorridorGraph:
    lane_label, states, layers, diagnostics = build_candidate_layers(
        world, route, policy, candidate_cell_provider=candidate_cell_provider,
    )
    edges: list[CorridorEdge] = []
    outgoing: dict[int, list[int]] = {state.state_id: [] for state in states}
    rejected = Counter()
    started = time.monotonic()
    for source_layer, sources in enumerate(layers[:-1]):
        for target_layer in range(source_layer + 1, min(len(layers), source_layer + 1 + policy.maximum_station_skip)):
            targets = layers[target_layer]
            for source_id in sources:
                source = states[source_id]
                for target_id in targets:
                    target = states[target_id]
                    direct = math.dist(source.pose[:2], target.pose[:2])
                    bound = min(
                        policy.maximum_local_edge_length_m,
                        policy.maximum_local_edge_ratio * direct + 0.05,
                    )
                    if direct <= policy.projection_epsilon_m or direct > policy.maximum_local_edge_length_m:
                        rejected["EDGE_DISTANCE_BOUND"] += 1
                        continue
                    accepted = []
                    for choice, (word, params) in enumerate(
                        dubins_choices(source.pose, target.pose, policy.turning_radius_m)
                    ):
                        analytic_length = policy.turning_radius_m * sum(params)
                        if analytic_length > bound:
                            rejected["DUBINS_LENGTH_BOUND"] += 1
                            continue
                        control = dubins_edge_from_parameters(
                            source.pose, target.pose, policy.turning_radius_m,
                            word, params,
                        )
                        progressive, _ = _progressive_edge(
                            route, control, source, target, policy.projection_epsilon_m,
                        )
                        if not progressive:
                            rejected["NON_MONOTONE_OR_OVERSHOOT_EDGE"] += 1
                            continue
                        rows, cols, inside = world.cells(control.samples)
                        if not np.all(inside) or np.any(world.grids["labels"][rows, cols] != lane_label):
                            rejected["LANE_INSTANCE_EDGE_ESCAPE"] += 1
                            continue
                        if not world.validate_edge(control, dense=True):
                            rejected["FULL_FOOTPRINT_OR_HARD_EDGE"] += 1
                            continue
                        n, correct, target_count, error_integral = _edge_semantics(world, control)
                        accepted.append((
                            50.0 * max(0, n - correct) + 20.0 * max(0, n - target_count)
                            + error_integral + control.length,
                            choice, control, n, correct, target_count, error_integral,
                        ))
                    if not accepted:
                        continue
                    # One deterministic, semantics-first control per state pair
                    # keeps the finite family bounded; discarded choices are
                    # reflected in the graph scope and are not a completeness claim.
                    _, choice, control, n, correct, target_count, error_integral = min(
                        accepted, key=lambda item: (item[0], item[2].length, item[1], item[2].word),
                    )
                    edge_id = len(edges)
                    edges.append(CorridorEdge(
                        edge_id=edge_id, source=source_id, target=target_id,
                        source_layer=source_layer, target_layer=target_layer,
                        dubins_choice=choice, length_m=float(control.length),
                        lane_samples=n, correct_samples=correct, target_samples=target_count,
                        error_integral_m2=error_integral, control=control,
                    ))
                    outgoing[source_id].append(edge_id)
                    if len(edges) > policy.maximum_graph_edges:
                        raise CorridorFailure(
                            "FINITE_GRAPH_RESOURCE_LIMIT",
                            f"edge count exceeded configured cap {policy.maximum_graph_edges}",
                        )
    for edge_ids in outgoing.values():
        edge_ids.sort(key=lambda edge_id: (
            edges[edge_id].target_layer, edges[edge_id].target,
            edges[edge_id].length_m, edges[edge_id].dubins_choice,
        ))
    diagnostics.update({
        "edge_count": len(edges), "edge_rejection_counts": dict(sorted(rejected.items())),
        "graph_build_wall_s": time.monotonic() - started,
        "finite_family": "bounded station/lateral/yaw candidates; one selected progressive Dubins word per state pair",
    })
    return CorridorGraph(route, lane_label, states, layers, edges, outgoing, diagnostics)


@dataclass(frozen=True)
class SearchLabel:
    label_id: int
    node: int
    profile: str
    score: float
    length_m: float
    wrong_side_m: float
    outside_target_m: float
    error_integral_m2: float
    parent: int
    edge_id: int
    signature: tuple[int, ...]


def _profile_score(profile: str, length: float, wrong: float, outside: float, error: float) -> float:
    if profile == "shortest":
        return length
    if profile == "balanced":
        return length + 20.0 * wrong + 8.0 * outside + error
    if profile == "semantic_first":
        return length + 100.0 * wrong + 50.0 * outside + 2.0 * error
    raise ValueError(profile)


def search_complete_paths(graph: CorridorGraph, policy: OrderedCorridorPolicy):
    """Return deterministic bounded labels reaching the exact goal state."""
    all_complete = []
    search_diagnostics = {}
    for profile in ("semantic_first", "balanced", "shortest"):
        at_node: dict[int, list[SearchLabel]] = {state.state_id: [] for state in graph.states}
        generated_label_count = 0

        def add(node, length, wrong, outside, error, parent, edge_id, signature):
            nonlocal generated_label_count
            score = _profile_score(profile, length, wrong, outside, error)
            label_id = generated_label_count
            generated_label_count += 1
            label = SearchLabel(
                label_id, node, profile, score, length, wrong, outside,
                error, parent, edge_id, signature,
            )
            bucket = at_node[node]
            bucket.append(label)
            bucket.sort(key=lambda item: (
                item.score, item.length_m,
                item.wrong_side_m, item.outside_target_m,
                item.signature,
            ))
            del bucket[policy.maximum_labels_per_node:]
            return label

        start = graph.layers[0][0]
        add(start, 0.0, 0.0, 0.0, 0.0, -1, -1, ())
        expanded = 0
        for layer in graph.layers[:-1]:
            for node in layer:
                for label in tuple(at_node[node]):
                    expanded += 1
                    for edge_id in graph.outgoing[node]:
                        edge = graph.edges[edge_id]
                        add(
                            edge.target,
                            label.length_m + edge.length_m,
                            label.wrong_side_m + edge.wrong_side_m,
                            label.outside_target_m + edge.outside_target_m,
                            label.error_integral_m2 + edge.error_integral_m2,
                            label.label_id, edge_id, label.signature + (edge_id,),
                        )
        goal = graph.layers[-1][0]
        complete = list(at_node[goal])
        # The exact edge signature is already carried by each live label.
        # Keeping every displaced label only for parent pointers caused
        # multi-gigabyte growth on long routes.
        all_complete.extend((label, []) for label in complete)
        search_diagnostics[profile] = {
            "expanded_label_count": expanded,
            "generated_label_count": generated_label_count,
            "stored_label_count": sum(len(bucket) for bucket in at_node.values()),
            "complete_label_count": len(complete),
        }
    unique = {}
    for label, labels in all_complete:
        unique.setdefault(label.signature, (label, labels))
    ordered = sorted(
        unique.values(),
        key=lambda item: (
            item[0].score, item[0].length_m, item[0].wrong_side_m,
            item[0].outside_target_m, item[0].profile, item[0].signature,
        ),
    )
    return ordered, search_diagnostics


def _controls_for_label(label: SearchLabel, labels: list[SearchLabel], graph: CorridorGraph):
    if label.signature:
        return [graph.edges[index].control for index in label.signature]
    edge_ids = []
    current = label
    while current.parent >= 0:
        edge_ids.append(current.edge_id)
        current = labels[current.parent]
    edge_ids.reverse()
    return [graph.edges[index].control for index in edge_ids]


def evaluate_controls(world, route: OrientedRoute, controls, semantic_map: SemanticMapV1) -> dict:
    if not controls:
        return {"gate_passed": False, "failure_codes": ["EMPTY_PATH"]}
    audit, path = world.audit(controls)
    points = [dict(
        x=float(x), y=float(y), yaw=float(yaw), source="kinematic",
        motion_direction="forward", steering=0.0,
        planner_backend=METHOD_ID, backend_version="offline_feasibility_v1",
    ) for x, y, yaw in path]
    metrics = semantic_metrics(world, points)
    hard = explicit_hard_audit(world, points, semantic_map)
    revisit = audit_revisits(path)
    sampled_points, station = resample_path(points)
    sampled_path = np.asarray([
        [point[key] for key in ("x", "y", "yaw")] for point in sampled_points
    ], dtype=np.float64)
    rows, cols, inside = world.cells(sampled_path)
    target = np.zeros(len(sampled_path), dtype=bool)
    if np.all(inside):
        lane = np.isin(world.grids["labels"][rows, cols], world.selected)
        target = (
            lane
            & world.grids["correct"][rows, cols].astype(bool)
            & (world.grids["error"][rows, cols] <= TransitionContractR2().lane_target_error_max_m)
        )
    if float(station[-1]) > TransitionContractR2().short_path_max_m:
        endpoint = TransitionContractR2().endpoint_transition_each_m
        target &= (station >= endpoint) & (station <= float(station[-1]) - endpoint)
    necessity = audit_path_necessity(
        sampled_path,
        start=world.start,
        goal=world.goal,
        target_mask=target,
        route_polyline=route.points,
    )
    dense = dense_interpolate(path)
    progress = audit_route_progress(route, dense)
    failures = []
    required = {
        "CANONICAL_FINAL_INVALID": bool(audit["canonical"]["final_valid_success"]),
        "PADDED_MASTER_COLLISION": bool(audit["padded_effective_master_collision_free"]),
        "ENDPOINT_MISMATCH": bool(audit["exact_endpoint_xy_yaw"]),
        "EDGE_DISCONTINUITY": bool(audit["edge_continuity"]),
        "CONTROL_REPLAY_MISMATCH": bool(audit["trace_replay_exact"]),
        "LANE_INSTANCE_ESCAPE": bool(audit["same_lane_instance"]),
        "CURVATURE_LIMIT": bool(audit["maximum_control_curvature_1pm"] <= 2.50),
        "PATH_LENGTH_BOUND": bool(audit["arc_length_m"] <= world.bound_length),
        "HARD_FEATURE_GATE": bool(hard["hard_feature_gate_passed"]),
        "R2_SEMANTIC_GATE": bool(metrics["semantic_gate_passed"]),
        "GEOMETRIC_REVISIT": bool(revisit["revisit_screen_passed"]),
        "NATURALNESS_SCREEN": bool(
            necessity.get("input_complete") is True
            and necessity.get("audit_passed") is True
        ),
        "ORDERED_PROGRESS": bool(progress["ordered_progress_gate_passed"]),
    }
    failures.extend(code for code, passed in required.items() if not passed)
    return {
        "gate_passed": not failures,
        "failure_codes": failures,
        "safety": audit,
        "semantics": metrics,
        "hard_features": hard,
        "revisit": revisit,
        "naturalness": necessity,
        "ordered_progress": progress,
        "points": points,
        "controls": [control.certificate() for control in controls],
    }


def _write_json(path: Path, value) -> None:
    with Path(path).open("x", encoding="utf8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def _serializable_policy(policy: OrderedCorridorPolicy) -> dict:
    value = asdict(policy)
    value["yaw_neighbor_bins"] = list(policy.yaw_neighbor_bins)
    return value


def run(
    inputs: Path,
    query: str,
    output: Path,
    policy: OrderedCorridorPolicy | None = None,
) -> dict:
    policy = policy or OrderedCorridorPolicy()
    inputs, output = Path(inputs).resolve(), Path(output).resolve()
    output.mkdir(parents=False, exist_ok=False)
    started, cpu = time.monotonic(), time.process_time()
    source = Path(__file__).resolve()
    shutil.copy2(source, output / "source_snapshot.py")
    protocol = {
        "protocol_id": PROTOCOL_ID, "method_id": METHOD_ID,
        "contract_revision": CONTRACT_REVISION,
        "architecture_id": "UNNAMED_ORDERED_CORRIDOR_CANDIDATE",
        "query_id": query, "policy": _serializable_policy(policy),
        "policy_hash": canonical_hash(_serializable_policy(policy)),
        "yaw_bin_count": YAW_BIN_COUNT, "motion_model": "forward_only_DUBIN",
        "minimum_turning_radius_m": 0.40,
        "implemented_turning_radius_m": policy.turning_radius_m,
        "used_historical_paths": False, "online": False,
        "finite_family_scope": "request route station/lateral/yaw graph; not continuous-space complete",
        "source_sha256_at_start": sha256_file(source),
        "input_meta_sha256": sha256_file(inputs / f"{query}.json"),
    }
    _write_json(output / "protocol.json", protocol)
    result = None
    witness = None
    try:
        world = ConstraintWorld(inputs, query)
        semantic_path = inputs.parent / "conversion_v1/semantic_map_v1.json"
        semantic_map = SemanticMapV1.load(semantic_path)
        if semantic_map.semantic_map_hash != world.meta["semantic_map_hash"]:
            raise CorridorFailure("SEMANTIC_BINDING_MISMATCH", "semantic feature hash does not match constraint input")
        route = OrientedRoute(
            world.meta["route_polyline"], world.start, world.goal,
            policy.endpoint_attachment_limit_m,
        )
        graph = build_graph(world, route, policy)
        candidates, search = search_complete_paths(graph, policy)
        evaluations = []
        for rank, (label, labels) in enumerate(candidates):
            controls = _controls_for_label(label, labels, graph)
            evaluated = evaluate_controls(world, route, controls, semantic_map)
            lane = evaluated.get("semantics", {}).get("active_window", {}).get("classes", {}).get("lane", {})
            evaluations.append({
                "rank": rank, "profile": label.profile, "score": label.score,
                "path_length_m": label.length_m,
                "correct_side_ratio": lane.get("correct_side_ratio"),
                "target_band_ratio": lane.get("target_band_ratio"),
                "lateral_error_p50_m": lane.get("lateral_error_p50_m"),
                "failure_codes": evaluated["failure_codes"],
                "gate_passed": evaluated["gate_passed"],
            })
            if evaluated["gate_passed"]:
                witness = evaluated
                break
        if witness is not None:
            failure_code = ""
            gate = True
        elif not candidates:
            failure_code = "NO_MONOTONE_TARGET_CORRIDOR"
            gate = False
        else:
            failure_code = "FINITE_ORDERED_GRAPH_NO_R2_WITNESS"
            gate = False
        result = {
            "protocol_id": PROTOCOL_ID, "method_id": METHOD_ID,
            "architecture_id": "UNNAMED_ORDERED_CORRIDOR_CANDIDATE",
            "contract_revision": CONTRACT_REVISION, "query_id": query,
            "gate_passed": gate, "failure_code": failure_code,
            "graph": graph.diagnostics, "search": search,
            "candidate_evaluations": evaluations,
            "route": {
                "route_hash": route.route_hash, "route_length_m": route.length_m,
                "reversed_for_query": route.reversed_for_query,
                "start_attachment_distance_m": route.start_attachment_distance_m,
                "goal_attachment_distance_m": route.goal_attachment_distance_m,
                "normal_endpoint_sum_m": route.normal_endpoint_sum_m,
                "reversed_endpoint_sum_m": route.reversed_endpoint_sum_m,
            },
            "finite_graph_upper_bound": {
                "target_candidate_layer_ratio": graph.diagnostics["target_candidate_layer_ratio_upper_bound"],
                "claim": "candidate-layer availability only; not a continuous-space or R2-path upper bound",
            },
            "input_npz_sha256": world.meta["npz_sha256"],
            "map_hash": world.meta["map_hash"],
            "semantic_map_hash": world.meta["semantic_map_hash"],
            "used_historical_paths": False, "online": False,
            "proof_scope": "bounded deterministic ordered graph only",
        }
    except CorridorFailure as error:
        result = {
            "protocol_id": PROTOCOL_ID, "method_id": METHOD_ID,
            "architecture_id": "UNNAMED_ORDERED_CORRIDOR_CANDIDATE",
            "contract_revision": CONTRACT_REVISION, "query_id": query,
            "gate_passed": False, "failure_code": error.code,
            "failure_detail": error.detail, "used_historical_paths": False,
            "online": False, "proof_scope": "fail-closed request-derived topology preflight",
        }
    result.update({
        "wall_s": time.monotonic() - started,
        "cpu_s": time.process_time() - cpu,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        "python": platform.python_version(),
        "source_sha256_at_start": protocol["source_sha256_at_start"],
    })
    if witness is not None:
        _write_json(output / "path.json", witness["points"])
        _write_json(output / "controls.json", {"edges": witness["controls"]})
        saved = dict(witness)
        saved.pop("points")
        saved.pop("controls")
        _write_json(output / "witness_audit.json", saved)
    _write_json(output / "result.json", result)
    hashes = {
        str(path.relative_to(output)): sha256_file(path)
        for path in sorted(output.iterdir()) if path.is_file() and path.name != "artifact_hashes.json"
    }
    _write_json(output / "artifact_hashes.json", hashes)
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--query", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--station-spacing", type=float, default=0.50)
    parser.add_argument("--lateral-spacing", type=float, default=0.25)
    parser.add_argument("--maximum-lateral-probe", type=float, default=6.0)
    parser.add_argument("--maximum-cells-per-station", type=int, default=5)
    parser.add_argument("--maximum-labels-per-node", type=int, default=8)
    args = parser.parse_args(argv)
    policy = OrderedCorridorPolicy(
        station_spacing_m=args.station_spacing,
        lateral_sample_spacing_m=args.lateral_spacing,
        maximum_lateral_probe_m=args.maximum_lateral_probe,
        maximum_cells_per_station=args.maximum_cells_per_station,
        maximum_labels_per_node=args.maximum_labels_per_node,
    )
    result = run(args.inputs, args.query, args.output, policy)
    print(json.dumps({
        "query_id": args.query, "gate_passed": result["gate_passed"],
        "failure_code": result.get("failure_code", ""),
        "wall_s": result["wall_s"],
    }, indent=2, sort_keys=True))
    return 0 if result["gate_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
