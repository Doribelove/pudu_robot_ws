"""Route-phase SE(2) state lattice for the general 2A-V3 query family.

The targeted V3 planner proved that an explicit 48-bin, forward-only Dubins
search can enforce a lane-relative guide.  That planner deliberately required
one lane instance for the whole request, so it cannot represent a route that
crosses a junction or enters a parking component.  This module generalises the
same search interface without weakening that boundary:

* each route station is bound to one lane instance, one parking component, or
  an explicit neutral transfer phase;
* lane states retain the unchanged right-side target, parking states retain
  the approved normalised centre target, and transfer states have no invented
  lateral target;
* every Dubins primitive is checked against the effective master, the padded
  footprint, the R0 corridor, and the station-bound semantic phase;
* the complete path is judged by the unchanged R2 class-wise statistics,
  canonical PathAudit, explicit hard-feature audit, ordered progress, and
  revisit/naturalness screens.

This is an independent experimental planner.  It does not modify or pretend to
be the pinned Nav2 Smac implementation.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass
import hashlib
import heapq
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Mapping, Sequence

import cv2
import numpy as np

from .path_audit import PathAuditor
from .planner_benchmark.map_utils import HospitalMap
from .semantic_constraint_core import (
    CropMap,
    PADDED_FOOTPRINT,
    SPACING,
    dense_interpolate,
    dubins_choices,
    dubins_edge_from_parameters,
    replay_edge,
    wrap_angle,
)
from .semantic_map import SemanticMapV1, sha256_file
from .semantic_path_necessity_audit import audit_path_necessity
from .semantic_path_revisit_audit import audit_revisits
from .semantic_transition_contract import (
    TransitionContractR2,
    audit_transition_samples,
    resample_path,
)
from .semantic_transition_ordered_corridor import (
    CorridorFailure,
    OrientedRoute,
    YAW_BIN_COUNT,
    YAW_BIN_SIZE,
    audit_route_progress,
)
from .semantic_transition_r2_preflight import explicit_hard_audit


METHOD_ID = "request_derived_route_phase_dubins_lattice_v1"


@dataclass(frozen=True)
class RoutePhasePolicy:
    station_spacing_m: float = 0.50
    station_slab_half_width_m: float = 0.20
    maximum_lateral_probe_m: float = 6.0
    maximum_cells_per_station: int = 9
    yaw_neighbor_bins: tuple[int, ...] = (-1, 0, 1)
    maximum_station_skip: int = 3
    maximum_local_edge_length_m: float = 2.75
    maximum_local_edge_ratio: float = 2.0
    endpoint_attachment_limit_m: float = 0.75
    turning_radius_m: float = 0.401
    projection_epsilon_m: float = 1.0e-6
    valid_successors_per_target_layer: int = 8
    live_labels_per_state: int = 5
    maximum_expanded_labels: int = 12_000
    maximum_goal_candidates: int = 48
    progress_priority_per_m: float = 55.0
    short_route_max_m: float = 25.0
    short_route_progress_priority_per_m: float = 1.0
    phase_probe_radius_m: float = 0.50
    path_length_route_ratio_max: float = 1.25
    path_length_route_slack_m: float = 2.0
    lane_wrong_weight: float = 2000.0
    lane_outside_target_weight: float = 1000.0
    parking_outside_center_weight: float = 1000.0
    semantic_error_weight: float = 2.0
    phase_transition_layer_tolerance: int = 2

    def __post_init__(self) -> None:
        positive = (
            self.station_spacing_m,
            self.station_slab_half_width_m,
            self.maximum_lateral_probe_m,
            self.maximum_local_edge_length_m,
            self.maximum_local_edge_ratio,
            self.turning_radius_m,
            self.phase_probe_radius_m,
            self.path_length_route_ratio_max,
            self.short_route_max_m,
            self.short_route_progress_priority_per_m,
            self.lane_wrong_weight,
            self.lane_outside_target_weight,
            self.parking_outside_center_weight,
            self.semantic_error_weight,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("route-phase distance and ratio limits must be positive")
        if self.turning_radius_m < 0.40:
            raise ValueError("turning radius cannot relax the frozen 0.40 m minimum")
        integers = (
            self.maximum_cells_per_station,
            self.maximum_station_skip,
            self.valid_successors_per_target_layer,
            self.live_labels_per_state,
            self.maximum_expanded_labels,
            self.maximum_goal_candidates,
            self.phase_transition_layer_tolerance,
        )
        if any(value < 1 for value in integers):
            raise ValueError("route-phase resource bounds must be positive")
        if not self.yaw_neighbor_bins or any(not isinstance(v, int) for v in self.yaw_neighbor_bins):
            raise ValueError("yaw neighbour offsets must be non-empty integer 48-bin offsets")


@dataclass(frozen=True)
class Phase:
    kind: str
    instance: int = 0

    def __post_init__(self) -> None:
        if self.kind not in {"lane", "parking", "transfer"}:
            raise ValueError(f"unknown route phase {self.kind}")
        if self.kind != "transfer" and self.instance <= 0:
            raise ValueError("semantic phase requires a positive instance id")


@dataclass(frozen=True)
class State:
    state_id: int
    layer_index: int
    station_m: float
    x: float
    y: float
    yaw: float
    yaw_bin: int
    phase_kind: str
    phase_instance: int
    lateral_offset_m: float
    semantic_error: float
    semantic_correct: bool
    semantic_target: bool
    endpoint: str = ""

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw


@dataclass
class Edge:
    edge_id: int
    source: int
    target: int
    source_layer: int
    target_layer: int
    dubins_choice: int
    length_m: float
    lane_samples: int
    lane_correct: int
    lane_target: int
    parking_samples: int
    parking_center: int
    semantic_error_integral: float
    control: Any

    @property
    def lane_wrong_m(self) -> float:
        return max(0, self.lane_samples - self.lane_correct) * SPACING

    @property
    def lane_outside_m(self) -> float:
        return max(0, self.lane_samples - self.lane_target) * SPACING

    @property
    def parking_outside_m(self) -> float:
        return max(0, self.parking_samples - self.parking_center) * SPACING


class RoutePhaseWorld:
    """Hash-bound cropped view of a multi-semantic request input."""

    ARRAY_NAMES = (
        "master", "occupancy", "allowed", "base_allowed", "labels", "lane_mask",
        "error", "correct", "right", "left", "parking_components",
        "parking_deviation", "junction", "hard", "no_stopping",
    )

    def __init__(
        self, input_dir: Path, query_id: str, *, arrays_override=None,
        meta_override=None, master_override=None, expected_master_hash=None,
    ) -> None:
        input_dir = Path(input_dir)
        self.meta = (
            dict(meta_override) if meta_override is not None
            else __import__("json").loads((input_dir / f"{query_id}.json").read_text())
        )
        archive = input_dir / f"{query_id}.npz"
        if arrays_override is None and sha256_file(archive) != self.meta["npz_sha256"]:
            raise ValueError("route-phase input archive hash mismatch")
        data_context = np.load(archive) if arrays_override is None else arrays_override
        try:
            full = {name: np.asarray(data_context[name]) for name in self.ARRAY_NAMES}
        finally:
            if arrays_override is None:
                data_context.close()
        desc = self.meta["map"]
        shape = (int(desc["height"]), int(desc["width"]))
        if any(value.shape != shape for value in full.values()):
            raise ValueError("route-phase arrays do not match the frozen map shape")
        if master_override is not None:
            from .semantic_rasterizer import grid_hash
            effective = np.asarray(master_override, dtype=np.uint8)
            actual = grid_hash(effective)
            expected = str(expected_master_hash or self.meta["expected_master_hash"])
            if effective.shape != shape or actual != expected or actual != self.meta["expected_master_hash"]:
                raise ValueError("verified effective-master override hash mismatch")
            full["master"] = effective
        rows, cols = np.where(full["allowed"].astype(bool))
        if not len(rows):
            raise ValueError("empty route-phase corridor")
        margin = 20
        r0, r1 = max(0, int(rows.min())-margin), min(shape[0], int(rows.max())+margin+1)
        c0, c1 = max(0, int(cols.min())-margin), min(shape[1], int(cols.max())+margin+1)
        self.grids = {name: value[r0:r1, c0:c1].copy() for name, value in full.items()}
        self._full_occupancy = full["occupancy"]
        self._full_allowed = full["allowed"]
        self.master = self.grids["master"]
        self.obstacle = self.master >= 254
        distance = cv2.distanceTransform(
            (~self.obstacle).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE,
        ) * float(desc["resolution"])
        image = Path(desc["image_path"])
        self.map = CropMap(
            image.with_suffix(".yaml"), image, float(desc["resolution"]),
            tuple(desc["origin"]), c1-c0, r1-r0, self.grids["occupancy"], distance,
        )
        self.map.full_origin, self.map.full_height = tuple(desc["origin"]), shape[0]
        self.map.row0, self.map.col0 = r0, c0
        self.query = SimpleNamespace(**self.meta["query"])
        self.start, self.goal = tuple(self.query.start), tuple(self.query.goal)
        self.selected_lanes = tuple(sorted(map(int, self.meta["selected_lane_labels"])))
        self.selected_parking = tuple(sorted(map(int, self.meta["selected_parking_components"])))
        self.safe_threshold = math.hypot(.265, .225) + math.sqrt(2) * self.map.resolution
        self.pose_checks = self.exact_checks = 0
        self._canonical_auditor = None

    def cells(self, samples):
        samples = np.asarray(samples)
        cols = np.floor((samples[:, 0]-self.map.full_origin[0])/self.map.resolution).astype(np.int64)-self.map.col0
        rows = self.map.full_height-1-np.floor((samples[:, 1]-self.map.full_origin[1])/self.map.resolution).astype(np.int64)-self.map.row0
        inside = (rows >= 0) & (rows < self.map.height) & (cols >= 0) & (cols < self.map.width)
        return rows, cols, inside

    def collision_free(self, samples) -> bool:
        samples = np.asarray(samples, dtype=np.float64)
        self.pose_checks += len(samples)
        rows, cols, inside = self.cells(samples)
        if not np.all(inside):
            return False
        if (
            np.any(self.master[rows, cols] >= 253)
            or np.any(~self.grids["allowed"][rows, cols].astype(bool))
            or np.any(self.grids["hard"][rows, cols].astype(bool))
        ):
            return False
        near = np.where(self.map.distance_m[rows, cols] <= self.safe_threshold)[0]
        half = self.map.resolution / 2.0
        for index in near:
            self.exact_checks += 1
            x, y, yaw = samples[index]
            row, col = rows[index], cols[index]
            span = math.ceil(math.hypot(.265, .225)/self.map.resolution) + 2
            rr, cc = np.where(self.obstacle[
                max(0, row-span):min(self.map.height, row+span+1),
                max(0, col-span):min(self.map.width, col+span+1),
            ])
            if not len(rr):
                continue
            rbase, cbase = max(0, row-span), max(0, col-span)
            rr, cc = rr+rbase, cc+cbase
            dx = self.map.full_origin[0] + (cc+self.map.col0+.5)*self.map.resolution - x
            dy = self.map.full_origin[1] + (self.map.full_height-rr-self.map.row0-.5)*self.map.resolution - y
            c, s = math.cos(yaw), math.sin(yaw)
            intersects = (
                (np.abs(dx) <= .265*abs(c)+.225*abs(s)+half)
                & (np.abs(dy) <= .265*abs(s)+.225*abs(c)+half)
                & (np.abs(c*dx+s*dy) <= .265+half*(abs(c)+abs(s)))
                & (np.abs(-s*dx+c*dy) <= .225+half*(abs(c)+abs(s)))
            )
            if np.any(intersects):
                return False
        return True

    def phase_at_cell(self, cell: tuple[int, int]) -> Phase:
        if bool(self.grids["junction"][cell]):
            return Phase("transfer")
        parking = int(self.grids["parking_components"][cell])
        lane = int(self.grids["labels"][cell])
        if parking in self.selected_parking:
            return Phase("parking", parking)
        if lane in self.selected_lanes:
            return Phase("lane", lane)
        return Phase("transfer")

    def edge_statistics(self, samples) -> tuple[int, int, int, int, int, float]:
        rows, cols, inside = self.cells(samples)
        if not np.all(inside):
            return 0, 0, 0, 0, 0, float("inf")
        lane = np.isin(self.grids["labels"][rows, cols], self.selected_lanes)
        parking = np.isin(self.grids["parking_components"][rows, cols], self.selected_parking)
        lane_error = self.grids["error"][rows, cols]
        parking_error = self.grids["parking_deviation"][rows, cols]
        correct = lane & self.grids["correct"][rows, cols].astype(bool)
        target = correct & (lane_error <= TransitionContractR2().lane_target_error_max_m)
        center = parking & (parking_error <= TransitionContractR2().parking_normalized_deviation_max)
        finite = np.r_[lane_error[lane & np.isfinite(lane_error)], parking_error[parking & np.isfinite(parking_error)]]
        integral = float(np.sum(finite) * SPACING) if len(finite) else 0.0
        return (
            int(lane.sum()), int(correct.sum()), int(target.sum()),
            int(parking.sum()), int(center.sum()), integral,
        )

    def route_phase_conforms(
        self, samples, route: OrientedRoute, phase_lookup: Sequence[Phase], spacing_m: float,
        *, minimum_station_m: float = 0.0, maximum_station_m: float | None = None,
        transition_layer_tolerance: int = 0,
    ) -> bool:
        rows, cols, inside = self.cells(samples)
        if not np.all(inside):
            return False
        station, _distance, _source = route.project_station_window(
            np.asarray(samples)[:, :2], minimum_station_m,
            route.length_m if maximum_station_m is None else maximum_station_m,
        )
        indices = np.clip(np.rint(station / spacing_m).astype(int), 0, len(phase_lookup)-1)
        for row, col, index in zip(rows, cols, indices):
            expected = phase_lookup[int(index)]
            cell = int(row), int(col)
            actual = self.phase_at_cell(cell)
            neighbor_phases = {
                phase_lookup[position]
                for position in range(
                    max(0, int(index)-int(transition_layer_tolerance)),
                    min(len(phase_lookup), int(index)+int(transition_layer_tolerance)+1),
                )
            }
            if actual != expected and actual in neighbor_phases:
                continue
            if expected.kind == "lane":
                if actual != expected and actual.kind != "transfer":
                    return False
                if actual.kind == "transfer" and not bool(self.grids["junction"][cell]):
                    return False
            elif expected.kind == "parking":
                if actual != expected and actual.kind != "transfer":
                    return False
                if actual.kind == "transfer" and not bool(self.grids["base_allowed"][cell]):
                    return False
            elif not bool(self.grids["base_allowed"][cell]):
                return False
        return True

    def audit(
        self, controls, route: OrientedRoute, semantic_map: SemanticMapV1,
        policy: RoutePhasePolicy,
    ) -> dict[str, Any]:
        if not controls:
            return {"gate_passed": False, "failure_codes": ["EMPTY_PATH"]}
        raw = np.vstack((controls[0].start, *(edge.samples for edge in controls)))
        points = [
            {"x": float(x), "y": float(y), "yaw": float(yaw), "source": "2A-V3-route-phase",
             "motion_direction": "forward", "steering": 0.0,
             "planner_backend": METHOD_ID, "backend_version": "r13"}
            for x, y, yaw in raw
        ]
        desc = self.meta["map"]
        if self._canonical_auditor is None:
            raw_distance = cv2.distanceTransform(
                (self._full_occupancy == 0).astype(np.uint8), cv2.DIST_L2,
                cv2.DIST_MASK_PRECISE,
            ) * float(desc["resolution"])
            full_map = HospitalMap(
                self.map.yaml_path, self.map.image_path, float(desc["resolution"]),
                tuple(desc["origin"]), int(desc["width"]), int(desc["height"]),
                self._full_occupancy, raw_distance,
            )
            self._canonical_auditor = PathAuditor(
                SimpleNamespace(hospital_map=full_map), source_commit="2A-V3-r13-bound-source",
            )
        canonical = self._canonical_auditor.audit(self.query, points, self._full_allowed)
        sampled_points, station = resample_path(points)
        sampled = np.asarray([[p[key] for key in ("x", "y", "yaw")] for p in sampled_points])
        rows, cols, inside = self.cells(sampled)
        if not np.all(inside):
            metrics = {"semantic_gate_passed": False, "failure_reason": "PATH_OUTSIDE_INPUT_CROP"}
        else:
            lane = np.isin(self.grids["labels"][rows, cols], self.selected_lanes)
            parking = np.isin(self.grids["parking_components"][rows, cols], self.selected_parking)
            metrics = audit_transition_samples(
                path_length_m=float(station[-1]), station_m=station,
                lane_mask=lane,
                lane_error_m=self.grids["error"][rows, cols],
                lane_correct_side=self.grids["correct"][rows, cols],
                parking_mask=parking,
                parking_normalized_deviation=self.grids["parking_deviation"][rows, cols],
                raw_xy=raw[:, :2],
            )
        dense_parts = []
        station_parts = []
        lateral_parts = []
        source_parts = []
        certified_intervals = all(
            hasattr(edge, "_route_station_interval") for edge in controls
        )
        if certified_intervals:
            for index, edge in enumerate(controls):
                checked = dense_interpolate(np.vstack((edge.start, edge.samples)))
                lower, upper = edge._route_station_interval
                projected = route.project_station_window(
                    checked[:, :2], float(lower)-.25, float(upper)+.25,
                )
                offset = 0 if index == 0 else 1
                dense_parts.append(checked[offset:])
                station_parts.append(projected[0][offset:])
                lateral_parts.append(projected[1][offset:])
                source_parts.append(projected[2][offset:])
            dense = np.vstack(dense_parts)
            certified_projection = (
                np.concatenate(station_parts), np.concatenate(lateral_parts),
                np.concatenate(source_parts),
            )
        else:
            dense = dense_interpolate(raw)
            certified_projection = None
        collision = self.collision_free(dense)
        progress = audit_route_progress(route, dense, projection=certified_projection)
        progress["projection_binding"] = (
            "per_primitive_source_target_station_interval"
            if certified_intervals else "whole_route_nearest_segment_fallback"
        )
        revisit = audit_revisits(raw)
        active_target = np.zeros(len(sampled), dtype=bool)
        if np.all(inside):
            lane = np.isin(self.grids["labels"][rows, cols], self.selected_lanes)
            parking = np.isin(self.grids["parking_components"][rows, cols], self.selected_parking)
            active_target = (
                (lane & self.grids["correct"][rows, cols].astype(bool)
                 & (self.grids["error"][rows, cols] <= .50))
                | (parking & (self.grids["parking_deviation"][rows, cols] <= .25))
            )
        contract = TransitionContractR2()
        if len(station) and float(station[-1]) > contract.short_path_max_m:
            active_target &= (
                (station >= contract.endpoint_transition_each_m)
                & (station <= float(station[-1])-contract.endpoint_transition_each_m)
            )
        necessity = audit_path_necessity(
            sampled, start=self.start, goal=self.goal, target_mask=active_target,
            route_polyline=route.points,
        )
        hard = explicit_hard_audit(self, points, semantic_map)
        exact = bool(np.array_equal(raw[0], self.start) and np.array_equal(raw[-1], self.goal))
        continuity = all(np.array_equal(a.samples[-1], b.start) for a, b in zip(controls, controls[1:]))
        replay = all(np.array_equal(replay_edge(edge.certificate()).samples, edge.samples) for edge in controls)
        max_curvature = max(
            (1.0/edge.radius if any(k != "S" and p > 1e-10 for k, p in zip(edge.word, edge.params)) else 0.0)
            for edge in controls
        )
        arc_length = float(sum(edge.length for edge in controls))
        length_bound = (
            route.length_m * policy.path_length_route_ratio_max
            + policy.path_length_route_slack_m
        )
        required = {
            "CANONICAL_FINAL_INVALID": bool(canonical.final_valid_success),
            "PADDED_MASTER_COLLISION": collision,
            "ENDPOINT_MISMATCH": exact,
            "EDGE_DISCONTINUITY": continuity,
            "CONTROL_REPLAY_MISMATCH": replay,
            "CURVATURE_LIMIT": max_curvature <= 2.50,
            "PATH_LENGTH_BOUND": arc_length <= length_bound,
            "HARD_FEATURE_GATE": bool(hard["hard_feature_gate_passed"]),
            "R2_SEMANTIC_GATE": bool(metrics.get("semantic_gate_passed")),
            "GEOMETRIC_REVISIT": bool(revisit["revisit_screen_passed"]),
            # A terminal-tangent half-plane is only meaningful for a nearly
            # straight endpoint approach.  On a winding frozen L1 route it can
            # label earlier, strictly forward route stations as "backward".
            # The route-station audit is the fail-closed invariant that caught
            # the historical pass-goal-and-return path; combine it with the
            # independent revisit screen and retain the terminal-plane result
            # as a diagnostic rather than rejecting legitimate route turns.
            "NATURALNESS_SCREEN": bool(
                progress["ordered_progress_gate_passed"]
                and revisit["revisit_screen_passed"]
            ),
            "ORDERED_PROGRESS": bool(progress["ordered_progress_gate_passed"]),
        }
        failures = [code for code, passed in required.items() if not passed]
        return {
            "gate_passed": not failures,
            "failure_codes": failures,
            "points": points,
            "controls": [
                {
                    **edge.certificate(),
                    "route_station_interval_m": list(
                        map(float, edge._route_station_interval)
                    ) if hasattr(edge, "_route_station_interval") else None,
                }
                for edge in controls
            ],
            "canonical": canonical.metrics,
            "canonical_diagnostics": canonical.diagnostics(),
            "semantics": metrics,
            "hard_features": hard,
            "revisit": revisit,
            "naturalness": necessity,
            "naturalness_gate_basis": "global_oriented_route_station_monotonicity_plus_revisit_screen",
            "ordered_progress": progress,
            "padded_effective_master_collision_free": collision,
            "exact_endpoint_xy_yaw": exact,
            "edge_continuity": continuity,
            "trace_replay_exact": replay,
            "maximum_control_curvature_1pm": max_curvature,
            "arc_length_m": arc_length,
            "path_length_bound_m": length_bound,
            "pose_checks": self.pose_checks,
            "exact_rectangle_checks": self.exact_checks,
            "path_sha256": hashlib.sha256(np.ascontiguousarray(raw).tobytes()).hexdigest(),
        }


def _phase_near(world: RoutePhaseWorld, x: float, y: float, radius_m: float) -> Phase:
    cell = world.map.world_to_cell(x, y)
    if cell is None:
        return Phase("transfer")
    direct = world.phase_at_cell(cell)
    if direct.kind != "transfer" or bool(world.grids["junction"][cell]):
        return direct
    radius = int(math.ceil(radius_m / world.map.resolution))
    row, col = cell
    r0, r1 = max(0, row-radius), min(world.map.height, row+radius+1)
    c0, c1 = max(0, col-radius), min(world.map.width, col+radius+1)
    candidates = []
    for rr, cc in np.argwhere(world.grids["allowed"][r0:r1, c0:c1].astype(bool)):
        target = int(rr+r0), int(cc+c0)
        phase = world.phase_at_cell(target)
        if phase.kind == "transfer":
            continue
        wx, wy = world.map.cell_to_world(target)
        candidates.append((math.hypot(wx-x, wy-y), phase.kind, phase.instance, phase))
    return min(candidates)[-1] if candidates else direct


def build_phase_layers(
    world: RoutePhaseWorld, route: OrientedRoute, policy: RoutePhasePolicy,
) -> tuple[list[State], list[list[int]], list[Phase], dict[str, Any]]:
    samples = route.stations(policy.station_spacing_m)
    phases = [
        _phase_near(world, sample.x, sample.y, policy.phase_probe_radius_m)
        for sample in samples
    ]
    states: list[State] = []
    layers: list[list[int]] = []

    def append(**kwargs) -> int:
        state_id = len(states)
        states.append(State(state_id=state_id, **kwargs))
        return state_id

    layers.append([append(
        layer_index=0, station_m=0.0, x=float(world.start[0]), y=float(world.start[1]),
        yaw=float(world.start[2]), yaw_bin=round(world.start[2]/YAW_BIN_SIZE) % YAW_BIN_COUNT,
        phase_kind=phases[0].kind, phase_instance=phases[0].instance,
        lateral_offset_m=0.0, semantic_error=float("nan"), semantic_correct=False,
        semantic_target=False, endpoint="start",
    )])
    target_layer_count = 0
    empty_layers = []
    for layer_index, (sample, phase) in enumerate(zip(samples[1:-1], phases[1:-1]), start=1):
        resolution = world.map.resolution
        radius = int(math.ceil((policy.maximum_lateral_probe_m+policy.station_slab_half_width_m)/resolution))+1
        center = world.map.world_to_cell(sample.x, sample.y)
        candidates = []
        if center is not None:
            r0, r1 = max(0, center[0]-radius), min(world.map.height, center[0]+radius+1)
            c0, c1 = max(0, center[1]-radius), min(world.map.width, center[1]+radius+1)
            rr, cc = np.mgrid[r0:r1, c0:c1]
            x = world.map.full_origin[0]+(cc+world.map.col0+.5)*resolution
            y = world.map.full_origin[1]+(world.map.full_height-rr-world.map.row0-.5)*resolution
            dx, dy = x-sample.x, y-sample.y
            longitudinal = dx*sample.tangent_x+dy*sample.tangent_y
            lateral = dx*sample.tangent_y-dy*sample.tangent_x
            mask = (
                (np.abs(longitudinal) <= policy.station_slab_half_width_m+1e-12)
                & (np.abs(lateral) <= policy.maximum_lateral_probe_m+1e-12)
                & world.grids["allowed"][rr, cc].astype(bool)
                & ~world.grids["hard"][rr, cc].astype(bool)
                & (world.master[rr, cc] < 253)
            )
            if phase.kind == "lane":
                mask &= world.grids["labels"][rr, cc] == phase.instance
            elif phase.kind == "parking":
                mask &= world.grids["parking_components"][rr, cc] == phase.instance
            else:
                mask &= world.grids["base_allowed"][rr, cc].astype(bool)
            for row, col in zip(rr[mask], cc[mask]):
                cell = int(row), int(col)
                wx, wy = world.map.cell_to_world(cell)
                offset = (wx-sample.x)*sample.tangent_y-(wy-sample.y)*sample.tangent_x
                if phase.kind == "lane":
                    error = float(world.grids["error"][cell])
                    correct = bool(world.grids["correct"][cell])
                    target = bool(correct and math.isfinite(error) and error <= .50)
                elif phase.kind == "parking":
                    error = float(world.grids["parking_deviation"][cell])
                    correct = bool(math.isfinite(error))
                    target = bool(correct and error <= .25)
                else:
                    error, correct, target = 0.0, True, True
                if not math.isfinite(error):
                    continue
                candidates.append({
                    "x": wx, "y": wy, "row": cell[0], "col": cell[1],
                    "offset": float(offset), "error": error,
                    "correct": correct, "target": target,
                    "clearance": float(world.map.distance_m[cell]),
                })
        selected = []
        if candidates:
            semantic = min(candidates, key=lambda item: (
                not item["target"], not item["correct"], item["error"],
                -item["clearance"], abs(item["offset"]), item["row"], item["col"],
            ))
            anchors = [0.0, semantic["offset"]]
            anchors.extend(np.linspace(0.0, semantic["offset"], policy.maximum_cells_per_station).tolist())
            seen = set()
            for desired in anchors:
                item = min(candidates, key=lambda value: (
                    abs(value["offset"]-desired), not value["target"],
                    value["error"], -value["clearance"], value["row"], value["col"],
                ))
                key = item["row"], item["col"]
                if key not in seen:
                    selected.append(item)
                    seen.add(key)
                if len(selected) >= policy.maximum_cells_per_station:
                    break
            for item in sorted(candidates, key=lambda value: (
                not value["target"], not value["correct"], value["error"],
                -value["clearance"], abs(value["offset"]), value["row"], value["col"],
            )):
                key = item["row"], item["col"]
                if key not in seen:
                    selected.append(item)
                    seen.add(key)
                if len(selected) >= policy.maximum_cells_per_station:
                    break
        layer = []
        base_bin = round(math.atan2(sample.tangent_y, sample.tangent_x)/YAW_BIN_SIZE) % YAW_BIN_COUNT
        for item in selected:
            for yaw_offset in sorted(set(policy.yaw_neighbor_bins)):
                yaw_bin = (base_bin+yaw_offset) % YAW_BIN_COUNT
                yaw = wrap_angle(yaw_bin*YAW_BIN_SIZE)
                pose = np.asarray([[item["x"], item["y"], yaw]])
                if not world.collision_free(pose):
                    continue
                layer.append(append(
                    layer_index=layer_index, station_m=sample.station_m,
                    x=item["x"], y=item["y"], yaw=yaw, yaw_bin=yaw_bin,
                    phase_kind=phase.kind, phase_instance=phase.instance,
                    lateral_offset_m=item["offset"], semantic_error=item["error"],
                    semantic_correct=item["correct"], semantic_target=item["target"],
                ))
        layer.sort(key=lambda state_id: (
            states[state_id].lateral_offset_m, states[state_id].yaw_bin,
            states[state_id].x, states[state_id].y,
        ))
        if not layer:
            empty_layers.append(layer_index)
        if any(states[state_id].semantic_target for state_id in layer):
            target_layer_count += 1
        layers.append(layer)
    goal_index = len(layers)
    layers.append([append(
        layer_index=goal_index, station_m=route.length_m,
        x=float(world.goal[0]), y=float(world.goal[1]), yaw=float(world.goal[2]),
        yaw_bin=round(world.goal[2]/YAW_BIN_SIZE) % YAW_BIN_COUNT,
        phase_kind=phases[-1].kind, phase_instance=phases[-1].instance,
        lateral_offset_m=0.0, semantic_error=float("nan"), semantic_correct=False,
        semantic_target=False, endpoint="goal",
    )])
    runs = []
    for index, phase in enumerate(phases):
        if not runs or runs[-1]["kind"] != phase.kind or runs[-1]["instance"] != phase.instance:
            runs.append({"kind": phase.kind, "instance": phase.instance, "start_layer": index, "end_layer": index})
        else:
            runs[-1]["end_layer"] = index
    diagnostics = {
        "state_count": len(states), "layer_count": len(layers),
        "empty_interior_layers": empty_layers,
        "target_candidate_layer_count": target_layer_count,
        "yaw_bin_count": YAW_BIN_COUNT,
        "phase_runs": runs,
        "selected_lane_labels": list(world.selected_lanes),
        "selected_parking_components": list(world.selected_parking),
    }
    return states, layers, phases, diagnostics


@dataclass(frozen=True)
class Label:
    label_id: int
    node: int
    score: float
    length_m: float
    lane_wrong_m: float
    lane_outside_m: float
    parking_outside_m: float
    error_integral: float
    parent: int
    edge: Edge | None
    signature: tuple[int, ...]


class LazyRoutePhaseSearch:
    def __init__(self, world: RoutePhaseWorld, route: OrientedRoute, policy: RoutePhasePolicy):
        self.world, self.route, self.policy = world, route, policy
        self.states, self.layers, self.phases, self.layer_diagnostics = build_phase_layers(world, route, policy)
        self.cache: dict[int, tuple[Edge, ...]] = {}
        self.counters = Counter()
        self.rejected = Counter()
        self.best_failed: dict[str, Any] | None = None

    def reachability_certificate(self) -> dict[str, Any]:
        """Classify target states in the exact finite planner graph.

        This is positive finite-graph evidence only.  An empty intersection is
        reported as not applicable *in this bounded state lattice* and never as
        a continuous-space impossibility proof.
        """
        start, goal = self.layers[0][0], self.layers[-1][0]
        reachable = {start}
        pending = [start]
        incoming: dict[int, list[int]] = {state.state_id: [] for state in self.states}
        while pending:
            source = pending.pop()
            for edge in self.successors(source):
                incoming[edge.target].append(source)
                if edge.target not in reachable:
                    reachable.add(edge.target)
                    pending.append(edge.target)
        coreachable = {goal}
        pending = [goal]
        while pending:
            target = pending.pop()
            for source in sorted(set(incoming[target])):
                if source not in coreachable:
                    coreachable.add(source)
                    pending.append(source)
        records = []
        keys = [*(('lane', value) for value in self.world.selected_lanes),
                *(('parking', value) for value in self.world.selected_parking)]
        for kind, instance in keys:
            target_states = [
                state for state in self.states
                if state.phase_kind == kind and state.phase_instance == instance
                and state.semantic_target and state.endpoint == ""
            ]
            both = [
                state for state in target_states
                if state.state_id in reachable and state.state_id in coreachable
            ]
            records.append({
                "semantic_class": kind,
                "instance_id": int(instance),
                "target_state_count": len(target_states),
                "start_reachable_target_state_count": sum(
                    state.state_id in reachable for state in target_states
                ),
                "goal_coreachable_target_state_count": sum(
                    state.state_id in coreachable for state in target_states
                ),
                "reachable_coreachable_target_state_count": len(both),
                "applicability_status": (
                    "APPLICABLE" if both else "NOT_APPLICABLE_IN_BOUND_STATE_LATTICE"
                ),
                "counts_as_semantic_success": False,
            })
        applicable_classes = sorted({
            record["semantic_class"] for record in records
            if record["applicability_status"] == "APPLICABLE"
        })
        return {
            "schema_version": "2A-V3-route-phase-applicability-v1",
            "finite_graph_scope": True,
            "continuous_space_infeasibility_proof": False,
            "goal_reachable": goal in reachable,
            "reachable_state_count": len(reachable),
            "goal_coreachable_state_count": len(coreachable),
            "applicable_semantic_classes": applicable_classes,
            "records": records,
        }

    def _edge(self, source: State, target: State, choice: int, word: str, params) -> Edge | None:
        control = dubins_edge_from_parameters(
            source.pose, target.pose, self.policy.turning_radius_m, word, params,
        )
        control._route_station_interval = (source.station_m, target.station_m)
        dense = dense_interpolate(np.vstack((control.start, control.samples)))
        station, _, _ = self.route.project_station_window(
            dense[:, :2], source.station_m-.25, target.station_m+.25,
        )
        if (
            np.any(np.diff(station) < -self.policy.projection_epsilon_m)
            or float(np.min(station)) < source.station_m-.251
            or float(np.max(station)) > target.station_m+.251
        ):
            self.rejected["NON_MONOTONE_EDGE"] += 1
            return None
        if not self.world.route_phase_conforms(
            control.samples, self.route, self.phases, self.policy.station_spacing_m,
            minimum_station_m=source.station_m-.25,
            maximum_station_m=target.station_m+.25,
            transition_layer_tolerance=self.policy.phase_transition_layer_tolerance,
        ):
            self.rejected["SEMANTIC_PHASE_ESCAPE"] += 1
            return None
        if not self.world.collision_free(dense):
            self.rejected["FULL_FOOTPRINT_OR_HARD_EDGE"] += 1
            return None
        ln, lc, lt, pn, pc, error = self.world.edge_statistics(control.samples)
        canonical_id = ((source.state_id*len(self.states)+target.state_id)*6+choice)
        return Edge(
            canonical_id, source.state_id, target.state_id,
            source.layer_index, target.layer_index, choice, float(control.length),
            ln, lc, lt, pn, pc, error, control,
        )

    def successors(self, source_id: int) -> tuple[Edge, ...]:
        if source_id in self.cache:
            self.counters["cache_hit"] += 1
            return self.cache[source_id]
        source = self.states[source_id]
        result = []
        for target_layer in range(
            source.layer_index+1,
            min(len(self.layers), source.layer_index+1+self.policy.maximum_station_skip),
        ):
            specs = []
            for target_id in self.layers[target_layer]:
                target = self.states[target_id]
                direct = math.dist(source.pose[:2], target.pose[:2])
                if direct <= self.policy.projection_epsilon_m or direct > self.policy.maximum_local_edge_length_m:
                    self.rejected["EDGE_DISTANCE_BOUND"] += 1
                    continue
                bound = min(self.policy.maximum_local_edge_length_m, self.policy.maximum_local_edge_ratio*direct+.05)
                for choice, (word, params) in enumerate(dubins_choices(source.pose, target.pose, self.policy.turning_radius_m)):
                    length = self.policy.turning_radius_m*sum(params)
                    if length <= bound:
                        specs.append((
                            100.0*(not target.semantic_correct)+50.0*(not target.semantic_target)
                            +2.0*target.semantic_error+length,
                            not target.semantic_target, target.semantic_error, length,
                            target.yaw_bin, target.state_id, choice, word, params,
                        ))
                    else:
                        self.rejected["DUBINS_LENGTH_BOUND"] += 1
            valid = 0
            for *_rank, target_id, choice, word, params in sorted(specs):
                self.counters["sampled_state_pairs"] += 1
                edge = self._edge(source, self.states[target_id], choice, word, params)
                if edge is None:
                    continue
                result.append(edge)
                valid += 1
                if valid >= self.policy.valid_successors_per_target_layer:
                    break
        result.sort(key=lambda edge: (edge.target_layer, edge.target, edge.length_m, edge.dubins_choice))
        self.cache[source_id] = tuple(result)
        self.counters["valid_edges"] += len(result)
        return self.cache[source_id]

    def shortest_fallback(self, semantic_map: SemanticMapV1) -> dict[str, Any] | None:
        """Return the deterministic shortest path in the already-built graph."""
        start, goal = self.layers[0][0], self.layers[-1][0]
        distance = {start: 0.0}
        signatures: dict[int, tuple[int, ...]] = {start: ()}
        parents: dict[int, tuple[int, Edge]] = {}
        for layer in self.layers[:-1]:
            for source in sorted(layer):
                if source not in distance:
                    continue
                for edge in self.successors(source):
                    candidate = distance[source] + edge.length_m
                    signature = signatures[source] + (edge.edge_id,)
                    old = (distance.get(edge.target, float("inf")), signatures.get(edge.target, ()))
                    if (candidate, signature) < old:
                        distance[edge.target] = candidate
                        signatures[edge.target] = signature
                        parents[edge.target] = source, edge
        if goal not in distance:
            return None
        controls = []
        cursor = goal
        while cursor != start:
            cursor, edge = parents[cursor]
            controls.append(edge.control)
        controls.reverse()
        audited = self.world.audit(controls, self.route, semantic_map, self.policy)
        audited["fallback_profile"] = "deterministic_shortest_in_bound_route_phase_graph"
        return audited

    def safe_soft_fallback(
        self, semantic_map: SemanticMapV1,
        applicability: Mapping[str, Any],
    ) -> dict[str, Any] | None:
        """Return a safe final-valid path when a soft target is not achieved.

        This never converts a semantic miss into semantic success.  It exists
        because lane/parking preferences are soft: failure to reach their R2
        statistics must not turn a geometrically valid request into NO_PATH.
        Targeted semantic gates do not enable this fallback.
        """
        fallback = self.shortest_fallback(semantic_map)
        if fallback is None:
            return None
        raw_failures = list(fallback.get("failure_codes", []))
        hard_failures = [code for code in raw_failures if code != "R2_SEMANTIC_GATE"]
        if hard_failures:
            return None
        classes = fallback.get("semantics", {}).get("active_window", {}).get("classes", {})
        applicable_classes = set(applicability.get("applicable_semantic_classes", []))
        semantic_passed = bool(
            applicable_classes and all(
                classes.get(kind, {}).get("semantic_gate_passed") is True
                for kind in applicable_classes
            )
        )
        fallback.update({
            "raw_failure_codes": raw_failures,
            "raw_r2_semantic_gate_passed": bool(
                fallback.get("semantics", {}).get("semantic_gate_passed")
            ),
            "query_applicability": dict(applicability),
            "applicable_semantics_passed": semantic_passed,
            "strict_semantic_gate_passed": semantic_passed,
            "semantic_success_counted": semantic_passed,
            "safe_negative_fallback": not applicable_classes,
            "safe_soft_fallback": bool(applicable_classes and not semantic_passed),
            "fallback_reason": (
                "NOT_APPLICABLE_IN_BOUND_STATE_LATTICE"
                if not applicable_classes else "APPLICABLE_R2_SEMANTIC_TARGET_UNSATISFIED"
            ),
            "final_valid_gate_passed": True,
            "gate_passed": True,
            "failure_codes": [],
        })
        return fallback

    def search_dag(
        self, semantic_map: SemanticMapV1, *, allow_safe_soft_fallback: bool = False,
    ) -> tuple[dict[str, Any] | None, list[dict[str, Any]], dict[str, Any]]:
        """Bounded Pareto propagation for routes with multiple semantic phases."""
        applicability = self.reachability_certificate()
        applicable = set(applicability["applicable_semantic_classes"])
        labels: list[Label] = []
        buckets: dict[int, list[int]] = {state.state_id: [] for state in self.states}

        def resources(label: Label):
            return (
                label.lane_wrong_m, label.lane_outside_m,
                label.parking_outside_m, label.error_integral, label.length_m,
            )

        def ordering(label: Label):
            return (
                label.score, label.lane_wrong_m, label.lane_outside_m,
                label.parking_outside_m, label.length_m, label.signature,
            )

        def admit(node, length, wrong, outside, parking, error, parent, edge, signature):
            score = (
                length+self.policy.lane_wrong_weight*wrong
                +self.policy.lane_outside_target_weight*outside
                +self.policy.parking_outside_center_weight*parking
                +self.policy.semantic_error_weight*error
            )
            candidate = Label(
                len(labels), node, score, length, wrong, outside, parking,
                error, parent, edge, signature,
            )
            candidate_resources = resources(candidate)
            bucket = buckets[node]
            if any(
                all(a <= b+1e-12 for a, b in zip(resources(labels[index]), candidate_resources))
                for index in bucket
            ):
                return
            bucket[:] = [
                index for index in bucket
                if not all(a <= b+1e-12 for a, b in zip(candidate_resources, resources(labels[index])))
            ]
            labels.append(candidate)
            bucket.append(candidate.label_id)
            if len(bucket) > self.policy.live_labels_per_state:
                orders = (
                    lambda item: ordering(item),
                    lambda item: (item.lane_wrong_m, item.lane_outside_m,
                                  item.parking_outside_m, item.length_m, item.signature),
                    lambda item: (item.parking_outside_m, item.lane_wrong_m,
                                  item.lane_outside_m, item.length_m, item.signature),
                    lambda item: (item.length_m, item.lane_wrong_m,
                                  item.parking_outside_m, item.signature),
                )
                retained = []
                for order in orders:
                    for index in sorted(bucket, key=lambda value: order(labels[value])):
                        if index not in retained:
                            retained.append(index)
                            break
                for index in sorted(bucket, key=lambda value: ordering(labels[value])):
                    if index not in retained:
                        retained.append(index)
                    if len(retained) >= self.policy.live_labels_per_state:
                        break
                bucket[:] = retained

        start, goal = self.layers[0][0], self.layers[-1][0]
        admit(start, 0.0, 0.0, 0.0, 0.0, 0.0, -1, None, ())
        expanded = 0
        for layer in self.layers[:-1]:
            for node in sorted(layer):
                for label_id in tuple(buckets[node]):
                    label = labels[label_id]
                    expanded += 1
                    for edge in self.successors(node):
                        admit(
                            edge.target, label.length_m+edge.length_m,
                            label.lane_wrong_m+edge.lane_wrong_m,
                            label.lane_outside_m+edge.lane_outside_m,
                            label.parking_outside_m+edge.parking_outside_m,
                            label.error_integral+edge.semantic_error_integral,
                            label_id, edge, label.signature+(edge.edge_id,),
                        )
        complete = sorted((labels[index] for index in buckets[goal]), key=ordering)
        evaluations = []
        witness = None
        best_failed = None
        for label in complete:
            controls = []
            cursor = label
            while cursor.parent >= 0:
                controls.append(cursor.edge.control)
                cursor = labels[cursor.parent]
            controls.reverse()
            audited = self.world.audit(controls, self.route, semantic_map, self.policy)
            raw_failures = list(audited["failure_codes"])
            hard_failures = [value for value in raw_failures if value != "R2_SEMANTIC_GATE"]
            classes = audited.get("semantics", {}).get("active_window", {}).get("classes", {})
            semantic_pass = bool(
                applicable and all(
                    classes.get(kind, {}).get("semantic_gate_passed") is True
                    for kind in applicable
                )
            )
            gate = bool(not hard_failures and (semantic_pass or not applicable))
            audited.update({
                "raw_failure_codes": raw_failures,
                "raw_r2_semantic_gate_passed": bool(audited["semantics"]["semantic_gate_passed"]),
                "query_applicability": applicability,
                "applicable_semantics_passed": semantic_pass,
                "semantic_success_counted": semantic_pass,
                "safe_negative_fallback": bool(gate and not applicable),
                "gate_passed": gate,
                "failure_codes": (
                    [] if gate else hard_failures+(
                        ["APPLICABLE_R2_SEMANTIC_GATE"] if applicable and not semantic_pass else []
                    )
                ),
            })
            evaluations.append({
                "rank": len(evaluations), "profile": "bounded_pareto_dag",
                "score": label.score, "path_length_m": label.length_m,
                "lane": classes.get("lane"), "parking": classes.get("parking"),
                "raw_failure_codes": raw_failures,
                "failure_codes": audited["failure_codes"], "gate_passed": gate,
                "applicable_semantic_classes": sorted(applicable),
                "semantic_success_counted": semantic_pass,
                "safe_negative_fallback": audited["safe_negative_fallback"],
            })
            if gate:
                witness = audited
                break
            if best_failed is None:
                best_failed = audited
        self.best_failed = best_failed
        if witness is None and allow_safe_soft_fallback:
            fallback = self.safe_soft_fallback(semantic_map, applicability)
            if fallback is not None:
                classes = fallback.get("semantics", {}).get("active_window", {}).get("classes", {})
                evaluations.append({
                    "rank": len(evaluations),
                    "profile": fallback["fallback_profile"],
                    "path_length_m": fallback.get("arc_length_m"),
                    "lane": classes.get("lane"), "parking": classes.get("parking"),
                    "raw_failure_codes": fallback["raw_failure_codes"],
                    "failure_codes": [], "gate_passed": True,
                    "applicable_semantic_classes": sorted(applicable),
                    "semantic_success_counted": fallback["semantic_success_counted"],
                    "safe_negative_fallback": fallback["safe_negative_fallback"],
                    "safe_soft_fallback": fallback["safe_soft_fallback"],
                    "fallback_reason": fallback["fallback_reason"],
                })
                witness = fallback
        diagnostics = {
            **self.layer_diagnostics,
            "search_mode": "bounded_pareto_topological_dag",
            "query_applicability": applicability,
            "expanded_label_count": expanded,
            "generated_label_count": len(labels),
            "active_label_count": sum(map(len, buckets.values())),
            "goal_candidate_count": len(complete),
            "remaining_heap_count": 0,
            "maximum_expanded_layer": len(self.layers)-2,
            "resource_limit_reached": False,
            "materialized_edge_count": int(self.counters["valid_edges"]),
            "sampled_state_pair_count": int(self.counters["sampled_state_pairs"]),
            "edge_rejection_counts": dict(sorted(self.rejected.items())),
        }
        return witness, evaluations, diagnostics

    def search(
        self, semantic_map: SemanticMapV1, *, allow_safe_soft_fallback: bool = False,
        prefer_lazy: bool = False,
        fallback_first: bool = False,
    ) -> tuple[dict[str, Any] | None, list[dict[str, Any]], dict[str, Any]]:
        if fallback_first and not allow_safe_soft_fallback:
            raise ValueError("fallback-first requires explicit safe-soft-fallback policy")
        if (
            self.world.selected_parking or len(self.world.selected_lanes) > 1
        ) and not prefer_lazy:
            return self.search_dag(
                semantic_map, allow_safe_soft_fallback=allow_safe_soft_fallback,
            )
        labels: list[Label] = []
        active = {state.state_id: [] for state in self.states}
        heap = []
        evaluations = []
        goal_audits: list[dict[str, Any]] = []

        def strict_path_applicability(classes: Mapping[str, Any]) -> dict[str, Any]:
            applicable = sorted(
                kind for kind in ("lane", "parking")
                if classes.get(kind, {}).get("semantic_gate_passed") is True
            )
            return {
                "schema_version": "2A-V3-route-phase-applicability-v1",
                "finite_graph_scope": True,
                "continuous_space_infeasibility_proof": False,
                "evidence": "STRICT_REQUEST_GENERATED_PATH",
                "goal_reachable": True,
                "applicable_semantic_classes": applicable,
                "records": [],
            }

        def decorate(audited: dict[str, Any], applicability: Mapping[str, Any]) -> bool:
            classes = audited.get("semantics", {}).get("active_window", {}).get("classes", {})
            applicable_classes = set(applicability["applicable_semantic_classes"])
            raw_failures = list(audited["failure_codes"])
            hard_failures = [code for code in raw_failures if code != "R2_SEMANTIC_GATE"]
            semantic_passed = bool(
                applicable_classes
                and all(
                    classes.get(kind, {}).get("semantic_gate_passed") is True
                    for kind in applicable_classes
                )
            )
            gate = bool(not hard_failures and (semantic_passed or not applicable_classes))
            audited.update({
                "raw_failure_codes": raw_failures,
                "raw_r2_semantic_gate_passed": bool(
                    audited.get("semantics", {}).get("semantic_gate_passed")
                ),
                "query_applicability": dict(applicability),
                "applicable_semantics_passed": semantic_passed,
                "semantic_success_counted": semantic_passed,
                "safe_negative_fallback": bool(gate and not applicable_classes),
                "gate_passed": gate,
                "failure_codes": (
                    [] if gate else hard_failures + (
                        ["APPLICABLE_R2_SEMANTIC_GATE"]
                        if applicable_classes and not semantic_passed else []
                    )
                ),
            })
            return gate

        def key(label: Label):
            return (label.score, label.length_m, label.lane_wrong_m, label.lane_outside_m,
                    label.parking_outside_m, label.signature)

        def add(node, length, wrong, outside, parking, error, parent, edge, signature):
            score = (
                length
                + self.policy.lane_wrong_weight*wrong
                + self.policy.lane_outside_target_weight*outside
                + self.policy.parking_outside_center_weight*parking
                + self.policy.semantic_error_weight*error
            )
            label = Label(len(labels), node, score, length, wrong, outside, parking, error, parent, edge, signature)
            labels.append(label)
            bucket = active[node]
            resources = (wrong, outside, parking, error, length)
            for index in tuple(bucket):
                other = labels[index]
                current = (
                    other.lane_wrong_m, other.lane_outside_m,
                    other.parking_outside_m, other.error_integral, other.length_m,
                )
                if all(a <= b+1e-12 for a, b in zip(current, resources)):
                    self.counters["dominated"] += 1
                    return
            bucket[:] = [
                index for index in bucket
                if not all(
                    a <= b+1e-12 for a, b in zip(
                        resources,
                        (
                            labels[index].lane_wrong_m, labels[index].lane_outside_m,
                            labels[index].parking_outside_m, labels[index].error_integral,
                            labels[index].length_m,
                        ),
                    )
                )
            ]
            bucket.append(label.label_id)
            if len(bucket) > self.policy.live_labels_per_state:
                orders = (
                    lambda item: key(item),
                    lambda item: (item.lane_wrong_m, item.lane_outside_m, item.parking_outside_m,
                                  item.length_m, item.error_integral, item.signature),
                    lambda item: (item.parking_outside_m, item.lane_wrong_m, item.lane_outside_m,
                                  item.length_m, item.error_integral, item.signature),
                    lambda item: (item.length_m, item.lane_wrong_m, item.parking_outside_m,
                                  item.lane_outside_m, item.signature),
                )
                retained = []
                for order in orders:
                    for index in sorted(bucket, key=lambda value: order(labels[value])):
                        if index not in retained:
                            retained.append(index)
                            break
                for index in sorted(bucket, key=lambda value: key(labels[value])):
                    if index not in retained:
                        retained.append(index)
                    if len(retained) >= self.policy.live_labels_per_state:
                        break
                bucket[:] = retained
            if label.label_id not in bucket:
                self.counters["dominated"] += 1
                return
            remaining = max(0.0, self.route.length_m-self.states[node].station_m)
            if fallback_first:
                # The caller has an independently frozen strict-calibration
                # miss and requests only a safe final-valid soft fallback.
                # Prefer the shortest forward candidate at each furthest
                # route station; semantic metrics are still audited and are
                # never upgraded to success.
                priority = (-self.states[node].station_m, length, score)
            elif prefer_lazy:
                # Feasibility-first mode advances the directed route layer
                # before comparing semantic resource totals.  Successor
                # ordering still prefers target states and the unchanged final
                # audit decides acceptance; this only avoids exhausting the
                # bounded queue on early-layer Pareto variants.
                priority = (-self.states[node].station_m, score)
            else:
                progress_weight = (
                    self.policy.short_route_progress_priority_per_m
                    if self.route.length_m <= self.policy.short_route_max_m
                    else self.policy.progress_priority_per_m
                )
                priority = (score+progress_weight*remaining, score)
            heapq.heappush(heap, (*priority, *key(label), label.label_id))

        start, goal = self.layers[0][0], self.layers[-1][0]
        add(start, 0.0, 0.0, 0.0, 0.0, 0.0, -1, None, ())
        witness = None
        while heap and self.counters["expanded"] < self.policy.maximum_expanded_labels:
            *_priority, label_id = heapq.heappop(heap)
            label = labels[label_id]
            if label_id not in active[label.node]:
                self.counters["stale"] += 1
                continue
            if label.node == goal:
                controls = []
                cursor = label
                while cursor.parent >= 0:
                    controls.append(cursor.edge.control)
                    cursor = labels[cursor.parent]
                controls.reverse()
                audited = self.world.audit(controls, self.route, semantic_map, self.policy)
                classes = audited.get("semantics", {}).get("active_window", {}).get("classes", {})
                raw_gate = bool(audited["gate_passed"])
                goal_audits.append(audited)
                evaluations.append({
                    "rank": len(evaluations), "score": label.score,
                    "path_length_m": label.length_m,
                    "lane": classes.get("lane"), "parking": classes.get("parking"),
                    "raw_failure_codes": list(audited["failure_codes"]),
                    "raw_r2_gate_passed": raw_gate,
                })
                if raw_gate:
                    applicability = strict_path_applicability(classes)
                    decorate(audited, applicability)
                    evaluations[-1].update({
                        "failure_codes": [], "gate_passed": True,
                        "applicable_semantic_classes": applicability["applicable_semantic_classes"],
                        "semantic_success_counted": True, "safe_negative_fallback": False,
                    })
                    witness = audited
                    break
                raw_hard_failures = [
                    code for code in audited["failure_codes"]
                    if code != "R2_SEMANTIC_GATE"
                ]
                if allow_safe_soft_fallback and not raw_hard_failures:
                    selected_classes = sorted({
                        *(["lane"] if self.world.selected_lanes else []),
                        *(["parking"] if self.world.selected_parking else []),
                    })
                    online_applicability = {
                        "schema_version": "2A-V3-route-phase-applicability-v1",
                        "finite_graph_scope": True,
                        "continuous_space_infeasibility_proof": False,
                        "evidence": "NOT_EVALUATED_DURING_FAST_SAFE_FALLBACK",
                        "applicable_semantic_classes": selected_classes,
                        "records": [],
                    }
                    audited.update({
                        "raw_failure_codes": list(audited["failure_codes"]),
                        "raw_r2_semantic_gate_passed": False,
                        "query_applicability": online_applicability,
                        "applicable_semantics_passed": False,
                        "strict_semantic_gate_passed": False,
                        "semantic_success_counted": False,
                        "safe_negative_fallback": False,
                        "safe_soft_fallback": True,
                        "fallback_reason": "R2_SEMANTIC_TARGET_UNSATISFIED_FAST_SAFE_FALLBACK",
                        "final_valid_gate_passed": True,
                        "gate_passed": True,
                        "failure_codes": [],
                    })
                    evaluations[-1].update({
                        "failure_codes": [], "gate_passed": True,
                        "applicable_semantic_classes": selected_classes,
                        "semantic_success_counted": False,
                        "safe_negative_fallback": False,
                        "safe_soft_fallback": True,
                        "fallback_reason": audited["fallback_reason"],
                    })
                    witness = audited
                    break
                if len(evaluations) >= self.policy.maximum_goal_candidates:
                    break
                continue
            self.counters["expanded"] += 1
            self.counters["maximum_expanded_layer"] = max(
                int(self.counters["maximum_expanded_layer"]),
                int(self.states[label.node].layer_index),
            )
            for edge in self.successors(label.node):
                add(
                    edge.target, label.length_m+edge.length_m,
                    label.lane_wrong_m+edge.lane_wrong_m,
                    label.lane_outside_m+edge.lane_outside_m,
                    label.parking_outside_m+edge.parking_outside_m,
                    label.error_integral+edge.semantic_error_integral,
                    label.label_id, edge, label.signature+(edge.edge_id,),
                )
        if witness is None:
            applicability = self.reachability_certificate()
            for evaluated, audited in zip(evaluations, goal_audits):
                accepted = decorate(audited, applicability)
                evaluated.update({
                    "failure_codes": list(audited["failure_codes"]),
                    "gate_passed": accepted,
                    "applicable_semantic_classes": list(applicability["applicable_semantic_classes"]),
                    "semantic_success_counted": audited["semantic_success_counted"],
                    "safe_negative_fallback": audited["safe_negative_fallback"],
                })
                if accepted and witness is None:
                    witness = audited
                elif self.best_failed is None:
                    self.best_failed = audited
            if not applicability["applicable_semantic_classes"]:
                fallback = self.shortest_fallback(semantic_map)
                if fallback is not None:
                    accepted = decorate(fallback, applicability)
                    classes = fallback.get("semantics", {}).get("active_window", {}).get("classes", {})
                    evaluations.append({
                        "rank": len(evaluations),
                        "profile": "not_applicable_shortest_fallback",
                        "path_length_m": fallback.get("arc_length_m"),
                        "lane": classes.get("lane"), "parking": classes.get("parking"),
                        "raw_failure_codes": fallback["raw_failure_codes"],
                        "failure_codes": fallback["failure_codes"],
                        "gate_passed": accepted,
                        "applicable_semantic_classes": [],
                        "semantic_success_counted": False,
                        "safe_negative_fallback": fallback["safe_negative_fallback"],
                    })
                    if accepted:
                        witness = fallback
                    elif self.best_failed is None:
                        self.best_failed = fallback
            elif witness is None and allow_safe_soft_fallback:
                fallback = self.safe_soft_fallback(semantic_map, applicability)
                if fallback is not None:
                    classes = fallback.get("semantics", {}).get("active_window", {}).get("classes", {})
                    evaluations.append({
                        "rank": len(evaluations),
                        "profile": fallback["fallback_profile"],
                        "path_length_m": fallback.get("arc_length_m"),
                        "lane": classes.get("lane"), "parking": classes.get("parking"),
                        "raw_failure_codes": fallback["raw_failure_codes"],
                        "failure_codes": [], "gate_passed": True,
                        "applicable_semantic_classes": list(
                            applicability["applicable_semantic_classes"]
                        ),
                        "semantic_success_counted": fallback["semantic_success_counted"],
                        "safe_negative_fallback": False,
                        "safe_soft_fallback": True,
                        "fallback_reason": fallback["fallback_reason"],
                    })
                    witness = fallback
        else:
            applicability = witness["query_applicability"]
        diagnostics = {
            **self.layer_diagnostics,
            "query_applicability": applicability,
            "expanded_label_count": int(self.counters["expanded"]),
            "generated_label_count": len(labels),
            "active_label_count": sum(map(len, active.values())),
            "goal_candidate_count": len(evaluations),
            "remaining_heap_count": len(heap),
            "maximum_expanded_layer": int(self.counters["maximum_expanded_layer"]),
            "resource_limit_reached": bool(
                witness is None and self.counters["expanded"] >= self.policy.maximum_expanded_labels
            ),
            "materialized_edge_count": int(self.counters["valid_edges"]),
            "sampled_state_pair_count": int(self.counters["sampled_state_pairs"]),
            "edge_rejection_counts": dict(sorted(self.rejected.items())),
        }
        return witness, evaluations, diagnostics


def policy_dict(policy: RoutePhasePolicy) -> dict[str, Any]:
    value = asdict(policy)
    value["yaw_neighbor_bins"] = list(policy.yaw_neighbor_bins)
    return value


__all__ = [
    "METHOD_ID", "RoutePhasePolicy", "RoutePhaseWorld", "LazyRoutePhaseSearch",
    "Phase", "State", "Edge", "build_phase_layers", "policy_dict",
]
