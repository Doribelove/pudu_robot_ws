"""Planner-independent parking metric applicability for 2A-V3 r16.

The approved R2 parking metric is intentionally unchanged.  Its deviation is
normalised by the maximum clearance of an entire connected parking semantic
component.  A single semantic component may, however, contain several aisles
with different widths.  A target cell in a wider neighbouring aisle must not
make the target appear available in the route-local aisle when a wall or
non-traversable strip separates the two.

This module evaluates a necessary condition before planning: at each active
route station, it finds the traversable cross-section component attached to
the frozen L1 route and asks whether that local component contains a
full-footprint-safe cell satisfying the unchanged R2 target.  If no more than
half of the active parking stations have such a cell, the semantic request is
not applicable to this frozen route geometry and must use the safe E0 Smac
fallback.  This is an input-geometry classification; it never treats a failed
planner run as evidence of inapplicability.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Mapping, Sequence

import cv2
import numpy as np

from .semantic_map import canonical_hash
from .semantic_route_phase_v3 import OrientedRoute, RoutePhaseWorld, _phase_near
from .semantic_transition_contract import TransitionContractR2


SCHEMA_VERSION = "PLN-02-2A-V3-R16-PARKING-APPLICABILITY-V1"


@dataclass(frozen=True)
class ParkingApplicabilityPolicyR16:
    station_spacing_m: float = 0.25
    station_slab_half_width_m: float = 0.20
    maximum_lateral_probe_m: float = 6.0
    phase_probe_radius_m: float = 0.50
    target_deviation_max: float = 0.25
    required_target_station_ratio_exclusive: float = 0.50

    def __post_init__(self) -> None:
        if any(
            value <= 0.0
            for value in (
                self.station_spacing_m,
                self.station_slab_half_width_m,
                self.maximum_lateral_probe_m,
                self.phase_probe_radius_m,
            )
        ):
            raise ValueError("parking applicability distances must be positive")
        if not 0.0 < self.target_deviation_max < 1.0:
            raise ValueError("parking target deviation must be in (0,1)")
        if not 0.0 < self.required_target_station_ratio_exclusive < 1.0:
            raise ValueError("parking applicability ratio must be in (0,1)")


def _station_runs(stations: Sequence[float], spacing_m: float) -> list[list[float]]:
    runs: list[list[float]] = []
    for station in stations:
        value = float(station)
        if not runs or value - runs[-1][1] > float(spacing_m) * 1.25:
            runs.append([value, value])
        else:
            runs[-1][1] = value
    return runs


def summarize_station_records(
    records: Sequence[Mapping[str, Any]],
    policy: ParkingApplicabilityPolicyR16 | None = None,
) -> dict[str, Any]:
    """Classify already measured route-local cross-section records."""

    policy = policy or ParkingApplicabilityPolicyR16()
    count = len(records)
    available = sum(record.get("target_available") is True for record in records)
    ratio = available / count if count else None
    applicable = bool(
        count
        and ratio is not None
        and ratio > policy.required_target_station_ratio_exclusive
    )
    unavailable_stations = [
        float(record["station_m"])
        for record in records
        if record.get("target_available") is not True
    ]
    best = [
        float(record["route_local_best_deviation"])
        for record in records
        if record.get("route_local_best_deviation") is not None
        and math.isfinite(float(record["route_local_best_deviation"]))
    ]
    return {
        "applicable": applicable,
        "classification": (
            "APPLICABLE_ROUTE_LOCAL_R2_TARGET"
            if applicable
            else "NOT_APPLICABLE_ROUTE_LOCAL_R2_TARGET"
        ),
        "dispatch": "E5_SEMANTIC" if applicable else "E0_NATIVE_SMAC_FALLBACK",
        "semantic_success_counted_on_fallback": False,
        "active_parking_station_count": count,
        "target_available_station_count": available,
        "target_available_station_ratio": ratio,
        "required_ratio_exclusive": policy.required_target_station_ratio_exclusive,
        "route_local_best_deviation_p50": (
            float(np.median(np.asarray(best, dtype=np.float64))) if best else None
        ),
        "route_local_best_deviation_min": min(best) if best else None,
        "route_local_best_deviation_max": max(best) if best else None,
        "unavailable_station_runs_m": _station_runs(
            unavailable_stations, policy.station_spacing_m
        ),
        "failure_code": (
            "" if applicable else "ROUTE_LOCAL_TARGET_COVERAGE_BELOW_CONTRACT"
        ),
    }


def audit_parking_metric_applicability(
    world: RoutePhaseWorld,
    route: OrientedRoute,
    policy: ParkingApplicabilityPolicyR16 | None = None,
) -> dict[str, Any]:
    """Audit the unchanged R2 target on route-attached local cross sections."""

    policy = policy or ParkingApplicabilityPolicyR16()
    contract = TransitionContractR2()
    samples = route.stations(policy.station_spacing_m)
    phases = [
        _phase_near(world, sample.x, sample.y, policy.phase_probe_radius_m)
        for sample in samples
    ]
    active_left, active_right = (
        (0.0, route.length_m)
        if route.length_m <= contract.short_path_max_m
        else (
            contract.endpoint_transition_each_m,
            route.length_m - contract.endpoint_transition_each_m,
        )
    )
    records: list[dict[str, Any]] = []
    for sample, phase in zip(samples, phases):
        if (
            phase.kind != "parking"
            or sample.station_m < active_left
            or sample.station_m > active_right
        ):
            continue
        center = world.map.world_to_cell(sample.x, sample.y)
        record: dict[str, Any] = {
            "station_m": float(sample.station_m),
            "phase_instance": int(phase.instance),
            "target_available": False,
            "route_local_best_deviation": None,
            "route_attached_cell_count": 0,
            "failure_code": "EMPTY_ROUTE_LOCAL_CROSS_SECTION",
        }
        if center is None:
            record["failure_code"] = "ROUTE_STATION_OUTSIDE_MAP"
            records.append(record)
            continue

        resolution = float(world.map.resolution)
        radius = int(
            math.ceil(
                (
                    policy.maximum_lateral_probe_m
                    + policy.station_slab_half_width_m
                )
                / resolution
            )
        ) + 1
        row0, row1 = (
            max(0, center[0] - radius),
            min(world.map.height, center[0] + radius + 1),
        )
        col0, col1 = (
            max(0, center[1] - radius),
            min(world.map.width, center[1] + radius + 1),
        )
        rows, cols = np.mgrid[row0:row1, col0:col1]
        x = (
            world.map.full_origin[0]
            + (cols + world.map.col0 + 0.5) * resolution
        )
        y = (
            world.map.full_origin[1]
            + (world.map.full_height - rows - world.map.row0 - 0.5)
            * resolution
        )
        dx, dy = x - sample.x, y - sample.y
        longitudinal = dx * sample.tangent_x + dy * sample.tangent_y
        lateral = dx * sample.tangent_y - dy * sample.tangent_x
        cross_section = (
            (np.abs(longitudinal) <= policy.station_slab_half_width_m + 1.0e-12)
            & (np.abs(lateral) <= policy.maximum_lateral_probe_m + 1.0e-12)
            & world.grids["allowed"][rows, cols].astype(bool)
            & ~world.grids["hard"][rows, cols].astype(bool)
            & (world.master[rows, cols] < 253)
            & (world.grids["parking_components"][rows, cols] == phase.instance)
        )
        components, labels = cv2.connectedComponents(
            cross_section.astype(np.uint8), connectivity=8
        )
        positions = np.argwhere(cross_section)
        if components <= 1 or not len(positions):
            records.append(record)
            continue
        distance = (
            (positions[:, 0] + row0 - center[0]) ** 2
            + (positions[:, 1] + col0 - center[1]) ** 2
        )
        seed = positions[int(np.argmin(distance))]
        seed_label = int(labels[tuple(seed)])
        attached = labels == seed_label
        deviation = np.asarray(
            world.grids["parking_deviation"][row0:row1, col0:col1],
            dtype=np.float32,
        )
        clearance = np.asarray(
            world.map.distance_m[row0:row1, col0:col1], dtype=np.float32
        )
        footprint_safe = attached & (clearance > float(world.safe_threshold))
        finite = footprint_safe & np.isfinite(deviation)
        record["route_attached_cell_count"] = int(np.count_nonzero(attached))
        record["footprint_safe_cell_count"] = int(np.count_nonzero(footprint_safe))
        if np.any(finite):
            best = float(np.min(deviation[finite]))
            target = finite & (deviation <= policy.target_deviation_max)
            record.update(
                {
                    "route_local_best_deviation": best,
                    "target_cell_count": int(np.count_nonzero(target)),
                    "target_available": bool(np.any(target)),
                    "failure_code": (
                        ""
                        if np.any(target)
                        else "NO_R2_TARGET_IN_ROUTE_LOCAL_CROSS_SECTION"
                    ),
                }
            )
        else:
            record["failure_code"] = "NO_FOOTPRINT_SAFE_LOCAL_CELL"
        records.append(record)

    summary = summarize_station_records(records, policy)
    return {
        "schema_version": SCHEMA_VERSION,
        "metric_definition_changed": False,
        "planner_outcome_used_for_classification": False,
        "continuous_space_infeasibility_claimed": False,
        "scope": "frozen_L1_route_local_cross_section_necessary_condition",
        "active_interval_m": [float(active_left), float(active_right)],
        "policy": asdict(policy),
        "summary": summary,
        "station_records": records,
        "binding_sha256": canonical_hash(
            {
                "map_hash": world.meta.get("map_hash"),
                "semantic_map_hash": world.meta.get("semantic_map_hash"),
                "query": world.meta.get("query"),
                "route_hash": route.route_hash,
                "expected_master_hash": world.meta.get("expected_master_hash"),
                "policy": asdict(policy),
                "schema_version": SCHEMA_VERSION,
            }
        ),
    }


__all__ = [
    "SCHEMA_VERSION",
    "ParkingApplicabilityPolicyR16",
    "audit_parking_metric_applicability",
    "summarize_station_records",
]
