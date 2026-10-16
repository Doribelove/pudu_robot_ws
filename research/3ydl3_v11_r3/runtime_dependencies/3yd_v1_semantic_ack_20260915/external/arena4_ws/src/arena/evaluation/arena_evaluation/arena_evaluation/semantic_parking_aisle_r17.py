"""Route-local parking aisle geometry for the 2A-V3 r17 research arm.

R2's component-normalised field is preserved as a compatibility metric.  R17
adds a separate aisle-normalised guide field.  Each frozen route station owns
only the traversable parking cross-section connected to the route seed; cells
behind a wall or in an adjacent wider aisle cannot affect its normaliser.

The field is a guide, not a safety layer.  The downstream 48-bin forward-only
Dubins lattice, effective master, full footprint and canonical audits remain
authoritative.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import math
from typing import Any

import cv2
import numpy as np

from .semantic_map import canonical_hash
from .semantic_route_phase_v3 import OrientedRoute, RoutePhaseWorld, _phase_near
from .semantic_transition_contract import TransitionContractR2


SCHEMA_VERSION = "PLN-02-2A-V3-R17-PARKING-AISLE-FIELD-V1"
METHOD_ID = "route_attached_obstacle_separated_parking_aisle_field_r17_v1"


@dataclass(frozen=True)
class ParkingAislePolicyR17:
    station_spacing_m: float = 0.25
    station_slab_half_width_m: float = 0.20
    maximum_lateral_probe_m: float = 6.0
    phase_probe_radius_m: float = 0.50
    target_deviation_max: float = 0.25
    required_target_station_ratio_exclusive: float = 0.50

    def __post_init__(self) -> None:
        distances = (
            self.station_spacing_m,
            self.station_slab_half_width_m,
            self.maximum_lateral_probe_m,
            self.phase_probe_radius_m,
        )
        if any(value <= 0.0 for value in distances):
            raise ValueError("r17 parking aisle distances must be positive")
        if not 0.0 < self.target_deviation_max < 1.0:
            raise ValueError("r17 parking target must lie in (0,1)")
        if not 0.0 < self.required_target_station_ratio_exclusive < 1.0:
            raise ValueError("r17 parking station ratio must lie in (0,1)")


@dataclass(frozen=True)
class ParkingAisleFieldResultR17:
    deviation: np.ndarray
    aisle_labels: np.ndarray
    gate_passed: bool
    failure_code: str
    diagnostics: dict[str, Any]

    def summary(self) -> dict[str, Any]:
        return {
            "gate_passed": self.gate_passed,
            "failure_code": self.failure_code,
            "diagnostics": self.diagnostics,
        }


def _active_interval(route_length_m: float) -> tuple[float, float]:
    contract = TransitionContractR2()
    if route_length_m <= contract.short_path_max_m:
        return 0.0, float(route_length_m)
    return (
        contract.endpoint_transition_each_m,
        float(route_length_m) - contract.endpoint_transition_each_m,
    )


def _cross_section(
    world: RoutePhaseWorld,
    sample: Any,
    phase_instance: int,
    policy: ParkingAislePolicyR17,
) -> tuple[tuple[int, int, int, int], np.ndarray, np.ndarray, np.ndarray] | None:
    center = world.map.world_to_cell(sample.x, sample.y)
    if center is None:
        return None
    resolution = float(world.map.resolution)
    radius = int(
        math.ceil(
            (policy.maximum_lateral_probe_m + policy.station_slab_half_width_m)
            / resolution
        )
    ) + 1
    row0, row1 = max(0, center[0] - radius), min(
        world.map.height, center[0] + radius + 1
    )
    col0, col1 = max(0, center[1] - radius), min(
        world.map.width, center[1] + radius + 1
    )
    rows, cols = np.mgrid[row0:row1, col0:col1]
    x = world.map.full_origin[0] + (cols + world.map.col0 + 0.5) * resolution
    y = (
        world.map.full_origin[1]
        + (world.map.full_height - rows - world.map.row0 - 0.5) * resolution
    )
    dx, dy = x - sample.x, y - sample.y
    longitudinal = dx * sample.tangent_x + dy * sample.tangent_y
    lateral = dx * sample.tangent_y - dy * sample.tangent_x
    mask = (
        (np.abs(longitudinal) <= policy.station_slab_half_width_m + 1.0e-12)
        & (np.abs(lateral) <= policy.maximum_lateral_probe_m + 1.0e-12)
        & world.grids["allowed"][rows, cols].astype(bool)
        & ~world.grids["hard"][rows, cols].astype(bool)
        & (world.master[rows, cols] < 253)
        & (world.grids["parking_components"][rows, cols] == phase_instance)
    )
    count, labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
    positions = np.argwhere(mask)
    if count <= 1 or not len(positions):
        return None
    distance = (
        (positions[:, 0] + row0 - center[0]) ** 2
        + (positions[:, 1] + col0 - center[1]) ** 2
    )
    seed = positions[int(np.argmin(distance))]
    attached = labels == int(labels[tuple(seed)])
    return (row0, row1, col0, col1), attached, np.abs(longitudinal), lateral


def build_route_local_aisle_field(
    world: RoutePhaseWorld,
    route: OrientedRoute,
    policy: ParkingAislePolicyR17 | None = None,
) -> ParkingAisleFieldResultR17:
    """Build a deterministic route-owned aisle-normalised parking guide."""

    policy = policy or ParkingAislePolicyR17()
    shape = world.master.shape
    selected = np.isin(
        world.grids["parking_components"], np.asarray(world.selected_parking)
    )
    region_clearance = cv2.distanceTransform(
        selected.astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE
    ) * float(world.map.resolution)
    combined_clearance = np.minimum(region_clearance, world.map.distance_m)
    deviation = np.full(shape, np.nan, dtype=np.float32)
    owner_distance = np.full(shape, np.inf, dtype=np.float32)
    aisle_labels = np.zeros(shape, dtype=np.int16)
    active_left, active_right = _active_interval(route.length_m)
    samples = route.stations(policy.station_spacing_m)
    phases = [
        _phase_near(world, sample.x, sample.y, policy.phase_probe_radius_m)
        for sample in samples
    ]
    records: list[dict[str, Any]] = []
    run_id = 0
    previous_parking = None
    for sample, phase in zip(samples, phases):
        if phase.kind != "parking":
            previous_parking = None
            continue
        key = int(phase.instance)
        if previous_parking != key:
            run_id += 1
        previous_parking = key
        section = _cross_section(world, sample, key, policy)
        record = {
            "station_m": float(sample.station_m),
            "parking_component": key,
            "aisle_run_id": run_id,
            "active_window": bool(active_left <= sample.station_m <= active_right),
            "target_available": False,
            "route_local_max_clearance_m": None,
            "route_local_component_deviation_best": None,
            "r2_component_deviation_best": None,
            "attached_cell_count": 0,
            "failure_code": "EMPTY_ROUTE_LOCAL_CROSS_SECTION",
        }
        if section is None:
            records.append(record)
            continue
        (row0, row1, col0, col1), attached, longitudinal, _lateral = section
        local_clearance = combined_clearance[row0:row1, col0:col1]
        footprint_safe = (
            attached
            & (world.map.distance_m[row0:row1, col0:col1] > world.safe_threshold)
        )
        record["attached_cell_count"] = int(np.count_nonzero(attached))
        record["footprint_safe_cell_count"] = int(np.count_nonzero(footprint_safe))
        if not np.any(footprint_safe):
            record["failure_code"] = "NO_FOOTPRINT_SAFE_ROUTE_LOCAL_CELL"
            records.append(record)
            continue
        maximum = float(np.max(local_clearance[footprint_safe]))
        if maximum <= 1.0e-9:
            record["failure_code"] = "ZERO_ROUTE_LOCAL_CLEARANCE"
            records.append(record)
            continue
        local_deviation = np.clip(1.0 - local_clearance / maximum, 0.0, 1.0)
        target = footprint_safe & (local_deviation <= policy.target_deviation_max)
        component_values = np.asarray(
            world.grids["parking_deviation"][row0:row1, col0:col1],
            dtype=np.float32,
        )
        record.update(
            {
                "target_available": bool(np.any(target)),
                "route_local_max_clearance_m": maximum,
                "route_local_component_deviation_best": float(
                    np.min(local_deviation[footprint_safe])
                ),
                "r2_component_deviation_best": float(
                    np.nanmin(component_values[footprint_safe])
                ),
                "target_cell_count": int(np.count_nonzero(target)),
                "failure_code": "" if np.any(target) else "NO_LOCAL_AISLE_TARGET",
            }
        )
        view = np.s_[row0:row1, col0:col1]
        replace = attached & (longitudinal < owner_distance[view])
        deviation_view = deviation[view]
        deviation_view[replace] = local_deviation[replace].astype(np.float32)
        owner_view = owner_distance[view]
        owner_view[replace] = longitudinal[replace].astype(np.float32)
        label_view = aisle_labels[view]
        label_view[replace] = int(run_id)
        records.append(record)

    active = [record for record in records if record["active_window"]]
    available = sum(record["target_available"] is True for record in active)
    ratio = available / len(active) if active else None
    passed = bool(
        active
        and ratio is not None
        and ratio > policy.required_target_station_ratio_exclusive
    )
    assigned = np.isfinite(deviation)
    diagnostics = {
        "schema_version": SCHEMA_VERSION,
        "method_id": METHOD_ID,
        "normalization_scope": "route_attached_obstacle_separated_cross_section",
        "r2_component_metric_preserved_separately": True,
        "policy": asdict(policy),
        "active_interval_m": [active_left, active_right],
        "parking_station_count": len(records),
        "active_parking_station_count": len(active),
        "active_target_available_station_count": available,
        "active_target_available_station_ratio": ratio,
        "assigned_cell_count": int(np.count_nonzero(assigned)),
        "aisle_run_count": int(run_id),
        "station_records": records,
        "field_sha256": hashlib.sha256(
            np.ascontiguousarray(
                np.nan_to_num(deviation, nan=-1.0), dtype=np.float32
            ).tobytes()
        ).hexdigest(),
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
    return ParkingAisleFieldResultR17(
        deviation=deviation,
        aisle_labels=aisle_labels,
        gate_passed=passed,
        failure_code="" if passed else "ROUTE_LOCAL_AISLE_TARGET_COVERAGE_FAILED",
        diagnostics=diagnostics,
    )


__all__ = [
    "METHOD_ID",
    "SCHEMA_VERSION",
    "ParkingAisleFieldResultR17",
    "ParkingAislePolicyR17",
    "build_route_local_aisle_field",
]
