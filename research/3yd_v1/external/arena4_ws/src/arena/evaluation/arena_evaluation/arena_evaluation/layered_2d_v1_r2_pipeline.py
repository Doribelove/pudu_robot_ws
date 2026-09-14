"""Independent latency-focused revision of the 2D-V1 planner.

The revision deliberately inherits the frozen r1 architecture: original
2A-V0 skeleton topology, Graph D* Lite, no L2, and corridor-wide Nav2 Smac
Hybrid using DUBIN motion.  Changes in this module are implementation and
measurement changes only; r1 entry points remain untouched.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np

from .dynamic_snapshot import DynamicSnapshot
from .graph_dstar_lite import GraphDStarLite, GraphEdge, INF
from . import layered_2d_v0_pipeline as v0
from . import layered_2d_v1_pipeline as r1


ARCHITECTURE_ID = "2D-V1"
IMPLEMENTATION_REVISION = "r2"
PROTOCOL_VERSION = r1.PROTOCOL_VERSION
L1_BACKEND = r1.L1_BACKEND
L3_BACKEND = r1.L3_BACKEND


def _elapsed_ms(started_ns: int) -> float:
    return (time.monotonic_ns() - started_ns) / 1.0e6


def _legacy_corridor_mask_timed(
    refined: v0.RefinedTopology,
    route_node_ids: Sequence[int],
    start_pose: Sequence[float],
    goal_pose: Sequence[float],
    *,
    padding_m: float,
    virtual_positions: Mapping[int, Tuple[float, float]],
) -> Tuple[np.ndarray, Dict[str, float]]:
    """Build the exact r1 full-grid mask while exposing non-overlapping phases."""
    hospital_map = refined.artifact.hospital_map
    raw_free = v0._raw_free_mask(refined.artifact)
    centerline, rasterization_ms = _rasterize_route_centerline(
        refined, route_node_ids, start_pose, goal_pose,
        virtual_positions=virtual_positions,
    )
    radius_cells = max(
        1,
        int(math.ceil(float(padding_m) / max(1.0e-9, float(hospital_map.resolution)))),
    )
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * radius_cells + 1, 2 * radius_cells + 1),
    )
    dilation_started_ns = time.monotonic_ns()
    mask = (cv2.dilate(centerline, kernel, iterations=1) > 0) & raw_free
    dilation_ms = _elapsed_ms(dilation_started_ns)
    return mask, {
        "corridor_mask_rasterization_ms": rasterization_ms,
        "corridor_mask_dilation_ms": dilation_ms,
        "corridor_mask_roi_height_cells": int(mask.shape[0]),
        "corridor_mask_roi_width_cells": int(mask.shape[1]),
    }


def _rasterize_route_centerline(
    refined: v0.RefinedTopology,
    route_node_ids: Sequence[int],
    start_pose: Sequence[float],
    goal_pose: Sequence[float],
    *,
    virtual_positions: Mapping[int, Tuple[float, float]],
) -> Tuple[np.ndarray, float]:
    """Rasterize with the exact r1 OpenCV calls and coordinate ordering."""
    hospital_map = refined.artifact.hospital_map
    raw_free = v0._raw_free_mask(refined.artifact)
    raster_started_ns = time.monotonic_ns()
    centerline = np.zeros(raw_free.shape, dtype=np.uint8)
    positions: Dict[int, Tuple[float, float]] = {
        int(node_id): (float(node.x), float(node.y))
        for node_id, node in refined.nodes.items()
    }
    positions.update({
        int(node_id): (float(value[0]), float(value[1]))
        for node_id, value in virtual_positions.items()
    })
    for first_id, second_id in zip(route_node_ids, route_node_ids[1:]):
        geometry: Optional[List[List[float]]] = None
        for target, edge, reverse in refined.adjacency.get(int(first_id), []):
            if int(target) == int(second_id):
                geometry = [
                    list(point)
                    for point in (reversed(edge.polyline) if reverse else edge.polyline)
                ]
                break
        if geometry is None:
            first = positions.get(int(first_id))
            second = positions.get(int(second_id))
            if first is not None and second is not None:
                geometry = [list(first), list(second)]
        for first, second in zip(geometry or [], (geometry or [])[1:]):
            a = hospital_map.world_to_cell(float(first[0]), float(first[1]))
            b = hospital_map.world_to_cell(float(second[0]), float(second[1]))
            if a is not None and b is not None:
                cv2.line(
                    centerline, (int(a[1]), int(a[0])), (int(b[1]), int(b[0])), 1, 1,
                )
    for pose in (start_pose, goal_pose):
        cell = hospital_map.world_to_cell(float(pose[0]), float(pose[1]))
        if cell is not None:
            centerline[cell] = 1
    for pose, backward, forward in (
        (start_pose, 0.25, 0.75), (goal_pose, 0.75, 0.25),
    ):
        direction = (math.cos(float(pose[2])), math.sin(float(pose[2])))
        a = (
            float(pose[0]) - backward * direction[0],
            float(pose[1]) - backward * direction[1],
        )
        b = (
            float(pose[0]) + forward * direction[0],
            float(pose[1]) + forward * direction[1],
        )
        ca = hospital_map.world_to_cell(*a)
        cb = hospital_map.world_to_cell(*b)
        if ca is not None and cb is not None:
            cv2.line(centerline, (int(ca[1]), int(ca[0])), (int(cb[1]), int(cb[0])), 1, 1)
    return centerline, _elapsed_ms(raster_started_ns)


def _json_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    ).hexdigest()


class CorridorMaskCache:
    """ROI-compressed, integrity-bound route masks for one map session."""

    CACHE_VERSION = "2d-v1-r2-roi-mask-v1"

    def __init__(
        self,
        refined: v0.RefinedTopology,
        footprint: Sequence[Sequence[float]],
        *,
        base_map_hash: str,
        topology_cache_key: str,
        topology_source_hash: str,
        corridor_semantics: str,
        corridor_profile: str,
    ) -> None:
        self.refined = refined
        self.footprint_hash = v0._footprint_hash(footprint)
        self.base_map_hash = str(base_map_hash)
        self.topology_cache_key = str(topology_cache_key)
        self.topology_source_hash = str(topology_source_hash)
        self.corridor_semantics = str(corridor_semantics)
        self.corridor_profile = str(corridor_profile)
        self.entries: Dict[str, Dict[str, Any]] = {}
        self.hits = 0
        self.misses = 0
        self.build_time_ms = 0.0
        self.lookup_time_ms = 0.0
        self._kernel_cache: Dict[int, np.ndarray] = {}

    @property
    def memory_bytes(self) -> int:
        return int(sum(
            len(entry["packed"]) + len(key.encode("ascii")) + 64
            for key, entry in self.entries.items()
        ))

    def _key(
        self,
        route_node_ids: Sequence[int],
        route_edge_ids: Sequence[str],
        start_pose: Sequence[float],
        goal_pose: Sequence[float],
        padding_m: float,
        extra_margin_m: float,
        snapshot: DynamicSnapshot,
    ) -> Tuple[str, str]:
        route_signature = _json_hash({
            "route_node_ids": [int(value) for value in route_node_ids],
            "route_edge_ids": [str(value) for value in route_edge_ids],
            "start_pose": [float(value) for value in start_pose],
            "goal_pose": [float(value) for value in goal_pose],
        })
        payload = {
            "cache_version": self.CACHE_VERSION,
            "base_map_hash": self.base_map_hash,
            "static_map_hash": str(self.refined.metadata.get("static_map_hash", "")),
            "topology_cache_key": self.topology_cache_key,
            "topology_source_hash": self.topology_source_hash,
            "route_signature": route_signature,
            "route_node_ids": [int(value) for value in route_node_ids],
            "route_edge_ids": [str(value) for value in route_edge_ids],
            "corridor_padding_m": float(padding_m),
            "corridor_extra_margin_m": float(extra_margin_m),
            "corridor_semantics": self.corridor_semantics,
            "corridor_profile": self.corridor_profile,
            "footprint_hash": self.footprint_hash,
            "resolution_m": float(self.refined.artifact.hospital_map.resolution),
            "inflation_profile": {
                "shape": "opencv_morph_ellipse",
                "iterations": 1,
                "unknown_is_collision": True,
            },
            "dynamic_snapshot_hash": str(snapshot.snapshot_hash),
            "dynamic_snapshot_id": str(snapshot.snapshot_id),
            "dynamic_map_version": str(snapshot.map_version),
        }
        return _json_hash(payload), route_signature

    def _kernel(self, radius_cells: int) -> np.ndarray:
        kernel = self._kernel_cache.get(int(radius_cells))
        if kernel is None:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (2 * int(radius_cells) + 1, 2 * int(radius_cells) + 1),
            )
            self._kernel_cache[int(radius_cells)] = kernel
        return kernel

    @staticmethod
    def _materialize(entry: Mapping[str, Any], full_shape: Tuple[int, int]) -> np.ndarray:
        mask = np.zeros(full_shape, dtype=bool)
        r0, r1_, c0, c1 = [int(value) for value in entry["bbox"]]
        height, width = [int(value) for value in entry["shape"]]
        if height and width:
            unpacked = np.unpackbits(
                np.frombuffer(entry["packed"], dtype=np.uint8), count=height * width,
            ).reshape((height, width)).astype(bool, copy=False)
            mask[r0:r1_, c0:c1] = unpacked
        return mask

    def get_or_build(
        self,
        route_node_ids: Sequence[int],
        route_edge_ids: Sequence[str],
        start_pose: Sequence[float],
        goal_pose: Sequence[float],
        *,
        padding_m: float,
        extra_margin_m: float,
        virtual_positions: Mapping[int, Tuple[float, float]],
        snapshot: DynamicSnapshot,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        if snapshot.is_expired():
            raise ValueError("refusing corridor cache lookup with an expired dynamic snapshot")
        lookup_started_ns = time.monotonic_ns()
        key, route_signature = self._key(
            route_node_ids, route_edge_ids, start_pose, goal_pose,
            padding_m, extra_margin_m, snapshot,
        )
        entry = self.entries.get(key)
        key_lookup_ms = _elapsed_ms(lookup_started_ns)
        full_shape = tuple(v0._raw_free_mask(self.refined.artifact).shape)
        if entry is not None:
            materialize_started_ns = time.monotonic_ns()
            mask = self._materialize(entry, full_shape)
            materialization_ms = _elapsed_ms(materialize_started_ns)
            lookup_ms = key_lookup_ms + materialization_ms
            self.hits += 1
            self.lookup_time_ms += lookup_ms
            return mask, {
                "corridor_cache_key": key,
                "corridor_route_signature": route_signature,
                "corridor_cache_hit": True,
                "corridor_cache_hits": self.hits,
                "corridor_cache_misses": self.misses,
                "corridor_cache_lookup_time_ms": lookup_ms,
                "corridor_cache_build_time_ms": 0.0,
                "corridor_cache_memory_bytes": self.memory_bytes,
                "corridor_mask_rasterization_ms": 0.0,
                "corridor_mask_dilation_ms": 0.0,
                "corridor_mask_materialization_ms": materialization_ms,
                "corridor_mask_hash_diagnostics_ms": 0.0,
                "corridor_mask_cell_count_diagnostics_ms": 0.0,
                "corridor_mask_total_time_ms": lookup_ms,
                "corridor_mask_roi_height_cells": int(entry["shape"][0]),
                "corridor_mask_roi_width_cells": int(entry["shape"][1]),
                "corridor_mask_hash": str(entry["mask_hash"]),
                "corridor_allowed_cells": int(entry["allowed_cells"]),
            }

        build_started_ns = time.monotonic_ns()
        effective_padding_m = float(padding_m) + float(extra_margin_m)
        centerline, rasterization_ms = _rasterize_route_centerline(
            self.refined, route_node_ids, start_pose, goal_pose,
            virtual_positions=virtual_positions,
        )
        radius_cells = max(1, int(math.ceil(
            effective_padding_m
            / max(1.0e-9, float(self.refined.artifact.hospital_map.resolution))
        )))
        nonzero = cv2.findNonZero(centerline)
        raw_free = v0._raw_free_mask(self.refined.artifact)
        if nonzero is None:
            r0 = r1_ = c0 = c1 = 0
            compact = np.zeros((0, 0), dtype=bool)
            dilation_ms = 0.0
        else:
            x, y, width, height = cv2.boundingRect(nonzero)
            r0 = max(0, int(y) - radius_cells)
            r1_ = min(int(raw_free.shape[0]), int(y + height) + radius_cells)
            c0 = max(0, int(x) - radius_cells)
            c1 = min(int(raw_free.shape[1]), int(x + width) + radius_cells)
            dilation_started_ns = time.monotonic_ns()
            compact = (
                cv2.dilate(
                    centerline[r0:r1_, c0:c1], self._kernel(radius_cells), iterations=1,
                ) > 0
            ) & raw_free[r0:r1_, c0:c1]
            dilation_ms = _elapsed_ms(dilation_started_ns)
        materialize_started_ns = time.monotonic_ns()
        mask = np.zeros(full_shape, dtype=bool)
        if compact.size:
            mask[r0:r1_, c0:c1] = compact
        materialization_ms = _elapsed_ms(materialize_started_ns)
        hash_started_ns = time.monotonic_ns()
        mask_hash = hashlib.sha256(
            np.ascontiguousarray(mask, dtype=np.uint8).tobytes()
        ).hexdigest()
        hash_ms = _elapsed_ms(hash_started_ns)
        count_started_ns = time.monotonic_ns()
        allowed_cells = int(np.count_nonzero(compact))
        count_ms = _elapsed_ms(count_started_ns)
        packed = np.packbits(compact.reshape(-1)).tobytes()
        entry = {
            "bbox": (r0, r1_, c0, c1),
            "shape": tuple(compact.shape),
            "packed": packed,
            "mask_hash": mask_hash,
            "allowed_cells": allowed_cells,
        }
        self.entries[key] = entry
        self.misses += 1
        build_ms = _elapsed_ms(build_started_ns)
        self.build_time_ms += build_ms
        self.lookup_time_ms += key_lookup_ms
        return mask, {
            "corridor_cache_key": key,
            "corridor_route_signature": route_signature,
            "corridor_cache_hit": False,
            "corridor_cache_hits": self.hits,
            "corridor_cache_misses": self.misses,
            "corridor_cache_lookup_time_ms": key_lookup_ms,
            "corridor_cache_build_time_ms": build_ms,
            "corridor_cache_memory_bytes": self.memory_bytes,
            "corridor_mask_rasterization_ms": rasterization_ms,
            "corridor_mask_dilation_ms": dilation_ms,
            "corridor_mask_materialization_ms": materialization_ms,
            "corridor_mask_hash_diagnostics_ms": hash_ms,
            "corridor_mask_cell_count_diagnostics_ms": count_ms,
            "corridor_mask_total_time_ms": key_lookup_ms + build_ms,
            "corridor_mask_roi_height_cells": int(compact.shape[0]),
            "corridor_mask_roi_width_cells": int(compact.shape[1]),
            "corridor_mask_hash": mask_hash,
            "corridor_allowed_cells": allowed_cells,
        }


@dataclass(frozen=True)
class EdgeSegmentRecord:
    edge_id: str
    segment_index: int
    first_x: float
    first_y: float
    second_x: float
    second_y: float
    min_x: float
    min_y: float
    max_x: float
    max_y: float


@dataclass
class EdgeSegmentSpatialIndex:
    """Deterministic uniform-grid index over persisted topology segments."""

    cell_size_m: float
    records: Tuple[EdgeSegmentRecord, ...]
    buckets: Dict[Tuple[int, int], Tuple[int, ...]]
    build_time_ms: float = 0.0

    @classmethod
    def build(
        cls, edges: Mapping[str, v0.RefinedEdge], cell_size_m: float = 4.0,
    ) -> "EdgeSegmentSpatialIndex":
        started_ns = time.monotonic_ns()
        size = max(0.1, float(cell_size_m))
        records: List[EdgeSegmentRecord] = []
        mutable_buckets: Dict[Tuple[int, int], List[int]] = {}
        for edge_id, edge in sorted(edges.items()):
            for segment_index, (first, second) in enumerate(zip(edge.polyline, edge.polyline[1:])):
                record = EdgeSegmentRecord(
                    str(edge_id), int(segment_index),
                    float(first[0]), float(first[1]), float(second[0]), float(second[1]),
                    min(float(first[0]), float(second[0])),
                    min(float(first[1]), float(second[1])),
                    max(float(first[0]), float(second[0])),
                    max(float(first[1]), float(second[1])),
                )
                record_index = len(records)
                records.append(record)
                for bucket_x in range(
                    math.floor(record.min_x / size), math.floor(record.max_x / size) + 1,
                ):
                    for bucket_y in range(
                        math.floor(record.min_y / size), math.floor(record.max_y / size) + 1,
                    ):
                        mutable_buckets.setdefault((bucket_x, bucket_y), []).append(record_index)
        index = cls(
            size,
            tuple(records),
            {key: tuple(sorted(set(values))) for key, values in mutable_buckets.items()},
        )
        index.build_time_ms = _elapsed_ms(started_ns)
        return index

    @property
    def memory_bytes(self) -> int:
        return int(
            len(self.records) * 80
            + sum(16 + len(values) * 8 for values in self.buckets.values())
            + sum(len(record.edge_id.encode("utf-8")) for record in self.records)
        )

    def query(self, x: float, y: float, radius_m: float) -> List[EdgeSegmentRecord]:
        radius = max(0.0, float(radius_m))
        record_ids: set[int] = set()
        for bucket_x in range(
            math.floor((float(x) - radius) / self.cell_size_m),
            math.floor((float(x) + radius) / self.cell_size_m) + 1,
        ):
            for bucket_y in range(
                math.floor((float(y) - radius) / self.cell_size_m),
                math.floor((float(y) + radius) / self.cell_size_m) + 1,
            ):
                record_ids.update(self.buckets.get((bucket_x, bucket_y), ()))
        radius_squared = radius * radius
        result = []
        for record_id in sorted(
            record_ids,
            key=lambda value: (
                self.records[value].edge_id, self.records[value].segment_index,
            ),
        ):
            record = self.records[record_id]
            dx = max(record.min_x - float(x), 0.0, float(x) - record.max_x)
            dy = max(record.min_y - float(y), 0.0, float(y) - record.max_y)
            if dx * dx + dy * dy <= radius_squared + 1.0e-12:
                result.append(record)
        return result


def _clone_candidate(candidate: v0.AttachmentCandidate) -> v0.AttachmentCandidate:
    return v0.AttachmentCandidate(
        int(candidate.candidate_id), float(candidate.x), float(candidate.y),
        float(candidate.distance_m), float(candidate.heading_error_rad),
        int(candidate.component_id), str(candidate.role),
        tuple((int(node_id), float(cost)) for node_id, cost in candidate.connections),
    )


def _serialize_candidate(candidate: v0.AttachmentCandidate) -> Dict[str, Any]:
    return {
        "candidate_id": int(candidate.candidate_id),
        "x": float(candidate.x),
        "y": float(candidate.y),
        "component_id": int(candidate.component_id),
        "role": str(candidate.role),
        "distance_m": float(candidate.distance_m),
        "heading_error_rad": float(candidate.heading_error_rad),
        "connections": [
            {"node_id": int(node_id), "cost": float(cost)}
            for node_id, cost in candidate.connections
        ],
    }


class Layered2DV1R2Pipeline(r1.Layered2DV1Pipeline):
    """r1-equivalent planner with phase-accountable execution."""

    def __init__(
        self,
        *args: Any,
        enable_corridor_cache: bool = True,
        enable_attachment_optimization: bool = True,
        base_map_hash: str = "",
        topology_cache_key: str = "",
        topology_source_hash: str = "",
        corridor_semantics: str = "raw_map_smac_aligned",
        **kwargs: Any,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.enable_corridor_cache = bool(enable_corridor_cache)
        self.enable_attachment_optimization = bool(enable_attachment_optimization)
        self.corridor_semantics = str(corridor_semantics)
        self.corridor_cache = CorridorMaskCache(
            self.refined,
            self.footprint,
            base_map_hash=base_map_hash or str(self.refined.metadata.get("static_map_hash", "")),
            topology_cache_key=topology_cache_key or str(
                self.refined.metadata.get("topology_cache_key", "")
            ),
            topology_source_hash=topology_source_hash or str(
                self.refined.artifact.metadata.get("source_hash", "")
            ),
            corridor_semantics=self.corridor_semantics,
            corridor_profile=self.corridor_profile,
        )
        self.edge_segment_index = EdgeSegmentSpatialIndex.build(self.refined.edges)
        self.endpoint_cache: Dict[str, Tuple[v0.AttachmentCandidate, ...]] = {}
        self.endpoint_cache_hits = 0
        self.endpoint_cache_misses = 0
        self.endpoint_cache_lookup_time_ms = 0.0
        self.endpoint_cache_build_time_ms = 0.0
        self._last_graph_timing: Dict[str, float] = {}

    @property
    def endpoint_cache_memory_bytes(self) -> int:
        return int(sum(
            len(key.encode("ascii"))
            + sum(96 + len(candidate.connections) * 24 for candidate in candidates)
            for key, candidates in self.endpoint_cache.items()
        ))

    def _endpoint_cache_key(
        self, pose: Sequence[float], snapshot: DynamicSnapshot,
    ) -> str:
        return _json_hash({
            "cache_version": "2d-v1-r2-endpoint-v1",
            "pose": [float(value) for value in pose],
            "topology_cache_key": self.corridor_cache.topology_cache_key,
            "topology_source_hash": self.corridor_cache.topology_source_hash,
            "footprint_hash": self.corridor_cache.footprint_hash,
            "collision_parameters": {
                "unknown_is_collision": True,
                "connection_spacing_m": float(self.refined.artifact.hospital_map.resolution),
                "radius_m": float(self.endpoint_radius_m),
                "candidate_limit": int(self.candidate_limit),
            },
            "dynamic_snapshot_hash": str(snapshot.snapshot_hash),
            "dynamic_snapshot_id": str(snapshot.snapshot_id),
            "dynamic_map_version": str(snapshot.map_version),
        })

    def _optimized_attachment_candidates(
        self, pose: Sequence[float], snapshot: DynamicSnapshot,
    ) -> Tuple[List[v0.AttachmentCandidate], Dict[str, Any]]:
        if snapshot.is_expired():
            raise ValueError("refusing endpoint cache lookup with an expired dynamic snapshot")
        cache_started_ns = time.monotonic_ns()
        cache_key = self._endpoint_cache_key(pose, snapshot)
        cached = self.endpoint_cache.get(cache_key)
        if cached is not None:
            candidates = [_clone_candidate(candidate) for candidate in cached]
            lookup_ms = _elapsed_ms(cache_started_ns)
            self.endpoint_cache_hits += 1
            self.endpoint_cache_lookup_time_ms += lookup_ms
            return candidates, {
                "endpoint_cache_hit": True,
                "endpoint_cache_key": cache_key,
                "endpoint_cache_lookup_time_ms": lookup_ms,
                "attachment_node_lookup_time_ms": 0.0,
                "edge_projection_candidate_lookup_time_ms": 0.0,
                "projection_connection_collision_filter_time_ms": 0.0,
                "attachment_candidate_ranking_time_ms": 0.0,
                "endpoint_cache_store_time_ms": 0.0,
                "edge_projection_segments_scanned": 0,
                "edge_projection_segments_total": len(self.edge_segment_index.records),
                "endpoint_cache_memory_bytes": self.endpoint_cache_memory_bytes,
                "attachment_lookup_time_ms": lookup_ms,
            }
        cache_lookup_ms = _elapsed_ms(cache_started_ns)
        self.endpoint_cache_lookup_time_ms += cache_lookup_ms
        self.endpoint_cache_misses += 1

        hospital_map = self.refined.artifact.hospital_map
        dynamic_cells = set(snapshot.inflated_cells(0))
        node_started_ns = time.monotonic_ns()
        indexed_node_ids = (
            self.refined.attachment_index.query(
                float(pose[0]), float(pose[1]), float(self.endpoint_radius_m),
            )
            if isinstance(self.refined.attachment_index, v0.RefinedNodeSpatialIndex)
            else sorted(self.refined.nodes)
        )
        raw_nodes: List[Tuple[Any, float, float, float]] = []
        for node_id in indexed_node_ids:
            node = self.refined.nodes.get(int(node_id))
            if node is None:
                continue
            distance = math.hypot(
                float(node.x) - float(pose[0]), float(node.y) - float(pose[1]),
            )
            node_cell = hospital_map.world_to_cell(float(node.x), float(node.y))
            if (
                distance > float(self.endpoint_radius_m)
                or (node_cell is not None and tuple(node_cell) in dynamic_cells)
            ):
                continue
            incident_tangents: List[float] = []
            for _neighbor, incident_edge, reverse in self.refined.adjacency.get(int(node_id), []):
                tangent = float(incident_edge.local_tangent)
                if reverse:
                    tangent = (tangent + math.pi) % (2.0 * math.pi)
                incident_tangents.append(tangent)
            heading_error = (
                min(
                    abs((tangent - float(pose[2]) + math.pi) % (2.0 * math.pi) - math.pi)
                    for tangent in incident_tangents
                )
                if incident_tangents else math.pi
            )
            penalty = distance + 0.5 * abs(heading_error) + 0.1 / max(
                0.05, float(node.clearance_m),
            )
            raw_nodes.append((node, distance, heading_error, penalty))
        node_lookup_ms = _elapsed_ms(node_started_ns)

        edge_started_ns = time.monotonic_ns()
        segment_records = self.edge_segment_index.query(
            float(pose[0]), float(pose[1]), float(self.endpoint_radius_m),
        )
        best_by_edge: Dict[str, Tuple[float, float, float]] = {}
        for record in segment_records:
            dx = record.second_x - record.first_x
            dy = record.second_y - record.first_y
            denominator = dx * dx + dy * dy
            fraction = (
                0.0 if denominator <= 1.0e-12 else max(
                    0.0,
                    min(
                        1.0,
                        ((float(pose[0]) - record.first_x) * dx
                         + (float(pose[1]) - record.first_y) * dy) / denominator,
                    ),
                )
            )
            x = record.first_x + fraction * dx
            y = record.first_y + fraction * dy
            distance = math.hypot(x - float(pose[0]), y - float(pose[1]))
            previous = best_by_edge.get(record.edge_id)
            if previous is None or distance < previous[0]:
                best_by_edge[record.edge_id] = (distance, x, y)
        raw_edges: List[Tuple[v0.RefinedEdge, float, float, float, float]] = []
        for edge_id, edge in sorted(self.refined.edges.items()):
            best = best_by_edge.get(str(edge_id))
            if best is None or best[0] > float(self.endpoint_radius_m):
                continue
            distance, x, y = best
            if len(edge.polyline) >= 2:
                last_first, last_second = edge.polyline[-2], edge.polyline[-1]
                final_tangent = math.atan2(
                    float(last_second[1]) - float(last_first[1]),
                    float(last_second[0]) - float(last_first[0]),
                )
            else:
                final_tangent = float(edge.local_tangent)
            # r1 retained the heading error from the final segment iteration,
            # not the nearest segment. Preserve that observable ranking exactly.
            heading_error = abs(
                (final_tangent - float(pose[2]) + math.pi) % (2.0 * math.pi) - math.pi
            )
            raw_edges.append((edge, distance, x, y, heading_error))
        edge_lookup_ms = _elapsed_ms(edge_started_ns)

        filtering_started_ns = time.monotonic_ns()
        candidates: List[v0.AttachmentCandidate] = []
        seen: set[Tuple[int, int, str]] = set()
        next_virtual = -1
        for node, distance, heading_error, penalty in raw_nodes:
            if hospital_map.footprint_collision(
                (node.x, node.y, float(pose[2])), self.footprint,
                unknown_is_collision=True,
            ):
                continue
            if not v0._connection_safe(
                hospital_map, pose, (node.x, node.y, float(pose[2])),
                self.footprint, hospital_map.resolution,
            ):
                continue
            key = (int(round(node.x / 0.05)), int(round(node.y / 0.05)), "node")
            if key in seen:
                continue
            seen.add(key)
            candidates.append(v0.AttachmentCandidate(
                next_virtual, node.x, node.y, distance, heading_error,
                node.component_id, node.role, ((node.node_id, penalty),),
            ))
            next_virtual -= 1
        for edge, distance, x, y, heading_error in raw_edges:
            if hospital_map.footprint_collision(
                (x, y, float(pose[2])), self.footprint, unknown_is_collision=True,
            ):
                continue
            if not v0._connection_safe(
                hospital_map, pose, (x, y, float(pose[2])),
                self.footprint, hospital_map.resolution,
            ):
                continue
            key = (int(round(x / 0.05)), int(round(y / 0.05)), "edge")
            if key in seen:
                continue
            seen.add(key)
            source = self.refined.nodes[edge.source_node]
            attach_cost = distance + 0.5 * heading_error + 0.1 / max(
                0.05, edge.min_clearance_m,
            )
            candidates.append(v0.AttachmentCandidate(
                next_virtual, x, y, distance, heading_error, source.component_id,
                "edge_projection",
                (
                    (edge.source_node, attach_cost + 0.5 * edge.length_m),
                    (edge.target_node, attach_cost + 0.5 * edge.length_m),
                ),
            ))
            next_virtual -= 1
        filtering_ms = _elapsed_ms(filtering_started_ns)

        ranking_started_ns = time.monotonic_ns()
        candidates.sort(key=lambda item: (
            item.distance_m + item.heading_error_rad, item.candidate_id,
        ))
        candidates = candidates[:max(1, int(self.candidate_limit))]
        candidates = [
            self._nearest_edge_endpoint(candidate, self.refined)
            for candidate in candidates
        ]
        candidates.sort(key=lambda candidate: (
            0 if str(candidate.role) == "original" else 1,
            float(candidate.distance_m),
            float(candidate.heading_error_rad),
            int(candidate.candidate_id),
        ))
        ranking_ms = _elapsed_ms(ranking_started_ns)

        store_started_ns = time.monotonic_ns()
        self.endpoint_cache[cache_key] = tuple(
            _clone_candidate(candidate) for candidate in candidates
        )
        store_ms = _elapsed_ms(store_started_ns)
        total_ms = sum((
            cache_lookup_ms, node_lookup_ms, edge_lookup_ms, filtering_ms,
            ranking_ms, store_ms,
        ))
        self.endpoint_cache_build_time_ms += sum((
            node_lookup_ms, edge_lookup_ms, filtering_ms, ranking_ms, store_ms,
        ))
        return candidates, {
            "endpoint_cache_hit": False,
            "endpoint_cache_key": cache_key,
            "endpoint_cache_lookup_time_ms": cache_lookup_ms,
            "attachment_node_lookup_time_ms": node_lookup_ms,
            "edge_projection_candidate_lookup_time_ms": edge_lookup_ms,
            "projection_connection_collision_filter_time_ms": filtering_ms,
            "attachment_candidate_ranking_time_ms": ranking_ms,
            "endpoint_cache_store_time_ms": store_ms,
            "edge_projection_segments_scanned": len(segment_records),
            "edge_projection_segments_total": len(self.edge_segment_index.records),
            "endpoint_cache_memory_bytes": self.endpoint_cache_memory_bytes,
            "attachment_lookup_time_ms": total_ms,
        }

    def _attach(
        self, query: Any, snapshot: DynamicSnapshot,
    ) -> Tuple[List[v0.AttachmentCandidate], List[v0.AttachmentCandidate], Dict[str, Any]]:
        if not self.enable_attachment_optimization:
            return super()._attach(query, snapshot)
        starts, start_timing = self._optimized_attachment_candidates(query.start, snapshot)
        goals, goal_timing = self._optimized_attachment_candidates(query.goal, snapshot)
        serialization_started_ns = time.monotonic_ns()
        serialized_starts = [_serialize_candidate(candidate) for candidate in starts]
        serialized_goals = [_serialize_candidate(candidate) for candidate in goals]
        serialization_ms = _elapsed_ms(serialization_started_ns)
        leaf_fields = (
            "endpoint_cache_lookup_time_ms", "attachment_node_lookup_time_ms",
            "edge_projection_candidate_lookup_time_ms",
            "projection_connection_collision_filter_time_ms",
            "attachment_candidate_ranking_time_ms", "endpoint_cache_store_time_ms",
        )
        diagnostics = {
            field: float(start_timing.get(field, 0.0)) + float(goal_timing.get(field, 0.0))
            for field in leaf_fields
        }
        diagnostics.update({
            "attachment_diagnostics_serialization_time_ms": serialization_ms,
            "attachment_lookup_time_ms": sum(diagnostics[field] for field in leaf_fields)
            + serialization_ms,
            "candidate_collision_check_time_ms": diagnostics[
                "projection_connection_collision_filter_time_ms"
            ],
            "attachment_candidate_count": len(starts) + len(goals),
            "start_attachment_candidate_count": len(starts),
            "goal_attachment_candidate_count": len(goals),
            "start_attachment_candidates": serialized_starts,
            "goal_attachment_candidates": serialized_goals,
            "start_endpoint_cache_hit": bool(start_timing.get("endpoint_cache_hit")),
            "goal_endpoint_cache_hit": bool(goal_timing.get("endpoint_cache_hit")),
            "endpoint_cache_hits": self.endpoint_cache_hits,
            "endpoint_cache_misses": self.endpoint_cache_misses,
            "endpoint_cache_memory_bytes": self.endpoint_cache_memory_bytes,
            "edge_segment_index_build_time_ms": self.edge_segment_index.build_time_ms,
            "edge_segment_index_memory_bytes": self.edge_segment_index.memory_bytes,
            "edge_projection_segments_scanned": int(
                start_timing.get("edge_projection_segments_scanned", 0)
            ) + int(goal_timing.get("edge_projection_segments_scanned", 0)),
            "edge_projection_segments_total": len(self.edge_segment_index.records) * 2,
            "endpoint_attachment_policy": "nearest_feasible_candidate_ranked",
            "attachment_rank_penalty_m": r1.ATTACHMENT_RANK_PENALTY_M,
        })
        return starts, goals, diagnostics

    def _make_graph(
        self,
        starts: Sequence[v0.AttachmentCandidate],
        goals: Sequence[v0.AttachmentCandidate],
        snapshot: DynamicSnapshot,
    ) -> Tuple[GraphDStarLite, int, int, Dict[int, Tuple[float, float]]]:
        del snapshot
        base_started_ns = time.monotonic_ns()
        base_nodes = set(self.refined.nodes)
        base_edges = [
            GraphEdge(
                edge.edge_id, edge.source_node, edge.target_node,
                edge.length_m, edge.static_cost, edge.min_clearance_m,
                bidirectional=True,
            )
            for edge in self.refined.edges.values()
        ]
        base_graph_ms = _elapsed_ms(base_started_ns)
        connection_started_ns = time.monotonic_ns()
        virtual_positions: Dict[int, Tuple[float, float]] = {}
        start_virtual = -1000000
        goal_virtual = -2000000
        for index, candidate in enumerate(starts):
            candidate_id = start_virtual - 100 - index
            virtual_positions[candidate_id] = (float(candidate.x), float(candidate.y))
            for target, cost in candidate.connections:
                base_edges.append(GraphEdge(
                    f"attach_start_{index}_{target}", candidate_id,
                    int(target), float(cost), bidirectional=False,
                ))
            base_nodes.add(candidate_id)
        for index, candidate in enumerate(goals):
            candidate_id = goal_virtual - 100 - index
            virtual_positions[candidate_id] = (float(candidate.x), float(candidate.y))
            for target, cost in candidate.connections:
                base_edges.append(GraphEdge(
                    f"attach_goal_{index}_{target}", int(target),
                    candidate_id, float(cost), bidirectional=False,
                ))
            base_nodes.add(candidate_id)
        for index, candidate in enumerate(starts):
            base_edges.append(GraphEdge(
                f"root_start_{index}", start_virtual, start_virtual - 100 - index,
                float(candidate.distance_m + candidate.heading_error_rad
                      + r1.ATTACHMENT_RANK_PENALTY_M * index),
                bidirectional=False,
            ))
        for index, candidate in enumerate(goals):
            base_edges.append(GraphEdge(
                f"root_goal_{index}", goal_virtual - 100 - index, goal_virtual,
                float(candidate.distance_m + candidate.heading_error_rad
                      + r1.ATTACHMENT_RANK_PENALTY_M * index),
                bidirectional=False,
            ))
        base_nodes.update((start_virtual, goal_virtual))
        connection_ms = _elapsed_ms(connection_started_ns)
        initialization_started_ns = time.monotonic_ns()
        planner = GraphDStarLite(
            base_nodes, base_edges, start_virtual, goal_virtual,
            edge_status=self.edge_status, edge_cost_override=self.edge_cost_override,
        )
        planner.node_positions = {
            int(node_id): (float(node.x), float(node.y))
            for node_id, node in self.refined.nodes.items()
        }
        planner.node_positions.update(virtual_positions)
        planner.state_representation = "original_topology_node_id"
        initialization_ms = _elapsed_ms(initialization_started_ns)
        self._last_graph_timing = {
            "base_graph_construction_time_ms": base_graph_ms,
            "temporary_connection_edge_construction_time_ms": connection_ms,
            "dstar_graph_initialization_time_ms": initialization_ms,
        }
        return planner, start_virtual, goal_virtual, virtual_positions

    @staticmethod
    def _with_identity(result: Any) -> Any:
        diagnostics = dict(getattr(result, "diagnostics", {}) or {})
        diagnostics.update({
            "architecture_id": ARCHITECTURE_ID,
            "implementation_revision": IMPLEMENTATION_REVISION,
            "l1_backend": L1_BACKEND,
            "l1_state_type": "original_topology_node_id",
            "topology_refinement_enabled": False,
            "l2_called": False,
            "l2_call_count": 0,
            "rrtstar_call_count": 0,
            "sst_call_count": 0,
        })
        return type(result)(
            result.success, result.points, result.failure_code, result.snapshot_id, diagnostics,
        )

    def _plan_route(
        self, query: Any, snapshot: DynamicSnapshot, *, reuse_graph: bool = False,
    ) -> Tuple[Optional[List[int]], Dict[str, Any]]:
        reuse_existing_graph = bool(reuse_graph and self.dstar is not None)
        if reuse_existing_graph:
            starts, goals, diagnostics = (), (), {"attachment_candidates_reused": True}
        else:
            starts, goals, diagnostics = self._attach(query, snapshot)
            if not starts or not goals:
                diagnostics = dict(diagnostics)
                diagnostics["l1_total_time_ms"] = float(
                    diagnostics.get("attachment_lookup_time_ms", 0.0)
                )
                return None, {**diagnostics, "failure_code": "L1_ENDPOINT_NOT_ATTACHABLE"}

        graph_started_ns = time.monotonic_ns()
        if self.dstar is None:
            self.dstar, _start_virtual, _goal_virtual, self._virtual_positions = self._make_graph(
                starts, goals, snapshot,
            )
        else:
            self._last_graph_timing = {
                "base_graph_construction_time_ms": 0.0,
                "temporary_connection_edge_construction_time_ms": 0.0,
                "dstar_graph_initialization_time_ms": 0.0,
            }
        graph_construction_ms = _elapsed_ms(graph_started_ns)

        stats = self.dstar.compute_shortest_path(
            timeout_s=5.0, max_expansions=v0.DEFAULT_MAX_EXPANSIONS,
        )
        extraction_started_ns = time.monotonic_ns()
        node_path = self.dstar.extract_path()
        route_extraction_ms = _elapsed_ms(extraction_started_ns)
        diagnostics = dict(diagnostics)
        diagnostics.update({
            **self._last_graph_timing,
            "graph_construction_time_ms": graph_construction_ms,
            "l1_dstar_initial_time_ms": float(stats.search_time_ms) if self._route_node_ids is None else 0.0,
            "l1_dstar_incremental_time_ms": float(stats.search_time_ms) if self._route_node_ids is not None else 0.0,
            "dstar_lite_search_time_ms": float(stats.search_time_ms),
            "route_extraction_time_ms": route_extraction_ms,
            "dstar_expanded_nodes": int(stats.expanded_nodes),
            "dstar_generated_nodes": int(stats.generated_nodes),
            "dstar_queue_pops": int(stats.queue_pops),
            "dstar_queue_pushes": int(stats.queue_pushes),
            "dstar_state_snapshot": self.dstar.state_snapshot(),
            "dstar_g_start": self.dstar.g.get(int(self.dstar.start), INF),
            "dstar_rhs_start": self.dstar.rhs.get(int(self.dstar.start), INF),
            "dstar_timeout_triggered": bool(stats.timeout_triggered),
            "dstar_no_path": bool(stats.no_path),
            "dstar_queue_size": int(stats.final_queue_size),
            "l1_graph_state_count": len(self.refined.nodes),
            "l1_route_search_ms": float(stats.search_time_ms),
        })
        if node_path is None:
            diagnostics["route_edge_resolution_time_ms"] = 0.0
            diagnostics["route_construction_time_ms"] = route_extraction_ms
            diagnostics["l1_total_time_ms"] = sum(float(diagnostics.get(field, 0.0) or 0.0) for field in (
                "attachment_lookup_time_ms", "graph_construction_time_ms",
                "dstar_lite_search_time_ms", "route_extraction_time_ms",
            ))
            return None, {
                **diagnostics,
                "failure_code": "L1_NO_ROUTE" if not stats.timeout_triggered else "L1_DSTAR_TIMEOUT",
            }

        self._route_node_ids = list(node_path)
        edge_resolution_started_ns = time.monotonic_ns()
        self._route_edge_ids = v0._route_edge_ids(self.refined, node_path)
        edge_resolution_ms = _elapsed_ms(edge_resolution_started_ns)
        diagnostics["route_edge_resolution_time_ms"] = edge_resolution_ms
        diagnostics["route_construction_time_ms"] = route_extraction_ms + edge_resolution_ms
        diagnostics["l1_total_time_ms"] = sum(float(diagnostics.get(field, 0.0) or 0.0) for field in (
            "attachment_lookup_time_ms", "graph_construction_time_ms",
            "dstar_lite_search_time_ms", "route_extraction_time_ms",
            "route_edge_resolution_time_ms",
        ))
        return list(node_path), diagnostics

    def _run_l3(
        self,
        query: Any,
        snapshot: DynamicSnapshot,
        node_path: Sequence[int],
        *,
        validate: bool = True,
        corridor_padding_m: Optional[float] = None,
    ) -> v0.Layered2DResult:
        selected_padding = self.corridor_padding_m if corridor_padding_m is None else float(corridor_padding_m)
        polyline_started_ns = time.monotonic_ns()
        route_points = v0._route_polyline(
            self.refined, node_path, query.start, query.goal, self._virtual_positions,
        )
        route_polyline_ms = _elapsed_ms(polyline_started_ns)
        if self.corridor_profile == "full_map":
            copy_started_ns = time.monotonic_ns()
            mask = v0._raw_free_mask(self.refined.artifact).copy()
            mask_diag = {
                "corridor_mask_rasterization_ms": 0.0,
                "corridor_mask_dilation_ms": 0.0,
                "corridor_mask_copy_ms": _elapsed_ms(copy_started_ns),
            }
        elif self.enable_corridor_cache:
            mask, mask_diag = self.corridor_cache.get_or_build(
                node_path,
                self._route_edge_ids,
                query.start,
                query.goal,
                padding_m=selected_padding,
                extra_margin_m=self.corridor_extra_margin_m,
                virtual_positions=self._virtual_positions,
                snapshot=snapshot,
            )
            mask_diag["corridor_mask_copy_ms"] = 0.0
        else:
            mask, mask_diag = _legacy_corridor_mask_timed(
                self.refined, node_path, query.start, query.goal,
                padding_m=selected_padding + self.corridor_extra_margin_m,
                virtual_positions=self._virtual_positions,
            )
            mask_diag["corridor_mask_copy_ms"] = 0.0
        self._route_mask = mask

        raw_free = v0._raw_free_mask(self.refined.artifact)
        if "corridor_mask_hash" in mask_diag:
            mask_hash = str(mask_diag["corridor_mask_hash"])
            allowed_count = int(mask_diag["corridor_allowed_cells"])
        else:
            hash_started_ns = time.monotonic_ns()
            mask_hash = hashlib.sha256(
                np.ascontiguousarray(mask, dtype=np.uint8).tobytes()
            ).hexdigest()
            hash_ms = _elapsed_ms(hash_started_ns)
            count_started_ns = time.monotonic_ns()
            allowed_count = int(np.count_nonzero(mask))
            count_ms = _elapsed_ms(count_started_ns)
            mask_diag.update({
            "corridor_mask_hash_diagnostics_ms": hash_ms,
                "corridor_mask_cell_count_diagnostics_ms": count_ms,
                "corridor_mask_total_time_ms": sum(
                    float(mask_diag.get(field, 0.0))
                    for field in (
                        "corridor_mask_rasterization_ms", "corridor_mask_dilation_ms",
                        "corridor_mask_copy_ms", "corridor_mask_hash_diagnostics_ms",
                        "corridor_mask_cell_count_diagnostics_ms",
                    )
                ),
            })
        total_free_count = int(np.count_nonzero(raw_free))

        hospital_map = self.refined.artifact.hospital_map
        endpoint_fields: Dict[str, Any] = {
            **mask_diag,
            "route_polyline_construction_time_ms": route_polyline_ms,
            "corridor_profile": self.corridor_profile,
            "corridor_min_clearance_m": v0._route_min_clearance(
                self.refined, self._route_edge_ids, query.start, query.goal,
            ),
            "corridor_free_cells": total_free_count,
            "corridor_mask_hash": mask_hash,
            "start_raw_map_cost": "not_available", "goal_raw_map_cost": "not_available",
            "start_inflated_cost": "not_available", "goal_inflated_cost": "not_available",
            "smac_start_cost": "not_available", "smac_goal_cost": "not_available",
            "start_is_lethal": "not_available", "goal_is_lethal": "not_available",
            "start_full_footprint_valid": "not_available", "goal_full_footprint_valid": "not_available",
            "start_in_corridor": "not_available", "goal_in_corridor": "not_available",
            "selected_start_attachment": "not_available", "selected_goal_attachment": "not_available",
            "attachment_candidate_count": "not_available", "selected_connection_edges": [],
        }
        endpoint_diag_started_ns = time.monotonic_ns()
        for label, pose in (("start", query.start), ("goal", query.goal)):
            cell = hospital_map.world_to_cell(float(pose[0]), float(pose[1]))
            if cell is None:
                endpoint_fields[f"{label}_raw_map_cost"] = "not_available"
                endpoint_fields[f"{label}_full_footprint_valid"] = False
                endpoint_fields[f"{label}_in_corridor"] = False
                endpoint_fields[f"{label}_is_lethal"] = True
            else:
                endpoint_fields[f"{label}_raw_map_cost"] = int(np.asarray(hospital_map.occupancy)[cell])
                endpoint_fields[f"{label}_inflated_cost"] = 0 if bool(raw_free[cell]) else 100
                endpoint_fields[f"{label}_full_footprint_valid"] = not bool(
                    hospital_map.footprint_collision(
                        tuple(pose), self.footprint, unknown_is_collision=True,
                    )
                )
                endpoint_fields[f"{label}_in_corridor"] = bool(mask[cell])
                endpoint_fields[f"{label}_is_lethal"] = not bool(raw_free[cell])
        endpoint_fields["endpoint_diagnostics_time_ms"] = _elapsed_ms(endpoint_diag_started_ns)
        virtual_ids = [int(item) for item in node_path if int(item) in self._virtual_positions]
        if virtual_ids:
            endpoint_fields["selected_start_attachment"] = virtual_ids[0]
            endpoint_fields["selected_goal_attachment"] = virtual_ids[-1]
            endpoint_fields["selected_connection_edges"] = [
                f"virtual:{first}->{second}"
                for first, second in zip(node_path, node_path[1:])
                if int(first) < 0 or int(second) < 0
            ]
        endpoint_fields["attachment_candidate_count"] = int(
            sum(1 for item in node_path if int(item) in self._virtual_positions)
        )
        if not self.l3_planner:
            return v0.Layered2DResult(
                False, failure_code="BACKEND_UNAVAILABLE", snapshot_id=snapshot.snapshot_id,
                diagnostics={**endpoint_fields, "l2_called": False, "l2_call_count": 0,
                             "l3_call_count": 0, "planner_search_started": False},
            )

        self._l3_call_count += 1
        result, l3_diag = self.l3_planner.plan(
            query, mask, snapshot, topology_artifact=self.refined,
        )
        points = list(getattr(result, "points", None) or []) if result is not None else None
        planner_success = bool(getattr(result, "planner_success", False) and points)
        diagnostics = {
            **endpoint_fields,
            "corridor_allowed_cells": allowed_count,
            "corridor_total_free_cells": total_free_count,
            "corridor_area_ratio": float(allowed_count / max(1, total_free_count)),
            "corridor_padding_m": selected_padding,
            "corridor_extra_margin_m": self.corridor_extra_margin_m,
            "topology_node_ids": list(node_path),
            "topology_edge_ids": list(self._route_edge_ids),
            "l2_called": False, "l2_call_count": 0,
            "l3_call_count": self._l3_call_count,
            "route_polyline": route_points,
            **dict(l3_diag or {}),
        }
        failure_code = "" if planner_success else str(
            diagnostics.get("failure_code") or getattr(result, "failure_code", "")
            or "L3_PLANNER_FAILED"
        )
        if planner_success and validate and self.validator is not None:
            validation_started_ns = time.monotonic_ns()
            validation = dict(
                self.validator(self.refined.artifact.hospital_map, query, points) or {}
            )
            diagnostics["pipeline_validation_time_ms"] = _elapsed_ms(validation_started_ns)
            diagnostics.update(validation)
            dynamic_started_ns = time.monotonic_ns()
            diagnostics["dynamic_collision_count"] = (
                v0.dynamic_collision_count(
                    self.refined.artifact.hospital_map, points, self.footprint, snapshot,
                )
                if snapshot.occupied_cells else 0
            )
            diagnostics["dynamic_collision_diagnostics_time_ms"] = _elapsed_ms(dynamic_started_ns)
            if diagnostics["dynamic_collision_count"]:
                diagnostics["failure_code"] = "DYNAMIC_FOOTPRINT_COLLISION"
            if diagnostics["dynamic_collision_count"] or not bool(
                validation.get(
                    "final_valid_success",
                    validation.get("static_footprint_valid", False)
                    and validation.get("kinematic_valid", False),
                )
            ):
                planner_success = False
                failure_code = str(validation.get("failure_code") or "L3_VALIDATION_FAILED")
        else:
            diagnostics.setdefault("pipeline_validation_time_ms", 0.0)
            diagnostics.setdefault("dynamic_collision_diagnostics_time_ms", 0.0)
        return v0.Layered2DResult(
            planner_success, points if planner_success else points, failure_code,
            snapshot.snapshot_id, diagnostics,
        )


__all__ = [
    "ARCHITECTURE_ID", "IMPLEMENTATION_REVISION", "PROTOCOL_VERSION",
    "L1_BACKEND", "L3_BACKEND", "CorridorMaskCache", "Layered2DV1R2Pipeline",
    "EdgeSegmentSpatialIndex", "_legacy_corridor_mask_timed",
    "_rasterize_route_centerline", "_serialize_candidate",
]
