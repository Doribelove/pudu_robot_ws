"""Independent ``2D-V0`` planner: refined topology D* Lite plus full-corridor Smac.

The module deliberately keeps the architecture boundary explicit:

* L1 states are refined topology node ids, never occupancy-grid cells.
* L2 is disabled and no Grid A* helper is imported or called.
* L3 is the existing Smac Hybrid DUBIN session, constrained by the selected
  topology corridor.

Static topology is immutable after construction.  Dynamic observations live in
``DynamicSnapshot`` and an edge overlay owned by :class:`Layered2DV0Pipeline`.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import os
import resource
import tempfile
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

from . import topology
from .dynamic_snapshot import DynamicSnapshot
from .graph_dstar_lite import GraphDStarLite, GraphEdge, GraphDStarSearchStats, INF


ARCHITECTURE_ID = "2D-V0"
IMPLEMENTATION_REVISION = "r4"
PROTOCOL_VERSION = "PLN-02-EXP-V1"
L1_BACKEND = "refined_skeleton_graph_dstar_lite"
L3_BACKEND = "Nav2 SmacPlannerHybrid DUBIN"
L2_CALL_COUNT = 0
RRTSTAR_CALL_COUNT = 0
SST_CALL_COUNT = 0
RESOLUTION_M = 0.05
RMIN_M = 0.40
MAX_CURVATURE = 2.50
ALLOW_REVERSE = False
ALLOW_IN_PLACE_ROTATION = False
DYNAMIC_OBSTACLES = True
DEFAULT_CORRIDOR_PADDING_M = 2.0
DEFAULT_REFINEMENT_SPACING_M = 2.0
DEFAULT_ENDPOINT_RADIUS_M = 8.0
DEFAULT_CANDIDATE_LIMIT = 16
DEFAULT_MAX_EXPANSIONS = 100000


@dataclass
class RefinedNode:
    node_id: int
    x: float
    y: float
    role: str
    component_id: int
    clearance_m: float
    source_node_id: Optional[int] = None
    source_edge_id: Optional[int] = None


@dataclass
class RefinedEdge:
    edge_id: str
    source_node: int
    target_node: int
    polyline: List[List[float]]
    length_m: float
    min_clearance_m: float
    mean_clearance_m: float
    corridor_width_m: float
    local_tangent: float
    turn_support_nodes: List[int]
    corridor_mask_id: str
    static_cost: float
    source_edge_id: int
    edge_cells: Tuple[Tuple[int, int], ...] = field(default_factory=tuple)


@dataclass
class AttachmentCandidate:
    candidate_id: int
    x: float
    y: float
    distance_m: float
    heading_error_rad: float
    component_id: int
    role: str
    connections: Tuple[Tuple[int, float], ...]


@dataclass
class RefinedNodeSpatialIndex:
    """Deterministic uniform-grid index for refined topology nodes.

    The index is deliberately over refined graph nodes only.  It is not a
    replacement for the occupancy grid and therefore cannot accidentally
    turn L1 into a grid search.
    """

    cell_size_m: float
    buckets: Dict[Tuple[int, int], List[int]]

    @classmethod
    def build(cls, nodes: Mapping[int, RefinedNode], cell_size_m: float = 5.0) -> "RefinedNodeSpatialIndex":
        size = max(0.1, float(cell_size_m))
        buckets: Dict[Tuple[int, int], List[int]] = {}
        for node_id, node in nodes.items():
            key = (math.floor(float(node.x) / size), math.floor(float(node.y) / size))
            buckets.setdefault(key, []).append(int(node_id))
        for values in buckets.values():
            values.sort()
        return cls(size, buckets)

    def query(self, x: float, y: float, radius_m: float) -> List[int]:
        radius = max(0.0, float(radius_m))
        min_x = math.floor((float(x) - radius) / self.cell_size_m)
        max_x = math.floor((float(x) + radius) / self.cell_size_m)
        min_y = math.floor((float(y) - radius) / self.cell_size_m)
        max_y = math.floor((float(y) + radius) / self.cell_size_m)
        return [
            int(node_id)
            for bucket_x in range(min_x, max_x + 1)
            for bucket_y in range(min_y, max_y + 1)
            for node_id in self.buckets.get((bucket_x, bucket_y), [])
        ]


@dataclass
class RefinedTopology:
    artifact: topology.TopologyArtifact
    nodes: Dict[int, RefinedNode]
    edges: Dict[str, RefinedEdge]
    metadata: Dict[str, Any]
    adjacency: Dict[int, List[Tuple[int, RefinedEdge, bool]]] = field(default_factory=dict)
    attachment_index: Any = field(default=None, repr=False, compare=False)
    edge_safety_certificates: Dict[str, bool] = field(default_factory=dict, repr=False, compare=False)

    def __post_init__(self) -> None:
        if not self.adjacency:
            self.adjacency = {int(node_id): [] for node_id in self.nodes}
            for edge in self.edges.values():
                self.adjacency.setdefault(int(edge.source_node), []).append((int(edge.target_node), edge, False))
                self.adjacency.setdefault(int(edge.target_node), []).append((int(edge.source_node), edge, True))
            for values in self.adjacency.values():
                values.sort(key=lambda item: (int(item[0]), str(item[1].edge_id), bool(item[2])))
        if not self.edge_safety_certificates:
            self.edge_safety_certificates = {str(edge_id): True for edge_id in self.edges}
        if self.attachment_index is None:
            self.attachment_index = RefinedNodeSpatialIndex.build(self.nodes)

    @property
    def node_count(self) -> int:
        return len(self.nodes)

    @property
    def edge_count(self) -> int:
        return len(self.edges)


@dataclass(frozen=True)
class Layered2DResult:
    success: bool
    points: Optional[List[Dict[str, Any]]] = None
    failure_code: str = ""
    snapshot_id: str = ""
    diagnostics: Mapping[str, Any] = field(default_factory=dict)


def _footprint_hash(footprint: Sequence[Sequence[float]]) -> str:
    payload = json.dumps([[float(x), float(y)] for x, y in footprint], separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _source_hash() -> str:
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def _atomic_write_text(path: Path, content: str) -> None:
    """Commit a cache artifact atomically so readers never see a partial file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def _path_hash(points: Sequence[Mapping[str, Any]]) -> str:
    normalized = [{key: value for key, value in point.items() if key != "path_hash"} for point in points]
    return hashlib.sha256(json.dumps(normalized, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")).hexdigest()


def _enrich_path(points: Optional[List[Dict[str, Any]]], source_commit: Optional[str]) -> str:
    if not points:
        return ""
    for point in points:
        point["source_commit"] = source_commit or "unknown"
    digest = _path_hash(points)
    for point in points:
        point["path_hash"] = digest
    return digest


def _polyline_length(polyline: Sequence[Sequence[float]]) -> float:
    return sum(math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1])) for a, b in zip(polyline, polyline[1:]))


def _polyline_cumulative(polyline: Sequence[Sequence[float]]) -> Tuple[List[float], float]:
    cumulative = [0.0]
    for first, second in zip(polyline, polyline[1:]):
        cumulative.append(cumulative[-1] + math.hypot(float(second[0]) - float(first[0]), float(second[1]) - float(first[1])))
    return cumulative, cumulative[-1]


def _sample_polyline_with_distances(
    polyline: Sequence[Sequence[float]],
    spacing_m: float,
    *,
    bend_angle_rad: float = math.radians(30.0),
) -> Tuple[List[List[float]], List[float]]:
    """Sample a polyline while retaining source arc-length coordinates.

    Refined topology nodes are deliberately sparse, but the edge geometry
    must remain the original skeleton geometry.  Keeping the arc-length
    anchors lets callers split the source polyline without replacing bends by
    a chord between two samples.
    """
    points = [[float(point[0]), float(point[1])] for point in polyline]
    if len(points) < 2:
        return points, [0.0] if points else []
    cumulative, total = _polyline_cumulative(points)
    targets = {0.0, float(total)}
    spacing = max(0.25, float(spacing_m))
    cursor = spacing
    while cursor < total:
        targets.add(float(cursor))
        cursor += spacing
    for index in range(1, len(points) - 1):
        ax, ay = points[index - 1]
        bx, by = points[index]
        cx, cy = points[index + 1]
        first = math.atan2(by - ay, bx - ax)
        second = math.atan2(cy - by, cx - bx)
        delta = abs((second - first + math.pi) % (2.0 * math.pi) - math.pi)
        if delta >= bend_angle_rad:
            targets.add(float(cumulative[index]))
    result: List[List[float]] = []
    result_distances: List[float] = []
    for distance in sorted(targets):
        segment = next((index for index in range(len(cumulative) - 1) if cumulative[index + 1] + 1.0e-9 >= distance), len(cumulative) - 2)
        local = max(0.0, min(1.0, (distance - cumulative[segment]) / max(1.0e-12, cumulative[segment + 1] - cumulative[segment])))
        x = points[segment][0] + local * (points[segment + 1][0] - points[segment][0])
        y = points[segment][1] + local * (points[segment + 1][1] - points[segment][1])
        if not result or math.hypot(x - result[-1][0], y - result[-1][1]) > 1.0e-6:
            result.append([x, y])
            result_distances.append(float(distance))
    return result, result_distances


def _sample_polyline(
    polyline: Sequence[Sequence[float]],
    spacing_m: float,
    *,
    bend_angle_rad: float = math.radians(30.0),
) -> List[List[float]]:
    return _sample_polyline_with_distances(
        polyline, spacing_m, bend_angle_rad=bend_angle_rad,
    )[0]


def _polyline_slice(
    polyline: Sequence[Sequence[float]],
    cumulative: Sequence[float],
    start_distance: float,
    end_distance: float,
) -> List[List[float]]:
    """Return the exact source geometry between two arc-length anchors."""
    if len(polyline) < 2:
        return [[float(point[0]), float(point[1])] for point in polyline]
    first_distance = max(0.0, min(float(start_distance), float(cumulative[-1])))
    last_distance = max(first_distance, min(float(end_distance), float(cumulative[-1])))

    def point_at(distance: float) -> List[float]:
        segment = next(
            (index for index in range(len(cumulative) - 1)
             if cumulative[index + 1] + 1.0e-9 >= distance),
            len(cumulative) - 2,
        )
        fraction = (distance - cumulative[segment]) / max(
            1.0e-12, cumulative[segment + 1] - cumulative[segment],
        )
        return [
            float(polyline[segment][0]) + fraction * (float(polyline[segment + 1][0]) - float(polyline[segment][0])),
            float(polyline[segment][1]) + fraction * (float(polyline[segment + 1][1]) - float(polyline[segment][1])),
        ]

    result = [point_at(first_distance)]
    for index, distance in enumerate(cumulative[1:-1], start=1):
        if first_distance + 1.0e-9 < float(distance) < last_distance - 1.0e-9:
            point = [float(polyline[index][0]), float(polyline[index][1])]
            if math.hypot(point[0] - result[-1][0], point[1] - result[-1][1]) > 1.0e-6:
                result.append(point)
    last = point_at(last_distance)
    if math.hypot(last[0] - result[-1][0], last[1] - result[-1][1]) > 1.0e-6:
        result.append(last)
    return result


def _edge_cells(hospital_map: Any, polyline: Sequence[Sequence[float]]) -> Tuple[Tuple[int, int], ...]:
    cells: set[Tuple[int, int]] = set()
    for first, second in zip(polyline, polyline[1:]):
        start = hospital_map.world_to_cell(float(first[0]), float(first[1]))
        goal = hospital_map.world_to_cell(float(second[0]), float(second[1]))
        if start is None or goal is None:
            continue
        row_delta = int(goal[0]) - int(start[0])
        col_delta = int(goal[1]) - int(start[1])
        steps = max(1, abs(row_delta), abs(col_delta))
        for index in range(steps + 1):
            fraction = index / steps
            cells.add((int(round(start[0] + fraction * row_delta)), int(round(start[1] + fraction * col_delta))))
    return tuple(sorted(cells))


def build_refined_topology(
    artifact: topology.TopologyArtifact,
    *,
    spacing_m: float = DEFAULT_REFINEMENT_SPACING_M,
    support_angle_deg: float = 30.0,
) -> RefinedTopology:
    """Refine long skeleton edges while preserving topology-level scale."""
    original_nodes = {int(node.node_id): node for node in artifact.graph.nodes}
    nodes: Dict[int, RefinedNode] = {
        node_id: RefinedNode(node_id, float(node.x), float(node.y), "original", int(node.component_id), float(node.clearance_m), source_node_id=node_id)
        for node_id, node in original_nodes.items()
    }
    edges: Dict[str, RefinedEdge] = {}
    next_node_id = max(nodes, default=-1) + 1
    resolution = float(artifact.hospital_map.resolution)
    for source_edge in sorted(artifact.graph.edges, key=lambda item: int(item.edge_id)):
        sampled, sampled_distances = _sample_polyline_with_distances(
            source_edge.polyline,
            spacing_m,
            bend_angle_rad=math.radians(float(support_angle_deg)),
        )
        if len(sampled) < 2:
            continue
        source_cumulative, _source_length = _polyline_cumulative(source_edge.polyline)
        sequence = [int(source_edge.source)]
        for point_index, point in enumerate(sampled[1:-1], start=1):
            cell = artifact.hospital_map.world_to_cell(float(point[0]), float(point[1]))
            clearance = float(artifact.distance_m[cell]) if cell is not None else float(source_edge.min_clearance_m)
            role = "turn_support" if point_index < len(sampled) - 1 else "edge_sample"
            nodes[next_node_id] = RefinedNode(next_node_id, point[0], point[1], role, int(original_nodes[source_edge.source].component_id), clearance, source_edge_id=int(source_edge.edge_id))
            sequence.append(next_node_id)
            next_node_id += 1
        sequence.append(int(source_edge.target))
        for part, (left_id, right_id) in enumerate(zip(sequence, sequence[1:])):
            left = sampled[part]
            right = sampled[part + 1]
            # Keep every source skeleton vertex between two refined anchors.
            # A straight chord can cut through a wall at a gentle bend and is
            # unsafe as both a corridor centerline and a static certificate.
            segment = _polyline_slice(
                source_edge.polyline,
                source_cumulative,
                sampled_distances[part],
                sampled_distances[part + 1],
            )
            length = _polyline_length(segment)
            edge_id = f"e{int(source_edge.edge_id)}_{part}"
            cells = _edge_cells(artifact.hospital_map, segment)
            clearance = max(0.0, float(source_edge.min_clearance_m))
            edges[edge_id] = RefinedEdge(
                edge_id=edge_id, source_node=left_id, target_node=right_id, polyline=segment,
                length_m=length, min_clearance_m=clearance,
                mean_clearance_m=float(source_edge.mean_clearance_m),
                corridor_width_m=float(source_edge.min_width_m),
                local_tangent=math.atan2(right[1] - left[1], right[0] - left[0]),
                turn_support_nodes=[node_id for node_id in (left_id, right_id) if nodes[node_id].role == "turn_support"],
                corridor_mask_id=f"source_edge_{int(source_edge.edge_id)}",
                static_cost=length + (0.25 / max(0.05, clearance)),
                source_edge_id=int(source_edge.edge_id), edge_cells=cells,
            )
    metadata = {
        "schema_version": 1,
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "refined_topology_version": "refined_skeleton_v1",
        "refinement_spacing_m": float(spacing_m),
        "support_angle_deg": float(support_angle_deg),
        "source_topology_algorithm": artifact.metadata.get("algorithm", topology.TOPOLOGY_ALGORITHM_VERSION),
        "skeleton_backend": artifact.metadata.get("skeleton_backend", "unknown"),
        "graph_nodes": len(nodes), "graph_edges": len(edges),
        "static_map_hash": artifact.metadata.get("map_sha256", ""),
        "resolution": resolution,
    }
    return RefinedTopology(artifact, nodes, edges, metadata)


def save_refined_topology(refined: RefinedTopology, directory: str | Path) -> Path:
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": 1,
        "metadata": refined.metadata,
        "nodes": [asdict(node) for node in refined.nodes.values()],
        "edges": [asdict(edge) for edge in refined.edges.values()],
        "edge_safety_certificates": refined.edge_safety_certificates,
        "attachment_index": {
            "cell_size_m": float(refined.attachment_index.cell_size_m),
            "buckets": {f"{key[0]},{key[1]}": list(values) for key, values in refined.attachment_index.buckets.items()},
        } if isinstance(refined.attachment_index, RefinedNodeSpatialIndex) else None,
    }
    # Files are committed independently before the manifest is published. A
    # reader only treats a cache as valid when both payload files and the
    # manifest are present, so an interrupted build cannot become a hit.
    _atomic_write_text(directory / "refined_topology.json", json.dumps(payload, indent=2, sort_keys=True))
    _atomic_write_text(directory / "refined_topology_metadata.yaml", yaml.safe_dump(refined.metadata, sort_keys=False))
    return directory


def load_refined_topology(artifact: topology.TopologyArtifact, directory: str | Path, expected_metadata: Optional[Mapping[str, Any]] = None) -> RefinedTopology:
    directory = Path(directory).resolve()
    payload = json.loads((directory / "refined_topology.json").read_text(encoding="utf-8"))
    metadata = dict(payload.get("metadata") or {})
    if expected_metadata:
        for key, expected in expected_metadata.items():
            if metadata.get(key) != expected:
                raise ValueError(f"refined topology cache is stale for {key}: {metadata.get(key)!r} != {expected!r}")
    nodes = {int(item["node_id"]): RefinedNode(**item) for item in payload.get("nodes", [])}
    edges = {}
    for item in payload.get("edges", []):
        item = dict(item)
        item["edge_cells"] = tuple(tuple(int(value) for value in cell) for cell in item.get("edge_cells", []))
        edges[str(item["edge_id"])] = RefinedEdge(**item)
    index_payload = payload.get("attachment_index") or {}
    attachment_index = None
    if index_payload:
        buckets: Dict[Tuple[int, int], List[int]] = {}
        for key, values in (index_payload.get("buckets") or {}).items():
            try:
                bucket = tuple(int(value) for value in str(key).split(",", 1))
            except (TypeError, ValueError):
                continue
            if len(bucket) == 2:
                buckets[(bucket[0], bucket[1])] = [int(value) for value in values]
        attachment_index = RefinedNodeSpatialIndex(float(index_payload.get("cell_size_m", 5.0)), buckets)
    return RefinedTopology(artifact, nodes, edges, metadata, attachment_index=attachment_index, edge_safety_certificates={str(k): bool(v) for k, v in (payload.get("edge_safety_certificates") or {}).items()})


def prepare_refined_topology(
    artifact: topology.TopologyArtifact,
    footprint: Sequence[Sequence[float]],
    cache_root: str | Path,
    *,
    spacing_m: float = DEFAULT_REFINEMENT_SPACING_M,
    support_angle_deg: float = 30.0,
    padding_m: float = 0.05,
    safety_margin_m: float = 0.05,
    allow_unknown: bool = False,
    planner_parameter_profile: str = "lighter_smoother",
) -> Tuple[RefinedTopology, Dict[str, Any]]:
    """Load or build a metadata-bound refined topology in a new cache root."""
    expected = {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "refined_topology_version": "refined_skeleton_v1",
        "static_map_hash": artifact.metadata.get("map_sha256", ""),
        "map_id": artifact.hospital_map.map_id,
        "resolution": float(artifact.hospital_map.resolution),
        "map_yaml_hash": hashlib.sha256(Path(artifact.hospital_map.yaml_path).read_bytes()).hexdigest() if getattr(artifact.hospital_map, "yaml_path", None) and Path(artifact.hospital_map.yaml_path).exists() else "",
        "footprint_hash": _footprint_hash(footprint),
        "padding_m": float(padding_m),
        "safety_margin_m": float(safety_margin_m),
        "allow_unknown": bool(allow_unknown),
        "skeleton_backend": artifact.metadata.get("skeleton_backend", "unknown"),
        "source_topology_algorithm": artifact.metadata.get("algorithm", topology.TOPOLOGY_ALGORITHM_VERSION),
        "source_hash": _source_hash(),
        "graph_dstar_source_hash": hashlib.sha256(Path(__file__).with_name("graph_dstar_lite.py").read_bytes()).hexdigest(),
        "planner_parameter_profile": str(planner_parameter_profile),
        "refinement_spacing_m": float(spacing_m),
        "support_angle_deg": float(support_angle_deg),
    }
    key = hashlib.sha256(json.dumps(expected, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()
    directory = Path(cache_root).resolve() / artifact.hospital_map.map_id / key
    manifest = directory / "refined_cache_manifest.yaml"
    started = time.monotonic_ns()
    miss_reason = "manifest_missing"
    if manifest.exists():
        try:
            stored = yaml.safe_load(manifest.read_text(encoding="utf-8")) or {}
            payload_path = directory / "refined_topology.json"
            metadata_path = directory / "refined_topology_metadata.yaml"
            if stored.get("cache_key") == key and stored.get("metadata") == expected and payload_path.is_file() and metadata_path.is_file():
                refined = load_refined_topology(artifact, directory, expected)
                return refined, {**expected, "topology_cache_key": key, "topology_cache_hit": True, "cache_state": "cache_hit", "cache_miss": False, "cache_invalidated": False, "cache_rebuild": False, "refined_topology_load_time_ms": (time.monotonic_ns() - started) / 1.0e6, "refined_topology_build_count": 0, "refined_topology_cache_directory": str(directory), "cache_miss_reason": ""}
            miss_reason = "metadata_mismatch"
        except (OSError, ValueError, KeyError, json.JSONDecodeError, yaml.YAMLError):
            miss_reason = "cache_corrupt_or_stale"
    elif directory.parent.exists():
        # A different key under the same map/cache root means the inputs
        # changed (for example refinement spacing or footprint), rather than
        # a first-ever build. Keep this distinction visible in the manifest.
        try:
            if any(item.is_dir() and (item / "refined_cache_manifest.yaml").exists() for item in directory.parent.iterdir()):
                miss_reason = "metadata_mismatch"
        except OSError:
            pass
    directory.mkdir(parents=True, exist_ok=True)
    build_started = time.monotonic_ns()
    refined = build_refined_topology(artifact, spacing_m=spacing_m, support_angle_deg=support_angle_deg)
    refined.metadata.update(expected)
    save_refined_topology(refined, directory)
    _atomic_write_text(manifest, yaml.safe_dump({"schema_version": 1, "cache_key": key, "metadata": expected, "cache_state": "cache_rebuild", "cache_miss_reason": miss_reason}, sort_keys=False))
    return refined, {**expected, "topology_cache_key": key, "topology_cache_hit": False, "cache_state": "cache_rebuild", "cache_miss": True, "cache_invalidated": miss_reason != "manifest_missing", "cache_rebuild": True, "refined_topology_build_time_ms": (time.monotonic_ns() - build_started) / 1.0e6, "refined_topology_load_time_ms": 0.0, "refined_topology_build_count": 1, "refined_topology_cache_directory": str(directory), "cache_miss_reason": miss_reason}


def _raw_free_mask(artifact: topology.TopologyArtifact) -> np.ndarray:
    occupancy = np.asarray(artifact.hospital_map.occupancy)
    return (occupancy >= 0) & (occupancy < 100)


def _topology_source_hash() -> str:
    return hashlib.sha256(Path(topology.__file__).read_bytes()).hexdigest()


def prepare_static_topology(
    hospital_map: Any,
    footprint: Sequence[Sequence[float]],
    cache_root: str | Path,
    *,
    padding_m: float = 0.05,
    safety_margin_m: float = 0.05,
    allow_unknown: bool = False,
) -> Tuple[topology.TopologyArtifact, Dict[str, Any]]:
    """Load/build the static topology once under a metadata-bound cache key."""
    map_hash = topology.map_input_hash(hospital_map.yaml_path, hospital_map.image_path)
    yaml_hash = hashlib.sha256(Path(hospital_map.yaml_path).read_bytes()).hexdigest()
    image_hash = hashlib.sha256(Path(hospital_map.image_path).read_bytes()).hexdigest()
    skeleton_backend = "scikit-image" if topology._has_skimage() else "numpy_zhang_suen"
    expected = {
        "map_id": str(hospital_map.map_id),
        "map_sha256": map_hash,
        "map_yaml_sha256": yaml_hash,
        "map_image_sha256": image_hash,
        "resolution": float(hospital_map.resolution),
        "width": int(hospital_map.width),
        "height": int(hospital_map.height),
        "origin": [float(value) for value in hospital_map.origin],
        "footprint_hash": topology.footprint_hash(footprint),
        "padding_m": float(padding_m),
        "safety_margin_m": float(safety_margin_m),
        "allow_unknown": bool(allow_unknown),
        "algorithm": topology.TOPOLOGY_ALGORITHM_VERSION,
        "skeleton_backend": skeleton_backend,
        "source_hash": _topology_source_hash(),
    }
    key = hashlib.sha256(json.dumps(expected, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()
    directory = Path(cache_root).resolve() / str(hospital_map.map_id) / key
    manifest = directory / "topology_cache_manifest.yaml"
    started = time.monotonic_ns()
    miss_reason = "manifest_missing"
    if manifest.exists():
        try:
            stored = yaml.safe_load(manifest.read_text(encoding="utf-8")) or {}
            if stored.get("cache_key") == key and stored.get("metadata") == expected:
                artifact = topology.load_topology(
                    directory, hospital_map, footprint,
                    padding_m=padding_m, safety_margin_m=safety_margin_m,
                    allow_unknown=allow_unknown,
                )
                artifact.graph.adjacency()  # materialize once for all queries
                return artifact, {
                    **expected, "topology_cache_key": key, "topology_cache_hit": True,
                    "topology_load_time_ms": (time.monotonic_ns() - started) / 1.0e6,
                    "topology_build_time_ms": 0.0, "topology_build_count": 0,
                    "topology_load_count": 1,
                    "topology_cache_directory": str(directory), "cache_miss_reason": "",
                }
            miss_reason = "metadata_mismatch"
        except (OSError, ValueError, KeyError, json.JSONDecodeError, yaml.YAMLError):
            miss_reason = "cache_corrupt_or_stale"
    directory.mkdir(parents=True, exist_ok=True)
    build_started = time.monotonic_ns()
    artifact = topology.build_topology(
        hospital_map, footprint, padding_m=padding_m,
        safety_margin_m=safety_margin_m, allow_unknown=allow_unknown,
    )
    artifact.graph.adjacency()
    artifact.metadata.update(expected)
    topology.save_topology(artifact, directory)
    manifest.write_text(yaml.safe_dump({"schema_version": 1, "cache_key": key, "metadata": expected}, sort_keys=False), encoding="utf-8")
    return artifact, {
        **expected, "topology_cache_key": key, "topology_cache_hit": False,
        "topology_load_time_ms": 0.0,
        "topology_build_time_ms": (time.monotonic_ns() - build_started) / 1.0e6,
        "topology_build_count": 1, "topology_cache_directory": str(directory),
        "cache_miss_reason": miss_reason,
    }


def corridor_mask_for_route(
    refined: RefinedTopology,
    route_node_ids: Sequence[int],
    start_pose: Sequence[float],
    goal_pose: Sequence[float],
    *,
    padding_m: float = DEFAULT_CORRIDOR_PADDING_M,
    virtual_positions: Optional[Mapping[int, Tuple[float, float]]] = None,
) -> np.ndarray:
    """Rasterize a route corridor from raw occupancy, with one Smac inflation."""
    hospital_map = refined.artifact.hospital_map
    raw_free = _raw_free_mask(refined.artifact)
    centerline = np.zeros(raw_free.shape, dtype=np.uint8)
    positions: Dict[int, Tuple[float, float]] = {node_id: (node.x, node.y) for node_id, node in refined.nodes.items()}
    if virtual_positions:
        positions.update({int(node_id): (float(value[0]), float(value[1])) for node_id, value in virtual_positions.items()})
    for first_id, second_id in zip(route_node_ids, route_node_ids[1:]):
        geometry: Optional[List[List[float]]] = None
        for target, edge, reverse in refined.adjacency.get(int(first_id), []):
            if int(target) == int(second_id):
                geometry = [list(point) for point in (reversed(edge.polyline) if reverse else edge.polyline)]
                break
        if geometry is None:
            first = positions.get(int(first_id)); second = positions.get(int(second_id))
            if first is not None and second is not None:
                geometry = [list(first), list(second)]
        if not geometry:
            continue
        for first, second in zip(geometry, geometry[1:]):
            a = hospital_map.world_to_cell(float(first[0]), float(first[1]))
            b = hospital_map.world_to_cell(float(second[0]), float(second[1]))
            if a is not None and b is not None:
                cv2.line(centerline, (int(a[1]), int(a[0])), (int(b[1]), int(b[0])), 1, 1)
    for pose in (start_pose, goal_pose):
        cell = hospital_map.world_to_cell(float(pose[0]), float(pose[1]))
        if cell is not None:
            centerline[cell] = 1
    for pose, backward, forward in ((start_pose, 0.25, 0.75), (goal_pose, 0.75, 0.25)):
        direction = (math.cos(float(pose[2])), math.sin(float(pose[2])))
        a = (float(pose[0]) - backward * direction[0], float(pose[1]) - backward * direction[1])
        b = (float(pose[0]) + forward * direction[0], float(pose[1]) + forward * direction[1])
        ca = hospital_map.world_to_cell(*a); cb = hospital_map.world_to_cell(*b)
        if ca is not None and cb is not None:
            cv2.line(centerline, (int(ca[1]), int(ca[0])), (int(cb[1]), int(cb[0])), 1, 1)
    radius_cells = max(1, int(math.ceil(float(padding_m) / max(1.0e-9, float(hospital_map.resolution)))))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius_cells + 1, 2 * radius_cells + 1))
    return (cv2.dilate(centerline, kernel, iterations=1) > 0) & raw_free


def _connection_safe(hospital_map: Any, first: Sequence[float], second: Sequence[float], footprint: Sequence[Sequence[float]], radius_m: float) -> bool:
    distance = math.hypot(float(second[0]) - float(first[0]), float(second[1]) - float(first[1]))
    steps = max(1, int(math.ceil(distance / max(0.05, float(radius_m)))))
    for index in range(steps + 1):
        fraction = index / steps
        pose = (float(first[0]) + fraction * (float(second[0]) - float(first[0])), float(first[1]) + fraction * (float(second[1]) - float(first[1])), float(first[2]) if len(first) > 2 else math.atan2(float(second[1]) - float(first[1]), float(second[0]) - float(first[0])))
        if hospital_map.footprint_collision(pose, footprint, unknown_is_collision=True):
            return False
    return True


def dynamic_collision_count(hospital_map: Any, points: Sequence[Mapping[str, Any]], footprint: Sequence[Sequence[float]], snapshot: DynamicSnapshot) -> int:
    """Count sampled footprint poses intersecting the dynamic occupied layer."""
    occupied = set(snapshot.inflated_cells(0))
    collisions = 0
    for point in points:
        x, y, yaw = float(point["x"]), float(point["y"]), float(point.get("yaw", 0.0))
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        transformed = [(x + cos_yaw * float(px) - sin_yaw * float(py), y + sin_yaw * float(px) + cos_yaw * float(py)) for px, py in footprint]
        if not transformed:
            continue
        min_x = min(value[0] for value in transformed) - float(hospital_map.resolution)
        max_x = max(value[0] for value in transformed) + float(hospital_map.resolution)
        min_y = min(value[1] for value in transformed) - float(hospital_map.resolution)
        max_y = max(value[1] for value in transformed) + float(hospital_map.resolution)
        min_cell = hospital_map.world_to_cell(min_x, min_y); max_cell = hospital_map.world_to_cell(max_x, max_y)
        hit = False
        if min_cell is not None and max_cell is not None:
            for row in range(min(min_cell[0], max_cell[0]), max(min_cell[0], max_cell[0]) + 1):
                for col in range(min(min_cell[1], max_cell[1]), max(min_cell[1], max_cell[1]) + 1):
                    if (row, col) in occupied:
                        center = hospital_map.cell_to_world((row, col))
                        if cv2.pointPolygonTest(np.asarray(transformed, dtype=np.float32), (float(center[0]), float(center[1])), False) >= 0:
                            hit = True
                            break
                if hit:
                    break
        collisions += int(hit)
    return collisions


def _session_log_cursor(session: Any) -> int:
    path = getattr(getattr(session, "stack", None), "log_file", None)
    if path is None:
        return 0
    try:
        return int(Path(path).stat().st_size)
    except OSError:
        return 0


def _session_log_delta(session: Any, cursor: int) -> str:
    path = getattr(getattr(session, "stack", None), "log_file", None)
    if path is None:
        return ""
    try:
        with Path(path).open("rb") as stream:
            stream.seek(max(0, int(cursor)))
            return stream.read().decode("utf-8", errors="replace")
    except OSError:
        return ""


def _classify_smac_failure(
    result_code: str,
    result_diagnostics: Mapping[str, Any],
    log_delta: str = "",
) -> Tuple[str, Any, str]:
    """Classify a Smac failure without inferring unknown action outcomes.

    The planner log is authoritative for Smac-specific failures.  When no
    planner evidence is available, an aborted action is explicitly reported as
    ``ACTION_ABORTED_UNKNOWN`` and the search-start flag remains unknown.
    """
    details = " ".join(
        str(result_diagnostics.get(key, ""))
        for key in ("failure_detail", "error_message", "planner_error", "smac_failure_detail")
    )
    text = f"{log_delta or ''} {details}".lower()
    code = str(result_code or "")
    direct_codes = {
        "START_IN_LETHAL_SPACE": ("START_IN_LETHAL_SPACE", False, "Smac reported starting point in lethal space"),
        "GOAL_IN_LETHAL_SPACE": ("GOAL_IN_LETHAL_SPACE", False, "Smac reported goal point in lethal space"),
        "NO_PATH_IN_CORRIDOR": ("NO_PATH_IN_CORRIDOR", True, "Smac reported no valid path"),
        "SMAC_MAX_ITERATIONS": ("SMAC_MAX_ITERATIONS", True, "Smac exceeded maximum iterations"),
        "PLANNER_TIMEOUT": ("PLANNER_TIMEOUT", "not_available", "Smac or action timeout"),
        "COSTMAP_UPDATE_TIMEOUT": ("COSTMAP_UPDATE_TIMEOUT", False, "Costmap update acknowledgement timed out"),
    }
    if code in direct_codes:
        return direct_codes[code]
    if code in {"SERVER_UNAVAILABLE", "ACTION_REJECTED", "BACKEND_UNAVAILABLE"}:
        return "BACKEND_UNAVAILABLE", False, details or f"Smac action unavailable: {code}"
    if code == "COSTMAP_UPDATE_TIMEOUT" or ("costmap" in text and "timeout" in text):
        return "COSTMAP_UPDATE_TIMEOUT", False, "Costmap update acknowledgement timed out"
    if "starting point in lethal space" in text or ("starting point" in text and "lethal" in text):
        return "START_IN_LETHAL_SPACE", False, "Smac reported starting point in lethal space"
    if "goal point in lethal space" in text or ("goal point" in text and "lethal" in text):
        return "GOAL_IN_LETHAL_SPACE", False, "Smac reported goal point in lethal space"
    if any(token in text for token in ("exceeded maximum iterations", "maximum iterations", "max iterations", "iteration limit")):
        return "SMAC_MAX_ITERATIONS", True, "Smac exceeded maximum iterations"
    if any(token in text for token in ("no valid path", "cannot create feasible plan", "failed to generate a valid path")):
        return "NO_PATH_IN_CORRIDOR", True, "Smac reported no valid path"
    if code in {"CLIENT_TIMEOUT", "PLANNER_TIMEOUT"} or "timed out" in text or "timeout" in text:
        return "PLANNER_TIMEOUT", "not_available", "Smac or action timeout"
    if code == "ACTION_ABORTED":
        return "ACTION_ABORTED_UNKNOWN", "not_available", "Nav2 action returned ABORTED without a recognized planner reason"
    if code and code not in {"SUCCEEDED", ""}:
        return "ACTION_ABORTED_UNKNOWN", "not_available", f"Unrecognized Smac result code={code}"
    return "ACTION_ABORTED_UNKNOWN", "not_available", "Smac returned no successful path"


def attachment_candidates(
    refined: RefinedTopology,
    pose: Sequence[float],
    footprint: Sequence[Sequence[float]],
    *,
    radius_m: float = DEFAULT_ENDPOINT_RADIUS_M,
    limit: int = DEFAULT_CANDIDATE_LIMIT,
    snapshot: Optional[DynamicSnapshot] = None,
) -> List[AttachmentCandidate]:
    """Return deterministic node, edge-projection and support-node candidates."""
    hospital_map = refined.artifact.hospital_map
    dynamic_cells = set(snapshot.inflated_cells(0)) if snapshot else set()
    candidates: List[AttachmentCandidate] = []
    seen: set[Tuple[int, int, str]] = set()
    next_virtual = -1
    indexed_node_ids: Iterable[int]
    if isinstance(refined.attachment_index, RefinedNodeSpatialIndex):
        indexed_node_ids = refined.attachment_index.query(float(pose[0]), float(pose[1]), float(radius_m))
    else:
        indexed_node_ids = sorted(refined.nodes)
    for node_id in indexed_node_ids:
        node = refined.nodes.get(int(node_id))
        if node is None:
            continue
        distance = math.hypot(float(node.x) - float(pose[0]), float(node.y) - float(pose[1]))
        node_cell = hospital_map.world_to_cell(float(node.x), float(node.y))
        if distance > float(radius_m) or (node_cell is not None and tuple(node_cell) in dynamic_cells):
            continue
        if hospital_map.footprint_collision((node.x, node.y, float(pose[2])), footprint, unknown_is_collision=True):
            continue
        if not _connection_safe(hospital_map, pose, (node.x, node.y, float(pose[2])), footprint, hospital_map.resolution):
            continue
        # Use the smallest tangent error of the incident topology edges as a
        # real attachment prior. This does not add yaw to the L1 state space.
        incident_tangents: List[float] = []
        for _neighbor, incident_edge, reverse in refined.adjacency.get(int(node_id), []):
            tangent = float(incident_edge.local_tangent)
            if reverse:
                tangent = (tangent + math.pi) % (2.0 * math.pi)
            incident_tangents.append(tangent)
        if incident_tangents:
            heading_error = min(
                abs((tangent - float(pose[2]) + math.pi) % (2.0 * math.pi) - math.pi)
                for tangent in incident_tangents
            )
        else:
            heading_error = math.pi
        key = (int(round(node.x / 0.05)), int(round(node.y / 0.05)), "node")
        if key in seen:
            continue
        seen.add(key)
        penalty = distance + 0.5 * abs(heading_error) + 0.1 / max(0.05, node.clearance_m)
        candidates.append(AttachmentCandidate(next_virtual, node.x, node.y, distance, heading_error, node.component_id, node.role, ((node_id, penalty),)))
        next_virtual -= 1
    # Edge projections are bounded and deterministic; unlike a grid search,
    # this loop is over the refined topology edge set only.
    for edge_id, edge in sorted(refined.edges.items()):
        best: Optional[Tuple[float, float, float, float]] = None
        for first, second in zip(edge.polyline, edge.polyline[1:]):
            dx = float(second[0]) - float(first[0]); dy = float(second[1]) - float(first[1])
            denominator = dx * dx + dy * dy
            fraction = 0.0 if denominator <= 1.0e-12 else max(0.0, min(1.0, ((float(pose[0]) - first[0]) * dx + (float(pose[1]) - first[1]) * dy) / denominator))
            x = first[0] + fraction * dx; y = first[1] + fraction * dy
            distance = math.hypot(x - float(pose[0]), y - float(pose[1]))
            tangent = math.atan2(dy, dx)
            heading_error = abs((tangent - float(pose[2]) + math.pi) % (2.0 * math.pi) - math.pi)
            if best is None or (distance, edge_id) < (best[0], edge_id):
                best = (distance, x, y, tangent)
        if best is None or best[0] > float(radius_m):
            continue
        distance, x, y, tangent = best
        if hospital_map.footprint_collision((x, y, float(pose[2])), footprint, unknown_is_collision=True):
            continue
        if not _connection_safe(hospital_map, pose, (x, y, float(pose[2])), footprint, hospital_map.resolution):
            continue
        key = (int(round(x / 0.05)), int(round(y / 0.05)), "edge")
        if key in seen:
            continue
        seen.add(key)
        source = refined.nodes[edge.source_node]
        target = refined.nodes[edge.target_node]
        attach_cost = distance + 0.5 * heading_error + 0.1 / max(0.05, edge.min_clearance_m)
        candidates.append(AttachmentCandidate(next_virtual, x, y, distance, heading_error, source.component_id, "edge_projection", ((edge.source_node, attach_cost + 0.5 * edge.length_m), (edge.target_node, attach_cost + 0.5 * edge.length_m))))
        next_virtual -= 1
    candidates.sort(key=lambda item: (item.distance_m + item.heading_error_rad, item.candidate_id))
    return candidates[:max(1, int(limit))]


class SmacHybridAdapter:
    """Use an existing map-level SmacSession for one whole corridor."""

    def __init__(self, session: Any, backend_spec: Any, *, footprint: Sequence[Sequence[float]], source_commit: Optional[str] = None, force_full_update: bool = False) -> None:
        self.session = session
        self.backend_spec = backend_spec
        self.footprint = footprint
        self.source_commit = source_commit
        self.force_full_update = bool(force_full_update)
        self.calls = 0

    def plan(self, query: Any, corridor_mask: np.ndarray, snapshot: DynamicSnapshot, *, topology_artifact: RefinedTopology) -> Tuple[Any, Dict[str, Any]]:
        if self.session is None or not hasattr(self.session, "plan"):
            return None, {"backend_called": False, "planner_search_started": False, "failure_code": "BACKEND_UNAVAILABLE", "action_status": "not_available", "action_result_code": "BACKEND_UNAVAILABLE", "smac_log_excerpt": ""}
        static_free = _raw_free_mask(topology_artifact.artifact)
        allowed = np.asarray(corridor_mask, dtype=bool) & static_free
        for row, col in snapshot.inflated_cells(0):
            if 0 <= row < allowed.shape[0] and 0 <= col < allowed.shape[1]:
                allowed[row, col] = False
        started = time.monotonic_ns()
        log_cursor = _session_log_cursor(self.session)
        self.calls += 1
        try:
            result = self.session.plan(query, self.backend_spec, source="2d_v0_l3_smac", allowed_mask=allowed, force_full_update=self.force_full_update)
        except Exception as exc:  # pragma: no cover - ROS-specific
            log_delta = _session_log_delta(self.session, log_cursor)
            failure_code, search_started, failure_detail = _classify_smac_failure("ACTION_ABORTED", {"failure_detail": str(exc)}, log_delta)
            return None, {"backend_called": True, "planner_search_started": search_started, "failure_code": failure_code, "failure_detail": failure_detail, "action_status": "EXCEPTION", "action_result_code": "ACTION_ABORTED", "smac_log_excerpt": log_delta[-2000:], "smac_log_delta": log_delta, "planner_time_ms": (time.monotonic_ns() - started) / 1.0e6}
        if result is not None and getattr(result, "points", None):
            result.points = [dict(point) for point in result.points]
            path_hash = _enrich_path(result.points, self.source_commit)
        else:
            path_hash = ""
        diagnostics = dict(getattr(result, "diagnostics", {}) or {})
        log_delta = _session_log_delta(self.session, log_cursor)
        raw_action_status = diagnostics.get("action_status", "not_available")
        raw_result_code = diagnostics.get("action_result_code") or getattr(result, "failure_code", "") or ("SUCCEEDED" if getattr(result, "planner_success", False) else "")
        classified_code = ""
        classified_started: Any = diagnostics.get("planner_search_started", "not_available")
        classified_detail = ""
        if not bool(getattr(result, "planner_success", False) and getattr(result, "points", None)):
            classified_code, classified_started, classified_detail = _classify_smac_failure(raw_result_code, diagnostics, log_delta)
            # Keep the structured reason on the PlanResult as well as in the
            # diagnostics map so callers cannot accidentally fall back to the
            # generic ACTION_ABORTED value.
            try:
                result.failure_code = classified_code
                result.failure_detail = classified_detail
            except AttributeError:
                pass
        diagnostics.update({
            "backend_called": True,
            "planner_search_started": classified_started if classified_code else classified_started,
            "l3_call_count": self.calls,
            "action_status": raw_action_status,
            "action_result_code": raw_result_code,
            "smac_failure_code": classified_code or getattr(result, "failure_code", "") or "",
            "failure_code": classified_code,
            "failure_detail": classified_detail or diagnostics.get("failure_detail", ""),
            "smac_log_excerpt": log_delta[-2000:],
            "smac_log_delta": log_delta,
            "smac_log_path": str(getattr(getattr(self.session, "stack", None), "log_file", "") or ""),
            "l3_query_call_count": int(diagnostics.get("backend_call_count", 1) or 1),
            "path_hash": path_hash,
            "planner_wall_time_ms": diagnostics.get("wall_time_ms", (time.monotonic_ns() - started) / 1.0e6),
        })
        return result, diagnostics


def _route_edge_ids(refined: RefinedTopology, node_path: Sequence[int]) -> List[str]:
    edge_ids: List[str] = []
    for first, second in zip(node_path, node_path[1:]):
        candidates = [edge for target, edge, _reverse in refined.adjacency.get(int(first), []) if int(target) == int(second)]
        if candidates:
            edge_ids.append(min(candidates, key=lambda edge: (edge.length_m, str(edge.edge_id))).edge_id)
    return edge_ids


def _route_polyline(refined: RefinedTopology, node_path: Sequence[int], start_pose: Sequence[float], goal_pose: Sequence[float], virtual_positions: Mapping[int, Tuple[float, float]]) -> List[List[float]]:
    points: List[List[float]] = [[float(start_pose[0]), float(start_pose[1])]]
    positions: Dict[int, Tuple[float, float]] = {
        node_id: (node.x, node.y) for node_id, node in refined.nodes.items()
    }
    positions.update({int(node_id): (float(value[0]), float(value[1])) for node_id, value in virtual_positions.items()})
    for first_id, second_id in zip(node_path, node_path[1:]):
        geometry: Optional[List[List[float]]] = None
        for target, edge, reverse in refined.adjacency.get(int(first_id), []):
            if int(target) == int(second_id):
                geometry = [list(point) for point in (reversed(edge.polyline) if reverse else edge.polyline)]
                break
        if geometry is None:
            first = positions.get(int(first_id)); second = positions.get(int(second_id))
            if first is not None and second is not None:
                geometry = [list(first), list(second)]
        for position in (geometry or [])[1:]:
            if not points or math.hypot(float(position[0]) - points[-1][0], float(position[1]) - points[-1][1]) > 1.0e-6:
                points.append([float(position[0]), float(position[1])])
    points.append([float(goal_pose[0]), float(goal_pose[1])])
    return points


def _route_min_clearance(
    refined: RefinedTopology,
    edge_ids: Sequence[str],
    start_pose: Sequence[float],
    goal_pose: Sequence[float],
) -> float:
    """Report clearance of the selected centerline, not obstacle-adjacent mask cells."""
    values = [
        float(refined.edges[edge_id].min_clearance_m)
        for edge_id in edge_ids
        if edge_id in refined.edges and math.isfinite(float(refined.edges[edge_id].min_clearance_m))
    ]
    hospital_map = refined.artifact.hospital_map
    clearance_fn = getattr(hospital_map, "clearance", None)
    if callable(clearance_fn):
        for pose in (start_pose, goal_pose):
            clearance = clearance_fn(float(pose[0]), float(pose[1]))
            if clearance is not None and math.isfinite(float(clearance)):
                values.append(float(clearance))
    return max(0.0, min(values)) if values else 0.0


class Layered2DV0Pipeline:
    """Initial and incremental planning for the independent 2D-V0 candidate."""

    def __init__(
        self,
        refined: RefinedTopology,
        *,
        footprint: Sequence[Sequence[float]],
        l3_planner: Optional[Any] = None,
        corridor_padding_m: float = DEFAULT_CORRIDOR_PADDING_M,
        endpoint_radius_m: float = DEFAULT_ENDPOINT_RADIUS_M,
        candidate_limit: int = DEFAULT_CANDIDATE_LIMIT,
        corridor_profile: str = "padding",
        corridor_fallback_policy: str = "bounded",
        corridor_extra_margin_m: float = 0.0,
        validator: Optional[Callable[[Any, Any, Optional[Sequence[Mapping[str, Any]]]], Mapping[str, Any]]] = None,
    ) -> None:
        self.refined = refined
        self.footprint = footprint
        self.l3_planner = l3_planner
        self.corridor_padding_m = float(corridor_padding_m)
        self.endpoint_radius_m = float(endpoint_radius_m)
        self.candidate_limit = max(2, int(candidate_limit))
        self.corridor_profile = str(corridor_profile)
        if str(corridor_fallback_policy) not in {"bounded", "none"}:
            raise ValueError("corridor_fallback_policy must be 'bounded' or 'none'")
        self.corridor_fallback_policy = str(corridor_fallback_policy)
        self.corridor_extra_margin_m = max(0.0, float(corridor_extra_margin_m))
        self.validator = validator
        self.current_snapshot = DynamicSnapshot.empty(snapshot_id="static", map_shape=refined.artifact.free_mask.shape)
        self.edge_status: Dict[str, str] = {edge_id: GraphDStarLite.AVAILABLE for edge_id in refined.edges}
        self.edge_cost_override: Dict[str, float] = {}
        self.dstar: Optional[GraphDStarLite] = None
        self._virtual_positions: Dict[int, Tuple[float, float]] = {}
        self._route_node_ids: Optional[List[int]] = None
        self._route_edge_ids: List[str] = []
        self._route_mask: Optional[np.ndarray] = None
        self._last_result: Optional[Layered2DResult] = None
        self._l1_reroute_count = 0
        self._l3_call_count = 0
        self._query_state_key: Optional[Tuple[str, Tuple[float, ...], Tuple[float, ...]]] = None

    def _make_graph(self, starts: Sequence[AttachmentCandidate], goals: Sequence[AttachmentCandidate], snapshot: DynamicSnapshot) -> Tuple[GraphDStarLite, int, int, Dict[int, Tuple[float, float]]]:
        base_nodes = set(self.refined.nodes)
        base_edges = [GraphEdge(edge.edge_id, edge.source_node, edge.target_node, edge.length_m, edge.static_cost, edge.min_clearance_m, bidirectional=True) for edge in self.refined.edges.values()]
        virtual_positions: Dict[int, Tuple[float, float]] = {}
        start_virtual = -1000000
        goal_virtual = -2000000
        for index, candidate in enumerate(starts):
            candidate_id = start_virtual - 100 - index
            virtual_positions[candidate_id] = (candidate.x, candidate.y)
            for target, cost in candidate.connections:
                base_edges.append(GraphEdge(f"attach_start_{index}_{target}", candidate_id, int(target), float(cost), bidirectional=False))
            base_nodes.add(candidate_id)
        for index, candidate in enumerate(goals):
            candidate_id = goal_virtual - 100 - index
            virtual_positions[candidate_id] = (candidate.x, candidate.y)
            for target, cost in candidate.connections:
                base_edges.append(GraphEdge(f"attach_goal_{index}_{target}", int(target), candidate_id, float(cost), bidirectional=False))
            base_nodes.add(candidate_id)
        # A virtual root connects all endpoint candidates, allowing one D* Lite
        # search to choose the attachment pair instead of pairwise Graph A*.
        root_edges = []
        for index, candidate in enumerate(starts):
            root_edges.append(GraphEdge(f"root_start_{index}", start_virtual, start_virtual - 100 - index, candidate.distance_m + candidate.heading_error_rad, bidirectional=False))
        for index, candidate in enumerate(goals):
            # Goal endpoint connections point into the goal root and are
            # one-way, preventing reverse endpoint routes.
            root_edges.append(GraphEdge(f"root_goal_{index}", goal_virtual - 100 - index, goal_virtual, candidate.distance_m + candidate.heading_error_rad, bidirectional=False))
        base_edges.extend(root_edges)
        base_nodes.update((start_virtual, goal_virtual))
        planner = GraphDStarLite(base_nodes, base_edges, start_virtual, goal_virtual, edge_status=self.edge_status, edge_cost_override=self.edge_cost_override)
        planner.node_positions = {node_id: (node.x, node.y) for node_id, node in self.refined.nodes.items()}
        planner.node_positions.update(virtual_positions)
        return planner, start_virtual, goal_virtual, virtual_positions

    def _attach(self, query: Any, snapshot: DynamicSnapshot) -> Tuple[List[AttachmentCandidate], List[AttachmentCandidate], Dict[str, Any]]:
        started = time.monotonic_ns()
        starts = attachment_candidates(self.refined, query.start, self.footprint, radius_m=self.endpoint_radius_m, limit=self.candidate_limit, snapshot=snapshot)
        goals = attachment_candidates(self.refined, query.goal, self.footprint, radius_m=self.endpoint_radius_m, limit=self.candidate_limit, snapshot=snapshot)
        def serialize(candidate: AttachmentCandidate) -> Dict[str, Any]:
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
        return starts, goals, {
            "attachment_candidate_count": len(starts) + len(goals),
            "start_attachment_candidate_count": len(starts),
            "goal_attachment_candidate_count": len(goals),
            "attachment_lookup_time_ms": (time.monotonic_ns() - started) / 1.0e6,
            "start_attachment_candidates": [serialize(candidate) for candidate in starts],
            "goal_attachment_candidates": [serialize(candidate) for candidate in goals],
        }

    def _plan_route(self, query: Any, snapshot: DynamicSnapshot, *, reuse_graph: bool = False) -> Tuple[Optional[List[int]], Dict[str, Any]]:
        route_started_ns = time.monotonic_ns()
        reuse_existing_graph = bool(reuse_graph and self.dstar is not None)
        if reuse_existing_graph:
            starts, goals, diagnostics = (), (), {"attachment_candidates_reused": True}
        else:
            starts, goals, diagnostics = self._attach(query, snapshot)
            if not starts or not goals:
                return None, {**diagnostics, "failure_code": "L1_ENDPOINT_NOT_ATTACHABLE"}
        if self.dstar is None:
            self.dstar, _start_virtual, _goal_virtual, self._virtual_positions = self._make_graph(starts, goals, snapshot)
        stats = self.dstar.compute_shortest_path(timeout_s=5.0, max_expansions=DEFAULT_MAX_EXPANSIONS)
        node_path = self.dstar.extract_path()
        diagnostics.update({
            "l1_dstar_initial_time_ms": float(stats.search_time_ms) if self._route_node_ids is None else 0.0,
            "l1_dstar_incremental_time_ms": float(stats.search_time_ms) if self._route_node_ids is not None else 0.0,
            "dstar_expanded_nodes": int(stats.expanded_nodes), "dstar_generated_nodes": int(stats.generated_nodes),
            "dstar_queue_pops": int(stats.queue_pops), "dstar_queue_pushes": int(stats.queue_pushes),
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
            return None, {**diagnostics, "failure_code": "L1_NO_ROUTE" if not stats.timeout_triggered else "L1_DSTAR_TIMEOUT"}
        self._route_node_ids = list(node_path)
        diagnostics["route_construction_time_ms"] = (time.monotonic_ns() - route_started_ns) / 1.0e6 - float(stats.search_time_ms)
        self._route_edge_ids = _route_edge_ids(self.refined, node_path)
        return list(node_path), diagnostics

    def _run_l3(
        self,
        query: Any,
        snapshot: DynamicSnapshot,
        node_path: Sequence[int],
        *,
        validate: bool = True,
        corridor_padding_m: Optional[float] = None,
    ) -> Layered2DResult:
        selected_padding = self.corridor_padding_m if corridor_padding_m is None else float(corridor_padding_m)
        route_points = _route_polyline(self.refined, node_path, query.start, query.goal, self._virtual_positions)
        if self.corridor_profile == "full_map":
            mask = _raw_free_mask(self.refined.artifact).copy()
        else:
            mask = corridor_mask_for_route(
                self.refined, node_path, query.start, query.goal,
                padding_m=selected_padding + self.corridor_extra_margin_m,
                virtual_positions=self._virtual_positions,
            )
        self._route_mask = mask
        mask_hash = hashlib.sha256(np.ascontiguousarray(mask, dtype=np.uint8).tobytes()).hexdigest()
        hospital_map = self.refined.artifact.hospital_map
        raw_free = _raw_free_mask(self.refined.artifact)
        endpoint_fields: Dict[str, Any] = {
            "corridor_profile": self.corridor_profile,
            "corridor_min_clearance_m": _route_min_clearance(
                self.refined, self._route_edge_ids, query.start, query.goal,
            ),
            "corridor_free_cells": int(np.count_nonzero(raw_free)),
            "corridor_mask_hash": mask_hash,
            "start_raw_map_cost": "not_available",
            "goal_raw_map_cost": "not_available",
            "start_inflated_cost": "not_available",
            "goal_inflated_cost": "not_available",
            "smac_start_cost": "not_available",
            "smac_goal_cost": "not_available",
            "start_is_lethal": "not_available",
            "goal_is_lethal": "not_available",
            "start_full_footprint_valid": "not_available",
            "goal_full_footprint_valid": "not_available",
            "start_in_corridor": "not_available",
            "goal_in_corridor": "not_available",
            "selected_start_attachment": "not_available",
            "selected_goal_attachment": "not_available",
            "attachment_candidate_count": "not_available",
            "selected_connection_edges": [],
        }
        for label, pose in (("start", query.start), ("goal", query.goal)):
            cell = hospital_map.world_to_cell(float(pose[0]), float(pose[1]))
            if cell is None:
                endpoint_fields[f"{label}_raw_map_cost"] = "not_available"
                endpoint_fields[f"{label}_full_footprint_valid"] = False
                endpoint_fields[f"{label}_in_corridor"] = False
                endpoint_fields[f"{label}_is_lethal"] = True
            else:
                raw_value = int(np.asarray(hospital_map.occupancy)[cell])
                endpoint_fields[f"{label}_raw_map_cost"] = raw_value
                endpoint_fields[f"{label}_inflated_cost"] = 0 if bool(_raw_free_mask(self.refined.artifact)[cell]) else 100
                endpoint_fields[f"{label}_full_footprint_valid"] = not bool(hospital_map.footprint_collision(tuple(pose), self.footprint, unknown_is_collision=True))
                endpoint_fields[f"{label}_in_corridor"] = bool(mask[cell])
                endpoint_fields[f"{label}_is_lethal"] = not bool(raw_free[cell])
        virtual_ids = [int(item) for item in node_path if int(item) in self._virtual_positions]
        if virtual_ids:
            endpoint_fields["selected_start_attachment"] = virtual_ids[0]
            endpoint_fields["selected_goal_attachment"] = virtual_ids[-1]
            endpoint_fields["selected_connection_edges"] = [
                f"virtual:{first}->{second}" for first, second in zip(node_path, node_path[1:])
                if int(first) < 0 or int(second) < 0
            ]
        endpoint_fields["attachment_candidate_count"] = int(sum(1 for item in node_path if int(item) in self._virtual_positions))
        if not self.l3_planner:
            return Layered2DResult(False, failure_code="BACKEND_UNAVAILABLE", snapshot_id=snapshot.snapshot_id, diagnostics={**endpoint_fields, "l2_called": False, "l2_call_count": 0, "l3_call_count": 0, "planner_search_started": False})
        self._l3_call_count += 1
        result, l3_diag = self.l3_planner.plan(query, mask, snapshot, topology_artifact=self.refined)
        points = list(getattr(result, "points", None) or []) if result is not None else None
        planner_success = bool(getattr(result, "planner_success", False) and points)
        diagnostics = {
            **endpoint_fields,
            "corridor_allowed_cells": int(np.count_nonzero(mask)),
            "corridor_total_free_cells": int(np.count_nonzero(_raw_free_mask(self.refined.artifact))),
            "corridor_free_cells": int(np.count_nonzero(_raw_free_mask(self.refined.artifact))),
            "corridor_area_ratio": float(np.count_nonzero(mask) / max(1, np.count_nonzero(_raw_free_mask(self.refined.artifact)))),
            "corridor_padding_m": selected_padding,
            "corridor_extra_margin_m": self.corridor_extra_margin_m,
            "topology_node_ids": list(node_path), "topology_edge_ids": list(self._route_edge_ids),
            "l2_called": False, "l2_call_count": 0,
            "l3_call_count": self._l3_call_count,
            "route_polyline": route_points,
            **dict(l3_diag or {}),
        }
        failure_code = "" if planner_success else str(l3_diag.get("failure_code") or getattr(result, "failure_code", "") or "L3_PLANNER_FAILED")
        if planner_success and validate and self.validator is not None:
            validation = dict(self.validator(self.refined.artifact.hospital_map, query, points) or {})
            diagnostics.update(validation)
            diagnostics["dynamic_collision_count"] = dynamic_collision_count(self.refined.artifact.hospital_map, points, self.footprint, snapshot)
            if diagnostics["dynamic_collision_count"]:
                diagnostics["failure_code"] = "DYNAMIC_FOOTPRINT_COLLISION"
            if diagnostics["dynamic_collision_count"] or not bool(validation.get("final_valid_success", validation.get("static_footprint_valid", False) and validation.get("kinematic_valid", False))):
                planner_success = False
                failure_code = str(validation.get("failure_code") or "L3_VALIDATION_FAILED")
        return Layered2DResult(planner_success, points if planner_success else points, failure_code, snapshot.snapshot_id, diagnostics)

    def _corridor_retry_paddings(self, initial_padding: float) -> List[float]:
        """Return the fixed, bounded corridor expansion sequence."""
        if self.corridor_profile != "padding" or self.corridor_fallback_policy != "bounded":
            return []
        return [
            padding for padding in (4.0, 6.0)
            if float(padding) > float(initial_padding) + 1.0e-9
        ]

    @staticmethod
    def _corridor_retryable_failure(code: str) -> bool:
        return str(code) in {
            "START_IN_LETHAL_SPACE", "GOAL_IN_LETHAL_SPACE",
            "NO_PATH_IN_CORRIDOR", "SMAC_MAX_ITERATIONS",
        }

    def plan_initial(self, query: Any, snapshot: Optional[DynamicSnapshot] = None) -> Layered2DResult:
        snap = snapshot or DynamicSnapshot.empty(map_shape=self.refined.artifact.free_mask.shape)
        self.current_snapshot = snap
        self.dstar = None
        self._route_node_ids = None
        self._route_edge_ids = []
        self._virtual_positions = {}
        self._query_state_key = (
            str(getattr(query, "query_id", "")),
            tuple(float(value) for value in query.start),
            tuple(float(value) for value in query.goal),
        )
        node_path, diagnostics = self._plan_route(query, snap)
        if node_path is None:
            result = Layered2DResult(False, failure_code=diagnostics.get("failure_code", "L1_NO_ROUTE"), snapshot_id=snap.snapshot_id, diagnostics={**diagnostics, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "l2_called": False, "l2_call_count": 0, "rrtstar_call_count": 0, "sst_call_count": 0})
            self._last_result = result
            return result
        initial_padding = self.corridor_padding_m
        result = self._run_l3(query, snap, node_path, corridor_padding_m=initial_padding)
        attempt_history: List[Dict[str, Any]] = []

        def record_attempt(attempt_result: Layered2DResult, padding: float) -> None:
            diag = dict(attempt_result.diagnostics or {})
            # Keep the retry audit compact: the full Smac excerpt remains in
            # the final query diagnostics, while each attempt records the
            # exact mask and planner outcome used for that call.
            attempt_history.append({
                "attempt_index": len(attempt_history) + 1,
                "corridor_padding_m": float(padding),
                "corridor_mask_hash": diag.get("corridor_mask_hash", ""),
                "allowed_grid_cells": diag.get("corridor_allowed_cells", 0),
                "corridor_area_ratio": diag.get("corridor_area_ratio", 0.0),
                "failure_code": str(attempt_result.failure_code or diag.get("failure_code", "")),
                "action_status": diag.get("action_status", "not_available"),
                "action_result_code": diag.get("action_result_code", "not_available"),
                "planner_search_started": diag.get("planner_search_started", "not_available"),
                "planner_time_ms": diag.get("planner_wall_time_ms", diag.get("planning_time_ms", "not_available")),
                "smac_log_excerpt": diag.get("smac_log_excerpt", ""),
            })

        record_attempt(result, initial_padding)
        retry_paddings: List[float] = []
        retry_failures: List[str] = []
        if not result.success and self._corridor_retryable_failure(result.failure_code):
            for padding in self._corridor_retry_paddings(initial_padding):
                retry_failures.append(str(result.failure_code))
                retry_paddings.append(float(padding))
                candidate = self._run_l3(query, snap, node_path, corridor_padding_m=padding)
                result = candidate
                record_attempt(result, padding)
                if result.success:
                    break
        result = Layered2DResult(
            result.success,
            result.points,
            result.failure_code,
            snap.snapshot_id,
            {
                **dict(result.diagnostics),
                "corridor_fallback_policy": self.corridor_fallback_policy,
                "corridor_initial_padding_m": initial_padding,
                "corridor_retry_paddings_m": retry_paddings,
                "corridor_retry_failures": retry_failures,
                "corridor_fallback_used": bool(retry_paddings),
                "corridor_fallback_attempt_count": len(retry_paddings),
                "attempts": attempt_history,
            },
        )
        result = Layered2DResult(result.success, result.points, result.failure_code, snap.snapshot_id, {**diagnostics, **dict(result.diagnostics), "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "l1_reroute_count": self._l1_reroute_count})
        self._last_result = result
        return result

    def _changed_edges(self, snapshot: DynamicSnapshot) -> List[str]:
        occupied = set(snapshot.inflated_cells(0))
        previous = set(self.current_snapshot.inflated_cells(0))
        changed_cells = occupied.symmetric_difference(previous)
        return [edge_id for edge_id, edge in self.refined.edges.items() if changed_cells.intersection(edge.edge_cells) or occupied.intersection(edge.edge_cells)]

    def update_dynamic(self, query: Any, snapshot: DynamicSnapshot) -> Layered2DResult:
        query_key = (
            str(getattr(query, "query_id", "")),
            tuple(float(value) for value in query.start),
            tuple(float(value) for value in query.goal),
        )
        if self._query_state_key is not None and query_key != self._query_state_key:
            return self.plan_initial(query, snapshot)
        if self.dstar is None or self._route_node_ids is None:
            return self.plan_initial(query, snapshot)
        changed_edges = self._changed_edges(snapshot)
        if not changed_edges:
            result = Layered2DResult(True, self._last_result.points if self._last_result else None, snapshot_id=snapshot.snapshot_id, diagnostics={"dynamic_replan_triggered": False, "changed_edge_count": 0, "snapshot_id": snapshot.snapshot_id, "snapshot_hash": snapshot.snapshot_hash, "l2_called": False, "l2_call_count": 0, "l3_call_count": self._l3_call_count})
            self.current_snapshot = snapshot
            self._last_result = result
            return result
        occupied = set(snapshot.inflated_cells(0))
        statuses = {
            edge_id: (GraphDStarLite.BLOCKED_PENDING if occupied.intersection(self.refined.edges[edge_id].edge_cells) else GraphDStarLite.RECOVERING)
            for edge_id in changed_edges
        }
        costs = {
            edge_id: (float(self.refined.edges[edge_id].length_m * 1000.0) if statuses[edge_id] == GraphDStarLite.BLOCKED_PENDING else float(self.refined.edges[edge_id].static_cost))
            for edge_id in changed_edges
        }
        self.edge_status.update(statuses); self.edge_cost_override.update(costs)
        affected = self.dstar.update_edges(changed_edges, statuses=statuses, costs=costs)
        self.current_snapshot = snapshot
        self._l1_reroute_count += 1
        node_path, route_diag = self._plan_route(query, snapshot, reuse_graph=True)
        if node_path is None:
            result = Layered2DResult(False, failure_code=route_diag.get("failure_code", "L1_NO_ROUTE"), snapshot_id=snapshot.snapshot_id, diagnostics={**route_diag, "dynamic_replan_triggered": True, "changed_edge_count": len(changed_edges), "affected_graph_nodes": affected, "snapshot_hash": snapshot.snapshot_hash, "l2_called": False, "l2_call_count": 0, "l3_call_count": self._l3_call_count})
            self._last_result = result
            return result
        result = self._run_l3(query, snapshot, node_path)
        # A failed L3 check is feedback about the selected corridor, not proof
        # that the static topology is invalid.  Mark only the selected edges
        # BLOCKED_PENDING and perform one bounded graph re-route.
        if not result.success and self._route_edge_ids:
            feedback_edges = list(self._route_edge_ids)
            feedback_status = {edge_id: GraphDStarLite.BLOCKED_PENDING for edge_id in feedback_edges if edge_id in self.refined.edges}
            feedback_cost = {edge_id: float(self.refined.edges[edge_id].length_m * 1000.0) for edge_id in feedback_status}
            if feedback_status:
                self.edge_status.update(feedback_status); self.edge_cost_override.update(feedback_cost)
                feedback_affected = self.dstar.update_edges(feedback_status, statuses=feedback_status, costs=feedback_cost)
                alternate_path, alternate_diag = self._plan_route(query, snapshot, reuse_graph=True)
                if alternate_path is not None:
                    alternate = self._run_l3(query, snapshot, alternate_path)
                    result = Layered2DResult(alternate.success, alternate.points, alternate.failure_code, snapshot.snapshot_id, {**dict(alternate.diagnostics), "l3_failure_feedback_used": True, "l3_failure_feedback_edge_count": len(feedback_status), "l3_failure_feedback_affected_nodes": feedback_affected, "alternate_route_diagnostics": alternate_diag})
                else:
                    result = Layered2DResult(False, result.points, result.failure_code or "L3_VALIDATION_FAILED", snapshot.snapshot_id, {**dict(result.diagnostics), "l3_failure_feedback_used": True, "l3_failure_feedback_edge_count": len(feedback_status), "l3_failure_feedback_affected_nodes": feedback_affected})
        result = Layered2DResult(result.success, result.points, result.failure_code, snapshot.snapshot_id, {**route_diag, **dict(result.diagnostics), "dynamic_replan_triggered": True, "changed_edge_count": len(changed_edges), "affected_graph_nodes": affected, "snapshot_hash": snapshot.snapshot_hash, "l1_reroute_count": self._l1_reroute_count})
        self._last_result = result
        return result


def _demo() -> Dict[str, Any]:
    """Offline smoke showing graph-state D* Lite and dynamic edge recovery."""
    class DemoMap:
        map_id = "demo"
        resolution = 1.0
        width = 20
        height = 10
        occupancy = np.zeros((height, width), dtype=np.int8)

        def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
            return int(round(y)), int(round(x))

        def footprint_collision(self, pose: Any, footprint: Any, unknown_is_collision: bool = True) -> bool:
            del pose, footprint, unknown_is_collision
            return False

    nodes = [
        topology.TopologyNode(0, 1.0, 5.0, 5, 1, 2, 1.0, 2.0, 1),
        topology.TopologyNode(1, 5.0, 5.0, 5, 5, 3, 1.0, 2.0, 1),
        topology.TopologyNode(2, 5.0, 2.0, 2, 5, 2, 1.0, 2.0, 1),
        topology.TopologyNode(3, 9.0, 2.0, 2, 9, 2, 1.0, 2.0, 1),
        topology.TopologyNode(4, 9.0, 5.0, 5, 9, 2, 1.0, 2.0, 1),
    ]
    edges = [
        topology.TopologyEdge(0, 0, 1, 4.0, 1.0, 1.0, 2.0, 2, [[1.0, 5.0], [5.0, 5.0]]),
        topology.TopologyEdge(1, 1, 4, 4.0, 1.0, 1.0, 2.0, 2, [[5.0, 5.0], [9.0, 5.0]]),
        topology.TopologyEdge(2, 0, 2, 3.0, 1.0, 1.0, 2.0, 2, [[1.0, 5.0], [5.0, 2.0]]),
        topology.TopologyEdge(3, 2, 3, 4.0, 1.0, 1.0, 2.0, 2, [[5.0, 2.0], [9.0, 2.0]]),
        topology.TopologyEdge(4, 3, 4, 3.0, 1.0, 1.0, 2.0, 2, [[9.0, 2.0], [9.0, 5.0]]),
    ]
    artifact = topology.TopologyArtifact(DemoMap(), np.ones((10, 20), dtype=bool), np.ones((10, 20), dtype=bool), np.ones((10, 20), dtype=np.float32), np.ones((10, 20), dtype=np.int32), topology.TopologyGraph(nodes, edges), {"map_sha256": "demo", "algorithm": topology.TOPOLOGY_ALGORITHM_VERSION, "skeleton_backend": "numpy_zhang_suen"})
    refined = build_refined_topology(artifact, spacing_m=2.0)
    graph_edges = [GraphEdge(edge.edge_id, edge.source_node, edge.target_node, edge.length_m, edge.static_cost) for edge in refined.edges.values()]
    graph = GraphDStarLite(refined.nodes, graph_edges, 0, 4)
    graph.node_positions = {node_id: (node.x, node.y) for node_id, node in refined.nodes.items()}
    initial = graph.compute_shortest_path(); initial_path = graph.extract_path()
    middle = "e0_0"
    graph.update_edges([middle], statuses={middle: GraphDStarLite.BLOCKED}, costs={middle: INF})
    updated = graph.compute_shortest_path(); updated_path = graph.extract_path()
    return {
        "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "l1_backend": L1_BACKEND, "l1_state_type": "refined_topology_node_id",
        "l2_called": False, "l2_call_count": 0, "l3_backend": L3_BACKEND,
        "initial": {"success": initial_path is not None, "path": initial_path, "expanded_nodes": initial.expanded_nodes, "generated_nodes": initial.generated_nodes},
        "dynamic_update": {"blocked_edge": middle, "success": updated_path is not None, "path": updated_path, "expanded_nodes": updated.expanded_nodes, "generated_nodes": updated.generated_nodes},
        "static_map_mutated": False, "rrtstar_call_count": 0, "sst_call_count": 0,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the independent 2D-V0 refined-topology D* Lite + Smac pipeline")
    parser.add_argument("--architecture", choices=("2d_v0",), default="2d_v0")
    parser.add_argument("--map-yaml", type=Path)
    parser.add_argument("--query-json", type=Path)
    parser.add_argument("--snapshot-json", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v0_dynamic_smoke_v1"))
    parser.add_argument("--topology-cache-dir", type=Path)
    parser.add_argument("--refined-cache-dir", type=Path)
    parser.add_argument("--corridor-padding-m", type=float, default=DEFAULT_CORRIDOR_PADDING_M)
    parser.add_argument("--corridor-profile", choices=("padding", "full_map"), default="padding")
    parser.add_argument(
        "--costmap-reset-policy", choices=("full", "light"), default="full",
        help="reset the shared Smac costmap before each query (full is the deterministic default)",
    )
    parser.add_argument(
        "--corridor-fallback-policy", choices=("bounded", "none"), default="bounded",
        help="on a failed corridor plan, retry the same topology route at 4 m then 6 m",
    )
    parser.add_argument("--query-ids", nargs="*", default=None, help="optional fixed query IDs for a diagnostic run")
    parser.add_argument("--corridor-diagnostic", action="store_true", help="run an independent 2/4/6 m/full-map diagnostic sweep")
    parser.add_argument("--ros-domain-id", type=int, default=0)
    parser.add_argument("--demo", action="store_true", help="run the offline graph-only D* Lite smoke")
    return parser


def _write_demo_outputs(output: Path, result: Mapping[str, Any]) -> None:
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir(exist_ok=True)
    (output / "protocol.yaml").write_text(yaml.safe_dump({"schema_version": 1, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "protocol_version": PROTOCOL_VERSION, "l1": "refined skeleton graph D* Lite", "l2_called": False, "l2_call_count": 0, "l3": "full selected topology corridor Smac Hybrid DUBIN", "resolution_m": RESOLUTION_M, "minimum_turning_radius_m": RMIN_M, "maximum_curvature": MAX_CURVATURE, "allow_reverse": ALLOW_REVERSE, "allow_in_place_rotation": ALLOW_IN_PLACE_ROTATION, "dynamic_obstacles": DYNAMIC_OBSTACLES, "smoke_mode": "offline_graph_demo"}, sort_keys=False), encoding="utf-8")
    (output / "manifest.yaml").write_text(yaml.safe_dump({"schema_version": 1, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "smoke_status": "offline_demo_only", "l2_call_count": 0, "rrtstar_call_count": 0, "sst_call_count": 0, "static_map_mutated": False}, sort_keys=False), encoding="utf-8")
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({"architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "source_hash": _source_hash(), "graph_dstar_source_hash": hashlib.sha256(Path(__file__).with_name("graph_dstar_lite.py").read_bytes()).hexdigest()}, sort_keys=False), encoding="utf-8")
    (output / "runs.csv").write_text(f"architecture_id,implementation_revision,run_mode,l2_called,l2_call_count,l3_call_count,final_valid_success,failure_code\n{ARCHITECTURE_ID},{IMPLEMENTATION_REVISION},demo,false,0,0,false,OFFLINE_DEMO_NO_SMAC\n", encoding="utf-8")
    (output / "backend_call_log.csv").write_text("architecture_id,stage,called,physical_backend_call_count\n2D-V0,L2,false,0\n", encoding="utf-8")
    (output / "path_metrics.csv").write_text("architecture_id,static_footprint_valid,kinematic_valid,dynamic_collision_count\n2D-V0,not_available,not_available,not_available\n", encoding="utf-8")
    report = ["# 2D-V0 dynamic smoke", "", "This is an offline graph-state smoke only; no ROS/Smac request was made.", "", f"- Architecture: `{ARCHITECTURE_ID}` revision `{IMPLEMENTATION_REVISION}`.", "- L1 state type: refined topology node ids; D* Lite state is not a grid.", "- L2 calls: 0; RRTstar/SST calls: 0/0.", f"- Initial D* Lite: {json.dumps(result.get('initial', {}), sort_keys=True)}", f"- Dynamic edge update: {json.dumps(result.get('dynamic_update', {}), sort_keys=True)}", "- Static map mutation: false.", "- Real Smac and full path validation remain required for the ROS-backed P0/P1 smoke; this artifact is not a validity or performance claim."]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")


def _load_queries(path: Path) -> List[Any]:
    """Load a fixed JSON query list without modifying or re-generating it."""
    from .planner_benchmark.models import Query
    payload = json.loads(path.read_text(encoding="utf-8"))
    items = payload.get("tasks", []) if isinstance(payload, dict) else payload
    if isinstance(payload, dict) and isinstance(payload.get("maps"), dict):
        first_map = next(iter(payload["maps"].values()), {})
        items = first_map.get("tasks", [])
    queries = []
    for item in items or []:
        query_id = str(item.get("id", item.get("query_id", "")))
        if not query_id:
            raise ValueError("query JSON item is missing id/query_id")
        queries.append(Query(query_id, [float(value) for value in item["start"]], [float(value) for value in item["goal"]], str(item.get("label", "")), int(item.get("seed", 0)), "UNVALIDATED"))
    if not queries:
        raise ValueError(f"query JSON contains no tasks: {path}")
    return queries


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    """Write deterministic union-of-fields CSV while preserving JSON diagnostics."""
    path.parent.mkdir(parents=True, exist_ok=True)
    materialized = [dict(row) for row in rows]
    fields: List[str] = []
    seen = set()
    for row in materialized:
        for key in row:
            if key not in seen:
                fields.append(str(key))
                seen.add(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in materialized:
            writer.writerow({key: json.dumps(value, sort_keys=True, default=str) if isinstance(value, (dict, list, tuple)) else value for key, value in row.items()})


def _run_corridor_diagnostic(args: argparse.Namespace) -> int:
    """Run isolated corridor profiles and aggregate auditable diagnostics.

    Each profile gets its own child output and planner session. This keeps
    masks and action logs independent while preserving identical inputs and
    Smac parameters across the 2/4/6 m and full-map comparisons.
    """
    output = Path(args.output_dir).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir(exist_ok=True)
    profiles = (("2m", 2.0, "padding"), ("4m", 4.0, "padding"), ("6m", 6.0, "padding"), ("full_map", 0.0, "full_map"))
    rows: List[Dict[str, Any]] = []
    call_rows: List[Dict[str, Any]] = []
    metric_rows: List[Dict[str, Any]] = []
    availability = {
        "smac_costmap_start_goal": "not_available_without_costmap_service_or_topic_readback",
        "start_inflated_cost": "not_available_without_costmap_service_or_topic_readback",
        "goal_inflated_cost": "not_available_without_costmap_service_or_topic_readback",
        "expanded_generated_states": "not_available_from_compute_path_action",
    }
    for name, padding, profile in profiles:
        child_args = copy.copy(args)
        child_args.corridor_diagnostic = False
        child_args.output_dir = output / f"corridor_{name}"
        child_args.topology_cache_dir = output / "shared_topology_cache"
        child_args.refined_cache_dir = output / "shared_refined_topology_cache"
        child_args.corridor_padding_m = padding if profile == "padding" else float(args.corridor_padding_m)
        child_args.corridor_profile = profile
        # Keep each diagnostic profile independent.  A 2 m row must represent
        # a 2 m request, rather than silently including a 4/6 m retry.
        child_args.corridor_fallback_policy = "none"
        # The requested IDs are the fixed failed-query diagnostic set unless
        # the caller explicitly supplies another fixed subset.
        if not child_args.query_ids:
            child_args.query_ids = [
                "A2B-01", "A2B-02", "A2B-03", "A2B-05", "A2B-06", "A2B-07",
                "A2B-08", "A2B-09", "A2B-10", "A2B-12", "A2B-15", "A2B-17",
                "A2B-19", "A2B-20",
            ]
        _run_ros_smoke(child_args)
        runs_path = Path(child_args.output_dir) / "runs.csv"
        if runs_path.exists():
            with runs_path.open(newline="", encoding="utf-8") as stream:
                for row in csv.DictReader(stream):
                    row["corridor_profile"] = name
                    rows.append(row)
        calls_path = Path(child_args.output_dir) / "backend_call_log.csv"
        if calls_path.exists():
            with calls_path.open(newline="", encoding="utf-8") as stream:
                for row in csv.DictReader(stream):
                    row["corridor_profile"] = name
                    call_rows.append(row)
        metrics_path = Path(child_args.output_dir) / "path_metrics.csv"
        if metrics_path.exists():
            with metrics_path.open(newline="", encoding="utf-8") as stream:
                for row in csv.DictReader(stream):
                    row["corridor_profile"] = name
                    metric_rows.append(row)
    _write_csv(output / "corridor_diagnostic_summary.csv", rows)
    _write_csv(output / "endpoint_costmap_diagnostic.csv", rows)
    _write_csv(output / "backend_call_log.csv", call_rows)
    _write_csv(output / "path_metrics.csv", metric_rows)
    failure_counts: Dict[str, int] = {}
    for item in rows:
        code = str(item.get("failure_code") or "")
        if code:
            failure_counts[code] = failure_counts.get(code, 0) + 1
    _write_csv(output / "failure_summary.csv", [{"failure_code": code, "count": count} for code, count in sorted(failure_counts.items())])
    _atomic_write_text(output / "metric_availability.yaml", yaml.safe_dump(availability, sort_keys=False))
    manifest = {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_version": PROTOCOL_VERSION,
        "diagnostic_only": True,
        "profiles": [name for name, _padding, _profile in profiles],
        "query_count": len({row.get("query_id") for row in rows}),
        "record_count": len(rows),
        "static_smoke_success_rate_excluded": True,
        "failure_counts": failure_counts,
    }
    _atomic_write_text(output / "manifest.yaml", yaml.safe_dump(manifest, sort_keys=False))
    _atomic_write_text(output / "protocol.yaml", yaml.safe_dump({"architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "protocol_version": PROTOCOL_VERSION, "profiles": [name for name, _padding, _profile in profiles]}, sort_keys=False))
    map_yaml_hash = hashlib.sha256(args.map_yaml.read_bytes()).hexdigest() if args.map_yaml and args.map_yaml.exists() else "not_available"
    query_hash = hashlib.sha256(args.query_json.read_bytes()).hexdigest() if args.query_json and args.query_json.exists() else "not_available"
    _atomic_write_text(output / "source_manifest.yaml", yaml.safe_dump({"architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "protocol_version": PROTOCOL_VERSION, "pipeline_source_hash": _source_hash(), "map_yaml_sha256": map_yaml_hash, "query_json_sha256": query_hash}, sort_keys=False))
    _atomic_write_text(output / "topology_manifest.yaml", yaml.safe_dump({"shared_topology_cache": str(output / "shared_topology_cache"), "shared_refined_topology_cache": str(output / "shared_refined_topology_cache"), "profiles": [name for name, _padding, _profile in profiles]}, sort_keys=False))
    report = [
        "# 2D-V0 corridor diagnostic sweep",
        "",
        f"Architecture: `{ARCHITECTURE_ID}` revision `{IMPLEMENTATION_REVISION}`.",
        "Profiles are isolated child runs and are excluded from formal success-rate statistics.",
        "Raw action logs and per-profile runs.csv are retained under each corridor_* directory.",
        f"Aggregated failure counts: {json.dumps(failure_counts, sort_keys=True)}.",
        "Static validity gate remains pending until a fixed corridor profile reaches the required query coverage; no dynamic experiment is started by this diagnostic.",
    ]
    _atomic_write_text(output / "final_report.md", "\n".join(report) + "\n")
    return 0


def _run_ros_smoke(args: argparse.Namespace) -> int:
    """Run a bounded ROS-backed P0 smoke when explicit inputs are supplied."""
    if args.map_yaml is None or args.query_json is None:
        raise ValueError("ROS-backed execution requires --map-yaml and --query-json")
    from . import unified_four_backends_smoke as legacy
    from .planner_benchmark.map_utils import HospitalMap, sha256_file

    output = Path(args.output_dir).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir(exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = str(int(args.ros_domain_id))
    hospital_map = HospitalMap.load(args.map_yaml)
    if not math.isclose(float(hospital_map.resolution), RESOLUTION_M, rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError(f"map resolution must be {RESOLUTION_M}, got {hospital_map.resolution}")
    topology_cache = args.topology_cache_dir or (output / "topology_cache")
    topology_artifact, topology_info = prepare_static_topology(
        hospital_map, legacy.FOOTPRINT, topology_cache,
        padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    free_mask = topology_artifact.free_mask
    distance_m = topology_artifact.distance_m
    ctx = legacy.MapContext(hospital_map.map_id, hospital_map, free_mask, distance_m, sha256_file(hospital_map.image_path), sha256_file(hospital_map.yaml_path), args.map_yaml)
    refined_cache = args.refined_cache_dir or (output / "refined_topology_cache")
    refined, refined_info = prepare_refined_topology(topology_artifact, legacy.FOOTPRINT, refined_cache)
    queries = _load_queries(args.query_json)
    if args.query_ids:
        requested = {str(item) for item in args.query_ids}
        queries = [query for query in queries if query.query_id in requested]
        if not queries:
            raise ValueError("none of --query-ids exist in query JSON")
    spec = legacy.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"Smac Hybrid backend unavailable: {spec.reason}")
    session = legacy.SmacSession(ctx, output, map_yaml=args.map_yaml, log_tag=f"2d_v0_{hospital_map.map_id}", local_mask_updates=True, optimization_profile="v7_candidate", smac_parameter_profile="lighter_smoother", optimization_stage="step3_delta_map")
    session.start()
    adapter = SmacHybridAdapter(session, spec, footprint=legacy.FOOTPRINT, source_commit=legacy._source_commit())
    pipeline = Layered2DV0Pipeline(
        refined,
        footprint=legacy.FOOTPRINT,
        l3_planner=adapter,
        corridor_padding_m=float(args.corridor_padding_m),
        corridor_profile=str(args.corridor_profile),
        corridor_fallback_policy=str(args.corridor_fallback_policy),
        validator=lambda _map, query, points: legacy.validate_path(ctx, query, points),
    )
    snapshot = DynamicSnapshot.empty(snapshot_id="static", map_shape=free_mask.shape)
    if args.snapshot_json:
        snapshot_payload = json.loads(args.snapshot_json.read_text(encoding="utf-8"))
        snapshot = DynamicSnapshot.from_cells(str(snapshot_payload.get("snapshot_id", "snapshot-1")), snapshot_payload.get("occupied_cells", []), timestamp=float(snapshot_payload.get("timestamp", time.time())), confidence=snapshot_payload.get("obstacle_confidence", {}), ttl=snapshot_payload.get("ttl"), map_version=str(snapshot_payload.get("map_version", "")), map_shape=free_mask.shape)
    rows: List[Dict[str, Any]] = []
    calls: List[Dict[str, Any]] = []
    metrics: List[Dict[str, Any]] = []
    started_session = time.monotonic_ns()
    try:
        for query in queries:
            before = resource.getrusage(resource.RUSAGE_SELF)
            started = time.monotonic_ns()
            # Adapter.calls is session-wide; snapshot it for per-query audit fields.
            query_l3_calls_before = int(adapter.calls)
            if session is not None:
                # A full reset is the validity-first default.  Delta-only
                # updates remain available as an explicit opt-in, but a
                # static-layer acknowledgement is not equivalent to an
                # inflation-layer clear on every Nav2 version.
                reset_info = session.reset_query_state(
                    query.query_id,
                    restore_base_map=(str(args.costmap_reset_policy) == "full"),
                )
            else:
                reset_info = {}
            result = pipeline.plan_initial(query, snapshot)
            query_l3_call_count = max(0, int(adapter.calls) - query_l3_calls_before)
            after = resource.getrusage(resource.RUSAGE_SELF)
            diagnostics = dict(result.diagnostics)
            points = result.points
            final_valid = bool(result.success and diagnostics.get("final_valid_success", True))
            run_id = f"{hospital_map.map_id}_{query.query_id}_2d_v0"
            path_file = ""
            path_hash = ""
            if points:
                encoded = [dict(point) for point in points]
                # SmacHybridAdapter enriches each point with the canonical
                # hash (computed without the hash field itself). Reuse it so
                # the CSV, manifest and path file all identify the same path.
                path_hash = str(encoded[0].get("path_hash", ""))
                if not path_hash:
                    path_hash = _enrich_path(encoded, legacy._source_commit())
                path_file = f"paths/{run_id}.json"
                (output / path_file).write_text(json.dumps(encoded, indent=2, sort_keys=True), encoding="utf-8")
            wall_ms = (time.monotonic_ns() - started) / 1.0e6
            cpu_ms = max(0.0, (after.ru_utime - before.ru_utime + after.ru_stime - before.ru_stime) * 1000.0)
            row = {
                "run_id": run_id, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "map_id": hospital_map.map_id, "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256, "query_id": query.query_id, "run_mode": "smoke", "start_x": query.start[0], "start_y": query.start[1], "start_yaw": query.start[2], "goal_x": query.goal[0], "goal_y": query.goal[1], "goal_yaw": query.goal[2], "l1_backend": L1_BACKEND, "l1_success": bool(diagnostics.get("topology_node_ids")), "l2_called": False, "l2_call_count": 0, "l3_backend": L3_BACKEND, "l3_call_count": int(diagnostics.get("l3_call_count", 0)), "topology_node_ids": diagnostics.get("topology_node_ids", []), "topology_edge_ids": diagnostics.get("topology_edge_ids", []), "corridor_mask_hash": diagnostics.get("corridor_mask_hash", ""), "allowed_grid_cells": diagnostics.get("corridor_allowed_cells", 0), "total_free_grid_cells": diagnostics.get("corridor_total_free_cells", 0), "corridor_area_ratio": diagnostics.get("corridor_area_ratio", 0.0), "corridor_padding_m": args.corridor_padding_m, "dynamic_snapshot_id": snapshot.snapshot_id, "dynamic_snapshot_hash": snapshot.snapshot_hash, "l1_dstar_initial_time_ms": diagnostics.get("l1_dstar_initial_time_ms", 0.0), "l1_dstar_incremental_time_ms": diagnostics.get("l1_dstar_incremental_time_ms", 0.0), "dstar_expanded_nodes": diagnostics.get("dstar_expanded_nodes", 0), "dstar_generated_nodes": diagnostics.get("dstar_generated_nodes", 0), "attachment_candidate_count": diagnostics.get("attachment_candidate_count", 0), "pipeline_wall_time_ms": wall_ms, "pipeline_cpu_total_ms": cpu_ms, "hybrid_planning_time_ms": diagnostics.get("planning_time_ms", diagnostics.get("planner_wall_time_ms", 0.0)), "action_status": diagnostics.get("action_status", "not_available"), "planner_search_started": diagnostics.get("planner_search_started", "not_available"), "peak_rss": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss, "peak_pss": "not_available", "static_footprint_valid": diagnostics.get("static_footprint_valid", "not_available"), "kinematic_valid": diagnostics.get("kinematic_valid", "not_available"), "final_valid_success": final_valid, "failure_code": "" if final_valid else (diagnostics.get("failure_code") or result.failure_code or "L3_PLANNER_FAILED"), "path_hash": path_hash, "path_file": path_file, "topology_cache_hit": topology_info.get("topology_cache_hit", False), "topology_cache_key": topology_info.get("topology_cache_key", ""), "topology_cache_load_time_ms": topology_info.get("topology_load_time_ms", 0.0), "topology_build_time_ms": topology_info.get("topology_build_time_ms", 0.0), "topology_build_count": topology_info.get("topology_build_count", 0), "refined_topology_cache_hit": refined_info.get("topology_cache_hit", False), "refined_topology_load_time_ms": refined_info.get("refined_topology_load_time_ms", 0.0), "refined_topology_cache_load_time_ms": refined_info.get("refined_topology_load_time_ms", 0.0), "refined_topology_node_count": refined.node_count, "refined_topology_edge_count": refined.edge_count, "rrtstar_call_count": 0, "sst_call_count": 0,
            }
            row.update({
                "protocol_version": PROTOCOL_VERSION,
                "corridor_profile": diagnostics.get("corridor_profile", args.corridor_profile),
                "corridor_free_cells": diagnostics.get("corridor_free_cells", diagnostics.get("corridor_total_free_cells", 0)),
                "corridor_min_clearance_m": diagnostics.get("corridor_min_clearance_m", "not_available"),
                "action_result_code": diagnostics.get("action_result_code", "not_available"),
                "smac_failure_code": diagnostics.get("smac_failure_code", ""),
                "smac_log_excerpt": diagnostics.get("smac_log_excerpt", ""),
                "smac_log_path": diagnostics.get("smac_log_path", "not_available"),
                "l3_query_call_count": diagnostics.get("l3_query_call_count", diagnostics.get("backend_call_count", 0)),
                "costmap_update_before_hash": diagnostics.get("previous_mask_hash", "not_available"),
                "costmap_update_expected_hash": diagnostics.get("expected_mask_hash", "not_available"),
                "costmap_update_after_hash": diagnostics.get("applied_mask_hash", "not_available"),
                "costmap_update_time_ms": diagnostics.get("local_map_update_ms", "not_available"),
                "costmap_update_messages": diagnostics.get("local_map_update_messages", "not_available"),
                "costmap_update_mode": diagnostics.get("local_map_update_mode", "not_available"),
                "start_raw_map_cost": diagnostics.get("start_raw_map_cost", "not_available"),
                "goal_raw_map_cost": diagnostics.get("goal_raw_map_cost", "not_available"),
                "start_inflated_cost": diagnostics.get("start_inflated_cost", "not_available"),
                "goal_inflated_cost": diagnostics.get("goal_inflated_cost", "not_available"),
                "smac_start_cost": diagnostics.get("smac_start_cost", "not_available"),
                "smac_goal_cost": diagnostics.get("smac_goal_cost", "not_available"),
                "start_is_lethal": diagnostics.get("start_is_lethal", "not_available"),
                "goal_is_lethal": diagnostics.get("goal_is_lethal", "not_available"),
                "start_full_footprint_valid": diagnostics.get("start_full_footprint_valid", "not_available"),
                "goal_full_footprint_valid": diagnostics.get("goal_full_footprint_valid", "not_available"),
                "start_in_corridor": diagnostics.get("start_in_corridor", "not_available"),
                "goal_in_corridor": diagnostics.get("goal_in_corridor", "not_available"),
                "selected_start_attachment": diagnostics.get("selected_start_attachment", "not_available"),
                "selected_goal_attachment": diagnostics.get("selected_goal_attachment", "not_available"),
                "refined_topology_cache_state": refined_info.get("cache_state", "not_available"),
                "refined_topology_cache_miss_reason": refined_info.get("cache_miss_reason", ""),
                "costmap_reset_policy": str(args.costmap_reset_policy),
                "corridor_fallback_policy": str(args.corridor_fallback_policy),
                "corridor_fallback_used": diagnostics.get("corridor_fallback_used", False),
                "corridor_fallback_attempt_count": diagnostics.get("corridor_fallback_attempt_count", 0),
                "corridor_retry_paddings_m": diagnostics.get("corridor_retry_paddings_m", []),
                "corridor_retry_failures": diagnostics.get("corridor_retry_failures", []),
                "query_session_reset_mode": reset_info.get("query_session_reset_mode", "not_available"),
                "session_reset_fallback": reset_info.get("session_reset_fallback", False),
                "session_reset_fallback_reason": reset_info.get("session_reset_fallback_reason", ""),
                "query_session_reset_ms": reset_info.get("query_session_reset_ms", "not_available"),
            })
            # Normalize session-wide adapter diagnostics to this query.  A
            # query can issue several bounded corridor attempts, while an L1
            # failure issues none; both cases must be auditable per row.
            row["l3_call_count"] = query_l3_call_count
            row["l3_call_count_total"] = int(adapter.calls)
            row["l3_query_call_count"] = query_l3_call_count
            row["corridor_padding_m"] = diagnostics.get("corridor_padding_m", args.corridor_padding_m)
            rows.append(row)
            calls.append({"run_id": run_id, "architecture_id": ARCHITECTURE_ID, "stage": "L3", "called": bool(diagnostics.get("l3_call_count", 0)), "physical_backend_call_count": int(diagnostics.get("l3_call_count", 0)), "planner_backend": L3_BACKEND, "l2_called": False, "l2_call_count": 0, "failure_code": row["failure_code"], "planner_search_started": row["planner_search_started"], "corridor_mask_hash": row["corridor_mask_hash"], "dynamic_snapshot_id": snapshot.snapshot_id})
            calls[-1]["called"] = bool(query_l3_call_count)
            calls[-1]["physical_backend_call_count"] = query_l3_call_count
            metrics.append({"run_id": run_id, "query_id": query.query_id, "path_hash": path_hash, "final_valid_success": final_valid, "static_footprint_valid": row["static_footprint_valid"], "kinematic_valid": row["kinematic_valid"], "path_length_m": diagnostics.get("path_length_m"), "minimum_clearance_m": diagnostics.get("minimum_clearance_m"), "maximum_curvature": diagnostics.get("maximum_curvature"), "reverse_distance_m": diagnostics.get("reverse_distance_m", 0.0), "in_place_rotation_count": diagnostics.get("in_place_rotation_count", 0)})
    finally:
        session.close()
    _write_csv(output / "runs.csv", rows)
    _write_csv(output / "path_metrics.csv", metrics)
    _write_csv(output / "backend_call_log.csv", calls)
    failure_counts: Dict[str, int] = {}
    for item in rows:
        code = str(item.get("failure_code") or "")
        if code:
            failure_counts[code] = failure_counts.get(code, 0) + 1
    _write_csv(output / "failure_summary.csv", [{"failure_code": code, "count": count} for code, count in sorted(failure_counts.items())])
    _write_csv(output / "session_timing.csv", [{"architecture_id": ARCHITECTURE_ID, "session_start_count": session.session_start_count, "session_close_count": session.session_close_count, "session_restart_count": session.session_restart_count, "session_startup_time_ms": session.stack_startup_time_ms, "session_shutdown_time_ms": session.stack_shutdown_time_ms}])
    _atomic_write_text(output / "manifest.yaml", yaml.safe_dump({"architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "protocol_version": PROTOCOL_VERSION, "map_id": hospital_map.map_id, "query_count": len(rows), "final_valid_count": sum(bool(row["final_valid_success"]) for row in rows), "l2_call_count": 0, "rrtstar_call_count": 0, "sst_call_count": 0, "topology_cache_hit": topology_info.get("topology_cache_hit", False), "topology_build_count": topology_info.get("topology_build_count", 0), "topology_build_time_ms": topology_info.get("topology_build_time_ms", 0.0), "topology_load_time_ms": topology_info.get("topology_load_time_ms", 0.0), "topology_cache_key": topology_info.get("topology_cache_key", ""), "refined_topology_cache_hit": refined_info.get("topology_cache_hit", False), "refined_topology_cache_state": refined_info.get("cache_state", "not_available"), "refined_topology_cache_miss_reason": refined_info.get("cache_miss_reason", ""), "refined_topology_node_count": refined.node_count, "refined_topology_edge_count": refined.edge_count}, sort_keys=False))
    # Keep the session-wide total alongside the per-query counts in runs.csv.
    manifest_path = output / "manifest.yaml"
    manifest_payload = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
    manifest_payload["l3_call_count_total"] = int(adapter.calls)
    _atomic_write_text(manifest_path, yaml.safe_dump(manifest_payload, sort_keys=False))
    _atomic_write_text(output / "protocol.yaml", yaml.safe_dump({"architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "protocol_version": PROTOCOL_VERSION, "resolution_m": RESOLUTION_M, "minimum_turning_radius_m": RMIN_M, "maximum_curvature": MAX_CURVATURE, "allow_reverse": ALLOW_REVERSE, "allow_in_place_rotation": ALLOW_IN_PLACE_ROTATION, "dynamic_obstacles": bool(args.snapshot_json), "l1": "refined skeleton graph D* Lite", "l2_called": False, "l3": "full topology corridor Smac Hybrid DUBIN", "corridor_profile": args.corridor_profile, "corridor_padding_m": args.corridor_padding_m, "costmap_reset_policy": str(args.costmap_reset_policy), "corridor_fallback_policy": str(args.corridor_fallback_policy)}, sort_keys=False))
    _atomic_write_text(output / "source_manifest.yaml", yaml.safe_dump({"architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "protocol_version": PROTOCOL_VERSION, "pipeline_source_hash": _source_hash(), "graph_dstar_source_hash": hashlib.sha256(Path(__file__).with_name("graph_dstar_lite.py").read_bytes()).hexdigest(), "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256, "query_json": str(args.query_json), "query_json_hash": hashlib.sha256(args.query_json.read_bytes()).hexdigest()}, sort_keys=False))
    _atomic_write_text(output / "topology_manifest.yaml", yaml.safe_dump({"architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "static_topology": topology_info, "refined_topology": {**refined.metadata, **refined_info}}, sort_keys=False))
    _atomic_write_text(output / "metric_availability.yaml", yaml.safe_dump({"smac_costmap_start_goal": "not_available_without_costmap_readback", "start_inflated_cost": "not_available_without_costmap_readback", "goal_inflated_cost": "not_available_without_costmap_readback", "expanded_generated_states": "not_available_from_compute_path_action", "peak_pss": "not_collected_by_current_resource_monitor"}, sort_keys=False))
    report = ["# 2D-V0 pipeline smoke", "", f"- Architecture: `{ARCHITECTURE_ID}` revision `{IMPLEMENTATION_REVISION}`; protocol `{PROTOCOL_VERSION}`.", f"- Refined topology: {refined.node_count} nodes / {refined.edge_count} edges; L1 states are topology node ids.", "- L2 calls: 0; RRTstar/SST calls: 0/0.", f"- Final-valid: {sum(bool(row['final_valid_success']) for row in rows)}/{len(rows)}.", f"- Failure counts: {json.dumps(failure_counts, sort_keys=True)}.", f"- Smac calls: {adapter.calls}; session start/close/restart={session.session_start_count}/{session.session_close_count}/{session.session_restart_count}.", f"- Static topology cache: hit={topology_info.get('topology_cache_hit', False)}, build_count={topology_info.get('topology_build_count', 0)}; refined cache state={refined_info.get('cache_state', 'not_available')}.", "- Dynamic snapshots are an overlay and never modify the static occupancy or refined topology.", "- Smac costmap start/goal costs and expanded/generated states are not available from the current action interface; fields are explicitly marked not_available.", "- Static smoke gate: NOT PASSED; dynamic P1/P2/P3 smoke is not started.", "- See runs.csv, failure_summary.csv, metric_availability.yaml and path_metrics.csv for per-query diagnostics."]
    _atomic_write_text(output / "final_report.md", "\n".join(report) + "\n")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.demo:
        result = _demo()
        _write_demo_outputs(args.output_dir, result)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    if args.corridor_diagnostic:
        return _run_corridor_diagnostic(args)
    return _run_ros_smoke(args)


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "ARCHITECTURE_ID", "IMPLEMENTATION_REVISION", "PROTOCOL_VERSION", "AttachmentCandidate", "Layered2DResult", "Layered2DV0Pipeline", "RefinedEdge", "RefinedNode", "RefinedNodeSpatialIndex", "RefinedTopology", "SmacHybridAdapter", "_classify_smac_failure", "attachment_candidates", "build_parser", "build_refined_topology", "corridor_mask_for_route", "dynamic_collision_count", "load_refined_topology", "main", "prepare_refined_topology", "prepare_static_topology", "save_refined_topology",
]
