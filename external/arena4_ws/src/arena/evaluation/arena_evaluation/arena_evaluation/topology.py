"""Static Hospital map topology extraction and topology-guided grid search.

This module deliberately has no ROS dependency.  The input is only a static
occupancy map and the fixed Jackal footprint.  A topology-preserving NumPy
Zhang-Suen implementation is used when scikit-image is unavailable.
"""

from __future__ import annotations

import gzip
import hashlib
import heapq
import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

from .planner_benchmark.map_utils import HospitalMap, sha256_file


TOPOLOGY_ALGORITHM_VERSION = "skeleton_distance_transform_v1"
TOPOLOGY_FAILURE_CODES = {
    "TOPOLOGY_BUILD_FAILED",
    "TOPOLOGY_EMPTY_GRAPH",
    "TOPOLOGY_START_NOT_ATTACHABLE",
    "TOPOLOGY_GOAL_NOT_ATTACHABLE",
    "TOPOLOGY_COMPONENT_MISMATCH",
    "TOPOLOGY_NO_ROUTE",
    "CORRIDOR_NO_PATH",
    "CORRIDOR_EXPANDED",
    "FULL_GRID_FALLBACK",
    "FULL_GRID_FAILED",
    "STATIC_FOOTPRINT_COLLISION",
    "TOPOLOGY_FALSE_FAILURE",
}

Cell = Tuple[int, int]
WorldPoint = Tuple[float, float]
_NEIGHBORS = (
    (-1, -1), (-1, 0), (-1, 1), (0, -1),
    (0, 1), (1, -1), (1, 0), (1, 1),
)


@dataclass
class TopologyNode:
    node_id: int
    x: float
    y: float
    pixel_x: int
    pixel_y: int
    degree: int
    clearance_m: float
    channel_width_m: float
    component_id: int


@dataclass
class TopologyEdge:
    edge_id: int
    source: int
    target: int
    length_m: float
    min_clearance_m: float
    mean_clearance_m: float
    min_width_m: float
    pixel_count: int
    polyline: List[List[float]]


@dataclass
class TopologyGraph:
    nodes: List[TopologyNode] = field(default_factory=list)
    edges: List[TopologyEdge] = field(default_factory=list)

    @property
    def components(self) -> int:
        return len({node.component_id for node in self.nodes})

    def adjacency(self) -> Dict[int, List[Tuple[int, TopologyEdge, bool]]]:
        result: Dict[int, List[Tuple[int, TopologyEdge, bool]]] = {node.node_id: [] for node in self.nodes}
        for edge in self.edges:
            result.setdefault(edge.source, []).append((edge.target, edge, False))
            result.setdefault(edge.target, []).append((edge.source, edge, True))
        return result


@dataclass
class TopologyArtifact:
    hospital_map: HospitalMap
    free_mask: np.ndarray
    skeleton: np.ndarray
    distance_m: np.ndarray
    free_components: np.ndarray
    graph: TopologyGraph
    metadata: Dict[str, object]

    @property
    def height(self) -> int:
        return int(self.free_mask.shape[0])

    @property
    def width(self) -> int:
        return int(self.free_mask.shape[1])


@dataclass
class Attachment:
    node_id: int
    distance_m: float
    component_id: int


@dataclass
class TopologyRoute:
    node_ids: List[int]
    edge_ids: List[int]
    length_m: float
    min_width_m: float
    polyline: List[List[float]]


@dataclass
class AStarResult:
    """Path and deterministic search accounting for one grid A* query."""

    path: Optional[List[Cell]]
    expanded_nodes: int
    generated_nodes: int
    max_open_set_size: int
    allowed_grid_cells: int
    total_free_grid_cells: int
    search_space_ratio: float
    path_cost: Optional[float]
    search_time_ms: float
    failure_code: str = ""
    configured_timeout_s: Optional[float] = None
    timeout_triggered: bool = False
    timeout_checks: int = 0


def footprint_hash(footprint: Sequence[Sequence[float]]) -> str:
    payload = json.dumps([[float(x), float(y)] for x, y in footprint], separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def map_input_hash(map_yaml: Path, image_path: Path) -> str:
    digest = hashlib.sha256()
    for path in (map_yaml, image_path):
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _footprint_kernel(
    resolution: float,
    footprint: Sequence[Sequence[float]],
    padding_m: float,
    safety_margin_m: float,
    angle_count: int = 24,
) -> np.ndarray:
    """Rasterize the complete rectangular footprint over all headings."""
    points = np.asarray(footprint, dtype=np.float64)
    radius = float(np.max(np.hypot(points[:, 0], points[:, 1]))) + padding_m + safety_margin_m
    half_cells = int(math.ceil(radius / resolution)) + 2
    size = 2 * half_cells + 1
    kernel = np.zeros((size, size), dtype=np.uint8)
    center = half_cells
    margin_cells = int(math.ceil((padding_m + safety_margin_m) / resolution))
    for angle in np.linspace(0.0, math.pi, max(4, angle_count), endpoint=False):
        cos_a, sin_a = math.cos(float(angle)), math.sin(float(angle))
        transformed = []
        for px, py in points:
            world_x = cos_a * px - sin_a * py
            world_y = sin_a * px + cos_a * py
            col = int(round(center + world_x / resolution))
            row = int(round(center - world_y / resolution))
            transformed.append([col, row])
        cv2.fillPoly(kernel, [np.asarray(transformed, dtype=np.int32)], 1)
    if margin_cells:
        diameter = 2 * margin_cells + 1
        disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (diameter, diameter))
        kernel = cv2.dilate(kernel, disk)
    kernel[center, center] = 1
    return kernel


def preprocess_static_map(
    hospital_map: HospitalMap,
    footprint: Sequence[Sequence[float]],
    *,
    padding_m: float = 0.05,
    safety_margin_m: float = 0.05,
    allow_unknown: bool = False,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return inflated obstacles, free mask, distance transform and components."""
    base_obstacle = hospital_map.occupancy == 100
    if not allow_unknown:
        base_obstacle |= hospital_map.occupancy < 0
    kernel = _footprint_kernel(hospital_map.resolution, footprint, padding_m, safety_margin_m)
    inflated = cv2.dilate(base_obstacle.astype(np.uint8), kernel, iterations=1).astype(bool)
    free = ~inflated
    if not allow_unknown:
        free &= hospital_map.occupancy >= 0
    distance_m = cv2.distanceTransform(free.astype(np.uint8), cv2.DIST_L2, 5) * float(hospital_map.resolution)
    component_count, components = cv2.connectedComponents(free.astype(np.uint8), connectivity=8)
    del component_count
    return inflated, free, distance_m.astype(np.float32), components.astype(np.int32)


def _zhang_suen_skeleton(binary: np.ndarray) -> np.ndarray:
    """Topology-preserving Zhang-Suen thinning implemented with NumPy."""
    image = binary.astype(np.uint8).copy()
    image[[0, -1], :] = 0
    image[:, [0, -1]] = 0
    while True:
        changed = False
        for phase in (0, 1):
            p2 = np.roll(image, -1, axis=0)
            p3 = np.roll(p2, -1, axis=1)
            p4 = np.roll(image, -1, axis=1)
            p5 = np.roll(p4, 1, axis=0)
            p6 = np.roll(image, 1, axis=0)
            p7 = np.roll(p6, 1, axis=1)
            p8 = np.roll(image, 1, axis=1)
            p9 = np.roll(p2, 1, axis=1)
            neighbors = np.stack([p2, p3, p4, p5, p6, p7, p8, p9], axis=0)
            neighbor_count = neighbors.sum(axis=0)
            transitions = sum(
                ((neighbors[index] == 0) & (neighbors[(index + 1) % 8] == 1))
                for index in range(8)
            )
            if phase == 0:
                condition = (
                    (image == 1) & (neighbor_count >= 2) & (neighbor_count <= 6)
                    & (transitions == 1) & ((p2 * p4 * p6) == 0) & ((p4 * p6 * p8) == 0)
                )
            else:
                condition = (
                    (image == 1) & (neighbor_count >= 2) & (neighbor_count <= 6)
                    & (transitions == 1) & ((p2 * p4 * p8) == 0) & ((p2 * p6 * p8) == 0)
                )
            if np.any(condition):
                image[condition] = 0
                changed = True
        if not changed:
            break
    return image.astype(bool)


def extract_skeleton(free_mask: np.ndarray) -> np.ndarray:
    """Use scikit-image when available, otherwise topology-preserving thinning."""
    try:
        from skimage.morphology import skeletonize  # type: ignore

        return np.asarray(skeletonize(free_mask), dtype=bool)
    except ImportError:
        return _zhang_suen_skeleton(free_mask)


def _neighbors(cell: Cell, shape: Tuple[int, int]) -> Iterable[Cell]:
    row, col = cell
    for dr, dc in _NEIGHBORS:
        candidate = (row + dr, col + dc)
        if 0 <= candidate[0] < shape[0] and 0 <= candidate[1] < shape[1]:
            yield candidate


def _cell_distance(a: Cell, b: Cell) -> float:
    return math.hypot(float(a[0] - b[0]), float(a[1] - b[1]))


def _representative(coords: np.ndarray) -> Cell:
    center = coords.mean(axis=0)
    index = int(np.argmin(np.sum((coords - center) ** 2, axis=1)))
    return int(coords[index, 0]), int(coords[index, 1])


def build_graph(
    skeleton: np.ndarray,
    distance_m: np.ndarray,
    resolution: float,
    hospital_map: HospitalMap,
    *,
    loop_sample_spacing_m: float = 2.0,
) -> TopologyGraph:
    """Compress degree-2 skeleton runs into topology edges."""
    graph = TopologyGraph()
    if not np.any(skeleton):
        return graph
    _, component_labels = cv2.connectedComponents(skeleton.astype(np.uint8), connectivity=8)
    node_at = np.full(skeleton.shape, -1, dtype=np.int32)
    component_for_node: Dict[int, int] = {}
    for component_id in range(1, int(component_labels.max()) + 1):
        coords = np.argwhere(component_labels == component_id)
        if len(coords) == 0:
            continue
        component_set = {tuple(map(int, cell)) for cell in coords}
        key_mask = np.zeros(skeleton.shape, dtype=np.uint8)
        for row, col in component_set:
            degree = sum(candidate in component_set for candidate in _neighbors((row, col), skeleton.shape))
            if degree != 2:
                key_mask[row, col] = 1
        if not np.any(key_mask):
            stride = max(1, int(round(loop_sample_spacing_m / resolution)))
            for row, col in coords[::stride]:
                key_mask[int(row), int(col)] = 1
        cluster_count, cluster_labels = cv2.connectedComponents(key_mask, connectivity=8)
        for cluster_id in range(1, cluster_count):
            cluster_coords = np.argwhere(cluster_labels == cluster_id)
            if len(cluster_coords) == 0:
                continue
            row, col = _representative(cluster_coords)
            node_id = len(graph.nodes)
            node_at[cluster_labels == cluster_id] = node_id
            x, y = hospital_map.cell_to_world((row, col))
            clearance = float(distance_m[row, col])
            graph.nodes.append(TopologyNode(
                node_id=node_id,
                x=x,
                y=y,
                pixel_x=col,
                pixel_y=row,
                degree=0,
                clearance_m=clearance,
                channel_width_m=2.0 * clearance,
                component_id=component_id,
            ))
            component_for_node[node_id] = component_id

    edge_seen = set()
    for node in list(graph.nodes):
        cluster_pixels = np.argwhere(node_at == node.node_id)
        for start_row, start_col in cluster_pixels:
            start = (int(start_row), int(start_col))
            for first in _neighbors(start, skeleton.shape):
                if not skeleton[first] or node_at[first] == node.node_id:
                    continue
                path = [start, first]
                previous, current = start, first
                target = int(node_at[current])
                guard = 0
                while target < 0 and guard < skeleton.size:
                    guard += 1
                    candidates = [candidate for candidate in _neighbors(current, skeleton.shape)
                                  if skeleton[candidate] and candidate != previous]
                    if not candidates:
                        break
                    previous, current = current, candidates[0]
                    path.append(current)
                    target = int(node_at[current])
                if target < 0 or target == node.node_id:
                    continue
                endpoint_key = frozenset((node.node_id, target))
                if endpoint_key in edge_seen:
                    continue
                edge_seen.add(endpoint_key)
                length_cells = sum(_cell_distance(a, b) for a, b in zip(path, path[1:]))
                clearances = [float(distance_m[row, col]) for row, col in path]
                polyline = [list(hospital_map.cell_to_world(cell)) for cell in path]
                graph.edges.append(TopologyEdge(
                    edge_id=len(graph.edges),
                    source=node.node_id,
                    target=target,
                    length_m=length_cells * resolution,
                    min_clearance_m=min(clearances, default=0.0),
                    mean_clearance_m=float(np.mean(clearances)) if clearances else 0.0,
                    min_width_m=2.0 * min(clearances, default=0.0),
                    pixel_count=len(path),
                    polyline=polyline,
                ))
    degrees = {node.node_id: 0 for node in graph.nodes}
    for edge in graph.edges:
        degrees[edge.source] += 1
        degrees[edge.target] += 1
    for node in graph.nodes:
        node.degree = degrees[node.node_id]
    return graph


def build_topology(
    hospital_map: HospitalMap,
    footprint: Sequence[Sequence[float]],
    *,
    padding_m: float = 0.05,
    safety_margin_m: float = 0.05,
    allow_unknown: bool = False,
) -> TopologyArtifact:
    inflated, free, distance_m, components = preprocess_static_map(
        hospital_map,
        footprint,
        padding_m=padding_m,
        safety_margin_m=safety_margin_m,
        allow_unknown=allow_unknown,
    )
    del inflated
    skeleton = extract_skeleton(free)
    graph = build_graph(skeleton, distance_m, hospital_map.resolution, hospital_map)
    metadata = {
        "schema_version": 1,
        "map_id": hospital_map.map_id,
        "map_sha256": map_input_hash(hospital_map.yaml_path, hospital_map.image_path),
        "resolution": hospital_map.resolution,
        "origin": list(hospital_map.origin),
        "width": hospital_map.width,
        "height": hospital_map.height,
        "footprint_hash": footprint_hash(footprint),
        "footprint": [[float(x), float(y)] for x, y in footprint],
        "padding_m": float(padding_m),
        "safety_margin_m": float(safety_margin_m),
        "allow_unknown": bool(allow_unknown),
        "algorithm": TOPOLOGY_ALGORITHM_VERSION,
        "skeleton_backend": "scikit-image" if _has_skimage() else "numpy_zhang_suen",
        "graph_nodes": len(graph.nodes),
        "graph_edges": len(graph.edges),
        "graph_components": graph.components,
    }
    return TopologyArtifact(hospital_map, free, skeleton, distance_m, components, graph, metadata)


def _has_skimage() -> bool:
    try:
        import skimage  # noqa: F401

        return True
    except ImportError:
        return False


def save_topology(artifact: TopologyArtifact, directory: str | Path) -> Path:
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    graph_payload = {
        "schema_version": 1,
        "algorithm": TOPOLOGY_ALGORITHM_VERSION,
        "nodes": [asdict(node) for node in artifact.graph.nodes],
        "edges": [asdict(edge) for edge in artifact.graph.edges],
    }
    (directory / "topology_graph.json").write_text(json.dumps(graph_payload, indent=2))
    np.savez_compressed(
        directory / "topology_arrays.npz",
        free_mask=artifact.free_mask.astype(np.uint8),
        skeleton=artifact.skeleton.astype(np.uint8),
        distance_m=artifact.distance_m.astype(np.float32),
        free_components=artifact.free_components.astype(np.int32),
    )
    (directory / "topology_metadata.yaml").write_text(yaml.safe_dump(artifact.metadata, sort_keys=False))
    return directory


def load_topology(
    directory: str | Path,
    hospital_map: HospitalMap,
    footprint: Sequence[Sequence[float]],
    *,
    padding_m: float = 0.05,
    safety_margin_m: float = 0.05,
    allow_unknown: bool = False,
) -> TopologyArtifact:
    directory = Path(directory).resolve()
    metadata = yaml.safe_load((directory / "topology_metadata.yaml").read_text()) or {}
    expected = {
        "map_sha256": map_input_hash(hospital_map.yaml_path, hospital_map.image_path),
        "map_id": hospital_map.map_id,
        "resolution": float(hospital_map.resolution),
        "origin": list(hospital_map.origin),
        "width": int(hospital_map.width),
        "height": int(hospital_map.height),
        "footprint_hash": footprint_hash(footprint),
        "padding_m": float(padding_m),
        "safety_margin_m": float(safety_margin_m),
        "allow_unknown": bool(allow_unknown),
        "algorithm": TOPOLOGY_ALGORITHM_VERSION,
    }
    for key, value in expected.items():
        if metadata.get(key) != value:
            raise ValueError(f"topology artifact is stale for {key}: {metadata.get(key)!r} != {value!r}")
    arrays = np.load(directory / "topology_arrays.npz")
    payload = json.loads((directory / "topology_graph.json").read_text())
    graph = TopologyGraph(
        nodes=[TopologyNode(**node) for node in payload.get("nodes", [])],
        edges=[TopologyEdge(**edge) for edge in payload.get("edges", [])],
    )
    return TopologyArtifact(
        hospital_map=hospital_map,
        free_mask=np.asarray(arrays["free_mask"], dtype=bool),
        skeleton=np.asarray(arrays["skeleton"], dtype=bool),
        distance_m=np.asarray(arrays["distance_m"], dtype=np.float32),
        free_components=np.asarray(arrays["free_components"], dtype=np.int32),
        graph=graph,
        metadata=metadata,
    )


def attach_pose(
    artifact: TopologyArtifact,
    pose: Sequence[float],
    footprint: Sequence[Sequence[float]],
    *,
    max_radius_m: float = 5.0,
    allow_unknown: bool = False,
) -> Optional[Attachment]:
    candidates = []
    for node in artifact.graph.nodes:
        distance = math.hypot(node.x - float(pose[0]), node.y - float(pose[1]))
        if distance > max_radius_m:
            continue
        cell = (node.pixel_y, node.pixel_x)
        if not artifact.free_mask[cell]:
            continue
        if artifact.hospital_map.footprint_collision(pose=(node.x, node.y, float(pose[2])), footprint=footprint, unknown_is_collision=not allow_unknown):
            continue
        candidates.append((distance, node))
    if not candidates:
        return None
    distance, node = min(candidates, key=lambda item: item[0])
    return Attachment(node.node_id, float(distance), node.component_id)


def search_topology(artifact: TopologyArtifact, start_id: int, goal_id: int) -> Optional[TopologyRoute]:
    if start_id == goal_id:
        node = artifact.graph.nodes[start_id]
        return TopologyRoute([start_id], [], 0.0, node.channel_width_m, [[node.x, node.y]])
    adjacency = artifact.graph.adjacency()
    queue = [(0.0, start_id)]
    distances = {start_id: 0.0}
    previous: Dict[int, Tuple[int, TopologyEdge, bool]] = {}
    while queue:
        cost, node_id = heapq.heappop(queue)
        if node_id == goal_id:
            break
        if cost != distances.get(node_id):
            continue
        for target, edge, reverse in adjacency.get(node_id, []):
            candidate = cost + edge.length_m
            if candidate < distances.get(target, float("inf")):
                distances[target] = candidate
                previous[target] = (node_id, edge, reverse)
                heapq.heappush(queue, (candidate, target))
    if goal_id not in distances:
        return None
    nodes = [goal_id]
    edges: List[TopologyEdge] = []
    directions: List[bool] = []
    cursor = goal_id
    while cursor != start_id:
        parent, edge, reverse = previous[cursor]
        nodes.append(parent)
        edges.append(edge)
        directions.append(reverse)
        cursor = parent
    nodes.reverse()
    edges.reverse()
    directions.reverse()
    polyline: List[List[float]] = []
    min_width = float("inf")
    for edge, reverse in zip(edges, directions):
        points = list(reversed(edge.polyline)) if reverse else edge.polyline
        if polyline and points:
            points = points[1:]
        polyline.extend(points)
        min_width = min(min_width, edge.min_width_m)
    if not polyline:
        polyline = [[artifact.graph.nodes[node].x, artifact.graph.nodes[node].y] for node in nodes]
    return TopologyRoute(nodes, [edge.edge_id for edge in edges], distances[goal_id], 0.0 if min_width == float("inf") else min_width, polyline)


def _kernel_for_radius(radius_m: float, resolution: float) -> np.ndarray:
    radius_cells = max(1, int(math.ceil(radius_m / resolution)))
    diameter = 2 * radius_cells + 1
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (diameter, diameter))


def corridor_mask(
    artifact: TopologyArtifact,
    route: TopologyRoute,
    start_cell: Cell,
    goal_cell: Cell,
    padding_m: float,
) -> np.ndarray:
    mask = np.zeros_like(artifact.free_mask, dtype=np.uint8)
    for x, y in route.polyline:
        cell = artifact.hospital_map.world_to_cell(x, y)
        if cell is not None:
            mask[cell] = 1
    mask[start_cell] = 1
    mask[goal_cell] = 1
    mask = cv2.dilate(mask, _kernel_for_radius(padding_m, artifact.hospital_map.resolution)) > 0
    return mask & artifact.free_mask


def astar_grid(
    free_mask: np.ndarray,
    start: Cell,
    goal: Cell,
    allowed_mask: Optional[np.ndarray] = None,
    *,
    resolution: float = 1.0,
    return_stats: bool = False,
    timeout_s: Optional[float] = None,
) -> Optional[List[Cell]] | AStarResult:
    """Run the fixed 8-connected Euclidean A* used by every L2 mode.

    The historical path-only return remains the default.  Phase 6 callers use
    ``return_stats=True`` so the exact same search also reports its work.
    """
    started = time.monotonic_ns()
    deadline_ns = (started + int(float(timeout_s) * 1e9)) if timeout_s is not None else None
    shape = free_mask.shape
    total_free = int(np.count_nonzero(free_mask))
    effective_allowed = free_mask if allowed_mask is None else (free_mask & allowed_mask)
    allowed_count = int(np.count_nonzero(effective_allowed))
    ratio = float(allowed_count / total_free) if total_free else 0.0
    expanded = 0
    generated = 0
    max_open = 0
    timeout_checks = 0
    path: Optional[List[Cell]] = None
    failure = ""
    if not (0 <= start[0] < shape[0] and 0 <= start[1] < shape[1] and 0 <= goal[0] < shape[0] and 0 <= goal[1] < shape[1]):
        failure = "INVALID_ENDPOINT"
    elif not (free_mask[start] and free_mask[goal]):
        failure = "INVALID_ENDPOINT"
    elif allowed_mask is not None and not (allowed_mask[start] and allowed_mask[goal]):
        failure = "ENDPOINT_OUTSIDE_ALLOWED"
    else:
        queue = [(0.0, 0.0, start)]
        came_from: Dict[Cell, Cell] = {}
        costs = {start: 0.0}
        generated = 1
        max_open = 1
        while queue:
            timeout_checks += 1
            if deadline_ns is not None and time.monotonic_ns() >= deadline_ns:
                failure = "TIMEOUT"
                break
            _, cost, current = heapq.heappop(queue)
            if cost != costs.get(current):
                continue
            expanded += 1
            if current == goal:
                path = [current]
                cursor = current
                while cursor in came_from:
                    cursor = came_from[cursor]
                    path.append(cursor)
                path.reverse()
                break
            for candidate in _neighbors(current, shape):
                # Check the monotonic deadline inside the neighbor expansion
                # as well as before popping a state.  This keeps a large
                # static-grid request from crossing its configured timeout
                # while processing the final state's successors.
                timeout_checks += 1
                if deadline_ns is not None and time.monotonic_ns() >= deadline_ns:
                    failure = "TIMEOUT"
                    break
                if not effective_allowed[candidate]:
                    continue
                step = _cell_distance(current, candidate)
                new_cost = cost + step
                if new_cost >= costs.get(candidate, float("inf")):
                    continue
                if candidate not in costs:
                    generated += 1
                costs[candidate] = new_cost
                heuristic = _cell_distance(candidate, goal)
                came_from[candidate] = current
                heapq.heappush(queue, (new_cost + heuristic, new_cost, candidate))
                max_open = max(max_open, len(queue))
            if failure == "TIMEOUT":
                break
        if path is None and not failure:
            failure = "NO_PATH"
    result = AStarResult(
        path=path,
        expanded_nodes=expanded,
        generated_nodes=generated,
        max_open_set_size=max_open,
        allowed_grid_cells=allowed_count,
        total_free_grid_cells=total_free,
        search_space_ratio=ratio,
        path_cost=(float(path_length_cells(path, resolution)) if path is not None else None),
        search_time_ms=(time.monotonic_ns() - started) / 1e6,
        failure_code=failure,
        configured_timeout_s=float(timeout_s) if timeout_s is not None else None,
        timeout_triggered=failure == "TIMEOUT",
        timeout_checks=timeout_checks,
    )
    return result if return_stats else result.path


def path_length_cells(path: Sequence[Cell], resolution: float) -> float:
    return sum(_cell_distance(a, b) for a, b in zip(path, path[1:])) * resolution


def cells_to_poses(artifact: TopologyArtifact, path: Sequence[Cell], start_yaw: float, goal_yaw: float) -> List[Dict[str, float]]:
    poses = []
    for index, cell in enumerate(path):
        x, y = artifact.hospital_map.cell_to_world(cell)
        if index == 0:
            yaw = float(start_yaw)
        elif index == len(path) - 1:
            yaw = float(goal_yaw)
        else:
            prev = artifact.hospital_map.cell_to_world(path[index - 1])
            nxt = artifact.hospital_map.cell_to_world(path[index + 1])
            yaw = math.atan2(nxt[1] - prev[1], nxt[0] - prev[0])
        poses.append({"x": float(x), "y": float(y), "yaw": float(yaw)})
    return poses


def static_collision_count(
    artifact: TopologyArtifact,
    poses: Sequence[Dict[str, float]],
    footprint: Sequence[Sequence[float]],
    *,
    allow_unknown: bool = False,
) -> int:
    return sum(
        artifact.hospital_map.footprint_collision(
            (pose["x"], pose["y"], pose["yaw"]), footprint,
            unknown_is_collision=not allow_unknown,
        )
        for pose in poses
    )


def save_path(path: Path, points: Sequence[Dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(path, "wt", encoding="utf-8") as stream:
        json.dump(list(points), stream)
