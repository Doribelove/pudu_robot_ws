"""L1 semantic edge annotations and deterministic semantic graph routing."""

from __future__ import annotations

import heapq
import math
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from .semantic_map import SemanticMapV1, canonical_hash, point_in_polygon
from .semantic_rasterizer import RasterizedSemantics
from .topology import TopologyEdge, TopologyRoute


DEFAULT_EDGE_POLICY: Dict[str, Any] = {
    "semantic_costs_enabled": True,
    "hard_semantics_enabled": True,
    "class_cost_per_m": {
        "unlabelled": 0.15,
        "lane": 0.0,
        "junction_area": 0.05,
        "parking_area": 0.10,
        "speed_bumps": 2.0,
        "fence_area": 0.0,
        "no_stopping": 0.0,
    },
    "zone_entry_penalty": 0.20,
    "wrong_way_penalty_per_m": 10.0,
    "sample_spacing_m": 0.05,
}


def topology_graph_hash(topology: Any) -> str:
    return canonical_hash({
        "nodes": [
            [int(node.node_id), float(node.x), float(node.y), int(node.component_id)]
            for node in topology.graph.nodes
        ],
        "edges": [
            [
                int(edge.edge_id), int(edge.source), int(edge.target),
                float(edge.length_m), edge.polyline,
            ]
            for edge in topology.graph.edges
        ],
    })


def semantic_edge_cache_key(
    *, base_map_hash: str, semantic_map_hash: str, policy_hash: str,
    topology_hash: str, direction_signature: str,
) -> str:
    return canonical_hash({
        "base_map_hash": str(base_map_hash),
        "semantic_map_hash": str(semantic_map_hash),
        "policy_hash": str(policy_hash),
        "topology_graph_hash": str(topology_hash),
        "direction_signature": str(direction_signature),
    })


@dataclass
class RegionCoverage:
    semantic_class: str
    semantic_ids: List[str]
    length_m: float
    direction_relation: str


@dataclass
class EdgeSemanticAnnotation:
    edge_id: int
    traversal_reversed: bool
    base_length_m: float
    region_coverage: List[RegionCoverage] = field(default_factory=list)
    semantic_integral: float = 0.0
    zone_entry_penalty: float = 0.0
    direction_penalty: float = 0.0
    total_cost: float = 0.0
    blocked: bool = False
    block_reason: str = ""
    cache_key: str = ""

    def to_dict(self) -> Dict[str, Any]:
        value = asdict(self)
        value["region_coverage"] = [asdict(item) for item in self.region_coverage]
        return value


class EdgeSemanticAnnotator:
    """Annotate each directed traversal using the normalized semantic map."""

    def __init__(
        self, hospital_map: Any, semantic_map: SemanticMapV1,
        raster: RasterizedSemantics, *, base_map_hash: str,
        topology_hash: str, policy: Optional[Mapping[str, Any]] = None,
    ) -> None:
        semantic_map.validate_against_map(hospital_map)
        self.hospital_map = hospital_map
        self.semantic_map = semantic_map
        # SemanticMapV1 computes its canonical hash from the complete feature
        # payload.  It is immutable for this annotator, so never recompute that
        # O(map-size) value for every topology edge.
        self.semantic_map_hash = semantic_map.semantic_map_hash
        self.raster = raster
        self.base_map_hash = str(base_map_hash)
        self.topology_hash = str(topology_hash)
        self.policy = {
            **DEFAULT_EDGE_POLICY,
            **dict(policy or {}),
            "class_cost_per_m": {
                **DEFAULT_EDGE_POLICY["class_cost_per_m"],
                **dict((policy or {}).get("class_cost_per_m") or {}),
            },
        }
        self.policy_hash = canonical_hash(self.policy)
        self._cache: Dict[Tuple[int, bool], EdgeSemanticAnnotation] = {}
        self._feature_bounds = [
            (
                feature,
                min(point[0] for point in feature.coordinates),
                min(point[1] for point in feature.coordinates),
                max(point[0] for point in feature.coordinates),
                max(point[1] for point in feature.coordinates),
            )
            for feature in semantic_map.features if feature.geometry_type == "polygon"
        ]
        self._features_by_class: Dict[str, List[Tuple[Any, float, float, float, float]]] = {}
        for value in self._feature_bounds:
            self._features_by_class.setdefault(value[0].semantic_class, []).append(value)

    def _features_at(self, point: Sequence[float], semantic_class: str) -> List[Any]:
        x, y = float(point[0]), float(point[1])
        return [
            feature for feature, min_x, min_y, max_x, max_y in self._feature_bounds
            if feature.semantic_class == semantic_class
            and min_x <= x <= max_x and min_y <= y <= max_y
            and point_in_polygon((x, y), feature.coordinates)
        ]

    @staticmethod
    def _points_in_polygon(xs: np.ndarray, ys: np.ndarray, coordinates: Sequence[Sequence[float]]) -> np.ndarray:
        """Vectorized even/odd containment for edge sample points."""
        inside = np.zeros(xs.shape, dtype=bool)
        points = np.asarray(coordinates, dtype=np.float64)
        if len(points) < 3:
            return inside
        xj, yj = points[-1]
        for xi, yi in points:
            crosses = ((yi > ys) != (yj > ys)) & (
                xs < (xj - xi) * (ys - yi) / ((yj - yi) + 1.0e-300) + xi
            )
            inside ^= crosses
            xj, yj = xi, yi
        return inside

    def _sample_polyline(
        self, points: Sequence[Sequence[float]], spacing: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        polyline = np.asarray(points, dtype=np.float64)
        if polyline.ndim != 2 or polyline.shape[0] < 2:
            empty = np.asarray([], dtype=np.float64)
            return empty, empty, empty, empty, empty
        starts = polyline[:-1]
        deltas = np.diff(polyline, axis=0)
        segment_lengths = np.hypot(deltas[:, 0], deltas[:, 1])
        nonzero = segment_lengths > 1.0e-12
        starts, deltas, segment_lengths = starts[nonzero], deltas[nonzero], segment_lengths[nonzero]
        if not len(segment_lengths):
            empty = np.asarray([], dtype=np.float64)
            return empty, empty, empty, empty, empty
        counts = np.maximum(1, np.ceil(segment_lengths / spacing).astype(np.int64))
        segment_indices = np.repeat(np.arange(len(counts), dtype=np.int64), counts)
        repeated_counts = counts[segment_indices]
        first_offsets = np.repeat(np.cumsum(counts) - counts, counts)
        within_segment = np.arange(int(np.sum(counts)), dtype=np.int64) - first_offsets
        fractions = (within_segment.astype(np.float64) + 0.5) / repeated_counts
        sample_points = starts[segment_indices] + fractions[:, None] * deltas[segment_indices]
        unit_lengths = segment_lengths[segment_indices] / repeated_counts
        tangents = deltas[segment_indices] / segment_lengths[segment_indices, None]
        return (
            sample_points[:, 0], sample_points[:, 1], unit_lengths,
            tangents[:, 0], tangents[:, 1],
        )

    @staticmethod
    def _explicit_relation(feature: Any, tangent: Tuple[float, float]) -> str:
        value = feature.properties.get("explicit_direction")
        direction: Optional[Tuple[float, float]] = None
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            direction = (float(value[0]), float(value[1]))
        elif isinstance(value, Mapping) and "x" in value and "y" in value:
            direction = (float(value["x"]), float(value["y"]))
        elif isinstance(value, (int, float)):
            direction = (math.cos(float(value)), math.sin(float(value)))
        if direction is None:
            return "explicit_unparseable"
        norm = math.hypot(*direction)
        if norm <= 1e-12:
            return "explicit_unparseable"
        dot = tangent[0] * direction[0] / norm + tangent[1] * direction[1] / norm
        return "with_direction" if dot >= 0.0 else "against_direction"

    def annotate(self, edge: TopologyEdge, *, reversed_traversal: bool = False) -> EdgeSemanticAnnotation:
        key = (int(edge.edge_id), bool(reversed_traversal))
        if key in self._cache:
            return self._cache[key]
        points = list(reversed(edge.polyline)) if reversed_traversal else list(edge.polyline)
        spacing = max(0.01, float(self.policy["sample_spacing_m"]))
        coverage: Dict[str, float] = {}
        ids: Dict[str, set[str]] = {}
        relations: Dict[str, set[str]] = {}
        semantic_integral = 0.0
        direction_penalty = 0.0
        xs, ys, unit_lengths, tangent_x, tangent_y = self._sample_polyline(points, spacing)
        origin_x, origin_y = float(self.hospital_map.origin[0]), float(self.hospital_map.origin[1])
        resolution = float(self.hospital_map.resolution)
        cols = np.floor((xs - origin_x) / resolution).astype(np.int64)
        rows_from_bottom = np.floor((ys - origin_y) / resolution).astype(np.int64)
        rows = int(self.hospital_map.height) - 1 - rows_from_bottom
        valid = (
            (rows >= 0) & (rows < int(self.hospital_map.height))
            & (cols >= 0) & (cols < int(self.hospital_map.width))
        )
        blocked_samples = ~valid
        if np.any(valid):
            blocked_samples[valid] |= self.raster.hard_footprint_mask[rows[valid], cols[valid]]
        blocked = bool(np.any(blocked_samples)) and bool(self.policy["hard_semantics_enabled"])
        block_reason = "HARD_SEMANTIC_FOOTPRINT_CONFLICT" if blocked else ""

        semantic_classes = sorted(self.raster.masks)
        active_rows: List[np.ndarray] = []
        for semantic_class in semantic_classes:
            active = np.zeros(xs.shape, dtype=bool)
            if np.any(valid):
                active[valid] = self.raster.masks[semantic_class][rows[valid], cols[valid]]
            active_rows.append(active)
        active_matrix = np.vstack(active_rows) if active_rows else np.zeros((0, len(xs)), dtype=bool)
        unlabelled = ~np.any(active_matrix, axis=0)
        if np.any(unlabelled):
            semantic_classes.append("unlabelled")
            active_matrix = np.vstack((active_matrix, unlabelled))
        entries = int(np.count_nonzero(active_matrix[:, 0])) if active_matrix.shape[1] else 0
        if active_matrix.shape[1] > 1:
            entries += int(np.count_nonzero(active_matrix[:, 1:] & ~active_matrix[:, :-1]))

        for class_index, semantic_class in enumerate(semantic_classes):
            active = active_matrix[class_index]
            if not np.any(active):
                continue
            length = float(np.sum(unit_lengths[active]))
            coverage[semantic_class] = length
            if self.policy["semantic_costs_enabled"]:
                semantic_integral += length * float(
                    self.policy["class_cost_per_m"].get(semantic_class, 0.0)
                )
            if semantic_class == "unlabelled":
                continue
            for feature, min_x, min_y, max_x, max_y in self._features_by_class.get(semantic_class, []):
                candidates = active & (xs >= min_x) & (xs <= max_x) & (ys >= min_y) & (ys <= max_y)
                if not np.any(candidates):
                    continue
                candidate_indices = np.flatnonzero(candidates)
                contained = self._points_in_polygon(
                    xs[candidate_indices], ys[candidate_indices], feature.coordinates,
                )
                if not np.any(contained):
                    continue
                feature_indices = candidate_indices[contained]
                ids.setdefault(semantic_class, set()).add(feature.semantic_id)
                if feature.direction_rule == "explicit":
                    value = feature.properties.get("explicit_direction")
                    direction: Optional[Tuple[float, float]] = None
                    if isinstance(value, (list, tuple)) and len(value) >= 2:
                        direction = (float(value[0]), float(value[1]))
                    elif isinstance(value, Mapping) and "x" in value and "y" in value:
                        direction = (float(value["x"]), float(value["y"]))
                    elif isinstance(value, (int, float)):
                        direction = (math.cos(float(value)), math.sin(float(value)))
                    if direction is None or math.hypot(*direction) <= 1.0e-12:
                        relations.setdefault(semantic_class, set()).add("explicit_unparseable")
                    else:
                        norm = math.hypot(*direction)
                        with_direction = (
                            tangent_x[feature_indices] * direction[0] / norm
                            + tangent_y[feature_indices] * direction[1] / norm
                        ) >= 0.0
                        if np.any(with_direction):
                            relations.setdefault(semantic_class, set()).add("with_direction")
                        if np.any(~with_direction):
                            relations.setdefault(semantic_class, set()).add("against_direction")
                            if self.policy["semantic_costs_enabled"]:
                                direction_penalty += float(np.sum(unit_lengths[feature_indices[~with_direction]])) * float(
                                    self.policy["wrong_way_penalty_per_m"]
                                )
                elif feature.direction_rule == "route_tangent_right":
                    relations.setdefault(semantic_class, set()).add("query_route_tangent")
        region_coverage = [
            RegionCoverage(
                semantic_class=semantic_class,
                semantic_ids=sorted(ids.get(semantic_class, set())),
                length_m=float(length),
                direction_relation="+".join(sorted(relations.get(semantic_class, {"not_directional"}))),
            )
            for semantic_class, length in sorted(coverage.items())
        ]
        entry_penalty = (
            max(0, entries - (1 if coverage else 0)) * float(self.policy["zone_entry_penalty"])
            if self.policy["semantic_costs_enabled"] else 0.0
        )
        direction_signature = "reverse" if reversed_traversal else "forward"
        result = EdgeSemanticAnnotation(
            edge_id=int(edge.edge_id), traversal_reversed=bool(reversed_traversal),
            base_length_m=float(edge.length_m), region_coverage=region_coverage,
            semantic_integral=float(semantic_integral),
            zone_entry_penalty=float(entry_penalty),
            direction_penalty=float(direction_penalty),
            total_cost=float("inf") if blocked else float(edge.length_m + semantic_integral + entry_penalty + direction_penalty),
            blocked=blocked, block_reason=block_reason,
            cache_key=semantic_edge_cache_key(
                base_map_hash=self.base_map_hash,
                semantic_map_hash=self.semantic_map_hash,
                policy_hash=self.policy_hash,
                topology_hash=self.topology_hash,
                direction_signature=f"edge={edge.edge_id}:{direction_signature}",
            ),
        )
        self._cache[key] = result
        return result

    def precompute(self, edges: Sequence[TopologyEdge]) -> None:
        """Populate both directed annotations once before online queries."""
        for edge in edges:
            self.annotate(edge, reversed_traversal=False)
            self.annotate(edge, reversed_traversal=True)


class SemanticEdgeRouter:
    def __init__(self, topology: Any, annotator: EdgeSemanticAnnotator) -> None:
        self.topology = topology
        self.annotator = annotator
        self._multi_route_cache: Dict[
            Tuple[Tuple[int, ...], Tuple[int, ...], Tuple[Tuple[int, float], ...], Tuple[Tuple[int, float], ...]],
            Optional[Tuple[TopologyRoute, int, int]],
        ] = {}

    def search(self, start_id: int, goal_id: int) -> Optional[TopologyRoute]:
        selected = self.search_any([int(start_id)], [int(goal_id)])
        return selected[0] if selected is not None else None

    def search_any(
        self, start_ids: Sequence[int], goal_ids: Sequence[int], *,
        start_costs: Optional[Mapping[int, float]] = None,
        goal_costs: Optional[Mapping[int, float]] = None,
    ) -> Optional[Tuple[TopologyRoute, int, int]]:
        """One semantic Dijkstra including optional endpoint costs."""
        ordered_starts = tuple(dict.fromkeys(int(value) for value in start_ids))
        ordered_goals = tuple(dict.fromkeys(int(value) for value in goal_ids))
        start_weights = {
            node_id: max(0.0, float((start_costs or {}).get(node_id, 0.0)))
            for node_id in ordered_starts
        }
        goal_weights = {
            node_id: max(0.0, float((goal_costs or {}).get(node_id, 0.0)))
            for node_id in ordered_goals
        }
        cache_key = (
            ordered_starts, ordered_goals,
            tuple(sorted(start_weights.items())), tuple(sorted(goal_weights.items())),
        )
        if cache_key in self._multi_route_cache:
            return self._multi_route_cache[cache_key]
        if not ordered_starts or not ordered_goals:
            self._multi_route_cache[cache_key] = None
            return None
        adjacency = self.topology.graph.adjacency()
        queue: List[Tuple[float, int]] = [(start_weights[value], value) for value in ordered_starts]
        heapq.heapify(queue)
        distances: Dict[int, float] = {value: start_weights[value] for value in ordered_starts}
        origins: Dict[int, int] = {value: value for value in ordered_starts}
        previous: Dict[int, Tuple[int, TopologyEdge, bool]] = {}
        goal_set = set(ordered_goals)
        selected_goal: Optional[int] = None
        selected_total = float("inf")
        while queue:
            cost, node_id = heapq.heappop(queue)
            if cost != distances.get(node_id):
                continue
            if cost >= selected_total:
                break
            if node_id in goal_set:
                total = cost + goal_weights[node_id]
                if total < selected_total or (
                    total == selected_total and (selected_goal is None or node_id < selected_goal)
                ):
                    selected_goal = node_id
                    selected_total = total
            for target, edge, reversed_traversal in adjacency.get(node_id, []):
                annotation = self.annotator.annotate(edge, reversed_traversal=reversed_traversal)
                if annotation.blocked:
                    continue
                candidate = cost + annotation.total_cost
                if candidate < distances.get(int(target), float("inf")):
                    distances[int(target)] = candidate
                    previous[int(target)] = (int(node_id), edge, bool(reversed_traversal))
                    origins[int(target)] = origins[int(node_id)]
                    heapq.heappush(queue, (candidate, int(target)))
        if selected_goal is None:
            self._multi_route_cache[cache_key] = None
            return None
        selected_start = origins[selected_goal]
        if selected_start == selected_goal:
            node = next(item for item in self.topology.graph.nodes if int(item.node_id) == selected_start)
            route = TopologyRoute([selected_start], [], 0.0, float(node.channel_width_m), [[node.x, node.y]])
            setattr(route, "semantic_cost", float(selected_total))
            setattr(route, "endpoint_attachment_cost_m", float(
                start_weights[selected_start] + goal_weights[selected_goal]
            ))
            setattr(route, "semantic_edge_annotations", [])
            result = (route, selected_start, selected_goal)
            self._multi_route_cache[cache_key] = result
            return result
        node_ids = [selected_goal]
        edge_values: List[Tuple[TopologyEdge, bool]] = []
        cursor = selected_goal
        while cursor != selected_start:
            parent, edge, reversed_traversal = previous[cursor]
            edge_values.append((edge, reversed_traversal))
            node_ids.append(parent)
            cursor = parent
        node_ids.reverse()
        edge_values.reverse()
        polyline: List[List[float]] = []
        physical_length = 0.0
        min_width = float("inf")
        annotations = []
        for edge, reversed_traversal in edge_values:
            points = list(reversed(edge.polyline)) if reversed_traversal else list(edge.polyline)
            if polyline and points:
                points = points[1:]
            polyline.extend([list(point) for point in points])
            physical_length += float(edge.length_m)
            min_width = min(min_width, float(edge.min_width_m))
            annotations.append(self.annotator.annotate(edge, reversed_traversal=reversed_traversal).to_dict())
        route = TopologyRoute(
            node_ids=node_ids,
            edge_ids=[int(edge.edge_id) for edge, _ in edge_values],
            length_m=physical_length,
            min_width_m=0.0 if min_width == float("inf") else min_width,
            polyline=polyline,
        )
        setattr(route, "semantic_cost", float(selected_total))
        setattr(route, "endpoint_attachment_cost_m", float(
            start_weights[selected_start] + goal_weights[selected_goal]
        ))
        setattr(route, "semantic_edge_annotations", annotations)
        setattr(route, "semantic_policy_hash", self.annotator.policy_hash)
        result = (route, selected_start, selected_goal)
        self._multi_route_cache[cache_key] = result
        return result


__all__ = [
    "DEFAULT_EDGE_POLICY", "RegionCoverage", "EdgeSemanticAnnotation",
    "EdgeSemanticAnnotator", "SemanticEdgeRouter", "topology_graph_hash",
    "semantic_edge_cache_key",
]
