"""Deterministic, map-derived query set for private real-pdmap experiments."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml
from scipy import ndimage

from .planner_benchmark.models import Query
from .semantic_map import canonical_hash
from .semantic_rasterizer import RasterizedSemantics


QUERY_SET_VERSION = "pudu_wanda_3f_semantic_queries_v1"


@dataclass
class QueryIntent:
    query_id: str
    category: str
    start_semantics: List[str]
    goal_semantics: List[str]
    footprint_safe: bool
    connected_component: int
    purpose_verified: bool
    verification: Dict[str, Any]


def _classes(raster: RasterizedSemantics, cell: Tuple[int, int]) -> List[str]:
    return sorted(
        key for key, mask in raster.masks.items() if bool(mask[cell])
    ) or ["unlabelled"]


def _candidate_cells(mask: np.ndarray, components: np.ndarray, *, limit: int = 6000) -> np.ndarray:
    cells = np.argwhere(mask & (components > 0))
    if len(cells) > limit:
        indices = np.linspace(0, len(cells) - 1, limit).astype(np.int64)
        cells = cells[indices]
    return cells


def _largest_common_component(*cell_sets: np.ndarray, components: np.ndarray) -> int:
    shared: Optional[set[int]] = None
    counts: Dict[int, int] = {}
    for cells in cell_sets:
        values = {int(components[tuple(cell)]) for cell in cells if int(components[tuple(cell)]) > 0}
        shared = values if shared is None else shared & values
        for cell in cells:
            value = int(components[tuple(cell)])
            if value > 0:
                counts[value] = counts.get(value, 0) + 1
    if not shared:
        raise ValueError("no common footprint-safe component for query intent")
    return max(shared, key=lambda value: (counts.get(value, 0), -value))


def _farthest_pair(
    first_cells: np.ndarray, second_cells: np.ndarray, components: np.ndarray,
    *, minimum_distance_cells: float = 30.0,
) -> Tuple[Tuple[int, int], Tuple[int, int], int]:
    component = _largest_common_component(first_cells, second_cells, components=components)
    first = first_cells[np.asarray([components[tuple(cell)] == component for cell in first_cells])]
    second = second_cells[np.asarray([components[tuple(cell)] == component for cell in second_cells])]
    if not len(first) or not len(second):
        raise ValueError("empty component-filtered query candidates")
    # Two deterministic farthest-point passes avoid an O(N^2) matrix.
    anchor = first[0]
    second_pick = second[int(np.argmax(np.sum((second - anchor) ** 2, axis=1)))]
    first_pick = first[int(np.argmax(np.sum((first - second_pick) ** 2, axis=1)))]
    if math.hypot(*(first_pick - second_pick)) < minimum_distance_cells:
        raise ValueError("query candidates are too close")
    return tuple(map(int, first_pick)), tuple(map(int, second_pick)), component


def _line_intersects(mask: np.ndarray, first: Tuple[int, int], second: Tuple[int, int]) -> bool:
    canvas = np.zeros(mask.shape, dtype=np.uint8)
    cv2.line(canvas, (first[1], first[0]), (second[1], second[0]), 1, 1, cv2.LINE_8)
    return bool(np.any(canvas.astype(bool) & mask))


def generate_query_set(
    hospital_map: Any, free_mask: np.ndarray, components: np.ndarray,
    raster: RasterizedSemantics, *, seed: int = 20260903,
) -> Tuple[List[Query], List[QueryIntent], Dict[str, Any]]:
    del seed  # Selection is deterministic and intentionally non-random.
    safe = np.asarray(free_mask, dtype=bool) & ~raster.hard_footprint_mask & ~raster.no_stopping_mask
    lane = safe & raster.masks.get("lane", np.zeros_like(safe))
    parking = safe & raster.masks.get("parking_area", np.zeros_like(safe))
    junction = raster.masks.get("junction_area", np.zeros_like(safe))
    labelled = np.zeros_like(safe)
    for value in raster.masks.values():
        labelled |= value
    unlabelled = safe & ~labelled
    lane_distance = ndimage.distance_transform_edt(
        raster.masks.get("lane", np.zeros_like(safe)), sampling=float(hospital_map.resolution),
    )
    narrow = lane & (2.0 * lane_distance < 1.10)
    lane_cells = _candidate_cells(lane, components)
    parking_cells = _candidate_cells(parking, components)
    unlabelled_cells = _candidate_cells(unlabelled, components)
    narrow_cells = _candidate_cells(narrow, components)
    if not len(lane_cells) or not len(parking_cells) or not len(unlabelled_cells):
        raise ValueError("real map lacks safe lane/parking/unlabelled candidates")

    definitions: List[Tuple[str, str, Tuple[int, int], Tuple[int, int], int, Dict[str, Any]]] = []
    lane_start, lane_goal, lane_component = _farthest_pair(lane_cells, lane_cells, components)
    definitions.append(("real-lane-forward", "lane_forward", lane_start, lane_goal, lane_component, {}))
    definitions.append(("real-lane-reverse", "lane_reverse", lane_goal, lane_start, lane_component, {}))

    # Prefer lane points close to, but not inside, junctions for the transition query.
    junction_distance = ndimage.distance_transform_edt(~junction, sampling=float(hospital_map.resolution))
    near_junction_cells = _candidate_cells(lane & (junction_distance <= 5.0), components)
    if len(near_junction_cells) >= 2:
        start, goal, component = _farthest_pair(
            near_junction_cells, near_junction_cells, components, minimum_distance_cells=20.0,
        )
    else:
        start, goal, component = lane_start, lane_goal, lane_component
    definitions.append(("real-lane-junction-lane", "lane_junction_lane", start, goal, component, {
        "direct_line_intersects_junction": _line_intersects(junction, start, goal),
    }))

    start, goal, component = _farthest_pair(lane_cells, parking_cells, components)
    definitions.append(("real-lane-to-parking", "lane_to_parking", start, goal, component, {}))
    start, goal, component = _farthest_pair(
        parking_cells, parking_cells, components, minimum_distance_cells=8.0,
    )
    definitions.append(("real-parking-internal", "parking_internal", start, goal, component, {}))

    hard_near = cv2.dilate(
        raster.hard_footprint_mask.astype(np.uint8),
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (121, 121)),
    ).astype(bool) & safe
    hard_near_cells = _candidate_cells(hard_near, components)
    try:
        start, goal, component = _farthest_pair(hard_near_cells, hard_near_cells, components)
        direct_intersection = _line_intersects(raster.hard_footprint_mask, start, goal)
    except ValueError:
        start, goal, component = _farthest_pair(unlabelled_cells, unlabelled_cells, components)
        direct_intersection = False
    definitions.append(("real-forbidden-detour", "forbidden_detour", start, goal, component, {
        "direct_line_intersects_forbidden_footprint": direct_intersection,
    }))

    start, goal, component = _farthest_pair(unlabelled_cells, unlabelled_cells, components)
    definitions.append(("real-unlabelled", "unlabelled", start, goal, component, {}))
    if len(narrow_cells) >= 2:
        start, goal, component = _farthest_pair(
            narrow_cells, narrow_cells, components, minimum_distance_cells=15.0,
        )
    else:
        start, goal, component = lane_start, lane_goal, lane_component
    definitions.append(("real-narrow-lane", "narrow_channel", start, goal, component, {
        "narrow_candidates_available": bool(len(narrow_cells) >= 2),
    }))

    queries: List[Query] = []
    intents: List[QueryIntent] = []
    for query_id, category, start_cell, goal_cell, component, verification in definitions:
        sx, sy = hospital_map.cell_to_world(start_cell)
        gx, gy = hospital_map.cell_to_world(goal_cell)
        yaw = math.atan2(gy - sy, gx - sx)
        query = Query(
            query_id=query_id,
            start=[sx, sy, yaw], goal=[gx, gy, yaw], category=category,
            seed=20260903, validation_status="VALIDATED_STATIC_FOOTPRINT_COMPONENT",
        )
        start_classes, goal_classes = _classes(raster, start_cell), _classes(raster, goal_cell)
        purpose = {
            "lane_forward": "lane" in start_classes and "lane" in goal_classes,
            "lane_reverse": "lane" in start_classes and "lane" in goal_classes,
            "lane_junction_lane": "lane" in start_classes and "lane" in goal_classes,
            "lane_to_parking": "lane" in start_classes and "parking_area" in goal_classes,
            "parking_internal": "parking_area" in start_classes and "parking_area" in goal_classes,
            "forbidden_detour": bool(verification.get("direct_line_intersects_forbidden_footprint")),
            "unlabelled": start_classes == ["unlabelled"] and goal_classes == ["unlabelled"],
            "narrow_channel": bool(verification.get("narrow_candidates_available")),
        }[category]
        queries.append(query)
        intents.append(QueryIntent(
            query_id=query_id, category=category,
            start_semantics=start_classes, goal_semantics=goal_classes,
            footprint_safe=bool(safe[start_cell] and safe[goal_cell]),
            connected_component=int(component), purpose_verified=bool(purpose),
            verification={
                **verification,
                "start_cell": list(start_cell), "goal_cell": list(goal_cell),
                "euclidean_distance_m": math.hypot(gx - sx, gy - sy),
                "start_no_stopping": bool(raster.no_stopping_mask[start_cell]),
                "goal_no_stopping": bool(raster.no_stopping_mask[goal_cell]),
                "start_hard_semantic_conflict": bool(raster.hard_footprint_mask[start_cell]),
                "goal_hard_semantic_conflict": bool(raster.hard_footprint_mask[goal_cell]),
            },
        ))
    metadata = {
        "schema_version": QUERY_SET_VERSION,
        "seed": 20260903,
        "map_hash": hospital_map.sha256,
        "semantic_map_hash": raster.semantic_map_hash,
        "query_hash": canonical_hash([
            {"query_id": query.query_id, "start": query.start, "goal": query.goal, "category": query.category}
            for query in queries
        ]),
        "all_endpoints_footprint_safe": all(item.footprint_safe for item in intents),
        "all_endpoints_connected": all(item.connected_component > 0 for item in intents),
        "purpose_verified_count": sum(item.purpose_verified for item in intents),
    }
    return queries, intents, metadata


def save_query_set(
    path: str | Path, queries: Sequence[Query], intents: Sequence[QueryIntent],
    metadata: Dict[str, Any], *, overwrite: bool = False,
) -> Path:
    target = Path(path)
    if target.exists() and not overwrite:
        raise FileExistsError(f"refusing to overwrite query set: {target}")
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        **metadata,
        "queries": [
            {
                "query_id": query.query_id, "start": query.start, "goal": query.goal,
                "category": query.category, "seed": query.seed,
                "validation_status": query.validation_status,
            }
            for query in queries
        ],
        "intent_validation": [asdict(item) for item in intents],
    }
    target.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return target


__all__ = ["QUERY_SET_VERSION", "QueryIntent", "generate_query_set", "save_query_set"]
