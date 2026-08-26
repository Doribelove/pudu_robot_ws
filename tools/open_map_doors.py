#!/usr/bin/env python3
"""Open fixed-width doors between enclosed free-space components.

The input is a ROS trinary PGM. Components are connected through short,
occupied-only gaps, so unknown exterior pixels are never converted to free.
The default threshold ignores tiny furniture fragments while opening all
room-sized regions.
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial import cKDTree


FREE = 254
UNKNOWN = 205
OCCUPIED = 0


def boundary_points(labels: np.ndarray, component_ids: list[int], sample_step: int):
    points = []
    point_labels = []
    kernel = np.ones((3, 3), dtype=np.uint8)
    for component_id in component_ids:
        mask = (labels == component_id).astype(np.uint8)
        boundary = (mask == 1) & (cv2.erode(mask, kernel) != mask)
        rows, cols = np.where(boundary)
        selected = np.arange(0, len(rows), sample_step)
        points.append(np.c_[rows[selected], cols[selected]])
        point_labels.extend([component_id] * len(selected))
    if not points:
        return np.empty((0, 2), dtype=np.int32), np.empty((0,), dtype=np.int32)
    return np.vstack(points), np.asarray(point_labels, dtype=np.int32)


def segment_is_wall_only(image: np.ndarray, start: np.ndarray, end: np.ndarray):
    length = max(2, int(np.linalg.norm(end - start) * 2 + 2))
    rows = np.linspace(start[0], end[0], length).astype(np.int32)
    cols = np.linspace(start[1], end[1], length).astype(np.int32)
    values = image[rows, cols]
    return not np.any(values == UNKNOWN) and int(np.count_nonzero(values == OCCUPIED)) >= 2


def find_candidate_edges(image: np.ndarray, labels: np.ndarray, stats: np.ndarray, min_area: int, sample_step: int, max_gap_cells: float):
    component_ids = [i for i in range(1, len(stats)) if stats[i, cv2.CC_STAT_AREA] >= min_area]
    points, point_labels = boundary_points(labels, component_ids, sample_step)
    if len(points) == 0:
        return component_ids, {}
    tree = cKDTree(points)
    edges: dict[tuple[int, int], tuple[float, np.ndarray, np.ndarray]] = {}
    query_k = min(32, len(points))
    for component_id in component_ids:
        component_points = points[point_labels == component_id]
        distances, indices = tree.query(
            component_points,
            k=query_k,
            distance_upper_bound=max_gap_cells,
        )
        for row in range(len(component_points)):
            for distance, point_index in zip(np.atleast_1d(distances[row]), np.atleast_1d(indices[row])):
                if not np.isfinite(distance) or point_index >= len(point_labels):
                    continue
                other_id = int(point_labels[point_index])
                if other_id == component_id:
                    continue
                edge_key = tuple(sorted((component_id, other_id)))
                if edge_key in edges and edges[edge_key][0] <= distance:
                    break
                other_point = points[point_index]
                if segment_is_wall_only(image, component_points[row], other_point):
                    edges[edge_key] = (float(distance), component_points[row].copy(), other_point.copy())
    return component_ids, edges


def choose_spanning_edges(component_ids, edges):
    parent = {component_id: component_id for component_id in component_ids}

    def find(component_id):
        while parent[component_id] != component_id:
            parent[component_id] = parent[parent[component_id]]
            component_id = parent[component_id]
        return component_id

    selected = []
    for edge_key, edge in sorted(edges.items(), key=lambda item: item[1][0]):
        left, right = edge_key
        root_left, root_right = find(left), find(right)
        if root_left == root_right:
            continue
        parent[root_left] = root_right
        selected.append((edge_key, edge))
    return selected, len({find(component_id) for component_id in component_ids})


def apply_doors(image: np.ndarray, selected_edges, door_cells: int):
    output = image.copy()
    records = []
    for (left, right), (gap, start, end) in selected_edges:
        start = start.astype(np.float64)
        end = end.astype(np.float64)
        normal = end - start
        normal_length = max(float(np.linalg.norm(normal)), 1.0)
        normal /= normal_length
        tangent = np.asarray([-normal[1], normal[0]])
        midpoint = (start + end) / 2.0
        half_width = door_cells / 2.0
        cross_half_width = normal_length / 2.0 + 4.0
        corners_row_col = np.asarray(
            [
                midpoint - tangent * half_width - normal * cross_half_width,
                midpoint + tangent * half_width - normal * cross_half_width,
                midpoint + tangent * half_width + normal * cross_half_width,
                midpoint - tangent * half_width + normal * cross_half_width,
            ],
            dtype=np.int32,
        )
        # OpenCV points are (x, y), while all component calculations use
        # (row, column) = (y, x).
        polygon_xy = corners_row_col[:, ::-1].reshape(-1, 1, 2)
        door_mask = np.zeros_like(output, dtype=np.uint8)
        cv2.fillPoly(door_mask, [polygon_xy], 1)
        # A door can replace walls, but it must never turn exterior unknown
        # into free space.
        output[(door_mask > 0) & (image != UNKNOWN)] = FREE
        records.append(
            {
                "components": [int(left), int(right)],
                "wall_gap_cells": round(gap, 3),
                "start_row_col": [int(start[0]), int(start[1])],
                "end_row_col": [int(end[0]), int(end[1])],
                "door_width_cells": int(door_cells),
                "door_width_m": float(door_cells * 0.05),
            }
        )
    return output, records


def update_stats(map_dir: Path, image: np.ndarray):
    stats_path = map_dir / "map_stats.json"
    stats = json.loads(stats_path.read_text()) if stats_path.exists() else {}
    yaml_data = yaml.safe_load((map_dir / "map.yaml").read_text())
    resolution = float(yaml_data["resolution"])
    stats.update(
        {
            "width": int(image.shape[1]),
            "height": int(image.shape[0]),
            "resolution": resolution,
            "origin": [float(x) for x in yaml_data["origin"]],
            "extent_m": [round(image.shape[1] * resolution, 6), round(image.shape[0] * resolution, 6)],
            "canvas_area_m2": round(image.shape[1] * image.shape[0] * resolution * resolution, 3),
            "pixels": {str(value): int(np.count_nonzero(image == value)) for value in (FREE, OCCUPIED, UNKNOWN)},
        }
    )
    stats_path.write_text(json.dumps(stats, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", required=True, help="directory containing map.pgm and map.yaml")
    parser.add_argument("--min-component-area", type=int, default=200, help="minimum free component in pixels")
    parser.add_argument("--sample-step", type=int, default=4, help="boundary sampling stride in pixels")
    parser.add_argument("--max-gap-m", type=float, default=12.5, help="maximum wall gap considered for a door")
    parser.add_argument("--door-width-m", type=float, default=2.0)
    parser.add_argument("--in-place", action="store_true", help="backup and replace map.pgm")
    args = parser.parse_args()
    map_dir = Path(args.map_dir)
    yaml_data = yaml.safe_load((map_dir / "map.yaml").read_text())
    resolution = float(yaml_data["resolution"])
    if abs(resolution - 0.05) > 1e-9:
        raise SystemExit(f"expected 0.05 m/cell, got {resolution}")
    if args.door_width_m <= 0:
        raise SystemExit("--door-width-m must be positive")
    image = cv2.imread(str(map_dir / "map.pgm"), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise SystemExit(f"cannot read {map_dir / 'map.pgm'}")
    free = (image == FREE).astype(np.uint8)
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(free, 8)
    component_ids, edges = find_candidate_edges(
        image,
        labels,
        stats,
        args.min_component_area,
        max(1, args.sample_step),
        args.max_gap_m / resolution,
    )
    selected_edges, root_count = choose_spanning_edges(component_ids, edges)
    door_cells = max(1, round(args.door_width_m / resolution))
    output, records = apply_doors(image, selected_edges, door_cells)
    post_free = (output == FREE).astype(np.uint8)
    post_count, _, post_stats, _ = cv2.connectedComponentsWithStats(post_free, 8)
    significant_after = sorted(
        [int(post_stats[i, cv2.CC_STAT_AREA]) for i in range(1, post_count) if post_stats[i, cv2.CC_STAT_AREA] >= args.min_component_area],
        reverse=True,
    )
    if len(significant_after) > 1:
        raise RuntimeError(
            f"door graph did not connect all significant components: {len(significant_after)} remain"
        )
    output_path = map_dir / "map.pgm"
    if args.in_place:
        backup_path = map_dir / "map.before_doors.pgm"
        if not backup_path.exists():
            shutil.copy2(output_path, backup_path)
    if not cv2.imwrite(str(output_path if args.in_place else map_dir / "map.with_doors.pgm"), output):
        raise RuntimeError("failed to write output PGM")
    update_stats(map_dir, output)
    height, width = output.shape
    origin = [float(x) for x in yaml_data["origin"]]
    for record in records:
        for key in ("start_row_col", "end_row_col"):
            row, col = record[key]
            record[key.replace("_row_col", "_world_xy")] = [
                round(origin[0] + col * resolution, 6),
                round(origin[1] + (height - 1 - row) * resolution, 6),
            ]
    metadata = {
        "resolution_m": resolution,
        "door_width_m": args.door_width_m,
        "door_width_cells": door_cells,
        "minimum_component_area_cells": args.min_component_area,
        "components_before": int(component_count - 1),
        "significant_components_before": len(component_ids),
        "candidate_edges": len(edges),
        "doors_added": len(records),
        "component_roots_after_selected_doors": int(root_count),
        "components_after": int(post_count - 1),
        "largest_free_component_cells_after": significant_after[0] if significant_after else 0,
        "free_pixels_after": int(np.count_nonzero(output == FREE)),
        "doors": records,
    }
    (map_dir / "map_doors.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps({k: v for k, v in metadata.items() if k != "doors"}, indent=2))


if __name__ == "__main__":
    main()
