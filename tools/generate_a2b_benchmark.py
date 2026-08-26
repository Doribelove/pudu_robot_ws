#!/usr/bin/env python3
"""Generate a deterministic 20-pair A2B benchmark for each Arena map."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np


RESOLUTION = 0.05
# The task requirement is >0.60 m. Use a larger construction margin so the
# endpoint remains robust after footprint inflation and raster discretization.
ENDPOINT_CLEARANCE_M = 0.80
REQUESTED_CLEARANCE_M = 0.60
SNAP_RADIUS_M = 3.0

MAPS = {
    "arena_small_080x100_2d_005": (80.0, 100.0, "small"),
    "arena_small_100x100_2d_005": (100.0, 100.0, "small"),
    "arena_medium_160x200_2d_005": (160.0, 200.0, "medium"),
    "arena_medium_200x200_2d_005": (200.0, 200.0, "medium"),
    "arena_large_250x300_2d_005": (250.0, 300.0, "large"),
    "arena_large_300x500_2d_005": (300.0, 500.0, "large"),
    "arena_xlarge_500x500_2d_005": (500.0, 500.0, "xlarge"),
    "arena_xlarge_600x400_2d_005": (600.0, 400.0, "xlarge"),
    "mentor_map_20260825_005": (164.35, 75.60, "medium"),
    "mentor_map_20260825_005_4x_area": (328.70, 151.20, "medium"),
}

MENTOR_MAPS = {
    "mentor_map_20260825_005",
    "mentor_map_20260825_005_4x_area",
}

MIN_SEPARATION_RATIO = {
    "small": 0.18,
    "medium": 0.20,
    "large": 0.22,
    "xlarge": 0.25,
}

# Coordinates are normalized map coordinates: x / width and y / height.
# They intentionally cover the main corridors, plaza, halls, rooms, doors, and
# the warehouse block. The same normalized tasks are used at every scale.
TASKS = [
    ("A2B-01", "long_east_west", (0.08, 0.50), (0.92, 0.50), "center", "none", ["east_west_main_corridor", "central_plaza"]),
    ("A2B-02", "long_north_south", (0.50, 0.08), (0.50, 0.92), "center", "none", ["north_south_main_corridor", "central_plaza"]),
    ("A2B-03", "long_diagonal_sw_ne", (0.10, 0.12), (0.90, 0.88), "none", "none", ["wide_corridor", "central_plaza"]),
    ("A2B-04", "long_diagonal_nw_se", (0.10, 0.88), (0.90, 0.12), "none", "none", ["wide_corridor", "central_plaza"]),
    ("A2B-05", "room_to_room_diagonal", (0.16, 0.78), (0.84, 0.22), "edge", "left", ["office_teaching_room", "2m_door"]),
    ("A2B-06", "warehouse_to_room_diagonal", (0.84, 0.78), (0.16, 0.22), "edge", "right", ["warehouse_shelf_aisle", "office_teaching_room", "2m_door"]),
    ("A2B-07", "room_to_warehouse_diagonal", (0.16, 0.22), (0.84, 0.78), "edge", "left", ["office_teaching_room", "warehouse_shelf_aisle", "2m_door"]),
    ("A2B-08", "room_cross_diagonal", (0.84, 0.22), (0.16, 0.78), "edge", "right", ["office_teaching_room", "wide_hall", "2m_door"]),
    ("A2B-09", "west_corridor_to_plaza", (0.10, 0.50), (0.36, 0.50), "center", "none", ["east_west_main_corridor", "central_plaza"]),
    ("A2B-10", "east_corridor_to_plaza", (0.90, 0.50), (0.64, 0.50), "center", "none", ["east_west_main_corridor", "central_plaza"]),
    ("A2B-11", "south_corridor_to_plaza", (0.50, 0.10), (0.50, 0.36), "center", "none", ["north_south_main_corridor", "central_plaza"]),
    ("A2B-12", "north_corridor_to_plaza", (0.50, 0.90), (0.50, 0.64), "center", "none", ["north_south_main_corridor", "central_plaza"]),
    ("A2B-13", "upper_block_cross", (0.16, 0.78), (0.84, 0.78), "edge", "right", ["office_teaching_room", "warehouse_shelf_aisle", "wide_hall"]),
    ("A2B-14", "lower_block_cross", (0.16, 0.22), (0.84, 0.22), "edge", "left", ["office_teaching_room", "wide_hall", "2m_door"]),
    ("A2B-15", "southwest_to_northeast", (0.16, 0.24), (0.84, 0.76), "edge", "left", ["office_teaching_room", "warehouse_shelf_aisle", "2m_door"]),
    ("A2B-16", "southeast_to_northwest", (0.84, 0.24), (0.16, 0.76), "edge", "right", ["office_teaching_room", "warehouse_shelf_aisle", "2m_door"]),
    ("A2B-17", "plaza_east_west_local", (0.35, 0.50), (0.65, 0.50), "center", "none", ["central_plaza", "wide_corridor"]),
    ("A2B-18", "plaza_north_south_local", (0.50, 0.35), (0.50, 0.65), "center", "none", ["central_plaza", "wide_corridor"]),
    ("A2B-19", "northwest_to_southeast", (0.12, 0.76), (0.88, 0.24), "none", "none", ["office_teaching_room", "wide_corridor", "2m_door"]),
    ("A2B-20", "northeast_to_southwest", (0.88, 0.76), (0.12, 0.24), "none", "none", ["warehouse_shelf_aisle", "wide_corridor", "2m_door"]),
]

# The mentor map is a real warehouse raster rather than a generated Arena
# layout. These normalized anchors follow its connected main corridors and
# shelf aisles. The 4x-area map uses the same anchors because it is an exact
# 2x linear nearest-neighbor expansion of the source map.
MENTOR_TASKS = [
    ("A2B-01", "long_east_west_central", (0.074080, 0.532077), (0.948433, 0.525463), "center", "none", ["east_west_main_corridor", "central_transfer_zone"]),
    ("A2B-02", "long_north_south_central", (0.488135, 0.177579), (0.527685, 0.882606), "center", "none", ["north_south_crossing", "wide_cross_aisle"]),
    ("A2B-03", "long_diagonal_sw_ne", (0.090204, 0.128638), (0.929875, 0.900463), "none", "none", ["perimeter_corridor", "warehouse_shelf_aisle", "wide_cross_aisle"]),
    ("A2B-04", "long_diagonal_nw_se", (0.102981, 0.803902), (0.900974, 0.252315), "none", "none", ["perimeter_corridor", "warehouse_shelf_aisle", "wide_cross_aisle"]),
    ("A2B-05", "lower_east_west_cross", (0.119106, 0.252976), (0.877852, 0.245040), "edge", "left", ["lower_cross_aisle", "warehouse_shelf_aisle"]),
    ("A2B-06", "upper_east_west_cross", (0.122148, 0.735119), (0.879678, 0.773479), "edge", "right", ["upper_cross_aisle", "warehouse_shelf_aisle"]),
    ("A2B-07", "west_north_south_aisle", (0.192120, 0.142526), (0.190599, 0.825728), "edge", "left", ["vertical_aisle", "west_warehouse_block"]),
    ("A2B-08", "east_north_south_aisle", (0.798144, 0.232474), (0.796623, 0.866733), "edge", "right", ["vertical_aisle", "east_warehouse_block"]),
    ("A2B-09", "west_to_central_transfer", (0.071646, 0.490410), (0.432765, 0.549934), "center", "none", ["west_main_corridor", "central_transfer_zone"]),
    ("A2B-10", "east_to_central_transfer", (0.951780, 0.490410), (0.562975, 0.465939), "center", "none", ["east_main_corridor", "central_transfer_zone"]),
    ("A2B-11", "south_to_central_transfer", (0.467752, 0.166997), (0.483267, 0.438823), "center", "none", ["south_cross_aisle", "central_transfer_zone"]),
    ("A2B-12", "north_to_central_transfer", (0.527685, 0.882606), (0.519471, 0.562500), "center", "none", ["north_cross_aisle", "central_transfer_zone"]),
    ("A2B-13", "upper_block_cross", (0.212200, 0.706680), (0.775936, 0.732474), "edge", "right", ["upper_cross_aisle", "warehouse_shelf_aisle"]),
    ("A2B-14", "lower_block_cross", (0.220414, 0.284722), (0.785671, 0.315146), "edge", "left", ["lower_cross_aisle", "warehouse_shelf_aisle"]),
    ("A2B-15", "southwest_to_northeast", (0.140706, 0.168981), (0.820049, 0.824405), "edge", "left", ["vertical_aisle", "wide_cross_aisle", "warehouse_shelf_aisle"]),
    ("A2B-16", "southeast_to_northwest", (0.868117, 0.197421), (0.180256, 0.820437), "edge", "right", ["vertical_aisle", "wide_cross_aisle", "warehouse_shelf_aisle"]),
    ("A2B-17", "central_east_west_local", (0.313812, 0.516865), (0.676453, 0.501653), "center", "none", ["central_transfer_zone", "east_west_main_corridor"]),
    ("A2B-18", "central_north_south_local", (0.500000, 0.280093), (0.502130, 0.710648), "center", "none", ["central_transfer_zone", "north_south_crossing"]),
    ("A2B-19", "northwest_to_southeast_inner", (0.250228, 0.720569), (0.750989, 0.280754), "none", "none", ["warehouse_shelf_aisle", "wide_cross_aisle"]),
    ("A2B-20", "northeast_to_southwest_inner", (0.750989, 0.722553), (0.250228, 0.280093), "none", "none", ["warehouse_shelf_aisle", "wide_cross_aisle"]),
]


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def snap_endpoint(
    distance: np.ndarray,
    image: np.ndarray,
    x_m: float,
    y_m: float,
    component_mask: np.ndarray | None = None,
) -> tuple[float, float, float, float]:
    height, width = image.shape
    requested_x = x_m / RESOLUTION
    requested_row = (height - 1) - y_m / RESOLUTION
    x = round(requested_x)
    row = round(requested_row)
    radius = round(SNAP_RADIUS_M / RESOLUTION)
    row0, row1 = max(0, row - radius), min(height, row + radius + 1)
    col0, col1 = max(0, x - radius), min(width, x + radius + 1)
    yy, xx = np.ogrid[row0:row1, col0:col1]
    candidates = (
        (xx - x) ** 2 + (yy - row) ** 2 <= radius**2
    ) & (image[row0:row1, col0:col1] > 200)
    candidates &= distance[row0:row1, col0:col1] > ENDPOINT_CLEARANCE_M
    if component_mask is not None:
        candidates &= component_mask[row0:row1, col0:col1]
    rows, cols = np.where(candidates)
    if rows.size == 0:
        raise ValueError(
            f"could not find a free endpoint near ({x_m:.3f}, {y_m:.3f})"
        )
    index = np.argmin((cols + col0 - x) ** 2 + (rows + row0 - row) ** 2)
    selected_row = int(rows[index] + row0)
    selected_col = int(cols[index] + col0)
    actual_x = (selected_col + 0.5) * RESOLUTION
    actual_y = (height - selected_row - 0.5) * RESOLUTION
    snap_offset = math.hypot(actual_x - x_m, actual_y - y_m)
    return actual_x, actual_y, float(distance[selected_row, selected_col]), snap_offset


def build_map_tasks(world_dir: Path, name: str, width_m: float, height_m: float, band: str):
    image_path = world_dir / "map" / "map.pgm"
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(image_path)
    free = image > 200
    components, labels = cv2.connectedComponents(free.astype(np.uint8), 8)
    if components < 2:
        raise ValueError(f"map has no free component: {name}")
    distance = cv2.distanceTransform(free.astype(np.uint8), cv2.DIST_L2, 5) * RESOLUTION
    task_templates = MENTOR_TASKS if name in MENTOR_MAPS else TASKS
    component_mask = None
    if name in MENTOR_MAPS:
        component_sizes = np.bincount(labels.ravel())
        component_sizes[0] = 0
        component_mask = labels == int(np.argmax(component_sizes))
    min_separation = min(width_m, height_m) * MIN_SEPARATION_RATIO[band]
    tasks = []
    for task_id, label, start_norm, goal_norm, preference, side, features in task_templates:
        requested_start = [start_norm[0] * width_m, start_norm[1] * height_m]
        requested_goal = [goal_norm[0] * width_m, goal_norm[1] * height_m]
        start_x, start_y, start_clearance, start_snap = snap_endpoint(
            distance, image, *requested_start, component_mask
        )
        goal_x, goal_y, goal_clearance, goal_snap = snap_endpoint(
            distance, image, *requested_goal, component_mask
        )
        start_row = image.shape[0] - 1 - math.floor(start_y / RESOLUTION)
        start_col = math.floor(start_x / RESOLUTION)
        goal_row = image.shape[0] - 1 - math.floor(goal_y / RESOLUTION)
        goal_col = math.floor(goal_x / RESOLUTION)
        if labels[start_row, start_col] != labels[goal_row, goal_col]:
            raise ValueError(f"disconnected endpoint pair {name}/{task_id}")
        direct_distance = math.hypot(goal_x - start_x, goal_y - start_y)
        if direct_distance < min_separation:
            raise ValueError(
                f"short endpoint pair {name}/{task_id}: {direct_distance:.2f} < {min_separation:.2f} m"
            )
        heading = math.atan2(goal_y - start_y, goal_x - start_x)
        goal_offset = {
            "A2B-03": math.pi / 2,
            "A2B-04": -math.pi / 2,
            "A2B-05": math.pi / 2,
            "A2B-06": -math.pi / 2,
            "A2B-07": math.pi,
            "A2B-08": math.pi,
            "A2B-15": math.pi / 2,
            "A2B-16": -math.pi / 2,
            "A2B-19": math.pi / 2,
            "A2B-20": -math.pi / 2,
        }.get(task_id, 0.0)
        tasks.append(
            {
                "id": task_id,
                "label": label,
                "start": [round(start_x, 3), round(start_y, 3), round(heading, 4)],
                "goal": [round(goal_x, 3), round(goal_y, 3), round(wrap_angle(heading + goal_offset), 4)],
                "start_norm": [round(start_norm[0], 4), round(start_norm[1], 4)],
                "goal_norm": [round(goal_norm[0], 4), round(goal_norm[1], 4)],
                "preference": preference,
                "preference_side": side,
                "feature_tags": features,
                "start_clearance_m": round(start_clearance, 3),
                "goal_clearance_m": round(goal_clearance, 3),
                "required_clearance_m": REQUESTED_CLEARANCE_M,
                "direct_distance_m": round(direct_distance, 3),
                "minimum_direct_distance_m": round(min_separation, 3),
                "snap_offset_m": round(max(start_snap, goal_snap), 3),
                "free_component": int(labels[start_row, start_col]),
            }
        )
    return tasks


def write_outputs(source_root: Path, manifest_root: Path) -> None:
    manifest_root.mkdir(parents=True, exist_ok=True)
    manifest = {
        "schema": "arena_a2b_benchmark.v1",
        "description": f"Fixed 20 start-goal pairs per Arena map; {len(MAPS) * 20} deterministic tasks total.",
        "resolution_m": RESOLUTION,
        "endpoint_clearance_requirement_m": REQUESTED_CLEARANCE_M,
        "endpoint_validation_threshold_m": ENDPOINT_CLEARANCE_M,
        "separation_policy": MIN_SEPARATION_RATIO,
        "maps": {},
    }
    csv_rows = []
    for name, (width_m, height_m, band) in MAPS.items():
        world_dir = source_root / name
        tasks = build_map_tasks(world_dir, name, width_m, height_m, band)
        scenario = {
            "benchmark": "arena_a2b_benchmark_20",
            "world": name,
            "map_extent_m": [width_m, height_m],
            "resolution_m": RESOLUTION,
            "endpoint_clearance_requirement_m": REQUESTED_CLEARANCE_M,
            "minimum_direct_distance_m": round(min(width_m, height_m) * MIN_SEPARATION_RATIO[band], 3),
            "task_layout": "mentor_warehouse" if name in MENTOR_MAPS else "generated_arena",
            "robots": [
                {
                    "id": task["id"],
                    "start": task["start"],
                    "goal": task["goal"],
                    "preference": task["preference"],
                    "preference_side": task["preference_side"],
                }
                for task in tasks
            ],
            "tasks": tasks,
            "obstacles": {"static": [], "dynamic": [], "interactive": []},
        }
        scenario_path = world_dir / "scenarios" / "a2b_benchmark_20.json"
        scenario_path.parent.mkdir(parents=True, exist_ok=True)
        scenario_path.write_text(json.dumps(scenario, indent=2) + "\n")
        manifest["maps"][name] = {
            "band": band,
            "extent_m": [width_m, height_m],
            "scenario": str(scenario_path),
            "task_count": len(tasks),
            "task_layout": "mentor_warehouse" if name in MENTOR_MAPS else "generated_arena",
            "tasks": tasks,
        }
        for task in tasks:
            csv_rows.append(
                {
                    "world": name,
                    "band": band,
                    "task_id": task["id"],
                    "label": task["label"],
                    "start_x_m": task["start"][0],
                    "start_y_m": task["start"][1],
                    "start_yaw_rad": task["start"][2],
                    "goal_x_m": task["goal"][0],
                    "goal_y_m": task["goal"][1],
                    "goal_yaw_rad": task["goal"][2],
                    "preference": task["preference"],
                    "preference_side": task["preference_side"],
                    "start_clearance_m": task["start_clearance_m"],
                    "goal_clearance_m": task["goal_clearance_m"],
                    "direct_distance_m": task["direct_distance_m"],
                    "minimum_direct_distance_m": task["minimum_direct_distance_m"],
                    "feature_tags": ";".join(task["feature_tags"]),
                }
            )
    (manifest_root / "arena_a2b_benchmark_20.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    fields = list(csv_rows[0])
    with (manifest_root / "arena_a2b_benchmark_20.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(csv_rows)
    print(json.dumps({"maps": len(MAPS), "tasks": len(csv_rows), "output": str(manifest_root)}, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", required=True, type=Path)
    parser.add_argument("--manifest-root", required=True, type=Path)
    args = parser.parse_args()
    write_outputs(args.source_root, args.manifest_root)


if __name__ == "__main__":
    main()
