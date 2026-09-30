"""Footprint and endpoint-contract sensitivity for frozen mirror-positive.

This is an offline diagnostic. It never changes the frozen input or validates
an altered contract as a witness. The any-yaw projection is deliberately
optimistic: yaw continuity and curvature are omitted, so a merge is only a
necessary-condition signal while a split is strong sampled evidence.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .semantic_architecture_explorer import _rectangle_kernel
from .semantic_constraint_core import ConstraintWorld


def _scaled_kernel(resolution: float, yaw: float, scale: float) -> np.ndarray:
    """Exact rectangle-vs-cell kernel for a diagnostic footprint scale."""
    half_cell = resolution / 2.0
    hx, hy = 0.265 * scale, 0.225 * scale
    span = int(math.ceil(math.hypot(hx, hy) / resolution)) + 2
    coordinates = np.arange(-span, span + 1, dtype=float) * resolution
    dy, dx = np.meshgrid(coordinates, coordinates, indexing="ij")
    c, s = math.cos(yaw), math.sin(yaw)
    ac, ass = abs(c), abs(s)
    return ((np.abs(dx) <= hx * ac + hy * ass + half_cell)
            & (np.abs(dy) <= hx * ass + hy * ac + half_cell)
            & (np.abs(c * dx + s * dy) <= hx + half_cell * (ac + ass))
            & (np.abs(-s * dx + c * dy) <= hy + half_cell * (ac + ass))).astype(np.uint8)


def _subcell(world: ConstraintWorld, pose, scale: int):
    row, col = world.map.world_to_cell(*pose[:2])
    return int(row * scale + scale // 2), int(col * scale + scale // 2)


def run(inputs: Path, query: str, position_scale: int, yaw_samples: int,
        footprint_scales: list[float]) -> dict:
    world = ConstraintWorld(inputs, query)
    scale = int(position_scale)
    resolution = world.map.resolution / scale
    legal = ((np.isin(world.grids["labels"], world.selected))
             & world.grids["allowed"] & ~world.grids["hard"]
             & (world.master < 253))
    legal = np.repeat(np.repeat(legal, scale, axis=0), scale, axis=1)
    obstacle = np.repeat(np.repeat(world.obstacle.astype(np.uint8), scale, axis=0), scale, axis=1)
    correct = np.repeat(np.repeat(world.grids["correct"], scale, axis=0), scale, axis=1)
    error = np.repeat(np.repeat(world.grids["error"], scale, axis=0), scale, axis=1)
    target = legal & correct & (error <= 0.50)
    start = _subcell(world, world.start, scale)
    goal = _subcell(world, world.goal, scale)
    records = []
    for footprint_scale in footprint_scales:
        free = np.zeros_like(legal, dtype=bool)
        for index in range(int(yaw_samples)):
            yaw = 2.0 * math.pi * index / yaw_samples
            blocked = cv2.dilate(obstacle, _scaled_kernel(resolution, yaw, footprint_scale)) != 0
            free |= legal & ~blocked
        count, components, stats, _ = cv2.connectedComponentsWithStats(free.astype(np.uint8), 8)
        start_component = int(components[start])
        goal_component = int(components[goal])
        target_labels, target_counts = np.unique(components[target], return_counts=True)
        target_distribution = {int(k): int(v) for k, v in zip(target_labels, target_counts) if int(k) != 0}
        target_components = sorted(target_distribution, key=target_distribution.get, reverse=True)
        main_target = int(target_components[0]) if target_components else 0
        records.append({
            "footprint_scale": float(footprint_scale),
            "half_length_m": float(0.265 * footprint_scale),
            "half_width_m": float(0.225 * footprint_scale),
            "component_count": int(count - 1),
            "start_component": start_component,
            "goal_component": goal_component,
            "start_goal_same_component": bool(start_component == goal_component),
            "main_target_component": main_target,
            "main_target_subcells": int(target_distribution.get(main_target, 0)),
            "target_in_start_component_subcells": int(target_distribution.get(start_component, 0)),
            "target_distribution": target_distribution,
        })

    # Endpoint transition distance is computed once on the baseline footprint:
    # the Euclidean distance from each frozen endpoint cell to the main target
    # component in the optimistic projection. It is not an endpoint change.
    baseline = min(records, key=lambda r: abs(r["footprint_scale"] - 1.0))
    baseline_scale = baseline["footprint_scale"]
    free = np.zeros_like(legal, dtype=bool)
    for index in range(int(yaw_samples)):
        yaw = 2.0 * math.pi * index / yaw_samples
        blocked = cv2.dilate(obstacle, _scaled_kernel(resolution, yaw, baseline_scale)) != 0
        free |= legal & ~blocked
    count, components, stats, _ = cv2.connectedComponentsWithStats(free.astype(np.uint8), 8)
    target_labels, target_counts = np.unique(components[target], return_counts=True)
    target_distribution = {int(k): int(v) for k, v in zip(target_labels, target_counts) if int(k) != 0}
    main_target = max(target_distribution, key=target_distribution.get)
    yy, xx = np.where(components == main_target)
    # Map subcells back to world coordinates using the cropped map transform.
    nearest = {}
    for name, cell in (("start", start), ("goal", goal)):
        d2 = (yy - cell[0]).astype(np.float64) ** 2 + (xx - cell[1]).astype(np.float64) ** 2
        idx = int(np.argmin(d2))
        row, col = int(yy[idx]), int(xx[idx])
        nearest[name] = {
            "distance_m": float(math.sqrt(float(d2[idx])) * resolution),
            "subcell": [row, col],
            "world_xy": [float(world.map.full_origin[0] + ((col / scale) + world.map.col0 + 0.5) * world.map.resolution),
                         float(world.map.full_origin[1] + (world.map.full_height - (row / scale) - world.map.row0 - 0.5) * world.map.resolution)],
        }
    return {
        "query": query,
        "position_scale": scale,
        "position_resolution_m": resolution,
        "yaw_samples": int(yaw_samples),
        "yaw_step_deg": 360.0 / yaw_samples,
        "frozen_footprint": {"half_length_m": 0.265, "half_width_m": 0.225},
        "frozen_start": list(world.start),
        "frozen_goal": list(world.goal),
        "records": records,
        "baseline_endpoint_to_main_target": nearest,
        "scope": "any-yaw union projection; altered footprint rows are diagnostic contract sensitivity only; no row is a frozen-contract witness",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--query", default="r3-mirror-1-positive")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--position-scale", type=int, default=2)
    parser.add_argument("--yaw-samples", type=int, default=72)
    parser.add_argument("--footprint-scales", type=float, nargs="+",
                        default=[0.50, 0.60, 0.70, 0.80, 0.90, 1.00])
    args = parser.parse_args()
    args.output.mkdir(parents=False, exist_ok=False)
    result = run(args.inputs, args.query, args.position_scale, args.yaw_samples, args.footprint_scales)
    (args.output / "contract_sensitivity.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    with (args.output / "footprint_sensitivity.csv").open("w", newline="", encoding="utf-8") as stream:
        rows = result["records"]
        fields = ["footprint_scale", "half_length_m", "half_width_m", "component_count", "start_component", "goal_component", "start_goal_same_component", "main_target_component", "main_target_subcells", "target_in_start_component_subcells"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: row[field] for field in fields} for row in rows)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
