"""Optimistic configuration-space transition audit for the frozen positive query.

This is an offline diagnostic.  It unions all sampled yaw footprints before
connected-component analysis, so every reported connection is optimistic with
respect to continuous SE(2) motion.  It never changes the frozen contract.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .semantic_architecture_explorer import _rectangle_kernel
from .semantic_constraint_core import ConstraintWorld


def run(inputs: Path, query: str, position_scale: int, yaw_samples: int) -> dict:
    world = ConstraintWorld(inputs, query)
    scale = int(position_scale)
    resolution = world.map.resolution / scale
    valid = ((np.isin(world.grids["labels"], world.selected))
             & world.grids["allowed"] & ~world.grids["hard"]
             & (world.master < 253))
    valid = np.repeat(np.repeat(valid, scale, axis=0), scale, axis=1)
    obstacle = np.repeat(np.repeat(world.obstacle.astype(np.uint8), scale, axis=0), scale, axis=1)
    free = np.zeros_like(valid, dtype=bool)
    for index in range(int(yaw_samples)):
        yaw = 2.0 * math.pi * index / yaw_samples
        blocked = cv2.dilate(obstacle, _rectangle_kernel(resolution, yaw)) != 0
        free |= valid & ~blocked

    count, components, stats, _ = cv2.connectedComponentsWithStats(free.astype(np.uint8), 8)
    target0 = (valid & np.repeat(np.repeat(world.grids["correct"], scale, 0), scale, 1)
               & (np.repeat(np.repeat(world.grids["error"], scale, 0), scale, 1) <= .5))

    def subcell(pose):
        row, col = world.map.world_to_cell(*pose[:2])
        return int(row * scale + scale // 2), int(col * scale + scale // 2)

    start, goal = subcell(world.start), subcell(world.goal)
    start_component, goal_component = int(components[start]), int(components[goal])
    records = []
    for component in range(1, count):
        cells = components == component
        target = target0 & cells
        yy, xx = np.where(target)
        if len(yy) == 0:
            continue
        # Diameter is a geometric upper diagnostic for the target component;
        # it is not used as a feasibility proof for arbitrary paths.
        diameter = 0.0
        if len(yy) > 1:
            diameter = float(max(np.ptp(xx) * resolution, np.ptp(yy) * resolution))
        records.append({
            "component": component,
            "free_subcells": int(stats[component, cv2.CC_STAT_AREA]),
            "target_subcells": int(len(yy)),
            "target_fraction_of_component": float(len(yy) / stats[component, cv2.CC_STAT_AREA]),
            "target_bbox_subcells": [int(xx.min()), int(yy.min()), int(xx.max()), int(yy.max())],
            "target_bbox_span_m": [float(np.ptp(xx) * resolution), float(np.ptp(yy) * resolution)],
            "target_bbox_max_span_m": diameter,
        })

    start_record = next((r for r in records if r["component"] == start_component), None)
    result = {
        "query": query,
        "position_resolution_m": resolution,
        "position_scale": scale,
        "yaw_samples": int(yaw_samples),
        "yaw_step_deg": 360.0 / yaw_samples,
        "start_cell": list(start),
        "goal_cell": list(goal),
        "start_component": start_component,
        "goal_component": goal_component,
        "start_goal_same_component": start_component == goal_component,
        "component_count": count - 1,
        "target_components": records,
        "start_component_target": start_record,
        "optimistic_projection_scope": (
            "any-yaw union of exact rectangle-vs-map-cell footprint dilation; "
            "yaw continuity and curvature are omitted, so disconnection is a "
            "sampled projection result and not a continuous-space proof"
        ),
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--query", default="r3-mirror-1-positive")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--position-scale", type=int, default=5)
    parser.add_argument("--yaw-samples", type=int, default=360)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    result = run(args.inputs, args.query, args.position_scale, args.yaw_samples)
    (args.output / "transition_audit.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
