"""Goal-plane topology audit for the frozen semantic-transition contract.

This diagnostic asks a necessary question before any expensive trajectory
search: does the optimistic, full-footprint start component contain target-band
positions on or before the request's terminal plane?  It never changes the
semantic field or turns the sampled projection into a continuous-space proof.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import resource
import shutil
import time

import cv2
import numpy as np

from .semantic_architecture_explorer import _rectangle_kernel
from .semantic_constraint_core import ConstraintWorld
from .semantic_constraint_study import fresh, seal, write_json
from .semantic_map import canonical_hash, sha256_file


PROTOCOL_ID = "PLN-02-SEMANTIC-GOAL-PLANE-TOPOLOGY-R0-V1"


def _oriented_terminal_tangent(route, start, goal, *, footprint_length_m=0.53):
    points = np.asarray(route, dtype=float)[:, :2]
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
        raise ValueError("route_polyline must contain at least two finite points")
    if not np.all(np.isfinite(points)):
        raise ValueError("route_polyline must be finite")
    keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1.0e-9]
    points = points[keep]
    if len(points) < 2:
        raise ValueError("route_polyline has no translation")
    start = np.asarray(start, dtype=float)[:2]
    goal = np.asarray(goal, dtype=float)[:2]
    normal = np.linalg.norm(points[0] - start) + np.linalg.norm(points[-1] - goal)
    reversed_sum = np.linalg.norm(points[-1] - start) + np.linalg.norm(points[0] - goal)
    reversed_for_query = bool(reversed_sum < normal)
    if reversed_for_query:
        points = points[::-1].copy()
    # Endpoint attachment commonly contributes one diagonal map-cell step.
    # Estimate the terminal lane tangent over one full vehicle length instead
    # of allowing that final quantisation step to rotate the goal plane.
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    index = len(points) - 2
    accumulated = float(distances[index])
    while index > 0 and accumulated < float(footprint_length_m):
        index -= 1
        accumulated += float(distances[index])
    tangent = points[-1] - points[index]
    tangent /= np.linalg.norm(tangent)
    return points, tangent, {
        "route_reversed_for_query": reversed_for_query,
        "route_normal_endpoint_sum_m": float(normal),
        "route_reversed_endpoint_sum_m": float(reversed_sum),
        "terminal_tangent_window_m": float(accumulated),
        "terminal_tangent_window_source": "one_full_padded_footprint_length",
    }


def classify_target_progress(components, target, free, *, start_component, signed_goal_progress,
                             numeric_tolerance_m):
    """Classify target cells without imposing a tunable behavioural threshold."""
    components = np.asarray(components)
    target = np.asarray(target, dtype=bool)
    free = np.asarray(free, dtype=bool)
    progress = np.asarray(signed_goal_progress, dtype=float)
    if components.shape != target.shape or target.shape != progress.shape or target.shape != free.shape:
        raise ValueError("components, target, free and signed progress must have identical shapes")
    if not math.isfinite(float(numeric_tolerance_m)) or numeric_tolerance_m < 0.0:
        raise ValueError("numeric tolerance must be finite and non-negative")
    reachable = target & free & (components == int(start_component))
    before = reachable & (progress < -numeric_tolerance_m)
    on = reachable & (np.abs(progress) <= numeric_tolerance_m)
    after = reachable & (progress > numeric_tolerance_m)
    footprint_infeasible = target & ~free
    free_other_component = target & free & (components != int(start_component))
    reachable_count = int(np.count_nonzero(reachable))
    before_or_on = int(np.count_nonzero(before | on))
    status = (
        "NO_TARGET_IN_SAMPLED_OPTIMISTIC_START_COMPONENT" if reachable_count == 0 else
        "NO_PREGOAL_TARGET_IN_SAMPLED_OPTIMISTIC_START_COMPONENT" if before_or_on == 0 else
        "PREGOAL_TARGET_PRESENT_IN_SAMPLED_OPTIMISTIC_START_COMPONENT"
    )
    values = progress[reachable]
    return {
        "status": status,
        "sampled_projection_pre_goal_target_present": before_or_on > 0,
        "reachable_target_count": reachable_count,
        "reachable_target_before_goal_count": int(np.count_nonzero(before)),
        "reachable_target_on_goal_plane_count": int(np.count_nonzero(on)),
        "reachable_target_after_goal_count": int(np.count_nonzero(after)),
        "footprint_infeasible_target_count": int(np.count_nonzero(footprint_infeasible)),
        "free_other_component_target_count": int(np.count_nonzero(free_other_component)),
        "reachable_target_after_goal_ratio": (
            float(np.count_nonzero(after) / reachable_count) if reachable_count else None
        ),
        "reachable_target_progress_min_m": float(values.min()) if len(values) else None,
        "reachable_target_progress_max_m": float(values.max()) if len(values) else None,
        "numeric_tolerance_m": float(numeric_tolerance_m),
    }


def _subcell_progress(world, shape, scale, goal, tangent):
    rows = np.arange(shape[0], dtype=np.float64)
    cols = np.arange(shape[1], dtype=np.float64)
    x = world.map.full_origin[0] + (
        world.map.col0 + (cols + 0.5) / scale
    ) * world.map.resolution
    y = world.map.full_origin[1] + (
        world.map.full_height - world.map.row0 - (rows + 0.5) / scale
    ) * world.map.resolution
    # Broadcasting avoids materialising two full coordinate grids at once.
    return ((x[None, :] - goal[0]) * tangent[0]
            + (y[:, None] - goal[1]) * tangent[1])


def run(*, inputs: Path, query: str, output: Path, position_scale: int = 5,
        yaw_samples: int = 72):
    output = fresh(output)
    started = time.monotonic()
    world = ConstraintWorld(inputs, query)
    scale = int(position_scale)
    if scale <= 0 or int(yaw_samples) <= 0:
        raise ValueError("position_scale and yaw_samples must be positive")
    route, tangent, route_diagnostics = _oriented_terminal_tangent(
        world.meta.get("route_polyline", []), world.start, world.goal,
    )
    resolution = world.map.resolution / scale
    base_legal = (
        np.isin(world.grids["labels"], world.selected)
        & world.grids["allowed"] & ~world.grids["hard"] & (world.master < 253)
    )
    legal = np.repeat(np.repeat(base_legal, scale, 0), scale, 1)
    obstacle = np.repeat(np.repeat(world.obstacle.astype(np.uint8), scale, 0), scale, 1)
    free = np.zeros_like(legal, dtype=bool)
    for index in range(int(yaw_samples)):
        yaw = 2.0 * math.pi * index / int(yaw_samples)
        free |= legal & ~(cv2.dilate(obstacle, _rectangle_kernel(resolution, yaw)) != 0)
    component_count, components = cv2.connectedComponents(free.astype(np.uint8), 8)

    def subcell(pose):
        cell = world.map.world_to_cell(*pose[:2])
        if cell is None:
            raise ValueError("request endpoint lies outside the frozen crop")
        return (int(cell[0] * scale + scale // 2),
                int(cell[1] * scale + scale // 2))

    start_cell, goal_cell = subcell(world.start), subcell(world.goal)
    start_component = int(components[start_cell])
    goal_component = int(components[goal_cell])
    correct = np.repeat(np.repeat(world.grids["correct"], scale, 0), scale, 1)
    error = np.repeat(np.repeat(world.grids["error"], scale, 0), scale, 1)
    target = legal & correct & (error <= 0.50)
    progress = _subcell_progress(world, free.shape, scale, world.goal, tangent)
    classification = classify_target_progress(
        components, target, free, start_component=start_component,
        signed_goal_progress=progress, numeric_tolerance_m=resolution,
    )
    start_goal_same = bool(start_component and start_component == goal_component)
    sampled_projection_present = bool(
        start_goal_same and classification["sampled_projection_pre_goal_target_present"]
    )
    result = {
        "protocol_id": PROTOCOL_ID,
        "architecture_id": "UNNAMED_SEMANTIC_PLANNER_CANDIDATE",
        "query_id": query,
        "result_code": (
            "ENDPOINT_NOT_IN_OPTIMISTIC_FREE_COMPONENT" if not start_component or not goal_component else
            "START_GOAL_COMPONENT_MISMATCH" if not start_goal_same else
            classification["status"]
        ),
        "sampled_projection_pre_goal_target_present": sampled_projection_present,
        "start_goal_same_sampled_optimistic_component": start_goal_same,
        "start_component": start_component,
        "goal_component": goal_component,
        "component_count": int(component_count - 1),
        "start_cell": list(start_cell),
        "goal_cell": list(goal_cell),
        "classification": classification,
        "terminal_tangent": tangent.tolist(),
        "route_diagnostics": route_diagnostics,
        "route_hash": canonical_hash(route.tolist()),
        "position_resolution_m": resolution,
        "position_scale": scale,
        "yaw_samples": int(yaw_samples),
        "yaw_step_deg": 360.0 / int(yaw_samples),
        "projection_sha256": hashlib.sha256(np.ascontiguousarray(free).tobytes()).hexdigest(),
        "components_sha256": hashlib.sha256(np.ascontiguousarray(components).tobytes()).hexdigest(),
        "continuous_space_infeasibility_proof": False,
        "proof_scope": (
            "sampled-yaw union of full padded-footprint free positions; the reported "
            "absence is a necessary-condition failure in this optimistic finite projection, "
            "not a proof over every continuous yaw and trajectory"
        ),
        "wall_s": time.monotonic() - started,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
    }
    sources = [Path(__file__), Path(__file__).with_name("semantic_constraint_core.py"),
               Path(__file__).with_name("semantic_architecture_explorer.py")]
    manifest = {
        "protocol_id": PROTOCOL_ID,
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "python": platform.python_version(),
        "input_json_sha256": sha256_file(Path(inputs) / f"{query}.json"),
        "input_npz_sha256": sha256_file(Path(inputs) / f"{query}.npz"),
        "map_hash": world.meta["map_hash"],
        "semantic_map_hash": world.meta["semantic_map_hash"],
        "source_hashes": {str(path): sha256_file(path) for path in sources},
        "used_historical_path_or_witness": False,
        "online": False,
    }
    write_json(output / "manifest.json", manifest)
    write_json(output / "goal_plane_topology.json", result)
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    for path in sources:
        shutil.copy2(path, snapshot / path.name)
    seal(output)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--query", default="r3-mirror-1-positive")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--position-scale", type=int, default=5)
    parser.add_argument("--yaw-samples", type=int, default=72)
    args = parser.parse_args(argv)
    result = run(inputs=args.inputs, query=args.query, output=args.output,
                 position_scale=args.position_scale, yaw_samples=args.yaw_samples)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["sampled_projection_pre_goal_target_present"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
