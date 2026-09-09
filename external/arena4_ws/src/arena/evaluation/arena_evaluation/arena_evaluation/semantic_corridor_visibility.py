"""Offline corridor-decomposition feasibility probe for PLN-02.

This is deliberately independent of the resource lattice, Hybrid A*, and
continuous waypoint optimizer.  It builds an optimistic SE(2) free-space
projection, extracts its medial-axis corridor, and tests curvature-constrained
Dubins connections through corridor portals.  Every resulting path is sent to
the unchanged ConstraintWorld audit.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import resource
import time
import heapq

import cv2
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from .semantic_constraint_core import ConstraintWorld, dubins_edge
from .semantic_constraint_study import fresh, seal, write_json


def _kernel(resolution, yaw):
    half = resolution / 2.0
    span = int(math.ceil(math.hypot(.265, .225) / resolution)) + 2
    z = np.arange(-span, span + 1, dtype=float) * resolution
    dy, dx = np.meshgrid(z, z, indexing="ij")
    c, s = math.cos(yaw), math.sin(yaw)
    return ((np.abs(dx) <= .265 * abs(c) + .225 * abs(s) + half) &
            (np.abs(dy) <= .265 * abs(s) + .225 * abs(c) + half) &
            (np.abs(c * dx + s * dy) <= .265 + half * (abs(c) + abs(s))) &
            (np.abs(-s * dx + c * dy) <= .225 + half * (abs(c) + abs(s)))).astype(np.uint8)


def _cell(world, pose, scale):
    row, col = world.map.world_to_cell(*pose[:2])
    return row * scale + scale // 2, col * scale + scale // 2


def _skeleton_graph(skeleton, clearance, resolution):
    coords = np.argwhere(skeleton)
    index = {tuple(p): i for i, p in enumerate(coords)}
    rows, cols, data = [], [], []
    for i, (r, c) in enumerate(coords):
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if not (dr or dc):
                    continue
                j = index.get((int(r + dr), int(c + dc)))
                if j is None:
                    continue
                step = math.hypot(dr, dc) * resolution
                # Prefer wide corridors while preserving connectivity.
                width = max(float(clearance[r, c]), float(clearance[coords[j, 0], coords[j, 1]]))
                cost = step * (1.0 + 0.08 / max(width, resolution))
                rows.append(i); cols.append(j); data.append(cost)
    return coords, csr_matrix((data, (rows, cols)), shape=(len(coords), len(coords)))


def _astar_free(free, src, dst, clearance, semantic=None):
    """Weighted portal search on the optimistic projection.

    This fallback keeps the corridor method useful when thinning removes a
    one-cell neck. It still produces only a geometric centerline; SE(2) and
    footprint feasibility are decided later by the exact audit.
    """
    h, w = free.shape
    sr, sc = map(int, src); gr, gc = map(int, dst)
    if not (free[sr, sc] and free[gr, gc]):
        return []
    parent = {}
    best = {(sr, sc): 0.0}
    heap = [(math.hypot(gr - sr, gc - sc), 0.0, sr, sc)]
    while heap:
        _, cost, r, c = heapq.heappop(heap)
        if cost != best.get((r, c)):
            continue
        if (r, c) == (gr, gc):
            break
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if not (dr or dc):
                    continue
                rr, cc = r + dr, c + dc
                if rr < 0 or rr >= h or cc < 0 or cc >= w or not free[rr, cc]:
                    continue
                step = math.hypot(dr, dc)
                # A small clearance term selects the center of the corridor,
                # while preserving every connected route in the graph.
                local = step * (1.0 + .08 / max(float(clearance[rr, cc]), .5))
                if semantic is not None:
                    # A target cell is cheaper, while wrong-side cells are
                    # expensive.  The grid search remains geometric and the
                    # final score always comes from the unchanged path audit.
                    local *= float(semantic[rr, cc])
                nc = cost + local
                if nc < best.get((rr, cc), float("inf")):
                    best[(rr, cc)] = nc
                    parent[(rr, cc)] = (r, c)
                    heapq.heappush(heap, (nc + math.hypot(gr - rr, gc - cc), nc, rr, cc))
    if (gr, gc) not in best:
        return []
    out = [(gr, gc)]
    while out[-1] != (sr, sc):
        out.append(parent[out[-1]])
    return list(reversed(out))


def run(args):
    output = fresh(args.output)
    world = ConstraintWorld(args.inputs, args.query)
    source = Path(__file__)
    write_json(output / "manifest.json", {
        "protocol_id": "PLN-02-CORRIDOR-VISIBILITY-R0-V1",
        "query_id": args.query,
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "input_json_sha256": hashlib.sha256((Path(args.inputs) / f"{args.query}.json").read_bytes()).hexdigest(),
        "input_npz_sha256": hashlib.sha256((Path(args.inputs) / f"{args.query}.npz").read_bytes()).hexdigest(),
        "map_sha256": world.meta.get("map_sha256") or hashlib.sha256(
            Path(world.meta["map"]["image_path"]).read_bytes()).hexdigest(),
        "scale": args.scale, "yaw_bins": args.yaw_bins,
        "architecture_id": "UNNAMED_UNTIL_TARGETED_OFFLINE_3_OF_3",
        "scope": "bounded_offline_corridor_probe_not_online_planner",
    })
    started, cpu = time.monotonic(), time.process_time()
    scale = int(args.scale)
    resolution = world.map.resolution / scale
    obstacle = np.repeat(np.repeat(world.obstacle.astype(np.uint8), scale, 0), scale, 1)
    legal = ((world.master < 253) & world.grids["allowed"] & ~world.grids["hard"] &
             np.isin(world.grids["labels"], world.selected))
    legal = np.repeat(np.repeat(legal, scale, 0), scale, 1)
    any_free = np.zeros_like(legal)
    # The projection is optimistic about yaw continuity, but every candidate
    # below is audited with its actual yaw and full rectangle footprint.
    for k in range(args.yaw_bins):
        yaw = 2.0 * math.pi * k / args.yaw_bins
        any_free |= legal & ~(cv2.dilate(obstacle, _kernel(resolution, yaw)) != 0)
    distance = cv2.distanceTransform(any_free.astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    skeleton = cv2.ximgproc.thinning((any_free * 255).astype(np.uint8), cv2.ximgproc.THINNING_ZHANGSUEN) > 0
    coords, graph = _skeleton_graph(skeleton, distance * resolution, resolution)
    src = np.asarray(_cell(world, world.start, scale))
    dst = np.asarray(_cell(world, world.goal, scale))
    if len(coords) == 0:
        raise RuntimeError("empty corridor skeleton")
    src_idx = int(np.argmin(np.sum((coords - src) ** 2, axis=1)))
    dst_idx = int(np.argmin(np.sum((coords - dst) ** 2, axis=1)))
    dist, pred = dijkstra(graph, indices=src_idx, return_predecessors=True)
    route = []
    cur = dst_idx
    while cur >= 0 and cur != src_idx:
        route.append(cur)
        cur = int(pred[cur])
    if cur == src_idx:
        route.append(src_idx)
    route = list(reversed(route)) if route and route[-1] == dst_idx else []
    if not route:
        # Medial-axis thinning can sever a narrow neck. Use a weighted
        # visibility corridor over the same optimistic configuration-space
        # projection; this is still independent of SE(2) lattice/Hybrid.
        free_route = _astar_free(any_free, src, dst, distance * resolution)
        route = [int(np.argmin(np.sum((coords - np.asarray(p)) ** 2, axis=1))) for p in free_route]
    # Reduce the pixel chain to portal candidates at several spacings.  Yaw is
    # tangent-derived except at exact frozen endpoints.
    candidates = []
    # Search several semantic corridor costs.  The baseline route is retained
    # for comparison; each weighted route is audited independently.
    semantic_maps = [("geometric", None)]
    labels = np.repeat(np.repeat(world.grids["labels"], scale, 0), scale, 1)
    error = np.repeat(np.repeat(world.grids["error"], scale, 0), scale, 1)
    correct = np.repeat(np.repeat(world.grids["correct"], scale, 0), scale, 1)
    target = correct & (error <= .50) & np.isin(labels, world.selected)
    sem_base = np.ones_like(distance, dtype=float)
    sem_base[target] = .18
    sem_base[correct & ~target] = 1.8
    sem_base[~correct] = 4.5
    # Lower target multiplier and progressively stronger penalty on wrong side.
    for factor in (0.5, 1.0, 2.0, 4.0):
        semantic_maps.append((f"target_weight_{factor:g}", np.where(target, .18, np.where(correct, 1.0, factor))))
    for route_name, semantic in semantic_maps:
        if semantic is not None:
            weighted = _astar_free(any_free, src, dst, distance * resolution, semantic=semantic)
            route = [int(np.argmin(np.sum((coords - np.asarray(p)) ** 2, axis=1))) for p in weighted]
        if route:
            xy = []
            for idx in route:
                r, c = coords[idx]
                x, y = world.map.cell_to_world((int(r // scale), int(c // scale)))
                xy.append((x, y))
            xy = np.asarray(xy, dtype=float)
            for stride in (max(1, int(.35 / resolution)), max(1, int(.55 / resolution)), max(1, int(.85 / resolution))):
                picks = list(range(0, len(xy), stride))
                if picks[-1] != len(xy) - 1:
                    picks.append(len(xy) - 1)
                poses = [world.start]
                for j in picks[1:-1]:
                    a = xy[max(0, j - stride)]; b = xy[min(len(xy) - 1, j + stride)]
                    poses.append((float(xy[j, 0]), float(xy[j, 1]), math.atan2(float(b[1] - a[1]), float(b[0] - a[0]))))
                poses.append(world.goal)
                edges = []
                valid = True
                for a, b in zip(poses, poses[1:]):
                    edge = dubins_edge(a, b, radius=.401)
                    if edge is None or not world.validate_edge(edge, dense=True):
                        valid = False; break
                    edges.append(edge)
                audit = {"gate_passed": False, "failure_code": "CORRIDOR_CONNECTION_REJECTED"}
                if valid:
                    audit, _ = world.audit(edges)
                candidates.append({"route_cost": route_name, "stride_cells": stride, "portal_count": len(poses), "valid_edges": valid, "audit": audit, "poses": poses})
                if audit.get("gate_passed"):
                    break
    best = None
    for row in candidates:
        a = row["audit"]
        if a.get("padded_effective_master_collision_free") and (best is None or (a.get("lane_target_band_ratio", 0), a.get("lane_correct_side_ratio", 0)) > (best["audit"].get("lane_target_band_ratio", 0), best["audit"].get("lane_correct_side_ratio", 0))):
            best = row
    result = {
        "protocol_id": "PLN-02-CORRIDOR-VISIBILITY-R0-V1",
        "candidate_architecture": "configuration_space_medial_axis_corridor_dubins_portals",
        "query_id": args.query, "gate_passed": bool(best and best["audit"].get("gate_passed")),
        "component_connected": bool(route), "skeleton_nodes": int(len(coords)),
        "route_nodes": int(len(route)), "corridor_scale": scale,
        "yaw_bins_projection": args.yaw_bins, "candidate_count": len(candidates),
        "best": best, "wall_ms": (time.monotonic() - started) * 1000.0,
        "cpu_ms": (time.process_time() - cpu) * 1000.0,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        "proof_scope": "optimistic_any_yaw_projection_and_finite_portal_strides; not continuous infeasibility proof",
    }
    write_json(output / "candidate_summary.json", [
        {"route_cost": row.get("route_cost"), "stride_cells": row.get("stride_cells"),
         "portal_count": row.get("portal_count"), "valid_edges": row.get("valid_edges"),
         "gate_passed": bool(row.get("audit", {}).get("gate_passed")),
         "collision_free": bool(row.get("audit", {}).get("padded_effective_master_collision_free", False)),
         "side": row.get("audit", {}).get("lane_correct_side_ratio"),
         "target_band": row.get("audit", {}).get("lane_target_band_ratio"),
         "p50_m": row.get("audit", {}).get("lane_target_error_p50_m"),
         "path_length_m": row.get("audit", {}).get("path_length_m"),
         "failure_code": row.get("audit", {}).get("failure_code", "")}
        for row in candidates
    ])
    overlay = np.full((*world.master.shape, 3), 255, dtype=np.uint8)
    overlay[world.master >= 253] = (35, 35, 35)
    overlay[target[::scale, ::scale]] = (150, 220, 150)
    if best:
        points = []
        for pose in best["poses"]:
            rr, cc = world.map.world_to_cell(*pose[:2])
            points.append((int(cc), int(rr)))
        if len(points) > 1:
            cv2.polylines(overlay, [np.asarray(points, dtype=np.int32)], False, (30, 70, 225), 2)
    for pose, color in ((world.start, (255, 60, 0)), (world.goal, (180, 0, 180))):
        rr, cc = world.map.world_to_cell(*pose[:2])
        cv2.circle(overlay, (int(cc), int(rr)), 4, color, -1)
    cv2.imwrite(str(output / "corridor_overlay.png"), overlay)
    write_json(output / "result.json", result)
    write_json(output / "corridor_certificate.json", {"skeleton_sha256": hashlib.sha256(skeleton.tobytes()).hexdigest(), "route": route, "source": [int(x) for x in src], "goal": [int(x) for x in dst]})
    np.savez_compressed(output / "corridor_projection.npz", any_free=any_free, skeleton=skeleton, distance=distance)
    seal(output)
    print(json.dumps(result, indent=2, sort_keys=True, default=str))
    return 0 if result["gate_passed"] else 2


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--inputs", type=Path, default=Path("/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/constraint_inputs_20260907T021418Z"))
    p.add_argument("--query", default="r3-mirror-1-positive")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--scale", type=int, choices=(1, 2, 5), default=2)
    p.add_argument("--yaw-bins", type=int, default=72)
    return run(p.parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
