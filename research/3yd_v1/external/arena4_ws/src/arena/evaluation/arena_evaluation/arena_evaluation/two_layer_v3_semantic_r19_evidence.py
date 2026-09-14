"""Read-only metric and geometry analysis of an existing r19 output.

Stored paths are used only for audit/plots; this CLI does not plan or promote.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from . import two_layer_v3_semantic_r17_parking_aisle as r17
from . import two_layer_v3_semantic_r19 as r19


def run(run_dir, output):
    run_dir, output = Path(run_dir).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    frozen = json.loads((run_dir / "protocol.json").read_text())
    for path, digest in frozen["source_hashes"].items():
        if r17.sha256_file(Path(path)) != digest:
            raise ValueError(f"SOURCE_HASH_MISMATCH: {path}")
    config = frozen["config"]
    parent_path = r17.PACKAGE_ROOT / "config" / config["parent_config"]["path"]
    c17, c15, alg, parent_alg, parent, *_ = r17._load(parent_path)
    prepared, _ = r17.r15._prepare_static(algorithm=alg, parent=parent, output=output)
    ctx, sm, raster, topology, _, router = prepared[:6]
    builder = r17.expanded.AuditOnlyPreferenceBuilderR14(
        ctx.hospital_map, raster, policy=parent["regional_preference"], semantic_map=sm)
    composer = r17.SemanticCostmapComposerR2(policy=parent["l3_soft_cost"], inflation_cache_capacity=2)
    query_path = r17.PACKAGE_ROOT / "config" / c15["frozen_bindings"]["expanded32_path"]
    queries, _, meta = r17.load_query_set(query_path,
        actual_map_hash=c17["frozen_bindings"]["map_hash"],
        actual_semantic_map_hash=c17["frozen_bindings"]["semantic_map_hash"])
    query = next(q for q in queries if q.query_id == config["sentinel_query"])
    _, _, _, _, full_allowed, md, arrays = r17.r15._prepare_query_state(
        query=query, query_hash=meta["query_hash"],
        selector=r17.r15._selector(parent, topology, router), ctx=ctx,
        semantic_map=sm, raster=raster, topology=topology,
        route_phase_algorithm=parent_alg, parent=parent, builder=builder, composer=composer)
    frozen_md = json.loads((run_dir / "input_metadata.json").read_text())
    for key in ("expected_master_hash", "route_hash", "roi_hash", "search_roi_hash"):
        if frozen_md[key] != md[key]:
            raise ValueError(f"INPUT_BINDING_MISMATCH: {key}")
    world = r17.CompactRoutePhaseWorldR14(output, query.query_id,
        arrays_override=arrays, meta_override=md,
        full_occupancy=ctx.hospital_map.occupancy, full_allowed=full_allowed)
    points = json.loads((run_dir / "path.json").read_text())
    samples, station = r17.resample_path(points)
    xy = np.array([[p[k] for k in ("x", "y", "yaw")] for p in samples])
    rows, cols, inside = world.cells(xy)
    if not inside.all():
        raise ValueError("PATH_OUTSIDE_BOUND_CROP")
    labels = world.grids["labels"][rows, cols]
    correct = world.grids["correct"][rows, cols]
    error = world.grids["error"][rows, cols]
    active = np.ones(len(xy), bool) if station[-1] <= 12 else ((station >= 6) & (station <= station[-1]-6))
    states = json.loads((run_dir / "states.json").read_text())
    lane_summary = []
    for instance in world.selected_lanes:
        selected = active & (labels == instance)
        layers = {}
        for s in states:
            if s["phase_kind"] == "lane" and s["phase_instance"] == instance:
                layers.setdefault(s["layer_index"], []).append(s)
        values = error[selected]
        poses = xy[selected]
        ray_distances = []
        for side in (1., -1.):
            distances = np.full(len(poses), np.nan)
            pending = np.ones(len(poses), bool)
            normals = np.column_stack((np.sin(poses[:, 2]), -np.cos(poses[:, 2]))) * side
            for step in np.arange(world.map.resolution, 30.0, world.map.resolution):
                if not pending.any():
                    break
                indices = np.flatnonzero(pending)
                probed = poses[indices].copy()
                probed[:, :2] += normals[indices]*step
                rr, cc, ok = world.cells(probed)
                outside = ~ok
                outside[ok] |= world.grids["labels"][rr[ok], cc[ok]] != instance
                hit = indices[outside]
                distances[hit] = step
                pending[hit] = False
            ray_distances.append(distances)
        right, left = ray_distances
        rays_valid = np.isfinite(right) & np.isfinite(left)
        lane_summary.append({
            "lane_instance": instance, "active_samples": int(selected.sum()),
            "correct_samples": int(np.count_nonzero(correct[selected])),
            "correct_side_ratio": float(np.mean(correct[selected])) if selected.any() else None,
            "error_p50_m": float(np.median(values)) if len(values) and np.isfinite(values).all() else None,
            "nonfinite_error_count": int(np.count_nonzero(~np.isfinite(values))),
            "sampled_state_layers": len(layers),
            "state_layers_with_target": sum(any(s["semantic_target"] for s in layer) for layer in layers.values()),
            "classification_from_planner_failure": False,
            "path_yaw_lane_boundary_probe": {
                "valid_sample_count": int(rays_valid.sum()),
                "resolution_m": world.map.resolution,
                "scope": "semantic_instance_boundary_not_obstacle_distance",
                "right_distance_p50_m": float(np.median(right[rays_valid])) if rays_valid.any() else None,
                "left_distance_p50_m": float(np.median(left[rays_valid])) if rays_valid.any() else None,
                "correct_side_ratio": float(np.mean(right[rays_valid] <= left[rays_valid])) if rays_valid.any() else None,
                "agreement_with_stored_field_ratio": float(np.mean((right[rays_valid] <= left[rays_valid]) == correct[selected][rays_valid])) if rays_valid.any() else None,
            },
        })
    result = {"artifact_type": "POST_RUN_AUDIT_NOT_PLANNING", "source_run": str(run_dir),
              "source_path_sha256": r17.sha256_file(run_dir / "path.json"),
              "input_binding_recomputed": True, "lanes": lane_summary,
              "active_interval_m": [float(station[active][0]), float(station[active][-1])],
              "aggregate_correct_side_ratio": float(np.mean(correct[active & np.isin(labels, world.selected_lanes)]))}
    r19.write_json(output / "lane_instance_audit.json", result)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    ref = np.asarray(json.loads((run_dir / "reference.json").read_text())["polyline"])
    route = np.asarray(md["route_polyline"])
    fig, axes = plt.subplots(1, 3, figsize=(16, 8), constrained_layout=True)
    parking = np.isin(world.grids["parking_components"][rows, cols], world.selected_parking)
    colors = np.where(~active, "#999999", np.where(parking, "#009988", np.where(correct, "#0077bb", "#cc3311")))
    targets = [None, xy[active & (labels == 4), :2], np.array([query.goal[:2]])]
    for ax, target, title in zip(axes, targets, ("Full request-generated path", "Lane instance 4: failing segment", "Exact goal attachment")):
        coords = xy[:, :2] if target is None or not len(target) else target
        pad = 2. if target is not None and len(target) == 1 else 1.
        bounds = [coords[:, 0].min()-pad, coords[:, 0].max()+pad, coords[:, 1].min()-pad, coords[:, 1].max()+pad]
        m = ctx.hospital_map
        c0 = max(0, int((bounds[0]-m.origin[0])/m.resolution))
        c1 = min(m.width, int((bounds[1]-m.origin[0])/m.resolution)+1)
        r0 = max(0, m.height-int((bounds[3]-m.origin[1])/m.resolution)-1)
        r1 = min(m.height, m.height-int((bounds[2]-m.origin[1])/m.resolution)+1)
        ax.imshow(m.occupancy[r0:r1, c0:c1] != 0, cmap="gray_r", origin="upper",
                  extent=(m.origin[0]+c0*m.resolution, m.origin[0]+c1*m.resolution,
                          m.origin[1]+(m.height-r1)*m.resolution, m.origin[1]+(m.height-r0)*m.resolution))
        ax.plot(route[:, 0], route[:, 1], color="#d89000", lw=1, alpha=.7)
        ax.plot(ref[:, 0], ref[:, 1], color="#8844aa", lw=.8, ls="--")
        ax.scatter(xy[::3, 0], xy[::3, 1], c=colors[::3], s=3, zorder=3)
        for pose, color in ((query.start, "#222222"), (query.goal, "#000000")):
            ax.scatter(pose[0], pose[1], c=color, marker="*", s=70, zorder=5)
            ax.arrow(pose[0], pose[1], .7*np.cos(pose[2]), .7*np.sin(pose[2]),
                     color=color, head_width=.15, length_includes_head=True, zorder=5)
        ax.set(xlim=bounds[:2], ylim=bounds[2:], title=title, xlabel="world x (m)", ylabel="world y (m)")
        ax.set_aspect("equal")
    fig.legend(handles=[Line2D([0],[0],color=c,label=l) for c,l in (
        ("#0077bb","lane correct side"),("#cc3311","lane wrong side"),
        ("#009988","parking"),("#999999","excluded endpoint window"),
        ("#8844aa","geometric reference"),("#d89000","frozen L1 route"))],
        loc="outside lower center" if tuple(map(int,matplotlib.__version__.split('.')[:2])) >= (3,7) else "lower center", ncol=3)
    fig.savefig(output / "path_lane_and_goal.png", dpi=160)
    plt.close(fig)
    r19.write_json(output / "manifest.json", {str(p.relative_to(output)):r17.sha256_file(p)
                     for p in sorted(output.rglob("*")) if p.is_file()})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.run_dir, args.output), indent=2))


if __name__ == "__main__":
    main()
