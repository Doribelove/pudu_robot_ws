"""Deterministically freeze the 2A-V3 r14 32-query evaluation set.

Selection is input-only: four endpoint windows are derived from every member
of the authoritative selected8 set and accepted using map, semantic-class,
connected-component, footprint and topology-route checks.  The rejected v1
diagnostic mixed historical negative queries whose 48-bin E0 validity was not
part of their old contract.  This v2 rule has no access to E0 or E5 outcomes,
preventing post-hoc success selection while retaining 32 queries (within the
frozen 30--50 requirement) across all eight selected8 route categories.
"""
from __future__ import annotations

from dataclasses import asdict
import argparse
import math
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import yaml

from . import semantic_static_cache_v3 as static_cache
from . import two_layer_v2_semantic_r1_benchmark as r1
from . import two_layer_v3_semantic_r14_benchmark as r14
from .planner_benchmark.models import Query
from .regional_preference_r1 import orient_route_for_query
from .semantic_map import canonical_hash, sha256_file
from .semantic_query_defaults import load_query_set
from .semantic_query_set import QueryIntent, save_query_set


QUERY_SET_ID = "pudu_wanda_3f_2a_v3_r14_expanded32_v2"
SEED = 20260908
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SELECTED8 = PACKAGE_ROOT / "config/pudu_wanda_3f_selected8_gt50m_r2_v2.yaml"
DEFAULT_LEGACY8 = Path(
    "/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/"
    "real_ablation_r1_diag_v20_final8/real_query_set.yaml"
)
DEFAULT_OUTPUT = PACKAGE_ROOT / "config/pudu_wanda_3f_v3_r14_expanded32_v2.yaml"


def _classes(raster: Any, cell: tuple[int, int]) -> list[str]:
    return sorted(name for name, mask in raster.masks.items() if bool(mask[cell])) or ["unlabelled"]


def _semantic_mask(raster: Any, classes: Sequence[str]) -> np.ndarray:
    if list(classes) == ["unlabelled"]:
        labelled = np.zeros(raster.hard_mask.shape, dtype=bool)
        for mask in raster.masks.values():
            labelled |= np.asarray(mask, dtype=bool)
        return ~labelled
    preferred = "parking_area" if "parking_area" in classes else (
        "lane" if "lane" in classes else str(classes[0])
    )
    return np.asarray(raster.masks.get(preferred, np.zeros(raster.hard_mask.shape, bool)), dtype=bool)


def _snap(
    *, ctx: Any, raster: Any, components: np.ndarray, safe: np.ndarray,
    target: tuple[float, float], classes: Sequence[str], component: int,
    radius_m: float = 3.0,
) -> tuple[int, int]:
    cell = ctx.hospital_map.world_to_cell(*target)
    if cell is None:
        raise ValueError("expanded endpoint target lies outside map")
    radius = int(math.ceil(radius_m / ctx.hospital_map.resolution))
    r0, r1_ = max(0, cell[0]-radius), min(safe.shape[0], cell[0]+radius+1)
    c0, c1 = max(0, cell[1]-radius), min(safe.shape[1], cell[1]+radius+1)
    valid = (
        safe[r0:r1_, c0:c1]
        & (components[r0:r1_, c0:c1] == int(component))
        & _semantic_mask(raster, classes)[r0:r1_, c0:c1]
    )
    choices = np.argwhere(valid)
    if not len(choices):
        raise ValueError("no class-preserving safe endpoint near deterministic target")
    choices[:, 0] += r0
    choices[:, 1] += c0
    distance = (choices[:, 0]-cell[0])**2 + (choices[:, 1]-cell[1])**2
    order = np.lexsort((choices[:, 1], choices[:, 0], distance))
    selected = choices[int(order[0])]
    return int(selected[0]), int(selected[1])


def _interpolate(first: Sequence[float], second: Sequence[float], fraction: float) -> tuple[float, float]:
    return (
        float(first[0]) + float(fraction)*(float(second[0])-float(first[0])),
        float(first[1]) + float(fraction)*(float(second[1])-float(first[1])),
    )


def _route_pose(polyline: Sequence[Sequence[float]], fraction: float) -> tuple[float, float, float]:
    points = np.asarray(polyline, dtype=np.float64)
    lengths = np.hypot(*(np.diff(points[:, :2], axis=0).T))
    station = np.r_[0.0, np.cumsum(lengths)]
    target = float(np.clip(fraction, 0.0, 1.0))*float(station[-1])
    index = min(len(lengths)-1, int(np.searchsorted(station, target, side="right")-1))
    index = max(0, index)
    ratio = 0.0 if lengths[index] <= 1.0e-12 else (target-station[index])/lengths[index]
    point = points[index, :2] + ratio*(points[index+1, :2]-points[index, :2])
    delta = points[index+1, :2]-points[index, :2]
    return float(point[0]), float(point[1]), math.atan2(float(delta[1]), float(delta[0]))


def freeze(
    *, output: Path = DEFAULT_OUTPUT, selected8: Path = DEFAULT_SELECTED8,
    legacy8: Path = DEFAULT_LEGACY8, config_path: Path = r14.DEFAULT_CONFIG,
) -> Path:
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite frozen query set: {output}")
    config, _parent_algorithm, parent, _selected = r14._load_config(config_path)
    selected_queries, selected_intents, selected_meta = load_query_set(
        selected8, actual_map_hash=config["frozen_bindings"]["map_hash"],
        actual_semantic_map_hash=config["frozen_bindings"]["semantic_map_hash"],
        require_default_contract=True,
    )
    prepared = static_cache.prepare_static_cached(
        Path("/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/extracted"),
        Path("/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/conversion_v1/semantic_map_v1.json"),
        Path("/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/real_ablation_r1_diag_v20_final8/topology_cache"),
        parent, cache_root=Path(config["static_cache"]["root"]),
        maximum_disk_entries=int(config["static_cache"]["maximum_disk_entries"]),
    )
    ctx, _semantic_map, raster, topology, _annotator, router = prepared[:6]
    safe = (
        np.asarray(topology.free_mask, dtype=bool)
        & ~np.asarray(raster.hard_footprint_mask, dtype=bool)
        & ~np.asarray(raster.no_stopping_mask, dtype=bool)
    )
    components = np.asarray(topology.free_components)
    sources: list[tuple[Query, QueryIntent, list[tuple[float, float]]]] = []
    selected_windows = [(0.0, 1.0), (.04, .96), (.08, .92), (.12, .88)]
    for query, intent in zip(selected_queries, selected_intents):
        sources.append((query, intent, selected_windows))
    e0_switches = r1.ArmSwitches.parse(parent["ablation_arms"]["E0"])
    selector = r1._selector_for_arm(
        e0_switches, topology, router, {},
        preferred_attachment_radius_m=float(parent["endpoint_attachment"]["preferred_radius_m"]),
        attachment_cost_weight=float(parent["endpoint_attachment"]["cost_weight"]),
    )
    queries: list[Query] = []
    intents: list[QueryIntent] = []
    ordinal = 0
    for source_query, source_intent, windows in sources:
        source_start_cell = ctx.hospital_map.world_to_cell(*source_query.start[:2])
        source_goal_cell = ctx.hospital_map.world_to_cell(*source_query.goal[:2])
        if source_start_cell is None or source_goal_cell is None:
            raise ValueError(f"{source_query.query_id}: source endpoint outside map")
        component = int(components[source_start_cell])
        if component <= 0 or int(components[source_goal_cell]) != component:
            raise ValueError(f"{source_query.query_id}: source topology component mismatch")
        _source_sn, _source_gn, source_route, source_reason = selector(
            topology, source_query,
            cache_mode=r1.r2_runtime.CACHE_MODE_OPTIMIZED, timing={},
        )
        if source_route is None:
            raise ValueError(
                f"{source_query.query_id}: source neutral topology route failed: {source_reason}"
            )
        source_route, _source_orientation = orient_route_for_query(source_route, source_query)
        for lower, upper in windows:
            ordinal += 1
            if lower == 0.0 and upper == 1.0:
                start_cell = ctx.hospital_map.world_to_cell(*source_query.start[:2])
                goal_cell = ctx.hospital_map.world_to_cell(*source_query.goal[:2])
                if start_cell is None or goal_cell is None:
                    raise ValueError("frozen source endpoint outside map")
            else:
                start_target = _route_pose(source_route.polyline, lower)
                goal_target = _route_pose(source_route.polyline, upper)
                snap_radius = 3.0
                try:
                    start_cell = _snap(
                        ctx=ctx, raster=raster, components=components, safe=safe,
                        target=start_target[:2],
                        classes=source_intent.start_semantics, component=component,
                        radius_m=snap_radius,
                    )
                    goal_cell = _snap(
                        ctx=ctx, raster=raster, components=components, safe=safe,
                        target=goal_target[:2],
                        classes=source_intent.goal_semantics, component=component,
                        radius_m=snap_radius,
                    )
                except ValueError as error:
                    raise ValueError(
                        f"{source_query.query_id} window {lower:.2f}:{upper:.2f}: {error}"
                    ) from error
            if not (safe[start_cell] and safe[goal_cell]):
                raise ValueError("expanded endpoint failed frozen safety mask")
            start_xy = ctx.hospital_map.cell_to_world(start_cell)
            goal_xy = ctx.hospital_map.cell_to_world(goal_cell)
            if lower == 0.0 and upper == 1.0:
                start_yaw, goal_yaw = float(source_query.start[2]), float(source_query.goal[2])
            else:
                start_yaw, goal_yaw = float(start_target[2]), float(goal_target[2])
            query = Query(
                query_id=f"x32-{ordinal:02d}-{source_query.query_id}",
                start=[*start_xy, start_yaw],
                goal=[*goal_xy, goal_yaw],
                category=source_query.category, seed=SEED,
                validation_status="INPUT_ONLY_MAP_SEMANTIC_TOPOLOGY_VALIDATED",
            )
            _sn, _gn, route, reason = selector(
                topology, query, cache_mode=r1.r2_runtime.CACHE_MODE_OPTIMIZED, timing={},
            )
            if route is None:
                raise ValueError(f"{query.query_id}: neutral topology route failed: {reason}")
            route, orientation = orient_route_for_query(route, query)
            minimum_clearance = min(float(ctx.distance_m[start_cell]), float(ctx.distance_m[goal_cell]))
            queries.append(query)
            intents.append(QueryIntent(
                query_id=query.query_id, category=query.category,
                start_semantics=_classes(raster, start_cell),
                goal_semantics=_classes(raster, goal_cell),
                footprint_safe=True, connected_component=component, purpose_verified=True,
                verification={
                    "source_query_id": source_query.query_id,
                    "source_query_set": "selected8",
                    "interpolation_window": [float(lower), float(upper)],
                    "selection_used_planner_outcome": False,
                    "start_cell": list(start_cell), "goal_cell": list(goal_cell),
                    "minimum_endpoint_clearance_m": minimum_clearance,
                    "euclidean_distance_m": math.dist(start_xy, goal_xy),
                    "neutral_topology_route_length_m": float(route.length_m),
                    "route_reversed_for_query": bool(orientation.get("route_reversed_for_query", False)),
                },
            ))
    if len(queries) != 32:
        raise AssertionError(f"expanded query count drifted: {len(queries)}")
    metadata = {
        "schema_version": QUERY_SET_ID,
        "seed": SEED,
        "map_hash": config["frozen_bindings"]["map_hash"],
        "semantic_map_hash": config["frozen_bindings"]["semantic_map_hash"],
        "query_hash": canonical_hash([
            {"query_id": q.query_id, "start": q.start, "goal": q.goal, "category": q.category}
            for q in queries
        ]),
        "all_endpoints_footprint_safe": True,
        "all_endpoints_connected": True,
        "purpose_verified_count": len(intents),
        "selection_used_planner_outcome": False,
        "source_files": {
            "selected8": str(Path(selected8).resolve()),
            "selected8_sha256": sha256_file(Path(selected8).resolve()),
            "selected8_query_hash": selected_meta["query_hash"],
            "legacy8_excluded_from_v2": str(Path(legacy8).resolve()),
            "legacy8_exclusion_reason": (
                "historical negative-query validity was not frozen for the mandated 48-bin E0 arm"
            ),
        },
        "static_cache": dict(static_cache.LAST_CACHE_TELEMETRY),
    }
    save_query_set(output, queries, intents, metadata)
    return output


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--selected8", type=Path, default=DEFAULT_SELECTED8)
    parser.add_argument("--legacy8", type=Path, default=DEFAULT_LEGACY8)
    parser.add_argument("--config", type=Path, default=r14.DEFAULT_CONFIG)
    args = parser.parse_args(argv)
    target = freeze(
        output=args.output, selected8=args.selected8, legacy8=args.legacy8,
        config_path=args.config,
    )
    print(yaml.safe_dump({"query_set": str(target), "sha256": sha256_file(target)}, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
