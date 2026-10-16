"""Freeze an independently applicable mirror pair for PLN-02 architecture 2A-V3.

The selector deliberately separates semantic *applicability* from semantic
path acceptance.  It may reuse the frozen r3 topology-node candidate pool, but
it ignores every historical eligibility, viability and path field.  For each
candidate direction it reconstructs the current L1 route, semantic field and
effective master from the bound map, then builds a finite ordered SE(2) graph.

A direction is applicable only when a raw target-band state lies on or before
the directed goal plane and belongs to both the start-reachable and
goal-coreachable sets of that graph.  The saved certificate is a replayable
forward-only Dubins chain through that state.  This is positive finite-graph
evidence, not a continuous-space completeness or infeasibility claim.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import gc
import hashlib
import json
import math
from pathlib import Path
import platform
import resource
import shutil
import subprocess
import time
from types import SimpleNamespace
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from . import two_layer_v2_semantic_r1_benchmark as r1
from . import two_layer_v2_semantic_r3_benchmark as r3
from .planner_benchmark.models import Query
from .regional_preference_r1 import expand_roi_to_route_lane_instances, orient_route_for_query
from .regional_preference_r3 import RegionalPreferenceBuilderR3
from .semantic_constraint_core import ConstraintWorld, dense_interpolate
from .semantic_costmap_r2 import SemanticCostmapComposerR2
from .semantic_map import canonical_hash, sha256_file
from .semantic_rasterizer import grid_hash
from .semantic_transition_ordered_corridor import (
    CorridorFailure,
    OrderedCorridorPolicy,
    OrientedRoute,
    audit_route_progress,
    build_graph,
)


ARCHITECTURE_ID = "2A-V3"
IMPLEMENTATION_REVISION = "r0-long-transition-directed-se2-corridor"
PROTOCOL_ID = "PLN-02-2A-V3-R0-LONG-TRANSITION-DIRECTED-SE2-CORRIDOR-V1"
ROOT = Path(__file__).resolve().parents[7]
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE_ROOT / "config/two_layer_v3_semantic_r0_long_mirror.yaml"
DEFAULT_EXTRACTED = ROOT / "private_data/pudu_wanda_3f/extracted"
DEFAULT_SEMANTIC_MAP = ROOT / "private_data/pudu_wanda_3f/results/conversion_v1/semantic_map_v1.json"
DEFAULT_TOPOLOGY = ROOT / "private_data/pudu_wanda_3f/results/real_ablation_r1_diag_v20_final8/topology_cache"
DEFAULT_LEGACY_TARGETS = PACKAGE_ROOT / "config/pudu_wanda_3f_r3_targeted_preflight3_v1.yaml"


class ApplicabilityFailure(RuntimeError):
    """Expected fail-closed selection outcome with a stable code."""

    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = str(code)
        self.detail = str(detail)


def _write_json(path: Path, payload: Any) -> None:
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)
        stream.write("\n")


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def _deep_merge(base: Mapping[str, Any], override: Mapping[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, Mapping) and isinstance(result.get(key), Mapping):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def _load_extended_mapping(
    path: Path, extension_key: str, *, seen: set[Path] | None = None,
) -> dict[str, Any]:
    path = Path(path).resolve()
    visited = set() if seen is None else set(seen)
    if path in visited:
        raise ValueError(f"configuration inheritance cycle at {path}")
    visited.add(path)
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    extension = payload.get(extension_key)
    if not extension:
        return payload
    base_path = Path(str(extension["path"]))
    if not base_path.is_absolute():
        base_path = path.parent / base_path
    base_path = base_path.resolve()
    if sha256_file(base_path) != str(extension["sha256"]):
        raise ValueError(f"extended mapping changed: {base_path}")
    base = _load_extended_mapping(base_path, extension_key, seen=visited)
    return _deep_merge(base, payload)


def _load_config(
    path: Path, *, expected_identity: Mapping[str, str] | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    path = Path(path).resolve()
    wrapper = _load_extended_mapping(path, "extends_config")
    identities = dict(expected_identity or {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
    })
    for key, expected in identities.items():
        if wrapper.get(key) != expected:
            raise ValueError(f"2A-V3 identity mismatch for {key}")
    parent = Path(str(wrapper["parent_config"]["path"]))
    if not parent.is_absolute():
        parent = path.parent / parent
    actual_parent_hash = sha256_file(parent)
    if actual_parent_hash != str(wrapper["parent_config"]["sha256"]):
        raise ValueError("frozen parent configuration changed")
    bindings = wrapper["frozen_bindings"]
    for item in ("source_candidate_pool", "legacy_positive_applicability_evidence"):
        value = bindings[item]
        pairs = [("path", "sha256")] if item == "source_candidate_pool" else [
            ("manifest_path", "manifest_sha256"), ("result_path", "result_sha256")
        ]
        for path_key, hash_key in pairs:
            bound = Path(str(value[path_key]))
            if not bound.is_absolute():
                bound = ROOT / bound
            if sha256_file(bound) != str(value[hash_key]):
                raise ValueError(f"frozen binding changed: {bound}")
    semantics = wrapper["semantic_applicability"]
    immutable = {
        "raw_semantic_target_unchanged": True,
        "require_same_semantic_feature_instance": True,
        "require_complete_padded_footprint": True,
        "require_static_start_reachable_and_goal_coreachable": True,
        "require_target_on_or_before_directed_goal_plane": True,
        "require_replayable_forward_only_se2_chain": True,
        "absent_intersection_status": "NOT_APPLICABLE",
        "absent_intersection_counts_as_success": False,
        "yaw_bins": 48,
        "motion_model": "DUBIN",
        "allow_reverse": False,
        "allow_in_place_rotation": False,
        "minimum_turning_radius_m": 0.40,
        "maximum_curvature_1pm": 2.50,
    }
    for key, expected in immutable.items():
        if semantics.get(key) != expected:
            raise ValueError(f"immutable applicability rule mismatch for {key}")
    with r3._r3_bindings():
        parent_config = r1._load_config(parent)
    return wrapper, parent_config


def _ordered_policy(config: Mapping[str, Any]) -> OrderedCorridorPolicy:
    raw = dict(config["ordered_se2_certificate"])
    raw["yaw_neighbor_bins"] = tuple(int(value) for value in raw["yaw_neighbor_bins"])
    fields = set(OrderedCorridorPolicy.__dataclass_fields__)
    return OrderedCorridorPolicy(**{key: value for key, value in raw.items() if key in fields})


def _map_cell_candidate_cells(
    world: ConstraintWorld, sample: Any, lane_label: int,
    policy: OrderedCorridorPolicy, *, station_slab_half_width_m: float,
) -> list[dict[str, Any]]:
    """Select actual map cells from a local route-station slab.

    The r0 line sampler could choose the smallest-error target cell even when
    that particular cell had insufficient footprint clearance, while omitting
    a nearby safer target cell in the same station bin.  V3 ranks raw target
    cells by static clearance first and keeps several of them.  No target value
    or boundary distance is changed.
    """
    center = world.map.world_to_cell(sample.x, sample.y)
    if center is None:
        return []
    resolution = float(world.map.resolution)
    radius = int(math.ceil(
        (float(policy.maximum_lateral_probe_m) + float(station_slab_half_width_m))
        / resolution
    )) + 1
    row0 = max(0, int(center[0]) - radius)
    row1 = min(world.map.height, int(center[0]) + radius + 1)
    col0 = max(0, int(center[1]) - radius)
    col1 = min(world.map.width, int(center[1]) + radius + 1)
    rows, cols = np.mgrid[row0:row1, col0:col1]
    x = world.map.full_origin[0] + (cols + world.map.col0 + 0.5) * resolution
    y = world.map.full_origin[1] + (
        world.map.full_height - rows - world.map.row0 - 0.5
    ) * resolution
    dx, dy = x - float(sample.x), y - float(sample.y)
    longitudinal = dx * float(sample.tangent_x) + dy * float(sample.tangent_y)
    lateral = dx * float(sample.tangent_y) - dy * float(sample.tangent_x)
    local = (
        (np.abs(longitudinal) <= float(station_slab_half_width_m) + 1.0e-12)
        & (np.abs(lateral) <= float(policy.maximum_lateral_probe_m) + 1.0e-12)
    )
    grid_slice = np.s_[row0:row1, col0:col1]
    labels = world.grids["labels"][grid_slice]
    allowed = world.grids["allowed"][grid_slice]
    hard = world.grids["hard"][grid_slice]
    master = world.master[grid_slice]
    errors = world.grids["error"][grid_slice]
    correct = world.grids["correct"][grid_slice].astype(bool)
    valid = (
        local & (labels == int(lane_label)) & allowed & ~hard & (master < 253)
        & np.isfinite(errors)
    )
    rr, cc = np.where(valid)
    values: list[dict[str, Any]] = []
    for local_row, local_col in zip(rr.tolist(), cc.tolist()):
        row, col = row0 + local_row, col0 + local_col
        error = float(errors[local_row, local_col])
        side = bool(correct[local_row, local_col])
        values.append({
            "x": float(x[local_row, local_col]),
            "y": float(y[local_row, local_col]),
            "row": int(row), "col": int(col),
            "offset": float(lateral[local_row, local_col]),
            "longitudinal_offset": float(longitudinal[local_row, local_col]),
            "error": error, "correct": side,
            "target": bool(side and error <= 0.50 + 1.0e-12),
            "clearance": float(world.map.distance_m[row, col]),
        })
    if not values:
        return []

    selected: list[dict[str, Any]] = []
    identities: set[tuple[int, int]] = set()

    def keep(item: dict[str, Any]) -> None:
        identity = (item["row"], item["col"])
        if identity not in identities and len(selected) < policy.maximum_cells_per_station:
            selected.append(item)
            identities.add(identity)

    target_values = sorted(
        (item for item in values if item["target"]),
        key=lambda item: (
            -item["clearance"], item["error"], abs(item["longitudinal_offset"]),
            abs(item["offset"]), item["row"], item["col"],
        ),
    )
    # Keep several independently footprint-checkable raw targets before adding
    # transition ladder states.  The later graph builder still performs the
    # exact yaw-specific padded-footprint check.
    for item in target_values[: min(3, policy.maximum_cells_per_station)]:
        keep(item)
    route_anchor = min(values, key=lambda item: (
        abs(item["offset"]), abs(item["longitudinal_offset"]),
        -item["clearance"], item["row"], item["col"],
    ))
    keep(route_anchor)
    semantic_anchor = target_values[0] if target_values else min(
        values,
        key=lambda item: (
            not item["correct"], item["error"], -item["clearance"],
            abs(item["offset"]), abs(item["longitudinal_offset"]),
            item["row"], item["col"],
        ),
    )
    for fraction in np.linspace(0.0, 1.0, policy.maximum_cells_per_station):
        desired = route_anchor["offset"] + float(fraction) * (
            semantic_anchor["offset"] - route_anchor["offset"]
        )
        keep(min(values, key=lambda item: (
            abs(item["offset"] - desired), abs(item["longitudinal_offset"]),
            not item["correct"], item["error"], -item["clearance"],
            item["row"], item["col"],
        )))
    for item in sorted(values, key=lambda item: (
        not item["correct"], item["error"], -item["clearance"],
        abs(item["offset"]), abs(item["longitudinal_offset"]),
        item["row"], item["col"],
    )):
        keep(item)
        if len(selected) >= policy.maximum_cells_per_station:
            break
    return selected


def _endpoint_yaws(polyline: Sequence[Sequence[float]]) -> tuple[float, float]:
    points = np.asarray(polyline, dtype=np.float64)[:, :2]
    if len(points) < 2:
        raise ApplicabilityFailure("ROUTE_BINDING_FAILED", "route has fewer than two points")
    start_index = next((index for index in range(1, len(points))
                        if not np.array_equal(points[index], points[0])), None)
    end_index = next((index for index in range(len(points) - 2, -1, -1)
                      if not np.array_equal(points[index], points[-1])), None)
    if start_index is None or end_index is None:
        raise ApplicabilityFailure("ROUTE_BINDING_FAILED", "route has no positive translation")
    return (
        math.atan2(points[start_index, 1] - points[0, 1],
                   points[start_index, 0] - points[0, 0]),
        math.atan2(points[-1, 1] - points[end_index, 1],
                   points[-1, 0] - points[end_index, 0]),
    )


def _undirected_route_hash(polyline: Sequence[Sequence[float]]) -> str:
    points = [[round(float(p[0]), 9), round(float(p[1]), 9)] for p in polyline]
    return canonical_hash(min(points, list(reversed(points))))


def _ranked_pool(config: Mapping[str, Any]) -> list[dict[str, Any]]:
    bound = config["frozen_bindings"]["source_candidate_pool"]
    path = Path(str(bound["path"]))
    if not path.is_absolute():
        path = ROOT / path
    payload = json.loads(path.read_text(encoding="utf-8"))
    policy = config["mirror_selection"]
    records = []
    for source in payload.get("candidates", []):
        directions = source.get("directions", [])
        if len(directions) != 2 or any("start" not in row or "goal" not in row for row in directions):
            continue
        first = directions[0]
        distance = math.dist(first["start"][:2], first["goal"][:2])
        clearance = float(source["minimum_endpoint_clearance_m"])
        if clearance < float(policy["minimum_endpoint_clearance_m"]):
            continue
        if not (float(policy["minimum_euclidean_separation_m"]) <= distance
                <= float(policy["maximum_euclidean_separation_m"])):
            continue
        records.append({
            "candidate_id": str(source["candidate_id"]),
            "lane_label": int(source["lane_label"]),
            "lane_semantic_id": str(source["lane_semantic_id"]),
            "endpoint_node_ids": [int(value) for value in source["endpoint_node_ids"]],
            "minimum_endpoint_clearance_m": clearance,
            "endpoint_euclidean_separation_m": float(distance),
            "directions": [
                {
                    "direction": str(row["direction"]),
                    "start_xy": [float(value) for value in row["start"][:2]],
                    "goal_xy": [float(value) for value in row["goal"][:2]],
                }
                for row in directions
            ],
        })
    return sorted(records, key=lambda row: (
        -row["minimum_endpoint_clearance_m"],
        -row["endpoint_euclidean_separation_m"],
        row["lane_semantic_id"],
        tuple(row["directions"][0]["start_xy"]),
        row["candidate_id"],
    ))


def _resolve_query_and_route(
    selector: Any, topology: Any, candidate_id: str, direction: Mapping[str, Any],
) -> tuple[Query, Any, dict[str, Any], str]:
    start_xy = direction["start_xy"]
    goal_xy = direction["goal_xy"]
    direct_yaw = math.atan2(goal_xy[1] - start_xy[1], goal_xy[0] - start_xy[0])
    probe = Query(
        query_id=f"{candidate_id}-{direction['direction']}",
        start=[*start_xy, direct_yaw], goal=[*goal_xy, direct_yaw],
        category="lane_mirror_applicability_probe", seed=20260907,
    )
    _sn, _gn, route, reason = selector(
        topology, probe, cache_mode=r1.r2_runtime.CACHE_MODE_OPTIMIZED, timing={},
    )
    if route is None:
        raise ApplicabilityFailure("L1_ROUTE_FAILED", str(reason))
    route, _ = orient_route_for_query(route, probe)
    start_yaw, goal_yaw = _endpoint_yaws(route.polyline)
    query = Query(
        query_id=probe.query_id,
        start=[*start_xy, start_yaw], goal=[*goal_xy, goal_yaw],
        category="lane_mirror_applicability_probe", seed=20260907,
    )
    _sn, _gn, final_route, reason = selector(
        topology, query, cache_mode=r1.r2_runtime.CACHE_MODE_OPTIMIZED, timing={},
    )
    if final_route is None:
        raise ApplicabilityFailure("L1_ROUTE_FAILED", str(reason))
    final_route, orientation = orient_route_for_query(final_route, query)
    return query, final_route, orientation, str(reason)


def _prepare_input(
    *, directory: Path, query: Query, route: Any, orientation: Mapping[str, Any],
    ctx: Any, semantic_map: Any, raster: Any, topology: Any,
    builder: RegionalPreferenceBuilderR3, composer: SemanticCostmapComposerR2,
    parent_config: Mapping[str, Any], targeted_hash: str,
    identity: Mapping[str, str] | None = None,
    artifact_sink: dict[str, Any] | None = None,
    write_artifacts: bool = True,
) -> dict[str, Any]:
    directory.mkdir(parents=True, exist_ok=False)
    roi_started = time.monotonic_ns()
    allowed = r1.r2_runtime._raw_corridor_mask(
        ctx, topology, route, query, float(parent_config["roi"]["r0_padding_m"]),
    )
    allowed, roi_diagnostics = expand_roi_to_route_lane_instances(
        ctx.hospital_map, raster, semantic_map, route.polyline, allowed,
        free_mask=r1.r2_runtime._raw_free_mask(ctx),
        route_probe_radius_m=float(parent_config["roi"].get("lane_route_probe_radius_m", 0.50)),
    )
    roi_ms = (time.monotonic_ns() - roi_started) / 1.0e6
    field_started = time.monotonic_ns()
    preference = builder.build(
        route.polyline, goal=query.goal, allowed_mask=allowed,
        relaxation_level="R0", planning_preference_enabled=True,
        route_diagnostics=orientation,
    )
    field_ms = (time.monotonic_ns() - field_started) / 1.0e6
    compose_started = time.monotonic_ns()
    composition = composer.compose(
        ctx.hospital_map.occupancy, raster, preference, allowed_mask=allowed,
        hard_semantics_enabled=True, soft_class_costs_enabled=True,
        regional_preference_enabled=True, hard_semantics_use_footprint=True,
    )
    compose_ms = (time.monotonic_ns() - compose_started) / 1.0e6
    selected = sorted({
        int(value) for value in preference.diagnostics.get("guide_lane_instance_ids", [])
        if int(value) > 0
    })
    if len(selected) != 1:
        raise ApplicabilityFailure(
            "LANE_INSTANCE_BINDING_FAILED",
            f"expected one selected lane instance, got {selected}",
        )
    archive = directory / f"{query.query_id}.npz"
    serialization_started = time.monotonic_ns()
    if write_artifacts:
        np.savez_compressed(
            archive,
            master=composition.expected_master_cost,
            occupancy=ctx.hospital_map.occupancy,
            allowed=allowed,
            labels=preference.lane_instance_id,
            error=preference.lane_error_m,
            correct=preference.lane_correct_side,
            right=preference.lane_distance_to_right_m,
            left=preference.lane_distance_to_left_m,
            hard=raster.hard_footprint_mask,
            no_stopping=raster.no_stopping_mask,
        )
    serialization_ms = (time.monotonic_ns() - serialization_started) / 1.0e6
    bound_identity = dict(identity or {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
    })
    meta = {
        **bound_identity,
        "query": query.as_dict(),
        "map_hash": ctx.map_sha256,
        "semantic_map_hash": semantic_map.semantic_map_hash,
        "targeted_query_content_hash": targeted_hash,
        "expected_master_hash": composition.expected_master_hash,
        "route_hash": canonical_hash(route.polyline),
        "undirected_route_hash": _undirected_route_hash(route.polyline),
        "roi_hash": grid_hash(allowed),
        "selected_lane_labels": selected,
        "guide_polylines_world": preference.diagnostics.get("guide_polylines_world", []),
        "route_polyline": route.polyline,
        "route_orientation": dict(orientation),
        "roi_diagnostics": roi_diagnostics,
        "preference_diagnostics": preference.diagnostics,
        "timing": {
            "roi_build_ms": roi_ms,
            "field_build_ms": field_ms,
            "compose_ms": compose_ms,
            "input_serialization_ms": serialization_ms,
        },
        "map": {
            "resolution": ctx.hospital_map.resolution,
            "origin": ctx.hospital_map.origin,
            "width": ctx.hospital_map.width,
            "height": ctx.hospital_map.height,
            "image_path": str(ctx.hospital_map.image_path),
        },
        "npz_sha256": sha256_file(archive) if write_artifacts else "",
    }
    if write_artifacts:
        _write_json(directory / f"{query.query_id}.json", meta)
    if artifact_sink is not None:
        artifact_sink.update({
            "allowed": allowed,
            "preference": preference,
            "composition": composition,
            "arrays": {
                "master": composition.expected_master_cost,
                "occupancy": ctx.hospital_map.occupancy,
                "allowed": allowed,
                "labels": preference.lane_instance_id,
                "error": preference.lane_error_m,
                "correct": preference.lane_correct_side,
                "right": preference.lane_distance_to_right_m,
                "left": preference.lane_distance_to_left_m,
                "hard": raster.hard_footprint_mask,
                "no_stopping": raster.no_stopping_mask,
            },
            "timing": dict(meta["timing"]),
        })
    return meta


def _target_chain(graph: Any) -> tuple[list[int], int, dict[str, Any]] | None:
    start = graph.layers[0][0]
    goal = graph.layers[-1][0]
    reachable = {start}
    parent: dict[int, int] = {}
    for layer in graph.layers[:-1]:
        for state_id in sorted(layer):
            if state_id not in reachable:
                continue
            for edge_id in graph.outgoing[state_id]:
                edge = graph.edges[edge_id]
                if edge.target not in reachable:
                    reachable.add(edge.target)
                    parent[edge.target] = edge_id
    incoming: dict[int, list[int]] = {state.state_id: [] for state in graph.states}
    for edge in graph.edges:
        incoming[edge.target].append(edge.edge_id)
    for edge_ids in incoming.values():
        edge_ids.sort(key=lambda index: (
            graph.edges[index].source_layer, graph.edges[index].source,
            graph.edges[index].length_m, index,
        ))
    coreachable = {goal}
    successor: dict[int, int] = {}
    for layer in reversed(graph.layers[1:]):
        for state_id in sorted(layer, reverse=True):
            if state_id not in coreachable:
                continue
            for edge_id in incoming[state_id]:
                edge = graph.edges[edge_id]
                if edge.source not in coreachable:
                    coreachable.add(edge.source)
                    successor[edge.source] = edge_id
    targets = [
        state for state in graph.states
        if state.target_band and state.state_id in reachable and state.state_id in coreachable
        and state.station_m <= graph.route.length_m + 1.0e-6
    ]
    if not targets:
        return None
    target = min(targets, key=lambda state: (
        abs(state.station_m - 0.5 * graph.route.length_m),
        state.error_m, abs(state.lateral_offset_m), state.yaw_bin, state.state_id,
    ))
    before = []
    cursor = target.state_id
    while cursor != start:
        edge_id = parent[cursor]
        before.append(edge_id)
        cursor = graph.edges[edge_id].source
    before.reverse()
    after = []
    cursor = target.state_id
    while cursor != goal:
        edge_id = successor[cursor]
        after.append(edge_id)
        cursor = graph.edges[edge_id].target
    return before + after, target.state_id, {
        "start_reachable_state_count": len(reachable),
        "goal_coreachable_state_count": len(coreachable),
        "reachable_coreachable_target_state_count": len(targets),
        "selected_target_state": asdict(target),
    }


def _chain_certificate(
    input_dir: Path, query_id: str, policy: OrderedCorridorPolicy,
    *, candidate_cell_provider=None,
) -> dict[str, Any]:
    world = ConstraintWorld(input_dir, query_id)
    route = OrientedRoute(
        world.meta["route_polyline"], world.start, world.goal,
        endpoint_attachment_limit_m=policy.endpoint_attachment_limit_m,
    )
    try:
        graph = build_graph(
            world, route, policy,
            candidate_cell_provider=candidate_cell_provider,
        )
    except CorridorFailure as error:
        return {
            "applicable": False,
            "status": "NOT_APPLICABLE",
            "failure_code": error.code,
            "failure_detail": error.detail,
            "continuous_space_infeasibility_proof": False,
            "finite_graph_positive_certificate": False,
        }
    chain = _target_chain(graph)
    if chain is None:
        return {
            "applicable": False,
            "status": "NOT_APPLICABLE",
            "failure_code": "NO_REPLAYABLE_PREGOAL_TARGET_CHAIN",
            "continuous_space_infeasibility_proof": False,
            "graph": graph.diagnostics,
        }
    edge_ids, target_state_id, reachability = chain
    controls = [graph.edges[index].control for index in edge_ids]
    safety, path = world.audit(controls)
    dense = dense_interpolate(path)
    progress = audit_route_progress(route, dense)
    target_state = graph.states[target_state_id]
    hard = {
        "canonical_final_valid_success": bool(safety["canonical"]["final_valid_success"]),
        "padded_effective_master_collision_free": bool(safety["padded_effective_master_collision_free"]),
        "exact_endpoint_xy_yaw": bool(safety["exact_endpoint_xy_yaw"]),
        "edge_continuity": bool(safety["edge_continuity"]),
        "trace_replay_exact": bool(safety["trace_replay_exact"]),
        "same_lane_instance": bool(safety["same_lane_instance"]),
        "no_stopping_goal_violation": bool(safety["no_stopping_goal_violation"]),
        "maximum_control_curvature_1pm": float(safety["maximum_control_curvature_1pm"]),
        "path_length_bound_passed": bool(safety["arc_length_m"] <= world.bound_length),
    }
    hard_gate = bool(
        all(hard[key] for key in (
            "canonical_final_valid_success", "padded_effective_master_collision_free",
            "exact_endpoint_xy_yaw", "edge_continuity", "trace_replay_exact",
            "same_lane_instance", "path_length_bound_passed",
        ))
        and not hard["no_stopping_goal_violation"]
        and hard["maximum_control_curvature_1pm"] <= 2.50
    )
    applicable = bool(
        hard_gate and progress["ordered_progress_gate_passed"]
        and target_state.target_band
        and target_state.station_m <= route.length_m + policy.projection_epsilon_m
    )
    return {
        "applicable": applicable,
        "status": "APPLICABLE" if applicable else "NOT_APPLICABLE",
        "failure_code": "" if applicable else "CHAIN_HARD_OR_ORDERED_AUDIT_FAILED",
        "continuous_space_infeasibility_proof": False,
        "finite_graph_positive_certificate": applicable,
        "graph": graph.diagnostics,
        "reachability": reachability,
        "target_state_id": int(target_state_id),
        "target_state_before_or_on_goal_plane": bool(target_state.station_m <= route.length_m + 1.0e-6),
        "hard_safety": hard,
        "hard_safety_gate_passed": hard_gate,
        "ordered_progress": progress,
        "path": [[float(value) for value in pose] for pose in path],
        "path_sha256": hashlib.sha256(np.ascontiguousarray(path).tobytes()).hexdigest(),
        "controls": [control.certificate() for control in controls],
        "edge_ids": edge_ids,
        "semantic_diagnostic_only": {
            "correct_side_ratio": safety.get("lane_correct_side_ratio"),
            "target_band_ratio": safety.get("lane_target_band_ratio"),
            "lateral_error_p50_m": safety.get("lane_target_error_p50_m"),
            "r2_gate_from_chain": bool(safety.get("gate_passed")),
        },
    }


def _classify_legacy_positive(config: Mapping[str, Any]) -> dict[str, Any]:
    evidence = config["frozen_bindings"]["legacy_positive_applicability_evidence"]
    path = Path(str(evidence["result_path"]))
    if not path.is_absolute():
        path = ROOT / path
    result = json.loads(path.read_text(encoding="utf-8"))
    classification = result["classification"]
    absent = bool(
        result.get("sampled_projection_pre_goal_target_present") is False
        and int(classification.get("reachable_target_before_goal_count", -1)) == 0
        and int(classification.get("reachable_target_on_goal_plane_count", -1)) == 0
    )
    return {
        "query_id": "r3-mirror-1-positive",
        "status": "NOT_APPLICABLE" if absent else "REQUIRES_REVIEW",
        "counts_as_success": False,
        "retained_as_negative_regression": True,
        "reason": "NO_PREGOAL_TARGET_IN_FROZEN_SAMPLED_PROJECTION" if absent else "PREGOAL_TARGET_PRESENT",
        "sampled_projection_only": True,
        "continuous_space_infeasibility_proof": False,
        "source_result_sha256": sha256_file(path),
        "reachable_target_before_goal_count": classification.get("reachable_target_before_goal_count"),
        "reachable_target_on_goal_plane_count": classification.get("reachable_target_on_goal_plane_count"),
        "reachable_target_after_goal_count": classification.get("reachable_target_after_goal_count"),
        "nearest_reachable_target_progress_m": classification.get("reachable_target_progress_min_m"),
    }


def _manifest_files(output: Path) -> dict[str, str]:
    return {
        str(path.relative_to(output)): sha256_file(path)
        for path in sorted(output.rglob("*"))
        if path.is_file() and path.name != "artifact_hashes.json"
    }


def _process_audit() -> str:
    result = subprocess.run(
        ["ps", "-eo", "pid,ppid,lstart,args"], check=False,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    return "\n".join(line for line in result.stdout.splitlines() if any(
        token in line.lower() for token in ("ros2", "nav2", "smac", "semantic_applicability_v3")
    )) + "\n"


def run(
    *, output: Path, config_path: Path = DEFAULT_CONFIG,
    extracted: Path = DEFAULT_EXTRACTED, semantic_map_path: Path = DEFAULT_SEMANTIC_MAP,
    topology_cache: Path = DEFAULT_TOPOLOGY,
) -> dict[str, Any]:
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    source = Path(__file__).resolve()
    config_path = Path(config_path).resolve()
    initial_hashes = {str(path): sha256_file(path) for path in (
        source, config_path,
        Path(__file__).with_name("semantic_transition_ordered_corridor.py"),
        Path(__file__).with_name("semantic_constraint_core.py"),
    )}
    config, parent_config = _load_config(config_path)
    policy = _ordered_policy(config)
    slab_half_width = float(
        config["ordered_se2_certificate"].get("station_slab_half_width_m", 0.0)
    )
    candidate_cell_provider = None
    if config["ordered_se2_certificate"].get("candidate_cell_source") == "map_cells_in_local_station_slab":
        candidate_cell_provider = lambda world, sample, lane_label, selected_policy: (
            _map_cell_candidate_cells(
                world, sample, lane_label, selected_policy,
                station_slab_half_width_m=slab_half_width,
            )
        )
    _write_json(output / "protocol.json", {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
        "configuration": config,
        "ordered_policy": asdict(policy),
        "source_hashes_at_start": initial_hashes,
        "used_historical_path_or_witness": False,
        "source_candidate_pool_use": "endpoint coordinates only",
        "processes_before": _process_audit(),
    })
    with r3._r3_bindings():
        prepared = r1._prepare(
            Path(extracted).resolve(), Path(semantic_map_path).resolve(),
            Path(topology_cache).resolve(), parent_config, output=None,
        )
    ctx, semantic_map, raster, topology, _annotator, router = prepared[:6]
    if ctx.map_sha256 != config["frozen_bindings"]["map_hash"]:
        raise ValueError("map hash mismatch")
    if semantic_map.semantic_map_hash != config["frozen_bindings"]["semantic_map_hash"]:
        raise ValueError("semantic map hash mismatch")
    selector = r1._semantic_selector(topology, router)
    builder = RegionalPreferenceBuilderR3(
        ctx.hospital_map, raster, policy=parent_config["regional_preference"],
        semantic_map=semantic_map,
    )
    composer = SemanticCostmapComposerR2(
        policy=parent_config["l3_soft_cost"], inflation_cache_capacity=2,
    )
    maximum_attachment = float(config["mirror_selection"]["maximum_l1_endpoint_attachment_distance_m"])
    attempts: list[dict[str, Any]] = []
    selected: dict[str, Any] | None = None
    for rank, candidate in enumerate(_ranked_pool(config)):
        candidate_dir = output / "probes" / candidate["candidate_id"]
        candidate_dir.mkdir(parents=True, exist_ok=False)
        directions = []
        for raw in candidate["directions"]:
            direction_started = time.monotonic()
            try:
                query, route, orientation, reason = _resolve_query_and_route(
                    selector, topology, candidate["candidate_id"], raw,
                )
                attachments = (
                    float(orientation["route_start_distance_m"]),
                    float(orientation["route_end_distance_m"]),
                )
                if max(attachments) > maximum_attachment:
                    raise ApplicabilityFailure(
                        "ENDPOINT_ATTACHMENT_LIMIT",
                        f"{max(attachments):.6f} > {maximum_attachment:.6f}",
                    )
                input_dir = candidate_dir / raw["direction"]
                meta = _prepare_input(
                    directory=input_dir, query=query, route=route, orientation=orientation,
                    ctx=ctx, semantic_map=semantic_map, raster=raster, topology=topology,
                    builder=builder, composer=composer, parent_config=parent_config,
                    targeted_hash="SELECTION_PROBE_NOT_FROZEN",
                )
                certificate = _chain_certificate(
                    input_dir, query.query_id, policy,
                    candidate_cell_provider=candidate_cell_provider,
                )
                direction_result = {
                    "direction": raw["direction"], "query": query.as_dict(),
                    "route_reason": reason,
                    "route_hash": meta["route_hash"],
                    "undirected_route_hash": meta["undirected_route_hash"],
                    "route_start_distance_m": attachments[0],
                    "route_end_distance_m": attachments[1],
                    "certificate": certificate,
                    "applicable": bool(certificate["applicable"]),
                    "wall_s": time.monotonic() - direction_started,
                }
                _write_json(candidate_dir / f"{raw['direction']}_certificate.json", direction_result)
            except ApplicabilityFailure as error:
                direction_result = {
                    "direction": raw["direction"], "applicable": False,
                    "failure_code": error.code, "failure_detail": error.detail,
                    "wall_s": time.monotonic() - direction_started,
                }
                _write_json(candidate_dir / f"{raw['direction']}_certificate.json", direction_result)
            directions.append(direction_result)
            gc.collect()
            if direction_result.get("applicable") is not True:
                break
        route_match = bool(
            len(directions) == 2
            and all(row.get("undirected_route_hash") for row in directions)
            and len({row["undirected_route_hash"] for row in directions}) == 1
        )
        pair_applicable = bool(
            route_match and len(directions) == 2
            and all(row.get("applicable") is True for row in directions)
        )
        record = {
            **{key: value for key, value in candidate.items() if key != "directions"},
            "selection_rank": rank,
            "same_undirected_l1_route": route_match,
            "directions": directions,
            "bidirectionally_applicable": pair_applicable,
        }
        attempts.append(record)
        _write_json(candidate_dir / "pair_result.json", record)
        if pair_applicable:
            selected = record
            break
    if selected is None:
        result = {
            "architecture_id": ARCHITECTURE_ID,
            "implementation_revision": IMPLEMENTATION_REVISION,
            "protocol_id": PROTOCOL_ID,
            "selection_status": "NO_BIDIRECTIONALLY_APPLICABLE_PAIR",
            "selected_candidate_id": None,
            "attempts": attempts,
            "legacy_positive": _classify_legacy_positive(config),
            "online_eligible": False,
            "continuous_space_infeasibility_proof": False,
        }
    else:
        direction_queries = []
        for index, row in enumerate(selected["directions"]):
            query_data = dict(row["query"])
            query_data["query_id"] = f"v3-mirror-{index + 1}-{row['direction']}"
            query_data["category"] = "lane_mirror_applicable_regression"
            query_data["validation_status"] = "V3_BIDIRECTIONAL_APPLICABILITY_CERTIFIED"
            direction_queries.append(query_data)
        legacy_targets = yaml.safe_load(DEFAULT_LEGACY_TARGETS.read_text(encoding="utf-8"))
        south = next(item for item in legacy_targets["queries"] if item["query_id"] == "cmp2-02-lane-south")
        frozen_queries = direction_queries + [south]
        query_hash = canonical_hash([
            {key: item[key] for key in ("query_id", "start", "goal", "category")}
            for item in frozen_queries
        ])
        frozen = {
            "schema_version": "PLN-02-2A-V3-R0-TARGETED-QUERY-V1",
            "architecture_id": ARCHITECTURE_ID,
            "implementation_revision": IMPLEMENTATION_REVISION,
            "protocol_id": PROTOCOL_ID,
            "map_hash": ctx.map_sha256,
            "semantic_map_hash": semantic_map.semantic_map_hash,
            "query_hash": query_hash,
            "source_candidate_id": selected["candidate_id"],
            "selection_rule": config["mirror_selection"]["selection_rule"],
            "queries": frozen_queries,
            "excluded_negative_regressions": [_classify_legacy_positive(config)],
        }
        frozen_path = output / "pudu_wanda_3f_v3_targeted3_r0.yaml"
        with frozen_path.open("x", encoding="utf-8") as stream:
            yaml.safe_dump(frozen, stream, allow_unicode=True, sort_keys=False)
        result = {
            "architecture_id": ARCHITECTURE_ID,
            "implementation_revision": IMPLEMENTATION_REVISION,
            "protocol_id": PROTOCOL_ID,
            "selection_status": "FROZEN",
            "selected_candidate_id": selected["candidate_id"],
            "selected": selected,
            "attempts": attempts,
            "frozen_query_set": str(frozen_path),
            "frozen_query_hash": query_hash,
            "legacy_positive": frozen["excluded_negative_regressions"][0],
            "offline_triad_eligible": True,
            "online_eligible": False,
            "online_gate_reason": "OFFLINE_TRIAD_NOT_YET_RUN",
        }
    final_hashes = {str(path): sha256_file(path) for path in map(Path, initial_hashes)}
    if final_hashes != initial_hashes:
        raise RuntimeError("source changed while applicability selection was running")
    result.update({
        "wall_s": time.monotonic() - started,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        "source_hashes_at_end": final_hashes,
        "python": platform.python_version(),
        "processes_after": _process_audit(),
    })
    _write_json(output / "selection_result.json", result)
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    shutil.copy2(source, snapshot / source.name)
    shutil.copy2(config_path, snapshot / config_path.name)
    _write_json(output / "artifact_hashes.json", _manifest_files(output))
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--extracted", type=Path, default=DEFAULT_EXTRACTED)
    parser.add_argument("--semantic-map", type=Path, default=DEFAULT_SEMANTIC_MAP)
    parser.add_argument("--topology-cache", type=Path, default=DEFAULT_TOPOLOGY)
    args = parser.parse_args(argv)
    result = run(
        output=args.output, config_path=args.config, extracted=args.extracted,
        semantic_map_path=args.semantic_map, topology_cache=args.topology_cache,
    )
    print(json.dumps({
        "selection_status": result["selection_status"],
        "selected_candidate_id": result.get("selected_candidate_id"),
        "frozen_query_hash": result.get("frozen_query_hash"),
        "wall_s": result["wall_s"],
    }, indent=2, sort_keys=True))
    return 0 if result["selection_status"] == "FROZEN" else 2


if __name__ == "__main__":
    raise SystemExit(main())
