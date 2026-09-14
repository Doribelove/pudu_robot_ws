"""Prepare and run the 2A-V3 r13 route-phase SE(2) offline gate.

Inputs are regenerated from the frozen map, semantic map, selected8 request,
and L1 route.  Saved paths are never consumed by the planner.  The prepare and
plan phases use exclusive output creation so prior experiments remain read-only.
"""
from __future__ import annotations

import argparse
import gc
import json
import math
from pathlib import Path
import platform
import resource
import shutil
import time
from typing import Any, Mapping, Sequence

import cv2
import numpy as np
import yaml

from . import semantic_applicability_v3 as applicability
from . import two_layer_v2_semantic_r1_benchmark as r1
from . import two_layer_v2_semantic_r3_benchmark as r3
from . import two_layer_v3_semantic_r0_benchmark as v3_targeted
from .planner_benchmark.models import Query
from .regional_preference_r1 import expand_roi_to_route_lane_instances, orient_route_for_query
from .regional_preference_r3 import RegionalPreferenceBuilderR3
from .semantic_costmap_r2 import SemanticCostmapComposerR2
from .semantic_map import SemanticMapV1, canonical_hash, sha256_file
from .semantic_query_defaults import load_query_set
from .semantic_rasterizer import grid_hash
from .semantic_route_phase_v3 import (
    LazyRoutePhaseSearch,
    RoutePhasePolicy,
    RoutePhaseWorld,
    OrientedRoute,
    policy_dict,
)
from .semantic_transition_ordered_corridor import CorridorFailure


ARCHITECTURE_ID = "2A-V3"
IMPLEMENTATION_REVISION = "r13-route-phase-multisemantic-state-lattice"
PROTOCOL_ID = "PLN-02-2A-V3-R13-ROUTE-PHASE-MULTISEMANTIC-V1"
SCHEMA_VERSION = "PLN-02-2A-V3-R13-OFFLINE-V1"
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ROOT = Path(__file__).resolve().parents[7]
DEFAULT_CONFIG = PACKAGE_ROOT / "config/two_layer_v3_semantic_r13_route_phase.yaml"
DEFAULT_EXTRACTED = ROOT / "private_data/pudu_wanda_3f/extracted"
DEFAULT_SEMANTIC_MAP = ROOT / "private_data/pudu_wanda_3f/results/conversion_v1/semantic_map_v1.json"
DEFAULT_TOPOLOGY = ROOT / "private_data/pudu_wanda_3f/results/real_ablation_r1_diag_v20_final8/topology_cache"


def _identity() -> dict[str, str]:
    return {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
    }


def _write_json(path: Path, payload: Any) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)
        stream.write("\n")


def _load_config(path: Path) -> tuple[dict[str, Any], dict[str, Any], Path]:
    path = path.resolve()
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    for key, value in _identity().items():
        if config.get(key) != value:
            raise ValueError(f"2A-V3 r13 identity mismatch for {key}")
    parent_path = Path(str(config["parent_config"]["path"]))
    if not parent_path.is_absolute():
        parent_path = path.parent / parent_path
    parent_path = parent_path.resolve()
    if sha256_file(parent_path) != config["parent_config"]["sha256"]:
        raise ValueError("frozen parent config changed")
    query_path = Path(str(config["query_set"]["path"]))
    if not query_path.is_absolute():
        query_path = path.parent / query_path
    query_path = query_path.resolve()
    if sha256_file(query_path) != config["query_set"]["file_sha256"]:
        raise ValueError("frozen selected8 file changed")
    targeted_path = Path(str(config["targeted_query_set"]["path"]))
    if not targeted_path.is_absolute():
        targeted_path = path.parent / targeted_path
    if sha256_file(targeted_path.resolve()) != config["targeted_query_set"]["file_sha256"]:
        raise ValueError("frozen targeted query file changed")
    with r3._r3_bindings():
        parent = r1._load_config(parent_path)
    return config, parent, query_path


def _query_scope(
    *, config: Mapping[str, Any], query_path: Path, scope: str,
    map_path: Path,
) -> tuple[list[Query], str, Path]:
    if scope == "selected8":
        queries, _content, metadata = load_query_set(
            query_path, actual_map_hash=sha256_file(map_path),
            actual_semantic_map_hash=config["frozen_bindings"]["semantic_map_hash"],
            require_default_contract=True,
        )
        expected = str(config["query_set"]["query_hash"])
        actual = str(metadata["query_hash"])
        source = query_path
    elif scope == "targeted":
        binding = config["targeted_query_set"]
        source = Path(str(binding["path"]))
        if not source.is_absolute():
            source = query_path.parent / source
        source = source.resolve()
        raw = applicability._load_extended_mapping(source, "extends_targeted")
        if raw.get("map_hash") != config["frozen_bindings"]["map_hash"]:
            raise ValueError("targeted map hash changed")
        if raw.get("semantic_map_hash") != config["frozen_bindings"]["semantic_map_hash"]:
            raise ValueError("targeted semantic map hash changed")
        actual = v3_targeted._targeted_hash(raw.get("queries", []))
        expected = str(binding["query_hash"])
        queries = [
            Query(
                query_id=str(row["query_id"]),
                start=list(map(float, row["start"])),
                goal=list(map(float, row["goal"])),
                category=str(row["category"]),
                seed=int(row.get("seed", raw.get("seed", 20260907))),
            )
            for row in raw.get("queries", [])
        ]
    else:
        raise ValueError(f"unknown query scope: {scope}")
    if actual != expected:
        raise ValueError(f"{scope} query content hash changed")
    return queries, actual, source


def _policy(config: Mapping[str, Any]) -> RoutePhasePolicy:
    raw = dict(config["route_phase_policy"])
    raw["yaw_neighbor_bins"] = tuple(map(int, raw["yaw_neighbor_bins"]))
    return RoutePhasePolicy(**raw)


def _route_parking_components(hospital_map, parking_components, route_polyline, radius_m=.50):
    selected = set()
    points = np.asarray(route_polyline, dtype=np.float64)[:, :2]
    for first, second in zip(points, points[1:]):
        length = float(np.linalg.norm(second-first))
        for fraction in np.linspace(0.0, 1.0, max(2, int(math.ceil(length/.10))+1)):
            point = first+(second-first)*fraction
            cell = hospital_map.world_to_cell(float(point[0]), float(point[1]))
            if cell is None:
                continue
            value = int(parking_components[cell])
            if value:
                selected.add(value)
                continue
            radius = int(math.ceil(radius_m/hospital_map.resolution))
            row, col = cell
            view = parking_components[
                max(0, row-radius):min(hospital_map.height, row+radius+1),
                max(0, col-radius):min(hospital_map.width, col+radius+1),
            ]
            choices = sorted(int(v) for v in np.unique(view) if int(v) > 0)
            if len(choices) == 1:
                selected.add(choices[0])
    return sorted(selected)


def _query_hash(queries) -> str:
    return canonical_hash([
        {key: row[key] for key in ("query_id", "start", "goal", "category")}
        for row in queries
    ])


def prepare_query_input(
    *, query: Query, route: Any, orientation: Mapping[str, Any], ctx: Any,
    semantic_map: Any, raster: Any, topology: Any,
    builder: RegionalPreferenceBuilderR3, composer: SemanticCostmapComposerR2,
    parent: Mapping[str, Any], query_set_hash: str,
) -> tuple[dict[str, Any], dict[str, np.ndarray], Any]:
    """Build one request-bound route-phase input without consuming a saved path.

    The returned arrays are deliberately suitable for both the offline writer
    and the online exact-master adapter.  Route identity contains the frozen L1
    node/edge/polyline hash as well as a separate oriented-polyline hash; this
    avoids treating geometrically equal routes with different topology bindings
    as interchangeable.
    """
    roi_started = time.monotonic_ns()
    base_allowed = r1.r2_runtime._raw_corridor_mask(
        ctx, topology, route, query, float(parent["roi"]["r0_padding_m"]),
    )
    free = r1.r2_runtime._raw_free_mask(ctx)
    expanded, roi_diagnostics = expand_roi_to_route_lane_instances(
        ctx.hospital_map, raster, semantic_map, route.polyline, base_allowed,
        free_mask=free,
        route_probe_radius_m=float(parent["roi"].get("lane_route_probe_radius_m", .50)),
    )
    roi_ms = (time.monotonic_ns()-roi_started)/1e6
    field_started = time.monotonic_ns()
    preference = builder.build(
        route.polyline, goal=query.goal, allowed_mask=expanded,
        relaxation_level="R0", planning_preference_enabled=True,
        route_diagnostics=orientation,
    )
    field_ms = (time.monotonic_ns()-field_started)/1e6

    parking_mask = np.asarray(
        raster.masks.get("parking_area", np.zeros_like(ctx.hospital_map.occupancy, bool)),
        bool,
    )
    _count, parking_components = cv2.connectedComponents(
        parking_mask.astype(np.uint8), connectivity=8,
    )
    lane_mask = np.asarray(raster.masks.get("lane", np.zeros_like(parking_mask)), bool)
    junction = np.asarray(raster.masks.get("junction_area", np.zeros_like(parking_mask)), bool)
    inverse = {
        semantic_id: int(label)
        for label, semantic_id in builder._lane_instance_ids.items()
    }
    selected_lane_ids = list(roi_diagnostics.get("selected_lane_instance_ids", []))
    selected_lanes = sorted(
        inverse[value] for value in selected_lane_ids if value in inverse
    )
    selected_parking = _route_parking_components(
        ctx.hospital_map, parking_components, route.polyline,
    )
    labels = np.asarray(preference.lane_instance_id, dtype=np.int32)
    parking_deviation = np.asarray(
        preference.parking_normalized_deviation, dtype=np.float32,
    )
    lane_allowed = np.isin(labels, selected_lanes)
    parking_allowed = np.isin(parking_components, selected_parking)
    neutral = base_allowed & (junction | ~(lane_mask | parking_mask))
    phase_support = (lane_allowed | parking_allowed | neutral) & free
    # ``expanded`` is the frozen R0 route-lane physical ROI: the actual lane
    # instances crossed by the route plus their necessary connector.  It is
    # not a generic relaxed/R3 expansion.  Primitive-level phase conformance
    # below still rejects an adjacent/wrong lane.  Publishing only the target
    # semantic pixels makes the ROI boundary itself lethal; Nav2 inflation can
    # then erase a valid target band even though the approved R0 route-lane ROI
    # contains it.
    publication_allowed = np.asarray(expanded, dtype=bool) & free
    allowed = phase_support
    for pose in (query.start, query.goal):
        cell = ctx.hospital_map.world_to_cell(float(pose[0]), float(pose[1]))
        if cell is not None and free[cell]:
            allowed[cell] = True
    if not selected_lanes and not selected_parking:
        raise RuntimeError(
            f"{query.query_id}: route has no bound lane or parking semantic instance"
        )
    # The exact-effective ACK and the independent search must consume the
    # identical final route-phase ROI.  Composing against the broader
    # lane-expanded probe mask would leave expected non-lethal cells outside
    # the published ROI and correctly fail byte-exact server verification.
    compose_started = time.monotonic_ns()
    composition = composer.compose(
        ctx.hospital_map.occupancy, raster, preference,
        allowed_mask=publication_allowed,
        hard_semantics_enabled=True, soft_class_costs_enabled=True,
        regional_preference_enabled=True, hard_semantics_use_footprint=True,
    )
    compose_ms = (time.monotonic_ns()-compose_started)/1e6
    arrays = {
        "master": np.asarray(composition.expected_master_cost, dtype=np.uint8),
        "occupancy": np.asarray(ctx.hospital_map.occupancy),
        "allowed": np.asarray(allowed, dtype=bool),
        "publication_allowed": np.asarray(publication_allowed, dtype=bool),
        "base_allowed": np.asarray(base_allowed, dtype=bool),
        "labels": labels,
        "lane_mask": lane_mask,
        "error": np.asarray(preference.lane_error_m, dtype=np.float32),
        "correct": np.asarray(preference.lane_correct_side, dtype=bool),
        "right": np.asarray(preference.lane_distance_to_right_m, dtype=np.float32),
        "left": np.asarray(preference.lane_distance_to_left_m, dtype=np.float32),
        "parking_components": parking_components.astype(np.int32),
        "parking_deviation": parking_deviation,
        "junction": junction,
        "hard": np.asarray(raster.hard_footprint_mask, dtype=bool),
        "no_stopping": np.asarray(raster.no_stopping_mask, dtype=bool),
    }
    metadata = {
        **_identity(), "query": query.as_dict(),
        "query_set_content_hash": str(query_set_hash),
        "map_hash": ctx.map_sha256,
        "semantic_map_hash": semantic_map.semantic_map_hash,
        "expected_master_hash": composition.expected_master_hash,
        "route_hash": r1._path_hash(route),
        "route_polyline_hash": canonical_hash(route.polyline),
        "route_polyline": route.polyline,
        "route_orientation": dict(orientation),
        "roi_hash": grid_hash(publication_allowed),
        "search_roi_hash": grid_hash(allowed),
        "phase_support_hash": grid_hash(phase_support),
        "base_roi_hash": grid_hash(base_allowed),
        "selected_lane_labels": selected_lanes,
        "selected_lane_semantic_ids": selected_lane_ids,
        "selected_parking_components": selected_parking,
        "roi_diagnostics": roi_diagnostics,
        "preference_diagnostics": preference.diagnostics,
        "map": {
            "resolution": ctx.hospital_map.resolution,
            "origin": ctx.hospital_map.origin,
            "width": ctx.hospital_map.width,
            "height": ctx.hospital_map.height,
            "image_path": str(ctx.hospital_map.image_path),
        },
        "timing": {
            "roi_build_ms": roi_ms,
            "field_build_ms": field_ms,
            "compose_ms": compose_ms,
        },
        "npz_sha256": "",
    }
    return metadata, arrays, composition


def prepare(
    *, output: Path, config_path: Path = DEFAULT_CONFIG,
    extracted: Path = DEFAULT_EXTRACTED,
    semantic_map_path: Path = DEFAULT_SEMANTIC_MAP,
    topology_cache: Path = DEFAULT_TOPOLOGY,
    scope: str = "selected8",
) -> dict[str, Any]:
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    config, parent, query_path = _load_config(config_path)
    map_path = extracted.resolve() / "optemap.pgm"
    queries, query_hash, query_source = _query_scope(
        config=config, query_path=query_path, scope=scope, map_path=map_path,
    )
    sources = [Path(__file__).resolve(), Path(config_path).resolve(), query_source,
               Path(__import__("arena_evaluation.semantic_route_phase_v3", fromlist=["x"]).__file__).resolve()]
    hashes = {str(path): sha256_file(path) for path in sources}
    _write_json(output / "protocol.json", {
        **_identity(), "schema_version": SCHEMA_VERSION,
        "mode": "prepare_request_derived_route_phase_inputs",
        "query_hash": query_hash, "query_scope": scope, "used_historical_paths": False,
        "source_hashes_at_start": hashes,
        "processes_before": applicability._process_audit(),
    })
    with r3._r3_bindings():
        prepared = r1._prepare(
            extracted.resolve(), semantic_map_path.resolve(), topology_cache.resolve(),
            parent, output=None,
        )
    ctx, semantic_map, raster, topology, _annotator, router = prepared[:6]
    if ctx.map_sha256 != config["frozen_bindings"]["map_hash"]:
        raise ValueError("map hash mismatch")
    if semantic_map.semantic_map_hash != config["frozen_bindings"]["semantic_map_hash"]:
        raise ValueError("semantic map hash mismatch")
    e4_switches = r1.ArmSwitches.parse(parent["ablation_arms"]["E4"])
    selector = r1._selector_for_arm(
        e4_switches, topology, router, {},
        preferred_attachment_radius_m=float(parent["endpoint_attachment"]["preferred_radius_m"]),
        attachment_cost_weight=float(parent["endpoint_attachment"]["cost_weight"]),
    )
    builder = RegionalPreferenceBuilderR3(
        ctx.hospital_map, raster, policy=parent["regional_preference"],
        semantic_map=semantic_map,
    )
    composer = SemanticCostmapComposerR2(policy=parent["l3_soft_cost"], inflation_cache_capacity=2)
    rows = []
    for query in queries:
        query_started = time.monotonic()
        _sn, _gn, route, reason = selector(
            topology, query, cache_mode=r1.r2_runtime.CACHE_MODE_OPTIMIZED, timing={},
        )
        if route is None:
            raise RuntimeError(f"L1 route failed for {query.query_id}: {reason}")
        route, orientation = orient_route_for_query(route, query)
        meta, arrays, _composition = prepare_query_input(
            query=query, route=route, orientation=orientation, ctx=ctx,
            semantic_map=semantic_map, raster=raster, topology=topology,
            builder=builder, composer=composer, parent=parent,
            query_set_hash=query_hash,
        )
        query_dir = output / query.query_id
        query_dir.mkdir()
        archive = query_dir / f"{query.query_id}.npz"
        np.savez_compressed(archive, **arrays)
        meta["npz_sha256"] = sha256_file(archive)
        _write_json(query_dir / f"{query.query_id}.json", meta)
        rows.append({
            "query_id": query.query_id, "route_hash": meta["route_hash"],
            "route_polyline_hash": meta["route_polyline_hash"],
            "roi_hash": meta["roi_hash"], "expected_master_hash": meta["expected_master_hash"],
            "selected_lane_labels": meta["selected_lane_labels"],
            "selected_parking_components": meta["selected_parking_components"],
            "npz_sha256": meta["npz_sha256"],
            "wall_s": time.monotonic()-query_started,
        })
        gc.collect()
    result = {
        **_identity(), "schema_version": SCHEMA_VERSION,
        "mode": "prepare_request_derived_route_phase_inputs",
        "query_hash": query_hash, "query_scope": scope,
        "query_count": len(rows), "rows": rows,
        "wall_s": time.monotonic()-started,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
        "python": platform.python_version(),
        "processes_after": applicability._process_audit(),
    }
    _write_json(output / "prepare_result.json", result)
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    for source in sources:
        shutil.copy2(source, snapshot / source.name)
    _write_json(output / "artifact_hashes.json", applicability._manifest_files(output))
    return result


def plan(
    *, inputs: Path, query_id: str, output: Path,
    config_path: Path = DEFAULT_CONFIG,
    semantic_map_path: Path = DEFAULT_SEMANTIC_MAP,
    scope: str = "selected8",
    allow_safe_soft_fallback: bool = False,
    prefer_lazy: bool = False,
    fallback_first: bool = False,
) -> dict[str, Any]:
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started, cpu_started = time.monotonic(), time.process_time()
    config, _parent, query_path = _load_config(config_path)
    queries, query_hash, query_source = _query_scope(
        config=config, query_path=query_path, scope=scope,
        map_path=DEFAULT_EXTRACTED / "optemap.pgm",
    )
    if query_id not in [query.query_id for query in queries]:
        raise ValueError(f"query is not in frozen {scope}: {query_id}")
    policy = _policy(config)
    source_paths = [Path(__file__).resolve(), Path(config_path).resolve(),
                    Path(__import__("arena_evaluation.semantic_route_phase_v3", fromlist=["x"]).__file__).resolve()]
    hashes = {str(path): sha256_file(path) for path in source_paths}
    _write_json(output / "protocol.json", {
        **_identity(), "schema_version": SCHEMA_VERSION, "mode": "fresh_offline_route_phase_plan",
        "query_id": query_id, "query_hash": query_hash, "query_scope": scope,
        "policy": policy_dict(policy), "used_historical_paths": False,
        "source_hashes_at_start": hashes,
        "processes_before": applicability._process_audit(),
    })
    result: dict[str, Any]
    witness = None
    try:
        world = RoutePhaseWorld(inputs.resolve() / query_id, query_id)
        if world.meta["query_set_content_hash"] != query_hash:
            raise CorridorFailure("QUERY_BINDING_MISMATCH", f"{scope} input hash changed")
        semantic_map = SemanticMapV1.load(semantic_map_path.resolve())
        if semantic_map.semantic_map_hash != world.meta["semantic_map_hash"]:
            raise CorridorFailure("SEMANTIC_BINDING_MISMATCH", "semantic map hash changed")
        route = OrientedRoute(
            world.meta["route_polyline"], world.start, world.goal,
            endpoint_attachment_limit_m=policy.endpoint_attachment_limit_m,
        )
        searcher = LazyRoutePhaseSearch(world, route, policy)
        witness, evaluations, search = searcher.search(
            semantic_map,
            allow_safe_soft_fallback=allow_safe_soft_fallback,
            prefer_lazy=prefer_lazy,
            fallback_first=fallback_first,
        )
        result = {
            **_identity(), "schema_version": SCHEMA_VERSION,
            "query_id": query_id, "gate_passed": witness is not None,
            "failure_code": "" if witness is not None else (
                "NO_ROUTE_PHASE_SE2_ROUTE" if not evaluations and not search["remaining_heap_count"]
                else "ROUTE_PHASE_GRAPH_NO_STRICT_WITNESS"
            ),
            "search": search, "candidate_evaluations": evaluations,
            "route": {"route_hash": route.route_hash, "route_length_m": route.length_m},
            "map_hash": world.meta["map_hash"],
            "semantic_map_hash": world.meta["semantic_map_hash"],
            "input_npz_sha256": world.meta["npz_sha256"],
            "safe_soft_fallback_allowed": bool(allow_safe_soft_fallback),
            "prefer_lazy_search": bool(prefer_lazy),
            "fallback_first_search": bool(fallback_first),
            "semantic_success_counted": (
                None if witness is None else bool(witness["semantic_success_counted"])
            ),
            "safe_soft_fallback": (
                False if witness is None else bool(witness.get("safe_soft_fallback", False))
            ),
            "used_historical_paths": False, "online": False,
        }
    except CorridorFailure as error:
        result = {
            **_identity(), "schema_version": SCHEMA_VERSION,
            "query_id": query_id, "gate_passed": False,
            "failure_code": error.code, "failure_detail": error.detail,
            "used_historical_paths": False, "online": False,
        }
    result.update({
        "wall_s": time.monotonic()-started,
        "cpu_s": time.process_time()-cpu_started,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
        "python": platform.python_version(),
        "processes_after": applicability._process_audit(),
        "source_hashes_at_start": hashes,
    })
    if witness is not None:
        _write_json(output / "path.json", witness.pop("points"))
        _write_json(output / "controls.json", {"edges": witness.pop("controls")})
        _write_json(output / "witness_audit.json", witness)
    elif 'searcher' in locals() and searcher.best_failed is not None:
        failed = searcher.best_failed
        _write_json(output / "best_failed_path.json", failed.pop("points"))
        _write_json(output / "best_failed_controls.json", {"edges": failed.pop("controls")})
        _write_json(output / "best_failed_audit.json", failed)
    _write_json(output / "result.json", result)
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    for source in source_paths:
        shutil.copy2(source, snapshot / source.name)
    _write_json(output / "artifact_hashes.json", applicability._manifest_files(output))
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    prep.add_argument("--output", type=Path, required=True)
    prep.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    prep.add_argument("--scope", choices=("targeted", "selected8"), default="selected8")
    run = sub.add_parser("plan")
    run.add_argument("--inputs", type=Path, required=True)
    run.add_argument("--query", required=True)
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    run.add_argument("--scope", choices=("targeted", "selected8"), default="selected8")
    run.add_argument("--allow-safe-soft-fallback", action="store_true")
    run.add_argument("--prefer-lazy", action="store_true")
    run.add_argument("--fallback-first", action="store_true")
    args = parser.parse_args(argv)
    if args.command == "prepare":
        result = prepare(output=args.output, config_path=args.config, scope=args.scope)
    else:
        result = plan(
            inputs=args.inputs, query_id=args.query, output=args.output,
            config_path=args.config, scope=args.scope,
            allow_safe_soft_fallback=args.allow_safe_soft_fallback,
            prefer_lazy=args.prefer_lazy,
            fallback_first=args.fallback_first,
        )
    print(json.dumps({
        "command": args.command, "query_id": result.get("query_id"),
        "query_count": result.get("query_count"), "gate_passed": result.get("gate_passed"),
        "failure_code": result.get("failure_code", ""), "wall_s": result.get("wall_s"),
    }, indent=2, sort_keys=True))
    return 0 if args.command == "prepare" or result.get("gate_passed") else 2


if __name__ == "__main__":
    raise SystemExit(main())
