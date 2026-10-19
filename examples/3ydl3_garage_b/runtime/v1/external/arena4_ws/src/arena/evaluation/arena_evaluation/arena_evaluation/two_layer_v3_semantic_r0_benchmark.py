"""Fail-closed offline preflight for the 2A-V3 directed SE(2) corridor.

This runner generates every input from the frozen map and request.  It does
not load an earlier path or witness.  ``prepare`` creates immutable per-query
effective-master inputs; ``plan`` constructs and searches a fresh, ordered,
forward-only 48-bin Dubins graph and applies the strict R2 semantic,
full-footprint safety, revisit, necessity and goal-plane audits.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import gc
import json
from pathlib import Path
import platform
import resource
import shutil
import time
from typing import Any, Mapping, Sequence

import yaml

from . import semantic_applicability_v3 as applicability
from . import two_layer_v2_semantic_r1_benchmark as r1
from . import two_layer_v2_semantic_r3_benchmark as r3
from .planner_benchmark.models import Query
from .regional_preference_r1 import orient_route_for_query
from .regional_preference_r3 import RegionalPreferenceBuilderR3
from .semantic_constraint_core import ConstraintWorld
from .semantic_costmap_r2 import SemanticCostmapComposerR2
from .semantic_map import SemanticMapV1, canonical_hash, sha256_file
from .semantic_transition_ordered_corridor import (
    CorridorFailure,
    OrientedRoute,
    _controls_for_label,
    evaluate_controls,
    build_graph,
    search_complete_paths,
)
from .semantic_transition_lazy_corridor import (
    LazyEdgeFactory, LazySearchPolicy, search_lazy,
)


ARCHITECTURE_ID = "2A-V3"
IMPLEMENTATION_REVISION = "r10-local-station-prefilter-strict-final"
PROTOCOL_ID = "PLN-02-2A-V3-R10-LOCAL-STATION-PREFILTER-STRICT-FINAL-V1"
SCHEMA_VERSION = "PLN-02-2A-V3-OFFLINE-PREFLIGHT-V1"
DEFAULT_CONFIG = applicability.PACKAGE_ROOT / "config/two_layer_v3_semantic_r10_local_projection.yaml"
DEFAULT_TARGETED = applicability.PACKAGE_ROOT / "config/pudu_wanda_3f_v3_r10_targeted_applicable_positive_v1.yaml"


def _identity() -> dict[str, str]:
    return {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
    }


def _load_config(
    path: Path, *, expected_identity: Mapping[str, str] | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    path = path.resolve()
    config, parent = applicability._load_config(
        path, expected_identity=expected_identity or _identity(),
    )
    predecessor = config.get("predecessor_config", {})
    predecessor_path = Path(str(predecessor.get("path", "")))
    if not predecessor_path.is_absolute():
        predecessor_path = path.parent / predecessor_path
    if sha256_file(predecessor_path) != str(predecessor.get("sha256")):
        raise ValueError("2A-V3 predecessor configuration changed")
    result_path = Path(str(predecessor.get("frozen_result", "")))
    if not result_path.is_absolute():
        result_path = applicability.ROOT / result_path
    if sha256_file(result_path) != str(predecessor.get("frozen_result_sha256")):
        raise ValueError("2A-V3 predecessor failure result changed")
    return config, parent


def _write_json(path: Path, payload: Any) -> None:
    applicability._write_json(path, payload)


def _targeted_hash(queries: Sequence[Mapping[str, Any]]) -> str:
    return canonical_hash([
        {key: query[key] for key in ("query_id", "start", "goal", "category")}
        for query in queries
    ])


def _load_targeted(
    path: Path, config: Mapping[str, Any], *,
    expected_identity: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    path = path.resolve()
    payload = applicability._load_extended_mapping(path, "extends_targeted")
    expected = {
        **(expected_identity or _identity()),
        "map_hash": config["frozen_bindings"]["map_hash"],
        "semantic_map_hash": config["frozen_bindings"]["semantic_map_hash"],
    }
    for key, value in expected.items():
        if payload.get(key) != value:
            raise ValueError(f"targeted query binding mismatch for {key}")
    actual_hash = _targeted_hash(payload.get("queries", []))
    if payload.get("query_hash") != actual_hash:
        raise ValueError("targeted query content hash mismatch")
    for evidence in payload.get("source_evidence", {}).values():
        evidence_path = Path(str(evidence["path"]))
        if not evidence_path.is_absolute():
            evidence_path = applicability.ROOT / evidence_path
        if sha256_file(evidence_path) != str(evidence["sha256"]):
            raise ValueError(f"frozen evidence changed: {evidence_path}")
    replay_path = applicability.ROOT / payload["source_evidence"]["deterministic_replay"]["path"]
    replay = json.loads(replay_path.read_text(encoding="utf-8"))
    if replay.get("gate_passed") is not True or replay.get("repetition_count", 0) < 3:
        raise ValueError("positive applicability replay gate is not satisfied")
    return payload


def _provider(config: Mapping[str, Any]):
    raw = config["ordered_se2_certificate"]
    if raw.get("candidate_cell_source") != "map_cells_in_local_station_slab":
        return None
    half_width = float(raw["station_slab_half_width_m"])
    return lambda world, sample, lane_label, policy: applicability._map_cell_candidate_cells(
        world, sample, lane_label, policy,
        station_slab_half_width_m=half_width,
    )


def _lazy_policy(config: Mapping[str, Any]) -> LazySearchPolicy:
    raw = config.get("lazy_search", {})
    fields = set(LazySearchPolicy.__dataclass_fields__)
    return LazySearchPolicy(**{key: value for key, value in raw.items() if key in fields})


def prepare(
    *, output: Path, config_path: Path = DEFAULT_CONFIG,
    targeted_path: Path = DEFAULT_TARGETED,
    extracted: Path = applicability.DEFAULT_EXTRACTED,
    semantic_map_path: Path = applicability.DEFAULT_SEMANTIC_MAP,
    topology_cache: Path = applicability.DEFAULT_TOPOLOGY,
) -> dict[str, Any]:
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    config, parent_config = _load_config(config_path)
    targeted = _load_targeted(targeted_path.resolve(), config)
    sources = [Path(__file__).resolve(), applicability.__file__, config_path.resolve(), targeted_path.resolve()]
    start_hashes = {str(Path(path)): sha256_file(Path(path)) for path in sources}
    _write_json(output / "protocol.json", {
        "schema_version": SCHEMA_VERSION,
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
        "mode": "prepare_request_derived_inputs",
        "query_hash": targeted["query_hash"],
        "used_historical_path_or_witness": False,
        "source_hashes_at_start": start_hashes,
        "processes_before": applicability._process_audit(),
    })
    with r3._r3_bindings():
        prepared = r1._prepare(
            extracted.resolve(), semantic_map_path.resolve(), topology_cache.resolve(),
            parent_config, output=None,
        )
    ctx, semantic_map, raster, topology, _annotator, router = prepared[:6]
    if ctx.map_sha256 != targeted["map_hash"]:
        raise ValueError("map hash mismatch")
    if semantic_map.semantic_map_hash != targeted["semantic_map_hash"]:
        raise ValueError("semantic map hash mismatch")
    selector = r1._semantic_selector(topology, router)
    builder = RegionalPreferenceBuilderR3(
        ctx.hospital_map, raster, policy=parent_config["regional_preference"],
        semantic_map=semantic_map,
    )
    composer = SemanticCostmapComposerR2(
        policy=parent_config["l3_soft_cost"], inflation_cache_capacity=2,
    )
    rows = []
    for raw in targeted["queries"]:
        query = Query(
            query_id=str(raw["query_id"]),
            start=[float(value) for value in raw["start"]],
            goal=[float(value) for value in raw["goal"]],
            category=str(raw["category"]), seed=int(raw.get("seed", targeted["seed"])),
        )
        query_started = time.monotonic()
        _sn, _gn, route, reason = selector(
            topology, query, cache_mode=r1.r2_runtime.CACHE_MODE_OPTIMIZED, timing={},
        )
        if route is None:
            raise RuntimeError(f"L1 route failed for {query.query_id}: {reason}")
        route, orientation = orient_route_for_query(route, query)
        metadata = applicability._prepare_input(
            directory=output / query.query_id, query=query, route=route,
            orientation=orientation, ctx=ctx, semantic_map=semantic_map,
            raster=raster, topology=topology, builder=builder, composer=composer,
            parent_config=parent_config, targeted_hash=targeted["query_hash"],
            identity=_identity(),
        )
        if query.query_id == "v3-applicable-mirror-positive":
            expected = targeted["intent_validation"][0]["verification"]
            if metadata["route_hash"] != expected["route_hash"]:
                raise RuntimeError("frozen positive L1 route hash changed")
            if metadata["undirected_route_hash"] != expected["undirected_route_hash"]:
                raise RuntimeError("frozen positive undirected route hash changed")
        rows.append({
            "query_id": query.query_id,
            "route_hash": metadata["route_hash"],
            "undirected_route_hash": metadata["undirected_route_hash"],
            "roi_hash": metadata["roi_hash"],
            "expected_master_hash": metadata["expected_master_hash"],
            "npz_sha256": metadata["npz_sha256"],
            "wall_s": time.monotonic() - query_started,
        })
        gc.collect()
    end_hashes = {str(Path(path)): sha256_file(Path(path)) for path in sources}
    if end_hashes != start_hashes:
        raise RuntimeError("source changed during input preparation")
    result = {
        "schema_version": SCHEMA_VERSION,
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
        "mode": "prepare_request_derived_inputs",
        "query_hash": targeted["query_hash"],
        "query_count": len(rows),
        "rows": rows,
        "wall_s": time.monotonic() - started,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        "python": platform.python_version(),
        "source_hashes_at_end": end_hashes,
        "processes_after": applicability._process_audit(),
    }
    _write_json(output / "prepare_result.json", result)
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    for source in sources:
        shutil.copy2(source, snapshot / Path(source).name)
    _write_json(output / "artifact_hashes.json", applicability._manifest_files(output))
    return result


def plan(
    *, inputs: Path, query_id: str, output: Path,
    config_path: Path = DEFAULT_CONFIG, targeted_path: Path = DEFAULT_TARGETED,
    semantic_map_path: Path = applicability.DEFAULT_SEMANTIC_MAP,
) -> dict[str, Any]:
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started, cpu_started = time.monotonic(), time.process_time()
    config, _ = _load_config(config_path)
    targeted = _load_targeted(targeted_path.resolve(), config)
    query_ids = [str(row["query_id"]) for row in targeted["queries"]]
    if query_id not in query_ids:
        raise ValueError(f"query is not in the frozen targeted set: {query_id}")
    policy = applicability._ordered_policy(config)
    sources = [
        Path(__file__).resolve(), Path(applicability.__file__),
        Path(build_graph.__code__.co_filename).resolve(), config_path.resolve(), targeted_path.resolve(),
    ]
    if config.get("performance_implementation", {}).get("graph_generation") == "lazy_guided":
        sources.append(Path(search_lazy.__code__.co_filename).resolve())
    source_hashes = {str(path): sha256_file(path) for path in sources}
    _write_json(output / "protocol.json", {
        "schema_version": SCHEMA_VERSION,
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
        "mode": "fresh_offline_directed_se2_plan",
        "query_id": query_id,
        "query_hash": targeted["query_hash"],
        "policy": asdict(policy),
        "lazy_search_policy": (
            asdict(_lazy_policy(config))
            if config.get("performance_implementation", {}).get("graph_generation") == "lazy_guided"
            else None
        ),
        "used_historical_path_or_witness": False,
        "source_hashes_at_start": source_hashes,
        "processes_before": applicability._process_audit(),
    })
    result: dict[str, Any]
    witness = None
    try:
        world = ConstraintWorld(inputs.resolve(), query_id)
        if world.meta.get("targeted_query_content_hash") != targeted["query_hash"]:
            raise CorridorFailure("QUERY_BINDING_MISMATCH", "prepared input query hash changed")
        semantic_map = SemanticMapV1.load(semantic_map_path.resolve())
        if semantic_map.semantic_map_hash != world.meta["semantic_map_hash"]:
            raise CorridorFailure("SEMANTIC_BINDING_MISMATCH", "semantic map hash changed")
        route = OrientedRoute(
            world.meta["route_polyline"], world.start, world.goal,
            endpoint_attachment_limit_m=policy.endpoint_attachment_limit_m,
        )
        if config.get("performance_implementation", {}).get("graph_generation") == "lazy_guided":
            lazy_policy = _lazy_policy(config)
            factory = LazyEdgeFactory(
                world, route, policy,
                candidate_cell_provider=_provider(config),
                valid_successors_per_target_layer=lazy_policy.valid_successors_per_target_layer,
            )
            witness, evaluations, search = search_lazy(
                factory, world, semantic_map, lazy_policy,
            )
            graph_diagnostics = factory.diagnostics()
            graph_diagnostics["fast_dense_edge_certificate_count"] = int(
                world.fast_dense_edge_certificates
            )
            graph_diagnostics["dense_edge_fallback_count"] = int(world.dense_edge_fallbacks)
            no_candidates = not evaluations and search["remaining_heap_count"] == 0
        else:
            graph = build_graph(
                world, route, policy, candidate_cell_provider=_provider(config),
            )
            candidates, search = search_complete_paths(graph, policy)
            evaluations = []
            for rank, (label, labels) in enumerate(candidates):
                controls = _controls_for_label(label, labels, graph)
                evaluated = evaluate_controls(world, route, controls, semantic_map)
                lane = evaluated.get("semantics", {}).get("active_window", {}).get("classes", {}).get("lane", {})
                evaluations.append({
                    "rank": rank,
                    "profile": label.profile,
                    "score": label.score,
                    "path_length_m": label.length_m,
                    "correct_side_ratio": lane.get("correct_side_ratio"),
                    "target_band_ratio": lane.get("target_band_ratio"),
                    "lateral_error_p50_m": lane.get("lateral_error_p50_m"),
                    "failure_codes": evaluated["failure_codes"],
                    "gate_passed": evaluated["gate_passed"],
                })
                if evaluated["gate_passed"]:
                    witness = evaluated
                    break
            graph_diagnostics = graph.diagnostics
            no_candidates = not candidates
        result = {
            "schema_version": SCHEMA_VERSION,
            "architecture_id": ARCHITECTURE_ID,
            "implementation_revision": IMPLEMENTATION_REVISION,
            "protocol_id": PROTOCOL_ID,
            "query_id": query_id,
            "gate_passed": witness is not None,
            "failure_code": "" if witness is not None else (
                "NO_DIRECTED_SE2_ROUTE" if no_candidates else "DIRECTED_SE2_GRAPH_NO_STRICT_WITNESS"
            ),
            "graph": graph_diagnostics,
            "search": search,
            "candidate_evaluations": evaluations,
            "route": {
                "route_hash": route.route_hash,
                "route_length_m": route.length_m,
                "start_attachment_distance_m": route.start_attachment_distance_m,
                "goal_attachment_distance_m": route.goal_attachment_distance_m,
            },
            "input_npz_sha256": world.meta["npz_sha256"],
            "map_hash": world.meta["map_hash"],
            "semantic_map_hash": world.meta["semantic_map_hash"],
            "used_historical_path_or_witness": False,
            "online": False,
            "proof_scope": "bounded deterministic ordered graph; not continuous-space complete",
        }
    except CorridorFailure as error:
        result = {
            "schema_version": SCHEMA_VERSION,
            "architecture_id": ARCHITECTURE_ID,
            "implementation_revision": IMPLEMENTATION_REVISION,
            "protocol_id": PROTOCOL_ID,
            "query_id": query_id,
            "gate_passed": False,
            "failure_code": error.code,
            "failure_detail": error.detail,
            "used_historical_path_or_witness": False,
            "online": False,
            "proof_scope": "fail-closed request-derived offline preflight",
        }
    result.update({
        "wall_s": time.monotonic() - started,
        "cpu_s": time.process_time() - cpu_started,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        "python": platform.python_version(),
        "source_hashes_at_start": source_hashes,
        "processes_after": applicability._process_audit(),
    })
    if witness is not None:
        _write_json(output / "path.json", witness["points"])
        _write_json(output / "controls.json", {"edges": witness["controls"]})
        audit = dict(witness)
        audit.pop("points")
        audit.pop("controls")
        _write_json(output / "witness_audit.json", audit)
    _write_json(output / "result.json", result)
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    for source in sources:
        shutil.copy2(source, snapshot / source.name)
    end_hashes = {str(path): sha256_file(path) for path in sources}
    if end_hashes != source_hashes:
        raise RuntimeError("source changed during planning")
    _write_json(output / "artifact_hashes.json", applicability._manifest_files(output))
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    prepare_parser = subparsers.add_parser("prepare")
    prepare_parser.add_argument("--output", type=Path, required=True)
    prepare_parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    prepare_parser.add_argument("--targeted", type=Path, default=DEFAULT_TARGETED)
    plan_parser = subparsers.add_parser("plan")
    plan_parser.add_argument("--inputs", type=Path, required=True)
    plan_parser.add_argument("--query", required=True)
    plan_parser.add_argument("--output", type=Path, required=True)
    plan_parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    plan_parser.add_argument("--targeted", type=Path, default=DEFAULT_TARGETED)
    args = parser.parse_args(argv)
    if args.command == "prepare":
        result = prepare(output=args.output, config_path=args.config, targeted_path=args.targeted)
    else:
        result = plan(
            inputs=args.inputs, query_id=args.query, output=args.output,
            config_path=args.config, targeted_path=args.targeted,
        )
    print(json.dumps({
        "command": args.command,
        "gate_passed": result.get("gate_passed"),
        "query_id": result.get("query_id"),
        "query_count": result.get("query_count"),
        "wall_s": result.get("wall_s"),
        "failure_code": result.get("failure_code", ""),
    }, indent=2, sort_keys=True))
    return 0 if args.command == "prepare" or result.get("gate_passed") else 2


if __name__ == "__main__":
    raise SystemExit(main())
