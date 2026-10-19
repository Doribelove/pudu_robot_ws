"""Online exact-ACK adapter for the independent PLN-02 2A-V3 planner.

Each request rebuilds its route, semantic field and directed SE(2) search from
the frozen map/query.  Nav2 publishes and serves the effective master costmap;
the state-lattice search starts only after byte-exact service readback and
publishes its fresh result as an echo-verified ``nav_msgs/Path``.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import json
import os
from pathlib import Path
import platform
import resource
import shlex
import shutil
import statistics
import time
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from . import semantic_applicability_v3 as applicability
from . import two_layer_v2_semantic_r1_benchmark as r1
from . import two_layer_v2_semantic_r3_benchmark as r3
from . import two_layer_v3_semantic_r0_benchmark as offline
from .planner_benchmark.models import Query
from .regional_preference_r1 import orient_route_for_query
from .regional_preference_r3 import RegionalPreferenceBuilderR3
from .semantic_constraint_core import ConstraintWorld
from .semantic_costmap_r2 import SemanticCostmapComposerR2
from .semantic_map import SemanticMapV1, sha256_file
from .semantic_transition_lazy_corridor import LazyEdgeFactory, search_lazy
from .semantic_transition_ordered_corridor import CorridorFailure, OrientedRoute
from .semantic_v3_online_session import VerifiedV3PlannerSession


ARCHITECTURE_ID = "2A-V3"
IMPLEMENTATION_REVISION = "r12-online-memory-request"
PROTOCOL_ID = "PLN-02-2A-V3-R12-ONLINE-MEMORY-REQUEST-V1"
SCHEMA_VERSION = "PLN-02-2A-V3-R12-ONLINE-RESULT-V1"
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ROOT = applicability.ROOT
DEFAULT_CONFIG = PACKAGE_ROOT / "config/two_layer_v3_semantic_r12_online_memory_request.yaml"


def _identity() -> dict[str, str]:
    return {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
    }


def _write_json(path: Path, payload: Any) -> None:
    applicability._write_json(path, payload)


def _resolve_bound(parent: Path, binding: Mapping[str, Any]) -> Path:
    path = Path(str(binding["path"]))
    if not path.is_absolute():
        path = parent / path
    path = path.resolve()
    if sha256_file(path) != str(binding["sha256"]):
        raise ValueError(f"frozen online binding changed: {path}")
    return path


def _load_online_config(path: Path) -> tuple[dict[str, Any], Path, Path, dict[str, Any], dict[str, Any]]:
    path = path.resolve()
    config = applicability._load_extended_mapping(path, "extends_online_config")
    for key, expected in _identity().items():
        if config.get(key) != expected:
            raise ValueError(f"2A-V3 online identity mismatch for {key}")
    predecessor = config.get("predecessor_online_config")
    if predecessor:
        _resolve_bound(path.parent, predecessor)
    for binding in config.get("offline_gate_evidence", {}).values():
        evidence_path = Path(str(binding["path"]))
        if not evidence_path.is_absolute():
            evidence_path = ROOT / evidence_path
        if sha256_file(evidence_path) != str(binding["sha256"]):
            raise ValueError(f"offline gate evidence changed: {evidence_path}")
        evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
        if evidence.get("gate_passed") is not True:
            raise ValueError(f"offline gate evidence did not pass: {evidence_path}")
    algorithm_path = _resolve_bound(path.parent, config["algorithm_config"])
    targeted_path = _resolve_bound(path.parent, config["targeted_query_set"])
    raw_algorithm = yaml.safe_load(algorithm_path.read_text(encoding="utf-8")) or {}
    algorithm_identity = {
        key: raw_algorithm[key]
        for key in ("architecture_id", "implementation_revision", "protocol_id")
    }
    algorithm, parent = offline._load_config(
        algorithm_path, expected_identity=algorithm_identity,
    )
    targeted = offline._load_targeted(
        targeted_path, algorithm, expected_identity=algorithm_identity,
    )
    motion = config["immutable_motion_contract"]
    required = {
        "yaw_bins": 48, "motion_model": "DUBIN", "allow_reverse": False,
        "allow_in_place_rotation": False, "minimum_turning_radius_m": 0.40,
        "maximum_curvature_1pm": 2.50,
    }
    for key, expected in required.items():
        if motion.get(key) != expected:
            raise ValueError(f"immutable online motion contract changed for {key}")
    return config, algorithm_path, targeted_path, algorithm, parent


def _run_search(
    *, input_dir: Path, query_id: str, verified_master: Any,
    algorithm: Mapping[str, Any], semantic_map_path: Path,
    meta: Mapping[str, Any] | None = None,
    arrays: Mapping[str, Any] | None = None,
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    policy = applicability._ordered_policy(algorithm)
    world = ConstraintWorld(
        input_dir, query_id, master_override=verified_master,
        expected_master_hash=None, meta_override=meta, arrays_override=arrays,
    )
    semantic_map = SemanticMapV1.load(semantic_map_path.resolve())
    if semantic_map.semantic_map_hash != world.meta["semantic_map_hash"]:
        raise CorridorFailure("SEMANTIC_BINDING_MISMATCH", "semantic map changed")
    route = OrientedRoute(
        world.meta["route_polyline"], world.start, world.goal,
        endpoint_attachment_limit_m=policy.endpoint_attachment_limit_m,
    )
    lazy_policy = offline._lazy_policy(algorithm)
    factory = LazyEdgeFactory(
        world, route, policy,
        candidate_cell_provider=offline._provider(algorithm),
        valid_successors_per_target_layer=lazy_policy.valid_successors_per_target_layer,
    )
    witness, evaluations, search = search_lazy(factory, world, semantic_map, lazy_policy)
    graph = factory.diagnostics()
    graph["fast_dense_edge_certificate_count"] = int(world.fast_dense_edge_certificates)
    graph["dense_edge_fallback_count"] = int(world.dense_edge_fallbacks)
    return {
        "gate_passed": witness is not None,
        "failure_code": "" if witness is not None else (
            "NO_DIRECTED_SE2_ROUTE"
            if not evaluations and search["remaining_heap_count"] == 0
            else "DIRECTED_SE2_GRAPH_NO_STRICT_WITNESS"
        ),
        "graph": graph,
        "search": search,
        "candidate_evaluations": evaluations,
        "route": {
            "route_hash": route.route_hash,
            "route_length_m": route.length_m,
            "start_attachment_distance_m": route.start_attachment_distance_m,
            "goal_attachment_distance_m": route.goal_attachment_distance_m,
        },
    }, witness


def _metric_row(query: Query, result: Mapping[str, Any], witness: Mapping[str, Any] | None) -> dict[str, Any]:
    if witness is None:
        return {
            "query_id": query.query_id, "category": query.category,
            "final_valid_success": False, "failure_code": result.get("failure_code", "PLANNING_FAILED"),
        }
    lane = witness["semantics"]["active_window"]["classes"]["lane"]
    canonical = witness["safety"]["canonical"]
    hard = witness["hard_features"]
    return {
        "query_id": query.query_id, "category": query.category,
        "final_valid_success": bool(witness["gate_passed"]), "failure_code": "",
        "correct_side_ratio": lane.get("correct_side_ratio"),
        "target_band_ratio": lane.get("target_band_ratio"),
        "lateral_error_p50_m": lane.get("lateral_error_p50_m"),
        "path_length_m": canonical.get("path_length_m"),
        "maximum_curvature_1pm": canonical.get("maximum_curvature"),
        "collision_violations": 0 if canonical.get("static_footprint_valid") else 1,
        "kinematic_violations": 0 if canonical.get("kinematic_valid") else 1,
        "hard_semantic_violations": 0 if hard.get("hard_feature_gate_passed") else 1,
        "no_stopping_goal_violations": hard.get("no_stopping_task_endpoint_violations"),
        "reverse_distance_m": canonical.get("reverse_distance_m"),
        "rotate_in_place_count": canonical.get("in_place_rotation_count"),
        "ordered_progress_gate_passed": witness["ordered_progress"].get("ordered_progress_gate_passed"),
        "revisit_screen_passed": witness["revisit"].get("revisit_screen_passed"),
        "naturalness_audit_passed": witness["naturalness"].get("audit_passed"),
    }


def run(
    *, output: Path, config_path: Path = DEFAULT_CONFIG,
    extracted: Path = applicability.DEFAULT_EXTRACTED,
    semantic_map_path: Path = applicability.DEFAULT_SEMANTIC_MAP,
    topology_cache: Path = applicability.DEFAULT_TOPOLOGY,
    ros_domain_id: int | None = None,
) -> dict[str, Any]:
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / "logs").mkdir()
    (output / "requests").mkdir()
    started = time.monotonic()
    config, algorithm_path, targeted_path, algorithm, parent_config = _load_online_config(config_path)
    raw_algorithm = yaml.safe_load(algorithm_path.read_text(encoding="utf-8")) or {}
    targeted = offline._load_targeted(
        targeted_path, algorithm,
        expected_identity={
            key: raw_algorithm[key]
            for key in ("architecture_id", "implementation_revision", "protocol_id")
        },
    )
    chosen_domain = int(
        config["online_interface"]["ros_domain_id"] if ros_domain_id is None else ros_domain_id
    )
    os.environ["ROS_DOMAIN_ID"] = str(chosen_domain)
    sources = [
        Path(__file__).resolve(), Path(VerifiedV3PlannerSession.__module__.replace(".", "/")),
        Path(applicability.__file__).resolve(), Path(offline.__file__).resolve(),
        Path(ConstraintWorld.__module__.replace(".", "/")), algorithm_path,
        targeted_path, config_path.resolve(), semantic_map_path.resolve(),
    ]
    # Resolve module placeholders without importing through the installed tree.
    sources[1] = Path(__file__).with_name("semantic_v3_online_session.py").resolve()
    sources[4] = Path(__file__).with_name("semantic_constraint_core.py").resolve()
    sources.extend([
        Path(__file__).with_name("semantic_transition_lazy_corridor.py").resolve(),
        Path(__file__).with_name("semantic_transition_ordered_corridor.py").resolve(),
    ])
    source_hashes = {str(path): sha256_file(path) for path in sources}
    reproduction = (
        f"ROS_DOMAIN_ID={chosen_domain} /usr/bin/python3 -m "
        "arena_evaluation.two_layer_v3_semantic_online "
        f"--output {shlex.quote(str(output))} --config {shlex.quote(str(config_path.resolve()))}"
    )
    (output / "reproduction_command.txt").write_text(reproduction + "\n", encoding="utf-8")
    _write_json(output / "protocol.json", {
        "schema_version": SCHEMA_VERSION, **_identity(),
        "query_hash": targeted["query_hash"], "query_count": len(targeted["queries"]),
        "ros_domain_id": chosen_domain, "configuration": config,
        "algorithm_configuration": algorithm,
        "used_historical_path_or_witness": False,
        "planner_result_source": "fresh_in_process_directed_se2_search",
        "effective_master_source": "Nav2 GetCostmap after exact content ACK",
        "source_hashes_at_start": source_hashes,
        "processes_before": applicability._process_audit(),
    })

    static_started = time.monotonic()
    with r3._r3_bindings():
        prepared = r1._prepare(
            extracted.resolve(), semantic_map_path.resolve(), topology_cache.resolve(),
            parent_config, output=None,
        )
    static_prepare_ms = (time.monotonic() - static_started) * 1000.0
    ctx, semantic_map, raster, topology, _annotator, router = prepared[:6]
    if ctx.map_sha256 != targeted["map_hash"]:
        raise ValueError("online map hash mismatch")
    if semantic_map.semantic_map_hash != targeted["semantic_map_hash"]:
        raise ValueError("online semantic map hash mismatch")
    selector = r1._semantic_selector(topology, router)
    builder = RegionalPreferenceBuilderR3(
        ctx.hospital_map, raster, policy=parent_config["regional_preference"],
        semantic_map=semantic_map,
    )
    composer = SemanticCostmapComposerR2(
        policy=parent_config["l3_soft_cost"], inflation_cache_capacity=2,
    )
    session = VerifiedV3PlannerSession(
        ctx, output, map_yaml=ctx.map_yaml,
        log_tag=f"2a_v3_r12_{int(time.time())}", local_mask_updates=True,
        optimization_profile="v7_candidate", smac_parameter_profile="baseline",
        optimization_stage="step3_delta_map", enable_mask_reuse_noop=True,
        costmap_ack_timeout_s=float(config["online_interface"]["exact_ack_timeout_s"]),
    )
    session.local_map_update_strategy = "roi_ack"
    results: list[dict[str, Any]] = []
    session_started = time.monotonic()
    session.start()
    session_start_ms = (time.monotonic() - session_started) * 1000.0
    try:
        for raw in targeted["queries"]:
            query_started = time.monotonic()
            query = Query(
                query_id=str(raw["query_id"]),
                start=[float(value) for value in raw["start"]],
                goal=[float(value) for value in raw["goal"]],
                category=str(raw["category"]),
                seed=int(raw.get("seed", targeted["seed"])),
            )
            request_dir = output / "requests" / query.query_id
            row: dict[str, Any] = {
                "architecture_id": ARCHITECTURE_ID,
                "implementation_revision": IMPLEMENTATION_REVISION,
                "protocol_id": PROTOCOL_ID,
                "arm": "E5-2A-V3", "relaxation_level": "R0",
                "used_historical_path_or_witness": False,
            }
            witness = None
            meta = None
            sink: dict[str, Any] = {}
            try:
                reset_started = time.monotonic()
                reset = session.reset_query_state(query.query_id, restore_base_map=False)
                reset_ms = (time.monotonic() - reset_started) * 1000.0
                l1_started = time.monotonic()
                _sn, _gn, route, reason = selector(
                    topology, query, cache_mode=r1.r2_runtime.CACHE_MODE_OPTIMIZED,
                    timing={},
                )
                if route is None:
                    raise CorridorFailure("L1_ROUTE_FAILED", str(reason))
                route, orientation = orient_route_for_query(route, query)
                l1_ms = (time.monotonic() - l1_started) * 1000.0
                meta = applicability._prepare_input(
                    directory=request_dir, query=query, route=route,
                    orientation=orientation, ctx=ctx, semantic_map=semantic_map,
                    raster=raster, topology=topology, builder=builder,
                    composer=composer, parent_config=parent_config,
                    targeted_hash=targeted["query_hash"], identity=_identity(),
                    artifact_sink=sink,
                    write_artifacts=False,
                )
                session.set_semantic_costmap(sink["composition"])
                publish_started = time.monotonic()
                ack = session.update_local_mask(sink["allowed"])
                publication_ack_ms = (time.monotonic() - publish_started) * 1000.0
                readback_started = time.monotonic()
                verified_master, verified = session.verified_master_snapshot(ack)
                readback_ms = (time.monotonic() - readback_started) * 1000.0
                search_started = time.monotonic()
                search_result, witness = _run_search(
                    input_dir=request_dir, query_id=query.query_id,
                    verified_master=verified_master, algorithm=algorithm,
                    semantic_map_path=semantic_map_path,
                    meta=meta, arrays=sink["arrays"],
                )
                search_audit_ms = (time.monotonic() - search_started) * 1000.0
                publish_path_ms = 0.0
                path_interface = None
                if witness is not None:
                    path_started = time.monotonic()
                    path_interface = session.publish_verified_path(
                        witness["points"], query_id=query.query_id,
                    )
                    publish_path_ms = (time.monotonic() - path_started) * 1000.0
                    _write_json(request_dir / "path.json", witness["points"])
                    _write_json(request_dir / "controls.json", {"edges": witness["controls"]})
                    audit = dict(witness)
                    audit.pop("points")
                    audit.pop("controls")
                    _write_json(request_dir / "witness_audit.json", audit)
                row.update(_metric_row(query, search_result, witness))
                row.update({
                    "route_hash": meta["route_hash"], "roi_hash": meta["roi_hash"],
                    "expected_master_hash": meta["expected_master_hash"],
                    "server_master_hash": verified["verified_master_hash"],
                    "costmap_ack": ack, "verified_master": verified,
                    "path_interface": path_interface,
                    "graph": search_result["graph"], "search": search_result["search"],
                    "candidate_evaluations": search_result["candidate_evaluations"],
                    "timing": {
                        "reset_ms": reset_ms, "l1_ms": l1_ms,
                        **sink["timing"],
                        "publication_and_ack_ms": publication_ack_ms,
                        "verified_readback_ms": readback_ms,
                        "se2_search_and_audit_ms": search_audit_ms,
                        "path_publication_echo_ms": publish_path_ms,
                    },
                    "session_reset": reset,
                })
            except (CorridorFailure, RuntimeError, ValueError) as error:
                row.update(_metric_row(query, {"failure_code": getattr(error, "code", type(error).__name__)}, None))
                row["failure_detail"] = str(error)
            row["request_wall_ms"] = (time.monotonic() - query_started) * 1000.0
            evidence_started = time.monotonic()
            if meta is not None and sink.get("arrays"):
                arrays = sink["arrays"]
                archive = request_dir / f"{query.query_id}.npz"
                np.savez_compressed(archive, **arrays)
                meta = dict(meta)
                meta["npz_sha256"] = sha256_file(archive)
                meta["timing"] = dict(meta.get("timing", {}))
                meta["timing"]["input_serialization_ms"] = 0.0
                meta["timing"]["evidence_serialization_outside_request_ms"] = (
                    (time.monotonic() - evidence_started) * 1000.0
                )
                _write_json(request_dir / f"{query.query_id}.json", meta)
            row["evidence_serialization_outside_request_ms"] = (
                (time.monotonic() - evidence_started) * 1000.0
            )
            row["peak_rss_bytes"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
            results.append(row)
            _write_json(output / f"result_{query.query_id}.json", row)
    finally:
        session.close()

    flat_rows = []
    for row in results:
        timing = row.get("timing", {})
        ack = row.get("costmap_ack", {})
        flat_rows.append({
            key: row.get(key) for key in (
                "architecture_id", "implementation_revision", "protocol_id", "arm",
                "query_id", "category", "relaxation_level", "final_valid_success",
                "failure_code", "correct_side_ratio", "target_band_ratio",
                "lateral_error_p50_m", "path_length_m", "maximum_curvature_1pm",
                "collision_violations", "kinematic_violations", "hard_semantic_violations",
                "no_stopping_goal_violations", "reverse_distance_m", "rotate_in_place_count",
                "request_wall_ms", "peak_rss_bytes",
                "evidence_serialization_outside_request_ms",
            )
        } | {
            "field_build_ms": timing.get("field_build_ms"),
            "compose_ms": timing.get("compose_ms"),
            "publication_and_ack_ms": timing.get("publication_and_ack_ms"),
            "se2_search_and_audit_ms": timing.get("se2_search_and_audit_ms"),
            "ack_hard_mismatch": ack.get("costmap_ack_hard_mismatch_cells"),
            "ack_soft_exact_mismatch": ack.get("costmap_ack_soft_exact_mismatch_cells"),
            "ack_stale_cells": ack.get("costmap_ack_stale_roi_cells"),
            "ack_hash_mismatch": ack.get("costmap_ack_hash_mismatch"),
            "ack_sequence_mismatch": ack.get("costmap_ack_sequence_mismatch"),
            "path_echo_verified": (row.get("path_interface") or {}).get("exact_echo_verified"),
        })
    with (output / "runs.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat_rows[0]))
        writer.writeheader()
        writer.writerows(flat_rows)

    ack_summary = {
        "query_count": len(results),
        "acknowledged_count": sum(
            row.get("costmap_ack", {}).get("costmap_update_acknowledged") is True
            for row in results
        ),
        "hard_exact_mismatch_cells": sum(int(row.get("costmap_ack", {}).get("costmap_ack_hard_mismatch_cells", 0)) for row in results),
        "soft_exact_mismatch_cells": sum(int(row.get("costmap_ack", {}).get("costmap_ack_soft_exact_mismatch_cells", 0)) for row in results),
        "stale_roi_cells": sum(int(row.get("costmap_ack", {}).get("costmap_ack_stale_roi_cells", 0)) for row in results),
        "hash_mismatch_count": sum(int(row.get("costmap_ack", {}).get("costmap_ack_hash_mismatch", 0)) for row in results),
        "sequence_mismatch_count": sum(int(row.get("costmap_ack", {}).get("costmap_ack_sequence_mismatch", 0)) for row in results),
    }
    ack_summary["gate_passed"] = bool(
        ack_summary["acknowledged_count"] == len(results)
        and not any(ack_summary[key] for key in (
            "hard_exact_mismatch_cells", "soft_exact_mismatch_cells", "stale_roi_cells",
            "hash_mismatch_count", "sequence_mismatch_count",
        ))
    )
    _write_json(output / "exact_ack_summary.json", ack_summary)
    request_times = [float(row["request_wall_ms"]) for row in results]
    performance = {
        "static_prepare_ms": static_prepare_ms,
        "nav2_session_start_ms": session_start_ms,
        "cold_request_p50_ms": statistics.median(request_times),
        "cold_request_samples_ms": request_times,
        "comparison_to_same_round_e0": "PENDING",
    }
    _write_json(output / "performance_summary.json", performance)
    correctness = bool(
        len(results) == len(targeted["queries"])
        and all(row.get("final_valid_success") is True for row in results)
        and all((row.get("path_interface") or {}).get("exact_echo_verified") is True for row in results)
    )
    gates = {
        "offline_parent_triad": "PASSED_R7",
        "online_targeted_correctness_and_safety": correctness,
        "exact_effective_content_ack": ack_summary["gate_passed"],
        "same_round_e0_e4_v3_latency": "PENDING",
        "targeted_gate_passed": False,
        "selected8_status": "NOT_RUN_SAME_ROUND_COMPARISON_PENDING",
        "grade": "C_PENDING_REQUIRED_EVIDENCE",
    }
    _write_json(output / "gate_results.json", gates)
    end_hashes = {str(path): sha256_file(path) for path in sources}
    if end_hashes != source_hashes:
        raise RuntimeError("source changed during online targeted run")
    final = {
        "schema_version": SCHEMA_VERSION, **_identity(),
        "query_hash": targeted["query_hash"], "rows": results,
        "static_prepare_ms": static_prepare_ms, "session_start_ms": session_start_ms,
        "wall_s": time.monotonic() - started,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        "python": platform.python_version(), "exact_ack_summary": ack_summary,
        "gate_results": gates, "source_hashes_at_end": end_hashes,
        "processes_after": applicability._process_audit(),
    }
    _write_json(output / "final_result.json", final)
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    for source in sources:
        shutil.copy2(source, snapshot / source.name)
    _write_json(output / "artifact_hashes.json", applicability._manifest_files(output))
    return final


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--extracted", type=Path, default=applicability.DEFAULT_EXTRACTED)
    parser.add_argument("--semantic-map", type=Path, default=applicability.DEFAULT_SEMANTIC_MAP)
    parser.add_argument("--topology-cache", type=Path, default=applicability.DEFAULT_TOPOLOGY)
    parser.add_argument("--ros-domain-id", type=int)
    args = parser.parse_args(argv)
    result = run(
        output=args.output, config_path=args.config, extracted=args.extracted,
        semantic_map_path=args.semantic_map, topology_cache=args.topology_cache,
        ros_domain_id=args.ros_domain_id,
    )
    print(json.dumps({
        "architecture_id": result["architecture_id"],
        "implementation_revision": result["implementation_revision"],
        "online_correctness": result["gate_results"]["online_targeted_correctness_and_safety"],
        "exact_ack": result["gate_results"]["exact_effective_content_ack"],
        "grade": result["gate_results"]["grade"],
        "wall_s": result["wall_s"],
    }, indent=2, sort_keys=True))
    return 0 if (
        result["gate_results"]["online_targeted_correctness_and_safety"]
        and result["gate_results"]["exact_effective_content_ack"]
    ) else 2


if __name__ == "__main__":
    raise SystemExit(main())
