"""Hard-stop fast iteration gates for 2A-V3 r15.

The focused stages deliberately exclude Smac path search from ACK stress and
exclude ROS from the parking oracle.  Full selected8/expanded32 evaluation is
permitted only after both focused gates pass.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import platform
import resource
import shutil
import statistics
import subprocess
import time
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from . import semantic_applicability_v3 as applicability
from . import semantic_static_cache_v3 as static_cache
from . import two_layer_v2_semantic_r1_benchmark as r1
from . import two_layer_v2_semantic_r3_benchmark as r3
from . import two_layer_v3_semantic_r13_benchmark as r13
from . import two_layer_v3_semantic_r14_benchmark as r14
from . import two_layer_v3_semantic_r14_expanded as expanded
from .path_audit import PathAuditor
from .regional_preference_r1 import orient_route_for_query
from .semantic_costmap_r2 import SemanticCostmapComposerR2
from .semantic_map import sha256_file
from .semantic_parking_reference_r15 import LexicographicParkingReferenceBuilderR15
from .semantic_query_defaults import load_query_set
from .semantic_route_phase_compact_r14 import CompactRoutePhaseWorldR14, prepare_query_input_compact
from .semantic_route_phase_v3 import LazyRoutePhaseSearch, OrientedRoute
from .semantic_v3_ack_r15 import SingleObservationExactAckSessionR15


ARCHITECTURE_ID = "2A-V3"
IMPLEMENTATION_REVISION = "r15-fast-iteration-single-read-ack"
PROTOCOL_ID = "PLN-02-2A-V3-R15-FAST-ITERATION-V1"
SCHEMA_VERSION = "PLN-02-2A-V3-R15-FAST-RESULT-V1"
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE_ROOT / "config/two_layer_v3_semantic_r15_fast.yaml"


def identity() -> dict[str, str]:
    return {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
    }


def percentile(values: Sequence[float], quantile: float) -> float | None:
    return float(np.percentile(np.asarray(values, dtype=np.float64), quantile)) if values else None


def current_rss_bytes() -> int:
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) * 1024
    except OSError:
        pass
    return 0


def _load(path: Path = DEFAULT_CONFIG) -> tuple[
    dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any], Path, Path, Path,
]:
    config_path = Path(path).resolve()
    config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    for key, value in identity().items():
        if config.get(key) != value:
            raise ValueError(f"r15 identity mismatch for {key}")
    parent_algorithm_path = config_path.parent / str(config["parent_r14_algorithm"]["path"])
    parent_expanded_path = config_path.parent / str(config["parent_r14_expanded"]["path"])
    parent_algorithm_path = parent_algorithm_path.resolve()
    parent_expanded_path = parent_expanded_path.resolve()
    if sha256_file(parent_algorithm_path) != str(config["parent_r14_algorithm"]["sha256"]):
        raise ValueError("r15 parent r14 algorithm hash mismatch")
    if sha256_file(parent_expanded_path) != str(config["parent_r14_expanded"]["sha256"]):
        raise ValueError("r15 parent r14 expanded hash mismatch")
    expanded_config, algorithm, parent_algorithm, parent, _, query_path = expanded._load(
        parent_expanded_path,
    )
    if Path(query_path).name != str(config["frozen_bindings"]["expanded32_path"]):
        raise ValueError("r15 expanded32 path drift")
    frozen = config["frozen_bindings"]
    bindings = {
        "expanded32_file_sha256": sha256_file(query_path),
        "expanded32_query_hash": expanded_config["query_set"]["query_hash"],
        "map_hash": algorithm["frozen_bindings"]["map_hash"],
        "semantic_map_hash": algorithm["frozen_bindings"]["semantic_map_hash"],
    }
    for key, actual in bindings.items():
        if str(frozen[key]) != str(actual):
            raise ValueError(f"r15 frozen binding drift for {key}")
    selected8 = (config_path.parent / str(frozen["selected8_path"])).resolve()
    if sha256_file(selected8) != str(frozen["selected8_file_sha256"]):
        raise ValueError("r15 selected8 file hash mismatch")
    return config, algorithm, parent_algorithm, parent, selected8, query_path, config_path


def _git_capture(worktree: Path) -> dict[str, Any]:
    def command(*args: str) -> str:
        result = subprocess.run(
            list(args), cwd=worktree, check=False, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
        return result.stdout
    return {
        "root": str(worktree),
        "head": command("git", "rev-parse", "HEAD").strip(),
        "branch": command("git", "branch", "--show-current").strip(),
        "status_porcelain_v1": command("git", "status", "--porcelain=v1"),
    }


def _write_common(
    output: Path, *, stage: str, config: Mapping[str, Any], sources: Sequence[Path],
    details: Mapping[str, Any], reproduction: str,
) -> dict[str, str]:
    source_hashes = {str(path): sha256_file(path) for path in sources}
    root = PACKAGE_ROOT.parents[5]
    nested = PACKAGE_ROOT.parent
    applicability._write_json(output / "protocol.json", {
        **identity(), "schema_version": SCHEMA_VERSION, "stage": stage,
        "configuration": config, "details": dict(details),
        "source_hashes_at_start": source_hashes,
        "root_git": _git_capture(root), "evaluation_git": _git_capture(nested),
        "processes_before": applicability._process_audit(),
        "python": platform.python_version(), "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
    })
    (output / "reproduction_command.txt").write_text(reproduction.rstrip() + "\n", encoding="utf-8")
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    for index, source in enumerate(sources):
        shutil.copy2(source, snapshot / f"{index:02d}_{source.name}")
    return source_hashes


def _prepare_static(
    *, algorithm: Mapping[str, Any], parent: Mapping[str, Any], output: Path,
) -> tuple[tuple[Any, ...], float]:
    started = time.monotonic()
    with r3._r3_bindings():
        prepared = static_cache.prepare_static_cached(
            applicability.DEFAULT_EXTRACTED, applicability.DEFAULT_SEMANTIC_MAP,
            applicability.DEFAULT_TOPOLOGY, parent, output=None,
            cache_root=Path(algorithm["static_cache"]["root"]),
            maximum_disk_entries=int(algorithm["static_cache"]["maximum_disk_entries"]),
        )
    return prepared, (time.monotonic() - started) * 1000.0


def _selector(parent: Mapping[str, Any], topology: Any, router: Any) -> Any:
    switches = r1.ArmSwitches.parse(parent["ablation_arms"]["E4"])
    return r1._selector_for_arm(
        switches, topology, router, {},
        preferred_attachment_radius_m=float(parent["endpoint_attachment"]["preferred_radius_m"]),
        attachment_cost_weight=float(parent["endpoint_attachment"]["cost_weight"]),
    )


def _route(selector: Any, topology: Any, query: Any) -> tuple[Any, dict[str, Any]]:
    _start, _goal, route, reason = selector(
        topology, query, cache_mode=r1.r2_runtime.CACHE_MODE_OPTIMIZED, timing={},
    )
    if route is None:
        raise RuntimeError(f"L1_ROUTE_FAILED:{reason}")
    return orient_route_for_query(route, query)


def _prepare_query_state(
    *, query: Any, query_hash: str, selector: Any, ctx: Any, semantic_map: Any,
    raster: Any, topology: Any, route_phase_algorithm: Mapping[str, Any],
    parent: Mapping[str, Any],
    builder: Any, composer: Any,
) -> tuple[Any, Any, Any, np.ndarray, np.ndarray, dict[str, Any], dict[str, np.ndarray]]:
    route, orientation = _route(selector, topology, query)
    metadata, arrays, composition, publication_allowed, full_allowed = prepare_query_input_compact(
        query=query, route=route, orientation=orientation, ctx=ctx,
        semantic_map=semantic_map, raster=raster, topology=topology,
        builder=builder, composer=composer, parent=parent,
        query_set_hash=query_hash,
        maximum_lateral_probe_m=float(
            r13._policy(route_phase_algorithm).maximum_lateral_probe_m
        ),
    )
    return route, orientation, composition, publication_allowed, full_allowed, metadata, arrays


def run_parking_oracle(*, output: Path, config_path: Path = DEFAULT_CONFIG) -> dict[str, Any]:
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / "witnesses").mkdir()
    run_started = time.monotonic()
    config, algorithm, parent_algorithm, parent, _selected8, query_path, resolved_config = _load(config_path)
    stage_config = config["fast_iteration"]["parking_oracle"]
    wanted = list(stage_config["query_ids"])
    queries, _intents, query_meta = load_query_set(
        query_path, actual_map_hash=config["frozen_bindings"]["map_hash"],
        actual_semantic_map_hash=config["frozen_bindings"]["semantic_map_hash"],
    )
    by_id = {query.query_id: query for query in queries}
    if any(query_id not in by_id for query_id in wanted):
        raise ValueError("r15 parking oracle contains unknown frozen query")
    sources = [
        Path(__file__).resolve(), resolved_config,
        Path(__file__).with_name("semantic_parking_reference_r15.py"),
        Path(__file__).with_name("semantic_route_phase_v3.py"),
        Path(__file__).with_name("semantic_route_phase_compact_r14.py"), query_path,
    ]
    _write_common(
        output, stage="parking-oracle", config=config, sources=sources,
        details={"query_ids": wanted, "query_hash": query_meta["query_hash"], "online": False},
        reproduction=(
            f"/usr/bin/python3 -m arena_evaluation.two_layer_v3_semantic_r15_fast "
            f"--stage parking-oracle --output {output} --config {resolved_config}"
        ),
    )
    prepared, static_prepare_ms = _prepare_static(algorithm=algorithm, parent=parent, output=output)
    ctx, semantic_map, raster, topology, _annotator, router = prepared[:6]
    builder = expanded.AuditOnlyPreferenceBuilderR14(
        ctx.hospital_map, raster, policy=parent["regional_preference"], semantic_map=semantic_map,
    )
    composer = SemanticCostmapComposerR2(policy=parent["l3_soft_cost"], inflation_cache_capacity=2)
    selector = _selector(parent, topology, router)
    canonical_auditor = PathAuditor(ctx, source_commit="2A-V3-r15-parking-oracle")
    reference_builder = LexicographicParkingReferenceBuilderR15(r14._parking_policy(algorithm))
    rows: list[dict[str, Any]] = []
    for query_id in wanted:
        started = time.monotonic()
        query = by_id[query_id]
        row: dict[str, Any] = {**identity(), "query_id": query_id, "category": query.category}
        try:
            route, orientation, _composition, publication_allowed, full_allowed, metadata, arrays = _prepare_query_state(
                query=query, query_hash=query_meta["query_hash"], selector=selector,
                ctx=ctx, semantic_map=semantic_map, raster=raster, topology=topology,
                route_phase_algorithm=parent_algorithm, parent=parent,
                builder=builder, composer=composer,
            )
            world = CompactRoutePhaseWorldR14(
                output, query_id, arrays_override=arrays, meta_override=metadata,
                full_occupancy=ctx.hospital_map.occupancy, full_allowed=full_allowed,
            )
            world._canonical_auditor = canonical_auditor
            original_route = OrientedRoute(
                metadata["route_polyline"], world.start, world.goal,
                endpoint_attachment_limit_m=r13._policy(parent_algorithm).endpoint_attachment_limit_m,
            )
            reference = reference_builder.build(world, original_route)
            row["reference"] = reference.to_dict()
            witness = None
            search = {}
            evaluations: list[dict[str, Any]] = []
            if reference.gate_passed:
                guide = OrientedRoute(
                    reference.polyline, world.start, world.goal,
                    endpoint_attachment_limit_m=r13._policy(parent_algorithm).endpoint_attachment_limit_m,
                )
                searcher = LazyRoutePhaseSearch(world, guide, r13._policy(parent_algorithm))
                witness, evaluations, search = searcher.search(
                    semantic_map, allow_safe_soft_fallback=False,
                    prefer_lazy=True, fallback_first=False,
                )
            row["search"] = search
            row["candidate_evaluation_count"] = len(evaluations)
            failure_counts: dict[str, int] = {}
            for candidate in evaluations:
                for code in candidate.get("failure_codes") or candidate.get("raw_failure_codes") or []:
                    failure_counts[str(code)] = failure_counts.get(str(code), 0) + 1
            candidate_parking = [
                candidate.get("parking") or {} for candidate in evaluations
            ]
            row["candidate_summary"] = {
                "failure_code_counts": failure_counts,
                "center_band_ratio_max": max((
                    float(value.get("parking_center_band_ratio", 0.0) or 0.0)
                    for value in candidate_parking
                ), default=None),
                "normalized_deviation_p50_min": min((
                    float(value["parking_center_normalized_deviation_p50"])
                    for value in candidate_parking
                    if value.get("parking_center_normalized_deviation_p50") is not None
                ), default=None),
                "raw_hard_failure_candidate_count": sum(
                    any(code != "R2_SEMANTIC_GATE" for code in candidate.get("raw_failure_codes", []))
                    for candidate in evaluations
                ),
            }
            applicability._write_json(
                output / f"{query_id}_candidate_evaluations.json",
                expanded._json_safe(evaluations),
            )
            if witness is None:
                row.update({
                    "gate_passed": False,
                    "failure_code": reference.failure_code or "NO_STRICT_SE2_WITNESS",
                    "semantic_success_counted": False,
                })
            else:
                summary = expanded.r13_online._row_from_witness(query, witness)
                parking = witness["semantics"]["active_window"]["classes"]["parking"]
                hard = witness.get("hard_features", {})
                canonical = witness.get("canonical", witness.get("safety", {}).get("canonical", {}))
                gates = {
                    "semantic_success": witness.get("semantic_success_counted") is True,
                    "parking_center_band": float(parking.get("parking_center_band_ratio", 0.0) or 0.0)
                    > float(stage_config["parking_center_band_ratio_min_exclusive"]),
                    "parking_deviation": float(
                        parking.get("parking_center_normalized_deviation_p50", np.inf) or np.inf
                    ) <= float(stage_config["parking_normalized_deviation_p50_max"]),
                    "curvature": float(witness.get("maximum_control_curvature_1pm", np.inf))
                    <= float(stage_config["maximum_curvature_1pm"]),
                    "static_footprint": canonical.get("static_footprint_valid") is True,
                    "kinematic": canonical.get("kinematic_valid") is True,
                    "hard_semantic": hard.get("hard_feature_gate_passed") is True,
                    "reverse": float(canonical.get("reverse_distance_m", np.inf)) == 0.0,
                    "in_place": int(canonical.get("in_place_rotation_count", -1)) == 0,
                    "trace_exact": witness.get("trace_replay_exact") is True,
                }
                row.update(summary)
                row.update({
                    "gates": gates, "gate_passed": all(gates.values()),
                    "failure_code": "" if all(gates.values()) else "PARKING_WITNESS_GATE_FAILED",
                })
                points = witness.pop("points")
                controls = witness.pop("controls")
                applicability._write_json(output / "witnesses" / f"{query_id}_path.json", points)
                applicability._write_json(output / "witnesses" / f"{query_id}_controls.json", {"edges": controls})
                applicability._write_json(output / "witnesses" / f"{query_id}_witness.json", expanded._json_safe(witness))
        except (RuntimeError, ValueError) as error:
            row.update({
                "gate_passed": False, "failure_code": type(error).__name__,
                "failure_detail": str(error), "semantic_success_counted": False,
            })
        row["wall_ms"] = (time.monotonic() - started) * 1000.0
        row["peak_rss_bytes"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
        row = expanded._json_safe(row)
        rows.append(row)
        applicability._write_json(output / f"result_{query_id}.json", row)
        if config["fast_iteration"]["early_stop_on_first_hard_failure"] and row["gate_passed"] is not True:
            break
    all_three = len(rows) == len(wanted) and all(row["gate_passed"] is True for row in rows)
    gate = {
        "parking_oracle_passed": all_three,
        "required_query_count": len(wanted), "executed_query_count": len(rows),
        "hard_stop_triggered": not all_three,
        "next_stage": "ACK_STRESS_AND_SENTINEL8_ALLOWED" if all_three else "STOP_BEFORE_ONLINE_PATH_EXPERIMENTS",
    }
    flat = [{
        "query_id": row["query_id"], "category": row["category"],
        "gate_passed": row.get("gate_passed"), "failure_code": row.get("failure_code"),
        "reference_target_ratio": (row.get("reference") or {}).get("target_sample_ratio"),
        "reference_curvature_1pm": (row.get("reference") or {}).get("maximum_reference_curvature_1pm"),
        "parking_center_band_ratio": row.get("parking_center_band_ratio"),
        "parking_normalized_deviation_p50": row.get("parking_normalized_deviation_p50"),
        "maximum_curvature_1pm": row.get("maximum_curvature_1pm"),
        "wall_ms": row.get("wall_ms"), "peak_rss_bytes": row.get("peak_rss_bytes"),
    } for row in rows]
    with (output / "parking_oracle_results.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat[0])); writer.writeheader(); writer.writerows(flat)
    final = {
        **identity(), "stage": "parking-oracle", "gate_results": gate, "rows": rows,
        "static_prepare_ms": static_prepare_ms, "wall_s": time.monotonic() - run_started,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        "processes_after": applicability._process_audit(),
    }
    applicability._write_json(output / "gate_results.json", gate)
    applicability._write_json(output / "final_result.json", final)
    applicability._write_json(output / "artifact_hashes.json", applicability._manifest_files(output))
    return final


def _ack_gate(rows: Sequence[Mapping[str, Any]], config: Mapping[str, Any], transitions: int) -> dict[str, Any]:
    stage = config["fast_iteration"]["ack_stress"]
    times = [float(row["wall_ms"]) for row in rows]
    rss = [int(row["current_rss_bytes"]) for row in rows]
    window = max(1, len(rss) // 10)
    growth = (
        statistics.median(rss[-window:]) - statistics.median(rss[:window])
        if rss else 0.0
    )
    exact = len(rows) == transitions and all(
        row.get("acknowledged") is True
        and int(row.get("exact_observations", 0)) == 1
        and all(int(row.get(key, 1) or 0) == 0 for key in (
            "hard_mismatch", "soft_mismatch", "stale_cells", "hash_mismatch", "sequence_mismatch",
        ))
        and row.get("full_repair") is not True
        for row in rows
    )
    peak = max((int(row["peak_rss_bytes"]) for row in rows), default=0)
    p99 = percentile(times, 99)
    checks = {
        "all_transitions_exact": exact,
        "p99_within_limit": p99 is not None and p99 <= float(stage["p99_ms_max"]),
        "peak_rss_within_limit": peak <= int(stage["peak_rss_bytes_max"]),
        "steady_rss_growth_within_limit": growth <= int(stage["steady_rss_growth_bytes_max"]),
    }
    return {
        "ack_stress_passed": all(checks.values()), "checks": checks,
        "executed_transitions": len(rows), "required_transitions": transitions,
        "p50_ms": statistics.median(times) if times else None,
        "p95_ms": percentile(times, 95), "p99_ms": p99,
        "peak_rss_bytes": peak, "steady_rss_growth_bytes": growth,
        "hard_stop_triggered": not all(checks.values()),
    }


def run_ack_stress(
    *, output: Path, config_path: Path = DEFAULT_CONFIG, transitions: int | None = None,
    ros_domain_id: int | None = None,
) -> dict[str, Any]:
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / "logs").mkdir()
    run_started = time.monotonic()
    config, algorithm, parent_algorithm, parent, selected8, _query_path, resolved_config = _load(config_path)
    stage_config = config["fast_iteration"]["ack_stress"]
    transition_count = int(stage_config["transitions"] if transitions is None else transitions)
    if transition_count < 1:
        raise ValueError("ACK transitions must be positive")
    domain = int(stage_config["ros_domain_id"] if ros_domain_id is None else ros_domain_id)
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    queries, _intents, query_meta = load_query_set(
        selected8, actual_map_hash=config["frozen_bindings"]["map_hash"],
        actual_semantic_map_hash=config["frozen_bindings"]["semantic_map_hash"],
        require_default_contract=True,
    )
    wanted = list(stage_config["query_ids"])
    by_id = {query.query_id: query for query in queries}
    if any(query_id not in by_id for query_id in wanted):
        raise ValueError("r15 ACK stress contains unknown frozen query")
    sources = [
        Path(__file__).resolve(), resolved_config,
        Path(__file__).with_name("semantic_v3_ack_r15.py"),
        Path(__file__).with_name("semantic_v3_ack_r14.py"), selected8,
    ]
    _write_common(
        output, stage="ack-stress", config=config, sources=sources,
        details={"query_ids": wanted, "transitions": transition_count, "smac_searches": 0},
        reproduction=(
            f"ROS_DOMAIN_ID={domain} /usr/bin/python3 -m "
            f"arena_evaluation.two_layer_v3_semantic_r15_fast --stage ack-stress "
            f"--output {output} --config {resolved_config} --transitions {transition_count} "
            f"--ros-domain-id {domain}"
        ),
    )
    prepared, static_prepare_ms = _prepare_static(algorithm=algorithm, parent=parent, output=output)
    ctx, semantic_map, raster, topology, _annotator, router = prepared[:6]
    builder = expanded.AuditOnlyPreferenceBuilderR14(
        ctx.hospital_map, raster, policy=parent["regional_preference"], semantic_map=semantic_map,
    )
    composer = SemanticCostmapComposerR2(policy=parent["l3_soft_cost"], inflation_cache_capacity=2)
    selector = _selector(parent, topology, router)
    states = []
    for query_id in wanted:
        query = by_id[query_id]
        route, orientation, composition, allowed, _full_allowed, metadata, arrays = _prepare_query_state(
            query=query, query_hash=query_meta["query_hash"], selector=selector,
            ctx=ctx, semantic_map=semantic_map, raster=raster, topology=topology,
            route_phase_algorithm=parent_algorithm, parent=parent,
            builder=builder, composer=composer,
        )
        del route, orientation, arrays
        states.append((query_id, composition, allowed, metadata))
    session = SingleObservationExactAckSessionR15(
        ctx, output, map_yaml=ctx.map_yaml, log_tag=f"2a_v3_r15_ack_{int(time.time())}",
        local_mask_updates=True, optimization_profile="v7_candidate",
        smac_parameter_profile="baseline", optimization_stage="step3_delta_map",
        enable_mask_reuse_noop=True, force_full_on_semantic_signature_change=False,
        planner_parameter_overrides={"angle_quantization_bins": 48},
        costmap_ack_timeout_s=float(stage_config["exact_ack_timeout_s"]),
    )
    session.local_map_update_strategy = "roi_ack"
    # Frozen r14 deterministic publication geometry.
    session.roi_tile_overlap_rows = 32
    session.roi_seam_repair_margin_rows = 16
    session.roi_max_payload_bytes = 128000
    rows: list[dict[str, Any]] = []
    try:
        session.start()
        for index in range(transition_count):
            started = time.monotonic()
            query_id, composition, allowed, metadata = states[index % len(states)]
            row: dict[str, Any] = {
                **identity(), "transition_index": index + 1, "query_id": query_id,
                "expected_master_hash": metadata["expected_master_hash"],
            }
            try:
                session.reset_query_state(f"ack:{index + 1}:{query_id}", restore_base_map=False)
                session.set_semantic_costmap(composition)
                ack = session.update_local_mask(allowed)
                _verified, verification = session.verified_master_snapshot(ack)
                row.update({
                    "acknowledged": ack.get("costmap_update_acknowledged"),
                    "status": ack.get("costmap_ack_status"),
                    "exact_observations": ack.get("r15_exact_observation_count"),
                    "ack_attempts": ack.get("costmap_ack_attempts"),
                    "ack_wait_ms": ack.get("costmap_ack_wait_ms"),
                    "hard_mismatch": ack.get("costmap_ack_hard_mismatch_cells"),
                    "soft_mismatch": ack.get("costmap_ack_soft_exact_mismatch_cells"),
                    "stale_cells": ack.get("costmap_ack_stale_roi_cells"),
                    "hash_mismatch": ack.get("costmap_ack_hash_mismatch"),
                    "sequence_mismatch": ack.get("costmap_ack_sequence_mismatch"),
                    "full_repair": ack.get("local_map_update_fallback", False),
                    "replay_scope": ack.get("deterministic_reinflation_replay_scope"),
                    "server_content_hash": ack.get("server_costmap_content_hash"),
                    "publication_sequence": ack.get("semantic_publication_sequence"),
                    "verified_master_hash": verification.get("verified_master_hash"),
                    "failure_code": "",
                })
            except (RuntimeError, ValueError) as error:
                row.update({
                    "acknowledged": False, "failure_code": type(error).__name__,
                    "failure_detail": str(error),
                })
            row["wall_ms"] = (time.monotonic() - started) * 1000.0
            row["current_rss_bytes"] = current_rss_bytes()
            row["peak_rss_bytes"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
            rows.append(expanded._json_safe(row))
            if row.get("acknowledged") is not True and config["fast_iteration"]["early_stop_on_first_hard_failure"]:
                break
    finally:
        try:
            session.close()
        except Exception:
            # Preserve the original gate failure while still attempting the
            # owner's idempotent process-group cleanup on partial startup.
            pass
    with (output / "ack_stress.csv").open("x", newline="", encoding="utf-8") as stream:
        keys = sorted({key for row in rows for key in row})
        writer = csv.DictWriter(stream, fieldnames=keys); writer.writeheader(); writer.writerows(rows)
    with (output / "ack_stress.jsonl").open("x", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, sort_keys=True, ensure_ascii=False) + "\n")
    gate = _ack_gate(rows, config, transition_count)
    final = {
        **identity(), "stage": "ack-stress", "gate_results": gate,
        "static_prepare_ms": static_prepare_ms, "wall_s": time.monotonic() - run_started,
        "smac_search_count": 0, "rows": len(rows),
        "processes_after": applicability._process_audit(),
    }
    applicability._write_json(output / "gate_results.json", gate)
    applicability._write_json(output / "performance_summary.json", gate)
    applicability._write_json(output / "final_result.json", final)
    applicability._write_json(output / "artifact_hashes.json", applicability._manifest_files(output))
    return final


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", required=True, choices=("parking-oracle", "ack-stress"))
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--transitions", type=int)
    parser.add_argument("--ros-domain-id", type=int)
    args = parser.parse_args(argv)
    if args.stage == "parking-oracle":
        result = run_parking_oracle(output=args.output, config_path=args.config)
    else:
        result = run_ack_stress(
            output=args.output, config_path=args.config,
            transitions=args.transitions, ros_domain_id=args.ros_domain_id,
        )
    print(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False))
    return 0 if result["gate_results"].get(
        "parking_oracle_passed" if args.stage == "parking-oracle" else "ack_stress_passed"
    ) else 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "ARCHITECTURE_ID", "IMPLEMENTATION_REVISION", "PROTOCOL_ID",
    "identity", "percentile", "run_ack_stress", "run_parking_oracle", "main",
]
