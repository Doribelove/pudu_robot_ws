"""PLN-02 2A-V2-r0 static semantic A/B benchmark.

The formal path remains L1 topology -> corridor-local full-route Smac Hybrid
DUBIN.  This module adds semantic L1 edge cost, a query-direction-conditioned
L3 cost grid, observable R0..R4 relaxation, semantic ACK and a second audit;
it does not import any dynamic-incremental pipeline.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import resource
import subprocess
import time
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml
from scipy import ndimage

from . import l1_l3_corridor_hybrid_smoke as r2_runtime
from . import path_audit
from . import unified_four_backends_smoke as legacy
from .edge_semantic_annotator import (
    EdgeSemanticAnnotator, SemanticEdgeRouter, topology_graph_hash,
)
from .pdmap_semantic_converter import convert_pdmap
from .planner_benchmark.map_utils import HospitalMap, sha256_file
from .planner_benchmark.models import Query
from .regional_preference import PreferenceField, RegionalPreferenceBuilder
from .semantic_costmap_composer import SemanticCostmapComposer
from .semantic_map import SemanticMapV1, canonical_hash
from .semantic_path_audit import SemanticPathAuditor
from .semantic_query_set import generate_query_set, save_query_set
from .semantic_rasterizer import SemanticRasterizer
from .semantic_relaxation import PreferenceRelaxationController
from .semantic_smac_session import SemanticSmacSession
from .topology import build_topology, load_topology, save_topology


ARCHITECTURE_ID = "2A-V2"
IMPLEMENTATION_REVISION = "r0"
PARENT_ARCHITECTURE = "2A-V1-r2-roi-pathaudit-v1"
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config/two_layer_v2_semantic.yaml"
ROOT = Path(__file__).resolve().parents[7]


def _refuse_nonempty(path: Path) -> None:
    if path.exists() and any(path.iterdir()):
        raise FileExistsError(f"refusing to overwrite non-empty output: {path}")


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    materialized = list(rows)
    fields: List[str] = []
    for row in materialized:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or ["empty"])
        writer.writeheader()
        for row in materialized:
            writer.writerow({
                key: json.dumps(value, sort_keys=True, ensure_ascii=False, default=str)
                if isinstance(value, (dict, list, tuple)) else value
                for key, value in row.items()
            })


def _git_state(path: Path) -> Dict[str, str]:
    def call(*args: str) -> str:
        try:
            return subprocess.check_output(
                ["git", "-C", str(path), *args], text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
        except (OSError, subprocess.SubprocessError):
            return "unavailable"
    return {"commit": call("rev-parse", "HEAD"), "status": call("status", "--short", "--branch")}


def _percentiles(rows: Sequence[Mapping[str, Any]], key: str) -> Dict[str, Optional[float]]:
    values = []
    for row in rows:
        try:
            value = float(row.get(key))
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return {
        f"p{percentile}": float(np.percentile(values, percentile)) if values else None
        for percentile in (50, 95, 99)
    }


def _context(map_yaml: Path) -> Any:
    hospital_map = HospitalMap.load(map_yaml)
    if not math.isclose(hospital_map.resolution, 0.05, abs_tol=1e-12):
        raise ValueError("2A-V2 requires exactly 0.05 m/cell")
    from .topology import preprocess_static_map
    _, free, distance, _ = preprocess_static_map(
        hospital_map, legacy.FOOTPRINT, padding_m=0.05,
        safety_margin_m=0.05, allow_unknown=False,
    )
    return legacy.MapContext(
        "pudu_wanda_3f", hospital_map, free, distance,
        sha256_file(hospital_map.image_path), sha256_file(hospital_map.yaml_path), map_yaml,
    )


def _zero_preference(shape: Tuple[int, int], policy_hash: str, route: Sequence[Sequence[float]]) -> PreferenceField:
    nan = np.full(shape, np.nan, dtype=np.float32)
    return PreferenceField(
        cost=np.zeros(shape, dtype=np.uint8),
        lane_distance_to_right_m=nan.copy(), lane_error_m=nan.copy(),
        lane_correct_side=np.zeros(shape, dtype=bool),
        parking_normalized_deviation=nan.copy(),
        junction_transition_factor=np.ones(shape, dtype=np.float32),
        direction_stability=np.ones(shape, dtype=np.float32),
        active_lateral_mask=np.zeros(shape, dtype=bool),
        relaxation_level="R0", policy_hash=policy_hash,
        route_hash=canonical_hash(route), diagnostics={"semantics_disabled": True},
    )


def _semantic_selector(topology: Any, router: SemanticEdgeRouter):
    def select(
        _topology: Any, query: Query, *, cache_mode: str,
        timing: Optional[Dict[str, Any]] = None,
    ) -> Tuple[Any, Any, Any, str]:
        started = time.monotonic_ns()
        start_timing: Dict[str, Any] = {}
        goal_timing: Dict[str, Any] = {}
        starts = r2_runtime._attachment_candidates(
            topology, query.start, cache_mode=cache_mode, timing=start_timing,
        )
        goals = r2_runtime._attachment_candidates(
            topology, query.goal, cache_mode=cache_mode, timing=goal_timing,
        )
        starts_by_id = {int(item.node_id): item for item in starts}
        goals_by_id = {int(item.node_id): item for item in goals}
        selected = router.search_any(list(starts_by_id), list(goals_by_id))
        if selected is not None:
            route, start_id, goal_id = selected
            if timing is not None:
                timing.update({
                    "start_lookup_ms": float(start_timing.get("lookup_ms", 0.0)),
                    "goal_lookup_ms": float(goal_timing.get("lookup_ms", 0.0)),
                    "start_collision_check_ms": float(start_timing.get("collision_check_ms", 0.0)),
                    "goal_collision_check_ms": float(goal_timing.get("collision_check_ms", 0.0)),
                    "adjacency_build_ms": 0.0,
                    "route_search_ms": (time.monotonic_ns() - started) / 1.0e6,
                    "route_construction_ms": 0.0,
                    "start_candidate_count": len(starts), "goal_candidate_count": len(goals),
                    "candidate_pair_attempts": 1,
                    "topology_adjacency_cache_hit": topology.graph.adjacency_cache_hit,
                    "endpoint_spatial_index_cache_hit": bool(
                        start_timing.get("spatial_index_cache_hit", False)
                        and goal_timing.get("spatial_index_cache_hit", False)
                    ),
                    "endpoint_candidate_cache_hit": bool(
                        start_timing.get("endpoint_candidate_cache_hit", False)
                        and goal_timing.get("endpoint_candidate_cache_hit", False)
                    ),
                    "route_cache_hit": False,
                })
            return starts_by_id[start_id], goals_by_id[goal_id], route, "semantic_multi_source_route"
        if timing is not None:
            timing.update({
                "route_search_ms": (time.monotonic_ns() - started) / 1.0e6,
                "candidate_pair_attempts": 1 if starts and goals else 0,
                "start_candidate_count": len(starts), "goal_candidate_count": len(goals),
            })
        return starts[0] if starts else None, goals[0] if goals else None, None, "semantic_no_route"
    return select


def _make_path_overlay(
    hospital_map: HospitalMap, raster: Any, paths: Mapping[str, Sequence[Mapping[str, Any]]],
    output: Path,
) -> None:
    base = np.asarray(cv2.imread(str(hospital_map.image_path), cv2.IMREAD_GRAYSCALE), dtype=np.uint8)
    canvas = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)
    overlay = canvas.copy()
    colors = {
        "lane": (60, 180, 60), "junction_area": (220, 210, 40),
        "parking_area": (210, 60, 190), "speed_bumps": (20, 180, 230),
        "fence_area": (180, 100, 30), "forbidden": (20, 20, 230),
    }
    for key, color in colors.items():
        mask = raster.masks.get(key)
        if mask is not None:
            overlay[np.asarray(mask, dtype=bool)] = color
    canvas = cv2.addWeighted(canvas, 0.65, overlay, 0.35, 0.0)
    for key, points in paths.items():
        color = (255, 80, 20) if key.startswith("A/") else (20, 20, 255)
        cells = []
        for point in points:
            cell = hospital_map.world_to_cell(float(point["x"]), float(point["y"]))
            if cell is not None:
                cells.append([cell[1], cell[0]])
        if len(cells) >= 2:
            cv2.polylines(canvas, [np.asarray(cells, dtype=np.int32)], False, color, 2, cv2.LINE_AA)
    if not cv2.imwrite(str(output), canvas):
        raise OSError(f"failed to save visualization: {output}")


def _summaries(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    result = []
    for arm in ("A_semantics_disabled", "B_semantics_enabled"):
        selected = [row for row in rows if row["arm"] == arm and row["run_mode"] == "measured"]
        valid = [row for row in selected if row.get("final_valid_success") is True]
        levels: Dict[str, int] = {}
        failures: Dict[str, int] = {}
        for row in selected:
            levels[str(row.get("relaxation_level"))] = levels.get(str(row.get("relaxation_level")), 0) + int(row.get("final_valid_success") is True)
            code = str(row.get("failure_code") or "")
            if code:
                failures[code] = failures.get(code, 0) + 1
        result.append({
            "arm": arm,
            "request_count": len(selected),
            "success_count": len(valid),
            "success_rate": len(valid) / len(selected) if selected else 0.0,
            "online_wall_ms": _percentiles(selected, "online_wall_ms"),
            "planning_time_ms": _percentiles(selected, "planning_time_ms"),
            "l1_time_ms": _percentiles(selected, "l1_time_ms"),
            "roi_time_ms": _percentiles(selected, "roi_time_ms"),
            "smac_time_ms": _percentiles(selected, "smac_time_ms"),
            "audit_time_ms": _percentiles(selected, "audit_time_ms"),
            "path_length_m": _percentiles(valid, "path_length_m"),
            "curvature_p95": _percentiles(valid, "curvature_p95"),
            "peak_rss_bytes": max((int(row.get("peak_rss_bytes") or 0) for row in selected), default=0),
            "lane_correct_side_ratio": _percentiles(valid, "lane_correct_side_ratio"),
            "lane_error_p50_m": _percentiles(valid, "base_center_to_right_boundary_error_p50_m"),
            "parking_center_p50": _percentiles(valid, "parking_center_normalized_deviation_p50"),
            "relaxation_trigger_rate": sum(str(row.get("relaxation_level")) != "R0" for row in selected) / len(selected) if selected else 0.0,
            "success_by_relaxation_level": levels,
            "costmap_acknowledged_count": sum(row.get("costmap_update_acknowledged") is True for row in selected),
            "costmap_ack_hard_mismatch_count": sum(int(row.get("costmap_ack_hard_mismatch_cells") or 0) for row in selected),
            "costmap_ack_soft_bound_mismatch_count": sum(int(row.get("costmap_ack_soft_mismatch_cells") or 0) for row in selected),
            "costmap_ack_soft_exact_mismatch_count": sum(int(row.get("costmap_ack_soft_exact_mismatch_cells") or 0) for row in selected),
            "hard_semantic_violation_count": sum(int(row.get("hard_semantic_violation_count") or 0) for row in selected),
            "collision_violation_count": sum(int(row.get("collision_violation_count") or 0) for row in selected),
            "kinematic_violation_count": sum(int(row.get("kinematic_violation_count") or 0) for row in selected),
            "no_stopping_goal_violation_count": sum(bool(row.get("no_stopping_goal_violation")) for row in selected),
            "failure_codes": failures,
        })
    return result


def run_real_ab(
    *, extracted_dir: Path, semantic_map_path: Path, output: Path, config_path: Path,
    warmups: int, repetitions: int, ros_domain_id: int,
    topology_cache: Optional[Path] = None,
) -> Path:
    _refuse_nonempty(output)
    output.mkdir(parents=True)
    (output / "paths").mkdir()
    config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    protocol = config["protocol"]
    if protocol.get("dynamic_obstacles") is not False:
        raise ValueError("formal 2A-V2 experiment requires dynamic_obstacles=false")
    if protocol.get("allow_reverse") is not False or protocol.get("allow_in_place_rotation") is not False:
        raise ValueError("formal 2A-V2 experiment requires forward-only, no in-place rotation")
    if float(protocol["minimum_turning_radius_m"]) != 0.40 or float(protocol["maximum_curvature_1pm"]) != 2.50:
        raise ValueError("formal 2A-V2 kinematic protocol mismatch")
    ctx = _context((extracted_dir / "optemap.yaml").resolve())
    semantic_map = SemanticMapV1.load(semantic_map_path)
    semantic_map.validate_against_map(ctx.hospital_map)
    raster = SemanticRasterizer(
        footprint=protocol["footprint"],
        safety_margin_m=float(protocol["semantic_safety_margin_m"]),
    ).rasterize(semantic_map, hospital_map=ctx.hospital_map)
    raster.save(output / "semantic_raster.npz")
    topology_dir = output / "topology_cache"
    topology_started = time.monotonic_ns()
    topology = (
        load_topology(
            topology_cache, ctx.hospital_map, protocol["footprint"],
            padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
        )
        if topology_cache is not None else
        build_topology(
            ctx.hospital_map, protocol["footprint"], padding_m=0.05,
            safety_margin_m=0.05, allow_unknown=False,
        )
    )
    topology_build_ms = (time.monotonic_ns() - topology_started) / 1.0e6
    save_topology(topology, topology_dir)
    graph_hash = topology_graph_hash(topology)
    annotator = EdgeSemanticAnnotator(
        ctx.hospital_map, semantic_map, raster, base_map_hash=ctx.map_sha256,
        topology_hash=graph_hash, policy=config["l1_edge_cost"],
    )
    semantic_edge_started = time.monotonic_ns()
    annotator.precompute(topology.graph.edges)
    semantic_edge_precompute_ms = (time.monotonic_ns() - semantic_edge_started) / 1.0e6
    edge_payload = {
        "schema_version": "2A-V2-edge-semantics-v1",
        "base_map_hash": ctx.map_sha256,
        "semantic_map_hash": semantic_map.semantic_map_hash,
        "policy_hash": annotator.policy_hash,
        "topology_graph_hash": graph_hash,
        "precompute_ms": semantic_edge_precompute_ms,
        "annotations": [
            annotator.annotate(edge, reversed_traversal=reversed_value).to_dict()
            for edge in topology.graph.edges for reversed_value in (False, True)
        ],
    }
    (output / "semantic_edges.json").write_text(
        json.dumps(edge_payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8",
    )
    router = SemanticEdgeRouter(topology, annotator)
    queries, intents, query_metadata = generate_query_set(
        ctx.hospital_map, topology.free_mask, topology.free_components, raster,
        seed=int(config["experiment"]["seed"]),
    )
    save_query_set(output / "real_query_set.yaml", queries, intents, query_metadata)
    canonical_auditor = path_audit.PathAuditor(ctx, source_commit=_git_state(ROOT)["commit"])
    semantic_auditor = SemanticPathAuditor(ctx.hospital_map, semantic_map, raster)
    preference_builder = RegionalPreferenceBuilder(
        ctx.hospital_map, raster, policy=config["regional_preference"],
    )
    composer = SemanticCostmapComposer(policy=config["l3_soft_cost"])
    spec = legacy.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"BACKEND_UNAVAILABLE: {spec.reason}")
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    session = SemanticSmacSession(
        ctx, output, map_yaml=ctx.map_yaml,
        log_tag=f"2a_v2_semantic_{int(time.time())}", local_mask_updates=True,
        optimization_profile="v7_candidate", smac_parameter_profile="baseline",
        optimization_stage="step3_delta_map", enable_mask_reuse_noop=True,
        costmap_ack_timeout_s=float(config["roi"]["ack_timeout_s"]),
    )
    session.local_map_update_strategy = "roi_ack"
    rows: List[Dict[str, Any]] = []
    paths: Dict[str, Sequence[Mapping[str, Any]]] = {}
    baseline_lengths: Dict[str, float] = {}
    session.start()
    try:
        for arm, semantics_enabled in (("A_semantics_disabled", False), ("B_semantics_enabled", True)):
            selector = _semantic_selector(topology, router) if semantics_enabled else r2_runtime._select_route_with_endpoint_attach
            for run_mode, count in (("warmup", warmups), ("measured", repetitions)):
                for repetition in range(1, count + 1):
                    for query in queries:
                        wall_started = time.monotonic_ns()
                        session.reset_query_state(query.query_id, restore_base_map=False)
                        current: Dict[str, Any] = {}
                        controller_cfg = config["preference_relaxation"]
                        controller = PreferenceRelaxationController(
                            enabled=bool(controller_cfg["enabled"] and semantics_enabled),
                            mode=str(controller_cfg["mode"]), levels=controller_cfg["levels"],
                        )

                        def attempt(level: str, _parameters: Mapping[str, Any]):
                            if semantics_enabled:
                                start_cell = ctx.hospital_map.world_to_cell(query.start[0], query.start[1])
                                goal_cell = ctx.hospital_map.world_to_cell(query.goal[0], query.goal[1])
                                if start_cell is None or goal_cell is None:
                                    return False, None, "INVALID_ENDPOINT", "preflight", True
                                if raster.no_stopping_mask[goal_cell]:
                                    return False, None, "NO_STOPPING_GOAL_VIOLATION", "preflight", False
                                if raster.hard_footprint_mask[start_cell] or raster.hard_footprint_mask[goal_cell]:
                                    return False, None, "HARD_SEMANTIC_ENDPOINT", "preflight", False
                            padding = float(config["roi"]["r3_padding_m"] if level == "R3" else config["roi"]["r0_padding_m"])

                            def mask_builder(_ctx: Any, _topology: Any, route: Any, _query: Query,
                                             start_cell: Any, goal_cell: Any, _padding: float, _semantics: str):
                                build_started = time.monotonic_ns()
                                if level == "R4":
                                    allowed = r2_runtime._raw_free_mask(ctx).copy()
                                else:
                                    allowed = r2_runtime._raw_corridor_mask(ctx, topology, route, query, padding)
                                if start_cell is not None:
                                    allowed[start_cell] = True
                                if goal_cell is not None:
                                    allowed[goal_cell] = True
                                if semantics_enabled:
                                    preference = preference_builder.build(
                                        route.polyline, goal=query.goal, allowed_mask=allowed,
                                        relaxation_level=level,
                                    )
                                else:
                                    preference = _zero_preference(allowed.shape, preference_builder.policy_hash, route.polyline)
                                semantic_costmap = composer.compose(
                                    ctx.hospital_map.occupancy, raster, preference,
                                    allowed_mask=allowed, semantics_enabled=semantics_enabled,
                                )
                                session.set_semantic_costmap(semantic_costmap)
                                current.update({
                                    "route": route, "preference": preference,
                                    "semantic_costmap": semantic_costmap,
                                    "roi_time_ms": (time.monotonic_ns() - build_started) / 1.0e6,
                                })
                                return allowed, {
                                    "semantic_map_hash": semantic_map.semantic_map_hash,
                                    "semantic_policy_hash": composer.policy_hash,
                                    "semantic_costmap_hash": semantic_costmap.expected_grid_hash,
                                    "semantic_expected_master_hash": semantic_costmap.expected_master_hash,
                                    "preference_field_hash": preference.field_hash,
                                    "relaxation_level": level,
                                    "hard_constraints_preserved": True,
                                    "l1_semantic_cost": getattr(route, "semantic_cost", route.length_m),
                                    "l1_semantic_edge_annotations": getattr(route, "semantic_edge_annotations", []),
                                    **preference.diagnostics,
                                }

                            result, diagnostics = r2_runtime.plan_l1_l3_corridor_hybrid(
                                ctx, query, topology, session, spec,
                                corridor_padding_m=padding,
                                corridor_semantics="raw_map_smac_aligned",
                                padding_schedule_m=(padding,),
                                validate_each_attempt=True,
                                cache_mode=r2_runtime.CACHE_MODE_OPTIMIZED,
                                corridor_mask_builder=mask_builder,
                                route_selector=selector,
                                canonical_path_auditor=canonical_auditor.audit,
                                skip_session_path_mask_validation=True,
                            )
                            current.update({"result": result, "diagnostics": diagnostics})
                            metrics = dict(getattr(result.path_audit, "metrics", {}) or {})
                            if result.points and "preference" in current:
                                semantic_audit_started = time.monotonic_ns()
                                semantic_audit = semantic_auditor.audit(
                                    result.points, current["preference"], relaxation_level=level,
                                    canonical_metrics=metrics,
                                    baseline_path_length_m=baseline_lengths.get(query.query_id),
                                )
                                current["semantic_audit_ms"] = (time.monotonic_ns() - semantic_audit_started) / 1.0e6
                                current["semantic_audit"] = semantic_audit
                                hard_held = semantic_audit.hard_constraints_held
                            else:
                                hard_held = True
                            success = bool(result.planner_success and result.points and hard_held)
                            failure = "" if success else str(
                                getattr(current.get("semantic_audit"), "failure_code", "")
                                or result.failure_code or diagnostics.get("failure_code") or "PLANNING_FAILED"
                            )
                            return success, result, failure, str(diagnostics.get("failure_code") or "L3"), hard_held

                        relaxed = controller.run(attempt)
                        result = current.get("result")
                        diagnostics = dict(getattr(result, "diagnostics", {}) or {}) if result is not None else {}
                        canonical = dict(getattr(getattr(result, "path_audit", None), "metrics", {}) or {})
                        semantic_audit = current.get("semantic_audit")
                        semantic_metrics = semantic_audit.to_dict() if semantic_audit is not None else {}
                        elapsed_ms = (time.monotonic_ns() - wall_started) / 1.0e6
                        points = list(getattr(result, "points", []) or [])
                        if relaxed.success and points:
                            path_key = f"{arm}/{query.query_id}/{repetition}"
                            if run_mode == "measured" and path_key not in paths:
                                paths[path_key] = points
                            path_file = output / "paths" / f"{arm}_{query.query_id}_{run_mode}_{repetition}.json"
                            path_file.write_text(json.dumps(points, indent=2) + "\n", encoding="utf-8")
                            if not semantics_enabled and run_mode == "measured":
                                baseline_lengths[query.query_id] = float(canonical.get("path_length_m") or 0.0)
                        usage = resource.getrusage(resource.RUSAGE_SELF)
                        row = {
                            "architecture_id": ARCHITECTURE_ID,
                            "implementation_revision": IMPLEMENTATION_REVISION,
                            "parent_architecture": PARENT_ARCHITECTURE,
                            "semantic_map_version": semantic_map.schema_version,
                            "source_map_hash": ctx.map_sha256,
                            "source_pdmap_hash": semantic_map.source_pdmap_hash,
                            "semantic_map_hash": semantic_map.semantic_map_hash,
                            "policy_hash": composer.policy_hash,
                            "topology_graph_hash": graph_hash,
                            "arm": arm, "semantics_enabled": semantics_enabled,
                            "query_id": query.query_id, "category": query.category,
                            "run_mode": run_mode, "repetition": repetition,
                            "action_success": bool(points),
                            "final_valid_success": bool(relaxed.success),
                            "failure_code": relaxed.failure_code,
                            "relaxation_level": relaxed.relaxation_level,
                            "relaxation_attempts": [asdict(item) for item in relaxed.attempts],
                            "hard_constraints_held": bool(semantic_metrics.get("hard_constraints_held", bool(relaxed.success))),
                            "online_wall_ms": elapsed_ms,
                            "planning_time_ms": diagnostics.get("l3_planning_time_ms", diagnostics.get("planning_time_ms")),
                            "l1_time_ms": diagnostics.get("l1_graph_search_ms", 0.0),
                            "roi_time_ms": current.get("roi_time_ms", 0.0),
                            "smac_time_ms": diagnostics.get("l3_action_wall_ms", 0.0),
                            "audit_time_ms": float(diagnostics.get("canonical_path_audit_ms") or 0.0) + float(current.get("semantic_audit_ms") or 0.0),
                            "peak_rss_bytes": int(diagnostics.get("stack_rss_peak_bytes") or usage.ru_maxrss * 1024),
                            "path_file": str(path_file.relative_to(output)) if relaxed.success and points else "",
                            "costmap_update_acknowledged": diagnostics.get("costmap_update_acknowledged"),
                            "costmap_ack_status": diagnostics.get("costmap_ack_status"),
                            "costmap_ack_checked_cells": diagnostics.get("costmap_ack_checked_cells"),
                            "costmap_ack_mismatch_cells": diagnostics.get("costmap_ack_mismatch_cells"),
                            "costmap_ack_hard_checked_cells": diagnostics.get("costmap_ack_hard_checked_cells"),
                            "costmap_ack_hard_mismatch_cells": diagnostics.get("costmap_ack_hard_mismatch_cells"),
                            "costmap_ack_soft_checked_cells": diagnostics.get("costmap_ack_soft_checked_cells"),
                            "costmap_ack_soft_mismatch_cells": diagnostics.get("costmap_ack_soft_mismatch_cells"),
                            "costmap_ack_soft_exact_mismatch_cells": diagnostics.get("costmap_ack_soft_exact_mismatch_cells"),
                            "semantic_publication_version": diagnostics.get("semantic_publication_version"),
                            "semantic_roi_sequence": diagnostics.get("semantic_roi_sequence"),
                            "semantic_policy_hash": diagnostics.get("semantic_policy_hash"),
                            "semantic_expected_grid_hash": diagnostics.get("semantic_expected_grid_hash"),
                            "semantic_expected_master_hash": diagnostics.get("semantic_expected_master_hash"),
                            "received_costmap_timestamp_ns": diagnostics.get("server_costmap_update_time_ns"),
                            **canonical,
                            **semantic_metrics,
                        }
                        rows.append(row)
                        with (output / "runs.partial.jsonl").open("a", encoding="utf-8") as stream:
                            stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True, default=str) + "\n")
    finally:
        session.close()
    summaries = _summaries(rows)
    _write_csv(output / "runs.csv", rows)
    _write_csv(output / "summary.csv", summaries)
    (output / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n", encoding="utf-8")
    _make_path_overlay(ctx.hospital_map, raster, paths, output / "real_ab_overlay.png")
    workspace_state = _git_state(ROOT)
    nav2_state = _git_state(ROOT / "external/arena4_ws/src/deps/nav2/navigation2")
    protocol_record = {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "parent_architecture": PARENT_ARCHITECTURE,
        "semantic_map_version": semantic_map.schema_version,
        "source_map_hash": ctx.map_sha256,
        "source_pdmap_hash": semantic_map.source_pdmap_hash,
        "semantic_map_hash": semantic_map.semantic_map_hash,
        "policy_hash": composer.policy_hash,
        "topology_graph_hash": graph_hash,
        "topology_build_ms": topology_build_ms,
        "semantic_edge_precompute_ms": semantic_edge_precompute_ms,
        "topology_cache_source": str(topology_cache) if topology_cache is not None else "built_for_this_run",
        "topology_nodes": len(topology.graph.nodes),
        "topology_edges": len(topology.graph.edges),
        "query_set": query_metadata,
        "warmups": warmups, "repetitions": repetitions,
        "ros_domain_id": ros_domain_id,
        "static_map": True, "dynamic_obstacles": False,
        "workspace_git": workspace_state,
        "nav2_git": nav2_state,
        "config": config,
    }
    (output / "protocol.json").write_text(json.dumps(protocol_record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    lines = [
        "# 2A-V2 semantic real-pdmap A/B", "",
        f"- architecture: `{ARCHITECTURE_ID}` / `{IMPLEMENTATION_REVISION}`",
        f"- parent: `{PARENT_ARCHITECTURE}`",
        "- environment: static map; `dynamic_obstacles=false`", "",
        "| Arm | Success | P50/P95/P99 online ms | Hard semantic | Collision | Kinematic |",
        "|---|---:|---|---:|---:|---:|",
    ]
    for summary in summaries:
        lines.append(
            f"| {summary['arm']} | {summary['success_count']}/{summary['request_count']} | "
            f"{summary['online_wall_ms']} | {summary['hard_semantic_violation_count']} | "
            f"{summary['collision_violation_count']} | {summary['kinematic_violation_count']} |"
        )
    lines.extend(["", "All values above are traceable to `runs.csv`, `summary.json`, per-path JSON, and the private overlay image."])
    (output / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output


def _weighted_astar(cost: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
    import heapq
    queue = [(0.0, 0.0, start)]
    best = {start: 0.0}
    previous: Dict[Tuple[int, int], Tuple[int, int]] = {}
    while queue:
        _, value, cell = heapq.heappop(queue)
        if value != best.get(cell):
            continue
        if cell == goal:
            path = [cell]
            while path[-1] != start:
                path.append(previous[path[-1]])
            return list(reversed(path))
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            nxt = (cell[0] + dr, cell[1] + dc)
            if not (0 <= nxt[0] < cost.shape[0] and 0 <= nxt[1] < cost.shape[1]) or cost[nxt] >= 254:
                continue
            step = math.hypot(dr, dc) * (1.0 + float(cost[nxt]) / 252.0 * 4.0)
            candidate = value + step
            if candidate < best.get(nxt, float("inf")):
                best[nxt] = candidate
                previous[nxt] = cell
                heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(queue, (candidate + heuristic, candidate, nxt))
    return []


def run_synthetic_smoke(output: Path, config_path: Path) -> Path:
    _refuse_nonempty(output)
    output.mkdir(parents=True)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    height, width, resolution = 100, 180, 0.05
    occupancy = np.zeros((height, width), dtype=np.int8)
    occupancy[[0, -1], :] = 100
    occupancy[:, [0, -1]] = 100
    distance = ndimage.distance_transform_edt(occupancy == 0, sampling=resolution).astype(np.float32)
    hospital_map = HospitalMap(
        Path("synthetic.yaml"), Path("synthetic.pgm"), resolution, (0.0, 0.0, 0.0),
        width, height, occupancy, distance,
    )
    features = []
    from .semantic_map import SemanticFeature
    for semantic_id, semantic_class, points, hard, soft, priority in (
        ("lane", "lane", [[0.5, 1.0], [8.4, 1.0], [8.4, 4.0], [0.5, 4.0], [0.5, 1.0]], False, True, 60),
        ("junction", "junction_area", [[4.0, 1.0], [4.8, 1.0], [4.8, 4.0], [4.0, 4.0], [4.0, 1.0]], False, True, 80),
        ("parking", "parking_area", [[7.0, 1.2], [8.3, 1.2], [8.3, 3.8], [7.0, 3.8], [7.0, 1.2]], False, True, 70),
        ("forbidden", "forbidden", [[5.4, 2.0], [5.9, 2.0], [5.9, 3.0], [5.4, 3.0], [5.4, 2.0]], True, False, 100),
    ):
        features.append(SemanticFeature(
            semantic_id, semantic_class, "polygon", points, hard=hard, soft=soft,
            direction_rule="route_tangent_right" if semantic_class == "lane" else "none",
            priority=priority, source_field=f"synthetic.{semantic_id}",
        ))
    semantic_map = SemanticMapV1(
        "map", resolution, (0.0, 0.0, 0.0), width, height, "synthetic",
        features=features, traffic_rules={"right_hand_drive": True},
    )
    raster = SemanticRasterizer(
        footprint=config["protocol"]["footprint"], safety_margin_m=0.05,
    ).rasterize(semantic_map, hospital_map=hospital_map)
    builder = RegionalPreferenceBuilder(hospital_map, raster, policy=config["regional_preference"])
    composer = SemanticCostmapComposer(policy=config["l3_soft_cost"])
    route = [[0.8, 2.5], [3.8, 2.5], [5.0, 2.5], [8.0, 2.5]]
    reverse = list(reversed(route))
    allowed = occupancy == 0
    forward_field = builder.build(route, goal=route[-1], allowed_mask=allowed, relaxation_level="R0")
    reverse_field = builder.build(reverse, goal=reverse[-1], allowed_mask=allowed, relaxation_level="R0")
    forward_map = composer.compose(occupancy, raster, forward_field, allowed_mask=allowed, semantics_enabled=True)
    reverse_map = composer.compose(occupancy, raster, reverse_field, allowed_mask=allowed, semantics_enabled=True)
    start = hospital_map.world_to_cell(*route[0])
    goal = hospital_map.world_to_cell(*route[-1])
    assert start is not None and goal is not None
    forward_path = _weighted_astar(forward_map.internal_cost, start, goal)
    reverse_path = _weighted_astar(reverse_map.internal_cost, goal, start)
    lane = raster.masks["lane"]
    forward_rows = np.argwhere((forward_field.cost < 32) & lane)[:, 0]
    reverse_rows = np.argwhere((reverse_field.cost < 32) & lane)[:, 0]
    smoke = {
        "schema_version": "2A-V2-synthetic-smoke-v1",
        "architecture_id": ARCHITECTURE_ID,
        "forward_path_found": bool(forward_path),
        "reverse_path_found": bool(reverse_path),
        "forward_preferred_row_p50": float(np.median(forward_rows)),
        "reverse_preferred_row_p50": float(np.median(reverse_rows)),
        "right_side_flipped": abs(float(np.median(forward_rows)) - float(np.median(reverse_rows))) > 10.0,
        "forward_hard_overlap": sum(bool(raster.hard_footprint_mask[cell]) for cell in forward_path),
        "reverse_hard_overlap": sum(bool(raster.hard_footprint_mask[cell]) for cell in reverse_path),
        "junction_cost_max": int(np.max(forward_field.cost[raster.masks["junction_area"]])),
        "soft_cost_max": int(max(np.max(forward_map.soft_cost), np.max(reverse_map.soft_cost))),
        "hard_cost": int(np.max(forward_map.internal_cost[raster.hard_mask])),
        "semantic_map_hash": semantic_map.semantic_map_hash,
        "forward_field_hash": forward_field.field_hash,
        "reverse_field_hash": reverse_field.field_hash,
    }
    smoke["passed"] = bool(
        smoke["forward_path_found"] and smoke["reverse_path_found"]
        and smoke["right_side_flipped"] and smoke["forward_hard_overlap"] == 0
        and smoke["reverse_hard_overlap"] == 0 and smoke["junction_cost_max"] == 0
        and smoke["soft_cost_max"] < 254 and smoke["hard_cost"] == 254
    )
    (output / "synthetic_smoke.json").write_text(json.dumps(smoke, indent=2) + "\n", encoding="utf-8")
    image = np.full((height, width, 3), 255, dtype=np.uint8)
    image[occupancy == 100] = (0, 0, 0)
    image[raster.masks["lane"]] = (220, 250, 220)
    image[raster.masks["junction_area"]] = (240, 240, 170)
    image[raster.masks["parking_area"]] = (245, 210, 240)
    image[raster.hard_mask] = (30, 30, 220)
    for path, color in ((forward_path, (255, 50, 20)), (reverse_path, (20, 20, 255))):
        if path:
            cv2.polylines(image, [np.asarray([[cell[1], cell[0]] for cell in path], np.int32)], False, color, 2)
    cv2.imwrite(str(output / "synthetic_smoke.png"), image)
    if not smoke["passed"]:
        raise RuntimeError(f"synthetic semantic smoke failed: {smoke}")
    return output


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run PLN-02 2A-V2-r0 static semantic conversion/smoke/A-B")
    parser.add_argument("--mode", choices=("convert", "synthetic-smoke", "real-ab"), default="synthetic-smoke")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--pdmap", type=Path)
    parser.add_argument("--extracted-dir", type=Path)
    parser.add_argument("--semantic-map", type=Path)
    parser.add_argument("--topology-cache", type=Path, help="validated prior topology cache to copy into a new run")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--warmups", type=int, default=0)
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--ros-domain-id", type=int, default=92)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    if args.mode == "convert":
        if args.pdmap is None and args.extracted_dir is None:
            raise SystemExit("convert mode requires --pdmap or --extracted-dir")
        digest = sha256_file(args.pdmap) if args.pdmap else ""
        convert_pdmap(
            pdmap=args.pdmap, extracted_dir=args.extracted_dir,
            source_pdmap_hash=digest, output_dir=args.output_dir,
        )
    elif args.mode == "synthetic-smoke":
        run_synthetic_smoke(args.output_dir.resolve(), args.config.resolve())
    else:
        if args.extracted_dir is None or args.semantic_map is None:
            raise SystemExit("real-ab mode requires --extracted-dir and --semantic-map")
        run_real_ab(
            extracted_dir=args.extracted_dir.resolve(), semantic_map_path=args.semantic_map.resolve(),
            output=args.output_dir.resolve(), config_path=args.config.resolve(),
            warmups=args.warmups, repetitions=args.repetitions,
            ros_domain_id=args.ros_domain_id,
            topology_cache=args.topology_cache.resolve() if args.topology_cache else None,
        )
    print(f"2A-V2 output: {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
