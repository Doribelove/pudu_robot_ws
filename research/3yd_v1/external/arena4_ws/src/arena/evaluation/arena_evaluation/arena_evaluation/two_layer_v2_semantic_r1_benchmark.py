"""PLN-02 2A-V2/r1 static semantic ablation and diagnostic runner.

r1 keeps the frozen r0/run12 evidence read-only.  It separates route selection,
L1 semantics, L3 hard/class/regional semantics, audit and relaxation; records
every R0..R4 attempt; and evaluates all arms with the same semantic auditor.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import os
import resource
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml
from scipy import ndimage

from . import l1_l3_corridor_hybrid_smoke as r2_runtime
from . import path_audit
from . import unified_four_backends_smoke as legacy
from .edge_semantic_annotator import EdgeSemanticAnnotator, SemanticEdgeRouter, topology_graph_hash
from .pdmap_semantic_converter import convert_pdmap
from .planner_benchmark.map_utils import HospitalMap, sha256_file
from .planner_benchmark.models import Query
from .regional_preference_r1 import (
    RegionalPreferenceBuilderR1, expand_roi_to_route_lane_instances,
    orient_route_for_query,
)
from .semantic_costmap_composer import SemanticCostmapComposer
from .semantic_map import SemanticFeature, SemanticMapV1, canonical_hash
from .semantic_path_audit import SemanticPathAuditor
from .semantic_query_set import generate_query_set, save_query_set
from .semantic_rasterizer import SemanticRasterizer, grid_hash
from .semantic_smac_session import SemanticSmacSession
from .topology import load_topology, save_topology
from .two_layer_v2_semantic_benchmark import (
    _context, _git_state, _refuse_nonempty, _semantic_selector,
    _weighted_astar, _write_csv,
)


ARCHITECTURE_ID = "2A-V2"
IMPLEMENTATION_REVISION = "r1"
PARENT_ARCHITECTURE = "2A-V2-r0"
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config/two_layer_v2_semantic_r1.yaml"
ROOT = Path(__file__).resolve().parents[7]
ARM_ORDER = ("E0", "E1", "E2", "E3", "E4")
ARM_COLORS = {
    "E0": (220, 80, 40), "E1": (220, 190, 20), "E2": (40, 180, 40),
    "E3": (20, 130, 240), "E4": (20, 20, 230),
}


@dataclass(frozen=True)
class ArmSwitches:
    route_selector: str
    l1_semantic_costs: bool
    l1_hard_semantics: bool
    l3_hard_semantics: bool
    l3_soft_class_costs: bool
    regional_preference: bool
    semantic_audit: bool
    relaxation: str
    fixed_l1_route_from: str = ""

    @classmethod
    def parse(cls, value: Mapping[str, Any]) -> "ArmSwitches":
        result = cls(**{
            key: value.get(key, "" if key == "fixed_l1_route_from" else None)
            for key in cls.__dataclass_fields__
        })
        if result.route_selector not in {"legacy", "multi_source"}:
            raise ValueError(f"invalid route selector: {result.route_selector}")
        if result.relaxation not in {"strict", "graceful"}:
            raise ValueError(f"invalid relaxation mode: {result.relaxation}")
        if result.semantic_audit is not True:
            raise ValueError("r1 semantic_audit must be enabled for every arm")
        for key in (
            "l1_semantic_costs", "l1_hard_semantics", "l3_hard_semantics",
            "l3_soft_class_costs", "regional_preference", "semantic_audit",
        ):
            if not isinstance(getattr(result, key), bool):
                raise ValueError(f"arm switch {key} must be boolean")
        return result

    @property
    def hash(self) -> str:
        return canonical_hash(asdict(self))


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


def _sum_timing(records: Sequence[Mapping[str, Any]], key: str) -> float:
    return float(sum(float(record.get("timing", {}).get(key) or 0.0) for record in records))


def _current_rss_bytes() -> Optional[int]:
    """Read resident memory, distinct from the monotonic ru_maxrss watermark."""
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) * 1024
    except (OSError, ValueError, IndexError):
        return None
    return None


def _path_hash(route: Any) -> str:
    return canonical_hash({
        "node_ids": list(route.node_ids), "edge_ids": list(route.edge_ids),
        "polyline": [[float(p[0]), float(p[1])] for p in route.polyline],
    })


def _validate_protocol(config: Mapping[str, Any]) -> None:
    if config.get("architecture_id") != ARCHITECTURE_ID or config.get("implementation_revision") != IMPLEMENTATION_REVISION:
        raise ValueError("r1 architecture identity mismatch")
    protocol = config["protocol"]
    required = {
        "resolution_m": 0.05, "dynamic_obstacles": False,
        "allow_reverse": False, "allow_in_place_rotation": False,
        "minimum_turning_radius_m": 0.40, "maximum_curvature_1pm": 2.50,
        "hard_semantic_mask": "footprint_expanded",
    }
    for key, expected in required.items():
        if protocol.get(key) != expected:
            raise ValueError(f"immutable protocol mismatch for {key}: {protocol.get(key)!r}")


def _load_config(
    config_path: Path,
    preference_policy_overrides: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    config = copy.deepcopy(yaml.safe_load(config_path.read_text()) or {})
    if preference_policy_overrides:
        regional = config.setdefault("regional_preference", {})
        unknown = sorted(set(preference_policy_overrides) - set(regional))
        if unknown:
            raise ValueError(f"unknown regional preference overrides: {unknown}")
        regional.update(dict(preference_policy_overrides))
        lane_cap = int(regional.get("lane_cost_cap", 64))
        parking_cap = int(regional.get("parking_cost_cap", 48))
        if not (1 <= lane_cap < 200 and 1 <= parking_cap < 200):
            raise ValueError("comfort preference caps must remain in [1, 199]")
    _validate_protocol(config)
    return config


def _prepare(
    extracted_dir: Path, semantic_map_path: Path, topology_cache: Path,
    config: Mapping[str, Any], *, output: Optional[Path] = None,
) -> Tuple[Any, Any, Any, Any, Any, Any, Any, float, float, float]:
    raster_started = time.monotonic_ns()
    ctx = _context((extracted_dir / "optemap.yaml").resolve())
    semantic_map = SemanticMapV1.load(semantic_map_path)
    semantic_map.validate_against_map(ctx.hospital_map)
    raster = SemanticRasterizer(
        footprint=config["protocol"]["footprint"],
        safety_margin_m=float(config["protocol"]["semantic_safety_margin_m"]),
    ).rasterize(semantic_map, hospital_map=ctx.hospital_map)
    raster_ms = (time.monotonic_ns() - raster_started) / 1.0e6
    topology_started = time.monotonic_ns()
    topology = load_topology(
        topology_cache, ctx.hospital_map, config["protocol"]["footprint"],
        padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    topology_ms = (time.monotonic_ns() - topology_started) / 1.0e6
    graph_hash = topology_graph_hash(topology)
    annotator = EdgeSemanticAnnotator(
        ctx.hospital_map, semantic_map, raster, base_map_hash=ctx.map_sha256,
        topology_hash=graph_hash, policy=config["l1_edge_cost"],
    )
    edge_started = time.monotonic_ns()
    annotator.precompute(topology.graph.edges)
    edge_ms = (time.monotonic_ns() - edge_started) / 1.0e6
    router = SemanticEdgeRouter(topology, annotator)
    queries, intents, query_metadata = generate_query_set(
        ctx.hospital_map, topology.free_mask, topology.free_components, raster,
        seed=int(config["experiment"]["seed"]),
    )
    if output is not None:
        raster.save(output / "semantic_raster.npz")
        save_topology(topology, output / "topology_cache")
        save_query_set(output / "real_query_set.yaml", queries, intents, query_metadata)
    return ctx, semantic_map, raster, topology, annotator, router, (queries, intents, query_metadata), raster_ms, topology_ms, edge_ms


def _selector_for_arm(
    switches: ArmSwitches, topology: Any, router: SemanticEdgeRouter,
    state: Dict[str, Any], fixed_route_hash: Optional[str] = None, *,
    preferred_attachment_radius_m: float = 2.0,
    attachment_cost_weight: float = 4.0,
):
    class _NeutralAnnotation:
        def __init__(self, edge: Any, reversed_traversal: bool) -> None:
            self.edge = edge
            self.blocked = False
            self.total_cost = float(edge.length_m)
            self.reversed_traversal = bool(reversed_traversal)

        def to_dict(self) -> Dict[str, Any]:
            return {
                "edge_id": int(self.edge.edge_id),
                "traversal_reversed": self.reversed_traversal,
                "base_length_m": float(self.edge.length_m),
                "total_cost": float(self.edge.length_m),
                "blocked": False,
                "policy": "neutral_endpoint_weighted",
            }

    class _NeutralAnnotator:
        policy_hash = "r1-neutral-endpoint-weighted-v1"

        @staticmethod
        def annotate(edge: Any, reversed_traversal: bool = False) -> Any:
            return _NeutralAnnotation(edge, reversed_traversal)

    neutral_router = SemanticEdgeRouter(topology, _NeutralAnnotator())

    def candidates_and_costs(
        _topology: Any, query: Query, selected_timing: Dict[str, Any],
    ) -> Tuple[List[Any], List[Any], List[Tuple[List[Any], List[Any]]], Dict[int, float], Dict[int, float]]:
        start_timing: Dict[str, Any] = {}
        goal_timing: Dict[str, Any] = {}
        starts = r2_runtime._attachment_candidates(
            _topology, query.start, cache_mode=r2_runtime.CACHE_MODE_OPTIMIZED,
            timing=start_timing,
        )
        goals = r2_runtime._attachment_candidates(
            _topology, query.goal, cache_mode=r2_runtime.CACHE_MODE_OPTIMIZED,
            timing=goal_timing,
        )
        start_distances = {
            int(item.node_id): math.hypot(float(item.x) - float(query.start[0]), float(item.y) - float(query.start[1]))
            for item in starts
        }
        goal_distances = {
            int(item.node_id): math.hypot(float(item.x) - float(query.goal[0]), float(item.y) - float(query.goal[1]))
            for item in goals
        }
        near_starts = [item for item in starts if start_distances[int(item.node_id)] <= preferred_attachment_radius_m]
        near_goals = [item for item in goals if goal_distances[int(item.node_id)] <= preferred_attachment_radius_m]
        stages: List[Tuple[List[Any], List[Any]]] = []
        if near_starts and near_goals:
            stages.append((near_starts, near_goals))
        if not stages or len(near_starts) != len(starts) or len(near_goals) != len(goals):
            stages.append((starts, goals))
        selected_timing.update({
            "start_lookup_ms": float(start_timing.get("lookup_ms", 0.0)),
            "goal_lookup_ms": float(goal_timing.get("lookup_ms", 0.0)),
            "start_collision_check_ms": float(start_timing.get("collision_check_ms", 0.0)),
            "goal_collision_check_ms": float(goal_timing.get("collision_check_ms", 0.0)),
            "start_candidate_count": len(starts), "goal_candidate_count": len(goals),
            "preferred_start_candidate_count": len(near_starts),
            "preferred_goal_candidate_count": len(near_goals),
            "preferred_attachment_radius_m": float(preferred_attachment_radius_m),
            "attachment_cost_weight": float(attachment_cost_weight),
            "endpoint_spatial_index_cache_hit": bool(
                start_timing.get("spatial_index_cache_hit", False)
                and goal_timing.get("spatial_index_cache_hit", False)
            ),
            "endpoint_candidate_cache_hit": bool(
                start_timing.get("endpoint_candidate_cache_hit", False)
                and goal_timing.get("endpoint_candidate_cache_hit", False)
            ),
        })
        return (
            starts, goals, stages,
            {key: attachment_cost_weight * value for key, value in start_distances.items()},
            {key: attachment_cost_weight * value for key, value in goal_distances.items()},
        )

    def neutral_multi_source(_topology: Any, query: Query, timing: Dict[str, Any]):
        starts, goals, stages, start_costs, goal_costs = candidates_and_costs(_topology, query, timing)
        route = None
        start_id = goal_id = None
        attempts = 0
        search_started = time.monotonic_ns()
        for stage_starts, stage_goals in stages:
            attempts += 1
            selected = neutral_router.search_any(
                [item.node_id for item in stage_starts], [item.node_id for item in stage_goals],
                start_costs=start_costs, goal_costs=goal_costs,
            )
            if selected is None:
                route, start_id, goal_id = None, None, None
            else:
                route, start_id, goal_id = selected
            if route is not None:
                break
        timing.update({
            "adjacency_build_ms": 0.0,
            "route_search_ms": (time.monotonic_ns() - search_started) / 1.0e6,
            "route_construction_ms": 0.0,
            "candidate_pair_attempts": attempts,
            "topology_adjacency_cache_hit": _topology.graph.adjacency_cache_hit,
            "route_cache_hit": False,
        })
        starts_by_id = {int(item.node_id): item for item in starts}
        goals_by_id = {int(item.node_id): item for item in goals}
        return (
            starts_by_id.get(int(start_id)) if start_id is not None else None,
            goals_by_id.get(int(goal_id)) if goal_id is not None else None,
            route,
            "multi_source_neutral_route" if route is not None else "multi_source_neutral_no_route",
        )

    def semantic_multi_source(_topology: Any, query: Query, timing: Dict[str, Any]):
        started = time.monotonic_ns()
        starts, goals, stages, start_costs, goal_costs = candidates_and_costs(_topology, query, timing)
        selected = None
        attempts = 0
        for stage_starts, stage_goals in stages:
            attempts += 1
            selected = router.search_any(
                [item.node_id for item in stage_starts], [item.node_id for item in stage_goals],
                start_costs=start_costs, goal_costs=goal_costs,
            )
            if selected is not None:
                break
        timing.update({
            "adjacency_build_ms": 0.0,
            "route_search_ms": (time.monotonic_ns() - started) / 1.0e6,
            "route_construction_ms": 0.0,
            "candidate_pair_attempts": attempts,
            "topology_adjacency_cache_hit": _topology.graph.adjacency_cache_hit,
            "route_cache_hit": False,
        })
        starts_by_id = {int(item.node_id): item for item in starts}
        goals_by_id = {int(item.node_id): item for item in goals}
        if selected is None:
            return None, None, None, "semantic_multi_source_no_route"
        route, start_id, goal_id = selected
        return starts_by_id[start_id], goals_by_id[goal_id], route, "semantic_multi_source_route"

    def select(_topology: Any, query: Query, *, cache_mode: str, timing: Optional[Dict[str, Any]] = None):
        selected_timing: Dict[str, Any] = {} if timing is None else timing
        if switches.l1_semantic_costs or switches.l1_hard_semantics:
            start, goal, route, reason = semantic_multi_source(_topology, query, selected_timing)
            selector_actual = "multi_source_semantic"
        elif switches.route_selector == "multi_source":
            start, goal, route, reason = neutral_multi_source(_topology, query, selected_timing)
            selector_actual = "multi_source_neutral"
        else:
            actual_mode = r2_runtime.CACHE_MODE_BASELINE
            start, goal, route, reason = r2_runtime._select_route_with_endpoint_attach(
                _topology, query, cache_mode=actual_mode, timing=selected_timing,
            )
            selector_actual = "legacy_pairwise"
        if route is not None:
            route, orientation = orient_route_for_query(route, query)
            route_hash = _path_hash(route)
            if fixed_route_hash is not None and route_hash != fixed_route_hash:
                raise RuntimeError(f"FIXED_L1_ROUTE_MISMATCH: {route_hash} != {fixed_route_hash}")
            state.update({
                "route": route, "route_hash": route_hash,
                "route_orientation": orientation, "route_selector_actual": selector_actual,
                "route_reason": reason,
            })
        else:
            state.update({"route": None, "route_selector_actual": selector_actual, "route_reason": reason})
        return start, goal, route, reason

    return select


def _relaxation_levels(switches: ArmSwitches, config: Mapping[str, Any]) -> List[str]:
    return list(config["preference_relaxation"]["levels"]) if switches.relaxation == "graceful" else ["R0"]


def _attempt_timing(diagnostics: Mapping[str, Any], state: Mapping[str, Any], wall_ms: float, audit_ms: float) -> Dict[str, float]:
    publish_ms = sum(float(diagnostics.get(key) or 0.0) for key in (
        "local_map_serialization_ms", "local_map_publication_ms",
        "costmap_ack_repair_serialization_ms", "costmap_ack_repair_publication_ms",
    ))
    result = {
        "wall_ms": float(wall_ms),
        "l1_ms": float(diagnostics.get("l1_graph_search_ms") or 0.0),
        "roi_build_ms": float(state.get("roi_build_ms") or 0.0),
        "field_build_ms": float(state.get("field_build_ms") or 0.0),
        "compose_ms": float(state.get("compose_ms") or 0.0),
        "publish_ms": publish_ms,
        "ack_wait_ms": float(diagnostics.get("costmap_ack_wait_ms") or 0.0),
        "smac_ms": float(diagnostics.get("l3_action_wall_ms") or 0.0),
        "planner_reported_ms": float(diagnostics.get("l3_planning_time_ms") or diagnostics.get("planning_time_ms") or 0.0),
        "audit_ms": float(audit_ms),
    }
    accounted = sum(result[key] for key in (
        "l1_ms", "roi_build_ms", "field_build_ms", "compose_ms", "publish_ms",
        "ack_wait_ms", "smac_ms", "audit_ms",
    ))
    result["unaccounted_process_ms"] = max(0.0, float(wall_ms) - accounted)
    return result


def _summaries(
    rows: Sequence[Mapping[str, Any]],
    arms: Optional[Sequence[str]] = None,
    *,
    p99_minimum_effective_samples: int = 100,
) -> List[Dict[str, Any]]:
    summaries = []
    selected_arms = list(arms) if arms is not None else [
        arm for arm in ARM_ORDER if any(row.get("arm") == arm for row in rows)
    ]
    for arm in selected_arms:
        selected = [r for r in rows if r["arm"] == arm and r["run_mode"] == "measured"]
        valid = [r for r in selected if r.get("final_valid_success") is True]
        levels = {level: sum(r.get("final_valid_success") is True and r.get("relaxation_level") == level for r in selected) for level in ("R0", "R1", "R2", "R3", "R4")}
        failures: Dict[str, int] = {}
        for row in selected:
            if row.get("failure_code"):
                failures[str(row["failure_code"])] = failures.get(str(row["failure_code"]), 0) + 1
        exact = sum(int(
            r.get("cumulative_costmap_ack_soft_exact_mismatch_cells")
            if r.get("cumulative_costmap_ack_soft_exact_mismatch_cells") is not None
            else r.get("costmap_ack_soft_exact_mismatch_cells") or 0
        ) for r in selected)
        checked = sum(int(
            r.get("cumulative_costmap_ack_soft_checked_cells")
            if r.get("cumulative_costmap_ack_soft_checked_cells") is not None
            else r.get("costmap_ack_soft_checked_cells") or 0
        ) for r in selected)
        summaries.append({
            "arm": arm, "request_count": len(selected), "success_count": len(valid),
            "success_rate": len(valid) / len(selected) if selected else 0.0,
            "r0_success_count": sum(r.get("final_valid_success") is True and r.get("relaxation_level") == "R0" for r in selected),
            "relaxation_trigger_rate": sum(len(r.get("attempt_records") or []) > 1 for r in selected) / len(selected) if selected else 0.0,
            "success_by_relaxation_level": levels,
            "cumulative_request_wall_ms": _percentiles(selected, "cumulative_request_wall_ms"),
            "final_attempt_wall_ms": _percentiles(selected, "final_attempt_wall_ms"),
            "cumulative_l1_ms": _percentiles(selected, "cumulative_l1_ms"),
            "cumulative_roi_build_ms": _percentiles(selected, "cumulative_roi_build_ms"),
            "cumulative_field_build_ms": _percentiles(selected, "cumulative_field_build_ms"),
            "cumulative_compose_ms": _percentiles(selected, "cumulative_compose_ms"),
            "cumulative_publish_ms": _percentiles(selected, "cumulative_publish_ms"),
            "cumulative_ack_wait_ms": _percentiles(selected, "cumulative_ack_wait_ms"),
            "cumulative_smac_ms": _percentiles(selected, "cumulative_smac_ms"),
            "cumulative_audit_ms": _percentiles(selected, "cumulative_audit_ms"),
            "cumulative_unaccounted_process_ms": _percentiles(
                selected, "cumulative_unaccounted_process_ms"
            ),
            "path_length_m": _percentiles(valid, "path_length_m"),
            "curvature_p95": _percentiles(valid, "curvature_p95"),
            "lane_correct_side_ratio": _percentiles(valid, "lane_correct_side_ratio"),
            "lane_error_p50_m": _percentiles(valid, "base_center_to_right_boundary_error_p50_m"),
            "parking_center_p50": _percentiles(valid, "parking_center_normalized_deviation_p50"),
            "peak_rss_bytes": max((int(r.get("peak_rss_bytes") or 0) for r in selected), default=0),
            "current_rss_bytes": _percentiles(selected, "current_rss_bytes"),
            "hard_semantic_violation_count": sum(int(r.get("hard_semantic_violation_count") or 0) for r in valid),
            "collision_violation_count": sum(int(r.get("collision_violation_count") or 0) for r in valid),
            "kinematic_violation_count": sum(int(r.get("kinematic_violation_count") or 0) for r in valid),
            "no_stopping_goal_violation_count": sum(bool(r.get("no_stopping_goal_violation")) for r in valid),
            "hard_constraints_not_applicable_count": sum(r.get("hard_constraints_held") is None for r in selected),
            "costmap_ack_hard_mismatch_cells": sum(int(
                r.get("cumulative_costmap_ack_hard_mismatch_cells")
                if r.get("cumulative_costmap_ack_hard_mismatch_cells") is not None
                else r.get("costmap_ack_hard_mismatch_cells") or 0
            ) for r in selected),
            "costmap_ack_soft_bound_mismatch_cells": sum(int(
                r.get("cumulative_costmap_ack_soft_mismatch_cells")
                if r.get("cumulative_costmap_ack_soft_mismatch_cells") is not None
                else r.get("costmap_ack_soft_mismatch_cells") or 0
            ) for r in selected),
            "costmap_ack_soft_exact_mismatch_cells": exact,
            "costmap_ack_soft_checked_cells": checked,
            "costmap_ack_soft_exact_mismatch_ratio": float(exact / checked) if checked else 0.0,
            "percentile_interpretation": "debug_only" if len(selected) < 20 else "measured_distribution",
            "p99_valid": len(selected) >= int(p99_minimum_effective_samples),
            "failure_codes": failures,
        })
    return summaries


def _paired(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    measured = [r for r in rows if r["run_mode"] == "measured"]
    by_key = {(r["arm"], r["query_id"], r["repetition"]): r for r in measured}
    result = []
    for row in measured:
        if row["arm"] == "E0":
            continue
        base = by_key.get(("E0", row["query_id"], row["repetition"]))
        pair_valid = bool(base and base.get("final_valid_success") is True and row.get("final_valid_success") is True)
        item = {
            "query_id": row["query_id"], "repetition": row["repetition"], "arm": row["arm"],
            "e0_success": bool(base and base.get("final_valid_success") is True),
            "arm_success": row.get("final_valid_success") is True, "paired_valid": pair_valid,
        }
        for metric in (
            "path_length_m", "curvature_p95", "lane_correct_side_ratio",
            "base_center_to_right_boundary_error_p50_m", "parking_center_normalized_deviation_p50",
        ):
            item[f"e0_{metric}"] = None if base is None else base.get(metric)
            item[f"arm_{metric}"] = row.get(metric)
            try:
                item[f"delta_{metric}"] = float(row.get(metric)) - float(base.get(metric)) if pair_valid else None
            except (TypeError, ValueError):
                item[f"delta_{metric}"] = None
        result.append(item)
    return result


def _draw_facets(ctx: Any, raster: Any, queries: Sequence[Query], rows: Sequence[Mapping[str, Any]], paths: Mapping[Tuple[str, str, int], Sequence[Mapping[str, Any]]], output: Path) -> None:
    facets = output / "facets"
    facets.mkdir(exist_ok=True)
    base = cv2.imread(str(ctx.hospital_map.image_path), cv2.IMREAD_GRAYSCALE)
    semantic_overlay = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)
    lane = np.asarray(raster.masks.get("lane", np.zeros(base.shape, bool)), bool)
    junction = np.asarray(raster.masks.get("junction_area", np.zeros(base.shape, bool)), bool)
    parking = np.asarray(raster.masks.get("parking_area", np.zeros(base.shape, bool)), bool)
    semantic_overlay[lane] = (180, 225, 180)
    semantic_overlay[junction] = (225, 225, 160)
    semantic_overlay[parking] = (225, 180, 220)
    measured = {(r["arm"], r["query_id"], int(r["repetition"])): r for r in rows if r["run_mode"] == "measured"}
    rendered = []
    for query in queries:
        cells = []
        for arm in ARM_ORDER:
            for point in paths.get((arm, query.query_id, 1), []):
                cell = ctx.hospital_map.world_to_cell(float(point["x"]), float(point["y"]))
                if cell is not None:
                    cells.append(cell)
        for pose in (query.start, query.goal):
            cell = ctx.hospital_map.world_to_cell(float(pose[0]), float(pose[1]))
            if cell is not None:
                cells.append(cell)
        if not cells:
            continue
        arr = np.asarray(cells)
        margin = 80
        r0, r1 = max(0, int(arr[:, 0].min()) - margin), min(base.shape[0], int(arr[:, 0].max()) + margin + 1)
        c0, c1 = max(0, int(arr[:, 1].min()) - margin), min(base.shape[1], int(arr[:, 1].max()) + margin + 1)
        panel = semantic_overlay[r0:r1, c0:c1].copy()
        for arm in ARM_ORDER:
            points = paths.get((arm, query.query_id, 1), [])
            line = []
            for point in points:
                cell = ctx.hospital_map.world_to_cell(float(point["x"]), float(point["y"]))
                if cell is not None:
                    line.append((cell[1] - c0, cell[0] - r0))
            if len(line) >= 2:
                cv2.polylines(panel, [np.asarray(line, np.int32)], False, ARM_COLORS[arm], 2, cv2.LINE_AA)
                stride = max(1, len(line) // 8)
                for index in range(stride, len(line), stride):
                    p0, p1 = line[max(0, index - stride // 3)], line[index]
                    cv2.arrowedLine(panel, p0, p1, ARM_COLORS[arm], 1, cv2.LINE_AA, tipLength=0.25)
        start = ctx.hospital_map.world_to_cell(float(query.start[0]), float(query.start[1]))
        goal = ctx.hospital_map.world_to_cell(float(query.goal[0]), float(query.goal[1]))
        if start is not None:
            cv2.circle(panel, (start[1] - c0, start[0] - r0), 7, (255, 255, 255), -1)
            cv2.circle(panel, (start[1] - c0, start[0] - r0), 7, (0, 0, 0), 2)
        if goal is not None:
            cv2.drawMarker(panel, (goal[1] - c0, goal[0] - r0), (0, 0, 0), cv2.MARKER_TILTED_CROSS, 16, 3)
        header = np.full((125, max(600, panel.shape[1]), 3), 245, np.uint8)
        cv2.putText(header, query.query_id, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 1, cv2.LINE_AA)
        for index, arm in enumerate(ARM_ORDER):
            row = measured.get((arm, query.query_id, 1), {})
            label = f"{arm} {row.get('relaxation_level','-')} {'OK' if row.get('final_valid_success') else row.get('failure_code','FAIL')}"
            y = 42 + index * 16
            cv2.line(header, (8, y - 4), (30, y - 4), ARM_COLORS[arm], 3)
            cv2.putText(header, label, (36, y), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 0, 0), 1, cv2.LINE_AA)
        if panel.shape[1] < header.shape[1]:
            panel = cv2.copyMakeBorder(panel, 0, 0, 0, header.shape[1] - panel.shape[1], cv2.BORDER_CONSTANT, value=(128, 128, 128))
        elif panel.shape[1] > header.shape[1]:
            header = cv2.copyMakeBorder(header, 0, 0, 0, panel.shape[1] - header.shape[1], cv2.BORDER_CONSTANT, value=(245, 245, 245))
        combined = np.vstack((header, panel))
        target = facets / f"{query.query_id}.png"
        cv2.imwrite(str(target), combined)
        rendered.append(combined)
    if rendered:
        thumbs = [cv2.resize(img, (600, 500), interpolation=cv2.INTER_AREA) for img in rendered]
        blank = np.full_like(thumbs[0], 128)
        while len(thumbs) % 2:
            thumbs.append(blank.copy())
        sheet = np.vstack([np.hstack(thumbs[i:i + 2]) for i in range(0, len(thumbs), 2)])
        cv2.imwrite(str(output / "paired_query_facets.png"), sheet)


def _draw_direction_panel(
    ctx: Any, query: Query, route: Any, allowed: np.ndarray,
    field: Any, diagnostics: Mapping[str, Any], output: Path,
) -> np.ndarray:
    cells = np.argwhere(allowed)
    margin = 30
    row0 = max(0, int(cells[:, 0].min()) - margin)
    row1 = min(allowed.shape[0], int(cells[:, 0].max()) + margin + 1)
    col0 = max(0, int(cells[:, 1].min()) - margin)
    col1 = min(allowed.shape[1], int(cells[:, 1].max()) + margin + 1)
    base = cv2.imread(str(ctx.hospital_map.image_path), cv2.IMREAD_GRAYSCALE)[row0:row1, col0:col1]
    panel = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)
    finite = np.isfinite(field.lane_error_m[row0:row1, col0:col1]) & allowed[row0:row1, col0:col1]
    cost = field.cost[row0:row1, col0:col1]
    heat = cv2.applyColorMap(np.clip(cost.astype(np.float32) * 255.0 / 64.0, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    blended = cv2.addWeighted(panel, 0.55, heat, 0.45, 0.0)
    panel[finite] = blended[finite]
    line = []
    for point in route.polyline:
        cell = ctx.hospital_map.world_to_cell(float(point[0]), float(point[1]))
        if cell is not None:
            line.append((cell[1] - col0, cell[0] - row0))
    if len(line) >= 2:
        cv2.polylines(panel, [np.asarray(line, np.int32)], False, (255, 255, 255), 2, cv2.LINE_AA)
        stride = max(1, len(line) // 12)
        for index in range(stride, len(line), stride):
            cv2.arrowedLine(panel, line[max(0, index - max(1, stride // 3))], line[index], (0, 0, 0), 2, cv2.LINE_AA, tipLength=0.3)
    start = ctx.hospital_map.world_to_cell(float(query.start[0]), float(query.start[1]))
    goal = ctx.hospital_map.world_to_cell(float(query.goal[0]), float(query.goal[1]))
    if start is not None:
        cv2.circle(panel, (start[1] - col0, start[0] - row0), 8, (255, 255, 255), -1)
        cv2.circle(panel, (start[1] - col0, start[0] - row0), 8, (0, 0, 0), 2)
    if goal is not None:
        cv2.drawMarker(panel, (goal[1] - col0, goal[0] - row0), (0, 0, 0), cv2.MARKER_TILTED_CROSS, 18, 3)
    width = max(700, panel.shape[1])
    header = np.full((64, width, 3), 245, np.uint8)
    cv2.putText(header, query.query_id, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 0), 1, cv2.LINE_AA)
    label = (
        f"route_reversed={diagnostics.get('route_reversed_for_query')}  "
        f"correct={diagnostics.get('preferred_correct_side_ratio', 0):.3f}  "
        f"error_p50={diagnostics.get('preferred_lane_error_p50_m', 0):.3f}m"
    )
    cv2.putText(header, label, (8, 48), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 0, 0), 1, cv2.LINE_AA)
    if panel.shape[1] < width:
        panel = cv2.copyMakeBorder(panel, 0, 0, 0, width - panel.shape[1], cv2.BORDER_CONSTANT, value=(128, 128, 128))
    result = np.vstack((header, panel))
    cv2.imwrite(str(output), result)
    return result


def run_synthetic_smoke(
    *, output: Path, config_path: Path,
    preference_policy_overrides: Optional[Mapping[str, Any]] = None,
) -> Path:
    """Exercise r1 direction reversal, caps and hard invariants without ROS."""
    _refuse_nonempty(output)
    output.mkdir(parents=True)
    config = _load_config(config_path, preference_policy_overrides)
    height, width, resolution = 100, 180, 0.05
    occupancy = np.zeros((height, width), dtype=np.int8)
    occupancy[[0, -1], :] = 100
    occupancy[:, [0, -1]] = 100
    distance = ndimage.distance_transform_edt(
        occupancy == 0, sampling=resolution,
    ).astype(np.float32)
    hospital_map = HospitalMap(
        Path("synthetic.yaml"), Path("synthetic.pgm"), resolution,
        (0.0, 0.0, 0.0), width, height, occupancy, distance,
    )
    definitions = (
        ("lane", "lane", [[0.5, 1.0], [8.4, 1.0], [8.4, 4.0], [0.5, 4.0], [0.5, 1.0]], False, True, 60),
        ("junction", "junction_area", [[4.0, 1.0], [4.8, 1.0], [4.8, 4.0], [4.0, 4.0], [4.0, 1.0]], False, True, 80),
        ("parking", "parking_area", [[7.0, 1.2], [8.3, 1.2], [8.3, 3.8], [7.0, 3.8], [7.0, 1.2]], False, True, 70),
        ("forbidden", "forbidden", [[5.4, 2.0], [5.9, 2.0], [5.9, 3.0], [5.4, 3.0], [5.4, 2.0]], True, False, 100),
    )
    features = [SemanticFeature(
        semantic_id, semantic_class, "polygon", points,
        hard=hard, soft=soft,
        direction_rule="route_tangent_right" if semantic_class == "lane" else "none",
        priority=priority, source_field=f"synthetic.{semantic_id}",
    ) for semantic_id, semantic_class, points, hard, soft, priority in definitions]
    semantic_map = SemanticMapV1(
        "map", resolution, (0.0, 0.0, 0.0), width, height, "synthetic-r1",
        features=features, traffic_rules={"right_hand_drive": True},
    )
    raster = SemanticRasterizer(
        footprint=config["protocol"]["footprint"], safety_margin_m=0.05,
    ).rasterize(semantic_map, hospital_map=hospital_map)
    builder = RegionalPreferenceBuilderR1(
        hospital_map, raster, policy=config["regional_preference"],
        semantic_map=semantic_map,
    )
    composer = SemanticCostmapComposer(policy=config["l3_soft_cost"])
    route = [[0.8, 2.5], [3.8, 2.5], [5.0, 2.5], [8.0, 2.5]]
    reverse = list(reversed(route))
    allowed = occupancy == 0
    forward = builder.build(route, goal=route[-1], allowed_mask=allowed)
    backward = builder.build(reverse, goal=reverse[-1], allowed_mask=allowed)
    forward_map = composer.compose(
        occupancy, raster, forward, allowed_mask=allowed,
        hard_semantics_enabled=True, soft_class_costs_enabled=True,
        regional_preference_enabled=True, hard_semantics_use_footprint=True,
    )
    backward_map = composer.compose(
        occupancy, raster, backward, allowed_mask=allowed,
        hard_semantics_enabled=True, soft_class_costs_enabled=True,
        regional_preference_enabled=True, hard_semantics_use_footprint=True,
    )
    start = hospital_map.world_to_cell(*route[0])
    goal = hospital_map.world_to_cell(*route[-1])
    assert start is not None and goal is not None
    forward_path = _weighted_astar(forward_map.internal_cost, start, goal)
    backward_path = _weighted_astar(backward_map.internal_cost, goal, start)
    forward_target = np.isfinite(forward.lane_error_m) & (forward.lane_error_m <= 0.5)
    backward_target = np.isfinite(backward.lane_error_m) & (backward.lane_error_m <= 0.5)
    forward_rows = np.argwhere(forward_target)[:, 0]
    backward_rows = np.argwhere(backward_target)[:, 0]
    payload = {
        "schema_version": "2A-V2-r1-synthetic-smoke-v1",
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "forward_path_found": bool(forward_path),
        "reverse_path_found": bool(backward_path),
        "forward_target_row_p50": float(np.median(forward_rows)),
        "reverse_target_row_p50": float(np.median(backward_rows)),
        "right_side_flipped": abs(float(np.median(forward_rows)) - float(np.median(backward_rows))) > 10.0,
        "forward_target_correct_side_ratio": float(np.mean(forward.lane_correct_side[forward_target])),
        "reverse_target_correct_side_ratio": float(np.mean(backward.lane_correct_side[backward_target])),
        "forward_hard_overlap": sum(bool(raster.hard_footprint_mask[cell]) for cell in forward_path),
        "reverse_hard_overlap": sum(bool(raster.hard_footprint_mask[cell]) for cell in backward_path),
        "junction_cost_max": int(np.max(forward.cost[raster.masks["junction_area"]])),
        "regional_cost_max": int(max(np.max(forward.cost), np.max(backward.cost))),
        "composed_soft_cost_max": int(max(np.max(forward_map.soft_cost), np.max(backward_map.soft_cost))),
        "hard_cost": int(np.max(forward_map.internal_cost[raster.hard_mask])),
        "policy": config["regional_preference"],
    }
    payload["passed"] = bool(
        payload["forward_path_found"] and payload["reverse_path_found"]
        and payload["right_side_flipped"]
        and payload["forward_target_correct_side_ratio"] >= 0.8
        and payload["reverse_target_correct_side_ratio"] >= 0.8
        and payload["forward_hard_overlap"] == 0 and payload["reverse_hard_overlap"] == 0
        and payload["junction_cost_max"] == 0
        and payload["regional_cost_max"] <= int(config["regional_preference"]["lane_cost_cap"])
        and payload["composed_soft_cost_max"] < 200 and payload["hard_cost"] == 254
    )
    (output / "synthetic_smoke.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
    )
    image = np.full((height, width, 3), 255, dtype=np.uint8)
    image[occupancy == 100] = (0, 0, 0)
    image[raster.masks["lane"]] = (220, 250, 220)
    image[raster.masks["junction_area"]] = (240, 240, 170)
    image[raster.masks["parking_area"]] = (245, 210, 240)
    image[raster.hard_mask] = (30, 30, 220)
    for path, color in ((forward_path, (255, 50, 20)), (backward_path, (20, 20, 255))):
        if path:
            cv2.polylines(
                image, [np.asarray([[cell[1], cell[0]] for cell in path], np.int32)],
                False, color, 2,
            )
    cv2.imwrite(str(output / "synthetic_smoke.png"), image)
    if not payload["passed"]:
        raise RuntimeError(f"r1 synthetic semantic smoke failed: {payload}")
    return output


def run_offline_diagnostic(
    *, extracted_dir: Path, semantic_map_path: Path, topology_cache: Path,
    output: Path, config_path: Path, r0_results: Optional[Path] = None,
    preference_policy_overrides: Optional[Mapping[str, Any]] = None,
) -> Path:
    _refuse_nonempty(output)
    output.mkdir(parents=True)
    config = _load_config(config_path, preference_policy_overrides)
    ctx, semantic_map, raster, topology, annotator, router, query_bundle, raster_ms, topology_ms, edge_ms = _prepare(
        extracted_dir, semantic_map_path, topology_cache, config, output=output,
    )
    queries, _, metadata = query_bundle
    selector = _semantic_selector(topology, router)
    builder = RegionalPreferenceBuilderR1(
        ctx.hospital_map, raster, policy=config["regional_preference"], semantic_map=semantic_map,
    )
    auditor = SemanticPathAuditor(ctx.hospital_map, semantic_map, raster)
    records = []
    panels = []
    facet_dir = output / "direction_facets"
    facet_dir.mkdir()
    for query in queries:
        _, _, route, reason = selector(topology, query, cache_mode=r2_runtime.CACHE_MODE_OPTIMIZED, timing={})
        if route is None:
            records.append({"query_id": query.query_id, "route_found": False, "reason": reason})
            continue
        route, orientation = orient_route_for_query(route, query)
        allowed = r2_runtime._raw_corridor_mask(ctx, topology, route, query, float(config["roi"]["r0_padding_m"]))
        lane_roi_diagnostics: Dict[str, Any] = {}
        if bool(config["roi"].get("expand_route_lane_instances", True)):
            allowed, lane_roi_diagnostics = expand_roi_to_route_lane_instances(
                ctx.hospital_map, raster, semantic_map, route.polyline, allowed,
                free_mask=r2_runtime._raw_free_mask(ctx),
                route_probe_radius_m=float(config["roi"].get("lane_route_probe_radius_m", 0.50)),
            )
        field = builder.build(route.polyline, goal=query.goal, allowed_mask=allowed, route_diagnostics=orientation)
        preferred = np.isfinite(field.lane_error_m) & allowed & (field.lane_error_m <= 0.50)
        record: Dict[str, Any] = {
            "query_id": query.query_id, "category": query.category, "route_found": True,
            "route_hash": _path_hash(route), **orientation,
            "preferred_lane_cell_count": int(np.count_nonzero(preferred)),
            "preferred_correct_side_ratio": float(np.mean(field.lane_correct_side[preferred])) if np.any(preferred) else None,
            "preferred_lane_error_p50_m": float(np.median(field.lane_error_m[preferred])) if np.any(preferred) else None,
            "field_build_ms": field.diagnostics.get("field_build_ms"),
            "field_crop_cells": field.diagnostics.get("field_crop_cells"),
            "lane_segment_direction_stability": field.diagnostics.get("lane_segment_direction_stability"),
            **lane_roi_diagnostics,
        }
        if r0_results is not None:
            path_file = r0_results / "paths" / f"B_semantics_enabled_{query.query_id}_measured_1.json"
            if path_file.exists():
                points = json.loads(path_file.read_text())
                audited = auditor.audit(points, field, relaxation_level="R0", canonical_metrics={})
                record["r0_path_reaudited_with_r1_geometry"] = audited.to_dict()
        panels.append(_draw_direction_panel(
            ctx, query, route, allowed, field, record,
            facet_dir / f"{query.query_id}.png",
        ))
        records.append(record)
    targets = [r for r in records if r.get("query_id") in {"real-lane-forward", "real-lane-reverse"}]
    gate = bool(len(targets) == 2 and all(
        r.get("route_reversed_for_query") is False
        and (r.get("preferred_correct_side_ratio") or 0.0) >= 0.8
        and (r.get("preferred_lane_error_p50_m") or float("inf")) <= 0.5
        for r in targets
    ))
    payload = {
        "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "schema_version": "2A-V2-r1-offline-direction-diagnostic-v1",
        "offline_gate_passed": gate, "records": records,
        "cold_start": {"raster_ms": raster_ms, "topology_load_ms": topology_ms, "semantic_edge_precompute_ms": edge_ms},
        "query_set": metadata,
    }
    (output / "direction_diagnostics.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    if panels:
        thumbs = [cv2.resize(panel, (600, 500), interpolation=cv2.INTER_AREA) for panel in panels]
        blank = np.full_like(thumbs[0], 128)
        while len(thumbs) % 2:
            thumbs.append(blank.copy())
        sheet = np.vstack([np.hstack(thumbs[index:index + 2]) for index in range(0, len(thumbs), 2)])
        cv2.imwrite(str(output / "direction_query_facets.png"), sheet)
    if not gate:
        raise RuntimeError("r1 offline forward/reverse direction gate failed")
    return output


def run_real_ablation(
    *, extracted_dir: Path, semantic_map_path: Path, topology_cache: Path,
    output: Path, config_path: Path, warmups: int, repetitions: int,
    ros_domain_id: int, arms: Sequence[str] = ARM_ORDER,
    query_ids: Optional[Sequence[str]] = None,
    preference_policy_overrides: Optional[Mapping[str, Any]] = None,
) -> Path:
    _refuse_nonempty(output)
    output.mkdir(parents=True)
    (output / "paths").mkdir()
    config = _load_config(config_path, preference_policy_overrides)
    selected_arms = [arm for arm in ARM_ORDER if arm in set(arms)]
    if not selected_arms:
        raise ValueError("at least one E0..E4 arm is required")
    arm_switches = {name: ArmSwitches.parse(config["ablation_arms"][name]) for name in selected_arms}
    ctx, semantic_map, raster, topology, annotator, router, query_bundle, raster_ms, topology_ms, edge_ms = _prepare(
        extracted_dir, semantic_map_path, topology_cache, config, output=output,
    )
    queries, _, query_metadata = query_bundle
    if query_ids:
        requested_queries = set(query_ids)
        queries = [query for query in queries if query.query_id in requested_queries]
        missing_queries = sorted(requested_queries - {query.query_id for query in queries})
        if missing_queries:
            raise ValueError(f"unknown query ids: {missing_queries}")
    arm_routers: Dict[str, SemanticEdgeRouter] = {}
    for arm, switches in arm_switches.items():
        if not (switches.l1_semantic_costs or switches.l1_hard_semantics):
            continue
        if switches.l1_semantic_costs and switches.l1_hard_semantics:
            arm_routers[arm] = router
            continue
        arm_policy = {
            **dict(config["l1_edge_cost"]),
            "semantic_costs_enabled": switches.l1_semantic_costs,
            "hard_semantics_enabled": switches.l1_hard_semantics,
        }
        arm_annotator = EdgeSemanticAnnotator(
            ctx.hospital_map, semantic_map, raster, base_map_hash=ctx.map_sha256,
            topology_hash=topology_graph_hash(topology), policy=arm_policy,
        )
        arm_annotator.precompute(topology.graph.edges)
        arm_routers[arm] = SemanticEdgeRouter(topology, arm_annotator)
    canonical_auditor = path_audit.PathAuditor(ctx, source_commit=_git_state(ROOT)["commit"])
    semantic_auditor = SemanticPathAuditor(ctx.hospital_map, semantic_map, raster)
    preference_builder = RegionalPreferenceBuilderR1(
        ctx.hospital_map, raster, policy=config["regional_preference"], semantic_map=semantic_map,
    )
    composer = SemanticCostmapComposer(policy=config["l3_soft_cost"])
    spec = legacy.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"BACKEND_UNAVAILABLE: {spec.reason}")
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    session = SemanticSmacSession(
        ctx, output, map_yaml=ctx.map_yaml,
        log_tag=f"2a_v2_r1_{int(time.time())}", local_mask_updates=True,
        optimization_profile="v7_candidate", smac_parameter_profile="baseline",
        optimization_stage="step3_delta_map", enable_mask_reuse_noop=True,
        costmap_ack_timeout_s=float(config["roi"]["ack_timeout_s"]),
    )
    session.local_map_update_strategy = "roi_ack"
    rows: List[Dict[str, Any]] = []
    paths: Dict[Tuple[str, str, int], Sequence[Mapping[str, Any]]] = {}
    baseline_lengths: Dict[Tuple[str, int], float] = {}
    fixed_route_hashes: Dict[Tuple[str, int, str], str] = {}
    field_cache: Dict[Tuple[str, str, str], Any] = {}
    roi_cache: Dict[Tuple[str, str, float, bool], np.ndarray] = {}
    roi_diagnostics_cache: Dict[Tuple[str, str, float, bool], Dict[str, Any]] = {}
    session.start()
    try:
        for run_mode, count in (("warmup", warmups), ("measured", repetitions)):
            for repetition in range(1, count + 1):
                for query in queries:
                    # Bound cache lifetime to one query across all arms.  This
                    # reuses E2/E3/E4 geometry but cannot accumulate full-map
                    # arrays over the whole suite.
                    field_cache.clear()
                    roi_cache.clear()
                    roi_diagnostics_cache.clear()
                    for arm in selected_arms:
                        switches = arm_switches[arm]
                        request_started = time.monotonic_ns()
                        session.reset_query_state(f"{arm}:{query.query_id}", restore_base_map=False)
                        levels = _relaxation_levels(switches, config)
                        attempt_records: List[Dict[str, Any]] = []
                        final: Dict[str, Any] = {}
                        skip_soft_levels = False
                        for level in levels:
                            if skip_soft_levels and level in {"R1", "R2"}:
                                continue
                            attempt_started = time.monotonic_ns()
                            state: Dict[str, Any] = {}
                            fixed_from = str(switches.fixed_l1_route_from or "")
                            fixed_hash = fixed_route_hashes.get((run_mode, repetition, query.query_id)) if fixed_from else None
                            selector = _selector_for_arm(
                                switches, topology, arm_routers.get(arm, router), state, fixed_hash,
                                preferred_attachment_radius_m=float(
                                    config.get("endpoint_attachment", {}).get("preferred_radius_m", 2.0)
                                ),
                                attachment_cost_weight=float(
                                    config.get("endpoint_attachment", {}).get("cost_weight", 4.0)
                                ),
                            )
                            start_cell = ctx.hospital_map.world_to_cell(query.start[0], query.start[1])
                            goal_cell = ctx.hospital_map.world_to_cell(query.goal[0], query.goal[1])
                            preflight_code = ""
                            if start_cell is None or goal_cell is None:
                                preflight_code = "INVALID_ENDPOINT"
                            elif raster.no_stopping_mask[goal_cell]:
                                preflight_code = "NO_STOPPING_GOAL_VIOLATION"
                            elif raster.hard_footprint_mask[start_cell] or raster.hard_footprint_mask[goal_cell]:
                                preflight_code = "HARD_SEMANTIC_ENDPOINT"
                            if preflight_code:
                                record = {
                                    "arm": arm, "query_id": query.query_id, "relaxation_level": level,
                                    "success": False, "failure_code": preflight_code,
                                    "failure_stage": "preflight", "hard_constraints_held": None,
                                    "preflight_constraint_rejection": True,
                                    "timing": {"wall_ms": (time.monotonic_ns() - attempt_started) / 1.0e6},
                                }
                                attempt_records.append(record)
                                final = {"failure_code": preflight_code, "hard_constraints_held": None}
                                break
                            padding = float(config["roi"]["r3_padding_m"] if level == "R3" else config["roi"]["r0_padding_m"])

                            def mask_builder(_ctx: Any, _topology: Any, route: Any, _query: Query, start: Any, goal: Any, _padding: float, _semantics: str):
                                route_hash = state["route_hash"]
                                expand_lane_roi = bool(
                                    switches.regional_preference
                                    and level in {"R0", "R1", "R2"}
                                    and config["roi"].get("expand_route_lane_instances", True)
                                )
                                roi_key = (
                                    query.query_id, route_hash,
                                    -1.0 if level == "R4" else padding,
                                    expand_lane_roi,
                                )
                                allowed = roi_cache.get(roi_key)
                                lane_roi_diagnostics = dict(roi_diagnostics_cache.get(roi_key, {}))
                                roi_hit = allowed is not None
                                roi_started = time.monotonic_ns()
                                if allowed is None:
                                    allowed = (
                                        r2_runtime._raw_free_mask(ctx).copy() if level == "R4"
                                        else r2_runtime._raw_corridor_mask(ctx, topology, route, query, padding)
                                    )
                                    if expand_lane_roi:
                                        allowed, lane_roi_diagnostics = expand_roi_to_route_lane_instances(
                                            ctx.hospital_map, raster, semantic_map, route.polyline, allowed,
                                            free_mask=r2_runtime._raw_free_mask(ctx),
                                            route_probe_radius_m=float(
                                                config["roi"].get("lane_route_probe_radius_m", 0.50)
                                            ),
                                        )
                                    if start is not None:
                                        allowed[start] = True
                                    if goal is not None:
                                        allowed[goal] = True
                                    roi_cache[roi_key] = allowed
                                    roi_diagnostics_cache[roi_key] = lane_roi_diagnostics
                                state["lane_roi_diagnostics"] = lane_roi_diagnostics
                                state["roi_build_ms"] = (time.monotonic_ns() - roi_started) / 1.0e6
                                state["roi_cache_hit"] = roi_hit
                                preference = None
                                field_ms = 0.0
                                if switches.regional_preference:
                                    field_key = (route_hash, grid_hash(allowed), level)
                                    preference = field_cache.get(field_key)
                                    field_hit = preference is not None
                                    if preference is None:
                                        field_started = time.monotonic_ns()
                                        base_key = (route_hash, field_key[1], "R0")
                                        base_preference = field_cache.get(base_key)
                                        if level in {"R1", "R2"} and base_preference is not None:
                                            preference = preference_builder.derive_relaxation(
                                                base_preference, goal=query.goal,
                                                allowed_mask=allowed, relaxation_level=level,
                                                planning_preference_enabled=True,
                                            )
                                        else:
                                            preference = preference_builder.build(
                                                route.polyline, goal=query.goal, allowed_mask=allowed,
                                                relaxation_level=level, planning_preference_enabled=True,
                                                route_diagnostics=state.get("route_orientation"),
                                            )
                                        field_ms = (time.monotonic_ns() - field_started) / 1.0e6
                                        field_cache[field_key] = preference
                                    state.update({"preference": preference, "planning_field_cache_hit": field_hit, "field_build_ms": field_ms})
                                compose_started = time.monotonic_ns()
                                semantic_costmap = composer.compose(
                                    ctx.hospital_map.occupancy, raster, preference, allowed_mask=allowed,
                                    hard_semantics_enabled=switches.l3_hard_semantics,
                                    soft_class_costs_enabled=switches.l3_soft_class_costs,
                                    regional_preference_enabled=switches.regional_preference,
                                    hard_semantics_use_footprint=True,
                                )
                                state["compose_ms"] = (time.monotonic_ns() - compose_started) / 1.0e6
                                state.update({"allowed": allowed, "semantic_costmap": semantic_costmap})
                                session.set_semantic_costmap(semantic_costmap)
                                return allowed, {
                                    "arm_switch_hash": switches.hash,
                                    "route_selector_actual": state.get("route_selector_actual"),
                                    "route_hash": route_hash, "fixed_l1_route_hash": fixed_hash,
                                    "roi_cache_hit": roi_hit, "planning_field_cache_hit": state.get("planning_field_cache_hit", False),
                                    "roi_build_ms": state.get("roi_build_ms", 0.0),
                                    "field_build_ms": state.get("field_build_ms", 0.0),
                                    "compose_ms": state.get("compose_ms", 0.0),
                                    **state.get("route_orientation", {}),
                                    **dict(state.get("lane_roi_diagnostics") or {}),
                                    **semantic_costmap.diagnostics,
                                    **(preference.diagnostics if preference is not None else {}),
                                }

                            cache_mode = r2_runtime.CACHE_MODE_BASELINE if switches.route_selector == "legacy" else r2_runtime.CACHE_MODE_OPTIMIZED
                            result, diagnostics = r2_runtime.plan_l1_l3_corridor_hybrid(
                                ctx, query, topology, session, spec,
                                corridor_padding_m=padding, corridor_semantics="raw_map_smac_aligned",
                                padding_schedule_m=(padding,), validate_each_attempt=True,
                                cache_mode=cache_mode, corridor_mask_builder=mask_builder,
                                route_selector=selector, canonical_path_auditor=canonical_auditor.audit,
                                skip_session_path_mask_validation=True,
                            )
                            points = list(getattr(result, "points", []) or [])
                            canonical = dict(getattr(getattr(result, "path_audit", None), "metrics", {}) or {})
                            semantic_metrics: Dict[str, Any] = {}
                            audit_ms = float(diagnostics.get("canonical_path_audit_ms") or 0.0)
                            if points and state.get("route") is not None:
                                preference = state.get("preference")
                                if preference is None:
                                    audit_key = (state["route_hash"], grid_hash(state["allowed"]), "AUDIT_R0")
                                    preference = field_cache.get(audit_key)
                                    audit_field_started = time.monotonic_ns()
                                    if preference is None:
                                        preference = preference_builder.build(
                                            state["route"].polyline, goal=query.goal, allowed_mask=state["allowed"],
                                            relaxation_level=level, planning_preference_enabled=False,
                                            route_diagnostics=state.get("route_orientation"),
                                        )
                                        field_cache[audit_key] = preference
                                    state["audit_field_build_ms"] = (time.monotonic_ns() - audit_field_started) / 1.0e6
                                    state["field_build_ms"] = float(state.get("field_build_ms") or 0.0) + state["audit_field_build_ms"]
                                semantic_started = time.monotonic_ns()
                                audited = semantic_auditor.audit(
                                    points, preference, relaxation_level=level,
                                    canonical_metrics=canonical,
                                    baseline_path_length_m=baseline_lengths.get((query.query_id, repetition)),
                                )
                                semantic_ms = (time.monotonic_ns() - semantic_started) / 1.0e6
                                audit_ms += semantic_ms
                                semantic_metrics = audited.to_dict()
                                hard_held: Optional[bool] = bool(audited.hard_constraints_held)
                            else:
                                audited = None
                                hard_held = None
                            success = bool(result.planner_success and points and hard_held is True)
                            failure_code = "" if success else str(
                                getattr(audited, "failure_code", "") or result.failure_code
                                or diagnostics.get("failure_code") or "PLANNING_FAILED"
                            )
                            wall_ms = (time.monotonic_ns() - attempt_started) / 1.0e6
                            timing = _attempt_timing(diagnostics, state, wall_ms, audit_ms)
                            record = {
                                "arm": arm, "query_id": query.query_id, "relaxation_level": level,
                                "success": success, "failure_code": failure_code,
                                "failure_stage": str(diagnostics.get("failure_code") or "L3"),
                                "hard_constraints_held": hard_held, "timing": timing,
                                "route_hash": state.get("route_hash"),
                                "route_selector_actual": state.get("route_selector_actual"),
                                "hard_semantics_use_footprint": True,
                                "route_orientation": state.get("route_orientation"),
                                "lane_roi_diagnostics": dict(state.get("lane_roi_diagnostics") or {}),
                                "soft_semantic_cells": int(state.get("semantic_costmap").diagnostics.get("soft_semantic_cells", 0)) if state.get("semantic_costmap") is not None else 0,
                                "soft_cost_histogram": (state.get("preference").diagnostics if state.get("preference") is not None else {}),
                                "composer_cache": {
                                    key: state.get("semantic_costmap").diagnostics.get(key)
                                    for key in (
                                        "base_geometry_cache_hit", "class_cost_cache_hit",
                                        "inflation_template_cache_hit", "inflation_template_build_ms",
                                        "inflation_cache_entries", "inflation_cache_capacity",
                                        "inflation_cache_evictions", "composer_cache_resident_bytes",
                                        "compose_active_bbox", "compose_active_cells", "compose_wall_ms",
                                    )
                                } if state.get("semantic_costmap") is not None else {},
                                "publication": {key: diagnostics.get(key) for key in (
                                    "local_map_update_mode", "local_map_update_messages",
                                    "local_map_update_cells", "local_map_update_bytes",
                                    "local_map_update_skipped", "local_map_publication_ms",
                                    "semantic_ack_reused", "semantic_noop_complete_key_verified",
                                )},
                                "costmap_ack": {key: diagnostics.get(key) for key in (
                                    "costmap_update_acknowledged", "costmap_ack_status",
                                    "costmap_ack_hard_checked_cells", "costmap_ack_hard_mismatch_cells",
                                    "costmap_ack_soft_checked_cells", "costmap_ack_soft_mismatch_cells",
                                    "costmap_ack_soft_exact_mismatch_cells", "costmap_ack_soft_exact_mismatch_ratio",
                                    "costmap_ack_stale_checked_cells", "costmap_ack_stale_roi_cells",
                                    "costmap_ack_hash_mismatch", "costmap_ack_sequence_mismatch",
                                    "costmap_ack_semantics", "semantic_publication_sequence",
                                    "semantic_publication_version", "semantic_policy_hash",
                                    "semantic_source_grid_hash", "semantic_expected_grid_hash",
                                    "semantic_expected_master_hash", "semantic_ack_roi_bbox",
                                    "semantic_effective_dirty_bbox", "semantic_exact_stable_observations",
                                    "semantic_exact_ack_key_hash", "server_costmap_content_hash",
                                    "server_affected_content_hash",
                                )},
                            }
                            attempt_records.append(record)
                            with (output / "attempts.jsonl").open("a", encoding="utf-8") as stream:
                                stream.write(json.dumps({"run_mode": run_mode, "repetition": repetition, **record}, ensure_ascii=False, sort_keys=True, default=str) + "\n")
                            final = {
                                "result": result, "diagnostics": diagnostics, "canonical": canonical,
                                "semantic_metrics": semantic_metrics, "hard_constraints_held": hard_held,
                                "success": success, "failure_code": failure_code, "state": state,
                            }
                            if level == "R0" and switches.relaxation == "graceful" and record["soft_semantic_cells"] == 0:
                                skip_soft_levels = bool(config["preference_relaxation"].get("skip_soft_only_levels_without_effective_soft_semantics", True))
                            # A generated path which fails audit is rejected,
                            # but graceful mode may still lower only comfort
                            # costs and ask Smac for a different path.  Every
                            # retry is independently audited against unchanged
                            # static/lethal/footprint/kinematic constraints.
                            if success:
                                break

                        result = final.get("result")
                        points = list(getattr(result, "points", []) or []) if result is not None else []
                        success = bool(final.get("success"))
                        if arm == "E2" and final.get("state", {}).get("route_hash"):
                            fixed_route_hashes[(run_mode, repetition, query.query_id)] = str(final["state"]["route_hash"])
                        if success and points:
                            path_file = output / "paths" / f"{arm}_{query.query_id}_{run_mode}_{repetition}.json"
                            path_file.write_text(json.dumps(points, indent=2) + "\n")
                            if run_mode == "measured":
                                paths[(arm, query.query_id, repetition)] = points
                            if arm == "E0":
                                baseline_lengths[(query.query_id, repetition)] = float(final.get("canonical", {}).get("path_length_m") or 0.0)
                        else:
                            path_file = None
                        request_ms = (time.monotonic_ns() - request_started) / 1.0e6
                        final_timing = attempt_records[-1].get("timing", {}) if attempt_records else {}
                        diagnostics = final.get("diagnostics", {})
                        semantic_metrics = final.get("semantic_metrics", {})
                        canonical = final.get("canonical", {})
                        usage = resource.getrusage(resource.RUSAGE_SELF)
                        ack_exact = int(diagnostics.get("costmap_ack_soft_exact_mismatch_cells") or 0)
                        ack_checked = int(diagnostics.get("costmap_ack_soft_checked_cells") or 0)
                        cumulative_ack_hard_mismatch = sum(int(
                            record.get("costmap_ack", {}).get("costmap_ack_hard_mismatch_cells") or 0
                        ) for record in attempt_records)
                        cumulative_ack_soft_checked = sum(int(
                            record.get("costmap_ack", {}).get("costmap_ack_soft_checked_cells") or 0
                        ) for record in attempt_records)
                        cumulative_ack_soft_mismatch = sum(int(
                            record.get("costmap_ack", {}).get("costmap_ack_soft_mismatch_cells") or 0
                        ) for record in attempt_records)
                        cumulative_ack_soft_exact = sum(int(
                            record.get("costmap_ack", {}).get("costmap_ack_soft_exact_mismatch_cells") or 0
                        ) for record in attempt_records)
                        row = {
                            "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
                            "parent_architecture": PARENT_ARCHITECTURE,
                            "semantic_map_version": semantic_map.schema_version,
                            "source_map_hash": ctx.map_sha256, "source_pdmap_hash": semantic_map.source_pdmap_hash,
                            "semantic_map_hash": semantic_map.semantic_map_hash, "policy_hash": composer.policy_hash,
                            "topology_graph_hash": topology_graph_hash(topology),
                            "arm": arm, "arm_description": config["ablation_arms"][arm].get("description"),
                            "arm_switches": asdict(switches), "arm_switch_hash": switches.hash,
                            **asdict(switches),
                            "hard_semantics_use_footprint": True,
                            "query_id": query.query_id, "category": query.category,
                            "run_mode": run_mode, "repetition": repetition,
                            "action_success": bool(points), "final_valid_success": success,
                            "failure_code": str(final.get("failure_code") or ""),
                            "relaxation_level": attempt_records[-1]["relaxation_level"] if attempt_records else "R0",
                            "attempt_records": attempt_records,
                            "executed_attempt_count": len(attempt_records),
                            "skipped_soft_only_levels": ["R1", "R2"] if skip_soft_levels else [],
                            "hard_constraints_held": final.get("hard_constraints_held"),
                            "hard_constraints_status": (
                                "HELD" if final.get("hard_constraints_held") is True else
                                "VIOLATED" if final.get("hard_constraints_held") is False else "NOT_APPLICABLE"
                            ),
                            "cumulative_request_wall_ms": request_ms,
                            "final_attempt_wall_ms": float(final_timing.get("wall_ms") or 0.0),
                            "cumulative_l1_ms": _sum_timing(attempt_records, "l1_ms"),
                            "cumulative_roi_build_ms": _sum_timing(attempt_records, "roi_build_ms"),
                            "cumulative_field_build_ms": _sum_timing(attempt_records, "field_build_ms"),
                            "cumulative_compose_ms": _sum_timing(attempt_records, "compose_ms"),
                            "cumulative_publish_ms": _sum_timing(attempt_records, "publish_ms"),
                            "cumulative_ack_wait_ms": _sum_timing(attempt_records, "ack_wait_ms"),
                            "cumulative_smac_ms": _sum_timing(attempt_records, "smac_ms"),
                            "cumulative_audit_ms": _sum_timing(attempt_records, "audit_ms"),
                            "cumulative_unaccounted_process_ms": _sum_timing(
                                attempt_records, "unaccounted_process_ms"
                            ),
                            "peak_rss_bytes": int(diagnostics.get("stack_rss_peak_bytes") or usage.ru_maxrss * 1024),
                            "current_rss_bytes": _current_rss_bytes(),
                            "path_file": str(path_file.relative_to(output)) if path_file else "",
                            "route_hash": final.get("state", {}).get("route_hash"),
                            "route_selector_actual": final.get("state", {}).get("route_selector_actual"),
                            **dict(final.get("state", {}).get("route_orientation") or {}),
                            "lane_roi_diagnostics": dict(
                                final.get("state", {}).get("lane_roi_diagnostics") or {}
                            ),
                            "costmap_update_acknowledged": diagnostics.get("costmap_update_acknowledged"),
                            "costmap_ack_status": diagnostics.get("costmap_ack_status"),
                            "costmap_ack_hard_checked_cells": diagnostics.get("costmap_ack_hard_checked_cells"),
                            "costmap_ack_hard_mismatch_cells": diagnostics.get("costmap_ack_hard_mismatch_cells"),
                            "costmap_ack_soft_checked_cells": ack_checked,
                            "costmap_ack_soft_mismatch_cells": diagnostics.get("costmap_ack_soft_mismatch_cells"),
                            "costmap_ack_soft_exact_mismatch_cells": ack_exact,
                            "costmap_ack_soft_exact_mismatch_ratio": float(ack_exact / ack_checked) if ack_checked else 0.0,
                            "costmap_ack_stale_checked_cells": diagnostics.get("costmap_ack_stale_checked_cells"),
                            "costmap_ack_stale_roi_cells": diagnostics.get("costmap_ack_stale_roi_cells"),
                            "costmap_ack_hash_mismatch": diagnostics.get("costmap_ack_hash_mismatch"),
                            "costmap_ack_sequence_mismatch": diagnostics.get("costmap_ack_sequence_mismatch"),
                            "semantic_publication_sequence": diagnostics.get("semantic_publication_sequence"),
                            "semantic_source_grid_hash": diagnostics.get("semantic_source_grid_hash"),
                            "semantic_expected_master_hash": diagnostics.get("semantic_expected_master_hash"),
                            "server_costmap_content_hash": diagnostics.get("server_costmap_content_hash"),
                            "costmap_ack_semantics": diagnostics.get("costmap_ack_semantics", "interval_not_exact"),
                            "cumulative_costmap_ack_hard_mismatch_cells": cumulative_ack_hard_mismatch,
                            "cumulative_costmap_ack_soft_checked_cells": cumulative_ack_soft_checked,
                            "cumulative_costmap_ack_soft_mismatch_cells": cumulative_ack_soft_mismatch,
                            "cumulative_costmap_ack_soft_exact_mismatch_cells": cumulative_ack_soft_exact,
                            "cumulative_costmap_ack_soft_exact_mismatch_ratio": (
                                float(cumulative_ack_soft_exact / cumulative_ack_soft_checked)
                                if cumulative_ack_soft_checked else 0.0
                            ),
                            **canonical, **semantic_metrics,
                        }
                        # The r1 definition wins if canonical empty-path metrics
                        # contain false placeholders: no generated path is N/A.
                        if not points:
                            row["hard_constraints_held"] = None
                            row["hard_constraints_status"] = "NOT_APPLICABLE"
                        rows.append(row)
                        with (output / "runs.partial.jsonl").open("a", encoding="utf-8") as stream:
                            stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True, default=str) + "\n")
    finally:
        session.close()

    summaries = _summaries(
        rows,
        selected_arms,
        p99_minimum_effective_samples=int(
            config["experiment"].get("p99_minimum_effective_samples", 100)
        ),
    )
    paired = _paired(rows)
    _write_csv(output / "runs.csv", rows)
    _write_csv(output / "summary.csv", summaries)
    _write_csv(output / "paired_comparisons.csv", paired)
    (output / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n")
    (output / "paired_comparisons.json").write_text(json.dumps(paired, indent=2) + "\n")
    _draw_facets(ctx, raster, queries, rows, paths, output)
    protocol = {
        "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "parent_architecture": PARENT_ARCHITECTURE, "semantic_map_version": semantic_map.schema_version,
        "source_map_hash": ctx.map_sha256, "source_pdmap_hash": semantic_map.source_pdmap_hash,
        "semantic_map_hash": semantic_map.semantic_map_hash, "policy_hash": composer.policy_hash,
        "topology_graph_hash": topology_graph_hash(topology), "query_set": query_metadata,
        "cold_start": {
            "semantic_raster_ms": raster_ms, "topology_load_ms": topology_ms,
            "semantic_edge_precompute_ms": edge_ms,
            "total_ms": raster_ms + topology_ms + edge_ms,
        },
        "warmups": warmups, "repetitions": repetitions, "arms": selected_arms,
        "selected_query_ids": [query.query_id for query in queries],
        "arm_switches": {arm: asdict(arm_switches[arm]) for arm in selected_arms},
        "ros_domain_id": ros_domain_id, "static_map": True, "dynamic_obstacles": False,
        "workspace_git": _git_state(ROOT),
        "nav2_git": _git_state(ROOT / "external/arena4_ws/src/deps/nav2/navigation2"),
        "config": config,
    }
    (output / "protocol.json").write_text(json.dumps(protocol, indent=2, ensure_ascii=False) + "\n")
    return output


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run PLN-02 static 2A-V2/r1 diagnostics and ablations")
    parser.add_argument(
        "--mode",
        choices=("convert", "synthetic-smoke", "offline-diagnostic", "real-ablation"),
        default="offline-diagnostic",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--pdmap", type=Path)
    parser.add_argument("--extracted-dir", type=Path)
    parser.add_argument("--semantic-map", type=Path)
    parser.add_argument("--topology-cache", type=Path)
    parser.add_argument("--r0-results", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--warmups", type=int, default=0)
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--ros-domain-id", type=int, default=92)
    parser.add_argument("--arms", default=",".join(ARM_ORDER), help="comma-separated subset of E0,E1,E2,E3,E4")
    parser.add_argument("--query-ids", default="", help="optional comma-separated diagnostic query subset")
    parser.add_argument(
        "--preference-policy-json", default="{}",
        help="diagnostic-only JSON overrides for existing regional_preference keys",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    preference_overrides = json.loads(args.preference_policy_json)
    if args.mode == "convert":
        if args.pdmap is None and args.extracted_dir is None:
            raise SystemExit("convert requires --pdmap or --extracted-dir")
        convert_pdmap(
            pdmap=args.pdmap, extracted_dir=args.extracted_dir,
            source_pdmap_hash=sha256_file(args.pdmap) if args.pdmap else "",
            output_dir=args.output_dir,
        )
    elif args.mode == "synthetic-smoke":
        run_synthetic_smoke(
            output=args.output_dir.resolve(), config_path=args.config.resolve(),
            preference_policy_overrides=preference_overrides,
        )
    else:
        if args.extracted_dir is None or args.semantic_map is None or args.topology_cache is None:
            raise SystemExit(f"{args.mode} requires --extracted-dir, --semantic-map and --topology-cache")
        common = {
            "extracted_dir": args.extracted_dir.resolve(),
            "semantic_map_path": args.semantic_map.resolve(),
            "topology_cache": args.topology_cache.resolve(),
            "output": args.output_dir.resolve(), "config_path": args.config.resolve(),
            "preference_policy_overrides": preference_overrides,
        }
        if args.mode == "offline-diagnostic":
            run_offline_diagnostic(**common, r0_results=args.r0_results.resolve() if args.r0_results else None)
        else:
            arms = [value.strip() for value in args.arms.split(",") if value.strip()]
            unknown = sorted(set(arms) - set(ARM_ORDER))
            if unknown:
                raise SystemExit(f"unknown arms: {unknown}")
            run_real_ablation(
                **common, warmups=args.warmups, repetitions=args.repetitions,
                ros_domain_id=args.ros_domain_id, arms=arms,
                query_ids=[value.strip() for value in args.query_ids.split(",") if value.strip()] or None,
            )
    print(f"2A-V2/r1 output: {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
