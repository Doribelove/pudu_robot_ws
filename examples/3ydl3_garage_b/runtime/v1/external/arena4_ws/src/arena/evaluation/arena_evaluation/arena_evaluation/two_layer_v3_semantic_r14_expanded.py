"""Paired E0/adaptive 2A-V3 r14 online evaluation on a frozen query set.

The adaptive arm runs the explicit forward-only SE(2) engine only after the
query-level semantic applicability certificate permits it.  A certified
inapplicable query is dispatched to the unchanged E0 native Smac path; an
applicable E5 search failure is never converted into a fallback success.
"""
from __future__ import annotations

import argparse
import csv
import ctypes
import gc
import json
import math
import os
from pathlib import Path
import platform
import resource
import shutil
import statistics
import time
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from . import semantic_applicability_v3 as applicability
from . import semantic_static_cache_v3 as static_cache
from . import two_layer_v2_semantic_r1_benchmark as r1
from . import two_layer_v2_semantic_r3_benchmark as r3
from . import two_layer_v3_semantic_r13_benchmark as r13_offline
from . import two_layer_v3_semantic_r13_online as r13_online
from . import two_layer_v3_semantic_r14_benchmark as r14_offline
from .path_audit import PathAuditor
from .regional_preference_r1 import RegionalPreferenceBuilderR1, orient_route_for_query
from .regional_preference_r3 import RegionalPreferenceBuilderR3
from .semantic_costmap_r2 import SemanticCostmapComposerR2
from .semantic_map import canonical_hash, sha256_file
from .semantic_path_audit import SemanticPathAuditor
from .semantic_query_defaults import load_query_set
from .semantic_route_phase_compact_r14 import (
    CompactRoutePhaseWorldR14, prepare_query_input_compact,
)
from .semantic_route_phase_v3 import OrientedRoute
from .semantic_v3_ack_r14 import DeterministicReinflationSessionR14


ARCHITECTURE_ID = "2A-V3"
IMPLEMENTATION_REVISION = "r14-parking-dispatch-ack-cache"
PROTOCOL_ID = "PLN-02-2A-V3-R14-EXPANDED-PAIRED-V1"
SCHEMA_VERSION = "PLN-02-2A-V3-R14-EXPANDED-RESULT-V1"
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE_ROOT / "config/two_layer_v3_semantic_r14_expanded.yaml"


class AuditOnlyPreferenceBuilderR14(RegionalPreferenceBuilderR3):
    """Build bound semantic geometry without a second 2-D planning guide.

    E5's reference-deviation term lives in its explicit route-phase SE(2)
    edges.  Retaining r3's raster guide in the master would duplicate the
    preference and allocate several full-map temporary grids.  The r1 parent
    still supplies the lane-instance, boundary and parking fields required by
    the unchanged E5 state cost and semantic audits.
    """

    def build(self, route: Sequence[Sequence[float]], **kwargs: Any) -> Any:
        kwargs["planning_preference_enabled"] = False
        field = RegionalPreferenceBuilderR1.build(self, route, **kwargs)
        field.diagnostics.update({
            "r14_audit_geometry_only": True,
            "r14_2d_regional_planning_cost_disabled": True,
            "r14_explicit_se2_reference_deviation_owner": "route_phase_edge_cost",
        })
        return field


def _identity() -> dict[str, str]:
    return {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
    }


def _current_rss_bytes() -> int:
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1])*1024
    except OSError:
        pass
    return 0


def _release() -> int:
    gc.collect()
    try:
        ctypes.CDLL("libc.so.6").malloc_trim(0)
    except (OSError, AttributeError):
        pass
    return _current_rss_bytes()


def _json_safe(value: Any) -> Any:
    """Replace diagnostic non-finite scalars with fail-closed JSON nulls."""
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, (np.floating, float)):
        numeric = float(value)
        return numeric if math.isfinite(numeric) else None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def _load(path: Path) -> tuple[
    dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any], Path, Path,
]:
    path = Path(path).resolve()
    config = yaml.safe_load(path.read_text()) or {}
    for key, value in _identity().items():
        if config.get(key) != value:
            raise ValueError(f"r14 expanded identity mismatch for {key}")
    algorithm_path = Path(str(config["algorithm_config"]["path"]))
    if not algorithm_path.is_absolute():
        algorithm_path = path.parent/algorithm_path
    algorithm_path = algorithm_path.resolve()
    expected_algorithm = str(config["algorithm_config"]["sha256"])
    if expected_algorithm != sha256_file(algorithm_path):
        raise ValueError("expanded algorithm configuration hash mismatch")
    algorithm, parent_algorithm, parent, _selected8 = r14_offline._load_config(algorithm_path)
    query_path = Path(str(config["query_set"]["path"]))
    if not query_path.is_absolute():
        query_path = path.parent/query_path
    query_path = query_path.resolve()
    if sha256_file(query_path) != str(config["query_set"]["file_sha256"]):
        raise ValueError("expanded query-set file hash mismatch")
    return config, algorithm, parent_algorithm, parent, algorithm_path, query_path


def _route(
    *, selector: Any, topology: Any, query: Any,
) -> tuple[Any, dict[str, Any], float]:
    started = time.monotonic()
    _sn, _gn, route, reason = selector(
        topology, query, cache_mode=r1.r2_runtime.CACHE_MODE_OPTIMIZED, timing={},
    )
    if route is None:
        raise RuntimeError(f"L1_ROUTE_FAILED:{reason}")
    route, orientation = orient_route_for_query(route, query)
    return route, orientation, (time.monotonic()-started)*1000.0


def _ack_fields(diagnostics: Mapping[str, Any]) -> dict[str, Any]:
    return {
        key: diagnostics.get(key) for key in (
            "costmap_update_acknowledged", "costmap_ack_status",
            "costmap_ack_semantics",
            "costmap_ack_hard_checked_cells", "costmap_ack_hard_mismatch_cells",
            "costmap_ack_soft_checked_cells", "costmap_ack_soft_exact_mismatch_cells",
            "costmap_ack_stale_checked_cells", "costmap_ack_stale_roi_cells",
            "costmap_ack_hash_mismatch", "costmap_ack_sequence_mismatch",
            "costmap_ack_wait_ms", "costmap_ack_attempts",
            "semantic_publication_sequence", "semantic_publication_version",
            "semantic_expected_master_hash", "semantic_expected_server_content_hash",
            "semantic_ack_roi_bbox",
            "server_costmap_content_hash", "local_map_update_mode",
            "local_map_update_fallback", "local_map_update_bytes",
            "deterministic_reinflation_reset_count", "deterministic_reinflation_reset_ms",
            "deterministic_reinflation_replay_count",
            "deterministic_reinflation_replay_messages",
            "deterministic_reinflation_replay_cells",
            "deterministic_reinflation_replay_scope",
        )
    }


def _semantic_row(metrics: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "lane_correct_side_ratio": metrics.get("lane_correct_side_ratio"),
        "lane_target_band_ratio": metrics.get("lane_target_band_ratio"),
        "lane_lateral_error_p50_m": metrics.get(
            "base_center_to_right_boundary_error_p50_m",
            metrics.get("lane_lateral_error_p50_m"),
        ),
        "parking_center_band_ratio": metrics.get("parking_center_band_ratio"),
        "parking_normalized_deviation_p50": metrics.get(
            "parking_center_normalized_deviation_p50"
        ),
    }


def _run_e0(
    *, query: Any, route: Any, orientation: Mapping[str, Any], ctx: Any,
    raster: Any, topology: Any, semantic_map: Any, parent: Mapping[str, Any],
    builder: Any, composer: Any, semantic_auditor: Any, canonical_auditor: Any,
    session: Any, backend_spec: Any,
) -> tuple[dict[str, Any], list[Mapping[str, Any]]]:
    timing: dict[str, float] = {}
    started = time.monotonic()
    roi_started = time.monotonic()
    allowed = r1.r2_runtime._raw_corridor_mask(
        ctx, topology, route, query, float(parent["roi"]["r0_padding_m"]),
    )
    start_cell = ctx.hospital_map.world_to_cell(*query.start[:2])
    goal_cell = ctx.hospital_map.world_to_cell(*query.goal[:2])
    if start_cell is None or goal_cell is None:
        raise RuntimeError("E0_INVALID_ENDPOINT")
    allowed[start_cell] = True
    allowed[goal_cell] = True
    timing["roi_build_ms"] = (time.monotonic()-roi_started)*1000.0
    compose_started = time.monotonic()
    composition = composer.compose(
        ctx.hospital_map.occupancy, raster, None, allowed_mask=allowed,
        hard_semantics_enabled=False, soft_class_costs_enabled=False,
        regional_preference_enabled=False, hard_semantics_use_footprint=True,
    )
    timing["compose_ms"] = (time.monotonic()-compose_started)*1000.0
    session.set_semantic_costmap(composition)
    action_started = time.monotonic()
    plan = session.plan(
        query, backend_spec, source="e0_native_smac_48bin",
        allowed_mask=allowed, skip_path_mask_validation=True,
    )
    timing["publication_ack_and_smac_ms"] = (time.monotonic()-action_started)*1000.0
    diagnostics = dict(plan.diagnostics or {})
    ack = _ack_fields(diagnostics)
    if ack.get("costmap_update_acknowledged") is not True:
        raise RuntimeError("E0_EXACT_ACK_FAILED")
    verify_started = time.monotonic()
    _verified_master, verified = session.verified_master_snapshot(ack)
    timing["verified_readback_ms"] = (time.monotonic()-verify_started)*1000.0
    points = list(plan.points or [])
    if not plan.planner_success or not points:
        return {
            "final_valid_success": False, "failure_code": plan.failure_code or "E0_SMAC_FAILED",
            "costmap_ack": ack, "verified_master": verified, "timing": timing,
            "semantic_success_counted": False,
        }, []
    field_started = time.monotonic()
    field = builder.build(
        route.polyline, goal=query.goal, allowed_mask=allowed,
        relaxation_level="R0", planning_preference_enabled=False,
        route_diagnostics=orientation,
    )
    timing["audit_field_build_ms"] = (time.monotonic()-field_started)*1000.0
    audit_started = time.monotonic()
    canonical = canonical_auditor.audit(query, points, allowed)
    semantic = semantic_auditor.audit(
        points, field, relaxation_level="R0", canonical_metrics=canonical.metrics,
    )
    semantic_metrics = semantic.to_dict()
    timing["audit_ms"] = (time.monotonic()-audit_started)*1000.0
    path_interface = session.publish_verified_path(points, query_id=query.query_id)
    hard_ok = bool(semantic.hard_constraints_held)
    final = bool(plan.planner_success and canonical.final_valid_success and hard_ok)
    row = {
        "final_valid_success": final,
        "failure_code": "" if final else (
            semantic_metrics.get("failure_code") or "E0_PATH_AUDIT_FAILED"
        ),
        "strict_semantic_gate_passed": False,
        "semantic_success_counted": False,
        "collision_violations": int(semantic_metrics.get("collision_violation_count", 0)),
        "kinematic_violations": int(semantic_metrics.get("kinematic_violation_count", 0)),
        "hard_semantic_violations": int(semantic_metrics.get("hard_semantic_violation_count", 0)),
        "no_stopping_goal_violations": int(bool(semantic_metrics.get("no_stopping_goal_violation", False))),
        "reverse_distance_m": float(semantic_metrics.get("reverse_distance_m", 0.0)),
        "rotate_in_place_count": int(semantic_metrics.get("in_place_rotation_count", 0)),
        "path_length_m": semantic_metrics.get("path_length_m"),
        "maximum_curvature_1pm": semantic_metrics.get("maximum_curvature"),
        "costmap_ack": ack, "verified_master": verified,
        "path_interface": path_interface, "timing": timing,
        "route_hash": canonical_hash(route.polyline),
        "route_orientation": dict(orientation), "native_smac_bins": 48,
        **_semantic_row(semantic_metrics),
    }
    row["timing"]["request_engine_ms"] = (time.monotonic()-started)*1000.0
    return row, points


def _run_e5_or_dispatch(
    *, query: Any, route: Any, orientation: Mapping[str, Any], query_hash: str,
    request_dir: Path, ctx: Any, semantic_map: Any, raster: Any, topology: Any,
    algorithm: Mapping[str, Any], parent: Mapping[str, Any], builder: Any,
    composer: Any, session: Any, canonical_auditor: Any,
    e0_arguments: Mapping[str, Any], dispatch_cache: static_cache.BoundedLRU,
) -> tuple[dict[str, Any], list[Mapping[str, Any]]]:
    timing: dict[str, float] = {}
    memory_stage_bytes: dict[str, int] = {"entry": _current_rss_bytes()}
    dispatch_key = canonical_hash({
        "map_hash": ctx.map_sha256,
        "semantic_map_hash": semantic_map.semantic_map_hash,
        "query": {
            "start": list(query.start), "goal": list(query.goal),
            "category": query.category,
        },
        "route": [list(point) for point in route.polyline],
        "applicability_policy": e0_arguments["r14_config"]["semantic_applicability"],
        "route_phase_policy": algorithm["route_phase_policy"],
    })
    cached_dispatch = dispatch_cache.get(dispatch_key)
    if cached_dispatch is not None and cached_dispatch.get("dispatch") == "E0_NATIVE_SMAC":
        e0_call = {
            key: value for key, value in e0_arguments.items() if key != "r14_config"
        }
        row, points = _run_e0(**e0_call)
        row.update({
            "dispatch": "E0_NATIVE_SMAC",
            "fallback_reason": cached_dispatch["failure_code"],
            "applicability": cached_dispatch.get("applicability"),
            "parking_reference": cached_dispatch.get("parking_reference"),
            "dispatch_cache_hit": True, "dispatch_cache_key": dispatch_key,
        })
        return row, points
    # A semantic route that does not bind both request endpoints cannot define
    # the requested lane-relative phase.  This is semantic inapplicability,
    # not an E5 search failure: dispatch to the independently prepared E0
    # route before allocating the full route-phase field.  Hash/schema/binding
    # errors discovered after this point still fail closed.
    attachment_limit = float(
        r13_offline._policy(algorithm).endpoint_attachment_limit_m
    )
    route_start_distance = orientation.get("route_start_distance_m")
    route_end_distance = orientation.get("route_end_distance_m")
    if (
        route_start_distance is None or route_end_distance is None
        or float(route_start_distance) > attachment_limit
        or float(route_end_distance) > attachment_limit
    ):
        preliminary = {
            "dispatch": "E0_NATIVE_SMAC",
            "failure_code": "SEMANTIC_ROUTE_ENDPOINT_ATTACH_INAPPLICABLE",
            "applicability": {
                "applicable": False,
                "classification": "SEMANTIC_ROUTE_ENDPOINT_ATTACH_INAPPLICABLE",
                "route_start_distance_m": route_start_distance,
                "route_end_distance_m": route_end_distance,
                "endpoint_attachment_limit_m": attachment_limit,
            },
            "parking_reference": None,
        }
        dispatch_cache.put(
            dispatch_key, preliminary,
            len(json.dumps(_json_safe(preliminary), sort_keys=True)),
        )
        e0_call = {
            key: value for key, value in e0_arguments.items() if key != "r14_config"
        }
        row, points = _run_e0(**e0_call)
        row.update({
            "dispatch": "E0_NATIVE_SMAC",
            "fallback_reason": preliminary["failure_code"],
            "applicability": preliminary["applicability"],
            "parking_reference": None,
            "dispatch_cache_hit": False,
            "dispatch_cache_key": dispatch_key,
        })
        return row, points
    prepare_started = time.monotonic()
    metadata, arrays, composition, publication_allowed, full_allowed = (
        prepare_query_input_compact(
        query=query, route=route, orientation=orientation, ctx=ctx,
        semantic_map=semantic_map, raster=raster, topology=topology,
        builder=builder, composer=composer, parent=parent,
        query_set_hash=query_hash,
        maximum_lateral_probe_m=float(
            r13_offline._policy(algorithm).maximum_lateral_probe_m
        ),
        )
    )
    memory_stage_bytes["after_prepare_input"] = _current_rss_bytes()
    timing.update(metadata["timing"])
    timing["input_total_ms"] = (time.monotonic()-prepare_started)*1000.0
    preliminary_world = CompactRoutePhaseWorldR14(
        request_dir, query.query_id, arrays_override=arrays, meta_override=metadata,
        full_occupancy=ctx.hospital_map.occupancy, full_allowed=full_allowed,
    )
    preliminary_world._canonical_auditor = canonical_auditor
    memory_stage_bytes["after_preliminary_world"] = _current_rss_bytes()
    original_route = OrientedRoute(
        metadata["route_polyline"], preliminary_world.start, preliminary_world.goal,
        endpoint_attachment_limit_m=r13_offline._policy(algorithm).endpoint_attachment_limit_m,
    )
    preliminary = r14_offline.classify_and_plan(
        world=preliminary_world, original_route=original_route,
        semantic_map=semantic_map, config=e0_arguments["r14_config"],
        parent_algorithm=algorithm, run_search=False,
    )
    e0_call = {
        key: value for key, value in e0_arguments.items() if key != "r14_config"
    }
    if preliminary["dispatch"] == "E0_NATIVE_SMAC":
        dispatch_cache.put(
            dispatch_key,
            {key: preliminary.get(key) for key in (
                "dispatch", "failure_code", "applicability", "parking_reference",
            )},
            len(json.dumps(_json_safe(preliminary), sort_keys=True)),
        )
        row, points = _run_e0(**e0_call)
        row.update({
            "dispatch": "E0_NATIVE_SMAC", "fallback_reason": preliminary["failure_code"],
            "applicability": preliminary.get("applicability"),
            "parking_reference": preliminary.get("parking_reference"),
            "dispatch_cache_hit": False, "dispatch_cache_key": dispatch_key,
        })
        row["timing"]["applicability_and_input_ms"] = timing["input_total_ms"]
        return row, points
    if preliminary["dispatch"] != "E5_R14_EXPLICIT_SE2":
        raise RuntimeError(preliminary["failure_code"] or "R14_APPLICABILITY_FAILED_CLOSED")
    # ``RoutePhaseWorld`` owns cropped copies of all query grids.  The
    # preliminary world is no longer needed after classification; keeping it
    # alive while constructing the verified online world doubled the largest
    # route-phase ROI and caused the r13/r14 multi-GB peak.
    del original_route, preliminary_world, preliminary
    memory_stage_bytes["after_preliminary_world_release"] = _release()
    session.set_semantic_costmap(composition)
    publication_started = time.monotonic()
    ack = session.update_local_mask(publication_allowed)
    memory_stage_bytes["after_publication_ack"] = _current_rss_bytes()
    timing["publication_and_ack_ms"] = (time.monotonic()-publication_started)*1000.0
    verified_started = time.monotonic()
    verified_master, verified = session.verified_master_snapshot(ack)
    timing["verified_readback_ms"] = (time.monotonic()-verified_started)*1000.0
    world = CompactRoutePhaseWorldR14(
        request_dir, query.query_id, arrays_override=arrays, meta_override=metadata,
        full_occupancy=ctx.hospital_map.occupancy, full_allowed=full_allowed,
        master_override=verified_master, expected_master_hash=metadata["expected_master_hash"],
    )
    world._canonical_auditor = canonical_auditor
    memory_stage_bytes["after_verified_world"] = _current_rss_bytes()
    # The verified world has copied all cropped search grids and retains only
    # the two full arrays required by canonical audit.  Release the remaining
    # request-sized views before constructing the state lattice.
    for name in tuple(arrays):
        if name not in {"occupancy", "allowed"}:
            arrays.pop(name, None)
    del verified_master, publication_allowed
    memory_stage_bytes["after_full_array_release"] = _release()
    original_route = OrientedRoute(
        metadata["route_polyline"], world.start, world.goal,
        endpoint_attachment_limit_m=r13_offline._policy(algorithm).endpoint_attachment_limit_m,
    )
    search_started = time.monotonic()
    decision = r14_offline.classify_and_plan(
        world=world, original_route=original_route, semantic_map=semantic_map,
        config=e0_arguments["r14_config"], parent_algorithm=algorithm, run_search=True,
    )
    memory_stage_bytes["after_se2_search"] = _current_rss_bytes()
    timing["se2_search_and_audit_ms"] = (time.monotonic()-search_started)*1000.0
    if decision["dispatch"] == "E0_NATIVE_SMAC":
        dispatch_cache.put(
            dispatch_key,
            {key: decision.get(key) for key in (
                "dispatch", "failure_code", "applicability", "parking_reference",
            )},
            len(json.dumps(_json_safe(decision), sort_keys=True)),
        )
        row, points = _run_e0(**e0_call)
        row.update({
            "dispatch": "E0_NATIVE_SMAC", "fallback_reason": decision["failure_code"],
            "applicability": decision.get("applicability"),
            "parking_reference": decision.get("parking_reference"),
            "dispatch_cache_hit": False, "dispatch_cache_key": dispatch_key,
        })
        row["timing"]["e5_applicability_probe_ms"] = sum(timing.values())
        return row, points
    witness = decision.get("witness")
    if decision["dispatch"] != "E5_R14_EXPLICIT_SE2" or witness is None:
        raise RuntimeError(decision["failure_code"] or "APPLICABLE_E5_STRICT_SEARCH_FAILED")
    row = r13_online._row_from_witness(query, witness)
    path_started = time.monotonic()
    path_interface = session.publish_verified_path(witness["points"], query_id=query.query_id)
    timing["path_publication_echo_ms"] = (time.monotonic()-path_started)*1000.0
    row.update({
        "dispatch": "E5_R14_EXPLICIT_SE2", "fallback_reason": "",
        "failure_code": "", "costmap_ack": ack, "verified_master": verified,
        "path_interface": path_interface, "timing": timing,
        "route_hash": metadata["route_hash"], "roi_hash": metadata["roi_hash"],
        "applicability": decision.get("applicability"),
        "parking_reference": decision.get("parking_reference"),
        "search": decision.get("search"),
        "dispatch_cache_hit": cached_dispatch is not None,
        "dispatch_cache_key": dispatch_key,
        "memory_stage_bytes": memory_stage_bytes,
    })
    if cached_dispatch is None:
        dispatch_cache.put(
            dispatch_key,
            {"dispatch": "E5_R14_EXPLICIT_SE2", "failure_code": "",
             "applicability": decision.get("applicability")},
            len(json.dumps(_json_safe(decision.get("applicability") or {}), sort_keys=True))+128,
        )
    points = list(witness["points"])
    return row, points


def _percentile(values: Sequence[float], q: float) -> float | None:
    return float(np.percentile(np.asarray(values, dtype=np.float64), q)) if values else None


def _balanced_pair_order(
    arms: Sequence[str], *, repetition: int, query_index: int,
) -> tuple[str, ...]:
    """Counterbalance the two arms without using any planner outcome.

    A fixed E0->ADAPTIVE order makes an inapplicable adaptive request inherit
    E0's already acknowledged costmap.  Alternating AB/BA by the frozen query
    ordinal and repetition balances that no-op advantage exactly across every
    32-query repetition while remaining deterministic and auditable.
    """
    values = tuple(arms)
    if set(values) == {"E0", "ADAPTIVE"} and len(values) == 2:
        canonical = ("E0", "ADAPTIVE")
        return canonical if (int(repetition) + int(query_index)) % 2 else canonical[::-1]
    return values


def run(
    *, output: Path, config_path: Path = DEFAULT_CONFIG,
    arms: Sequence[str] = ("E0", "ADAPTIVE"), query_ids: Sequence[str] = (),
    warmups: int | None = None, repetitions: int | None = None,
    ros_domain_id: int | None = None,
) -> dict[str, Any]:
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output/"logs").mkdir(); (output/"paths").mkdir()
    started = time.monotonic()
    config, algorithm, parent_algorithm, parent, algorithm_path, query_path = _load(config_path)
    queries, _intents, query_meta = load_query_set(
        query_path, actual_map_hash=algorithm["frozen_bindings"]["map_hash"],
        actual_semantic_map_hash=algorithm["frozen_bindings"]["semantic_map_hash"],
    )
    expected_query = config["query_set"]
    for key in ("query_set_id", "query_hash"):
        if str(query_meta.get(key)) != str(expected_query.get(key)):
            raise ValueError(f"expanded query-set binding mismatch for {key}")
    if len(queries) != int(expected_query["query_count"]):
        raise ValueError("expanded query-set count mismatch")
    if bool(query_meta.get("selection_used_planner_outcome", True)):
        raise ValueError("expanded query selection must be input-only")
    if query_ids:
        wanted = set(query_ids); queries = [query for query in queries if query.query_id in wanted]
        if {query.query_id for query in queries} != wanted:
            raise ValueError("unknown expanded query id")
    selected_arms = [arm for arm in ("E0", "ADAPTIVE") if arm in set(arms)]
    if not selected_arms:
        raise ValueError("at least one of E0,ADAPTIVE is required")
    warmup_count = int(config["measurement"]["warmups"] if warmups is None else warmups)
    measured_count = int(config["measurement"]["repetitions"] if repetitions is None else repetitions)
    domain = int(config["online_interface"]["ros_domain_id"] if ros_domain_id is None else ros_domain_id)
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    sources = [Path(__file__).resolve(), Path(config_path).resolve(), algorithm_path,
               query_path, Path(r14_offline.__file__).resolve(),
               Path(__file__).with_name("semantic_parking_reference_v3.py"),
               Path(__file__).with_name("semantic_v3_ack_r14.py"),
               Path(__file__).with_name("semantic_static_cache_v3.py"),
               Path(__file__).with_name("semantic_route_phase_compact_r14.py")]
    source_hashes = {str(path): sha256_file(path) for path in sources}
    applicability._write_json(output/"protocol.json", {
        **_identity(), "schema_version": SCHEMA_VERSION, "configuration": config,
        "query_set": query_meta, "query_count": len(queries), "query_ids": [q.query_id for q in queries],
        "arms": selected_arms, "warmups": warmup_count, "repetitions": measured_count,
        "ros_domain_id": domain, "static_map": True, "dynamic_obstacles": False,
        "selection_used_planner_outcome": False, "source_hashes_at_start": source_hashes,
        "processes_before": applicability._process_audit(),
    })
    static_started = time.monotonic()
    with r3._r3_bindings():
        prepared = static_cache.prepare_static_cached(
            applicability.DEFAULT_EXTRACTED, applicability.DEFAULT_SEMANTIC_MAP,
            applicability.DEFAULT_TOPOLOGY, parent, output=None,
            cache_root=Path(algorithm["static_cache"]["root"]),
            maximum_disk_entries=int(algorithm["static_cache"]["maximum_disk_entries"]),
        )
    static_prepare_ms = (time.monotonic()-static_started)*1000.0
    ctx, semantic_map, raster, topology, _annotator, router = prepared[:6]
    canonical_auditor = PathAuditor(ctx, source_commit="2A-V3-r14-expanded-bound-source")
    semantic_auditor = SemanticPathAuditor(ctx.hospital_map, semantic_map, raster)
    builder = AuditOnlyPreferenceBuilderR14(
        ctx.hospital_map, raster, policy=parent["regional_preference"], semantic_map=semantic_map,
    )
    composer = SemanticCostmapComposerR2(policy=parent["l3_soft_cost"], inflation_cache_capacity=2)
    e0_switches = r1.ArmSwitches.parse(parent["ablation_arms"]["E0"])
    e5_switches = r1.ArmSwitches.parse(parent["ablation_arms"]["E4"])
    e0_selector = r1._selector_for_arm(
        e0_switches, topology, router, {},
        preferred_attachment_radius_m=float(parent["endpoint_attachment"]["preferred_radius_m"]),
        attachment_cost_weight=float(parent["endpoint_attachment"]["cost_weight"]),
    )
    e5_selector = r1._selector_for_arm(
        e5_switches, topology, router, {},
        preferred_attachment_radius_m=float(parent["endpoint_attachment"]["preferred_radius_m"]),
        attachment_cost_weight=float(parent["endpoint_attachment"]["cost_weight"]),
    )
    backend_spec = r1.legacy.backend_availability()["hybrid_astar"]
    if not backend_spec.available:
        raise RuntimeError(f"native Smac unavailable: {backend_spec.reason}")
    session = DeterministicReinflationSessionR14(
        ctx, output, map_yaml=ctx.map_yaml, log_tag=f"2a_v3_r14_expanded_{int(time.time())}",
        local_mask_updates=True, optimization_profile="v7_candidate",
        smac_parameter_profile="baseline", optimization_stage="step3_delta_map",
        enable_mask_reuse_noop=True, force_full_on_semantic_signature_change=False,
        planner_parameter_overrides={"angle_quantization_bins": 48},
        costmap_ack_timeout_s=float(config["online_interface"]["exact_ack_timeout_s"]),
    )
    session.local_map_update_strategy = "roi_ack"
    session.roi_tile_overlap_rows = int(config["online_interface"]["roi_tile_overlap_rows"])
    session.roi_seam_repair_margin_rows = int(config["online_interface"]["roi_seam_repair_margin_rows"])
    session.roi_max_payload_bytes = int(config["online_interface"]["roi_max_payload_bytes"])
    rows: list[dict[str, Any]] = []
    rss_trace: list[dict[str, Any]] = []
    dispatch_cache = static_cache.BoundedLRU(capacity=64, maximum_bytes=1_048_576)
    session.start()
    try:
        for mode, count in (("warmup", warmup_count), ("measured", measured_count)):
            for repetition in range(1, count+1):
                for query_index, query in enumerate(queries):
                    pair_order = _balanced_pair_order(
                        selected_arms, repetition=repetition, query_index=query_index,
                    )
                    for pair_position, arm in enumerate(pair_order, start=1):
                        request_started = time.monotonic()
                        row: dict[str, Any] = {
                            **_identity(), "mode": mode, "repetition": repetition,
                            "arm": arm, "query_id": query.query_id, "category": query.category,
                            "relaxation_level": "R0", "used_historical_path_or_witness": False,
                            "pair_order": list(pair_order), "pair_position": pair_position,
                        }
                        points: list[Mapping[str, Any]] = []
                        try:
                            session.reset_query_state(
                                f"{arm}:{mode}:{repetition}:{query.query_id}", restore_base_map=False,
                            )
                            selector = e0_selector if arm == "E0" else e5_selector
                            route, orientation, l1_ms = _route(selector=selector, topology=topology, query=query)
                            if arm == "ADAPTIVE":
                                e0_route, e0_orientation, e0_l1_ms = _route(
                                    selector=e0_selector, topology=topology, query=query,
                                )
                            else:
                                e0_route, e0_orientation, e0_l1_ms = route, orientation, l1_ms
                            e0_arguments = {
                                "query": query, "route": e0_route, "orientation": e0_orientation,
                                "ctx": ctx, "raster": raster, "topology": topology,
                                "semantic_map": semantic_map, "parent": parent,
                                "builder": builder, "composer": composer,
                                "semantic_auditor": semantic_auditor,
                                "canonical_auditor": canonical_auditor,
                                "session": session, "backend_spec": backend_spec,
                                "r14_config": algorithm,
                            }
                            if arm == "E0":
                                outcome, points = _run_e0(**{
                                    key: value for key, value in e0_arguments.items()
                                    if key != "r14_config"
                                })
                                outcome["dispatch"] = "E0_NATIVE_SMAC_BASELINE"
                            else:
                                outcome, points = _run_e5_or_dispatch(
                                    query=query, route=route, orientation=orientation,
                                    query_hash=query_meta["query_hash"],
                                    request_dir=output, ctx=ctx, semantic_map=semantic_map,
                                    raster=raster, topology=topology, algorithm=parent_algorithm,
                                    parent=parent, builder=builder, composer=composer, session=session,
                                    canonical_auditor=canonical_auditor, e0_arguments=e0_arguments,
                                    dispatch_cache=dispatch_cache,
                                )
                            row.update(outcome)
                            row.setdefault("timing", {})["l1_ms"] = l1_ms
                            if arm == "ADAPTIVE":
                                row["timing"]["e0_fallback_route_prepare_ms"] = e0_l1_ms
                        except (RuntimeError, ValueError) as error:
                            row.update({
                                "final_valid_success": False,
                                "strict_semantic_gate_passed": False,
                                "semantic_success_counted": False,
                                "failure_code": type(error).__name__,
                                "failure_detail": str(error),
                            })
                        row["request_wall_ms"] = (time.monotonic()-request_started)*1000.0
                        row["peak_rss_bytes"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024
                        row = _json_safe(row)
                        points = _json_safe(points)
                        if points:
                            applicability._write_json(
                                output/"paths"/f"{mode}_{repetition:02d}_{arm}_{query.query_id}.json",
                                points,
                            )
                        rows.append(row)
                        applicability._write_json(
                            output/f"result_{mode}_{repetition:02d}_{arm}_{query.query_id}.json", row,
                        )
                        current = _release()
                        rss_trace.append({
                            "mode": mode, "repetition": repetition, "arm": arm,
                            "query_id": query.query_id, "current_rss_bytes_after_release": current,
                            "peak_rss_bytes": row["peak_rss_bytes"],
                        })
    finally:
        session.close()
    measured = [row for row in rows if row["mode"] == "measured"]
    flat = []
    for row in rows:
        ack = row.get("costmap_ack") or {}
        flat.append({
            key: row.get(key) for key in (
                "architecture_id", "implementation_revision", "protocol_id", "mode", "repetition",
                "arm", "pair_order", "pair_position", "query_id", "category", "dispatch",
                "fallback_reason", "failure_code",
                "final_valid_success", "strict_semantic_gate_passed", "semantic_success_counted",
                "lane_correct_side_ratio", "lane_target_band_ratio", "lane_lateral_error_p50_m",
                "parking_center_band_ratio", "parking_normalized_deviation_p50", "path_length_m",
                "maximum_curvature_1pm", "collision_violations", "kinematic_violations",
                "hard_semantic_violations", "no_stopping_goal_violations", "reverse_distance_m",
                "rotate_in_place_count", "request_wall_ms", "peak_rss_bytes",
            )
        } | {
            "acknowledged": ack.get("costmap_update_acknowledged"),
            "ack_hard_mismatch": ack.get("costmap_ack_hard_mismatch_cells"),
            "ack_soft_mismatch": ack.get("costmap_ack_soft_exact_mismatch_cells"),
            "ack_stale_cells": ack.get("costmap_ack_stale_roi_cells"),
            "ack_hash_mismatch": ack.get("costmap_ack_hash_mismatch"),
            "ack_sequence_mismatch": ack.get("costmap_ack_sequence_mismatch"),
            "timeout_full_repair": ack.get("local_map_update_fallback"),
        })
    with (output/"runs.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat[0])); writer.writeheader(); writer.writerows(flat)
    applicability._write_json(output/"rss_trace.json", rss_trace)
    summaries: dict[str, Any] = {}
    for arm in selected_arms:
        arm_rows = [row for row in measured if row["arm"] == arm]
        times = [float(row["request_wall_ms"]) for row in arm_rows]
        summaries[arm] = {
            "sample_count": len(arm_rows),
            "final_valid_count": sum(row.get("final_valid_success") is True for row in arm_rows),
            "semantic_success_count": sum(row.get("semantic_success_counted") is True for row in arm_rows),
            "e0_dispatch_count": sum(str(row.get("dispatch", "")).startswith("E0_") for row in arm_rows),
            "p50_ms": statistics.median(times) if times else None,
            "p95_ms": _percentile(times, 95), "p99_ms": _percentile(times, 99),
        }
    paired_ratio = None
    if all(arm in summaries and summaries[arm]["p50_ms"] for arm in ("E0", "ADAPTIVE")):
        paired_ratio = summaries["ADAPTIVE"]["p50_ms"]/summaries["E0"]["p50_ms"]
    published = [row for row in measured if row.get("costmap_ack")]
    exact = len(published) == len(measured) and bool(published) and all(
        row.get("costmap_ack", {}).get("costmap_update_acknowledged") is True
        and all(int(row.get("costmap_ack", {}).get(key, 0) or 0) == 0 for key in (
            "costmap_ack_hard_mismatch_cells", "costmap_ack_soft_exact_mismatch_cells",
            "costmap_ack_stale_roi_cells", "costmap_ack_hash_mismatch",
            "costmap_ack_sequence_mismatch",
        )) and row.get("costmap_ack", {}).get("local_map_update_fallback") is not True
        for row in published
    )
    lifecycle = {
        "static_cache": dict(static_cache.LAST_CACHE_TELEMETRY),
        "current_rss_first_10_percent_p50": None,
        "current_rss_last_10_percent_p50": None,
        "current_rss_growth_bytes": None,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
        "dispatch_cache_active_count": dispatch_cache.active_count,
        "dispatch_cache_resident_bytes": dispatch_cache.resident_bytes,
        "dispatch_cache_evictions": dispatch_cache.evictions,
    }
    rss_values = [int(item["current_rss_bytes_after_release"]) for item in rss_trace if item["mode"] == "measured"]
    if rss_values:
        edge = max(1, len(rss_values)//10)
        lifecycle.update({
            "current_rss_first_10_percent_p50": statistics.median(rss_values[:edge]),
            "current_rss_last_10_percent_p50": statistics.median(rss_values[-edge:]),
            "current_rss_growth_bytes": statistics.median(rss_values[-edge:])-statistics.median(rss_values[:edge]),
        })
    required_samples = int(algorithm["gates"]["measured_samples_per_arm_min"])
    per_arm_samples = {
        arm: int(summaries.get(arm, {}).get("sample_count", 0)) for arm in ("E0", "ADAPTIVE")
    }
    samples_formal = all(value >= required_samples for value in per_arm_samples.values())
    peak_rss = int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024)
    adaptive_summary = summaries.get("ADAPTIVE", {})
    e0_summary = summaries.get("E0", {})
    semantic_rows = [
        row for row in measured
        if row.get("arm") == "ADAPTIVE" and row.get("semantic_success_counted") is True
    ]
    semantic_query_count = len({row["query_id"] for row in semantic_rows})
    tail_gate = bool(
        samples_formal
        and float(adaptive_summary.get("p95_ms") or float("inf"))
        <= float(algorithm["gates"]["adaptive_p95_ms_max"])
        and float(adaptive_summary.get("p99_ms") or float("inf"))
        <= float(algorithm["gates"]["adaptive_p99_ms_max"])
        and float(e0_summary.get("p95_ms") or float("inf"))
        <= float(algorithm["gates"]["e0_p95_ms_max"])
        and float(e0_summary.get("p99_ms") or float("inf"))
        <= float(algorithm["gates"]["e0_p99_ms_max"])
    )
    gates = {
        "query_count_gate": len(queries) >= int(algorithm["gates"]["expanded_query_count_min"]),
        "paired_arms_present": set(selected_arms) == {"E0", "ADAPTIVE"},
        "formal_sample_count_gate": samples_formal,
        "measured_samples_per_arm": per_arm_samples,
        "all_final_valid": all(row.get("final_valid_success") is True for row in measured),
        "all_exact_ack_without_full_repair": exact,
        # A no-path row has no safety assertion; all_final_valid remains the
        # independent hard gate.  For generated paths, retain every frozen
        # zero-violation gate and admit only sub-nanometric floating error in
        # the analytical 2.50 1/m equality calculation.
        "all_safety": all(
            (
                row.get("collision_violations") in (0, 0.0)
                and row.get("kinematic_violations") in (0, 0.0)
                and row.get("hard_semantic_violations") in (0, 0.0)
                and row.get("no_stopping_goal_violations") in (0, 0.0)
                and row.get("reverse_distance_m") in (0, 0.0)
                and row.get("rotate_in_place_count") in (0, 0.0)
                and float(row.get("maximum_curvature_1pm") or 0.0) <= 2.50 + 1.0e-9
            )
            for row in measured if row.get("path_length_m") is not None
        ),
        "latency_ratio_gate": paired_ratio is not None and paired_ratio <= 2.0,
        "tail_statistics_formal": samples_formal,
        "tail_latency_gate": tail_gate,
        "semantic_query_coverage_gate": (
            len(semantic_rows) >= int(algorithm["gates"]["semantic_success_rows_min"])
            and semantic_query_count >= int(algorithm["gates"]["semantic_success_queries_min"])
        ),
        "static_cache_activation_gate": (
            static_prepare_ms <= float(algorithm["gates"]["static_prepare_ms_max"])
            and lifecycle["static_cache"].get("status") == "HIT"
        ),
        "peak_rss_gate": peak_rss <= int(algorithm["gates"]["peak_rss_bytes_max"]),
        "memory_non_monotonic_unbounded": bool(
            lifecycle["current_rss_growth_bytes"] is not None
            and lifecycle["current_rss_growth_bytes"]
            <= int(algorithm["gates"]["steady_rss_growth_bytes_max"])
        ),
    }
    boolean_gates = [value for value in gates.values() if isinstance(value, bool)]
    gates["promotion_prerequisites_passed"] = all(boolean_gates)
    final = {
        **_identity(), "schema_version": SCHEMA_VERSION, "rows": rows,
        "summaries": summaries, "adaptive_over_e0_p50_ratio": paired_ratio,
        "static_prepare_ms": static_prepare_ms, "memory_lifecycle": lifecycle,
        "gate_results": gates, "wall_s": time.monotonic()-started,
        "peak_rss_bytes": peak_rss,
        "python": platform.python_version(), "processes_after": applicability._process_audit(),
    }
    applicability._write_json(output/"performance_summary.json", summaries | {"adaptive_over_e0_p50_ratio": paired_ratio})
    applicability._write_json(output/"memory_lifecycle.json", lifecycle)
    applicability._write_json(output/"gate_results.json", gates)
    applicability._write_json(output/"final_result.json", final)
    command = (
        f"ROS_DOMAIN_ID={domain} /usr/bin/python3 -m arena_evaluation.two_layer_v3_semantic_r14_expanded "
        f"--output {output} --config {Path(config_path).resolve()} --arms {','.join(selected_arms)} "
        f"--warmups {warmup_count} --repetitions {measured_count}"
    )
    (output/"reproduction_command.txt").write_text(command+"\n")
    snapshot = output/"source_snapshot"; snapshot.mkdir()
    for source in sources: shutil.copy2(source, snapshot/source.name)
    end_hashes = {str(path): sha256_file(path) for path in sources}
    if end_hashes != source_hashes:
        raise RuntimeError("r14 source changed during expanded run")
    applicability._write_json(output/"artifact_hashes.json", applicability._manifest_files(output))
    return final


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--arms", default="E0,ADAPTIVE")
    parser.add_argument("--query-ids", default="")
    parser.add_argument("--warmups", type=int)
    parser.add_argument("--repetitions", type=int)
    parser.add_argument("--ros-domain-id", type=int)
    args = parser.parse_args(argv)
    result = run(
        output=args.output, config_path=args.config,
        arms=[value for value in args.arms.split(",") if value],
        query_ids=[value for value in args.query_ids.split(",") if value],
        warmups=args.warmups, repetitions=args.repetitions,
        ros_domain_id=args.ros_domain_id,
    )
    print(json.dumps({
        "gate_results": result["gate_results"], "summaries": result["summaries"],
        "peak_rss_bytes": result["peak_rss_bytes"],
    }, indent=2, sort_keys=True))
    return 0 if result["gate_results"]["promotion_prerequisites_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
