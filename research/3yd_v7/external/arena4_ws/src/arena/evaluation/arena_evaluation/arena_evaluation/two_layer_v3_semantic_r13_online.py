"""Online exact-ACK adapter for the 2A-V3 r13 route-phase planner.

Every path is generated from the current request after Nav2's effective master
costmap has been byte-exactly acknowledged.  The adapter never loads an old
path or witness.  A selected8 soft fallback remains an explicit SE(2) search
result, is fully safety-audited, and is never counted as semantic success.
"""
from __future__ import annotations

import argparse
from array import array
import csv
import gc
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
from . import two_layer_v3_semantic_r13_benchmark as offline
from . import two_layer_v3_semantic_online as single_lane_online
from .planner_benchmark.models import Query
from .path_audit import PathAuditor
from .regional_preference_r1 import orient_route_for_query
from .regional_preference_r3 import RegionalPreferenceBuilderR3
from .semantic_constraint_core import ConstraintWorld
from .semantic_costmap_r2 import SemanticCostmapComposerR2
from .semantic_map import SemanticMapV1, canonical_hash, sha256_file
from .semantic_route_phase_v3 import LazyRoutePhaseSearch, OrientedRoute, RoutePhaseWorld
from .semantic_transition_lazy_corridor import LazyEdgeFactory, search_lazy
from .semantic_transition_ordered_corridor import CorridorFailure
from .semantic_v3_online_session import VerifiedV3PlannerSession


ARCHITECTURE_ID = "2A-V3"
IMPLEMENTATION_REVISION = "r13-route-phase-multisemantic-state-lattice"
PROTOCOL_ID = "PLN-02-2A-V3-R13-ROUTE-PHASE-MULTISEMANTIC-ONLINE-V1"
SCHEMA_VERSION = "PLN-02-2A-V3-R13-ONLINE-RESULT-V1"
DEFAULT_CONFIG = offline.PACKAGE_ROOT / "config/two_layer_v3_semantic_r13_online.yaml"


class RoutePhaseV3PlannerSession(VerifiedV3PlannerSession):
    """V3 exact session with inflation-safe overlap between ROI tiles.

    Nav2 Humble updates the inflation layer once per ``OccupancyGridUpdate``.
    Adjacent non-overlapping horizontal tiles can therefore leave the previous
    master value at their common boundary.  The final exact-content ACK catches
    this, but waiting for its timeout before a full repair adds two seconds.
    r13 retains bounded small messages, overlaps adjacent tiles, then republishes
    narrow source strips across every tile seam and the ROI top/bottom.  Each
    repair strip exceeds the frozen InflationLayer radius, so the last update
    sees source cells on both sides of the seam.  This changes no expected value
    and the complete effective master is still byte-verified.
    """

    PUBLICATION_VERSION = "2A-V3-r13-overlapped-roi-exact-v1"
    roi_tile_overlap_rows = 32
    roi_seam_repair_margin_rows = 16

    def _publish_dirty_roi(
        self, expected: np.ndarray, changed: np.ndarray,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        if self._current_grid is None:
            raise RuntimeError("ROI update has no initialized grid")
        changed_cells = np.argwhere(np.asarray(changed, dtype=bool))
        self._begin_publication(np.asarray(changed, dtype=bool), full=False)
        if changed_cells.size == 0:
            return self._current_grid.copy(), {
                "local_map_serialization_ms": 0.0,
                "local_map_publication_ms": 0.0,
                "costmap_settle_ms": 0.0,
                "roi_bbox": [0, 0, 0, 0],
                "roi_changed_cells": 0,
                "roi_tile_overlap_rows": 0,
            }
        min_y = int(changed_cells[:, 0].min())
        max_y = int(changed_cells[:, 0].max()) + 1
        min_x = int(changed_cells[:, 1].min())
        max_x = int(changed_cells[:, 1].max()) + 1
        width = max_x-min_x
        max_payload = max(1024, int(getattr(self, "roi_max_payload_bytes", 128_000)))
        rows_per_tile = max(1, max_payload//max(1, width))
        overlap = min(
            max(0, int(getattr(self, "roi_tile_overlap_rows", 32))),
            max(0, rows_per_tile-1),
        )
        step = max(1, rows_per_tile-overlap)
        serialization_ms = publication_ms = settle_ms = 0.0
        message_count = published_cells = max_message_bytes = 0
        tile_starts: list[int] = []
        tile_y = min_y
        while tile_y < max_y:
            tile_y1 = min(max_y, tile_y+rows_per_tile)
            tile_starts.append(tile_y)
            serialize_started = time.monotonic_ns()
            patch = np.ascontiguousarray(
                expected[tile_y:tile_y1, min_x:max_x], dtype=np.int8,
            )
            message = self.OccupancyGridUpdate()
            message.header.frame_id = "map"
            message.header.stamp = self.client.node.get_clock().now().to_msg()
            message.x = min_x
            message.y = tile_y
            message.width = width
            message.height = tile_y1-tile_y
            message.data = array("b", patch.tobytes())
            serialization_ms += (time.monotonic_ns()-serialize_started)/1.0e6
            publish_started = time.monotonic_ns()
            self._local_update_publisher.publish(message)
            publication_ms += (time.monotonic_ns()-publish_started)/1.0e6
            message_count += 1
            tile_cells = width*(tile_y1-tile_y)
            published_cells += tile_cells
            max_message_bytes = max(max_message_bytes, tile_cells)
            pacing_s = max(0.0, float(getattr(self, "roi_publish_pacing_s", .001)))
            if pacing_s:
                settle_started = time.monotonic_ns()
                self.client.executor.spin_once(timeout_sec=pacing_s)
                settle_ms += (time.monotonic_ns()-settle_started)/1.0e6
            if tile_y1 >= max_y:
                break
            tile_y += step
        repair_margin = max(
            1, int(getattr(self, "roi_seam_repair_margin_rows", 16)),
        )
        repair_ranges = []
        for seam in [min_y, *tile_starts[1:], max_y]:
            y0 = max(min_y, int(seam)-repair_margin)
            y1 = min(max_y, int(seam)+repair_margin)
            if y1 <= y0 or (y0, y1) in repair_ranges:
                continue
            repair_ranges.append((y0, y1))
        for y0, y1 in repair_ranges:
            serialize_started = time.monotonic_ns()
            patch = np.ascontiguousarray(expected[y0:y1, min_x:max_x], dtype=np.int8)
            message = self.OccupancyGridUpdate()
            message.header.frame_id = "map"
            message.header.stamp = self.client.node.get_clock().now().to_msg()
            message.x = min_x
            message.y = y0
            message.width = width
            message.height = y1-y0
            message.data = array("b", patch.tobytes())
            serialization_ms += (time.monotonic_ns()-serialize_started)/1.0e6
            publish_started = time.monotonic_ns()
            self._local_update_publisher.publish(message)
            publication_ms += (time.monotonic_ns()-publish_started)/1.0e6
            message_count += 1
            tile_cells = width*(y1-y0)
            published_cells += tile_cells
            max_message_bytes = max(max_message_bytes, tile_cells)
            pacing_s = max(0.0, float(getattr(self, "roi_publish_pacing_s", .001)))
            if pacing_s:
                settle_started = time.monotonic_ns()
                self.client.executor.spin_once(timeout_sec=pacing_s)
                settle_ms += (time.monotonic_ns()-settle_started)/1.0e6
        applied = self._current_grid.copy()
        applied[min_y:max_y, min_x:max_x] = expected[min_y:max_y, min_x:max_x]
        return applied, {
            "local_map_serialization_ms": serialization_ms,
            "local_map_publication_ms": publication_ms,
            "costmap_settle_ms": settle_ms,
            "roi_bbox": [min_x, min_y, width, max_y-min_y],
            "roi_changed_cells": int(changed_cells.shape[0]),
            "roi_published_cells": int(published_cells),
            "roi_message_count": int(message_count),
            "roi_max_message_bytes": int(max_message_bytes),
            "roi_publish_pacing_ms": float(getattr(self, "roi_publish_pacing_s", .001))*1000.0,
            "roi_tile_overlap_rows": int(overlap),
            "roi_seam_repair_margin_rows": int(repair_margin),
            "roi_seam_repair_messages": int(len(repair_ranges)),
        }


def _identity() -> dict[str, str]:
    return {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
    }


def _write_json(path: Path, payload: Any) -> None:
    applicability._write_json(path, payload)


def _load_config(
    path: Path,
) -> tuple[
    dict[str, Any], Path, dict[str, Any], dict[str, Any], Path,
    dict[str, Any], Path,
]:
    path = path.resolve()
    config = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    for key, value in _identity().items():
        if config.get(key) != value:
            raise ValueError(f"r13 online identity mismatch for {key}")
    binding = config["algorithm_config"]
    algorithm_path = Path(str(binding["path"]))
    if not algorithm_path.is_absolute():
        algorithm_path = path.parent / algorithm_path
    algorithm_path = algorithm_path.resolve()
    if sha256_file(algorithm_path) != str(binding["sha256"]):
        raise ValueError("r13 online algorithm binding changed")
    algorithm, parent, selected8_path = offline._load_config(algorithm_path)
    single_binding = config["single_lane_engine"]
    single_online_path = Path(str(single_binding["online_config_path"]))
    if not single_online_path.is_absolute():
        single_online_path = path.parent / single_online_path
    single_online_path = single_online_path.resolve()
    if sha256_file(single_online_path) != str(single_binding["online_config_sha256"]):
        raise ValueError("r13 single-lane engine binding changed")
    (
        _single_config, single_algorithm_path, _single_targeted_path,
        single_algorithm, single_parent,
    ) = single_lane_online._load_online_config(single_online_path)
    if single_algorithm.get("implementation_revision") != str(
        single_binding["algorithm_revision"]
    ):
        raise ValueError("r13 single-lane engine revision changed")
    for key in ("map_hash", "semantic_map_hash"):
        if single_algorithm["frozen_bindings"].get(key) != algorithm["frozen_bindings"].get(key):
            raise ValueError(f"r13 engines do not share frozen {key}")
    for key in (
        "resolution_m", "allow_reverse", "allow_in_place_rotation",
        "minimum_turning_radius_m", "maximum_curvature_1pm",
    ):
        if single_parent["protocol"].get(key) != parent["protocol"].get(key):
            raise ValueError(f"r13 engines do not share immutable {key}")
    motion = config["immutable_motion_contract"]
    required = {
        "yaw_bins": 48, "motion_model": "DUBIN", "allow_reverse": False,
        "allow_in_place_rotation": False, "minimum_turning_radius_m": .40,
        "maximum_curvature_1pm": 2.50, "footprint_half_length_m": .265,
        "footprint_half_width_m": .225,
    }
    for key, value in required.items():
        if motion.get(key) != value:
            raise ValueError(f"immutable motion contract changed for {key}")
    interface = config["online_interface"]
    if (
        interface.get("exact_effective_master_ack") is not True
        or float(interface.get("fixed_settle_s", -1.0)) != 0.0
        or interface.get("historical_path_input_allowed") is not False
        or int(interface.get("roi_tile_overlap_rows", -1)) != 32
        or int(interface.get("roi_seam_repair_margin_rows", -1)) != 16
        or int(interface.get("roi_max_payload_bytes", -1)) != 128_000
    ):
        raise ValueError("online fail-closed interface contract changed")
    return (
        config, algorithm_path, algorithm, parent, selected8_path,
        single_algorithm, single_algorithm_path,
    )


def _query_plan(scope: str, query_id: str, config: Mapping[str, Any]) -> dict[str, bool]:
    if scope == "targeted":
        policy = config["dispatch_policy"]["targeted"]
        return {
            "engine": str(policy["engine"]),
            "prefer_lazy": False,
            "allow_safe_soft_fallback": bool(policy["allow_safe_soft_fallback"]),
            "fallback_first": False,
            "strict_semantic_required": bool(policy["strict_semantic_required"]),
        }
    policy = config["dispatch_policy"]["selected8"]
    strict = set(map(str, policy["strict_semantic_query_ids"]))
    fallback = set(map(str, policy["safe_fallback_query_ids"]))
    single_lane = set(map(str, policy.get("single_lane_engine_query_ids", [])))
    if query_id in strict:
        return {
            "engine": "single_lane_r10" if query_id in single_lane else "route_phase_r13",
            "prefer_lazy": True, "allow_safe_soft_fallback": False,
            "fallback_first": False, "strict_semantic_required": True,
        }
    if query_id in fallback:
        return {
            "engine": "route_phase_r13",
            "prefer_lazy": True, "allow_safe_soft_fallback": True,
            "fallback_first": True, "strict_semantic_required": False,
        }
    raise ValueError(f"selected8 query is absent from frozen dispatch policy: {query_id}")


def _run_single_lane_search(
    *, input_dir: Path, query_id: str, verified_master: np.ndarray,
    algorithm: Mapping[str, Any], semantic_map: SemanticMapV1,
    metadata: Mapping[str, Any], arrays: Mapping[str, Any],
    canonical_auditor: PathAuditor,
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    """Run the frozen r10 engine with a shared immutable map auditor.

    The r10 algorithm, candidate provider, motion primitives, tie-break and
    acceptance gates are unchanged.  Only the full-map distance transform
    owned by :class:`PathAuditor` is built once per online process instead of
    once per request.  The request-specific allowed mask is still supplied to
    every canonical audit, so this cannot reuse a prior request verdict.
    """
    policy = applicability._ordered_policy(algorithm)
    world = ConstraintWorld(
        input_dir, query_id, master_override=verified_master,
        expected_master_hash=None, meta_override=metadata,
        arrays_override=arrays,
    )
    if semantic_map.semantic_map_hash != world.meta["semantic_map_hash"]:
        raise CorridorFailure("SEMANTIC_BINDING_MISMATCH", "semantic map changed")
    world._canonical_auditor = canonical_auditor
    world._canonical_allowed = world._full_allowed
    route = OrientedRoute(
        world.meta["route_polyline"], world.start, world.goal,
        endpoint_attachment_limit_m=policy.endpoint_attachment_limit_m,
    )
    lazy_policy = single_lane_online.offline._lazy_policy(algorithm)
    factory = LazyEdgeFactory(
        world, route, policy,
        candidate_cell_provider=single_lane_online.offline._provider(algorithm),
        valid_successors_per_target_layer=lazy_policy.valid_successors_per_target_layer,
    )
    witness, evaluations, search = search_lazy(
        factory, world, semantic_map, lazy_policy,
    )
    graph = factory.diagnostics()
    graph["fast_dense_edge_certificate_count"] = int(
        world.fast_dense_edge_certificates
    )
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
        "canonical_auditor_reused": True,
    }, witness


def _row_from_witness(query: Query, witness: Mapping[str, Any] | None) -> dict[str, Any]:
    if witness is None:
        return {
            "query_id": query.query_id, "category": query.category,
            "final_valid_success": False, "strict_semantic_gate_passed": False,
            "semantic_success_counted": False,
        }
    classes = witness["semantics"]["active_window"]["classes"]
    lane = classes["lane"]
    parking = classes["parking"]
    safety = witness.get("safety", {})
    canonical = witness.get("canonical", safety.get("canonical", {}))
    hard = witness["hard_features"]
    return {
        "query_id": query.query_id, "category": query.category,
        "final_valid_success": bool(
            witness.get("gate_passed") and canonical.get("final_valid_success")
        ),
        "strict_semantic_gate_passed": bool(
            witness.get("semantic_success_counted", False)
        ),
        "semantic_success_counted": bool(witness.get("semantic_success_counted", False)),
        "safe_negative_fallback": bool(witness.get("safe_negative_fallback", False)),
        "safe_soft_fallback": bool(witness.get("safe_soft_fallback", False)),
        "fallback_reason": witness.get("fallback_reason", ""),
        "lane_correct_side_ratio": lane.get("correct_side_ratio"),
        "lane_target_band_ratio": lane.get("target_band_ratio"),
        "lane_lateral_error_p50_m": lane.get("lateral_error_p50_m"),
        "lane_semantic_gate_passed": lane.get("semantic_gate_passed"),
        "parking_center_band_ratio": parking.get("parking_center_band_ratio"),
        "parking_normalized_deviation_p50": parking.get(
            "parking_center_normalized_deviation_p50"
        ),
        "parking_semantic_gate_passed": parking.get("semantic_gate_passed"),
        "path_length_m": witness.get("arc_length_m", safety.get("arc_length_m")),
        "maximum_curvature_1pm": witness.get(
            "maximum_control_curvature_1pm", canonical.get("maximum_curvature")
        ),
        "collision_violations": 0 if canonical.get("static_footprint_valid") else 1,
        "kinematic_violations": 0 if canonical.get("kinematic_valid") else 1,
        "hard_semantic_violations": 0 if hard.get("hard_feature_gate_passed") else 1,
        "no_stopping_goal_violations": hard.get("no_stopping_task_endpoint_violations"),
        "reverse_distance_m": canonical.get("reverse_distance_m"),
        "rotate_in_place_count": canonical.get("in_place_rotation_count"),
        "ordered_progress_gate_passed": witness["ordered_progress"].get(
            "ordered_progress_gate_passed"
        ),
        "revisit_screen_passed": witness["revisit"].get("revisit_screen_passed"),
    }


def _percentile(values: Sequence[float], quantile: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), quantile))


def run(
    *, output: Path, scope: str, config_path: Path = DEFAULT_CONFIG,
    extracted: Path = applicability.DEFAULT_EXTRACTED,
    semantic_map_path: Path = applicability.DEFAULT_SEMANTIC_MAP,
    topology_cache: Path = applicability.DEFAULT_TOPOLOGY,
    ros_domain_id: int | None = None, warmups: int | None = None,
    repetitions: int | None = None,
) -> dict[str, Any]:
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / "logs").mkdir()
    (output / "requests").mkdir()
    started = time.monotonic()
    (
        config, algorithm_path, algorithm, parent, selected8_path,
        single_algorithm, single_algorithm_path,
    ) = _load_config(config_path)
    query_path = selected8_path
    queries, query_hash, query_source = offline._query_scope(
        config=algorithm, query_path=query_path, scope=scope,
        map_path=extracted.resolve() / "optemap.pgm",
    )
    count_warmup = int(config["measurement"]["warmups"] if warmups is None else warmups)
    count_measured = int(config["measurement"]["repetitions"] if repetitions is None else repetitions)
    if count_warmup < 0 or count_measured < 1:
        raise ValueError("online repetitions require warmups>=0 and repetitions>=1")
    chosen_domain = int(
        config["online_interface"]["ros_domain_id"]
        if ros_domain_id is None else ros_domain_id
    )
    os.environ["ROS_DOMAIN_ID"] = str(chosen_domain)
    sources = [
        Path(__file__).resolve(), config_path.resolve(), algorithm_path,
        single_algorithm_path,
        query_source, Path(offline.__file__).resolve(),
        Path(__file__).with_name("semantic_route_phase_v3.py").resolve(),
        Path(__file__).with_name("semantic_v3_online_session.py").resolve(),
        semantic_map_path.resolve(),
    ]
    source_hashes = {str(path): sha256_file(path) for path in sources}
    reproduction = (
        f"ROS_DOMAIN_ID={chosen_domain} /usr/bin/python3 -m "
        "arena_evaluation.two_layer_v3_semantic_r13_online "
        f"--scope {scope} --output {shlex.quote(str(output))} "
        f"--config {shlex.quote(str(config_path.resolve()))} "
        f"--warmups {count_warmup} --repetitions {count_measured}"
    )
    (output / "reproduction_command.txt").write_text(reproduction + "\n", encoding="utf-8")
    _write_json(output / "protocol.json", {
        **_identity(), "schema_version": SCHEMA_VERSION,
        "scope": scope, "query_hash": query_hash, "query_count": len(queries),
        "ros_domain_id": chosen_domain, "warmups": count_warmup,
        "repetitions": count_measured, "configuration": config,
        "algorithm_configuration": algorithm,
        "single_lane_algorithm_configuration": single_algorithm,
        "planner_result_source": "fresh_request_derived_route_phase_se2_search",
        "used_historical_path_or_witness": False,
        "effective_master_source": "Nav2 GetCostmap after byte-exact content ACK",
        "source_hashes_at_start": source_hashes,
        "processes_before": applicability._process_audit(),
    })

    static_started = time.monotonic()
    with r3._r3_bindings():
        prepared = r1._prepare(
            extracted.resolve(), semantic_map_path.resolve(), topology_cache.resolve(),
            parent, output=None,
        )
    static_prepare_ms = (time.monotonic()-static_started)*1000.0
    ctx, semantic_map, raster, topology, _annotator, router = prepared[:6]
    canonical_auditor = PathAuditor(
        ctx, source_commit="2A-V3-r13-bound-source",
    )
    if ctx.map_sha256 != algorithm["frozen_bindings"]["map_hash"]:
        raise ValueError("online map hash mismatch")
    if semantic_map.semantic_map_hash != algorithm["frozen_bindings"]["semantic_map_hash"]:
        raise ValueError("online semantic map hash mismatch")
    switches = r1.ArmSwitches.parse(parent["ablation_arms"]["E4"])
    selector = r1._selector_for_arm(
        switches, topology, router, {},
        preferred_attachment_radius_m=float(parent["endpoint_attachment"]["preferred_radius_m"]),
        attachment_cost_weight=float(parent["endpoint_attachment"]["cost_weight"]),
    )
    builder = RegionalPreferenceBuilderR3(
        ctx.hospital_map, raster, policy=parent["regional_preference"],
        semantic_map=semantic_map,
    )
    composer = SemanticCostmapComposerR2(
        policy=parent["l3_soft_cost"], inflation_cache_capacity=2,
    )
    session = RoutePhaseV3PlannerSession(
        ctx, output, map_yaml=ctx.map_yaml,
        log_tag=f"2a_v3_r13_{scope}_{int(time.time())}", local_mask_updates=True,
        optimization_profile="v7_candidate", smac_parameter_profile="baseline",
        optimization_stage="step3_delta_map", enable_mask_reuse_noop=True,
        force_full_on_semantic_signature_change=False,
        costmap_ack_timeout_s=float(config["online_interface"]["exact_ack_timeout_s"]),
    )
    session.roi_tile_overlap_rows = int(
        config["online_interface"].get("roi_tile_overlap_rows", 32)
    )
    session.roi_seam_repair_margin_rows = int(
        config["online_interface"].get("roi_seam_repair_margin_rows", 16)
    )
    session.roi_max_payload_bytes = int(
        config["online_interface"].get("roi_max_payload_bytes", 128_000)
    )
    session.local_map_update_strategy = "roi_ack"
    rows: list[dict[str, Any]] = []
    session_started = time.monotonic()
    session.start()
    session_start_ms = (time.monotonic()-session_started)*1000.0
    try:
        for mode, count in (("warmup", count_warmup), ("measured", count_measured)):
            for repetition in range(1, count+1):
                for query in queries:
                    request_dir = output / "requests" / f"{mode}_{repetition:02d}_{query.query_id}"
                    request_dir.mkdir()
                    request_started = time.monotonic()
                    row: dict[str, Any] = {
                        **_identity(), "scope": scope, "mode": mode,
                        "repetition": repetition, "arm": "E5-2A-V3-r13",
                        "relaxation_level": "R0",
                        "used_historical_path_or_witness": False,
                    }
                    metadata = arrays = composition = witness = None
                    try:
                        reset_started = time.monotonic()
                        reset = session.reset_query_state(
                            f"{scope}:{mode}:{repetition}:{query.query_id}",
                            restore_base_map=False,
                        )
                        reset_ms = (time.monotonic()-reset_started)*1000.0
                        l1_started = time.monotonic()
                        _sn, _gn, route, reason = selector(
                            topology, query,
                            cache_mode=r1.r2_runtime.CACHE_MODE_OPTIMIZED, timing={},
                        )
                        if route is None:
                            raise CorridorFailure("L1_ROUTE_FAILED", str(reason))
                        route, orientation = orient_route_for_query(route, query)
                        l1_ms = (time.monotonic()-l1_started)*1000.0
                        metadata, arrays, composition = offline.prepare_query_input(
                            query=query, route=route, orientation=orientation, ctx=ctx,
                            semantic_map=semantic_map, raster=raster, topology=topology,
                            builder=builder, composer=composer, parent=parent,
                            query_set_hash=query_hash,
                        )
                        session.set_semantic_costmap(composition)
                        publication_started = time.monotonic()
                        ack = session.update_local_mask(arrays["publication_allowed"])
                        publication_ack_ms = (time.monotonic()-publication_started)*1000.0
                        readback_started = time.monotonic()
                        verified_master, verified = session.verified_master_snapshot(ack)
                        readback_ms = (time.monotonic()-readback_started)*1000.0
                        # Preserve exact ACK evidence even if the downstream
                        # independent search fails closed.
                        row.update({
                            "route_hash": metadata["route_hash"],
                            "route_polyline_hash": metadata["route_polyline_hash"],
                            "roi_hash": metadata["roi_hash"],
                            "expected_master_hash": metadata["expected_master_hash"],
                            "server_master_hash": verified["verified_master_hash"],
                            "costmap_ack": ack,
                            "verified_master": verified,
                        })
                        search_started = time.monotonic()
                        dispatch = _query_plan(scope, query.query_id, config)
                        if dispatch["engine"] == "single_lane_r10":
                            search_result, witness = _run_single_lane_search(
                                input_dir=request_dir, query_id=query.query_id,
                                verified_master=verified_master,
                                algorithm=single_algorithm,
                                semantic_map=semantic_map,
                                metadata=metadata, arrays=arrays,
                                canonical_auditor=canonical_auditor,
                            )
                            evaluations = search_result["candidate_evaluations"]
                            search = search_result["search"]
                            if witness is not None:
                                witness.update({
                                    "semantic_success_counted": bool(witness["gate_passed"]),
                                    "strict_semantic_gate_passed": bool(witness["gate_passed"]),
                                    "safe_negative_fallback": False,
                                    "safe_soft_fallback": False,
                                })
                        else:
                            world = RoutePhaseWorld(
                                request_dir, query.query_id, arrays_override=arrays,
                                meta_override=metadata, master_override=verified_master,
                                expected_master_hash=metadata["expected_master_hash"],
                            )
                            world._canonical_auditor = canonical_auditor
                            oriented = OrientedRoute(
                                metadata["route_polyline"], world.start, world.goal,
                                endpoint_attachment_limit_m=offline._policy(
                                    algorithm
                                ).endpoint_attachment_limit_m,
                            )
                            searcher = LazyRoutePhaseSearch(
                                world, oriented, offline._policy(algorithm),
                            )
                            witness, evaluations, search = searcher.search(
                                semantic_map,
                                allow_safe_soft_fallback=dispatch["allow_safe_soft_fallback"],
                                prefer_lazy=dispatch["prefer_lazy"],
                                fallback_first=dispatch["fallback_first"],
                            )
                        search_audit_ms = (time.monotonic()-search_started)*1000.0
                        if witness is None:
                            raise CorridorFailure("NO_ROUTE_PHASE_SE2_RESULT", "search returned no final-valid path")
                        if dispatch["strict_semantic_required"] and not witness.get("semantic_success_counted"):
                            raise CorridorFailure("STRICT_SEMANTIC_GATE_FAILED", query.query_id)
                        path_started = time.monotonic()
                        path_interface = session.publish_verified_path(
                            witness["points"], query_id=query.query_id,
                        )
                        path_echo_ms = (time.monotonic()-path_started)*1000.0
                        row.update(_row_from_witness(query, witness))
                        row.update({
                            "failure_code": "", "dispatch": dispatch,
                            "route_hash": metadata["route_hash"],
                            "route_polyline_hash": metadata["route_polyline_hash"],
                            "roi_hash": metadata["roi_hash"],
                            "expected_master_hash": metadata["expected_master_hash"],
                            "server_master_hash": verified["verified_master_hash"],
                            "costmap_ack": ack, "verified_master": verified,
                            "path_interface": path_interface,
                            "search": search, "candidate_evaluations": evaluations,
                            "session_reset": reset,
                            "timing": {
                                "reset_ms": reset_ms, "l1_ms": l1_ms,
                                **metadata["timing"],
                                "publication_and_ack_ms": publication_ack_ms,
                                "verified_readback_ms": readback_ms,
                                "se2_search_and_audit_ms": search_audit_ms,
                                "path_publication_echo_ms": path_echo_ms,
                            },
                        })
                    except (CorridorFailure, RuntimeError, ValueError) as error:
                        row.update(_row_from_witness(query, None))
                        row["failure_code"] = getattr(error, "code", type(error).__name__)
                        row["failure_detail"] = str(error)
                    row["request_wall_ms"] = (time.monotonic()-request_started)*1000.0
                    evidence_started = time.monotonic()
                    if metadata is not None and arrays is not None:
                        archive = request_dir / f"{query.query_id}.npz"
                        np.savez_compressed(archive, **arrays)
                        saved_meta = dict(metadata)
                        saved_meta["npz_sha256"] = sha256_file(archive)
                        _write_json(request_dir / f"{query.query_id}.json", saved_meta)
                    if witness is not None:
                        points = witness.pop("points")
                        controls = witness.pop("controls")
                        _write_json(request_dir / "path.json", points)
                        _write_json(request_dir / "controls.json", {"edges": controls})
                        _write_json(request_dir / "witness_audit.json", witness)
                    row["evidence_serialization_outside_request_ms"] = (
                        time.monotonic()-evidence_started
                    )*1000.0
                    row["peak_rss_bytes"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024
                    rows.append(row)
                    _write_json(
                        output / f"result_{mode}_{repetition:02d}_{query.query_id}.json",
                        row,
                    )
                    gc.collect()
    finally:
        session.close()

    flat_rows = []
    for row in rows:
        timing = row.get("timing", {})
        ack = row.get("costmap_ack", {})
        flat_rows.append({
            key: row.get(key) for key in (
                "architecture_id", "implementation_revision", "protocol_id", "scope",
                "mode", "repetition", "arm", "query_id", "category",
                "relaxation_level", "final_valid_success", "strict_semantic_gate_passed",
                "semantic_success_counted", "safe_soft_fallback", "fallback_reason",
                "failure_code", "lane_correct_side_ratio", "lane_target_band_ratio",
                "lane_lateral_error_p50_m", "parking_center_band_ratio",
                "parking_normalized_deviation_p50", "path_length_m",
                "maximum_curvature_1pm", "collision_violations", "kinematic_violations",
                "hard_semantic_violations", "no_stopping_goal_violations",
                "reverse_distance_m", "rotate_in_place_count", "request_wall_ms",
                "peak_rss_bytes",
            )
        } | {
            "l1_ms": timing.get("l1_ms"),
            "roi_build_ms": timing.get("roi_build_ms"),
            "field_build_ms": timing.get("field_build_ms"),
            "compose_ms": timing.get("compose_ms"),
            "publication_and_ack_ms": timing.get("publication_and_ack_ms"),
            "verified_readback_ms": timing.get("verified_readback_ms"),
            "se2_search_and_audit_ms": timing.get("se2_search_and_audit_ms"),
            "path_publication_echo_ms": timing.get("path_publication_echo_ms"),
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
    measured = [row for row in rows if row["mode"] == "measured"]
    ack_summary = {
        "measured_count": len(measured),
        "acknowledged_count": sum(
            row.get("costmap_ack", {}).get("costmap_update_acknowledged") is True
            for row in measured
        ),
        "hard_exact_mismatch_cells": sum(int(row.get("costmap_ack", {}).get("costmap_ack_hard_mismatch_cells", 0)) for row in measured),
        "soft_exact_mismatch_cells": sum(int(row.get("costmap_ack", {}).get("costmap_ack_soft_exact_mismatch_cells", 0)) for row in measured),
        "stale_roi_cells": sum(int(row.get("costmap_ack", {}).get("costmap_ack_stale_roi_cells", 0)) for row in measured),
        "hash_mismatch_count": sum(int(row.get("costmap_ack", {}).get("costmap_ack_hash_mismatch", 0)) for row in measured),
        "sequence_mismatch_count": sum(int(row.get("costmap_ack", {}).get("costmap_ack_sequence_mismatch", 0)) for row in measured),
    }
    ack_summary["gate_passed"] = bool(
        ack_summary["acknowledged_count"] == len(measured)
        and not any(ack_summary[key] for key in (
            "hard_exact_mismatch_cells", "soft_exact_mismatch_cells", "stale_roi_cells",
            "hash_mismatch_count", "sequence_mismatch_count",
        ))
    )
    _write_json(output / "exact_ack_summary.json", ack_summary)
    request_times = [float(row["request_wall_ms"]) for row in measured]
    performance = {
        "static_prepare_ms": static_prepare_ms,
        "nav2_session_start_ms": session_start_ms,
        "request_p50_ms": statistics.median(request_times),
        "request_p95_ms": _percentile(request_times, 95),
        "request_p99_ms": _percentile(request_times, 99),
        "request_samples_ms": request_times,
        "sample_size_warning": "P95/P99_DEBUG_ONLY" if len(request_times) < 100 else "FORMAL",
        "same_round_e0_ratio": "PENDING_EXTERNAL_PAIRED_ARM",
    }
    _write_json(output / "performance_summary.json", performance)
    final_valid = all(row.get("final_valid_success") is True for row in measured)
    safety = all(
        row.get(key) in (0, 0.0) for row in measured
        for key in (
            "collision_violations", "kinematic_violations", "hard_semantic_violations",
            "no_stopping_goal_violations", "reverse_distance_m", "rotate_in_place_count",
        )
    )
    strict = all(row.get("strict_semantic_gate_passed") is True for row in measured)
    path_echo = all(
        (row.get("path_interface") or {}).get("exact_echo_verified") is True
        for row in measured
    )
    gate = {
        "scope": scope,
        "all_measured_final_valid": final_valid,
        "all_measured_safety": safety,
        "all_measured_exact_path_echo": path_echo,
        "exact_effective_content_ack": ack_summary["gate_passed"],
        "all_measured_strict_semantic": strict,
        "semantic_success_count": sum(
            row.get("semantic_success_counted") is True for row in measured
        ),
        "safe_fallback_count": sum(row.get("safe_soft_fallback") is True for row in measured),
        "scope_gate_passed": bool(
            final_valid and safety and path_echo and ack_summary["gate_passed"]
            and (strict if scope == "targeted" else True)
        ),
    }
    _write_json(output / "gate_results.json", gate)
    end_hashes = {str(path): sha256_file(path) for path in sources}
    if end_hashes != source_hashes:
        raise RuntimeError("r13 source changed during online run")
    final = {
        **_identity(), "schema_version": SCHEMA_VERSION, "scope": scope,
        "query_hash": query_hash, "rows": rows, "gate_results": gate,
        "exact_ack_summary": ack_summary, "performance": performance,
        "static_prepare_ms": static_prepare_ms, "session_start_ms": session_start_ms,
        "wall_s": time.monotonic()-started,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
        "python": platform.python_version(), "source_hashes_at_end": end_hashes,
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
    parser.add_argument("--scope", choices=("targeted", "selected8"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--extracted", type=Path, default=applicability.DEFAULT_EXTRACTED)
    parser.add_argument("--semantic-map", type=Path, default=applicability.DEFAULT_SEMANTIC_MAP)
    parser.add_argument("--topology-cache", type=Path, default=applicability.DEFAULT_TOPOLOGY)
    parser.add_argument("--ros-domain-id", type=int)
    parser.add_argument("--warmups", type=int)
    parser.add_argument("--repetitions", type=int)
    args = parser.parse_args(argv)
    result = run(
        output=args.output, scope=args.scope, config_path=args.config,
        extracted=args.extracted, semantic_map_path=args.semantic_map,
        topology_cache=args.topology_cache, ros_domain_id=args.ros_domain_id,
        warmups=args.warmups, repetitions=args.repetitions,
    )
    print(json.dumps({
        "architecture_id": result["architecture_id"], "scope": result["scope"],
        "gate_passed": result["gate_results"]["scope_gate_passed"],
        "semantic_success_count": result["gate_results"]["semantic_success_count"],
        "wall_s": result["wall_s"],
    }, indent=2, sort_keys=True))
    return 0 if result["gate_results"]["scope_gate_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
