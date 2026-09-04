"""Independent 2D-V2 route/runtime integration.

2D-V2 deliberately composes already validated pieces instead of maintaining a
second ROI/ACK or path-validation implementation:

* :mod:`layered_2d_v1_r2_pipeline` owns exact-pose attachment caching and the
  edge-segment spatial index;
* :class:`GraphDStarLite` owns persistent ``g/rhs/OPEN/km`` state;
* :mod:`two_layer_v1_r2_roi_pathaudit_benchmark` owns the shared adaptive
  corridor cache contract; and
* :mod:`unified_four_backends_smoke` and :mod:`path_audit` own the server
  content ACK and canonical PathAudit implementations.

No 2D-V1 or 2A-V1 entry point imports this module, so their defaults remain
unchanged.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import dynamic_incremental_value as incremental
from . import layered_2d_v1_pipeline as v1
from . import layered_2d_v1_r2_pipeline as v1r2
from . import l1_l3_corridor_hybrid_smoke as candidate
from . import path_audit
from . import topology
from . import two_layer_v1_formal_benchmark as adaptive
from . import two_layer_v1_r1_cache_benchmark as cache_v1
from . import two_layer_v1_r2_roi_pathaudit_benchmark as runtime_r2
from . import unified_four_backends_smoke as runtime
from .dynamic_snapshot import DynamicSnapshot
from .graph_dstar_lite import GraphDStarLite, INF


ARCHITECTURE_ID = "2D-V2"
IMPLEMENTATION_REVISION = "r0-enhanced-runtime-v1"
PARENT_ARCHITECTURE = "2D-V1-r3"
PROTOCOL_VERSION = "PLN-02-EXP-V1"
CORRIDOR_PROFILE = "topology_turn_adaptive_2m_4m"
CORRIDOR_SEMANTICS = "raw_map_smac_aligned"
BASE_PADDING_M = 2.0
TURN_PADDING_M = 4.0
ANGLE_QUANTIZATION_BINS = 48
ROI_MAX_MESSAGE_BYTES = 128 * 1024


def stable_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False,
        default=str,
    ).encode("utf-8")).hexdigest()


def _elapsed_ms(started_ns: int) -> float:
    return (time.monotonic_ns() - started_ns) / 1.0e6


def _route_from_dstar(
    artifact: topology.TopologyArtifact,
    graph_view: Any,
    node_path: Sequence[int],
) -> topology.TopologyRoute:
    """Project a virtual-root D* path back onto the immutable topology."""
    topology_nodes = [int(node) for node in node_path if int(node) >= 0]
    graph_edge_ids = v1r2.v0._route_edge_ids(graph_view, node_path)
    edge_ids = [int(edge_id.split("_", 1)[1]) for edge_id in graph_edge_ids
                if str(edge_id).startswith("topology_")]
    edges = {int(edge.edge_id): edge for edge in artifact.graph.edges}
    polyline = []
    min_width = math.inf
    length_m = 0.0
    for index, edge_id in enumerate(edge_ids):
        edge = edges[edge_id]
        length_m += float(edge.length_m)
        min_width = min(min_width, float(edge.min_width_m))
        reverse = False
        if index + 1 < len(topology_nodes):
            reverse = not (
                int(edge.source) == topology_nodes[index]
                and int(edge.target) == topology_nodes[index + 1]
            )
        points = list(reversed(edge.polyline)) if reverse else list(edge.polyline)
        points = [[float(point[0]), float(point[1])] for point in points]
        if polyline and points:
            points = points[1:]
        polyline.extend(points)
    if not polyline:
        nodes = {int(node.node_id): node for node in artifact.graph.nodes}
        polyline = [[float(nodes[node].x), float(nodes[node].y)]
                    for node in topology_nodes if node in nodes]
    return topology.TopologyRoute(
        topology_nodes, edge_ids, float(length_m),
        0.0 if math.isinf(min_width) else float(min_width), polyline,
    )


@dataclass
class _StaticRouteState:
    planner: GraphDStarLite
    first_search_complete: bool = False
    call_count: int = 0


class PersistentDStarRouteSelector:
    """2D-V1-r2 attachments plus persistent Graph D* Lite route search."""

    def __init__(
        self, ctx: Any, artifact: topology.TopologyArtifact,
        topology_info: Mapping[str, Any], *, source_hash: str,
    ) -> None:
        self.ctx = ctx
        self.artifact = artifact
        self.graph_view = v1.build_static_topology_view(artifact)
        self.graph_view.metadata["topology_cache_key"] = str(
            topology_info.get("topology_cache_key", "")
        )
        self.snapshot = DynamicSnapshot.empty(
            snapshot_id="static-v2-s0", timestamp=0.0,
            map_version=str(ctx.map_sha256), map_shape=artifact.free_mask.shape,
        )
        self.pipeline = v1r2.Layered2DV1R2Pipeline(
            self.graph_view, footprint=runtime.FOOTPRINT, l3_planner=None,
            corridor_padding_m=BASE_PADDING_M, corridor_profile="padding",
            corridor_fallback_policy="none", base_map_hash=str(ctx.map_sha256),
            topology_cache_key=str(topology_info.get("topology_cache_key", "")),
            topology_source_hash=str(topology_info.get("topology_source_hash", "")),
            corridor_semantics=CORRIDOR_SEMANTICS,
        )
        self.source_hash = str(source_hash)
        self.states: Dict[str, _StaticRouteState] = {}
        self.reinitialize_count = 0

    def query_key(self, query: Any) -> str:
        return stable_hash({
            "architecture_id": ARCHITECTURE_ID,
            "source_hash": self.source_hash,
            "map_hash": self.ctx.map_sha256,
            "map_shape": list(self.artifact.free_mask.shape),
            "origin": list(self.ctx.hospital_map.origin),
            "resolution": float(self.ctx.hospital_map.resolution),
            "topology_hash": self.graph_view.metadata.get("topology_cache_key", ""),
            "start_pose": [float(value) for value in query.start],
            "goal_pose": [float(value) for value in query.goal],
            "footprint": runtime.FOOTPRINT,
            "rmin_m": 0.4,
            "snapshot_id": self.snapshot.snapshot_id,
            "blocked_edge_digest": stable_hash({}),
        })

    def cache_binding(self, query: Any) -> Mapping[str, Any]:
        return {
            "query_key": self.query_key(query),
            "dynamic_snapshot_id": self.snapshot.snapshot_id,
            "blocked_edge_digest": stable_hash({}),
            "source_hash": self.source_hash,
        }

    def __call__(
        self, _artifact: topology.TopologyArtifact, query: Any, *,
        cache_mode: str = candidate.CACHE_MODE_OPTIMIZED,
        timing: Optional[Dict[str, Any]] = None,
    ) -> Tuple[Optional[Any], Optional[Any], Optional[topology.TopologyRoute], str]:
        if cache_mode != candidate.CACHE_MODE_OPTIMIZED:
            raise ValueError("2D-V2 only permits the integrity-bound optimized attachment path")
        attach_started = time.monotonic_ns()
        starts, goals, attach = self.pipeline._attach(query, self.snapshot)
        attach_wall_ms = _elapsed_ms(attach_started)
        if not starts or not goals:
            if timing is not None:
                timing.update({"start_candidate_count": len(starts),
                               "goal_candidate_count": len(goals),
                               "route_search_ms": 0.0,
                               "route_construction_ms": 0.0,
                               "dstar_reinitialized": False})
            return None, None, None, "endpoint_candidates_empty"

        key = self.query_key(query)
        state = self.states.get(key)
        graph_build_ms = 0.0
        if state is None:
            graph_started = time.monotonic_ns()
            planner, _start, _goal, _positions = self.pipeline._make_graph(
                starts, goals, self.snapshot,
            )
            graph_build_ms = _elapsed_ms(graph_started)
            state = _StaticRouteState(planner)
            self.states[key] = state
        search = state.planner.compute_shortest_path(timeout_s=5.0)
        extraction_started = time.monotonic_ns()
        node_path = state.planner.extract_path()
        extraction_ms = _elapsed_ms(extraction_started)
        construction_started = time.monotonic_ns()
        route = None if node_path is None else _route_from_dstar(
            self.artifact, self.graph_view, node_path,
        )
        construction_ms = _elapsed_ms(construction_started)
        state.first_search_complete = True
        state.call_count += 1
        if route is not None:
            route.v2_route_signature = stable_hash({
                "nodes": route.node_ids, "edges": route.edge_ids,
                "snapshot_id": self.snapshot.snapshot_id,
            })
        if timing is not None:
            timing.update({
                "start_lookup_ms": float(attach.get("attachment_lookup_time_ms", 0.0)) / 2.0,
                "goal_lookup_ms": float(attach.get("attachment_lookup_time_ms", 0.0)) / 2.0,
                "start_collision_check_ms": float(attach.get("projection_connection_collision_filter_time_ms", 0.0)) / 2.0,
                "goal_collision_check_ms": float(attach.get("projection_connection_collision_filter_time_ms", 0.0)) / 2.0,
                "start_candidate_count": len(starts), "goal_candidate_count": len(goals),
                "candidate_pair_attempts": 1, "adjacency_build_ms": graph_build_ms,
                "route_search_ms": float(search.search_time_ms),
                "route_construction_ms": extraction_ms + construction_ms,
                "dstar_lite_search_ms": float(search.search_time_ms),
                "dstar_expanded_nodes": int(search.expanded_nodes),
                "dstar_generated_nodes": int(search.generated_nodes),
                "dstar_queue_pushes": int(search.queue_pushes),
                "dstar_queue_pops": int(search.queue_pops),
                "dstar_state_memory_bytes": int(incremental._dstar_memory_bytes(state.planner)),
                "dstar_reinitialized": False,
                "dstar_state_reused": state.call_count > 1,
                "dstar_state_call_count": state.call_count,
                "attachment_wall_ms": attach_wall_ms,
                "endpoint_candidate_cache_hit": bool(
                    attach.get("start_endpoint_cache_hit")
                    and attach.get("goal_endpoint_cache_hit")
                ),
                "endpoint_spatial_index_cache_hit": True,
                "route_cache_hit": state.call_count > 1,
            })
            timing.update({f"v2_{key}": value for key, value in attach.items()
                           if not isinstance(value, (list, dict))})
        return starts[0], goals[0], route, (
            "persistent_dstar_route" if route is not None else "no_candidate_pair_route"
        )


class V2AdaptiveRouteMaskCache(runtime_r2.R2RouteMaskCache):
    """Adaptive 2/4 m cache keyed by V2 route and complete runtime binding."""

    def __init__(self, ctx: Any, artifact: Any, source_hash: str, cache_root: Path,
                 *, selector: PersistentDStarRouteSelector,
                 corridor_mode: str = "adaptive_2m_4m") -> None:
        super().__init__(ctx, artifact, source_hash, cache_root, endpoint_mode="baseline")
        if corridor_mode not in {"adaptive_2m_4m", "uniform_2m"}:
            raise ValueError("corridor_mode must be adaptive_2m_4m or uniform_2m")
        self.v2_selector = selector
        self.corridor_mode = corridor_mode

    def route_selector(self, artifact: Any, query: Any, *, cache_mode: str,
                       timing: Optional[Dict[str, Any]] = None):
        return self.v2_selector(artifact, query, cache_mode=cache_mode, timing=timing)

    def key(self, route: Any, query: Any, start_cell: Any, goal_cell: Any) -> str:
        return stable_hash({
            "base_key": cache_v1.RouteMaskCache.key(
                self, route, query, start_cell, goal_cell,
            ),
            "complete_start_pose": [float(value) for value in query.start],
            "complete_goal_pose": [float(value) for value in query.goal],
            "runtime": dict(self.v2_selector.cache_binding(query)),
            "corridor_mode": self.corridor_mode,
            "profile": CORRIDOR_PROFILE,
            "base_padding_m": BASE_PADDING_M,
            "turn_padding_m": TURN_PADDING_M,
            "minimum_turning_radius_m": 0.4,
            "footprint": runtime.FOOTPRINT,
            "resolution": float(self.ctx.hospital_map.resolution),
            "source_hash": self.source_hash,
        })

    def prepare(self, queries: Sequence[Any]) -> Dict[str, Any]:
        started_ns = time.monotonic_ns()
        graph_edges = {int(edge.edge_id): edge for edge in self.topology.graph.edges}
        for query in queries:
            start_cell, goal_cell = candidate._endpoint_cells(self.ctx, query)
            _start, _goal, route, _reason = self.route_selector(
                self.topology, query, cache_mode=candidate.CACHE_MODE_OPTIMIZED,
                timing={},
            )
            if route is None:
                continue
            key = self.key(route, query, start_cell, goal_cell)
            self._prepared_routes[query.query_id] = (key, route, start_cell, goal_cell)
            if key not in self.route_masks:
                if self.corridor_mode == "adaptive_2m_4m":
                    mask, diagnostics = adaptive.build_adaptive_corridor_mask(
                        self.ctx, self.topology, route, query, start_cell, goal_cell,
                        BASE_PADDING_M, CORRIDOR_SEMANTICS,
                    )
                else:
                    mask = candidate._build_corridor_mask(
                        self.ctx, self.topology, route, query, start_cell, goal_cell,
                        BASE_PADDING_M, CORRIDOR_SEMANTICS,
                    )
                    diagnostics = {
                        "corridor_mask_strategy": "uniform_2m_ablation",
                        "base_corridor_padding_m": BASE_PADDING_M,
                        "corner_corridor_padding_m": 0.0,
                        "no_6m_padding": True,
                    }
                diagnostics = dict(diagnostics)
                diagnostics.update({
                    "precomputed_mask_hash": runtime_r2._grid_hash(mask),
                    "precomputed_allowed_cells": int(np.count_nonzero(mask)),
                    "route_signature": self.route_signature(route),
                    "mask_cache_key": key,
                })
                self.route_masks[key] = (np.asarray(mask, dtype=bool), diagnostics)
                self.route_analysis[key] = dict(diagnostics)
                self.route_misses += 1
            for edge_id in route.edge_ids:
                edge = graph_edges.get(int(edge_id))
                if edge is not None:
                    self._cache_edge(edge, BASE_PADDING_M)
                    self._cache_edge(edge, TURN_PADDING_M)
            endpoint_key = self._endpoint_key(query, start_cell, goal_cell)
            self.endpoint_strips.setdefault(endpoint_key, {
                "query_id": query.query_id, "start_cell": list(start_cell),
                "goal_cell": list(goal_cell),
            })
        self.offline_build_ms = _elapsed_ms(started_ns)
        manifest = {
            "cache_version": "2d-v2-r0-adaptive-route-mask-v1",
            "architecture_id": ARCHITECTURE_ID,
            "implementation_revision": IMPLEMENTATION_REVISION,
            "map_sha256": self.ctx.map_sha256,
            "map_shape": list(self.topology.free_mask.shape),
            "origin": list(self.ctx.hospital_map.origin),
            "resolution": float(self.ctx.hospital_map.resolution),
            "topology_hash": self.topology_hash,
            "source_hash": self.source_hash,
            "profile": CORRIDOR_PROFILE,
            "corridor_mode": self.corridor_mode,
            "base_padding_m": BASE_PADDING_M,
            "turn_padding_m": TURN_PADDING_M,
            "route_count": len(self.route_masks),
            "edge_entry_count": len(self.edge_masks),
            "edge_cache_bytes": self.edge_cache_bytes,
            "offline_build_ms": self.offline_build_ms,
        }
        self.cache_root.mkdir(parents=True, exist_ok=True)
        (self.cache_root / "mask_cache_manifest.yaml").write_text(
            yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8",
        )
        return manifest


def corridor_dirty_transition(old_mask: np.ndarray, new_mask: np.ndarray) -> Dict[str, Any]:
    """Return the exact old/new corridor dirty union and closure certificate."""
    old = np.asarray(old_mask, dtype=bool)
    new = np.asarray(new_mask, dtype=bool)
    if old.shape != new.shape:
        raise ValueError("old and new corridor masks must have the same shape")
    dirty = old ^ new
    close = old & ~new
    open_ = new & ~old
    rows, columns = np.nonzero(dirty)
    bbox = None if not len(rows) else [int(rows.min()), int(rows.max()) + 1,
                                      int(columns.min()), int(columns.max()) + 1]
    applied = old.copy()
    applied[close] = False
    applied[open_] = True
    return {
        "dirty_mask": dirty, "close_mask": close, "open_mask": open_,
        "dirty_cells": int(np.count_nonzero(dirty)),
        "closed_cells": int(np.count_nonzero(close)),
        "opened_cells": int(np.count_nonzero(open_)),
        "dirty_bbox": bbox,
        "old_corridor_residual_cells": int(np.count_nonzero(applied ^ new)),
        "result_hash": hashlib.sha256(np.ascontiguousarray(applied, dtype=np.uint8).tobytes()).hexdigest(),
    }


class PersistentDynamicEpisode:
    """Pure-L1 V2 episode with the protocol state machine and no reinitialize."""

    def __init__(self, template: incremental.GraphTemplate,
                 edge_cells: Mapping[str, Sequence[Tuple[int, int]]], *,
                 map_version: str, map_shape: Sequence[int]) -> None:
        self.template = template
        self.overlay = incremental.DynamicEdgeOverlay(
            edge_cells, map_version=map_version, map_shape=map_shape,
        )
        self.index = self.overlay.index
        self.planner: Optional[GraphDStarLite] = None
        self.reinitialize_count = 0
        self.snapshot_count = 0

    def step(self, payload: str) -> Dict[str, Any]:
        prepared = self.overlay.consume_json(payload)
        if not prepared.accepted:
            return {"accepted": False, "failure_code": prepared.rejection_reason,
                    "reinitialize_count": self.reinitialize_count}
        if self.planner is None:
            self.planner = self.template.new_dstar(prepared.statuses)
            initial = True
        else:
            initial = False
            for edge_id in prepared.changed_edges:
                self.planner.edge_cost_override.pop(str(edge_id), None)
            self.planner.update_edges(
                prepared.changed_edges, statuses=prepared.changed_statuses,
            )
        search = self.planner.compute_shortest_path()
        node_path = self.planner.extract_path()
        edge_path, cost = incremental._path_edges_and_cost(
            self.template.adjacency, node_path, prepared.statuses,
        )
        blocked = {edge for edge, status in prepared.statuses.items()
                   if status == GraphDStarLite.BLOCKED}
        if blocked.intersection(edge_path):
            raise AssertionError("D* output contains a BLOCKED edge")
        oracle = incremental.deterministic_graph_astar(self.template, prepared.statuses)
        reachable = node_path is not None
        cost_error = 0.0 if not reachable else abs(float(cost) - float(oracle.cost))
        if reachable != (oracle.node_path is not None) or cost_error > 1e-9:
            raise AssertionError("persistent D* diverged from deterministic Graph A* oracle")
        self.snapshot_count += 1
        return {
            "accepted": True, "initial_plan": initial,
            "snapshot_id": prepared.snapshot.snapshot_id,
            "changed_edge_count": len(prepared.changed_edges),
            "reachable": reachable, "edge_path": list(edge_path),
            "path_cost": None if not reachable else float(cost),
            "expanded_nodes": int(search.expanded_nodes),
            "generated_nodes": int(search.generated_nodes),
            "queue_pushes": int(search.queue_pushes),
            "queue_pops": int(search.queue_pops),
            "search_ms": float(search.search_time_ms),
            "state_memory_bytes": int(incremental._dstar_memory_bytes(self.planner)),
            "g_rhs_open_km_reused": not initial,
            "reinitialize_count": self.reinitialize_count,
            "oracle_cost_error": cost_error,
        }


PathAuditor = path_audit.PathAuditor
SmacSession = runtime.SmacSession
