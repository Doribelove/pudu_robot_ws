"""Independent ``2D-V1`` pipeline.

``2D-V1`` keeps the same static topology representation as ``2A-V0`` and
replaces only the L1 graph searcher with graph-level D* Lite.  It deliberately
does not refine skeleton edges: L1 states are the persisted topology node ids
from :mod:`topology`, while L3 remains the existing full-corridor Smac Hybrid
DUBIN planner.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import resource
import time
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import topology
from .dynamic_snapshot import DynamicSnapshot
from .graph_dstar_lite import GraphDStarLite, GraphEdge, INF
from .layered_2d_v0_pipeline import (
    AttachmentCandidate,
    Layered2DV0Pipeline,
    RefinedEdge,
    RefinedNode,
    RefinedNodeSpatialIndex,
    RefinedTopology,
    SmacHybridAdapter,
    _enrich_path,
    _load_queries,
    _source_hash,
    _write_csv,
    prepare_static_topology,
)


ARCHITECTURE_ID = "2D-V1"
IMPLEMENTATION_REVISION = "r1"
PROTOCOL_VERSION = "PLN-02-EXP-V1"
L1_BACKEND = "static_skeleton_graph_dstar_lite"
L3_BACKEND = "Nav2 SmacPlannerHybrid DUBIN"
RESOLUTION_M = 0.05
RMIN_M = 0.40
MAX_CURVATURE = 2.50
ALLOW_REVERSE = False
ALLOW_IN_PLACE_ROTATION = False
DEFAULT_CORRIDOR_PADDING_M = 2.0
DEFAULT_ENDPOINT_RADIUS_M = 8.0
DEFAULT_CANDIDATE_LIMIT = 16
# Endpoint attachments are ranked before the topology distance.  This keeps
# the D* Lite route deterministic and compatible with the validated 2A
# attachment policy: use the nearest feasible attachment, then fall back to a
# farther candidate only when the nearer one cannot reach the opposite side.
ATTACHMENT_RANK_PENALTY_M = 100.0


def build_static_topology_view(artifact: topology.TopologyArtifact) -> RefinedTopology:
    """Adapt the original 2A topology artifact without inserting nodes.

    The returned view has the fields consumed by the shared L3 pipeline, but
    every node and edge maps one-to-one to the original persisted topology.
    """
    nodes = {
        int(node.node_id): RefinedNode(
            node_id=int(node.node_id),
            x=float(node.x),
            y=float(node.y),
            role="original",
            component_id=int(node.component_id),
            clearance_m=float(node.clearance_m),
            source_node_id=int(node.node_id),
        )
        for node in artifact.graph.nodes
    }
    edges: Dict[str, RefinedEdge] = {}
    for source_edge in artifact.graph.edges:
        edge_id = f"topology_{int(source_edge.edge_id)}"
        polyline = [[float(point[0]), float(point[1])] for point in source_edge.polyline]
        cells: List[Tuple[int, int]] = []
        for first, second in zip(polyline, polyline[1:]):
            start = artifact.hospital_map.world_to_cell(*first)
            end = artifact.hospital_map.world_to_cell(*second)
            if start is None or end is None:
                continue
            steps = max(1, abs(int(end[0]) - int(start[0])), abs(int(end[1]) - int(start[1])))
            for index in range(steps + 1):
                fraction = index / steps
                cells.append((
                    int(round(start[0] + fraction * (end[0] - start[0]))),
                    int(round(start[1] + fraction * (end[1] - start[1]))),
                ))
        if not cells and polyline:
            cell = artifact.hospital_map.world_to_cell(*polyline[0])
            if cell is not None:
                cells.append((int(cell[0]), int(cell[1])))
        if polyline:
            tangent = math.atan2(polyline[-1][1] - polyline[0][1], polyline[-1][0] - polyline[0][0])
        else:
            tangent = 0.0
        edges[edge_id] = RefinedEdge(
            edge_id=edge_id,
            source_node=int(source_edge.source),
            target_node=int(source_edge.target),
            polyline=polyline,
            length_m=float(source_edge.length_m),
            min_clearance_m=float(source_edge.min_clearance_m),
            mean_clearance_m=float(source_edge.mean_clearance_m),
            corridor_width_m=float(source_edge.min_width_m),
            local_tangent=tangent,
            turn_support_nodes=[],
            corridor_mask_id=f"source_edge_{int(source_edge.edge_id)}",
            static_cost=float(source_edge.length_m) + 0.25 / max(0.05, float(source_edge.min_clearance_m)),
            source_edge_id=int(source_edge.edge_id),
            edge_cells=tuple(sorted(set(cells))),
        )
    metadata = {
        "schema_version": 1,
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "topology_representation": "2a_v0_static_skeleton_graph",
        "refinement_enabled": False,
        "source_topology_algorithm": artifact.metadata.get("algorithm", topology.TOPOLOGY_ALGORITHM_VERSION),
        "skeleton_backend": artifact.metadata.get("skeleton_backend", "unknown"),
        "graph_nodes": len(nodes),
        "graph_edges": len(edges),
        "static_map_hash": artifact.metadata.get("map_sha256", ""),
        "resolution": float(artifact.hospital_map.resolution),
    }
    return RefinedTopology(
        artifact,
        nodes,
        edges,
        metadata,
        attachment_index=RefinedNodeSpatialIndex.build(nodes),
        edge_safety_certificates={edge_id: True for edge_id in edges},
    )


class Layered2DV1Pipeline(Layered2DV0Pipeline):
    """2D-V1 scheduler using the original topology and graph D* Lite."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        # Match the validated 2A raw-map corridor semantics: the requested
        # padding is followed by fixed footprint and bend margins. This is a
        # V1-only profile; the 2D-V0/2A entry points retain their defaults.
        kwargs.setdefault("corridor_extra_margin_m", 0.20)
        super().__init__(*args, **kwargs)

    @staticmethod
    def _with_identity(result: Any) -> Any:
        diagnostics = dict(getattr(result, "diagnostics", {}) or {})
        diagnostics.update({
            "architecture_id": ARCHITECTURE_ID,
            "implementation_revision": IMPLEMENTATION_REVISION,
            "l1_backend": L1_BACKEND,
            "l1_state_type": "original_topology_node_id",
            "topology_refinement_enabled": False,
            "l2_called": False,
            "l2_call_count": 0,
            "rrtstar_call_count": 0,
            "sst_call_count": 0,
        })
        return type(result)(result.success, result.points, result.failure_code, result.snapshot_id, diagnostics)

    def plan_initial(self, query: Any, snapshot: Optional[DynamicSnapshot] = None):
        return self._with_identity(super().plan_initial(query, snapshot))

    def update_dynamic(self, query: Any, snapshot: DynamicSnapshot):
        return self._with_identity(super().update_dynamic(query, snapshot))

    @staticmethod
    def _nearest_edge_endpoint(candidate: AttachmentCandidate, refined: Any) -> AttachmentCandidate:
        """Keep an edge projection connected to its nearest endpoint.

        The original two-layer route selector attaches to a concrete skeleton
        node.  Connecting a projection to both edge endpoints lets D* Lite
        silently choose the opposite side of a doorway, changing the corridor
        geometry even when the projection is effectively at one endpoint.
        Ties are resolved toward the larger node id, which is the downstream
        endpoint for the persisted edge ordering and remains deterministic.
        """
        if str(candidate.role) != "edge_projection" or len(candidate.connections) <= 1:
            return candidate
        scored = []
        for node_id, cost in candidate.connections:
            node = refined.nodes.get(int(node_id))
            if node is None:
                continue
            distance = math.hypot(float(node.x) - float(candidate.x), float(node.y) - float(candidate.y))
            scored.append((distance, -int(node_id), int(node_id), float(cost)))
        if not scored:
            return candidate
        _distance, _tie, node_id, cost = min(scored)
        candidate.connections = ((node_id, cost),)
        return candidate

    def _attach(self, query: Any, snapshot: DynamicSnapshot):
        starts, goals, diagnostics = super()._attach(query, snapshot)
        starts = [self._nearest_edge_endpoint(candidate, self.refined) for candidate in starts]
        goals = [self._nearest_edge_endpoint(candidate, self.refined) for candidate in goals]

        # The 2A reference selector searches concrete skeleton nodes. Keep
        # those candidates ahead of edge projections so a projection that is
        # a few centimetres closer cannot move the route to the other side of
        # a doorway. Edge projections remain available as bounded fallbacks.
        def candidate_order(candidate: AttachmentCandidate) -> Tuple[int, float, float, int]:
            return (
                0 if str(candidate.role) == "original" else 1,
                float(candidate.distance_m),
                float(candidate.heading_error_rad),
                int(candidate.candidate_id),
            )

        starts.sort(key=candidate_order)
        goals.sort(key=candidate_order)

        def serialize(candidate: AttachmentCandidate) -> Dict[str, Any]:
            return {
                "candidate_id": int(candidate.candidate_id),
                "x": float(candidate.x), "y": float(candidate.y),
                "component_id": int(candidate.component_id),
                "role": str(candidate.role),
                "distance_m": float(candidate.distance_m),
                "heading_error_rad": float(candidate.heading_error_rad),
                "connections": [
                    {"node_id": int(node_id), "cost": float(cost)}
                    for node_id, cost in candidate.connections
                ],
            }

        diagnostics = dict(diagnostics)
        diagnostics["start_attachment_candidates"] = [serialize(candidate) for candidate in starts]
        diagnostics["goal_attachment_candidates"] = [serialize(candidate) for candidate in goals]
        diagnostics["endpoint_attachment_policy"] = "nearest_feasible_candidate_ranked"
        diagnostics["attachment_rank_penalty_m"] = ATTACHMENT_RANK_PENALTY_M
        return starts, goals, diagnostics

    def _make_graph(self, starts: Sequence[Any], goals: Sequence[Any], snapshot: DynamicSnapshot):
        # Build the same directed virtual-endpoint graph as V0, but make the
        # candidate rank explicit.  Without this bounded rank term the
        # topology distance can outweigh a nearby attach point and select a
        # different doorway than the validated 2A route.
        base_nodes = set(self.refined.nodes)
        base_edges = [
            GraphEdge(
                edge.edge_id, edge.source_node, edge.target_node,
                edge.length_m, edge.static_cost, edge.min_clearance_m,
                bidirectional=True,
            )
            for edge in self.refined.edges.values()
        ]
        virtual_positions: Dict[int, Tuple[float, float]] = {}
        start_virtual = -1000000
        goal_virtual = -2000000
        for index, candidate in enumerate(starts):
            candidate_id = start_virtual - 100 - index
            virtual_positions[candidate_id] = (float(candidate.x), float(candidate.y))
            for target, cost in candidate.connections:
                base_edges.append(GraphEdge(
                    f"attach_start_{index}_{target}", candidate_id,
                    int(target), float(cost), bidirectional=False,
                ))
            base_nodes.add(candidate_id)
        for index, candidate in enumerate(goals):
            candidate_id = goal_virtual - 100 - index
            virtual_positions[candidate_id] = (float(candidate.x), float(candidate.y))
            for target, cost in candidate.connections:
                base_edges.append(GraphEdge(
                    f"attach_goal_{index}_{target}", int(target),
                    candidate_id, float(cost), bidirectional=False,
                ))
            base_nodes.add(candidate_id)

        root_edges = []
        for index, candidate in enumerate(starts):
            root_edges.append(GraphEdge(
                f"root_start_{index}", start_virtual,
                start_virtual - 100 - index,
                float(candidate.distance_m + candidate.heading_error_rad
                      + ATTACHMENT_RANK_PENALTY_M * index),
                bidirectional=False,
            ))
        for index, candidate in enumerate(goals):
            root_edges.append(GraphEdge(
                f"root_goal_{index}", goal_virtual - 100 - index,
                goal_virtual,
                float(candidate.distance_m + candidate.heading_error_rad
                      + ATTACHMENT_RANK_PENALTY_M * index),
                bidirectional=False,
            ))
        base_edges.extend(root_edges)
        base_nodes.update((start_virtual, goal_virtual))
        planner = GraphDStarLite(
            base_nodes, base_edges, start_virtual, goal_virtual,
            edge_status=self.edge_status,
            edge_cost_override=self.edge_cost_override,
        )
        planner.node_positions = {
            int(node_id): (float(node.x), float(node.y))
            for node_id, node in self.refined.nodes.items()
        }
        planner.node_positions.update(virtual_positions)
        planner.state_representation = "original_topology_node_id"
        return planner, start_virtual, goal_virtual, virtual_positions


def _path_hash(points: Sequence[Mapping[str, Any]]) -> str:
    normalized = [{key: value for key, value in point.items() if key != "path_hash"} for point in points]
    return hashlib.sha256(json.dumps(normalized, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _load_map_context(map_yaml: Path, artifact: topology.TopologyArtifact) -> Any:
    from . import unified_four_backends_smoke as legacy
    from .planner_benchmark.map_utils import sha256_file

    return legacy.MapContext(
        artifact.hospital_map.map_id,
        artifact.hospital_map,
        artifact.free_mask,
        artifact.distance_m,
        sha256_file(artifact.hospital_map.image_path),
        sha256_file(artifact.hospital_map.yaml_path),
        map_yaml,
    )


def _run_ros_smoke(args: argparse.Namespace) -> int:
    if args.map_yaml is None or args.query_json is None:
        raise ValueError("ROS-backed execution requires --map-yaml and --query-json")
    from . import unified_four_backends_smoke as legacy
    from .planner_benchmark.map_utils import HospitalMap

    output = Path(args.output_dir).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir(exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = str(int(args.ros_domain_id))
    hospital_map = HospitalMap.load(args.map_yaml)
    if not math.isclose(float(hospital_map.resolution), RESOLUTION_M, rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError(f"map resolution must be {RESOLUTION_M}, got {hospital_map.resolution}")
    artifact, topology_info = prepare_static_topology(
        hospital_map,
        legacy.FOOTPRINT,
        args.topology_cache_dir or (output / "topology_cache"),
        padding_m=0.05,
        safety_margin_m=0.05,
        allow_unknown=False,
    )
    graph_view = build_static_topology_view(artifact)
    queries = _load_queries(args.query_json)
    if args.query_ids:
        requested = {str(value) for value in args.query_ids}
        queries = [query for query in queries if query.query_id in requested]
    spec = legacy.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"Smac Hybrid backend unavailable: {spec.reason}")
    ctx = _load_map_context(args.map_yaml, artifact)
    session = legacy.SmacSession(
        ctx,
        output,
        map_yaml=args.map_yaml,
        log_tag=f"2d_v1_{hospital_map.map_id}",
        local_mask_updates=True,
        optimization_profile="v7_candidate",
        smac_parameter_profile="baseline",
        optimization_stage="step3_delta_map",
    )
    session.start()
    adapter = SmacHybridAdapter(session, spec, footprint=legacy.FOOTPRINT, source_commit=legacy._source_commit())
    pipeline = Layered2DV1Pipeline(
        graph_view,
        footprint=legacy.FOOTPRINT,
        l3_planner=adapter,
        corridor_padding_m=float(args.corridor_padding_m),
        corridor_profile="padding",
        corridor_fallback_policy=str(args.corridor_fallback_policy),
        validator=lambda _map, query, points: legacy.validate_path(ctx, query, points),
    )
    snapshot = DynamicSnapshot.empty(snapshot_id="static", map_shape=artifact.free_mask.shape)
    rows: List[Dict[str, Any]] = []
    calls: List[Dict[str, Any]] = []
    metrics: List[Dict[str, Any]] = []
    try:
        for query in queries:
            before_calls = int(adapter.calls)
            started = time.monotonic_ns()
            result = pipeline.plan_initial(query, snapshot)
            query_calls = int(adapter.calls) - before_calls
            diagnostics = dict(result.diagnostics)
            points = list(result.points or []) if result.points else []
            final_valid = bool(result.success and diagnostics.get("final_valid_success", True))
            run_id = f"{hospital_map.map_id}_{query.query_id}_2d_v1"
            path_hash = ""
            path_file = ""
            if points:
                path_hash = str(points[0].get("path_hash", "")) or _enrich_path(points, legacy._source_commit())
                path_file = f"paths/{run_id}.json"
                (output / path_file).write_text(json.dumps(points, indent=2, sort_keys=True), encoding="utf-8")
            wall_ms = (time.monotonic_ns() - started) / 1.0e6
            row = {
                "run_id": run_id,
                "architecture_id": ARCHITECTURE_ID,
                "implementation_revision": IMPLEMENTATION_REVISION,
                "protocol_version": PROTOCOL_VERSION,
                "map_id": hospital_map.map_id,
                "query_id": query.query_id,
                "l1_backend": L1_BACKEND,
                "l1_state_type": "original_topology_node_id",
                "topology_refinement_enabled": False,
                "topology_node_count": len(graph_view.nodes),
                "topology_edge_count": len(graph_view.edges),
                "topology_cache_hit": topology_info.get("topology_cache_hit", False),
                "topology_cache_load_time_ms": topology_info.get("topology_load_time_ms", 0.0),
                "topology_build_count": topology_info.get("topology_build_count", 0),
                "l1_success": bool(diagnostics.get("topology_node_ids")),
                "l2_called": False,
                "l2_call_count": 0,
                "l3_backend": L3_BACKEND,
                "l3_call_count": query_calls,
                "l3_call_count_total": int(adapter.calls),
                "topology_node_ids": diagnostics.get("topology_node_ids", []),
                "topology_edge_ids": diagnostics.get("topology_edge_ids", []),
                "corridor_mask_hash": diagnostics.get("corridor_mask_hash", ""),
                "corridor_padding_m": diagnostics.get("corridor_padding_m", args.corridor_padding_m),
                "corridor_area_ratio": diagnostics.get("corridor_area_ratio", 0.0),
                "dstar_expanded_nodes": diagnostics.get("dstar_expanded_nodes", 0),
                "dstar_generated_nodes": diagnostics.get("dstar_generated_nodes", 0),
                "pipeline_wall_time_ms": wall_ms,
                "action_status": diagnostics.get("action_status", "not_available"),
                "planner_search_started": diagnostics.get("planner_search_started", "not_available"),
                "static_footprint_valid": diagnostics.get("static_footprint_valid", "not_available"),
                "kinematic_valid": diagnostics.get("kinematic_valid", "not_available"),
                "final_valid_success": final_valid,
                "failure_code": "" if final_valid else str(diagnostics.get("failure_code") or result.failure_code or "L3_PLANNER_FAILED"),
                "path_hash": path_hash,
                "path_file": path_file,
                "rrtstar_call_count": 0,
                "sst_call_count": 0,
            }
            rows.append(row)
            calls.append({
                "run_id": run_id,
                "architecture_id": ARCHITECTURE_ID,
                "stage": "L3",
                "called": bool(query_calls),
                "physical_backend_call_count": query_calls,
                "planner_backend": L3_BACKEND,
                "l2_called": False,
                "l2_call_count": 0,
                "failure_code": row["failure_code"],
                "planner_search_started": row["planner_search_started"],
                "corridor_mask_hash": row["corridor_mask_hash"],
                "dynamic_snapshot_id": snapshot.snapshot_id,
            })
            metrics.append({
                "run_id": run_id,
                "query_id": query.query_id,
                "path_hash": path_hash,
                "final_valid_success": final_valid,
                "static_footprint_valid": row["static_footprint_valid"],
                "kinematic_valid": row["kinematic_valid"],
                "path_length_m": diagnostics.get("path_length_m"),
                "minimum_clearance_m": diagnostics.get("minimum_clearance_m"),
                "maximum_curvature": diagnostics.get("maximum_curvature"),
            })
    finally:
        session.close()
    _write_csv(output / "runs.csv", rows)
    _write_csv(output / "backend_call_log.csv", calls)
    _write_csv(output / "path_metrics.csv", metrics)
    failures: Dict[str, int] = {}
    for row in rows:
        if row["failure_code"]:
            failures[row["failure_code"]] = failures.get(row["failure_code"], 0) + 1
    _write_csv(output / "failure_summary.csv", [{"failure_code": key, "count": value} for key, value in sorted(failures.items())])
    manifest = {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_version": PROTOCOL_VERSION,
        "map_id": hospital_map.map_id,
        "query_count": len(rows),
        "final_valid_count": sum(bool(row["final_valid_success"]) for row in rows),
        "topology_representation": "2a_v0_static_skeleton_graph",
        "topology_refinement_enabled": False,
        "topology_node_count": len(graph_view.nodes),
        "topology_edge_count": len(graph_view.edges),
        "topology_cache_hit": topology_info.get("topology_cache_hit", False),
        "topology_build_count": topology_info.get("topology_build_count", 0),
        "topology_load_time_ms": topology_info.get("topology_load_time_ms", 0.0),
        "l2_call_count": 0,
        "rrtstar_call_count": 0,
        "sst_call_count": 0,
        "l3_call_count_total": int(adapter.calls),
        "smac_session_start_count": session.session_start_count,
        "smac_session_close_count": session.session_close_count,
        "smac_session_restart_count": session.session_restart_count,
    }
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    (output / "protocol.yaml").write_text(yaml.safe_dump({
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_version": PROTOCOL_VERSION,
        "l1": "2A-V0 static topology + Graph D* Lite",
        "topology_refinement_enabled": False,
        "l2_called": False,
        "l3": "full topology corridor Smac Hybrid DUBIN",
        "dynamic_obstacles": False,
    }, sort_keys=False), encoding="utf-8")
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "pipeline_source_hash": _source_hash(),
        "graph_dstar_source_hash": hashlib.sha256(Path(__file__).with_name("graph_dstar_lite.py").read_bytes()).hexdigest(),
    }, sort_keys=False), encoding="utf-8")
    report = [
        "# 2D-V1 static topology + graph D* Lite smoke",
        "",
        f"- Architecture: `{ARCHITECTURE_ID}` revision `{IMPLEMENTATION_REVISION}`.",
        "- L1 uses the original 2A-V0 skeleton topology; no refined skeleton nodes are inserted.",
        f"- Topology states: {len(graph_view.nodes)} nodes / {len(graph_view.edges)} edges.",
        f"- Final-valid: {manifest['final_valid_count']}/{len(rows)}.",
        f"- Smac calls: {adapter.calls}; session start/close/restart={session.session_start_count}/{session.session_close_count}/{session.session_restart_count}.",
        "- L2 calls: 0; RRTstar/SST calls: 0/0.",
        f"- Failure counts: {json.dumps(failures, sort_keys=True)}.",
        "- This static smoke is not a dynamic D* Lite performance claim.",
    ]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    return 0


def _demo() -> Dict[str, Any]:
    nodes = [0, 1, 2, 3]
    edges = [GraphEdge("a", 0, 1, 1.0), GraphEdge("b", 1, 3, 1.0), GraphEdge("c", 0, 2, 1.2), GraphEdge("d", 2, 3, 1.2)]
    graph = GraphDStarLite(nodes, edges, 0, 3)
    initial = graph.compute_shortest_path()
    initial_path = graph.extract_path()
    graph.update_edges(["a", "b"], statuses={"a": GraphDStarLite.BLOCKED, "b": GraphDStarLite.BLOCKED}, costs={"a": INF, "b": INF})
    updated = graph.compute_shortest_path()
    return {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "topology_refinement_enabled": False,
        "initial_path": initial_path,
        "initial_expanded_nodes": initial.expanded_nodes,
        "updated_path": graph.extract_path(),
        "updated_expanded_nodes": updated.expanded_nodes,
        "l2_called": False,
        "rrtstar_call_count": 0,
        "sst_call_count": 0,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run 2D-V1 original-topology Graph D* Lite + Smac")
    parser.add_argument("--map-yaml", type=Path)
    parser.add_argument("--query-json", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v1_static_smoke_v1"))
    parser.add_argument("--topology-cache-dir", type=Path)
    parser.add_argument("--corridor-padding-m", type=float, default=DEFAULT_CORRIDOR_PADDING_M)
    parser.add_argument("--corridor-fallback-policy", choices=("bounded", "none"), default="bounded")
    parser.add_argument("--query-ids", nargs="*", default=None)
    parser.add_argument("--ros-domain-id", type=int, default=0)
    parser.add_argument("--demo", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.demo:
        print(json.dumps(_demo(), indent=2, sort_keys=True))
        return 0
    return _run_ros_smoke(args)


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["ARCHITECTURE_ID", "IMPLEMENTATION_REVISION", "Layered2DV1Pipeline", "build_parser", "build_static_topology_view", "main"]
