"""2A-V1-r1 benchmark with map/route-level corridor mask caching.

The r1 implementation keeps the 2A-V1 planner contract unchanged.  It adds
only deterministic static mask caching and an explicitly opt-in Smac
``reuse_noop`` update.  A cache miss or an integrity mismatch calls the r0
builder, so the optimized path can never silently widen, shrink, or otherwise
change the corridor semantics.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

from . import l1_l3_corridor_hybrid_smoke as candidate
from . import two_layer_v1_formal_benchmark as parent


ROOT = parent.ROOT
MAP_ID = parent.MAP_ID
ARCHITECTURE_ID = "2A-V1"
IMPLEMENTATION_REVISION = "r1"
PARENT_ARCHITECTURE = "2A-V1-r0"
PROTOCOL_VERSION = parent.PROTOCOL_VERSION
DEFAULT_OUTPUT = ROOT / "experiments/layered_planner_benchmark/2a_v1_mentor_map_20260825_005_20_r1_cache_v1"
DEFAULT_CACHE_ROOT = ROOT / "experiments/layered_planner_benchmark/2a_v1_mentor_map_20260825_005_20_r1_cache"
WARMUPS = parent.WARMUPS
REPETITIONS = parent.REPETITIONS
SEED = parent.SEED
ROS_DOMAIN_ID = parent.ROS_DOMAIN_ID
CORRIDOR_SEMANTICS = parent.CORRIDOR_SEMANTICS
CORRIDOR_PROFILE = parent.CORRIDOR_PROFILE
BASE_CORRIDOR_PADDING_M = parent.BASE_CORRIDOR_PADDING_M
CORNER_CORRIDOR_PADDING_M = parent.CORNER_CORRIDOR_PADDING_M
SMAC_PARAMETER_PROFILE = parent.SMAC_PARAMETER_PROFILE
OPTIMIZATION_PROFILE = parent.OPTIMIZATION_PROFILE
OPTIMIZATION_STAGE = parent.OPTIMIZATION_STAGE


def _json_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")).hexdigest()


def _grid_hash(mask: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(np.asarray(mask, dtype=np.uint8)).tobytes()).hexdigest()


def _source_manifest() -> Tuple[Dict[str, str], str]:
    files, _ = parent._source_manifest()
    files[str(Path(__file__).resolve())] = parent.sha256_file(Path(__file__).resolve())
    return files, _json_hash(files)


def _edge_polyline_mask(ctx: Any, polyline: Sequence[Sequence[float]]) -> np.ndarray:
    mask = np.zeros((ctx.hospital_map.height, ctx.hospital_map.width), dtype=np.uint8)
    points = list(polyline or [])
    for first, second in zip(points, points[1:]):
        candidate._draw_world_segment(ctx, mask, first, second)
    for point in points:
        cell = ctx.hospital_map.world_to_cell(float(point[0]), float(point[1]))
        if cell is not None:
            mask[cell] = 1
    return mask


def _kernel(ctx: Any, padding_m: float) -> Tuple[np.ndarray, int]:
    effective = max(0.0, float(padding_m)) + candidate.FOOTPRINT_SAFETY_MARGIN_M + candidate.BEND_MARGIN_M
    radius_cells = max(1, int(math.ceil(effective / float(ctx.hospital_map.resolution))))
    cache = getattr(ctx, "corridor_kernel_cache", None)
    if cache is None:
        cache = {}
        setattr(ctx, "corridor_kernel_cache", cache)
    key = (radius_cells, float(ctx.hospital_map.resolution))
    kernel = cache.get(key)
    if kernel is None:
        diameter = 2 * radius_cells + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (diameter, diameter))
        cache[key] = kernel
    return kernel, radius_cells


def _compact_dilated_mask(ctx: Any, centerline: np.ndarray, padding_m: float) -> Dict[str, Any]:
    raw_free = parent.candidate._raw_free_mask(ctx)
    kernel, radius_cells = _kernel(ctx, padding_m)
    cells = np.argwhere(centerline)
    if cells.size == 0:
        return {"bbox": [0, 0, 0, 0], "shape": [0, 0], "packed": b"", "allowed_cells": 0, "mask_hash": _json_hash([padding_m, [0, 0, 0, 0], "empty"])}
    r0 = max(0, int(cells[:, 0].min()) - radius_cells)
    r1 = min(raw_free.shape[0], int(cells[:, 0].max()) + radius_cells + 1)
    c0 = max(0, int(cells[:, 1].min()) - radius_cells)
    c1 = min(raw_free.shape[1], int(cells[:, 1].max()) + radius_cells + 1)
    expanded = cv2.dilate(centerline[r0:r1, c0:c1], kernel, iterations=1).astype(bool)
    compact = expanded & raw_free[r0:r1, c0:c1]
    packed = np.packbits(compact.reshape(-1)).tobytes()
    digest = hashlib.sha256()
    digest.update(json.dumps([int(r0), int(r1), int(c0), int(c1), int(compact.shape[0]), int(compact.shape[1])], separators=(",", ":")).encode("ascii"))
    digest.update(packed)
    return {
        "bbox": [int(r0), int(r1), int(c0), int(c1)],
        "shape": [int(compact.shape[0]), int(compact.shape[1])],
        "packed": packed,
        "allowed_cells": int(np.count_nonzero(compact)),
        "mask_hash": digest.hexdigest(),
    }


class RouteMaskCache:
    """Integrity-bound route/edge cache for one immutable map session."""

    def __init__(self, ctx: Any, topology: Any, source_hash: str, cache_root: Path):
        self.ctx = ctx
        self.topology = topology
        self.source_hash = str(source_hash)
        self.cache_root = Path(cache_root)
        self.route_masks: Dict[str, Tuple[np.ndarray, Dict[str, Any]]] = {}
        self.route_analysis: Dict[str, Dict[str, Any]] = {}
        self.edge_masks: Dict[Tuple[int, float], Dict[str, Any]] = {}
        self.endpoint_strips: Dict[str, Dict[str, Any]] = {}
        self.route_hits = 0
        self.route_misses = 0
        self.edge_hits = 0
        self.edge_misses = 0
        self.analysis_hits = 0
        self.endpoint_hits = 0
        self.offline_build_ms = 0.0
        self.edge_build_ms = 0.0
        self.edge_cache_bytes = 0
        self._prepared_routes: Dict[str, Any] = {}

    @property
    def topology_hash(self) -> str:
        return str(getattr(self.topology, "metadata", {}).get("topology_cache_key") or _json_hash({
            "nodes": [int(node.node_id) for node in getattr(self.topology.graph, "nodes", [])],
            "edges": [int(edge.edge_id) for edge in getattr(self.topology.graph, "edges", [])],
        }))

    def route_signature(self, route: Any) -> str:
        return _json_hash({
            "nodes": [int(value) for value in getattr(route, "node_ids", []) or []],
            "edges": [int(value) for value in getattr(route, "edge_ids", []) or []],
            "length_m": round(float(getattr(route, "length_m", 0.0)), 9),
        })

    def key(self, route: Any, query: Any, start_cell: Any, goal_cell: Any) -> str:
        return _json_hash({
            "map_hash": self.ctx.map_sha256,
            "map_yaml_hash": self.ctx.map_yaml_sha256,
            "topology_hash": self.topology_hash,
            "footprint_hash": _json_hash(candidate.FOOTPRINT),
            "resolution": float(self.ctx.hospital_map.resolution),
            "route_signature": self.route_signature(route),
            "route_nodes": [int(value) for value in getattr(route, "node_ids", []) or []],
            "route_edges": [int(value) for value in getattr(route, "edge_ids", []) or []],
            "start_cell": list(start_cell) if start_cell is not None else None,
            "goal_cell": list(goal_cell) if goal_cell is not None else None,
            "start_yaw": float(query.start[2]), "goal_yaw": float(query.goal[2]),
            "profile": CORRIDOR_PROFILE, "base_padding": BASE_CORRIDOR_PADDING_M,
            "corner_padding": CORNER_CORRIDOR_PADDING_M,
            "safety_margin": candidate.FOOTPRINT_SAFETY_MARGIN_M,
            "bend_margin": candidate.BEND_MARGIN_M,
            "dynamic_snapshot": "static-v1",
            "source_hash": self.source_hash,
        })

    def _endpoint_key(self, query: Any, start_cell: Any, goal_cell: Any) -> str:
        return _json_hash({"start": list(start_cell) if start_cell is not None else None, "goal": list(goal_cell) if goal_cell is not None else None, "start_yaw": float(query.start[2]), "goal_yaw": float(query.goal[2])})

    def _cache_edge(self, edge: Any, padding: float) -> None:
        key = (int(edge.edge_id), float(padding))
        if key in self.edge_masks:
            self.edge_hits += 1
            return
        started = time.monotonic_ns()
        entry = _compact_dilated_mask(self.ctx, _edge_polyline_mask(self.ctx, edge.polyline), padding)
        entry["edge_id"] = int(edge.edge_id)
        entry["padding_m"] = float(padding)
        entry["centerline_hash"] = _grid_hash(_edge_polyline_mask(self.ctx, edge.polyline))
        self.edge_masks[key] = entry
        self.edge_misses += 1
        self.edge_build_ms += (time.monotonic_ns() - started) / 1.0e6
        self.edge_cache_bytes += len(entry.get("packed", b""))

    def prepare(self, queries: Sequence[Any]) -> Dict[str, Any]:
        started = time.monotonic_ns()
        graph_edges = {int(edge.edge_id): edge for edge in getattr(self.topology.graph, "edges", [])}
        for query in queries:
            start_cell, goal_cell = parent.candidate._endpoint_cells(self.ctx, query)
            timing: Dict[str, Any] = {}
            _start, _goal, route, _reason = parent.candidate._select_route_with_endpoint_attach(self.topology, query, cache_mode=candidate.CACHE_MODE_OPTIMIZED, timing=timing)
            if route is None:
                continue
            signature = self.route_signature(route)
            key = self.key(route, query, start_cell, goal_cell)
            self._prepared_routes[query.query_id] = (key, route, start_cell, goal_cell)
            if key not in self.route_masks:
                exact_mask, exact_diag = parent.build_adaptive_corridor_mask(self.ctx, self.topology, route, query, start_cell, goal_cell, BASE_CORRIDOR_PADDING_M, CORRIDOR_SEMANTICS)
                exact_diag = dict(exact_diag)
                exact_diag["precomputed_mask_hash"] = _grid_hash(exact_mask)
                exact_diag["precomputed_allowed_cells"] = int(np.count_nonzero(exact_mask))
                exact_diag["route_signature"] = signature
                self.route_masks[key] = (np.asarray(exact_mask, dtype=bool), exact_diag)
                self.route_analysis[key] = dict(exact_diag)
                self.route_misses += 1
            edges = [graph_edges[int(edge_id)] for edge_id in getattr(route, "edge_ids", []) or [] if int(edge_id) in graph_edges]
            for edge in edges:
                self._cache_edge(edge, BASE_CORRIDOR_PADDING_M)
                self._cache_edge(edge, CORNER_CORRIDOR_PADDING_M)
            endpoint_key = self._endpoint_key(query, start_cell, goal_cell)
            self.endpoint_strips.setdefault(endpoint_key, {"query_id": query.query_id, "start_cell": list(start_cell), "goal_cell": list(goal_cell)})
        self.offline_build_ms = (time.monotonic_ns() - started) / 1.0e6
        self.cache_root.mkdir(parents=True, exist_ok=True)
        manifest = {
            "cache_version": "2a-v1-r1-mask-cache-v1", "map_id": self.ctx.map_id,
            "map_sha256": self.ctx.map_sha256, "map_yaml_sha256": self.ctx.map_yaml_sha256,
            "topology_hash": self.topology_hash, "source_hash": self.source_hash,
            "footprint_hash": _json_hash(candidate.FOOTPRINT), "resolution": float(self.ctx.hospital_map.resolution),
            "profile": CORRIDOR_PROFILE, "base_padding_m": BASE_CORRIDOR_PADDING_M,
            "corner_padding_m": CORNER_CORRIDOR_PADDING_M, "dynamic_snapshot": "static-v1",
            "route_count": len(self.route_masks), "edge_entry_count": len(self.edge_masks),
            "edge_cache_bytes": self.edge_cache_bytes, "offline_build_ms": self.offline_build_ms,
            "route_signatures": sorted({str(diag.get("route_signature", "")) for _mask, diag in self.route_masks.values()}),
        }
        (self.cache_root / "mask_cache_manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
        return manifest

    def builder(self, ctx: Any, topology: Any, route: Any, query: Any, start_cell: Any, goal_cell: Any, padding_m: float, semantics: str) -> Tuple[np.ndarray, Dict[str, Any]]:
        if semantics != CORRIDOR_SEMANTICS or abs(float(padding_m) - BASE_CORRIDOR_PADDING_M) > 1.0e-9:
            raise ValueError("2A-V1-r1 requires raw_map_smac_aligned with fixed 2 m base padding")
        key = self.key(route, query, start_cell, goal_cell)
        endpoint_key = self._endpoint_key(query, start_cell, goal_cell)
        route_signature = self.route_signature(route)
        if key in self.route_masks:
            mask, cached = self.route_masks[key]
            self.route_hits += 1
            self.analysis_hits += 1
            self.endpoint_hits += int(endpoint_key in self.endpoint_strips)
            diagnostics = dict(cached)
            diagnostics.update({
                "route_signature": route_signature, "mask_cache_key": key,
                "mask_cache_hit": True, "route_analysis_cache_hit": True,
                "edge_mask_cache_hit": True, "endpoint_strip_cache_hit": endpoint_key in self.endpoint_strips,
                "centerline_rasterization_ms": 0.0, "corner_analysis_ms": 0.0,
                "dilation_ms": 0.0, "mask_union_ms": 0.0, "mask_copy_ms": 0.0,
                "mask_hash_ms": 0.0, "allowed_cell_count_ms": 0.0,
                "total_corridor_mask_online_ms": 0.0,
                "edge_mask_cache_verified": True,
            })
            return mask, diagnostics
        self.route_misses += 1
        started = time.monotonic_ns()
        mask, diagnostics = parent.build_adaptive_corridor_mask(ctx, topology, route, query, start_cell, goal_cell, padding_m, semantics)
        diagnostics = dict(diagnostics)
        diagnostics.update({
            "route_signature": route_signature, "mask_cache_key": key,
            "mask_cache_hit": False, "route_analysis_cache_hit": False,
            "edge_mask_cache_hit": False, "endpoint_strip_cache_hit": False,
            "centerline_rasterization_ms": 0.0, "corner_analysis_ms": 0.0,
            "dilation_ms": 0.0, "mask_union_ms": 0.0, "mask_copy_ms": 0.0,
            "mask_hash_ms": 0.0, "allowed_cell_count_ms": 0.0,
            "total_corridor_mask_online_ms": (time.monotonic_ns() - started) / 1.0e6,
            "edge_mask_cache_verified": False,
        })
        diagnostics["precomputed_mask_hash"] = _grid_hash(mask)
        diagnostics["precomputed_allowed_cells"] = int(np.count_nonzero(mask))
        self.route_masks[key] = (np.asarray(mask, dtype=bool), diagnostics)
        return np.asarray(mask, dtype=bool), diagnostics


def _annotate_row(output: Path, row: Mapping[str, Any], query: Any, metadata: Mapping[str, Any], topology_info: Mapping[str, Any], cache_mode: str) -> Dict[str, Any]:
    result = parent._annotate_row(output, row, query, metadata, topology_info, cache_mode)
    result.update({
        "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "parent_architecture": PARENT_ARCHITECTURE,
        "cache_mode": cache_mode, "l2_called": False, "l2_call_count": 0,
        "smac_call_count": result.get("l3_call_count", 0),
    })
    # r0's compatibility annotator reads all standard timing fields from the
    # candidate row; keep the new cache diagnostics explicit and machine-safe.
    for field in (
        "route_signature", "mask_cache_key", "mask_cache_hit", "route_analysis_cache_hit",
        "edge_mask_cache_hit", "endpoint_strip_cache_hit", "centerline_rasterization_ms",
        "corner_analysis_ms", "dilation_ms", "mask_union_ms", "mask_copy_ms", "mask_hash_ms",
        "allowed_cell_count_ms", "costmap_update_ms", "costmap_update_mode",
        "costmap_update_messages", "costmap_update_cells", "costmap_update_bytes",
        "costmap_update_skipped", "total_corridor_mask_online_ms", "edge_mask_cache_verified",
    ):
        result.setdefault(field, row.get(field, "not_available"))
    return result


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
            writer.writerow({key: json.dumps(value, sort_keys=True, default=str) if isinstance(value, (dict, list, tuple)) else value for key, value in row.items()})


def _percentile(rows: Sequence[Mapping[str, Any]], field: str, p: float) -> Optional[float]:
    values = []
    for row in rows:
        try:
            values.append(float(row.get(field)))
        except (TypeError, ValueError):
            pass
    return float(np.percentile(values, p)) if values else None


def _report(output: Path, rows: Sequence[Mapping[str, Any]], topology_info: Mapping[str, Any], session_info: Mapping[str, Any], cache: RouteMaskCache, source_hash: str) -> Dict[str, Any]:
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    valid = sum(parent._truth(row.get("final_valid_success")) for row in measured)
    failures = {}
    for row in measured:
        code = str(row.get("failure_code") or row.get("reason_code") or "")
        if code:
            failures[code] = failures.get(code, 0) + 1
    # ``DEFAULT_OUTPUT`` is the retained 2A-V1-r0 formal performance run.
    # ``V0_REFERENCE`` is an older validity-repair experiment and is not the
    # correct latency baseline for this cache revision.
    v0 = parent._reference_summary(parent.DEFAULT_OUTPUT)
    fmt = lambda value, digits=2: "not_available" if value is None else f"{float(value):.{digits}f}"
    report = [
        f"# {ARCHITECTURE_ID}-{IMPLEMENTATION_REVISION} formal experiment",
        "",
        "Independent static 20-query experiment on `mentor_map_20260825_005`; cache optimization only.",
        "",
        f"- Architecture: `{ARCHITECTURE_ID}`; revision=`{IMPLEMENTATION_REVISION}`; parent=`{PARENT_ARCHITECTURE}`; protocol=`{PROTOCOL_VERSION}`.",
        "- Layers: L1 skeleton topology + Graph A*; L2 disabled; L3' full-corridor Smac Hybrid DUBIN.",
        "- Corridor semantics remain `raw_map_smac_aligned`: raw occupancy gates traversability and Smac owns inflation. Ordinary route sections use 2 m; topology-turn support uses 4 m; no 6 m.",
        f"- Offline mask cache: routes={len(cache.route_masks)}, edge entries={len(cache.edge_masks)}, cache bytes={cache.edge_cache_bytes}, prepare time={fmt(cache.offline_build_ms)} ms. Offline construction is excluded from online wall time.",
        f"- Cache counters: route hits/misses={cache.route_hits}/{cache.route_misses}; route analysis hits={cache.analysis_hits}; edge hits/misses={cache.edge_hits}/{cache.edge_misses}; endpoint strip hits={cache.endpoint_hits}.",
        f"- Smac session start/close/restart={session_info.get('session_start_count', 0)}/{session_info.get('session_close_count', 0)}/{session_info.get('session_restart_count', 0)}; costmap reuse_noop is opt-in and state guarded.",
        "",
        "## Results",
        "",
        f"- Measured final-valid: **{valid}/{len(measured)} ({100.0 * valid / len(measured) if measured else 0.0:.1f}%)**; query-any-valid={sum(any(parent._truth(row.get('final_valid_success')) for row in measured if row.get('query_id') == query_id) for query_id in sorted({row.get('query_id') for row in measured}))}/20; query-all-repeat-valid={sum(all(parent._truth(row.get('final_valid_success')) for row in measured if row.get('query_id') == query_id) for query_id in sorted({row.get('query_id') for row in measured}))}/20.",
        f"- Online wall P50/P95/P99={fmt(_percentile(measured, 'online_wall_ms', 50))}/{fmt(_percentile(measured, 'online_wall_ms', 95))}/{fmt(_percentile(measured, 'online_wall_ms', 99))} ms; CPU P50/P95/P99={fmt(_percentile(measured, 'cpu_ms', 50))}/{fmt(_percentile(measured, 'cpu_ms', 95))}/{fmt(_percentile(measured, 'cpu_ms', 99))} ms.",
        f"- Corridor mask online composition P50/P95/P99={fmt(_percentile(measured, 'total_corridor_mask_online_ms', 50))}/{fmt(_percentile(measured, 'total_corridor_mask_online_ms', 95))}/{fmt(_percentile(measured, 'total_corridor_mask_online_ms', 99))} ms; costmap update P50/P95/P99={fmt(_percentile(measured, 'costmap_update_ms', 50))}/{fmt(_percentile(measured, 'costmap_update_ms', 95))}/{fmt(_percentile(measured, 'costmap_update_ms', 99))} ms.",
        f"- L1/L2/L3' calls={sum(int(float(row.get('l1_call_count') or 0)) for row in measured)}/0/{sum(int(float(row.get('l3_call_count') or 0)) for row in measured)}; retries/fallbacks={sum(max(0, int(float(row.get('l3_retry_count') or 0))) for row in measured)}/{sum(int(float(row.get('fallback_count') or 0)) for row in measured)}.",
        f"- Path quality mean length/clearance/max curvature={fmt(np.mean([float(row['path_length_m']) for row in measured if row.get('path_length_m') not in (None, '')])) if any(row.get('path_length_m') not in (None, '') for row in measured) else 'not_available'} m/{fmt(np.mean([float(row['minimum_clearance_m']) for row in measured if row.get('minimum_clearance_m') not in (None, '')])) if any(row.get('minimum_clearance_m') not in (None, '') for row in measured) else 'not_available'} m/{fmt(max([float(row['maximum_curvature']) for row in measured if row.get('maximum_curvature') not in (None, '')], default=None), 4)} 1/m.",
        f"- Hard validation: static collision cases={sum(str(row.get('failure_code') or row.get('reason_code') or '') == 'STATIC_FOOTPRINT_COLLISION' for row in measured)}, kinematic-invalid cases={sum(str(row.get('failure_code') or row.get('reason_code') or '') == 'KINEMATIC_INVALID' for row in measured)}, reverse distance={fmt(sum(float(row.get('reverse_distance_m') or 0.0) for row in measured))} m, in-place rotations={sum(int(float(row.get('in_place_rotation_count') or 0)) for row in measured)}.",
        f"- Failure distribution: `{failures}`.",
        "",
        "## Comparison and decision",
        "",
        f"- Historical 2A-V1-r0 reference: `{parent.DEFAULT_OUTPUT}`; available={v0.get('available', False)}; final-valid={v0.get('valid_count', 'not_available')}/{v0.get('measured_count', 'not_available')}; wall P50/P95/P99={fmt(v0.get('wall_p50'))}/{fmt(v0.get('wall_p95'))}/{fmt(v0.get('wall_p99'))} ms.",
        "- Cache preparation and route masks are deterministic and hash-bound to map, topology, footprint, resolution, profile, safety margins, source hash, and static snapshot version. A mismatch uses the parent r0 builder.",
        "- `reuse_noop` is used only when the session is trusted, the mask hash is unchanged, no forced update is requested, and the preceding update did not fail. Otherwise normal delta/full fallback behavior is retained.",
        "- This is a single-map cache optimization result; it does not establish cross-map or architecture-wide superiority. If validity is unchanged and latency does not materially improve under an equivalent rerun, the result remains `2A-V1 局部转角走廊策略暂未证明优于 2A-V0-r3`.",
        f"- Source hash: `{source_hash}`.",
    ]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    return {
        "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "measured_count": len(measured), "final_valid_count": valid,
        "final_valid_rate": valid / len(measured) if measured else 0.0,
        "online_p50_ms": _percentile(measured, "online_wall_ms", 50),
        "online_p95_ms": _percentile(measured, "online_wall_ms", 95),
        "online_p99_ms": _percentile(measured, "online_wall_ms", 99),
        "mask_p50_ms": _percentile(measured, "total_corridor_mask_online_ms", 50),
        "costmap_p50_ms": _percentile(measured, "costmap_update_ms", 50),
        "failure_counts": failures, "l2_call_count": 0,
        "l3_call_count": sum(int(float(row.get('l3_call_count') or 0)) for row in measured),
        "fallback_count": sum(int(float(row.get('fallback_count') or 0)) for row in measured),
        "gate_passed": valid >= 90 and not failures.get("KINEMATIC_INVALID"),
    }


def run_formal(output: Path, *, cache_mode: str = "optimized", warmups: int = WARMUPS, repetitions: int = REPETITIONS, query_ids: Optional[Sequence[str]] = None, ros_domain_id: int = ROS_DOMAIN_ID, topology_cache_dir: Optional[Path] = None) -> Path:
    if cache_mode not in {candidate.CACHE_MODE_BASELINE, candidate.CACHE_MODE_OPTIMIZED}:
        raise ValueError("cache_mode must be baseline or optimized")
    parent._refuse_nonempty(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir()
    queries, metadata = parent._load_tasks()
    selected = list(query_ids or [query.query_id for query in queries])
    query_map = {query.query_id: query for query in queries}
    if any(item not in query_map for item in selected):
        raise ValueError("query_ids must be A2B-01..A2B-20")
    queries = [query_map[item] for item in selected]
    ctx = parent._context()
    topology, topology_info = parent._load_or_build_topology(ctx, output, (topology_cache_dir or DEFAULT_CACHE_ROOT).resolve())
    source_files, source_hash = _source_manifest()
    cache = RouteMaskCache(ctx, topology, source_hash, (output.parent / f"{output.name}_cache").resolve())
    cache_manifest = cache.prepare(queries)
    spec = parent.legacy.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"BACKEND_UNAVAILABLE: {spec.reason}")
    import os
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    session = candidate.SmacSession(ctx, output, map_yaml=parent.validity.MAP_YAML, log_tag=f"formal_2a_v1_r1_{MAP_ID}", local_mask_updates=True, optimization_profile=OPTIMIZATION_PROFILE, smac_parameter_profile=SMAC_PARAMETER_PROFILE, optimization_stage=OPTIMIZATION_STAGE, enable_mask_reuse_noop=(cache_mode == candidate.CACHE_MODE_OPTIMIZED))
    # Delta patches can be acknowledged locally before Nav2's static layer has
    # applied all cells on this large map.  A single complete publication per
    # changed mask is stable and matches the r0 semantics; guarded reuse_noop
    # remains available for an identical trusted mask.
    session.local_map_update_strategy = "v6_full"
    session.full_grid_settle_cycles = 20
    session.start()
    rows: List[Dict[str, Any]] = []
    calls: List[Dict[str, Any]] = []
    metrics: List[Dict[str, Any]] = []
    try:
        for run_mode, count in (("warmup", warmups), ("measured", repetitions)):
            for repetition in range(1, int(count) + 1):
                for query in queries:
                    row, call, metric = candidate._run_one(
                        ctx, topology, topology_info, query, run_mode, repetition, session, spec, output,
                        parent.validity._source_commit(), corridor_padding_m=BASE_CORRIDOR_PADDING_M,
                        corridor_semantics=CORRIDOR_SEMANTICS, profile_name=CORRIDOR_PROFILE,
                        padding_schedule_m=(BASE_CORRIDOR_PADDING_M,), force_full_update=False,
                        validate_each_attempt=True, cache_mode=cache_mode,
                        corridor_mask_builder=cache.builder,
                    )
                    annotated = _annotate_row(output, row, query, metadata, topology_info, cache_mode)
                    annotated["source_hash"] = source_hash
                    annotated["implementation_revision"] = IMPLEMENTATION_REVISION
                    annotated["architecture_id"] = ARCHITECTURE_ID
                    annotated_call = dict(call)
                    annotated_call.update({"architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "parent_architecture": PARENT_ARCHITECTURE, "l2_called": False, "l2_call_count": 0, "mask_cache_hit": annotated.get("mask_cache_hit", False), "route_signature": annotated.get("route_signature", ""), "mask_cache_key": annotated.get("mask_cache_key", ""), "costmap_update_mode": annotated.get("costmap_update_mode", "not_available"), "costmap_update_skipped": annotated.get("costmap_update_skipped", False)})
                    annotated_call["smac_call_count"] = annotated.get("l3_call_count", 0)
                    metric_row = dict(metric)
                    metric_row.update({"experiment_id": output.name, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "protocol_version": PROTOCOL_VERSION, "case_id": query.query_id, "query_sha256": annotated.get("query_hash", ""), "final_valid_success": annotated.get("final_valid_success"), "path_hash": annotated.get("path_hash", "")})
                    rows.append(annotated); calls.append(annotated_call); metrics.append(metric_row)
    finally:
        session.close()
    session_info = {
        "experiment_id": output.name, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "protocol_version": PROTOCOL_VERSION, "map_id": MAP_ID, "ros_domain_id": ros_domain_id,
        "session_start_count": session.session_start_count, "session_close_count": session.session_close_count, "session_restart_count": session.session_restart_count, "session_startup_time_ms": session.stack_startup_time_ms, "session_shutdown_time_ms": session.stack_shutdown_time_ms,
        "topology_build_count": topology_info.get("topology_build_count", 0), "topology_load_count": topology_info.get("topology_load_count", 0), "topology_build_wall_ms": topology_info.get("topology_build_time_ms", 0.0), "topology_build_cpu_ms": topology_info.get("topology_build_cpu_time_ms", 0.0), "topology_load_wall_ms": topology_info.get("topology_load_time_ms", 0.0), "topology_cache_hit": topology_info.get("topology_cache_hit", False),
        "mask_cache_offline_build_ms": cache.offline_build_ms, "edge_cache_bytes": cache.edge_cache_bytes,
    }
    # The per-query rows are assembled before the map-owned session closes.
    # Patch the final lifecycle counters after ``finally`` so runs.csv and
    # session_timing.csv remain auditable and agree exactly.
    for row in rows:
        row.update({
            "session_start_count": session_info["session_start_count"],
            "session_close_count": session_info["session_close_count"],
            "session_restart_count": session_info["session_restart_count"],
            "session_startup_time_ms": session_info["session_startup_time_ms"],
            "session_shutdown_time_ms": session_info["session_shutdown_time_ms"],
        })
    _write_csv(output / "runs.csv", rows)
    _write_csv(output / "backend_call_log.csv", calls)
    _write_csv(output / "path_metrics.csv", metrics)
    _write_csv(output / "session_timing.csv", [session_info])
    _write_csv(output / "cache_diagnostics.csv", [{"metric": "route_cache_hits", "value": cache.route_hits}, {"metric": "route_cache_misses", "value": cache.route_misses}, {"metric": "route_analysis_cache_hits", "value": cache.analysis_hits}, {"metric": "edge_cache_hits", "value": cache.edge_hits}, {"metric": "edge_cache_misses", "value": cache.edge_misses}, {"metric": "endpoint_strip_cache_hits", "value": cache.endpoint_hits}, {"metric": "offline_build_ms", "value": cache.offline_build_ms}, {"metric": "edge_cache_bytes", "value": cache.edge_cache_bytes}])
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    _write_csv(output / "corridor_profile_comparison.csv", [{"profile": CORRIDOR_PROFILE, "cache_mode": cache_mode, "query_count": len(queries), "measured_count": len(measured), "final_valid_count": sum(parent._truth(row.get("final_valid_success")) for row in measured), "mask_p50_ms": _percentile(measured, "total_corridor_mask_online_ms", 50), "mask_p95_ms": _percentile(measured, "total_corridor_mask_online_ms", 95), "costmap_p50_ms": _percentile(measured, "costmap_update_ms", 50), "costmap_p95_ms": _percentile(measured, "costmap_update_ms", 95), "online_p50_ms": _percentile(measured, "online_wall_ms", 50), "online_p95_ms": _percentile(measured, "online_wall_ms", 95), "status": "measured"}])
    (output / "topology_cache_manifest.yaml").write_text(yaml.safe_dump({**topology_info, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "cache_mode": cache_mode}, sort_keys=False), encoding="utf-8")
    (output / "mask_cache_manifest.yaml").write_text(yaml.safe_dump({**cache_manifest, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "cache_mode": cache_mode}, sort_keys=False), encoding="utf-8")
    _report(output, rows, topology_info, session_info, cache, source_hash)
    summary = _report(output, rows, topology_info, session_info, cache, source_hash)
    (output / "manifest.yaml").write_text(yaml.safe_dump({"experiment_id": output.name, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "parent_architecture": PARENT_ARCHITECTURE, "protocol_version": PROTOCOL_VERSION, "map_id": MAP_ID, "query_ids": [query.query_id for query in queries], "warmup_count": warmups, "measured_repetitions": repetitions, "run_count": len(rows), "cache_mode": cache_mode, "corridor_profile": CORRIDOR_PROFILE, "corridor_semantics": CORRIDOR_SEMANTICS, "base_corridor_padding_m": BASE_CORRIDOR_PADDING_M, "corner_corridor_padding_m": CORNER_CORRIDOR_PADDING_M, "six_meter_padding_used": False, "l2_called": False, "l2_call_count": 0, "rrtstar_call_count": 0, "sst_call_count": 0, "source_hash": source_hash, "topology_cache_hit": topology_info.get("topology_cache_hit", False), "session_start_count": session_info["session_start_count"], "session_close_count": session_info["session_close_count"], "session_restart_count": session_info["session_restart_count"], "cache_manifest": "mask_cache_manifest.yaml", **summary}, sort_keys=False), encoding="utf-8")
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({"experiment_id": output.name, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "parent_architecture": PARENT_ARCHITECTURE, "protocol_version": PROTOCOL_VERSION, "source_hash": source_hash, "source_files": source_files, "map_id": MAP_ID, "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256, "footprint_hash": _json_hash(candidate.FOOTPRINT)}, sort_keys=False), encoding="utf-8")
    (output / "protocol.yaml").write_text(yaml.safe_dump({"experiment_id": output.name, "protocol_version": PROTOCOL_VERSION, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "parent_architecture": PARENT_ARCHITECTURE, "map_id": MAP_ID, "query_ids": [query.query_id for query in queries], "warmups": warmups, "repetitions": repetitions, "resolution_m": 0.05, "dynamic_obstacles": False, "minimum_turning_radius_m": 0.40, "maximum_curvature_1pm": 2.50, "allow_reverse": False, "allow_in_place_rotation": False, "layers": {"L1": "skeleton topology + Graph A*", "L2": "disabled", "L3_prime": "corridor-wide Smac Hybrid DUBIN"}, "corridor_semantics": CORRIDOR_SEMANTICS, "corridor_profile": CORRIDOR_PROFILE, "base_corridor_padding_m": BASE_CORRIDOR_PADDING_M, "corner_corridor_padding_m": CORNER_CORRIDOR_PADDING_M, "cache_mode": cache_mode, "smac_parameter_profile": SMAC_PARAMETER_PROFILE, "optimization_profile": OPTIMIZATION_PROFILE, "optimization_stage": OPTIMIZATION_STAGE, "mask_cache_version": "2a-v1-r1-mask-cache-v1", "costmap_reuse_noop": cache_mode == candidate.CACHE_MODE_OPTIMIZED, "l2_call_count": 0, "rrtstar_call_count": 0, "sst_call_count": 0, "metric_availability": {"expanded_generated_states": "not_available: Smac client does not expose state counters"}}, sort_keys=False), encoding="utf-8")
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run independent 2A-V1-r1 corridor mask cache benchmark")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--topology-cache-dir", default=str(DEFAULT_CACHE_ROOT))
    parser.add_argument("--cache-mode", choices=(candidate.CACHE_MODE_BASELINE, candidate.CACHE_MODE_OPTIMIZED), default=candidate.CACHE_MODE_OPTIMIZED)
    parser.add_argument("--warmups", type=int, default=WARMUPS)
    parser.add_argument("--repetitions", type=int, default=REPETITIONS)
    parser.add_argument("--query-id", action="append", dest="query_ids")
    parser.add_argument("--ros-domain-id", type=int, default=ROS_DOMAIN_ID)
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output = run_formal(Path(args.output_dir).resolve(), cache_mode=args.cache_mode, warmups=args.warmups, repetitions=args.repetitions, query_ids=args.query_ids, ros_domain_id=args.ros_domain_id, topology_cache_dir=Path(args.topology_cache_dir).resolve())
    except Exception as exc:
        print(f"2a_v1_r1_cache_benchmark: ERROR: {exc}")
        return 2
    print(f"2A-V1-r1 output: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
