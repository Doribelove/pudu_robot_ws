"""Independent formal 2A-V0 benchmark on the mentor 4x map.

The planner itself remains the existing L1 + corridor-wide Smac Hybrid
implementation.  This entry point owns only the frozen-task validation,
map-level cache/session lifecycle, measurement schema, and report generation.
It never calls the Grid A* stage or a fallback backend.
"""

from __future__ import annotations

import argparse
import collections
import csv
import datetime as dt
import hashlib
import json
import math
import os
import resource
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import l1_l3_corridor_hybrid_smoke as candidate
from . import l1_l3_corridor_hybrid_validity as validity
from . import layered_architecture_paired_benchmark as paired
from . import unified_four_backends_smoke as legacy
from .planner_benchmark.map_utils import sha256_file


ROOT = Path("/home/robot/pudu_robot_ws")
MAP_ID = paired.FOUR_X_MAP_ID
ARCHITECTURE_ID = "2A-V0"
IMPLEMENTATION_REVISION = "r3"
PROTOCOL_VERSION = "PLN-02-EXP-V1"
EXPERIMENT_KIND = "static_formal"
QUERY_SET_ID = "arena_a2b_benchmark_20"
DEFAULT_OUTPUT = ROOT / "experiments/layered_planner_benchmark/2a_v0_mentor_map_20260825_005_4x_area_20_r3_v1"
DEFAULT_CACHE_ROOT = DEFAULT_OUTPUT / "topology_cache"
WARMUPS = 3
REPETITIONS = 5
TIMEOUT_S = 5.0
SEED = 0
SMAC_PARAMETER_PROFILE = "lighter_smoother"
OPTIMIZATION_PROFILE = "v7_candidate"
OPTIMIZATION_STAGE = "step3_delta_map"
CORRIDOR_PROFILE = "bounded_corridor_expansion_full_update"
CORRIDOR_SEMANTICS = "raw_map_smac_aligned"
PADDING_SCHEDULE = (2.0, 4.0, 6.0)


def _json_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _numeric(value: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _truth(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _percentile(rows: Sequence[Mapping[str, Any]], field: str, p: float) -> Optional[float]:
    values = [_numeric(row.get(field)) for row in rows]
    values = [value for value in values if value is not None]
    return float(np.percentile(values, p)) if values else None


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    materialized = list(rows)
    fields: List[str] = []
    for row in materialized:
        for key in row:
            if key not in fields:
                fields.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or ["empty"])
        writer.writeheader()
        for row in materialized:
            encoded = {}
            for key, value in row.items():
                encoded[key] = json.dumps(value, sort_keys=True, default=str) if isinstance(value, (dict, list, tuple)) else value
            writer.writerow(encoded)


def _refuse_nonempty(path: Path) -> None:
    if path.exists() and any(path.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {path}")


def _set_validity_map() -> None:
    """Point the current default 2A-V0 implementation at the requested 4x map."""
    world = ROOT / "external/arena4_ws/src/arena/simulation-setup/worlds" / MAP_ID
    validity.MAP_ID = MAP_ID
    validity.WORLD = world
    validity.MAP_YAML = world / "map/map.yaml"
    validity.SCENARIO_JSON = world / "scenarios/a2b_benchmark_20.json"


def _load_tasks() -> Tuple[List[Any], Dict[str, Any]]:
    _set_validity_map()
    # The paired loader performs the exact JSON/CSV/scenario pose comparison.
    paired._configure_map(MAP_ID)
    queries, metadata = paired._load_tasks()
    metadata = dict(metadata)
    metadata.update({"query_set_id": QUERY_SET_ID, "query_order_seed": SEED, "protocol_version": PROTOCOL_VERSION})
    return queries, metadata


def _context() -> Any:
    _set_validity_map()
    return validity._context()


def _source_manifest(ctx: Any) -> Tuple[Dict[str, str], str]:
    files, _ = validity._source_manifest()
    files.update({
        str(ROOT / "docs/PLN-02_UNIFIED_EXPERIMENT_PROTOCOL_V1.md"): sha256_file(ROOT / "docs/PLN-02_UNIFIED_EXPERIMENT_PROTOCOL_V1.md"),
        str(ROOT / "docs/PLN-02_LAYERED_ARCHITECTURE_MASTER_PLAN.md"): sha256_file(ROOT / "docs/PLN-02_LAYERED_ARCHITECTURE_MASTER_PLAN.md"),
    })
    files[str(Path(__file__).resolve())] = sha256_file(Path(__file__).resolve())
    return files, _json_hash(files)


def _directory_bytes(path: Path) -> int:
    total = 0
    if not path.exists():
        return total
    for item in path.rglob("*"):
        if item.is_file():
            try:
                total += item.stat().st_size
            except OSError:
                pass
    return total


def _available_memory_mib() -> Optional[float]:
    try:
        for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
            if line.startswith("MemAvailable:"):
                return float(line.split()[1]) / 1024.0
    except (OSError, ValueError):
        return None
    return None


def _path_points(output: Path, row: Mapping[str, Any]) -> List[Dict[str, Any]]:
    path_file = str(row.get("path_file") or "")
    if not path_file:
        return []
    try:
        return list(json.loads((output / path_file).read_text(encoding="utf-8")))
    except (OSError, ValueError, TypeError):
        return []


def _quality_fields(output: Path, row: Mapping[str, Any], query: Any) -> Dict[str, Any]:
    points = _path_points(output, row)
    length = _numeric(row.get("path_length_m"))
    euclidean = math.hypot(float(query.goal[0]) - float(query.start[0]), float(query.goal[1]) - float(query.start[1]))
    turns: List[float] = []
    for first, second in zip(points, points[1:]):
        turns.append(abs(legacy._delta(float(second.get("yaw", 0.0)), float(first.get("yaw", 0.0)))))
    large_turn_count = sum(value > math.radians(45.0) for value in turns)
    return {
        "path_point_count": len(points),
        "euclidean_ratio": (length / euclidean) if length is not None and euclidean > 1.0e-9 else None,
        "reference_ratio": "not_available",
        "mean_clearance_m": "not_available",
        "total_heading_change_rad": sum(turns) if turns else 0.0,
        "large_turn_count": int(large_turn_count),
        "heading_change_rate_p95": "not_available",
    }


def _annotate_row(output: Path, row: Mapping[str, Any], query: Any, metadata: Mapping[str, Any], topology_info: Mapping[str, Any], cache_mode: str, ready_memory_mib: Optional[float]) -> Dict[str, Any]:
    result = dict(row)
    result["architecture"] = ARCHITECTURE_ID
    result["query_role"] = "raw"
    action_success = bool(result.get("planner_success"))
    failure = str(result.get("failure_code") or "")
    action_status = str(result.get("action_status") or "")
    result_code = "SUCCEEDED" if action_success else (action_status or failure or "ACTION_ABORTED")
    last_layer = "L1" if (not _truth(result.get("l1_free_start")) or not _truth(result.get("l1_free_goal")) or failure.startswith("L1_")) else "L3_PRIME"
    if failure and failure.startswith("L1_"):
        last_layer = "L1"
    wall = _numeric(result.get("pipeline_wall_time_ms"), 0.0) or 0.0
    cpu = _numeric(result.get("pipeline_cpu_total_ms"), 0.0) or 0.0
    calls = int(_numeric(result.get("l3_prime_call_count"), 0.0) or 0)
    peak_rss = _numeric(result.get("peak_rss"))
    peak_pss = _numeric(result.get("peak_pss"))
    diag = result.get("diagnostics") if isinstance(result.get("diagnostics"), Mapping) else {}
    fallback_used = bool(result.get("fallback_used") or diag.get("fallback_used"))
    fallback_reason = str(result.get("fallback_reason") or diag.get("fallback_reason") or "")
    quality = _quality_fields(output, result, query)
    result.update({
        "experiment_id": output.name,
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_version": PROTOCOL_VERSION,
        "experiment_kind": EXPERIMENT_KIND,
        "query_set_id": QUERY_SET_ID,
        "case_id": query.query_id,
        "seed": SEED,
        "repeat": result.get("repetition"),
        "cache_mode": cache_mode,
        "corridor_profile": CORRIDOR_PROFILE,
        "corridor_semantics": CORRIDOR_SEMANTICS,
        "result_code": result_code,
        "reason_code": failure or ("" if action_success else result_code),
        "last_layer": last_layer,
        "action_success": action_success,
        "online_wall_ms": wall,
        "planner_wall_ms": _numeric(result.get("l3_action_wall_ms"), _numeric(result.get("hybrid_planning_time_ms"))),
        "cpu_ms": cpu,
        "avg_cpu_percent": (100.0 * cpu / wall) if wall > 1.0e-9 else None,
        "ready_memory_mib": ready_memory_mib if ready_memory_mib is not None else "not_available",
        "peak_memory_mib": peak_rss / (1024.0 * 1024.0) if peak_rss is not None else "not_available",
        "RSS": peak_rss if peak_rss is not None else "not_available",
        "PSS": peak_pss if peak_pss is not None else "not_available",
        "goal_yaw_error_deg": math.degrees(_numeric(result.get("goal_yaw_error_rad"), 0.0) or 0.0),
        "l1_call_count": 1 if _truth(result.get("l1_success")) or _truth(diag.get("l1_route_selected")) else 0,
        "l1_time_ms": _numeric(result.get("l1_total_time_ms"), _numeric(result.get("l1_graph_search_ms"), 0.0)),
        "l1_route_search_nodes": "not_available",
        "l2_call_count": 0,
        "l2_time_ms": 0.0,
        "l2_search_nodes_expanded": "not_available",
        "l2_search_nodes_generated": "not_available",
        "l3_call_count": calls,
        "l3_time_ms": _numeric(result.get("hybrid_planning_time_ms"), 0.0),
        "l3_retry_count": max(0, calls - 1),
        "fallback_count": 1 if fallback_used else 0,
        "fallback_trace": fallback_reason,
        "topology_cache_hit": bool(topology_info.get("topology_cache_hit")),
        "topology_load_wall_ms": _numeric(topology_info.get("topology_load_time_ms"), 0.0),
        "topology_cache_bytes": topology_info.get("topology_cache_bytes", "not_available"),
        "timeout": result_code in {"CLIENT_TIMEOUT", "PLANNER_TIMEOUT", "TIMEOUT"},
        "right_censored": result_code in {"CLIENT_TIMEOUT", "PLANNER_TIMEOUT", "TIMEOUT"},
        "metric_availability_note": "expanded/generated states, mean clearance, reference ratio, heading rate, and ready memory are not exposed by the current Smac client/validator; recorded as not_available.",
        **quality,
    })
    # Keep the formal protocol's canonical names alongside the candidate
    # runner's legacy scalar fields; values are copied, never rewritten.
    result["query_sha256"] = result.get("query_sha256") or result.get("query_hash", "")
    result["start"] = result.get("start") or json.dumps([
        _numeric(result.get("start_x")), _numeric(result.get("start_y")), _numeric(result.get("start_yaw")),
    ], separators=(",", ":"))
    result["goal"] = result.get("goal") or json.dumps([
        _numeric(result.get("goal_x")), _numeric(result.get("goal_y")), _numeric(result.get("goal_yaw")),
    ], separators=(",", ":"))
    result["heading_jump_count"] = result.get("heading_jump_count", result.get("heading_discontinuity_count", 0))
    result["reverse_length_m"] = result.get("reverse_length_m", result.get("reverse_distance_m", 0.0))
    return result


def _annotate_call(output_row: Mapping[str, Any], call: Mapping[str, Any], query: Any, output: Path) -> Dict[str, Any]:
    item = dict(call)
    item.update({
        "experiment_id": output.name,
        "architecture": ARCHITECTURE_ID,
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_version": PROTOCOL_VERSION,
        "query_set_id": QUERY_SET_ID,
        "case_id": query.query_id,
        "seed": SEED,
        "l2_called": False,
        "l2_call_count": 0,
        "l3_call_count": output_row.get("l3_call_count", output_row.get("l3_prime_call_count", 0)),
        "action_success": output_row.get("action_success"),
        "final_valid_success": output_row.get("final_valid_success"),
        "result_code": output_row.get("result_code"),
        "reason_code": output_row.get("reason_code"),
        "last_layer": output_row.get("last_layer"),
        "fallback_count": output_row.get("fallback_count", 0),
        "fallback_trace": output_row.get("fallback_trace", ""),
        "cache_mode": output_row.get("cache_mode"),
        "topology_cache_hit": output_row.get("topology_cache_hit"),
        "query_sha256": item.get("query_sha256") or item.get("query_hash", ""),
    })
    return item


def _load_or_build_topology(ctx: Any, output: Path, cache_root: Path) -> Tuple[Any, Dict[str, Any]]:
    topology, info = validity._load_topology(ctx, output, cache_root)
    info = dict(info)
    info["topology_cache_bytes"] = _directory_bytes(Path(info.get("cache_directory", cache_root)))
    info["topology_build_cpu_ms"] = info.get("topology_build_cpu_time_ms", 0.0)
    return topology, info


def _historical_two_layer_summary(path: Path) -> Dict[str, Any]:
    """Return the two-layer row from a paired summary, independent of row order."""
    if not path.exists():
        return {}
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            for item in csv.DictReader(stream):
                if str(item.get("architecture", "")).strip().lower() == "two_layer":
                    return dict(item)
    except (OSError, csv.Error):
        return {}
    return {}


def _report(output: Path, rows: Sequence[Mapping[str, Any]], metadata: Mapping[str, Any], topology_info: Mapping[str, Any], session_info: Mapping[str, Any], cache_mode: str, source_hash: str, *, preflight_cache_build_count: int = 0) -> Dict[str, Any]:
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    valid = sum(_truth(row.get("final_valid_success")) for row in measured)
    failures = collections.Counter(str(row.get("reason_code") or row.get("failure_code") or "") for row in measured if row.get("reason_code") or row.get("failure_code"))
    walls = [row for row in measured if _numeric(row.get("online_wall_ms")) is not None]
    timeout_count = sum(_truth(row.get("timeout")) for row in measured)
    path_rows = [row for row in measured if _truth(row.get("final_valid_success"))]
    def fmt(value: Any, digits: int = 2) -> str:
        return "not_available" if value is None else f"{float(value):.{digits}f}"
    historical = ROOT / "experiments/layered_planner_benchmark/l1_l2_l3_vs_l1_l3prime_mentor_map_20260825_005_4x_area_20_v1/paired_summary.csv"
    baseline_summary = _historical_two_layer_summary(historical)
    total_cache_build_count = int(topology_info.get("topology_build_count", 0) or 0) + int(preflight_cache_build_count or 0)
    report = [
        f"# {ARCHITECTURE_ID} formal experiment",
        "",
        "This is an independent static 20-query experiment on `mentor_map_20260825_005_4x_area`; it is not a multi-map conclusion.",
        "",
        f"- Architecture: `{ARCHITECTURE_ID}`; implementation revision: `{IMPLEMENTATION_REVISION}`; protocol: `{PROTOCOL_VERSION}`; kind: `{EXPERIMENT_KIND}`.",
        f"- Map/query validation: `{MAP_ID}`, A2B-01..A2B-20; JSON/CSV/scenario poses matched exactly ({metadata.get('json_task_count')}/{metadata.get('csv_task_count')}/20); resolution=0.05 m/cell; dynamic_obstacles=false.",
        f"- Fixed constraints: Rmin=0.40 m, maximum curvature=2.50 1/m, allow_reverse=false, allow_in_place_rotation=false, full Jackal footprint validation.",
        f"- Cache mode: `{cache_mode}`; topology cache hit={bool(topology_info.get('topology_cache_hit'))}; formal-process build/load count={topology_info.get('topology_build_count', 0)}/{topology_info.get('topology_load_count', 0)}; lifecycle build count (including preflight)={total_cache_build_count}; cache bytes={topology_info.get('topology_cache_bytes', 'not_available')}; build wall/CPU={fmt(topology_info.get('topology_build_time_ms'))}/{fmt(topology_info.get('topology_build_cpu_ms'))} ms; load wall={fmt(topology_info.get('topology_load_time_ms'))} ms.",
        f"- The independent preflight built this cache {int(preflight_cache_build_count or 0)} time(s); the formal 160-run process loaded it once and did not rebuild it. Build and load costs are reported separately from online query wall time.",
        f"- Session start/close/restart={session_info.get('session_start_count', 0)}/{session_info.get('session_close_count', 0)}/{session_info.get('session_restart_count', 0)}; startup/shutdown={fmt(session_info.get('session_startup_time_ms'))}/{fmt(session_info.get('session_shutdown_time_ms'))} ms.",
        "",
        "## Results",
        "",
        f"- Measured final-valid: **{valid}/{len(measured)} ({100.0 * valid / len(measured) if measured else 0.0:.1f}%)**; query-any-valid={sum(any(_truth(row.get('final_valid_success')) for row in measured if row.get('query_id') == q) for q in sorted({row.get('query_id') for row in measured}))}/{len(set(row.get('query_id') for row in measured)) if measured else 0}; query-all-repeat-valid={sum(all(_truth(row.get('final_valid_success')) for row in measured if row.get('query_id') == q) for q in sorted({row.get('query_id') for row in measured}))}/{len(set(row.get('query_id') for row in measured)) if measured else 0}.",
        f"- Online wall P50/P95/P99={fmt(_percentile(walls, 'online_wall_ms', 50))}/{fmt(_percentile(walls, 'online_wall_ms', 95))}/{fmt(_percentile(walls, 'online_wall_ms', 99))} ms; success-only P50/P95/P99={fmt(_percentile(path_rows, 'online_wall_ms', 50))}/{fmt(_percentile(path_rows, 'online_wall_ms', 95))}/{fmt(_percentile(path_rows, 'online_wall_ms', 99))} ms; timeout rate={timeout_count}/{len(measured)}.",
        f"- Right-censored wall P50/P95/P99={fmt(_percentile([row for row in measured if _truth(row.get('right_censored'))], 'online_wall_ms', 50))}/{fmt(_percentile([row for row in measured if _truth(row.get('right_censored'))], 'online_wall_ms', 95))}/{fmt(_percentile([row for row in measured if _truth(row.get('right_censored'))], 'online_wall_ms', 99))} ms.",
        f"- CPU P50/P95/P99={fmt(_percentile(measured, 'cpu_ms', 50))}/{fmt(_percentile(measured, 'cpu_ms', 95))}/{fmt(_percentile(measured, 'cpu_ms', 99))} ms; RSS P50/P95/P99={fmt(_percentile(measured, 'RSS', 50), 0)}/{fmt(_percentile(measured, 'RSS', 95), 0)}/{fmt(_percentile(measured, 'RSS', 99), 0)} bytes; PSS P50/P95/P99={fmt(_percentile(measured, 'PSS', 50), 0)}/{fmt(_percentile(measured, 'PSS', 95), 0)}/{fmt(_percentile(measured, 'PSS', 99), 0)} bytes.",
        f"- Calls: L1={sum(int(_numeric(row.get('l1_call_count'), 0) or 0) for row in measured)}, L2={sum(int(_numeric(row.get('l2_call_count'), 0) or 0) for row in measured)}, L3'={sum(int(_numeric(row.get('l3_call_count'), 0) or 0) for row in measured)}; corridor expansions beyond the initial 2 m={sum(max(0, int(_numeric(row.get('l3_retry_count'), 0) or 0)) for row in measured)}; retries={sum(int(_numeric(row.get('l3_retry_count'), 0) or 0) for row in measured)}; fallbacks={sum(int(_numeric(row.get('fallback_count'), 0) or 0) for row in measured)}.",
        f"- Mean planner wall={fmt(np.mean([_numeric(row.get('planner_wall_ms'), 0.0) or 0.0 for row in measured]) if measured else None)} ms; mean corridor mask={fmt(np.mean([_numeric(row.get('corridor_mask_total_time_ms'), 0.0) or 0.0 for row in measured]) if measured else None)} ms.",
        f"- Mean layer timings (measured): L1={fmt(np.mean([_numeric(row.get('l1_time_ms'), 0.0) or 0.0 for row in measured]) if measured else None)} ms; L2=0.00 ms (disabled); L3'={fmt(np.mean([_numeric(row.get('l3_time_ms'), 0.0) or 0.0 for row in measured]) if measured else None)} ms.",
        f"- L1 cache-hit rates: adjacency={sum(_truth(row.get('topology_adjacency_cache_hit')) for row in measured)}/{len(measured)}, endpoint index={sum(_truth(row.get('endpoint_spatial_index_cache_hit')) for row in measured)}/{len(measured)}, route={sum(_truth(row.get('route_cache_hit')) for row in measured)}/{len(measured)}. Cached L1 paths report zero measured search time because no uncached search work was performed; the value is not treated as a speed estimate for an uncached run.",
        f"- Path quality over final-valid samples: length={fmt(np.mean([_numeric(row.get('path_length_m')) for row in path_rows if _numeric(row.get('path_length_m')) is not None]) if path_rows else None)} m; euclidean ratio={fmt(np.mean([_numeric(row.get('euclidean_ratio')) for row in path_rows if _numeric(row.get('euclidean_ratio')) is not None]) if path_rows else None, 4)}; minimum clearance={fmt(np.mean([_numeric(row.get('minimum_clearance_m')) for row in path_rows if _numeric(row.get('minimum_clearance_m')) is not None]) if path_rows else None)} m; maximum curvature={fmt(np.max([_numeric(row.get('maximum_curvature')) for row in path_rows if _numeric(row.get('maximum_curvature')) is not None]) if path_rows else None, 4)} 1/m; heading-rate P95=not_available; large turns={sum(int(_numeric(row.get('large_turn_count'), 0) or 0) for row in path_rows)}.",
        f"- Hard validation totals: collision cases={sum(not _truth(row.get('static_footprint_valid')) and _truth(row.get('action_success')) for row in measured)}, kinematic-invalid cases={sum(not _truth(row.get('kinematic_valid')) and _truth(row.get('action_success')) for row in measured)}, reverse distance={fmt(sum(_numeric(row.get('reverse_distance_m'), 0.0) or 0.0 for row in measured))} m, in-place rotations={sum(int(_numeric(row.get('in_place_rotation_count'), 0.0) or 0) for row in measured)}.",
        f"- Failure distribution: `{dict(failures)}`.",
        "",
        "## Historical comparison",
        "",
        f"- Historical same-map 3A-V0 reference: 48/100 final-valid, online P50/P95/P99=12811.86/40716.34/48112.95 ms, L2 calls=135 and local L3 calls=1729.",
        f"- Historical fixed-2 m 2A-V0 reference: {baseline_summary.get('final_valid_count', 'not_available')}/{baseline_summary.get('query_count', 'not_available')} final-valid, P50/P95/P99={baseline_summary.get('p50_ms', 'not_available')}/{baseline_summary.get('p95_ms', 'not_available')}/{baseline_summary.get('p99_ms', 'not_available')} ms. It is a trend reference, not a same-round rerun; this r3 run uses bounded 2/4/6 m expansion and optimized cache mode.",
        "- The map, A2B-01..A2B-20 task set, 0.05 m resolution, hard kinematic constraints, and lighter_smoother/v7_candidate Smac profile match the historical protocol. Cache/session lifecycle and implementation revision are separate runs, so this is not a strict same-round comparison.",
        "- The two-layer run never calls `plan_grid_astar`; L2 is structurally disabled. RRTstar/SST are disabled and have zero calls.",
        "",
        "## Metric availability and decision",
        "",
        "- `expanded/generated states`, `mean_clearance_m`, `reference_ratio`, and `heading_change_rate_p95` are `not_available` because the current Smac client/validator does not expose them; no values are estimated. `ready_memory_mib` is recorded when `/proc/meminfo` is readable.",
        f"- Formal validity gate (all 100 measured paths valid): **{'PASS' if valid == len(measured) else 'FAIL'}**. This result must be interpreted with the retained failure records; failed samples are not removed or relabeled.",
        "- The result is suitable for comparison against the historical 3A-V0 record only as a same-map trend unless build/profile/session metadata are identical. It does not establish general superiority across maps.",
        f"- Source hash: `{source_hash}`; all paths and failures remain in `paths/`, `runs.csv`, and `backend_call_log.csv`.",
    ]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    summary = {
        "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "final_valid_count": valid, "measured_count": len(measured), "final_valid_rate": valid / len(measured) if measured else 0.0,
        "online_p50_ms": _percentile(walls, "online_wall_ms", 50), "online_p95_ms": _percentile(walls, "online_wall_ms", 95), "online_p99_ms": _percentile(walls, "online_wall_ms", 99),
        "failure_counts": dict(failures), "l1_call_count": sum(int(_numeric(row.get("l1_call_count"), 0) or 0) for row in measured),
        "l2_call_count": 0, "l3_prime_call_count": sum(int(_numeric(row.get("l3_call_count"), 0) or 0) for row in measured),
        "fallback_count": sum(int(_numeric(row.get("fallback_count"), 0) or 0) for row in measured), "gate_passed": valid == len(measured),
        "preflight_cache_build_count": int(preflight_cache_build_count or 0), "topology_build_count_total": total_cache_build_count,
    }
    _write_csv(output / "summary.csv", [summary])
    failure_rows = [{"failure_code": code, "count": count} for code, count in sorted(failures.items())]
    if failure_rows:
        _write_csv(output / "failure_summary.csv", failure_rows)
    else:
        # Preserve the failure-summary schema even when the measured set is clean.
        (output / "failure_summary.csv").write_text("failure_code,count\n", encoding="utf-8")
    return summary


def run_formal(output: Path, *, cache_mode: str = "optimized", warmups: int = WARMUPS, repetitions: int = REPETITIONS, query_ids: Optional[Sequence[str]] = None, ros_domain_id: int = 118, topology_cache_dir: Optional[Path] = None, preflight_cache_build_count: int = 0) -> Path:
    if cache_mode not in {"baseline", "optimized"}:
        raise ValueError("cache_mode must be baseline or optimized")
    if warmups < 0 or repetitions <= 0:
        raise ValueError("warmups must be >= 0 and repetitions > 0")
    if preflight_cache_build_count < 0:
        raise ValueError("preflight_cache_build_count must be >= 0")
    _refuse_nonempty(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir()
    queries, metadata = _load_tasks()
    selected = list(query_ids or [query.query_id for query in queries])
    if selected != [query.query_id for query in queries] and set(selected) != {query.query_id for query in queries}:
        query_map = {query.query_id: query for query in queries}
        if any(item not in query_map for item in selected):
            raise ValueError("query_ids must be A2B-01..A2B-20")
        queries = [query_map[item] for item in selected]
    ctx = _context()
    cache_root = (topology_cache_dir or (output / "topology_cache")).resolve()
    topology, topology_info = _load_or_build_topology(ctx, output, cache_root)
    source_files, source_hash = _source_manifest(ctx)
    spec = legacy.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"BACKEND_UNAVAILABLE: {spec.reason}")
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    session = candidate.SmacSession(ctx, output, map_yaml=validity.MAP_YAML, log_tag=f"formal_2a_r3_{MAP_ID}", local_mask_updates=True, optimization_profile=OPTIMIZATION_PROFILE, smac_parameter_profile=SMAC_PARAMETER_PROFILE, optimization_stage=OPTIMIZATION_STAGE)
    session.start()
    rows: List[Dict[str, Any]] = []
    calls: List[Dict[str, Any]] = []
    metrics: List[Dict[str, Any]] = []
    try:
        for run_mode, count in (("warmup", warmups), ("measured", repetitions)):
            for repetition in range(1, int(count) + 1):
                for query in queries:
                    ready_memory = _available_memory_mib()
                    row, call, metric = candidate._run_one(
                        ctx, topology, topology_info, query, run_mode, repetition, session, spec,
                        output, validity._source_commit(), corridor_padding_m=PADDING_SCHEDULE[0],
                        corridor_semantics=CORRIDOR_SEMANTICS, profile_name=CORRIDOR_PROFILE,
                        padding_schedule_m=PADDING_SCHEDULE, force_full_update=True,
                        validate_each_attempt=True, cache_mode=cache_mode,
                    )
                    row = _annotate_row(output, row, query, metadata, topology_info, cache_mode, ready_memory)
                    row["source_hash"] = source_hash
                    call = _annotate_call(row, call, query, output)
                    metric_row = {**metric, "experiment_id": output.name, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "protocol_version": PROTOCOL_VERSION, "case_id": query.query_id, "action_success": row.get("action_success"), "static_footprint_valid": row.get("static_footprint_valid"), "kinematic_valid": row.get("kinematic_valid"), "final_valid_success": row.get("final_valid_success"), "result_code": row.get("result_code"), "reason_code": row.get("reason_code"), "path_hash": row.get("path_hash"), **_quality_fields(output, row, query)}
                    metric_row["query_sha256"] = metric_row.get("query_sha256") or metric_row.get("query_hash", "")
                    metric_row["heading_jump_count"] = metric_row.get("heading_jump_count", metric_row.get("heading_discontinuity_count", 0))
                    metric_row["reverse_length_m"] = metric_row.get("reverse_length_m", metric_row.get("reverse_distance_m", 0.0))
                    rows.append(row); calls.append(call); metrics.append(metric_row)
    finally:
        session.close()
    session_info = {
        "experiment_id": output.name, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_version": PROTOCOL_VERSION, "map_id": MAP_ID, "ros_domain_id": ros_domain_id,
        "session_start_count": session.session_start_count, "session_close_count": session.session_close_count, "session_restart_count": session.session_restart_count,
        "session_startup_time_ms": session.stack_startup_time_ms, "session_shutdown_time_ms": session.stack_shutdown_time_ms,
        "topology_build_count": topology_info.get("topology_build_count", 0), "topology_load_count": topology_info.get("topology_load_count", 0),
        "topology_build_wall_ms": topology_info.get("topology_build_time_ms", 0.0), "topology_build_cpu_ms": topology_info.get("topology_build_cpu_ms", 0.0),
        "topology_load_wall_ms": topology_info.get("topology_load_time_ms", 0.0), "topology_cache_hit": topology_info.get("topology_cache_hit", False),
        "preflight_cache_build_count": int(preflight_cache_build_count or 0),
        "topology_build_count_total": int(topology_info.get("topology_build_count", 0) or 0) + int(preflight_cache_build_count or 0),
    }
    _write_csv(output / "runs.csv", rows)
    _write_csv(output / "path_metrics.csv", metrics)
    _write_csv(output / "backend_call_log.csv", calls)
    _write_csv(output / "session_timing.csv", [session_info])
    (output / "topology_cache_manifest.yaml").write_text(yaml.safe_dump({**topology_info, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "cache_mode": cache_mode}, sort_keys=False), encoding="utf-8")
    (output / "protocol.yaml").write_text(yaml.safe_dump({
        "experiment_id": output.name, "protocol_version": PROTOCOL_VERSION, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "experiment_kind": EXPERIMENT_KIND,
        "map_id": MAP_ID, "query_set_id": QUERY_SET_ID, "query_ids": [query.query_id for query in queries], "seed": SEED,
        "warmups": warmups, "repetitions": repetitions, "resolution_m": 0.05, "dynamic_obstacles": False,
        "minimum_turning_radius_m": 0.40, "maximum_curvature_1pm": 2.50, "allow_reverse": False, "allow_in_place_rotation": False,
        "layers": {"L1": "skeleton distance-transform topology + Graph A*", "L2": "disabled", "L3_prime": "corridor-constrained Smac Hybrid DUBIN"},
        "corridor_profile": CORRIDOR_PROFILE, "corridor_semantics": CORRIDOR_SEMANTICS, "padding_schedule_m": list(PADDING_SCHEDULE),
        "cache_mode": cache_mode, "smac_parameter_profile": SMAC_PARAMETER_PROFILE, "optimization_profile": OPTIMIZATION_PROFILE, "optimization_stage": OPTIMIZATION_STAGE,
        "l2_called": False, "l2_call_count": 0, "rrtstar_call_count": 0, "sst_call_count": 0,
        "metric_availability": {"expanded_generated_states": "not_available: Smac client does not expose state counters", "mean_clearance_m": "not_available: validator exposes minimum only", "reference_ratio": "not_available: no approved reference path", "heading_change_rate_p95": "not_available: no temporal sampling", "ready_memory_mib": "not_available when /proc/meminfo is unavailable"},
    }, sort_keys=False), encoding="utf-8")
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({"experiment_id": output.name, "protocol_version": PROTOCOL_VERSION, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "source_commit": validity._source_commit(), "source_hash": source_hash, "source_files": source_files, "map_id": MAP_ID, "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256, "query_sha256": {query.query_id: paired._query_hash(query) for query in queries}, "footprint_hash": _json_hash(legacy.FOOTPRINT), "smac_parameter_profile": SMAC_PARAMETER_PROFILE}, sort_keys=False), encoding="utf-8")
    summary = _report(output, rows, metadata, topology_info, session_info, cache_mode, source_hash, preflight_cache_build_count=preflight_cache_build_count)
    (output / "manifest.yaml").write_text(yaml.safe_dump({
        "experiment_id": output.name, "protocol_version": PROTOCOL_VERSION, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "experiment_kind": EXPERIMENT_KIND,
        "map_id": MAP_ID, "query_set_id": QUERY_SET_ID, "query_ids": [query.query_id for query in queries], "warmup_count": warmups, "measured_repetitions": repetitions, "run_count": len(rows),
        "cache_mode": cache_mode, "topology_build_count": topology_info.get("topology_build_count", 0), "topology_load_count": topology_info.get("topology_load_count", 0), "topology_build_count_total": summary.get("topology_build_count_total", topology_info.get("topology_build_count", 0)), "preflight_cache_build_count": summary.get("preflight_cache_build_count", 0), "topology_build_wall_ms": topology_info.get("topology_build_time_ms", 0.0), "topology_build_cpu_ms": topology_info.get("topology_build_cpu_ms", 0.0), "topology_load_wall_ms": topology_info.get("topology_load_time_ms", 0.0), "topology_cache_bytes": topology_info.get("topology_cache_bytes", "not_available"),
        "session_start_count": session_info["session_start_count"], "session_close_count": session_info["session_close_count"], "session_restart_count": session_info["session_restart_count"], "l2_call_count": 0, "rrtstar_call_count": 0, "sst_call_count": 0, "source_hash": source_hash, "metric_availability": "see protocol.yaml", **summary,
    }, sort_keys=False), encoding="utf-8")
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the default 2A-V0 formal benchmark on mentor_map_20260825_005_4x_area")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--cache-mode", choices=("baseline", "optimized"), default="optimized")
    parser.add_argument("--warmups", type=int, default=WARMUPS)
    parser.add_argument("--repetitions", type=int, default=REPETITIONS)
    parser.add_argument("--query-id", action="append", dest="query_ids", help="bounded preflight subset; formal run uses all A2B-01..A2B-20")
    parser.add_argument("--ros-domain-id", type=int, default=118)
    parser.add_argument("--topology-cache-dir", default=None, help="independent cache root shared by preflight and formal runs")
    parser.add_argument("--preflight-cache-build-count", type=int, default=0, help="number of cache builds performed by a separate preflight process")
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output = run_formal(Path(args.output_dir).resolve(), cache_mode=args.cache_mode, warmups=args.warmups, repetitions=args.repetitions, query_ids=args.query_ids, ros_domain_id=args.ros_domain_id, topology_cache_dir=Path(args.topology_cache_dir).resolve() if args.topology_cache_dir else None, preflight_cache_build_count=args.preflight_cache_build_count)
    except Exception as exc:
        print(f"2a_v0_formal_benchmark: ERROR: {exc}")
        return 2
    print(f"2A-V0 output (implementation_revision={IMPLEMENTATION_REVISION}): {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
