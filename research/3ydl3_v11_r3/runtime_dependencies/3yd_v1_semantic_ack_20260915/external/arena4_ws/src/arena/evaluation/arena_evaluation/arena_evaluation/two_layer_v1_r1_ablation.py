"""Small, independent A-F ablation for the 2A-V1-r1 cache work.

This diagnostic intentionally runs only two frozen queries once per profile. It
does not change the formal runner or its default profile. Every row is produced
by the real Smac session and is kept outside the formal measured denominator.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import l1_l3_corridor_hybrid_smoke as candidate
from . import two_layer_v1_formal_benchmark as parent
from . import two_layer_v1_r1_cache_benchmark as r1


ROOT = parent.ROOT
DEFAULT_OUTPUT = ROOT / "experiments/layered_planner_benchmark/2a_v1_r1_ablation_v1"
DEFAULT_CACHE = ROOT / "experiments/layered_planner_benchmark/2a_v1_r1_ablation_topology_cache"
QUERY_IDS = ("A2B-02", "A2B-07")


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fields: List[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or ["empty"])
        writer.writeheader()
        for row in rows:
            writer.writerow({
                key: json.dumps(value, sort_keys=True, default=str)
                if isinstance(value, (dict, list, tuple)) else value
                for key, value in row.items()
            })


def _threshold_intervals(analysis: Mapping[str, Any], threshold: float) -> List[List[float]]:
    samples = [(float(item[0]), float(item[1])) for item in analysis.get("curvature_samples", []) or []]
    selected = [distance for distance, curvature in samples if curvature >= threshold]
    intervals: List[List[float]] = []
    for distance in selected:
        interval = [max(0.0, distance - 1.0), distance + 1.0]
        if intervals and interval[0] <= intervals[-1][1] + 0.11:
            intervals[-1][1] = max(intervals[-1][1], interval[1])
        else:
            intervals.append(interval)
    cumulative = analysis.get("route_cumulative_m")
    total = float(cumulative[-1]) if cumulative is not None and len(cumulative) else 0.0
    for interval in intervals:
        interval[1] = min(total, interval[1])
    return intervals


def _profile_builder(name: str):
    if name == "A_2m_full_route":
        return lambda ctx, topology, route, query, start, goal, padding, semantics: (
            candidate._build_corridor_mask(ctx, topology, route, query, start, goal, 2.0, semantics),
            {"ablation_profile": name},
        )
    if name == "B_adaptive_2m_4m":
        return lambda ctx, topology, route, query, start, goal, padding, semantics: (
            parent.build_adaptive_corridor_mask(ctx, topology, route, query, start, goal, 2.0, semantics)
        )
    if name == "C_2m_3m_4m":
        def build(ctx, topology, route, query, start, goal, padding, semantics):
            analysis = parent.analyze_topology_route(route, topology)
            base = candidate._build_corridor_mask(ctx, topology, route, query, start, goal, 2.0, semantics)
            medium = parent._corner_centerline(ctx, {**analysis, "corner_intervals_m": _threshold_intervals(analysis, 0.5)})
            high = parent._corner_centerline(ctx, {**analysis, "corner_intervals_m": _threshold_intervals(analysis, 1.0)})
            mask = np.asarray(base, dtype=bool)
            mask |= parent._dilate_raw(ctx, medium, 3.0)
            mask |= parent._dilate_raw(ctx, high, 4.0)
            mask &= candidate._raw_free_mask(ctx)
            return mask, {"ablation_profile": name, "corner_analysis": analysis}
        return build
    return None


def _summarize(name: str, rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    valid = sum(str(row.get("final_valid_success", "")).lower() == "true" for row in rows)
    values = lambda field: [float(row[field]) for row in rows if row.get(field) not in (None, "")]
    wall = values("pipeline_wall_time_ms")
    mask = values("total_corridor_mask_online_ms")
    update = values("costmap_update_ms")
    plan = values("hybrid_planning_time_ms")
    pct = lambda data, p: float(np.percentile(data, p)) if data else None
    return {
        "profile": name, "query_count": len(rows), "final_valid_count": valid,
        "mask_cache_hit_rate": sum(str(row.get("mask_cache_hit", "")).lower() == "true" for row in rows) / max(1, len(rows)),
        "mask_p50_ms": pct(mask, 50), "mask_p95_ms": pct(mask, 95),
        "costmap_update_p50_ms": pct(update, 50), "costmap_update_p95_ms": pct(update, 95),
        "smac_planning_p50_ms": pct(plan, 50), "smac_planning_p95_ms": pct(plan, 95),
        "online_wall_p50_ms": pct(wall, 50), "online_wall_p95_ms": pct(wall, 95),
        "allowed_cells": sorted({int(float(row.get("allowed_grid_cells") or 0)) for row in rows}),
        "failure_codes": sorted({str(row.get("failure_code") or "") for row in rows if row.get("failure_code")}),
    }


def run_ablation(output: Path, *, query_ids: Sequence[str] = QUERY_IDS, ros_domain_id: int = 231) -> Path:
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir()
    queries, _metadata = parent._load_tasks()
    query_map = {query.query_id: query for query in queries}
    selected = [query_map[item] for item in query_ids]
    ctx = parent._context()
    topology, topology_info = parent._load_or_build_topology(ctx, output, DEFAULT_CACHE)
    spec = parent.legacy.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"BACKEND_UNAVAILABLE: {spec.reason}")
    source_hash = r1._source_manifest()[1]
    route_cache = r1.RouteMaskCache(ctx, topology, source_hash, output / "cache")
    route_cache.prepare(selected)
    import os
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    session = candidate.SmacSession(
        ctx, output, map_yaml=parent.validity.MAP_YAML,
        log_tag="ablation_2a_v1_r1", local_mask_updates=True,
        optimization_profile=parent.OPTIMIZATION_PROFILE,
        smac_parameter_profile=parent.SMAC_PARAMETER_PROFILE,
        optimization_stage=parent.OPTIMIZATION_STAGE,
        enable_mask_reuse_noop=True,
    )
    session.start()
    all_rows: List[Dict[str, Any]] = []
    summaries: List[Dict[str, Any]] = []
    profiles = ("A_2m_full_route", "B_adaptive_2m_4m", "C_2m_3m_4m", "D_edge_cache_only", "E_edge_route_cache", "F_edge_route_costmap_reuse")
    try:
        for profile in profiles:
            session.enable_mask_reuse_noop = profile == "F_edge_route_costmap_reuse"
            if profile in {"A_2m_full_route", "B_adaptive_2m_4m", "C_2m_3m_4m"}:
                builder = _profile_builder(profile)
                mode = candidate.CACHE_MODE_BASELINE
            elif profile == "D_edge_cache_only":
                builder = _profile_builder("B_adaptive_2m_4m")
                mode = candidate.CACHE_MODE_BASELINE
            else:
                builder = route_cache.builder
                mode = candidate.CACHE_MODE_OPTIMIZED
            profile_rows: List[Dict[str, Any]] = []
            for query in selected:
                row, _call, _metric = candidate._run_one(
                    ctx, topology, topology_info, query, "ablation", 1, session, spec, output,
                    parent.validity._source_commit(), corridor_padding_m=2.0,
                    corridor_semantics=parent.CORRIDOR_SEMANTICS,
                    profile_name=profile, padding_schedule_m=(2.0,),
                    force_full_update=False, validate_each_attempt=True,
                    cache_mode=mode, corridor_mask_builder=builder,
                )
                row["ablation_profile"] = profile
                row["architecture_id"] = "2A-V1"
                row["implementation_revision"] = "r1"
                profile_rows.append(row)
                all_rows.append(row)
            summaries.append(_summarize(profile, profile_rows))
    finally:
        session.close()
    _write_csv(output / "runs.csv", all_rows)
    _write_csv(output / "corridor_profile_comparison.csv", summaries)
    _write_csv(output / "cache_diagnostics.csv", [{"metric": "route_cache_hits", "value": route_cache.route_hits}, {"metric": "route_cache_misses", "value": route_cache.route_misses}, {"metric": "edge_cache_hits", "value": route_cache.edge_hits}, {"metric": "edge_cache_misses", "value": route_cache.edge_misses}, {"metric": "offline_build_ms", "value": route_cache.offline_build_ms}])
    (output / "topology_cache_manifest.yaml").write_text(yaml.safe_dump(topology_info, sort_keys=False), encoding="utf-8")
    report = [
        "# 2A-V1-r1 A-F ablation (diagnostic)",
        "",
        "Two frozen queries (A2B-02, A2B-07), one real Smac call budget per profile; rows are diagnostic only and are excluded from the formal 100 measured cases.",
        "",
        f"- topology build/load: {topology_info.get('topology_build_count', 0)}/{topology_info.get('topology_load_count', 0)}; session start/close/restart: {session.session_start_count}/{session.session_close_count}/{session.session_restart_count}.",
        "- Profile A/B/C compare corridor geometry; D/E/F compare edge/route cache and guarded costmap reuse. No profile uses 6 m, L2, RRTstar, or SST.",
        "- All rows retain raw Smac status and final validation; no path fields are rewritten.",
        "",
        "| profile | final-valid | mask P50/P95 ms | costmap P50/P95 ms | Smac P50/P95 ms | online P50/P95 ms | failure |",
        "|---|---:|---:|---:|---:|---:|---|",
    ]
    for item in summaries:
        report.append(f"| {item['profile']} | {item['final_valid_count']}/{item['query_count']} | {item['mask_p50_ms']}/{item['mask_p95_ms']} | {item['costmap_update_p50_ms']}/{item['costmap_update_p95_ms']} | {item['smac_planning_p50_ms']}/{item['smac_planning_p95_ms']} | {item['online_wall_p50_ms']}/{item['online_wall_p95_ms']} | {item['failure_codes']} |")
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    (output / "manifest.yaml").write_text(yaml.safe_dump({"experiment_id": output.name, "architecture_id": "2A-V1", "implementation_revision": "r1", "experiment_kind": "ablation", "query_ids": list(query_ids), "profile_count": len(profiles), "run_count": len(all_rows), "l2_call_count": 0, "rrtstar_call_count": 0, "sst_call_count": 0, "metric_availability": {"expanded_generated_states": "not_available"}}, sort_keys=False), encoding="utf-8")
    return output


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Run the diagnostic A-F cache ablation")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--ros-domain-id", type=int, default=231)
    args = parser.parse_args(argv)
    try:
        output = run_ablation(Path(args.output_dir).resolve(), ros_domain_id=args.ros_domain_id)
    except Exception as exc:
        print(f"2a_v1_r1_ablation: ERROR: {exc}")
        return 2
    print(f"2A-V1-r1 ablation output: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
