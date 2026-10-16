"""Independent full-map endpoint-yaw diagnostics for A2B-07 and A2B-16."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

import numpy as np
import yaml

from . import endpoint_heading
from . import l1_l3_corridor_hybrid_smoke as candidate
from . import path_audit
from . import two_layer_v1_formal_benchmark as parent
from . import two_layer_v1_r2_roi_pathaudit_benchmark as r2
from .planner_benchmark.models import Query


DEFAULT_OUTPUT = parent.ROOT / "experiments/layered_planner_benchmark/2a_v1_r2_endpoint_diagnostics_v1"
DEFAULT_TOPOLOGY_CACHE = r2.DEFAULT_TOPOLOGY_CACHE
VARIANTS = ("original", "start_yaw_aligned", "goal_yaw_aligned", "both_yaw_aligned")


def _variant(query: Query, name: str, start_tangent: float, goal_tangent: float) -> Query:
    start = list(query.start)
    goal = list(query.goal)
    if name in {"start_yaw_aligned", "both_yaw_aligned"}:
        start[2] = float(start_tangent)
    if name in {"goal_yaw_aligned", "both_yaw_aligned"}:
        goal[2] = float(goal_tangent)
    return Query(
        query_id=f"{query.query_id}_{name}", start=start, goal=goal,
        category="endpoint_yaw_diagnostic", seed=query.seed,
        validation_status=query.validation_status,
    )


def _diagnostic_valid(metrics: Mapping[str, Any], *, reverse_mode: bool) -> bool:
    failure = str(metrics.get("failure_code") or "")
    if reverse_mode and failure == "REVERSE_MOTION":
        failure = ""
    return bool(
        metrics.get("static_footprint_valid")
        and float(metrics.get("maximum_curvature") or 0.0) <= 2.501
        and int(metrics.get("in_place_rotation_count") or 0) == 0
        and int(metrics.get("position_discontinuity_count") or 0) == 0
        and not failure
    )


def _run_session(
    session: Any, queries: Sequence[Query], variants: Mapping[str, Sequence[Query]],
    auditor: Any, spec: Any, raw_free: np.ndarray, output: Path,
    *, reverse_mode: bool, warmups: int, repetitions: int,
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for run_mode, count in (("warmup", warmups), ("measured", repetitions)):
        for repetition in range(1, count + 1):
            for original in queries:
                candidates = list(variants[original.query_id])
                if reverse_mode:
                    candidates = [candidates[0]]
                for query in candidates:
                    session.reset_query_state(query.query_id, restore_base_map=False)
                    started_ns = time.monotonic_ns()
                    result = session.plan(
                        query, spec, source="full_map_endpoint_diagnostic",
                        allowed_mask=raw_free, skip_path_mask_validation=True,
                    )
                    diagnostics = dict(result.diagnostics or {})
                    metrics: Dict[str, Any] = {}
                    path_hash = ""
                    path_file = ""
                    if result.points:
                        audit = auditor.audit(query, result.points, raw_free)
                        result.path_audit = audit
                        metrics = dict(audit.metrics)
                        diagnostics.update(audit.diagnostics())
                        path_hash = audit.path_hash
                        path_file = f"paths/{query.query_id}_{'reverse' if reverse_mode else 'forward'}_{run_mode}_{repetition}.json"
                        (output / path_file).write_text(
                            json.dumps(result.points, indent=2, sort_keys=True), encoding="utf-8",
                        )
                    valid = bool(result.planner_success and _diagnostic_valid(metrics, reverse_mode=reverse_mode))
                    rows.append({
                        "base_query_id": original.query_id,
                        "diagnostic_query_id": query.query_id,
                        "variant": "allow_reverse" if reverse_mode else query.query_id.removeprefix(original.query_id + "_"),
                        "motion_model": "REEDS_SHEPP" if reverse_mode else "DUBIN",
                        "allow_reverse_diagnostic": reverse_mode,
                        "run_mode": run_mode, "repetition": repetition,
                        "start_x": query.start[0], "start_y": query.start[1], "start_yaw": query.start[2],
                        "goal_x": query.goal[0], "goal_y": query.goal[1], "goal_yaw": query.goal[2],
                        "planner_success": result.planner_success,
                        "diagnostic_valid": valid,
                        "failure_code": metrics.get("failure_code") or result.failure_code,
                        "wall_time_ms": (time.monotonic_ns() - started_ns) / 1.0e6,
                        "smac_action_ms": diagnostics.get("l3_action_wall_ms", 0.0),
                        "smac_planning_ms": diagnostics.get("l3_planning_time_ms", 0.0),
                        "ros_path_conversion_ms": diagnostics.get("ros_path_conversion_ms", 0.0),
                        "point_annotation_ms": diagnostics.get("point_annotation_ms", 0.0),
                        "canonical_path_audit_ms": diagnostics.get("canonical_path_audit_ms", 0.0),
                        "static_footprint_valid": metrics.get("static_footprint_valid", False),
                        "kinematic_valid_under_formal_protocol": metrics.get("kinematic_valid", False),
                        "maximum_curvature": metrics.get("maximum_curvature"),
                        "reverse_distance_m": metrics.get("reverse_distance_m", 0.0),
                        "in_place_rotation_count": metrics.get("in_place_rotation_count", 0),
                        "path_hash": path_hash, "path_file": path_file,
                        "costmap_update_mode": diagnostics.get("local_map_update_mode", ""),
                    })
    return rows


def _classification(rows: Sequence[Mapping[str, Any]], corridor_results: Optional[Path]) -> List[Dict[str, Any]]:
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    corridor_failed: Dict[str, bool] = {}
    if corridor_results is not None:
        with (corridor_results / "runs.csv").open(newline="", encoding="utf-8") as stream:
            corridor_rows = list(csv.DictReader(stream))
        for query_id in ("A2B-07", "A2B-16"):
            selected = [row for row in corridor_rows if row.get("run_mode") == "measured" and row.get("query_id") == query_id]
            corridor_failed[query_id] = bool(selected and not any(r2._truth(row.get("final_valid_success")) for row in selected))
    result = []
    for query_id in ("A2B-07", "A2B-16"):
        outcomes = {
            variant: any(r2._truth(row.get("diagnostic_valid")) for row in measured if row.get("base_query_id") == query_id and row.get("variant") == variant)
            for variant in (*VARIANTS, "allow_reverse")
        }
        if outcomes["original"] and corridor_failed.get(query_id):
            classification = "ENDPOINT_MANEUVER_ENVELOPE_OR_CORRIDOR_ATTACHMENT"
        elif not outcomes["original"] and any(outcomes[name] for name in VARIANTS[1:]):
            classification = "SCENARIO_ENDPOINT_ORIENTATION"
        elif not any(outcomes[name] for name in VARIANTS) and outcomes["allow_reverse"]:
            classification = "FORWARD_ONLY_ENDPOINT_ORIENTATION_INFEASIBLE"
        elif not any(outcomes.values()):
            classification = "FULL_MAP_ALL_VARIANTS_FAILED_INVESTIGATE_MAP_OR_SMAC"
        else:
            classification = "MIXED_ENDPOINT_FEASIBILITY"
        result.append({
            "query_id": query_id, **{f"{key}_success": value for key, value in outcomes.items()},
            "corridor_failed": corridor_failed.get(query_id, "not_supplied"),
            "classification": classification,
        })
    return result


def run(
    output: Path, *, topology_cache_dir: Path, corridor_results: Optional[Path],
    warmups: int, repetitions: int, ros_domain_id: int,
) -> Path:
    parent._refuse_nonempty(output)
    output.mkdir(parents=True)
    (output / "paths").mkdir()
    all_queries, _metadata = parent._load_tasks()
    queries = [query for query in all_queries if query.query_id in {"A2B-07", "A2B-16"}]
    ctx = parent._context()
    topology, topology_info = parent._load_or_build_topology(ctx, output, topology_cache_dir.resolve())
    variants: Dict[str, Sequence[Query]] = {}
    tangent_rows = []
    for query in queries:
        timing: Dict[str, Any] = {}
        start, goal, route, reason = candidate._select_route_with_endpoint_attach(
            topology, query, cache_mode=candidate.CACHE_MODE_OPTIMIZED, timing=timing,
        )
        tangents = endpoint_heading._route_tangents(route) if route is not None else None
        if start is None or goal is None or route is None or tangents is None:
            raise RuntimeError(f"{query.query_id}: cannot derive topology tangents: {reason}")
        variants[query.query_id] = [_variant(query, name, tangents[0], tangents[1]) for name in VARIANTS]
        tangent_rows.append({
            "query_id": query.query_id, "start_node_id": int(start.node_id),
            "goal_node_id": int(goal.node_id), "route_start_tangent": tangents[0],
            "route_goal_tangent": tangents[1],
            "start_yaw_difference_deg": abs(candidate.legacy._delta(query.start[2], tangents[0])) * 180.0 / math.pi,
            "goal_yaw_difference_deg": abs(candidate.legacy._delta(query.goal[2], tangents[1])) * 180.0 / math.pi,
        })
    raw_free = candidate._raw_free_mask(ctx)
    auditor = path_audit.PathAuditor(ctx, source_commit=parent.validity._source_commit() or "unknown")
    spec = parent.legacy.backend_availability()["hybrid_astar"]
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    rows: List[Dict[str, Any]] = []
    for reverse_mode, domain_offset in ((False, 0), (True, 1)):
        os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id) + domain_offset)
        session = candidate.SmacSession(
            ctx, output, map_yaml=parent.validity.MAP_YAML,
            log_tag=f"endpoint_{'reverse' if reverse_mode else 'forward'}",
            local_mask_updates=True, optimization_profile="v7_candidate",
            smac_parameter_profile="lighter_smoother", optimization_stage="step3_delta_map",
            enable_mask_reuse_noop=True,
            planner_parameter_overrides={"motion_model_for_search": "REEDS_SHEPP"} if reverse_mode else None,
        )
        session.local_map_update_strategy = "v6_full"
        session.full_grid_settle_cycles = 20
        try:
            session.start()
            rows.extend(_run_session(
                session, queries, variants, auditor, spec, raw_free, output,
                reverse_mode=reverse_mode, warmups=warmups, repetitions=repetitions,
            ))
        finally:
            session.close()
    classifications = _classification(rows, corridor_results)
    r2._write_csv(output / "runs.csv", rows)
    r2._write_csv(output / "path_metrics.csv", rows)
    r2._write_csv(output / "endpoint_tangents.csv", tangent_rows)
    r2._write_csv(output / "classification.csv", classifications)
    source_files, source_hash = r2._source_manifest()
    source_files[str(Path(__file__).resolve())] = parent.sha256_file(Path(__file__).resolve())
    source_hash = r2._json_hash(source_files)
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({
        "source_hash": source_hash, "source_files": source_files,
        "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256,
    }, sort_keys=False), encoding="utf-8")
    (output / "protocol.yaml").write_text(yaml.safe_dump({
        "diagnostic_only": True, "formal_queries_modified": False,
        "query_ids": [query.query_id for query in queries], "variants": [*VARIANTS, "allow_reverse"],
        "full_map": True, "resolution_m": 0.05, "footprint": candidate.FOOTPRINT,
        "minimum_turning_radius_m": 0.4, "maximum_curvature_1pm": 2.5,
        "formal_allow_reverse": False, "reverse_variant_diagnostic_only": True,
        "warmups": warmups, "repetitions": repetitions,
    }, sort_keys=False), encoding="utf-8")
    (output / "manifest.yaml").write_text(yaml.safe_dump({
        "experiment_id": output.name, "source_hash": source_hash,
        "topology_info": topology_info, "classifications": classifications,
        "run_count": len(rows), "corridor_results": str(corridor_results or ""),
    }, sort_keys=False), encoding="utf-8")
    lines = ["# A2B-07 / A2B-16 endpoint diagnostics", "", "| Query | Original | Start aligned | Goal aligned | Both aligned | Reverse diagnostic | Classification |", "|---|---:|---:|---:|---:|---:|---|"]
    for item in classifications:
        lines.append(
            f"| {item['query_id']} | {item['original_success']} | {item['start_yaw_aligned_success']} | "
            f"{item['goal_yaw_aligned_success']} | {item['both_yaw_aligned_success']} | "
            f"{item['allow_reverse_success']} | {item['classification']} |"
        )
    (output / "final_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--topology-cache-dir", default=str(DEFAULT_TOPOLOGY_CACHE))
    parser.add_argument("--corridor-results")
    parser.add_argument("--warmups", type=int, default=1)
    parser.add_argument("--repetitions", type=int, default=2)
    parser.add_argument("--ros-domain-id", type=int, default=90)
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    args = parser.parse_args(argv)
    try:
        result = run(
            Path(args.output_dir).resolve(), topology_cache_dir=Path(args.topology_cache_dir).resolve(),
            corridor_results=Path(args.corridor_results).resolve() if args.corridor_results else None,
            warmups=args.warmups, repetitions=args.repetitions, ros_domain_id=args.ros_domain_id,
        )
    except Exception as exc:
        print(f"two_layer_v1_r2_endpoint_diagnostics: ERROR: {exc}")
        return 2
    print(f"endpoint diagnostics output: {result}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
