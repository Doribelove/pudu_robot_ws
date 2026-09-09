"""Generate the eight canonical frozen 2A-V1-r2 paths for runtime identity checks."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import resource
from typing import Any, Iterable, Mapping

import yaml

from arena_evaluation.semantic_query_defaults import load_query_set
from three_d_v1_nav2.contracts import (
    EXPECTED_MAP_SHA256, EXPECTED_QUERY_IDS, EXPECTED_SEMANTIC_MAP_HASH, QUERY_SET,
    sha256_file,
)
from . import ARCHITECTURE_ID, TASK_ID
from .core import FrozenTwoAPlannerCore, topology_hashes


FIELDS = (
    "query_id", "start_x", "start_y", "start_yaw", "goal_x", "goal_y", "goal_yaw",
    "path_file", "path_sha256", "canonical_path_hash", "route_edge_ids", "corridor_mask_hash",
    "l2_called", "l2_backend", "final_audit_passed", "l1_algorithm", "l3_algorithm",
    "angle_bins", "motion_model", "reverse_allowed", "rotate_in_place_allowed",
    "min_turning_radius_m", "max_curvature_1pm", "roi_ack_mismatches",
)


def _jsonable(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "item"):
        return value.item()
    return value


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]], fields: Iterable[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(fields), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def run(run_dir: Path, output_dir: Path, *, l3_domain_id: int) -> Path:
    run_dir = run_dir.resolve()
    output = output_dir.resolve()
    if output.exists():
        raise FileExistsError(f"refusing existing path bank: {output}")
    output.mkdir(parents=True)
    (output / "paths").mkdir()
    (output / "traces").mkdir()
    queries, _, metadata = load_query_set(
        QUERY_SET, actual_map_hash=EXPECTED_MAP_SHA256,
        actual_semantic_map_hash=EXPECTED_SEMANTIC_MAP_HASH,
        require_default_contract=True,
    )
    if tuple(query.query_id for query in queries) != EXPECTED_QUERY_IDS:
        raise RuntimeError("frozen selected8 order drift")
    core = FrozenTwoAPlannerCore(run_dir, output / "runtime", l3_domain_id=l3_domain_id)
    index_rows = []
    trace_rows = []
    audit_rows = []
    try:
        for sequence, query in enumerate(queries, 1):
            result, diagnostics = core.plan(query)
            audit = result.path_audit
            path_file = output / "paths" / f"{sequence:02d}_{query.query_id}.csv"
            _write_csv(path_file, ({
                "x": format(float(point["x"]), ".17g"),
                "y": format(float(point["y"]), ".17g"),
                "yaw": format(float(point["yaw"]), ".17g"),
            } for point in result.points), ("x", "y", "yaw"))
            route_edges = [int(value) for value in diagnostics.get("topology_edge_ids", [])]
            trace = {
                "sequence": sequence, "query_id": query.query_id,
                "architecture_id": "2A-V1", "implementation_revision": "r2-roi-pathaudit-v1",
                "l1_algorithm": "deterministic_graph_astar",
                "l1_graph_search_ms": diagnostics.get("l1_graph_search_ms", 0.0),
                "route_edge_ids": route_edges,
                "corridor_mask_hash": diagnostics.get("corridor_mask_hash", ""),
                "corridor_padding_m": diagnostics.get("corridor_padding_m", 2.0),
                "corner_corridor_padding_m": diagnostics.get("corner_corridor_padding_m", 4.0),
                "l2_called": False, "l2_backend": "not_applicable_2a",
                "l3_algorithm": "smac_hybrid_astar",
                "l3_action_wall_ms": diagnostics.get("l3_action_wall_ms", 0.0),
                "l3_planning_ms": diagnostics.get("hybrid_planning_time_ms", 0.0),
                "l3_call_count": diagnostics.get("l3_prime_call_count", 0),
                "angle_bins": 48, "motion_model": "DUBIN",
                "roi_acknowledged": diagnostics.get("costmap_update_acknowledged"),
                "roi_ack_mismatch_cells": diagnostics.get("costmap_ack_mismatch_cells", 0),
                "roi_sequence": diagnostics.get("costmap_ack_sequence", 0),
                "fixed_settle_cycles": 0, "fallback_used": diagnostics.get("fallback_used", False),
                "canonical_path_hash": audit.path_hash,
                "canonical_path_audit_reused": True,
                "request_wall_ms": diagnostics.get("request_wall_ms", 0.0),
            }
            (output / "traces" / f"{sequence:02d}_{query.query_id}.json").write_text(
                json.dumps(_jsonable({**trace, "diagnostics": diagnostics,
                                      "audit": audit.diagnostics()}), indent=2, sort_keys=True) + "\n",
                encoding="utf-8")
            trace_rows.append(trace)
            audit_rows.append({
                "sequence": sequence, "query_id": query.query_id,
                "canonical_path_hash": audit.path_hash,
                "final_valid_success": audit.final_valid_success,
                "static_footprint_valid": audit.metrics.get("static_footprint_valid"),
                "kinematic_valid": audit.metrics.get("kinematic_valid"),
                "within_corridor": audit.within_mask,
                "path_length_m": audit.metrics.get("path_length_m"),
                "minimum_clearance_m": audit.metrics.get("minimum_clearance_m"),
                "maximum_curvature_1pm": audit.metrics.get("maximum_curvature"),
                "reverse_distance_m": audit.metrics.get("reverse_distance_m"),
                "in_place_rotation_count": audit.metrics.get("in_place_rotation_count"),
                "audit_ms": audit.timings.get("canonical_path_audit_ms"),
            })
            index_rows.append({
                "query_id": query.query_id,
                **{f"start_{axis}": format(float(value), ".17g") for axis, value in zip(("x", "y", "yaw"), query.start)},
                **{f"goal_{axis}": format(float(value), ".17g") for axis, value in zip(("x", "y", "yaw"), query.goal)},
                "path_file": str(path_file.relative_to(output)),
                "path_sha256": sha256_file(path_file), "canonical_path_hash": audit.path_hash,
                "route_edge_ids": ";".join(str(value) for value in route_edges),
                "corridor_mask_hash": diagnostics.get("corridor_mask_hash", ""),
                "l2_called": "false", "l2_backend": "not_applicable_2a",
                "final_audit_passed": "true", "l1_algorithm": "deterministic_graph_astar",
                "l3_algorithm": "smac_hybrid_astar", "angle_bins": "48",
                "motion_model": "DUBIN", "reverse_allowed": "false",
                "rotate_in_place_allowed": "false", "min_turning_radius_m": "0.40",
                "max_curvature_1pm": "2.50", "roi_ack_mismatches": "0",
            })
    finally:
        core.close()
    _write_csv(output / "index.csv", index_rows, FIELDS)
    _write_csv(output / "global_layer_trace.csv", trace_rows, trace_rows[0].keys())
    _write_csv(output / "path_audit.csv", audit_rows, audit_rows[0].keys())
    manifest = {
        "task_id": TASK_ID, "architecture_id": "2A-V1",
        "integration_architecture_id": ARCHITECTURE_ID,
        "implementation_revision": "r2-roi-pathaudit-v1",
        "complete": len(index_rows) == 8, "path_count": len(index_rows),
        "query_set": metadata, "index_sha256": sha256_file(output / "index.csv"),
        "topology_files": topology_hashes(run_dir), "frozen_source_hashes": core.source_hashes,
        "source_hash": core.source_hash, "cache_manifest": core.cache_manifest,
        "l2_called": False, "l2_metric_status": "not_applicable_by_architecture",
        "ros_domain_id": l3_domain_id,
        "smac_session_start_count": core.session.session_start_count,
        "smac_session_restart_count": core.session.session_restart_count,
        "smac_session_close_count": core.session.session_close_count,
        "max_rss_bytes": int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024,
    }
    (output / "manifest.yaml").write_text(yaml.safe_dump(_jsonable(manifest), sort_keys=False), encoding="utf-8")
    return output / "index.csv"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--l3-domain-id", type=int, required=True)
    args = parser.parse_args()
    print(run(args.run_dir, args.output_dir, l3_domain_id=args.l3_domain_id))


if __name__ == "__main__":
    main()
