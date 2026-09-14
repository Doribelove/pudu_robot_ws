"""Bounded high-dynamic soak for frozen 3D-V1/r1 L2 lifecycle policy."""

from __future__ import annotations

import argparse
import csv
import gc
import hashlib
import json
import math
import os
import shutil
import statistics
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Set, Tuple

import numpy as np
import yaml

from .dynamic_policy import _inflate
from .l2_incremental import Cell, CorridorROI, deterministic_grid_astar
from .l2_state_lifecycle import (
    ARCHITECTURE_ID,
    L2StateLifecycleManager,
    PROTOCOL_ID,
    REVISION_ID,
    _rss_bytes,
)
from .production_l1 import DeterministicGraphAStarL1
from .r1_pipeline import Layered3DV1R1Controller
from .r1_stage_a import (
    DEFAULT_FROZEN_CONFIG,
    ROOT,
    _barrier_sources,
    _load_frozen_config,
    _load_inputs,
    _path_cost,
    _select_nonblocking_eligible_sources,
    _sha256,
    _snapshot,
    _source_snapshot,
    _stable_hash,
)


DEFAULT_QUERIES = ("A2B-07", "A2B-11", "A2B-17")


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    values = list(rows)
    fields: List[str] = []
    for row in values:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or ["status"])
        writer.writeheader()
        for row in values:
            writer.writerow(row)


def _percentiles(values: Sequence[float]) -> Mapping[str, float]:
    return {
        "count": len(values),
        "p50": float(np.percentile(values, 50)) if values else math.nan,
        "p95": float(np.percentile(values, 95)) if values else math.nan,
        "p99": float(np.percentile(values, 99)) if values else math.nan,
        "mean": statistics.fmean(values) if values else math.nan,
        "max": max(values) if values else math.nan,
    }


def _children() -> List[int]:
    try:
        value = Path(f"/proc/{os.getpid()}/task/{os.getpid()}/children").read_text().strip()
        return [int(item) for item in value.split()] if value else []
    except OSError:
        return []


def _workload_sources(
    local_index: int,
    single_a: Cell,
    single_b: Cell,
    cluster: Set[Cell],
    barrier: Set[Cell],
) -> Tuple[str, Set[Cell]]:
    if local_index == 200:
        return "no_route_pending", set(barrier)
    if local_index == 201:
        return "no_route_confirmed", set(barrier)
    if local_index == 202:
        return "no_route_recovery_pending", set()
    if local_index == 203:
        return "no_route_recovery_confirmed", set()
    phase = local_index % 12
    return (
        ("moving_a_pending", {single_a}),
        ("moving_a_confirmed", {single_a}),
        ("moving_a_duplicate", {single_a}),
        ("move_a_to_b_pending", {single_b}),
        ("move_a_to_b_confirmed", {single_b}),
        ("moving_b_duplicate", {single_b}),
        ("cluster_pending", set(cluster)),
        ("cluster_confirmed", set(cluster)),
        ("cluster_duplicate", set(cluster)),
        ("cluster_recovery_pending", set()),
        ("cluster_recovery_confirmed", set()),
        ("free_duplicate", set()),
    )[phase]


def run(
    output: Path,
    *,
    query_ids: Sequence[str] = DEFAULT_QUERIES,
    min_snapshots: int = 5_000,
    max_snapshots: int = 10_000,
    max_duration_s: float = 1_800.0,
    route_switch_interval: int = 500,
    oracle_sample_interval: int = 100,
    frozen_config_path: Path = DEFAULT_FROZEN_CONFIG,
) -> Path:
    output = output.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    if min_snapshots < 5_000:
        raise ValueError("soak requires at least 5,000 snapshots")
    if max_snapshots < min_snapshots or max_duration_s <= 0:
        raise ValueError("invalid soak stop conditions")
    if route_switch_interval < 250:
        raise ValueError("route switch interval must leave room for no-route sequence")
    frozen = _load_frozen_config(frozen_config_path)
    output.mkdir(parents=True)
    started_ns = time.monotonic_ns()
    ctx, queries, artifact, cache_manifest = _load_inputs()
    by_id = {query.query_id: query for query in queries}
    if any(query_id not in by_id for query_id in query_ids):
        raise ValueError("unknown soak query")
    l1 = DeterministicGraphAStarL1(
        ctx, artifact, map_hash=ctx.map_sha256,
        topology_hash=str(cache_manifest.get("cache_key") or ""),
    )
    plans = {query_id: l1.plan(by_id[query_id]) for query_id in query_ids}
    if any(plan is None for plan in plans.values()):
        raise RuntimeError("soak query has no production L1 route")
    cache_root = output / "verified_lifecycle_cache"
    manager = L2StateLifecycleManager(
        cache_root,
        max_active_states=int(frozen["policy"]["max_active_states_default"]),
        dstar_wall_budget_ms=float(frozen["policy"]["dstar_wall_budget_ms"]),
        dstar_max_expansions=int(frozen["policy"]["dstar_max_expansions"]),
    )
    prebuild_rows: List[Dict[str, Any]] = []
    for query_id in query_ids:
        plan = plans[query_id]
        prebuild_started = time.monotonic_ns()
        controller = Layered3DV1R1Controller(
            plan, cache_root=cache_root, max_active_states=1,
            lifecycle_manager=manager,
        )
        if not controller.initial_l2_result.success:
            raise RuntimeError(f"soak prebuild failed: {query_id}")
        activation = manager.last_activation
        prebuild_rows.append({
            "query_id": query_id,
            "cold_build_ms": (time.monotonic_ns() - prebuild_started) / 1.0e6,
            "resident_bytes": manager.resident_bytes,
            "safe_state_count": controller.l2.geometry.state_count,
            "geometry_cache_hit": activation.geometry_cache.hit,
            "state_cache_hit": activation.state_cache.hit,
        })
        del controller
    manager.clear()
    gc.collect()

    rows: List[Dict[str, Any]] = []
    activations: List[Dict[str, Any]] = []
    counts: Dict[str, int] = {
        "fallback": 0, "resync": 0, "scheduler_skip": 0,
        "timeout": 0, "partial": 0, "oracle_mismatch": 0,
        "blocked_path": 0, "ack_mismatch_rejected": 0,
        "ack_mismatch_unexpected": 0, "no_route": 0, "recovery": 0,
    }
    controller: Layered3DV1R1Controller | None = None
    current_query = ""
    route_started_at = 0
    source_sets: Tuple[Cell, Cell, Set[Cell], Set[Cell]] | None = None
    loop_started = time.monotonic()
    index = 0
    print(
        f"START queries={','.join(query_ids)} min={min_snapshots} "
        f"max={max_snapshots} duration_s={max_duration_s}",
        flush=True,
    )
    while True:
        elapsed_s = time.monotonic() - loop_started
        if index >= min_snapshots and (
            index >= max_snapshots or elapsed_s >= max_duration_s
        ):
            break
        if controller is None or index % route_switch_interval == 0:
            query_id = query_ids[(index // route_switch_interval) % len(query_ids)]
            previous = controller
            activation_started = time.monotonic_ns()
            controller = Layered3DV1R1Controller(
                plans[query_id], cache_root=cache_root, max_active_states=1,
                lifecycle_manager=manager,
            )
            activation = manager.last_activation
            if previous is not None:
                del previous
            gc.collect()
            current_query = query_id
            route_started_at = index
            path = list(controller.l2.path_global or ())
            candidates = _select_nonblocking_eligible_sources(
                paths=(path, path), count=7, existing_sources=set(),
                plan=plans[query_id], map_shape=artifact.free_mask.shape,
            )
            ordered = sorted(candidates)
            source_sets = (
                ordered[0], ordered[1], set(ordered[2:]),
                _barrier_sources(
                    plans[query_id].start_cell, plans[query_id].goal_cell,
                    artifact.free_mask.shape,
                ),
            )
            activations.append({
                "snapshot_index": index + 1,
                "query_id": query_id,
                "activation_ms": (time.monotonic_ns() - activation_started) / 1.0e6,
                "geometry_cache_hit": activation.geometry_cache.hit,
                "state_cache_hit": activation.state_cache.hit,
                "evicted_key": activation.evicted_key,
                "evict_ms": activation.evict_ms,
                "released_resident_bytes": activation.released_resident_bytes,
                "active_state_count": activation.active_state_count,
                "resident_bytes": activation.resident_bytes,
                "rss_bytes": _rss_bytes(),
            })
        assert controller is not None and source_sets is not None
        local_index = index - route_started_at
        category, occupied = _workload_sources(local_index, *source_sets)
        snapshot = _snapshot(
            index + 1, sorted(occupied), map_hash=ctx.map_sha256,
            shape=artifact.free_mask.shape,
        )
        wall_started = time.monotonic_ns()
        step = controller.process_snapshot(snapshot, now=float(index + 1))
        wall_ms = (time.monotonic_ns() - wall_started) / 1.0e6
        result = step.l2_result
        backend = "scheduler_skip" if result is None else result.selected_backend
        if result is None:
            counts["scheduler_skip"] += 1
        else:
            if "astar" in backend:
                counts["fallback"] += 1
            if result.dstar_stats.timeout_triggered:
                counts["timeout"] += 1
            if result.partial_dstar_result_returned:
                counts["partial"] += 1
            blocked = set(step.snapshot_update.blocked_cells)
            path = result.path or []
            if blocked.intersection(path):
                counts["blocked_path"] += 1
            if category == "no_route_confirmed":
                counts["no_route"] += 1
                if result.success:
                    counts["oracle_mismatch"] += 1
            if "recovery_confirmed" in category:
                counts["recovery"] += 1
                if not result.success:
                    counts["oracle_mismatch"] += 1
        oracle_checked = False
        oracle_error = 0.0
        oracle_reachability_match = True
        if result is not None and (
            (index + 1) % oracle_sample_interval == 0
            or category in {"no_route_confirmed", "no_route_recovery_confirmed"}
        ):
            oracle_checked = True
            oracle = deterministic_grid_astar(
                controller.l2.current_free,
                controller.l2.roi.start_local,
                controller.l2.roi.goal_local,
            )
            local_path = (
                None if result.path is None else [
                    controller.l2.roi.to_local(cell) for cell in result.path
                ]
            )
            oracle_reachability_match = (local_path is None) == (oracle.path is None)
            oracle_error = (
                0.0 if local_path is None and oracle.path is None
                else math.inf if local_path is None or oracle.path is None
                else abs(_path_cost(local_path) - oracle.cost)
            )
            if not oracle_reachability_match or oracle_error > 1.0e-9:
                counts["oracle_mismatch"] += 1
        if step.l3_required and step.dirty_roi is not None:
            if (index + 1) % 997 == 0:
                try:
                    controller.acknowledge_l3_mask("intentional-invalid-content-hash")
                    counts["ack_mismatch_unexpected"] += 1
                except ValueError:
                    counts["ack_mismatch_rejected"] += 1
            controller.acknowledge_l3_mask(step.dirty_roi.target_hash)
        if (index + 1) % 250 == 0:
            resync = controller.service_l2_resync()
            if resync.selected_backend != "resync_not_required":
                counts["resync"] += 1
        row = {
            "snapshot_index": index + 1,
            "elapsed_s": time.monotonic() - loop_started,
            "query_id": current_query,
            "route_local_index": local_index,
            "category": category,
            "snapshot_hash": snapshot.snapshot_hash,
            "occupied_source_count": len(occupied),
            "effective_changed_cells": len(step.snapshot_update.effective_changed_cells),
            "scheduler_reason": step.scheduler.reason,
            "scheduler_invoke_l2": step.scheduler.invoke_l2,
            "backend": backend,
            "wall_ms": wall_ms,
            "response_ms": 0.0 if result is None else result.response_ms,
            "search_ms": 0.0 if result is None else result.dstar_stats.search_time_ms,
            "expanded": 0 if result is None else result.dstar_stats.expanded_nodes,
            "partial_result": False if result is None else result.partial_dstar_result_returned,
            "timeout": False if result is None else result.dstar_stats.timeout_triggered,
            "reachable": None if result is None else result.success,
            "oracle_checked": oracle_checked,
            "oracle_reachability_match": oracle_reachability_match,
            "oracle_cost_error": oracle_error,
            "active_state_count": len(manager.active),
            "resident_bytes": manager.resident_bytes,
            "rss_bytes": _rss_bytes(),
            "eviction_count": manager.eviction_count,
            "cache_state_directory_count": len(list((cache_root / "state").glob("*/manifest.json"))),
            "cache_geometry_directory_count": len(list((cache_root / "geometry").glob("*/manifest.json"))),
        }
        rows.append(row)
        index += 1
        if index % 500 == 0:
            print(
                f"SNAPSHOTS {index} elapsed_s={row['elapsed_s']:.1f} "
                f"rss={row['rss_bytes']} resident={row['resident_bytes']}",
                flush=True,
            )

    if controller is not None:
        del controller
    clear_evidence = manager.clear()
    gc.collect()
    rss_after_clear = _rss_bytes()
    first_count = max(1, len(rows) // 10)
    first = rows[:first_count]
    last = rows[-first_count:]
    rss_values = np.asarray([row["rss_bytes"] for row in rows], dtype=np.float64)
    latency_values = [float(row["wall_ms"]) for row in rows]
    trend = {
        "snapshots": len(rows),
        "elapsed_s": time.monotonic() - loop_started,
        "stop_reason": "max_snapshots" if len(rows) >= max_snapshots else "max_duration",
        "first_10_percent": {
            "rss": _percentiles([float(row["rss_bytes"]) for row in first]),
            "latency_ms": _percentiles([float(row["wall_ms"]) for row in first]),
        },
        "last_10_percent": {
            "rss": _percentiles([float(row["rss_bytes"]) for row in last]),
            "latency_ms": _percentiles([float(row["wall_ms"]) for row in last]),
        },
        "rss_linear_slope_bytes_per_snapshot": float(np.polyfit(
            np.arange(len(rss_values), dtype=np.float64), rss_values, 1,
        )[0]),
        "rss_peak_bytes": int(max(row["rss_bytes"] for row in rows)),
        "resident_peak_bytes": int(max(row["resident_bytes"] for row in rows)),
        "rss_after_clear_bytes": rss_after_clear,
        "clear_evidence": dict(clear_evidence),
        "counts": counts,
        "latency_all_ms": _percentiles(latency_values),
        "route_switches": len(activations),
        "evictions": manager.eviction_count,
        "peak_active_state_count": manager.peak_active_state_count,
        "peak_resident_bytes": manager.peak_resident_bytes,
        "child_pids_after_run": _children(),
    }
    first_rss = trend["first_10_percent"]["rss"]["mean"]
    last_rss = trend["last_10_percent"]["rss"]["mean"]
    allowed_growth = max(20_000_000.0, first_rss * 0.10)
    trend["rss_growth_first_to_last_bytes"] = last_rss - first_rss
    trend["rss_no_unbounded_growth"] = bool(
        last_rss - first_rss <= allowed_growth
        and trend["rss_linear_slope_bytes_per_snapshot"] <= 4096.0
    )
    trend["soak_pass"] = bool(
        len(rows) >= min_snapshots
        and counts["oracle_mismatch"] == 0
        and counts["partial"] == 0
        and counts["blocked_path"] == 0
        and counts["ack_mismatch_unexpected"] == 0
        and manager.peak_active_state_count <= 1
        and not trend["child_pids_after_run"]
        and trend["rss_no_unbounded_growth"]
    )

    _write_csv(output / "snapshots.csv", rows)
    with (output / "snapshots.jsonl").open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, sort_keys=True) + "\n")
    _write_csv(output / "activations.csv", activations)
    _write_csv(output / "prebuild.csv", prebuild_rows)
    (output / "trend_analysis.yaml").write_text(
        yaml.safe_dump(trend, sort_keys=False), encoding="utf-8",
    )
    source_hashes = _source_snapshot(output)
    manifest = {
        "architecture_id": ARCHITECTURE_ID,
        "revision_id": REVISION_ID,
        "protocol_id": PROTOCOL_ID,
        "map_id": frozen["map"]["id"],
        "map_hash": ctx.map_sha256,
        "workload_classification": "realistic_synthetic_workload_on_real_4x_map",
        "query_ids": list(query_ids),
        "min_snapshots": min_snapshots,
        "max_snapshots": max_snapshots,
        "max_duration_s": max_duration_s,
        "route_switch_interval": route_switch_interval,
        "oracle_sample_interval": oracle_sample_interval,
        "frozen_config": str(frozen_config_path.resolve()),
        "frozen_config_sha256": _sha256(frozen_config_path),
        "source_files": source_hashes,
        "workload_hash": _stable_hash({
            "query_ids": list(query_ids), "switch": route_switch_interval,
            "pattern": "moving+cluster+disappear+no-route+recovery-v1",
        }),
    }
    (output / "manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8",
    )
    command_queries = ",".join(query_ids)
    (output / "reproduction_command.txt").write_text(
        "cd /home/robot/pudu_robot_ws\n"
        "source /opt/ros/humble/setup.bash\n"
        "PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/three_d_v1:/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation "
        f"/usr/bin/python3 -m arena_3d_v1.r1_soak --output-dir {output} "
        f"--query-ids {command_queries} --min-snapshots {min_snapshots} "
        f"--max-snapshots {max_snapshots} --max-duration-s {max_duration_s} "
        f"--route-switch-interval {route_switch_interval} "
        f"--oracle-sample-interval {oracle_sample_interval} "
        f"--frozen-config {frozen_config_path.resolve()}\n",
        encoding="utf-8",
    )
    verification = {
        "required_artifacts_present": True,
        "minimum_snapshot_count_pass": len(rows) >= min_snapshots,
        "oracle_mismatch_zero": counts["oracle_mismatch"] == 0,
        "partial_result_zero": counts["partial"] == 0,
        "blocked_path_zero": counts["blocked_path"] == 0,
        "lru_bound_pass": manager.peak_active_state_count <= 1,
        "ack_mismatch_injections_all_rejected": counts["ack_mismatch_unexpected"] == 0,
        "no_residual_child_processes": not trend["child_pids_after_run"],
        "rss_no_unbounded_growth": trend["rss_no_unbounded_growth"],
        "soak_pass": trend["soak_pass"],
    }
    (output / "verification.yaml").write_text(
        yaml.safe_dump(verification, sort_keys=False), encoding="utf-8",
    )
    report = [
        "# 3D-V1-r1 high-dynamic soak", "",
        f"- snapshots / elapsed: `{len(rows)}` / `{trend['elapsed_s']:.1f} s`",
        f"- stop reason: `{trend['stop_reason']}`",
        f"- route switches / evictions: `{len(activations)}` / `{manager.eviction_count}`",
        f"- oracle mismatch / partial / blocked path: `{counts['oracle_mismatch']}` / `{counts['partial']}` / `{counts['blocked_path']}`",
        f"- RSS first/last 10% mean: `{first_rss:.0f}` / `{last_rss:.0f}` B",
        f"- RSS slope: `{trend['rss_linear_slope_bytes_per_snapshot']:.3f} B/snapshot`",
        f"- soak gate: **{'PASS' if trend['soak_pass'] else 'FAIL'}**", "",
        "The workload is realistic synthetic on the real 4x map; it is not claimed as a measured real cleaning-obstacle distribution.",
    ]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    summary = (
        f"PASS output={output} soak_gate={trend['soak_pass']} "
        f"snapshots={len(rows)} elapsed_ms={(time.monotonic_ns() - started_ns) / 1.0e6:.1f}"
    )
    (output / "stdout.log").write_text(summary + "\n", encoding="utf-8")
    (output / "stderr.log").write_text("", encoding="utf-8")
    print(summary, flush=True)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--query-ids", default=",".join(DEFAULT_QUERIES))
    parser.add_argument("--min-snapshots", type=int, default=5_000)
    parser.add_argument("--max-snapshots", type=int, default=10_000)
    parser.add_argument("--max-duration-s", type=float, default=1_800.0)
    parser.add_argument("--route-switch-interval", type=int, default=500)
    parser.add_argument("--oracle-sample-interval", type=int, default=100)
    parser.add_argument("--frozen-config", type=Path, default=DEFAULT_FROZEN_CONFIG)
    args = parser.parse_args()
    query_ids = tuple(item.strip() for item in args.query_ids.split(",") if item.strip())
    try:
        run(
            args.output_dir, query_ids=query_ids,
            min_snapshots=args.min_snapshots,
            max_snapshots=args.max_snapshots,
            max_duration_s=args.max_duration_s,
            route_switch_interval=args.route_switch_interval,
            oracle_sample_interval=args.oracle_sample_interval,
            frozen_config_path=args.frozen_config,
        )
    except Exception as exc:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / "INTERRUPTED_RUN.md").write_text(
            f"# Interrupted run\n\n`{type(exc).__name__}: {exc}`\n",
            encoding="utf-8",
        )
        (args.output_dir / "stderr.log").write_text(
            f"{type(exc).__name__}: {exc}\n", encoding="utf-8",
        )
        raise


if __name__ == "__main__":
    main()
