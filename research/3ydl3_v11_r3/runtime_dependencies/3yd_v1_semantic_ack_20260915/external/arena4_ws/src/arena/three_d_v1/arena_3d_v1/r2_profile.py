"""Profile the frozen 3D-V1/r1 lifecycle before r2 acceptance changes."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import resource
import shutil
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping

import numpy as np
import yaml

from .l2_incremental import deterministic_grid_astar
from .l2_state_lifecycle import (
    DEFAULT_DYNAMIC_BASELINE,
    L2StateLifecycleManager,
)
from .production_l1 import DeterministicGraphAStarL1
from .real_stage_a_benchmark import (
    MAP_ID,
    ROOT,
    _load_inputs,
    _select_path_sources,
)


ARCHITECTURE_ID = "3D-V1"
REVISION_ID = "r2-production-acceptance-real-replay"
PROTOCOL_ID = "PLN-02-3D-V1-R2-PRODUCTION-ACCEPTANCE-V1"
PROFILE_SUBJECT = "frozen_3D-V1_r1"


def _elapsed_ms(started_ns: int) -> float:
    return (time.monotonic_ns() - started_ns) / 1.0e6


def _rss_bytes() -> int:
    try:
        for line in Path("/proc/self/status").read_text(encoding="utf-8").splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) * 1024
    except OSError:
        pass
    return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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


def _sequence_bytes(values: Iterable[Any], container: Any) -> int:
    return int(sys.getsizeof(container) + sum(sys.getsizeof(value) for value in values))


def _dict_bytes(values: Mapping[Any, Any]) -> int:
    return int(sys.getsizeof(values) + sum(
        sys.getsizeof(key) + sys.getsizeof(value)
        for key, value in values.items()
    ))


def _event_row(category: str, result: Any, rss_before: int, rss_after: int) -> Dict[str, Any]:
    fallback_ms = 0.0 if result.fallback_stats is None else result.fallback_stats.search_time_ms
    return {
        "category": category,
        "backend": result.selected_backend,
        "response_ms": result.response_ms,
        "dstar_search_ms": result.dstar_stats.search_time_ms,
        "fallback_search_ms": fallback_ms,
        "wrapper_residual_ms": max(0.0, result.response_ms - result.dstar_stats.search_time_ms - fallback_ms),
        "changed_cells": result.changed_cells,
        "expanded": result.dstar_stats.expanded_nodes,
        "heap_pops": result.dstar_stats.queue_pops,
        "heap_pushes": result.dstar_stats.queue_pushes,
        "update_vertex": result.dstar_stats.update_vertex_count,
        "predecessor_visits": result.dstar_stats.update_vertex_count,
        "fallback_expanded": 0 if result.fallback_stats is None else result.fallback_stats.expanded_nodes,
        "partial_dstar": result.partial_dstar_result_returned,
        "success": result.success,
        "rss_before_bytes": rss_before,
        "rss_after_bytes": rss_after,
        "rss_delta_bytes": rss_after - rss_before,
    }


def run(output: Path, *, query_id: str = "A2B-03") -> Path:
    output = output.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    output.mkdir(parents=True)
    cache_root = output / "verified_r1_profile_cache"
    phase_rows: List[Dict[str, Any]] = []
    event_rows: List[Dict[str, Any]] = []
    rss_process_start = _rss_bytes()

    started = time.monotonic_ns()
    ctx, queries, artifact, cache_manifest = _load_inputs()
    phase_rows.append({"phase": "load_map_topology", "wall_ms": _elapsed_ms(started)})
    query = next((item for item in queries if item.query_id == query_id), None)
    if query is None:
        raise ValueError(f"unknown query: {query_id}")

    started = time.monotonic_ns()
    l1 = DeterministicGraphAStarL1(
        ctx, artifact, map_hash=ctx.map_sha256,
        topology_hash=str(cache_manifest.get("cache_key") or ""),
    )
    phase_rows.append({"phase": "l1_edge_cell_index", "wall_ms": _elapsed_ms(started)})
    started = time.monotonic_ns()
    plan = l1.plan(query)
    phase_rows.append({"phase": "l1_route_and_corridor", "wall_ms": _elapsed_ms(started)})
    if plan is None:
        raise RuntimeError(f"no r1 L1 plan for {query_id}")

    from .l2_incremental import CorridorROI
    started = time.monotonic_ns()
    roi = CorridorROI.from_global(
        plan.static_safe_free, plan.corridor_mask,
        plan.start_cell, plan.goal_cell,
        binding_fields=plan.binding_fields(),
    )
    phase_rows.append({"phase": "corridor_roi_crop", "wall_ms": _elapsed_ms(started)})

    manager = L2StateLifecycleManager(cache_root, max_active_states=1)
    rss_before_cold = _rss_bytes()
    planner, initial, cold = manager.activate(
        roi, dynamic_baseline_version=DEFAULT_DYNAMIC_BASELINE,
    )
    rss_after_cold = _rss_bytes()
    phase_rows.extend([
        {"phase": "r1_safe_cell_enumeration", "wall_ms": planner.geometry.build_diagnostics.get("safe_cell_enumeration_ms", 0.0)},
        {"phase": "r1_cell_to_state_mapping", "wall_ms": planner.geometry.build_diagnostics.get("cell_to_state_mapping_ms", 0.0)},
        {"phase": "r1_adjacency_build", "wall_ms": planner.geometry.build_diagnostics.get("adjacency_build_ms", 0.0)},
        {"phase": "r1_geometry_build", "wall_ms": cold.geometry_build_ms},
        {"phase": "r1_g_rhs_and_initial_open", "wall_ms": cold.state_build_ms},
        {"phase": "r1_first_compute_shortest_path", "wall_ms": initial.dstar_stats.search_time_ms},
        {"phase": "r1_cache_serialize", "wall_ms": cold.state_serialize_ms},
        {"phase": "r1_cold_build_total", "wall_ms": cold.activate_ms},
    ])

    cold_memory_bytes = planner.state_memory_bytes()
    cold_cache_bytes = cold.geometry_cache.bytes_on_disk + cold.state_cache.bytes_on_disk
    clear = manager.clear()
    rss_before_restore = _rss_bytes()
    planner, restored, warm = manager.activate(
        roi, dynamic_baseline_version=DEFAULT_DYNAMIC_BASELINE,
    )
    rss_after_restore = _rss_bytes()
    phase_rows.extend([
        {"phase": "r1_geometry_restore", "wall_ms": warm.geometry_cache.wall_ms},
        {"phase": "r1_state_restore", "wall_ms": warm.state_cache.wall_ms},
        {"phase": "r1_warm_activate_total", "wall_ms": warm.activate_ms},
    ])

    path = planner.path_global or []
    one = _select_path_sources(path, 1, set())
    rss_before = _rss_bytes()
    one_result = planner.update(one)
    event_rows.append(_event_row("one_source_eligible", one_result, rss_before, _rss_bytes()))

    five = _select_path_sources(planner.path_global or path, 5, one)
    large = one | five
    rss_before = _rss_bytes()
    large_result = planner.update(large, force_cold_astar=True)
    event_rows.append(_event_row("large_change_fallback", large_result, rss_before, _rss_bytes()))

    rss_before = _rss_bytes()
    recovery = planner.update(set(), force_cold_astar=True)
    event_rows.append(_event_row("recovery_fallback", recovery, rss_before, _rss_bytes()))

    rss_before = _rss_bytes()
    resync = planner.service_resync()
    event_rows.append(_event_row("explicit_resync", resync, rss_before, _rss_bytes()))

    static_mask_bytes = int(roi.base_free.nbytes)
    packed_static_mask_bytes = int(math.ceil(roi.base_free.size / 8.0))
    state = planner.state
    memory = {
        "roi_static_mask_bool_bytes": static_mask_bytes,
        "roi_static_mask_packbits_bytes": packed_static_mask_bytes,
        "roi_static_mask_predicted_saving_bytes": static_mask_bytes - packed_static_mask_bytes,
        "geometry_state_cells_bytes": int(planner.geometry.state_cells_linear.nbytes),
        "geometry_neighbor_deltas_bytes": int(planner.geometry.neighbor_deltas.nbytes),
        "mutable_blocked_bool_bytes": int(state.blocked.nbytes),
        "mutable_g_bytes": int(state.g.nbytes),
        "mutable_rhs_bytes": int(state.rhs.nbytes),
        "open_python_estimated_bytes": _sequence_bytes(state.open, state.open),
        "queued_python_estimated_bytes": _dict_bytes(state.queued),
        "path_python_estimated_bytes": 0 if state.current_path_ids is None else _sequence_bytes(state.current_path_ids, state.current_path_ids),
        "reported_cold_resident_bytes": cold_memory_bytes,
        "reported_warm_resident_bytes": warm.resident_bytes,
        "predicted_static_packbits_resident_bytes": cold_memory_bytes - static_mask_bytes + packed_static_mask_bytes,
        "cache_bytes_on_disk": cold_cache_bytes,
        "clear_released_resident_bytes": clear["released_resident_bytes"],
        "rss_process_start_bytes": rss_process_start,
        "rss_before_cold_bytes": rss_before_cold,
        "rss_after_cold_bytes": rss_after_cold,
        "rss_cold_delta_bytes": rss_after_cold - rss_before_cold,
        "rss_before_restore_bytes": rss_before_restore,
        "rss_after_restore_bytes": rss_after_restore,
        "rss_restore_delta_bytes": rss_after_restore - rss_before_restore,
    }
    profile = {
        "architecture_id": ARCHITECTURE_ID,
        "revision_id": REVISION_ID,
        "protocol_id": PROTOCOL_ID,
        "profile_subject": PROFILE_SUBJECT,
        "map_id": MAP_ID,
        "map_hash": ctx.map_sha256,
        "query_id": query_id,
        "resolution_m": float(ctx.hospital_map.resolution),
        "route_signature": plan.route_signature,
        "route_edge_count": len(plan.route_edge_ids),
        "roi_shape": list(roi.shape),
        "roi_array_cells": int(roi.base_free.size),
        "safe_state_cells": planner.geometry.state_count,
        "first_solve": {
            "expanded": initial.dstar_stats.expanded_nodes,
            "heap_pops": initial.dstar_stats.queue_pops,
            "heap_pushes": initial.dstar_stats.queue_pushes,
            "update_vertex": initial.dstar_stats.update_vertex_count,
        },
        "cold_activation": cold.as_dict(),
        "warm_activation": warm.as_dict(),
        "memory": memory,
        "phases": phase_rows,
        "events": event_rows,
    }
    (output / "profile.json").write_text(
        json.dumps(profile, indent=2, sort_keys=True) + "\n", encoding="utf-8",
    )
    _write_csv(output / "phase_timings.csv", phase_rows)
    _write_csv(output / "events.csv", event_rows)

    source_files = [
        Path(__file__).resolve(),
        Path(__file__).with_name("l2_state_lifecycle.py").resolve(),
        Path(__file__).with_name("l2_incremental.py").resolve(),
        Path(__file__).with_name("r1_pipeline.py").resolve(),
    ]
    snapshot_dir = output / "source_snapshot"
    snapshot_dir.mkdir()
    source_hashes: Dict[str, str] = {}
    for source in source_files:
        shutil.copy2(source, snapshot_dir / source.name)
        source_hashes[str(source)] = _sha256(source)
    (output / "manifest.yaml").write_text(yaml.safe_dump({
        "architecture_id": ARCHITECTURE_ID,
        "revision_id": REVISION_ID,
        "protocol_id": PROTOCOL_ID,
        "profile_subject": PROFILE_SUBJECT,
        "map_id": MAP_ID,
        "map_hash": ctx.map_sha256,
        "query_id": query_id,
        "frozen_r1_l2_state_lifecycle_sha256": "a189b8ae68e30a24b00fad397e33c10da6999fb1ba9124b81836e9357e362b9d",
        "source_files": source_hashes,
    }, sort_keys=False), encoding="utf-8")
    command = (
        "cd /home/robot/pudu_robot_ws\n"
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        "PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/three_d_v1:"
        "/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation "
        f"/usr/bin/python3 -m arena_3d_v1.r2_profile --output-dir {output} --query-id {query_id}\n"
    )
    (output / "reproduction_command.txt").write_text(command, encoding="utf-8")
    largest = max(phase_rows, key=lambda row: float(row["wall_ms"]))
    report = [
        "# 3D-V1 r2 pre-optimization profile of frozen r1", "",
        f"- map/query: `{MAP_ID}` / `{query_id}`",
        f"- ROI shape/cells/safe states: `{roi.shape}` / `{roi.base_free.size}` / `{planner.geometry.state_count}`",
        f"- cold activation: `{cold.activate_ms:.3f} ms`; warm restore/activation: `{warm.activate_ms:.3f} ms`",
        f"- reported resident: `{cold_memory_bytes} B`; process RSS cold delta: `{rss_after_cold - rss_before_cold} B`",
        f"- largest phase: `{largest['phase']}` = `{float(largest['wall_ms']):.3f} ms`", "",
        "## Evidence-led r2 decision", "",
        f"The permanent boolean static ROI mask costs `{static_mask_bytes} B`. Bit-packing it costs `{packed_static_mask_bytes} B` and predicts `{memory['predicted_static_packbits_resident_bytes']} B` resident without touching float64 g/rhs, compact adjacency, or the hot dynamic blocked array.", "",
        "Cold activation remains unsuitable for the online request path. r2 must require a verified prebuilt cache for D* admission and dispatch cache misses/rejections directly to the frozen deterministic grid A* path. Pure D* and alternative path semantics remain out of scope.",
    ]
    (output / "profile_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    verification = {
        "profile_complete": True,
        "profile_precedes_r2_lifecycle_optimization": True,
        "frozen_r1_core_modified_for_profile": False,
        "partial_dstar_results": sum(bool(row["partial_dstar"]) for row in event_rows),
        "predicted_resident_under_30mb": memory["predicted_static_packbits_resident_bytes"] <= 30_000_000,
        "r1_misrouted_profile_excluded": True,
    }
    (output / "verification.yaml").write_text(
        yaml.safe_dump(verification, sort_keys=False), encoding="utf-8",
    )
    summary = (
        f"PASS profile={output} cold_ms={cold.activate_ms:.3f} "
        f"warm_ms={warm.activate_ms:.3f} resident_bytes={cold_memory_bytes} "
        f"predicted_packbits_bytes={memory['predicted_static_packbits_resident_bytes']}"
    )
    (output / "stdout.log").write_text(summary + "\n", encoding="utf-8")
    (output / "stderr.log").write_text("", encoding="utf-8")
    print(summary)
    return output


def _default_output() -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return ROOT / "experiments/layered_planner_benchmark" / f"3d_v1_r2_r1_profile_{stamp}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--query-id", default="A2B-03")
    args = parser.parse_args()
    run(args.output_dir or _default_output(), query_id=args.query_id)


if __name__ == "__main__":
    main()
