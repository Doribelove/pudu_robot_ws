"""Offline prebuild and verification CLI for 3D-V1/r2 L2 route caches."""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path
from typing import Any, Sequence

import yaml

from .l2_incremental import CorridorROI
from .production_l1 import DeterministicGraphAStarL1
from .r1_stage_a import _sha256, _source_snapshot
from .r2_stage_a import HELDOUT_QUERIES
from .r2_state_lifecycle import (
    ARCHITECTURE_ID,
    PROTOCOL_ID,
    REVISION_ID,
    R2L2StateLifecycleManager,
)
from .real_stage_a_benchmark import MAP_ID, _load_inputs


def run(
    output: Path,
    *,
    query_ids: Sequence[str],
    cache_root: Path,
    dstar_budget_ms: float = 500.0,
    max_active_states: int = 1,
) -> Path:
    output = output.resolve()
    cache_root = cache_root.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    output.mkdir(parents=True)
    ctx, queries, artifact, topology_manifest = _load_inputs()
    by_id = {query.query_id: query for query in queries}
    unknown = sorted(set(query_ids) - set(by_id))
    if unknown:
        raise ValueError(f"unknown queries: {unknown}")
    l1 = DeterministicGraphAStarL1(
        ctx,
        artifact,
        map_hash=ctx.map_sha256,
        topology_hash=str(topology_manifest.get("cache_key") or ""),
    )
    manager = R2L2StateLifecycleManager(
        cache_root,
        max_active_states=max_active_states,
        dstar_wall_budget_ms=dstar_budget_ms,
        dstar_max_expansions=20_000,
    )
    rows: list[dict[str, Any]] = []
    started = time.monotonic_ns()
    for query_id in query_ids:
        plan = l1.plan(by_id[query_id])
        if plan is None:
            raise RuntimeError(f"L1_NO_ROUTE: {query_id}")
        roi = CorridorROI.from_global(
            plan.static_safe_free,
            plan.corridor_mask,
            plan.start_cell,
            plan.goal_cell,
            binding_fields=plan.binding_fields(),
        )
        prebuild = manager.prebuild(roi, verify_oracle=True)
        manager.clear()
        planner, activation_result, activation = manager.activate(
            roi, verify_oracle=True,
        )
        row = {
            "query_id": query_id,
            **prebuild.as_dict(),
            **{f"activation_{key}": value for key, value in activation.as_dict().items()},
            "activation_success": activation_result.success,
            "activation_backend": activation_result.selected_backend,
            "activation_cost_error": activation_result.oracle_cost_error,
            "state_memory_bytes": planner.state_memory_bytes(),
            "online_synchronous_dstar_build_count": manager.synchronous_dstar_build_count,
        }
        rows.append(row)
        manager.clear()
        print(
            f"PREBUILT {query_id} total_ms={prebuild.total_ms:.1f} "
            f"activate_ms={activation.activate_ms:.1f} bytes={row['state_memory_bytes']}",
            flush=True,
        )
    with (output / "prebuild.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ["query_id"])
        writer.writeheader()
        writer.writerows(rows)
    (output / "prebuild.yaml").write_text(
        yaml.safe_dump(rows, sort_keys=False), encoding="utf-8",
    )
    source_hashes = _source_snapshot(output)
    manifest = {
        "architecture_id": ARCHITECTURE_ID,
        "revision_id": REVISION_ID,
        "protocol_id": PROTOCOL_ID,
        "map_id": MAP_ID,
        "map_hash": ctx.map_sha256,
        "query_ids": list(query_ids),
        "cache_root": str(cache_root),
        "offline_prebuild_not_charged_to_online": True,
        "source_files": source_hashes,
    }
    (output / "manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8",
    )
    activation_pass = all(
        row["activation_geometry_cache_hit"]
        and row["activation_state_cache_hit"]
        and row["activation_success"]
        and row["activation_backend"] in {
            "compact_persistent_dstar", "compact_dstar_cache_restore",
        }
        and float(row["activation_cost_error"] or 0.0) == 0.0
        for row in rows
    )
    verification = {
        "query_count": len(rows),
        "all_prebuild_success": all(row["success"] for row in rows),
        "all_verified_restore_success": activation_pass,
        "online_synchronous_dstar_build_zero": manager.synchronous_dstar_build_count == 0,
    }
    verification["prebuild_pass"] = all(verification.values())
    (output / "verification.yaml").write_text(
        yaml.safe_dump(verification, sort_keys=False), encoding="utf-8",
    )
    command_queries = ",".join(query_ids)
    (output / "reproduction_command.txt").write_text(
        "cd /home/robot/pudu_robot_ws\n"
        "PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/three_d_v1:"
        "/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation "
        f"/usr/bin/python3 -m arena_3d_v1.r2_cache_prebuild --output-dir {output} "
        f"--cache-root {cache_root} --query-ids {command_queries} "
        f"--dstar-budget-ms {dstar_budget_ms} --max-active-states {max_active_states}\n",
        encoding="utf-8",
    )
    summary = (
        f"COMPLETE output={output} prebuild_gate={verification['prebuild_pass']} "
        f"elapsed_ms={(time.monotonic_ns() - started) / 1.0e6:.1f}"
    )
    (output / "stdout.log").write_text(summary + "\n", encoding="utf-8")
    (output / "stderr.log").write_text("", encoding="utf-8")
    print(summary, flush=True)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cache-root", type=Path, required=True)
    parser.add_argument("--query-ids", default=",".join(HELDOUT_QUERIES))
    parser.add_argument("--dstar-budget-ms", type=float, default=500.0)
    parser.add_argument("--max-active-states", type=int, default=1)
    args = parser.parse_args()
    query_ids = tuple(item.strip() for item in args.query_ids.split(",") if item.strip())
    try:
        run(
            args.output_dir,
            query_ids=query_ids,
            cache_root=args.cache_root,
            dstar_budget_ms=args.dstar_budget_ms,
            max_active_states=args.max_active_states,
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
