"""Conditional multi-query ROS/Nav2/Smac integration evidence for 3D-V1/r2."""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import time
from pathlib import Path
from typing import Any, Sequence

import yaml
import numpy as np

from arena_evaluation import l1_l3_corridor_hybrid_smoke as production
from arena_evaluation import path_audit
from arena_evaluation import two_layer_2d_v1_4x_dynamic_incremental_benchmark as map4
from arena_evaluation import two_layer_v1_r1_cache_benchmark as runtime_profile
from arena_evaluation import unified_four_backends_smoke as runtime

from .l2_incremental import CorridorROI
from .dynamic_policy import _inflate
from .pipeline import ProductionL3Adapter
from .production_l1 import DeterministicGraphAStarL1
from .r1_stage_a import (
    _barrier_sources,
    _path_support,
    _sha256,
    _source_snapshot,
)
from .r2_pipeline import Layered3DV1R2Controller
from .r2_stage_a import DEFAULT_FROZEN_CONFIG, _load_frozen_config
from .r2_state_lifecycle import (
    ARCHITECTURE_ID,
    PROTOCOL_ID,
    REVISION_ID,
    R2L2StateLifecycleManager,
)
from .real_stage_a_benchmark import MAP_ID, _load_inputs, _snapshot


ROOT = Path("/home/robot/pudu_robot_ws")
DEFAULT_QUERIES = ("A2B-03", "A2B-07", "A2B-17")


def _select_midroute_large_sources(
    path: Sequence[tuple[int, int]],
    *,
    count: int,
    existing_sources: set[tuple[int, int]],
    plan: Any,
    map_shape: Sequence[int],
    inflation_radius: int = 7,
    endpoint_clearance_cells: int = 40,
) -> set[tuple[int, int]]:
    """Select path-relevant changes that remain valid after endpoint inflation."""
    if len(path) < 10:
        raise RuntimeError("path too short for a mid-route Stage-B workload")
    height, width = int(map_shape[0]), int(map_shape[1])
    support = _path_support(path, map_shape)
    fixed_blocked = _inflate(existing_sources, map_shape, inflation_radius)
    start, goal = path[0], path[-1]
    low = max(1, len(path) // 5)
    high = min(len(path) - 1, (len(path) * 4) // 5)
    anchors = path[low:high]
    indices = np.linspace(0, len(anchors) - 1, min(len(anchors), count * 128), dtype=int)
    distance = inflation_radius + 1
    selected: set[tuple[int, int]] = set()
    for index in indices:
        anchor = anchors[int(index)]
        for drow, dcolumn in ((-distance, 0), (0, -distance), (0, distance), (distance, 0)):
            candidate = (anchor[0] + drow, anchor[1] + dcolumn)
            if (
                candidate in existing_sources or candidate in selected
                or not (0 <= candidate[0] < height and 0 <= candidate[1] < width)
                or not plan.static_safe_free[candidate]
                or max(abs(candidate[0] - start[0]), abs(candidate[1] - start[1]))
                < endpoint_clearance_cells
                or max(abs(candidate[0] - goal[0]), abs(candidate[1] - goal[1]))
                < endpoint_clearance_cells
            ):
                continue
            blocked = _inflate(
                existing_sources | selected | {candidate}, map_shape, inflation_radius,
            )
            if fixed_blocked.intersection(path) or blocked.intersection(path):
                continue
            changed = _inflate({candidate}, map_shape, inflation_radius)
            if not changed.intersection(support):
                continue
            if not any(plan.corridor_mask[cell] for cell in changed):
                continue
            selected.add(candidate)
            if len(selected) == count:
                return selected
    raise RuntimeError(f"could not select {count} endpoint-safe large-change sources")


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "item"):
        return value.item()
    return value


def _children() -> list[int]:
    try:
        value = Path(f"/proc/{os.getpid()}/task/{os.getpid()}/children").read_text().strip()
        return [int(item) for item in value.split()] if value else []
    except OSError:
        return []


def _record(
    query_id: str,
    scenario: str,
    pending: Any,
    confirmed: Any,
    outcome: dict[str, Any] | None,
) -> dict[str, Any]:
    result = confirmed.l2_result
    diagnostics = {} if outcome is None else dict(outcome.get("diagnostics") or {})
    metrics = {} if outcome is None else dict(outcome.get("metrics") or {})
    return {
        "query_id": query_id,
        "scenario": scenario,
        "pending_scheduler_reason": pending.scheduler.reason,
        "confirmed_scheduler_reason": confirmed.scheduler.reason,
        "l2_called": result is not None,
        "l2_backend": "" if result is None else result.selected_backend,
        "l2_success": False if result is None else result.success,
        "l2_response_ms": 0.0 if result is None else result.response_ms,
        "l2_expanded": 0 if result is None else result.dstar_stats.expanded_nodes,
        "partial_dstar": False if result is None else result.partial_dstar_result_returned,
        "l1_called": confirmed.l1_graph_astar_called,
        "l1_reroute_succeeded": confirmed.l1_reroute_succeeded,
        "failure_code": confirmed.failure_code,
        "l3_required": confirmed.l3_required,
        "l3_called": False if outcome is None else bool(outcome.get("called")),
        "l3_final_valid": False if outcome is None else bool(outcome.get("success")),
        "l3_failure_code": "" if outcome is None else str(outcome.get("failure_code") or ""),
        "costmap_content_acknowledged": diagnostics.get("costmap_update_acknowledged"),
        "costmap_ack_mismatch_cells": diagnostics.get("costmap_ack_mismatch_cells"),
        "roi_message_count": diagnostics.get("roi_message_count"),
        "roi_max_message_bytes": diagnostics.get("roi_max_message_bytes"),
        "static_footprint_valid": metrics.get("static_footprint_valid"),
        "kinematic_valid": metrics.get("kinematic_valid"),
        "maximum_curvature": metrics.get("maximum_curvature"),
        "canonical_path_audit_reused": False if outcome is None else bool(
            outcome.get("canonical_path_audit_reused")
        ),
        "dirty_roi_changed_cells": 0 if confirmed.dirty_roi is None else confirmed.dirty_roi.changed_cells,
    }


def _write_not_run(output: Path, heldout: Path, failed: Sequence[str]) -> Path:
    output.mkdir(parents=True)
    marker = (
        "NOT_RUN_L2_GATE_FAILED\n"
        f"failed_gates={','.join(failed)}\n"
        "ROS/Nav2/Smac was not started.\n"
    )
    (output / "NOT_RUN_L2_GATE_FAILED").write_text(marker, encoding="utf-8")
    status = {
        "status": "NOT_RUN_L2_GATE_FAILED",
        "heldout_directory": str(heldout),
        "failed_gates": list(failed),
        "stage_b_processes_started": 0,
    }
    (output / "stage_b_status.yaml").write_text(
        yaml.safe_dump(status, sort_keys=False), encoding="utf-8",
    )
    (output / "verification.yaml").write_text(
        yaml.safe_dump({
            "heldout_stage_a_pass": False,
            "stage_b_not_started": True,
            "protocol_compliant": True,
        }, sort_keys=False), encoding="utf-8",
    )
    (output / "stdout.log").write_text(json.dumps(status) + "\n", encoding="utf-8")
    (output / "stderr.log").write_text("", encoding="utf-8")
    return output


def run(
    output: Path,
    *,
    heldout: Path,
    query_ids: Sequence[str] = DEFAULT_QUERIES,
    ros_domain_id: int = 141,
    costmap_ack_timeout_s: float = 3.0,
    frozen_config: Path = DEFAULT_FROZEN_CONFIG,
) -> Path:
    output = output.resolve()
    heldout = heldout.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    frozen = _load_frozen_config(frozen_config)
    gate_path = heldout / "gate_results.yaml"
    if not gate_path.is_file():
        raise ValueError("held-out gate evidence is incomplete")
    gates = yaml.safe_load(gate_path.read_text(encoding="utf-8")) or {}
    if gates.get("mode") != "heldout":
        raise ValueError("Stage-B gate input is not held-out evidence")
    required_gates = (
        "oracle_parity_pass", "scheduler_parity_pass", "recovery_pass",
        "p50_gate_pass", "p95_gate_pass", "p99_gate_pass",
        "warm_activation_gate_pass", "resident_reduction_gate_pass",
        "resident_target_pass", "lru_bound_pass", "cache_hit_pass",
        "no_sync_build_pass", "telemetry_complete_pass",
        "frozen_baselines_unchanged",
    )
    failed = [key for key in required_gates if gates.get(key) is not True]
    if gates.get("stage_a_pass") is not True or failed:
        return _write_not_run(output, heldout, failed)
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
    plans = {query_id: l1.plan(by_id[query_id]) for query_id in query_ids}
    if any(plan is None for plan in plans.values()):
        raise RuntimeError("representative Stage-B query has no L1 route")
    cache_root = output / "verified_r2_cache"
    manager = R2L2StateLifecycleManager(
        cache_root,
        max_active_states=1,
        dstar_wall_budget_ms=float(frozen["policy"]["dstar_wall_budget_ms"]),
        dstar_max_expansions=int(frozen["policy"]["dstar_max_expansions"]),
    )
    prebuild_rows: list[dict[str, Any]] = []
    for query_id in query_ids:
        plan = plans[query_id]
        assert plan is not None
        roi = CorridorROI.from_global(
            plan.static_safe_free,
            plan.corridor_mask,
            plan.start_cell,
            plan.goal_cell,
            binding_fields=plan.binding_fields(),
        )
        telemetry = manager.prebuild(roi, verify_oracle=True)
        prebuild_rows.append({"query_id": query_id, **telemetry.as_dict()})
    spec = runtime.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"BACKEND_UNAVAILABLE: {spec.reason}")
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    session = production.SmacSession(
        ctx,
        output,
        map_yaml=map4.FOUR_X_MAP_YAML,
        log_tag="3d_v1_r2_stage_b",
        local_mask_updates=True,
        optimization_profile=runtime_profile.OPTIMIZATION_PROFILE,
        smac_parameter_profile=runtime_profile.SMAC_PARAMETER_PROFILE,
        optimization_stage=runtime_profile.OPTIMIZATION_STAGE,
        enable_mask_reuse_noop=True,
        planner_parameter_overrides={"angle_quantization_bins": 48},
        costmap_ack_timeout_s=float(costmap_ack_timeout_s),
    )
    session.local_map_update_strategy = "roi_ack"
    session.full_grid_settle_cycles = 0
    auditor = path_audit.PathAuditor(ctx, source_commit=runtime._source_commit() or "unknown")
    rows: list[dict[str, Any]] = []
    timestamp = 0
    started = time.monotonic_ns()
    try:
        session.start()
        for query_id in query_ids:
            query = by_id[query_id]
            plan = plans[query_id]
            assert plan is not None
            controller = Layered3DV1R2Controller(
                plan,
                cache_root=cache_root,
                lifecycle_manager=manager,
                max_active_states=1,
                dynamic_inflation_radius_cells=7,
                dstar_wall_budget_ms=float(frozen["policy"]["dstar_wall_budget_ms"]),
                dstar_max_expansions=int(frozen["policy"]["dstar_max_expansions"]),
                dstar_attempt_max_changed_cells=2,
            )
            if not controller.initial_l2_result.success:
                raise RuntimeError(f"L2_INITIAL_NO_PATH: {query_id}")
            initial_path = list(controller.l2.path_global or ())
            eligible = {initial_path[len(initial_path) // 2]}
            barrier = _barrier_sources(
                plan.start_cell, plan.goal_cell, artifact.free_mask.shape,
            )
            adapter = ProductionL3Adapter(controller, auditor)

            def confirm(sources: set[tuple[int, int]], label: str) -> tuple[Any, Any]:
                nonlocal timestamp
                timestamp += 1
                pending = controller.process_snapshot(
                    _snapshot(
                        timestamp, sorted(sources), map_hash=ctx.map_sha256,
                        shape=artifact.free_mask.shape,
                    ),
                    l1_replan=lambda blocked: l1.plan(query, blocked),
                    now=float(timestamp),
                )
                timestamp += 1
                confirmed = controller.process_snapshot(
                    _snapshot(
                        timestamp, sorted(sources), map_hash=ctx.map_sha256,
                        shape=artifact.free_mask.shape,
                    ),
                    l1_replan=lambda blocked: l1.plan(query, blocked),
                    now=float(timestamp),
                )
                return pending, confirmed

            pending, confirmed = confirm(eligible, "eligible")
            eligible_outcome = adapter.plan(confirmed, query, session, spec)
            rows.append(_record(query_id, "eligible_dstar", pending, confirmed, eligible_outcome))

            repaired_path = list(controller.l2.path_global or ())
            cluster = _select_midroute_large_sources(
                repaired_path,
                count=5,
                existing_sources=eligible,
                plan=plan,
                map_shape=artifact.free_mask.shape,
            )
            pending, confirmed = confirm(eligible | cluster, "large")
            large_outcome = adapter.plan(confirmed, query, session, spec)
            rows.append(_record(query_id, "large_change_fallback", pending, confirmed, large_outcome))

            pending, confirmed = confirm(barrier, "no_route")
            rows.append(_record(query_id, "no_route", pending, confirmed, None))

            pending, confirmed = confirm(set(), "recovery")
            recovery_outcome = adapter.plan(confirmed, query, session, spec)
            rows.append(_record(query_id, "recovery", pending, confirmed, recovery_outcome))
    finally:
        session.close()
    elapsed_ms = (time.monotonic_ns() - started) / 1.0e6
    with (output / "stage_b_results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ["query_id"])
        writer.writeheader()
        writer.writerows(rows)
    with (output / "stage_b_results.jsonl").open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(_jsonable(row), sort_keys=True) + "\n")
    (output / "prebuild.yaml").write_text(
        yaml.safe_dump(prebuild_rows, sort_keys=False), encoding="utf-8",
    )
    scenario_by_query = {
        query_id: {row["scenario"]: row for row in rows if row["query_id"] == query_id}
        for query_id in query_ids
    }
    per_query_pass = {}
    for query_id, scenarios in scenario_by_query.items():
        eligible = scenarios.get("eligible_dstar", {})
        large = scenarios.get("large_change_fallback", {})
        no_route = scenarios.get("no_route", {})
        recovery = scenarios.get("recovery", {})
        per_query_pass[query_id] = bool(
            eligible.get("l2_backend") == "compact_persistent_dstar"
            and eligible.get("l3_final_valid")
            and "astar" in str(large.get("l2_backend", ""))
            and large.get("l3_final_valid")
            and not no_route.get("l2_success", True)
            and no_route.get("failure_code") in {
                "L1_NO_ROUTE", "L2_NO_PATH_AFTER_L1_REROUTE",
            }
            and recovery.get("l2_success")
            and recovery.get("l3_final_valid")
        )
    l3_rows = [row for row in rows if row["l3_called"]]
    residual_children = _children()
    verification = {
        "heldout_stage_a_pass": True,
        "query_count": len(query_ids),
        "four_scenarios_per_query": len(rows) == 4 * len(query_ids),
        "per_query_pass": per_query_pass,
        "all_l3_final_valid": bool(l3_rows) and all(row["l3_final_valid"] for row in l3_rows),
        "content_ack_all_pass": bool(l3_rows) and all(
            row["costmap_content_acknowledged"] is True
            and int(row["costmap_ack_mismatch_cells"] or 0) == 0
            for row in l3_rows
        ),
        "canonical_path_audit_single_instance_reused": bool(l3_rows) and all(
            row["canonical_path_audit_reused"] for row in l3_rows
        ),
        "smac_angle_quantization_bins": 48,
        "fixed_settle_cycles": session.full_grid_settle_cycles,
        "session_start_count": session.session_start_count,
        "session_restart_count": session.session_restart_count,
        "session_close_count": session.session_close_count,
        "online_synchronous_dstar_build_zero": manager.synchronous_dstar_build_count == 0,
        "residual_child_pids": residual_children,
    }
    verification["stage_b_pass"] = bool(
        all(per_query_pass.values())
        and verification["all_l3_final_valid"]
        and verification["content_ack_all_pass"]
        and verification["canonical_path_audit_single_instance_reused"]
        and verification["fixed_settle_cycles"] == 0
        and verification["session_start_count"] == 1
        and verification["session_restart_count"] == 0
        and verification["session_close_count"] == 1
        and verification["online_synchronous_dstar_build_zero"]
        and not residual_children
    )
    (output / "verification.yaml").write_text(
        yaml.safe_dump(verification, sort_keys=False), encoding="utf-8",
    )
    source_hashes = _source_snapshot(output)
    manifest = {
        "architecture_id": ARCHITECTURE_ID,
        "revision_id": REVISION_ID,
        "protocol_id": PROTOCOL_ID,
        "evidence_class": "multi-query integration evidence; not population performance",
        "map_id": MAP_ID,
        "map_hash": ctx.map_sha256,
        "query_ids": list(query_ids),
        "heldout_directory": str(heldout),
        "heldout_gate_sha256": _sha256(gate_path),
        "frozen_config": str(frozen_config.resolve()),
        "frozen_config_sha256": _sha256(frozen_config),
        "source_files": source_hashes,
    }
    (output / "manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8",
    )
    command_queries = ",".join(query_ids)
    (output / "reproduction_command.txt").write_text(
        "cd /home/robot/pudu_robot_ws\n"
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        "PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/three_d_v1:"
        "/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation "
        f"ROS_DOMAIN_ID={ros_domain_id} /usr/bin/python3 -m arena_3d_v1.r2_stage_b "
        f"--output-dir {output} --heldout {heldout} --query-ids {command_queries} "
        f"--ros-domain-id {ros_domain_id} --costmap-ack-timeout-s {costmap_ack_timeout_s} "
        f"--frozen-config {frozen_config.resolve()}\n",
        encoding="utf-8",
    )
    report = [
        "# 3D-V1-r2 conditional Stage B", "",
        f"- representative queries: `{', '.join(query_ids)}`",
        "- scenarios/query: eligible D*, large-change fallback, no-route, recovery",
        f"- final-valid L3 calls: `{sum(bool(row['l3_final_valid']) for row in l3_rows)}/{len(l3_rows)}`",
        f"- one Smac session / one canonical PathAudit instance reused: `{verification['canonical_path_audit_single_instance_reused']}`",
        f"- result: **{'PASS' if verification['stage_b_pass'] else 'FAIL'}**", "",
        "This is integration evidence only; it is not a population-level end-to-end performance claim.",
    ]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    summary = (
        f"PASS output={output} stage_b_gate={verification['stage_b_pass']} "
        f"elapsed_ms={elapsed_ms:.1f}"
    )
    (output / "stdout.log").write_text(summary + "\n", encoding="utf-8")
    (output / "stderr.log").write_text("", encoding="utf-8")
    print(summary, flush=True)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--heldout", type=Path, required=True)
    parser.add_argument("--query-ids", default=",".join(DEFAULT_QUERIES))
    parser.add_argument("--ros-domain-id", type=int, default=141)
    parser.add_argument("--costmap-ack-timeout-s", type=float, default=3.0)
    parser.add_argument("--frozen-config", type=Path, default=DEFAULT_FROZEN_CONFIG)
    args = parser.parse_args()
    query_ids = tuple(item.strip() for item in args.query_ids.split(",") if item.strip())
    try:
        run(
            args.output_dir,
            heldout=args.heldout,
            query_ids=query_ids,
            ros_domain_id=args.ros_domain_id,
            costmap_ack_timeout_s=args.costmap_ack_timeout_s,
            frozen_config=args.frozen_config,
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
