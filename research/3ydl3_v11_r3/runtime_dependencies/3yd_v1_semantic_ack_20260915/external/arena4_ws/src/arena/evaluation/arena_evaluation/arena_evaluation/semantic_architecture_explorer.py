"""Write-once PLN-02 next-architecture feasibility prototypes.

This module does not define an online planner or a new architecture id.  It
compares bounded offline methods against the frozen positive query and always
routes complete candidates through the existing ConstraintWorld audit.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import platform
import resource
import shutil
import subprocess
import sys
import time

import cv2
import numpy as np
from scipy.optimize import differential_evolution

from .semantic_constraint_core import ConstraintWorld, dubins_edge
from .semantic_constraint_independent import IndependentWitness, integrate_waypoints
from .semantic_constraint_lattice import ResourceLattice
from .semantic_constraint_study import fresh, seal, write_json
from .semantic_map import sha256_file
from .se2_semantic_guide import ExplicitSE2GuideOracle, SE2GuidePolicy


PROTOCOL_ID = "PLN-02-NEXT-ARCHITECTURE-EXPLORATION-R1-V1"
POSITIVE = "r3-mirror-1-positive"
DEFAULT_INPUTS = Path(
    "/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/"
    "constraint_inputs_20260907T021418Z"
)
SOURCE_FILES = (
    Path(__file__),
    Path(__file__).with_name("semantic_constraint_core.py"),
    Path(__file__).with_name("semantic_constraint_lattice.py"),
    Path(__file__).with_name("semantic_constraint_independent.py"),
    Path(__file__).with_name("se2_semantic_guide.py"),
)


class TargetBudgetHybrid(ExplicitSE2GuideOracle):
    """Hybrid A* whose edge cost explicitly pays measured target-band debt."""

    def __init__(self, *args, target_debt_weight=8.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.target_debt_weight = float(target_debt_weight)

    def _reference_increment(self, metrics, length_m):
        reference, wrong, master = super()._reference_increment(metrics, length_m)
        count = max(1, int(metrics.get("lane_samples", 0)))
        target_ratio = float(metrics.get("target_samples", 0)) / count
        return (
            reference + self.target_debt_weight * (1.0 - target_ratio) * length_m,
            wrong,
            master,
        )


def _jsonable(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


def _run(command, cwd):
    result = subprocess.run(
        command, cwd=cwd, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, check=False,
    )
    return {"argv": command, "returncode": result.returncode, "output": result.stdout}


def _manifest(output, args, world=None):
    payload = {
        "protocol_id": PROTOCOL_ID,
        "method": args.method,
        "arguments": vars(args),
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "python": sys.version,
        "platform": platform.platform(),
        "source_hashes": {str(path): sha256_file(path) for path in SOURCE_FILES},
        "scope": "bounded_offline_feasibility_exploration_not_online_planner",
        "architecture_id": "UNNAMED_UNTIL_TARGETED_OFFLINE_3_OF_3",
        "old_contract_path_length_bound_retained": True,
        "online_ack": "NOT_APPLICABLE_OFFLINE",
    }
    if world is not None:
        payload["input_binding"] = {
            "query_id": world.query.query_id,
            "input_npz_sha256": world.meta["npz_sha256"],
            "expected_master_hash": world.meta["expected_master_hash"],
            "semantic_map_hash": world.meta["semantic_map_hash"],
            "selected_lane_labels": world.meta["selected_lane_labels"],
            "start": world.start,
            "goal": world.goal,
            "maximum_path_length_m": world.bound_length,
        }
    write_json(output / "manifest.json", _jsonable(payload))
    for index, path in enumerate(SOURCE_FILES):
        shutil.copy2(path, output / f"source_{index:02d}_{path.name}")


def freeze(args):
    output = fresh(args.output)
    root = Path(args.workspace).resolve()
    nested = root / "external/arena4_ws/src/arena/evaluation"
    commands = {
        "root_status": _run(["git", "status", "--short", "--branch"], root),
        "root_head": _run(["git", "rev-parse", "HEAD"], root),
        "evaluation_status": _run(["git", "status", "--short", "--branch"], nested),
        "evaluation_head": _run(["git", "rev-parse", "HEAD"], nested),
        "nav2_head": _run(
            ["git", "rev-parse", "HEAD"], root / "external/nav2_reference_ws/src/navigation2"
        ),
    }
    _manifest(output, args)
    write_json(output / "repository_state.json", commands)
    bindings = {}
    for query in (POSITIVE, "r3-mirror-2-negative", "cmp2-02-lane-south"):
        prefix = Path(args.inputs) / query
        bindings[query] = {
            "json_sha256": sha256_file(prefix.with_suffix(".json")),
            "npz_sha256": sha256_file(prefix.with_suffix(".npz")),
        }
    write_json(output / "input_hashes.json", bindings)
    seal(output)
    return 0


def _rectangle_kernel(resolution, yaw):
    half_cell = resolution / 2.0
    span = int(math.ceil(math.hypot(.265, .225) / resolution)) + 2
    coordinates = np.arange(-span, span + 1, dtype=float) * resolution
    dy, dx = np.meshgrid(coordinates, coordinates, indexing="ij")
    cosine, sine = math.cos(yaw), math.sin(yaw)
    ac, ass = abs(cosine), abs(sine)
    # Exact separating-axis overlap between the padded rectangle and a map
    # cell square.  The kernel is centrally symmetric, as required by dilate.
    return (
        (np.abs(dx) <= .265 * ac + .225 * ass + half_cell)
        & (np.abs(dy) <= .265 * ass + .225 * ac + half_cell)
        & (np.abs(cosine * dx + sine * dy) <= .265 + half_cell * (ac + ass))
        & (np.abs(-sine * dx + cosine * dy) <= .225 + half_cell * (ac + ass))
    ).astype(np.uint8)


def topology(args):
    output = fresh(args.output)
    world = ConstraintWorld(args.inputs, args.query)
    _manifest(output, args, world)
    started, cpu = time.monotonic(), time.process_time()
    scale = args.position_scale
    resolution = world.map.resolution / scale
    obstacle = np.repeat(np.repeat(world.obstacle.astype(np.uint8), scale, 0), scale, 1)
    legal = (
        (world.master < 253) & world.grids["allowed"] & ~world.grids["hard"]
        & np.isin(world.grids["labels"], world.selected)
    )
    legal = np.repeat(np.repeat(legal, scale, 0), scale, 1)
    any_yaw_free = np.zeros(legal.shape, dtype=bool)
    free_counts = []
    for index in range(args.yaw_samples):
        yaw = 2.0 * math.pi * index / args.yaw_samples
        blocked = cv2.dilate(obstacle, _rectangle_kernel(resolution, yaw)) != 0
        free = legal & ~blocked
        any_yaw_free |= free
        free_counts.append(int(np.count_nonzero(free)))

    component_count, components = cv2.connectedComponents(any_yaw_free.astype(np.uint8), 8)
    target = (
        world.grids["correct"] & (world.grids["error"] <= .50)
        & np.isin(world.grids["labels"], world.selected)
    )
    target = np.repeat(np.repeat(target, scale, 0), scale, 1)

    def subcell(pose):
        row, col = world.map.world_to_cell(*pose[:2])
        return row * scale + scale // 2, col * scale + scale // 2

    start_cell, goal_cell = subcell(world.start), subcell(world.goal)
    start_component = int(components[start_cell])
    goal_component = int(components[goal_cell])
    target_labels, target_counts = np.unique(components[target], return_counts=True)
    target_distribution = {
        int(label): int(count) for label, count in zip(target_labels, target_counts)
    }
    reachable_target = target & (components == start_component)
    rr, cc = np.where(reachable_target)
    result = {
        "position_resolution_m": resolution,
        "position_scale_from_frozen_grid": scale,
        "yaw_samples": args.yaw_samples,
        "yaw_step_deg": 360.0 / args.yaw_samples,
        "maximum_position_cover_radius_m": math.sqrt(2.0) * resolution / 2.0,
        "maximum_yaw_cover_error_deg": 180.0 / args.yaw_samples,
        "component_count": component_count - 1,
        "start_component": start_component,
        "goal_component": goal_component,
        "start_goal_connected_in_optimistic_any_yaw_projection": start_component == goal_component,
        "target_component_distribution": target_distribution,
        "reachable_target_subcell_count": int(np.count_nonzero(reachable_target)),
        "total_target_subcell_count": int(np.count_nonzero(target)),
        "main_target_component_reachable": bool(
            target_distribution
            and max(target_distribution, key=target_distribution.get) == start_component
        ),
        "reachable_target_subcell_bounds": None if not len(rr) else {
            "row_min": int(rr.min()), "row_max": int(rr.max()),
            "col_min": int(cc.min()), "col_max": int(cc.max()),
        },
        "free_center_count_min": min(free_counts),
        "free_center_count_max": max(free_counts),
        "wall_ms": (time.monotonic() - started) * 1000.0,
        "cpu_ms": (time.process_time() - cpu) * 1000.0,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        "proof_scope": (
            "exhaustive_at_documented_position_and_yaw_samples; any-yaw 2D union is "
            "optimistic_about yaw continuity; disconnection proves no sampled-center crossing, "
            "but is not by itself a continuous-space infeasibility proof"
        ),
    }
    write_json(output / "topology_result.json", result)
    np.savez_compressed(
        output / "topology_certificate.npz", any_yaw_free=any_yaw_free,
        components=components, reachable_target=reachable_target,
    )
    image = np.zeros((*legal.shape, 3), dtype=np.uint8)
    image[legal] = (235, 235, 235)
    image[target] = (130, 210, 130)
    image[reachable_target] = (40, 180, 230)
    image[(components == start_component) & any_yaw_free] = np.maximum(
        image[(components == start_component) & any_yaw_free], (180, 150, 70)
    )
    for cell, color in ((start_cell, (255, 0, 0)), (goal_cell, (180, 0, 180))):
        cv2.circle(image, (cell[1], cell[0]), max(3, scale), color, -1)
    cv2.imwrite(str(output / "topology_overlay.png"), image)
    seal(output)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


def lattice(args):
    output = fresh(args.output)
    world = ConstraintWorld(args.inputs, args.query)
    _manifest(output, args, world)
    started, cpu = time.monotonic(), time.process_time()
    solver = ResourceLattice(
        world, spacing=args.station_spacing, lateral=args.lateral_spacing,
        extension=args.extension, yaw_offsets=tuple(args.yaw_offsets),
        skip=args.station_skip, lateral_step=args.lateral_step,
        timeout=args.timeout, max_labels=args.max_states,
    )
    code = solver.search()
    audit, path = world.audit(solver.solution_edges) if solver.solution_edges else (
        {"gate_passed": False, "failure_code": "NO_WITNESS"}, np.empty((0, 3))
    )
    result = {
        "candidate_architecture": "resource_constrained_multilabel_state_lattice",
        "result_code": code, "gate_passed": bool(audit.get("gate_passed")),
        "audit": audit, "graph_config": solver.config,
        "node_count": len(solver.poses), "expanded_labels": solver.expanded,
        "generated_labels": len(solver.labels.vertex),
        "cached_edge_count": sum(map(len, solver.adjacency.values())),
        "suffix_bounds_at_start": None if not solver.prepass_complete else {
            "side_upper": float(solver.suffix_side[0]),
            "target_upper": float(solver.suffix_target[0]),
            "length_lower": float(solver.suffix_length[0]),
        },
        "wall_ms": (time.monotonic() - started) * 1000.0,
        "cpu_ms": (time.process_time() - cpu) * 1000.0,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        "infeasibility_scope": "documented_finite_DAG_only",
    }
    write_json(output / "result.json", _jsonable(result))
    write_json(output / "path.json", [
        {"x": float(x), "y": float(y), "yaw": float(yaw)} for x, y, yaw in path
    ])
    seal(output)
    print(json.dumps(_jsonable(result), indent=2, sort_keys=True))
    return 0 if result["gate_passed"] else 2


def hybrid(args):
    output = fresh(args.output)
    world = ConstraintWorld(args.inputs, args.query)
    _manifest(output, args, world)
    policy = replace(
        SE2GuidePolicy(), timeout_s=args.timeout, max_iterations=args.max_states,
        reference_deviation_weight=args.reference_weight,
        wrong_side_weight=args.wrong_side_weight,
        heuristic_weight=args.heuristic_weight,
    )
    oracle = TargetBudgetHybrid(
        world.map, world.master, world.grids["labels"], world.grids["error"],
        world.grids["correct"], world.selected, policy=policy,
        binding={"protocol_id": PROTOCOL_ID, "query_id": args.query},
        reference_polylines=[world.meta["route_polyline"]],
        target_debt_weight=args.target_weight,
    )
    # dataclasses.replace() copies declared HospitalMap fields but CropMap's
    # full-map transform metadata is attached dynamically.
    collision_map = oracle.checker._collision_map
    collision_map.full_origin = world.map.full_origin
    collision_map.full_height = world.map.full_height
    collision_map.row0 = world.map.row0
    collision_map.col0 = world.map.col0
    started, cpu = time.monotonic(), time.process_time()
    candidate = oracle.search(args.query, world.start, world.goal)
    path = np.asarray(candidate.path, dtype=float).reshape((-1, 3))
    # The oracle result is retained verbatim.  A successful result is rebuilt
    # as exact Dubins edges before the unchanged frozen audit can qualify it.
    audit = {"gate_passed": False, "failure_code": "NO_WITNESS"}
    if candidate.witness_exists and len(path):
        breakpoints = [world.start]
        for item in candidate.primitive_trace:
            breakpoints.append(tuple(path[int(item["end_path_index"])]))
        breakpoints[-1] = world.goal
        edges = [dubins_edge(a, b, radius=.40) for a, b in zip(breakpoints, breakpoints[1:])]
        audit, audited_path = world.audit(edges)
        path = audited_path
    result = {
        "candidate_architecture": "target_budget_route_conditioned_kinodynamic_hybrid_astar",
        "oracle": asdict(candidate), "frozen_audit": audit,
        "gate_passed": bool(audit.get("gate_passed")), "policy": asdict(policy),
        "target_debt_weight": args.target_weight,
        "wall_ms_outer": (time.monotonic() - started) * 1000.0,
        "cpu_ms_outer": (time.process_time() - cpu) * 1000.0,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        "failure_scope": "bounded_48_bin_SE2_search_not_continuous_infeasibility_proof",
    }
    write_json(output / "result.json", _jsonable(result))
    write_json(output / "path.json", [
        {"x": float(x), "y": float(y), "yaw": float(yaw)} for x, y, yaw in path
    ])
    seal(output)
    print(json.dumps(_jsonable(result), indent=2, sort_keys=True))
    return 0 if result["gate_passed"] else 2


def optimize(args):
    output = fresh(args.output)
    world = ConstraintWorld(args.inputs, args.query)
    _manifest(output, args, world)
    witness = IndependentWitness(Path(args.inputs) / args.query)
    started, cpu = time.monotonic(), time.process_time()
    # One northbound entry, two continuous target-band controls, and one exit.
    # Headings are continuous variables here, not snapped to the 48-bin graph.
    bounds = [
        (-24.6, -23.6), (-6.2, -5.0), (.65, 2.35),
        (-27.2, -25.7), (-1.9, -.45), (1.20, 3.05),
        (-26.2, -24.7), (-5.1, -3.4), (3.15, 4.65),
        (-26.4, -25.2), (-8.8, -6.0), (3.65, 5.20),
    ]

    def unpack(values):
        controls = [tuple(map(float, values[index:index + 3])) for index in range(0, 12, 3)]
        return [world.start, *controls, world.goal]

    def objective(values):
        if time.monotonic() - started >= args.timeout:
            raise TimeoutError
        if witness.counter >= args.max_states:
            raise RuntimeError("candidate budget exhausted")
        witness.counter += 1
        waypoints = unpack(values)
        path, controls, length, residuals = integrate_waypoints(waypoints)
        metrics = witness.metrics(path, length)
        if metrics is None:
            return 1.0e6
        audited = witness.full_audit(path, controls, metrics)
        side = metrics["lane_correct_side_ratio"]
        band = metrics["lane_target_band_ratio"]
        error = metrics["lane_target_error_p50_m"]
        score = (
            1000.0 * (max(0.0, .8 - side) + max(0.0, .50001 - band))
            + 100.0 * max(0.0, error - .5) + length * .0001
            + 10000.0 * metrics["conservative_clearance_deficit_integral"]
            + 100.0 * metrics["same_lane_violations"]
            + 1000.0 * max(0.0, length - witness.limit)
            + (0.0 if audited["path_collision_free"] else 1000.0)
        )
        if score < witness.best_score:
            witness.best_score = score
            witness.best = (waypoints, path, controls, residuals, audited)
            witness.rows.append({"candidate": witness.counter, "score": score, **audited})
        return score

    stop = "OPTIMIZER_COMPLETED"
    try:
        differential_evolution(
            objective, bounds, seed=args.seed, maxiter=100000, popsize=10,
            polish=False, tol=1e-10, updating="immediate", workers=1,
        )
    except TimeoutError:
        stop = "TIMEOUT_NOT_INFEASIBILITY_PROOF"
    except RuntimeError as error:
        stop = str(error)

    audit = {"gate_passed": False, "failure_code": "NO_CANDIDATE"}
    path = np.empty((0, 3))
    controls = []
    raw_metrics = None
    waypoints = []
    if witness.best is not None:
        waypoints, _, controls, _, raw_metrics = witness.best
        edges = [dubins_edge(a, b, radius=.401) for a, b in zip(waypoints, waypoints[1:])]
        audit, path = world.audit(edges)
    result = {
        "candidate_architecture": "continuous_waypoint_global_optimization",
        "optimizer": "scipy_differential_evolution_with_continuous_xy_yaw_controls",
        "seed": args.seed, "stop_reason": stop,
        "candidate_count": witness.counter, "full_audit_count": witness.full_audit_count,
        "all_candidates_dense_audited": witness.counter == witness.full_audit_count,
        "best_search_score": witness.best_score if witness.best is not None else None,
        "raw_best_metrics": raw_metrics, "frozen_audit": audit,
        "gate_passed": bool(audit.get("gate_passed")),
        "raw_waypoints": waypoints, "controls": controls,
        "wall_ms": (time.monotonic() - started) * 1000.0,
        "cpu_ms": (time.process_time() - cpu) * 1000.0,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        "failure_scope": "bounded_parameter_family_not_continuous_infeasibility_proof",
    }
    write_json(output / "result.json", _jsonable(result))
    write_json(output / "path.json", [
        {"x": float(x), "y": float(y), "yaw": float(yaw)} for x, y, yaw in path
    ])
    seal(output)
    print(json.dumps(_jsonable(result), indent=2, sort_keys=True))
    return 0 if result["gate_passed"] else 2


def finalize(args):
    output = fresh(args.output)
    world = ConstraintWorld(args.inputs, POSITIVE)
    _manifest(output, args, world)
    run_root = Path(args.run_root)
    names = {
        "stage0": "next_arch_r1_stage0_20260907T113300Z",
        "topology": "next_arch_r1_topology_positive_1cm_20260907T113000Z",
        "lattice": "next_arch_r1_lattice_positive_20260907T113400Z",
        "hybrid": "next_arch_r1_target_budget_hybrid_positive_20260907T114300Z",
        "optimize": "next_arch_r1_optimize_dense_all_positive_20260907T120000Z",
        "historical": "constraint_stage1_delivery_final_20260907T030500Z",
    }
    directories = {key: run_root / value for key, value in names.items()}
    topology_result = json.loads((directories["topology"] / "topology_result.json").read_text())
    lattice_result = json.loads((directories["lattice"] / "result.json").read_text())
    hybrid_result = json.loads((directories["hybrid"] / "result.json").read_text())
    optimize_result = json.loads((directories["optimize"] / "result.json").read_text())
    historical_rows = json.loads((directories["historical"] / "per_query.json").read_text())
    if isinstance(historical_rows, dict):
        historical_rows = historical_rows.get("queries", historical_rows.get("rows", []))
    old_by_id = {row["query_id"]: row for row in historical_rows}

    candidate_rows = [
        {
            "method": "resource_constrained_multilabel_state_lattice",
            "result": lattice_result["result_code"],
            "gate_passed": lattice_result["gate_passed"],
            "states_or_candidates": lattice_result["generated_labels"],
            "wall_ms": lattice_result["wall_ms"],
            "peak_rss_bytes": lattice_result["peak_rss_bytes"],
            "correct_side_ratio": "", "target_band_ratio": "", "lateral_p50_m": "",
            "scope": lattice_result["infeasibility_scope"],
        },
        {
            "method": "target_budget_route_conditioned_kinodynamic_hybrid_astar",
            "result": hybrid_result["oracle"]["failure_code"],
            "gate_passed": hybrid_result["gate_passed"],
            "states_or_candidates": hybrid_result["oracle"]["diagnostics"]["expanded_state_count"],
            "wall_ms": hybrid_result["wall_ms_outer"],
            "peak_rss_bytes": hybrid_result["peak_rss_bytes"],
            "correct_side_ratio": hybrid_result["oracle"]["diagnostics"]
                .get("best_rejected_metric_candidate", {}).get("lane_correct_side_ratio", ""),
            "target_band_ratio": hybrid_result["oracle"]["diagnostics"]
                .get("best_rejected_metric_candidate", {}).get("lane_target_band_ratio", ""),
            "lateral_p50_m": hybrid_result["oracle"]["diagnostics"]
                .get("best_rejected_metric_candidate", {}).get("lane_target_error_p50_m", ""),
            "scope": hybrid_result["failure_scope"],
        },
        {
            "method": "continuous_waypoint_global_optimization",
            "result": optimize_result["stop_reason"],
            "gate_passed": optimize_result["gate_passed"],
            "states_or_candidates": optimize_result["candidate_count"],
            "wall_ms": optimize_result["wall_ms"],
            "peak_rss_bytes": optimize_result["peak_rss_bytes"],
            "correct_side_ratio": optimize_result["frozen_audit"]["lane_correct_side_ratio"],
            "target_band_ratio": optimize_result["frozen_audit"]["lane_target_band_ratio"],
            "lateral_p50_m": optimize_result["frozen_audit"]["lane_target_error_p50_m"],
            "scope": optimize_result["failure_scope"],
        },
    ]
    with (output / "candidate_comparison.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(candidate_rows[0]))
        writer.writeheader()
        writer.writerows(candidate_rows)
    write_json(output / "candidate_comparison.json", candidate_rows)

    positive = {
        "query_id": POSITIVE, "strict_witness": False, "gate_passed": False,
        "failure_code": "NO_STRICT_WITNESS_AFTER_THREE_INDEPENDENT_R1_METHODS",
        "best_safe_correct_side_ratio": optimize_result["frozen_audit"]["lane_correct_side_ratio"],
        "best_safe_target_band_ratio": optimize_result["frozen_audit"]["lane_target_band_ratio"],
        "best_safe_lateral_p50_m": optimize_result["frozen_audit"]["lane_target_error_p50_m"],
        "best_safe_path_length_m": optimize_result["frozen_audit"]["path_length_m"],
    }
    query_rows = [positive]
    for query_id in ("r3-mirror-2-negative", "cmp2-02-lane-south"):
        old = old_by_id[query_id]
        query_rows.append({
            "query_id": query_id, "strict_witness": bool(old.get("offline_witness_exists")),
            "gate_passed": bool(old.get("stage1_gate_passed")),
            "failure_code": old.get("reason_code", ""),
            "best_safe_correct_side_ratio": old.get("lane_correct_side_ratio"),
            "best_safe_target_band_ratio": old.get("lane_target_band_ratio"),
            "best_safe_lateral_p50_m": old.get("lane_target_error_p50_m"),
            "best_safe_path_length_m": old.get("path_length_m"),
            "evidence_source": str(directories["historical"]),
        })
    write_json(output / "per_query.json", query_rows)
    with (output / "per_query.csv").open("x", newline="") as stream:
        fields = sorted({key for row in query_rows for key in row})
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(query_rows)

    path_rows = json.loads((directories["optimize"] / "path.json").read_text())
    path = np.asarray([[row["x"], row["y"], row["yaw"]] for row in path_rows])
    rows, cols, _ = world.cells(path)
    target = world.grids["correct"] & (world.grids["error"] <= .5)
    image = np.full((*world.master.shape, 3), 255, dtype=np.uint8)
    image[world.master >= 253] = (145, 150, 155)
    image[world.master >= 254] = (25, 30, 35)
    image[target] = (145, 215, 145)
    cv2.polylines(image, [np.column_stack((cols, rows)).astype(np.int32)], False, (35, 70, 225), 2)
    for pose, color in ((world.start, (210, 90, 20)), (world.goal, (160, 45, 160))):
        row, col = world.map.world_to_cell(*pose[:2])
        cv2.circle(image, (col, row), 5, color, -1)
    r0, r1 = max(0, int(rows.min()) - 25), min(world.map.height, int(rows.max()) + 26)
    c0, c1 = max(0, int(cols.min()) - 25), min(world.map.width, int(cols.max()) + 26)
    crop = image[r0:r1, c0:c1]
    cv2.imwrite(str(output / "positive_best_safe_overlay.png"), cv2.resize(
        crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST,
    ))
    shutil.copy2(directories["topology"] / "topology_overlay.png", output / "positive_topology_1cm_1deg.png")

    gate = {
        "verdict": "C1", "grade_B_achieved": False,
        "offline_targeted_passed": 2, "offline_targeted_total": 3,
        "positive_continuous_infeasibility_proved": False,
        "v3_defined": False,
        "online_adapter": "NOT_RUN_OFFLINE_GATE_FAILED",
        "online_three_arm_48_bin": "NOT_RUN_OFFLINE_GATE_FAILED",
        "selected8": "NOT_RUN_OFFLINE_GATE_FAILED",
        "exact_server_ack": "NOT_RUN_OFFLINE_GATE_FAILED",
        "latency_gate": "NOT_RUN_OFFLINE_GATE_FAILED",
        "reason": (
            "mirror positive has no strict witness; bounded failures and 1cm/1deg topology "
            "evidence do not constitute a continuous-space infeasibility proof"
        ),
        "topology_summary": topology_result,
    }
    write_json(output / "gate_results.json", gate)
    excluded = [
        "next_arch_r1_stage0_20260907T113000Z",
        "next_arch_r1_hybrid_positive_20260907T113400Z",
        "next_arch_r1_hybrid_positive_retry_20260907T113600Z",
        "next_arch_r1_hybrid_positive_retry2_20260907T113900Z",
        "next_arch_r1_optimize_positive_20260907T113400Z",
    ]
    write_json(output / "directory_roles.json", {
        "authoritative": {key: str(value) for key, value in directories.items()},
        "excluded": {name: "incomplete_or_superseded_diagnostic" for name in excluded},
        "this_directory": "authoritative_aggregation",
    })
    commands = [
        "semantic_architecture_explorer freeze --output <new-stage0-dir>",
        "semantic_architecture_explorer topology --position-scale 5 --yaw-samples 360 --output <new-dir>",
        "semantic_architecture_explorer lattice --timeout 120 --output <new-dir>",
        "semantic_architecture_explorer hybrid --timeout 120 --reference-weight 0 --wrong-side-weight 4 --target-weight 12 --heuristic-weight 1 --output <new-dir>",
        "semantic_architecture_explorer optimize --timeout 120 --seed 20260907 --output <new-dir>",
    ]
    (output / "reproduction_commands.txt").write_text("\n".join(commands) + "\n")
    seal(output)
    print(json.dumps(gate, indent=2, sort_keys=True))
    return 2


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument(
        "method", choices=("freeze", "topology", "lattice", "hybrid", "optimize", "finalize")
    )
    result.add_argument("--output", required=True, type=Path)
    result.add_argument("--inputs", type=Path, default=DEFAULT_INPUTS)
    result.add_argument("--query", default=POSITIVE)
    result.add_argument("--workspace", type=Path, default=Path("/home/robot/pudu_robot_ws"))
    result.add_argument(
        "--run-root", type=Path,
        default=Path("/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results"),
    )
    result.add_argument("--timeout", type=float, default=60.0)
    result.add_argument("--max-states", type=int, default=1_000_000)
    result.add_argument("--seed", type=int, default=20260907)
    result.add_argument("--position-scale", type=int, choices=(1, 2, 5), default=2)
    result.add_argument("--yaw-samples", type=int, default=360)
    result.add_argument("--station-spacing", type=float, default=.8)
    result.add_argument("--lateral-spacing", type=float, default=.2)
    result.add_argument("--extension", type=float, default=8.0)
    result.add_argument("--yaw-offsets", type=int, nargs="+", default=(-4, -2, 0, 2, 4))
    result.add_argument("--station-skip", type=int, default=2)
    result.add_argument("--lateral-step", type=float, default=.35)
    result.add_argument("--reference-weight", type=float, default=4.0)
    result.add_argument("--wrong-side-weight", type=float, default=3.0)
    result.add_argument("--target-weight", type=float, default=8.0)
    result.add_argument("--heuristic-weight", type=float, default=2.0)
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    if not 0 < args.timeout <= 120:
        raise SystemExit("--timeout must be in (0, 120]")
    if not 1 <= args.max_states <= 1_000_000:
        raise SystemExit("--max-states must be in [1, 1000000]")
    if args.yaw_samples < 48 or args.yaw_samples > 1440:
        raise SystemExit("--yaw-samples must be in [48, 1440]")
    return globals()[args.method](args)


if __name__ == "__main__":
    raise SystemExit(main())
