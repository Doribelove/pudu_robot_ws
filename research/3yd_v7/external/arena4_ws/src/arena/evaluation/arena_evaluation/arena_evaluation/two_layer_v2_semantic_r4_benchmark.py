"""PLN-02 2A-V2 r4 explicit SE(2)-guide feasibility runner.

Stage 2 is deliberately isolated from ROS.  It constructs the frozen r3 R0
effective master and asks whether the exact targeted requests have a qualifying
48-bin, forward-DUBIN, full-footprint witness.  A failed witness is a C1 hard
stop; this runner never starts or impersonates an online E5 planner.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import resource
import shlex
import shutil
import subprocess
import sys
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Dict, Iterator, Mapping, Optional, Sequence

import cv2
import numpy as np
import yaml

from . import two_layer_v2_semantic_r1_benchmark as r1
from . import two_layer_v2_semantic_r2_benchmark as r2
from .regional_preference_r1 import expand_roi_to_route_lane_instances, orient_route_for_query
from .regional_preference_r3 import RegionalPreferenceBuilderR3
from .se2_semantic_guide import ExplicitSE2GuideOracle, SE2GuidePolicy
from .semantic_costmap_r2 import SemanticCostmapComposerR2
from .semantic_map import canonical_hash, sha256_file
from .semantic_rasterizer import grid_hash


ARCHITECTURE_ID = "2A-V2"
IMPLEMENTATION_REVISION = "r4-se2-guide-interface"
PROTOCOL_ID = "PLN-02-2A-V2-R4-SE2-GUIDE-INTERFACE-V1"
PARENT_ARCHITECTURE = "2A-V2-r3"
ROOT = Path(__file__).resolve().parents[7]
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE_ROOT / "config/two_layer_v2_semantic_r4.yaml"
DEFAULT_TARGETS = PACKAGE_ROOT / "config/pudu_wanda_3f_r3_targeted_preflight3_v1.yaml"
DEFAULT_SELECTED8 = PACKAGE_ROOT / "config/pudu_wanda_3f_selected8_gt50m_r2_v2.yaml"
DEFAULT_R3_RESULT = (
    ROOT / "private_data/pudu_wanda_3f/results/real_ablation_r3_targeted_preflight3_v1"
)


def _git_read(path: Path, *arguments: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(path), *arguments], check=False,
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
    )
    return result.stdout.strip()


def _json_default(value: Any) -> Any:
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"not JSON serializable: {type(value).__name__}")


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False, default=_json_default)
        + "\n",
        encoding="utf-8",
    )


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        path.write_text("\n", encoding="utf-8")
        return
    fields = sorted({str(key) for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fields})


def _process_audit() -> str:
    result = subprocess.run(
        ["ps", "-eo", "pid,ppid,etimes,args"], check=False,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    lines = [line for line in result.stdout.splitlines() if any(
        token in line.lower() for token in ("ros2", "nav2", "smac", "rviz", "r4_benchmark")
    )]
    return "\n".join(lines) + ("\n" if lines else "")


def _load_config(path: Path) -> Dict[str, Any]:
    wrapper = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if wrapper.get("architecture_id") != ARCHITECTURE_ID:
        raise ValueError("r4 architecture identity mismatch")
    if wrapper.get("implementation_revision") != IMPLEMENTATION_REVISION:
        raise ValueError("r4 implementation identity mismatch")
    if wrapper.get("protocol_id") != PROTOCOL_ID:
        raise ValueError("r4 protocol identity mismatch")
    parent_info = wrapper.get("parent_config") or {}
    parent = Path(str(parent_info.get("path", "")))
    if not parent.is_absolute():
        parent = path.parent / parent
    expected_hash = str(parent_info.get("sha256", ""))
    actual_hash = sha256_file(parent)
    if actual_hash != expected_hash:
        raise ValueError(f"frozen r3 parent config hash mismatch: {actual_hash} != {expected_hash}")
    merged = yaml.safe_load(parent.read_text(encoding="utf-8")) or {}
    merged.update({
        "schema_version": wrapper["schema_version"],
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
        "parent_architecture": PARENT_ARCHITECTURE,
        "parent_config": {**parent_info, "resolved_path": str(parent.resolve())},
        "frozen_bindings": wrapper["frozen_bindings"],
        "se2_guide": wrapper["se2_guide"],
        "stage2_gate": wrapper["stage2_gate"],
        "stage4_online_gate": wrapper["stage4_online_gate"],
        "hard_stop": wrapper["hard_stop"],
    })
    guide = merged["se2_guide"]
    required = {
        "yaw_bins": 48, "motion_model": "DUBIN", "allow_reverse": False,
        "allow_in_place_rotation": False, "minimum_turning_radius_m": 0.40,
        "maximum_curvature_1pm": 2.50,
    }
    for name, expected in required.items():
        if guide.get(name) != expected:
            raise ValueError(f"immutable SE(2) policy mismatch for {name}")
    return merged


@contextmanager
def _r4_bindings() -> Iterator[None]:
    values = {
        "ARCHITECTURE_ID": ARCHITECTURE_ID,
        "IMPLEMENTATION_REVISION": IMPLEMENTATION_REVISION,
        "PARENT_ARCHITECTURE": PARENT_ARCHITECTURE,
        "DEFAULT_CONFIG": DEFAULT_CONFIG,
        "RegionalPreferenceBuilderR1": RegionalPreferenceBuilderR3,
        "SemanticCostmapComposer": SemanticCostmapComposerR2,
    }
    previous = {name: getattr(r1, name) for name in values}
    try:
        for name, value in values.items():
            setattr(r1, name, value)
        yield
    finally:
        for name, value in previous.items():
            setattr(r1, name, value)


def _query_content_hash(path: Path) -> str:
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    claimed = str(payload.get("query_hash", ""))
    actual = canonical_hash([
        {key: item[key] for key in ("query_id", "start", "goal", "category")}
        for item in payload.get("queries", [])
    ])
    if claimed and claimed != actual:
        raise ValueError(f"query content hash mismatch: {actual} != {claimed}")
    return actual


def _old_e4_path(root: Path, query_id: str) -> list[tuple[float, float, float]]:
    path = root / "paths" / f"E4_{query_id}_measured_1.json"
    if not path.exists():
        return []
    payload = json.loads(path.read_text(encoding="utf-8"))
    return [(float(item["x"]), float(item["y"]), float(item["yaw"])) for item in payload]


def _draw_polyline(image: np.ndarray, hospital_map: Any, poses: Sequence[Sequence[float]], color: tuple[int, int, int], width: int) -> list[tuple[int, int]]:
    cells = []
    for pose in poses:
        cell = hospital_map.world_to_cell(float(pose[0]), float(pose[1]))
        if cell is not None:
            cells.append((int(cell[1]), int(cell[0])))
    if len(cells) >= 2:
        cv2.polylines(image, [np.asarray(cells, np.int32)], False, color, width, cv2.LINE_AA)
    return cells


def _draw_heading(image: np.ndarray, hospital_map: Any, pose: Sequence[float], color: tuple[int, int, int]) -> list[tuple[int, int]]:
    origin = hospital_map.world_to_cell(float(pose[0]), float(pose[1]))
    tip = hospital_map.world_to_cell(
        float(pose[0]) + 0.8 * np.cos(float(pose[2])),
        float(pose[1]) + 0.8 * np.sin(float(pose[2])),
    )
    if origin is None or tip is None:
        return []
    first, second = (origin[1], origin[0]), (tip[1], tip[0])
    cv2.arrowedLine(image, first, second, color, 3, cv2.LINE_AA, tipLength=0.35)
    return [first, second]


def _write_overlay(
    path: Path, hospital_map: Any, query: Any, route: Any, preference: Any,
    witness: Sequence[Sequence[float]], old_e4: Sequence[Sequence[float]], title: str,
) -> None:
    base = cv2.imread(str(hospital_map.image_path), cv2.IMREAD_GRAYSCALE)
    image = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)
    all_cells: list[tuple[int, int]] = []
    all_cells += _draw_polyline(image, hospital_map, route.polyline, (30, 30, 30), 3)
    for segment in preference.diagnostics.get("guide_polylines_world", []):
        all_cells += _draw_polyline(image, hospital_map, segment, (0, 180, 0), 4)
    all_cells += _draw_polyline(image, hospital_map, old_e4, (255, 180, 0), 3)
    all_cells += _draw_polyline(image, hospital_map, witness, (0, 0, 230), 3)
    all_cells += _draw_heading(image, hospital_map, query.start, (255, 0, 0))
    all_cells += _draw_heading(image, hospital_map, query.goal, (180, 0, 180))
    if all_cells:
        coords = np.asarray(all_cells, dtype=np.int32)
        margin = 80
        c0 = max(0, int(coords[:, 0].min()) - margin)
        c1 = min(image.shape[1], int(coords[:, 0].max()) + margin + 1)
        r0 = max(0, int(coords[:, 1].min()) - margin)
        r1_ = min(image.shape[0], int(coords[:, 1].max()) + margin + 1)
        image = image[r0:r1_, c0:c1]
    cv2.putText(image, title, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 220), 2)
    cv2.putText(
        image, "black=L1 green=r3-2D cyan=historical-E4 red=r4-SE2",
        (12, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (20, 20, 20), 1,
    )
    cv2.imwrite(str(path), image)


def _rss_bytes() -> int:
    return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024


def _snapshot_and_manifest(output: Path, reproduction: str, sources: Sequence[Path]) -> None:
    snapshot = output / "source_snapshot"
    snapshot.mkdir(exist_ok=True)
    source_hashes: Dict[str, str] = {}
    for index, source in enumerate(sources):
        if not source.is_file():
            continue
        target = snapshot / f"{index:02d}_{source.name}"
        shutil.copy2(source, target)
        source_hashes[str(source.resolve())] = sha256_file(source)
    (output / "reproduction_command.txt").write_text(reproduction + "\n", encoding="utf-8")
    artifacts = {
        str(path.relative_to(output)): sha256_file(path)
        for path in sorted(output.rglob("*"))
        if path.is_file() and path.name != "manifest.json"
    }
    manifest = {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
        "workspace_branch": _git_read(ROOT, "branch", "--show-current"),
        "workspace_head": _git_read(ROOT, "rev-parse", "HEAD"),
        "workspace_status": _git_read(ROOT, "status", "--short"),
        "evaluation_head": _git_read(PACKAGE_ROOT.parent, "rev-parse", "HEAD"),
        "nav2_head": _git_read(
            ROOT / "external/arena4_ws/src/deps/nav2/navigation2", "rev-parse", "HEAD",
        ),
        "source_hashes": source_hashes,
        "artifact_hashes": artifacts,
    }
    _write_json(output / "manifest.json", manifest)


def _stage2_report(rows: Sequence[Mapping[str, Any]], all_passed: bool) -> str:
    outcome = "STAGE2_PASS_PENDING_E5" if all_passed else "C1"
    lines = [
        "# 2A-V2 r4 Stage 2 explicit SE(2) oracle", "",
        f"Outcome: **{outcome}**.", "",
        "This is an offline feasibility result, not an online Smac success and not a production claim.", "",
        "| query | witness | failure | side ratio | lateral P50 (m) | curvature (1/m) | wall (ms) |",
        "|---|---:|---|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "| {query_id} | {witness_exists} | {failure_code} | {lane_correct_side_ratio} | "
            "{lane_target_error_p50_m} | {maximum_curvature_1pm} | {wall_ms} |".format(**row)
        )
    lines += ["", "The oracle used the frozen endpoints, 48 yaw bins, forward-only DUBIN primitives, "
              "Rmin=0.40 m, curvature<=2.50 1/m, the complete padded Jackal footprint, the "
              "same selected lane instance, and the r3 R0 exact expected effective master."]
    if not all_passed:
        lines += ["", "Per the frozen hard stop, online E5 and selected8 are NOT_RUN."]
    return "\n".join(lines) + "\n"


def run_stage2(args: argparse.Namespace, arguments: Sequence[str]) -> int:
    output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to reuse non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir(exist_ok=True)
    (output / "overlays").mkdir(exist_ok=True)
    (output / "process_audit_before.txt").write_text(_process_audit(), encoding="utf-8")
    config = _load_config(args.config.resolve())
    policy = SE2GuidePolicy.from_mapping(config["se2_guide"])
    policy.validate()
    query_hash = _query_content_hash(args.query_set.resolve())
    frozen = config["frozen_bindings"]
    if query_hash != frozen["targeted_query_hash"]:
        raise ValueError("targeted query hash does not match frozen r4 binding")

    protocol = {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
        "stage": "stage2_same_primitive_offline_oracle",
        "parent_architecture": PARENT_ARCHITECTURE,
        "native_smac_explicit_se2_input": False,
        "online_e5_prohibited_until_all_stage2_witnesses": True,
        "configuration": config,
    }
    _write_json(output / "protocol.json", protocol)

    with _r4_bindings(), r2._query_set_binding(args.query_set.resolve(), require_default_contract=False):
        r1._validate_protocol(config)
        prepared = r1._prepare(
            args.extracted_dir.resolve(), args.semantic_map.resolve(),
            args.topology_cache.resolve(), config, output=output,
        )
    ctx, semantic_map, raster, topology, _annotator, router, query_bundle = prepared[:7]
    queries = query_bundle[0]
    actual_ids = [query.query_id for query in queries]
    if actual_ids != list(frozen["targeted_query_ids"]):
        raise ValueError(f"target query order mismatch: {actual_ids}")
    if ctx.map_sha256 != frozen["map_hash"]:
        raise ValueError("map hash mismatch")
    if semantic_map.semantic_map_hash != frozen["semantic_map_hash"]:
        raise ValueError("semantic map hash mismatch")

    selector = r1._semantic_selector(topology, router)
    builder = RegionalPreferenceBuilderR3(
        ctx.hospital_map, raster, policy=config["regional_preference"], semantic_map=semantic_map,
    )
    composer = SemanticCostmapComposerR2(
        policy=config["l3_soft_cost"],
        inflation_cache_capacity=int(config["cache"]["composer_inflation_capacity"]),
    )
    rows: list[Dict[str, Any]] = []
    traces: list[Dict[str, Any]] = []
    costs: list[Dict[str, Any]] = []
    detailed: list[Dict[str, Any]] = []
    route_bindings: Dict[str, Any] = {}

    for query in queries:
        query_started = time.monotonic()
        _start_node, _goal_node, route, reason = selector(
            topology, query, cache_mode=r1.r2_runtime.CACHE_MODE_OPTIMIZED, timing={},
        )
        if route is None:
            row = {
                "query_id": query.query_id, "witness_exists": False,
                "failure_code": "NO_SE2_ROUTE", "failure_detail": reason,
                "lane_correct_side_ratio": "", "lane_target_error_p50_m": "",
                "maximum_curvature_1pm": "", "reverse_distance_m": "",
                "in_place_rotation_count": "", "wall_ms": (time.monotonic() - query_started) * 1000.0,
                "gate_passed": False,
            }
            rows.append(row)
            detailed.append(row)
            continue
        route, orientation = orient_route_for_query(route, query)
        allowed = r1.r2_runtime._raw_corridor_mask(
            ctx, topology, route, query, float(config["roi"]["r0_padding_m"]),
        )
        allowed, roi_diagnostics = expand_roi_to_route_lane_instances(
            ctx.hospital_map, raster, semantic_map, route.polyline, allowed,
            free_mask=r1.r2_runtime._raw_free_mask(ctx),
            route_probe_radius_m=float(config["roi"]["lane_route_probe_radius_m"]),
        )
        preference = builder.build(
            route.polyline, goal=query.goal, allowed_mask=allowed,
            relaxation_level="R0", planning_preference_enabled=True,
            route_diagnostics=orientation,
        )
        selected_labels = list(preference.diagnostics.get("guide_lane_instance_ids", []))
        semantic_costmap = composer.compose(
            ctx.hospital_map.occupancy, raster, preference, allowed_mask=allowed,
            hard_semantics_enabled=True, soft_class_costs_enabled=True,
            regional_preference_enabled=True, hard_semantics_use_footprint=True,
        )
        route_hash = r1._path_hash(route)
        binding = {
            "architecture_id": ARCHITECTURE_ID,
            "implementation_revision": IMPLEMENTATION_REVISION,
            "map_hash": ctx.map_sha256,
            "semantic_map_hash": semantic_map.semantic_map_hash,
            "query_set_hash": query_hash,
            "query_hash": canonical_hash(query.as_dict()),
            "query_id": query.query_id,
            "route_hash": route_hash,
            "roi_hash": grid_hash(allowed),
            "endpoint_pose": {"start": query.start, "goal": query.goal},
            "footprint": config["se2_guide"]["footprint"],
            "yaw_bins": 48, "motion_model": "DUBIN",
            "costmap_hash": semantic_costmap.expected_master_hash,
            "r0_only": True,
        }
        route_bindings[query.query_id] = {
            **binding, **orientation, **roi_diagnostics,
            "route_reason": reason,
            "guide_status": preference.diagnostics.get("guide_status"),
            "guide_hash": preference.diagnostics.get("guide_hash"),
            "selected_lane_labels": selected_labels,
        }
        try:
            oracle = ExplicitSE2GuideOracle(
                ctx.hospital_map, semantic_costmap.expected_master_cost,
                np.asarray(preference.lane_instance_id, dtype=np.int32),
                np.asarray(preference.lane_error_m, dtype=np.float32),
                np.asarray(preference.lane_correct_side, dtype=bool),
                selected_labels, policy=policy, binding=binding,
                reference_polylines=preference.diagnostics.get("guide_polylines_world", []),
            )
            result = oracle.search(query.query_id, query.start, query.goal)
        except ValueError as error:
            result = None
            failure_detail = str(error)
        if result is None:
            diagnostics: Dict[str, Any] = {}
            witness = False
            failure_code = "LANE_INSTANCE_DISCONNECTED"
            path_poses: Sequence[Sequence[float]] = ()
            primitive_trace: Sequence[Mapping[str, Any]] = ()
        else:
            diagnostics = result.diagnostics
            witness = bool(result.witness_exists)
            failure_code = result.failure_code
            failure_detail = result.failure_detail
            path_poses = result.path
            primitive_trace = result.primitive_trace
        gate_passed = bool(
            witness
            and diagnostics.get("semantic_metric_gate_passed") is True
            and diagnostics.get("path_collision_free") is True
            and float(diagnostics.get("reverse_distance_m", float("inf"))) <= 0.0
            and int(diagnostics.get("in_place_rotation_count", 1)) == 0
            and float(diagnostics.get("maximum_curvature_1pm", float("inf"))) <= 2.50
            and tuple(path_poses[-1][:2]) == tuple(float(value) for value in query.goal[:2])
        )
        row = {
            "query_id": query.query_id,
            "category": query.category,
            "witness_exists": witness,
            "failure_code": failure_code,
            "failure_detail": failure_detail,
            "gate_passed": gate_passed,
            "route_hash": route_hash,
            "roi_hash": binding["roi_hash"],
            "costmap_hash": semantic_costmap.expected_master_hash,
            "binding_hash": diagnostics.get("binding_hash", ""),
            "start_yaw_bin": diagnostics.get("start_yaw_bin", ""),
            "goal_yaw_bin": diagnostics.get("goal_yaw_bin", ""),
            "lane_correct_side_ratio": diagnostics.get("lane_correct_side_ratio", ""),
            "lane_target_error_p50_m": diagnostics.get("lane_target_error_p50_m", ""),
            "lane_target_band_ratio": diagnostics.get("lane_target_band_ratio", ""),
            "maximum_curvature_1pm": diagnostics.get("maximum_curvature_1pm", ""),
            "reverse_distance_m": diagnostics.get("reverse_distance_m", ""),
            "in_place_rotation_count": diagnostics.get("in_place_rotation_count", ""),
            "minimum_collision_margin_m": diagnostics.get("minimum_collision_margin_m", ""),
            "path_length_m": diagnostics.get("path_length_m", ""),
            "expanded_state_count": diagnostics.get("expanded_state_count", ""),
            "wall_ms": diagnostics.get("wall_ms", (time.monotonic() - query_started) * 1000.0),
            "peak_rss_bytes": _rss_bytes(),
            "relaxation_level": "R0",
        }
        rows.append(row)
        detailed.append({
            "query": query.as_dict(), "row": row,
            "route_binding": route_bindings[query.query_id],
            "preference_diagnostics": preference.diagnostics,
            "semantic_costmap_diagnostics": semantic_costmap.diagnostics,
            "oracle_diagnostics": diagnostics,
        })
        _write_json(output / "paths" / f"E5-r4-oracle_{query.query_id}.json", [
            {"x": float(pose[0]), "y": float(pose[1]), "yaw": float(pose[2])}
            for pose in path_poses
        ])
        for item in primitive_trace:
            traces.append({"query_id": query.query_id, **dict(item)})
        costs.append({"query_id": query.query_id, **dict(diagnostics.get("cost_breakdown", {}))})
        _write_overlay(
            output / "overlays" / f"{query.query_id}.png", ctx.hospital_map,
            query, route, preference, path_poses,
            _old_e4_path(args.r3_targeted_result.resolve(), query.query_id),
            f"{query.query_id}: {'WITNESS' if witness else failure_code}",
        )

    _write_csv(output / "se2_oracle_results.csv", rows)
    _write_csv(output / "runs.csv", rows)
    _write_csv(output / "primitive_trace.csv", traces)
    _write_csv(output / "cost_breakdown.csv", costs)
    _write_json(output / "se2_oracle_results.json", detailed)
    _write_json(output / "route_bindings.json", route_bindings)
    all_passed = bool(len(rows) == 3 and all(row["gate_passed"] for row in rows))
    gate = {
        "stage": "stage2_same_primitive_offline_oracle",
        "required_query_ids": list(frozen["targeted_query_ids"]),
        "all_targeted_witnesses_exist": all_passed,
        "gate_passed": all_passed,
        "decision": "PROCEED_TO_STAGE3_E5" if all_passed else "C1",
        "online_e5": "PENDING" if all_passed else "NOT_RUN_STAGE2_GATE_FAILED",
        "selected8": "NOT_RUN_PENDING_STAGE4" if all_passed else "NOT_RUN_STAGE2_GATE_FAILED",
    }
    _write_json(output / "gate_results.json", gate)
    _write_json(output / "performance_summary.json", {
        "scope": "offline_oracle_only", "query_count": len(rows),
        "wall_ms_by_query": {row["query_id"]: row["wall_ms"] for row in rows},
        "peak_rss_bytes": max((int(row["peak_rss_bytes"]) for row in rows), default=_rss_bytes()),
        "online_latency": "NOT_APPLICABLE",
    })
    _write_json(output / "exact_ack_summary.json", {
        "status": "NOT_APPLICABLE_OFFLINE_STAGE2",
        "expected_effective_mapping": "pinned_humble_propagation_exact_v1",
        "server_content_ack_performed": False,
        "online_e5_gate": "PENDING" if all_passed else "NOT_RUN_STAGE2_GATE_FAILED",
    })
    (output / "final_report.md").write_text(_stage2_report(rows, all_passed), encoding="utf-8")
    for stage in ((3, "online E5 implementation"), (4, "targeted online ROS"), (5, "frozen selected8")):
        if not all_passed:
            (output / f"STAGE{stage[0]}_NOT_RUN_STAGE2_GATE_FAILED.md").write_text(
                f"# {stage[1]}\n\nNOT_RUN_STAGE2_GATE_FAILED. Stage 2 decision: C1.\n",
                encoding="utf-8",
            )
    (output / "runner_stdout.txt").write_text(
        f"stage2_gate_passed={all_passed}\ndecision={gate['decision']}\n",
        encoding="utf-8",
    )
    (output / "runner_stderr.txt").write_text("", encoding="utf-8")
    (output / "process_audit_after.txt").write_text(_process_audit(), encoding="utf-8")
    _write_json(output / "verification.json", {
        "runner_completed": True, "stage2_gate_passed": all_passed,
        "online_e5_started": False, "ros_processes_started": False,
        "query_endpoints_unchanged": True, "yaw_bins": 48,
        "pinned_nav2_modified_by_runner": False,
    })
    reproduction = "two_layer_v2_semantic_r4_benchmark " + " ".join(
        shlex.quote(value) for value in arguments
    )
    sources = (
        Path(__file__), Path(__file__).with_name("se2_semantic_guide.py"),
        args.config.resolve(), args.query_set.resolve(), DEFAULT_SELECTED8,
        Path(__file__).with_name("regional_preference_r3.py"),
        Path(__file__).with_name("semantic_costmap_r2.py"),
        PACKAGE_ROOT / "test/test_two_layer_v2_semantic_r4.py",
        PACKAGE_ROOT / "setup.py",
    )
    _snapshot_and_manifest(output, reproduction, sources)
    print(json.dumps(gate, indent=2, sort_keys=True))
    return 0 if all_passed else 2


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run PLN-02 2A-V2/r4 explicit 48-bin SE(2) feasibility gates",
    )
    parser.add_argument("--mode", choices=("se2-oracle", "validate-config"), default="se2-oracle")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--query-set", type=Path, default=DEFAULT_TARGETS)
    parser.add_argument("--extracted-dir", type=Path)
    parser.add_argument("--semantic-map", type=Path)
    parser.add_argument("--topology-cache", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--r3-targeted-result", type=Path, default=DEFAULT_R3_RESULT)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    args = _parser().parse_args(arguments)
    config = _load_config(args.config.resolve())
    if args.mode == "validate-config":
        print(json.dumps({
            "architecture_id": config["architecture_id"],
            "implementation_revision": config["implementation_revision"],
            "protocol_id": config["protocol_id"],
            "se2_policy_hash": SE2GuidePolicy.from_mapping(config["se2_guide"]).policy_hash,
        }, indent=2, sort_keys=True))
        return 0
    missing = [
        name for name in ("extracted_dir", "semantic_map", "topology_cache", "output_dir")
        if getattr(args, name) is None
    ]
    if missing:
        raise SystemExit("se2-oracle requires " + ", ".join(f"--{name.replace('_', '-')}" for name in missing))
    return run_stage2(args, arguments)


if __name__ == "__main__":
    raise SystemExit(main())
