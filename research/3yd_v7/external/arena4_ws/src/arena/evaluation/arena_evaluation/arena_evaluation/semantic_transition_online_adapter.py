"""Offline replay verifier for the endpoint-transition semantic contract.

The historical module name is retained for import compatibility only. This
verifier consumes a saved SE(2) path. It reuses the frozen canonical
``PathAuditor`` for safety and applies the proposed six metre endpoint mask
only to semantic applicability metrics. No online planner runs, ROS node is
started, or navigation message is published by this entry point.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import platform
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from .planner_benchmark.map_utils import HospitalMap
from .path_audit import PathAuditor
from .semantic_constraint_core import ConstraintWorld, PADDED_FOOTPRINT, dense_interpolate
from .semantic_map import sha256_file


PROTOCOL_ID = "PLN-02-SEMANTIC-TRANSITION-OFFLINE-REPLAY-R0-V1"
CONTRACT_ID = "endpoint-transition-each-side-6m-diagnostic-candidate"


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    return value


def _load_points(path_file: Path) -> list[dict[str, Any]]:
    raw = json.loads(path_file.read_text(encoding="utf-8"))
    if isinstance(raw, Mapping):
        raw = raw.get("poses", raw.get("path", []))
    if not isinstance(raw, list) or not raw:
        raise ValueError("reference path must be a non-empty JSON pose list")
    points: list[dict[str, Any]] = []
    for row in raw:
        point = {
            "x": float(row["x"]), "y": float(row["y"]), "yaw": float(row["yaw"]),
            "velocity": float(row.get("velocity", 0.0)),
            "steering": float(row.get("steering", 0.0)),
            "source": str(row.get("source", "semantic_transition_offline_replay")),
            "motion_direction": str(row.get("motion_direction", "forward")),
            "planner_backend": str(row.get("planner_backend", "semantic_transition_reference")),
            "backend_version": str(row.get("backend_version", "r0")),
        }
        points.append(point)
    return points


def _full_auditor(world: ConstraintWorld) -> tuple[PathAuditor, np.ndarray]:
    desc = world.meta["map"]
    yaml_path = Path(desc["image_path"]).with_suffix(".yaml")
    full_map = HospitalMap.load(yaml_path)
    if full_map.sha256 != world.meta["map_hash"]:
        raise ValueError("canonical map hash differs from the frozen input")
    if (full_map.width != desc["width"] or full_map.height != desc["height"]
            or full_map.resolution != desc["resolution"]
            or tuple(full_map.origin) != tuple(desc["origin"])):
        raise ValueError("canonical map geometry differs from the frozen input")
    with np.load(world.input_path) as data:
        allowed = np.asarray(data["allowed"], dtype=bool)
    return PathAuditor(SimpleNamespace(hospital_map=full_map),
                       source_commit="semantic_transition_offline_replay_r0",
                       footprint=PADDED_FOOTPRINT), allowed


def _resample_points(points: Sequence[Mapping[str, Any]], spacing_m: float) -> tuple[list[dict[str, float]], np.ndarray]:
    """Resample a path at fixed arc-length spacing, retaining both endpoints."""
    if not points:
        raise ValueError("path is empty")
    if not math.isfinite(spacing_m) or spacing_m <= 0.0:
        raise ValueError("sample spacing must be finite and positive")
    if len(points) < 2:
        return [{"x": float(points[0]["x"]), "y": float(points[0]["y"]), "yaw": float(points[0]["yaw"])}], np.asarray([0.0])
    xy = np.asarray([[float(p["x"]), float(p["y"])] for p in points], dtype=float)
    station = np.zeros(len(xy), dtype=float)
    station[1:] = np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))
    total = float(station[-1])
    if total <= 1.0e-12:
        return [{"x": float(points[0]["x"]), "y": float(points[0]["y"]), "yaw": float(points[0]["yaw"])}], np.asarray([0.0])
    yaw = np.unwrap(np.asarray([float(p["yaw"]) for p in points], dtype=float))
    positions = np.arange(0.0, total, max(float(spacing_m), 1.0e-6), dtype=float)
    positions = np.r_[positions, total]
    positions = np.unique(positions)
    return [{"x": float(np.interp(s, station, xy[:, 0])),
             "y": float(np.interp(s, station, xy[:, 1])),
             "yaw": float(np.interp(s, station, yaw))} for s in positions], positions


def _semantic_metrics(world: ConstraintWorld, points: Sequence[Mapping[str, Any]],
                      transition_m: float, sample_spacing_m: float = 0.025) -> dict[str, Any]:
    points, station = _resample_points(points, sample_spacing_m)
    xy = np.asarray([[float(p["x"]), float(p["y"])] for p in points], dtype=float)
    if not math.isfinite(transition_m) or transition_m < 0.0:
        raise ValueError("endpoint transition must be finite and non-negative")
    if station[-1] <= 2.0 * float(transition_m):
        return {
            "endpoint_transition_each_m": float(transition_m),
            "sample_spacing_m": float(sample_spacing_m),
            "applicable_sample_count": 0,
            "path_sample_count": int(len(points)),
            "path_length_m": float(station[-1]),
            "correct_side_ratio": None,
            "target_band_ratio": None,
            "lateral_error_p50_m": None,
            "semantic_gate_passed": False,
            "contract_metric_status": "INVALID_CONTRACT_METRIC",
            "failure_code": "INVALID_CONTRACT_METRIC",
            "invalid_contract_metric_reason": "SHORT_PATH_FOR_ENDPOINT_TRANSITION_CONTRACT",
        }
    rows, cols, inside = world.cells(xy)
    active = inside & (station >= transition_m) & (station <= station[-1] - transition_m)
    if not np.all(inside):
        return {"applicable_sample_count": int(np.count_nonzero(active)),
                "path_length_m": float(station[-1]), "semantic_gate_passed": False,
                "contract_metric_status": "INVALID_CONTRACT_METRIC",
                "failure_code": "INVALID_CONTRACT_METRIC",
                "invalid_contract_metric_reason": "PATH_OUTSIDE_SEMANTIC_CROP"}
    labels = world.grids["labels"][rows, cols]
    lane = np.isin(labels, world.selected)
    finite = np.isfinite(world.grids["right"][rows, cols])
    active &= lane & finite
    correct = world.grids["correct"][rows, cols].astype(bool)
    error = world.grids["error"][rows, cols].astype(float)
    target = correct & (error <= 0.50)
    count = int(np.count_nonzero(active))
    side = float(np.mean(correct[active])) if count else None
    band = float(np.mean(target[active])) if count else None
    p50 = float(np.median(error[active])) if count else None
    passed = bool(count and side >= 0.80 and band > 0.50 and p50 <= 0.50)
    return {
        "endpoint_transition_each_m": float(transition_m),
        "sample_spacing_m": float(sample_spacing_m),
        "applicable_sample_count": count,
        "path_sample_count": int(len(points)),
        "path_length_m": float(station[-1]),
        "correct_side_ratio": side,
        "target_band_ratio": band,
        "lateral_error_p50_m": p50,
        "semantic_gate_passed": passed,
        "contract_metric_status": "VALID_CONTRACT_METRIC" if count else "INVALID_CONTRACT_METRIC",
        "failure_code": "" if passed else ("SEMANTIC_GATE_FAILED" if count else "INVALID_CONTRACT_METRIC"),
        "invalid_contract_metric_reason": "" if count else "EMPTY_APPLICABLE_REGION",
    }


def _reference_path_payload(points: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Return plain replay JSON; this payload is not a ROS message."""
    return {
        "schema": "offline_se2_reference_pose_list_v1",
        "frame_id": "map",
        "poses": [{"x": float(p["x"]), "y": float(p["y"]), "yaw": float(p["yaw"])} for p in points],
    }


def to_nav_msgs_path(points: Sequence[Mapping[str, Any]], *, frame_id: str = "map"):
    """Build the actual ROS message when the pinned ROS environment is active."""
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as NavPath

    message = NavPath()
    message.header.frame_id = frame_id
    for point in points:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(point["x"])
        pose.pose.position.y = float(point["y"])
        pose.pose.orientation.z = math.sin(float(point["yaw"]) * 0.5)
        pose.pose.orientation.w = math.cos(float(point["yaw"]) * 0.5)
        message.poses.append(pose)
    return message


def run(*, inputs: Path, query: str, path: Path, output: Path,
        transition_m: float = 6.0, contract_config: Path | None = None) -> dict[str, Any]:
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"write-once output is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic_ns()
    world = ConstraintWorld(inputs, query)
    points = _load_points(path)
    contract = {}
    if contract_config is not None:
        contract_config = Path(contract_config)
        contract = yaml.safe_load(contract_config.read_text(encoding="utf-8")) or {}
        declared = float(contract.get("semantic_sampling", {}).get("endpoint_transition_each_m", transition_m))
        if abs(declared - float(transition_m)) > 1.0e-9:
            raise ValueError("--transition-m conflicts with contract config")
        transition_m = declared
        sample_spacing_m = float(contract.get("semantic_sampling", {}).get("sample_spacing_m", 0.025))
    else:
        sample_spacing_m = 0.025
    canonical, allowed = _full_auditor(world)
    canonical_result = canonical.audit(world.query, points, allowed)
    poses = np.asarray([[p["x"], p["y"], p["yaw"]] for p in points], dtype=float)
    dense_poses = dense_interpolate(poses)
    effective_safe = bool(world.collision_free(dense_poses))
    exact_endpoints = bool(np.array_equal(poses[0], world.start) and np.array_equal(poses[-1], world.goal))
    endpoint_cell = world.map.world_to_cell(*world.goal[:2])
    no_stopping_goal = bool(endpoint_cell is None or world.grids["no_stopping"][endpoint_cell])
    hard_audit = {
        "canonical_full_path_passed": canonical_result.final_valid_success,
        "padded_effective_master_collision_free": effective_safe,
        "exact_endpoint_xy_yaw": exact_endpoints,
        "same_lane_instance_full_path": world.semantic_counts(dense_poses) is not None,
        "no_stopping_goal_violation": no_stopping_goal,
        "complete_padded_footprint": [list(vertex) for vertex in PADDED_FOOTPRINT],
        "dense_safety_pose_count": len(dense_poses),
        "maximum_curvature_strict_passed": bool(canonical_result.metrics["maximum_curvature"] <= 2.50),
        "path_length_bound_m": float(world.bound_length),
        "path_length_bound_passed": bool(canonical_result.metrics["path_length_m"] <= world.bound_length),
        "relaxation_level": "R0",
        "relaxation_binding_scope": "frozen_ConstraintWorld_input_allowed_roi_and_expected_master",
        "control_sequence_replay_verified": False,
        "control_sequence_replay_status": "NOT_CHECKED_SAVED_POSE_REPLAY_ONLY",
    }
    semantic = _semantic_metrics(world, points, float(transition_m), sample_spacing_m)
    reference_path = _reference_path_payload(points)
    path_sha = sha256_file(path)
    manifest = {
        "protocol_id": PROTOCOL_ID,
        "contract_id": CONTRACT_ID,
        "architecture_id": "UNNAMED_CONTRACT_CANDIDATE",
        "scope": "offline_saved_reference_replay_only",
        "online_planner_run": False,
        "ros_node_started": False,
        "nav_msgs_path_published": False,
        "query": query,
        "inputs": str(Path(inputs).resolve()),
        "path_file": str(Path(path).resolve()),
        "path_sha256": path_sha,
        "input_npz_sha256": world.meta["npz_sha256"],
        "map_hash": world.meta["map_hash"],
        "semantic_map_hash": world.meta.get("semantic_map_hash"),
        "transition_m_each_side": float(transition_m),
        "sample_spacing_m": float(sample_spacing_m),
        "contract_config": str(contract_config.resolve()) if contract_config else None,
        "contract_config_sha256": sha256_file(contract_config) if contract_config else None,
        "contract_revision": contract.get("contract_revision") if contract else None,
        "python": sys.version,
        "platform": platform.platform(),
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    result = {
        "protocol_id": PROTOCOL_ID,
        "contract_id": CONTRACT_ID,
        "query": query,
        "path_sha256": path_sha,
        "canonical": canonical_result.metrics,
        "canonical_diagnostics": canonical_result.diagnostics(),
        "canonical_final_valid_success": canonical_result.final_valid_success,
        "full_path_safety_audit": hard_audit,
        "full_path_invariants": hard_audit,
        "semantic": semantic,
        "reference_path": reference_path,
        "offline_replay_gate_passed": bool(canonical_result.final_valid_success and effective_safe
                                          and exact_endpoints and not no_stopping_goal
                                          and hard_audit["same_lane_instance_full_path"]
                                          and hard_audit["maximum_curvature_strict_passed"]
                                          and hard_audit["path_length_bound_passed"]
                                          and semantic["semantic_gate_passed"]),
        "online_planner_run": False,
        "ros_node_started": False,
        "nav_msgs_path_published": False,
        "wall_ms": (time.monotonic_ns() - started) / 1.0e6,
    }
    result["replay_final_valid_success"] = result["offline_replay_gate_passed"]
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True, default=_jsonable) + "\n", encoding="utf-8")
    (output / "path.json").write_text(json.dumps(points, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reference_poses.json").write_text(json.dumps(reference_path, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "result.json").write_text(json.dumps(result, indent=2, sort_keys=True, default=_jsonable) + "\n", encoding="utf-8")
    with (output / "result.csv").open("w", newline="", encoding="utf-8") as stream:
        fields = ["query", "canonical_final_valid_success", "offline_replay_gate_passed",
                  "correct_side_ratio", "target_band_ratio", "lateral_error_p50_m",
                  "path_length_m", "wall_ms"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerow({
            "query": query,
            "canonical_final_valid_success": canonical_result.final_valid_success,
            "offline_replay_gate_passed": result["offline_replay_gate_passed"],
            **{key: semantic.get(key) for key in fields if key in semantic},
            "wall_ms": result["wall_ms"],
        })
    hashes = {}
    for item in sorted(output.iterdir()):
        if item.is_file() and item.name != "artifact_hashes.json":
            hashes[item.name] = sha256_file(item)
    (output / "artifact_hashes.json").write_text(json.dumps(hashes, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Offline replay verifier for saved endpoint-transition reference paths")
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--query", required=True)
    parser.add_argument("--path", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--transition-m", type=float, default=6.0)
    parser.add_argument("--contract-config", type=Path, default=None)
    args = parser.parse_args(argv)
    print(json.dumps(run(inputs=args.inputs, query=args.query, path=args.path,
                         output=args.output, transition_m=args.transition_m,
                         contract_config=args.contract_config),
                    indent=2, sort_keys=True, default=_jsonable))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
