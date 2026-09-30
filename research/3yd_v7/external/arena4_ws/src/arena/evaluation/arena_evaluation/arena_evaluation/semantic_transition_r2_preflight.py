"""Strict offline replay for the approved short-full/long-trim contract."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import shutil
import time

import numpy as np
import yaml

from .semantic_constraint_core import ConstraintWorld, dense_interpolate
from .semantic_map import SemanticMapV1, sha256_file, point_in_polygon
from .semantic_path_audit import SemanticPathAuditor
from .semantic_path_revisit_audit import audit_revisits
from .semantic_query_defaults import DEFAULT_QUERY_SET_PATH, load_query_set
from .semantic_transition_contract import TransitionContractR2, audit_transition_samples, resample_path
from .semantic_transition_online_adapter import _load_points, run as replay_saved_path
from .semantic_transition_preflight import REQUIRED, TARGETED_HASH, WITNESSES, replay_controls, write_json


def semantic_metrics(world, points):
    samples, station = resample_path(points)
    samples = np.asarray([[p[k] for k in ("x", "y", "yaw")] for p in samples])
    rows, cols, inside = world.cells(samples)
    if not np.all(inside):
        raise ValueError("path outside frozen semantic input crop")
    return audit_transition_samples(
        path_length_m=float(station[-1]), station_m=station,
        lane_mask=np.isin(world.grids["labels"][rows, cols], world.selected),
        lane_error_m=world.grids["error"][rows, cols],
        lane_correct_side=world.grids["correct"][rows, cols],
        raw_xy=[[p["x"], p["y"]] for p in points],
    )


def explicit_hard_audit(world, points, semantic_map):
    """Check explicit directions from actual features, including both endpoints."""
    poses = np.asarray([[p[k] for k in ("x", "y", "yaw")] for p in points])
    dense = dense_interpolate(poses)
    endpoints = [world.map.world_to_cell(*p[:2]) for p in (world.start, world.goal)]
    no_stop = sum(cell is None or bool(world.grids["no_stopping"][cell]) for cell in endpoints)
    explicit = [f for f in semantic_map.features if f.direction_rule == "explicit"]
    wrong_distance = 0.0
    unsupported = []
    for feature in explicit:
        direction = SemanticPathAuditor._explicit_direction(feature.properties.get("explicit_direction"))
        if direction is None or feature.geometry_type != "polygon":
            unsupported.append(feature.semantic_id)
            continue
        for a, b in zip(dense, dense[1:]):
            delta = b[:2] - a[:2]
            midpoint = (a[:2] + b[:2]) * 0.5
            if point_in_polygon(midpoint, feature.coordinates) and float(np.dot(delta, direction)) < -1e-12:
                wrong_distance += float(np.linalg.norm(delta))
    return {
        "semantic_features_hash": semantic_map.semantic_map_hash,
        "explicit_feature_count": len(explicit), "unsupported_explicit_features": unsupported,
        "explicit_wrong_way_distance_m": wrong_distance,
        "no_stopping_task_endpoint_violations": no_stop,
        "hard_feature_gate_passed": not unsupported and no_stop == 0 and wrong_distance == 0.0,
        "scope": "full_path_features_and_start_goal; not_runtime_stopping_guarantee",
    }


def full_gate(replay, controls, hard, metrics, revisits=None):
    inv = replay["full_path_invariants"]
    keys = ("canonical_full_path_passed", "padded_effective_master_collision_free",
            "exact_endpoint_xy_yaw", "same_lane_instance_full_path", "maximum_curvature_strict_passed",
            "path_length_bound_passed")
    return bool(all(inv.get(k) is True for k in keys)
                and not inv["no_stopping_goal_violation"]
                and inv["relaxation_level"] == "R0"
                and controls["control_replay_passed"]
                and controls["arc_length_m"] <= inv["path_length_bound_m"]
                and hard["hard_feature_gate_passed"] and metrics["semantic_gate_passed"]
                and revisits is not None and revisits["revisit_screen_passed"])


def run(workspace, output):
    output.mkdir(parents=False, exist_ok=False)
    package = workspace / "external/arena4_ws/src/arena/evaluation/arena_evaluation"
    config = package / "config/pudu_wanda_3f_semantic_endpoint_transition_r2.yaml"
    policy = yaml.safe_load(config.read_text())
    if policy["contract_revision"] != TransitionContractR2().contract_revision:
        raise ValueError("contract identity mismatch")
    data = workspace / "private_data/pudu_wanda_3f"
    results = data / "results"
    previous = results / "transition_preflight3_r1_20260907T050200Z/manifest.json"
    frozen = json.loads(previous.read_text())
    semantic_map = SemanticMapV1.load(results / "conversion_v1/semantic_map_v1.json")
    map_hash = sha256_file(data / "extracted/optemap.pgm")
    if map_hash != policy["inputs"]["map_hash"] or semantic_map.semantic_map_hash != policy["inputs"]["semantic_map_hash"]:
        raise ValueError("frozen map mismatch")
    query_file = package / "config/pudu_wanda_3f_r3_targeted_preflight3_v1.yaml"
    queries, _, query_meta = load_query_set(query_file, actual_map_hash=map_hash,
                                          actual_semantic_map_hash=semantic_map.semantic_map_hash)
    if tuple(q.query_id for q in queries) != REQUIRED or query_meta["query_hash"] != TARGETED_HASH:
        raise ValueError("frozen targeted queries mismatch")
    _, _, selected8 = load_query_set(DEFAULT_QUERY_SET_PATH, actual_map_hash=map_hash,
                                    actual_semantic_map_hash=semantic_map.semantic_map_hash, require_default_contract=True)
    inputs = results / "constraint_inputs_20260907T021418Z"
    rows = []
    for query, witness, binding in zip(queries, WITNESSES, frozen["witnesses"]):
        path, cert, kind = results / witness[0], results / witness[1], witness[2]
        meta = inputs / f"{query.query_id}.json"
        archive = inputs / f"{query.query_id}.npz"
        checks = ((path, "path_sha256"), (cert, "controls_sha256"),
                  (meta, "input_meta_sha256"), (archive, "input_npz_sha256"))
        if binding["query_id"] != query.query_id or any(sha256_file(p) != binding[k] for p, k in checks):
            raise ValueError(f"frozen witness binding drift: {query.query_id}")
        world = ConstraintWorld(inputs, query.query_id)
        if world.start != tuple(query.start) or world.goal != tuple(query.goal):
            raise ValueError("exact query pose mismatch")
        controls = replay_controls(path, cert, kind)
        points = _load_points(path)
        replay = replay_saved_path(inputs=inputs, query=query.query_id, path=path,
                                   output=output / query.query_id)
        metrics = semantic_metrics(world, points)
        hard = explicit_hard_audit(world, points, semantic_map)
        revisits = audit_revisits([[p[k] for k in ("x", "y", "yaw")] for p in points])
        strict = full_gate(replay, controls, hard, metrics, revisits)
        write_json(output / f"{query.query_id}_audit.json", {
            "contract_revision": policy["contract_revision"], "metrics": metrics,
            "hard_features": hard, "controls": controls, "full_path_invariants": replay["full_path_invariants"],
            "revisit_audit": revisits,
            "control_replay_verified_by_outer_audit": True, "strict_offline_gate_passed": strict,
        })
        lane = metrics["active_window"]["classes"]["lane"]
        rows.append({"query_id": query.query_id, "path_length_m": metrics["path_length_m"],
                     "short_path_full_semantics": metrics["short_path_full_semantics"],
                     "correct_side_ratio": lane["correct_side_ratio"], "target_band_ratio": lane["target_band_ratio"],
                     "lateral_error_p50_m": lane["lateral_error_p50_m"], "active_samples": lane["sample_count"],
                     "revisit_screen_passed": revisits["revisit_screen_passed"],
                     "strict_offline_gate_passed": strict})
    passed = all(row["strict_offline_gate_passed"] for row in rows) and len(rows) == 3
    gate = {"architecture_id": "2A-V2", "contract_revision": policy["contract_revision"],
            "protocol_id": policy["protocol_id"], "targeted_offline_gate_passed": passed,
            "offline_pass_count": sum(r["strict_offline_gate_passed"] for r in rows),
            "online_implementation_eligible": passed, "promotion_gate_passed": False,
            "grade": "NOT_GRADED_ONLINE_NOT_RUN" if passed else "C1", "per_query": rows,
            "online": "NOT_RUN", "ack": "NOT_APPLICABLE_OFFLINE", "selected8": "NOT_RUN"}
    write_json(output / "gate.json", gate)
    with (output / "per_query.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    shutil.copy2(config, output / config.name)
    write_json(output / "manifest.json", {
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "scope": "offline_witness_replay_only; does_not_prove_online_planner", "source": str(Path(__file__)),
        "source_sha256": sha256_file(Path(__file__)), "contract_config_sha256": sha256_file(config),
        "frozen_r1_manifest_sha256": sha256_file(previous), "frozen_r1_manifest": str(previous),
        "map_hash": map_hash, "semantic_map_hash": semantic_map.semantic_map_hash,
        "targeted": query_meta, "selected8": selected8,
    })
    write_json(output / "artifact_hashes.json", {str(p.relative_to(output)): sha256_file(p)
               for p in sorted(output.rglob("*")) if p.is_file()})
    return gate


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    gate = run(args.workspace.resolve(), args.output.resolve())
    print(json.dumps(gate, indent=2))
    return 0 if gate["targeted_offline_gate_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
