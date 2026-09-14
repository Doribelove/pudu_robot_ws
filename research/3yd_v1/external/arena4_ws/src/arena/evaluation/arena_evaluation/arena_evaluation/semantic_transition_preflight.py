"""Replay all three frozen witnesses before enabling the transition contract.

This entry point is offline only. It verifies control replay and input identity
in addition to the saved-pose verifier, and never launches a planner or ROS.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys
import time

import numpy as np
import yaml

from .semantic_constraint_core import ConstraintWorld, dubins_edge, replay_edge
from .semantic_map import SemanticMapV1, sha256_file
from .semantic_query_defaults import DEFAULT_QUERY_SET_FILENAME, load_query_set
from .semantic_transition_online_adapter import _load_points, _semantic_metrics, run as replay_saved_path


REQUIRED = ("r3-mirror-1-positive", "r3-mirror-2-negative", "cmp2-02-lane-south")
CONTRACT_REVISION = "semantic-endpoint-transition-6m-r1"
TARGETED_HASH = "66212b05ef6c4d16eaedafc1c27866387c3d74aa85bf5ac69bca92487254667d"
WITNESSES = (
    ("next_arch_r1_optimize_dense_all_positive_20260907T120000Z/path.json",
     "next_arch_r1_optimize_dense_all_positive_20260907T120000Z/result.json", "waypoints"),
    ("constraint_stage1_delivery_final_20260907T030500Z/paths/constraint_independent_aux_negative_20260907T023821Z_path.json",
     "constraint_stage1_delivery_final_20260907T030500Z/paths/constraint_independent_aux_negative_20260907T023821Z_certificate.json", "certificate"),
    ("constraint_stage1_delivery_final_20260907T030500Z/paths/constraint_independent_aux_south_20260907T023821Z_path.json",
     "constraint_stage1_delivery_final_20260907T030500Z/paths/constraint_independent_aux_south_20260907T023821Z_certificate.json", "certificate"),
)


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def replay_controls(path_file: Path, controls_file: Path, kind: str) -> dict:
    payload = json.loads(controls_file.read_text())
    if kind == "certificate":
        certificates = payload["edges"]
        edges = [replay_edge(item) for item in certificates]
        hashes_match = all(
            hashlib.sha256(edge.samples.tobytes()).hexdigest() == cert["sample_hash"]
            for edge, cert in zip(edges, certificates)
        )
    elif kind == "waypoints":
        waypoints = payload["raw_waypoints"]
        edges = [dubins_edge(a, b, radius=0.401) for a, b in zip(waypoints, waypoints[1:])]
        hashes_match = True
    else:
        raise ValueError(f"unsupported control certificate: {kind}")
    if not edges or any(edge is None for edge in edges):
        raise ValueError("empty or unreplayable controls")
    path = np.vstack((edges[0].start, *(edge.samples for edge in edges)))
    original = np.asarray([[row[k] for k in ("x", "y", "yaw")] for row in _load_points(path_file)])
    equal = bool(np.array_equal(path, original))
    continuous = all(np.array_equal(a.samples[-1], b.start) for a, b in zip(edges, edges[1:]))
    forward = all(edge.radius >= 0.40 and all(p >= 0.0 for p in edge.params) for edge in edges)
    return {
        "control_replay_passed": bool(equal and continuous and forward and hashes_match),
        "saved_path_matches_replay_exactly": equal,
        "edge_continuity": continuous,
        "forward_only_radius_passed": forward,
        "certificate_sample_hashes_match": hashes_match,
        "arc_length_m": sum(edge.length for edge in edges),
        "replay_pose_count": len(path),
        "edges": [edge.certificate() for edge in edges],
    }


def aggregate(rows: list[dict]) -> dict:
    ids = tuple(row["query_id"] for row in rows)
    complete = ids == REQUIRED
    passed = complete and all(row.get("offline_gate_passed") is True for row in rows)
    return {
        "architecture_id": "2A-V2", "contract_revision": CONTRACT_REVISION,
        "protocol_id": "PLN-02-SEMANTIC-ENDPOINT-TRANSITION-R1-V1",
        "implementation_revision": "r1-offline-transition-preflight-audited",
        "required_query_ids": list(REQUIRED), "actual_query_ids": list(ids),
        "required_queries_present_exactly_once": complete,
        "offline_pass_count": sum(row.get("offline_gate_passed") is True for row in rows),
        "offline_all_three_passed": bool(passed),
        "targeted_offline_gate_passed": bool(passed),
        "online_eligible": bool(passed),
        "promotion_gate_passed": False,
        "grade": "NOT_GRADED_ONLINE_NOT_RUN" if passed else "C1",
        "failure_scope": "frozen_witness_replay_not_continuous_infeasibility_proof",
        "online_planner": "NOT_RUN_OFFLINE_GATE_FAILED" if not passed else "NOT_IMPLEMENTED",
        "three_arm_48_bin_comparison": "NOT_RUN",
        "exact_server_ack": "NOT_APPLICABLE_OFFLINE",
        "selected8": "NOT_RUN", "cold_latency": "NOT_RUN",
        "per_query": rows,
    }


def plot_windows(rows: list[dict], output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(10, 6), constrained_layout=True)
    for ax, row, name in zip(axes, rows, ("positive", "negative", "south")):
        length = row["path_length_m"]
        ax.broken_barh([(0.0, length)], (0.0, 1.0), facecolors="#e7e7e7")
        if length > 12.0:
            ax.broken_barh([(6.0, length - 12.0)], (0.0, 1.0), facecolors="#177f72")
        else:
            ax.broken_barh([(0.0, length)], (0.0, 1.0), facecolors="#bf4145", hatch="//")
        ax.set_xlim(0, max(14.0, length + 1.0))
        ax.set_ylim(-0.2, 1.4)
        ax.set_yticks([])
        ax.set_xlabel("Path arc length (m)")
        status = "PASS" if row["offline_gate_passed"] else row["contract_status"]
        ax.set_title(f"{name}: L={length:.3f} m; active samples={row['active_samples']}; {status}", fontsize=11)
    fig.suptitle("Frozen paths: grey = excluded; green = semantic window; red = empty window", fontsize=12)
    fig.savefig(output / "transition_windows.png", dpi=160)
    plt.close(fig)


def run(workspace: Path, output: Path) -> dict:
    output.mkdir(parents=False, exist_ok=False)
    package = workspace / "external/arena4_ws/src/arena/evaluation/arena_evaluation"
    configs = package / "config"
    config = configs / "pudu_wanda_3f_semantic_endpoint_transition_r1.yaml"
    contract = yaml.safe_load(config.read_text())
    if (contract["contract_revision"] != CONTRACT_REVISION
            or contract["semantic_sampling"]["endpoint_transition_each_m"] != 6.0
            or contract["semantic_sampling"]["sample_spacing_m"] != 0.025):
        raise ValueError("unsupported contract revision or sampling configuration")
    data = workspace / "private_data/pudu_wanda_3f"
    results = data / "results"
    inputs = results / "constraint_inputs_20260907T021418Z"
    map_file = data / "extracted/optemap.pgm"
    semantic_file = results / "conversion_v1/semantic_map_v1.json"
    actual_map = sha256_file(map_file)
    actual_semantic = SemanticMapV1.load(semantic_file).semantic_map_hash
    if actual_map != contract["map_hash"] or actual_semantic != contract["semantic_map_hash"]:
        raise ValueError("frozen map/semantic map identity mismatch")
    query_file = configs / "pudu_wanda_3f_r3_targeted_preflight3_v1.yaml"
    queries, _, targeted = load_query_set(query_file, actual_map_hash=actual_map, actual_semantic_map_hash=actual_semantic)
    if targeted["query_hash"] != TARGETED_HASH or tuple(q.query_id for q in queries) != REQUIRED:
        raise ValueError("targeted queries changed")
    _, _, selected8 = load_query_set(configs / DEFAULT_QUERY_SET_FILENAME, actual_map_hash=actual_map,
                                     actual_semantic_map_hash=actual_semantic, require_default_contract=True)
    manifest = {
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "scope": "offline_control_and_saved_path_replay_only; no_new_search",
        "query_override_reason": "user_authorized_frozen_targeted3_contract_preflight",
        "map_hash": actual_map, "semantic_map_hash": actual_semantic,
        "contract_config_sha256": sha256_file(config), "targeted": targeted,
        "selected8_identity_check": selected8, "python": sys.version,
        "argv": sys.argv, "source_sha256": {}, "witnesses": [],
    }
    source_dir = output / "source_snapshot"
    source_dir.mkdir()
    for name in ("semantic_transition_preflight.py", "semantic_transition_online_adapter.py",
                 "semantic_constraint_core.py", "se2_semantic_guide.py", "path_audit.py", "semantic_query_defaults.py"):
        source = package / "arena_evaluation" / name
        shutil.copy2(source, source_dir / name)
        manifest["source_sha256"][name] = sha256_file(source)
    shutil.copy2(config, output / config.name)
    shutil.copy2(query_file, output / query_file.name)
    rows = []
    for query, (path_rel, control_rel, kind) in zip(queries, WITNESSES):
        path_file, controls_file = results / path_rel, results / control_rel
        meta_path = inputs / f"{query.query_id}.json"
        meta = json.loads(meta_path.read_text())
        if (meta["targeted_query_content_hash"] != TARGETED_HASH
                or meta["map_hash"] != actual_map or meta["semantic_map_hash"] != actual_semantic
                or meta["query"]["start"] != list(query.start) or meta["query"]["goal"] != list(query.goal)
                or meta["preference_diagnostics"].get("relaxation_level") != "R0"):
            raise ValueError(f"input binding mismatch: {query.query_id}")
        controls = replay_controls(path_file, controls_file, kind)
        item_dir = output / query.query_id
        replay = replay_saved_path(inputs=inputs, query=query.query_id, path=path_file,
                                   output=item_dir, contract_config=config)
        world = ConstraintWorld(inputs, query.query_id)
        full = _semantic_metrics(world, _load_points(path_file), 0.0)
        semantic = replay["semantic"]
        write_json(output / f"{query.query_id}_controls.json", controls)
        write_json(output / f"{query.query_id}_full_metrics.json", full)
        row = {
            "query_id": query.query_id, "path_length_m": semantic["path_length_m"],
            "canonical_valid": replay["canonical_final_valid_success"],
            "control_replay_passed": controls["control_replay_passed"],
            "correct_side_ratio": semantic["correct_side_ratio"],
            "target_band_ratio": semantic["target_band_ratio"],
            "lateral_error_p50_m": semantic["lateral_error_p50_m"],
            "active_samples": semantic["applicable_sample_count"],
            "contract_status": semantic["contract_metric_status"],
            "failure_reason": semantic["invalid_contract_metric_reason"] or semantic["failure_code"],
            "full_path_correct_side_ratio": full["correct_side_ratio"],
            "full_path_target_band_ratio": full["target_band_ratio"],
            "full_path_lateral_error_p50_m": full["lateral_error_p50_m"],
            "offline_gate_passed": bool(replay["replay_final_valid_success"] and controls["control_replay_passed"]),
        }
        rows.append(row)
        manifest["witnesses"].append({
            "query_id": query.query_id, "path_file": str(path_file), "path_sha256": sha256_file(path_file),
            "controls_file": str(controls_file), "controls_sha256": sha256_file(controls_file),
            "input_meta_sha256": sha256_file(meta_path), "input_npz_sha256": meta["npz_sha256"],
            "expected_master_hash": meta["expected_master_hash"], "route_hash": meta["route_hash"],
            "roi_hash": meta["roi_hash"],
        })
    gate = aggregate(rows)
    write_json(output / "gate.json", gate)
    write_json(output / "manifest.json", manifest)
    with (output / "per_query.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    plot_windows(rows, output)
    write_json(output / "artifact_hashes.json", {
        str(path.relative_to(output)): sha256_file(path) for path in sorted(output.rglob("*")) if path.is_file()
    })
    return gate


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    gate = run(args.workspace.resolve(), args.output.resolve())
    print(json.dumps(gate, indent=2, allow_nan=False))
    return 0 if gate["offline_all_three_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
