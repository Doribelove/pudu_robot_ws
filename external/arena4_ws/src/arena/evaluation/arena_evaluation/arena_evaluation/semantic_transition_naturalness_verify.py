"""Fail-closed, write-once verification of a semantic transition candidate.

This is an offline verifier, not a planner and not online evidence.  It
independently replays a candidate's exact Dubins certificates, evaluates the
frozen R2 semantic contract, applies the path-necessity screen, and enumerates
deterministic start/control-knot-to-goal Dubins shortcuts.  Its public result
uses deliberately distinct gate names; it never emits an unqualified
``gate_passed`` field.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib
import json
import math
from pathlib import Path
import time
from typing import Any, Sequence

import numpy as np

from .semantic_constraint_core import ConstraintWorld, dubins_choices, dubins_edge, replay_edge
from .semantic_map import SemanticMapV1, sha256_file
from .semantic_path_necessity_audit import audit_path_necessity
from .semantic_path_revisit_audit import audit_revisits
from .semantic_transition_contract import TransitionContractR2, resample_path
from .semantic_transition_preflight import replay_controls, write_json
from .semantic_transition_r2_preflight import explicit_hard_audit, semantic_metrics


PROTOCOL_ID = "PLN-02-SEMANTIC-NATURALNESS-VERIFY-V1"
IMPLEMENTATION_REVISION = "semantic-transition-naturalness-verifier-r2-fail-closed"
DUBINS_RADIUS_M = 0.401
STRICT_MAXIMUM_CURVATURE_1PM = 2.50
LENGTH_EPS_M = 1.0e-9
R2_CONTRACT_CONFIG_SHA256 = "4e63f5af208bc70909104a110ad1175efe6488ca5d7d1557de3451404f3cc994"


def _transition_config_path() -> Path:
    """Resolve package data in both source and non-symlink colcon installs."""
    package = Path(importlib.import_module(
        "arena_evaluation.semantic_constraint_core"
    ).__file__).resolve().parent
    name = "pudu_wanda_3f_semantic_endpoint_transition_r2.yaml"
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory("arena_evaluation")) / "config" / name)
    except (ImportError, ModuleNotFoundError):
        pass
    except Exception as error:
        if error.__class__.__name__ != "PackageNotFoundError":
            raise
    candidates.append(package.parent / "config" / name)
    for path in candidates:
        if path.is_file():
            path = path.resolve()
            actual = sha256_file(path)
            if actual != R2_CONTRACT_CONFIG_SHA256:
                raise ValueError(
                    f"frozen R2 contract config hash mismatch: {path}; "
                    f"expected={R2_CONTRACT_CONFIG_SHA256}, actual={actual}"
                )
            return path
    raise FileNotFoundError(f"cannot resolve installed/source transition config: {name}")


def gate_dependency_paths() -> dict[str, Path]:
    """Resolve every local source/config that directly determines a gate."""
    package = Path(importlib.import_module(
        "arena_evaluation.semantic_constraint_core"
    ).__file__).resolve().parent
    dependencies = {
        "arena_evaluation/semantic_constraint_core.py": package / "semantic_constraint_core.py",
        "arena_evaluation/semantic_transition_contract.py": package / "semantic_transition_contract.py",
        "arena_evaluation/semantic_transition_r2_preflight.py": package / "semantic_transition_r2_preflight.py",
        "arena_evaluation/semantic_transition_preflight.py": package / "semantic_transition_preflight.py",
        "arena_evaluation/semantic_map.py": package / "semantic_map.py",
        "arena_evaluation/semantic_path_audit.py": package / "semantic_path_audit.py",
        "arena_evaluation/path_audit.py": package / "path_audit.py",
        "arena_evaluation/se2_semantic_guide.py": package / "se2_semantic_guide.py",
        "config/pudu_wanda_3f_semantic_endpoint_transition_r2.yaml": _transition_config_path(),
    }
    missing = [name for name, path in dependencies.items() if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"missing gate dependencies: {missing}")
    return {name: path.resolve() for name, path in dependencies.items()}


def gate_dependency_hashes() -> dict[str, str]:
    return {name: sha256_file(path) for name, path in gate_dependency_paths().items()}


def _capture_snapshot(files: dict[str, Path]) -> dict[str, Any]:
    resolved = {name: path.resolve() for name, path in files.items()}
    missing = [name for name, path in resolved.items() if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"verification snapshot inputs missing: {missing}")
    hashes = {name: sha256_file(path) for name, path in resolved.items()}
    return {
        "files": {name: str(path) for name, path in resolved.items()},
        "sha256": hashes,
        "snapshot_sha256": hashlib.sha256(
            json.dumps(hashes, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest(),
    }


def _assert_snapshot_unchanged(snapshot: dict[str, Any]) -> None:
    files = snapshot.get("files")
    hashes = snapshot.get("sha256")
    if not isinstance(files, dict) or not isinstance(hashes, dict) or set(files) != set(hashes):
        raise ValueError("invalid verification snapshot")
    current = _capture_snapshot({name: Path(path) for name, path in files.items()})
    if current["sha256"] != hashes or current["snapshot_sha256"] != snapshot.get("snapshot_sha256"):
        raise RuntimeError("verification dependency/input changed during or after verification")


def _points(path: np.ndarray) -> list[dict[str, float]]:
    return [
        {"x": float(x), "y": float(y), "yaw": float(yaw)}
        for x, y, yaw in np.asarray(path, dtype=float)
    ]


def _path_from_edges(edges: Sequence[Any]) -> np.ndarray:
    if not edges:
        raise ValueError("at least one replayable edge is required")
    return np.vstack((edges[0].start, *(edge.samples for edge in edges)))


def _active_target_samples(world: ConstraintWorld, points: list[dict[str, float]]):
    sampled_points, station = resample_path(points)
    sampled_path = np.asarray(
        [[point[key] for key in ("x", "y", "yaw")] for point in sampled_points],
        dtype=float,
    )
    rows, cols, inside = world.cells(sampled_path)
    if not np.all(inside):
        raise ValueError("resampled path leaves the frozen semantic input crop")
    lane = np.isin(world.grids["labels"][rows, cols], world.selected)
    target = lane & world.grids["correct"][rows, cols].astype(bool)
    target &= world.grids["error"][rows, cols] <= TransitionContractR2().lane_target_error_max_m
    length = float(station[-1])
    if length <= TransitionContractR2().short_path_max_m:
        active = np.ones(len(station), dtype=bool)
        active_interval = [0.0, length]
    else:
        endpoint = TransitionContractR2().endpoint_transition_each_m
        active = (station >= endpoint) & (station <= length - endpoint)
        active_interval = [endpoint, length - endpoint]
    return sampled_path, active & target, {
        "sample_spacing_m": TransitionContractR2().sample_spacing_m,
        "sample_count": int(len(station)),
        "active_interval_m": active_interval,
        "active_sample_count": int(np.count_nonzero(active)),
        "active_target_sample_count": int(np.count_nonzero(active & target)),
    }


def _hard_safety_evidence(
    audit: dict[str, Any],
    hard_features: dict[str, Any],
) -> tuple[dict[str, bool], bool]:
    canonical = audit["canonical"]
    checks = {
        "canonical_final_valid_success": canonical.get("final_valid_success") is True,
        "canonical_static_footprint_valid": canonical.get("static_footprint_valid") is True,
        "canonical_kinematic_valid": canonical.get("kinematic_valid") is True,
        "padded_effective_master_collision_free": audit.get("padded_effective_master_collision_free") is True,
        "no_stopping_goal_clear": audit.get("no_stopping_goal_violation") is False,
        "zero_reverse_distance": canonical.get("reverse_distance_m") == 0,
        "zero_in_place_rotation": canonical.get("in_place_rotation_count") == 0,
        "control_curvature_within_limit": (
            math.isfinite(float(audit.get("maximum_control_curvature_1pm", math.inf)))
            and float(audit["maximum_control_curvature_1pm"]) <= STRICT_MAXIMUM_CURVATURE_1PM
        ),
        "geometric_curvature_within_limit": (
            math.isfinite(float(canonical.get("maximum_curvature", math.inf)))
            and float(canonical["maximum_curvature"]) <= STRICT_MAXIMUM_CURVATURE_1PM
        ),
        "explicit_hard_features_clear": hard_features.get("hard_feature_gate_passed") is True,
    }
    return checks, bool(checks and all(checks.values()))


def _targeted_binding_evidence(
    audit: dict[str, Any], *, control_replay_passed: bool,
) -> tuple[dict[str, bool], bool]:
    """Keep candidate/request integrity separate from physical safety."""
    checks = {
        "control_replay_passed": control_replay_passed is True,
        "exact_endpoint_xy_yaw": audit.get("exact_endpoint_xy_yaw") is True,
        "edge_continuity": audit.get("edge_continuity") is True,
        "trace_replay_exact": audit.get("trace_replay_exact") is True,
        "same_lane_instance": audit.get("same_lane_instance") is True,
    }
    return checks, bool(checks and all(checks.values()))


def _evaluate_edges(
    world: ConstraintWorld,
    semantic_map: SemanticMapV1,
    edges: Sequence[Any],
    *,
    control_replay_passed: bool,
) -> dict[str, Any]:
    path = _path_from_edges(edges)
    points = _points(path)
    audit, replayed_path = world.audit(edges)
    if not np.array_equal(path, replayed_path):
        control_replay_passed = False
    metrics = semantic_metrics(world, points)
    hard_features = explicit_hard_audit(world, points, semantic_map)
    hard_checks, hard_passed = _hard_safety_evidence(audit, hard_features)
    binding_checks, binding_passed = _targeted_binding_evidence(
        audit, control_replay_passed=control_replay_passed,
    )
    sampled_path, active_target, sampling = _active_target_samples(world, points)
    revisit = audit_revisits(path)
    naturalness = audit_path_necessity(
        sampled_path,
        start=world.start,
        goal=world.goal,
        target_mask=active_target,
        route_polyline=world.meta.get("route_polyline"),
    )
    semantic_passed = metrics.get("semantic_gate_passed") is True
    naturalness_passed = (
        naturalness.get("input_complete") is True
        and naturalness.get("audit_passed") is True
        and revisit.get("revisit_screen_passed") is True
    )
    strict_passed = hard_passed and binding_passed and semantic_passed and naturalness_passed
    path_length = float(sum(edge.length for edge in edges))
    return {
        "gates": {
            "hard_safety_gate_passed": hard_passed,
            "targeted_binding_gate_passed": binding_passed,
            "r2_semantic_gate_passed": semantic_passed,
            "naturalness_gate_passed": naturalness_passed,
            "strict_acceptance_gate_passed": strict_passed,
        },
        "hard_safety_checks": hard_checks,
        "targeted_binding_checks": binding_checks,
        "canonical_path_audit": audit["canonical"],
        "hard_feature_audit": hard_features,
        "r2_semantic_audit": metrics,
        "naturalness_audit": naturalness,
        "revisit_audit": revisit,
        "semantic_sampling": sampling,
        "path_length_m": path_length,
        "research_diagnostics": {
            "legacy_optimizer_bound_length_m": float(world.bound_length),
            "path_length_within_legacy_optimizer_bound": (
                path_length <= float(world.bound_length) + LENGTH_EPS_M
            ),
            "legacy_optimizer_bound_is_acceptance_gate": False,
        },
        "path_pose_count": int(len(path)),
        "control_words": [edge.word for edge in edges],
        "maximum_control_curvature_1pm": float(audit["maximum_control_curvature_1pm"]),
    }


def _shortcut_row(
    *,
    origin_kind: str,
    kept_edges: int,
    choice_index: int,
    connector: Any,
    evaluation: dict[str, Any],
    original_length_m: float,
) -> dict[str, Any]:
    gates = evaluation["gates"]
    lane = evaluation["r2_semantic_audit"]["active_window"]["classes"]["lane"]
    naturalness = evaluation["naturalness_audit"]
    revisit = evaluation["revisit_audit"]
    length = float(evaluation["path_length_m"])
    return {
        "origin_kind": origin_kind,
        "kept_original_edge_count": kept_edges,
        "connector_choice_index": choice_index,
        "connector_word": connector.word,
        "connector_length_m": float(connector.length),
        "shortcut_path_length_m": length,
        "length_saved_m": original_length_m - length,
        "strictly_shorter": length < original_length_m - LENGTH_EPS_M,
        "hard_safety_gate_passed": gates["hard_safety_gate_passed"],
        "targeted_binding_gate_passed": gates["targeted_binding_gate_passed"],
        "r2_semantic_gate_passed": gates["r2_semantic_gate_passed"],
        "naturalness_gate_passed": gates["naturalness_gate_passed"],
        "strict_acceptance_gate_passed": gates["strict_acceptance_gate_passed"],
        "lane_correct_side_ratio": lane.get("correct_side_ratio"),
        "lane_target_band_ratio": lane.get("target_band_ratio"),
        "lane_lateral_error_p50_m": lane.get("lateral_error_p50_m"),
        "naturalness_status": naturalness.get("status", "INVALID_OUTPUT"),
        "naturalness_failure_codes": list(naturalness.get("failure_codes", ["INVALID_OUTPUT"])),
        "revisit_screen_passed": revisit.get("revisit_screen_passed") is True,
        "revisit_status": revisit.get("status", "INVALID_OUTPUT"),
        "maximum_goal_plane_overshoot_m": naturalness.get("maximum_goal_plane_overshoot_m"),
        "cumulative_backward_route_progress_m": naturalness.get("cumulative_backward_route_progress_m"),
        "target_credit_after_goal_plane_count": (
            naturalness.get("target_sample_attribution", {}).get("after_goal_plane_count")
        ),
    }


def _enumerate_shortcuts(
    world: ConstraintWorld,
    semantic_map: SemanticMapV1,
    original_edges: Sequence[Any],
) -> list[dict[str, Any]]:
    original_length = float(sum(edge.length for edge in original_edges))
    origins: list[tuple[str, int, tuple[float, float, float]]] = [
        ("START_TO_GOAL", 0, tuple(map(float, world.start)))
    ]
    origins.extend(
        ("CONTROL_KNOT_TO_GOAL", kept, tuple(map(float, original_edges[kept - 1].goal)))
        for kept in range(1, len(original_edges))
    )
    rows = []
    for origin_kind, kept, origin in origins:
        choices = dubins_choices(origin, world.goal, DUBINS_RADIUS_M)
        for choice_index in range(len(choices)):
            connector = dubins_edge(origin, world.goal, DUBINS_RADIUS_M, choice_index)
            if connector is None:
                continue
            edges = [*original_edges[:kept], connector]
            evaluation = _evaluate_edges(
                world,
                semantic_map,
                edges,
                control_replay_passed=True,
            )
            rows.append(_shortcut_row(
                origin_kind=origin_kind,
                kept_edges=kept,
                choice_index=choice_index,
                connector=connector,
                evaluation=evaluation,
                original_length_m=original_length,
            ))
    return rows


def _shortcut_summary(rows: Sequence[dict[str, Any]], candidate: dict[str, Any]) -> dict[str, Any]:
    hard_safe_shorter = [
        row for row in rows
        if (
            row["strictly_shorter"]
            and row["hard_safety_gate_passed"]
            and row["targeted_binding_gate_passed"]
        )
    ]
    semantic_losing = [row for row in hard_safe_shorter if not row["r2_semantic_gate_passed"]]
    strict_dominating = [row for row in hard_safe_shorter if row["strict_acceptance_gate_passed"]]
    attribution = candidate["naturalness_audit"].get("target_sample_attribution", {})
    padding = bool(
        candidate["gates"]["r2_semantic_gate_passed"]
        and hard_safe_shorter
        and semantic_losing
        and attribution.get("after_goal_plane_count", 0) > 0
    )
    failure_codes = []
    if hard_safe_shorter:
        failure_codes.append("SAFE_SHORTCUT_DISPROVES_HARD_DETOUR_NECESSITY")
    if padding:
        failure_codes.append("SEMANTIC_WINDOW_PADDING_EVIDENCE")
    return {
        "enumerated_shortcut_count": len(rows),
        "hard_safe_shortcut_count": sum(row["hard_safety_gate_passed"] for row in rows),
        "hard_safe_strictly_shorter_count": len(hard_safe_shorter),
        "hard_safe_shorter_semantic_failure_count": len(semantic_losing),
        "strictly_dominating_shortcut_count": len(strict_dominating),
        "hard_detour_necessity_disproven": bool(hard_safe_shorter),
        "semantic_window_padding_evidence": padding,
        "failure_codes": failure_codes,
        "scope": (
            "deterministic_start_and_original_control_knot_to_exact_goal_Dubins_choices; "
            "constructive_shortcuts_not_a_global_optimality_proof"
        ),
    }


def _candidate_files(candidate: Path) -> tuple[Path, Path]:
    path_file = candidate / "path.json"
    controls_file = candidate / "controls.json"
    if not controls_file.is_file():
        controls_file = candidate / "certificate.json"
    if not path_file.is_file() or not controls_file.is_file():
        raise FileNotFoundError(
            "candidate must contain path.json and controls.json or certificate.json"
        )
    return path_file, controls_file


def _candidate_edges_from_files(
    path_file: Path, controls_file: Path,
) -> tuple[list[Any], dict[str, Any]]:
    replay = replay_controls(path_file, controls_file, "certificate")
    payload = json.loads(controls_file.read_text(encoding="utf-8"))
    certificates = payload.get("edges")
    if not isinstance(certificates, list) or not certificates:
        raise ValueError("controls.json must contain a non-empty edges list")
    edges = [replay_edge(certificate) for certificate in certificates]
    return edges, replay


def _candidate_edges(candidate: Path) -> tuple[list[Any], dict[str, Any]]:
    return _candidate_edges_from_files(*_candidate_files(candidate))


def verify_candidate(
    *,
    inputs: Path,
    query: str,
    candidate: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    inputs, candidate = inputs.resolve(), candidate.resolve()
    path_file, controls_file = _candidate_files(candidate)
    semantic_file = (inputs.parent / "conversion_v1/semantic_map_v1.json").resolve()
    dependency_paths = gate_dependency_paths()
    snapshot_files = {
        "candidate_path": path_file,
        "candidate_controls": controls_file,
        "input_meta": inputs / f"{query}.json",
        "input_npz": inputs / f"{query}.npz",
        "semantic_map": semantic_file,
        "verifier_source": Path(__file__),
        "necessity_audit_source": Path(audit_path_necessity.__code__.co_filename),
        "revisit_audit_source": Path(audit_revisits.__code__.co_filename),
        **{f"gate_dependency::{name}": path for name, path in dependency_paths.items()},
    }
    verification_snapshot = _capture_snapshot(snapshot_files)
    world = ConstraintWorld(inputs, query)
    semantic_map = SemanticMapV1.load(semantic_file)
    if semantic_map.semantic_map_hash != world.meta.get("semantic_map_hash"):
        raise ValueError("semantic map binding mismatch")
    edges, control_replay = _candidate_edges_from_files(path_file, controls_file)
    candidate_evaluation = _evaluate_edges(
        world,
        semantic_map,
        edges,
        control_replay_passed=control_replay.get("control_replay_passed") is True,
    )
    shortcuts = _enumerate_shortcuts(world, semantic_map, edges)
    shortcut_summary = _shortcut_summary(shortcuts, candidate_evaluation)
    # A positive geometry screen is mandatory.  Shortcut evidence cannot turn
    # a failed screen into a pass; it only adds a stronger failure explanation.
    naturalness_passed = bool(
        candidate_evaluation["gates"]["naturalness_gate_passed"]
        and not shortcut_summary["semantic_window_padding_evidence"]
    )
    gates = dict(candidate_evaluation["gates"])
    gates["naturalness_gate_passed"] = naturalness_passed
    gates["strict_acceptance_gate_passed"] = bool(
        gates["hard_safety_gate_passed"]
        and gates["targeted_binding_gate_passed"]
        and gates["r2_semantic_gate_passed"]
        and gates["naturalness_gate_passed"]
    )
    candidate_evaluation["gates"] = gates
    # The candidate and every source/config which determines a gate must stay
    # byte-identical for the entire verification.  Otherwise no result exists.
    _assert_snapshot_unchanged(verification_snapshot)
    dependency_hashes = {
        name: verification_snapshot["sha256"][f"gate_dependency::{name}"]
        for name in dependency_paths
    }
    result = {
        "protocol_id": PROTOCOL_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "contract_revision": TransitionContractR2().contract_revision,
        "scope": "offline_candidate_replay_and_naturalness_verification_not_online_planning_evidence",
        "query_id": query,
        "candidate": str(candidate),
        "gates": gates,
        "candidate_evaluation": candidate_evaluation,
        "shortcut_summary": shortcut_summary,
        "verification_snapshot": verification_snapshot,
        "verification_input_snapshot_match": True,
        "gate_dependency_sha256": dependency_hashes,
        "control_replay": {
            key: value for key, value in control_replay.items() if key != "edges"
        },
        "bindings": {
            "path_sha256": verification_snapshot["sha256"]["candidate_path"],
            "controls_sha256": verification_snapshot["sha256"]["candidate_controls"],
            "input_meta_sha256": verification_snapshot["sha256"]["input_meta"],
            "input_npz_sha256": verification_snapshot["sha256"]["input_npz"],
            "declared_input_npz_sha256": world.meta.get("npz_sha256"),
            "map_hash": world.meta.get("map_hash"),
            "semantic_map_hash": world.meta.get("semantic_map_hash"),
            "semantic_map_file_sha256": verification_snapshot["sha256"]["semantic_map"],
            "expected_master_hash": world.meta.get("expected_master_hash"),
            "route_hash": world.meta.get("route_hash"),
        },
    }
    return result, shortcuts


def _csv_rows(shortcuts: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for shortcut in shortcuts:
        row = dict(shortcut)
        row["naturalness_failure_codes"] = "|".join(shortcut["naturalness_failure_codes"])
        rows.append(row)
    return rows


def write_artifacts(
    output: Path,
    result: dict[str, Any],
    shortcuts: Sequence[dict[str, Any]],
) -> None:
    snapshot = result.get("verification_snapshot")
    if not isinstance(snapshot, dict):
        raise ValueError("verification snapshot is required for fail-closed artifact writing")
    _assert_snapshot_unchanged(snapshot)
    dependency_hashes = result.get("gate_dependency_sha256")
    if not isinstance(dependency_hashes, dict):
        raise ValueError("gate dependency hashes are required")
    snapshot_dependencies = {
        name.removeprefix("gate_dependency::"): value
        for name, value in snapshot["sha256"].items()
        if name.startswith("gate_dependency::")
    }
    if dependency_hashes != snapshot_dependencies:
        raise ValueError("gate dependency hash binding mismatch")
    output.mkdir(parents=False, exist_ok=False)
    verification = output / "verification.json"
    shortcut_csv = output / "shortcut_evidence.csv"
    write_json(verification, result)
    rows = _csv_rows(shortcuts)
    if not rows:
        raise ValueError("shortcut enumeration unexpectedly produced no rows")
    with shortcut_csv.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    _assert_snapshot_unchanged(snapshot)
    manifest = {
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "protocol_id": PROTOCOL_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "query_id": result["query_id"],
        "write_once": True,
        "source_sha256": snapshot["sha256"]["verifier_source"],
        "necessity_audit_source_sha256": snapshot["sha256"]["necessity_audit_source"],
        "revisit_audit_source_sha256": snapshot["sha256"]["revisit_audit_source"],
        "gate_dependency_sha256": dependency_hashes,
        "verification_input_snapshot_sha256": snapshot["snapshot_sha256"],
        "verification_input_snapshot_match": True,
        "verification_sha256": sha256_file(verification),
        "shortcut_evidence_sha256": sha256_file(shortcut_csv),
        "bound_input_hashes": result["bindings"],
    }
    write_json(output / "manifest.json", manifest)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--query", required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result, shortcuts = verify_candidate(
        inputs=args.inputs,
        query=args.query,
        candidate=args.candidate,
    )
    write_artifacts(args.output.resolve(), result, shortcuts)
    print(json.dumps(result["gates"], indent=2, sort_keys=True))
    return 0 if result["gates"]["strict_acceptance_gate_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
