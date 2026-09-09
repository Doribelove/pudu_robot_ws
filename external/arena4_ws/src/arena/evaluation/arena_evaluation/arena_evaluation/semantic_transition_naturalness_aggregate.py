"""Aggregate the frozen three-query R2 naturalness preflight fail closed.

This consumes only independently written verifier directories.  It validates
their artifact hashes, source-policy identity, input bindings and exact query
order before deciding whether online or selected8 work is eligible to start.
It is an offline gate, never online-planner evidence.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import time
from typing import Any, Sequence

from .semantic_map import sha256_file
from .semantic_path_necessity_audit import audit_path_necessity
from .semantic_path_revisit_audit import audit_revisits
from .semantic_transition_preflight import write_json
from .semantic_transition_naturalness_verify import (
    IMPLEMENTATION_REVISION as VERIFIER_IMPLEMENTATION_REVISION,
    PROTOCOL_ID as VERIFIER_PROTOCOL_ID,
    _assert_snapshot_unchanged,
    _capture_snapshot,
)
from .semantic_query_defaults import load_query_set


PROTOCOL_ID = "PLN-02-SEMANTIC-NATURALNESS-TRIAD-R0-V1"
REQUIRED_QUERIES = (
    "r3-mirror-1-positive",
    "r3-mirror-2-negative",
    "cmp2-02-lane-south",
)
TARGETED_QUERY_CONTENT_HASH = "66212b05ef6c4d16eaedafc1c27866387c3d74aa85bf5ac69bca92487254667d"
MAP_HASH = "05cf18d0df40235f69ba5f0168bb490f9175541431c0c516a962e7ce1965529a"
SEMANTIC_MAP_HASH = "2560a4f4c86a86aeaf9993262648aaeb26998948e79fe3b92ecf47b6e69d0553"
CONTRACT_REVISION = "semantic-endpoint-transition-short-full-6m-r2"
TARGETED_QUERY_SET_FILENAME = "pudu_wanda_3f_r3_targeted_preflight3_v1.yaml"
R2_CONTRACT_CONFIG_FILENAME = "pudu_wanda_3f_semantic_endpoint_transition_r2.yaml"
SOURCE_CONFIG_DIRECTORY = (
    Path(__file__).resolve().parent.parent
    / "config"
)
TARGETED_QUERY_SET_SHA256 = "2092f57ffc41bca885f783abc17e630ddecd3311035957957c55f873b9225f01"
R2_CONTRACT_CONFIG_SHA256 = "4e63f5af208bc70909104a110ad1175efe6488ca5d7d1557de3451404f3cc994"
FROZEN_INPUT_MANIFEST_SHA256 = "38225fde75bc0eb37a74b4497284bec2f3260318cce15fded635f5c9a2becc45"

_GATE_DEPENDENCY_MODULES = (
    "semantic_constraint_core.py",
    "semantic_transition_contract.py",
    "semantic_transition_r2_preflight.py",
    "semantic_transition_preflight.py",
    "semantic_map.py",
    "semantic_path_audit.py",
    "path_audit.py",
    "se2_semantic_guide.py",
)


def _ament_share_directory() -> Path | None:
    """Return the installed package share without requiring ament in source tests."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("arena_evaluation")).resolve()
    except (ImportError, ModuleNotFoundError):
        return None
    except Exception as error:
        # PackageNotFoundError differs between ROS distributions.  Only treat a
        # genuine lookup miss as a source-tree fallback; other ament failures are
        # not safe to hide.
        if error.__class__.__name__ == "PackageNotFoundError":
            return None
        raise


def _resolve_frozen_config(
    filename: str,
    expected_sha256: str,
    *,
    explicit: Path | None = None,
) -> Path:
    """Resolve one frozen config in install or source layout and verify content."""
    if explicit is not None:
        candidates = [Path(explicit).resolve()]
    else:
        candidates = []
        share = _ament_share_directory()
        if share is not None:
            candidates.append(share / "config" / filename)
        source = SOURCE_CONFIG_DIRECTORY / filename
        if source not in candidates:
            candidates.append(source)
    for candidate in candidates:
        if not candidate.is_file():
            continue
        actual = sha256_file(candidate)
        if actual != expected_sha256:
            raise ValueError(
                f"frozen config hash mismatch: {candidate}; "
                f"expected={expected_sha256}, actual={actual}"
            )
        return candidate
    raise FileNotFoundError(
        f"frozen config not found ({filename}); checked: "
        + ", ".join(map(str, candidates))
    )


def _current_gate_dependency_paths() -> dict[str, Path]:
    """Resolve verifier dependencies in source and installed layouts."""
    package = Path(__file__).resolve().parent
    dependencies = {
        f"arena_evaluation/{name}": package / name
        for name in _GATE_DEPENDENCY_MODULES
    }
    dependencies[f"config/{R2_CONTRACT_CONFIG_FILENAME}"] = _resolve_frozen_config(
        R2_CONTRACT_CONFIG_FILENAME,
        R2_CONTRACT_CONFIG_SHA256,
    )
    missing = [name for name, path in dependencies.items() if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"missing installed/source gate dependencies: {missing}")
    return {name: path.resolve() for name, path in dependencies.items()}


def _current_gate_dependency_hashes() -> dict[str, str]:
    return {
        name: sha256_file(path)
        for name, path in _current_gate_dependency_paths().items()
    }


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _load_verified_result(directory: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    directory = directory.resolve()
    verification_file = directory / "verification.json"
    manifest_file = directory / "manifest.json"
    if not verification_file.is_file() or not manifest_file.is_file():
        raise FileNotFoundError(f"missing verification.json or manifest.json: {directory}")
    verification, manifest = _load_json(verification_file), _load_json(manifest_file)
    if sha256_file(verification_file) != manifest.get("verification_sha256"):
        raise ValueError(f"verification hash mismatch: {directory}")
    if verification.get("query_id") != manifest.get("query_id"):
        raise ValueError(f"query binding mismatch: {directory}")
    if verification.get("protocol_id") != manifest.get("protocol_id"):
        raise ValueError(f"protocol binding mismatch: {directory}")
    snapshot = verification.get("verification_snapshot")
    if (
        verification.get("verification_input_snapshot_match") is not True
        or manifest.get("verification_input_snapshot_match") is not True
        or not isinstance(snapshot, dict)
        or snapshot.get("snapshot_sha256") != manifest.get("verification_input_snapshot_sha256")
    ):
        raise ValueError(f"verification input snapshot binding mismatch: {directory}")
    if manifest.get("bound_input_hashes") != verification.get("bindings"):
        raise ValueError(f"manifest input binding mismatch: {directory}")
    if manifest.get("gate_dependency_sha256") != verification.get("gate_dependency_sha256"):
        raise ValueError(f"manifest gate dependency binding mismatch: {directory}")
    return verification, manifest


def _candidate_bound_files(verification: dict[str, Any]) -> tuple[Path, Path]:
    directory = Path(verification.get("candidate", "")).resolve()
    path_file = directory / "path.json"
    controls_file = directory / "controls.json"
    if not controls_file.is_file():
        controls_file = directory / "certificate.json"
    if not path_file.is_file() or not controls_file.is_file():
        raise FileNotFoundError(f"missing bound candidate path/controls: {directory}")
    return path_file, controls_file


def _aggregation_snapshot_files(
    *,
    inputs: Path,
    result_directories: Sequence[Path],
    loaded: Sequence[tuple[dict[str, Any], dict[str, Any]]],
    targeted_query_set: Path,
) -> dict[str, Path]:
    files: dict[str, Path] = {
        "aggregate_source": Path(__file__),
        "verifier_source": Path(__file__).with_name("semantic_transition_naturalness_verify.py"),
        "necessity_audit_source": Path(audit_path_necessity.__code__.co_filename),
        "revisit_audit_source": Path(audit_revisits.__code__.co_filename),
        "targeted_query_set": targeted_query_set,
        "input_artifact_manifest": inputs / "artifact_hashes.json",
        "semantic_map": inputs.parent / "conversion_v1/semantic_map_v1.json",
    }
    for name, path in _current_gate_dependency_paths().items():
        files[f"gate_dependency::{name}"] = path
    for query in REQUIRED_QUERIES:
        files[f"input_meta::{query}"] = inputs / f"{query}.json"
        files[f"input_npz::{query}"] = inputs / f"{query}.npz"
    for index, (directory, (verification, _)) in enumerate(zip(result_directories, loaded)):
        files[f"verification::{index}"] = Path(directory).resolve() / "verification.json"
        files[f"verification_manifest::{index}"] = Path(directory).resolve() / "manifest.json"
        path_file, controls_file = _candidate_bound_files(verification)
        files[f"candidate_path::{index}"] = path_file
        files[f"candidate_controls::{index}"] = controls_file
    return files


def aggregate(
    *,
    inputs: Path,
    result_directories: Sequence[Path],
    targeted_query_set: Path | None = None,
) -> dict[str, Any]:
    if len(result_directories) != len(REQUIRED_QUERIES):
        raise ValueError("exactly three result directories are required")
    inputs = Path(inputs).resolve()
    result_directories = tuple(Path(path).resolve() for path in result_directories)
    targeted_query_set_path = _resolve_frozen_config(
        TARGETED_QUERY_SET_FILENAME,
        TARGETED_QUERY_SET_SHA256,
        explicit=targeted_query_set,
    )
    loaded_probe = [_load_verified_result(path) for path in result_directories]
    aggregation_snapshot = _capture_snapshot(_aggregation_snapshot_files(
        inputs=inputs,
        result_directories=result_directories,
        loaded=loaded_probe,
        targeted_query_set=targeted_query_set_path,
    ))
    loaded = [_load_verified_result(path) for path in result_directories]
    if loaded != loaded_probe:
        raise RuntimeError("verification inputs changed while aggregation snapshot was captured")
    query_ids = tuple(result[0].get("query_id") for result in loaded)
    if query_ids != REQUIRED_QUERIES:
        raise ValueError(f"query order/content mismatch: {query_ids!r}")

    identity_fields = (
        "protocol_id",
        "implementation_revision",
        "source_sha256",
        "necessity_audit_source_sha256",
        "revisit_audit_source_sha256",
    )
    identity = {field: {manifest.get(field) for _, manifest in loaded} for field in identity_fields}
    mismatched = [field for field, values in identity.items() if len(values) != 1]
    if mismatched:
        raise ValueError(f"verifier identity mismatch: {mismatched}")
    verifier_source = Path(__file__).with_name("semantic_transition_naturalness_verify.py")
    necessity_source = Path(audit_path_necessity.__code__.co_filename)
    revisit_source = Path(audit_revisits.__code__.co_filename)
    expected_identity = {
        "protocol_id": VERIFIER_PROTOCOL_ID,
        "implementation_revision": VERIFIER_IMPLEMENTATION_REVISION,
        "source_sha256": sha256_file(verifier_source),
        "necessity_audit_source_sha256": sha256_file(necessity_source),
        "revisit_audit_source_sha256": sha256_file(revisit_source),
    }
    actual_identity = {field: next(iter(values)) for field, values in identity.items()}
    if actual_identity != expected_identity:
        raise ValueError("verifier identity does not match current frozen implementation")
    expected_dependencies = _current_gate_dependency_hashes()
    for _, manifest in loaded:
        if manifest.get("gate_dependency_sha256") != expected_dependencies:
            raise ValueError("gate dependency hashes do not match current frozen implementation")

    frozen_queries, _, query_metadata = load_query_set(
        targeted_query_set_path,
        actual_map_hash=MAP_HASH,
        actual_semantic_map_hash=SEMANTIC_MAP_HASH,
    )
    if tuple(query.query_id for query in frozen_queries) != REQUIRED_QUERIES:
        raise ValueError("frozen targeted query IDs/order mismatch")
    if query_metadata.get("query_hash") != TARGETED_QUERY_CONTENT_HASH:
        raise ValueError("frozen targeted query content hash mismatch")
    frozen_by_id = {query.query_id: query.as_dict() for query in frozen_queries}
    input_manifest_file = inputs / "artifact_hashes.json"
    if sha256_file(input_manifest_file) != FROZEN_INPUT_MANIFEST_SHA256:
        raise ValueError("frozen input artifact manifest hash mismatch")
    frozen_input_hashes = _load_json(input_manifest_file)

    rows = []
    targeted_hashes = set()
    for expected_query, (verification, manifest), directory in zip(
        REQUIRED_QUERIES, loaded, result_directories
    ):
        meta_file = inputs / f"{expected_query}.json"
        npz_file = inputs / f"{expected_query}.npz"
        meta = _load_json(meta_file)
        if meta.get("query") != frozen_by_id[expected_query]:
            raise ValueError(f"frozen query pose/content mismatch: {expected_query}")
        if frozen_input_hashes.get(meta_file.name) != sha256_file(meta_file):
            raise ValueError(f"frozen input JSON artifact mismatch: {expected_query}")
        if frozen_input_hashes.get(npz_file.name) != sha256_file(npz_file):
            raise ValueError(f"frozen input NPZ artifact mismatch: {expected_query}")
        bindings = verification.get("bindings", {})
        if sha256_file(meta_file) != bindings.get("input_meta_sha256"):
            raise ValueError(f"input metadata hash mismatch: {expected_query}")
        if not npz_file.is_file() or sha256_file(npz_file) != bindings.get("input_npz_sha256"):
            raise ValueError(f"input NPZ hash mismatch: {expected_query}")
        if bindings.get("declared_input_npz_sha256") != bindings.get("input_npz_sha256"):
            raise ValueError(f"declared input NPZ hash mismatch: {expected_query}")
        for field in ("map_hash", "semantic_map_hash", "expected_master_hash", "route_hash"):
            if meta.get(field) != bindings.get(field):
                raise ValueError(f"{field} binding mismatch: {expected_query}")
        semantic_file = inputs.parent / "conversion_v1/semantic_map_v1.json"
        if not semantic_file.is_file() or sha256_file(semantic_file) != bindings.get(
            "semantic_map_file_sha256"
        ):
            raise ValueError(f"semantic map file hash mismatch: {expected_query}")
        path_file, controls_file = _candidate_bound_files(verification)
        if not path_file.is_file() or sha256_file(path_file) != bindings.get("path_sha256"):
            raise ValueError(f"candidate path hash mismatch: {expected_query}")
        if not controls_file.is_file() or sha256_file(controls_file) != bindings.get(
            "controls_sha256"
        ):
            raise ValueError(f"candidate controls hash mismatch: {expected_query}")
        if verification.get("contract_revision") != CONTRACT_REVISION:
            raise ValueError(f"contract revision mismatch: {expected_query}")
        targeted_hashes.add(meta.get("targeted_query_content_hash"))
        candidate = verification["candidate_evaluation"]
        lane = candidate["r2_semantic_audit"]["active_window"]["classes"]["lane"]
        naturalness = candidate["naturalness_audit"]
        revisit = candidate.get("revisit_audit", {})
        gates = verification["gates"]
        if candidate.get("gates") != gates:
            raise ValueError(f"candidate/top-level gate mismatch: {expected_query}")
        derived_strict = bool(
            gates.get("hard_safety_gate_passed") is True
            and gates.get("targeted_binding_gate_passed") is True
            and gates.get("r2_semantic_gate_passed") is True
            and gates.get("naturalness_gate_passed") is True
        )
        if gates.get("strict_acceptance_gate_passed") is not derived_strict:
            raise ValueError(f"derived strict gate mismatch: {expected_query}")
        rows.append({
            "query_id": expected_query,
            "result_directory": str(Path(directory).resolve()),
            "targeted_binding_gate_passed": gates.get("targeted_binding_gate_passed") is True,
            "hard_safety_gate_passed": gates.get("hard_safety_gate_passed") is True,
            "r2_semantic_gate_passed": gates.get("r2_semantic_gate_passed") is True,
            "naturalness_gate_passed": gates.get("naturalness_gate_passed") is True,
            "strict_acceptance_gate_passed": gates.get("strict_acceptance_gate_passed") is True,
            "path_length_m": candidate.get("path_length_m"),
            "lane_correct_side_ratio": lane.get("correct_side_ratio"),
            "lane_target_band_ratio": lane.get("target_band_ratio"),
            "lane_lateral_error_p50_m": lane.get("lateral_error_p50_m"),
            "naturalness_failure_codes": list(naturalness.get("failure_codes", [])),
            "revisit_screen_passed": revisit.get("revisit_screen_passed") is True,
            "revisit_status": revisit.get("status"),
            "maximum_goal_plane_overshoot_m": naturalness.get("maximum_goal_plane_overshoot_m"),
            "cumulative_backward_route_progress_m": naturalness.get(
                "cumulative_backward_route_progress_m"
            ),
            "nonlocal_reverse_footprint_overlap_count": naturalness.get(
                "nonlocal_reverse_footprint_overlap_count"
            ),
            "target_credit_after_goal_plane_count": naturalness.get(
                "target_sample_attribution", {}
            ).get("after_goal_plane_count"),
            "verification_sha256": manifest["verification_sha256"],
        })
    if targeted_hashes != {TARGETED_QUERY_CONTENT_HASH}:
        raise ValueError("targeted query-set binding mismatch")
    if any(row_bindings[0]["bindings"].get("map_hash") != MAP_HASH for row_bindings in loaded):
        raise ValueError("frozen map hash mismatch")
    if any(
        row_bindings[0]["bindings"].get("semantic_map_hash") != SEMANTIC_MAP_HASH
        for row_bindings in loaded
    ):
        raise ValueError("frozen semantic map hash mismatch")

    pass_count = sum(row["strict_acceptance_gate_passed"] for row in rows)
    all_passed = pass_count == len(REQUIRED_QUERIES)
    result = {
        "protocol_id": PROTOCOL_ID,
        "scope": "offline_three_query_naturalness_gate_not_online_planning_evidence",
        "required_query_ids": list(REQUIRED_QUERIES),
        "required_queries_present_exactly_once_in_order": True,
        "targeted_query_content_hash": next(iter(targeted_hashes)),
        "targeted_query_set_path": str(targeted_query_set_path),
        "targeted_query_set_sha256": TARGETED_QUERY_SET_SHA256,
        "frozen_input_manifest_sha256": FROZEN_INPUT_MANIFEST_SHA256,
        "verifier_identity": actual_identity,
        "gate_dependency_sha256": expected_dependencies,
        "aggregation_snapshot": aggregation_snapshot,
        "aggregation_input_snapshot_match": True,
        "offline_pass_count": pass_count,
        "offline_required_count": len(REQUIRED_QUERIES),
        "targeted_offline_gate_passed": all_passed,
        "online_eligible": all_passed,
        "online_stage": "ELIGIBLE_NOT_RUN" if all_passed else "NOT_RUN_OFFLINE_GATE_FAILED",
        "selected8_stage": "NOT_RUN_PREREQUISITE_PENDING" if all_passed else "NOT_RUN_OFFLINE_GATE_FAILED",
        "grade": "NOT_GRADED_ONLINE_NOT_RUN" if all_passed else "C",
        "failure_query_ids": [row["query_id"] for row in rows if not row["strict_acceptance_gate_passed"]],
        "per_query": rows,
    }
    _assert_snapshot_unchanged(aggregation_snapshot)
    return result


def _csv_value(value: Any) -> Any:
    if isinstance(value, list):
        return "|".join(map(str, value))
    return value


def write_artifacts(output: Path, result: dict[str, Any], command: str) -> None:
    snapshot = result.get("aggregation_snapshot")
    if not isinstance(snapshot, dict) or result.get("aggregation_input_snapshot_match") is not True:
        raise ValueError("aggregation snapshot is required for fail-closed artifact writing")
    _assert_snapshot_unchanged(snapshot)
    output.mkdir(parents=False, exist_ok=False)
    write_json(output / "gate_results.json", result)
    rows = [{key: _csv_value(value) for key, value in row.items()} for row in result["per_query"]]
    with (output / "per_query.csv").open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output / "reproduction_command.txt").write_text(command.rstrip() + "\n", encoding="utf-8")
    try:
        _assert_snapshot_unchanged(snapshot)
    except Exception:
        (output / "EXCLUDED_INPUT_CHANGED.md").write_text(
            "# EXCLUDED\n\nA bound source, input, candidate, or verifier result changed "
            "during aggregation. This partial directory is not evidence.\n",
            encoding="utf-8",
        )
        raise
    manifest = {
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "protocol_id": PROTOCOL_ID,
        "source_sha256": snapshot["sha256"]["aggregate_source"],
        "aggregation_input_snapshot_sha256": snapshot["snapshot_sha256"],
        "aggregation_input_snapshot_match": True,
        "gate_results_sha256": sha256_file(output / "gate_results.json"),
        "per_query_sha256": sha256_file(output / "per_query.csv"),
        "reproduction_command_sha256": sha256_file(output / "reproduction_command.txt"),
        "input_verification_hashes": {
            row["query_id"]: row["verification_sha256"] for row in result["per_query"]
        },
        "write_once": True,
    }
    write_json(output / "manifest.json", manifest)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--positive", type=Path, required=True)
    parser.add_argument("--negative", type=Path, required=True)
    parser.add_argument("--south", type=Path, required=True)
    parser.add_argument(
        "--targeted-query-set",
        type=Path,
        help=(
            "optional explicit frozen targeted-query YAML; installed package share "
            "then source-tree config are used when omitted"
        ),
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result_directories = (args.positive, args.negative, args.south)
    result = aggregate(
        inputs=args.inputs.resolve(),
        result_directories=result_directories,
        targeted_query_set=(args.targeted_query_set.resolve() if args.targeted_query_set else None),
    )
    command = " ".join((
        "/usr/bin/python3 -m arena_evaluation.semantic_transition_naturalness_aggregate",
        f"--inputs {args.inputs.resolve()}",
        f"--positive {args.positive.resolve()}",
        f"--negative {args.negative.resolve()}",
        f"--south {args.south.resolve()}",
        f"--targeted-query-set {result['targeted_query_set_path']}",
        f"--output {args.output.resolve()}",
    ))
    write_artifacts(args.output.resolve(), result, command)
    print(json.dumps({key: result[key] for key in (
        "offline_pass_count", "offline_required_count", "targeted_offline_gate_passed",
        "online_stage", "selected8_stage", "grade", "failure_query_ids",
    )}, indent=2, sort_keys=True))
    return 0 if result["targeted_offline_gate_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
