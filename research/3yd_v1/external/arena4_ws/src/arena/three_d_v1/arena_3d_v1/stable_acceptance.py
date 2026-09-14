"""Independent release runner for 3D-V1-r2-stable (never imported by production)."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import yaml

from . import r2_soak, r2_stage_a, r2_stage_b
from .stable_contract import (
    DEFAULT_STABLE_CONFIG,
    PRODUCTION_BASELINE_ID,
    PROTOCOL_ID,
    RELEASE_CANDIDATE_ID,
    SOURCE_REVISION,
    load_stable_config,
    sha256_file,
)
from .stable_pipeline import Layered3DV1StableController


ROOT = Path("/home/robot/pudu_robot_ws")
FROZEN_R2_HELDOUT = ROOT / "experiments/layered_planner_benchmark/3d_v1_r2_heldout_20260904_01"


def _patch_common(output: Path, *, evidence_type: str) -> None:
    config = load_stable_config()
    manifest_path = output / "manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
    manifest.update({
        "architecture_id": "3D-V1",
        "production_baseline_id": PRODUCTION_BASELINE_ID,
        "revision_id": PRODUCTION_BASELINE_ID,
        "source_revision": SOURCE_REVISION,
        "release_candidate": RELEASE_CANDIDATE_ID,
        "protocol_id": PROTOCOL_ID,
        "evidence_type": evidence_type,
        "stable_config": str(DEFAULT_STABLE_CONFIG),
        "stable_config_sha256": config["_config_sha256"],
        "accepted_implementation_reused": "Layered3DV1R2Controller via Layered3DV1StableController",
        "production_runtime_contains_research_arms": False,
    })
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")


def run_stage_a(
    output: Path,
    *,
    query_ids: Sequence[str] = r2_stage_a.HELDOUT_QUERIES,
    repetitions: int = 10,
    mode: str = "heldout",
) -> Path:
    previous = r2_stage_a.Layered3DV1R2Controller
    r2_stage_a.Layered3DV1R2Controller = Layered3DV1StableController
    try:
        result = r2_stage_a.run(
            output,
            mode=mode,
            query_ids=query_ids,
            repetitions=repetitions,
            dstar_budget_ms=500.0,
            max_active_states=1,
            frozen_config_path=r2_stage_a.DEFAULT_FROZEN_CONFIG,
        )
    finally:
        r2_stage_a.Layered3DV1R2Controller = previous
    _patch_common(result, evidence_type=f"stable-release-stage-a-three-arm-{mode}")
    gate_path = result / "gate_results.yaml"
    gate = yaml.safe_load(gate_path.read_text(encoding="utf-8")) or {}
    gate.update({
        "production_baseline_id": PRODUCTION_BASELINE_ID,
        "source_revision": SOURCE_REVISION,
        "candidate_arm": "C_r2_acceptance label executing Layered3DV1StableController",
        "stable_stage_a_pass": gate.get("stage_a_pass") is True,
    })
    gate_path.write_text(yaml.safe_dump(gate, sort_keys=False), encoding="utf-8")
    verification_path = result / "verification.yaml"
    verification = yaml.safe_load(verification_path.read_text(encoding="utf-8")) or {}
    verification.update({
        "candidate_controller": "Layered3DV1StableController",
        "production_baseline_id": PRODUCTION_BASELINE_ID,
        "stable_stage_a_pass": gate.get("stage_a_pass") is True,
    })
    verification_path.write_text(yaml.safe_dump(verification, sort_keys=False), encoding="utf-8")
    report_path = result / "final_report.md"
    report_path.write_text(
        report_path.read_text(encoding="utf-8").replace(
            "# 3D-V1-r2 heldout Stage A", "# 3D-V1-r2-stable release Stage A"
        )
        + "\nThe C arm executed the stable production controller facade; A/B/C arm logic remains in this independent acceptance runner only.\n",
        encoding="utf-8",
    )
    (result / "reproduction_command.txt").write_text(
        "cd /home/robot/pudu_robot_ws\n"
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        "PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/three_d_v1:"
        "/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation "
        f"/usr/bin/python3 -m arena_3d_v1.stable_acceptance stage-a --output-dir {result} "
        f"--query-ids {','.join(query_ids)} --repetitions {repetitions} --mode {mode}\n",
        encoding="utf-8",
    )
    return result


def run_soak(
    output: Path,
    *,
    query_ids: Sequence[str] = r2_soak.DEFAULT_QUERIES,
    min_snapshots: int = 5_000,
    max_snapshots: int = 10_000,
    max_duration_s: float = 1_800.0,
) -> Path:
    previous = r2_soak.Layered3DV1R2Controller
    r2_soak.Layered3DV1R2Controller = Layered3DV1StableController
    try:
        result = r2_soak.run(
            output,
            query_ids=query_ids,
            min_snapshots=min_snapshots,
            max_snapshots=max_snapshots,
            max_duration_s=max_duration_s,
            route_switch_interval=500,
            oracle_sample_interval=100,
            frozen_config_path=r2_stage_a.DEFAULT_FROZEN_CONFIG,
        )
    finally:
        r2_soak.Layered3DV1R2Controller = previous
    _patch_common(result, evidence_type="stable-release-10000-snapshot-soak")
    verification_path = result / "verification.yaml"
    verification = yaml.safe_load(verification_path.read_text(encoding="utf-8")) or {}
    verification.update({
        "candidate_controller": "Layered3DV1StableController",
        "production_baseline_id": PRODUCTION_BASELINE_ID,
        "stable_soak_pass": verification.get("soak_pass") is True,
    })
    verification_path.write_text(yaml.safe_dump(verification, sort_keys=False), encoding="utf-8")
    (result / "reproduction_command.txt").write_text(
        "cd /home/robot/pudu_robot_ws\n"
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        "PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/three_d_v1:"
        "/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation "
        f"/usr/bin/python3 -m arena_3d_v1.stable_acceptance soak --output-dir {result} "
        f"--query-ids {','.join(query_ids)} --min-snapshots {min_snapshots} "
        f"--max-snapshots {max_snapshots} --max-duration-s {max_duration_s}\n",
        encoding="utf-8",
    )
    return result


def run_stage_b(
    output: Path,
    *,
    heldout: Path,
    query_ids: Sequence[str] = r2_stage_b.DEFAULT_QUERIES,
    ros_domain_id: int = 241,
    costmap_ack_timeout_s: float = 10.0,
) -> Path:
    previous = r2_stage_b.Layered3DV1R2Controller
    r2_stage_b.Layered3DV1R2Controller = Layered3DV1StableController
    try:
        result = r2_stage_b.run(
            output,
            heldout=heldout,
            query_ids=query_ids,
            ros_domain_id=ros_domain_id,
            costmap_ack_timeout_s=costmap_ack_timeout_s,
            frozen_config=r2_stage_a.DEFAULT_FROZEN_CONFIG,
        )
    finally:
        r2_stage_b.Layered3DV1R2Controller = previous
    _patch_common(result, evidence_type="stable-release-three-query-stage-b")
    verification_path = result / "verification.yaml"
    verification = yaml.safe_load(verification_path.read_text(encoding="utf-8")) or {}
    verification.update({
        "candidate_controller": "Layered3DV1StableController",
        "production_baseline_id": PRODUCTION_BASELINE_ID,
        "stable_stage_b_pass": verification.get("stage_b_pass") is True,
    })
    verification_path.write_text(yaml.safe_dump(verification, sort_keys=False), encoding="utf-8")
    (result / "reproduction_command.txt").write_text(
        "cd /home/robot/pudu_robot_ws\n"
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        "PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/three_d_v1:"
        "/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation "
        f"ROS_DOMAIN_ID={ros_domain_id} /usr/bin/python3 -m arena_3d_v1.stable_acceptance "
        f"stage-b --output-dir {result} --heldout {heldout.resolve()} "
        f"--query-ids {','.join(query_ids)} --ros-domain-id {ros_domain_id} "
        f"--costmap-ack-timeout-s {costmap_ack_timeout_s}\n",
        encoding="utf-8",
    )
    return result


def _read_csv(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _cache_bindings(root: Path) -> Dict[str, Tuple[str, str]]:
    result: Dict[str, Tuple[str, str]] = {}
    for manifest in sorted(root.rglob("manifest.json")):
        if not any(part in {"geometry", "state"} for part in manifest.parts):
            continue
        value = json.loads(manifest.read_text(encoding="utf-8"))
        digest = str(value.get("binding_hash", ""))
        result[f"{manifest.parent.parent.name}:{digest}"] = (
            str(value.get("schema_version", "")),
            json.dumps(value.get("binding") or {}, sort_keys=True),
        )
    return result


def compare_stage_a_equivalence(
    output: Path,
    *,
    stable_stage_a: Path,
    frozen_r2: Path = FROZEN_R2_HELDOUT,
) -> Path:
    output = output.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty equivalence output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    stable_rows = {
        (row["query_id"], row["repetition"], row["snapshot_index"]): row
        for row in _read_csv(stable_stage_a / "runs.csv")
        if row["arm"] == "C_r2_acceptance"
    }
    frozen_rows = {
        (row["query_id"], row["repetition"], row["snapshot_index"]): row
        for row in _read_csv(frozen_r2 / "runs.csv")
        if row["arm"] == "C_r2_acceptance"
    }
    keys = sorted(set(stable_rows).union(frozen_rows))
    fields = (
        "snapshot_hash", "scheduler_reason", "scheduler_invoke_l2", "backend",
        "reachable", "cost", "cost_error", "blocked_or_recovering_in_path",
        "partial_dstar", "path_hash",
    )
    rows: List[Dict[str, Any]] = []
    for key in keys:
        stable = stable_rows.get(key)
        frozen = frozen_rows.get(key)
        differences = [] if stable is not None and frozen is not None else ["ROW_MISSING"]
        if stable is not None and frozen is not None:
            differences.extend(field for field in fields if stable.get(field) != frozen.get(field))
        rows.append({
            "query_id": key[0],
            "repetition": key[1],
            "snapshot_index": key[2],
            "equivalent": not differences,
            "differences": ";".join(differences),
            "stable_backend": "" if stable is None else stable.get("backend", ""),
            "frozen_backend": "" if frozen is None else frozen.get("backend", ""),
            "snapshot_hash": "" if stable is None else stable.get("snapshot_hash", ""),
        })
    with (output / "equivalence.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ["equivalent"])
        writer.writeheader()
        writer.writerows(rows)
    stable_bindings = _cache_bindings(stable_stage_a / "verified_r2_cache")
    frozen_bindings = _cache_bindings(frozen_r2 / "verified_r2_cache")
    stable_gate = yaml.safe_load((stable_stage_a / "gate_results.yaml").read_text(encoding="utf-8")) or {}
    verification = {
        "architecture_id": "3D-V1",
        "production_baseline_id": PRODUCTION_BASELINE_ID,
        "source_revision": SOURCE_REVISION,
        "release_candidate": RELEASE_CANDIDATE_ID,
        "protocol_id": PROTOCOL_ID,
        "stable_stage_a": str(stable_stage_a.resolve()),
        "frozen_r2_stage_a": str(frozen_r2.resolve()),
        "paired_rows": len(rows),
        "row_key_sets_equal": set(stable_rows) == set(frozen_rows),
        "scheduler_backend_failure_cost_path_equivalent": all(row["equivalent"] for row in rows),
        "cache_binding_keys_and_fields_equal": stable_bindings == frozen_bindings,
        "online_synchronous_build_zero": int(stable_gate.get("synchronous_dstar_build_count", -1)) == 0,
        "partial_dstar_zero": int(stable_gate.get("partial_dstar_results", -1)) == 0,
        "blocked_or_recovering_path_zero": int(stable_gate.get("blocked_or_recovering_in_path", -1)) == 0,
        "canonical_cost_error_zero": float(stable_gate.get("canonical_cost_error_max", -1)) == 0.0,
        "ack_equivalence": "NOT_APPLICABLE_STAGE_A; verified in stable Stage B",
        "stable_source_snapshot_present": (
            stable_stage_a / "source_snapshot/arena_3d_v1/stable_pipeline.py"
        ).is_file(),
    }
    verification["equivalence_pass"] = all(
        verification[key] is True for key in (
            "row_key_sets_equal", "scheduler_backend_failure_cost_path_equivalent",
            "cache_binding_keys_and_fields_equal", "online_synchronous_build_zero",
            "partial_dstar_zero", "blocked_or_recovering_path_zero",
            "canonical_cost_error_zero", "stable_source_snapshot_present",
        )
    )
    (output / "verification.yaml").write_text(
        yaml.safe_dump(verification, sort_keys=False), encoding="utf-8",
    )
    (output / "manifest.yaml").write_text(
        yaml.safe_dump({
            **verification,
            "stable_manifest_sha256": sha256_file(stable_stage_a / "manifest.yaml"),
            "frozen_r2_manifest_sha256": sha256_file(frozen_r2 / "manifest.yaml"),
        }, sort_keys=False), encoding="utf-8",
    )
    (output / "reproduction_command.txt").write_text(
        "cd /home/robot/pudu_robot_ws\n"
        "PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/three_d_v1:"
        "/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation "
        f"/usr/bin/python3 -m arena_3d_v1.stable_acceptance equivalence --output-dir {output} "
        f"--stable-stage-a {stable_stage_a.resolve()} --frozen-r2 {frozen_r2.resolve()}\n",
        encoding="utf-8",
    )
    (output / "stdout.log").write_text(
        f"equivalence_pass={verification['equivalence_pass']} paired_rows={len(rows)}\n",
        encoding="utf-8",
    )
    (output / "stderr.log").write_text("", encoding="utf-8")
    if not verification["equivalence_pass"]:
        raise RuntimeError("stable/frozen-r2 Stage-A equivalence failed")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Independent RC acceptance runner for 3D-V1-r2-stable. Research arms "
            "live only here and are never imported by the production factory."
        )
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    stage_a = subparsers.add_parser("stage-a")
    stage_a.add_argument("--output-dir", type=Path, required=True)
    stage_a.add_argument("--query-ids", default=",".join(r2_stage_a.HELDOUT_QUERIES))
    stage_a.add_argument("--repetitions", type=int, default=10)
    stage_a.add_argument("--mode", choices=("calibration", "heldout"), default="heldout")
    soak = subparsers.add_parser("soak")
    soak.add_argument("--output-dir", type=Path, required=True)
    soak.add_argument("--query-ids", default=",".join(r2_soak.DEFAULT_QUERIES))
    soak.add_argument("--min-snapshots", type=int, default=5_000)
    soak.add_argument("--max-snapshots", type=int, default=10_000)
    soak.add_argument("--max-duration-s", type=float, default=1_800.0)
    stage_b = subparsers.add_parser("stage-b")
    stage_b.add_argument("--output-dir", type=Path, required=True)
    stage_b.add_argument("--heldout", type=Path, required=True)
    stage_b.add_argument("--query-ids", default=",".join(r2_stage_b.DEFAULT_QUERIES))
    stage_b.add_argument("--ros-domain-id", type=int, default=241)
    stage_b.add_argument("--costmap-ack-timeout-s", type=float, default=10.0)
    equivalence = subparsers.add_parser("equivalence")
    equivalence.add_argument("--output-dir", type=Path, required=True)
    equivalence.add_argument("--stable-stage-a", type=Path, required=True)
    equivalence.add_argument("--frozen-r2", type=Path, default=FROZEN_R2_HELDOUT)
    args = parser.parse_args()
    if args.command == "stage-a":
        run_stage_a(
            args.output_dir,
            query_ids=tuple(item for item in args.query_ids.split(",") if item),
            repetitions=args.repetitions,
            mode=args.mode,
        )
    elif args.command == "soak":
        run_soak(
            args.output_dir,
            query_ids=tuple(item for item in args.query_ids.split(",") if item),
            min_snapshots=args.min_snapshots,
            max_snapshots=args.max_snapshots,
            max_duration_s=args.max_duration_s,
        )
    elif args.command == "stage-b":
        run_stage_b(
            args.output_dir,
            heldout=args.heldout,
            query_ids=tuple(item for item in args.query_ids.split(",") if item),
            ros_domain_id=args.ros_domain_id,
            costmap_ack_timeout_s=args.costmap_ack_timeout_s,
        )
    else:
        compare_stage_a_equivalence(
            args.output_dir,
            stable_stage_a=args.stable_stage_a,
            frozen_r2=args.frozen_r2,
        )


if __name__ == "__main__":
    main()
