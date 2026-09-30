"""Static 20-query regression for 2D-V3 on the unchanged V2 substrate."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

import yaml

from . import layered_2d_v3_pipeline as v3
from . import two_layer_2d_v2_static_benchmark as v2_static
from .planner_benchmark.map_utils import sha256_file


ROOT = Path("/home/robot/pudu_robot_ws")
FROZEN_BASELINES = (
    ROOT / "experiments/layered_planner_benchmark/2d_v2_static_mentor_map_005_r0_20260903_154754",
    ROOT / "experiments/layered_planner_benchmark/2d_v2_dynamic_4x_area_r0_20260903_154947",
    ROOT / "experiments/layered_planner_benchmark/2d_v1_dynamic_incremental_value_v1_20260903_134619",
    ROOT / "experiments/layered_planner_benchmark/2d_v1_dynamic_incremental_4x_area_v1_20260903_150321",
    ROOT / "experiments/layered_planner_benchmark/2a_v1_mentor_map_20260825_005_20_r2_roi_pathaudit_v1",
)
ROS_DOMAIN_ID = 103


def _default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return ROOT / "experiments/layered_planner_benchmark" / f"2d_v3_static_mentor_map_005_r0_{stamp}"


def _tree_hash(directory: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(item for item in directory.rglob("*") if item.is_file()):
        digest.update(str(path.relative_to(directory)).encode("utf-8")); digest.update(b"\0")
        digest.update(bytes.fromhex(sha256_file(path)))
    return digest.hexdigest()


def _hashes() -> Dict[str, str]:
    return {str(path): _tree_hash(path) for path in FROZEN_BASELINES}


def _rewrite_csv_identity(path: Path) -> None:
    if not path.is_file():
        return
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle)); fields = list(rows[0]) if rows else []
    for field in ("architecture_id", "implementation_revision", "parent_architecture"):
        if field not in fields:
            fields.append(field)
    for row in rows:
        row.update({"architecture_id": v3.ARCHITECTURE_ID,
                    "implementation_revision": v3.IMPLEMENTATION_REVISION,
                    "parent_architecture": v3.PARENT_ARCHITECTURE})
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(rows)


def _upgrade_source_snapshot(output: Path) -> str:
    manifest_path = output / "source_snapshot_manifest.yaml"
    payload = yaml.safe_load(manifest_path.read_text()) or {}
    source = Path(__file__).resolve()
    target = output / "source_snapshot" / f"v3_{source.name}"
    shutil.copyfile(source, target)
    payload.setdefault("files", []).append({
        "source": str(source), "snapshot": str(target.relative_to(output)),
        "sha256": sha256_file(target), "bytes": target.stat().st_size,
    })
    payload["combined_hash"] = hashlib.sha256(json.dumps(
        [[row["snapshot"], row["sha256"]] for row in payload["files"]],
        sort_keys=True, separators=(",", ":"),
    ).encode("utf-8")).hexdigest()
    manifest_path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return payload["combined_hash"]


def run_formal(output: Path, *, warmups: int = 3, repetitions: int = 5,
               ros_domain_id: int = ROS_DOMAIN_ID) -> Path:
    output = output.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    before = _hashes()
    v2_static.run_formal(
        output, warmups=warmups, repetitions=repetitions,
        ros_domain_id=ros_domain_id, angle_bins=48,
        corridor_mode="adaptive_2m_4m",
    )
    for name in ("runs.csv", "backend_call_log.csv", "path_metrics.csv"):
        _rewrite_csv_identity(output / name)
    with (output / "runs.csv").open(newline="", encoding="utf-8") as handle:
        rows = [row for row in csv.DictReader(handle) if row.get("run_mode") == "measured"]
    success = [row for row in rows if row.get("final_valid_success", "").lower() == "true"]
    p50 = float(__import__("numpy").percentile([float(row["online_wall_ms"]) for row in success], 50))
    p95 = float(__import__("numpy").percentile([float(row["online_wall_ms"]) for row in success], 95))
    by_query = {query: [row for row in rows if row["query_id"] == query]
                for query in ("A2B-07", "A2B-16", "A2B-19")}
    strict_gates = {
        "final_valid_95_of_100": len(success) >= 95,
        "a2b07_5_of_5": sum(row.get("final_valid_success", "").lower() == "true" for row in by_query["A2B-07"]) == 5,
        "a2b16_truthful_l1_no_route": all(row.get("failure_code") == "L1_NO_ROUTE" for row in by_query["A2B-16"]),
        "accepted_paths_safe": all(
            row.get("static_footprint_valid", "").lower() == "true"
            and row.get("kinematic_valid", "").lower() == "true"
            and float(row.get("reverse_distance_m") or 0.0) <= 1e-9
            and int(float(row.get("in_place_rotation_count") or 0)) == 0
            and float(row.get("maximum_curvature") or 0.0) <= 2.5 + 1e-6
            for row in success
        ),
        "ack_100_percent": all(row.get("costmap_update_acknowledged", "").lower() == "true" for row in success),
        "mismatch_cells_zero": sum(int(float(row.get("costmap_ack_mismatch_cells") or 0)) for row in success) == 0,
        "settle_and_normal_clear_zero": all(
            float(row.get("costmap_settle_ms") or 0.0) == 0.0
            and float(row.get("local_costmap_clear_ms") or 0.0) == 0.0 for row in success
        ),
        "one_l3_call_per_routable_request": all(int(float(row.get("l3_call_count") or 0)) == 1 for row in success),
        "canonical_pathaudit_reused": all(
            row.get("canonical_path_audit_reused", "").lower() == "true"
            and row.get("final_validation_reused_canonical_audit", "").lower() == "true"
            for row in success
        ),
        "success_p50_within_v2_plus_5_percent": p50 <= 316.026 * 1.05,
        "success_p95_within_v2_plus_5_percent": p95 <= 464.482 * 1.05,
        "success_p50_ms": p50, "success_p95_ms": p95,
        "success_p50_limit_ms": 316.026 * 1.05,
        "success_p95_limit_ms": 464.482 * 1.05,
    }
    strict_gates["static_gate_pass"] = all(
        value for key, value in strict_gates.items() if isinstance(value, bool)
    )
    for name in ("protocol.yaml", "manifest.yaml"):
        path = output / name
        payload = yaml.safe_load(path.read_text()) or {}
        payload.update({"architecture_id": v3.ARCHITECTURE_ID,
                        "implementation_revision": v3.IMPLEMENTATION_REVISION,
                        "parent_architecture": v3.PARENT_ARCHITECTURE,
                        "static_runtime_semantics": "bit-for-bit V2 substrate regression",
                        "v3_dynamic_selector_active": False,
                        "v3_strict_static_gates": strict_gates})
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    source_hash = _upgrade_source_snapshot(output)
    after = _hashes()
    if before != after:
        raise AssertionError("frozen baseline changed during V3 static run")
    verification_path = output / "verification.yaml"
    verification = yaml.safe_load(verification_path.read_text()) or {}
    verification.update({"architecture_id": v3.ARCHITECTURE_ID,
                         "frozen_baselines_before": before,
                         "frozen_baselines_after": after,
                         "frozen_baselines_unchanged": before == after,
                         "v3_source_snapshot_hash": source_hash,
                         "v3_strict_static_gates": strict_gates})
    verification_path.write_text(yaml.safe_dump(verification, sort_keys=False), encoding="utf-8")
    report_path = output / "final_report.md"
    original = report_path.read_text(encoding="utf-8")
    report_path.write_text(
        "# 2D-V3 static regression\n\n"
        f"- V3 strict static gate: **{'PASS' if strict_gates['static_gate_pass'] else 'FAIL'}**.\n"
        f"- Success P50/P95: {p50:.3f}/{p95:.3f} ms; limits "
        f"{strict_gates['success_p50_limit_ms']:.3f}/{strict_gates['success_p95_limit_ms']:.3f} ms.\n\n"
        "This is a fresh execution of the frozen 2D-V2 engineering substrate. "
        "The V3 dynamic selector is inactive because `dynamic_obstacles=false`; "
        "48 bins, adaptive 2/4 m corridor, ROI content ACK, settle=0, normal "
        "clear=0, and canonical PathAudit remain unchanged.\n\n"
        + original.replace("# 2D-V2", "## Reused V2 runtime report", 1),
        encoding="utf-8",
    )
    reproduction = (
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        f"ROS_DOMAIN_ID={ros_domain_id} ros2 run arena_evaluation two_layer_2d_v3_static_benchmark "
        "--output-dir /tmp/REPLACE_WITH_NEW_WRITE_ONCE_DIR --warmups 3 --repetitions 5 "
        f"--ros-domain-id {ros_domain_id} --no-dynamic-obstacles\n"
    )
    (output / "reproduction_command.txt").write_text(reproduction, encoding="utf-8")
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--warmups", type=int, default=3)
    parser.add_argument("--repetitions", type=int, default=5)
    parser.add_argument("--ros-domain-id", type=int, default=ROS_DOMAIN_ID)
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output = run_formal(args.output_dir or _default_output(), warmups=args.warmups,
                            repetitions=args.repetitions, ros_domain_id=args.ros_domain_id)
    except Exception as exc:
        print(f"two_layer_2d_v3_static_benchmark: ERROR: {exc}")
        return 2
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
