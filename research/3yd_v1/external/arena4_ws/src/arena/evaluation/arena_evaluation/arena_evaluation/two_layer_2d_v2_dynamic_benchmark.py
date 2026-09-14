"""2D-V2 4x-area dynamic value experiment.

Stage A intentionally calls the frozen paired L1 harness because 2D-V2 keeps
the 2D-V1-r3 D* core, attachment policy, state machine, Graph A* oracle and
scenario generator unchanged.  The wrapper changes no algorithm input and
adds the V2 identity, frozen-input audit and V2 artifact contract.  Stage B is
hard-gated and must never start after a Stage-A failure.
"""

from __future__ import annotations

import argparse
import csv
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

import yaml

from . import layered_2d_v2_pipeline as v2
from . import two_layer_2d_v1_4x_dynamic_incremental_benchmark as stage_a
from . import two_layer_2d_v2_static_benchmark as static
from .planner_benchmark.map_utils import sha256_file


ROOT = Path("/home/robot/pudu_robot_ws")
ARCHITECTURE_ID = v2.ARCHITECTURE_ID
IMPLEMENTATION_REVISION = v2.IMPLEMENTATION_REVISION
PARENT_ARCHITECTURE = v2.PARENT_ARCHITECTURE
PROTOCOL_VERSION = v2.PROTOCOL_VERSION
DEFAULT_ROS_DOMAIN_ID = 119
DEFAULT_SEED = stage_a.DEFAULT_SEED
DEFAULT_WARMUPS = stage_a.DEFAULT_WARMUPS
DEFAULT_REPETITIONS = stage_a.DEFAULT_REPETITIONS
DEFAULT_MAIN_QUERY_COUNT = stage_a.DEFAULT_MAIN_QUERY_COUNT
FROZEN = (static.FROZEN_2A_R2, static.FROZEN_2D_R2, static.FROZEN_2D_R3)
CONFIG = Path(__file__).resolve().parents[1] / "config/two_layer_2d_v2_r0_enhanced.yaml"


def _default_output() -> Path:
    return ROOT / "experiments/layered_planner_benchmark" / (
        f"2d_v2_dynamic_4x_area_r0_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )


def _rewrite_csv_identity(path: Path) -> None:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = list(reader.fieldnames or [])
        rows = list(reader)
    for field in ("architecture_id", "implementation_revision", "parent_architecture"):
        if field not in fields:
            fields.append(field)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            row.update({"architecture_id": ARCHITECTURE_ID,
                        "implementation_revision": IMPLEMENTATION_REVISION,
                        "parent_architecture": PARENT_ARCHITECTURE})
            writer.writerow(row)


def _extend_source_snapshot(output: Path) -> Dict[str, Any]:
    path = output / "source_snapshot_manifest.yaml"
    payload = yaml.safe_load(path.read_text()) or {}
    records = list(payload.get("files") or [])
    sources = [Path(__file__).resolve(), Path(v2.__file__).resolve(), CONFIG,
               Path(__file__).resolve().parents[1] / "test/test_layered_2d_v2_pipeline.py",
               Path(__file__).resolve().parents[1] / "test/test_two_layer_2d_v2_benchmark.py"]
    directory = output / "source_snapshot"
    start = len(records)
    for offset, source in enumerate(sources):
        target = directory / f"{start + offset:02d}_{source.name}"
        shutil.copyfile(source, target)
        records.append({"source": str(source), "snapshot": str(target.relative_to(output)),
                        "sha256": sha256_file(target), "bytes": target.stat().st_size})
    payload.update({"file_count": len(records), "files": records,
                    "combined_hash": v2.stable_hash([[row["snapshot"], row["sha256"]] for row in records])})
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return payload


def _write_v2_report(output: Path, manifest: Mapping[str, Any], static_manifest: Mapping[str, Any]) -> str:
    gates = dict(manifest["stage_a"])
    static_pass = bool(static_manifest.get("summary", {}).get("static_gate_pass"))
    if not static_pass:
        verdict = "C"
        decision = "静态正确性、ACK 或路径验收门槛未通过；拒绝 2D-V2，保留 2A-V1-r2。"
    elif not gates.get("stage_a_pass"):
        verdict = "B"
        decision = "V2 工程底座有效，但 D* 在 4x Stage A 未通过；底座应并回 2A，D* 转向 3D-V0 L2。"
    else:
        verdict = "A" if manifest.get("stage_b_status") == "PASSED_ENGINEERING_GATE" else "B"
        decision = ("2D-V2 可作为大规模动态地图候选。" if verdict == "A" else
                    "D* 只有算法收益或端到端收益不足，不替换 Graph A*。")
    lines = [
        "# 2D-V2 r0 4x-area dynamic incremental experiment", "",
        f"- Final predefined verdict: **{verdict}**. {decision}",
        f"- Stage A correctness: {gates['correctness_rows'] - gates['correctness_failures']}/"
        f"{gates['correctness_rows']}; all controls failures={gates['oracle_failures_total']}.",
        f"- Path-affected expanded-node P50 reduction: {100*gates['expanded_nodes_p50_reduction']:.2f}% "
        f"(gate={gates['expanded_nodes_gate_pass']}).",
        f"- Complete L1 P50 reduction vs cold Graph A*: {100*gates['full_l1_p50_reduction']:.2f}% "
        f"(gate={gates['full_l1_p50_gate_pass']}).",
        f"- Complete L1 P95 D*/A*: {gates['incremental_full_l1_p95_ms']:.4f}/"
        f"{gates['cold_graph_astar_full_l1_p95_ms']:.4f} ms (gate={gates['p95_gate_pass']}).",
        f"- Implicit D* reinitialize: {not gates['incremental_no_reinitialize']}.",
        f"- Stage B: `{manifest['stage_b_status']}`. A Stage-A failure makes ROS startup impermissible.",
        "", "## Attribution", "",
        "- Stage A compares only persistent D* Lite against deterministic cold Graph A* on identical graph/snapshots/changed edges.",
        "- ROI/ACK, 48 bins, adaptive corridor, PathAudit and scheduler skips are not attributed to D*.",
        "- Complete L1 includes snapshot parse, cell→edge lookup, transition, update_edges, search and route extraction.",
        "", "## Reproduction", "", "```bash",
        "source /opt/ros/humble/setup.bash",
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash",
        "v2_dyn=/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2d_v2_dynamic_4x_area_r0_$(date +%Y%m%d_%H%M%S)",
        f"ROS_DOMAIN_ID={DEFAULT_ROS_DOMAIN_ID} ros2 run arena_evaluation two_layer_2d_v2_dynamic_benchmark --output-dir \"$v2_dyn\" --static-result {static_manifest.get('output_directory', '<STATIC_RESULT>')} --warmups {DEFAULT_WARMUPS} --repetitions {DEFAULT_REPETITIONS} --main-query-count {DEFAULT_MAIN_QUERY_COUNT} --seed {DEFAULT_SEED} --ros-domain-id {DEFAULT_ROS_DOMAIN_ID}",
        "```", "",
    ]
    (output / "final_report.md").write_text("\n".join(lines), encoding="utf-8")
    return verdict


def run_formal(output: Path, *, static_result: Path, warmups: int = DEFAULT_WARMUPS,
               repetitions: int = DEFAULT_REPETITIONS,
               main_query_count: int = DEFAULT_MAIN_QUERY_COUNT,
               seed: int = DEFAULT_SEED,
               ros_domain_id: int = DEFAULT_ROS_DOMAIN_ID) -> Path:
    static_result = static_result.resolve()
    static_manifest_path = static_result / "manifest.yaml"
    if not static_manifest_path.is_file():
        raise FileNotFoundError(f"missing V2 static manifest: {static_manifest_path}")
    static_manifest = yaml.safe_load(static_manifest_path.read_text()) or {}
    if static_manifest.get("architecture_id") != ARCHITECTURE_ID:
        raise ValueError("static result is not 2D-V2")
    static_manifest["output_directory"] = str(static_result)
    frozen_before = {str(path): static._tree_hash(path) for path in FROZEN}
    # The V2 Stage-A input is intentionally identical to the frozen r4 paired
    # algorithm harness.  Force Stage-A-only there; this wrapper alone owns the
    # V2 Stage-B admission decision.
    stage_a.run_formal(
        output, warmups=warmups, repetitions=repetitions,
        main_query_count=main_query_count, seed=seed,
        ros_domain_id=ros_domain_id, ros_repetitions=3, stage_a_only=True,
    )
    manifest_path = output / "manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text()) or {}
    gates = dict(manifest.get("stage_a") or {})
    if gates.get("stage_a_pass"):
        # Starting the V1 downstream harness would invalidate attribution.
        # Admission remains explicit so an unexpected pass cannot silently run
        # the wrong ROS implementation.
        manifest["stage_b_status"] = "NOT_RUN_V2_STAGE_B_IMPLEMENTATION_REQUIRED"
    else:
        manifest["stage_b_status"] = "NOT_RUN_STAGE_A_FAILED"
    for name in ("runs.csv", "correctness_oracle.csv", "timing_summary.csv",
                 "expanded_nodes_summary.csv", "per_scenario_summary.csv",
                 "memory_summary.csv", "topology_scale_comparison.csv",
                 "scenario_manifest.csv", "paired_ros_runs.csv"):
        path = output / name
        if path.is_file():
            _rewrite_csv_identity(path)
    shutil.copyfile(output / "runs.csv", output / "paired_algorithm_runs.csv")
    shutil.copyfile(output / "timing_summary.csv", output / "phase_timing_summary.csv")
    shutil.copyfile(output / "per_scenario_summary.csv", output / "per_scenario_results.csv")
    with (output / "cache_diagnostics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["component", "source", "status"])
        writer.writeheader()
        writer.writerow({"component": "persistent_dstar_state", "source": "memory_summary.csv", "status": "measured"})
        writer.writerow({"component": "cell_to_edge_index", "source": "topology_scale_comparison.csv", "status": "measured"})
    with (output / "ack_diagnostics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["stage", "status", "reason"])
        writer.writeheader()
        writer.writerow({"stage": "B", "status": manifest["stage_b_status"],
                         "reason": "Stage A failed; no ROS/ACK process was started" if not gates.get("stage_a_pass") else "V2 Stage B not executed"})
    protocol_path = output / "protocol.yaml"
    protocol = yaml.safe_load(protocol_path.read_text()) or {}
    protocol.update({"architecture_id": ARCHITECTURE_ID,
                     "implementation_revision": IMPLEMENTATION_REVISION,
                     "parent_architecture": PARENT_ARCHITECTURE,
                     "status": "candidate", "experiment_kind": "dynamic_incremental",
                     "stage_b_status": manifest["stage_b_status"],
                     "v2_downstream": {"corridor": "adaptive_2m_4m", "roi_content_ack": True,
                                       "angle_quantization_bins": 48, "fixed_settle_cycles": 0,
                                       "canonical_path_audit": True},
                     "stage_a_shared_downstream_condition": "not invoked by either arm"})
    protocol_path.write_text(yaml.safe_dump(protocol, sort_keys=False), encoding="utf-8")
    snapshot = _extend_source_snapshot(output)
    frozen_after = {name: static._tree_hash(Path(name)) for name in frozen_before}
    if frozen_after != frozen_before:
        raise RuntimeError("a frozen formal experiment changed during V2 dynamic run")
    manifest.update({"architecture_id": ARCHITECTURE_ID,
                     "implementation_revision": IMPLEMENTATION_REVISION,
                     "parent_architecture": PARENT_ARCHITECTURE,
                     "static_result": str(static_result),
                     "frozen_directory_hashes_before": frozen_before,
                     "frozen_directory_hashes_after": frozen_after,
                     "frozen_directories_unchanged": True,
                     "source_snapshot_file_count": snapshot["file_count"],
                     "source_snapshot_hash": snapshot["combined_hash"]})
    verdict = _write_v2_report(output, manifest, static_manifest)
    manifest["final_verdict"] = verdict
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    verification_path = output / "verification.yaml"
    verification = yaml.safe_load(verification_path.read_text()) or {}
    verification.update({"architecture_id": ARCHITECTURE_ID,
                         "implementation_revision": IMPLEMENTATION_REVISION,
                         "frozen_directory_hashes_before": frozen_before,
                         "frozen_directory_hashes_after": frozen_after,
                         "frozen_references_unchanged": True,
                         "stage_b_status": manifest["stage_b_status"],
                         "v2_required_artifacts": "pending_post_run_validation"})
    verification_path.write_text(yaml.safe_dump(verification, sort_keys=False), encoding="utf-8")
    required = ("paired_algorithm_runs.csv", "phase_timing_summary.csv",
                "per_scenario_results.csv", "cache_diagnostics.csv", "ack_diagnostics.csv")
    missing = [name for name in required if not (output / name).is_file()]
    if missing:
        raise RuntimeError(f"missing V2 dynamic artifacts: {missing}")
    verification["v2_required_artifacts"] = "passed"
    verification_path.write_text(yaml.safe_dump(verification, sort_keys=False), encoding="utf-8")
    (output / "stdout.log").write_text(
        (output / "stdout.log").read_text() + f"v2_verdict={verdict}\n",
        encoding="utf-8",
    )
    reproduction = (
        "source /opt/ros/humble/setup.bash\n"
        "source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash\n"
        "v2_dyn=/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/"
        "2d_v2_dynamic_4x_area_r0_$(date +%Y%m%d_%H%M%S)\n"
        f"ROS_DOMAIN_ID={ros_domain_id} ros2 run arena_evaluation two_layer_2d_v2_dynamic_benchmark "
        f"--output-dir \"$v2_dyn\" --static-result {static_result} --warmups {warmups} "
        f"--repetitions {repetitions} --main-query-count {main_query_count} --seed {seed} "
        f"--ros-domain-id {ros_domain_id}\n"
    )
    (output / "reproduction_command.txt").write_text(reproduction, encoding="utf-8")
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run 2D-V2 r0 4x-area dynamic incremental experiment")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--static-result", type=Path, required=True)
    parser.add_argument("--warmups", type=int, default=DEFAULT_WARMUPS)
    parser.add_argument("--repetitions", type=int, default=DEFAULT_REPETITIONS)
    parser.add_argument("--main-query-count", type=int, default=DEFAULT_MAIN_QUERY_COUNT)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--ros-domain-id", type=int, default=DEFAULT_ROS_DOMAIN_ID)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output = run_formal(args.output_dir or _default_output(), static_result=args.static_result,
                            warmups=args.warmups, repetitions=args.repetitions,
                            main_query_count=args.main_query_count, seed=args.seed,
                            ros_domain_id=args.ros_domain_id)
    except Exception as exc:
        print(f"two_layer_2d_v2_dynamic_benchmark: ERROR: {exc}")
        return 2
    print(f"2D-V2 dynamic output: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
