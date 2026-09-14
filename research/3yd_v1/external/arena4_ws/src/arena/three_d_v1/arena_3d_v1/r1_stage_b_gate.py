"""Record conditional Stage-B disposition from the frozen held-out L2 gate."""

from __future__ import annotations

import argparse
import json
import os
import shutil
from pathlib import Path
from typing import Any, Dict

import yaml

from .l2_state_lifecycle import ARCHITECTURE_ID, PROTOCOL_ID, REVISION_ID
from .r1_stage_a import (
    DEFAULT_FROZEN_CONFIG,
    _load_frozen_config,
    _sha256,
    _source_snapshot,
)


def _preexisting_ros_processes() -> list[dict[str, Any]]:
    result = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace",
            ).strip()
        except OSError:
            continue
        if any(token in command for token in (
            "/nav2_", "planner_benchmark_stack.launch.py", "/smac_",
        )):
            result.append({"pid": int(entry.name), "command": command})
    return sorted(result, key=lambda value: value["pid"])


def run(
    output: Path, *, heldout: Path, frozen_config: Path = DEFAULT_FROZEN_CONFIG,
) -> Path:
    output = output.resolve()
    heldout = heldout.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    frozen = _load_frozen_config(frozen_config)
    gate_path = heldout / "gate_results.yaml"
    manifest_path = heldout / "manifest.yaml"
    if not gate_path.is_file() or not manifest_path.is_file():
        raise ValueError("held-out gate evidence is incomplete")
    gates: Dict[str, Any] = yaml.safe_load(gate_path.read_text(encoding="utf-8")) or {}
    if gates.get("mode") != "heldout":
        raise ValueError("Stage-B gate input is not held-out evidence")
    if gates.get("stage_a_pass") is True:
        raise RuntimeError("L2 gate passed; a real ROS/Nav2 Stage-B runner is required")
    hard_gate_keys = (
        "oracle_parity_pass", "scheduler_parity_pass", "recovery_pass",
        "p50_gate_pass", "p95_gate_pass", "p99_gate_pass",
        "warm_activation_gate_pass", "resident_reduction_gate_pass",
        "resident_target_pass", "lru_bound_pass", "cache_hit_pass",
        "frozen_baselines_unchanged",
    )
    failed = sorted(key for key in hard_gate_keys if gates.get(key) is not True)
    target_misses = sorted(
        key for key in ("p95_target_pass", "p99_target_pass")
        if gates.get(key) is False
    )
    preexisting_ros = _preexisting_ros_processes()
    output.mkdir(parents=True)
    marker = (
        "NOT_RUN_L2_GATE_FAILED\n"
        f"failed_gates={','.join(failed)}\n"
        "Smac/Nav2 was not started; L3 tuning cannot mask an L2 held-out failure.\n"
    )
    (output / "NOT_RUN_L2_GATE_FAILED").write_text(marker, encoding="utf-8")
    shutil.copy2(gate_path, output / "heldout_gate_results.yaml")
    source_hashes = _source_snapshot(output)
    status = {
        "status": "NOT_RUN_L2_GATE_FAILED",
        "failed_hard_gates": failed,
        "non_blocking_target_misses": target_misses,
        "heldout_directory": str(heldout),
        "heldout_gate_sha256": _sha256(gate_path),
        "heldout_manifest_sha256": _sha256(manifest_path),
        "stage_b_processes_started": 0,
        "smac_queries_executed": 0,
        "preexisting_ros_processes_observed": preexisting_ros,
        "preexisting_processes_not_attributed_to_this_run": True,
        "reason": "frozen held-out Stage-A hard gate failed",
    }
    (output / "stage_b_status.yaml").write_text(
        yaml.safe_dump(status, sort_keys=False), encoding="utf-8",
    )
    manifest = {
        "architecture_id": ARCHITECTURE_ID,
        "revision_id": REVISION_ID,
        "protocol_id": PROTOCOL_ID,
        "frozen_config": str(frozen_config.resolve()),
        "frozen_config_sha256": _sha256(frozen_config),
        "heldout_directory": str(heldout),
        "heldout_gate_sha256": _sha256(gate_path),
        "workload_classification": frozen["workload"]["classification"],
        "source_files": source_hashes,
    }
    (output / "manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8",
    )
    verification = {
        "heldout_stage_a_pass": False,
        "marker_present": True,
        "stage_b_not_started": True,
        "stage_b_child_process_count": 0,
        "preexisting_ros_process_count": len(preexisting_ros),
        "preexisting_processes_left_untouched": True,
        "protocol_compliant": True,
    }
    (output / "verification.yaml").write_text(
        yaml.safe_dump(verification, sort_keys=False), encoding="utf-8",
    )
    command = (
        "cd /home/robot/pudu_robot_ws\n"
        "PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/three_d_v1:/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation "
        f"/usr/bin/python3 -m arena_3d_v1.r1_stage_b_gate --output-dir {output} "
        f"--heldout {heldout} --frozen-config {frozen_config.resolve()}\n"
    )
    (output / "reproduction_command.txt").write_text(command, encoding="utf-8")
    (output / "stdout.log").write_text(
        json.dumps(status, sort_keys=True) + "\n", encoding="utf-8",
    )
    (output / "stderr.log").write_text("", encoding="utf-8")
    print(json.dumps(status, sort_keys=True))
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--heldout", type=Path, required=True)
    parser.add_argument("--frozen-config", type=Path, default=DEFAULT_FROZEN_CONFIG)
    args = parser.parse_args()
    try:
        run(args.output_dir, heldout=args.heldout, frozen_config=args.frozen_config)
    except Exception as exc:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / "INTERRUPTED_RUN.md").write_text(
            f"# Interrupted run\n\n`{type(exc).__name__}: {exc}`\n",
            encoding="utf-8",
        )
        (args.output_dir / "stderr.log").write_text(
            f"{type(exc).__name__}: {exc}\n", encoding="utf-8",
        )
        raise


if __name__ == "__main__":
    main()
