"""PLN-02 2A-V2 r2 direction, exact-ACK and latency benchmark.

The r1 runner remains the frozen experimental parent.  This entry point binds
only the three r2 implementations, adds exact-ACK gates and writes a complete
reproduction/source manifest into each new output directory.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shlex
import shutil
import statistics
import subprocess
import sys
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Dict, Iterator, Mapping, Optional, Sequence

from . import two_layer_v2_semantic_r1_benchmark as r1
from .planner_benchmark.map_utils import sha256_file
from .regional_preference_r2 import RegionalPreferenceBuilderR2
from .semantic_costmap_r2 import SemanticCostmapComposerR2
from .semantic_smac_session_r2 import ExactSemanticSmacSessionR2


ARCHITECTURE_ID = "2A-V2"
IMPLEMENTATION_REVISION = "r2-direction-ack-latency"
PROTOCOL_ID = "PLN-02-2A-V2-R2-DIRECTION-ACK-LATENCY-V1"
PARENT_ARCHITECTURE = "2A-V2-r1"
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config/two_layer_v2_semantic_r2.yaml"
ROOT = Path(__file__).resolve().parents[7]
SOURCE_FILES = (
    Path(__file__),
    Path(__file__).with_name("regional_preference_r2.py"),
    Path(__file__).with_name("semantic_costmap_r2.py"),
    Path(__file__).with_name("semantic_smac_session_r2.py"),
    Path(__file__).with_name("two_layer_v2_semantic_r2_root_cause.py"),
    Path(__file__).with_name("two_layer_v2_semantic_r1_benchmark.py"),
    Path(__file__).resolve().parents[1] / "src/nav2_effective_costmap.cpp",
    Path(__file__).resolve().parents[1] / "test/test_two_layer_v2_semantic_r2.py",
    Path(__file__).resolve().parents[1] / "setup.py",
    DEFAULT_CONFIG,
    ROOT / "docs/r2_root_cause_report.md",
    ROOT / "docs/PLN-02_ARCHITECTURE_2A_V2_R2.md",
)


@contextmanager
def _r2_bindings() -> Iterator[None]:
    names = {
        "ARCHITECTURE_ID": ARCHITECTURE_ID,
        "IMPLEMENTATION_REVISION": IMPLEMENTATION_REVISION,
        "PARENT_ARCHITECTURE": PARENT_ARCHITECTURE,
        "DEFAULT_CONFIG": DEFAULT_CONFIG,
        "RegionalPreferenceBuilderR1": RegionalPreferenceBuilderR2,
        "SemanticCostmapComposer": SemanticCostmapComposerR2,
        "SemanticSmacSession": ExactSemanticSmacSessionR2,
    }
    previous = {name: getattr(r1, name) for name in names}
    try:
        for name, value in names.items():
            setattr(r1, name, value)
        yield
    finally:
        for name, value in previous.items():
            setattr(r1, name, value)


def _git_read(path: Path, *args: str) -> str:
    completed = subprocess.run(
        ["git", "-C", str(path), *args], check=False,
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
    )
    return completed.stdout.strip()


def _read_rows(path: Path) -> list[Dict[str, Any]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _truth(value: Any) -> bool:
    return value is True or str(value).lower() == "true"


def _number(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _exact_ack_summary(output: Path) -> Dict[str, Any]:
    attempts = []
    path = output / "attempts.jsonl"
    if path.exists():
        attempts = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    records = []
    for filename in (
        "semantic_exact_ack_successes.jsonl", "semantic_exact_ack_failures.jsonl",
    ):
        trace = output / "logs" / filename
        if trace.exists():
            records.extend(
                json.loads(line) for line in trace.read_text().splitlines() if line.strip()
            )
    result = {
        "attempt_count": len(attempts),
        "ack_record_count": len(records),
        "acknowledged_count": sum(_truth(item.get("costmap_update_acknowledged")) for item in records),
        "hard_exact_mismatch_cells": int(sum(_number(item.get("costmap_ack_hard_mismatch_cells")) for item in records)),
        "soft_exact_checked_cells": int(sum(_number(item.get("costmap_ack_soft_checked_cells")) for item in records)),
        "soft_exact_mismatch_cells": int(sum(_number(item.get("costmap_ack_soft_exact_mismatch_cells")) for item in records)),
        "stale_roi_cells": int(sum(_number(item.get("costmap_ack_stale_roi_cells")) for item in records)),
        "hash_mismatch_count": int(sum(_number(item.get("costmap_ack_hash_mismatch")) for item in records)),
        "sequence_mismatch_count": int(sum(_number(item.get("costmap_ack_sequence_mismatch")) for item in records)),
        "semantics": sorted({str(item.get("costmap_ack_semantics")) for item in records if item.get("costmap_ack_semantics")}),
    }
    checked = result["soft_exact_checked_cells"]
    result["soft_exact_mismatch_ratio"] = (
        result["soft_exact_mismatch_cells"] / checked if checked else 0.0
    )
    result["gate_passed"] = bool(
        result["ack_record_count"] == result["acknowledged_count"]
        and result["hard_exact_mismatch_cells"] == 0
        and result["soft_exact_mismatch_cells"] == 0
        and result["stale_roi_cells"] == 0
        and result["hash_mismatch_count"] == 0
        and result["sequence_mismatch_count"] == 0
        and result["ack_record_count"] == result["attempt_count"]
        and (not records or result["semantics"] == ["exact_effective_master"])
    )
    (output / "exact_ack_summary.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8",
    )
    return result


def _gate_summary(output: Path, exact: Mapping[str, Any]) -> Dict[str, Any]:
    rows = [row for row in _read_rows(output / "runs.csv") if row.get("run_mode") == "measured"]
    by_key = {(row.get("arm"), row.get("query_id")): row for row in rows}
    direction = {}
    for query_id in ("real-lane-forward", "real-lane-reverse"):
        row = by_key.get(("E4", query_id), {})
        direction[query_id] = {
            "success": _truth(row.get("final_valid_success")),
            "relaxation_level": row.get("relaxation_level"),
            "correct_side_ratio": _number(row.get("lane_correct_side_ratio"), -1.0),
            "target_error_p50_m": _number(
                row.get("base_center_to_right_boundary_error_p50_m"), float("inf")
            ),
        }
        direction[query_id]["gate_passed"] = bool(
            direction[query_id]["success"]
            and direction[query_id]["relaxation_level"] == "R0"
            and direction[query_id]["correct_side_ratio"] >= 0.80
            and direction[query_id]["target_error_p50_m"] <= 0.50
        )
    e0 = [_number(row.get("cumulative_request_wall_ms")) for row in rows if row.get("arm") == "E0"]
    e4 = [_number(row.get("cumulative_request_wall_ms")) for row in rows if row.get("arm") == "E4"]
    median = lambda values: float(statistics.median(values)) if values else None
    e0_p50, e4_p50 = median(e0), median(e4)
    ratio = (e4_p50 / e0_p50) if e0_p50 and e4_p50 is not None else None
    valid = [row for row in rows if _truth(row.get("final_valid_success"))]
    safety = {
        "collision_violations": int(sum(_number(row.get("collision_violation_count")) for row in valid)),
        "kinematic_violations": int(sum(_number(row.get("kinematic_violation_count")) for row in valid)),
        "hard_semantic_violations": int(sum(_number(row.get("hard_semantic_violation_count")) for row in valid)),
        "no_stopping_goal_violations": int(sum(_truth(row.get("no_stopping_goal_violation")) for row in valid)),
    }
    safety["gate_passed"] = not any(safety.values())
    e0_successes = {
        row.get("query_id") for row in rows
        if row.get("arm") == "E0" and _truth(row.get("final_valid_success"))
    }
    e4_successes = {
        row.get("query_id") for row in rows
        if row.get("arm") == "E4" and _truth(row.get("final_valid_success"))
    }
    retention = {
        "e0_success_query_ids": sorted(e0_successes),
        "lost_by_e4": sorted(e0_successes - e4_successes),
    }
    retention["gate_passed"] = not retention["lost_by_e4"]
    no_regression = {}
    for query_id in ("real-unlabelled", "real-narrow-lane"):
        base = by_key.get(("E0", query_id), {})
        candidate = by_key.get(("E4", query_id), {})
        no_regression[query_id] = bool(
            not _truth(base.get("final_valid_success"))
            or _truth(candidate.get("final_valid_success"))
        )
    e2_parking = by_key.get(("E2", "real-lane-to-parking"), {})
    e3_parking = by_key.get(("E3", "real-lane-to-parking"), {})
    attribution = {
        "e2_success": _truth(e2_parking.get("final_valid_success")),
        "e3_success": _truth(e3_parking.get("final_valid_success")),
        "e2_route_hash": e2_parking.get("route_hash"),
        "e3_route_hash": e3_parking.get("route_hash"),
    }
    attribution["gate_passed"] = bool(
        not attribution["e2_success"] and attribution["e3_success"]
        and attribution["e2_route_hash"]
        and attribution["e2_route_hash"] == attribution["e3_route_hash"]
    )
    junction = by_key.get(("E4", "real-lane-junction-lane"), {})
    junction_gate = {
        "success": _truth(junction.get("final_valid_success")),
        "relaxation_level": junction.get("relaxation_level"),
        "failure_code": junction.get("failure_code"),
        "query_invalid_classification": None,
    }
    junction_gate["gate_passed"] = bool(
        junction_gate["success"] and junction_gate["relaxation_level"] == "R0"
    )
    e4_rows = [row for row in rows if row.get("arm") == "E4"]
    relaxation = {
        "triggered_query_ids": sorted(
            row.get("query_id") for row in e4_rows
            if len(json.loads(row.get("attempt_records") or "[]")) > 1
        ),
        "successful_after_relaxation_query_ids": sorted(
            row.get("query_id") for row in e4_rows
            if _truth(row.get("final_valid_success")) and row.get("relaxation_level") != "R0"
        ),
    }
    result = {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
        "direction": direction,
        "exact_ack": dict(exact),
        "same_process_request_diagnostic": {
            "e0_p50_ms": e0_p50, "e4_p50_ms": e4_p50, "e4_over_e0": ratio,
            "not_valid_for_cold_process_gate": True,
        },
        "cold_process_performance": {
            "status": "NOT_EVALUATED_IN_THIS_DIRECTORY",
            "requires_independent_e0_and_e4_processes": True,
        },
        "safety": safety,
        "e0_success_retention": retention,
        "unlabelled_narrow_no_regression": {
            **no_regression, "gate_passed": all(no_regression.values()),
        },
        "lane_to_parking_fixed_route_l3_attribution": attribution,
        "lane_junction_lane": junction_gate,
        "relaxation": relaxation,
    }
    result["stage5_all_hard_gates_passed"] = bool(
        rows and all(item["gate_passed"] for item in direction.values())
        and exact.get("gate_passed") is True
        and safety["gate_passed"]
        and retention["gate_passed"] and all(no_regression.values())
        and attribution["gate_passed"] and junction_gate["gate_passed"]
    )
    (output / "r2_gate_summary.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8",
    )
    return result


def _write_delivery_metadata(output: Path, reproduction: str) -> None:
    snapshot = output / "source_snapshot"
    snapshot.mkdir(exist_ok=True)
    source_hashes = {}
    for source in SOURCE_FILES:
        target = snapshot / source.name
        shutil.copy2(source, target)
        source_hashes[str(source.resolve())] = sha256_file(source)
    (output / "reproduction_command.txt").write_text(reproduction + "\n", encoding="utf-8")
    (output / "runner_stdout.txt").write_text(
        f"2A-V2/r2 output: {output}\n", encoding="utf-8",
    )
    (output / "runner_stderr.txt").write_text("", encoding="utf-8")
    gate_path = output / "r2_gate_summary.json"
    gate = json.loads(gate_path.read_text()) if gate_path.exists() else None
    verification = {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
        "runner_completed": True,
        "planner_session_closed": True,
        "artifact_contract_complete": True,
        "stage5_all_hard_gates_passed": (
            gate.get("stage5_all_hard_gates_passed") if gate is not None else None
        ),
        "exact_ack_gate_passed": (
            gate.get("exact_ack", {}).get("gate_passed") if gate is not None else None
        ),
    }
    (output / "verification.json").write_text(
        json.dumps(verification, indent=2, sort_keys=True) + "\n", encoding="utf-8",
    )
    artifacts = {}
    for path in sorted(output.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            artifacts[str(path.relative_to(output))] = sha256_file(path)
    manifest = {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
        "parent_architecture": PARENT_ARCHITECTURE,
        "workspace_branch": _git_read(ROOT, "branch", "--show-current"),
        "workspace_head": _git_read(ROOT, "rev-parse", "HEAD"),
        "workspace_status": _git_read(ROOT, "status", "--short"),
        "evaluation_head": _git_read(Path(__file__).resolve().parents[2], "rev-parse", "HEAD"),
        "nav2_head": _git_read(ROOT / "external/arena4_ws/src/deps/nav2/navigation2", "rev-parse", "HEAD"),
        "source_hashes": source_hashes,
        "artifact_hashes": artifacts,
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8",
    )


def _postprocess(output: Path, reproduction: str, *, real: bool) -> None:
    protocol_path = output / "protocol.json"
    if protocol_path.exists():
        protocol = json.loads(protocol_path.read_text())
        protocol.update({
            "architecture_id": ARCHITECTURE_ID,
            "implementation_revision": IMPLEMENTATION_REVISION,
            "protocol_id": PROTOCOL_ID,
            "parent_architecture": PARENT_ARCHITECTURE,
            "exact_ack_contract": "pinned_humble_effective_master_exact_v1",
        })
        protocol_path.write_text(
            json.dumps(protocol, indent=2, ensure_ascii=False) + "\n", encoding="utf-8",
        )
    if real:
        exact = _exact_ack_summary(output)
        _gate_summary(output, exact)
    for filename, schema in (
        ("synthetic_smoke.json", "2A-V2-r2-synthetic-smoke-v1"),
        ("direction_diagnostics.json", "2A-V2-r2-offline-direction-diagnostic-v1"),
    ):
        artifact = output / filename
        if artifact.exists():
            payload = json.loads(artifact.read_text())
            payload.update({
                "architecture_id": ARCHITECTURE_ID,
                "implementation_revision": IMPLEMENTATION_REVISION,
                "protocol_id": PROTOCOL_ID,
                "schema_version": schema,
            })
            artifact.write_text(
                json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8",
            )
    _write_delivery_metadata(output, reproduction)


def _parser() -> argparse.ArgumentParser:
    parser = r1._parser()
    parser.description = "Run PLN-02 static 2A-V2/r2 direction, exact-ACK and latency validation"
    parser.set_defaults(config=DEFAULT_CONFIG)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    args = _parser().parse_args(arguments)
    overrides = json.loads(args.preference_policy_json)
    reproduction = "two_layer_v2_semantic_r2_benchmark " + " ".join(
        shlex.quote(value) for value in arguments
    )
    with _r2_bindings():
        if args.mode == "convert":
            return r1.main(arguments)
        if args.mode == "synthetic-smoke":
            r1.run_synthetic_smoke(
                output=args.output_dir.resolve(), config_path=args.config.resolve(),
                preference_policy_overrides=overrides,
            )
            real = False
        else:
            if args.extracted_dir is None or args.semantic_map is None or args.topology_cache is None:
                raise SystemExit(
                    f"{args.mode} requires --extracted-dir, --semantic-map and --topology-cache"
                )
            common = {
                "extracted_dir": args.extracted_dir.resolve(),
                "semantic_map_path": args.semantic_map.resolve(),
                "topology_cache": args.topology_cache.resolve(),
                "output": args.output_dir.resolve(),
                "config_path": args.config.resolve(),
                "preference_policy_overrides": overrides,
            }
            if args.mode == "offline-diagnostic":
                r1.run_offline_diagnostic(
                    **common,
                    r0_results=args.r0_results.resolve() if args.r0_results else None,
                )
                real = False
            else:
                arms = [value.strip() for value in args.arms.split(",") if value.strip()]
                unknown = sorted(set(arms) - set(r1.ARM_ORDER))
                if unknown:
                    raise SystemExit(f"unknown arms: {unknown}")
                r1.run_real_ablation(
                    **common, warmups=args.warmups, repetitions=args.repetitions,
                    ros_domain_id=args.ros_domain_id, arms=arms,
                    query_ids=[value.strip() for value in args.query_ids.split(",") if value.strip()] or None,
                )
                real = True
    _postprocess(args.output_dir.resolve(), reproduction, real=real)
    print(f"2A-V2/r2 output: {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
