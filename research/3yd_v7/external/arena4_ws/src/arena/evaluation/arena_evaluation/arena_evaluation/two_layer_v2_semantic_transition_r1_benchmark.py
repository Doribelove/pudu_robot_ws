"""Disabled online entry point for the endpoint-transition semantic contract.

The available implementation is an offline replay study, not an online
planner adapter. This entry point reports that boundary without importing
the legacy benchmark, starting ROS, or deriving a promotion from saved rows.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Optional, Sequence


CONFIG = Path(__file__).resolve().parents[1] / "config/pudu_wanda_3f_semantic_endpoint_transition_r1.yaml"
ARCHITECTURE_ID = "2A-V2"
IMPLEMENTATION_REVISION = "r1-semantic-endpoint-transition-6m-disabled-online"
PROTOCOL_ID = "PLN-02-SEMANTIC-ENDPOINT-TRANSITION-R1-V1"
CONTRACT_REVISION = "semantic-endpoint-transition-6m-r1"
TARGETED_QUERY_IDS = (
    "r3-mirror-1-positive",
    "r3-mirror-2-negative",
    "cmp2-02-lane-south",
)
MISSING_GATES = (
    "verified_targeted_offline_3_of_3",
    "implemented_online_planner_adapter",
    "targeted_online_3_of_3",
    "same_round_48_bin_three_arm_comparison",
    "selected8_all_queries_all_measured_repeats",
    "per_repeat_e0_success_retention",
    "exact_effective_content_ack_nonempty_and_complete",
    "full_path_safety_endpoint_kinematic_r0_invariants",
    "cold_latency_hard_limit",
)


def disabled_status(offline_gate: Optional[Path] = None) -> dict[str, Any]:
    """Return a refusal; an arbitrary external JSON cannot enable execution."""
    report: dict[str, Any] = {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
        "contract_revision": CONTRACT_REVISION,
        "status": "NOT_RUN_OFFLINE_GATE_FAILED",
        "online_adapter_status": "NOT_IMPLEMENTED_ONLINE_ADAPTER",
        "online_started": False,
        "promotion_gate_passed": False,
        "targeted_query_ids": list(TARGETED_QUERY_IDS),
        "missing_gates": list(MISSING_GATES),
        "offline_gate_file": str(offline_gate) if offline_gate else None,
        "offline_gate_status": "MISSING",
        "detail": (
            "No verified three-query offline gate is wired to this entry point. "
            "The saved-reference replay is not an online planner adapter. "
            "The 6 m contract has no applicable interval for the 10.55 m "
            "negative witness. Online execution remains disabled."
        ),
    }
    if offline_gate is not None:
        try:
            payload = json.loads(offline_gate.read_text(encoding="utf-8"))
        except (OSError, ValueError) as error:
            report["offline_gate_status"] = "UNREADABLE_OR_INVALID"
            report["offline_gate_error"] = str(error)
        else:
            report["offline_gate_status"] = "UNVERIFIED_EXTERNAL_REPORT"
            report["offline_gate_payload_type"] = type(payload).__name__
            report["detail"] += (
                " External report contents are diagnostic only; acceptance needs "
                "bound query, map, footprint, contract and path evidence."
            )
    return report


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Report the disabled PLN-02 endpoint-transition online benchmark. "
            "This command never starts ROS or computes a promotion result."
        ),
    )
    parser.add_argument("--mode", choices=("real-ablation",), default="real-ablation")
    parser.add_argument("--offline-gate", type=Path, help="optional diagnostic report; cannot enable execution")
    parser.add_argument("--config", type=Path, default=CONFIG, help="contract metadata, not a planner configuration")
    parser.add_argument("--output-dir", type=Path, help="reserved; no output directory is created while disabled")
    # Retain familiar invocation flags without delegating them to a ROS runner.
    parser.add_argument("--extracted-dir", type=Path)
    parser.add_argument("--semantic-map", type=Path)
    parser.add_argument("--topology-cache", type=Path)
    parser.add_argument("--query-set", type=Path)
    parser.add_argument("--query-ids", default="")
    parser.add_argument("--arms", default="E0,E4")
    parser.add_argument("--warmups", type=int, default=1)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--ros-domain-id", type=int)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(list(sys.argv[1:] if argv is None else argv))
    print(json.dumps(disabled_status(args.offline_gate), indent=2, sort_keys=True))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
