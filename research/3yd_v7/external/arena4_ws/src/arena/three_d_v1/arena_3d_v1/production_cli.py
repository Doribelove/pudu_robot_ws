"""Default command-line entry point for the stable 3D-V1 production runtime."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

from arena_evaluation.dynamic_snapshot import DynamicSnapshot

from .production_io import load_plan_bundle
from .production_runtime import create_controller, production_selection
from .stable_contract import DEFAULT_STABLE_CONFIG, PRODUCTION_BASELINE_ID, stable_contract


def _result_record(controller: Any, step: Optional[Any] = None) -> Dict[str, Any]:
    result = controller.initial_l2_result if step is None else step.l2_result
    diagnostics: Mapping[str, Any] = (
        controller.initial_l2_result.diagnostics if step is None else step.diagnostics
    )
    return {
        **stable_contract(),
        "cache_status": controller.cache_status,
        "selected_backend": (
            result.selected_backend if result is not None else diagnostics.get("selected_backend")
        ),
        "fallback_reason": diagnostics.get("fallback_reason", "NONE"),
        "l2_success": None if result is None else result.success,
        "failure_code": (
            result.failure_code if step is None and result is not None
            else "" if step is None else step.failure_code
        ),
        "partial_dstar_result_returned": (
            False if result is None else result.partial_dstar_result_returned
        ),
        "online_synchronous_dstar_build": controller.lifecycle.synchronous_dstar_build_count,
        "route_signature": controller.plan.route_signature,
        "telemetry": dict(diagnostics),
    }


def _snapshot(path: Path, *, plan: Any) -> DynamicSnapshot:
    value = json.loads(path.resolve().read_text(encoding="utf-8"))
    return DynamicSnapshot.from_cells(
        str(value["snapshot_id"]),
        [tuple(int(item) for item in cell) for cell in value.get("occupied_cells", ())],
        timestamp=float(value["timestamp"]),
        map_version=str(value.get("map_version", plan.map_hash)),
        map_shape=tuple(int(item) for item in value.get("map_shape", plan.corridor_mask.shape)),
        confidence={
            str(key): float(item)
            for key, item in (value.get("obstacle_confidence") or {}).items()
        },
        ttl=None if value.get("ttl") is None else float(value["ttl"]),
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            f"Plan with {PRODUCTION_BASELINE_ID}, the only default 3D-V1 production "
            "revision. Legacy r0/r1/r2 and pure-D* requests fail closed; cache "
            "miss/reject falls back to deterministic grid A* without an online build."
        )
    )
    parser.add_argument("--plan-bundle", type=Path)
    parser.add_argument("--cache-bundle", type=Path)
    parser.add_argument("--snapshot-json", type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_STABLE_CONFIG)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--verify-l2-oracle", action="store_true")
    parser.add_argument("--print-contract", action="store_true")
    args = parser.parse_args()
    if args.print_contract:
        record = {**stable_contract(), **production_selection()}
    else:
        if args.plan_bundle is None:
            parser.error("--plan-bundle is required unless --print-contract is used")
        verified = load_plan_bundle(args.plan_bundle)
        controller = create_controller(
            verified.plan,
            cache_bundle=args.cache_bundle,
            start_pose=verified.start_pose,
            goal_pose=verified.goal_pose,
            config_path=args.config,
            verify_l2_oracle=args.verify_l2_oracle,
        )
        step = None
        if args.snapshot_json is not None:
            step = controller.process_snapshot(
                _snapshot(args.snapshot_json, plan=verified.plan)
            )
        record = _result_record(controller, step)
    rendered = json.dumps(record, indent=2, sort_keys=True, default=str) + "\n"
    if args.output_json is None:
        print(rendered, end="")
    else:
        args.output_json.resolve().parent.mkdir(parents=True, exist_ok=True)
        args.output_json.resolve().write_text(rendered, encoding="utf-8")


if __name__ == "__main__":
    main()
