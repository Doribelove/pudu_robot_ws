"""Console entry point for 3D-V0 smoke and deterministic D* Lite demo."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .dstar_lite import DStarLite
from .dynamic_snapshot import DynamicSnapshot


def _demo() -> dict:
    free = np.ones((24, 32), dtype=bool)
    free[11, 4:28] = False
    planner = DStarLite(free, (20, 2), (2, 29))
    initial = planner.compute_shortest_path()
    initial_path = planner.extract_path()
    snapshot = DynamicSnapshot.from_cells("demo-1", [(15, 14), (14, 14), (13, 14)], map_shape=free.shape)
    # This update is outside the initial path and should not invalidate it;
    # it still exercises the persistent state and changed-cell accounting.
    from .dynamic_snapshot import apply_dynamic_snapshot
    _, _, changed = apply_dynamic_snapshot(free, snapshot)
    planner.update_cells(changed, traversable=free)
    update = planner.compute_shortest_path()
    return {
        "architecture_id": "3D-V0",
        "implementation_revision": "r1",
        "initial": {"success": initial_path is not None, "expanded_nodes": initial.expanded_nodes, "path_length": len(initial_path or [])},
        "dynamic_update": {"snapshot_id": snapshot.snapshot_id, "changed_cells": len(changed), "expanded_nodes": update.expanded_nodes, "path_length": len(planner.extract_path() or [])},
        "l1_called": False,
        "l2_called": True,
        "l3_backend": "SmacHybridAdapter (not started in demo)",
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run or inspect the independent 3D-V0 dynamic layered planner."
    )
    parser.add_argument("--architecture", choices=("3d_v0", "3a_v0"), default="3d_v0")
    parser.add_argument("--map-yaml", type=Path, help="Static map YAML for a future ROS-backed run")
    parser.add_argument("--query-json", type=Path, help="Query JSON for a future ROS-backed run")
    parser.add_argument("--snapshot-json", type=Path, help="Dynamic snapshot JSON")
    parser.add_argument("--output", type=Path, help="Write result JSON to this path")
    parser.add_argument("--demo", action="store_true", help="Run a small offline D* Lite demonstration")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.architecture == "3a_v0":
        result = {
            "architecture_id": "3A-V0",
            "status": "delegated_static_baseline",
            "entrypoint": "fixed_layered_pipeline_smoke",
        }
    else:
        result = _demo() if args.demo else {
            "architecture_id": "3D-V0", "implementation_revision": "r1",
            "status": "implemented", "message": "Use --demo for an offline check or provide a ROS-backed runner.",
        }
    payload = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
