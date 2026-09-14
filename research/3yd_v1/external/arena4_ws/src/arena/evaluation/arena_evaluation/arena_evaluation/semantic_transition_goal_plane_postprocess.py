"""Add a directed goal-plane audit to a frozen any-yaw topology certificate.

The expensive footprint projection is not recomputed.  This command verifies
the existing NPZ and topology-result hashes, then classifies every reachable
target subcell relative to a stable, query-oriented terminal route tangent.
The result remains a sampled necessary-condition diagnostic, not a continuous
configuration-space infeasibility proof.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any

import numpy as np

from .semantic_constraint_core import ConstraintWorld
from .semantic_map import sha256_file
from .semantic_transition_goal_plane_topology import (
    _oriented_terminal_tangent,
    _subcell_progress,
)
from .semantic_transition_preflight import write_json


PROTOCOL_ID = "PLN-02-SEMANTIC-GOAL-PLANE-POSTPROCESS-R0-V1"


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def classify_reachable_target_progress(mask, progress, *, tolerance_m: float) -> dict[str, Any]:
    mask = np.asarray(mask, dtype=bool)
    progress = np.asarray(progress, dtype=float)
    if mask.shape != progress.shape or mask.ndim != 2:
        raise ValueError("reachable target and progress must be identical 2D shapes")
    if not math.isfinite(float(tolerance_m)) or tolerance_m < 0.0:
        raise ValueError("tolerance must be finite and non-negative")
    values = progress[mask]
    before = int(np.count_nonzero(values < -tolerance_m))
    on = int(np.count_nonzero(np.abs(values) <= tolerance_m))
    after = int(np.count_nonzero(values > tolerance_m))
    total = int(len(values))
    if before + on + after != total:
        raise ValueError("non-finite progress in reachable target mask")
    return {
        "reachable_target_count": total,
        "reachable_target_before_goal_count": before,
        "reachable_target_on_goal_plane_count": on,
        "reachable_target_after_goal_count": after,
        "reachable_target_after_goal_ratio": after / total if total else None,
        "reachable_target_progress_min_m": float(values.min()) if total else None,
        "reachable_target_progress_max_m": float(values.max()) if total else None,
        "numeric_tolerance_m": float(tolerance_m),
        "sampled_projection_pre_goal_target_present": before + on > 0,
        "status": (
            "NO_TARGET_IN_FROZEN_SAMPLED_PROJECTION" if total == 0 else
            "NO_PREGOAL_TARGET_IN_FROZEN_SAMPLED_PROJECTION" if before + on == 0 else
            "PREGOAL_TARGET_PRESENT_IN_FROZEN_SAMPLED_PROJECTION"
        ),
    }


def partition_target_space(target, any_yaw_free, components, progress, *,
                           start_component: int, tolerance_m: float) -> dict[str, Any]:
    """Separate raw target geometry from sampled footprint/connectivity limits."""
    target = np.asarray(target, dtype=bool)
    free = np.asarray(any_yaw_free, dtype=bool)
    components = np.asarray(components)
    progress = np.asarray(progress, dtype=float)
    if not (target.shape == free.shape == components.shape == progress.shape) or target.ndim != 2:
        raise ValueError("target partition arrays must have identical 2D shapes")

    def counts(mask):
        values = progress[np.asarray(mask, dtype=bool)]
        return {
            "count": int(len(values)),
            "before_goal_count": int(np.count_nonzero(values < -tolerance_m)),
            "on_goal_plane_count": int(np.count_nonzero(np.abs(values) <= tolerance_m)),
            "after_goal_count": int(np.count_nonzero(values > tolerance_m)),
        }

    categories = {
        "sampled_footprint_infeasible": target & ~free,
        "sampled_free_start_component": target & free & (components == int(start_component)),
        "sampled_free_other_component": target & free & (components != int(start_component)),
    }
    result = {name: counts(mask) for name, mask in categories.items()}
    partition_count = sum(value["count"] for value in result.values())
    raw_count = int(np.count_nonzero(target))
    if partition_count != raw_count:
        raise ValueError("target partition is not exhaustive")
    result["raw_target"] = counts(target)
    result["partition_count_matches_raw_target"] = True
    return result


def run(
    *,
    inputs: Path,
    query: str,
    certificate: Path,
    topology_result: Path,
) -> dict[str, Any]:
    inputs, certificate, topology_result = inputs.resolve(), certificate.resolve(), topology_result.resolve()
    artifact_hashes_file = certificate.parent / "artifact_hashes.json"
    artifact_hashes = _json(artifact_hashes_file)
    expected_certificate_hash = artifact_hashes.get(certificate.name)
    expected_topology_hash = artifact_hashes.get(topology_result.name)
    if expected_certificate_hash != sha256_file(certificate):
        raise ValueError("topology certificate hash mismatch")
    if expected_topology_hash != sha256_file(topology_result):
        raise ValueError("topology result hash mismatch")
    frozen = _json(topology_result)
    resolution = float(frozen.get("position_resolution_m", math.nan))
    scale = int(frozen.get("position_scale_from_frozen_grid", 0))
    yaw_samples = int(frozen.get("yaw_samples", 0))
    if not math.isclose(resolution, 0.01, abs_tol=1.0e-12) or scale != 5 or yaw_samples != 360:
        raise ValueError("expected frozen 1 cm / 360-yaw topology certificate")
    if frozen.get("start_goal_connected_in_optimistic_any_yaw_projection") is not True:
        raise ValueError("frozen certificate does not bind start and goal to one optimistic component")

    with np.load(certificate, allow_pickle=False) as archive:
        if set(archive.files) != {"any_yaw_free", "components", "reachable_target"}:
            raise ValueError("unexpected topology certificate arrays")
        any_yaw_free = np.asarray(archive["any_yaw_free"], dtype=bool)
        components = np.asarray(archive["components"])
        reachable_target = np.asarray(archive["reachable_target"], dtype=bool)
    if int(np.count_nonzero(reachable_target)) != int(frozen.get("reachable_target_subcell_count", -1)):
        raise ValueError("reachable target count mismatch")

    world = ConstraintWorld(inputs, query)
    route, tangent, route_diagnostics = _oriented_terminal_tangent(
        world.meta.get("route_polyline", []), world.start, world.goal
    )
    progress = _subcell_progress(world, reachable_target.shape, scale, world.goal, tangent)
    raw_target = (
        np.asarray(world.grids["correct"], dtype=bool)
        & (np.asarray(world.grids["error"], dtype=float) <= 0.50)
        & np.isin(world.grids["labels"], world.selected)
    )
    raw_target = np.repeat(np.repeat(raw_target, scale, axis=0), scale, axis=1)
    if int(np.count_nonzero(raw_target)) != int(frozen.get("total_target_subcell_count", -1)):
        raise ValueError("frozen raw target count mismatch")
    start_component = int(frozen.get("start_component", 0))
    reconstructed_reachable = raw_target & (components == start_component)
    if not np.array_equal(reconstructed_reachable, reachable_target):
        raise ValueError("frozen reachable target mask mismatch")
    classification = classify_reachable_target_progress(
        reachable_target, progress, tolerance_m=resolution
    )
    target_partition = partition_target_space(
        raw_target,
        any_yaw_free,
        components,
        progress,
        start_component=start_component,
        tolerance_m=resolution,
    )
    return {
        "protocol_id": PROTOCOL_ID,
        "scope": "verified_postprocess_of_frozen_optimistic_sampled_projection",
        "query_id": query,
        "result_code": classification["status"],
        "sampled_projection_pre_goal_target_present": classification[
            "sampled_projection_pre_goal_target_present"
        ],
        "classification": classification,
        "target_space_partition": target_partition,
        "terminal_tangent": tangent.tolist(),
        "route_diagnostics": route_diagnostics,
        "position_resolution_m": resolution,
        "yaw_samples": yaw_samples,
        "maximum_position_cover_radius_m": frozen.get("maximum_position_cover_radius_m"),
        "maximum_yaw_cover_error_deg": frozen.get("maximum_yaw_cover_error_deg"),
        "source_projection_recomputed": False,
        "continuous_space_infeasibility_proof": False,
        "proof_scope": (
            "1 cm centers and 360 sampled yaws; any-yaw 2D union is optimistic about yaw "
            "continuity; absence before the goal plane is not a proof over every continuous pose"
        ),
        "bindings": {
            "input_json_sha256": sha256_file(inputs / f"{query}.json"),
            "input_npz_sha256": sha256_file(inputs / f"{query}.npz"),
            "map_hash": world.meta.get("map_hash"),
            "semantic_map_hash": world.meta.get("semantic_map_hash"),
            "route_hash": world.meta.get("route_hash"),
            "topology_certificate_sha256": sha256_file(certificate),
            "topology_result_sha256": sha256_file(topology_result),
            "artifact_hashes_sha256": sha256_file(artifact_hashes_file),
            "reachable_target_sha256": hashlib.sha256(
                np.ascontiguousarray(reachable_target).tobytes()
            ).hexdigest(),
            "any_yaw_free_sha256": hashlib.sha256(
                np.ascontiguousarray(any_yaw_free).tobytes()
            ).hexdigest(),
            "components_sha256": hashlib.sha256(
                np.ascontiguousarray(components).tobytes()
            ).hexdigest(),
            "raw_target_sha256": hashlib.sha256(
                np.ascontiguousarray(raw_target).tobytes()
            ).hexdigest(),
        },
    }


def write_artifacts(output: Path, result: dict[str, Any], command: str) -> None:
    output.mkdir(parents=False, exist_ok=False)
    result_file = output / "goal_plane_postprocess.json"
    write_json(result_file, result)
    (output / "reproduction_command.txt").write_text(command.rstrip() + "\n", encoding="utf-8")
    write_json(output / "manifest.json", {
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "protocol_id": PROTOCOL_ID,
        "source_sha256": sha256_file(Path(__file__)),
        "goal_plane_postprocess_sha256": sha256_file(result_file),
        "reproduction_command_sha256": sha256_file(output / "reproduction_command.txt"),
        "source_projection_recomputed": False,
        "bindings": result["bindings"],
        "write_once": True,
    })


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--query", default="r3-mirror-1-positive")
    parser.add_argument("--certificate", type=Path, required=True)
    parser.add_argument("--topology-result", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result = run(
        inputs=args.inputs,
        query=args.query,
        certificate=args.certificate,
        topology_result=args.topology_result,
    )
    command = " ".join((
        "/usr/bin/python3 -m arena_evaluation.semantic_transition_goal_plane_postprocess",
        f"--inputs {args.inputs.resolve()}",
        f"--query {args.query}",
        f"--certificate {args.certificate.resolve()}",
        f"--topology-result {args.topology_result.resolve()}",
        f"--output {args.output.resolve()}",
    ))
    write_artifacts(args.output.resolve(), result, command)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["sampled_projection_pre_goal_target_present"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
