"""Offline analysis of endpoint semantic transition zones.

This tool never changes the frozen path, map, footprint, or old-contract
metrics. It reports hypothetical semantic applicability masks over an already
audited, replayable path so a revised contract can be reviewed separately.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import sys
import time
from pathlib import Path

import numpy as np

from .semantic_constraint_core import ConstraintWorld
from .semantic_map import sha256_file


def _write_json(path: Path, payload) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def _path_array(path_file: Path) -> np.ndarray:
    raw = json.loads(path_file.read_text())
    return np.asarray([[row["x"], row["y"], row["yaw"]] for row in raw], dtype=float)


def _hash_array(values: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(values).tobytes()).hexdigest()


def _metrics(world: ConstraintWorld, path: np.ndarray, prefix: float, suffix: float):
    rows, cols, inside = world.cells(path)
    if not bool(np.all(inside)):
        raise ValueError("path contains an out-of-map sample")
    lane = np.isin(world.grids["labels"][rows, cols], world.selected)
    finite = np.isfinite(world.grids["right"][rows, cols])
    if not bool(np.all(lane & finite)):
        raise ValueError("path sample is outside selected lane semantics")
    correct = world.grids["correct"][rows, cols].astype(bool)
    error = world.grids["error"][rows, cols].astype(float)
    target = correct & (error <= 0.50)
    station = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]
    total = float(station[-1])
    applicable = (station >= float(prefix) - 1e-9) & (station <= total - float(suffix) + 1e-9)
    used = applicable & lane & finite
    count = int(np.count_nonzero(used))
    if count == 0:
        return {
            "prefix_transition_m": float(prefix), "suffix_transition_m": float(suffix),
            "applicable_sample_count": 0, "applicable_arc_length_m": 0.0,
            "correct_side_ratio": None, "target_band_ratio": None,
            "lateral_error_p50_m": None, "gate_passed": False,
            "all_path_sample_count": int(len(path)), "path_length_m": total,
        }, station, used, correct, target, error
    side = float(np.mean(correct[used]))
    band = float(np.mean(target[used]))
    p50 = float(np.median(error[used]))
    arc = float(station[np.where(used)[0][-1]] - station[np.where(used)[0][0]])
    passed = bool(side >= 0.80 and band > 0.50 and p50 <= 0.50)
    return {
        "prefix_transition_m": float(prefix), "suffix_transition_m": float(suffix),
        "applicable_sample_count": count, "applicable_arc_length_m": arc,
        "correct_side_ratio": side, "target_band_ratio": band,
        "lateral_error_p50_m": p50, "gate_passed": passed,
        "all_path_sample_count": int(len(path)), "path_length_m": total,
    }, station, used, correct, target, error


def _plot(path: np.ndarray, station: np.ndarray, chosen, output: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        _write_json(output / "plot_unavailable.json", {"error": str(exc)})
        return
    fig, ax = plt.subplots(figsize=(8, 8), dpi=160)
    ax.plot(path[:, 0], path[:, 1], color="#9aa0a6", linewidth=1.2, label="full audited path")
    _, prefix, suffix, result, station, used = chosen
    ax.scatter(path[used, 0], path[used, 1], s=2.0, c="#2ca02c", label="applicable semantic interval")
    ax.scatter(path[~used, 0], path[~used, 1], s=2.0, c="#d62728", label="endpoint transition zone")
    ax.scatter([path[0, 0], path[-1, 0]], [path[0, 1], path[-1, 1]], c="#111111", s=18, zorder=4)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(f"Endpoint transition candidate: {prefix:.2f} m / {suffix:.2f} m")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.legend(loc="best", fontsize=7)
    fig.tight_layout(); fig.savefig(output / "endpoint_transition_overlay.png"); plt.close(fig)


def run(args) -> int:
    output = Path(args.output)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"write-once output is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    world = ConstraintWorld(args.inputs, args.query)
    path_file = Path(args.path)
    path = _path_array(path_file)
    all_result, station, used_all, correct, target, error = _metrics(world, path, 0.0, 0.0)
    # Search a 0.25 m grid for the smallest total endpoint transition budget.
    grid = [round(0.25 * i, 2) for i in range(0, int(args.max_transition / 0.25) + 1)]
    rows = []
    candidates = []
    for prefix in grid:
        for suffix in grid:
            result, st, used, *_ = _metrics(world, path, prefix, suffix)
            result["total_transition_budget_m"] = float(prefix + suffix)
            result["max_transition_side_m"] = float(max(prefix, suffix))
            rows.append(result)
            candidates.append(("grid", prefix, suffix, result, st, used))
    passing = [c for c in candidates if c[3]["gate_passed"]]
    passing.sort(key=lambda c: (c[3]["total_transition_budget_m"], c[3]["max_transition_side_m"], c[1], c[2]))
    symmetric = []
    for length in grid:
        result, st, used, *_ = _metrics(world, path, length, length)
        result["total_transition_budget_m"] = float(2 * length)
        symmetric.append(result)
    chosen = passing[0] if passing else None
    manifest = {
        "protocol_id": "PLN-02-ENDPOINT-TRANSITION-ANALYSIS-R0-V1",
        "scope": "hypothetical semantic applicability mask; frozen path/map/footprint and old metrics retained",
        "architecture_id": "UNNAMED_CONTRACT_CANDIDATE",
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "python": sys.version, "platform": platform.platform(),
        "query_id": args.query, "input_dir": str(Path(args.inputs).resolve()),
        "path_file": str(path_file.resolve()), "path_sha256": sha256_file(path_file),
        "input_json_sha256": sha256_file(Path(args.inputs) / f"{args.query}.json"),
        "input_npz_sha256": sha256_file(Path(args.inputs) / f"{args.query}.npz"),
        "expected_master_hash": world.meta["expected_master_hash"],
        "semantic_map_hash": world.meta["semantic_map_hash"],
        "start": world.start, "goal": world.goal,
        "old_contract_metrics": all_result,
        "grid_step_m": 0.25, "max_transition_m_per_side": args.max_transition,
        "selection_rule": "minimum prefix+suffix budget, then minimum max side, then prefix, suffix",
        "candidate_count": len(rows),
        "chosen_candidate": chosen[3] if chosen else None,
        "old_contract_retained": True,
    }
    _write_json(output / "manifest.json", manifest)
    _write_json(output / "old_contract_full_path.json", {**all_result, "path_sha256": sha256_file(path_file)})
    _write_json(output / "candidate_grid.json", rows)
    _write_json(output / "symmetric_scan.json", symmetric)
    _write_json(output / "chosen_candidate.json", chosen[3] if chosen else {"gate_passed": False})
    with (output / "candidate_grid.csv").open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=sorted(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    # Save the applicability mask and station coordinates for deterministic replay.
    np.savez_compressed(output / "path_semantic_masks.npz", path=path, station=station,
                        full_lane=used_all, correct=correct, target=target, error=error)
    if chosen:
        result, st, used, *_ = _metrics(world, path, chosen[1], chosen[2])
        _write_json(output / "chosen_replay.json", {
            "prefix_transition_m": chosen[1], "suffix_transition_m": chosen[2],
            "path_sha256": sha256_file(path_file), "pose_count": len(path),
            "applicable_indices": np.where(used)[0].tolist(), "metrics": result,
        })
        _plot(path, st, chosen, output)
    _write_json(output / "artifact_hashes.json", {
        p.name: sha256_file(p) for p in output.iterdir() if p.is_file() and p.name != "artifact_hashes.json"
    })
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--inputs", required=True)
    parser.add_argument("--query", default="r3-mirror-1-positive")
    parser.add_argument("--path", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--max-transition", type=float, default=14.0)
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
