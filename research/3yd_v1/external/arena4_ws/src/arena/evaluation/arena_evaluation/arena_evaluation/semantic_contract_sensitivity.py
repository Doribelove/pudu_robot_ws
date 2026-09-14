"""Measure a proposed endpoint-transition semantic contract on a frozen path.

This is a diagnostic protocol.  It never changes the frozen PLN-02 gate and
does not claim a new architecture or production acceptance.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from .semantic_constraint_core import ConstraintWorld
from .semantic_path_audit import SemanticPathAuditor
from .semantic_map import sha256_file


PROTOCOL_ID = "PLN-02-SEMANTIC-CONTRACT-SENSITIVITY-R0-V1"


def _jsonable(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    return value


def run(inputs: Path, query: str, path_file: Path, source_result: Path, output: Path, trims: list[float]) -> dict:
    output.mkdir(parents=False, exist_ok=False)
    world = ConstraintWorld(inputs, query)
    raw = json.loads(path_file.read_text())
    points = [
        {"x": float(item["x"]), "y": float(item["y"]), "yaw": float(item["yaw"]),
         "source": "kinematic", "motion_direction": "forward"}
        for item in raw
    ]
    samples = np.asarray(SemanticPathAuditor._samples(SimpleNamespace(hospital_map=world.map), points))
    rows, cols, inside = world.cells(samples)
    lane = inside & np.isin(world.grids["labels"][rows, cols], world.selected)
    if not np.all(lane):
        raise RuntimeError("frozen path contains samples outside the selected lane")
    correct = world.grids["correct"][rows, cols].astype(bool)
    errors = world.grids["error"][rows, cols].astype(float)
    target = correct & (errors <= 0.50)
    cumulative = np.zeros(len(samples), dtype=float)
    cumulative[1:] = np.cumsum(np.hypot(np.diff(samples[:, 0]), np.diff(samples[:, 1])))
    path_length = float(cumulative[-1])

    rows_out = []
    for trim in trims:
        trim = float(trim)
        active = (cumulative >= trim) & (cumulative <= path_length - trim)
        n = int(np.count_nonzero(active))
        c = int(np.count_nonzero(correct & active))
        t = int(np.count_nonzero(target & active))
        values = errors[active]
        rows_out.append({
            "endpoint_transition_each_m": trim,
            "evaluated_sample_count": n,
            "correct_sample_count": c,
            "target_sample_count": t,
            "correct_side_ratio": (c / n) if n else None,
            "target_band_ratio": (t / n) if n else None,
            "lateral_error_p50_m": float(np.median(values)) if n else None,
            "semantic_gate_candidate": bool(
                n and c / n >= 0.80 and t / n > 0.50 and np.median(values) <= 0.50
            ),
        })

    source_payload = json.loads(source_result.read_text())
    frozen_audit = source_payload.get("frozen_audit")
    if frozen_audit is None:
        frozen_audit = source_payload.get("audit", source_payload)
    manifest = {
        "protocol_id": PROTOCOL_ID,
        "architecture_id": "UNNAMED_CONTRACT_CANDIDATE",
        "implementation_revision": "r0",
        "scope": "diagnostic_only; frozen_contract_unchanged",
        "query": query,
        "inputs": str(inputs),
        "path_file": str(path_file),
        "path_sha256": sha256_file(path_file),
        "source_result": str(source_result),
        "source_result_sha256": sha256_file(source_result),
        "input_npz_sha256": world.meta["npz_sha256"],
        "map_hash": world.meta["map_hash"],
        "semantic_map_hash": world.meta["semantic_map_hash"],
        "frozen_safety_gate": frozen_audit,
        "metric_definition": "exclude equal-length endpoint transition windows; no changes to path or safety audit",
        "trims_m": trims,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True, default=_jsonable) + "\n")
    (output / "results.json").write_text(json.dumps(rows_out, indent=2, sort_keys=True) + "\n")
    with (output / "results.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows_out[0]))
        writer.writeheader()
        writer.writerows(rows_out)
    summary = {
        "protocol_id": PROTOCOL_ID,
        "query": query,
        "frozen_path_length_m": path_length,
        "frozen_path_safety_valid": bool(frozen_audit.get("canonical", {}).get("final_valid_success")),
        "candidate_passes": [row for row in rows_out if row["semantic_gate_candidate"]],
        "minimum_candidate_trim_m": next((row["endpoint_transition_each_m"] for row in rows_out if row["semantic_gate_candidate"]), None),
        "rows": len(rows_out),
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--query", default="r3-mirror-1-positive")
    parser.add_argument("--path", type=Path, required=True)
    parser.add_argument("--source-result", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trims", type=float, nargs="+", default=[0, 1, 2, 3, 4, 5, 6, 7, 8])
    args = parser.parse_args(argv)
    print(json.dumps(run(args.inputs, args.query, args.path, args.source_result, args.output, args.trims), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
