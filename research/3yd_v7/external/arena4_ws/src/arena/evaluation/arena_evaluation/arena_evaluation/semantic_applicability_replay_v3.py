"""Independent-process replay verifier for a 2A-V3 applicability certificate."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any

from . import semantic_applicability_v3 as applicability
from .semantic_map import canonical_hash, sha256_file


SCHEMA_VERSION = "PLN-02-2A-V3-APPLICABILITY-REPLAY-V1"


def _provider(config: dict[str, Any]):
    certificate = config["ordered_se2_certificate"]
    if certificate.get("candidate_cell_source") != "map_cells_in_local_station_slab":
        return None
    half_width = float(certificate["station_slab_half_width_m"])
    return lambda world, sample, lane_label, policy: applicability._map_cell_candidate_cells(
        world, sample, lane_label, policy,
        station_slab_half_width_m=half_width,
    )


def _deterministic_view(certificate: dict[str, Any]) -> dict[str, Any]:
    """Remove only explicitly measured timing from the replayed certificate."""
    value = copy.deepcopy(certificate)
    value.get("graph", {}).pop("graph_build_wall_s", None)
    return value


def run_single(
    *, input_dir: Path, query_id: str, config_path: Path, output: Path,
) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=False)
    config, _ = applicability._load_config(config_path)
    certificate = applicability._chain_certificate(
        input_dir, query_id, applicability._ordered_policy(config),
        candidate_cell_provider=_provider(config),
    )
    deterministic = _deterministic_view(certificate)
    payload = {
        "schema_version": SCHEMA_VERSION,
        "architecture_id": applicability.ARCHITECTURE_ID,
        "implementation_revision": applicability.IMPLEMENTATION_REVISION,
        "protocol_id": applicability.PROTOCOL_ID,
        "query_id": query_id,
        "applicable": bool(certificate.get("applicable")),
        "certificate_hash": canonical_hash(deterministic),
        "path_sha256": certificate.get("path_sha256"),
        "controls_hash": canonical_hash(certificate.get("controls", [])),
        "edge_ids_hash": canonical_hash(certificate.get("edge_ids", [])),
        "input_npz_sha256": sha256_file(input_dir / f"{query_id}.npz"),
        "input_meta_sha256": sha256_file(input_dir / f"{query_id}.json"),
        "config_sha256": sha256_file(config_path),
    }
    applicability._write_json(output / "certificate.json", certificate)
    applicability._write_json(output / "determinism.json", payload)
    return payload


def run(
    *, input_dir: Path, query_id: str, config_path: Path, output: Path,
    repetitions: int,
) -> dict[str, Any]:
    if repetitions < 3:
        raise ValueError("at least three independent replays are required")
    output.mkdir(parents=True, exist_ok=False)
    meta_path = input_dir / f"{query_id}.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    expected = {
        "architecture_id": applicability.ARCHITECTURE_ID,
        "implementation_revision": applicability.IMPLEMENTATION_REVISION,
        "protocol_id": applicability.PROTOCOL_ID,
    }
    for key, value in expected.items():
        if meta.get(key) != value:
            raise ValueError(f"input identity mismatch for {key}")
    applicability._write_json(output / "protocol.json", {
        "schema_version": SCHEMA_VERSION,
        **expected,
        "query_id": query_id,
        "repetitions": repetitions,
        "fresh_process_per_replay": True,
        "comparison_excludes_only": ["graph.graph_build_wall_s"],
        "input_npz_sha256": sha256_file(input_dir / f"{query_id}.npz"),
        "input_meta_sha256": sha256_file(meta_path),
        "config_sha256": sha256_file(config_path),
    })
    environment = os.environ.copy()
    package_root = str(applicability.PACKAGE_ROOT)
    current = environment.get("PYTHONPATH", "")
    environment["PYTHONPATH"] = package_root + (os.pathsep + current if current else "")
    rows: list[dict[str, Any]] = []
    for index in range(repetitions):
        replay_dir = output / f"replay_{index + 1:02d}"
        command = [
            sys.executable, "-m", "arena_evaluation.semantic_applicability_replay_v3",
            "--single", "--input-dir", str(input_dir), "--query-id", query_id,
            "--config", str(config_path), "--output", str(replay_dir),
        ]
        completed = subprocess.run(
            command, check=False, capture_output=True, text=True,
            env=environment, timeout=900,
        )
        (output / f"replay_{index + 1:02d}.stdout").write_text(
            completed.stdout, encoding="utf-8",
        )
        (output / f"replay_{index + 1:02d}.stderr").write_text(
            completed.stderr, encoding="utf-8",
        )
        if completed.returncode != 0:
            raise RuntimeError(f"replay {index + 1} failed with {completed.returncode}")
        rows.append(json.loads((replay_dir / "determinism.json").read_text(encoding="utf-8")))
    fields = ("certificate_hash", "path_sha256", "controls_hash", "edge_ids_hash")
    identical = all(len({str(row[field]) for row in rows}) == 1 for field in fields)
    passed = bool(identical and all(row["applicable"] for row in rows))
    summary = {
        "schema_version": SCHEMA_VERSION,
        **expected,
        "query_id": query_id,
        "repetition_count": repetitions,
        "all_applicable": all(row["applicable"] for row in rows),
        "deterministic_hashes_identical": identical,
        "gate_passed": passed,
        "rows": rows,
    }
    applicability._write_json(output / "replay_summary.json", summary)
    applicability._write_json(output / "artifact_hashes.json", {
        str(path.relative_to(output)): sha256_file(path)
        for path in sorted(output.rglob("*")) if path.is_file()
    })
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--query-id", required=True)
    parser.add_argument("--config", type=Path, default=applicability.DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--single", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.single:
        result = run_single(
            input_dir=args.input_dir.resolve(), query_id=args.query_id,
            config_path=args.config.resolve(), output=args.output.resolve(),
        )
    else:
        result = run(
            input_dir=args.input_dir.resolve(), query_id=args.query_id,
            config_path=args.config.resolve(), output=args.output.resolve(),
            repetitions=args.repetitions,
        )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
