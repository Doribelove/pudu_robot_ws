"""Offline applicability and parking-reference gate for 2A-V3 r14."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import platform
import resource
import shutil
import time
from typing import Any, Mapping, Sequence

import yaml

from . import semantic_applicability_v3 as applicability
from . import two_layer_v3_semantic_r13_benchmark as r13
from .semantic_map import SemanticMapV1, sha256_file
from .semantic_parking_reference_v3 import (
    ContinuousParkingReferenceBuilder, ParkingReferencePolicy,
)
from .semantic_route_phase_v3 import LazyRoutePhaseSearch, OrientedRoute, RoutePhaseWorld


ARCHITECTURE_ID = "2A-V3"
IMPLEMENTATION_REVISION = "r14-parking-dispatch-ack-cache"
PROTOCOL_ID = "PLN-02-2A-V3-R14-PARKING-DISPATCH-ACK-CACHE-V1"
SCHEMA_VERSION = "PLN-02-2A-V3-R14-OFFLINE-RESULT-V1"
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE_ROOT/"config/two_layer_v3_semantic_r14_engineering.yaml"


def _identity() -> dict[str, str]:
    return {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
    }


def _load_config(path: Path = DEFAULT_CONFIG) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], Path]:
    path = Path(path).resolve()
    config = yaml.safe_load(path.read_text()) or {}
    for key, value in _identity().items():
        if config.get(key) != value:
            raise ValueError(f"r14 identity mismatch for {key}")
    binding = config["parent_algorithm"]
    parent_path = Path(str(binding["path"]))
    if not parent_path.is_absolute():
        parent_path = path.parent/parent_path
    parent_path = parent_path.resolve()
    if sha256_file(parent_path) != str(binding["sha256"]):
        raise ValueError("r14 parent algorithm hash mismatch")
    parent_algorithm, parent_v2, selected8 = r13._load_config(parent_path)
    frozen = config["frozen_bindings"]
    for key in ("map_hash", "semantic_map_hash", "query_hash"):
        expected = (
            parent_algorithm["query_set"].get(key)
            if key == "query_hash"
            else parent_algorithm["frozen_bindings"].get(key)
        )
        if frozen.get(key) != expected:
            raise ValueError(f"r14 changed frozen parent binding {key}")
    required = {
        "resolution_m": .05, "yaw_bins": 48, "motion_model": "DUBIN",
        "allow_reverse": False, "allow_in_place_rotation": False,
        "minimum_turning_radius_m": .40, "maximum_curvature_1pm": 2.50,
        "footprint_half_length_m": .265, "footprint_half_width_m": .225,
    }
    for key, value in required.items():
        if frozen.get(key) != value:
            raise ValueError(f"r14 immutable contract mismatch for {key}")
    return config, parent_algorithm, parent_v2, selected8


def _parking_policy(config: Mapping[str, Any]) -> ParkingReferencePolicy:
    return ParkingReferencePolicy(**dict(config["parking_reference"]))


def classify_and_plan(
    *, world: RoutePhaseWorld, original_route: OrientedRoute,
    semantic_map: SemanticMapV1, config: Mapping[str, Any],
    parent_algorithm: Mapping[str, Any], run_search: bool = True,
) -> dict[str, Any]:
    """Return a fail-closed E5/E0 decision and optional fresh E5 witness."""
    parking_reference = ContinuousParkingReferenceBuilder(
        _parking_policy(config)
    ).build(world, original_route)
    if not parking_reference.gate_passed:
        return {
            "dispatch": "FAIL_CLOSED", "failure_code": parking_reference.failure_code,
            "parking_reference": parking_reference.to_dict(), "witness": None,
            "search": {}, "candidate_evaluations": [],
        }
    minimum_ratio = float(
        config["semantic_applicability"]["parking_minimum_reference_target_ratio"]
    )
    if world.selected_parking and parking_reference.target_sample_ratio <= minimum_ratio:
        return {
            "dispatch": "E0_NATIVE_SMAC",
            "failure_code": "PARKING_TARGET_CONTINUOUS_COVERAGE_NOT_APPLICABLE",
            "semantic_success_counted": False,
            "parking_reference": parking_reference.to_dict(), "witness": None,
            "search": {}, "candidate_evaluations": [],
            "applicability": {
                "status": "NOT_APPLICABLE",
                "basis": "continuous_parking_reference_target_ratio",
                "observed_ratio": parking_reference.target_sample_ratio,
                "required_ratio_exclusive": minimum_ratio,
                "fallback": "E0_NATIVE_SMAC",
            },
        }
    if not run_search:
        return {
            "dispatch": "E5_R14_EXPLICIT_SE2", "failure_code": "",
            "semantic_success_counted": None,
            "parking_reference": parking_reference.to_dict(), "witness": None,
            "search": {}, "candidate_evaluations": [],
            "applicability": {"status": "CANDIDATE_APPLICABLE"},
        }
    route = OrientedRoute(
        parking_reference.polyline, world.start, world.goal,
        endpoint_attachment_limit_m=r13._policy(parent_algorithm).endpoint_attachment_limit_m,
    )
    searcher = LazyRoutePhaseSearch(world, route, r13._policy(parent_algorithm))
    witness, evaluations, search = searcher.search(
        semantic_map, allow_safe_soft_fallback=False, prefer_lazy=True,
        fallback_first=False,
    )
    if witness is not None and witness.get("semantic_success_counted") is True:
        witness["parking_reference"] = parking_reference.to_dict()
        witness["parking_reference_se2_certified"] = bool(
            witness.get("trace_replay_exact")
            and witness.get("maximum_control_curvature_1pm", 999.0) <= 2.50
            and witness.get("padded_effective_master_collision_free")
        )
        if not witness["parking_reference_se2_certified"]:
            return {
                "dispatch": "FAIL_CLOSED", "failure_code": "REFERENCE_SE2_CERTIFICATE_FAILED",
                "parking_reference": parking_reference.to_dict(), "witness": None,
                "search": search, "candidate_evaluations": evaluations,
            }
        return {
            "dispatch": "E5_R14_EXPLICIT_SE2", "failure_code": "",
            "semantic_success_counted": True,
            "parking_reference": parking_reference.to_dict(), "witness": witness,
            "search": search, "candidate_evaluations": evaluations,
            "applicability": {"status": "APPLICABLE_AND_STRICT_WITNESS"},
        }
    certificate = search.get("query_applicability") or searcher.reachability_certificate()
    applicable = list(certificate.get("applicable_semantic_classes") or [])
    if not applicable:
        return {
            "dispatch": "E0_NATIVE_SMAC", "failure_code": "NO_REACHABLE_SEMANTIC_TARGET",
            "semantic_success_counted": False,
            "parking_reference": parking_reference.to_dict(), "witness": None,
            "search": search, "candidate_evaluations": evaluations,
            "applicability": certificate,
        }
    return {
        "dispatch": "FAIL_CLOSED", "failure_code": "APPLICABLE_E5_STRICT_SEARCH_FAILED",
        "semantic_success_counted": False,
        "parking_reference": parking_reference.to_dict(), "witness": None,
        "search": search, "candidate_evaluations": evaluations,
        "applicability": certificate,
    }


def diagnose(
    *, inputs: Path, output: Path, query_ids: Sequence[str],
    config_path: Path = DEFAULT_CONFIG,
    semantic_map_path: Path = applicability.DEFAULT_SEMANTIC_MAP,
    run_search: bool = False,
) -> dict[str, Any]:
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    config, parent_algorithm, _parent_v2, _selected8 = _load_config(config_path)
    semantic_map = SemanticMapV1.load(Path(semantic_map_path).resolve())
    sources = [Path(__file__).resolve(), Path(config_path).resolve(),
               Path(__file__).with_name("semantic_parking_reference_v3.py")]
    hashes = {str(path): sha256_file(path) for path in sources}
    rows = []
    for query_id in query_ids:
        query_started = time.monotonic()
        world = RoutePhaseWorld(Path(inputs)/query_id, query_id)
        route = OrientedRoute(
            world.meta["route_polyline"], world.start, world.goal,
            endpoint_attachment_limit_m=r13._policy(parent_algorithm).endpoint_attachment_limit_m,
        )
        result = classify_and_plan(
            world=world, original_route=route, semantic_map=semantic_map,
            config=config, parent_algorithm=parent_algorithm, run_search=run_search,
        )
        witness = result.pop("witness", None)
        if witness is not None:
            points = witness.pop("points")
            controls = witness.pop("controls")
            applicability._write_json(output/f"{query_id}_path.json", points)
            applicability._write_json(output/f"{query_id}_controls.json", {"edges": controls})
            applicability._write_json(output/f"{query_id}_witness.json", witness)
        row = {
            "query_id": query_id, "dispatch": result["dispatch"],
            "failure_code": result["failure_code"],
            "parking_reference_target_ratio": result["parking_reference"]["target_sample_ratio"],
            "parking_reference_length_m": result["parking_reference"]["reference_length_m"],
            "parking_reference_connected": result["parking_reference"]["gate_passed"],
            "semantic_success_counted": result.get("semantic_success_counted"),
            "wall_ms": (time.monotonic()-query_started)*1000.0,
        }
        rows.append(row)
        applicability._write_json(output/f"{query_id}_decision.json", {**result, **row})
    with (output/"results.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    final = {
        **_identity(), "schema_version": SCHEMA_VERSION, "rows": rows,
        "run_search": bool(run_search), "wall_s": time.monotonic()-started,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
        "python": platform.python_version(), "source_hashes": hashes,
    }
    applicability._write_json(output/"protocol.json", {
        **_identity(), "config": config, "source_hashes": hashes,
        "inputs": str(Path(inputs).resolve()), "query_ids": list(query_ids),
    })
    applicability._write_json(output/"final_result.json", final)
    (output/"reproduction_command.txt").write_text(
        f"/usr/bin/python3 -m arena_evaluation.two_layer_v3_semantic_r14_benchmark "
        f"--inputs {Path(inputs).resolve()} --output {output} --queries {','.join(query_ids)}"
        f"{' --run-search' if run_search else ''}\n"
    )
    snapshot = output/"source_snapshot"; snapshot.mkdir()
    for source in sources:
        shutil.copy2(source, snapshot/source.name)
    applicability._write_json(output/"artifact_hashes.json", applicability._manifest_files(output))
    return final


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--queries", required=True, help="comma-separated query ids")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--run-search", action="store_true")
    args = parser.parse_args(argv)
    result = diagnose(
        inputs=args.inputs, output=args.output,
        query_ids=[value for value in args.queries.split(",") if value],
        config_path=args.config, run_search=args.run_search,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
