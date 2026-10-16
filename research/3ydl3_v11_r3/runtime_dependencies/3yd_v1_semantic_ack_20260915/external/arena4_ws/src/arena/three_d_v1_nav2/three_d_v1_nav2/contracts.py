"""Immutable input and source contracts for the integrated experiment."""

from __future__ import annotations

import hashlib
import csv
import math
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence

import yaml


ROOT = Path("/home/robot/pudu_robot_ws")
PDMAP = ROOT / "private_data/pudu_wanda_3f/source/LTMjMTEjMDcwNl8yd_S4h_i_vi0z5qW8.pdmap"
QUERY_SET = ROOT / (
    "external/arena4_ws/src/arena/evaluation/arena_evaluation/config/"
    "pudu_wanda_3f_selected8_gt50m_r2_v2.yaml"
)
FROZEN_SNAPSHOT = ROOT / (
    "experiments/layered_planner_benchmark/3d_v1_r1_heldout_20260904_01/source_snapshot"
)
THREE_D_V1_SOURCE = ROOT / "external/arena4_ws/src/arena/three_d_v1"

EXPECTED_PDMAP_SHA256 = "ffb5c838f282a9074c4afcf69915b24fb875cc1008d298c6c835ea39ce03d731"
EXPECTED_MAP_SHA256 = "05cf18d0df40235f69ba5f0168bb490f9175541431c0c516a962e7ce1965529a"
EXPECTED_SEMANTIC_MAP_HASH = "2560a4f4c86a86aeaf9993262648aaeb26998948e79fe3b92ecf47b6e69d0553"
EXPECTED_QUERY_HASH = "7e2a5ddb7a91b175779c0cfc1063dad77bf1c926ee52be94c350203204bac43e"
EXPECTED_QUERY_FILE_SHA256 = "b3307d4578447131e71db16156cd2f72e9f5042f98bdce0d75d21fe81300738d"
EXPECTED_QUERY_IDS = (
    "cmp2-01-lane-north", "cmp2-02-lane-south", "cmp2-03-speed-bump",
    "cmp2-04-multi-junction", "cmp2-05-junction-turn", "cmp2-06-lane-to-parking",
    "cmp2-07-parking-to-lane", "cmp2-08-parking-internal",
)

FROZEN_FILES = (
    "arena_3d_v1/r1_pipeline.py",
    "arena_3d_v1/l2_state_lifecycle.py",
    "arena_3d_v1/production_l1.py",
    "arena_3d_v1/pipeline.py",
    "arena_3d_v1/dynamic_policy.py",
    "arena_3d_v1/l2_incremental.py",
    "config/three_d_v1_r1_l2_lifecycle.yaml",
)

MAP_RESOLUTION_M = 0.05
MAP_ORIGIN = (-81.025999, -84.258104, 0.0)
MAP_WIDTH = 2138
MAP_HEIGHT = 4020
FOOTPRINT = ((0.255, 0.215), (0.255, -0.215), (-0.255, -0.215), (-0.255, 0.215))
FOOTPRINT_PADDING_M = 0.010
GOAL_POSITION_TOLERANCE_M = 0.125
GOAL_YAW_TOLERANCE_RAD = math.radians(5.0)
CONTROLLER_GOAL_POSITION_TOLERANCE_M = 0.110
CONTROLLER_GOAL_YAW_MAX_RAD = math.radians(4.0)
CONTROLLER_TO_FINAL_YAW_MARGIN_RAD = 0.005


class ContractError(RuntimeError):
    """The run must stop because a frozen contract no longer matches."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ContractError(message)


def controller_goal_yaw_tolerance(goal_yaw: float, l3_endpoint_yaw: float) -> float:
    """Return the frozen per-query Nav2 yaw budget relative to the L3 endpoint.

    Nav2's controller goal checker evaluates the path endpoint, while the final
    mission audit evaluates the original action goal. Reserve the known 48-bin
    endpoint offset plus a fixed audit margin inside the authoritative 5 degree
    final tolerance.
    """
    endpoint_offset = abs(math.atan2(
        math.sin(l3_endpoint_yaw - goal_yaw),
        math.cos(l3_endpoint_yaw - goal_yaw),
    ))
    tolerance = min(
        CONTROLLER_GOAL_YAW_MAX_RAD,
        GOAL_YAW_TOLERANCE_RAD - endpoint_offset - CONTROLLER_TO_FINAL_YAW_MARGIN_RAD,
    )
    require(tolerance > 0.0, "L3 endpoint yaw leaves no controller acceptance budget")
    return tolerance


def verify_frozen_sources(
    source_root: Path = THREE_D_V1_SOURCE, snapshot_root: Path = FROZEN_SNAPSHOT,
) -> Dict[str, str]:
    """Require every runtime r1 file to equal the authoritative source snapshot."""
    hashes: Dict[str, str] = {}
    for relative in FROZEN_FILES:
        current = source_root / relative
        snapshot = snapshot_root / relative
        require(current.is_file(), f"missing current frozen-r1 source: {current}")
        require(snapshot.is_file(), f"missing authoritative frozen-r1 snapshot: {snapshot}")
        current_hash = sha256_file(current)
        snapshot_hash = sha256_file(snapshot)
        require(current_hash == snapshot_hash, f"frozen-r1 source drift: {relative}")
        hashes[str(current)] = current_hash
    return hashes


def verify_map_yaml(path: Path) -> Mapping[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    require(isinstance(payload, dict), "map YAML must be a mapping")
    require(float(payload.get("resolution")) == MAP_RESOLUTION_M, "map resolution drift")
    origin = tuple(float(value) for value in payload.get("origin", ()))
    require(origin == MAP_ORIGIN, f"map origin drift: {origin}")
    require(payload.get("mode", "trinary") == "trinary", "map mode must be trinary")
    require(payload.get("negate") == 0, "map negate drift")
    require(payload.get("occupied_thresh") == 0.65, "map occupied threshold drift")
    require(payload.get("free_thresh") == 0.196, "map free threshold drift")
    return payload


def verify_static_inputs(
    *, pdmap: Path = PDMAP, query_set: Path = QUERY_SET,
    map_pgm: Path, map_yaml: Path, semantic_map_hash: str,
) -> Dict[str, Any]:
    """Fail closed on every map/query/source identity and geometric constant."""
    from arena_evaluation.semantic_query_defaults import load_query_set

    require(pdmap.is_file(), f"pdmap missing: {pdmap}")
    require(sha256_file(pdmap) == EXPECTED_PDMAP_SHA256, "source pdmap SHA-256 mismatch")
    require(map_pgm.is_file(), f"derived PGM missing: {map_pgm}")
    require(sha256_file(map_pgm) == EXPECTED_MAP_SHA256, "occupancy PGM SHA-256 mismatch")
    require(sha256_file(query_set) == EXPECTED_QUERY_FILE_SHA256, "query YAML file SHA-256 mismatch")
    require(semantic_map_hash == EXPECTED_SEMANTIC_MAP_HASH, "semantic canonical hash mismatch")
    map_metadata = verify_map_yaml(map_yaml)
    image_path = (map_yaml.parent / str(map_metadata["image"])).resolve()
    require(image_path == map_pgm.resolve(), "map YAML references a different image")
    from PIL import Image
    with Image.open(map_pgm) as image:
        require(image.size == (MAP_WIDTH, MAP_HEIGHT), "map dimensions drift")
    queries, intents, query_metadata = load_query_set(
        query_set, actual_map_hash=EXPECTED_MAP_SHA256,
        actual_semantic_map_hash=EXPECTED_SEMANTIC_MAP_HASH,
        require_default_contract=True,
    )
    require(tuple(item.query_id for item in queries) == EXPECTED_QUERY_IDS, "query order drift")
    require(query_metadata.get("query_hash") == EXPECTED_QUERY_HASH, "logical query hash mismatch")
    require(len(intents) == 8, "intent count must be eight")
    return {
        "pdmap_sha256": sha256_file(pdmap),
        "map_sha256": sha256_file(map_pgm),
        "map_yaml_sha256": sha256_file(map_yaml),
        "query_file_sha256": sha256_file(query_set),
        "query_hash": query_metadata["query_hash"],
        "semantic_map_hash": semantic_map_hash,
        "query_ids": list(EXPECTED_QUERY_IDS),
        "resolution_m": float(map_metadata["resolution"]),
        "origin": list(map_metadata["origin"]),
        "width": MAP_WIDTH,
        "height": MAP_HEIGHT,
        "footprint": [list(vertex) for vertex in FOOTPRINT],
        "footprint_padding_m": FOOTPRINT_PADDING_M,
    }


def verify_run_inputs(run_root: Path) -> Dict[str, Any]:
    """Validate actual inputs before any simulator or navigation process starts."""
    from arena_evaluation.semantic_map import SemanticMapV1
    from arena_evaluation.semantic_query_defaults import load_query_set

    run_root = run_root.resolve()
    bank = run_root / "path_bank"
    manifest = yaml.safe_load((bank / "manifest.yaml").read_text())
    require(manifest.get("complete") is True and manifest.get("path_count") == 8,
            "path bank is incomplete")
    semantic = SemanticMapV1.load(run_root / "derived_map/semantic_conversion/semantic_map_v1.json")
    require(semantic.frame_id == "map", "semantic frame drift")
    require(semantic.source_pdmap_hash == EXPECTED_PDMAP_SHA256, "semantic source drift")
    inputs = verify_static_inputs(
        map_pgm=run_root / "derived_map/extracted/optemap.pgm",
        map_yaml=run_root / "derived_map/extracted/optemap.yaml",
        semantic_map_hash=semantic.semantic_map_hash,
    )
    inputs["frozen_sources"] = verify_frozen_sources()
    index = bank / "index.csv"
    require(sha256_file(index) == manifest["index_sha256"], "path bank index drift")
    for name, expected in manifest["topology_files"].items():
        require(sha256_file(run_root / "derived_map/topology_cache" / name) == expected,
                f"topology drift: {name}")
    queries, _, _ = load_query_set(
        QUERY_SET, actual_map_hash=inputs["map_sha256"],
        actual_semantic_map_hash=inputs["semantic_map_hash"], require_default_contract=True,
    )
    with index.open(newline="") as stream:
        records = list(csv.DictReader(stream))
    require(tuple(row["query_id"] for row in records) == EXPECTED_QUERY_IDS, "path-bank order drift")
    for query, record in zip(queries, records):
        for label, values in (("start", query.start), ("goal", query.goal)):
            require(tuple(float(record[f"{label}_{axis}"]) for axis in ("x", "y", "yaw")) == tuple(values),
                    f"path-bank coordinates drift: {query.query_id} {label}")
        path = bank / record["path_file"]
        require(sha256_file(path) == record["path_sha256"], f"path drift: {query.query_id}")
        require(record["final_audit_passed"] == "true", f"unaudited path: {query.query_id}")
    return inputs


def verify_vehicle_contract(values: Mapping[str, Any]) -> None:
    expected = {
        "allow_reverse": False,
        "allow_rotate_in_place": False,
        "minimum_turning_radius_m": 0.40,
        "maximum_curvature_1pm": 2.50,
    }
    for key, value in expected.items():
        require(values.get(key) == value, f"vehicle contract drift for {key}: {values.get(key)!r}")


__all__ = [name for name in globals() if name.startswith("EXPECTED_")] + [
    "ContractError", "CONTROLLER_GOAL_POSITION_TOLERANCE_M", "CONTROLLER_GOAL_YAW_MAX_RAD",
    "CONTROLLER_TO_FINAL_YAW_MARGIN_RAD", "FOOTPRINT", "FOOTPRINT_PADDING_M",
    "GOAL_POSITION_TOLERANCE_M", "GOAL_YAW_TOLERANCE_RAD", "MAP_HEIGHT", "MAP_ORIGIN",
    "MAP_RESOLUTION_M", "MAP_WIDTH", "controller_goal_yaw_tolerance",
    "PDMAP", "QUERY_SET", "sha256_file", "verify_frozen_sources", "verify_map_yaml",
    "verify_static_inputs", "verify_vehicle_contract",
]
