"""Immutable map, query, source and path-bank contracts for 2A-V1-r2."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Dict

import yaml

from three_d_v1_nav2.contracts import (
    EXPECTED_MAP_SHA256,
    EXPECTED_PDMAP_SHA256,
    EXPECTED_QUERY_FILE_SHA256,
    EXPECTED_QUERY_HASH,
    EXPECTED_QUERY_IDS,
    EXPECTED_SEMANTIC_MAP_HASH,
    PDMAP,
    QUERY_SET,
    ContractError,
    require,
    sha256_file,
    verify_static_inputs,
)


ROOT = Path("/home/robot/pudu_robot_ws")
DELIVERY = ROOT / (
    "experiments/deliverables/"
    "2a_v1_r2_roi_pathaudit_repro_bundle_v1_20260903T050314Z"
)
SNAPSHOT = DELIVERY / "source_snapshot/workspace"
ARCHIVE = DELIVERY.with_suffix(".tar.zst")
EXPECTED_ARCHIVE_SHA256 = (
    "156818afd8f7f63a900948f517bb8094cd486f49441c28903be6542c78920d3a"
)

RUNTIME_SOURCE_RELATIVES = (
    "external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/endpoint_heading.py",
    "external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/l1_l3_corridor_hybrid_smoke.py",
    "external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/l1_l3_corridor_hybrid_validity.py",
    "external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/path_audit.py",
    "external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/planner_benchmark/runner.py",
    "external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/topology.py",
    "external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/two_layer_v1_formal_benchmark.py",
    "external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/two_layer_v1_r1_cache_benchmark.py",
    "external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/two_layer_v1_r2_roi_pathaudit_benchmark.py",
    "external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/unified_four_backends_smoke.py",
    "external/arena4_ws/src/arena/evaluation/arena_evaluation/config/planner_benchmark_strict_forward_smac_hybrid.yaml",
    "external/arena4_ws/src/deps/nav2/navigation2/nav2_smac_planner/include/nav2_smac_planner/a_star.hpp",
    "external/arena4_ws/src/deps/nav2/navigation2/nav2_smac_planner/include/nav2_smac_planner/smac_planner_hybrid.hpp",
    "external/arena4_ws/src/deps/nav2/navigation2/nav2_smac_planner/src/a_star.cpp",
    "external/arena4_ws/src/deps/nav2/navigation2/nav2_smac_planner/src/smac_planner_hybrid.cpp",
)


def verify_frozen_2a_sources() -> Dict[str, str]:
    require(DELIVERY.is_dir(), f"frozen 2A delivery missing: {DELIVERY}")
    require(ARCHIVE.is_file(), f"frozen 2A archive missing: {ARCHIVE}")
    require(sha256_file(ARCHIVE) == EXPECTED_ARCHIVE_SHA256, "2A delivery archive SHA-256 mismatch")
    hashes: Dict[str, str] = {}
    for relative in RUNTIME_SOURCE_RELATIVES:
        frozen = SNAPSHOT / relative
        current = ROOT / relative
        require(frozen.is_file(), f"frozen 2A source missing: {frozen}")
        require(current.is_file(), f"current 2A source missing: {current}")
        frozen_hash = sha256_file(frozen)
        if relative.endswith("/l1_l3_corridor_hybrid_smoke.py"):
            # This one file has later worktree edits. The runtime loader uses
            # the immutable delivery snapshot directly rather than altering it.
            hashes[str(frozen)] = frozen_hash
        else:
            require(sha256_file(current) == frozen_hash, f"frozen 2A source drift: {relative}")
            hashes[str(current)] = frozen_hash
    return hashes


def verify_2a_run_inputs(run_root: Path) -> Dict[str, Any]:
    from arena_evaluation.semantic_map import SemanticMapV1
    from arena_evaluation.semantic_query_defaults import load_query_set

    run_root = run_root.resolve()
    bank = run_root / "path_bank"
    semantic = SemanticMapV1.load(run_root / "derived_map/semantic_conversion/semantic_map_v1.json")
    inputs = verify_static_inputs(
        map_pgm=run_root / "derived_map/extracted/optemap.pgm",
        map_yaml=run_root / "derived_map/extracted/optemap.yaml",
        semantic_map_hash=semantic.semantic_map_hash,
    )
    inputs["frozen_2a_sources"] = verify_frozen_2a_sources()
    manifest = yaml.safe_load((bank / "manifest.yaml").read_text(encoding="utf-8"))
    require(manifest.get("architecture_id") == "2A-V1", "path bank is not frozen 2A-V1")
    require(manifest.get("implementation_revision") == "r2-roi-pathaudit-v1", "2A revision drift")
    require(manifest.get("complete") is True and manifest.get("path_count") == 8,
            "2A path bank is incomplete")
    index = bank / "index.csv"
    require(sha256_file(index) == manifest.get("index_sha256"), "2A path bank index drift")
    for name, expected in manifest.get("topology_files", {}).items():
        require(sha256_file(run_root / "derived_map/topology_cache" / name) == expected,
                f"topology drift: {name}")
    queries, _, _ = load_query_set(
        QUERY_SET, actual_map_hash=EXPECTED_MAP_SHA256,
        actual_semantic_map_hash=EXPECTED_SEMANTIC_MAP_HASH,
        require_default_contract=True,
    )
    with index.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    require(tuple(row["query_id"] for row in rows) == EXPECTED_QUERY_IDS, "2A query order drift")
    for query, row in zip(queries, rows):
        for label, values in (("start", query.start), ("goal", query.goal)):
            actual = tuple(float(row[f"{label}_{axis}"]) for axis in ("x", "y", "yaw"))
            require(actual == tuple(values), f"2A coordinate drift: {query.query_id} {label}")
        path = bank / row["path_file"]
        require(sha256_file(path) == row["path_sha256"], f"2A path drift: {query.query_id}")
        require(row.get("final_audit_passed") == "true", f"unaudited 2A path: {query.query_id}")
        require(row.get("l2_called") == "false", f"2A path falsely claims L2: {query.query_id}")
    return inputs


__all__ = [
    "ARCHIVE", "DELIVERY", "EXPECTED_ARCHIVE_SHA256", "RUNTIME_SOURCE_RELATIVES",
    "SNAPSHOT", "verify_2a_run_inputs", "verify_frozen_2a_sources",
]
