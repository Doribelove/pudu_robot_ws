"""Write-once input freeze and preparation for constrained PLN-02 research.

This module does not implement or name an online architecture. Historical r4
code is used only to reconstruct its exact inputs, never to reuse old paths.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import resource
import shutil
import subprocess
import sys
import time

import numpy as np
import yaml

from . import two_layer_v2_semantic_r4_benchmark as r4
from .semantic_map import canonical_hash, sha256_file
from .semantic_rasterizer import grid_hash

ROOT = r4.ROOT
PACKAGE = r4.PACKAGE_ROOT
CONFIG = PACKAGE / "config/semantic_constraint_feasibility_r0.yaml"


def write_json(path, payload):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True, ensure_ascii=False, default=r4._json_default)
        stream.write("\n")


def fresh(path):
    path = Path(path).resolve()
    path.mkdir(parents=True, exist_ok=False)
    return path


def git_bytes(repo, *args):
    return subprocess.check_output(["git", "-C", str(repo), *args])


def audit_repo(repo):
    status = git_bytes(repo, "status", "--porcelain=v1", "-z", "--untracked-files=all")
    changed = set()
    for command in (("diff", "--name-only", "-z"), ("diff", "--cached", "--name-only", "-z"),
                    ("ls-files", "--others", "--exclude-standard", "-z")):
        changed.update(x.decode() for x in git_bytes(repo, *command).split(b"\0") if x)
    files = {}
    for name in sorted(changed):
        p = repo / name
        if p.is_file():
            files[name] = {"sha256": sha256_file(p), "size": p.stat().st_size}
        else:
            files[name] = {"missing_or_directory": True}
    return {
        "repo": str(repo), "head": git_bytes(repo, "rev-parse", "HEAD").decode().strip(),
        "branch": git_bytes(repo, "branch", "--show-current").decode().strip(),
        "status": status.decode().replace("\0", "\n"),
        "status_sha256": hashlib.sha256(status).hexdigest(),
        "unstaged_patch_sha256": hashlib.sha256(git_bytes(repo, "diff", "--binary")).hexdigest(),
        "staged_patch_sha256": hashlib.sha256(git_bytes(repo, "diff", "--cached", "--binary")).hexdigest(),
        "preexisting_changed_files": files,
    }


def seal(output):
    write_json(output / "artifact_hashes.json", {
        str(p.relative_to(output)): sha256_file(p)
        for p in sorted(output.rglob("*")) if p.is_file()
    })


def freeze(output):
    output = fresh(output)
    config = yaml.safe_load(CONFIG.read_text())
    repos = (ROOT, PACKAGE.parent, ROOT / "external/arena4_ws/src/deps/nav2/navigation2")
    write_json(output / "worktree_before.json", [audit_repo(p) for p in repos])
    sources = [
        CONFIG, Path(__file__), r4.DEFAULT_CONFIG, r4.DEFAULT_TARGETS, r4.DEFAULT_SELECTED8,
        PACKAGE.parent / "AGENTS.md", ROOT / "docs/P0_EVALUATION_DEFINITION.md",
        ROOT / "docs/PLN-02_UNIFIED_EXPERIMENT_PROTOCOL_V1.md",
        ROOT / "docs/PLN-02_ARCHITECTURE_2A_V2_R4.md",
        ROOT / "private_data/pudu_wanda_3f/results/r4_stage2_se2_oracle_targeted_v4/final_report.md",
    ]
    sources += sorted((PACKAGE / "arena_evaluation").glob("*.py"))
    sources += sorted((PACKAGE / "src").glob("*.cpp"))
    sources += sorted((PACKAGE / "config").glob("*semantic*.yaml"))
    sources += [ROOT / "private_data/pudu_wanda_3f/extracted/optemap.yaml",
                ROOT / "private_data/pudu_wanda_3f/extracted/optemap.pgm",
                ROOT / "private_data/pudu_wanda_3f/results/conversion_v1/semantic_map_v1.json"]
    sources = list(dict.fromkeys(sources))
    hashes = {str(p): sha256_file(p) for p in sources}
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    for index, p in enumerate(sources):
        if p.stat().st_size < 2_000_000:
            shutil.copy2(p, snapshot / f"{index:03d}_{p.name}")
    historical = {}
    oldroot = ROOT / "private_data/pudu_wanda_3f/results"
    for folder in sorted(oldroot.iterdir()):
        if folder.is_dir() and (folder.name.startswith("r4_") or "r3" in folder.name):
            entries = {str(p.relative_to(folder)): sha256_file(p)
                       for p in sorted(folder.rglob("*")) if p.is_file()}
            historical[str(folder)] = {"file_count": len(entries), "hash": canonical_hash(entries), "files": entries}
    write_json(output / "historical_results_hashes.json", historical)
    config["resolved_parent_config_sha256"] = hashes[str(r4.DEFAULT_CONFIG)]
    config["targeted_query_content_hash"] = r4._query_content_hash(r4.DEFAULT_TARGETS)
    config["selected8_query_content_hash"] = r4._query_content_hash(r4.DEFAULT_SELECTED8)
    config["source_hashes"] = hashes
    config["freeze_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    write_json(output / "protocol.json", config)
    write_json(output / "environment.json", {
        "platform": platform.platform(), "python": sys.version, "executable": sys.executable,
        "cpu_count": os.cpu_count(), "affinity": sorted(os.sched_getaffinity(0)),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"), "rmw": os.environ.get("RMW_IMPLEMENTATION"),
        "processes_before": subprocess.check_output(["ps", "-eo", "pid,ppid,lstart,args"], text=True),
        "self_pid": os.getpid(), "owned_ros_processes": [],
    })
    seal(output)
    print(json.dumps({"stage0": str(output), "protocol_sha256": sha256_file(output / "protocol.json")}))


def prepare(output, stage0):
    output = fresh(output)
    frozen = json.loads((stage0 / "protocol.json").read_text())
    for p in (r4.DEFAULT_CONFIG, r4.DEFAULT_TARGETS, r4.DEFAULT_SELECTED8):
        if sha256_file(p) != frozen["source_hashes"][str(p)]:
            raise ValueError(f"frozen input changed: {p}")
    config = r4._load_config(r4.DEFAULT_CONFIG)
    started = time.monotonic()
    with r4._r4_bindings(), r4.r2._query_set_binding(r4.DEFAULT_TARGETS, require_default_contract=False):
        r4.r1._validate_protocol(config)
        prepared = r4.r1._prepare(
            ROOT / "private_data/pudu_wanda_3f/extracted",
            ROOT / "private_data/pudu_wanda_3f/results/conversion_v1/semantic_map_v1.json",
            ROOT / "private_data/pudu_wanda_3f/results/real_ablation_r1_diag_v20_final8/topology_cache",
            config, output=None,
        )
    ctx, semantic_map, raster, topology, _, router, bundle = prepared[:7]
    queries = bundle[0]
    assert [q.query_id for q in queries] == config["frozen_bindings"]["targeted_query_ids"]
    assert ctx.map_sha256 == config["frozen_bindings"]["map_hash"]
    assert semantic_map.semantic_map_hash == config["frozen_bindings"]["semantic_map_hash"]
    selector = r4.r1._semantic_selector(topology, router)
    builder = r4.RegionalPreferenceBuilderR3(ctx.hospital_map, raster, policy=config["regional_preference"], semantic_map=semantic_map)
    composer = r4.SemanticCostmapComposerR2(policy=config["l3_soft_cost"], inflation_cache_capacity=2)
    oldbindings = json.loads((r4.ROOT / "private_data/pudu_wanda_3f/results/r4_stage2_se2_oracle_targeted_v4/route_bindings.json").read_text())
    for query in queries:
        _, _, route, reason = selector(topology, query, cache_mode=r4.r1.r2_runtime.CACHE_MODE_OPTIMIZED, timing={})
        if route is None:
            raise ValueError(f"route unavailable {query.query_id}: {reason}")
        route, orientation = r4.orient_route_for_query(route, query)
        allowed = r4.r1.r2_runtime._raw_corridor_mask(ctx, topology, route, query, float(config["roi"]["r0_padding_m"]))
        allowed, roi_diag = r4.expand_roi_to_route_lane_instances(
            ctx.hospital_map, raster, semantic_map, route.polyline, allowed,
            free_mask=r4.r1.r2_runtime._raw_free_mask(ctx),
            route_probe_radius_m=float(config["roi"]["lane_route_probe_radius_m"]),
        )
        pref = builder.build(route.polyline, goal=query.goal, allowed_mask=allowed,
                             relaxation_level="R0", planning_preference_enabled=True, route_diagnostics=orientation)
        cm = composer.compose(ctx.hospital_map.occupancy, raster, pref, allowed_mask=allowed,
                              hard_semantics_enabled=True, soft_class_costs_enabled=True,
                              regional_preference_enabled=True, hard_semantics_use_footprint=True)
        binding = {
            "query": query.as_dict(), "map_hash": ctx.map_sha256,
            "semantic_map_hash": semantic_map.semantic_map_hash,
            "targeted_query_content_hash": frozen["targeted_query_content_hash"],
            "stage0_protocol_sha256": sha256_file(stage0 / "protocol.json"),
            "expected_master_hash": cm.expected_master_hash,
            "route_hash": r4.r1._path_hash(route), "roi_hash": grid_hash(allowed),
            "selected_lane_labels": list(pref.diagnostics.get("guide_lane_instance_ids", [])),
            "guide_polylines_world": pref.diagnostics.get("guide_polylines_world", []),
            "route_polyline": route.polyline, "route_orientation": orientation, "roi_diagnostics": roi_diag,
            "preference_diagnostics": pref.diagnostics,
            "map": {"resolution": ctx.hospital_map.resolution, "origin": ctx.hospital_map.origin,
                    "width": ctx.hospital_map.width, "height": ctx.hospital_map.height,
                    "image_path": str(ctx.hospital_map.image_path)},
        }
        old = oldbindings[query.query_id]
        for key, oldkey in (("expected_master_hash", "costmap_hash"), ("route_hash", "route_hash"), ("roi_hash", "roi_hash")):
            if binding[key] != old[oldkey]:
                raise ValueError(f"r4 input mismatch {query.query_id} {key}")
        path = output / f"{query.query_id}.npz"
        np.savez_compressed(path, master=cm.expected_master_cost, occupancy=ctx.hospital_map.occupancy,
                            allowed=allowed, labels=pref.lane_instance_id, error=pref.lane_error_m,
                            correct=pref.lane_correct_side, right=pref.lane_distance_to_right_m,
                            left=pref.lane_distance_to_left_m, hard=raster.hard_footprint_mask,
                            no_stopping=raster.no_stopping_mask)
        binding["npz_sha256"] = sha256_file(path)
        write_json(output / f"{query.query_id}.json", binding)
        print(json.dumps({"prepared": query.query_id, "expected_master": cm.expected_master_hash}), flush=True)
    write_json(output / "preparation.json", {"wall_s": time.monotonic() - started,
               "peak_rss_mib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
               "fresh_inputs_reconstructed": True, "old_paths_read": False, "ros_processes_started": []})
    seal(output)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("freeze", "prepare"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stage0", type=Path)
    args = parser.parse_args(argv)
    if args.mode == "freeze":
        freeze(args.output)
    else:
        if args.stage0 is None:
            parser.error("prepare requires --stage0")
        prepare(args.output, args.stage0.resolve())


if __name__ == "__main__":
    main()
