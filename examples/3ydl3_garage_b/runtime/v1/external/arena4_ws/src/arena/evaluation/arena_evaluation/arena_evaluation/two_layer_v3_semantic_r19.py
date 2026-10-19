"""Fast request-generated parking sentinel for 2A-V3 r19."""
from __future__ import annotations

import argparse
import contextlib
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import platform
import resource
import shutil
import subprocess
import time

import numpy as np
import yaml

from . import two_layer_v3_semantic_r17_parking_aisle as r17
from .semantic_goal_reachability_r19 import GoalReachableSearchR19
from .semantic_map import sha256_file
from .semantic_rasterizer import grid_hash

REVISION = "r19-goal-coreachable-corridor"
PROTOCOL = "PLN-02-2A-V3-R19-GOAL-COREACHABILITY-V1"
DEFAULT_CONFIG = r17.PACKAGE_ROOT / "config/two_layer_v3_semantic_r19_goal_reachability.yaml"
WORKSPACE = r17.PACKAGE_ROOT.parents[5]


def write_json(path, data):
    Path(path).write_text(json.dumps(r17.expanded._json_safe(data), indent=2,
                                   sort_keys=True, ensure_ascii=False) + "\n")


def load(config_path=DEFAULT_CONFIG):
    config_path = Path(config_path).resolve()
    config = yaml.safe_load(config_path.read_text())
    if (config["architecture_id"], config["implementation_revision"], config["protocol_id"]) != (
        "2A-V3", REVISION, PROTOCOL,
    ):
        raise ValueError("R19_IDENTITY_MISMATCH")
    parent = config_path.parent / config["parent_config"]["path"]
    if sha256_file(parent) != config["parent_config"]["sha256"]:
        raise ValueError("R17_PARENT_HASH_MISMATCH")
    if config["maximum_expanded_states"] != 12000 or config["graph_wall_budget_s"] > 30:
        raise ValueError("R19_RESOURCE_BOUND_DRIFT")
    return config, r17._load(parent)


def process_audit():
    return subprocess.check_output(["ps", "-eo", "pid,ppid,pgid,lstart,args"], text=True)


def run(output, config_path=DEFAULT_CONFIG):
    config, parents = load(config_path)
    parent17, parent15, algorithm, parent_algorithm, parent, *_paths = parents
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    output.joinpath("processes_before.txt").write_text(process_audit())
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    sources = sorted([*r17.PACKAGE_ROOT.joinpath("arena_evaluation").glob("*.py"),
                      *r17.PACKAGE_ROOT.joinpath("config").glob("*.yaml"),
                      r17.PACKAGE_ROOT / "setup.py"])
    sources += [WORKSPACE / "docs" / name for name in (
        "PLN-02_ARCHITECTURE_2A_V3_R17.md", "PLN-02_ARCHITECTURE_2A_V3_R16.md",
        "PLN-02_SEMANTIC_TRANSITION_CONTRACT_R2.md",
    )]
    hashes = {str(path): sha256_file(path) for path in sources}
    for path in sources:
        destination = snapshot / path.relative_to(WORKSPACE)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, destination)
    protocol = {"architecture_id": "2A-V3", "implementation_revision": REVISION,
                "protocol_id": PROTOCOL, "config": config,
                "parent_frozen_bindings": parent17["frozen_bindings"],
                "source_hashes": hashes, "python": platform.python_version(),
                "root_git": r17.r15._git_capture(WORKSPACE),
                "evaluation_git": r17.r15._git_capture(r17.PACKAGE_ROOT.parent),
                "nav2_git": r17.r15._git_capture(WORKSPACE / "external/arena4_ws/src/deps/nav2/navigation2")}
    write_json(output / "protocol.json", protocol)
    output.joinpath("reproduction_command.txt").write_text(
        "source /opt/ros/humble/setup.bash\n"
        f"source {WORKSPACE}/external/arena4_ws/install/setup.bash\n"
        f"export PYTHONPATH={r17.PACKAGE_ROOT}:$PYTHONPATH\n"
        f"/usr/bin/python3 -m arena_evaluation.two_layer_v3_semantic_r19 --output <NEW_DIRECTORY> --config {Path(config_path).resolve()}\n")
    result = {"architecture_id": "2A-V3", "implementation_revision": REVISION,
              "query_id": config["sentinel_query"], "gate_passed": False,
              "hard_constraints_held": None, "online_started": False}
    try:
        query_path = r17.PACKAGE_ROOT / "config" / parent15["frozen_bindings"]["expanded32_path"]
        queries, _, query_meta = r17.load_query_set(
            query_path, actual_map_hash=parent17["frozen_bindings"]["map_hash"],
            actual_semantic_map_hash=parent17["frozen_bindings"]["semantic_map_hash"])
        if query_meta["query_hash"] != parent17["frozen_bindings"]["expanded32_query_hash"]:
            raise ValueError("QUERY_CONTENT_HASH_MISMATCH")
        query = next(q for q in queries if q.query_id == config["sentinel_query"])
        result["query"] = query.as_dict()
        prepared, result["static_prepare_ms"] = r17.r15._prepare_static(
            algorithm=algorithm, parent=parent, output=output)
        ctx, semantic_map, raster, topology, _, router = prepared[:6]
        builder = r17.expanded.AuditOnlyPreferenceBuilderR14(
            ctx.hospital_map, raster, policy=parent["regional_preference"], semantic_map=semantic_map)
        composer = r17.SemanticCostmapComposerR2(policy=parent["l3_soft_cost"], inflation_cache_capacity=2)
        selector = r17.r15._selector(parent, topology, router)
        (_, _, _, _, full_allowed, metadata, arrays) = r17.r15._prepare_query_state(
            query=query, query_hash=query_meta["query_hash"], selector=selector, ctx=ctx,
            semantic_map=semantic_map, raster=raster, topology=topology,
            route_phase_algorithm=parent_algorithm, parent=parent, builder=builder, composer=composer)
        write_json(output / "input_metadata.json", metadata)
        world = r17.CompactRoutePhaseWorldR14(
            output, query.query_id, arrays_override=arrays, meta_override=metadata,
            full_occupancy=ctx.hospital_map.occupancy, full_allowed=full_allowed)
        world._canonical_auditor = r17.PathAuditor(ctx, source_commit=REVISION)
        base = r17.r13._policy(parent_algorithm)
        conn = parent17["se2_local_connection"]
        policy = replace(base, maximum_station_skip=int(conn["maximum_station_skip"]),
                         maximum_local_edge_length_m=float(conn["maximum_local_edge_length_m"]),
                         maximum_local_edge_ratio=float(conn["maximum_local_edge_ratio"]))
        route = r17.OrientedRoute(metadata["route_polyline"], world.start, world.goal,
                                  endpoint_attachment_limit_m=policy.endpoint_attachment_limit_m)
        original_field = world.grids["parking_deviation"].copy()
        field = r17.build_route_local_aisle_field(world, route, r17._aisle_policy(parent17))
        write_json(output / "aisle_field.json", field.summary())
        if not field.gate_passed:
            raise ValueError(field.failure_code)
        world.grids["parking_deviation"] = field.deviation
        selected = np.isin(world.grids["parking_components"], world.selected_parking)
        world.grids["allowed"][selected & ~np.isfinite(field.deviation)] = False
        ref = r17.ContinuousParkingReferenceBuilder(
            r17.r14.ParkingReferencePolicy(**parent17["parking_reference"])).build(world, route)
        write_json(output / "reference.json", ref.to_dict())
        if not ref.gate_passed or ref.target_sample_ratio <= .5:
            raise ValueError("REFERENCE_GATE_FAILED")
        guide = r17.OrientedRoute(ref.polyline, world.start, world.goal,
                                  endpoint_attachment_limit_m=policy.endpoint_attachment_limit_m)
        write_json(output / "effective_bindings.json", {
            "source_route_hash": route.route_hash, "guide_hash": guide.route_hash,
            "master_crop_hash": grid_hash(world.master), "aisle_field_hash": field.diagnostics["field_sha256"],
            "allowed_crop_hash": grid_hash(world.grids["allowed"]), "query": query.as_dict(),
            "full_expected_master_hash": metadata["expected_master_hash"],
            "snapshot_origin": "OFFLINE_EXPECTED_MASTER_NOT_SERVER_ACK"})
        search = GoalReachableSearchR19(world, guide, policy)
        candidate, diagnostics = search.solve(
            semantic_map, wall_budget_s=config["graph_wall_budget_s"],
            terminal_attachment=bool(config.get("terminal_attachment", False)))
        write_json(output / "search.json", diagnostics)
        # Save geometry even on failure; next analysis can reproduce state
        # connectivity without mistaking any stored path for online output.
        write_json(output / "states.json", [s.__dict__ for s in search.states])
        result["search"] = diagnostics
        if candidate:
            write_json(output / "path.json", candidate.pop("points"))
            write_json(output / "controls.json", candidate.pop("controls"))
            path = json.loads(output.joinpath("path.json").read_text())
            candidate["r2_component_compatibility"] = r17._audit_field(world, path, original_field)
            write_json(output / "path_audit.json", candidate)
            result["gate_passed"] = candidate["gate_passed"]
            # A semantic/naturalness rejection is not itself a collision or
            # vehicle-constraint violation. Report the physical audit directly.
            canonical = candidate["canonical"]
            result["hard_constraints_held"] = bool(
                canonical["static_footprint_valid"] and canonical["kinematic_valid"]
                and canonical["maximum_curvature"] <= 2.5
                and canonical["reverse_distance_m"] == 0
                and canonical["in_place_rotation_count"] == 0
                and candidate["exact_endpoint_xy_yaw"]
                and candidate["padded_effective_master_collision_free"]
                and candidate["hard_features"]["hard_feature_gate_passed"])
        result["failure_code"] = diagnostics.get("failure_code", diagnostics["graph_status"])
    except Exception as error:
        result.update(failure_code=type(error).__name__, failure_detail=str(error))
        import traceback
        traceback.print_exc()
    finally:
        drift = {path: sha256_file(Path(path)) for path, old in hashes.items()
                 if sha256_file(Path(path)) != old}
        result["source_drift"] = drift
        if drift:
            result["gate_passed"] = False
            result["failure_code"] = "SOURCE_CHANGED_DURING_RUN"
        result["wall_s"] = time.monotonic()-started
        result["peak_rss_bytes"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024
        result["next_stage"] = "THREE_QUERY_OFFLINE" if result["gate_passed"] else "NOT_RUN_SENTINEL_GATE_FAILED"
        write_json(output / "result.json", result)
        output.joinpath("processes_after.txt").write_text(process_audit())
        write_json(output / "manifest.json", {
            str(path.relative_to(output)): sha256_file(path)
            for path in sorted(output.rglob("*")) if path.is_file()})
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args(argv)
    # run() owns the unique output directory; CLI logs are imported after
    # completion by the caller so no directory needs to be reused.
    result = run(args.output, args.config)
    print(json.dumps(r17.expanded._json_safe(result), indent=2))
    return 0 if result["gate_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
