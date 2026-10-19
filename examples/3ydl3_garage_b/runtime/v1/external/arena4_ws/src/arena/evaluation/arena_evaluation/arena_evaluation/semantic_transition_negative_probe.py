"""Bounded smooth-connection probe for the frozen negative transition query.

No map, endpoint, contract or safety changes occur. Candidate generation uses
physical target-band portals and larger turning radii. Loops, excessive turn,
and large backward route progress are excluded before semantic evaluation.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import itertools
import json
import math
from pathlib import Path
import resource
import shutil
import time

import numpy as np

from .semantic_constraint_core import ConstraintWorld, dense_interpolate, dubins_edge, replay_edge
from .semantic_map import sha256_file
from .semantic_transition_online_adapter import _full_auditor, _semantic_metrics


QUERY = "r3-mirror-2-negative"
RADII = (0.401, 0.5, 0.65, 0.8, 1.0, 1.25, 1.5, 1.75, 2.0)


def naturalness(path: np.ndarray, edges, start, goal) -> dict:
    direction = np.asarray(goal[:2]) - np.asarray(start[:2])
    direction /= np.linalg.norm(direction)
    station = (path[:, :2] - np.asarray(start[:2])) @ direction
    regression = float(np.maximum(-np.diff(station), 0.0).sum())
    max_turn = max((p for edge in edges for k, p in zip(edge.word, edge.params) if k != "S"), default=0.0)
    total_turn = sum(p for edge in edges for k, p in zip(edge.word, edge.params) if k != "S")
    # Exact non-adjacent segment intersections on a 0.1 m downsample suffice
    # as an anti-loop diagnostic; they are not the collision checker.
    xy = path[::4, :2]
    if not np.array_equal(xy[-1], path[-1, :2]):
        xy = np.vstack((xy, path[-1, :2]))
    crossings = 0
    for i in range(len(xy) - 1):
        a, b = xy[i], xy[i + 1]
        c, d = xy[i + 2:-1], xy[i + 3:]
        if not len(c):
            continue
        ab, cd = b - a, d - c
        cross = lambda u, v: u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]
        first, second = cross(ab, c - a), cross(ab, d - a)
        third, fourth = cross(cd, a - c), cross(cd, b - c)
        crossings += int(np.count_nonzero((first * second < -1e-12) & (third * fourth < -1e-12)))
    passed = bool(max_turn <= math.pi and total_turn <= 2 * math.pi
                  and regression <= 0.5 and crossings == 0)
    return {"naturalness_passed": passed, "maximum_arc_turn_rad": float(max_turn),
            "total_absolute_turn_rad": float(total_turn), "backward_route_progress_m": regression,
            "self_intersection_count": crossings}


def portal_candidates(world):
    # Fixed lane-station rows; X comes only from the input's actual target band.
    entry_y = (-11.5, -12.0, -12.5, -13.0, -13.5)
    exit_y = (-16.0, -16.5, -17.0, -17.5, -18.0)
    def at_y(y):
        r, _ = world.map.world_to_cell(world.start[0], y)
        allowed = ((world.grids["labels"][r] == world.selected[0]) & world.grids["correct"][r]
                   & (world.grids["error"][r] <= .5) & (world.master[r] < 253))
        cols = np.flatnonzero(allowed)
        if not len(cols):
            return []
        # Cover inner/central/outer parts without arbitrary offset waypoints.
        chosen = np.unique(cols[np.linspace(0, len(cols)-1, 5).astype(int)])
        return [(world.map.cell_to_world((r, int(c)))[0], float(y), -math.pi/2) for c in chosen]
    return [p for y in entry_y for p in at_y(y)], [p for y in exit_y for p in at_y(y)]


def candidate_specs(world, strategy):
    entries, exits = portal_candidates(world)
    if strategy == "direct_radius":
        for radius in RADII + (2.5, 3.0, 4.0):
            yield {"radius_m": radius, "waypoints": [world.start, world.goal]}
    elif strategy == "single_target_portal":
        for radius, pose in itertools.product(RADII, entries + exits):
            yield {"radius_m": radius, "waypoints": [world.start, pose, world.goal]}
    elif strategy == "target_corridor":
        for radius, a, b in itertools.product(RADII, entries, exits):
            # Match one side of the physical target corridor, avoiding zigzags.
            if abs(a[0] - b[0]) <= .20:
                yield {"radius_m": radius, "waypoints": [world.start, a, b, world.goal]}
    else:
        raise ValueError(strategy)


def run(inputs: Path, output: Path, strategy: str, timeout: float, maximum: int) -> dict:
    output.mkdir(parents=False, exist_ok=False)
    world = ConstraintWorld(inputs, QUERY)
    source = Path(__file__)
    shutil.copy2(source, output / source.name)
    started, cpu = time.monotonic(), time.process_time()
    rows, saved = [], []
    canonical = allowed = None
    failures = Counter()
    for index, spec in enumerate(candidate_specs(world, strategy)):
        if index >= maximum or time.monotonic() - started >= timeout:
            break
        edges = [dubins_edge(a, b, spec["radius_m"]) for a, b in zip(spec["waypoints"], spec["waypoints"][1:])]
        row = {"candidate_index": index, "strategy": strategy, **spec}
        if any(e is None for e in edges):
            row["failure_code"] = "NO_DUBINS_CONNECTION"
            rows.append(row); failures[row["failure_code"]] += 1; continue
        path = np.vstack((edges[0].start, *(e.samples for e in edges)))
        row.update(naturalness(path, edges, world.start, world.goal))
        row["arc_length_m"] = float(sum(e.length for e in edges))
        row["maximum_control_curvature_1pm"] = 1.0 / spec["radius_m"]
        if not row["naturalness_passed"]:
            row["failure_code"] = "UNNATURAL_LOOP_OR_PROGRESS"
            rows.append(row); failures[row["failure_code"]] += 1; continue
        dense = dense_interpolate(path)
        safe = bool(world.collision_free(dense) and world.semantic_counts(dense) is not None)
        row["dense_safety_valid"] = safe
        if not safe:
            row["failure_code"] = "FOOTPRINT_OR_HARD_SEMANTIC"
            rows.append(row); failures[row["failure_code"]] += 1; continue
        points = [{"x":float(x),"y":float(y),"yaw":float(yaw),"source":"kinematic",
                   "motion_direction":"forward","steering":0.,
                   "planner_backend":"negative_natural_connection_probe","backend_version":"r0"} for x,y,yaw in path]
        full = _semantic_metrics(world, points, 0.0)
        window = _semantic_metrics(world, points, 6.0)
        row.update({"full_side":full["correct_side_ratio"],"full_band":full["target_band_ratio"],
                    "full_p50_m":full["lateral_error_p50_m"],"window":window})
        if canonical is None:
            canonical, allowed = _full_auditor(world)
        audit = canonical.audit(world.query, points, allowed)
        control_replay = all(np.array_equal(replay_edge(e.certificate()).samples,e.samples) for e in edges)
        exact = bool(np.array_equal(path[0],world.start) and np.array_equal(path[-1],world.goal))
        cell = world.map.world_to_cell(*world.goal[:2])
        hard = bool(audit.final_valid_success and audit.metrics["maximum_curvature"] <= 2.5
                    and control_replay and exact and not world.grids["no_stopping"][cell]
                    and row["arc_length_m"] <= world.bound_length)
        row.update({"canonical":audit.metrics,"control_replay_passed":control_replay,
                    "exact_endpoint":exact,"hard_gate_passed":hard,
                    "transition_gate_passed":bool(hard and window["semantic_gate_passed"]),
                    "path_length_m":full["path_length_m"]})
        row["failure_code"] = "" if row["transition_gate_passed"] else (
            "HARD_GATE_FAILED" if not hard else window["failure_code"])
        failures[row["failure_code"] or "PASS"] += 1
        # Save every safe candidate's controls/path for independent review.
        folder=output/f"candidate_{index:05d}"
        folder.mkdir()
        for name,payload in (("path.json",points),("controls.json",{"edges":[e.certificate() for e in edges]}),("audit.json",row)):
            (folder/name).write_text(json.dumps(payload,indent=2,allow_nan=False)+"\n")
        row["artifact_dir"] = folder.name
        saved.append(row)
        rows.append(row)
    manifest = {"protocol_id":"PLN-02-NEGATIVE-NATURAL-CONNECTION-PROBE-R0-V1",
                "contract_revision":"semantic-endpoint-transition-6m-r1","query_id":QUERY,
                "strategy":strategy,"seed":0,"deterministic":True,"timeout_s":timeout,"max_candidates":maximum,
                "source_sha256":sha256_file(source),"input_meta_sha256":sha256_file(inputs/f"{QUERY}.json"),
                "input_npz_sha256":world.meta["npz_sha256"],"map_hash":world.meta["map_hash"],
                "semantic_map_hash":world.meta["semantic_map_hash"],"start":world.start,"goal":world.goal,
                "naturalness_policy":{"max_single_turn_rad":math.pi,"max_total_turn_rad":2*math.pi,
                                       "max_backward_route_progress_m":.5,"self_intersections":0},
                "objective":"sample lower-curvature alternatives through true target portals; no length reward",
                "scope":"bounded_offline_candidate_family_not_infeasibility_proof_or_online_result"}
    summary={"candidate_count":len(rows),"safe_saved_count":len(saved),
             "transition_pass_count":sum(r.get("transition_gate_passed",False) for r in rows),
             "failure_counts":dict(failures),"wall_s":time.monotonic()-started,"cpu_s":time.process_time()-cpu,
             "peak_rss_bytes":resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
             "safe_length_range_m":[min((r["path_length_m"] for r in saved),default=None),max((r["path_length_m"] for r in saved),default=None)],
             "best_passing":min((r for r in saved if r["transition_gate_passed"]),key=lambda r:r["path_length_m"],default=None)}
    for name,payload in (("manifest.json",manifest),("candidates.json",rows),("summary.json",summary)):
        (output/name).write_text(json.dumps(payload,indent=2,allow_nan=False)+"\n")
    hashes={str(p.relative_to(output)):sha256_file(p) for p in output.rglob("*") if p.is_file()}
    (output/"artifact_hashes.json").write_text(json.dumps(hashes,indent=2,sort_keys=True)+"\n")
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--inputs",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--strategy",choices=("direct_radius","single_target_portal","target_corridor"),required=True)
    p.add_argument("--timeout",type=float,default=60.0)
    p.add_argument("--max-candidates",type=int,default=10000)
    a=p.parse_args()
    print(json.dumps(run(a.inputs,a.output,a.strategy,min(a.timeout,60.),min(a.max_candidates,10000)),indent=2))


if __name__=="__main__":
    main()
