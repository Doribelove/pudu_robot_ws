"""Bounded live-input Hybrid candidate under the approved R2 statistics.

This is an offline candidate experiment, not the ROS online comparison. The
existing explicit-reference 48-bin core is reused without changing Nav2.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import resource
import time

import numpy as np

from .semantic_constraint_core import ConstraintWorld, dubins_edge
from .semantic_map import sha256_file
from .semantic_transition_contract import TransitionContractR2, audit_transition_samples, resample_path
from .semantic_transition_preflight import write_json
from .se2_semantic_guide import ExplicitSE2GuideOracle, SE2GuidePolicy, wrap_angle


class TransitionHybrid(ExplicitSE2GuideOracle):
    def _snapped_pose(self, pose):
        return tuple(map(float, pose)) if self.map.world_to_cell(*pose[:2]) is not None else None

    def _path_metrics(self, path, trace):
        result = super()._path_metrics(path, trace)
        if not result.get("path_collision_free"):
            return result
        points = [dict(zip(("x", "y", "yaw"), pose)) for pose in path]
        samples, stations = resample_path(points)
        cells = [self.map.world_to_cell(p["x"], p["y"]) for p in samples]
        if any(cell is None for cell in cells):
            result["semantic_metric_gate_passed"] = False
            return result
        rows, cols = np.array(cells).T
        audit = audit_transition_samples(
            path_length_m=float(stations[-1]), station_m=stations,
            lane_mask=np.isin(self.labels[rows, cols], tuple(self.selected)),
            lane_error_m=self.error[rows, cols], lane_correct_side=self.correct[rows, cols],
        )
        lane = audit["active_window"]["classes"]["lane"]
        result.update({"contract_revision": TransitionContractR2().contract_revision,
                       "full_path_lane_correct_side_ratio": result["lane_correct_side_ratio"],
                       "lane_correct_side_ratio": lane.get("correct_side_ratio"),
                       "lane_target_error_p50_m": lane.get("lateral_error_p50_m"),
                       "lane_target_band_ratio": lane.get("target_band_ratio"),
                       "contract_metrics": audit,
                       "semantic_metric_gate_passed": bool(audit["semantic_gate_passed"]
                         and result["reverse_distance_m"] == 0.0 and result["in_place_rotation_count"] == 0
                         and result["path_length_m"] <= self.length_bound)})
        return result


def run(inputs, query, output, timeout=60.0):
    output.mkdir(parents=False, exist_ok=False)
    source_hash = sha256_file(Path(__file__))
    initialization = time.monotonic()
    world = ConstraintWorld(inputs, query)
    policy = replace(SE2GuidePolicy(), minimum_turning_radius_m=.401,
                     maximum_curvature_1pm=1/.401, timeout_s=timeout, reference_deviation_weight=2.0,
                     reference_yaw_weight=1.0, wrong_side_weight=3.0)
    oracle = TransitionHybrid(
        world.map, world.master, world.grids["labels"], world.grids["error"], world.grids["correct"],
        world.selected, policy=policy, binding={"query": world.meta["query"], "input_sha256": world.meta["npz_sha256"]},
        reference_polylines=world.meta["guide_polylines_world"],
    )
    oracle.length_bound = world.bound_length
    for name in ("full_origin", "full_height", "row0", "col0"):
        setattr(oracle.checker._collision_map, name, getattr(world.map, name))
    init_s = time.monotonic() - initialization
    begin, cpu = time.monotonic(), time.process_time()
    candidate = oracle.search(query, world.start, world.goal)
    result = {"candidate": "explicit_reference_48bin_hybrid", "query_id": query,
              "contract_revision": TransitionContractR2().contract_revision,
              "source_sha256_at_start": source_hash, "used_historical_paths": False,
              "input_meta_sha256": sha256_file(inputs / f"{query}.json"),
              "input_npz_sha256": world.meta["npz_sha256"], "policy": asdict(policy),
              "map_hash": world.meta["map_hash"], "semantic_map_hash": world.meta["semantic_map_hash"],
              "initializer_s": init_s, "solve_wall_s": time.monotonic()-begin,
              "solve_cpu_s": time.process_time()-cpu,
              "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
              "oracle": asdict(candidate), "strict_gate_passed": False,
              "proof_scope": "bounded_48bin_search_not_continuous_proof", "online": False}
    if candidate.witness_exists:
        path = np.asarray(candidate.path)
        knots = [world.start, *(tuple(path[int(item["end_path_index"])]) for item in candidate.primitive_trace)]
        knots[-1] = world.goal
        edges = [dubins_edge(a, b, radius=.401) for a, b in zip(knots, knots[1:])]
        if all(edge is not None for edge in edges):
            audit, exact_path = world.audit(edges)
            result["canonical_audit"] = audit
            points = [{"x":float(x), "y":float(y), "yaw":float(yaw)} for x,y,yaw in exact_path]
            from .semantic_transition_r2_preflight import semantic_metrics
            semantic = semantic_metrics(world, points)
            result["reconstructed_semantic_metrics"] = semantic
            result["strict_gate_passed"] = bool(audit.get("canonical",{}).get("final_valid_success")
                and audit["padded_effective_master_collision_free"] and audit["exact_endpoint_xy_yaw"]
                and audit["trace_replay_exact"] and audit["same_lane_instance"]
                and audit["maximum_control_curvature_1pm"] <= 2.50 and semantic["semantic_gate_passed"])
            write_json(output / "path.json", points)
            write_json(output / "controls.json", {"edges":[e.certificate() for e in edges]})
    write_json(output / "result.json", result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--query", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args(argv)
    result = run(args.inputs, args.query, args.output, args.timeout)
    print(json.dumps({k:v for k,v in result.items() if k != "oracle"}, indent=2))
    return 0 if result["strict_gate_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
