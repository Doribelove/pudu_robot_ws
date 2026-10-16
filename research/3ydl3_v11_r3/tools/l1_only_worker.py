"""One fresh process for the isolated Map-C L1 search comparison."""

import argparse
import gc
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_r13_astar_experiment_record_20260923')
V6_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import worker as baseline_harness
for name in list(sys.modules):
    if name.startswith('three_yd_v5'):
        del sys.modules[name]
sys.path.insert(0, str(V6_ROOT / 'src'))
from three_yd_v5.topology_astar import AStarTopology


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--round", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--queries", nargs="+", default=[f"Q{i:02}" for i in range(1, 11)])
    parser.add_argument("--first-budget", type=int, default=200000)
    parser.add_argument("--alternate-budget", type=int, default=200000)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    os.environ["THREEYD_L1_FIRST_BUDGET"] = str(args.first_budget)
    os.environ["THREEYD_L1_ALTERNATE_BUDGET"] = str(args.alternate_budget)
    os.environ["THREEYD_L1_LEGACY_COMPAT"] = "1"

    manifest = json.loads((ROOT / "manifest.json").read_text())
    entry = next(m for m in manifest["maps"] if m["map_id"] == args.map_id)
    for name, digest in entry["input_hashes"].items():
        assert baseline_harness.sha(ROOT / entry["inputs"] / name) == digest
    available = os.sched_getaffinity(0)
    if manifest["cpu"] in available:
        os.sched_setaffinity(0, {manifest["cpu"]})

    tracker = baseline_harness.Tracker()
    initial_before = baseline_harness.resource_state()
    begin = time.perf_counter_ns()
    cpu_begin = time.process_time_ns()
    source = ROOT / entry["inputs"]
    data = json.loads((source / "queries.json").read_text())
    world = baseline_harness.L1World(source, tracker)
    graph = AStarTopology(world, source / "topology.json", args.output / "graph_report")
    initial_after = baseline_harness.resource_state()
    baseline_harness.save(args.output / "initialization.json", {
        "wall_ms": (time.perf_counter_ns() - begin) / 1e6,
        "cpu_ms": (time.process_time_ns() - cpu_begin) / 1e6,
        "before": initial_before,
        "after": initial_after,
        "l1_adjacency_pairs": graph.edges_count,
        "L2_L3_initialized": False,
        "occupancy_loaded": False,
    })
    print("INIT", args.round, round((time.perf_counter_ns() - begin) / 1e9, 3), "s", flush=True)

    rows = []
    for qid in args.queries:
        descriptor = next(item for item in data["scenarios"] if item["id"] == qid)
        baseline_harness.clear(graph)
        gc.collect()
        query = baseline_harness.make_query(descriptor)
        before = baseline_harness.resource_state()
        t0, c0 = time.perf_counter_ns(), time.process_time_ns()
        routes, outputs, error = [], [], None
        try:
            routes = graph.routes(query, limit=manifest["candidate_limit"])
            outputs = [graph.channel_plan(route) for route in routes]
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
        wall_ms = (time.perf_counter_ns() - t0) / 1e6
        cpu_ms = (time.process_time_ns() - c0) / 1e6
        after = baseline_harness.resource_state()
        failures = ([error] if error else []) + baseline_harness.audit_outputs(graph, query, outputs)
        stats = getattr(graph, "_probe_stats", {})
        output = {
            "version": "3YD-V7-r1-L1", "map_id": entry["map_id"],
            "round": args.round, "query": descriptor,
            "input_hashes": entry["input_hashes"],
            "candidates": outputs,
            "endpoint_access": [graph.contexts[r.signature]["endpoint_access"] for r in routes],
            "validation_failures": failures,
        }
        baseline_harness.save(args.output / f"{qid}_output.json", output)
        row = {
            "round": args.round, "query_id": qid,
            "distance_m": descriptor["grid_precheck_length_m"],
            "wall_ms": wall_ms, "cpu_ms": cpu_ms,
            "success": not failures, "failures": failures,
            "candidate_count": len(outputs), "search": stats,
            "candidates": [{
                "region_sequence": task["region_sequence"],
                "polyline_points": len(task["right_reference_polyline"]),
                "search_expansions_at_candidate": route.diagnostics["l1_expanded"],
                "search_cost_m": route.diagnostics["l1_path_cost_m"],
                "output_sha256": baseline_harness.canonical_hash(task),
            } for route, task in zip(routes, outputs)],
            "resources_before": before, "resources_after": after,
            "rss_change_bytes": after["rss_bytes"] - before["rss_bytes"],
            "output_sha256": baseline_harness.canonical_hash(outputs),
            "query_cache_hit": False,
            "formal_navigation_success": None,
        }
        baseline_harness.save(args.output / f"{qid}_measurement.json", row)
        rows.append(row)
        print(qid, "OK" if not failures else failures,
              f"{wall_ms:.2f} ms", len(outputs), "routes",
              stats.get("total_expansions", "error"), "expanded", flush=True)
    assert not any(name.startswith(("arena_3d_v1.l2_", "three_yd_v5.controller",
                                    "three_yd_v5.recovery", "arena_3d_v1.pipeline"))
                   for name in sys.modules)
    baseline_harness.save(args.output / "batch.json", {
        "version": "3YD-V7-r1-L1", "map_id": entry["map_id"],
        "optimization_flags": {k: os.environ.get(k, '1') for k in
            ('THREEYD_V7_GOAL_STOP', 'THREEYD_V7_COMPACT_LEGACY')},
        "round": args.round, "first_budget": args.first_budget,
        "alternate_budget": args.alternate_budget,
        "transition_table": os.environ.get("THREEYD_V6_TRANSITION_TABLE") == "1",
        "candidate_limit": manifest["candidate_limit"],
        "queries": rows, "query_cache_hits": 0,
        "L2_L3_initialized": False, "ROS_initialized": False,
        "occupancy_loaded": False,
        "final_resources": baseline_harness.resource_state(),
    })


if __name__ == "__main__":
    main()
