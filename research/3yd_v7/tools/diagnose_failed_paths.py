"""Read-only semantic traversal audit of saved r6 rejected trajectories."""
import json
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, '/home/robot/文档/ChatGPT/dddd-任务/output/3yd_fixed10_20260922')
from fixed10_binding import resolve
from arena_3d_v1.semantic_world import SemanticWorld, densify
from arena_evaluation.planner_benchmark.models import Query
from three_yd_v5.topology import V5Topology
from three_yd_v5.controller import V5Traffic

ROOT = Path('/home/robot/workspaces/3yd_v6_r6_20260924/results/formal_r6')
CASES = {
    '3yd_synth_multiloop_a_v1': ['Q02'],
    '3yd_synth_twinwing_b_v1': ['Q02', 'Q03', 'Q10'],
    '3yd_synth_trizone_c_v2': ['Q01', 'Q02'],
    'pudu_wanda_3f': ['Q09'],
}

def intervals(p, labels):
    arcs = np.r_[0., np.cumsum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1))]
    starts = np.flatnonzero(np.r_[True, labels[1:] != labels[:-1]])
    ends = np.r_[starts[1:], len(labels) - 1]
    return [(int(labels[a]), round(float(arcs[b] - arcs[a]), 3),
             round(float(arcs[a]), 2), round(float(arcs[b]), 2))
            for a, b in zip(starts, ends)]

def main():
    for map_id, names in CASES.items():
        inp, _, _ = resolve(map_id)
        cache = Path('/tmp/3yd_r7_diagnose') / map_id
        (cache / 'world').mkdir(parents=True, exist_ok=True)
        (cache / 'graph').mkdir(exist_ok=True)
        world = SemanticWorld(inp / 'map.yaml', inp / 'semantic.json', cache_root=cache / 'world')
        graph = V5Topology(world, inp / 'topology.json', cache / 'graph')
        queries = {q['id']: q for q in json.loads((inp / 'queries.json').read_text())['scenarios']}
        for name in names:
            case = ROOT / map_id / 'cases' / f'{name}_1'
            if not (case / 'rejected_candidate.npy').exists():
                print(map_id, name, 'NO_CANDIDATE')
                continue
            item = queries[name]
            query = Query(name, tuple(item['start'][k] for k in ('x', 'y', 'yaw_rad')),
                          tuple(item['goal'][k] for k in ('x', 'y', 'yaw_rad')), seed=20260916)
            route = graph.routes(query, limit=3)[-1]
            audit = V5Traffic(world, graph, route)
            p = np.load(case / 'rejected_candidate.npy')
            dense = densify(p, ds=.025)
            rr, cc = world.cells(dense)
            labels = graph.subregions[rr, cc]
            lane_ids = {rid for rid, item in graph.regions.items() if item['kind'] == 'lane'}
            ordered = [(rid, length, lo, hi) for rid, length, lo, hi in intervals(dense, labels)
                       if rid in lane_ids and length > .02]
            unexpected = [(rid, length, lo, hi) for rid, length, lo, hi in ordered
                          if rid not in {x[0] for x in audit.roads}]
            print(json.dumps({'map': map_id, 'query': name,
                              'expected': [x[0] for x in audit.roads],
                              'visited': audit.audit(p).get('visited_roads'),
                              'unexpected_intervals': unexpected,
                              'lane_intervals': ordered}, ensure_ascii=False))

if __name__ == '__main__':
    main()
