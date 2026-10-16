"""Compare V6 L1 against the frozen r14 source on identical L1-only inputs."""
import argparse
import importlib
import json
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_r13_astar_experiment_record_20260923')
sys.path.insert(0, str(REFERENCE / 'tools'))
import worker as harness

for name in list(sys.modules):
    if name.startswith('three_yd_v5'):
        del sys.modules[name]
sys.path.insert(0, '/home/robot/workspaces/3yd_v5_r14_l1_astar_20260923/src')
from three_yd_v5.topology_astar import AStarTopology as Original

for name in list(sys.modules):
    if name.startswith('three_yd_v5'):
        del sys.modules[name]
sys.path.insert(0, str(ROOT / 'src'))
Optimized = importlib.import_module('three_yd_v5.topology_astar').AStarTopology

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map-id', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--expected', help='Earlier exact-reference comparison result')
    args = ap.parse_args()
    manifest = json.loads((REFERENCE / 'manifest.json').read_text())
    entry = next(x for x in manifest['maps'] if x['map_id'] == args.map_id)
    inp = REFERENCE / entry['inputs']
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    world = harness.L1World(inp, harness.Tracker())
    with tempfile.TemporaryDirectory() as tmp:
        original = None
        expected = json.loads(Path(args.expected).read_text()) if args.expected else None
        old_s = expected['old_initialization_s'] if expected else None
        if expected is None:
            start = time.perf_counter()
            original = Original(world, inp / 'topology.json', Path(tmp) / 'old')
            old_s = time.perf_counter() - start
        start = time.perf_counter()
        new = Optimized(world, inp / 'topology.json', Path(tmp) / 'new')
        new_s = time.perf_counter() - start
        if original is not None:
            assert original.subregions.shape == new.subregions.shape
            assert (original.subregions == new.subregions).all()
            assert dict(original.contacts) == dict(new.contacts), 'parking contact geometry changed'
            assert {k: set(v) for k, v in original.adj.items()} == {k: set(v) for k, v in new.adj.items()}
        data = json.loads((inp / 'queries.json').read_text())
        records = []
        for d in data['scenarios']:
            q = harness.make_query(d)
            for graph in (original, new):
                if graph is None:
                    continue
                graph._cache.clear()
                graph.tasks.clear()
                graph.contexts.clear()
            old_ms = None
            old_tasks = None
            if original is not None:
                start = time.perf_counter()
                old_routes = original.routes(q, limit=3)
                old_ms = (time.perf_counter() - start) * 1000
                old_tasks = [original.task(x) for x in old_routes]
            start = time.perf_counter()
            new_routes = new.routes(q, limit=3)
            new_ms = (time.perf_counter() - start) * 1000
            new_tasks = [new.task(x) for x in new_routes]
            if old_tasks is not None:
                assert old_tasks == new_tasks, f'{q.query_id}: L1 output changed'
            else:
                reference = next(x for x in expected['queries'] if x['query_id'] == q.query_id)
                old_ms = reference['old_ms']
                assert harness.canonical_hash(new_tasks) == reference['output_hash'], f'{q.query_id}: L1 output changed'
            records.append({'query_id': q.query_id, 'old_ms': old_ms,
                            'v6_ms': new_ms, 'candidate_count': len(new_tasks),
                            'output_hash': harness.canonical_hash(new_tasks)})
            print(args.map_id, q.query_id, round(old_ms, 2), round(new_ms, 2), flush=True)
        report = {'map_id': args.map_id, 'old_initialization_s': old_s,
                  'v6_initialization_s': new_s, 'contacts_exact': True,
                  'output_exact': True, 'queries': records}
        (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print('INIT', args.map_id, round(old_s, 2), round(new_s, 2), flush=True)

if __name__ == '__main__':
    main()
