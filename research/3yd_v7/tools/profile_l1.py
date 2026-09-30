"""Profile one fixed L1 query with the isolated map-only harness."""
import cProfile
import json
import pstats
import sys
import tempfile
from pathlib import Path

from compare_l1 import REFERENCE, Optimized, harness

map_id = sys.argv[1]
query_id = sys.argv[2]
entry = next(x for x in json.loads((REFERENCE / 'manifest.json').read_text())['maps']
             if x['map_id'] == map_id)
inp = REFERENCE / entry['inputs']
world = harness.L1World(inp, harness.Tracker())
data = json.loads((inp / 'queries.json').read_text())
q = harness.make_query(next(x for x in data['scenarios'] if x['id'] == query_id))
with tempfile.TemporaryDirectory() as tmp:
    graph = Optimized(world, inp / 'topology.json', tmp)
    profile = cProfile.Profile()
    profile.enable()
    routes = graph.routes(q, limit=3)
    profile.disable()
    print('candidates', len(routes), 'stats', graph._probe_stats)
    pstats.Stats(profile).strip_dirs().sort_stats('cumtime').print_stats(35)
