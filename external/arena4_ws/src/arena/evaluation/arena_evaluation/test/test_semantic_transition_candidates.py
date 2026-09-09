import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from arena_evaluation.semantic_transition_optimize import grid_routes


def test_map_derived_routes_do_not_need_query_identity_or_historical_paths():
    h, w = 80, 50
    fake_map = SimpleNamespace(full_origin=(10., -10.), full_height=h, row0=0, col0=0,
                              resolution=.05, distance_m=np.full((h,w), 1.))
    fake_map.world_to_cell = lambda x,y: (h-1-int((y+10.)/.05), int((x-10.)/.05))
    grids = {'allowed': np.ones((h,w),bool), 'hard': np.zeros((h,w),bool),
             'labels': np.ones((h,w),int), 'correct': np.ones((h,w),bool),
             'error': np.full((h,w),.1)}
    world = SimpleNamespace(master=np.zeros((h,w),np.uint8), grids=grids, selected=[1], map=fake_map,
                            start=(10.5,-9.5,0.), goal=(12.,-6.5,1.), bound_length=40.)
    routes, info = grid_routes(world)
    assert routes and info['map_resolution_m'] == .05
    assert info['route_seed_count'] <= 6
    assert all(np.all(np.isfinite(route)) for route in routes)


def test_fresh_candidates_do_not_embed_targeted_endpoint_coordinates():
    package = Path(__file__).parents[1]/'arena_evaluation'
    for name in ('semantic_transition_optimize.py','semantic_transition_roadmap.py','semantic_transition_hybrid_probe.py'):
        source = (package/name).read_text()
        ast.parse(source)
        assert 'WITNESSES' not in source
        assert 'next_arch_r1_optimize_dense_all_positive' not in source
        assert '-25.750998' not in source
