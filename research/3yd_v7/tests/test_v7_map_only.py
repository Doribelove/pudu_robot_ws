import json
from pathlib import Path

import numpy as np
import pytest
from PIL import Image
from scipy.sparse.csgraph import dijkstra

from arena_evaluation.planner_benchmark.map_utils import HospitalMap
from arena_evaluation.planner_benchmark.models import Query
from three_yd_v5.free_space_topology import DeferredOccupancyMap, FreeSpaceTopology, load_map_only_world


def write_map(tmp_path, image):
    Image.fromarray(image.astype(np.uint8)).save(tmp_path / 'map.pgm')
    (tmp_path / 'map.yaml').write_text(
        'image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n')
    return tmp_path / 'map.yaml'


def test_map_only_pgm_interpretation_matches_existing_loader(tmp_path):
    image = np.full((40, 50), 254, np.uint8)
    image[0, :] = 0
    image[10:14, 20:25] = 205
    map_yaml = write_map(tmp_path, image)
    original = HospitalMap.load(map_yaml)
    adapter = DeferredOccupancyMap(map_yaml)
    assert np.array_equal(adapter.occupancy, original.occupancy)
    assert adapter.sha256 == original.sha256


def test_map_only_route_is_free_and_has_no_traffic_rule(tmp_path):
    image = np.full((120, 120), 254, np.uint8)
    image[:, 59:62] = 0
    image[50:71, 59:62] = 254
    map_yaml = write_map(tmp_path, image)
    world = load_map_only_world(map_yaml, tmp_path / 'unused')
    topology = FreeSpaceTopology(world, tmp_path / 'graph')
    query = Query('Q01', (1.025, 1.025, 0.), (5.025, 1.025, 0.))
    route = topology.routes(query)[0]
    plan = topology.channel_plan(route)
    line = np.asarray(plan['right_reference_polyline'])
    assert plan['mode'] == 'free_space'
    assert plan['right_side_rule'] is None
    assert len(line) >= 2
    assert all(topology._clear_line(a, b) for a, b in zip(line, line[1:]))


def test_map_only_does_not_claim_disconnected_goal(tmp_path):
    image = np.full((120, 120), 254, np.uint8)
    image[:, 59:62] = 0
    map_yaml = write_map(tmp_path, image)
    world = load_map_only_world(map_yaml, tmp_path / 'unused')
    topology = FreeSpaceTopology(world, tmp_path / 'graph')
    query = Query('Q01', (1.025, 1.025, 0.), (5.025, 1.025, 0.))
    with pytest.raises(RuntimeError, match='NO_FREE_SPACE_ROUTE'):
        topology.routes(query)


def test_map_only_requires_005_resolution(tmp_path):
    image = np.full((20, 20), 254, np.uint8)
    map_yaml = write_map(tmp_path, image)
    map_yaml.write_text(map_yaml.read_text().replace('resolution: 0.05', 'resolution: 0.10'))
    with pytest.raises(ValueError, match='0_05_M_RESOLUTION'):
        load_map_only_world(map_yaml, tmp_path / 'unused')


def test_compiled_coarse_graph_does_not_cut_diagonal_obstacle_corner():
    free = np.asarray([[True, False], [False, True]])
    graph = FreeSpaceTopology._compile_graph(free)
    assert not np.isfinite(dijkstra(graph, indices=0)[3])
