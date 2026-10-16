"""Small-map independent seam, topology equivalence, and cache regressions."""
from collections import defaultdict
from copy import deepcopy
from dataclasses import replace
import json
import math

import numpy as np
from PIL import Image
import pytest

from arena_evaluation.planner_benchmark.map_utils import HospitalMap
from arena_evaluation.tiled_topology_r3 import (
    TileConfig, TiledTopology, compress_skeleton, file_hash,
)
from arena_evaluation.topology import build_topology


FOOTPRINT = ((.255, .215), (.255, -.215), (-.255, -.215), (-.255, .215))


def test_query_artifact_fast_copy_does_not_alias_persistent_polyline(tmp_path):
    m=_map(tmp_path,'channel');tiled=TiledTopology(m,FOOTPRINT,tmp_path/'cache',_config())
    tiled.build_coarse();selected=tiled.candidate_tiles(m.cell_to_world((64,24)),m.cell_to_world((64,168)))
    first=tiled.artifact(selected);reference=deepcopy(first.graph.edges)
    first.graph.edges[0].polyline[0][0]+=1000
    first.graph.nodes[0].x+=1000
    second=tiled.artifact(selected)
    assert second.graph.edges==reference
    assert second.graph.nodes[0].x!=first.graph.nodes[0].x


def _map(tmp_path, design='channel'):
    occupancy = np.full((128, 192), 100, np.int8)
    if design == 'channel':
        occupancy[43:86, 4:-4] = 0
    elif design == 'parallel':
        occupancy[17:49, 4:-4] = 0
        occupancy[79:111, 4:-4] = 0
    elif design == 'wall_on_seam':
        occupancy[4:-4, 4:-4] = 0
        occupancy[:, 62:66] = 100
    elif design == 'dogleg':
        occupancy[16:50, 4:116] = 0
        occupancy[16:112, 80:116] = 0
        occupancy[78:112, 80:-4] = 0
    elif design == 'open':
        occupancy[4:-4, 4:-4] = 0
    else:
        raise ValueError(design)
    root = tmp_path / design
    root.mkdir(parents=True, exist_ok=True)
    pgm = root / 'map.pgm'
    Image.fromarray(np.where(occupancy == 0, 254, 0).astype(np.uint8)).save(pgm)
    yaml = root / 'map.yaml'
    yaml.write_text('image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return HospitalMap.load(yaml)


def _config(**kwargs):
    return TileConfig(tile_cells=64, halo_cells=16, memory_tiles=2, **kwargs)


def _coarse_components(tiled):
    labels = {}
    component = 0
    for node in sorted(tiled.coarse):
        if node in labels:
            continue
        component += 1
        queue = [node]
        labels[node] = component
        for current in queue:
            for neighbor, _ in tiled.coarse[current]:
                if neighbor not in labels:
                    labels[neighbor] = component
                    queue.append(neighbor)
    return labels


def _assert_same_partition(a, b):
    forward, reverse = defaultdict(set), defaultdict(set)
    for x, y in zip(a, b):
        forward[int(x)].add(int(y))
        reverse[int(y)].add(int(x))
    assert all(len(values) == 1 for values in forward.values())
    assert all(len(values) == 1 for values in reverse.values())


def test_true_channel_across_tile_seams_is_connected_and_portals_are_safe(tmp_path):
    m = _map(tmp_path)
    tiled = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    certificate = tiled.build_coarse()
    assert certificate['validated'] is True
    assert tiled.portals
    start, goal = m.cell_to_world((64, 24)), m.cell_to_world((64, 168))
    selected = tiled.candidate_tiles(start, goal)
    assert selected
    artifact = tiled.artifact(selected)
    assert artifact.graph.components == 1
    assert len(artifact.graph.edges) > 1
    for portal in tiled.portals:
        a, b = portal['cells']
        assert max(abs(a[0] - b[0]), abs(a[1] - b[1])) == 1
        world_a, world_b = m.cell_to_world(a), m.cell_to_world(b)
        yaw = math.atan2(world_b[1] - world_a[1], world_b[0] - world_a[0])
        for world in (world_a, world_b):
            assert not m.footprint_collision((*world, yaw), FOOTPRINT, unknown_is_collision=True)


@pytest.mark.parametrize('design', ['parallel', 'wall_on_seam'])
def test_adjacent_but_disconnected_seam_regions_are_not_falsely_joined(tmp_path, design):
    m = _map(tmp_path, design)
    tiled = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    tiled.build_coarse()
    artifact = tiled.artifact(tiled.tiles)
    assert artifact.graph.components == 2
    assert len(set(_coarse_components(tiled).values())) == 2
    if design == 'parallel':
        start, goal = m.cell_to_world((32, 32)), m.cell_to_world((96, 160))
    else:
        start, goal = m.cell_to_world((64, 32)), m.cell_to_world((64, 160))
        assert not any(set(map(tuple, p['tiles'])) == {(0, 0), (0, 1)} for p in tiled.portals)
    # Discovery may include multiple conservative components. It must never
    # turn that candidate set into a certified cross-obstacle connection.
    selected = tiled.candidate_tiles(start, goal)
    discovered = tiled.artifact(selected)
    assert discovered.graph.components == 2
    from arena_evaluation.reachable_endpoint_r3 import ReachableEndpointSelector
    from arena_evaluation.planner_benchmark.models import Query
    selector = ReachableEndpointSelector(discovered, FOOTPRINT)
    result = selector(discovered, Query('separated', (*start, 0.), (*goal, 0.), seed=0))
    assert result[2] is None
    assert selector.last_certificate['failure']
    assert not selector.last_certificate['selected_hashes']


@pytest.mark.parametrize('design', ['channel', 'parallel', 'wall_on_seam', 'dogleg'])
def test_small_tiled_and_original_full_topology_have_equivalent_connectivity(tmp_path, design):
    m = _map(tmp_path, design)
    original = build_topology(m, FOOTPRINT)
    tiled = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    tiled.build_coarse()
    artifact = tiled.artifact(tiled.tiles)
    assembled = np.zeros_like(original.free_mask)
    merged_components = np.zeros_like(original.free_components)
    coarse_labels = _coarse_components(tiled)
    for tile in tiled.tiles:
        r0, r1, c0, c1 = tiled.bounds(tile)
        data = tiled.tile(tile)
        assembled[r0:r1, c0:c1] = data['free']
        local = merged_components[r0:r1, c0:c1]
        for label in range(1, int(data['labels'].max()) + 1):
            local[data['labels'] == label] = coarse_labels[(*tile, label)]
    assert np.array_equal(assembled, original.free_mask)
    _assert_same_partition(original.free_components[assembled], merged_components[assembled])
    old_by_node, new_by_node = [], []
    for node in artifact.graph.nodes:
        old_component = original.free_components[node.pixel_y, node.pixel_x]
        assert old_component > 0
        old_by_node.append(old_component)
        new_by_node.append(node.component_id)
    _assert_same_partition(old_by_node, new_by_node)
    assert set(old_by_node) == set(np.unique(original.free_components)) - {0}
    assert artifact.graph.components == original.graph.components


def test_degree_two_chain_is_compressed_and_forced_portal_is_retained(tmp_path):
    m = _map(tmp_path, 'open')
    skeleton = np.zeros((7, 13), bool)
    skeleton[3, 1:12] = True
    distance = np.ones_like(skeleton, np.float32)
    graph, ids = compress_skeleton(skeleton, distance, m, offset=(20, 20))
    assert len(graph.nodes) == 2 and len(graph.edges) == 1
    assert graph.edges[0].pixel_count == 11
    assert graph.edges[0].length_m == pytest.approx(.5)
    assert len(graph.edges[0].polyline) == 11
    split, ids = compress_skeleton(skeleton, distance, m, offset=(20, 20), forced=((3, 6),))
    assert len(split.nodes) == 3 and len(split.edges) == 2
    assert (3, 6) in ids
    assert sum(edge.length_m for edge in split.edges) == pytest.approx(.5)


def test_degree_two_loop_and_parallel_edges_are_not_deleted(tmp_path):
    m = _map(tmp_path, 'open')
    skeleton = np.zeros((5, 5), bool)
    for cell in ((1, 2), (2, 1), (3, 2), (2, 3)):
        skeleton[cell] = True
    distance = np.ones_like(skeleton, np.float32)
    graph, _ = compress_skeleton(skeleton, distance, m, offset=(20, 20))
    assert len(graph.nodes) == 1 and len(graph.edges) == 1
    assert graph.edges[0].source == graph.edges[0].target
    assert graph.edges[0].length_m == pytest.approx(4 * math.sqrt(2) * .05)
    parallel, _ = compress_skeleton(skeleton, distance, m, offset=(20, 20), forced=((1, 2), (3, 2)))
    assert len(parallel.nodes) == 2 and len(parallel.edges) == 2
    assert {frozenset((edge.source, edge.target)) for edge in parallel.edges} == {frozenset((0, 1))}


def test_seam_and_graph_hashes_are_deterministic_across_independent_builds(tmp_path):
    m = _map(tmp_path, 'dogleg')
    a = TiledTopology(m, FOOTPRINT, tmp_path / 'cache_a', _config())
    b = TiledTopology(m, FOOTPRINT, tmp_path / 'cache_b', _config())
    sa, sb = a.build_all(), b.build_all()
    assert sa['topology_hash'] == sb['topology_hash']
    assert sa['seam_hash'] == sb['seam_hash']
    assert a.portals == b.portals
    assert [p['id'] for p in a.portals] == list(range(len(a.portals)))
    assert sa['tile_completed_count'] == sa['tile_count'] == len(a.tiles)
    assert sa['seam_valid'] is True
    assert sa['topology_cache_bytes'] > 0


def test_corrupt_tile_data_is_invalidated_locally_and_other_tiles_still_hit(tmp_path):
    m = _map(tmp_path, 'open')
    a = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    original = {tile: a.tile(tile)['free'].copy() for tile in a.tiles}
    damaged, untouched = a.tiles[0], a.tiles[-1]
    p = a._path(damaged).with_suffix('.npz')
    with p.open('ab') as stream:
        stream.write(b'injected_corruption')
    untouched_path = a._path(untouched).with_suffix('.npz')
    original_hash = file_hash(untouched_path)
    b = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    assert np.array_equal(b.tile(damaged)['free'], original[damaged])
    assert np.array_equal(b.tile(untouched)['free'], original[untouched])
    assert b.stats['tile_cache_miss_count'] == 1
    assert b.stats['tile_cache_hit_count'] == 1
    assert file_hash(untouched_path) == original_hash


def test_corrupt_graph_hash_is_rebuilt_from_verified_tile(tmp_path):
    m = _map(tmp_path)
    a = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    a.build_coarse()
    target = (0, 1)
    a.refine(target)
    path = a._path(target).with_suffix('.graph.json')
    correct = json.loads(path.read_text())
    damaged = deepcopy(correct)
    damaged['graph']['nodes'][0]['x'] += 10.
    path.write_text(json.dumps(damaged))
    b = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    b.build_coarse()
    b.refine(target)
    assert json.loads(path.read_text())['graph'] == correct['graph']


@pytest.mark.parametrize('kind', ['tile_metadata', 'graph_metadata'])
def test_truncated_cache_metadata_invalidates_only_its_local_artifact(tmp_path, kind):
    m = _map(tmp_path)
    a = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    a.build_coarse()
    target = (0, 1)
    expected_graph, _ = a.refine(target)
    suffix = '.json' if kind == 'tile_metadata' else '.graph.json'
    path = a._path(target).with_suffix(suffix)
    path.write_text('{"binding":')
    b = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    b.build_coarse()
    graph, _ = b.refine(target)
    assert len(graph.nodes) == len(expected_graph.nodes)
    assert len(graph.edges) == len(expected_graph.edges)
    assert json.loads(path.read_text())['binding'] == b.key


def test_corrupt_persisted_skeleton_is_repaired_before_cache_is_trusted(tmp_path):
    m = _map(tmp_path)
    a = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    a.build_coarse()
    target = (0, 1)
    a.refine(target)
    path = a._path(target).with_suffix('.skeleton.npz')
    with np.load(path, allow_pickle=False) as data:
        expected = data['skeleton'].copy()
    path.write_bytes(b'truncated-skeleton')
    b = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    b.build_coarse()
    b.refine(target)
    with np.load(path, allow_pickle=False) as data:
        assert np.array_equal(data['skeleton'], expected)


def test_tile_cache_key_binds_map_resolution_origin_footprint_and_configuration(tmp_path):
    m = _map(tmp_path)
    cfg = _config()
    base = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', cfg)
    configurations = [replace(cfg, tile_cells=96), replace(cfg, halo_cells=20),
                      replace(cfg, padding_m=.1), replace(cfg, safety_margin_m=.1),
                      replace(cfg, algorithm='next-algorithm')]
    assert all(TiledTopology(m, FOOTPRINT, tmp_path / 'cache', c).key != base.key for c in configurations)
    changed = deepcopy(m)
    changed.origin = (.05, 0., 0.)
    assert TiledTopology(changed, FOOTPRINT, tmp_path / 'cache', cfg).key != base.key
    changed = deepcopy(m)
    changed.resolution = .1
    assert TiledTopology(changed, FOOTPRINT, tmp_path / 'cache', cfg).key != base.key
    larger = tuple((x * 1.1, y * 1.1) for x, y in FOOTPRINT)
    assert TiledTopology(m, larger, tmp_path / 'cache', cfg).key != base.key
    m.yaml_path.write_text(m.yaml_path.read_text() + '# new-input-byte\n')
    assert TiledTopology(m, FOOTPRINT, tmp_path / 'cache', cfg).key != base.key


def test_tile_array_and_graph_memory_follow_lru_bounds(tmp_path):
    m = _map(tmp_path, 'open')
    tiled = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    tiled.build_coarse()
    for tile in tiled.tiles:
        tiled.tile(tile)
        tiled.refine(tile)
        assert len(tiled.lru) <= tiled.config.memory_tiles
        assert len(tiled.graph_lru) <= tiled.config.memory_tiles
        assert tile in tiled.lru and tile in tiled.graph_lru
        assert all(v['free'].shape[0] <= tiled.config.tile_cells and
                   v['free'].shape[1] <= tiled.config.tile_cells for v in tiled.lru.values())


def test_halo_smaller_than_full_footprint_and_boundary_support_is_rejected(tmp_path):
    m = _map(tmp_path)
    with pytest.raises(ValueError, match='halo'):
        TiledTopology(m, FOOTPRINT, tmp_path / 'cache', replace(_config(), halo_cells=4))
    with pytest.raises(ValueError):
        TiledTopology(m, FOOTPRINT, tmp_path / 'cache', replace(_config(), memory_tiles=0))


@pytest.mark.parametrize('damage', ['duplicate', 'nonadjacent', 'wrong_component'])
def test_seam_certificate_rejects_false_or_duplicate_connections(tmp_path, damage):
    m = _map(tmp_path)
    tiled = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    tiled.build_coarse()
    if damage == 'duplicate':
        tiled.portals.append(deepcopy(tiled.portals[0]))
    elif damage == 'nonadjacent':
        tiled.portals[0]['cells'][0][0] += 3
    else:
        tiled.portals[0]['components'][0][2] += 99
    with pytest.raises(ValueError):
        tiled.validate_seams()


@pytest.mark.parametrize('damage', ['deleted', 'reordered'])
def test_seam_certificate_rejects_missing_or_misordered_portals(tmp_path, damage):
    m = _map(tmp_path)
    tiled = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    tiled.build_coarse()
    assert len(tiled.portals) > 1
    if damage == 'deleted':
        tiled.portals.clear()
    else:
        tiled.portals.reverse()
    with pytest.raises(ValueError):
        tiled.validate_seams()


def test_rebuilding_coarse_graph_does_not_accumulate_duplicate_portals(tmp_path):
    m = _map(tmp_path)
    tiled = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    first = deepcopy(tiled.build_coarse())
    second = tiled.build_coarse()
    assert second['validated'] is True
    assert first['hash'] == second['hash']
    assert first['portals'] == second['portals']


def test_bounded_pgm_loader_matches_frozen_occupancy(tmp_path):
    from arena_evaluation.tiled_topology_r3 import load_map_bounded
    original = _map(tmp_path, 'dogleg')
    loaded = load_map_bounded(original.yaml_path, tmp_path / 'streamed')
    assert isinstance(loaded.occupancy, np.memmap)
    assert np.array_equal(loaded.occupancy, original.occupancy)
    assert loaded.origin == original.origin
    assert loaded.resolution == original.resolution
    tiled = TiledTopology(loaded, FOOTPRINT, tmp_path / 'tiles_streamed', _config())
    reference = TiledTopology(original, FOOTPRINT, tmp_path / 'tiles_regular', _config())
    for tile in tiled.tiles:
        assert np.array_equal(tiled.tile(tile)['free'], reference.tile(tile)['free'])


def test_coarse_discovery_keeps_endpoint_components_across_pose_reachable_neck(tmp_path):
    from arena_evaluation.reachable_endpoint_r3 import ReachableEndpointSelector
    from arena_evaluation.planner_benchmark.models import Query
    m = _map(tmp_path, 'open')
    m.occupancy[:, 80:84] = 100
    m.occupancy[57:71, 80:84] = 0  # 0.70m door: rectangle fits, all-heading envelope does not
    # Bind the synthetic occupancy change to persisted map input bytes.
    from PIL import Image
    Image.fromarray(np.where(m.occupancy == 0, 254, 0).astype(np.uint8)).save(m.image_path)
    m = HospitalMap.load(m.yaml_path)
    tiled = TiledTopology(m, FOOTPRINT, tmp_path / 'cache', _config())
    tiled.build_coarse()
    start = (*m.cell_to_world((64, 60)), 0.)
    goal = (*m.cell_to_world((64, 160)), 0.)
    selected = tiled.candidate_tiles(start, goal)
    assert selected, 'nearest conservative component must not reject reachable endpoint alternatives'
    artifact = tiled.artifact(selected)
    selector = ReachableEndpointSelector(artifact, FOOTPRINT)
    result = selector(artifact, Query('neck', start, goal, seed=0))
    assert result[2] is not None
    assert selector.last_certificate['selected_hashes']


@pytest.mark.parametrize('filename', ['occupancy.bin', 'distance.bin', 'complete.json'])
def test_bounded_map_cache_corruption_is_verified_and_rebuilt(tmp_path, filename):
    from arena_evaluation.tiled_topology_r3 import load_map_bounded
    original = _map(tmp_path)
    cache = tmp_path / 'bounded'
    loaded = load_map_bounded(original.yaml_path, cache)
    victim = next(cache.rglob(filename))
    before = file_hash(victim)
    with victim.open('r+b') as stream:
        value = stream.read(1)
        stream.seek(0);stream.write(bytes([value[0] ^ 1]))
    assert file_hash(victim) != before
    repaired = load_map_bounded(original.yaml_path, cache)
    assert np.array_equal(repaired.occupancy, original.occupancy)
    assert np.count_nonzero(repaired.distance_m) == 0
    assert file_hash(victim) == before


def test_obstacle_free_halo_distance_is_finite_conservative_lower_bound(tmp_path):
    m = _map(tmp_path, 'open')
    tiled = TiledTopology(m, FOOTPRINT, tmp_path / 'cache',
                          replace(_config(), tile_cells=16))
    t = (3, 5)
    r0,r1,c0,c1 = tiled.bounds(t)
    distance = tiled.tile(t)['distance']
    assert np.all(np.isfinite(distance))
    assert np.max(distance) < 10.
    assert np.all(distance <= m.distance_m[r0:r1,c0:c1] + 1e-6)


@pytest.mark.parametrize('operation',['candidate_tiles','artifact'])
def test_expired_query_deadline_rejects_before_loading_or_refining_tiles(tmp_path,monkeypatch,operation):
    import arena_evaluation.tiled_topology_r3 as module
    from types import SimpleNamespace
    m=_map(tmp_path);tiled=TiledTopology(m,FOOTPRINT,tmp_path/'cache',_config())
    tiled.build_coarse()
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:10.))
    def forbidden(*args,**kwargs):raise AssertionError('expired query must not load another tile')
    monkeypatch.setattr(tiled,'tile',forbidden);monkeypatch.setattr(tiled,'refine',forbidden)
    with pytest.raises(module.TopologyDeadlineExceeded):
        if operation=='candidate_tiles':tiled.candidate_tiles((1.,3.),(8.,3.),deadline=10.)
        else:tiled.artifact(tiled.tiles,deadline=10.)


def test_artifact_deadline_stops_between_tiles_preserving_complete_local_cache(tmp_path,monkeypatch):
    import arena_evaluation.tiled_topology_r3 as module
    from types import SimpleNamespace
    m=_map(tmp_path);tiled=TiledTopology(m,FOOTPRINT,tmp_path/'cache',_config())
    tiled.build_coarse();first=tiled.tiles[0];prepared=tiled.refine(first)
    clock=SimpleNamespace(now=10.);calls=[]
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:clock.now))
    def refine(tile,*,deadline=None):
        calls.append(tile);clock.now=12.;return prepared
    monkeypatch.setattr(tiled,'refine',refine)
    with pytest.raises(module.TopologyDeadlineExceeded):tiled.artifact(tiled.tiles,deadline=11.)
    assert calls==[first]
    assert first in tiled.refined_tiles
    assert tiled._path(first).with_suffix('.graph.json').is_file()
