"""Independent geometric regressions for query-local r3 endpoint edges.

These tests use the real HospitalMap footprint predicate at 0.05 m/cell.
Bounded connector exhaustion is deliberately not called global infeasibility.
"""
from copy import deepcopy
from dataclasses import asdict, replace
import hashlib
import math
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt

from arena_evaluation.planner_benchmark.map_utils import HospitalMap
from arena_evaluation.reachable_endpoint_r3 import (
    ConnectorConfig, LocalConnector,
    ReachableEndpointSelector, digest, wrap,
)
from arena_evaluation.topology import TopologyArtifact, TopologyEdge, TopologyGraph, TopologyNode


FOOTPRINT = ((.255, .215), (.255, -.215), (-.255, -.215), (-.255, .215))


def _map(*, width_m=10., height_m=8., corridor=None, wall_x=None, unknown=False):
    resolution = .05
    shape = (round(height_m / resolution), round(width_m / resolution))
    occupancy = np.zeros(shape, dtype=np.int8)
    occupancy[[0, -1], :] = 100
    occupancy[:, [0, -1]] = 100
    if corridor is not None:
        low, high = corridor
        ys = (shape[0] - np.arange(shape[0]) - .5) * resolution
        occupancy[(ys < low) | (ys > high), :] = 100
    if wall_x is not None:
        occupancy[:, round(wall_x / resolution)] = -1 if unknown else 100
    return HospitalMap(
        Path('/synthetic/map.yaml'), Path('/synthetic/map.pgm'), resolution,
        (0., 0., 0.), shape[1], shape[0], occupancy,
        distance_transform_edt(occupancy == 0) * resolution,
    )


def _topology(hospital_map, segments):
    """Segments are (component_id, [(x,y), ...]); each is a compressed edge."""
    nodes, edges = [], []
    for eid, (component, points) in enumerate(segments):
        for nid, point in ((2 * eid, points[0]), (2 * eid + 1, points[-1])):
            row, col = hospital_map.world_to_cell(*point)
            clearance = hospital_map.clearance(*point)
            nodes.append(TopologyNode(nid, *point, col, row, 1, clearance, 2 * clearance, component))
        length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        clearance = min(hospital_map.clearance(*p) for p in points)
        edges.append(TopologyEdge(eid, 2 * eid, 2 * eid + 1, length, clearance,
                                  clearance, 2 * clearance, len(points), [list(p) for p in points]))
    mask = hospital_map.occupancy == 0
    return TopologyArtifact(hospital_map, mask, mask.copy(), hospital_map.distance_m,
                            np.zeros(mask.shape, np.int32), TopologyGraph(nodes, edges),
                            {'map_sha256': hashlib.sha256(hospital_map.occupancy.tobytes()).hexdigest()})


def _query(start=(2., 4., 0.), goal=(7., 4., 0.), query_id='synthetic'):
    return SimpleNamespace(start=tuple(start), goal=tuple(goal), query_id=query_id)


def _config(**kwargs):
    # Exact Dubins words suffice for positive fixtures; bound every negative
    # fixture by expansions, with generous wall time to avoid scheduler flakes.
    return ConnectorConfig(radius_m=5., max_candidates=8, max_expanded=0,
                           max_length_m=12., **kwargs)


@pytest.mark.parametrize('unknown', [False, True])
def test_nearby_topology_across_obstacle_is_rejected(unknown):
    m = _map(wall_x=4., unknown=unknown)
    topology = _topology(m, [(1, [(4.6, 4.), (8., 4.)])])
    selector = ReachableEndpointSelector(topology, FOOTPRINT, _config())
    query = _query(start=(3.2, 4., 0.))
    assert math.dist(query.start[:2], (4.6, 4.)) < 5.
    result = selector(topology, query)
    certificate = selector.last_certificate
    assert result[2] is None
    assert result[3] == 'ENDPOINT_LOCAL_FREE_DISCONNECTED'
    assert certificate['start_candidates']
    assert all(r['failure'] == 'ENDPOINT_LOCAL_FREE_DISCONNECTED'
               for r in certificate['start_candidates'])
    assert 'bounded' in certificate['negative_proof_scope']
    assert 'not global' in certificate['negative_proof_scope']


def test_local_distance_acceleration_cannot_skip_map_boundary_footprint_collision():
    m = _map()
    # Map bounds are collision even when the first/last raster cells are free.
    m.occupancy[:] = 0
    connector = LocalConnector(m, FOOTPRINT, _config())
    pose = (.1, 4., 0.)
    assert m.footprint_collision(pose, FOOTPRINT, unknown_is_collision=True)
    connector.free_component(pose)
    assert connector.safe([pose]) is False


def test_farther_candidate_in_goal_component_wins_over_nearest():
    m = _map()
    topology = _topology(m, [(1, [(1., 3.), (2.5, 3.)]),
                             (2, [(1., 5.), (8., 5.)])])
    config = replace(_config(), max_length_m=3.5)
    selector = ReachableEndpointSelector(topology, FOOTPRINT, config)
    query = _query(start=(2., 3.5, 0.), goal=(7., 5., 0.))
    raw = selector.index.query(query.start, config.radius_m, config.max_candidates)
    assert raw[0][1] == 0
    start, goal, route, reason = selector(topology, query)
    assert route is not None, reason
    assert start['component'] == goal['component'] == 2
    assert start['edge_id'] == 1
    assert any(r.get('component') == 1 and not r['failure']
               for r in selector.last_certificate['start_candidates'])


def test_goal_closure_precedes_unrelated_start_hybrid_search(monkeypatch):
    m=_map();topology=_topology(m,[(1,[(3.,4.),(6.,4.)]),
                                 (2,[(6.,5.),(8.,5.)]),(3,[(3.,6.),(6.,6.)])])
    selector=ReachableEndpointSelector(topology,FOOTPRINT,_config());query=_query()
    original=selector.connector.connect
    def record(edge,point,component,*,goal=False,success=True,fraction=0.):
        value={'edge_id':edge,'component':component,'point':list(point),'segment':0,
               'fraction':fraction,'arclength_m':3.*fraction,'direction':1,'tangent_yaw':0.,
               'stats':{'expanded':0,'generated':1},'failure':'ENDPOINT_LOCAL_SE2_NO_PATH'}
        if success:
            a,b=((*point,0.),query.goal) if goal else (query.start,(*point,0.))
            connected,_=original(a,b,time.monotonic()+5.,hybrid=False);assert connected
            path,length,word=connected
            value.update(path=path,length_m=length,word=word,min_clearance_m=1.,turn_rad=0.,
                         tangent_error_deg=0.,failure='',hash=digest(path))
        return value
    start=record(0,(3.,4.),1);goal=record(1,(6.,5.),2,goal=True)
    wasted=record(2,(3.,6.),3,success=False)
    closure=record(0,(6.,4.),1,goal=True,success=False,fraction=1.)
    def candidates(pose,is_goal,deadline):
        out,records=([goal],[goal,closure]) if is_goal else ([start],[start,wasted])
        return out,records,{'attempts':2,'expanded':0,'generated':2,'candidate_count':2,'projection_count':2}
    monkeypatch.setattr(selector,'_candidates',candidates);searched=[]
    def bounded_connect(a,b,deadline,**kwargs):
        searched.append((tuple(a),tuple(b)))
        assert tuple(a)==(6.,4.,0.), 'An unrelated start component must not consume the shared deadline first'
        return original(a,b,deadline,hybrid=False)
    monkeypatch.setattr(selector.connector,'connect',bounded_connect)
    selected_start,selected_goal,route,reason=selector(topology,query,deadline=time.monotonic()+5.)
    assert route is not None,reason
    assert selected_start['component']==selected_goal['component']==1
    assert searched==[((6.,4.,0.),query.goal)]
    assert not wasted.get('hybrid_attempted') and closure['hybrid_attempted']


def test_nearby_disconnected_edges_do_not_hide_reachable_farther_projection():
    m = _map(width_m=16., wall_x=4.)
    topology = _topology(m, [(1, [(4.6, 3.), (6., 3.)]),
                             (2, [(1., 6.), (3., 6.)])])
    # The near edge is across the wall. The farther edge is on this side;
    # use a long map so the configured second-stage radius can reach it.
    query = _query(start=(3., .8, math.pi / 2), goal=(2., 6., 0.))
    config = replace(_config(), radius_m=8., max_length_m=15.)
    selector = ReachableEndpointSelector(topology, FOOTPRINT, config)
    near = selector.index.query(query.start, 5., config.max_candidates)
    assert near and all(v[1] == 0 for v in near)
    result = selector(topology, query)
    assert result[2] is not None, result[3]
    assert result[0]['edge_id'] == 1


def test_obstacle_guidance_finds_bounded_forward_detour_without_relaxing_geometry():
    m=_map(width_m=12.,height_m=8.)
    # A long shelf requires a detour to the right; Euclidean distance points
    # directly through the shelf. Both endpoints and the detour are unchanged.
    for row in range(m.height):
        y=m.cell_to_world((row,0))[1]
        if 3.8<=y<=4.2:m.occupancy[row,:round(8./m.resolution)]=100
    m.distance_m=distance_transform_edt(m.occupancy==0)*m.resolution
    cfg=replace(_config(),radius_m=12.,max_length_m=25.,max_expanded=2500)
    start=(2.,3.,0.);goal=(2.,5.,math.pi)
    connector=LocalConnector(m,FOOTPRINT,cfg);connector.free_component(goal)
    estimate=connector.obstacle_heuristic(goal)
    assert estimate(start)>12.
    result,stats=connector.connect(start,goal,time.monotonic()+30.)
    assert result is not None,stats
    path,length,_=result
    assert stats['expanded']<=cfg.max_expanded and length<=cfg.max_length_m
    assert max(p[0] for p in path)>8.
    assert math.dist(path[0][:2],start[:2])<1e-9
    assert math.dist(path[-1][:2],goal[:2])<1e-9
    assert all(not m.footprint_collision(p,FOOTPRINT,unknown_is_collision=True) for p in path)
    # A heuristic cannot turn an occupied goal into a valid connector.
    blocked=(2.,4.,0.)
    rejected,_=connector.connect(start,blocked,time.monotonic()+30.,hybrid=False)
    assert rejected is None


def test_obstacle_guidance_cache_binds_grid_and_uses_finite_fallback():
    m=_map();connector=LocalConnector(m,FOOTPRINT,_config())
    goal=(7.,4.,0.);connector.free_component(goal)
    first=connector.obstacle_heuristic(goal)
    key=connector._obstacle_field_cache[0]
    assert math.isfinite(first((-1.,4.,0.)))
    m.occupancy[:,round(4./m.resolution)]=100
    connector.free_component(goal);second=connector.obstacle_heuristic(goal)
    assert connector._obstacle_field_cache[0]!=key
    assert math.isfinite(second((2.,4.,0.)))
    assert second((2.,4.,0.))==5.


def test_local_screen_replenishes_quota_hidden_by_many_wall_projections():
    m=_map(width_m=16.,wall_x=4.)
    blocked=[(1,[(4.6,1.+i*.12),(12.,1.+i*.12)]) for i in range(20)]
    topology=_topology(m,blocked+[(2,[(2.5,5.),(2.5,7.)])])
    config=replace(_config(),radius_m=8.,max_candidates=2)
    query=_query(start=(3.,1.,math.pi/2),goal=(2.5,6.,math.pi/2))
    selector=ReachableEndpointSelector(topology,FOOTPRINT,config)
    raw=selector.index.query(query.start,config.radius_m,config.max_candidates)
    assert len(raw)==2 and all(v[1]<20 for v in raw)
    before=digest({'nodes':[asdict(v) for v in topology.graph.nodes],
                   'edges':[asdict(v) for v in topology.graph.edges]})
    start,goal,route,reason=selector(topology,query)
    assert route is not None,reason
    assert start['component']==goal['component']==2
    assert selector.last_certificate['diagnostics']['start_local_screen_replenished']
    assert selector.last_certificate['diagnostics']['initial_local_screen_rejections']>=2
    assert any(v['failure']=='ENDPOINT_LOCAL_FREE_DISCONNECTED'
               for v in selector.last_certificate['start_candidates'])
    for connector in (start,goal):
        assert all(not m.footprint_collision(p,FOOTPRINT,unknown_is_collision=True)
                   for p in connector['path'])
    repeat=ReachableEndpointSelector(topology,FOOTPRINT,config)
    assert repeat(topology,query)[2] is not None
    assert repeat.last_certificate['hash']==selector.last_certificate['hash']
    assert before==digest({'nodes':[asdict(v) for v in topology.graph.nodes],
                           'edges':[asdict(v) for v in topology.graph.edges]})


def test_edge_middle_projection_creates_virtual_edges_without_mutation():
    m = _map(width_m=14.)
    topology = _topology(m, [(1, [(1., 4.), (13., 4.)])])
    before = digest({'nodes': [asdict(n) for n in topology.graph.nodes],
                     'edges': [asdict(e) for e in topology.graph.edges]})
    selector = ReachableEndpointSelector(topology, FOOTPRINT, _config())
    query = _query(start=(6., 3., 0.), goal=(8., 4., 0.))
    assert min(math.dist(query.start[:2], (n.x, n.y)) for n in topology.graph.nodes) > 5.
    start, goal, route, reason = selector(topology, query)
    assert route is not None, reason
    assert 0. < start['fraction'] < 1.
    assert route.node_ids[0] == -1 and route.node_ids[-1] == -2
    assert route.polyline[0] == list(query.start[:2])
    assert route.polyline[-1] == list(query.goal[:2])
    assert selector.last_certificate['diagnostics']['virtual_topology_edge_count'] >= 2
    after = digest({'nodes': [asdict(n) for n in topology.graph.nodes],
                    'edges': [asdict(e) for e in topology.graph.edges]})
    assert before == after


@pytest.mark.parametrize('goal', [(1., 4., 0.), (6., 4., math.pi)])
def test_positionally_connected_but_forward_dubins_cannot_turn_in_narrow_corridor(goal):
    m = _map(corridor=(3.65, 4.35))
    config = replace(_config(), max_expanded=120, max_length_m=8.)
    connector = LocalConnector(m, FOOTPRINT, config)
    start = (2., 4., 0.)
    assert not m.footprint_collision(start, FOOTPRINT, unknown_is_collision=True)
    assert not m.footprint_collision(goal, FOOTPRINT, unknown_is_collision=True)
    assert m.connected(m.world_to_cell(*start[:2]), m.world_to_cell(*goal[:2]), allow_unknown=False)
    screen = connector.free_component(start)
    row, col = m.world_to_cell(*goal[:2])
    r0, c0, labels, label = screen
    assert label > 0 and labels[row - r0, col - c0] == label
    result, stats = connector.connect(start, goal, time.monotonic() + 20.)
    assert result is None
    assert 0 < stats['expanded'] <= config.max_expanded
    assert not stats['budget_exhausted']


def test_incompatible_tangents_produce_bounded_negative_certificate():
    m = _map(corridor=(3.65, 4.35))
    topology = _topology(m, [(1, [(4., 3.8), (4., 4.2)])])
    selector = ReachableEndpointSelector(topology, FOOTPRINT,
                                         replace(_config(), max_expanded=80, max_candidates=1))
    result = selector(topology, _query())
    cert = selector.last_certificate
    assert result[2] is None
    assert result[3] in {'ENDPOINT_TANGENT_INCOMPATIBLE', 'ENDPOINT_LOCAL_SE2_NO_PATH'}
    assert {r['direction'] for r in cert['start_candidates']} == {-1, 1}
    assert all(r['failure'] for r in cert['start_candidates'])
    assert 'bounded' in cert['negative_proof_scope'] and 'not global' in cert['negative_proof_scope']


def test_same_input_selects_identical_connectors_and_hash_after_reconstruction():
    m = _map()
    topology = _topology(m, [(1, [(1., 4.), (9., 4.)])])
    query = _query(start=(2., 3., 0.), goal=(7., 5., 0.))
    a = ReachableEndpointSelector(topology, FOOTPRINT, _config())
    b = ReachableEndpointSelector(topology, FOOTPRINT, _config())
    assert a(topology, query)[2] is not None
    assert b(topology, query)[2] is not None
    assert a.last_certificate['hash'] == b.last_certificate['hash']
    assert a.last_certificate['selected_hashes'] == b.last_certificate['selected_hashes']
    first = deepcopy(a.last_certificate)
    timing = {}
    assert a(topology, query, timing=timing)[2] is not None
    assert timing['endpoint_connector_cache_hit'] is True
    assert a.last_certificate['hash'] == first['hash']


def test_endpoint_cache_isolates_cold_and_hot_returned_geometry():
    topology = _topology(_map(), [(1, [(1., 4.), (5., 4.), (9., 4.)])])
    # Freeze the full graph after the legacy lazy adjacency cache exists, so
    # equality below tests geometry/alias mutation rather than cache creation.
    topology.graph.adjacency()
    topology_hash = digest(asdict(topology.graph))
    selector = ReachableEndpointSelector(topology, FOOTPRINT, _config())
    query = _query(start=(2., 3., 0.), goal=(7., 5., 0.))
    cold = selector(topology, query)
    route_hash = digest(asdict(cold[2]))
    certificate_hash = selector.last_certificate['hash']
    start_path_hash = digest(cold[0]['path'])
    for point in cold[2].polyline:
        point[0] += 100.
    cold[0]['path'][0] = (100., 100., 0.)
    assert digest(asdict(topology.graph)) == topology_hash
    hot = selector(topology, query)
    assert digest(asdict(hot[2])) == route_hash
    assert digest(hot[0]['path']) == start_path_hash
    assert selector.last_certificate['hash'] == certificate_hash
    hot[2].polyline[-1][1] += 100.
    selector.last_certificate['start_candidates'].clear()
    again = selector(topology, query)
    assert isinstance(again, tuple)
    assert digest(asdict(again[2])) == route_hash
    assert selector.last_certificate['start_candidates']
    assert selector.last_certificate['hash'] == certificate_hash


def test_endpoint_certificate_does_not_expose_private_cache_binding():
    topology = _topology(_map(), [(1, [(1., 4.), (9., 4.)])])
    selector = ReachableEndpointSelector(topology, FOOTPRINT, _config())
    query = _query()
    assert selector(topology, query)[2] is not None
    binding_hash = digest(selector.binding)
    certificate_hash = selector.last_certificate['hash']
    selector.last_certificate['binding']['config']['radius_m'] += 1.
    assert digest(selector.binding) == binding_hash
    assert selector(topology, query)[2] is not None
    selector.last_certificate['binding']['config']['radius_m'] += 1.
    assert digest(selector.binding) == binding_hash
    assert selector(topology, query)[2] is not None
    assert selector.last_certificate['hash'] == certificate_hash


def test_cached_negative_endpoint_result_preserves_none_and_failure():
    topology = _topology(_map(), [(1, [(8., 6.), (9., 6.)])])
    selector = ReachableEndpointSelector(topology, FOOTPRINT, replace(_config(), radius_m=.5))
    query = _query(start=(2., 2., 0.), goal=(3., 2., 0.))
    cold = selector(topology, query)
    assert cold == (None, None, None, 'ENDPOINT_NO_NEARBY_TOPOLOGY')
    first_hash = selector.last_certificate['hash']
    timing = {}
    hot = selector(topology, query, timing=timing)
    assert hot == cold and isinstance(hot, tuple)
    assert selector.last_certificate['hash'] == first_hash
    assert timing['endpoint_connector_cache_hit'] is True
    assert timing['endpoint_spatial_index_cache_hit'] is True
    assert timing['local_connector_attempts'] == timing['local_connector_expanded'] == 0
    assert timing['endpoint_cache_materialization_wall_ms'] >= 0.


def test_cache_binding_changes_for_map_graph_footprint_resolution_config_and_revision():
    base = _topology(_map(), [(1, [(1., 4.), (9., 4.)])])
    cfg = _config()
    selector = ReachableEndpointSelector(base, FOOTPRINT, cfg)
    reference = digest(selector.binding)
    changes = []
    changed = deepcopy(base); changed.metadata['map_sha256'] = 'different-map'
    changes.append(ReachableEndpointSelector(changed, FOOTPRINT, cfg))
    changed = deepcopy(base); changed.graph.edges[0].polyline[0][0] += .05
    changes.append(ReachableEndpointSelector(changed, FOOTPRINT, cfg))
    changed = deepcopy(base); changed.hospital_map.resolution = .1
    changes.append(ReachableEndpointSelector(changed, FOOTPRINT, cfg))
    changed = deepcopy(base); changed.hospital_map.origin = (.05, 0., 0.)
    changes.append(ReachableEndpointSelector(changed, FOOTPRINT, cfg))
    larger = tuple((x * 1.1, y * 1.1) for x, y in FOOTPRINT)
    changes.append(ReachableEndpointSelector(base, larger, cfg))
    changes.append(ReachableEndpointSelector(base, FOOTPRINT, replace(cfg, max_expanded=1)))
    changes.append(ReachableEndpointSelector(base, FOOTPRINT, replace(cfg, algorithm='next-revision')))
    assert all(digest(item.binding) != reference for item in changes)
    query = _query()
    assert selector(base, query)[2] is not None
    old_key = selector.last_certificate['key']
    moved = _query(start=(2.05, 4., 0.))
    assert selector(base, moved)[2] is not None
    assert selector.last_certificate['key'] != old_key
    yaw_changed = _query(start=(2., 4., .05))
    assert selector(base, yaw_changed)[2] is not None
    assert selector.last_certificate['key'] != old_key


def test_selected_connector_replay_is_exact_full_footprint_forward_and_curvature_safe():
    m = _map()
    topology = _topology(m, [(1, [(1., 4.), (9., 4.)])])
    selector = ReachableEndpointSelector(topology, FOOTPRINT, _config())
    query = _query(start=(2., 3., .2), goal=(7., 5., .1))
    start, goal, route, reason = selector(topology, query)
    assert route is not None, reason
    assert start['path'][0] == query.start
    assert goal['path'][-1] == query.goal
    for record in (start, goal):
        for pose in record['path']:
            assert not m.footprint_collision(pose, FOOTPRINT, unknown_is_collision=True)
        for a, b in zip(record['path'], record['path'][1:]):
            distance = math.dist(a[:2], b[:2])
            if distance < 1e-10:
                assert abs(wrap(b[2] - a[2])) < 1e-10
                continue
            assert distance <= .025 + 1e-9
            midpoint_yaw = a[2] + .5 * wrap(b[2] - a[2])
            assert (b[0] - a[0]) * math.cos(midpoint_yaw) + (b[1] - a[1]) * math.sin(midpoint_yaw) > 0
            # Chord correction turns the finite sampled yaw difference back
            # into the continuous constant-curvature bound of its arc.
            curvature = 2 * math.sin(abs(wrap(b[2] - a[2])) / 2) / distance
            assert curvature <= 2.5 + 1e-7
        assert len(record['hash']) == 64


def test_frozen_connector_configuration_rejects_weaker_radius_or_heading_contract():
    with pytest.raises(ValueError):
        ConnectorConfig(minimum_turning_radius_m=.39)
    with pytest.raises(ValueError):
        ConnectorConfig(heading_bins=24)
    with pytest.raises(ValueError):
        ConnectorConfig(sample_spacing_m=.05)


def test_existing_virtual_route_skips_irrelevant_component_hybrid_search(monkeypatch):
    m = _map()
    topology = _topology(m, [(1, [(1., 4.), (9., 4.)]),
                             (2, [(1., 6.), (9., 6.)])])
    selector = ReachableEndpointSelector(topology, FOOTPRINT, _config())
    original = selector.connector.connect
    calls = []
    def connect(start, goal, deadline, *, hybrid=True):
        calls.append(hybrid)
        assert not hybrid, 'a certified route already exists; no unrelated component search'
        if abs(start[1]-6.) < 1e-8 or abs(goal[1]-6.) < 1e-8:
            return None, {'expanded': 0, 'generated': 1}
        return original(start, goal, deadline, hybrid=False)
    monkeypatch.setattr(selector.connector, 'connect', connect)
    start, goal, route, reason = selector(topology, _query())
    assert route is not None, reason
    assert start['component'] == goal['component'] == 1
    assert any(r.get('component') == 2 and r['failure'] == 'ENDPOINT_LOCAL_SE2_NO_PATH'
               for r in selector.last_certificate['start_candidates'])
    assert calls and not any(calls)


def test_geometry_cache_rebinds_certificate_query_identity():
    topology = _topology(_map(), [(1, [(1., 4.), (9., 4.)])])
    selector = ReachableEndpointSelector(topology, FOOTPRINT, _config())
    assert selector(topology, _query(query_id='query_A'))[2] is not None
    first = deepcopy(selector.last_certificate)
    timing = {}
    assert selector(topology, _query(query_id='query_B'), timing=timing)[2] is not None
    assert timing['endpoint_connector_cache_hit'] is True
    second = selector.last_certificate
    assert second['query_id'] == 'query_B'
    assert second['hash'] != first['hash']
    assert second['selected_hashes'] == first['selected_hashes']
    assert second['hash'] == digest({k:v for k,v in second.items() if k not in {'diagnostics','hash'}})


def test_tiny_components_do_not_exhaust_projection_quota_for_long_route():
    m = _map(width_m=14.)
    segments = [(100, [(float(x),4.) for x in np.arange(1.,13.1,.5)])]
    segments += [(i+1, [(5.5,2.+.04*i),(5.55,2.+.04*i)]) for i in range(28)]
    topology = _topology(m, segments)
    selector = ReachableEndpointSelector(topology, FOOTPRINT,
                                         replace(_config(), max_candidates=32))
    values = selector.index.query((6.,3.,0.),5.,32)
    assert len(values) == 32
    components = [selector.index.nodes[selector.edges[v[1]].source].component_id for v in values]
    assert components.count(100) >= 8
    assert len(set(components)) >= 8, 'retain alternatives in nearby components'
    assert values == selector.index.query((6.,3.,0.),5.,32)


def test_full_footprint_sweep_rejects_obstacle_between_safe_saved_poses():
    # Reduced from frozen mentor A2B-14; no original map/query is changed.
    occupancy=np.zeros((80,80),np.int8);occupancy[62,27]=100
    m=HospitalMap(Path('/synthetic/sweep.yaml'),Path('/synthetic/sweep.pgm'),.05,
                  (128.5,22.5,0.),80,80,occupancy,distance_transform_edt(occupancy==0)*.05)
    first=(129.53412756915083,23.279697811078584,-2.0812527080147807)
    second=(129.52127486475518,23.25833506597842,-2.143590508002315)
    midpoint=(129.5278676937473,23.26891627925515,-2.112421608008548)
    assert not m.footprint_collision(first,FOOTPRINT,unknown_is_collision=True)
    assert not m.footprint_collision(second,FOOTPRINT,unknown_is_collision=True)
    assert m.footprint_collision(midpoint,FOOTPRINT,unknown_is_collision=True)
    connector=LocalConnector(m,FOOTPRINT,_config())
    assert connector.safe([first,second]) is False
    connector.free_component(first)
    assert connector.safe([first,second]) is False


@pytest.mark.parametrize('expiry_boundary', ['before_refinement', 'inside_free_component'])
def test_refinement_deadline_exit_is_not_a_stable_negative_cache(monkeypatch, expiry_boundary):
    import arena_evaluation.reachable_endpoint_r3 as module
    topology = _topology(_map(), [(1, [(1., 4.), (9., 4.)])])
    selector = ReachableEndpointSelector(topology, FOOTPRINT, _config())
    clock = [0.]
    expire = [True]
    candidate_calls = []
    hybrid_calls = []
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    def candidates(pose, goal, deadline):
        candidate_calls.append(goal)
        if expire[0] and goal and expiry_boundary == 'before_refinement':
            clock[0] = 2.
        return [], [{'failure': 'ENDPOINT_LOCAL_SE2_NO_PATH', 'component': 1,
                     'point': (4., 4.), 'tangent_yaw': 0.}], {
                     'attempts': 1, 'expanded': 0, 'generated': 0,
                     'candidate_count': 1, 'projection_count': 1}
    def free_component(pose):
        if expire[0] and expiry_boundary == 'inside_free_component':
            clock[0] = 2.
    def connect(start, goal, deadline):
        hybrid_calls.append((start, goal))
        return None, {'expanded': 1, 'generated': 1}
    monkeypatch.setattr(selector, '_candidates', candidates)
    monkeypatch.setattr(selector.connector, 'free_component', free_component)
    monkeypatch.setattr(selector.connector, 'connect', connect)
    timing = {}
    result = selector(topology, _query(), deadline=1., timing=timing)
    assert result[2] is None
    assert result[3] == 'ENDPOINT_CONNECTOR_BUDGET_EXHAUSTED'
    assert timing['local_connector_budget_exhausted'] is True
    assert selector.last_certificate['negative_proof_scope'] == 'bounded local candidates; not global SE2 infeasibility'
    assert not selector.cache
    assert not hybrid_calls
    expire[0] = False
    timing = {}
    later = selector(topology, _query(), deadline=10., timing=timing)
    assert timing['endpoint_connector_cache_hit'] is False
    assert len(candidate_calls) == 4
    assert hybrid_calls, 'the later request must actually retry previously skipped refinement'
    assert later[3] == 'ENDPOINT_LOCAL_SE2_NO_PATH'
