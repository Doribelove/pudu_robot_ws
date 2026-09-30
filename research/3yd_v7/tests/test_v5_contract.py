from pathlib import Path
from types import SimpleNamespace
import json
import numpy as np
import pytest
from arena_3d_v1.semantic_world import SemanticWorld
from three_yd_v5.topology import V5Topology
from three_yd_v5.controller import V5Controller,centerward_offset_guide

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture(scope='module')
def prepared(tmp_path_factory):
    root=tmp_path_factory.mktemp('v5')
    world=SemanticWorld(ROOT/'inputs/map.yaml',ROOT/'inputs/semantic.json',cache_root=root/'world')
    topology=V5Topology(world,ROOT/'inputs/topology.json',root/'graph')
    queries={q['id']:SimpleNamespace(query_id=q['id'],start=tuple(q['start'][k] for k in ('x','y','yaw_rad')),goal=tuple(q['goal'][k] for k in ('x','y','yaw_rad'))) for q in json.loads((ROOT/'inputs/queries.json').read_text())['scenarios']}
    return world,topology,queries,root


def test_l1_all_queries_minimal_output_and_ignores_endpoint_heading(prepared):
    world,topology,queries,_=prepared
    for q in queries.values():
        route=topology.plan(q);task=topology.task(route)
        assert len(task['right_reference_polyline'])>=2
        assert np.isfinite(task['right_reference_polyline']).all()
        assert 'visits' not in task and 'gates' not in task and 'endpoint_access' not in task
        assert task['selected_route_has_branches'] is False
        assert all(x['selected_successor_only'] for x in task['junction_joins'])
        assert not route.interface_ids
        original=q.start;q.start=(*q.start[:2],q.start[2]+2.)
        assert topology.plan(q).signature==route.signature
        q.start=original


def test_q02_direct_road_and_q03_three_roads(prepared):
    _,t,qs,_=prepared
    a=t.task(t.plan(qs['Q02']))
    assert len(a['right_reference_polyline'])==2
    assert {rid for rid in a['region_sequence'] if t.regions[rid]['kind']=='lane'}=={1}
    b=t.task(t.plan(qs['Q03']))
    assert [rid for rid in dict.fromkeys(b['region_sequence']) if t.regions[rid]['kind']=='lane']==[16,13,7]


def test_l2_unique_xy_states_and_soft_clearance_metadata(prepared):
    w,t,qs,root=prepared
    c=V5Controller(w,t,t.plan(qs['Q06']),cache_root=root/'l2')
    g=c.l2.geometry
    cells=[g.global_cell(i) for i in range(g.state_count)]
    assert len(set(cells))==g.state_count and not hasattr(g,'stage')
    assert len(c.reference_plan.world_xy)<len(c.raw_reference_xy)
    assert c.reference_plan.world_xy.shape[1]==2
    assert np.allclose(c.reference_plan.world_xy[0],qs['Q06'].start[:2])
    assert np.allclose(c.reference_plan.world_xy[-1],qs['Q06'].goal[:2])
    assert c.reference_mode=='v5_r4_conservative_circle'
    assert c.rectangle_bridge_cells.shape==(0,2)
    attempt=c.timing['activation_attempts'][0]
    assert attempt['hard_obstacle_model']=='original circumscribed circle; every represented fine cell clear'
    assert attempt['static_obstacle_model']=='confirmed V5-r3 conservative primary'
    assert attempt['l2_vehicle_feasibility_is_final'] is False
    assert attempt['shortcut_cost_field']=='r8 selected L1 guide inward offset 0.30m across roads and junctions; hard obstacles and direction enforced'
    assert attempt['l2_soft_target_offset_m']==.30
    applied=np.asarray([item['applied_offset_m'] for item in c.l2_guide_offset_records])
    assert np.allclose(applied,.30)


def test_centerward_offset_uses_road_travel_direction(prepared):
    _,topology,queries,_=prepared
    route=topology.plan(queries['Q06']);task=topology.task(route)
    target,records,info=centerward_offset_guide(task['right_reference_polyline'],topology,
        topology.contexts[route.signature]['unique_roads'],
        [rid for rid in route.region_ids if topology.regions[rid]['kind']=='junction_area'],.30)
    source=np.asarray([item['source_xy'] for item in records])
    directions=np.asarray([item['direction'] for item in records])
    left=np.column_stack((-directions[:,1],directions[:,0]))
    applied=np.asarray([item['applied_offset_m'] for item in records])
    assert np.allclose(target-source,applied[:,None]*left,atol=1e-8)
    assert info['corner_region_count']>0


def test_corner_region_uses_full_offset_without_exemption(prepared):
    _,topology,queries,_=prepared
    route=topology.plan(queries['Q03']);task=topology.task(route)
    target,records,info=centerward_offset_guide(task['right_reference_polyline'],topology,
        topology.contexts[route.signature]['unique_roads'],
        [rid for rid in route.region_ids if topology.regions[rid]['kind']=='junction_area'],.30)
    applied=np.asarray([item['applied_offset_m'] for item in records])
    assert info['corner_region_count']>0
    assert np.allclose(applied,.30)
    assert info['transition_outside_polygon_m']==0.
    assert info['corner_offset_policy']=='full 0.30 m; no exemption or taper'
    source=np.asarray([item['source_xy'] for item in records])
    assert np.allclose(np.linalg.norm(target-source,axis=1),.30,atol=1e-7)


def test_q06_selected_junctions_trim_stubs_without_triangle(prepared):
    _,topology,queries,_=prepared
    route=topology.plan(queries['Q06']);task=topology.task(route)
    points=np.asarray(task['right_reference_polyline'],float)
    assert len(points)==4
    assert len(task['junction_joins'])==2
    assert all(item['accepted'] and item['method']=='tangent_intersection'
               for item in task['junction_joins'])
    # The r6 geometry contained two >120 degree portal-end reversals.
    vectors=np.diff(points,axis=0)
    turns=[]
    for a,b in zip(vectors,vectors[1:]):
        turns.append(np.degrees(np.arccos(np.clip(np.dot(a,b)/np.linalg.norm(a)/np.linalg.norm(b),-1.,1.))))
    assert max(turns)<100.


def test_complete_topology_keeps_unselected_successors(prepared):
    _,topology,_,_=prepared
    # Compiling one route must not mutate the reusable complete adjacency.
    before={rid:tuple(sorted(neighbors)) for rid,neighbors in topology.adj.items()}
    assert {26,27,28}.issubset(topology.adj[12])
    assert before=={rid:tuple(sorted(neighbors)) for rid,neighbors in topology.adj.items()}


def test_disconnected_query_keeps_strict_failure_without_unverified_reference(prepared):
    w,t,qs,root=prepared
    with pytest.raises(RuntimeError,match='V5_L2_RECTANGLE_BRIDGE_NOT_FOUND'):
        V5Controller(w,t,t.plan(qs['Q04']),cache_root=root/'l2-q04')
    failed=list((root/'l2-q04').glob('failed_reference_*.json'))
    assert len(failed)==1
    attempt=json.loads(failed[0].read_text())['activation_attempts'][0]
    assert attempt['rectangle_recovery_attempted']
    assert attempt['state_dimensions']==['x','y']


def test_parking_ownership_preserved(prepared):
    _,t,qs,_=prepared
    for name,owned in [('Q08',1),('Q09',2)]:
        route=t.plan(qs[name]);ctx=t.contexts[route.signature]
        role='start' if name=='Q08' else 'goal'
        assert ctx['endpoint_access'][role]['road_id']==owned


def test_heading_free_l2_marks_large_endpoint_mismatch_for_local_l3(prepared):
    w,t,qs,root=prepared
    route=t.plan(qs['Q06'])
    c=V5Controller(w,t,route,cache_root=root/'l2-endpoint-turn')
    endpoint=[x for x in c.corner_records if x.get('kind')=='endpoint_heading_mismatch']
    assert endpoint
    assert endpoint[0]['index']==0
    assert endpoint[0]['turn_angle_deg']>=30.
