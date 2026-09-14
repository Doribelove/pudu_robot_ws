import ast
import copy
import json
import math
from pathlib import Path
import numpy as np
import pytest
from stl_v0.model import load_scene
from stl_v0.logic import candidate_routes
from stl_v0.bezier import evaluate,certify_curvature,split
from stl_v0.solver import plan
from stl_v0.output import sample_trajectory
from stl_v0.map_input import grow_rectangle

ROOT=Path(__file__).resolve().parents[1]

def data(name):return json.loads((ROOT/'scenes'/f'{name}.json').read_text())

@pytest.mark.parametrize('name',['straight','turn','right_band','rule_route'])
def test_generated_paths_obey_endpoint_body_region_curvature_and_timing(name):
    s=load_scene(data(name));r=plan(s,budget_s=15,candidates=4)
    assert r['research_valid'],r['candidate_results']
    t=r['trajectory'];rows=sample_trajectory(t)
    assert np.allclose([rows[0]['x'],rows[0]['y']],s.start[:2],atol=1e-8)
    assert np.allclose([rows[-1]['x'],rows[-1]['y']],s.goal[:2],atol=1e-8)
    for value,expected in [(rows[0]['yaw'],s.start[2]),(rows[-1]['yaw'],s.goal[2])]:
        assert abs(math.atan2(math.sin(value-expected),math.cos(value-expected)))<1e-6
    for i,p in enumerate(t['control_points']):
        reg=s.regions[t['route'][i]]
        for xy in np.array(p):assert reg.contains(xy,tol=2e-7)
        assert certify_curvature(p,2.5)['valid']
    assert max(abs(x['curvature_per_m']) for x in rows)<=2.5+1e-8
    assert min(x['speed_mps'] for x in rows)>0
    assert max(x['speed_mps'] for x in rows)<=s.speed_max_mps+1e-6
    assert rows[-1]['time_s']<=s.task.horizon_s+1e-6
    for w in t['witnesses']:assert w['window_s'][0]-1e-8<=w['time_s']<=w['window_s'][1]+1e-8
    assert r['production_accepted'] is False and r['r2_semantic_acceptance'] is None
    if name=='right_band':
        assert max(x['knot_displacement_max_m'] for x in r['candidate_results'] if 'knot_displacement_max_m' in x)>.35
    if name=='rule_route':assert 'upper' not in [s.regions[i].id for i in t['route']]


def test_infeasible_deadline_is_not_success_or_continuous_infeasibility_proof():
    r=plan(load_scene(data('deadline_negative')),budget_s=3,candidates=2,maxiter=30)
    assert not r['research_valid'] and r['trajectory'] is None
    assert not r['completeness_claim']
    assert r['r2_semantic_acceptance'] is None


def test_rule_changes_candidate_graph_and_missing_obligation_rejects():
    d=data('rule_route');s=load_scene(d);paths,_=candidate_routes(s,4)
    assert paths and all(1 not in p for p in paths)
    d['task']['always']=[{'predicate':'safe'}];d['task']['ordered_eventually']=[]
    paths,_=candidate_routes(load_scene(d),4)
    assert any(1 in p for p in paths) and any(2 in p for p in paths)
    d['task']['ordered_eventually']=[{'label':'missing','window_s':[0,40]}]
    assert candidate_routes(load_scene(d),2)[0]==[]


def test_unsupported_stl_is_fail_closed():
    d=data('straight');d['task']['until']={}
    with pytest.raises(ValueError,match='UNSUPPORTED_FIELDS'):load_scene(d)
    d=data('straight');d['task']['always']=[{'predicate':'magic'}]
    with pytest.raises(ValueError,match='UNSUPPORTED_STL'):load_scene(d)


def test_invalid_geometry_and_nonfinite_inputs_rejected():
    d=data('straight');d['start'][0]=float('nan')
    with pytest.raises(ValueError):load_scene(d)
    d=data('straight');d['regions'][0]['bounds']=[0,0,0,2]
    with pytest.raises(ValueError):load_scene(d)
    d=data('straight');d['resolution_m']=.1
    with pytest.raises(ValueError):load_scene(d)


def test_continuous_curve_check_detects_cusp_and_tight_turn():
    cusp=np.array([[0,0],[1,0],[-1,0],[0,0]],float)
    assert not certify_curvature(cusp,2.5)['valid']
    tight=np.array([[0,0],[.01,0],[.02,.01],[.02,.02]])
    assert not certify_curvature(tight,2.5)['valid']
    straight=np.array([[0,0],[1,0],[2,0],[3,0]],float)
    assert certify_curvature(straight,2.5)['valid']


def test_subdivision_replays_same_curve():
    p=np.array([[0,0],[1,0],[2,1],[2,3]],float);a,b=split(p)
    u=np.linspace(0,1,41)
    assert np.allclose(evaluate(a,u)[0],evaluate(p,u/2)[0])
    assert np.allclose(evaluate(b,u)[0],evaluate(p,.5+u/2)[0])


def test_rectangles_never_include_unsafe_cells():
    a=np.ones((20,25),bool);a[5:15,12]=False;a[0,:]=False
    for y,x in [(3,3),(16,15),(6,18)]:
        for direction in [False,True]:
            x0,y0,x1,y1=grow_rectangle(a,y,x,vertical_first=direction)
            assert a[y0:y1,x0:x1].all()
            assert x0<=x<x1 and y0<=y<y1


def test_no_legacy_architecture_imports():
    forbidden=('arena_','two_layer','three_d','nav2','rclpy','layered_')
    for p in (ROOT/'stl_v0').glob('*.py'):
        for node in ast.walk(ast.parse(p.read_text())):
            names=[]
            if isinstance(node,ast.Import):names=[x.name for x in node.names]
            elif isinstance(node,ast.ImportFrom) and node.module:names=[node.module]
            assert not any(name.startswith(forbidden) for name in names),(p,names)


def test_independent_footprint_audit_checks_whole_body_and_unknown():
    from stl_v0.audit import audit_footprint
    cfg={'resolution':.05,'origin':[0,0,0]}
    free=np.ones((60,60),bool)
    row={'x':1.5,'y':1.5,'yaw':0.}
    assert audit_footprint([row],free,cfg)['valid']
    # Obstacle under front corner while center pixel remains free.
    free[29,34]=False
    assert not audit_footprint([row],free,cfg)['valid']
    assert audit_footprint([],free,cfg)['valid'] is None
    assert not audit_footprint([{'x':.1,'y':.1,'yaw':0.}],free,cfg)['valid']


def test_graph_deadline_and_nonpositive_budget_fail_closed():
    import time
    paths,info=candidate_routes(load_scene(data('straight')),deadline=time.monotonic()-1)
    assert paths==[] and info['status']=='GRAPH_BUDGET_EXHAUSTED'
    with pytest.raises(ValueError,match='INVALID_BUDGET'):
        plan(load_scene(data('straight')),budget_s=0)


def test_c1_physical_velocity_shared_at_transitions():
    s=load_scene(data('turn'));r=plan(s,5,3)
    t=r['trajectory'];p=np.array(t['control_points']);dt=np.array(t['durations_s'])
    outgoing=3*(p[:-1,3]-p[:-1,2])/dt[:-1,None]
    incoming=3*(p[1:,1]-p[1:,0])/dt[1:,None]
    assert np.allclose(outgoing,incoming,atol=1e-10)
    assert np.allclose(p[:-1,3],p[1:,0],atol=1e-10)


def test_cover_repairs_edge_touch_and_holes_without_unsafe_inclusion():
    from stl_v0.cover import repair_cover,components
    a=np.ones((12,18),bool);a[4:8,7:11]=False
    boxes=[(0,0,7,12),(11,0,18,12),(7,0,11,4),(7,8,11,12)]
    assert components(boxes)[0]==4
    repaired,info=repair_cover(a,boxes)
    assert components(repaired)[0]==1
    assert info['added_bridge_regions']>0 and info['unbridged_safe_adjacencies']==0
    for x0,y0,x1,y1 in repaired:assert a[y0:y1,x0:x1].all()


def test_cover_fills_thin_safe_connection_but_does_not_join_disconnected_rooms():
    from stl_v0.cover import repair_cover,components
    a=np.zeros((12,25),bool);a[1:6,1:6]=True;a[1:6,10:15]=True;a[3,6:10]=True;a[8:11,20:24]=True
    repaired,info=repair_cover(a,[(1,1,6,6),(10,1,15,6),(20,8,24,11)])
    assert info['uncovered_safe_pixels_before']==4 and info['uncovered_safe_pixels_after']==0
    assert components(repaired)[0]==2
    for x0,y0,x1,y1 in repaired:assert a[y0:y1,x0:x1].all()


def test_semantic_halfspaces_remove_false_bbox_adjacency():
    from stl_v0.logic import overlap
    from stl_v0.model import Region
    a=Region('a',np.array([0.,0.,2.,2.]),halfspaces=[[1,0,.5]])
    b=Region('b',np.array([0.,0.,2.,2.]),halfspaces=[[-1,0,-1.5]])
    assert overlap(a,b) is None


def test_fixed_pose_uses_same_semantic_feasible_seed_as_joint():
    from stl_v0.solver import solve_route
    import time
    s=load_scene(data('right_band'));route=candidate_routes(s,1)[0][0]
    joint,_=solve_route(s,route,time.monotonic()+5,transition_mode='joint')
    fixed,_=solve_route(s,route,time.monotonic()+5,transition_mode='fixed_pose')
    assert np.allclose(joint['initial_knots'],fixed['initial_knots'])
    for p in np.array(joint['initial_knots'])[1:-1]:assert p[1]<=-.4+1e-8


def test_independent_local_rule_audit_detects_wrong_side_and_wrong_heading():
    from stl_v0.semantic import audit_local_rule
    rule={'centerline_x_of_y':[0.,0.],'active_y_interval_m':[-1.,1.],'right_offset_m':.35}
    def trajectory(x,reverse=False):
        p=np.c_[np.full(4,x),np.linspace(-2,2,4)]
        if reverse:p=p[::-1]
        return {'control_points':[p.tolist()],'durations_s':[10.]}
    t=trajectory(.5);assert audit_local_rule(sample_trajectory(t),t,rule)['valid']
    for t in [trajectory(-.5),trajectory(.5,True)]:assert not audit_local_rule(sample_trajectory(t),t,rule)['valid']
    assert audit_local_rule([],None,rule)['valid'] is None


def test_local_rule_activation_boundary_does_not_include_inactive_curve():
    from stl_v0.semantic import audit_local_rule
    t={'control_points':[[[0,-2],[0,-1],[.5,0],[.5,1]]],'durations_s':[10.]}
    rule={'centerline_x_of_y':[0,0],'right_offset_m':.2,'active_y_interval_m':[0,.8]}
    assert audit_local_rule(sample_trajectory(t),t,rule)['valid']


def test_sparse_candidate_family_does_not_get_displaced_by_added_regions():
    d=data('straight')
    for region in d['regions']:region['labels']=['sparse_seed_cover']
    d['regions'].append({'id':'new_large_box','bounds':[-2,-3,10,3]})
    s=load_scene(d);new=len(s.regions)-1
    sparse,_=candidate_routes(s,3,required_label='sparse_seed_cover',width_weight=0.)
    all_routes,_=candidate_routes(s,3,width_weight=0.)
    assert sparse and all(new not in route for route in sparse)
    assert any(new in route for route in all_routes)


def test_spatial_sweep_matches_brute_force_with_touching_and_nested_boxes():
    from stl_v0.spatial import box_pairs
    rng=np.random.default_rng(20260910);lo=rng.uniform(-5,5,(120,2));hi=lo+rng.uniform(.01,7,(120,2));b=np.c_[lo,hi]
    b=np.r_[b,[[0,0,1,1],[1,0,2,1],[.2,.2,.8,.8],[.5,0,.500000001,1]]]
    for epsilon in [0,1e-6]:
        got,_=box_pairs(b,epsilon)
        wanted=[(i,j) for i in range(len(b)) for j in range(i+1,len(b)) if np.all(np.minimum(b[i,2:],b[j,2:])-np.maximum(b[i,:2],b[j,:2])>epsilon)]
        assert got.tolist()==[list(x) for x in wanted]


def test_geometry_cache_invalidates_changed_halfspace_but_task_is_not_cached():
    from stl_v0.geometry_graph import prepare_geometry,clear_geometry_cache
    clear_geometry_cache();d=data('straight');s=load_scene(d)
    _,a=prepare_geometry(s);_,b=prepare_geometry(s);assert not a['cache_hit'] and b['cache_hit']
    d['regions'][1]['halfspaces']=[[0,1,-.2]];_,c=prepare_geometry(load_scene(d));assert not c['cache_hit']
    d['task']['ordered_eventually']=[{'label':'missing','window_s':[0,40]}]
    assert candidate_routes(load_scene(d),2)[0]==[]


def test_preparation_cache_reuse_invalidation_and_corruption(tmp_path):
    from stl_v0.prepared_cache import cached_json,prepared_scene
    calls=[]
    def factory():calls.append(1);return {'geometry':[(1,2)]}
    a,t=cached_json(tmp_path,'key',factory);b,u=cached_json(tmp_path,'key',factory)
    assert a==b and len(calls)==1 and not t['cache_hit'] and u['cache_hit']
    (tmp_path/'key.json').write_text('{"key":"key","sha256":"wrong","payload":{}}')
    with pytest.raises(ValueError,match='CORRUPT'):cached_json(tmp_path,'key',factory)
    safe=np.ones((60,80),bool);ns=np.zeros_like(safe);cfg={'resolution':.05,'origin':[0,0,0]};q={'query_id':'test','category':'test','start':[1,1,0],'goal':[2,1,0]}
    cold,a=prepared_scene(safe,ns,cfg,q,{},tmp_path,margin_m=1.)
    hot,b=prepared_scene(safe,ns,cfg,q,{},tmp_path,margin_m=1.);assert b['cache_hit']
    assert hot==cold
    safe[1,1]=False;_,c=prepared_scene(safe,ns,cfg,q,{},tmp_path,margin_m=1.);assert c['key']!=a['key']
    q['goal'][2]=.2;_,e=prepared_scene(safe,ns,cfg,q,{},tmp_path,margin_m=1.);assert e['key']!=c['key']


def test_subdivided_route_keeps_physical_continuity_and_constraints():
    from stl_v0.solver import solve_route
    import time
    scene=load_scene(data('turn'));route=candidate_routes(scene,1)[0][0]
    log,t=solve_route(scene,route,time.monotonic()+10,segments_per_region=2)
    assert t is not None,log
    assert len(t['route'])==2*len(route)
    cp=np.asarray(t['control_points']);dt=np.asarray(t['durations_s'])
    assert np.allclose(cp[:-1,3],cp[1:,0])
    assert np.allclose(3*(cp[:-1,3]-cp[:-1,2])/dt[:-1,None],3*(cp[1:,1]-cp[1:,0])/dt[1:,None])
    for i,p in enumerate(cp):
        assert certify_curvature(p,2.5)['valid']
        assert all(scene.regions[t['route'][i]].contains(v,2e-7) for v in p)
