import math,json
from types import SimpleNamespace
from dataclasses import replace
import numpy as np
import pytest
from test_semantic_dual_map import make_world
from arena_evaluation.planner_benchmark.models import Query
from arena_3d_v1.multires_grid import GridView
from arena_3d_v1.multires_topology import SemanticTopology
from arena_3d_v1.local_fine import LocalFineGraph
from arena_3d_v1.recovery_pipeline import RecoveryController,CandidateRecovery

def test_local_edges_never_jump_corner_obstacle_and_weighted_oracle(tmp_path):
    w=make_world(tmp_path,'pillar');g=SemanticTopology(w,tmp_path/'g');v=GridView(w,3)
    q=Query('q',(2.,3.,0.),(18.,3.,0.),seed=0);r=g.plan(q);mask=g.fine_mask(r)
    st=w.map.world_to_cell(*q.start[:2]);go=w.map.world_to_cell(*q.goal[:2])
    graph=LocalFineGraph(w,v,mask,st,go,r.signature)
    potential=np.zeros(w.safe.shape);potential[:,200:220]=.7
    rc,a=graph.search(potential);_,b=graph.search(potential,heuristic=False)
    assert abs(a['cost_cells']-b['cost_cells'])<1e-8
    assert all(graph._edge(tuple(x),tuple(y)) for x,y in zip(rc,rc[1:]))
    # Fine obstacle deliberately inserted between two active coarse centers.
    pairs=[(tuple(x),tuple(y)) for x,y in zip(rc,rc[1:]) if max(abs(y-x))==3]
    x,y=pairs[0];mid=((x[0]+y[0])//2,(x[1]+y[1])//2);graph.safe[mid]=False
    assert not graph._edge(x,y)

def test_patch_budget_does_not_silently_relax_clearance(tmp_path):
    w=make_world(tmp_path);g=SemanticTopology(w,tmp_path/'g');v=GridView(w,3)
    q=Query('q',(2.,3.,0.),(28.,3.,0.),seed=0);r=g.plan(q)
    with pytest.raises(RuntimeError,match='LOCAL_FINE_BUDGET_EXCEEDED'):
        LocalFineGraph(w,v,g.fine_mask(r),w.map.world_to_cell(*q.start[:2]),w.map.world_to_cell(*q.goal[:2]),r.signature,max_fine_cells=1)

def test_portal_samples_remain_lightweight_and_restore(tmp_path):
    w=make_world(tmp_path,'pillar');g=SemanticTopology(w,tmp_path/'g',portal_samples=3)
    restored=SemanticTopology(w,tmp_path/'g',portal_samples=3)
    assert restored.cache_hit and len(g.nodes)==len(restored.nodes)
    assert not list((tmp_path/'g').rglob('geometry.npz'))

def test_route_candidate_failure_retries_and_does_not_close_graph(tmp_path):
    w=make_world(tmp_path,'pillar');g=SemanticTopology(w,tmp_path/'g',portal_samples=3)
    q=Query('q',(2.,5.,0.),(18.,5.,0.),seed=0);initial=g.plan(q);original_nodes=dict(g.nodes);calls=[]
    class ControlledBackend:
        def __init__(self,world,topology,route,**kwargs):self.route=route;self.lifecycle=SimpleNamespace(clear=lambda:None)
        def plan_l3(self,*args,**kwargs):
            calls.append(self.route)
            return {'success':len(calls)>1,'failure_code':'L3_NO_PATH' if len(calls)==1 else ''}
    c,r,out,stats=CandidateRecovery(w,g,cache_root=tmp_path/'c',controller_class=ControlledBackend).plan(q,None,None,None,initial_route=initial)
    assert out['success'] and len(stats['attempts'])==2
    assert stats['attempts'][0]['portal_sequence']!=stats['attempts'][1]['portal_sequence']
    assert dict(g.nodes)==original_nodes and g.plan(q).interface_ids==initial.interface_ids
    assert stats['persistent_channel_updates']==0

def test_ack_failure_never_masks_as_an_alternative_route(tmp_path):
    w=make_world(tmp_path,'pillar');g=SemanticTopology(w,tmp_path/'g');q=Query('q',(2.,5.,0.),(18.,5.,0.),seed=0)
    class AckFailure:
        def __init__(self,*args,**kwargs):self.lifecycle=SimpleNamespace(clear=lambda:None)
        def plan_l3(self,*args,**kwargs):return {'success':False,'failure_code':'COSTMAP_CONTENT_ACK_FAILED'}
    _,_,out,stats=CandidateRecovery(w,g,cache_root=tmp_path/'c',controller_class=AckFailure).plan(q,None,None,None)
    assert not out['success'] and len(stats['attempts'])==1 and stats['candidate_searches']==1

def test_integrity_failure_propagates_without_route_retry(tmp_path):
    w=make_world(tmp_path,'pillar');g=SemanticTopology(w,tmp_path/'g');q=Query('q',(2.,5.,0.),(18.,5.,0.),seed=0)
    class Corrupt:
        def __init__(self,*args,**kwargs):raise ValueError('LOCAL_REFERENCE_CACHE_CORRUPT')
    with pytest.raises(ValueError,match='CACHE_CORRUPT'):
        CandidateRecovery(w,g,cache_root=tmp_path/'c',controller_class=Corrupt).plan(q,None,None,None)

def test_direction_check_includes_intermediate_fine_cells(tmp_path):
    w=make_world(tmp_path);g=SemanticTopology(w,tmp_path/'g');v=GridView(w,3)
    q=Query('q',(2.,3.,0.),(28.,3.,0.),seed=0);r=g.plan(q)
    graph=LocalFineGraph(w,v,g.fine_mask(r),w.map.world_to_cell(*q.start[:2]),w.map.world_to_cell(*q.goal[:2]),r.signature)
    rc,_=graph.search();x,y=next((tuple(a),tuple(b)) for a,b in zip(rc,rc[1:]) if max(abs(b-a))==3)
    membership=np.zeros(w.safe.shape,bool);membership[(x[0]+y[0])//2,(x[1]+y[1])//2]=True
    yaw=math.atan2(-(y[0]-x[0]),y[1]-x[1])+math.pi
    graph.directions=[(membership,yaw)]
    assert not graph._edge(x,y)

def test_candidate_attempt_limit_is_explicit(tmp_path):
    w=make_world(tmp_path,'pillar');g=SemanticTopology(w,tmp_path/'g');q=Query('q',(2.,5.,0.),(18.,5.,0.),seed=0)
    class Failure:
        def __init__(self,*args,**kwargs):self.lifecycle=SimpleNamespace(clear=lambda:None)
        def plan_l3(self,*args,**kwargs):return {'success':False,'failure_code':'L3_SEARCH_TIMEOUT'}
    _,_,out,stats=CandidateRecovery(w,g,cache_root=tmp_path/'c',max_attempts=1,controller_class=Failure).plan(q,None,None,None)
    assert not out['success'] and len(stats['attempts'])==1 and stats['persistent_channel_updates']==0

def test_lane_axis_adjustment_is_limited_to_visited_junctions(tmp_path):
    from arena_3d_v1.multires_pipeline import reference_route
    w=make_world(tmp_path)
    # Long semantic lanes extend beyond the cropped test map.
    w.features[1]=replace(w.features[1],coordinates=[[-20,0],[10,0],[10,6],[-20,6],[-20,0]])
    w.features[3]=replace(w.features[3],coordinates=[[16,0],[46,0],[46,6],[16,6],[16,0]])
    plan=SimpleNamespace(corridor_mask=w.safe,route_signature='scope')
    c=object.__new__(RecoveryController)
    x=np.linspace(2.,28.,261);xy=np.column_stack((x,3.+.2*np.sin(x)))
    original=reference_route(w,plan,xy);fixed=c._reference_route(w,plan,xy)
    outside=(x<6.)|(x>20.)
    assert np.allclose(fixed.poses[outside,2],original.poses[outside,2])
    assert np.any(abs(fixed.poses[:,2]-original.poses[:,2])>.01)
    ordinary=xy[x<8.]
    assert np.allclose(c._reference_route(w,plan,ordinary).poses,reference_route(w,plan,ordinary).poses)
