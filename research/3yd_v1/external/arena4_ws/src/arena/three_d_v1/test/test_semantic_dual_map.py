import math
from types import SimpleNamespace
import numpy as np
import pytest
from arena_3d_v1.semantic_world import SweptChecker,WORK
from arena_3d_v1.semantic_l2 import SemanticL2Lifecycle,weighted_astar
from arena_3d_v1.l2_incremental import CorridorROI
from arena_3d_v1.pipeline import L1Plan

def make_world(tmp_path,kind='junction',rules=None):
 from PIL import Image
 import yaml
 from arena_evaluation.semantic_map import SemanticMapV1,SemanticFeature
 from arena_3d_v1.semantic_world import SemanticWorld,json_write
 w,h=(30,6) if kind=='junction' else (20,10)
 grid=np.zeros((int((h+2)/.05),int((w+2)/.05)),np.uint8);grid[20:-20,20:-20]=255
 if kind=='pillar':grid[100:140,200:240]=0
 Image.fromarray(grid).save(tmp_path/'map.pgm')
 (tmp_path/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':.05,'origin':[-1.,-1.,0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
 areas=[('left','lane',0,10),('junction','junction_area',10,16),('right','lane',16,30)] if kind=='junction' else [('parking','parking_area',0,w)]
 features=[SemanticFeature(k,cl,'polygon',[[a,0],[b,0],[b,h],[a,h],[a,0]],soft=cl=='lane') for k,cl,a,b in areas]
 sm=SemanticMapV1('map',.05,(-1.,-1.,0.),grid.shape[1],grid.shape[0],'controlled',features)
 json_write(tmp_path/'semantic.json',sm.to_dict())
 return SemanticWorld(tmp_path/'map.yaml',tmp_path/'semantic.json',rules=rules)

def roi():
 safe=np.ones((24,40),bool)
 p=L1Plan(safe,safe,(12,2),(12,37),'testmap',(0.,0.,0.),.05,'testgraph',('r0',),'vehicle','route')
 return CorridorROI.from_global(safe,safe,p.start_cell,p.goal_cell,binding_fields=p.binding_fields())

def test_weighted_dstar_astar_and_recovery(tmp_path):
 field=np.zeros((24,40));field[:12]=1
 manager=SemanticL2Lifecycle(tmp_path,field)
 planner,result,_=manager.activate(roi(),verify_oracle=True)
 assert result.success and result.oracle_cost_error<1e-9
 updated=planner.update([(12,20)],verify_oracle=True)
 assert updated.success and updated.oracle_cost_error<1e-9
 # A cost decrease must be processed even away from an existing path.
 changed=np.ones(planner.state.geometry.state_count)
 cells=np.asarray([planner.geometry.global_cell(i) for i in range(planner.geometry.state_count)])
 changed[cells[:,0]<10]=0
 planner.change_cost(changed)
 ids,oracle=weighted_astar(planner.state)
 assert abs(planner.state.path_cost(planner.state.current_path_ids)-oracle.cost)<1e-9
 recovered=planner.update([],force_cold_astar=True,verify_oracle=True)
 assert recovered.success and recovered.oracle_cost_error<1e-9

def test_weight_bindings_do_not_reuse_state(tmp_path):
 f=np.zeros((24,40));m=SemanticL2Lifecycle(tmp_path,f,1)
 a,_,_=m.activate(roi());key=a.binding_hash
 m.weight=2.;b,_,t=m.activate(roi())
 assert b.binding_hash!=key and not t.as_dict()['state_cache_hit']

def test_nonfinite_and_negative_cost_rejected(tmp_path):
 for v in [math.nan,-1.]:
  m=SemanticL2Lifecycle(tmp_path,np.full((24,40),v))
  with pytest.raises(ValueError):m.activate(roi())

def test_swept_collision_between_clear_endpoints():
 hard=np.zeros((80,100),bool);hard[39:41,49:51]=True
 c=SweptChecker(hard,.05,(0.,0.,0.))
 assert c.check([[1.,2.,0.]]) and c.check([[4.,2.,0.]])
 assert not c.check([[1.,2.,0.],[4.,2.,0.]])

def test_corner_contact_is_collision_and_unknown_is_hard():
 hard=np.zeros((80,100),bool);hard[39,40]=True
 c=SweptChecker(hard,.05,(0.,0.,0.))
 assert not c.check([[2.-.265,2.,0.]])
 assert c.check([[1.,1.,0.]])

def test_rotation_sweep_not_just_endpoint_points():
 hard=np.zeros((100,100),bool);hard[43,54]=True
 c=SweptChecker(hard,.05,(0.,0.,0.))
 assert not c.check([[2.5,2.5,0.],[2.5,2.5,math.pi/2]])

def test_rule_changes_usable_graph_with_identical_occupancy(tmp_path):
 from arena_3d_v1.semantic_graph import SemanticPoseGraph
 from arena_evaluation.planner_benchmark.models import Query
 w=make_world(tmp_path);a=SemanticPoseGraph(w,tmp_path/'g');a.prepare()
 q=Query('straight',(3.,3.,0.),(27.,3.,0.),seed=0)
 assert a.plan(q) is not None
 w2=make_world(tmp_path,rules={'forbidden_transitions':[['left','junction','right']]})
 b=SemanticPoseGraph(w2,tmp_path/'g');b.prepare()
 assert np.array_equal(w.hard,w2.hard) and len(b.edges)<len(a.edges)
 assert any(r['status']=='FORBIDDEN_RULE' for r in b.records)
 assert b.plan(q) is None

def test_internal_pillar_nodes_and_pose_splices(tmp_path):
 from arena_3d_v1.semantic_graph import SemanticPoseGraph
 w=make_world(tmp_path,'pillar');g=SemanticPoseGraph(w,tmp_path/'g');g.prepare()
 assert any(n.semantic_from==n.semantic_to for n in g.nodes.values())
 assert g.edges
 for e in g.edges.values():
  assert np.allclose(e.path[0],g.nodes[e.source].pose,atol=1e-8)
  assert np.allclose(e.path[-1],g.nodes[e.target].pose,atol=1e-8)
  assert g.nodes[e.source].to_region==g.nodes[e.target].from_region
  assert w.checker.check(e.path)

def test_ordered_matching_abstains_on_repeated_visit(tmp_path):
 from arena_3d_v1.semantic_reference import ordered_match
 w=make_world(tmp_path,'pillar')
 x=np.linspace(2,7,51);p=np.column_stack([x,np.full(len(x),2.),np.zeros(len(x))])
 back=p[::-1].copy();back[:,2]=math.pi
 route=SimpleNamespace(edges=[SimpleNamespace(path=p),SimpleNamespace(path=back)])
 cell=w.map.world_to_cell(4,2)
 _,_,ix,d,amb=ordered_match(w,route,np.array([cell[0]]),np.array([cell[1]]))
 assert ix[0]>=0 and amb[0]

def test_direction_rule_and_temporary_endpoints_do_not_mutate_graph(tmp_path):
 from arena_3d_v1.semantic_graph import SemanticPoseGraph
 from arena_evaluation.planner_benchmark.models import Query
 w=make_world(tmp_path,rules={'one_way':[{'semantic_id':'left','yaw':0.}]});g=SemanticPoseGraph(w,tmp_path/'g');g.prepare()
 n,e=len(g.nodes),len(g.edges)
 assert g.plan(Query('ok',(2.,3.,0.),(8.,3.,0.),seed=0)) is not None
 assert g.plan(Query('reverse',(8.,3.,math.pi),(2.,3.,math.pi),seed=0)) is None
 assert (n,e)==(len(g.nodes),len(g.edges))
 assert all(not v.temporary for v in g.nodes.values())

def test_unverified_connections_never_admitted(tmp_path):
 from arena_3d_v1.semantic_graph import SemanticPoseGraph
 w=make_world(tmp_path);g=SemanticPoseGraph(w,tmp_path/'g',connection_timeout=0);g.prepare()
 assert not g.edges and any(r['status']=='UNVERIFIED_TIMEOUT' for r in g.records)

def test_reference_only_and_off_l2_dynamic_changes_trigger_l3(tmp_path):
 from arena_3d_v1.semantic_graph import SemanticPoseGraph
 from arena_3d_v1.semantic_pipeline import SemanticR1Controller
 from arena_evaluation.planner_benchmark.models import Query
 from arena_evaluation.dynamic_snapshot import DynamicSnapshot
 w=make_world(tmp_path);g=SemanticPoseGraph(w,tmp_path/'g');g.prepare();q=Query('x',(3.,3.,0.),(27.,3.,0.),seed=0)
 r=g.plan(q);c=SemanticR1Controller(w,r,cache_root=tmp_path/'l2')
 _,cost=c.prepare_l3();c.acknowledged_reference_key=cost.policy_hash;c.reference_changed=False
 # An off-path obstacle must still trigger L3 because L3 can deviate from L2.
 cell=w.map.world_to_cell(6.,4.5)
 def snap(i,occupied):return DynamicSnapshot.from_cells(str(i),occupied,timestamp=float(i),map_version=w.map.sha256,map_shape=w.hard.shape)
 c.process_snapshot(snap(1,[cell]),now=1.);s=c.process_snapshot(snap(2,[cell]),now=2.)
 assert s.l3_required and s.dirty_roi.closed_cells>0
 c.change_preference('left',2.)
 s=c.process_snapshot(snap(3,[cell]),now=3.)
 assert s.l3_required

def test_dynamic_edge_exclusion_and_recovery_without_graph_mutation(tmp_path):
 from arena_3d_v1.semantic_graph import SemanticPoseGraph
 from arena_evaluation.planner_benchmark.models import Query
 w=make_world(tmp_path);g=SemanticPoseGraph(w,tmp_path/'g');g.prepare()
 q=Query('x',(3.,3.,0.),(27.,3.,0.),seed=0);r=g.plan(q);assert r
 before=(len(g.nodes),len(g.edges));col=w.map.world_to_cell(12.,3.)[1]
 blocked=[(i,col) for i in range(w.map.height)]
 assert g.plan(q,blocked_cells=blocked) is None
 assert g.last_query_diagnostics['dynamic_rejected_edges']>0
 assert g.plan(q) is not None and before==(len(g.nodes),len(g.edges))

def test_corrupt_semantic_and_weighted_caches_rejected(tmp_path):
 from arena_3d_v1.semantic_world import SemanticWorld
 w=make_world(tmp_path);cache=tmp_path/'cache'
 w=SemanticWorld(tmp_path/'map.yaml',tmp_path/'semantic.json',cache_root=cache)
 next(cache.glob('*.npz')).write_bytes(b'corrupt')
 with pytest.raises(ValueError,match='integrity'):SemanticWorld(tmp_path/'map.yaml',tmp_path/'semantic.json',cache_root=cache)
 m=SemanticL2Lifecycle(tmp_path/'l2',np.zeros((24,40)));p,_,_=m.activate(roi());key=p.binding_hash
 payload,_=m.state_cache._paths(key);payload.write_bytes(b'corrupt');m.clear()
 p,r,t=m.activate(roi(),verify_oracle=True)
 assert r.success and t.as_dict()['state_reject']=='CONTENT_HASH_MISMATCH'

def test_parallel_crossing_and_dense_repeated_matching(tmp_path):
 from arena_3d_v1.semantic_reference import ordered_match
 w=make_world(tmp_path,'pillar');x=np.linspace(2,8,601)
 p=np.column_stack([x,np.full(len(x),2.),np.zeros(len(x))]);back=p[::-1].copy();back[:,1]=2.2;back[:,2]=math.pi
 route=SimpleNamespace(edges=[SimpleNamespace(path=p),SimpleNamespace(path=back)])
 rc=w.map.world_to_cell(4,2.05);_,_,ix,d,amb=ordered_match(w,route,np.array([rc[0]]),np.array([rc[1]]));assert amb[0]
 crossing=np.column_stack([np.full(len(x),4.),x-2.,np.full(len(x),math.pi/2)])
 route=SimpleNamespace(edges=[SimpleNamespace(path=p),SimpleNamespace(path=crossing)])
 rc=w.map.world_to_cell(4,2.);_,_,ix,d,amb=ordered_match(w,route,np.array([rc[0]]),np.array([rc[1]]));assert amb[0]

def test_forbidden_turn_history_survives_internal_splices(tmp_path):
 import json
 from arena_3d_v1.semantic_world import SemanticWorld
 from arena_3d_v1.semantic_graph import SemanticPoseGraph
 from arena_evaluation.planner_benchmark.models import Query
 make_world(tmp_path)
 p=tmp_path/'semantic.json';data=json.loads(p.read_text())
 for feature in data['features']:
  if feature['semantic_id']=='left':feature['coordinates']=[[0,0],[8,0],[8,6],[0,6],[0,0]]
  elif feature['semantic_id']=='junction':feature['coordinates']=[[8,0],[22,0],[22,6],[8,6],[8,0]]
  else:feature['coordinates']=[[22,0],[30,0],[30,6],[22,6],[22,0]]
 data.pop('semantic_map_hash',None);p.write_text(json.dumps(data))
 w=SemanticWorld(tmp_path/'map.yaml',p,rules={'forbidden_transitions':[['left','junction','right']]})
 g=SemanticPoseGraph(w,tmp_path/'g');g.prepare()
 assert any(n.from_region==n.to_region and w.features[n.semantic_to].semantic_id=='junction' for n in g.nodes.values())
 assert g.plan(Query('banned',(3.,3.,0.),(27.,3.,0.),seed=0)) is None

def test_map_config_vehicle_and_semantic_versions_invalidate_both_maps(tmp_path):
 import yaml,json
 from arena_3d_v1.semantic_world import SemanticWorld
 from arena_3d_v1.semantic_graph import SemanticPoseGraph
 w=make_world(tmp_path);g=SemanticPoseGraph(w,tmp_path/'g')
 larger=SemanticWorld(tmp_path/'map.yaml',tmp_path/'semantic.json',vehicle=(.40,.30,.50))
 assert larger.key!=w.key and not np.array_equal(larger.safe,w.safe)
 assert SemanticPoseGraph(larger,tmp_path/'g').key!=g.key
 mp=tmp_path/'map.yaml';cfg=yaml.safe_load(mp.read_text());cfg['free_thresh']=.10;mp.write_text(yaml.safe_dump(cfg))
 w2=SemanticWorld(mp,tmp_path/'semantic.json')
 assert np.array_equal(w2.map.occupancy,w.map.occupancy) and w2.key!=w.key
 assert SemanticPoseGraph(w2,tmp_path/'g').key!=g.key

def test_graph_cache_corruption_is_rejected(tmp_path):
 from arena_3d_v1.semantic_graph import SemanticPoseGraph
 w=make_world(tmp_path);g=SemanticPoseGraph(w,tmp_path/'g');g.prepare()
 (tmp_path/'g'/g.key/'geometry.npz').write_bytes(b'corrupt')
 with pytest.raises(ValueError,match='integrity'):SemanticPoseGraph(w,tmp_path/'g')

def test_changed_resident_cost_does_not_alias_original_binding(tmp_path):
 m=SemanticL2Lifecycle(tmp_path,np.zeros((24,40)));p,r,_=m.activate(roi())
 original=list(p.path_global)
 altered=np.linspace(0.,1.,p.geometry.state_count);p.change_cost(altered)
 restored,r,t=m.activate(roi(),verify_oracle=True)
 assert r.success and list(restored.path_global)==original
 assert not t.as_dict()['active_hit']
 assert np.all(restored.state.potential==0)
