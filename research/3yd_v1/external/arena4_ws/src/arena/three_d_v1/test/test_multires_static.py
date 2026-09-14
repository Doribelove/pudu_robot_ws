import json, math
import numpy as np
import pytest
from test_semantic_dual_map import make_world
from arena_3d_v1.multires_grid import GridView, aggregate, make_roi, guide_astar
from arena_3d_v1.multires_topology import SemanticTopology
from arena_3d_v1.multires_pipeline import MultiresController
from arena_evaluation.planner_benchmark.models import Query

def test_origin_aggregation_and_padded_border():
    fine=np.ones((5,7),bool)
    coarse=aggregate(fine,3,all_free=True,pad_value=False)
    assert coarse.tolist()==[[False,False,False],[True,True,False]]
    occupied=np.zeros((5,7),bool);occupied[-2,1]=True
    assert aggregate(occupied,3).tolist()==[[False,False,False],[True,False,False]]

def test_coordinate_roundtrip_and_preserved_fine_obstacles(tmp_path):
    w=make_world(tmp_path,'pillar');v=GridView(w,3)
    for rc in [(5,5),(20,40),(50,50)]:
        xy=v.world_points([rc[0]],[rc[1]])
        r,c=v.cells(xy);assert (int(r[0]),int(c[0]))==rc
        fr,fc=w.cells(xy);assert np.allclose(w.world_points(fr,fc),xy)
    assert np.array_equal(v.hard,aggregate(w.hard,3,pad_value=True))
    assert np.array_equal(v.safe,aggregate(w.safe,3,all_free=True,pad_value=False))

def test_topology_forbidden_turn_and_one_way(tmp_path):
    w=make_world(tmp_path,rules={'forbidden_transitions':[['left','junction','right']]})
    g=SemanticTopology(w,tmp_path/'graph')
    with pytest.raises(RuntimeError,match='TOPOLOGY_NO_ROUTE'):
        g.plan(Query('forbidden',(3.,3.,0.),(27.,3.,0.),seed=0))
    w=make_world(tmp_path,rules={'one_way':[{'semantic_id':'left','yaw':0.}]})
    g=SemanticTopology(w,tmp_path/'graph2')
    assert g.plan(Query('forward',(2.,3.,0.),(8.,3.,0.),seed=0))
    with pytest.raises(RuntimeError,match='TOPOLOGY_NO_ROUTE'):
        g.plan(Query('reverse',(8.,3.,math.pi),(2.,3.,math.pi),seed=0))

def test_sparse_graph_and_cache_integrity(tmp_path):
    w=make_world(tmp_path);g=SemanticTopology(w,tmp_path/'g')
    q=Query('q',(2.,3.,0.),(28.,3.,0.),seed=0)
    a=g.plan(q);b=g.plan(q)
    assert b.diagnostics['cache_hit'] and a.signature==b.signature
    assert not list((tmp_path/'g').rglob('geometry.npz'))
    restored=SemanticTopology(w,tmp_path/'g');assert restored.cache_hit and restored.plan(q).signature==a.signature
    path=next((tmp_path/'g').rglob('graph.json'));meta=json.loads(path.read_text());meta['regions_hash']='bad';path.write_text(json.dumps(meta))
    with pytest.raises(ValueError,match='CACHE_MISMATCH'):SemanticTopology(w,tmp_path/'g')

def test_multires_weighted_oracle_and_fine_projection(tmp_path):
    w=make_world(tmp_path);g=SemanticTopology(w,tmp_path/'g')
    q=Query('q',(2.,3.,0.),(28.,3.,0.),seed=0);r=g.plan(q)
    c=MultiresController(w,g,r,cache_root=tmp_path/'state',factor=3,verify_l2_oracle=True)
    assert c.initial_l2_result.success and not c.fallbacks
    assert c.initial_l2_result.oracle_cost_error<1e-8
    first=c.l2.binding_hash
    d=MultiresController(w,g,r,cache_root=tmp_path/'state',factor=3,side='left',verify_l2_oracle=True)
    assert d.l2.binding_hash!=first
    assert np.array_equal(c._target_mask(),d._target_mask())
    e=MultiresController(w,g,r,cache_root=tmp_path/'state',factor=3)
    assert e.timing['reference_cache_hit'] and e.initial_l2_result.diagnostics['activation']['state_cache_hit']
    assert np.array_equal(c.reference_cells,e.reference_cells)

def test_narrow_channel_falls_back_without_closing_fine_map(tmp_path):
    from PIL import Image
    import yaml
    from arena_evaluation.semantic_map import SemanticMapV1,SemanticFeature
    from arena_3d_v1.semantic_world import SemanticWorld,json_write
    width,height=10.,.95;grid=np.zeros((59,240),np.uint8);grid[20:39,20:220]=255
    Image.fromarray(grid).save(tmp_path/'map.pgm')
    (tmp_path/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':.05,'origin':[-1.,-1.,0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
    f=SemanticFeature('narrow','lane','polygon',[[0,0],[width,0],[width,height],[0,height],[0,0]],soft=True)
    sm=SemanticMapV1('map',.05,(-1.,-1.,0.),240,59,'narrow',[f]);json_write(tmp_path/'semantic.json',sm.to_dict())
    w=SemanticWorld(tmp_path/'map.yaml',tmp_path/'semantic.json');g=SemanticTopology(w,tmp_path/'g')
    q=Query('narrow',(1.,.475,0.),(9.,.475,0.),seed=0);r=g.plan(q)
    c=MultiresController(w,g,r,cache_root=tmp_path/'l2',factor=3)
    assert c.fallbacks and c.view.factor==1 and c.initial_l2_result.success
    from arena_3d_v1.semantic_world import densify
    assert w.checker.check(densify([[1.,.475,0.],[9.,.475,0.]]))

def test_same_hard_corridor_for_both_resolutions(tmp_path):
    w=make_world(tmp_path);g=SemanticTopology(w,tmp_path/'g');q=Query('q',(2.,3.,0.),(28.,3.,0.),seed=0);r=g.plan(q)
    a=MultiresController(w,g,r,cache_root=tmp_path/'a',factor=1)
    b=MultiresController(w,g,r,cache_root=tmp_path/'b',factor=3)
    assert np.array_equal(a._target_mask(),b._target_mask())

def test_reference_integrity_error_cannot_be_hidden_by_fine_fallback(tmp_path):
    w=make_world(tmp_path);g=SemanticTopology(w,tmp_path/'g')
    q=Query('q',(2.,3.,0.),(28.,3.,0.),seed=0);r=g.plan(q)
    c=MultiresController(w,g,r,cache_root=tmp_path/'state',factor=3)
    assert c.reference_plan.world_key==w.key and not c.reference_plan.world_xy.flags.writeable
    p=next((tmp_path/'state/reference').rglob('field.json'));m=json.loads(p.read_text());m['arrays_hash']='broken';p.write_text(json.dumps(m))
    with pytest.raises(ValueError,match='REFERENCE_CACHE_CORRUPT'):
        MultiresController(w,g,r,cache_root=tmp_path/'state',factor=3)

def test_topology_cache_integer_key_order_is_stable(tmp_path):
    from arena_evaluation.semantic_map import canonical_hash
    w=make_world(tmp_path);g=SemanticTopology(w,tmp_path/'g')
    file=next((tmp_path/'g').rglob('graph.json'));meta=json.loads(file.read_text());meta.pop('content_hash')
    meta['parents']={int(k):v for k,v in meta['parents'].items()}
    # Same valid graph, plus unused region IDs that expose 2/10 key ordering.
    meta['parents'].update({2:1,10:1,100:1});meta['content_hash']=canonical_hash(meta)
    file.write_text(json.dumps(meta))
    restored=SemanticTopology(w,tmp_path/'g');assert restored.cache_hit
    assert restored.parent_semantics[100]==1
