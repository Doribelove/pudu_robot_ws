from types import SimpleNamespace as S
import copy,math
import numpy as np
import pytest
from scipy import ndimage
from arena_evaluation.path_audit import PathAuditor
from arena_evaluation.semantic_costmap_composer import internal_soft_to_occupancy,occupancy_to_static_layer
from three_yd_v5.v9_storage import AuditClearanceGate,StorageAuditor,audit_binding,occupancy_on_free_support,static_lut
from three_yd_v5.r4_tiles import TiledSoft


def binding(shape=(73,89)):
    return (((.265,.225),(-.265,.225),(-.265,-.225),(.265,-.225)),shape,.05,.385)


def test_packed_gate_preserves_every_cell_decision_and_bounds():
    rng=np.random.default_rng(132);b=binding();cutoff=b[-1]+1.e-12
    a=rng.uniform(0,2,b[1]);a[0,:6]=[0,cutoff,np.nextafter(cutoff,np.inf),np.nan,np.inf,-np.inf]
    gate=AuditClearanceGate(a,b,rows=7)
    rr,cc=np.indices(a.shape);values=gate[rr,cc]
    expected=~np.isfinite(a)|(a<=cutoff)
    actual=~np.isfinite(values)|(values<=cutoff)
    assert np.array_equal(actual,expected)
    assert gate.nbytes<a.nbytes/50
    assert gate[-1,-1]==gate[a.shape[0]-1,a.shape[1]-1]
    with pytest.raises(IndexError):gate[a.shape[0],0]
    with pytest.raises(TypeError):np.asarray(gate)
    with pytest.raises(ValueError):gate.packed.flags.writeable=True


def test_original_audit_output_and_exact_collision_selection_are_identical(monkeypatch):
    from arena_evaluation.planner_benchmark.models import Query
    from arena_evaluation.planner_benchmark.map_utils import HospitalMap
    rng=np.random.default_rng(22)
    m=HospitalMap.__new__(HospitalMap);m.height=80;m.width=100;m.resolution=.05;m.origin=(0.,0.,0.)
    m.occupancy=np.zeros((80,100),np.int8);m.occupancy[[0,-1]]=100;m.occupancy[:,[0,-1]]=100
    m.occupancy[15:55,45:50]=100;m.distance_m=ndimage.distance_transform_edt(m.occupancy!=100)*.05
    ctx=S(hospital_map=m);q=Query('audit',(1.,1.,0.),(4.,1.,0.))
    points=[{'x':x,'y':1.,'yaw':0.,'source':'test','motion_direction':'forward','steering':0.,'planner_backend':'test','backend_version':'frozen'} for x in np.linspace(1.,4.,121)]
    monkeypatch.setenv('THREEYD_V9_R1_MODE','control');old=PathAuditor(ctx,source_commit='same')
    monkeypatch.setenv('THREEYD_V9_R1_MODE','S');new=StorageAuditor(ctx,source_commit='same')
    for y in (1.,2.,2.3):
        poses=copy.deepcopy(points)
        for p in poses:p['y']=y
        left,right=[a.audit(q,copy.deepcopy(poses),np.ones(m.occupancy.shape,bool)) for a in (old,new)]
        for field in ('metrics','path_hash','pose_hash','mask_hash','within_mask','sampled_pose_count','exact_footprint_check_count'):
            assert getattr(left,field)==getattr(right,field),field
    new.footprint[0][0]+=.01
    with pytest.raises(ValueError,match='GEOMETRY_CHANGED'):new.audit(q,points,None)


def test_supported_occupancy_matches_original_with_dense_and_tiled_soft():
    rng=np.random.default_rng(44);shape=(137,193)
    source=rng.choice([-1,0,20,99,100],shape).astype(np.int8);hard=rng.random(shape)<.83
    soft=rng.uniform(0,252,shape).astype(np.float32)
    tiled=TiledSoft(shape,140,np.ones(shape,bool),[((0,137,0,193),soft)])
    for field in (soft,tiled):
        old=np.maximum(source.astype(np.int16),internal_soft_to_occupancy(np.asarray(field))).astype(np.int8)
        old[source<0]=-1;old[hard]=100
        actual=occupancy_on_free_support(source,field,hard,rows=13)
        assert np.array_equal(old,actual)
        assert np.array_equal(static_lut(actual),occupancy_to_static_layer(actual))


def test_static_table_matches_all_int8_values():
    a=np.arange(256,dtype=np.uint8).view(np.int8).reshape(16,16)
    assert np.array_equal(static_lut(a),occupancy_to_static_layer(a))


def test_streamed_properties_preserve_live_mutation_and_immutable_hashes(monkeypatch):
    from arena_evaluation.semantic_costmap_composer import SemanticCostmap
    from arena_evaluation.semantic_rasterizer import grid_hash
    from three_yd_v5.v9_maps import seal_grid
    a=np.zeros((41,59),np.int8);master=a.view(np.uint8)
    grid=SemanticCostmap(a,a,master,a,master>0,master>0,'p','s','f',True,{})
    monkeypatch.setenv('THREEYD_V9_MODE','control');monkeypatch.setenv('THREEYD_V9_R1_MODE','S')
    current=seal_grid(grid);alias=a.view()
    for n in range(3):
        assert current.expected_grid_hash==grid_hash(a) and current.expected_master_hash==grid_hash(master)
        alias[n,n]=n+1
    monkeypatch.setenv('THREEYD_V9_MODE','A')
    sealed=seal_grid(grid);before=sealed.expected_grid_hash;alias[0,1]=9
    assert sealed.expected_grid_hash==before and sealed.expected_grid_hash!=grid_hash(a)
