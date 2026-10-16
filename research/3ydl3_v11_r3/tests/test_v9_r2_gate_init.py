from types import SimpleNamespace as S
import math
import numpy as np
import pytest
from scipy import ndimage
from three_yd_v5.v9_gate_init import direct_gate,lattice_kernel,DirectStorageAuditor
from three_yd_v5.v9_storage import StorageAuditor


def binding(shape,res=.05,threshold=.382990205):return ((),shape,res,threshold)


@pytest.mark.parametrize('shape',[(1,1),(1,43),(31,1),(37,53),(115,91)])
def test_exact_all_cells_including_edges_rows_and_no_obstacles(shape):
    rng=np.random.default_rng(110);res=.05
    masks=[np.zeros(shape,np.int8),np.full(shape,100,np.int8),rng.choice([0,0,0,-1,1,100],shape).astype(np.int8)]
    for site in ((0,0),(shape[0]-1,shape[1]-1),(shape[0]//2,shape[1]//2)):
        a=np.zeros(shape,np.int8);a[site]=100;masks.append(a)
    rr,cc=np.indices(shape)
    for a in masks:
        b=binding(shape,res);gate,_=direct_gate(a,b,rows=7)
        old=ndimage.distance_transform_edt(a==0,sampling=res)
        assert np.array_equal(gate[rr,cc]<=gate.cutoff,old<=gate.cutoff)
        with pytest.raises(ValueError):gate.packed.flags.writeable=True


def test_threshold_lattice_ties_fall_back_instead_of_claiming_equivalence(monkeypatch):
    b=binding((50,60),threshold=math.sqrt(50)*.05-1e-12)
    with pytest.raises(ValueError,match='AMBIGUITY'):lattice_kernel(b)
    monkeypatch.setenv('THREEYD_V9_R1_MODE','S');monkeypatch.setenv('THREEYD_V9_R2_MODE','D')
    # Choose footprint whose exact parent threshold hits a lattice level.
    res=.05;radius=math.sqrt(50)*res-math.sqrt(2)*res/2-1e-12
    m=S(occupancy=np.zeros((50,60),np.int8),resolution=res,height=50,width=60)
    auditor=DirectStorageAuditor(S(hospital_map=m),source_commit='same',footprint=[(radius,0.)])
    assert not auditor.storage_diagnostics['direct_initialized']
    assert auditor.storage_diagnostics['direct_fallback']=='GATE_THRESHOLD_LATTICE_AMBIGUITY'


def test_direct_constructor_and_parent_full_audits_match(monkeypatch):
    from arena_evaluation.planner_benchmark.map_utils import HospitalMap
    from arena_evaluation.planner_benchmark.models import Query
    m=HospitalMap.__new__(HospitalMap);m.height=80;m.width=100;m.resolution=.05;m.origin=(0.,0.,0.)
    m.occupancy=np.zeros((80,100),np.int8);m.occupancy[[0,-1]]=100;m.occupancy[:,[0,-1]]=100
    m.occupancy[15:55,45:50]=-1;m.distance_m=ndimage.distance_transform_edt(m.occupancy==0)*.05
    ctx=S(hospital_map=m);q=Query('same',(1.,1.,0.),(4.,1.,0.))
    monkeypatch.setenv('THREEYD_V9_R1_MODE','S');monkeypatch.setenv('THREEYD_V9_R2_MODE','control')
    old=StorageAuditor(ctx,source_commit='same')
    monkeypatch.setenv('THREEYD_V9_R2_MODE','D');new=DirectStorageAuditor(ctx,source_commit='same')
    assert new.storage_diagnostics['direct_initialized']
    for y in (1.,2.,2.3):
        def points():return [{'x':x,'y':y,'yaw':0.,'source':'same','motion_direction':'forward','steering':0.,'planner_backend':'same','backend_version':'same'} for x in np.linspace(1.,4.,121)]
        a,b=[auditor.audit(q,points(),np.ones(m.occupancy.shape,bool)) for auditor in (old,new)]
        for field in ('metrics','path_hash','pose_hash','mask_hash','within_mask','sampled_pose_count','exact_footprint_check_count'):
            assert getattr(a,field)==getattr(b,field),field


def test_unknown_parent_constructor_runs_parent_and_preserves_audit_field(monkeypatch):
    from three_yd_v5 import v9_gate_init
    monkeypatch.setenv('THREEYD_V9_R1_MODE','S');monkeypatch.setenv('THREEYD_V9_R2_MODE','D')
    monkeypatch.setattr(v9_gate_init,'PARENT_INIT_SHA256','changed')
    m=S(occupancy=np.zeros((31,43),np.int8),resolution=.05,height=31,width=43)
    auditor=DirectStorageAuditor(S(hospital_map=m),source_commit='same')
    assert not auditor.storage_diagnostics['direct_initialized']
    assert auditor.storage_diagnostics['direct_fallback']=='PARENT_CONSTRUCTOR_CHANGED'
    rr,cc=np.indices(m.occupancy.shape);gate=auditor._distance_to_unsafe_m
    old=ndimage.distance_transform_edt(m.occupancy==0,sampling=m.resolution)
    assert np.array_equal(gate[rr,cc]<=gate.cutoff,old<=gate.cutoff)
