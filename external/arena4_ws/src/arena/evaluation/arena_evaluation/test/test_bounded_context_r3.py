import math
from types import SimpleNamespace
import cv2
import numpy as np
import pytest
from scipy import ndimage
from arena_evaluation.bounded_context_r3 import BoundedField
from arena_evaluation import unified_four_backends_smoke as legacy
from arena_evaluation.topology import preprocess_static_map


def hospital(occ):
    return SimpleNamespace(occupancy=occ,width=occ.shape[1],height=occ.shape[0],resolution=.05)


@pytest.mark.parametrize('seed',range(6))
def test_chamfer_and_inflated_fields_match_full_map(seed):
    rng=np.random.default_rng(seed);occ=rng.choice([0,-1,100],(257,311),p=[.96,.01,.03]).astype(np.int8)
    m=hospital(occ);rr,cc=np.indices(occ.shape)
    old=cv2.distanceTransform((occ!=100).astype(np.uint8),cv2.DIST_L2,5)*.05
    field=BoundedField(m,'occupied_distance',tile_cells=32,max_bytes=32768)
    np.testing.assert_array_equal(field[rr,cc],old)
    _,free,dist,_=preprocess_static_map(m,legacy.FOOTPRINT)
    for kind,expected in [('free',free),('inflated_distance',dist)]:
        f=BoundedField(m,kind,footprint=legacy.FOOTPRINT,tile_cells=32,max_bytes=32768)
        np.testing.assert_array_equal(f[rr,cc],expected)
        assert f.bytes<=f.max_bytes
    assert field.stats['max_window_cells']<occ.size
    with pytest.raises(MemoryError):np.asarray(field)


@pytest.mark.parametrize('mode',['unknown','sparse','wall','free'])
def test_audit_near_predicate_equals_global_edt_at_seams_and_map_edges(mode):
    occ=np.zeros((217,241),np.int8)
    if mode=='unknown':occ[::31,::29]=-1
    if mode=='sparse':occ[190,210]=100
    if mode=='wall':occ[:,65]=100
    m=hospital(occ);threshold=max(math.hypot(*v) for v in legacy.FOOTPRINT)+math.sqrt(2)*.05/2
    f=BoundedField(m,'audit',tile_cells=32,max_bytes=32768,safe_threshold=threshold)
    rr,cc=np.indices(occ.shape);old=ndimage.distance_transform_edt(occ==0,sampling=.05)
    got=f[rr,cc]
    np.testing.assert_array_equal(got<=threshold+1e-12,old<=threshold+1e-12)
    np.testing.assert_array_equal(got[old<=threshold],old[old<=threshold])
    assert f.bytes<=f.max_bytes


def test_unresolved_window_fails_instead_of_returning_invented_clearance():
    occ=np.zeros((1200,1200),np.int8);occ[0,0]=100
    f=BoundedField(hospital(occ),'occupied_distance',tile_cells=32,max_halo_cells=32)
    with pytest.raises(RuntimeError,match='UNCERTIFIED'):f[600,600]


def test_canonical_audit_result_matches_dense_on_real_polygon_collision_geometry(tmp_path):
    import copy
    from arena_evaluation.planner_benchmark.map_utils import HospitalMap
    from arena_evaluation.path_audit import PathAuditor
    from arena_evaluation.planner_benchmark.models import Query
    occ=np.zeros((160,180),np.int8);occ[:,0]=100;occ[0,:]=100;occ[70:90,85:90]=-1
    dist=cv2.distanceTransform((occ!=100).astype(np.uint8),cv2.DIST_L2,5)*.05
    m=HospitalMap(tmp_path/'map.yaml',tmp_path/'map.pgm',.05,(0.,0.,0.),180,160,occ,dist)
    dense=SimpleNamespace(hospital_map=m);bounded=SimpleNamespace(hospital_map=m,bounded_memory=True)
    old,new=PathAuditor(dense,source_commit='parity'),PathAuditor(bounded,source_commit='parity')
    for y in [2.,4.,.05]:
        points=[{'x':float(x),'y':y,'yaw':0.,'steering':0.,'motion_direction':'forward',
                 'source':'kinematic','planner_backend':'hybrid_astar','backend_version':'test'} for x in np.arange(1.,6.,.025)]
        q=SimpleNamespace(start=(1.,y,0.),goal=(float(points[-1]['x']),y,0.))
        a=old.audit(q,copy.deepcopy(points),occ==0);b=new.audit(q,copy.deepcopy(points),occ==0)
        assert a.metrics==b.metrics and a.final_valid_success==b.final_valid_success
        assert a.exact_footprint_check_count==b.exact_footprint_check_count
        assert a.pose_hash==b.pose_hash and a.mask_hash==b.mask_hash


def test_ready_baseline_is_query_independent_and_failure_never_yields_receipt():
    from arena_evaluation.exact_ack_r3 import ExactAckSmacSession,ExactAckFailure
    session=object.__new__(ExactAckSmacSession)
    session.ctx=SimpleNamespace(hospital_map=SimpleNamespace(height=5,width=7))
    calls=[]
    def update(mask):
        calls.append(mask.copy());assert session.request_id=='READY:closed-baseline'
        assert session._ready_ack_window_s==90.
        return {'costmap_update_acknowledged':True,'costmap_ack_mismatch_cells':0}
    session.update_local_mask=update
    receipt=session.prepare_ready_baseline()
    assert len(calls)==1 and not calls[0].any() and receipt['query_dependent'] is False
    assert not hasattr(session,'_ready_ack_window_s')
    session.begin_request('test-query',math.inf)
    assert session.last_exact_ack is None and session.active_publication is None
    def fail(mask):raise ExactAckFailure('EXACT_ACK_FAILED_CLOSED')
    session.update_local_mask=fail
    with pytest.raises(ExactAckFailure):session.prepare_ready_baseline()
    assert not hasattr(session,'_ready_ack_window_s')


def test_wide_open_endpoint_clearance_matches_frozen_chamfer():
    occ=np.zeros((1300,1800),np.int8);occ[:,0]=100;occ[:,-1]=100
    m=hospital(occ);f=BoundedField(m,'occupied_distance')
    expected=cv2.distanceTransform((occ!=100).astype(np.uint8),cv2.DIST_L2,5)*.05
    assert f[650,480]==expected[650,480]
    assert f[650,900]==expected[650,900]
    assert f.bytes<=f.max_bytes


def test_complete_master_snapshot_bandwidth_is_size_bounded():
    from arena_evaluation.exact_ack_r3 import master_snapshot_frequency
    for cells in [100,5_000_000,16_000_000,60_000_000,100_000_000]:
        hz=master_snapshot_frequency(cells)
        assert hz<=40. and hz*cells<=300_000_000.
    assert master_snapshot_frequency(60_000_000)==5.
    with pytest.raises(ValueError):master_snapshot_frequency(0)


def test_ack_subscription_is_retired_on_success_and_failure():
    from arena_evaluation.exact_ack_r3 import ExactAckSmacSession,ExactAckFailure
    from collections import deque
    s=object.__new__(ExactAckSmacSession);destroyed=[]
    s.client=SimpleNamespace(node=SimpleNamespace(destroy_subscription=destroyed.append))
    s._atomic_buffer=SimpleNamespace(frames=deque([b'old full snapshot']))
    for fail in [False,True]:
        s._atomic_subscription='reader'
        def transaction(*a,**kw):
            if fail:raise ExactAckFailure('EXACT_ACK_FAILED_CLOSED')
            return {'verified':True}
        s._update_local_mask_transaction=transaction
        if fail:
            with pytest.raises(ExactAckFailure):s.update_local_mask(None)
        else:assert s.update_local_mask(None)=={'verified':True}
        assert s._atomic_subscription is None and not s._atomic_buffer.frames
    assert destroyed==['reader','reader']
