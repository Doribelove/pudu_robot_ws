from types import SimpleNamespace
import cv2
import numpy as np
import pytest
from three_yd_v5.r11_fields import reference_soft,occupancy_chunked,static_chunked,bounded_inflation
from arena_evaluation.semantic_costmap_composer import internal_soft_to_occupancy,occupancy_to_static_layer,SemanticCostmap
from arena_evaluation.semantic_costmap_r2 import pinned_nav2_effective_master

def test_tiled_conversion_all_values_and_rounding_edges():
    rng=np.random.default_rng(611)
    soft=rng.uniform(-1,253,(213,317)).astype(np.float32)
    soft[:101,:101]=np.linspace(0,252,10201).reshape(101,101)
    source=rng.integers(-1,101,soft.shape,dtype=np.int8)
    expected=np.maximum(source.astype(np.int16),internal_soft_to_occupancy(soft)).astype(np.int8)
    expected[source<0]=-1
    assert np.array_equal(occupancy_chunked(source,soft),expected)
    assert np.array_equal(static_chunked(expected),occupancy_to_static_layer(expected))

def test_roi_soft_is_bit_exact_at_edges_and_saturation():
    rng=np.random.default_rng(12);shape=(431,709)
    for ref in [np.array([[0,0],[200,400],[430,708]]),np.array([[75,200],[120,270]]),np.array([[430,0],[430,708]])]:
        for resolution in (.05,.15):
            drivable=rng.random(shape)>.2;seed=np.ones(shape,np.uint8)
            cv2.polylines(seed,[ref[:,::-1].astype(np.int32)],False,0,1)
            distance=cv2.distanceTransform(seed,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*resolution
            original=(140*np.minimum(1.,distance/2.)).astype(np.float32);original[~drivable]=200.
            actual,_=reference_soft(shape,ref,resolution,drivable)
            assert np.array_equal(original,actual)

def test_bounded_master_is_bit_exact_including_unknown_and_boundary():
    rng=np.random.default_rng(37)
    for at_edge in (False,True):
        for _ in range(12):
            a=np.full((151,237),254,np.uint8)
            y,x=(0,0) if at_edge else (45,70)
            a[y:y+60,x:x+90]=rng.integers(0,256,(60,90),dtype=np.uint8)
            original=pinned_nav2_effective_master(a,resolution=.05,inflation_radius_m=.55,cost_scaling_factor=3.,inscribed_radius_m=.225)
            actual,_=bounded_inflation(a,.05)
            assert np.array_equal(actual,original)

def test_full_composition_hash_and_all_fields_identical(monkeypatch):
    from three_yd_v5.fast_costmap import compose_bounded
    rng=np.random.default_rng(6);shape=(231,349)
    w=SimpleNamespace(hard=rng.random(shape)<.05,map=SimpleNamespace(occupancy=np.zeros(shape,np.int8),resolution=.05),semantic=SimpleNamespace(semantic_map_hash='map'))
    c=SimpleNamespace(world=w,reference_cells=np.array([[90,120],[110,180]]),global_drivable=np.ones(shape,bool),plan=SimpleNamespace(corridor_mask=np.ones(shape,bool)),dynamic=None,task={'signature':'route'})
    mask=np.zeros(shape,bool);mask[60:140,90:210]=True
    monkeypatch.setenv('THREEYD_R11_MODE','control');old=compose_bounded(c,mask)
    for mode in ('A','B','AB'):
        monkeypatch.setenv('THREEYD_R11_MODE',mode);new=compose_bounded(c,mask)
        for key in ('internal_cost','occupancy_grid','expected_master_cost','soft_cost','hard_semantic_mask','affected_mask'):
            assert np.array_equal(getattr(old,key),getattr(new,key)),key
        assert old.policy_hash==new.policy_hash
        assert old.expected_grid_hash==new.expected_grid_hash
        assert old.expected_master_hash==new.expected_master_hash

def ack_session():
    from three_yd_v5.performance_session import PerformanceSession
    s=object.__new__(PerformanceSession);a=np.zeros((4,5),np.uint8)
    s._semantic_costmap=SemanticCostmap(a,a.astype(np.int8),a,a.astype(np.float32),a.astype(bool),np.ones(a.shape,bool),'policy','map','reference',True,{})
    s.exact_stable_observations=2;s._semantic_publication_sequence=0;s._last_server_update_time_ns=100;s._last_exact_expected_master=a.copy()
    s._active_publication_baseline_timestamp_ns=-1;s._active_publication_bbox=(0,0,0,0)
    s._semantic_roi_sequence=s._costmap_ack_sequence=0;s._exact_ack_cache={};s._semantic_ack_cache={}
    s._append_ack_trace=lambda *args:None;s.client=None;s.costmap_ack_timeout_s=.01
    s._current_grid=a.astype(np.int8);s.current_query_id='test';s.tile_interval_s=0;s.roi_publish_pacing_s=0
    s.tile_events=[];s._exact_repair_context=None
    return s

def test_identical_policy_publication_reasserts_and_versions(monkeypatch):
    from nav_msgs.msg import OccupancyGrid
    from map_msgs.msg import OccupancyGridUpdate
    from builtin_interfaces.msg import Time
    monkeypatch.setenv('THREEYD_R11_MODE','A');monkeypatch.setenv('THREEYD_R10_MODE','ABC')
    s=ack_session();sent=[]
    s._local_update_publisher=SimpleNamespace(get_subscription_count=lambda:1,publish=sent.append)
    s.client=SimpleNamespace(node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time()))))
    s.OccupancyGridUpdate=OccupancyGridUpdate
    s._publish_dirty_roi(s._current_grid,np.zeros_like(s._current_grid,bool))
    assert s._semantic_publication_sequence==1
    assert s._active_publication_baseline_timestamp_ns==100
    assert s._active_publication_bbox==(0,0,1,1)
    assert len(sent)==1 and len(sent[0].data)==1

def test_old_ack_timestamp_is_rejected_and_two_new_observations_required():
    from arena_evaluation.semantic_smac_session_r2 import ExactSemanticSmacSessionR2
    s=ack_session();changed=np.ones_like(s._current_grid,bool);s._begin_publication(changed)
    calls=[]
    def snapshot(deadline):
        calls.append(1);return np.zeros_like(s._current_grid,np.uint8),100
    s._server_costmap_snapshot=snapshot
    result=ExactSemanticSmacSessionR2._wait_for_costmap_ack(s,s._current_grid,changed,timeout_s=.005)
    assert not result['costmap_update_acknowledged']
    calls.clear()
    def fresh(deadline):
        calls.append(1);return np.zeros_like(s._current_grid,np.uint8),101
    s._server_costmap_snapshot=fresh;s.exact_stable_observations=2
    result=ExactSemanticSmacSessionR2._wait_for_costmap_ack(s,s._current_grid,changed,timeout_s=.1)
    assert result['costmap_update_acknowledged'] and len(calls)==2
    assert result['semantic_publication_sequence']==1

def test_crop_origin_preserves_world_cells_and_support():
    from three_yd_v5.r11_crop import crop_origin
    from arena_evaluation.planner_benchmark.map_utils import HospitalMap
    from pathlib import Path
    from dataclasses import replace
    source=HospitalMap(Path('/tmp/map.yaml'),Path('/tmp/map.png'),.05,(-79.62192,-237.371491,0),2304,6960,np.zeros((1,1)),np.zeros((1,1)))
    for bounds in [(0,800,0,600),(1234,5876,218,1419),(6000,6960,1900,2304)]:
        a,b,c,d=bounds
        crop=replace(source,width=d-c,height=b-a,origin=crop_origin(source.origin,source.height,bounds,source.resolution))
        for row,col in [(0,0),(b-a-1,d-c-1),((b-a)//2,(d-c)//2)]:
            xy=source.cell_to_world((row+a,col+c))
            assert np.allclose(crop.cell_to_world((row,col)),xy,atol=1e-12,rtol=0)
            assert crop.world_to_cell(*xy)==(row,col)

def test_ack_publication_change_mid_readback_fails_closed():
    from arena_evaluation.semantic_smac_session_r2 import ExactSemanticSmacSessionR2
    s=ack_session();changed=np.ones_like(s._current_grid,bool);s._begin_publication(changed)
    def snapshot(deadline):
        s._semantic_publication_sequence=2
        return np.zeros_like(s._current_grid,np.uint8),101
    s._server_costmap_snapshot=snapshot
    result=ExactSemanticSmacSessionR2._wait_for_costmap_ack(s,s._current_grid,changed,timeout_s=.005)
    assert not result['costmap_update_acknowledged']
    assert result['costmap_ack_sequence_mismatch']==1
