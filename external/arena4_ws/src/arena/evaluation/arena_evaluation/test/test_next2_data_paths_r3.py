"""Equivalent corridor/ACK data paths and bounded map-only preparation."""
import math
from types import SimpleNamespace
import numpy as np
import cv2
import pytest
from arena_evaluation import _endpoint_geometry_r3 as native
from arena_evaluation import exact_ack_r3 as ack

@pytest.mark.parametrize('radius',[0,1,2,13,42,45])
@pytest.mark.parametrize('density',[0.,.001,.03,.8,1.])
def test_sparse_dilation_exact_opencv(radius,density):
    rng=np.random.default_rng(901)
    raster=(rng.random((139,163))<density).astype(np.uint8)
    kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*radius+1,2*radius+1))
    radii=((kernel.sum(axis=1)-1)//2).astype(int).tolist()
    result=np.frombuffer(native.dilate_runs(raster,radii),np.uint8).reshape(raster.shape)
    np.testing.assert_array_equal(result,cv2.dilate(raster,kernel))


def message():
    from nav2_msgs.msg import Costmap
    m=Costmap();m.header.frame_id='map';m.header.stamp.sec=7;m.header.stamp.nanosec=123
    m.metadata.layer='master';m.metadata.size_x=5;m.metadata.size_y=3;m.metadata.resolution=.05
    m.metadata.origin.orientation.w=1.
    m.data=list(range(15));return m


def test_raw_full_master_equals_ros_deserializer_and_is_immutable():
    from rclpy.serialization import serialize_message,deserialize_message
    from nav2_msgs.msg import Costmap
    payload=serialize_message(message());decoded=deserialize_message(payload,Costmap)
    viewed=ack.raw_costmap_view(payload)
    assert viewed.data.obj is payload and viewed.data.readonly
    assert viewed.data.tobytes()==bytes(decoded.data)
    assert viewed.metadata.resolution==decoded.metadata.resolution
    buf=ack.AtomicReadbackBuffer(SimpleNamespace(width=5,height=3,resolution=.05,origin=(0.,0.,0.)))
    buf.push(payload);data,stamp=buf.consume()
    assert stamp==7000000123 and not data.flags.writeable
    np.testing.assert_array_equal(data,np.arange(15,dtype=np.uint8).reshape(3,5))
    buf.push(payload);assert buf.consume() is None


def test_raw_truncation_and_wrong_metadata_cannot_ack():
    from rclpy.serialization import serialize_message
    payload=serialize_message(message())
    for n in range(len(payload)):
        with pytest.raises((ack.ExactAckFailure,UnicodeError)):ack.raw_costmap_view(payload[:n])
    with pytest.raises(ack.ExactAckFailure):ack.raw_costmap_view(payload+b'\x00')
    buf=ack.AtomicReadbackBuffer(SimpleNamespace(width=6,height=3,resolution=.05,origin=(0.,0.,0.)))
    with pytest.raises(ack.ExactAckFailure,match='METADATA'):buf.push(payload)

@pytest.mark.parametrize('density',[0.,.0001,.01,1.])
def test_tight_chunks_cover_dirty_and_full_inflation_halo(density):
    rng=np.random.default_rng(9);dirty=rng.random((553,717))<density
    covered=np.zeros(dirty.shape,np.uint8)
    for x,y,w,h in ack.dirty_chunk_boxes(dirty):
        assert w*h<65536
        covered[y:y+h,x:x+w]=1
    needed=cv2.dilate(dirty.astype(np.uint8),np.ones((25,25),np.uint8))
    assert np.all(covered>=needed)
    yy,xx=np.nonzero(dirty)
    expected=(int(xx.min()),int(yy.min()),int(xx.max()-xx.min()+1),int(yy.max()-yy.min()+1)) if len(xx) else (0,0,0,0)
    assert ack.dirty_bbox(dirty)==expected


def test_corridor_scalar_bend_angles_equal_vector_reference():
    rng=np.random.default_rng(17)
    for pts in rng.normal(size=(3000,3,2)):
        a=pts[1]-pts[0];b=pts[2]-pts[1]
        old=abs(math.atan2(float(a[0]*b[1]-a[1]*b[0]),float(np.dot(a,b))))
        new=abs(math.atan2(a[0]*b[1]-a[1]*b[0],a[0]*b[0]+a[1]*b[1]))
        assert (old<math.radians(15))==(new<math.radians(15))


def test_reusable_preparation_is_map_only_bounded_and_query_artifacts_do_not_alias(tmp_path):
    from test_tiled_topology_r3 import _map,_config,FOOTPRINT
    from arena_evaluation.reusable_tile_preparation_r3 import ReusableTileTopology
    from arena_evaluation.tiled_topology_r3 import TiledTopology
    m=_map(tmp_path,'dogleg');tile=ReusableTileTopology(m,FOOTPRINT,tmp_path/'cache',_config())
    tile.build_coarse();info=tile.prepare_map()
    assert info['query_dependent_work'] is False and info['seam_valid']
    assert tile.graph_bytes<=tile.graph_limit_bytes and len(tile.lru)<=tile.config.memory_tiles
    all_tiles=tile.tiles;a=tile.artifact(all_tiles)
    baseline=TiledTopology(m,FOOTPRINT,tmp_path/'cache',_config());baseline.build_coarse();b=baseline.artifact(all_tiles)
    assert a.graph.nodes==b.graph.nodes and a.graph.edges==b.graph.edges
    a.graph.edges[0].polyline[0][0]+=999
    assert tile.artifact(all_tiles).graph.edges==b.graph.edges
    tiny=ReusableTileTopology(m,FOOTPRINT,tmp_path/'cache',_config(),graph_limit_bytes=1)
    tiny.build_coarse();tiny.prepare_map()
    assert tiny.graph_bytes==0 and not tiny.reusable_graphs


def test_complete_corridor_mask_equals_frozen_opencv_reference(monkeypatch):
    from test_two_layer_v1_r3_benchmark import _context
    from arena_evaluation import two_layer_v1_r3_benchmark as r
    ctx=_context();pts=[[3.,5.],[7.,5.],[7.,8.],[13.,8.],[13.,11.],[18.,11.]]
    route=SimpleNamespace(polyline=pts)
    for expansion in (False,True):
        first,info=r.corridor(ctx,route,expansion)
        with monkeypatch.context() as patch:
            patch.setattr(r,'_native_geometry',None)
            reference,oldinfo=r.corridor(ctx,route,expansion)
        np.testing.assert_array_equal(first,reference);assert info==oldinfo


def test_pose_aware_candidate_rank_obeys_forward_model():
    from arena_evaluation.reachable_endpoint_r3 import dubins_distance
    assert dubins_distance((0.,0.,0.),(1.,0.,0.),.4)==pytest.approx(1.)
    assert dubins_distance((0.,0.,0.),(1.,0.,math.pi),.4)>1.


def test_source_grid_fast_path_matches_frozen_parent_for_all_int8_values():
    from arena_evaluation.unified_four_backends_smoke import SmacSession
    rng=np.random.default_rng(14);grid=np.arange(-128,128,dtype=np.int16).astype(np.int8).reshape(16,16)
    session=ack.ExactAckSmacSession.__new__(ack.ExactAckSmacSession)
    session.ctx=SimpleNamespace(hospital_map=SimpleNamespace(width=16,height=16,occupancy=grid))
    for mask in [np.ones(grid.shape,bool),rng.random(grid.shape)<.6,np.zeros(grid.shape,bool)]:
        _,old=SmacSession._grid_for_mask(session,mask)
        _,new=session._grid_for_mask(mask)
        np.testing.assert_array_equal(old,new);assert new.flags.c_contiguous


def test_hybrid_quota_preserves_first_candidate_and_spreads_remaining_attempts(monkeypatch):
    import time
    from test_reachable_endpoint_r3 import _map,_topology,_config,FOOTPRINT
    from arena_evaluation.reachable_endpoint_r3 import ReachableEndpointSelector
    m=_map();t=_topology(m,[(2,[[2.,4.],[8.,4.]])]);s=ReachableEndpointSelector(t,FOOTPRINT,_config())
    records=[{'point':[x,4.],'component':2,'tangent_yaw':0.,'failure':'ENDPOINT_LOCAL_SE2_NO_PATH'} for x in [3.,3.05,3.1,4.2,5.4,6.6]]
    attempted=[]
    def connect(a,b,deadline):
        attempted.append(b[:2]);return None,{'expanded':2500,'generated':3000}
    monkeypatch.setattr(s.connector,'connect',connect)
    stats={'attempts':0,'expanded':0,'generated':0}
    s._refine_candidates((2.,4.,0.),False,[],records,stats,[{'component':2}],time.monotonic()+5.)
    assert attempted==[(3.,4.),(4.2,4.),(5.4,4.),(6.6,4.)]
    assert stats['attempts']==4 and stats['expanded']==10000
