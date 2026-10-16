import ctypes,hashlib,struct
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from three_yd_v5.r6_delta import DeltaDecoder,HEADER
from three_yd_v5.r6_ack import exact_compare,grid_digest
from three_yd_v5.r6_session import MapFlowMixin

@pytest.fixture
def codec():
    lib=ctypes.CDLL(str(Path(__file__).resolve().parents[1]/'native/libdelta_codec_test.so'))
    lib.codec_create.argtypes=[ctypes.c_uint64];lib.codec_create.restype=ctypes.c_void_p
    lib.codec_destroy.argtypes=[ctypes.c_void_p]
    lib.codec_encode.argtypes=[ctypes.c_void_p,np.ctypeslib.ndpointer(dtype=np.uint8,flags='C_CONTIGUOUS'),ctypes.c_uint32,ctypes.c_uint32,ctypes.c_uint64,ctypes.c_char_p,ctypes.c_uint64,ctypes.c_uint64,ctypes.POINTER(ctypes.c_uint64)]
    lib.codec_encode.restype=ctypes.c_void_p;state=lib.codec_create(1234567)
    def send(a,d,*,epoch=None,base=None):
        size=ctypes.c_uint64();ptr=lib.codec_encode(state,a,a.shape[1],a.shape[0],912,d.token.encode(),d.epoch if epoch is None else epoch,d.sequence if base is None else base,ctypes.byref(size))
        return ctypes.string_at(ptr,size.value),'3yd_delta_v1|'+d.token
    yield send
    lib.codec_destroy(state)

@pytest.mark.parametrize('shape',[(31,49),(257,301),(600,1300)])
def test_native_deltas_reconstruct_all_cells_and_clear_previous_routes(codec,shape):
    rng=np.random.default_rng(1213);a=rng.integers(0,256,shape,dtype=np.uint8);d=DeltaDecoder(shape)
    first=True
    for i in range(40):
        data,layer=codec(a,d);got,info=d.decode(data,layer)
        assert np.array_equal(got,a)
        assert info['r6_complete_snapshot_verified']
        if first:assert info['r6_readback_full'];first=False
        else:assert info['r6_readback_wire_bytes']<=a.size+80
        y=int(rng.integers(0,shape[0]));x=int(rng.integers(0,shape[1]));a[y:min(y+13,shape[0]),x:min(x+17,shape[1])]=i%256
        # Changes outside any planning tube are just as observable.
        a[-1,-1]=i


def test_missing_reply_forces_a_complete_resync(codec):
    a=np.full((300,400),254,np.uint8);d=DeltaDecoder(a.shape)
    d.decode(*codec(a,d));a[5,5]=0;codec(a,d) # drop this response
    a[250,250]=90
    got,info=d.decode(*codec(a,d));assert info['r6_readback_full'];assert np.array_equal(a,got)


def test_no_change_has_header_only_but_fresh_sequence(codec):
    a=np.ones((300,400),np.uint8);d=DeltaDecoder(a.shape);d.decode(*codec(a,d));seq=d.sequence
    data,layer=codec(a,d);assert len(data)==80
    got,info=d.decode(data,layer);assert d.sequence==seq+1;assert info['r6_readback_patch_count']==0;assert np.array_equal(got,a)

@pytest.mark.parametrize('corruption',['payload','magic','shape','sequence','owner','truncated','base'])
def test_corruption_is_rejected_and_local_cache_invalidated(codec,corruption):
    a=np.ones((300,400),np.uint8);d=DeltaDecoder(a.shape);d.decode(*codec(a,d));a[0,0]=2
    data,layer=codec(a,d);data=bytearray(data)
    if corruption=='payload':data[-1]^=1
    elif corruption=='magic':data[0]=0
    elif corruption=='shape':struct.pack_into('<I',data,32,1)
    elif corruption=='sequence':struct.pack_into('<Q',data,16,d.sequence)
    elif corruption=='owner':layer+='z'
    elif corruption=='truncated':data=data[:-1]
    elif corruption=='base':struct.pack_into('<Q',data,24,0)
    with pytest.raises(RuntimeError):d.decode(data,layer)
    assert d.grid is None and d.sequence==0
    got,info=d.decode(*codec(a,d));assert info['r6_readback_full'];assert np.array_equal(got,a)

@pytest.mark.parametrize('shape',[(51,72),(159,301),(300,401)])
def test_streaming_ack_matches_original_all_cell_classes_and_hashes(shape):
    rng=np.random.default_rng(778);master=rng.integers(0,256,shape,dtype=np.uint8);previous=rng.integers(0,256,shape,dtype=np.uint8)
    expected=rng.integers(-1,101,shape,dtype=np.int8);soft=rng.integers(0,3,shape).astype(np.float32);hard=rng.random(shape)>.7;changed=rng.random(shape)>.8
    semantic=SimpleNamespace(expected_master_cost=np.flipud(master),hard_semantic_mask=np.flipud(hard),soft_cost=np.flipud(soft))
    for server in (master.copy(),rng.integers(0,256,shape,dtype=np.uint8)):
      for prev in (None,previous,master.copy()):
        got=exact_compare(semantic,expected,changed,prev,server,rows=17)
        dirty=np.ones(shape,bool) if prev is None else prev!=master
        affected=changed|dirty|hard|(soft>0);hc=affected&(hard|(expected==100));sc=affected&(soft>0)&~hc;ordinary=affected&~hc&~sc;bad=server!=master
        for k,v in [('affected',affected),('hard',hc),('soft',sc),('stale',dirty),('ordinary',ordinary)]:
            assert got[k]==int(v.sum())
            if k!='affected':assert got[k+'_bad']==int((v&bad).sum())
        assert got['mismatch']==int((affected&bad).sum())
        assert got['full_hash']==hashlib.sha256(server.tobytes()).hexdigest()
        assert got['affected_hash']==hashlib.sha256(server[affected].tobytes()).hexdigest()

@pytest.mark.parametrize('flip',[False,True])
def test_streamed_digest_keeps_original_row_order(flip):
    a=np.arange(900,dtype=np.int16).reshape(30,30)
    if flip:a=np.flipud(a)
    assert grid_digest(a,7)==hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()


def test_smaller_recovery_mask_still_closes_cells(monkeypatch):
    monkeypatch.setenv('THREEYD_V8_R6_MODE','A')
    class Base:
        def _grid_for_mask(self,m):
            a=self._semantic_costmap.occupancy_grid.copy();a[~m]=100;return m,np.flipud(a)
    class Session(MapFlowMixin,Base):pass
    s=Session();source=np.full((20,30),100,np.int8);source[5:8,10:12]=0;s._semantic_costmap=SimpleNamespace(occupancy_grid=source,expected_master_cost=np.zeros(source.shape,np.uint8))
    mask=source==0;_,out=s._grid_for_mask(mask);assert np.shares_memory(source,out);assert not out.flags.writeable
    with pytest.raises(ValueError):source[0,0]=0
    smaller=mask.copy();smaller[6,11]=False;_,out=s._grid_for_mask(smaller);assert out[20-1-6,11]==100;assert source[6,11]==0

@pytest.mark.parametrize('limit',[128000,512000])
def test_larger_publication_tiles_cover_old_and_new_regions_exactly(limit):
    from three_yd_v5.r10_tiles import rectangles
    a=np.full((900,1600),100,np.int8);b=a.copy()
    a[70:80,10:1550]=12;b[840:855,15:1520]=42
    changed=a!=b;boxes,info=rectangles(changed,limit);out=a.copy();covered=np.zeros(a.shape,bool)
    for y0,y1,x0,x1 in boxes:
        assert (y1-y0)*(x1-x0)<=limit
        out[y0:y1,x0:x1]=b[y0:y1,x0:x1];covered[y0:y1,x0:x1]=True
    assert np.all(covered[changed]) and np.array_equal(out,b)

class FakeClockExecutor:
    def spin_once(self,timeout_sec):pass


def ack_session(frames):
    shape=(11,17);occupancy=np.ones(shape,np.int8);master=np.full(shape,2,np.uint8)
    semantic=SimpleNamespace(occupancy_grid=occupancy,expected_master_cost=master,hard_semantic_mask=np.zeros(shape,bool),soft_cost=np.ones(shape,np.float32),policy_hash='policy')
    s=SimpleNamespace(_semantic_costmap=semantic,costmap_ack_timeout_s=.02,_semantic_publication_sequence=2,_active_publication_bbox=(0,0,17,11),_last_exact_expected_master=np.flipud(master.copy()),_active_publication_baseline_timestamp_ns=100,_costmap_ack_sequence=0,_semantic_roi_sequence=0,exact_stable_observations=2,PUBLICATION_VERSION='test',_semantic_ack_cache={},ack_events=[],current_query_id='Q',client=SimpleNamespace(executor=FakeClockExecutor()))
    s._content_signature=lambda a:(a.policy_hash,grid_digest(a.occupancy_grid),grid_digest(a.expected_master_cost))
    s._exact_cache_key=lambda **kw:tuple(kw.values());s._append_ack_trace=lambda *a:None
    events=iter(frames);last=frames[-1]
    def read(deadline):return next(events,last)
    s._server_costmap_snapshot=read
    return s,np.flipud(occupancy),np.zeros(shape,bool)


def test_ack_requires_two_exact_fresh_actual_snapshots():
    from three_yd_v5.r6_ack import wait_exact
    bad=np.zeros((11,17),np.uint8);good=np.full((11,17),2,np.uint8)
    s,expected,changed=ack_session([(bad,101),(good,102),(good,103)])
    result,_=wait_exact(s,expected,changed)
    assert result['costmap_update_acknowledged'] and result['costmap_ack_attempts']==3
    assert result['semantic_exact_stable_observations']==2
    assert result['costmap_ack_checked_cells']==expected.size
    assert not s._last_exact_expected_master.flags.writeable

@pytest.mark.parametrize('bad_content',[True,False])
def test_ack_cannot_accept_wrong_content_or_stale_snapshot(bad_content):
    from three_yd_v5.r6_ack import wait_exact
    frame=np.full((11,17),0 if bad_content else 2,np.uint8)
    s,expected,changed=ack_session([(frame,101 if bad_content else 100)])
    result,remaining=wait_exact(s,expected,changed,timeout_s=.001)
    assert result is None and remaining==0
    assert s._costmap_ack_sequence==0


def test_ack_rejects_policy_change_during_readback():
    from three_yd_v5.r6_ack import wait_exact
    good=np.full((11,17),2,np.uint8);s,expected,changed=ack_session([(good,101)])
    def read(deadline):s._semantic_publication_sequence+=1;return good,101
    s._server_costmap_snapshot=read
    with pytest.raises(RuntimeError,match='publication changed'):wait_exact(s,expected,changed)
