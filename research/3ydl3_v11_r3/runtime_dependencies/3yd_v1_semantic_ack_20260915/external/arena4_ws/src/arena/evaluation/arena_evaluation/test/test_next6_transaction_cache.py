from types import SimpleNamespace
from collections import deque
import hashlib,json,struct,time
import numpy as np
import pytest
from arena_evaluation.bounded_context_r3 import BoundedField
from arena_evaluation.exact_ack_r3 import ExactAckSmacSession,Publication,ExactAckFailure

def field(tmp_path,**kw):
    occ=np.zeros((200,200),np.int8);occ[:,0]=100;occ[100,:]=100
    m=SimpleNamespace(occupancy=occ,height=200,width=200,resolution=.05)
    return BoundedField(m,'occupied_distance',tile_cells=32,max_bytes=8192,disk_cache=tmp_path,map_hash='map1',**kw)

def test_disk_cache_survives_memory_eviction_and_process_context(tmp_path):
    a=field(tmp_path);expected=a[30,30]
    for c in range(0,200,32):a[30,c]
    assert (0,0) not in a.entries
    assert a[30,30]==expected and a.stats['disk_hits']==1
    b=field(tmp_path);assert b[30,30]==expected and b.stats['disk_hits']==1
    assert b.stats['max_window_cells']==0 and a.bytes<=8192

def test_cache_corruption_and_config_binding(tmp_path):
    a=field(tmp_path);a[30,30]
    b=field(tmp_path,max_halo_cells=512);assert a.disk_cache!=b.disk_cache
    p=next(a.disk_cache.glob('*.tile'));payload=p.read_bytes();p.write_bytes(payload[:-1]+bytes([payload[-1]^1]))
    with pytest.raises(RuntimeError,match='HASH_MISMATCH'):field(tmp_path)[30,30]

def test_disk_capacity_never_exceeded(tmp_path):
    a=field(tmp_path,max_disk_bytes=4200)
    for c in range(0,200,32):a[30,c]
    assert a.disk_bytes<=4200 and sum(p.stat().st_size for p in a.disk_cache.glob('*.tile'))<=4200

@pytest.mark.parametrize('receipt_kind',['exact','stale','wrong_hash','missing'])
def test_source_commit_receipt_binding_and_two_stage_ack(receipt_kind):
    from map_msgs.msg import OccupancyGridUpdate
    s=object.__new__(ExactAckSmacSession);s._transfer_id=0;s._transfer_receipts=deque(maxlen=8)
    s._transaction_ack_deadline=None;s.request_deadline=time.monotonic()+.03
    s._atomic_buffer=SimpleNamespace(reset_floor=lambda _:None)
    s.OccupancyGridUpdate=OccupancyGridUpdate;s.active_publication=Publication(1,'source',(0,0,3,4),'effective',(4,3),'map','request')
    s._trace=lambda _:None;s.last_exact_ack=None
    sent=[]
    def publish(m):
        sent.append(m)
        if not m.width and not m.height:
            payload=bytes(m.data).decode();v=json.loads(payload)
            assert v['messages']==len(sent)-1 and v['publication']['request_id']=='request'
            h=hashlib.sha256()
            for chunk in sent[:-1]:h.update(struct.pack('<4I',chunk.x,chunk.y,chunk.width,chunk.height));h.update(bytes(chunk.data))
            assert h.hexdigest()==v['chunks_sha256']
            if receipt_kind=='exact':s._transfer_receipts.append(payload)
            elif receipt_kind!='missing':
                v['transfer_id' if receipt_kind=='stale' else 'chunks_sha256']=0
                s._transfer_receipts.append(json.dumps(v))
    clock=SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1,to_msg=lambda:__import__('builtin_interfaces.msg',fromlist=['Time']).Time()))
    s.client=SimpleNamespace(node=SimpleNamespace(get_clock=lambda:clock),executor=SimpleNamespace(spin_once=lambda **kw:None))
    s._local_update_publisher=SimpleNamespace(publish=publish)
    if receipt_kind=='exact':
        info=s._publish_chunks(np.zeros((4,3),np.int8),np.ones((4,3),bool))
        assert info['source_consumption_verified'] and s.last_exact_ack is None
    else:
        with pytest.raises(ExactAckFailure,match='NOT_CONSUMED'):s._publish_chunks(np.zeros((4,3),np.int8),np.ones((4,3),bool))

@pytest.mark.parametrize('seed',range(12))
def test_bounded_minimum_matches_every_frozen_scalar_lookup(seed):
    import cv2
    rng=np.random.default_rng(seed);occ=np.zeros((513,601),np.int8)
    if seed%3==0:occ[0,0]=100
    elif seed%3==1:occ[rng.integers(0,513,80),rng.integers(0,601,80)]=100
    else:occ[:,0]=100;occ[250,300]=-1
    m=SimpleNamespace(occupancy=occ,height=513,width=601,resolution=.05)
    a=BoundedField(m,'occupied_distance',tile_cells=32,max_bytes=8192)
    expected=cv2.distanceTransform((occ!=100).astype(np.uint8),cv2.DIST_L2,5)*.05
    cells=list(zip(rng.integers(0,513,400),rng.integers(0,601,400)))
    assert a.minimum_at(cells)==min(float(expected[r,c]) for r,c in cells)
    assert a.minimum_at(cells+[None])==0.

def test_yaml_occupancy_interpretation_invalidates_persistent_cache(tmp_path):
    from arena_evaluation.bounded_context_r3 import bounded_context
    from arena_evaluation.unified_four_backends_smoke import FOOTPRINT
    import yaml
    image=tmp_path/'map.pgm'
    pixels=np.full((80,80),255,np.uint8);pixels[:,0]=0;pixels[:,40]=128
    image.write_bytes(b'P5\n80 80\n255\n'+pixels.tobytes())
    config={'image':'map.pgm','resolution':.05,'origin':[0.,0.,0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196}
    path=tmp_path/'map.yaml';path.write_text(yaml.safe_dump(config))
    a=bounded_context(path,'test',tmp_path/'cache',FOOTPRINT)
    first=a.hospital_map.distance_m[30,42];prefix=a.hospital_map.distance_m.disk_cache
    config['occupied_thresh']=.4;path.write_text(yaml.safe_dump(config))
    b=bounded_context(path,'test',tmp_path/'cache',FOOTPRINT)
    assert b.hospital_map.distance_m.disk_cache!=prefix
    assert b.hospital_map.distance_m[30,42]<first
    c=bounded_context(path,'test',tmp_path/'cache',FOOTPRINT)
    assert c.hospital_map.distance_m[30,42]==b.hospital_map.distance_m[30,42]
    assert c.hospital_map.distance_m.stats['disk_hits']==1
