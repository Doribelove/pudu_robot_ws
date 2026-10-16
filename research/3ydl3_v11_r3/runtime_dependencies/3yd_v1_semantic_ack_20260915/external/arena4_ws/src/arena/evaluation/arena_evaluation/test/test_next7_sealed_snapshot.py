"""A real Unix-socket + sealed descriptor roundtrip and fail-closed regressions."""
import copy
import fcntl
import hashlib
import json
import mmap
import os
from pathlib import Path
import socket
import struct
import threading
import time
from array import array
from types import SimpleNamespace
import numpy as np
import pytest
from arena_evaluation.sealed_snapshot_r3 import SealedSnapshotClient,SnapshotError

M=SimpleNamespace(height=3,width=4,resolution=.05,origin=(1.,2.,0.))
DATA=bytes(range(12))


def descriptor(sealed=True):
    fd=os.memfd_create('test-master',os.MFD_ALLOW_SEALING|os.MFD_CLOEXEC)
    os.write(fd,DATA)
    if sealed:fcntl.fcntl(fd,fcntl.F_ADD_SEALS,fcntl.F_SEAL_WRITE|fcntl.F_SEAL_GROW|fcntl.F_SEAL_SHRINK|fcntl.F_SEAL_SEAL)
    return fd


def metadata(request,manifest,started):
    return {'version':1,'index':1,'created_steady_ns':started,
        'request_sha256':hashlib.sha256(request).hexdigest(),'manifest_sha256':hashlib.sha256(manifest.encode()).hexdigest(),
        'sha256':hashlib.sha256(DATA).hexdigest(),'shape':[3,4],'resolution':struct.unpack('f',struct.pack('f',.05))[0],'origin':[1.,2.],
        'frame':'map','layer':'master'}


@pytest.mark.parametrize('change',[
    {'version':2},{'index':0},{'created_steady_ns':0},
    {'created_steady_ns':10**30},{'request_sha256':'bad'},{'manifest_sha256':'bad'},
    {'shape':[4,3]},{'resolution':.1},{'origin':[1.,3.]},{'origin':[float('nan'),2.]},
    {'frame':'odom'},{'layer':'static'},{'sha256':'bad'}])
def test_reject_metadata(change):
    c=SealedSnapshotClient('',M,os.getpid());request=b'nonce';manifest='publication'
    started=time.monotonic_ns();meta=metadata(request,manifest,started);meta.update(change);fd=descriptor()
    try:
        with pytest.raises(SnapshotError):c.validate(fd,meta,request,manifest,started,time.monotonic()+1)
    finally:os.close(fd)


def test_unsealed_rejected_and_deadline():
    c=SealedSnapshotClient('',M,os.getpid());started=time.monotonic_ns();meta=metadata(b'n','p',started);fd=descriptor(False)
    try:
        with pytest.raises(SnapshotError,match='MUTABLE'):c.validate(fd,meta,b'n','p',started,time.monotonic()+1)
        with pytest.raises(SnapshotError,match='DEADLINE'):c.validate(fd,meta,b'n','p',started,time.monotonic()-1)
    finally:os.close(fd)


def test_descriptor_wrong_length():
    fd=descriptor(False);os.ftruncate(fd,11)
    fcntl.fcntl(fd,fcntl.F_ADD_SEALS,fcntl.F_SEAL_WRITE|fcntl.F_SEAL_GROW|fcntl.F_SEAL_SHRINK|fcntl.F_SEAL_SEAL)
    c=SealedSnapshotClient('',M,os.getpid());t=time.monotonic_ns()
    try:
        with pytest.raises(SnapshotError,match='SIZE'):c.validate(fd,metadata(b'n','p',t),b'n','p',t,time.monotonic()+1)
    finally:os.close(fd)


def pair_server(path,mode='valid'):
    listener=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);listener.bind(str(path));listener.listen(1)
    errors=[]
    def serve():
        try:
            with listener.accept()[0] as peer:
                request=peer.recv(16384);manifest=json.loads(request)['manifest']
                assert json.loads(request)['snapshot_count']==2
                first=metadata(request,manifest,time.monotonic_ns());second=metadata(request,manifest,time.monotonic_ns());second['index']=2
                fds=[descriptor(),descriptor()]
                try:
                    assert os.fstat(fds[0]).st_ino!=os.fstat(fds[1]).st_ino
                    delivered=list(fds)
                    if mode=='duplicate':delivered=[fds[0],fds[0]]
                    if mode=='missing':delivered=fds[:1]
                    if mode=='order':second['index']=1
                    if mode=='hash':second['sha256']='0'*64
                    if mode=='binding':second['manifest_sha256']='0'*64
                    body=json.dumps({'version':2,'snapshots':[first,second]}).encode()
                    peer.sendmsg([body],[(socket.SOL_SOCKET,socket.SCM_RIGHTS,array('i',delivered))])
                finally:
                    for fd in fds:os.close(fd)
        except BaseException as e:errors.append(e)
    worker=threading.Thread(target=serve);worker.start()
    return listener,worker,errors


def test_real_two_full_immutable_snapshots(tmp_path):
    path=tmp_path/'s';listener,worker,errors=pair_server(path)
    try:
        c=SealedSnapshotClient(path,M,os.getpid())
        for index in [1,2]:
            grid,_=c.read('bound publication',time.monotonic()+2)
            assert grid.tobytes()==DATA and not grid.flags.writeable and c.last_index==index
            with pytest.raises(ValueError):grid[0,0]=99
        assert c.last_hash==hashlib.sha256(DATA).hexdigest() and not c._pending
    finally:worker.join(3);listener.close()
    assert not worker.is_alive() and not errors


@pytest.mark.parametrize('mode',['duplicate','missing','order','hash','binding'])
def test_pair_cannot_count_duplicate_missing_stale_or_wrong_second_frame(tmp_path,mode):
    path=tmp_path/'s';listener,worker,errors=pair_server(path,mode)
    try:
        c=SealedSnapshotClient(path,M,os.getpid())
        with pytest.raises(SnapshotError):c.read('publication',time.monotonic()+2)
        assert c.last_index==0 and not c._pending
    finally:worker.join(3);listener.close()
    assert not worker.is_alive() and not errors


def test_publication_changes_do_not_reuse_certificate():
    c=SealedSnapshotClient('',M,os.getpid());t=time.monotonic_ns();meta=metadata(b'n','old sequence/bbox/hash',t);fd=descriptor()
    try:
        for publication in ['stale sequence','wrong bbox','wrong source hash','wrong request id']:
            with pytest.raises(SnapshotError,match='BINDING'):c.validate(fd,meta,b'n',publication,t,time.monotonic()+1)
    finally:os.close(fd)


def test_ros_float32_resolution_is_exact_not_tolerance():
    c=SealedSnapshotClient('',M,os.getpid());t=time.monotonic_ns();meta=metadata(b'n','p',t);fd=descriptor()
    try:
        grid=c.validate(fd,meta,b'n','p',t,time.monotonic()+1)
        assert grid.tobytes()==DATA
        meta['resolution']=.05
        with pytest.raises(SnapshotError,match='METADATA'):c.validate(fd,meta,b'n','p',t,time.monotonic()+1)
    finally:os.close(fd)


@pytest.mark.parametrize('seed',[1,2,3,4])
def test_capped_clearance_equals_full_chamfer(seed):
    import cv2
    from arena_evaluation.bounded_context_r3 import BoundedField
    rng=np.random.default_rng(seed)
    occ=np.zeros((291,347),np.int8)
    occ[rng.random(occ.shape)<.003]=100
    occ[rng.random(occ.shape)<.002]=-1
    hospital=SimpleNamespace(occupancy=occ,height=291,width=347,resolution=.05)
    field=BoundedField(hospital,'occupied_distance',tile_cells=32)
    cells=[(int(r),int(c)) for r,c in zip(rng.integers(0,291,2000),rng.integers(0,347,2000))]
    cells.extend([(0,0),(290,346),(31,31),(32,32),(63,63)])
    full=cv2.distanceTransform((occ!=100).astype(np.uint8),cv2.DIST_L2,5)*hospital.resolution
    for cap in [.1,.5,1.]:
        observed=field.capped_at(cells,cap)
        expected=np.minimum(np.asarray([full[r,c] for r,c in cells],np.float32),cap)
        np.testing.assert_array_equal(observed,expected)
    assert field.stats['max_window_cells']<= (32+2*(int(np.ceil(1./.05/.98))+4))**2


def test_capped_clearance_open_map_and_invalid_bounds():
    from arena_evaluation.bounded_context_r3 import BoundedField
    hospital=SimpleNamespace(occupancy=np.zeros((150,150),np.int8),height=150,width=150,resolution=.05)
    field=BoundedField(hospital,'occupied_distance',tile_cells=32)
    assert list(field.capped_at([(0,0),(149,149),(75,75),None],.5))==[.5,.5,.5,0.]
    with pytest.raises(IndexError):field.capped_at([(150,0)],.5)
    with pytest.raises(ValueError):field.capped_at([(0,0)],float('nan'))
    with pytest.raises(ValueError):field.capped_at([(0,0)],1000.)


def chord_context(block=False):
    from arena_evaluation import two_layer_v1_r3_benchmark as r
    occ=np.zeros((160,160),np.int8)
    if block:occ[50:71,50:71]=100
    m=SimpleNamespace(occupancy=occ,height=160,width=160,resolution=.05,origin=(0.,0.,0.))
    m.world_to_cell=lambda x,y:(int(y/.05),int(x/.05)) if 0<=x<8 and 0<=y<8 else None
    return SimpleNamespace(hospital_map=m,map_sha256=hashlib.sha256(occ).hexdigest(),map_yaml_sha256='test-yaml'),r.legacy.FOOTPRINT


def test_chords_swept_safe_deterministic_and_bounded():
    from arena_evaluation.corridor_shortcuts_r3 import certified_chords
    ctx,footprint=chord_context()
    points=[(1.,1.),(1.,5.),(5.,5.)]
    first,attempts=certified_chords(ctx,points,footprint,max_attempts=1,max_accepted=1)
    second,again=certified_chords(ctx,points,footprint,max_attempts=1,max_accepted=1)
    assert len(first)==1 and attempts==again==1 and first==second
    assert first[0]['length_m']<8. and len(first[0]['hash'])==64
    assert certified_chords(ctx,points,footprint,max_attempts=0)==([],0)


def test_chord_cannot_cut_static_obstacle_or_unknown():
    from arena_evaluation.corridor_shortcuts_r3 import certified_chords
    ctx,footprint=chord_context(True);points=[(1.,1.),(1.,5.),(5.,5.)]
    assert certified_chords(ctx,points,footprint)[0]==[]
    ctx.hospital_map.occupancy[50:71,50:71]=-1
    assert certified_chords(ctx,points,footprint)[0]==[]


def test_retry_chords_keep_original_route_and_free_mask():
    from arena_evaluation import two_layer_v1_r3_benchmark as r
    from arena_evaluation.bounded_context_r3 import BoundedField
    ctx,footprint=chord_context();m=ctx.hospital_map
    m.distance_m=BoundedField(m,'occupied_distance',tile_cells=32)
    route=SimpleNamespace(polyline=[(1.,1.),(1.,5.),(5.,5.)])
    first,info=r.corridor(ctx,route,False,certified_chords_enabled=True);expanded,extra=r.corridor(ctx,route,True,certified_chords_enabled=True)
    assert info['corridor_chord_count']==1 and extra['corridor_chord_count']==1
    assert info['corridor_chord_certificates']==extra['corridor_chord_certificates']
    assert np.all(expanded[first]) and np.all(m.occupancy[expanded]==0)


def test_verified_sealed_hash_is_reused_but_content_still_compared():
    from dataclasses import FrozenInstanceError
    from arena_evaluation.sealed_snapshot_r3 import SealedGrid
    from arena_evaluation.exact_ack_r3 import grid_hash,compare_exact,Publication,FrozenExpectedMaster
    fd=descriptor()
    try:
        server=SealedGrid(fd,(3,4),hashlib.sha256(DATA).hexdigest())
        with pytest.raises(OSError):os.pwrite(fd,b'X',0)
        with pytest.raises(FrozenInstanceError):server.sha256='forged'
        with pytest.raises(ValueError):server.array.setflags(write=True)
        with pytest.raises(SnapshotError):SealedGrid(fd,(3,4),'0'*64)
    finally:os.close(fd)
    assert grid_hash(server)==hashlib.sha256(DATA).hexdigest()
    original=np.frombuffer(DATA,np.uint8).reshape(3,4)
    for changes in [False,True]:
        expected=original.copy()
        if changes:expected[1,2]+=1
        frozen=FrozenExpectedMaster(expected)
        token=Publication(1,'source',(0,0,4,3),frozen.sha256,(3,4),'map','request')
        result=compare_exact(token,token,frozen,server)
        assert result['mismatch_cells']==int(changes)
        assert result['acknowledged']==(not changes)


@pytest.mark.parametrize('width,height,expected',[(6000,10000,True),(12000,8000,False),(10000,10000,False),(1,2**32-1,True),(1,2**32,False)])
def test_pinned_smac_capacity(width,height,expected):
    from arena_evaluation.two_layer_v1_r3_benchmark import pinned_smac_capacity
    bins=1 if width==1 else 48
    assert pinned_smac_capacity(width,height,bins)['supported'] is expected


def test_oversized_smac_rejected_before_any_query_work():
    from arena_evaluation.two_layer_v1_r3_benchmark import run_query
    def forbidden(*a,**kw):raise AssertionError('unsupported backend must not run')
    ctx=SimpleNamespace(map_id='test',hospital_map=SimpleNamespace(width=12000,height=8000))
    row=run_query(ctx,SimpleNamespace(query_id='fixed'),None,forbidden,None,None,None,'request')
    assert row['failure_code']=='PINNED_SMAC_INDEX_CAPACITY'
    assert row['planner_search_started'] is False and row['attempts']==[]


def test_production_corridor_does_not_enable_regressed_chord_search(monkeypatch):
    from arena_evaluation import two_layer_v1_r3_benchmark as r,corridor_shortcuts_r3 as experimental
    from arena_evaluation.bounded_context_r3 import BoundedField
    ctx,footprint=chord_context();m=ctx.hospital_map
    m.distance_m=BoundedField(m,'occupied_distance',tile_cells=32)
    def forbidden(*a,**kw):raise AssertionError('experimental search-space changes disabled')
    monkeypatch.setattr(experimental,'certified_chords',forbidden)
    route=SimpleNamespace(polyline=[(1.,1.),(1.,5.),(5.,5.)])
    for expansion in (False,True):
        mask,info=r.corridor(ctx,route,expansion)
        assert info['corridor_chord_count']==0 and info['corridor_chord_attempts']==0
        assert np.all(m.occupancy[mask]==0)
