"""Bounded Linux sealed-memfd readback of a native mutex-protected master.

The FD is only a transport: the caller still compares every cell twice.
No source receipt, checksum or shared-memory presence alone constitutes ACK.
"""
from array import array
from dataclasses import dataclass
import fcntl
import hashlib
import json
import math
import mmap
import os
import socket
import struct
import time
import uuid
import numpy as np


class SnapshotError(RuntimeError):
    pass


@dataclass(frozen=True,init=False)
class SealedGrid:
    """Immutable full bytes whose SHA was computed from a write-sealed FD."""
    _mapping: object
    shape: tuple
    sha256: str

    def __init__(self,fd,shape,expected_hash):
        size=math.prod(shape)
        required=fcntl.F_SEAL_WRITE|fcntl.F_SEAL_GROW|fcntl.F_SEAL_SHRINK|fcntl.F_SEAL_SEAL
        if (len(shape)!=2 or not 0<size<=128*1024**2 or os.fstat(fd).st_size!=size or
                fcntl.fcntl(fd,fcntl.F_GET_SEALS)&required!=required):
            raise SnapshotError('SEALED_IMMUTABILITY_REQUIRED')
        view=mmap.mmap(fd,size,access=mmap.ACCESS_READ)
        value=hashlib.sha256(view).hexdigest()
        if value!=expected_hash:view.close();raise SnapshotError('SEALED_CONTENT_HASH')
        object.__setattr__(self,'_mapping',view);object.__setattr__(self,'shape',tuple(shape));object.__setattr__(self,'sha256',value)

    @property
    def array(self):return np.frombuffer(self._mapping,dtype=np.uint8).reshape(self.shape)
    @property
    def flags(self):return self.array.flags
    def tobytes(self):return self._mapping[:]
    def __array__(self,dtype=None,copy=None):
        a=self.array
        if dtype is not None:a=a.astype(dtype,copy=False)
        return a.copy() if copy else a
    def __ne__(self,other):return self.array!=other
    def __getitem__(self,key):return self.array[key]
    def __setitem__(self,key,value):raise ValueError('sealed snapshot is read-only')


class SealedSnapshotClient:
    def __init__(self, path, map_, server_pid):
        self.path=str(path);self.map=map_;self.server_pid=server_pid
        self.last_index=0;self.last_stamp_ns=0;self.last_hash='';self.last_metadata=None
        self._pending=[];self._pending_manifest=None

    def read(self, manifest, deadline):
        if self._pending_manifest!=manifest:
            self._pending.clear();self._pending_manifest=None
        if not self._pending:self._read_pair(manifest,deadline)
        if time.monotonic()>=deadline:
            self._pending.clear();raise SnapshotError('SEALED_DEADLINE')
        result,metadata=self._pending.pop(0)
        if metadata['index']<=self.last_index or metadata['created_steady_ns']<=self.last_stamp_ns:
            self._pending.clear();raise SnapshotError('SEALED_STALE_SNAPSHOT')
        self.last_index=metadata['index'];self.last_stamp_ns=metadata['created_steady_ns']
        self.last_hash=metadata['sha256'];self.last_metadata=metadata
        return result,self.last_stamp_ns

    def _read_pair(self, manifest, deadline):
        started=time.monotonic_ns()
        request=json.dumps({'manifest':manifest,'nonce':uuid.uuid4().hex,'snapshot_count':2,
                            'deadline_ns':int(deadline*1e9)},sort_keys=True,separators=(',',':')).encode()
        if len(request)>16384:raise SnapshotError('SEALED_REQUEST_SIZE')
        descriptors=[]
        try:
            with socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET) as channel:
                channel.settimeout(max(.000001,deadline-time.monotonic()))
                channel.connect(self.path)
                pid,uid,_=struct.unpack('3i',channel.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,12))
                if pid!=self.server_pid or uid!=os.getuid():raise SnapshotError('SEALED_PEER_IDENTITY')
                if channel.send(request)!=len(request):raise SnapshotError('SEALED_PARTIAL_REQUEST')
                channel.settimeout(max(.000001,deadline-time.monotonic()))
                payload,control,flags,_=channel.recvmsg(16384,socket.CMSG_SPACE(8*array('i').itemsize),socket.MSG_CMSG_CLOEXEC)
                for level,kind,body in control:
                    if level==socket.SOL_SOCKET and kind==socket.SCM_RIGHTS:
                        fds=array('i');fds.frombytes(body[:len(body)//fds.itemsize*fds.itemsize]);descriptors.extend(fds)
                if flags&(socket.MSG_TRUNC|socket.MSG_CTRUNC) or len(descriptors)!=2:
                    raise SnapshotError('SEALED_DESCRIPTOR_COUNT_OR_TRUNCATION')
                envelope=json.loads(payload);metadata=envelope['snapshots']
                if envelope['version']!=2 or len(metadata)!=2:raise SnapshotError('SEALED_PAIR_PROTOCOL')
                if ((os.fstat(descriptors[0]).st_dev,os.fstat(descriptors[0]).st_ino)==
                        (os.fstat(descriptors[1]).st_dev,os.fstat(descriptors[1]).st_ino)):
                    raise SnapshotError('SEALED_DUPLICATED_OBJECT')
                grids=[self.validate(fd,meta,request,manifest,started,deadline)
                       for fd,meta in zip(descriptors,metadata)]
                if (metadata[1]['index']<=metadata[0]['index'] or
                        metadata[1]['created_steady_ns']<=metadata[0]['created_steady_ns']):
                    raise SnapshotError('SEALED_PAIR_ORDER')
                # Two distinct master copies taken in one complete-master lock.
                # Both objects remain immutable; each is fully compared by ACK.
                self._pending=list(zip(grids,metadata));self._pending_manifest=manifest
        except (OSError,ValueError,KeyError,TypeError) as exc:
            raise SnapshotError('SEALED_READBACK_ERROR: '+str(exc)) from exc
        finally:
            for fd in descriptors:os.close(fd)

    def validate(self, fd, meta, request, manifest, started, deadline):
        m=self.map
        if time.monotonic()>=deadline:raise SnapshotError('SEALED_DEADLINE')
        if (meta['version']!=1 or meta['request_sha256']!=hashlib.sha256(request).hexdigest() or
                meta['manifest_sha256']!=hashlib.sha256(manifest.encode()).hexdigest()):
            raise SnapshotError('SEALED_PUBLICATION_BINDING')
        if (type(meta['index']) is not int or meta['index']<=self.last_index or
                type(meta['created_steady_ns']) is not int or meta['created_steady_ns']<started or
                meta['created_steady_ns']<=self.last_stamp_ns or meta['created_steady_ns']>time.monotonic_ns()):
            raise SnapshotError('SEALED_STALE_SNAPSHOT')
        if (meta['shape']!=[m.height,m.width] or meta['frame']!='map' or meta['layer']!='master' or
                meta['resolution']!=struct.unpack('f',struct.pack('f',m.resolution))[0] or len(meta['origin'])!=2 or
                any(not math.isfinite(v) or not math.isclose(v,expected,rel_tol=0.,abs_tol=1e-9)
                    for v,expected in zip(meta['origin'],m.origin))):
            raise SnapshotError('SEALED_MAP_METADATA')
        size=m.height*m.width
        if not 0<size<=128*1024**2 or os.fstat(fd).st_size!=size:raise SnapshotError('SEALED_SIZE')
        required=fcntl.F_SEAL_WRITE|fcntl.F_SEAL_GROW|fcntl.F_SEAL_SHRINK|fcntl.F_SEAL_SEAL
        if fcntl.fcntl(fd,fcntl.F_GET_SEALS)&required!=required:raise SnapshotError('SEALED_MUTABLE_DESCRIPTOR')
        result=SealedGrid(fd,(m.height,m.width),meta['sha256'])
        if time.monotonic()>=deadline:raise SnapshotError('SEALED_DEADLINE')
        return result
