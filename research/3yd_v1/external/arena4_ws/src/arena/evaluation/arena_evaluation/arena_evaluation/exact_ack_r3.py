"""Full-content fail-closed ACK transactions, without changing pinned Nav2.

The native mutex-protected costmap_raw snapshot has no server publication
sequence. Sequence below is an immutable
single-writer transaction identity; it is never inferred from response time.
Two complete matching observations bind the returned bytes to that identity.
"""
from __future__ import annotations
from array import array
from collections import OrderedDict, deque
from contextlib import contextmanager
from dataclasses import asdict, dataclass
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import tempfile
import time
from types import SimpleNamespace
import numpy as np
from .unified_four_backends_smoke import SmacSession, PlanResult
from .smac_contract_r3 import frozen_planner_overrides, read_runtime_contract
from .sealed_snapshot_r3 import SealedGrid


@contextmanager
def defer_transport_gc():
    """Keep the serial client's RPC/action exchange responsive.

    Restore cyclic collection before returning to local validation. Collection
    is not removed from the absolute request deadline, and reference counting
    remains active throughout. This controller has one Python request thread.
    """
    enabled = gc.isenabled()
    if enabled:
        gc.disable()
    try:
        yield
    finally:
        if enabled:
            gc.enable()


def grid_hash(grid):
    if isinstance(grid,SealedGrid):return grid.sha256
    # Hash the same complete C-order bytes without materializing a second
    # full grid when the input is already contiguous. Strided inputs still
    # receive the same canonical contiguous conversion as before.
    contiguous=np.ascontiguousarray(grid)
    return hashlib.sha256(memoryview(contiguous)).hexdigest()


@dataclass(frozen=True, init=False)
class FrozenExpectedMaster:
    """Own immutable complete bytes; hash once without trusting mutable arrays."""
    payload: bytes
    shape: tuple
    sha256: str

    def __init__(self, grid):
        array = np.ascontiguousarray(grid)
        if array.ndim != 2 or array.dtype != np.uint8:
            raise ValueError('expected master must be a 2D uint8 grid')
        payload = array.tobytes()
        object.__setattr__(self, 'payload', payload)
        object.__setattr__(self, 'shape', array.shape)
        object.__setattr__(self, 'sha256', hashlib.sha256(payload).hexdigest())

    @property
    def array(self):
        return np.frombuffer(self.payload, dtype=np.uint8).reshape(self.shape)

    @property
    def nbytes(self):
        return len(self.payload)

    def __array__(self, dtype=None, copy=None):
        array = self.array
        if dtype is not None:
            array = array.astype(dtype, copy=False)
        return array.copy() if copy else array

    def __getitem__(self, key):
        return self.array[key]

    def __setitem__(self, key, value):
        raise ValueError('assignment destination is read-only')


@dataclass(frozen=True)
class Publication:
    sequence: int
    source_grid_hash: str
    roi_bbox: tuple
    expected_effective_hash: str
    expected_shape: tuple
    map_hash: str
    request_id: str
    backend_context: str = ""

    @property
    def hash(self):
        return hashlib.sha256(json.dumps(asdict(self),sort_keys=True).encode()).hexdigest()


def compare_exact(publication, active, expected, server, *, readback_hash=None):
    """Never accept an equality check for another transaction or shape."""
    if isinstance(expected, FrozenExpectedMaster):
        eh = expected.sha256
        expected = expected.array
    else:
        eh = grid_hash(expected)
    sequence_bad=publication.sequence != active.sequence
    binding_bad=publication != active
    shape_bad=tuple(server.shape)!=publication.expected_shape or tuple(expected.shape)!=publication.expected_shape
    sh=grid_hash(server)
    hash_bad=eh!=publication.expected_effective_hash or (readback_hash is not None and sh!=readback_hash)
    mismatch=None;hard=soft=stale=None
    if not shape_bad:
        different=server!=expected;mismatch=int(np.count_nonzero(different))
        if mismatch==0:
            # Classifying an empty difference cannot find hard, soft or stale
            # cells. Avoid extra whole-grid masks on the exact-equality path;
            # shape, transaction and both content hashes are still mandatory.
            hard=soft=stale=0
        else:
            hard=int(np.count_nonzero(different & (expected>=253)))
            soft=int(np.count_nonzero(different & (expected<253)))
            stale=int(np.count_nonzero(different & (server>expected)))
    valid=not(binding_bad or shape_bad or hash_bad or mismatch!=0 or sh!=eh)
    return {'acknowledged':valid,'mismatch_cells':mismatch,'hard_mismatch':hard,
            'soft_mismatch':soft,'stale_cells':stale,'sequence_mismatch':int(sequence_bad),
            'binding_mismatch':int(binding_bad),'shape_mismatch':int(shape_bad),
            'hash_mismatch':int(hash_bad or sh!=eh),'server_readback_hash':sh,
            'publication':asdict(publication),'publication_hash':publication.hash}


class ExactAckFailure(RuntimeError):
    pass


def raw_costmap_view(payload):
    """Read the frozen Humble Costmap IDL without copying its complete grid.

    Only immutable CDR v1 is accepted; lengths and alignment are checked before
    exposing a view. Metadata still goes through the ordinary ACK validator.
    """
    if not isinstance(payload,bytes) or payload[:4] not in (b'\x00\x01\x00\x00',b'\x00\x00\x00\x00'):
        raise ExactAckFailure('ATOMIC_READBACK_CDR_ENCODING')
    endian='<' if payload[1] else '>';offset=4
    def take(fmt,alignment):
        nonlocal offset
        offset+=(-(offset-4))%alignment
        size=struct.calcsize(endian+fmt)
        if offset+size>len(payload):raise ExactAckFailure('ATOMIC_READBACK_CDR_TRUNCATED')
        value=struct.unpack_from(endian+fmt,payload,offset);offset+=size
        return value
    def string():
        nonlocal offset
        length,=take('I',4)
        if not 1<=length<=1024 or offset+length>len(payload) or payload[offset+length-1]!=0:
            raise ExactAckFailure('ATOMIC_READBACK_CDR_STRING')
        value=payload[offset:offset+length-1].decode('utf-8');offset+=length
        return value
    sec,nsec=take('iI',4);frame=string()
    for _ in range(2):take('iI',4)  # metadata map_load_time and update_time
    layer=string();resolution,width,height=take('fII',4)
    x,y,z,qx,qy,qz,qw=take('7d',8)
    length,=take('I',4)
    if nsec>=10**9 or length!=width*height or offset+length!=len(payload):
        raise ExactAckFailure('ATOMIC_READBACK_LENGTH_MISMATCH')
    origin=SimpleNamespace(position=SimpleNamespace(x=x,y=y,z=z),
                           orientation=SimpleNamespace(x=qx,y=qy,z=qz,w=qw))
    return SimpleNamespace(header=SimpleNamespace(frame_id=frame,stamp=SimpleNamespace(sec=sec,nanosec=nsec)),
        metadata=SimpleNamespace(layer=layer,resolution=resolution,size_x=width,size_y=height,origin=origin),
        data=memoryview(payload)[offset:])


def dirty_bbox(dirty):
    """Bounded O(width+height) coordinate storage even for a full dirty map."""
    rows=np.flatnonzero(np.any(dirty,axis=1));cols=np.flatnonzero(np.any(dirty,axis=0))
    if not len(rows):return (0,0,0,0)
    return int(cols[0]),int(rows[0]),int(cols[-1]-cols[0]+1),int(rows[-1]-rows[0]+1)


def dirty_chunk_boxes(dirty,side=224,overlap=12):
    """Tight dirty rectangles with the unchanged inflation-seam overlap."""
    height,width=dirty.shape
    for y in range(0,height,side):
        for x in range(0,width,side):
            bx,by,bw,bh=dirty_bbox(dirty[y:y+side,x:x+side])
            if not bw:continue
            x0=max(0,x+bx-overlap);y0=max(0,y+by-overlap)
            x1=min(width,x+bx+bw+overlap);y1=min(height,y+by+bh+overlap)
            yield x0,y0,x1-x0,y1-y0


def require_reliable_subscriptions(endpoints, reliable):
    if not endpoints or any(e.qos_profile.reliability!=reliable for e in endpoints):
        raise ExactAckFailure('RELIABLE_UPDATE_SUBSCRIPTION_REQUIRED')


def discover_reliable_updates(node,executor,reliable,deadline):
    while time.monotonic()<deadline:
        endpoints=node.get_subscriptions_info_by_topic('/map_updates')
        if any(e.node_name=='global_costmap' for e in endpoints):
            require_reliable_subscriptions(endpoints,reliable)
            return endpoints
        executor.spin_once(timeout_sec=min(.01,max(0.,deadline-time.monotonic())))
    raise ExactAckFailure('UPDATE_SUBSCRIPTION_DISCOVERY_TIMEOUT')


class AtomicReadbackBuffer:
    """Two immutable full masters, validating map metadata and fresh timestamps.

    Header time is stamped inside the native master mutex. It establishes
    freshness after publication; it is not a server publication-sequence echo.
    """
    def __init__(self,map_):
        self.map=map_;self.frames=deque(maxlen=2);self.floor_ns=0
        self.index=0;self.last_index=0;self.last_stamp_ns=0

    def reset_floor(self,stamp_ns):
        self.floor_ns=int(stamp_ns);self.frames.clear()

    def push(self,message):
        raw=isinstance(message,bytes)
        if raw:message=raw_costmap_view(message)
        m=self.map;meta=message.metadata;origin=meta.origin
        if (message.header.frame_id!='map' or meta.layer!='master' or
                not all(math.isfinite(v) for v in (origin.position.x,origin.position.y,
                    origin.position.z,origin.orientation.x,origin.orientation.y,
                    origin.orientation.z,origin.orientation.w)) or
                (meta.size_y,meta.size_x)!=(m.height,m.width) or
                meta.resolution!=struct.unpack('f',struct.pack('f',m.resolution))[0] or
                not math.isclose(origin.position.x,m.origin[0],rel_tol=0.,abs_tol=1e-9) or
                not math.isclose(origin.position.y,m.origin[1],rel_tol=0.,abs_tol=1e-9) or
                abs(origin.position.z)>1e-12 or abs(origin.orientation.x)>1e-12 or
                abs(origin.orientation.y)>1e-12 or abs(origin.orientation.z)>1e-12 or
                abs(origin.orientation.w-1.)>1e-12):
            raise ExactAckFailure('ATOMIC_READBACK_MAP_METADATA_MISMATCH')
        stamp=int(message.header.stamp.sec)*10**9+int(message.header.stamp.nanosec)
        if stamp<=self.floor_ns or stamp<=self.last_stamp_ns:return
        payload=message.data if raw else bytes(message.data)
        if len(payload)!=m.height*m.width:raise ExactAckFailure('ATOMIC_READBACK_LENGTH_MISMATCH')
        data=np.frombuffer(payload,dtype=np.uint8).reshape((m.height,m.width))
        self.index+=1;self.frames.append((self.index,stamp,data))

    def consume(self):
        while self.frames:
            index,stamp,data=self.frames.popleft()
            if stamp>self.floor_ns and stamp>self.last_stamp_ns and index>self.last_index:
                self.last_index=index;self.last_stamp_ns=stamp
                return data,stamp
        return None


def master_snapshot_frequency(cells):
    """Bound complete-master payload to 300 MB/s and the original 40 Hz cap."""
    if cells<=0:raise ValueError('positive map size required')
    return min(40.,300_000_000./cells)


class ExactAckSmacSession(SmacSession):
    def __init__(self,*args,**kwargs):
        kwargs['planner_parameter_overrides']=frozen_planner_overrides(kwargs.get('planner_parameter_overrides'))
        # StaticLayer uses SystemDefaultsQoS. Scope its reader profile to the
        # one update topic; explicit safety/configuration QoS remain unchanged.
        transport=Path(__file__).resolve().parents[1]/'config'/'two_layer_v1_r3_transport.xml'
        if not transport.is_file():
            from ament_index_python.packages import get_package_share_directory
            transport=Path(get_package_share_directory('arena_evaluation'))/'config'/'two_layer_v1_r3_transport.xml'
        if not transport.is_file():raise ExactAckFailure('RELIABLE_TRANSPORT_PROFILE_MISSING')
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(transport)
        super().__init__(*args,**kwargs)
        from rclpy.qos import QoSProfile,QoSReliabilityPolicy
        self._map_update_qos=QoSProfile(depth=512,reliability=QoSReliabilityPolicy.RELIABLE)
        self.transport_profile_hash=hashlib.sha256(transport.read_bytes()).hexdigest()
        import yaml
        params=yaml.safe_load(self.params_file.read_text())
        params['global_costmap']['global_costmap']['ros__parameters'].setdefault('static_layer',{})['plugin']='pln_transactional_costmap/TransactionalStaticLayer'
        self._snapshot_directory=tempfile.TemporaryDirectory(prefix='pln-snapshot-')
        self._snapshot_path=Path(self._snapshot_directory.name)/'master.sock'
        params['global_costmap']['global_costmap']['ros__parameters']['static_layer']['snapshot_socket']=str(self._snapshot_path)
        params['global_costmap']['global_costmap']['ros__parameters']['publish_frequency']=master_snapshot_frequency(
            self.ctx.hospital_map.height*self.ctx.hospital_map.width)
        self._sealed_client=None;self._consumed_manifest=None
        self.params_file.write_text(yaml.safe_dump(params,sort_keys=False))
        self.smac_config_hash=hashlib.sha256(self.params_file.read_bytes()).hexdigest()
        self._atomic_buffer=AtomicReadbackBuffer(self.ctx.hospital_map)
        self._atomic_subscription=None;self._snapshot_errors=0
        self.local_map_update_strategy='exact_r3'
        self.publication_sequence=0;self.active_publication=None
        self.request_deadline=math.inf;self.request_id='';self.ack_trace=[]
        self.last_exact_ack=None;self._expected_cache=OrderedDict();self._expected_cache_bytes=0
        self.expected_cache_limit_bytes=512*1024**2;self._expected_cache_hit=False
        self._effective_compute_ms=0.;self._confirmed_server_budget=None;self._budget_update_peak_s=0.
        self._budget_state_uncertain=False;self._budget_diagnostics={}
        self._server_settle_grace_s=.8
        self.runtime_safety_contract=None
        self.trace_file=self.params_file.parent/'exact_ack.jsonl'
        self._enforce_server_budget=True;self._budget_client=None;self._unresolved_timeout=False
        self._transfer_id=0;self._receipt_subscription=None;self._transfer_receipts=deque(maxlen=8)
        self._transaction_ack_deadline=None

    def start(self):
        super().start()
        from .sealed_snapshot_r3 import SealedSnapshotClient
        self._sealed_client=SealedSnapshotClient(self._snapshot_path,self.ctx.hospital_map,self.planner_pid)
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile,QoSReliabilityPolicy
        self._receipt_subscription=self.client.node.create_subscription(
            String,'/map_transaction_receipt',lambda msg:self._transfer_receipts.append(msg.data),
            QoSProfile(depth=8,reliability=QoSReliabilityPolicy.RELIABLE))
        from .unified_four_backends_smoke import FOOTPRINT
        self.runtime_safety_contract=read_runtime_contract(self,FOOTPRINT)
        with (self.params_file.parent/'runtime_safety_contract.json').open('x') as stream:
            json.dump(self.runtime_safety_contract,stream,indent=2)
        from rclpy.qos import QoSReliabilityPolicy
        endpoints=discover_reliable_updates(self.client.node,self.client.executor,
                                             QoSReliabilityPolicy.RELIABLE,time.monotonic()+5.)
        self.transport_qos=[{'node':e.node_name,'reliability':str(e.qos_profile.reliability),
                             'depth':e.qos_profile.depth} for e in endpoints]
        require_reliable_subscriptions(endpoints,QoSReliabilityPolicy.RELIABLE)
        self._trace({'event':'transport_verified','profile_hash':self.transport_profile_hash,
                     'subscriptions':self.transport_qos,'writer_depth':512,'writer_reliable':True})
        # Confirm the initial frozen limit without rebuilding Smac's 3.9M-entry
        # Dubins lookup table through a redundant SetParameters call.
        from rcl_interfaces.srv import GetParameters
        client=self.client.node.create_client(GetParameters,'/planner_server/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=10.):raise ExactAckFailure('INITIAL_BUDGET_SERVICE_UNAVAILABLE')
            request=GetParameters.Request();request.names=['GridBased.max_planning_time']
            future=client.call_async(request);deadline=time.monotonic()+10.
            while not future.done() and time.monotonic()<deadline:self.client.executor.spin_once(timeout_sec=.01)
            if not future.done():raise ExactAckFailure('INITIAL_BUDGET_READ_TIMEOUT')
            values=future.result().values
            if len(values)!=1 or values[0].type!=3 or values[0].double_value!=5.:
                raise ExactAckFailure('FROZEN_INITIAL_SERVER_BUDGET_MISMATCH')
            self._confirmed_server_budget=5.
        finally:self.client.node.destroy_client(client)

    def _start_atomic_readback(self):
        if getattr(self,'_sealed_client',None) is not None:return
        if getattr(self,'_atomic_subscription',None) is not None:return
        from nav2_msgs.msg import Costmap
        from rclpy.qos import QoSProfile,QoSDurabilityPolicy,QoSReliabilityPolicy
        self._atomic_subscription=self.client.node.create_subscription(
            Costmap,'/global_costmap/costmap_raw',self._receive_atomic_snapshot,
            QoSProfile(depth=2,reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),raw=True)

    def _stop_atomic_readback(self):
        if getattr(self,'_sealed_client',None) is not None:self._sealed_client._pending.clear()
        if getattr(self,'_atomic_subscription',None) is not None:
            self.client.node.destroy_subscription(self._atomic_subscription)
            self._atomic_subscription=None
        if hasattr(self,'_atomic_buffer'):self._atomic_buffer.frames.clear()

    def _receive_atomic_snapshot(self,message):
        try:self._atomic_buffer.push(message)
        except (ExactAckFailure,ValueError,TypeError,AttributeError) as exc:
            self._snapshot_errors+=1
            self._trace({'event':'readback_error','source':'atomic_master_topic','error':str(exc)})

    def _server_costmap_snapshot(self,deadline):
        if getattr(self,'_sealed_client',None) is not None:
            value=self._sealed_client.read(self._consumed_manifest,deadline)
            self._trace({'event':'sealed_snapshot_read',**self._sealed_client.last_metadata})
            return value
        while time.monotonic()<deadline:
            value=self._atomic_buffer.consume()
            if value is not None:
                self._trace({'event':'atomic_snapshot_read','snapshot_index':self._atomic_buffer.last_index,
                             'server_header_stamp_ns':value[1],'publication_floor_ns':self._atomic_buffer.floor_ns})
                return value
            self.client.executor.spin_once(timeout_sec=min(.01,max(0.,deadline-time.monotonic())))
        raise ExactAckFailure('ATOMIC_READBACK_TIMEOUT')

    def close(self):
        if self._atomic_subscription is not None and self.client is not None:
            self.client.node.destroy_subscription(self._atomic_subscription)
            self._atomic_subscription=None
        self._atomic_buffer.frames.clear()
        super().close()
        if hasattr(self,'_snapshot_directory'):self._snapshot_directory.cleanup()

    def prepare_ready_baseline(self, timeout_s=90.):
        """Establish a query-independent closed overlay before accepting requests.

        The immutable static map is unchanged. Every query still needs its own
        two complete exact observations after opening its corridor. Initialization
        has a separate bounded window, recorded outside the 7-second request.
        """
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('READY baseline timeout must be positive and finite')
        started=time.monotonic()
        self.begin_request('READY:closed-baseline',started+timeout_s)
        self._ready_ack_window_s=timeout_s
        self._current_grid=None
        try:
            info=self.update_local_mask(np.zeros((self.ctx.hospital_map.height,
                                                 self.ctx.hospital_map.width),bool))
            info.update(ready_baseline_wall_ms=(time.monotonic()-started)*1000,
                        ready_baseline_kind='query_independent_all_blocked_overlay',
                        query_dependent=False,planner_search_started=False)
            self.ready_baseline_receipt=info
            return dict(info)
        finally:
            del self._ready_ack_window_s

    def begin_request(self, request_id, deadline):
        self.request_id=str(request_id);self.request_deadline=float(deadline)
        self.last_exact_ack=None;self.active_publication=None;self._costmap_state_trusted=False
        self._transaction_ack_deadline=None
        self._budget_diagnostics={};self._snapshot_errors=0
        self._local_mask_info={'costmap_update_acknowledged':False,
                               'costmap_ack_status':'transaction_not_verified',
                               'costmap_ack_mismatch_cells':None,
                               'request_id':self.request_id,'planner_search_started':False}

    def _trace(self,event):
        event={'request_id':self.request_id,'monotonic':time.monotonic(),**event}
        self.ack_trace.append(event)
        with self.trace_file.open('a') as f:f.write(json.dumps(event,sort_keys=True)+'\n')

    def _grid_for_mask(self,allowed_mask):
        mask=np.asarray(allowed_mask,dtype=bool);m=self.ctx.hospital_map
        if mask.shape!=(m.height,m.width):raise ValueError('local mask shape does not match the map')
        # Same unknown/clipping and ROS row orientation as the frozen parent,
        # with one contiguous int8 destination rather than int16 temporaries.
        source=np.array(np.flipud(m.occupancy),dtype=np.int8,order='C',copy=True)
        np.clip(source,-1,100,out=source);source[~np.flipud(mask)]=100
        return mask,source

    def _expected(self,source):
        from . import _nav2_effective_costmap
        key=(grid_hash(source),self.ctx.map_sha256,self.smac_config_hash,source.shape,
             '.05/.55/3/.225:effective-master-v1')
        self._expected_cache_hit=key in self._expected_cache
        if self._expected_cache_hit:
            expected,self._effective_compute_ms=self._expected_cache.pop(key)
            self._expected_cache[key]=(expected,self._effective_compute_ms)
            return expected
        started=time.monotonic()
        static=np.zeros(source.shape,np.uint8)
        static[source==100]=254;static[source<0]=255
        payload=_nav2_effective_costmap.inflate(static,int(static.shape[1]),int(static.shape[0]),
                                               .05,.55,3.,.225)
        expected=FrozenExpectedMaster(np.frombuffer(payload,dtype=np.uint8).reshape(source.shape))
        self._effective_compute_ms=(time.monotonic()-started)*1000
        if expected.nbytes<=self.expected_cache_limit_bytes:
            while self._expected_cache and self._expected_cache_bytes+expected.nbytes>self.expected_cache_limit_bytes:
                _,(old,_) = self._expected_cache.popitem(last=False);self._expected_cache_bytes-=old.nbytes
            self._expected_cache[key]=(expected,self._effective_compute_ms);self._expected_cache_bytes+=expected.nbytes
        return expected

    def _publish_chunks(self,source,dirty,*,overlap=12):
        """One update topic, deterministic overlapping <=64KiB source chunks."""
        self._atomic_buffer.reset_floor(self.client.node.get_clock().now().nanoseconds)
        ser=pub=0.;cells=messages=0;boxes=[];height,width=source.shape;side=224
        transactional=hasattr(self,'_transfer_id')
        if transactional:
            self._transfer_id+=1;transfer_id=self._transfer_id;digest=hashlib.sha256()
            self._transfer_receipts.clear()
        for x0,y0,w,h in dirty_chunk_boxes(dirty,side,overlap):
            if time.monotonic()>=self.request_deadline:raise ExactAckFailure('REQUEST_DEADLINE_PUBLICATION')
            x1=x0+w;y1=y0+h
            begin=time.monotonic();msg=self.OccupancyGridUpdate();msg.header.frame_id='map'
            msg.header.stamp=self.client.node.get_clock().now().to_msg()
            if transactional:
                msg.header.stamp.sec=transfer_id//10**9;msg.header.stamp.nanosec=transfer_id%10**9
            msg.x=x0;msg.y=y0;msg.width=x1-x0;msg.height=y1-y0
            msg.data=array('b',np.ascontiguousarray(source[y0:y1,x0:x1]).tobytes())
            if transactional:
                digest.update(struct.pack('<4I',x0,y0,x1-x0,y1-y0));digest.update(msg.data.tobytes())
            ser+=(time.monotonic()-begin)*1000;begin=time.monotonic()
            self._local_update_publisher.publish(msg)
            # Publishing progresses in native DDS; pump ready callbacks without
            # adding a fixed idle millisecond to every chunk. Content ACK,
            # reliability, deadline and repair requirements remain unchanged.
            self.client.executor.spin_once(timeout_sec=0.)
            pub+=(time.monotonic()-begin)*1000;cells+=(y1-y0)*(x1-x0);messages+=1
            boxes.append([x0,y0,x1-x0,y1-y0])
        stats={'serialization_ms':ser,'publication_ms':pub,'cells':cells,'messages':messages,'chunks':boxes}
        if transactional:
            # The server receipt confirms source consumption, never effective ACK.
            manifest={'version':1,'transfer_id':transfer_id,'messages':messages,'bytes':cells,
                      'chunks_sha256':digest.hexdigest(),'publication':asdict(self.active_publication)}
            payload=json.dumps(manifest,sort_keys=True,separators=(',',':'))
            marker=self.OccupancyGridUpdate();marker.header.frame_id='map'
            marker.header.stamp.sec=transfer_id//10**9;marker.header.stamp.nanosec=transfer_id%10**9
            marker.data=array('b',payload.encode())
            started=time.monotonic()
            if self._transaction_ack_deadline is None:
                self._transaction_ack_deadline=min(self.request_deadline,started+getattr(self,'_ready_ack_window_s',3.))
            self._local_update_publisher.publish(marker)
            while payload not in self._transfer_receipts and time.monotonic()<self._transaction_ack_deadline:
                self.client.executor.spin_once(timeout_sec=min(.005,max(0.,self._transaction_ack_deadline-time.monotonic())))
            consumed=payload in self._transfer_receipts
            stats.update(source_consumption_wait_ms=(time.monotonic()-started)*1000,
                         transfer_id=transfer_id,source_consumption_verified=consumed)
            self._trace({'event':'source_transaction_receipt',**stats,'manifest':manifest})
            if not consumed:raise ExactAckFailure('SOURCE_TRANSACTION_NOT_CONSUMED')
            self._consumed_manifest=payload
            self._atomic_buffer.reset_floor(self.client.node.get_clock().now().nanoseconds)
        return stats

    def update_local_mask(self,allowed_mask,**kwargs):
        try:
            return self._update_local_mask_transaction(allowed_mask,**kwargs)
        except BaseException:
            self._current_grid=None
            raise
        finally:
            # A static single-writer map needs fresh readbacks during an ACK,
            # not a continuous stream during connectors or the Smac action.
            # Every later transaction subscribes again and requires two new
            # complete exact frames after its own publication floor.
            self._stop_atomic_readback()

    def _update_local_mask_transaction(self,allowed_mask,**kwargs):
        started=time.monotonic();self.last_exact_ack=None;self._costmap_state_trusted=False
        self._transaction_ack_deadline=None
        self._local_mask_info={'costmap_update_acknowledged':False,'costmap_ack_mismatch_cells':None,
                               'request_id':self.request_id,'planner_search_started':False}
        mask,source=self._grid_for_mask(allowed_mask);build=time.monotonic()
        expected=self._expected(source)
        if not isinstance(expected,FrozenExpectedMaster):expected=FrozenExpectedMaster(expected)
        expected_ms=(time.monotonic()-build)*1000
        dirty=np.ones(source.shape,bool) if self._current_grid is None else source!=self._current_grid
        bbox=dirty_bbox(dirty)
        self.publication_sequence+=1
        token=Publication(self.publication_sequence,grid_hash(source),bbox,expected.sha256,source.shape,
                          self.ctx.map_sha256,self.request_id,getattr(self,"backend_context_binding",""))
        self.active_publication=token
        self._trace({'event':'publication_created','publication':asdict(token),'expected_build_ms':expected_ms})
        stats=self._publish_chunks(source,dirty)
        if hasattr(self,'_atomic_subscription'):self._start_atomic_readback()
        repairs=repair_cells=repair_chunks=0;repair_ms=readback_ms=scan_ms=0.;readback_errors=0;last=None
        stable=0;fallback=False;repair_history=[]
        ack_started=time.monotonic();ack_deadline=getattr(self,'_transaction_ack_deadline',None) or min(self.request_deadline,ack_started+getattr(self,'_ready_ack_window_s',3.))
        # A readback can observe the master while the inflation worker is still
        # processing queued updates. Wait a bounded processing interval before
        # spending a repair; equality itself is never inferred from elapsed time.
        settle_s=getattr(self,'_server_settle_grace_s',
                         min(.8,max(0.,getattr(self,'_effective_compute_ms',0.)/1000*1.25)))
        # Local oracle acceleration does not imply faster remote inflation.
        repair_not_before=ack_started+settle_s
        while time.monotonic()<ack_deadline:
            rb=time.monotonic()
            try:server,_response_time=self._server_costmap_snapshot(ack_deadline)
            except RuntimeError as exc:
                readback_ms+=(time.monotonic()-rb)*1000;readback_errors+=1
                self._trace({'event':'readback_error','error':str(exc)})
                if getattr(self,'_sealed_client',None) is not None:break
                continue
            readback_ms+=(time.monotonic()-rb)*1000;sc=time.monotonic()
            last=compare_exact(token,self.active_publication,expected,server)
            scan_ms+=(time.monotonic()-sc)*1000;self._trace({'event':'readback',**last})
            if last['acknowledged']:
                stable+=1
                if stable>=2:break
            else:
                stable=0
                if time.monotonic()<repair_not_before:pass
                elif repairs<2:
                    rb=time.monotonic();info=self._publish_chunks(source,server!=expected.array)
                    repair_ms+=(time.monotonic()-rb)*1000;repairs+=1
                    repair_cells+=info['cells'];repair_chunks+=info['messages'];repair_history.append(info)
                    repair_not_before=time.monotonic()+settle_s
                elif not fallback:
                    rb=time.monotonic();info=self._publish_chunks(source,np.ones(source.shape,bool))
                    repair_ms+=(time.monotonic()-rb)*1000;fallback=True;repair_history.append(info)
                    self._trace({'event':'full_update_fallback','publication':asdict(token),'stats':info})
                    repair_not_before=time.monotonic()+settle_s
            del server  # Release consumed mmap before another pair can be received.
            self.client.executor.spin_once(timeout_sec=min(.01,max(0.,ack_deadline-time.monotonic())))
        success=stable>=2 and last is not None and last['acknowledged'] and time.monotonic()<=ack_deadline
        total=(time.monotonic()-started)*1000
        self._local_mask_info={
            'costmap_update_acknowledged':success,'costmap_ack_status':'exact_verified' if success else 'exact_failed_closed',
            'planner_search_started':False,'costmap_ack_mismatch_cells':last['mismatch_cells'] if last else None,
            'costmap_ack_sequence':token.sequence,'publication':asdict(token),
            'server_costmap_content_hash':last['server_readback_hash'] if last else '',
            'hard_mismatch':last['hard_mismatch'] if last else None,'soft_mismatch':last['soft_mismatch'] if last else None,
            'stale_cells':last['stale_cells'] if last else None,'hash_mismatch':last['hash_mismatch'] if last else None,
            'sequence_mismatch':last['sequence_mismatch'] if last else None,
            'readback_error':readback_errors+getattr(self,'_snapshot_errors',0),
            'server_readback_source':'native_mutex_sealed_memfd' if getattr(self,'_sealed_client',None) else 'native_mutex_protected_costmap_raw',
            'server_snapshot_index':getattr(getattr(self,'_sealed_client',None) or getattr(self,'_atomic_buffer',None),'last_index',None),
            'server_snapshot_stamp_ns':getattr(getattr(self,'_sealed_client',None) or getattr(self,'_atomic_buffer',None),'last_stamp_ns',None),
            'costmap_ack_repair_count':repairs,'costmap_ack_repair_cells':repair_cells,'repaired_chunks':repair_chunks,
            'full_update_fallback':fallback,'repair_history':repair_history,
            'roi_build_ms':(build-started)*1000,'expected_effective_build_ms':expected_ms,
            'expected_effective_cache_hit':getattr(self,'_expected_cache_hit',False),
            'expected_effective_cache_bytes':getattr(self,'_expected_cache_bytes',0),
            'repair_processing_grace_s':settle_s,
            'source_consumption_wait_ms':stats.get('source_consumption_wait_ms',0.),
            'source_consumption_verified':stats.get('source_consumption_verified',False),
            'serialization_ms':stats['serialization_ms'],'publication_ms':stats['publication_ms'],
            'costmap_ack_wait_ms':(time.monotonic()-ack_started)*1000,'readback_ms':readback_ms,
            'mismatch_scan_ms':scan_ms,'repair_ms':repair_ms,'local_map_update_ms':total,
            'costmap_update_ms':total,'total_costmap_update_ms':total,'roi_bbox':list(bbox),
            'local_map_update_mode':'exact_roi','local_map_update_messages':stats['messages'],
            'local_map_update_cells':stats['cells'],'local_map_update_bytes':stats['cells'],
            'local_map_update_fallback':fallback,'local_map_update_fallback_reason':'exact_mismatch' if fallback else '',
            'applied_mask_hash':token.source_grid_hash,'expected_mask_hash':token.source_grid_hash}
        self._trace({'event':'transaction_complete',**self._local_mask_info})
        if not success:
            # A failed partial publication invalidates the previous source baseline.
            self._current_grid=None
            raise ExactAckFailure('EXACT_ACK_FAILED_CLOSED')
        self._current_grid=source.copy();self._current_allowed_mask=mask.copy();self._costmap_state_trusted=True
        self.last_exact_ack=token
        return dict(self._local_mask_info)

    @defer_transport_gc()
    def _set_server_budget(self,remaining):
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter
        if getattr(self,'_budget_state_uncertain',False):raise ExactAckFailure('SESSION_SERVER_BUDGET_UNCERTAIN')
        started=time.monotonic()
        reserve=max(1.25,getattr(self,'_budget_update_peak_s',0.)+.3)
        budget=min(5.,math.floor(max(0.,remaining-reserve)*2.)/2.)
        confirmed=getattr(self,'_confirmed_server_budget',None)
        # Rebuilding Smac's lookup table merely to gain half a second is
        # counterproductive. Keep a confirmed fitting budget unless the
        # potential upgrade exceeds the measured/conservative rebuild reserve.
        # A limit which no longer fits must still be reduced and confirmed.
        if confirmed is not None and confirmed<=remaining-.3 and budget-confirmed<=reserve:
            self._budget_diagnostics={'server_budget_cache_hit':True,'server_budget_update_ms':0.,
                                      'smac_remaining_budget_s':confirmed,
                                      'server_budget_reuse_policy':'upgrade_gain_exceeds_rebuild_reserve_v1',
                                      'server_budget_upgrade_gain_s':max(0.,budget-confirmed),
                                      'server_budget_rebuild_reserve_s':reserve}
            self._trace({'event':'server_search_budget','budget_s':confirmed,**self._budget_diagnostics})
            return confirmed
        if budget<=0:raise ExactAckFailure('REQUEST_BUDGET_TOO_SMALL_TO_RECONFIGURE')
        try:
            if self._budget_client is None:
                self._budget_client=self.client.node.create_client(SetParameters,'/planner_server/set_parameters')
            if not self._budget_client.wait_for_service(timeout_sec=min(.2,max(0.,remaining))):
                raise ExactAckFailure('REQUEST_BUDGET_SERVICE_UNAVAILABLE')
            request=SetParameters.Request()
            request.parameters=[Parameter('GridBased.max_planning_time',Parameter.Type.DOUBLE,budget).to_parameter_msg()]
            # An unanswered RPC may already have changed the server. Retire the
            # old confirmation before dispatch; ambiguous outcomes require a new session.
            self._confirmed_server_budget=None;self._budget_state_uncertain=True
            future=self._budget_client.call_async(request)
            while not future.done() and time.monotonic()<self.request_deadline:
                self.client.executor.spin_once(timeout_sec=.005)
            if not future.done():raise ExactAckFailure('REQUEST_BUDGET_UPDATE_FAILED')
            results=future.result().results
            if len(results)!=1 or not results[0].successful:raise ExactAckFailure('REQUEST_BUDGET_UPDATE_FAILED')
        except Exception as exc:
            self._budget_diagnostics={'server_budget_cache_hit':False,
                                      'server_budget_update_ms':(time.monotonic()-started)*1000,
                                      'server_budget_requested_s':budget,
                                      'server_budget_state_uncertain':getattr(self,'_budget_state_uncertain',False),
                                      'server_budget_error':str(exc)}
            self._trace({'event':'server_budget_error',**self._budget_diagnostics})
            if isinstance(exc,ExactAckFailure):raise
            raise ExactAckFailure('REQUEST_BUDGET_TRANSPORT_ERROR') from exc
        elapsed=time.monotonic()-started
        self._confirmed_server_budget=budget;self._budget_state_uncertain=False
        self._budget_update_peak_s=max(getattr(self,'_budget_update_peak_s',0.),elapsed)
        self._budget_diagnostics={'server_budget_cache_hit':False,'server_budget_update_ms':elapsed*1000,
                                  'smac_remaining_budget_s':budget}
        self._trace({'event':'server_search_budget','budget_s':budget,**self._budget_diagnostics})
        return budget

    def plan(self,query,spec,*,allowed_mask=None,**kwargs):
        try:
            contract=getattr(self,'runtime_safety_contract',None)
            if not contract or contract.get('verified') is not True or contract.get('map_hash')!=self.ctx.map_sha256:
                raise ExactAckFailure('RUNTIME_SAFETY_CONTRACT_REQUIRED')
            if getattr(self,'_unresolved_timeout',False):raise ExactAckFailure('SESSION_UNRESOLVED_ACTION_TIMEOUT')
            if getattr(self,'_budget_state_uncertain',False):raise ExactAckFailure('SESSION_SERVER_BUDGET_UNCERTAIN')
            if allowed_mask is None:raise ExactAckFailure('EXACT_ACK_MASK_REQUIRED')
            info=self.update_local_mask(allowed_mask)
            if self.last_exact_ack is None or self.last_exact_ack != self.active_publication or self.last_exact_ack.request_id!=self.request_id:
                raise ExactAckFailure('EXACT_ACK_TRANSACTION_CHANGED')
            remaining=self.request_deadline-time.monotonic()
            if remaining<=0:raise ExactAckFailure('REQUEST_DEADLINE_BEFORE_SEARCH')
            if getattr(self,'_enforce_server_budget',False):
                info['smac_remaining_budget_s']=self._set_server_budget(remaining)
                info.update(getattr(self,'_budget_diagnostics',{}))
            remaining=self.request_deadline-time.monotonic()
            if remaining<=0:raise ExactAckFailure('REQUEST_DEADLINE_BEFORE_SEARCH')
            if getattr(self,'_enforce_server_budget',False) and info['smac_remaining_budget_s']>remaining-.3:
                raise ExactAckFailure('SERVER_BUDGET_EXCEEDS_REMAINING_DEADLINE')
            self.client.timeout=min(7.,remaining)
            with defer_transport_gc():
                result=super().plan(query,spec,allowed_mask=None,**kwargs)
            result.diagnostics={**(result.diagnostics or {}),**info,'planner_search_started':True,
                                'runtime_safety_contract_verified':True,'runtime_safety_contract_sha256':contract['sha256']}
            if result.failure_code=='CLIENT_TIMEOUT':
                self._unresolved_timeout=True
                self._trace({'event':'unresolved_action_timeout','requires_batch_restart':True})
            return result
        except ExactAckFailure as exc:
            return PlanResult(planner_backend=spec.backend,backend_version=spec.version,
                              source=kwargs.get('source','hybrid_astar'),failure_code=str(exc),
                              failure_detail=str(exc),diagnostics={**self._local_mask_info,**getattr(self,'_budget_diagnostics',{}),'planner_search_started':False,
                                                                 'request_id':self.request_id})
