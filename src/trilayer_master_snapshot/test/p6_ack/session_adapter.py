"""Test adapter for unchanged P6 exact ACK using a real Nav2 snapshot service.

This carries synthetic expected-content state; it never fabricates server data.
"""
import os
import time
from types import SimpleNamespace
import numpy as np
from nav2_msgs.srv import GetCostmap
from p6_ack_fixture.r6_delta import DeltaDecoder
from p6_ack_fixture.r6_ack import grid_digest, wait_exact


class SnapshotSession:
    PUBLICATION_VERSION = 'synthetic-static-layer-contract-test'

    def __init__(self, client, executor, shape, resolution, origin=(0.,0.)):
        # Exercise the original unoptimized exact-compare path, not hidden
        # historical environment settings or mocks of its decision function.
        for name in ('THREEYD_V9_MODE','THREEYD_V10_R4_MODE','THREEYD_V8_FIRST_MODE'):
            if os.environ.get(name, 'control') != 'control':
                raise RuntimeError('test requires control branch: '+name)
        self.client=SimpleNamespace(executor=executor)
        self._service=client
        self._shape=shape;self._resolution=resolution;self._origin=origin
        self._r6_decoder=DeltaDecoder(shape,version=2)
        self._last_exact_expected_master=None
        self._semantic_publication_sequence=0
        self._active_publication_baseline_timestamp_ns=-1
        self._costmap_ack_sequence=self._semantic_roi_sequence=0
        self._semantic_ack_cache={};self._exact_ack_cache={}
        self.exact_stable_observations=2
        self.costmap_ack_timeout_s=1.
        self.current_query_id='synthetic-static-layer'
        self.ack_events=[];self.traces=[]
        self.before_read=None

    def expect(self, occupancy, master):
        occupancy=np.asarray(occupancy,np.int8).reshape(self._shape)
        master=np.asarray(master,np.uint8).reshape(self._shape)
        # P6 semantic storage is vertically opposite ROS row-major storage.
        self._semantic_costmap=SimpleNamespace(
            occupancy_grid=np.flipud(occupancy).copy(),
            expected_master_cost=np.flipud(master).copy(),
            hard_semantic_mask=np.flipud(master==254).copy(),
            soft_cost=np.flipud(np.where((master>0)&(master<254),master,0)).copy(),
            policy_hash='synthetic-static-only')
        self._semantic_publication_sequence+=1
        self._active_publication_bbox=(0,0,self._shape[1],self._shape[0])
        self.expected=occupancy.copy();self.changed=np.ones(self._shape,bool)

    @staticmethod
    def _content_signature(semantic):
        return (semantic.policy_hash,grid_digest(semantic.occupancy_grid),
                grid_digest(semantic.expected_master_cost))

    def _exact_cache_key(self, **kwargs):
        return tuple(sorted(kwargs.items()))

    def _append_ack_trace(self, filename, result):
        self.traces.append((filename,result))

    def _server_costmap_snapshot(self, deadline):
        if self.before_read is not None:self.before_read(self)
        request=GetCostmap.Request();request.specs.layer=self._r6_decoder.request_layer()
        future=self._service.call_async(request)
        while not future.done() and time.monotonic()<deadline:
            self.client.executor.spin_once(timeout_sec=min(.01,max(0.,deadline-time.monotonic())))
        if not future.done():
            self._service.remove_pending_request(future);future.cancel()
            self._r6_decoder.invalidate()
            raise RuntimeError('snapshot deadline')
        msg=future.result().map;m=msg.metadata
        if ((m.size_y,m.size_x)!=self._shape or msg.header.frame_id!='map' or
            abs(m.resolution-self._resolution)>1e-7 or
            abs(m.origin.position.x-self._origin[0])>1e-7 or
            abs(m.origin.position.y-self._origin[1])>1e-7 or
            abs(m.origin.orientation.w-1.)>1e-7 or
            any(abs(getattr(m.origin.orientation,k))>1e-7 for k in ('x','y','z'))):
            self._r6_decoder.invalidate();raise RuntimeError('snapshot geometry mismatch')
        grid,_=self._r6_decoder.decode(msg.data,m.layer)
        return grid,m.update_time.sec*1000000000+m.update_time.nanosec

    def wait(self, timeout=1.):
        return wait_exact(self,self.expected,self.changed,timeout_s=timeout)
