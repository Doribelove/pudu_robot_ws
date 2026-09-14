"""Same-resolution, bounded native ROI contexts; global canonical audit remains final.

The ROI includes the maximum permitted retry corridor plus inflation/footprint
halo. Oversized rectangles are refused before allocation, never truncated.
"""
from __future__ import annotations
from dataclasses import dataclass, asdict
from types import SimpleNamespace
from pathlib import Path
from collections import deque
import hashlib
import json
import math
import time
import numpy as np
from .exact_ack_r3 import ExactAckSmacSession, ExactAckFailure, AtomicReadbackBuffer, master_snapshot_frequency
from .sealed_snapshot_r3 import SealedSnapshotClient
from .path_audit import PathAuditor
from . import two_layer_v1_r3_benchmark as runner

VERSION='r3-next10-smoothing-budget-floor-rc8'
MAX_CELLS=80*1024*1024
READY_CONTEXT_MAX_CELLS=32*1024*1024


def canonical(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)


@dataclass(frozen=True)
class RoiBounds:
    top: int
    left: int
    height: int
    width: int

    @property
    def cells(self):return self.height*self.width

    def capacity(self,max_cells=MAX_CELLS):
        info=runner.pinned_smac_capacity(self.width,self.height)
        info.update(roi_bbox_top_left=[self.left,self.top,self.width,self.height],
                    cells=self.cells,max_cells=max_cells,memory_supported=self.cells<=max_cells)
        info['index_supported']=info['supported']
        info['failure_code']=('PINNED_SMAC_INDEX_CAPACITY' if not info['index_supported'] else
                              'ROI_MEMORY_CELL_LIMIT' if not info['memory_supported'] else '')
        info['supported']=info['supported'] and info['memory_supported']
        return info


def route_bounds(map_,route,*,max_cells=MAX_CELLS):
    if map_.resolution!=.05 or map_.origin[2]!=0:raise ValueError('ROI_FROZEN_GEOMETRY_REQUIRED')
    if not route.polyline:raise ValueError('ROI_EMPTY_ROUTE')
    cells=[map_.world_to_cell(*p[:2]) for p in route.polyline]
    if any(p is None for p in cells):raise ValueError('ROI_ROUTE_OUTSIDE_MAP')
    # Maximum corner widening 4m + 0.5m clearance term (retry); raster roundoff,
    # complete inflation support and footprint bounding circle remain inside.
    corridor_radius=max(4.5,2.+runner.baseline.FOOTPRINT_SAFETY_MARGIN_M+runner.baseline.BEND_MARGIN_M)
    halo=math.ceil((corridor_radius+.55+max(math.hypot(*v) for v in runner.legacy.FOOTPRINT))/.05)+3
    rows,cols=zip(*cells)
    top=max(0,min(rows)-halo);left=max(0,min(cols)-halo)
    bottom=min(map_.height,max(rows)+halo+1);right=min(map_.width,max(cols)+halo+1)
    return RoiBounds(top,left,bottom-top,right-left)


class WindowField:
    def __init__(self,field,bounds):self.field=field;self.bounds=bounds
    def __getitem__(self,key):
        r,c=key;return self.field[np.asarray(r)+self.bounds.top,np.asarray(c)+self.bounds.left]
    def minimum_at(self,cells):
        translated=[None if cell is None else (cell[0]+self.bounds.top,cell[1]+self.bounds.left) for cell in cells]
        return self.field.minimum_at(translated)
    def capped_at(self,cells,cap):
        return self.field.capped_at([(r+self.bounds.top,c+self.bounds.left) for r,c in cells],cap)


class RoiMap:
    def __init__(self,global_map,bounds):
        self.global_map=global_map;self.bounds=bounds
        self.width=bounds.width;self.height=bounds.height;self.resolution=global_map.resolution
        self.origin=(global_map.origin[0]+bounds.left*self.resolution,
                     global_map.origin[1]+(global_map.height-bounds.top-bounds.height)*self.resolution,0.)
        self.occupancy=global_map.occupancy[bounds.top:bounds.top+bounds.height,bounds.left:bounds.left+bounds.width]
        self.distance_m=WindowField(global_map.distance_m,bounds)
        self.image_path=global_map.image_path;self.yaml_path=global_map.yaml_path
    def world_to_cell(self,x,y):
        cell=self.global_map.world_to_cell(x,y)
        if cell is None:return None
        r,c=cell[0]-self.bounds.top,cell[1]-self.bounds.left
        return (r,c) if 0<=r<self.height and 0<=c<self.width else None
    def cell_to_world(self,cell):
        return self.global_map.cell_to_world((cell[0]+self.bounds.top,cell[1]+self.bounds.left))
    def clearance(self,*args):return self.global_map.clearance(*args)
    def footprint_collision(self,*args,**kwargs):return self.global_map.footprint_collision(*args,**kwargs)


def crop_context(ctx,bounds):
    return SimpleNamespace(**{**vars(ctx),'hospital_map':RoiMap(ctx.hospital_map,bounds),'bounded_memory':True})


def bootstrap_context(ctx,directory):
    # Never start the map server with the original xlarge image. A tiny blocked
    # map is sufficient until a certified request context is installed.
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    pgm=directory/'bootstrap.pgm';yaml=directory/'bootstrap.yaml'
    pgm.write_bytes(b'P5\n16 16\n255\n'+bytes(256))
    import yaml as y
    yaml.write_text(y.safe_dump({'image':str(pgm),'resolution':.05,'origin':list(ctx.hospital_map.origin),
                                'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
    result=crop_context(ctx,RoiBounds(ctx.hospital_map.height-16,0,16,16))
    result.hospital_map.occupancy=np.full((16,16),100,np.int8);result.map_yaml=yaml
    return result


class RoiCorridorCache:
    def __init__(self,session,straight_half_width_m=1.2):
        self.session=session;self.cache=runner.CorridorCache(straight_half_width_m=straight_half_width_m)
    def get(self,ctx,route,expansion=False):
        bounds=self.session.bounds_for_route(route)
        cap=bounds.capacity(self.session.max_cells)
        self.session.last_roi_capacity=cap
        if not cap['supported']:raise ExactAckFailure('ROI_BACKEND_CAPACITY')
        local=crop_context(ctx,bounds)
        started=time.monotonic();self.session.activate_context(local,bounds)
        switched_ms=(time.monotonic()-started)*1000
        mask,info=self.cache.get(local,route,expansion)
        self.session.local_auditor=PathAuditor(local,source_commit=VERSION)
        return mask,{**info,'backend_context_switch_ms':switched_ms,'backend_capacity':cap,
                     'backend_context':json.loads(self.session.backend_context_binding)}


class GlobalRoiAuditor:
    """Require BOTH unchanged global canonical audit and ROI canonical audit.

    Global metrics and footprint decisions are authoritative. The additional
    canonical call validates the cropped corridor at exactly the same sampling
    density, without materializing a global mask or relaxing any threshold.
    """
    def __init__(self,global_auditor,session):self.global_auditor=global_auditor;self.session=session
    def __getattr__(self,name):return getattr(self.global_auditor,name)
    def audit(self,query,points,mask):
        global_result=self.global_auditor.audit(query,points,None)
        local_result=self.session.local_auditor.audit(query,points,mask)
        global_result.within_mask=global_result.within_mask and local_result.within_mask
        if not local_result.final_valid_success and not global_result.metrics.get('failure_code'):
            global_result.metrics['failure_code']=local_result.metrics.get('failure_code') or 'ROI_CANONICAL_AUDIT_REJECTED'
        global_result.metrics['final_valid_success']=global_result.final_valid_success
        geometry=json.loads(self.session.backend_context_binding)
        geometry.pop('generation')  # Transaction identity is in ACK, not mask geometry.
        global_result.mask_hash=hashlib.sha256((canonical(geometry)+local_result.mask_hash).encode()).hexdigest()
        global_result.timings['roi_canonical_audit_ms']=local_result.timings['canonical_path_audit_ms']
        global_result.timings['canonical_path_audit_ms']+=local_result.timings['canonical_path_audit_ms']
        return global_result


class RoiExactAckSmacSession(ExactAckSmacSession):
    roi_mode=True
    def __init__(self,ctx,output,*,max_cells=MAX_CELLS,
                 ready_context_max_cells=READY_CONTEXT_MAX_CELLS,corridor_straight_half_width_m=1.2,**kwargs):
        self.global_ctx=ctx;self.max_cells=int(max_cells)
        if not 256<=self.max_cells<=MAX_CELLS:raise ValueError('ROI_MAX_CELLS_OUT_OF_RANGE')
        if not 0<=ready_context_max_cells<=MAX_CELLS:
            raise ValueError('READY_CONTEXT_MAX_CELLS_OUT_OF_RANGE')
        self.ready_context_max_cells=min(int(ready_context_max_cells),self.max_cells)
        boot=bootstrap_context(ctx,Path(output)/'backend_context')
        kwargs['map_yaml']=boot.map_yaml
        super().__init__(boot,output,**kwargs)
        import yaml
        params=yaml.safe_load(self.params_file.read_text())
        params['global_costmap']['global_costmap']['ros__parameters']['publish_frequency']=master_snapshot_frequency(self.max_cells)
        params['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']['plugin']='pln_transactional_costmap/BoundedRoiInflationLayer'
        config=params['global_costmap']['global_costmap']['ros__parameters']['static_layer']
        config.update(bounded_context=True,context_max_cells=self.max_cells,
                      context_map_hash=ctx.map_sha256,context_map_yaml_hash=ctx.map_yaml_sha256,
                      context_global_width=ctx.hospital_map.width,context_global_height=ctx.hospital_map.height,
                      context_global_origin_x=float(ctx.hospital_map.origin[0]),
                      context_global_origin_y=float(ctx.hospital_map.origin[1]))
        self.params_file.write_text(yaml.safe_dump(params,sort_keys=False))
        self.smac_config_hash=hashlib.sha256(self.params_file.read_bytes()).hexdigest()
        self.backend_context_binding='';self.context_generation=0;self.context_uncertain=False
        self._context_receipts=deque(maxlen=8);self._context_bounds=None
        self._corridor_cache=RoiCorridorCache(self,corridor_straight_half_width_m)

    def bounds_for_route(self,route):
        m=self.global_ctx.hospital_map
        query_bounds=route_bounds(m,route)
        # A bounded map fits one persistent native context. Preserve its exact
        # global origin/lattice across requests; avoid needless resize and
        # floating-coordinate changes. Larger maps keep query-sized contexts.
        if m.height*m.width<=self.ready_context_max_cells:
            return RoiBounds(0,0,m.height,m.width)
        return query_bounds

    def prepare_ready_baseline(self,timeout_s=90.):
        m=self.global_ctx.hospital_map
        if m.height*m.width>self.ready_context_max_cells:
            return super().prepare_ready_baseline(timeout_s)
        if not math.isfinite(timeout_s) or timeout_s<=0:
            raise ValueError('READY baseline timeout must be positive and finite')
        started=time.monotonic()
        self.begin_request('READY:closed-baseline',started+timeout_s)
        self._ready_ack_window_s=timeout_s
        bounds=RoiBounds(0,0,m.height,m.width)
        try:
            self.activate_context(crop_context(self.global_ctx,bounds),bounds)
            # The staged all-blocked template is not an ACK. A source commit
            # and two complete exact snapshots are still required, including
            # when no source cells differ from that native template.
            info=self.update_local_mask(np.zeros((m.height,m.width),bool))
            info.update(ready_baseline_wall_ms=(time.monotonic()-started)*1000,
                        ready_baseline_kind='bounded_persistent_global_context',
                        ready_context_cells=bounds.cells,query_dependent=False,
                        planner_search_started=False)
            self.ready_baseline_receipt=info
            return dict(info)
        finally:
            del self._ready_ack_window_s

    def start(self):
        super().start()
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile,QoSReliabilityPolicy
        qos=QoSProfile(depth=8,reliability=QoSReliabilityPolicy.RELIABLE)
        self._context_pub=self.client.node.create_publisher(String,'/map_context',qos)
        self._context_sub=self.client.node.create_subscription(String,'/map_context_receipt',
            lambda msg:self._context_receipts.append(msg.data),qos)

    def plan(self,query,spec,**kwargs):
        if self.context_uncertain or not self.backend_context_binding:
            return runner.legacy.PlanResult(planner_backend=spec.backend,backend_version=spec.version,
                failure_code='ROI_CONTEXT_NOT_CONFIRMED',diagnostics={'planner_search_started':False})
        return super().plan(query,spec,**kwargs)

    def activate_context(self,local,bounds):
        if self._action_in_progress or self._unresolved_timeout or self.context_uncertain:
            raise ExactAckFailure('ROI_CONTEXT_SESSION_UNCERTAIN')
        if bounds==self._context_bounds:return
        if not bounds.capacity(self.max_cells)['supported']:raise ExactAckFailure('ROI_BACKEND_CAPACITY')
        self.context_generation+=1
        binding=canonical({'version':1,'generation':self.context_generation,
            'map_hash':self.global_ctx.map_sha256,'map_yaml_hash':self.global_ctx.map_yaml_sha256,
            'global_shape':[self.global_ctx.hospital_map.height,self.global_ctx.hospital_map.width],
            'bbox':[bounds.left,bounds.top,bounds.width,bounds.height],
            'origin':list(local.hospital_map.origin),'resolution':.05,'max_cells':self.max_cells})
        command=canonical({'binding':binding,'transfer_floor':self._transfer_id,
                           'deadline_ns':int(self.request_deadline*1e9)})
        from std_msgs.msg import String
        self._costmap_state_trusted=False;self.last_exact_ack=None;self.active_publication=None
        self._current_grid=None;self._current_allowed_mask=None;self.context_uncertain=True
        self._stop_atomic_readback();self._context_receipts.clear()
        while self._context_pub.get_subscription_count()<1 and time.monotonic()<self.request_deadline:
            self.client.executor.spin_once(timeout_sec=.005)
        self._context_pub.publish(String(data=command))
        while command not in self._context_receipts and time.monotonic()<self.request_deadline:
            self.client.executor.spin_once(timeout_sec=.005)
        if command not in self._context_receipts:raise ExactAckFailure('ROI_CONTEXT_INSTALL_TIMEOUT')
        self.context_uncertain=False;self.backend_context_binding=binding;self._context_bounds=bounds
        self.ctx=local;self._atomic_buffer=AtomicReadbackBuffer(local.hospital_map)
        self._sealed_client=SealedSnapshotClient(self._snapshot_path,local.hospital_map,self.planner_pid)
        # Native staging specifies an all-blocked source template. The next
        # source commit atomically installs geometry and its changed cells.
        # Staging receipt is NOT effective ACK: every byte is checked afterward.
        self._current_grid=np.full((bounds.height,bounds.width),100,np.int8)
        self._trace({'event':'roi_context_staged','binding':json.loads(binding),
                     'source_receipt_only':True,'transfer_floor':self._transfer_id})
