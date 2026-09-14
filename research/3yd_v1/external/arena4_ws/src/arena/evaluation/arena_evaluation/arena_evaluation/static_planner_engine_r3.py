"""One immutable-map planning engine for the supervised static service.

This owns a single ROS session. Cancellation, deadlines for synchronous work,
queueing, and recovery are enforced by its external supervisor. The engine
never keeps a growing list of request results or accepts an uncertain session.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from collections import deque
import hashlib
import math
import os
import re
import time

from . import two_layer_v1_r3_benchmark as runner
from .bounded_context_r3 import bounded_context
from .path_audit import PathAuditor
from .planner_benchmark.models import Query
from .roi_backend_r3 import RoiExactAckSmacSession,GlobalRoiAuditor,VERSION,MAX_CELLS
from .tiled_topology_r3 import TileConfig
from .reusable_tile_preparation_r3 import ReusableTileTopology
from .bounded_query_preparation_r3 import BoundedQueryTopologyPreparer,MAX_QUERY_CACHE_BYTES
from .owned_processes_r3 import process_stat

PINNED_SMAC_SHA256='2c62b5586c7cbac1665c7e7660a833dca3124c965185de9cad996d00927b7739'


@dataclass(frozen=True)
class EngineConfig:
    map_yaml: str
    map_id: str
    cache: str
    ros_domain_id: int
    roi_max_cells: int=MAX_CELLS
    ready_context_max_cells: int=32*1024**2
    ready_ack_timeout_s: float=90.
    query_budget_s: float=7.
    query_cache_max_bytes: int=MAX_QUERY_CACHE_BYTES

    def __post_init__(self):
        if not Path(self.map_yaml).is_file():raise ValueError('MAP_YAML_NOT_FOUND')
        object.__setattr__(self,'map_yaml',str(Path(self.map_yaml).resolve()))
        object.__setattr__(self,'cache',str(Path(self.cache).resolve()))
        if not isinstance(self.map_id,str) or re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}',self.map_id) is None:
            raise ValueError('MAP_ID_INVALID')
        if not 0<=self.ros_domain_id<=232:raise ValueError('ROS_DOMAIN_ID_INVALID')
        if not 256<=self.roi_max_cells<=MAX_CELLS:raise ValueError('ROI_CAPACITY_INVALID')
        if not 0<=self.ready_context_max_cells<=self.roi_max_cells:raise ValueError('READY_CAPACITY_INVALID')
        if not isinstance(self.query_cache_max_bytes,int) or isinstance(self.query_cache_max_bytes,bool) or not 1<=self.query_cache_max_bytes<=MAX_QUERY_CACHE_BYTES:
            raise ValueError('QUERY_CACHE_BYTE_LIMIT_INVALID')
        if self.query_budget_s!=7.:raise ValueError('FROZEN_REQUEST_BUDGET_REQUIRED')
        if not math.isfinite(self.ready_ack_timeout_s) or not 0<self.ready_ack_timeout_s<=90.:
            raise ValueError('READY_ACK_TIMEOUT_INVALID')


class StaticPlannerEngine:
    def __init__(self,config,output):
        self.config=config;self.output=Path(output).resolve();self.session=None
        self.state='INITIALIZING';self.initialization={};self._planner_identity=None

    def backend_available(self):
        if self._planner_identity is None:return False
        pid,born=self._planner_identity;row=process_stat(pid)
        return row is not None and row['start_ticks']==born and row['state'] not in {'Z','X'}

    def start(self):
        if self.session is not None or self.state!='INITIALIZING':raise RuntimeError('ENGINE_ALREADY_STARTED')
        self.output.mkdir(parents=True,exist_ok=False)
        os.environ['ROS_DOMAIN_ID']=str(self.config.ros_domain_id)
        os.environ['ROS_LOG_DIR']=str(self.output/'ros_logs')
        started=time.monotonic()
        try:
            begin=time.monotonic()
            self.ctx=bounded_context(self.config.map_yaml,self.config.map_id,self.config.cache,runner.legacy.FOOTPRINT)
            self.initialization['map_load_wall_ms']=(time.monotonic()-begin)*1000
            begin=time.monotonic()
            self.tile=ReusableTileTopology(self.ctx.hospital_map,runner.legacy.FOOTPRINT,
                                           self.config.cache,TileConfig())
            self.tile.build_coarse();self.initialization.update(self.tile.prepare_map())
            self.initialization['topology_prepare_wall_ms']=(time.monotonic()-begin)*1000
            self.preparer=BoundedQueryTopologyPreparer(self.tile,runner.legacy.FOOTPRINT,max_bytes=self.config.query_cache_max_bytes)
            global_auditor=PathAuditor(self.ctx,source_commit=VERSION)
            self.spec=runner.legacy.backend_availability()['hybrid_astar']
            if not self.spec.available:raise RuntimeError('PINNED_SMAC_UNAVAILABLE:'+self.spec.reason)
            self.session=RoiExactAckSmacSession(self.ctx,self.output,
                max_cells=self.config.roi_max_cells,ready_context_max_cells=self.config.ready_context_max_cells,
                map_yaml=self.ctx.map_yaml,log_tag='service',local_mask_updates=True,
                optimization_profile='v7_candidate',smac_parameter_profile='lighter_smoother',
                optimization_stage='step3_delta_map',planner_parameter_overrides={
                    'benchmark_instrumentation':True,'smoother':{'w_data':.25,'w_smooth':.25}})
            self.session.ack_trace=deque(maxlen=256)
            self.auditor=GlobalRoiAuditor(global_auditor,self.session)
            begin=time.monotonic();self.session.start()
            identity=process_stat(self.session.planner_pid)
            if identity is None or identity['state'] in {'Z','X'}:raise RuntimeError('BACKEND_PROCESS_DIED')
            self._planner_identity=(self.session.planner_pid,identity['start_ticks'])
            self.initialization['ros_start_wall_ms']=(time.monotonic()-begin)*1000
            if not self.session._get_costmap_client.wait_for_service(timeout_sec=30.):
                raise RuntimeError('COSTMAP_STARTUP_SERVICE_UNAVAILABLE')
            text=Path(f'/proc/{self.session.planner_pid}/maps').read_text()
            libraries=sorted({line.split()[-1] for line in text.splitlines() if 'libnav2_smac_planner.so' in line})
            if not libraries or any(hashlib.sha256(Path(f).read_bytes()).hexdigest()!=PINNED_SMAC_SHA256 for f in libraries):
                raise RuntimeError('PINNED_SMAC_BINARY_MISMATCH')
            ready=self.session.prepare_ready_baseline(self.config.ready_ack_timeout_s)
            if not self.backend_available():raise RuntimeError('BACKEND_PROCESS_DIED')
            if not ready.get('costmap_update_acknowledged') or ready.get('costmap_ack_mismatch_cells')!=0:
                raise RuntimeError('READY_EXACT_ACK_REQUIRED')
            self.initialization.update(total_initialization_wall_ms=(time.monotonic()-started)*1000,
                                       ready_baseline_wall_ms=ready['ready_baseline_wall_ms'],
                                       query_preparation_before_ready=0)
            self.state='READY'
            return {'state':self.state,'map_hash':self.ctx.map_sha256,'map_yaml_hash':self.ctx.map_yaml_sha256,
                    'implementation_revision':VERSION,'initialization':dict(self.initialization),
                    'ready_receipt':ready,'planner_pid':self.session.planner_pid}
        except BaseException:
            self.state='FAILED'
            self.close()
            raise

    def plan(self,request,deadline):
        if self.state!='READY':raise RuntimeError('ENGINE_NOT_READY')
        for key in ['start','goal']:
            pose=request.get(key)
            if not isinstance(pose,(list,tuple)) or len(pose)!=3 or any(
                    isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in pose):
                raise ValueError('INVALID_ENDPOINT_POSE')
        rid=request.get('request_id')
        if not isinstance(rid,str) or not rid or len(rid)>128:raise ValueError('INVALID_REQUEST_ID')
        if not math.isfinite(deadline):raise ValueError('INVALID_REQUEST_DEADLINE')
        if not self.backend_available():
            self.state='FAILED'
            return {'request_id':rid,'failure_code':'BACKEND_PROCESS_DIED','final_valid_success':False,
                    'points':[],'planner_search_started':False,'session_recovery_required':True}
        remaining=min(self.config.query_budget_s,deadline-time.monotonic())
        if remaining<=0:return {'request_id':rid,'failure_code':'REQUEST_DEADLINE','final_valid_success':False,'points':[]}
        query=Query(query_id=rid,start=list(request['start']),goal=list(request['goal']),category='static_service',seed=0)
        self.state='BUSY'
        try:
            result=runner.run_query(self.ctx,query,None,None,self.session,self.spec,self.auditor,
                                    rid,remaining,preparer=self.preparer)
            result['implementation_revision']=VERSION
            result['session_recovery_required']=bool(self.session._unresolved_timeout or
                self.session._budget_state_uncertain or self.session.context_uncertain)
            if not self.backend_available():
                result.update(final_valid_success=False,failure_code='BACKEND_PROCESS_DIED',
                              session_recovery_required=True)
            # Refused candidates remain explicit diagnostic evidence. Only
            # final-valid output is exposed in the path returned for execution.
            if not result['final_valid_success']:
                result['rejected_candidate_points']=result.pop('points',[])
                result['points']=[]
            self.state='FAILED' if result['session_recovery_required'] else 'READY'
            return result
        except BaseException:
            self.state='FAILED'
            raise

    def close(self):
        if self.session is not None:
            try:self.session.close()
            finally:self.session=None
        if self.state!='FAILED':self.state='STOPPED'
