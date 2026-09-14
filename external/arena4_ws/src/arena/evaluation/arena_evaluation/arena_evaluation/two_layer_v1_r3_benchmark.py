"""Fixed-input 2A-V1 r3 stage runner. All paths are write-once per attempt."""
from __future__ import annotations
import argparse
import csv
from collections import OrderedDict
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import resource
import sys
import time
from types import SimpleNamespace
import cv2
import numpy as np

from . import unified_four_backends_smoke as legacy
from . import l1_l3_corridor_hybrid_smoke as baseline
from .path_audit import PathAuditor
from .reachable_endpoint_r3 import ConnectorConfig,ReachableEndpointSelector,REVISION,digest
from .tiled_topology_r3 import (TiledTopology,TileConfig,atomic_json,load_map_bounded,
                               TopologyDeadlineExceeded)
from .exact_ack_r3 import ExactAckSmacSession, SERVER_BUDGET_REUSE_FLOOR_S, SERVER_BUDGET_REUSE_POLICY
from .reachable_endpoint_r3 import _native_geometry
from .reusable_tile_preparation_r3 import ReusableTileTopology

REVISION='r3-next7-sealed-pair-equivalent-corridor-capacity-guard'
PROTOCOL_ID='PLN-02-2A-V1-R3-REACHABLE-ENDPOINT-TILED-TOPOLOGY-EXACT-ACK-V1'


def _check_query_deadline(deadline,stage):
    if time.monotonic()>=deadline:raise TopologyDeadlineExceeded(stage)


class QueryTopologyPreparer:
    """Resolve query-dependent tiles and indexes inside the request deadline."""
    def __init__(self,tile,footprint,capacity=32):
        if capacity<1:raise ValueError('query topology cache capacity must be positive')
        self.tile=tile;self.footprint=footprint;self.capacity=capacity
        self.entries=OrderedDict();self.last_certificate=None;self.last_preparation=None

    def resolve(self,query,deadline,timing):
        begin=time.monotonic();before=dict(self.tile.stats)
        self.last_certificate=None;self.last_preparation=None
        key=digest({'tile_binding':self.tile.key,'start':query.start,'goal':query.goal,
                    'footprint':self.footprint,'connector':asdict(ConnectorConfig()),
                    'coarse_selection':getattr(self.tile,'selection_algorithm','frozen-nearest-seed-v1'),
                    'algorithm':'lazy-query-topology-shared-deadline-v1'})
        detail={'query_id':query.query_id,'start':list(query.start),'goal':list(query.goal),
                'tile_binding':self.tile.key,'key':key,'stage':'cache_lookup','complete':False}
        timing.update(query_topology_cache_hit=False,query_topology_coarse_search_ms=0.,
                      query_topology_artifact_wall_ms=0.,query_topology_selector_build_ms=0.)
        try:
            _check_query_deadline(deadline,'query_topology_cache_lookup')
            if key in self.entries:
                topology,selector=self.entries.pop(key);self.entries[key]=(topology,selector)
                timing['query_topology_cache_hit']=True
            else:
                detail['stage']='coarse_query';started=time.monotonic()
                try:selected=self.tile.candidate_tiles(query.start,query.goal,deadline=deadline)
                finally:timing['query_topology_coarse_search_ms']=(time.monotonic()-started)*1000
                _check_query_deadline(deadline,'coarse_query_complete')
                detail['selected_tiles']=[list(t) for t in selected]
                detail['stage']='artifact';started=time.monotonic()
                try:topology=self.tile.artifact(selected,deadline=deadline)
                finally:timing['query_topology_artifact_wall_ms']=(time.monotonic()-started)*1000
                topology.metadata['coarse_selection_certificate']=getattr(self.tile,'last_selection_certificate',None)
                _check_query_deadline(deadline,'query_selector_start')
                detail['stage']='selector_index';started=time.monotonic()
                try:selector=ReachableEndpointSelector(topology,self.footprint)
                finally:timing['query_topology_selector_build_ms']=(time.monotonic()-started)*1000
                _check_query_deadline(deadline,'query_selector_complete')
                self.entries[key]=(topology,selector)
                while len(self.entries)>self.capacity:self.entries.popitem(last=False)
            _check_query_deadline(deadline,'query_topology_complete')
            timing['query_topology_selected_tile_count']=len(topology.metadata.get('tiles',[]))
            detail.update(stage='complete',complete=True,topology_hash=selector.binding['topology'])
            detail['coarse_selection_certificate']=topology.metadata.get('coarse_selection_certificate')
            return topology,selector
        finally:
            timing['query_topology_prepare_wall_ms']=(time.monotonic()-begin)*1000
            timing['query_topology_cache_entries']=len(self.entries)
            timing['query_topology_deadline_overshoot_ms']=max(0.,time.monotonic()-deadline)*1000
            for name in ('tile_refine_wall_ms','topology_load_wall_ms','tile_cache_hit_count','tile_cache_miss_count'):
                timing[name]=self.tile.stats.get(name,0)-before.get(name,0)
            timing['query_topology_preparation_stage']=detail['stage']
            detail.update(timing=dict(timing),deadline_remaining_s=max(0.,deadline-time.monotonic()))
            self.last_preparation=detail


def queries_for_stage(queries,stage):
    # Select the cold query before any query-dependent graph or index work.
    return queries[:1] if stage=='cold' else queries


class CorridorCache:
    """Bounded packed masks; every hit returns an independent writable array."""
    def __init__(self,max_bytes=128*1024**2,straight_half_width_m=1.2):
        if not math.isfinite(straight_half_width_m) or not .6<=straight_half_width_m<=1.2:
            raise ValueError('CORRIDOR_STRAIGHT_WIDTH_OUT_OF_RANGE')
        self.straight_half_width_m=straight_half_width_m
        self.max_bytes=max_bytes;self.bytes=0;self.entries=OrderedDict()

    def get(self,ctx,route,expansion=False):
        m=ctx.hospital_map
        key=digest({'map':ctx.map_sha256,'shape':[m.height,m.width],'resolution':m.resolution,
                    'origin':m.origin,'route':route.polyline,'expansion':bool(expansion),
                    'footprint':legacy.FOOTPRINT,'safety':baseline.FOOTPRINT_SAFETY_MARGIN_M,
                    'bend':baseline.BEND_MARGIN_M,'algorithm':'adaptive-corridor-exact-capped-v3'})
        key=digest({'base':key,'straight_half_width_m':self.straight_half_width_m})
        if key in self.entries:
            packed,shape,info=self.entries.pop(key);self.entries[key]=(packed,shape,info)
            mask=np.unpackbits(packed,count=math.prod(shape)).reshape(shape).astype(bool)
            return mask,{**info,'corridor_cache_hit':True,'corridor_cache_bytes':self.bytes}
        if self.straight_half_width_m==1.2:
            mask,info=corridor(ctx,route,expansion=expansion)
        else:
            mask,info=corridor(ctx,route,expansion=expansion,
                               straight_half_width_m=self.straight_half_width_m)
        packed=np.packbits(mask)
        if packed.nbytes<=self.max_bytes:
            while self.entries and self.bytes+packed.nbytes>self.max_bytes:
                _,(old,_,_)=self.entries.popitem(last=False);self.bytes-=old.nbytes
            self.entries[key]=(packed,mask.shape,dict(info));self.bytes+=packed.nbytes
        return mask,{**info,'corridor_cache_hit':False,'corridor_cache_bytes':self.bytes}


def corridor(ctx,route,expansion=False,*,certified_chords_enabled=False,straight_half_width_m=1.2):
    if not math.isfinite(straight_half_width_m) or not .6<=straight_half_width_m<=1.2:
        raise ValueError('CORRIDOR_STRAIGHT_WIDTH_OUT_OF_RANGE')
    m=ctx.hospital_map;center=np.zeros((m.height,m.width),np.uint8)
    points=route.polyline;cells=[m.world_to_cell(*p[:2]) for p in points]
    valid=[(c[1],c[0]) for c in cells if c is not None]
    if valid:cv2.polylines(center,[np.asarray(valid,np.int32)],False,1,1)
    half_width=(2.+baseline.FOOTPRINT_SAFETY_MARGIN_M+baseline.BEND_MARGIN_M) if expansion else straight_half_width_m
    radius=math.ceil(half_width/m.resolution)
    kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*radius+1,2*radius+1))
    if _native_geometry is not None and hasattr(_native_geometry,'dilate_runs'):
        radii=((kernel.sum(axis=1)-1)//2).astype(int).tolist()
        mask=np.frombuffer(_native_geometry.dilate_runs(center,radii),np.uint8).reshape(center.shape).copy()
    else:mask=cv2.dilate(center,kernel)
    # Evaluate turns over .5m support, not noisy adjacent skeleton pixels.
    corner_count=0;corners=[]
    for i in range(1,len(points)-1):
        lo=i-1;hi=i+1
        while lo>0 and math.dist(points[lo],points[i])<.5:lo-=1
        while hi<len(points)-1 and math.dist(points[hi],points[i])<.5:hi+=1
        ax=points[i][0]-points[lo][0];ay=points[i][1]-points[lo][1]
        bx=points[hi][0]-points[i][0];by=points[hi][1]-points[i][1]
        angle=abs(math.atan2(ax*by-ay*bx,ax*bx+ay*by))
        if angle<math.radians(15):continue
        cell=cells[i]
        if cell is None:continue
        corners.append((i,cell))
    if hasattr(getattr(m,'distance_m',None),'capped_at'):
        clearances=m.distance_m.capped_at([cell for _,cell in corners],.5)
    else:clearances=[float(m.clearance(*points[i]) or 0.) for i,_ in corners]
    for (_,cell),clearance in zip(corners,clearances):
        widening=(4. if expansion else 2.) + max(0.,.5-float(clearance))
        cv2.circle(mask,(cell[1],cell[0]),math.ceil(widening/m.resolution),1,-1);corner_count+=1
    chords=[];chord_attempts=0
    if certified_chords_enabled:
        # Research-only opt-in. Production retains the frozen corridor geometry.
        from .corridor_shortcuts_r3 import certified_chords
        chords,chord_attempts=certified_chords(ctx,points,legacy.FOOTPRINT)
    for cert in chords:
        a,b=[m.world_to_cell(*pose[:2]) for pose in cert['poses']]
        if a is None or b is None:raise RuntimeError('certified chord outside map')
        cv2.line(mask,(a[1],a[0]),(b[1],b[0]),1,2*radius+1)
    return (mask.astype(bool)&(m.occupancy==0)),{'corner_count':corner_count,'corridor_expansion':expansion,
        'corridor_chord_attempts':chord_attempts,'corridor_chord_count':len(chords),'corridor_chord_certificates':chords,
        'corridor_straight_half_width_m':half_width}


def exact_ack_valid(diagnostics,request_id):
    return (diagnostics.get('costmap_update_acknowledged') is True
            and all(diagnostics.get(key)==0 for key in ('costmap_ack_mismatch_cells',
                    'hard_mismatch','soft_mismatch','stale_cells','hash_mismatch','sequence_mismatch'))
            and diagnostics.get('publication',{}).get('request_id')==request_id)


def pinned_smac_capacity(width,height,heading_bins=48):
    """Pinned AStar's unsigned 32-bit exclusive max_index must not wrap."""
    states=int(width)*int(height)*int(heading_bins)
    return {'width':int(width),'height':int(height),'heading_bins':int(heading_bins),
            'required_states':states,'exclusive_index_limit':2**32-1,
            'supported':min(width,height,heading_bins)>0 and states<=2**32-1,
            'reason':'pinned AStarAlgorithm::createPath unsigned int max_index'}


def run_query(ctx,query,topology,selector,session,spec,auditor,request_id,budget_s=7.,*,preparer=None):
    begin=time.monotonic();cpu=time.process_time();deadline=begin+budget_s
    capacity=pinned_smac_capacity(ctx.hospital_map.width,ctx.hospital_map.height)
    if not capacity['supported'] and not getattr(session,'roi_mode',False):
        return {'request_id':request_id,'query_id':query.query_id,'map_id':ctx.map_id,
                'architecture_id':'2A-V1','implementation_revision':REVISION,'protocol_id':PROTOCOL_ID,
                'final_valid_success':False,'action_success':False,'planner_search_started':False,
                'failure_code':'PINNED_SMAC_INDEX_CAPACITY','backend_capacity':capacity,
                'fallback_used':False,'l2_calls':0,'l3_calls':0,'attempts':[],
                'corridor_retry_used':False,'online_wall_ms':(time.monotonic()-begin)*1000,
                'process_cpu_ms':(time.process_time()-cpu)*1000,'remaining_budget_s':max(0.,deadline-time.monotonic()),
                'rss_mib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024}
    session.begin_request(request_id,deadline);diag={}
    try:
        if preparer is not None:
            topology,selector=preparer.resolve(query,deadline,diag)
        _check_query_deadline(deadline,'endpoint_selection_start')
        start,goal,route,reason=selector(topology,query,timing=diag,deadline=deadline)
        if preparer is not None:preparer.last_certificate=selector.last_certificate
    except TopologyDeadlineExceeded as exc:
        start=goal=route=None;reason='TOPOLOGY_REQUEST_DEADLINE';diag['topology_deadline_stage']=str(exc)
    except Exception as exc:
        start=goal=route=None
        reason=('TOPOLOGY_PREPARATION_ERROR' if preparer is not None and
                diag.get('query_topology_preparation_stage')!='complete' else 'ENDPOINT_CONNECTOR_EXCEPTION')
        diag['preparation_or_endpoint_exception']=str(exc)
    base={'request_id':request_id,'query_id':query.query_id,'map_id':ctx.map_id,
          'architecture_id':'2A-V1','implementation_revision':REVISION,'protocol_id':PROTOCOL_ID,
          'final_valid_success':False,'action_success':False,'planner_search_started':False,
          'failure_code':'','fallback_used':False,'l2_calls':0,'attempts':[],**diag}
    if route is None:
        base['failure_code']=reason;base['compatibility_failure_group']='L1_ENDPOINT_NOT_ATTACHABLE'
    else:
        for attempt in range(2):
            remaining=deadline-time.monotonic()
            if remaining<=0:
                base['failure_code']='REQUEST_DEADLINE';break
            if attempt and getattr(session,'_enforce_server_budget',False):
                from .request_budget_r3 import retry_admission
                previous=base['attempts'][-1]
                admission=retry_admission(remaining,
                    confirmed_search_s=getattr(session,'_confirmed_server_budget',None),
                    reconfigure_peak_s=getattr(session,'_budget_update_peak_s',0.),
                    corridor_ms=previous.get('corridor_build_ms',0.),
                    costmap_ms=previous.get('total_costmap_update_ms',0.))
                base['retry_admission']=admission
                if not admission['admitted']:
                    base['failure_code']='REQUEST_BUDGET_TOO_SMALL_FOR_RETRY'
                    base['retry_rejected_before_corridor']=True
                    break
            if not hasattr(session,'_corridor_cache'):session._corridor_cache=CorridorCache()
            mb=time.monotonic()
            try:mask,maskinfo=session._corridor_cache.get(ctx,route,expansion=bool(attempt))
            except RuntimeError as exc:
                base['failure_code']=str(exc);base['planner_search_started']=False
                base['backend_context_failure_ms']=(time.monotonic()-mb)*1000
                base['backend_capacity']=getattr(session,'last_roi_capacity',{})
                break
            mask_ms=(time.monotonic()-mb)*1000
            cursor=baseline._session_log_cursor(session)
            try:
                result=session.plan(query,spec,source='kinematic',allowed_mask=mask,skip_path_mask_validation=True)
            except Exception as exc:
                result=legacy.PlanResult(planner_backend=spec.backend,backend_version=spec.version,
                                         failure_code='PIPELINE_EXCEPTION',failure_detail=str(exc),
                                         diagnostics={'planner_search_started':False,'failure_detail':str(exc)})
            logs=baseline._session_log_delta(session,cursor)
            rd=dict(result.diagnostics or {});rd.update(baseline._parse_smac_benchmark_metrics(logs))
            base['action_success']=result.planner_success;base['planner_search_started']=rd.get('planner_search_started',False)
            audit=None
            if result.planner_success:
                base['points']=result.points
                try:
                    audit=auditor.audit(query,result.points,mask)
                    base.update({**audit.metrics,**audit.diagnostics()})
                    exact=exact_ack_valid(rd,request_id)
                    base['final_valid_success']=audit.final_valid_success and exact
                    failure='' if base['final_valid_success'] else (audit.metrics.get('failure_code') or ('EXACT_ACK_REQUIRED' if not exact else 'PATH_OUTSIDE_CORRIDOR'))
                except Exception as exc:
                    base['final_valid_success']=False;failure='PATH_AUDIT_EXCEPTION'
                    base['path_audit_exception']=repr(exc)
            elif rd.get('planner_search_started'):
                failure,_,detail=baseline._classify_smac_failure(result.failure_code,rd,logs);rd['failure_detail']=detail
            else:failure=result.failure_code
            if time.monotonic()>deadline:
                base['final_valid_success']=False;failure='REQUEST_DEADLINE'
            base['failure_code']=failure
            attempt_record={**rd,'attempt':attempt,'remaining_before_s':remaining,'remaining_after_s':max(0.,deadline-time.monotonic()),
                            'retry_reason':base.get('primary_failure','') if attempt else '',
                            'failure_code':failure,'corridor_build_ms':mask_ms,**maskinfo}
            base['attempts'].append(attempt_record)
            protected={'failure_code','final_valid_success','static_footprint_valid','kinematic_valid','action_success','points'}
            if audit is not None:protected.update(audit.metrics);protected.update(audit.diagnostics())
            base.update({k:v for k,v in rd.items() if k not in protected})
            if not attempt:base['primary_failure']=failure
            if base['final_valid_success'] or not rd.get('planner_search_started') or failure not in {'SMAC_MAX_ITERATIONS','NO_PATH_IN_CORRIDOR','NO_PATH','L3_PRIME_FAILED'}:break
    if preparer is not None and hasattr(preparer,'reconcile'):
        accounting_begin=time.monotonic();base.update(preparer.reconcile())
        base['query_cache_finalize_ms']=(time.monotonic()-accounting_begin)*1000
        if base['final_valid_success'] and time.monotonic()>deadline:
            base.update(final_valid_success=False,failure_code='REQUEST_DEADLINE')
    base['online_wall_ms']=(time.monotonic()-begin)*1000;base['process_cpu_ms']=(time.process_time()-cpu)*1000
    base['remaining_budget_s']=max(0.,deadline-time.monotonic());base['rss_mib']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
    base['corridor_retry_used']=len(base['attempts'])>1
    base['l3_calls']=sum(bool(v.get('planner_search_started')) for v in base['attempts'])
    return base


def write_csv(path,rows):
    fields=sorted(set().union(*(r.keys() for r in rows))) if rows else []
    with Path(path).open('x',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for row in rows:writer.writerow({k:json.dumps(v) if isinstance(v,(dict,list,tuple)) else v for k,v in row.items()})


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--campaign',type=Path,required=True);p.add_argument('--map-id',required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--cache',type=Path,required=True)
    p.add_argument('--stage',choices=['topology','offline','smoke','formal','cold'],required=True)
    p.add_argument('--ros-domain-id',type=int,default=173);p.add_argument('--tile-cells',type=int,default=512)
    p.add_argument('--tile-halo-cells',type=int,default=32);p.add_argument('--tile-memory-capacity',type=int,default=4)
    p.add_argument('--map-preparation',choices=['tiles','lazy'],default='tiles')
    p.add_argument('--backend-context',choices=['full','roi'],default='full')
    p.add_argument('--roi-max-cells',type=int,default=80*1024*1024)
    p.add_argument('--ready-context-max-cells',type=int,default=32*1024*1024)
    p.add_argument('--query-cache-max-bytes',type=int,default=3*1024**3)
    p.add_argument('--corridor-straight-half-width-m',type=float,default=1.2,
                   help='research profile: straight tube half width; bends and one expansion retain original margins')
    p.add_argument('--online-context',choices=['bounded','dense'],default='bounded')
    p.add_argument('--ready-baseline-timeout-s',type=float,default=90.)
    p.add_argument('--query-id',action='append');p.add_argument('--request-budget-s',type=float,default=7.)
    args=p.parse_args(argv)
    if not 1<=args.query_cache_max_bytes<=3*1024**3:
        p.error('query cache capacity must be within [1, 3221225472] bytes')
    if not math.isfinite(args.corridor_straight_half_width_m) or not .6<=args.corridor_straight_half_width_m<=1.2:
        p.error('corridor straight half width must be finite and within [0.6, 1.2] m')
    if args.backend_context!='roi' and args.corridor_straight_half_width_m!=1.2:
        p.error('research corridor width requires the ROI backend')
    initialization_begin=time.monotonic();initialization={}
    if args.backend_context=='roi':
        global REVISION
        from .roi_backend_r3 import VERSION
        REVISION=VERSION
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_DOMAIN_ID']=str(args.ros_domain_id)
    sys.path.insert(0,str(args.campaign/'tools'));import r2_adapter
    _,parent,_,_=r2_adapter.configure(args.map_id)
    load_begin=time.monotonic();queries,metadata=parent._load_tasks()
    original_query_count=len(queries)
    if args.query_id:queries=[q for q in queries if q.query_id in args.query_id]
    if args.stage=='formal' and (len(queries)!=20 or args.query_id):raise ValueError('formal requires fixed complete 20 queries')
    queries=queries_for_stage(queries,args.stage)
    initialization['input_load_wall_ms']=(time.monotonic()-load_begin)*1000
    load_begin=time.monotonic()
    if args.stage=='topology':
        hospital=load_map_bounded(parent.validity.MAP_YAML,args.cache)
        ctx=SimpleNamespace(hospital_map=hospital)
    elif args.online_context=='bounded':
        from .bounded_context_r3 import bounded_context
        ctx=bounded_context(parent.validity.MAP_YAML,args.map_id,args.cache,legacy.FOOTPRINT)
    else:ctx=parent._context()
    initialization['map_load_wall_ms']=(time.monotonic()-load_begin)*1000
    load_begin=time.monotonic()
    tile_class=ReusableTileTopology if args.map_preparation=='tiles' else TiledTopology
    tile=tile_class(ctx.hospital_map,legacy.FOOTPRINT,args.cache,TileConfig(tile_cells=args.tile_cells,halo_cells=args.tile_halo_cells,memory_tiles=args.tile_memory_capacity))
    initialization['topology_object_init_wall_ms']=(time.monotonic()-load_begin)*1000
    protocol={'architecture_id':'2A-V1','implementation_revision':REVISION,'protocol_id':PROTOCOL_ID,
              'map_preparation':args.map_preparation,'online_context':args.online_context,
              'backend_context':args.backend_context,'roi_max_cells':args.roi_max_cells,
              'ready_context_max_cells':args.ready_context_max_cells,
              'query_cache_max_bytes':args.query_cache_max_bytes,
              'corridor_straight_half_width_m':args.corridor_straight_half_width_m,
              'ready_baseline':('bounded_persistent_global_else_16x16_bootstrap_exact' if args.backend_context=='roi' else 'query_independent_all_blocked_exact'),
              'roi_context_timing_scope':'geometry staging, atomic geometry+source commit, inflation and full ROI ACK are inside request deadline',
              'ready_baseline_timeout_s':args.ready_baseline_timeout_s,
              'stage':args.stage,'queries':[asdict(q) for q in queries],'metadata':metadata,
              'original_fixed_query_count':original_query_count,'active_query_count':len(queries),
              'online_timing_scope':'lazy query topology preparation through final canonical audit',
              'deadline_semantics':'success cutoff with checks between bounded synchronous work units; not hard realtime return',
              'cold_preparation_scope':'selected cold query only',
              'safety':{'footprint':legacy.FOOTPRINT,'resolution':.05,'DUBIN':True,'heading_bins':48,
                        'Rmin':.40,'unknown_is_collision':True,'reverse':False,'in_place_rotation':False,
                        'canonical_PathAudit':'frozen_r2'},'request_budget_s':args.request_budget_s,
              'tile_config':asdict(tile.config),'connector_config':asdict(ConnectorConfig())}
    protocol['ack_readback']={'transport':'sealed_memfd_pair_v2','full_copies':2,'separate_objects':True,'capture_scope':'two full copies under one complete-master lock','ready_completed_source_update_cycles':2,'request_ack_timeout_s':3.,'per_object_max_bytes':128*1024**2}
    protocol['runtime_caches']={'corridor_packed_max_bytes':128*1024**2,'expected_effective_max_bytes':512*1024**2,'query_topology_capacity':32,'query_topology_max_bytes':512*1024**2}
    protocol['server_budget_policy']={'maximum_s':5.,'reconfiguration_reserve_s':1.25,'quantum_s':.5,
                                      'upgrade_policy':SERVER_BUDGET_REUSE_POLICY,
                                      'opportunistic_reuse_floor_s':SERVER_BUDGET_REUSE_FLOOR_S,
                                      'post_reconfiguration_remaining_recheck':True}
    protocol['cyclic_gc_policy']={'defer_during':['server_budget_rpc','complete_smac_action_exchange'],
                                  'restore_before_local_validation':True,
                                  'collection_in_absolute_request_budget':True}
    atomic_json(output/'protocol.json',protocol)
    if args.stage=='topology':
        with (output/'tile_progress.jsonl').open('x') as f:
            def progress(v):f.write(json.dumps(v)+'\n');f.flush()
            stats=tile.build_all(progress)
        atomic_json(output/'topology_result.json',stats);print(json.dumps(stats),flush=True);return 0
    load_begin=time.monotonic();tile.build_coarse()
    initialization['coarse_graph_build_wall_ms']=(time.monotonic()-load_begin)*1000
    if args.map_preparation=='tiles':initialization.update(tile.prepare_map())
    from .bounded_query_preparation_r3 import BoundedQueryTopologyPreparer
    preparer=BoundedQueryTopologyPreparer(tile,legacy.FOOTPRINT,max_bytes=args.query_cache_max_bytes);prep=[]
    if args.stage=='offline':
        for query in queries:
            selected=tile.candidate_tiles(query.start,query.goal)
            artifact=tile.artifact(selected);selector=ReachableEndpointSelector(artifact,legacy.FOOTPRINT)
            timing={};a,b,route,reason=selector(artifact,query,timing=timing)
            atomic_json(output/f'{query.query_id}_certificate.json',selector.last_certificate)
            prep.append({'query_id':query.query_id,'failure_code':reason if route is None else '',**timing})
            print(query.query_id,reason,flush=True)
    if args.stage=='offline':
        write_csv(output/'attachment_diagnostics.csv',prep)
        atomic_json(output/'topology_preflight_stats.json',{'measurement_identity':'nonformal_functional_preflight',**tile.stats})
        return 0
    audit_begin=time.monotonic()
    auditor=PathAuditor(ctx,source_commit=REVISION)
    initialization['audit_prepare_wall_ms']=(time.monotonic()-audit_begin)*1000
    spec=legacy.backend_availability()['hybrid_astar']
    if not spec.available:raise RuntimeError(spec.reason)
    session_class=ExactAckSmacSession;session_kwargs={}
    if args.backend_context=='roi':
        from .roi_backend_r3 import RoiExactAckSmacSession,GlobalRoiAuditor
        session_class=RoiExactAckSmacSession;session_kwargs={'max_cells':args.roi_max_cells,
            'ready_context_max_cells':args.ready_context_max_cells,
            'corridor_straight_half_width_m':args.corridor_straight_half_width_m}
    session=session_class(ctx,output,**session_kwargs,map_yaml=ctx.map_yaml,log_tag='r3_'+args.map_id,
                               local_mask_updates=True,optimization_profile='v7_candidate',
                               smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',
                               planner_parameter_overrides={'benchmark_instrumentation':True,
                                                            'smoother':{'w_data':.25,'w_smooth':.25}})
    # Keep pinned planner parameters unchanged. Instrumentation is the existing
    # compile-time extension; report unavailable if it emits no measurements.
    if args.backend_context=='roi':auditor=GlobalRoiAuditor(auditor,session)
    rows=[]
    try:
        start_begin=time.monotonic();session.start()
        initialization['session_start_wall_ms']=(time.monotonic()-start_begin)*1000
        start_begin=time.monotonic()
        if not session._get_costmap_client.wait_for_service(timeout_sec=30.):
            raise RuntimeError('COSTMAP_STARTUP_SERVICE_UNAVAILABLE')
        initialization['costmap_service_ready_wait_ms']=(time.monotonic()-start_begin)*1000
        ready_receipt=session.prepare_ready_baseline(args.ready_baseline_timeout_s)
        atomic_json(output/'ready_baseline_receipt.json',ready_receipt)
        initialization['ready_baseline_wall_ms']=ready_receipt['ready_baseline_wall_ms']
        initialization.update(total_initialization_wall_ms=(time.monotonic()-initialization_begin)*1000,
                              query_topology_preparations_before_first_request=0,
                              active_query_count=len(queries),original_fixed_query_count=original_query_count)
        atomic_json(output/'initialization_timing.json',initialization)
        maps_text=Path(f'/proc/{session.planner_pid}/maps').read_text()
        loaded=sorted({line.split()[-1] for line in maps_text.splitlines() if 'libnav2_smac_planner.so' in line})
        from .tiled_topology_r3 import file_hash
        binary_manifest={path:file_hash(path) for path in loaded}
        atomic_json(output/'loaded_smac_binary.json',binary_manifest)
        if not loaded or set(binary_manifest.values())!={'2c62b5586c7cbac1665c7e7660a833dca3124c965185de9cad996d00927b7739'}:
            raise RuntimeError('PINNED_SMAC_BINARY_MISMATCH')
        loops=[('warmup',3),('measured',5)] if args.stage=='formal' else [(args.stage,1)]
        with (output/'runs.jsonl').open('x') as stream:
            for mode,count in loops:
                for repetition in range(1,count+1):
                    for query in queries:
                        rid=f'{mode}:{repetition}:{query.query_id}'
                        row=run_query(ctx,query,None,None,session,spec,auditor,rid,args.request_budget_s,preparer=preparer)
                        row.update(run_mode=mode,repetition=repetition)
                        points=row.pop('points',None)
                        if points:atomic_json(output/f'{mode}_{repetition}_{query.query_id}_path.json',points)
                        if preparer.last_certificate:atomic_json(output/f'{mode}_{repetition}_{query.query_id}_connector.json',preparer.last_certificate)
                        if preparer.last_preparation:
                            atomic_json(output/f'{mode}_{repetition}_{query.query_id}_topology_preparation.json',
                                        {'request_id':rid,**preparer.last_preparation})
                        rows.append(row);stream.write(json.dumps(row)+'\n');stream.flush()
                        print(mode,repetition,query.query_id,row['final_valid_success'],row['failure_code'],round(row['online_wall_ms'],2),flush=True)
                        if session._unresolved_timeout or session._budget_state_uncertain or getattr(session,'context_uncertain',False):
                            reason='unconfirmed server budget update' if session._budget_state_uncertain else 'unresolved action timeout'
                            atomic_json(output/'exclusion.json',{'status':'EXCLUDED','reason':reason+'; entire batch requires restart'})
                            raise RuntimeError('UNRESOLVED_SESSION_STATE_BATCH_EXCLUDED')
    finally:
        session.close()
        if getattr(ctx,'bounded_memory',False):
            fields={'hospital_distance':ctx.hospital_map.distance_m,'free_mask':ctx.free_mask,
                    'context_distance':ctx.distance_m,'audit_distance':auditor._distance_to_unsafe_m}
            atomic_json(output/'bounded_memory_summary.json',{name:{'stats':value.stats,
                'cache_bytes':value.bytes,'max_cache_bytes':value.max_bytes,
                'tile_cells':value.tile_cells,'max_halo_cells':value.max_halo_cells}
                for name,value in fields.items()})
    write_csv(output/'runs.csv',rows)
    atomic_json(output/'completion.json',{'rows':len(rows),'measured':sum(r['run_mode']=='measured' for r in rows),
                                         'valid_all':sum(r['final_valid_success'] for r in rows),
                                         'measured_valid':sum(r['final_valid_success'] for r in rows if r['run_mode']=='measured'),
                                         'warmup_valid':sum(r['final_valid_success'] for r in rows if r['run_mode']=='warmup'),
                                         'complete':True})
    return 0


if __name__=='__main__':raise SystemExit(main())
