"""Separate experimental route-structure policy. Never claims unreachability.

No query/map ID rule. Early sections are a scheduling choice, not a new mask,
vehicle model, acceptance rule or final-path cache. If rejected, one original
whole-route attempt remains; already-rejected sections are not repeated.
"""
import math,time
import numpy as np
from .r3_options import enabled
from .r6_cache import get_cache
from .bounded_segments import strategy_cache,recover
from three_yd_v4.r5_controller import SoftEdgeController


def evidence(controller):
    xy=np.asarray(controller.reference_plan.world_xy,float)
    ds=np.linalg.norm(np.diff(xy,axis=0),axis=1)
    delta=np.diff(xy,axis=0)
    yaws=np.arctan2(delta[:,1],delta[:,0])
    turns=np.abs(np.arctan2(np.sin(np.diff(yaws)),np.cos(np.diff(yaws))))
    count=len(getattr(controller,'rectangle_bridge_cells',[]))
    data={'reference_length_m':float(ds.sum()),'rectangle_bridge_cells':count,
          'sharp_turns':int(np.count_nonzero(turns>=math.pi/6)),
          'policy':'free_space + length>=250m + rectangle>=64 cells + >=4 turns of 30deg',
          'heuristic_not_proof_of_whole_failure':True}
    data['selected']=bool(controller.generic_mode and ds.sum()>=250. and count>=64 and data['sharp_turns']>=4)
    return data


def merge_diagnostics(items):
    ds=[x for x in items if x]
    if not ds:return {}
    d=dict(ds[-1])
    for key in ('local_map_update_ms','l3_action_wall_ms','l3_planning_time_ms','backend_call_count',
                'wall_time_ms','planning_time_ms','planner_cpu_total_ms'):
        d[key]=sum(float(x.get(key) or 0.) for x in ds)
    for key in ('planner_rss_peak_bytes','planner_pss_peak_bytes','stack_rss_peak_bytes','stack_pss_peak_bytes'):
        values=[x[key] for x in ds if x.get(key) is not None]
        if values:d[key]=max(values)
    d['costmap_update_acknowledged']=all(x.get('costmap_update_acknowledged') is True for x in ds)
    d['costmap_ack_mismatch_cells']=max(x.get('costmap_ack_mismatch_cells',0) for x in ds)
    return d


def plan(controller,query,session,auditor,spec,*,enabled_cost=True,cap=140):
    if not enabled('B'):return None
    data=evidence(controller)
    if not data['selected']:return None
    _,key=strategy_cache(controller,query)
    rejected=get_cache(controller.views,'r3-early-section-rejections',max_entries=16)
    if rejected.get(key):return None
    started=time.monotonic()
    # First send/ACK the same bounded costmap. The unchanged eight-second
    # vehicle recovery window begins after preparation, as it did after the
    # original whole action. All preparation remains in end-to-end timing.
    ack=controller.sync_only(session)
    prime_ms=(time.monotonic()-started)*1000
    log={**data,'map_prime_ms':prime_ms,'prime_ack':ack,'max_vehicle_calls':5,
         'section_budget_s':8.,'whole_fallback_action_timeout_s':session.client.timeout,
         'same_scope':True,'same_soft_cost':True,'global_fallback':False}
    if ack.get('costmap_update_acknowledged') is not True:
        return {'called':False,'success':False,'failure_code':'EARLY_SECTIONS_PRIME_ACK_FAILED',
                'diagnostics':ack,'r3_early_sections':log}
    initial={'called':False,'success':False,'failure_code':'EARLY_SECTION_POLICY',
             'diagnostics':{**ack,'backend_call_count':0,'local_map_update_ms':ack.get('local_map_update_ms',0.)}}
    result=recover(controller,query,session,auditor,spec,initial,cap=cap)
    log['section_success']=bool(result.get('success'))
    if result.get('success'):
        log['whole_fallback_called']=False
        result['r3_early_sections']=log
        return result
    # No partial trajectory is a valid query output. One bounded original
    # whole attempt is retained; failed sections will not be retried.
    controller.trajectory=None;controller.trajectory_version=None
    rejected[key]=True
    fallback=SoftEdgeController.plan_l3(controller,query,session,auditor,spec,enabled=enabled_cost,cap=cap)
    log.update(whole_fallback_called=True,section_failure=result.get('failure_code'),
               section_log=result.get('r1_segmented_recovery'),whole_fallback_success=bool(fallback.get('success')))
    fallback['diagnostics']=merge_diagnostics([result.get('diagnostics',{}),fallback.get('diagnostics',{})])
    repair=result.get('curvature_repair',{});other=fallback.get('curvature_repair',{})
    fallback['curvature_repair']={**other,'wall_ms':repair.get('wall_ms',0.)+other.get('wall_ms',0.)}
    fallback['r3_early_sections']=log
    return fallback
