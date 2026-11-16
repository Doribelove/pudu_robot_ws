"""Runtime search-domain recovery; frozen L1/L3 implementations are reused.

A failed local XY domain is not a global no-path proof. Fresh topology data
is request-owned and built from the current obstacle overlay. No base graph,
physical constraint, original map or previously published route is edited.
"""
from dataclasses import replace
import os
import time
from pathlib import Path
import numpy as np
from three_yd_v5.recovery import V5Runner
from three_yd_v5.v12_degradation import result_stage, failure_stage


class CorridorRunner(V5Runner):
    """Limit a new candidate's solver work using the existing phase cap."""
    def _plan_original(self,query,session,auditor,spec):
        state=session._curve_request
        old=getattr(session,'_v12_phase_L3_cap',None)
        cap=state['new_used_s']+2.
        session._v12_phase_L3_cap=min(cap,old) if old is not None else cap
        try:
            result=super()._plan_original(query,session,auditor,spec)
            out=result[2]
            if (not out.get('success') and out.get('failure_code')=='CURVE_BUDGET'
                    and state['new_used_s']<state['config']['new_l3_budget_s']-.02):
                out['failure_code']='CURVE_V13_CORRIDOR_SLICE_EXHAUSTED'
                out['failure_stage']='L3'
            return result
        finally:
            if old is None:del session._v12_phase_L3_cap
            else:session._v12_phase_L3_cap=old


def bind_current_pose(service,pose):
    """Actual pose changes never turn a resolved goal into the requested goal."""
    pose=np.asarray(pose,float)
    if pose.shape!=(3,) or not np.isfinite(pose).all():raise ValueError('V13_INVALID_CURRENT_POSE')
    if service.query is None:raise ValueError('V13_CURRENT_POSE_WITHOUT_TASK')
    incoming=replace(getattr(service,'requested_query',service.query),start=tuple(map(float,pose)))
    moved=tuple(incoming.start)!=tuple(service.query.start)
    changed=moved or tuple(incoming.goal)!=tuple(service.query.goal)
    return incoming,moved,changed


def eligible(service, out):
    return (os.environ.get('V13_CORRIDOR_RECOVERY','1')=='1'
            and not out.get('success') and result_stage(out) in ('L1','L2','L3')
            and failure_stage(out.get('failure_code',''))!='infrastructure'
            and service.dynamic is not None and bool(np.any(service.dynamic.mask)))


def recover(service,query,previous,controller_class):
    state=service.session._curve_request
    started=time.monotonic();attempts=[]
    meta={'trigger':previous.get('failure_code'),'current_start':list(query.start),
          'fixed_goal':list(query.goal),'dynamic_version':service.dynamic.version,
          'original_local_recovery':previous.get('candidate_recovery'),
          'original_degradation':previous.get('v12_degradation'),
          'attempts':attempts,'L1_L3_algorithms_unchanged':True,
          'physical_infeasibility_proven':False,'full_map_Smac':False}
    if hasattr(service,'v13_domain_recoveries'):service.v13_domain_recoveries.append(meta)
    base=service.topology
    # Parent recovery already tried local scope enlargement. Requery the
    # unchanged neutral L1 on CURRENT hard cells, then a 2D global L2 guide.
    modes=(['frozen_L1_current_obstacles'] if getattr(base,'mode',None)=='free_space' else [])+['full_map_L2_new_corridor']
    chosen=(None,None,previous)
    for mode in modes:
        remaining=state['deadline']-time.monotonic()
        l3_remaining=state['config']['new_l3_budget_s']-state['new_used_s']
        if remaining<=.05 or l3_remaining<=.05:
            code='V13_REQUEST_RECOVERY_BUDGET_EXHAUSTED' if remaining<=.05 else 'V13_L3_RECOVERY_BUDGET_EXHAUSTED'
            attempts.append({'kind':mode,'executed':False,'reason':'SHARED_REQUEST_OR_L3_BUDGET',
                'remaining_request_s':max(0.,remaining),'remaining_L3_s':max(0.,l3_remaining)})
            # Preserve the causal budget failure; the old local L2 no-path
            # must not overwrite a later exhausted global/kinematic recovery.
            chosen=(None,None,{'success':False,'failure_code':code,'failure_stage':'L3' if l3_remaining<=.05 else 'budget',
                'physical_infeasibility_proven':False,'prior_failure_code':chosen[2].get('failure_code')})
            break
        tick=time.monotonic();detail={'kind':mode,'executed':True};attempts.append(detail)
        active=getattr(service.session,'active_controller',None)
        if active is not None:
            active.trajectory=None;active.trajectory_version=None;active.lifecycle.clear()
        service.session.active_controller=None;service.session.active_route=None
        c=None;r=None
        try:
            if mode=='frozen_L1_current_obstacles':
                from three_yd_v5.free_space_topology import FreeSpaceTopology
                top=FreeSpaceTopology(service.world,Path(service.root)/f'v13_event_{service.dynamic.version}_topology')
                detail['topology_preparation_ms']=top.preparation_ms
                detail['base_topology_mutated']=False
            else:
                from three_yd_v5.v12_global_l2 import topology_from_grid
                top,grid=topology_from_grid(base,query,service.session,dynamic=service.dynamic,
                    local_budget_s=min(3.,max(.01,remaining)))
                detail['grid_search']=grid
            # A separate view/cache namespace prevents old failed-domain
            # entries or state IDs from being rebound to a new corridor.
            views={k:v for k,v in service.views.items() if isinstance(k,int)}
            runner=CorridorRunner(service.world,top,Path(service.root)/f'v13_event_{service.dynamic.version}_{mode}',
                views,service.dynamic,controller_class=controller_class,max_region_routes=service.max_region_routes)
            detail['per_candidate_L3_cap_s']=2.
            c,r,out=runner.plan(query,service.session,service.auditor,service.spec)
            detail.update(success=bool(out.get('success')),failure_code=out.get('failure_code'),
                candidate_recovery=out.get('candidate_recovery'),degradation=out.get('v12_degradation'))
            chosen=(c,r,out)
            if out.get('success'):
                c.v13_recovery_domain={'kind':mode,'map_version':service.dynamic.version}
                service.views=views
                meta['selected']=mode
                break
        except (RuntimeError,ValueError,TimeoutError) as error:
            if failure_stage(error)=='infrastructure':raise
            detail.update(success=False,failure_code=str(error),grid_trials=getattr(error,'trials',None))
            chosen=(None,None,{'success':False,'failure_code':str(error),'failure_stage':failure_stage(error)})
        finally:detail['wall_ms']=(time.monotonic()-tick)*1000
    meta['wall_ms']=(time.monotonic()-started)*1000
    chosen[2]['v13_corridor_recovery']=meta
    return chosen
