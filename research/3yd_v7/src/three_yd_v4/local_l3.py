"""Whole-path validation, bounded local vehicle repair, full-Smac fallback."""
from dataclasses import replace
from types import SimpleNamespace
import math
import time
import cv2
import numpy as np
from arena_3d_v1.semantic_world import SweptChecker, densify, wrap, digest
from arena_3d_v1.curvature_repair import violating_segments, truthful_points, fit_connector
from arena_evaluation.semantic_constraint_core import dubins_choices
from arena_evaluation.unified_four_backends_smoke import PlanResult
from .controller import SoftEdgeController


def reference_poses(xy,start,goal):
    """Simplification proposes geometry only; subsequent full sweep validates it."""
    points=np.vstack((np.asarray(start)[:2],xy,np.asarray(goal)[:2]))
    points=points[np.r_[True,np.linalg.norm(np.diff(points,axis=0),axis=1)>1e-7]]
    simple=cv2.approxPolyDP(points.astype(np.float32),.08,False).reshape(-1,2).astype(float)
    simple[0]=start[:2];simple[-1]=goal[:2]
    out=[]
    for a,b in zip(simple,simple[1:]):
        length=np.linalg.norm(b-a)
        if length<1e-8:continue
        n=max(1,math.ceil(length/.05));yaw=math.atan2(b[1]-a[1],b[0]-a[0])
        out.extend([*(a+(b-a)*i/n),yaw] for i in range(n))
    if not out:raise ValueError('EMPTY_REFERENCE')
    out.append([*goal[:2],goal[2]]);out[0]=list(start)
    return np.asarray(out,float)


def risk_segments(world,p,mask):
    bad,lengths=violating_segments(p,1/world.vehicle[2]);bad=set(map(int,bad))
    checker=SweptChecker(world.hard|~mask,world.map.resolution,world.map.origin,*world.vehicle[:2])
    def locate(a,b):
        if checker.check(densify(p[a:b+1])):return
        if b-a<=1:bad.add(a);return
        middle=(a+b)//2;locate(a,middle);locate(middle,b)
    locate(0,len(p)-1)
    delta=np.diff(p,axis=0);heading=p[:-1,2]+wrap(delta[:,2])/2
    motion=np.arctan2(delta[:,1],delta[:,0])
    bad.update(map(int,np.flatnonzero(abs(wrap(motion-heading))>math.radians(5))))
    return np.asarray(sorted(bad),int),lengths


def repair_windows(p,bad,margin=1.):
    length=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1);arc=np.r_[0,np.cumsum(length)]
    windows=[]
    for i in bad:
        a=max(0,int(np.searchsorted(arc,arc[i]-margin))-1)
        b=min(len(p)-1,int(np.searchsorted(arc,arc[min(i+1,len(p)-1)]+margin)))
        if windows and a<=windows[-1][1]:windows[-1][1]=max(windows[-1][1],b)
        else:windows.append([a,b])
    coverage=sum(float(arc[b]-arc[a]) for a,b in windows)
    return windows,coverage,float(arc[-1])


def anchored_connector(world,p,a,b,mask,deadline):
    """Fit actual geometry to the ORIGINAL anchor poses and adjacent curvature.

    Frozen Smac may return endpoint poses within its allowed tolerance, which
    is insufficient for an exact splice. Never hide that offset by changing yaw
    metadata. Short collision-free connectors can repair the real geometry.
    """
    length=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
    left=float(wrap(p[a,2]-p[a-1,2])/length[a-1]) if a else 0.
    right=float(wrap(p[b+1,2]-p[b,2])/length[b]) if b<len(p)-1 else 0.
    nominal=float(length[a:b].sum());candidates=[]
    for radius in (.45,.6,.8):
        for word,par in dubins_choices(p[a],p[b],radius):
            distance=sum(par)*radius
            if distance>min(6.,nominal*1.25+.2):continue
            candidates.append((distance,radius,word,par))
    checker=SweptChecker(world.hard|~mask,world.map.resolution,world.map.origin,*world.vehicle[:2])
    trials=0
    for _,radius,word,par in sorted(candidates,key=lambda item:item[0])[:4]:
        for slack in (.005,.02,.05):
            if time.monotonic()>=deadline:return None,{'success':False,'failure':'ANCHOR_FIT_BUDGET','trials':trials}
            if 'S' not in word:continue
            adjusted=list(par);adjusted[word.index('S')]+=slack/radius;trials+=1
            try:patch,fit=fit_connector(p[a],p[b],word,adjusted,radius,left,right,deadline,1/world.vehicle[2])
            except ValueError:continue
            except TimeoutError:return None,{'success':False,'failure':'ANCHOR_FIT_BUDGET','trials':trials}
            splice=np.vstack((p[max(0,a-1):a],patch,p[b+1:min(len(p),b+2)]))
            if len(violating_segments(splice,1/world.vehicle[2])[0]):continue
            if not checker.check(densify(splice)):continue
            return patch,{'success':True,'trials':trials,'fit':fit,'exact_original_anchor_poses':True}
    return None,{'success':False,'failure':'NO_VALID_ANCHORED_CONNECTOR','trials':trials}


class LocalRepairController(SoftEdgeController):
    l3_mode='validate_all_repair_local'

    def plan_l3(self,query,session,auditor,spec,**kwargs):
        started=time.monotonic();log={'mode':self.l3_mode,'full_path_checked':True,'local_calls':[],
            'fallback_full':False,'local_search_budget_s':8.,'repair_windows_limit':6}
        def fallback(reason):
            log.update(fallback_full=True,reason=reason,local_stage_ms=(time.monotonic()-started)*1000)
            out=super(LocalRepairController,self).plan_l3(query,session,auditor,spec,**kwargs)
            out['local_l3']=log
            return out
        if not self.initial_l2_result.success:return fallback('DYNAMIC_REFERENCE_INVALID')
        if tuple(self.world.vehicle)!=tuple(session.bound_vehicle):raise ValueError('VEHICLE_VERSION_MISMATCH')
        if (self.reference_plan.world_key!=self.world.key or
            self.reference_plan.fine_corridor_hash!=digest(self.plan.corridor_mask) or
            self.reference_plan.topology_signature!=self.task['signature']):
            raise ValueError('REFERENCE_BINDING_MISMATCH')
        version=0 if self.dynamic is None else self.dynamic.version
        mask=self._target_mask().copy()
        p=reference_poses(self.reference_plan.world_xy,query.start,query.goal)
        bad,_=risk_segments(self.world,p,mask)
        windows,coverage,length=repair_windows(p,bad)
        log.update(risk_segments=len(bad),initial_windows=windows,repair_length_m=coverage,path_length_m=length,
                   repair_fraction=coverage/length if length else 1.)
        # Point-only references have no vehicle certificate and should not
        # spend a series of local attempts before the already-required fallback.
        if 'point_guide_unverified' in self.reference_mode:return fallback('POINT_REFERENCE_REQUIRES_FULL_VEHICLE_SEARCH')
        if len(windows)>6 or coverage>max(8.,.5*length):return fallback('REPAIR_COVERAGE_LIMIT')
        ack=self.sync_only(session)
        if ack.get('costmap_update_acknowledged') is not True:
            return {'success':False,'failure_code':'COSTMAP_CONTENT_ACK_FAILED','diagnostics':ack,'local_l3':log,'map_version':version}
        template=SimpleNamespace(source='v4_validated_l2_reference',planner_backend='v4_local_smac',backend_version='frozen_smac_48bins')
        deadline=time.monotonic()+8.
        # Reverse order keeps the untouched prefix's original indices valid.
        for a,b in reversed(windows):
            if time.monotonic()>=deadline:return fallback('LOCAL_BUDGET_EXHAUSTED')
            local=replace(query,query_id=f'{query.query_id}_local_{a}_{b}',start=tuple(p[a]),goal=tuple(p[b]))
            result=session.plan(local,spec,source='v4_local_smac_hybrid',allowed_mask=mask,
                window_start_index=a,window_end_index=b,window_path_length_m=float(np.linalg.norm(np.diff(p[a:b+1,:2],axis=0),axis=1).sum()),
                skip_path_mask_validation=True)
            diagnostics=dict(result.diagnostics or {})
            log['local_calls'].append({'anchors':[a,b],'success':bool(result.planner_success),
                'failure':result.failure_code,'ack':diagnostics.get('costmap_update_acknowledged')})
            if diagnostics.get('costmap_update_acknowledged') is not True:
                return {'success':False,'failure_code':'LOCAL_COSTMAP_ACK_FAILED','diagnostics':diagnostics,'local_l3':log,'map_version':version}
            if not result.planner_success or not result.points:return fallback('LOCAL_SMAC_FAILED')
            patch=np.asarray([[v['x'],v['y'],v['yaw']] for v in result.points])
            fitted,fit=anchored_connector(self.world,p,a,b,mask,min(deadline,time.monotonic()+1.5))
            log['local_calls'][-1]['anchor_fit']=fit
            if fitted is not None:patch=fitted
            p=np.vstack((p[:a],patch,p[b+1:]))
        template.source='v4_reference_with_local_smac' if log['local_calls'] else 'v4_reference_candidate'
        template.planner_backend='v4_l3_validator_and_local_smac' if log['local_calls'] else 'v4_l3_validator'
        points=truthful_points(p,template,len(p),len(p))
        ca,ind,traffic,jump,failures=self._audit(query,p,mask,auditor,points)
        if failures==['RAW_STEERING_JUMP']:
            from three_yd_v3.channel_repair import repair_path
            remaining=max(0.,deadline-time.monotonic())
            if remaining>.1:
                repaired,repair=repair_path(self.world,self.traffic,query,p,mask,budget_s=min(3.,remaining))
                log['splice_curvature_repair']=repair
                if repaired is not None:
                    p=repaired;template.source='v4_reference_local_smac_and_curvature_fit'
                    points=truthful_points(p,template,len(p),len(p))
                    ca,ind,traffic,jump,failures=self._audit(query,p,mask,auditor,points)
        # In addition to unchanged acceptance, a reused reference must align
        # translation with heading. Assigning a smooth yaw to a kink is invalid.
        d=np.diff(p,axis=0);motion=np.arctan2(d[:,1],d[:,0]);mid=p[:-1,2]+wrap(d[:,2])/2
        mismatch=float(np.max(abs(wrap(motion-mid)),initial=0))
        if mismatch>math.radians(5):failures.append('REFERENCE_HEADING_MOTION_MISMATCH')
        if failures:return fallback('POST_SPLICE_REJECTED:'+','.join(failures))
        if self.dynamic and self.dynamic.version!=version:
            self.trajectory=None;self.trajectory_version=None
            return {'success':False,'failure_code':'MAP_VERSION_CHANGED_DURING_LOCAL_REPAIR','local_l3':log,'map_version':version}
        result=PlanResult(planner_success=True,points=points,failure_code='',planner_backend='v4_validated_reference_with_local_smac',
            backend_version='v4-local-v1',source='v4_full_validated_local_repair',diagnostics=ack,path_audit=ca)
        self.trajectory=p;self.trajectory_version=version
        log.update(local_stage_ms=(time.monotonic()-started)*1000,all_final_checks_passed=True,
                   max_heading_motion_error_deg=math.degrees(mismatch))
        return {'success':True,'geometry_success':True,'failure_code':'','result':result,'metrics':ca.metrics,
            'independent_audit':ind,'traffic_audit':traffic,'constraint_audit':{'max_raw_steering_jump_deg':jump,'steering_jump_valid':True,'traffic':traffic},
            'diagnostics':{**ack,**ca.diagnostics()},'map_version':version,'local_l3':log,
            'reference_diagnostics':self.prepare_grid().diagnostics,'multires_diagnostics':self.timing,'l3_mode':self.l3_mode}
