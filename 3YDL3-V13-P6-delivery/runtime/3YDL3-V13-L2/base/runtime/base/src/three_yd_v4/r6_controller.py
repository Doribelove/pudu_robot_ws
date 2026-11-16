"""r6: preserve quick r5 successes, turn-conditioned L2, bounded pose repair.

No success without original canonical, swept rectangle, traffic and steering
checks. Recovery is bounded per query, never global, and never edits obstacles.
"""
from types import SimpleNamespace
from pathlib import Path
import time,math
import numpy as np
from arena_3d_v1.semantic_world import digest,SweptChecker,densify,wrap
from arena_3d_v1.curvature_repair import truthful_points
from arena_evaluation.unified_four_backends_smoke import PlanResult
from .r5_bounded import R5BoundedController,reference_neighborhood
from .r5_controller import SoftEdgeController
from .r6_lattice_l2 import turn_reference
from .r6_optimizer import optimize_path

class R6Controller(R5BoundedController):
    radii=(.8,)
    turn_radii=(.8,1.6)
    max_feedback_rounds=2
    optimizer_budget_s=5.

    def _mask(self,ref,query,radius):
        base=SoftEdgeController._target_mask(self)
        mask=reference_neighborhood(self.world,ref,query.start,query.goal,radius)&base
        n=int(mask.sum());limit=min(self.max_search_cells,int(mask.size*self.max_map_fraction))
        if n>limit:raise RuntimeError('L3_SPATIAL_BUDGET')
        return mask

    def _spline_attempt(self,query,session,auditor,ref,log,radius=.8):
        self._search_mask=self._mask(ref,query,radius)
        version=0 if self.dynamic is None else self.dynamic.version
        mask=self._target_mask().copy()
        deadline=getattr(session,'r6_deadline',time.monotonic()+60)
        budget=min(self.optimizer_budget_s,deadline-time.monotonic())
        if budget<=.05:raise RuntimeError('R6_QUERY_BUDGET')
        p,opt=optimize_path(self.world,ref,query.start,query.goal,mask,budget_s=budget)
        template=SimpleNamespace(source='r6_bounded_rectangle_spline',planner_backend='r6_pose_refinement',backend_version='r6')
        points=truthful_points(p,template,len(p),len(p))
        ca,ind,traffic,jump,failures=self._audit(query,p,mask,auditor,points)
        d=np.diff(p,axis=0);motion=np.arctan2(d[:,1],d[:,0]);mid=p[:-1,2]+wrap(d[:,2])/2
        mismatch=float(np.max(abs(wrap(motion-mid)),initial=0))*180/math.pi
        if mismatch>5:failures.append('REFERENCE_HEADING_MOTION_MISMATCH')
        stage={**opt,'radius_m':radius,'search_cells':int(mask.sum()),'failures':failures,'independent_audit':ind,
               'traffic':traffic,'steering_jump_deg':jump,'heading_motion_error_deg':mismatch,'strict_success':not failures}
        artifact=self.root/'r6_candidates';artifact.mkdir(exist_ok=True)
        stem=f'{query.query_id}_{self.task["signature"][:12]}_{time.time_ns()}'
        np.save(artifact/(stem+'_candidate.npy'),p);np.save(artifact/(stem+'_reference.npy'),ref)
        stage.update(candidate_file=str(artifact/(stem+'_candidate.npy')),reference_file=str(artifact/(stem+'_reference.npy')),not_executable=bool(failures))
        log['pose_attempts'].append(stage)
        self.r6_last_candidate=p
        self.r6_last_reference=ref.copy()
        if failures:return None,p,stage
        ack=self.sync_only(session)
        if ack.get('costmap_update_acknowledged') is not True:raise RuntimeError('R6_COSTMAP_ACK_FAILED')
        if self.dynamic and self.dynamic.version!=version:raise RuntimeError('R6_MAP_VERSION_CHANGED')
        # ACK and final sweep certify the same immutable mask/version snapshot.
        self.trajectory=p;self.trajectory_version=version
        result=PlanResult(planner_success=True,points=points,failure_code='',planner_backend=template.planner_backend,
            backend_version='r6',source=template.source,diagnostics=ack,path_audit=ca)
        return {'success':True,'geometry_success':True,'failure_code':'','result':result,'metrics':ca.metrics,
            'independent_audit':ind,'traffic_audit':traffic,'constraint_audit':{'max_raw_steering_jump_deg':jump,'steering_jump_valid':True,'traffic':traffic},
            'diagnostics':ack,'map_version':version,'legacy_contract_success':True,'reference_diagnostics':self.prepare_grid().diagnostics},p,stage

    def _feedback(self,p,query,log):
        checker=SweptChecker(self.world.hard|~SoftEdgeController._target_mask(self),self.world.map.resolution,self.world.map.origin,*self.world.vehicle[:2])
        bad=[i for i,pose in enumerate(p) if not checker.check(pose[None,:])]
        groups=[]
        for i in bad:
            if groups and i<=groups[-1][-1]+3:groups[-1].append(i)
            else:groups.append([i])
        if not groups:return False
        geo=self.l2.geometry;rc=np.asarray([geo.global_cell(i) for i in range(geo.state_count)],np.int32)
        xy=self.view.world_points(rc[:,0],rc[:,1]);cost=self.l2.state.potential.copy();records=[]
        for ids in groups:
            center=p[ids[len(ids)//2],:2];distance=np.linalg.norm(xy-center,axis=1)
            cost+=300*np.maximum(0.,1.-distance/.65)
            records.append({'center_xy':center.tolist(),'candidate_indices':[ids[0],ids[-1]],'reason':'EXACT_BODY_COLLISION',
                            'query_local_soft_penalty':True,'physical_obstacle_added':False})
        self.l2.state.replace_potential(cost)
        result=self.l2._solve(force=True)
        log['feedback'].append({'regions':records,'l2_success':result.success,'response_ms':result.response_ms})
        if not result.success:return False
        path=np.asarray(self.l2.path_global);self.initial_l2_result=result
        self._set_reference(self.view.world_points(path[:,0],path[:,1]));return True

    def plan_l3(self,query,session,auditor,spec,*,enabled=True,cap=140):
        if not enabled:raise ValueError('L2_SOFT_COST_REQUIRED')
        started=time.monotonic();self.trajectory=None;self.trajectory_version=None
        log={'version':'3YD-V4-r6','turn_attempts':[],'pose_attempts':[],'feedback':[],
             'global_fallback':False,'max_feedback_rounds':self.max_feedback_rounds,'baseline_first_radius_m':.8}
        # Every query follows the same quick branch; no Q06/Q07 special case.
        self._search_mask=self._mask(self.reference_plan.world_xy,query,.8)
        out=SoftEdgeController.plan_l3(self,query,session,auditor,spec,enabled=True,cap=cap)
        log['baseline']={'success':bool(out.get('success')),'failure':out.get('failure_code'),'wall_ms':(time.monotonic()-started)*1000}
        if out.get('success'):
            out['legacy_contract_success']=True
        elif any(x in str(out.get('failure_code')) for x in ('ACK','CONTENT','VERSION','BINDING','CORRUPT')):
            pass
        else:
            for iteration in range(self.max_feedback_rounds+1):
                if time.monotonic()>getattr(session,'r6_deadline',float('inf')):out={'success':False,'failure_code':'R6_QUERY_BUDGET'};break
                turn=None
                for radius in self.turn_radii:
                    turn,info=turn_reference(self,query,radius=radius,budget_s=3.)
                    info['round']=iteration;log['turn_attempts'].append(info)
                    if turn is not None:break
                if turn is None:
                    out={'success':False,'failure_code':'L2_TURN_REFERENCE_UNAVAILABLE'};break
                self._set_reference(turn)
                self.initial_l2_result.diagnostics['r6_turn_refinement']=info
                candidate,p,stage=self._spline_attempt(query,session,auditor,turn,log,.8)
                if candidate is not None:out=candidate;break
                out={'success':False,'geometry_success':False,'failure_code':'R6_REJECTED:'+','.join(stage['failures']),
                     'independent_audit':stage['independent_audit'],'traffic_audit':stage['traffic'],
                     'constraint_audit':{'max_raw_steering_jump_deg':stage['steering_jump_deg'],'steering_jump_valid':stage['steering_jump_deg']<=15,'traffic':stage['traffic']}}
                # One fresh bounded Smac call tests whether the turn-conditioned
                # L2 reference resolves the native search before cost feedback.
                if iteration==0:
                    native=SoftEdgeController.plan_l3(self,query,session,auditor,spec,enabled=True,cap=cap)
                    log['turn_native']={'success':bool(native.get('success')),'failure':native.get('failure_code')}
                    if native.get('success'):out=native;out['legacy_contract_success']=True;break
                    if any(x in str(native.get('failure_code')) for x in ('ACK','CONTENT','VERSION','BINDING','CORRUPT')):out=native;break
                if iteration>=self.max_feedback_rounds or not self._feedback(p,query,log):break
        if not out.get('success'):self.trajectory=None;self.trajectory_version=None
        log['wall_ms']=(time.monotonic()-started)*1000
        out['r6']=log;out['local_l3']=log
        out['bounded_l3']={'global_fallback':False,'max_radius_m':.8,'search_cells':int(self._target_mask().sum()),
            'limit_cells':min(self.max_search_cells,int(self.world.hard.size*self.max_map_fraction)),
            'L2_soft_cost_enabled':True,'storage':'original full-size persistent map; only bounded neighborhood available'}
        return out
