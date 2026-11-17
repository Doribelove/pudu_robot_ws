"""Owned, serial, persistent P6 session. One instance per fresh interpreter.

All events enter the original IncrementalService. This wrapper validates input,
controls lifecycle, and gates executable output; it never substitutes planners,
ACKs or original sequence/version outcomes. External timeout supervision required.
"""
import hashlib,json,math,re,threading,time,traceback
from pathlib import Path
from .api import MapOnlyRequest,SemanticRequest,_json
from .runtime_owner import RuntimeOwner

class SessionStateError(RuntimeError):pass

def _number(v):return isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
def _id(v):return isinstance(v,str) and re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}',v) is not None and '..' not in v

class P6Session:
    def __init__(self,request,*,resource_root=None,solver_root=None):
        if not isinstance(request,MapOnlyRequest):raise TypeError('MapOnlyRequest or SemanticRequest required')
        self.request=request;self._owner=RuntimeOwner(request,resource_root=resource_root,solver_root=solver_root)
        self._lock=threading.Lock();self._state='new';self._published=False;self._published_version=None
        self._events=0;self._close_result=None;self._planned=False
    def _acquire(self,*,open_required=True):
        if not self._lock.acquire(blocking=False):raise SessionStateError('Concurrent session operations are forbidden')
        if open_required and self._state!='open':
            self._lock.release();raise SessionStateError('Session is not open: '+self._state)
    def open(self):
        self._acquire(open_required=False)
        try:
            if self._state!='new':raise SessionStateError('Session cannot be reopened: '+self._state)
            if not _id(self.request.query_id):raise ValueError('Invalid query_id')
            self._state='opening'
            try:self._owner.open()
            except BaseException:
                self._close_result=self._owner.close();self._state='closed';raise
            self._state='open';return self
        finally:self._lock.release()
    def _pose(self,pose,label):
        if not isinstance(pose,(tuple,list)) or len(pose)!=3 or not all(_number(v) for v in pose):raise ValueError(label+' must be three finite numeric values')
        m=self._owner.world.map
        if not(m.origin[0]<=pose[0]<m.origin[0]+m.width*m.resolution and m.origin[1]<=pose[1]<m.origin[1]+m.height*m.resolution):raise ValueError(label+' outside map')
        return tuple(float(v) for v in pose)
    def _versions(self):
        service=self._owner.service
        return {'dynamic_version':int(service.dynamic.version),'trajectory_version':getattr(service.controller,'trajectory_version',None),'sequence':int(service.dynamic.sequence),'native_pid':self._owner.session.stack.process.pid}
    def _integrity(self):
        errors=self._owner.integrity_errors()
        process=self._owner.session.stack.process
        if process.poll() is not None:errors.append('Native worker exited: '+str(process.returncode))
        if errors:
            self._published=False;raise SessionStateError('; '.join(errors))
    def _trajectory(self):
        self._integrity();versions=self._versions()
        path=[]
        if self._published and self._published_version==versions['dynamic_version']:
            candidate=self._owner.service.current_trajectory()
            if candidate is not None and versions['trajectory_version']==versions['dynamic_version']:path=_json(candidate)
        return {'available':bool(path),'path':path,'frame_id':'map',**versions}
    def current_trajectory(self):
        self._acquire()
        try:return self._trajectory()
        finally:self._lock.release()
    def map_context(self):
        self._acquire()
        try:
            self._integrity()
            import numpy as np
            world=self._owner.world;m=world.map
            def mask_hash(mask):
                return hashlib.sha256(np.flipud(np.asarray(mask,dtype=np.uint8)).copy(order='C').tobytes(order='C')).hexdigest()
            static_hash=mask_hash(self._owner.service.dynamic.static_hard)
            effective_hash=mask_hash(world.hard)
            return {'frame_id':'map','width':m.width,'height':m.height,'resolution':m.resolution,'origin':list(m.origin[:2]),'hard_mask_sha256':static_hash,'static_hard_mask_sha256':static_hash,'effective_hard_mask_sha256':effective_hash,'dynamic_version':int(self._owner.service.dynamic.version),'sequence':int(self._owner.service.dynamic.sequence)}
        finally:self._lock.release()
    def _execute(self,operation,call):
        # Caller holds the operation lock; withdraw exposure before any failure.
        self._published=False;self._published_version=None
        self._integrity();before=len(self._owner.session.ack_events);start=time.monotonic()
        result={};planned=None;failure=None
        try:
            result=call();planned=result.pop('result',None)
            self._integrity()
            self._published=bool(result.get('success'))
            self._published_version=int(self._owner.service.dynamic.version) if self._published else None
        except BaseException:failure=traceback.format_exc();self._published=False
        current=self._trajectory() if failure is None else {'available':False,'path':[],**self._versions()}
        success=failure is None and bool(result.get('success')) and current['available']
        if not success:self._published=False
        data=_json({'schema':'trilayer-p6-session-event-v1','operation':operation,'success':success,'path':current['path'] if success else [],'frame_id':'map','result':result,'plan_result':planned,'exception':failure,'ack_events':self._owner.session.ack_events[before:],'wall_s':time.monotonic()-start,**self._versions()})
        self._events+=1
        (self._owner.out/f'event_{self._events:06d}.json').write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
        return data
    def plan(self,start=None,goal=None,*,query_id=None,seed=None):
        self._acquire()
        try:
            self._published=False
            start=self._pose(self.request.start if start is None else start,'start');goal=self._pose(self.request.goal if goal is None else goal,'goal')
            if isinstance(self.request,SemanticRequest) and (start!=tuple(self.request.start) or goal!=tuple(self.request.goal)):
                raise ValueError('Semantic session start/goal are fixed to the validated request; open a fresh process for different endpoints')
            qid=self.request.query_id if query_id is None else query_id
            if not _id(qid):raise ValueError('Invalid query_id')
            if isinstance(self.request,SemanticRequest) and qid!=self.request.query_id:raise ValueError('Semantic session query_id is bound to validated parking inputs')
            seed=self.request.seed if seed is None else seed
            if type(seed) is not int:raise ValueError('seed must be integer')
            from arena_evaluation.planner_benchmark.models import Query
            q=Query(qid,start,goal,seed=seed)
            def call():
                self._owner.session.reset_query_state(qid,restore_base_map=False)
                self._planned=True
                return self._owner.service.plan(q)
            return self._execute('plan',call)
        finally:self._lock.release()
    def _event(self,sequence,upsert,clear,observed_free,current_pose):
        if type(sequence) is not int:raise ValueError('sequence must be an integer')
        if type(observed_free) is not bool:raise ValueError('observed_free must be boolean')
        if upsert is None:upsert={}
        if not isinstance(upsert,dict) or len(upsert)>64:raise ValueError('upsert must map at most 64 object IDs to polygons')
        if not isinstance(clear,(list,tuple)) or len(clear)>64 or any(not _id(k) for k in clear) or len(set(clear))!=len(clear):raise ValueError('clear must contain unique object IDs')
        normalized={}
        for key,polygon in upsert.items():
            if not _id(key):raise ValueError('Invalid obstacle ID')
            if key in clear:raise ValueError('An event cannot both upsert and clear one object')
            if not isinstance(polygon,(list,tuple)) or not 3<=len(polygon)<=128:raise ValueError('polygon must have 3..128 vertices')
            points=[]
            for pt in polygon:
                if not isinstance(pt,(list,tuple)) or len(pt)!=2 or not all(_number(v) for v in pt):raise ValueError('polygon vertices must be finite numeric xy')
                self._pose([pt[0],pt[1],0.],'polygon vertex');points.append([pt[0],pt[1]])
            if len({tuple(p) for p in points})!=len(points):raise ValueError('polygon has duplicate vertices')
            area=sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(points,points[1:]+points[:1]))
            if abs(area)<1e-10:raise ValueError('polygon is degenerate')
            def orient(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
            n=len(points)
            for i in range(n):
                for j in range(i+1,n):
                    if j in (i,(i+1)%n) or i==(j+1)%n:continue
                    a,b,c,d=points[i],points[(i+1)%n],points[j],points[(j+1)%n]
                    if orient(a,b,c)*orient(a,b,d)<=0 and orient(c,d,a)*orient(c,d,b)<=0 and max(min(a[0],b[0]),min(c[0],d[0]))<=min(max(a[0],b[0]),max(c[0],d[0])) and max(min(a[1],b[1]),min(c[1],d[1]))<=min(max(a[1],b[1]),max(c[1],d[1])):raise ValueError('polygon self-intersects')
            normalized[key]=points
        pose=None if current_pose is None else self._pose(current_pose,'current_pose')
        return normalized,list(clear),pose
    def update(self,sequence,*,upsert=None,clear=(),observed_free=False,current_pose=None):
        self._acquire()
        try:
            self._published=False
            if not self._planned:raise SessionStateError('plan must be called before update')
            upsert,clear,current_pose=self._event(sequence,upsert,clear,observed_free,current_pose)
            # Original DynamicObstacles handles duplicate/stale/conflict and free-observation semantics.
            return self._execute('update',lambda:self._owner.service.update(sequence,upsert=upsert,clear=clear,observed_free=observed_free,current_pose=current_pose))
        finally:self._lock.release()
    def close(self):
        self._acquire(open_required=False)
        try:
            if self._state=='closed':return self._close_result
            self._published=False;self._published_version=None
            self._close_result=self._owner.close();self._state='closed'
            return self._close_result
        finally:self._lock.release()
    def __enter__(self):return self.open()
    def __exit__(self,exc_type,exc,tb):
        outcome=self.close()
        if exc_type is None and (outcome['cleanup_errors'] or outcome['worker_exit']!=0):raise SessionStateError('Session cleanup failed: '+str(outcome['cleanup_errors']))
        return False
