"""Bounded, ordered pose initializers. All output still needs strict L3 audit."""
import ctypes,math,time
from pathlib import Path
import numpy as np
from .v10_storage import collision_checker

_LIB=None
_WINDOW_LIB=None

class PoseSearch:
    def __init__(self,world,physical,labels,roads):
        global _LIB
        self.world=world;self.checker=collision_checker(world.hard|~physical,world)
        if np.shape(labels)!=np.shape(world.hard):
            raise ValueError('POSE_SEED_LABEL_SHAPE')
        from .v12_fields import BlockField
        self.blocked_labels=type(labels) is BlockField
        roads=list(roads)
        self.labels=labels if self.blocked_labels else np.ascontiguousarray(labels,dtype=np.int16)
        largest=max((int(rid) for rid,_ in roads),default=-1) if self.blocked_labels else int(self.labels.max())
        self.directions=np.zeros((largest+1,2),float)
        for rid,direction in roads:
            direction=np.asarray(direction,float)
            if np.linalg.norm(self.directions[rid])>.5 and np.dot(self.directions[rid],direction)<.999:
                raise ValueError('POSE_SEED_AMBIGUOUS_ROAD_DIRECTION')
            self.directions[rid]=direction
        if _LIB is None:
            _LIB=ctypes.CDLL(str(Path(__file__).resolve().parents[2]/'native/libfailure_heading.so'))
            fn=_LIB.endpoint_seed_heading
            fn.argtypes=[ctypes.c_size_t,ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_void_p,ctypes.c_void_p]+[ctypes.c_double]*7+[ctypes.c_int,ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p,ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int,ctypes.c_double]
            fn.restype=ctypes.c_int
        self.fn=_LIB.endpoint_seed_heading
        if self.blocked_labels:
            global _WINDOW_LIB
            if _WINDOW_LIB is None:
                _WINDOW_LIB=ctypes.CDLL(str(Path(__file__).resolve().parents[2]/'native/libv12_failure_heading.so'))
                fn=_WINDOW_LIB.v12_endpoint_seed_heading
                fn.argtypes=_LIB.endpoint_seed_heading.argtypes+[ctypes.c_int]*4;fn.restype=ctypes.c_int
            self.fn=_WINDOW_LIB.v12_endpoint_seed_heading
        self.output=np.empty((10000,3),float)
    def search(self,a,b,radius,deadline,budget,reverse=False,max_nodes=500000):
        aa,bb=(np.array(b,float),np.array(a,float)) if reverse else (np.array(a,float),np.array(b,float))
        if reverse:aa[2]+=math.pi;bb[2]+=math.pi
        remaining=min(budget,deadline-time.monotonic())
        if remaining<=0:return None,{'status':'deadline'}
        read_started=time.monotonic();c=self.checker;stats=np.zeros(3,np.int32)
        labels=self.labels;extra=();label_bytes=0
        if self.blocked_labels:
            # The original native search admits only the endpoint box +/-2m.
            # Include one extra grid cell on every edge for floor rounding.
            h,w=c.hard.shape;ox,oy=c.origin[:2]
            x0,x1=min(aa[0],bb[0])-2.,max(aa[0],bb[0])+2.
            y0,y1=min(aa[1],bb[1])-2.,max(aa[1],bb[1])+2.
            c0=max(0,math.floor((x0-ox)/c.res)-1);c1=min(w,math.floor((x1-ox)/c.res)+2)
            r0=max(0,h-2-math.floor((y1-oy)/c.res));r1=min(h,h+1-math.floor((y0-oy)/c.res))
            if r0>=r1 or c0>=c1:return None,{'status':3,'label_window_bytes':0}
            labels=np.ascontiguousarray(self.labels[r0:r1,c0:c1].read(),dtype=np.int16)
            extra=(r0,c0,r1-r0,c1-c0);label_bytes=labels.nbytes
            # Label IO is charged to the same per-connection deadline.
            remaining=min(budget-(time.monotonic()-read_started),deadline-time.monotonic())
            if remaining<=0:return None,{'status':'deadline','label_window_bytes':label_bytes}
        n=self.fn(ctypes.cast(c.fn,ctypes.c_void_p).value,c.hard.ctypes.data,*c.hard.shape,aa.ctypes.data,bb.ctypes.data,c.res,*c.origin[:2],c.half_length,c.half_width,radius,remaining,max_nodes,self.output.ctypes.data,len(self.output),stats.ctypes.data,labels.ctypes.data,self.directions.ctypes.data,len(self.directions),-1. if reverse else 1.,*extra)
        info={'expanded':int(stats[0]),'shots':int(stats[1]),'status':int(stats[2]),'reverse_computation':reverse,'radius':radius,'blocked_label_window_bytes':label_bytes}
        if n<=0:return None,info
        p=self.output[:n].copy()
        if reverse:p=p[::-1].copy();p[:,2]=np.arctan2(np.sin(p[:,2]+math.pi),np.cos(p[:,2]+math.pi))
        for actual,expected in [(p[0],a),(p[-1],b)]:
            if (np.max(abs(actual[:2]-np.asarray(expected)[:2]))>1e-7 or
                abs(math.atan2(math.sin(actual[2]-expected[2]),math.cos(actual[2]-expected[2])))>1e-7):
                raise ValueError('POSE_CHAIN_ENDPOINT_BINDING')
        return p,info

def pose_chain(world,physical,labels,roads,guide,start,goal,deadline):
    from .curve_l3 import clean_guide,interp
    tick=time.monotonic();log={'initialization_only':True,'selected':False,'connections':[]}
    p,s=clean_guide(guide,start,goal,.22);engine=PoseSearch(world,physical,labels,roads)
    def at_pose(at):
        xy=interp(p,s,[at])[0];before,after=interp(p,s,[max(0.,at-.3),min(s[-1],at+.3)])
        v=after-before
        return np.r_[xy,math.atan2(v[1],v[0])],np.linalg.norm(before+after-2*xy)
    mismatch=abs(math.atan2(math.sin(math.atan2(*(p[-1]-p[-2])[::-1])-goal[2]),math.cos(math.atan2(*(p[-1]-p[-2])[::-1])-goal[2])))
    tail=None;tail_at=s[-1];tail_anchor=np.asarray(goal)
    if mismatch>math.radians(75):
        for distance in (24.,18.,12.,6.):
            if distance>.4*s[-1] or time.monotonic()>=deadline:continue
            at=s[-1]-distance;a,_=at_pose(at)
            if not engine.checker.check(a[None,:]):continue
            for radius,budget in ((world.vehicle[2]*2.,.025),(world.vehicle[2]*1.5,.025),(world.vehicle[2]*1.25,.025),(world.vehicle[2]*1.0125,.2 if distance<=12. else 2.8)):
                tail,detail=engine.search(a,goal,radius,deadline,budget,True)
                log['connections'].append(dict(detail,terminal=True,anchor_distance=distance,found=tail is not None))
                if tail is not None:tail_at=at;tail_anchor=a;break
            if tail is not None:break
        if tail is None:return None,dict(log,failure='POSE_CHAIN_TERMINAL',wall_ms=(time.monotonic()-tick)*1000)
    anchors=[np.asarray(start)];stations=[0.]
    for nominal in np.arange(5.,tail_at-2.,5.):
        for offset in (0.,-.5,.5,-1.,1.,-1.5,1.5,-2.,2.):
            at=nominal+offset
            if at-stations[-1]<2 or tail_at-at<2:continue
            pose,bend=at_pose(at)
            if bend>.003 or not engine.checker.check(pose[None,:]):continue
            anchors.append(pose);stations.append(at);break
    anchors.append(tail_anchor);stations.append(tail_at)
    links={};last=len(anchors)-1;dead=set();route=[]
    def connect(i):
        if i==last:return True
        choices=[j for j in range(min(last,i+4),i,-1) if stations[j]-stations[i]<=21 and j not in dead]
        # Finish the current travel direction before attempting a corner.
        # This avoids spending the local budget crossing two turns at once.
        def priority(j):
            delta=abs(math.atan2(math.sin(anchors[j][2]-anchors[i][2]),math.cos(anchors[j][2]-anchors[i][2])))
            return (int(delta>math.radians(30))+int(delta>math.radians(100)),-j)
        choices.sort(key=priority)
        for j in choices:
            if time.monotonic()>=deadline:return False
            result=None
            for radius in (world.vehicle[2]*2,world.vehicle[2]*1.5,world.vehicle[2]*1.25,world.vehicle[2]*1.0125):
                extended=j-i>=3 and radius==world.vehicle[2]*2
                # Probe both travel-equivalent search directions before a
                # costly expansion. The original extended reverse search is
                # retained when neither bounded probe produces an initializer.
                searches=[(True,.07,10000),(False,.07,10000)]
                if extended:searches.append((True,.6,100000))
                for reverse,budget,nodes in searches:
                    result,detail=engine.search(anchors[i],anchors[j],radius,deadline,budget,reverse,max_nodes=nodes)
                    log['connections'].append(dict(detail,start_index=i,goal_index=j,found=result is not None))
                    if result is not None:break
                if result is not None or time.monotonic()>=deadline:break
            if result is not None:
                links[(i,j)]=result;route.append((i,j))
                if connect(j):return True
                route.pop()
        dead.add(i);return False
    if not connect(0):
        return None,dict(log,failure='POSE_CHAIN_BUDGET' if time.monotonic()>=deadline else 'POSE_CHAIN_DISCONNECTED',wall_ms=(time.monotonic()-tick)*1000)
    parts=[links[e] for e in route]
    if tail is not None:parts.append(tail)
    result=np.vstack([parts[0],*[x[1:] for x in parts[1:]]])
    if not engine.checker.check(result):raise ValueError('POSE_CHAIN_BODY_RECHECK')
    log.update(selected=True,anchors=len(anchors),links=route,poses=len(result),wall_ms=(time.monotonic()-tick)*1000)
    return result,log
