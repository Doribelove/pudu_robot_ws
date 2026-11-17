"""Bounded pose-aware terminal initializer. Never an executable result."""
import ctypes,math,time
from pathlib import Path
import numpy as np

_LIB=None

def search(a,b,world,hard,budget,radius,max_nodes=60000):
    global _LIB
    from .v10_storage import collision_checker
    checker=collision_checker(hard,world)
    if _LIB is None:
        _LIB=ctypes.CDLL(str(Path(__file__).resolve().parents[2]/'native/libfailure_endpoint.so'))
        _LIB.endpoint_seed.argtypes=[ctypes.c_size_t,ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_void_p,ctypes.c_void_p]+[ctypes.c_double]*7+[ctypes.c_int,ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p]
        _LIB.endpoint_seed.restype=ctypes.c_int
    a=np.ascontiguousarray(a,dtype=float);b=np.ascontiguousarray(b,dtype=float)
    output=np.empty((10000,3),float);stats=np.zeros(3,np.int32)
    n=_LIB.endpoint_seed(ctypes.cast(checker.fn,ctypes.c_void_p).value,checker.hard.ctypes.data,*checker.hard.shape,a.ctypes.data,b.ctypes.data,checker.res,*checker.origin[:2],checker.half_length,checker.half_width,radius,budget,max_nodes,output.ctypes.data,len(output),stats.ctypes.data)
    return (output[:n].copy() if n>0 else None),{'expanded':int(stats[0]),'shots':int(stats[1]),'status':int(stats[2]),'native_return':n}

def terminal_guide(world,physical,guide,start,goal,deadline):
    from .curve_l3 import clean_guide,interp
    tick=time.monotonic();log={'selected':False,'initialization_only':True,'trials':[]}
    p,s=clean_guide(guide,start,goal,.22)
    direction=p[-1]-p[-2]
    mismatch=abs(math.atan2(math.sin(math.atan2(direction[1],direction[0])-goal[2]),math.cos(math.atan2(direction[1],direction[0])-goal[2])))
    log['goal_heading_mismatch_deg']=math.degrees(mismatch)
    if mismatch<math.radians(75):return None,log
    hard=world.hard|~physical
    # Looking before the last few bends permits approach from the other side
    # of a local wall. The footprint predicate and L1 physical scope remain.
    for distance in (14.,10.,6.,3.):
        if distance>.4*s[-1] or time.monotonic()>=deadline:continue
        at=s[-1]-distance;xy=interp(p,s,[at])[0];v=np.diff(interp(p,s,[at-.05,at+.05]),axis=0)[0]
        anchor=np.r_[xy,math.atan2(v[1],v[0])]
        a=np.asarray(goal,float).copy();b=anchor.copy();a[2]+=math.pi;b[2]+=math.pi
        for scale in (2.,1.5,1.25):
            budget=min(.2,deadline-time.monotonic())
            if budget<=.005:break
            seed,detail=search(a,b,world,hard,budget,world.vehicle[2]*scale)
            log['trials'].append(dict(detail,anchor_distance_m=distance,radius_m=world.vehicle[2]*scale))
            if seed is None:continue
            # Reverse computational traversal, retaining forward robot motion.
            seed=seed[::-1].copy();seed[:,2]=np.arctan2(np.sin(seed[:,2]+math.pi),np.cos(seed[:,2]+math.pi))
            if not np.allclose(seed[0,:2],anchor[:2],atol=1e-7,rtol=0) or not np.allclose(seed[-1,:2],goal[:2],atol=1e-7,rtol=0):raise ValueError('TERMINAL_SEED_ENDPOINT_BINDING')
            angles=np.array([seed[0,2]-anchor[2],seed[-1,2]-goal[2]])
            if np.any(np.abs(np.arctan2(np.sin(angles),np.cos(angles)))>1e-7):raise ValueError('TERMINAL_SEED_HEADING_BINDING')
            result=np.vstack((p[s<at],seed[:,:2]));log.update(selected=True,anchor_distance_m=distance,local_center_bounds=[seed[:,:2].min(0).tolist(),seed[:,:2].max(0).tolist()],wall_ms=(time.monotonic()-tick)*1000)
            return result,log
    log['wall_ms']=(time.monotonic()-tick)*1000
    return None,log
