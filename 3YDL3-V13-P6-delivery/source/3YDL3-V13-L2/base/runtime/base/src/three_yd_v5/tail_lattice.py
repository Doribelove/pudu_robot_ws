"""Bounded endpoint-only pose search for spline initialization.

Every primitive is swept in the existing mask. This never supplies a final
trajectory, expands a scope, changes a fixed pose, or calls a legacy L3 backend.
"""
import heapq,math,time
import numpy as np


def angle(x):return (x+math.pi)%(2*math.pi)-math.pi


def search(a,b,world,hard,deadline,max_nodes=2000):
    from .v10_storage import collision_checker
    from arena_evaluation.semantic_constraint_core import dubins_choices,dubins_edge_from_parameters
    from arena_3d_v1.semantic_world import densify
    checker=collision_checker(hard,world);a=np.asarray(a,float);b=np.asarray(b,float)
    if not checker.check(np.vstack((a,b))):
        # Only the two endpoint poses are checked here, not their chord.
        if not checker.check(a[None]) or not checker.check(b[None]):return None,{'reason':'endpoint_body_blocked'}
    low=np.minimum(a[:2],b[:2])-1.5;high=np.maximum(a[:2],b[:2])+1.5
    def key(p):return (round(p[0]/.075),round(p[1]/.075),round(angle(p[2])/math.radians(7.5)))
    def heuristic(p):return float(np.linalg.norm(p[:2]-b[:2]))
    nodes=[(a,-1,None,0.)];queue=[(heuristic(a),0)];cost={key(a):0.};expanded=0;shots=0
    length=.15;steps=np.linspace(0,length,6);radius=max(.42,world.vehicle[2]*1.05)
    while queue and expanded<max_nodes and time.monotonic()<deadline:
        _,i=heapq.heappop(queue);pose,parent,edge,g=nodes[i]
        if g>cost.get(key(pose),float('inf'))+1e-8:continue
        expanded+=1
        if expanded==1 or expanded%5==0 or heuristic(pose)<1.:
            for rad in (radius,max(.5,radius),max(.65,radius)):
                choices=sorted(dubins_choices(tuple(pose),tuple(b),rad),key=lambda v:sum(v[1]))
                for word,parameters in choices:
                    if rad*sum(parameters)>heuristic(pose)+4.:continue
                    if time.monotonic()>=deadline:break
                    connector=dubins_edge_from_parameters(tuple(pose),tuple(b),rad,word,parameters)
                    samples=densify(np.vstack((pose,connector.samples)));shots+=1
                    if np.any(samples[:,:2]<low) or np.any(samples[:,:2]>high) or not checker.check(samples):continue
                    chain=[samples];j=i
                    while nodes[j][1]>=0:chain.append(nodes[j][2]);j=nodes[j][1]
                    result=np.vstack(list(reversed(chain)))
                    result=result[np.r_[True,np.linalg.norm(np.diff(result[:,:2],axis=0),axis=1)>1e-8]]
                    return result,{'expanded':expanded,'shots':shots,'radius_m':radius,'scope_unchanged':True}
        for curvature in (0.,1/radius,-1/radius,.5/radius,-.5/radius):
            yaw=pose[2]+curvature*steps
            if curvature:
                x=pose[0]+(np.sin(yaw)-math.sin(pose[2]))/curvature
                y=pose[1]-(np.cos(yaw)-math.cos(pose[2]))/curvature
            else:x=pose[0]+steps*math.cos(pose[2]);y=pose[1]+steps*math.sin(pose[2])
            samples=np.c_[x,y,np.arctan2(np.sin(yaw),np.cos(yaw))];end=samples[-1];k=key(end)
            new=g+length*(1.+.04*abs(curvature))
            if new>=cost.get(k,float('inf')) or np.any(end[:2]<low) or np.any(end[:2]>high):continue
            if not checker.check(samples):continue
            cost[k]=new;nodes.append((end,i,samples,new));heapq.heappush(queue,(new+heuristic(end),len(nodes)-1))
    return None,{'expanded':expanded,'shots':shots,'reason':'bounded_endpoint_search_exhausted'}


def seed(p,s,start,goal,world,hard,deadline):
    from .curve_l3 import interp
    logs=[]
    for role,pose in (('start',start),('goal',goal)):
        delta=p[1]-p[0] if role=='start' else p[-1]-p[-2]
        mismatch=abs(angle(math.atan2(delta[1],delta[0])-pose[2]))
        if mismatch<math.radians(25):continue
        tick=time.monotonic();chosen=None;trials=[]
        for distance in (4.,2.,6.):
            if distance>.4*s[-1]:continue
            at=distance if role=='start' else s[-1]-distance
            xy=interp(p,s,[at])[0];before,after=interp(p,s,[max(0,at-.1),min(s[-1],at+.1)])
            anchor=tuple(xy)+(math.atan2(*(after-before)[::-1]),)
            a,b=(pose,anchor) if role=='start' else (anchor,pose)
            curve,log=search(a,b,world,hard,min(deadline,time.monotonic()+.75))
            trials.append(dict(log,anchor_distance_m=distance))
            if curve is not None:chosen=(at,curve);break
            if time.monotonic()>=deadline:break
        logs.append({'endpoint':role,'selected':chosen is not None,'trials':trials,'wall_ms':(time.monotonic()-tick)*1000})
        if chosen is None:continue
        at,curve=chosen
        p=np.vstack((curve[:,:2],p[s>at])) if role=='start' else np.vstack((p[s<at],curve[:,:2]))
        p=p[np.r_[True,np.linalg.norm(np.diff(p,axis=0),axis=1)>1e-7]]
        s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
    return p,s,logs
