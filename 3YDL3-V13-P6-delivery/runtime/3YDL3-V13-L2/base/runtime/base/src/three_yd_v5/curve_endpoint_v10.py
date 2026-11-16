"""Local backwards SEARCH for a forward-only endpoint initializer.

The returned sequence always travels forwards from the original guide anchor
into the unchanged goal pose. This module does not certify or deliver paths.
"""
import heapq,math,time
import numpy as np
from arena_3d_v1.semantic_world import SweptChecker,densify
from arena_evaluation.semantic_constraint_core import dubins_choices,dubins_edge_from_parameters

def connect(world,mask,start,goal,deadline,max_nodes=2500,radius=.45):
    checker=SweptChecker(world.hard|~mask,world.map.resolution,world.map.origin,*world.vehicle[:2])
    a=np.asarray(start,float);b=np.asarray(goal,float);radius=max(float(radius),world.vehicle[2]*1.01)
    if np.linalg.norm(a[:2]-b[:2])>5.5:return None,{'reason':'LOCAL_DISTANCE_LIMIT'}
    def key(p):return (round(p[0]/.08),round(p[1]/.08),round((p[2]%(2*math.pi))/(2*math.pi/96))%96)
    def h(p):return np.linalg.norm(p[:2]-a[:2])+.2*abs(math.atan2(math.sin(p[2]-a[2]),math.cos(p[2]-a[2])))
    nodes=[(b,-1,None,0.)];frontier=[(h(b),0)];best={key(b):0.};expanded=0;shots=0
    ds=-np.linspace(.04,.24,6);steers=(0.,1/radius,-1/radius,.5/radius,-.5/radius)
    while frontier and expanded<max_nodes and time.monotonic()<deadline:
        _,index=heapq.heappop(frontier);p,parent,edge,g=nodes[index]
        if g>best.get(key(p),float('inf'))+1e-8:continue
        expanded+=1
        if expanded%4==1:
            choices=sorted(dubins_choices(tuple(a),tuple(p),radius),key=lambda q:sum(q[1]))
            for word,pars in choices:
                if time.monotonic()>=deadline:break
                if radius*sum(pars)>np.linalg.norm(a[:2]-b[:2])+5.-g:continue
                shot=dubins_edge_from_parameters(tuple(a),tuple(p),radius,word,pars)
                head=densify(np.vstack((a,shot.samples)));shots+=1
                if checker.check(head):
                    parts=[head];j=index
                    while nodes[j][1]>=0:
                        parts.append(nodes[j][2][::-1]);j=nodes[j][1]
                    path=np.vstack(parts);keep=np.r_[True,np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1)>1e-8];path=path[keep]
                    return path,{'expanded':expanded,'shots':shots,'radius_m':radius,'forward_only':True,'search_direction':'from_goal','node_cap':max_nodes}
        for k in steers:
            y=p[2]+k*ds
            if k:xy=p[:2]+np.c_[(np.sin(y)-math.sin(p[2]))/k,(-np.cos(y)+math.cos(p[2]))/k]
            else:xy=p[:2]+ds[:,None]*[math.cos(p[2]),math.sin(p[2])]
            edge=np.vstack((p,np.c_[xy,y]));q=edge[-1];newg=g+.24;ky=key(q)
            if np.linalg.norm(q[:2]-b[:2])>3. or newg>5. or newg>=best.get(ky,float('inf'))-1e-8:continue
            if not checker.check(edge):continue
            best[ky]=newg;nodes.append((q,index,edge,newg));heapq.heappush(frontier,(newg+1.6*h(q),len(nodes)-1))
    return None,{'expanded':expanded,'shots':shots,'reason':'LOCAL_SEARCH_LIMIT','node_cap':max_nodes}
