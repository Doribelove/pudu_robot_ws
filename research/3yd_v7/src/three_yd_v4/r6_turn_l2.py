"""Query-local turn-memory refinement of D*'s 0.15 m reference.

Uses the very same hard rings, semantic occurrence graph, direction rules and
soft edge costs. The search state also records incoming direction and distance
since the last turn; it does not pretend a first-order grid cost bounds turns.
"""
import heapq,math,time
import numpy as np
from scipy.spatial import cKDTree
from arena_3d_v1.semantic_world import wrap
DIRECTIONS=[(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1)]
INDEX={d:i for i,d in enumerate(DIRECTIONS)}

def turn_allowed(previous,new,run):
    turn=abs((new-previous+4)%8-4)
    return turn<=1 and (not turn or run>=2)

def turn_reference(c,query,*,radius=.8,budget_s=3.,max_states=160000):
    t=time.monotonic();state=c.l2.state;geo=state.geometry
    rc=np.asarray([geo.global_cell(i) for i in range(geo.state_count)],np.int32)
    xy=c.view.world_points(rc[:,0],rc[:,1]);dist=cKDTree(c.reference_plan.world_xy).query(xy)[0]
    allowed=dist<=radius;allowed[state.start_id]=True;allowed[state.goal_id]=True
    head=int(np.round(query.start[2]/(math.pi/4)))%8;goalhead=int(np.round(query.goal[2]/(math.pi/4)))%8
    encode=lambda node,heading,run:node*24+heading*3+run
    start=encode(state.start_id,head,2);g={start:0.};parent={};heap=[(state._heuristic(state.start_id,state.goal_id),0.,start)];expanded=0;found=None
    edge_cache={}
    while heap:
        if expanded%128==0 and (time.monotonic()-t>budget_s or len(g)>=max_states):break
        _,cost,key=heapq.heappop(heap)
        if cost!=g[key]:continue
        u=key//24;heading=(key%24)//3;run=key%3;expanded+=1
        if u==state.goal_id and abs((heading-goalhead+4)%8-4)<=1:found=key;break
        if u not in edge_cache:
            edges=[]
            for v in geo.neighbor_ids(u):
                if not allowed[v]:continue
                dr,dc=rc[v]-rc[u];d=INDEX.get((int(dr),int(dc)))
                if d is None:continue
                ec=state._edge_cost(u,v)
                if math.isfinite(ec):edges.append((v,d,ec))
            edge_cache[u]=edges
        for v,d,ec in edge_cache[u]:
            if not turn_allowed(heading,d,run):continue
            nr=min(2,run+1) if d==heading else 0
            nk=encode(v,d,nr);ng=cost+ec+(2. if d!=heading else 0.)
            if ng<g.get(nk,math.inf):
                g[nk]=ng;parent[nk]=key
                heapq.heappush(heap,(ng+1.5*state._heuristic(v,state.goal_id),ng,nk))
    log={'backend':'DStar_reference_turn_memory_AStar','success':found is not None,'max_grid_turn_deg':45.,
         'straight_steps_between_turns':2,'expanded':expanded,'states':len(g),'wall_ms':(time.monotonic()-t)*1000,
         'scope_radius_m':radius,'same_obstacle_rings':True,'same_soft_edge_costs':True,'not_vehicle_certificate':True,
         'failure':'' if found is not None else ('L2_TURN_BUDGET' if heap else 'L2_TURN_NO_REFERENCE')}
    if found is None:return None,log
    ids=[found]
    while ids[-1]!=start:ids.append(parent[ids[-1]])
    ids.reverse();nodes=np.array(ids)//24;ref=xy[nodes]
    heading=np.arctan2(np.diff(ref[:,1]),np.diff(ref[:,0]));traffic=c.traffic.audit(np.column_stack((ref,np.r_[heading,heading[-1]])))
    if not traffic['valid']:return None,{**log,'success':False,'failure':'L2_TURN_TRAFFIC','traffic':traffic}
    log.update(path_nodes=len(ref),reference_length_m=float(np.linalg.norm(np.diff(ref,axis=0),axis=1).sum()))
    return ref,log
