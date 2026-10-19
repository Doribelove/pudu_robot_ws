"""Sixteen translation directions and length-aware turn constraints for L2.
Knight chords use BOTH legal 8-neighbour walks covering every crossed cell.
The reference is still a point guide, not a swept-vehicle certificate.
"""
import heapq,math,time
import numpy as np
from scipy.spatial import cKDTree
VECTORS=[(dr,dc) for dr in range(-2,3) for dc in range(-2,3)
         if (max(abs(dr),abs(dc))==1 and (dr or dc)) or sorted([abs(dr),abs(dc)])==[1,2]]
VECTORS.sort(key=lambda v:math.atan2(-v[0],v[1]));VI={v:i for i,v in enumerate(VECTORS)}
ANGLES=np.array([math.atan2(-r,c) for r,c in VECTORS]);LENGTHS=np.array([math.hypot(r,c)*.15 for r,c in VECTORS])

def turn_curvature(a,b):
    angle=abs(math.atan2(math.sin(ANGLES[b]-ANGLES[a]),math.cos(ANGLES[b]-ANGLES[a])))
    return 2*math.sin(angle/2)/((LENGTHS[a]+LENGTHS[b])/2),angle


def turn_reference(c,query,*,radius=.8,budget_s=3.,max_states=160000):
    t=time.monotonic();state=c.l2.state;geo=state.geometry
    rc=np.asarray([geo.global_cell(i) for i in range(geo.state_count)],np.int32)
    xy=c.view.world_points(rc[:,0],rc[:,1]);allowed=cKDTree(c.reference_plan.world_xy).query(xy)[0]<=radius
    allowed[state.start_id]=allowed[state.goal_id]=True
    head=int(np.argmin(abs(np.arctan2(np.sin(ANGLES-query.start[2]),np.cos(ANGLES-query.start[2])))))
    goalhead=int(np.argmin(abs(np.arctan2(np.sin(ANGLES-query.goal[2]),np.cos(ANGLES-query.goal[2])))))
    legal=np.zeros((16,16),bool)
    for i in range(16):
        for j in range(16):
            k,a=turn_curvature(i,j);legal[i,j]=k<=2.5+1e-9 and a<=math.pi/4+1e-9
    base_cache={};edges_cache={}
    def base_edges(u):
        if u not in base_cache:
            result={}
            for v in geo.neighbor_ids(u):
                if not allowed[v]:continue
                cost=state._edge_cost(u,v)
                if not math.isfinite(cost):continue
                d=tuple(map(int,rc[v]-rc[u]));result.setdefault(d,[]).append((v,cost))
            base_cache[u]=result
        return base_cache[u]
    def edges(u):
        if u in edges_cache:return edges_cache[u]
        first=base_edges(u);result=[(v,VI[d],cost) for d,values in first.items() for v,cost in values]
        for direction,(dr,dc) in enumerate(VECTORS):
            if max(abs(dr),abs(dc))<2:continue
            cardinal=(int(np.sign(dr)),0) if abs(dr)==2 else (0,int(np.sign(dc)))
            diagonal=(int(np.sign(dr)),int(np.sign(dc)))
            forward={};other={}
            for a,ca in first.get(cardinal,[]):
                for v,cb in base_edges(a).get(diagonal,[]):forward[v]=min(forward.get(v,math.inf),ca+cb)
            for b,cb in first.get(diagonal,[]):
                for v,ca in base_edges(b).get(cardinal,[]):other[v]=min(other.get(v,math.inf),ca+cb)
            for v in forward.keys()&other.keys():
                result.append((v,direction,(forward[v]+other[v])*.5*math.sqrt(5)/(1+math.sqrt(2))))
        edges_cache[u]=result;return result
    start=state.start_id*16+head;g={start:0.};parent={};heap=[(state._heuristic(state.start_id,state.goal_id),0.,start)];found=None;expanded=0
    while heap:
        if expanded%128==0 and (time.monotonic()-t>budget_s or len(g)>=max_states):break
        _,cost,key=heapq.heappop(heap)
        if cost!=g[key]:continue
        u,heading=divmod(key,16);expanded+=1
        if u==state.goal_id and turn_curvature(heading,goalhead)[1]<=math.pi/4+1e-9:found=key;break
        for v,d,ec in edges(u):
            if not legal[heading,d]:continue
            angle=turn_curvature(heading,d)[1];nk=v*16+d;ng=cost+ec+.5*angle
            if ng<g.get(nk,math.inf):
                g[nk]=ng;parent[nk]=key;heapq.heappush(heap,(ng+1.5*state._heuristic(v,state.goal_id),ng,nk))
    log={'backend':'DStar_reference_16_direction_turn_lattice','success':found is not None,'direction_count':16,
         'max_grid_turn_deg':45.,'reference_curvature_limit_1pm':2.5,'fixed_straight_step_constraint':False,
         'expanded':expanded,'states':len(g),'wall_ms':(time.monotonic()-t)*1000,'scope_radius_m':radius,
         'same_obstacle_rings':True,'same_soft_edge_costs':True,'chords_require_both_covering_walks':True,
         'not_vehicle_certificate':True,'failure':'' if found is not None else ('L2_TURN_BUDGET' if heap else 'L2_TURN_NO_REFERENCE')}
    if found is None:return None,log
    keys=[found]
    while keys[-1]!=start:keys.append(parent[keys[-1]])
    keys.reverse();nodes=np.asarray(keys)//16;ref=xy[nodes]
    h=np.arctan2(np.diff(ref[:,1]),np.diff(ref[:,0]));traffic=c.traffic.audit(np.column_stack((ref,np.r_[h,h[-1]])))
    if not traffic['valid']:return None,{**log,'success':False,'failure':'L2_TURN_TRAFFIC','traffic':traffic}
    bends=[turn_curvature(a%16,b%16) for a,b in zip(keys[1:],keys[2:])]
    log.update(path_nodes=len(ref),reference_length_m=float(np.linalg.norm(np.diff(ref,axis=0),axis=1).sum()),
               actual_max_turn_deg=math.degrees(max([a for k,a in bends],default=0.)),actual_max_discrete_curvature_1pm=max([k for k,a in bends],default=0.))
    return ref,log
