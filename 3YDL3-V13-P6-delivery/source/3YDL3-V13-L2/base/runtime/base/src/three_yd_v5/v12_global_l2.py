"""Full-map 2D search after L1 rejection; L3 remains a bounded strict solver."""
import copy
import ctypes
import math
from pathlib import Path
import time
from types import SimpleNamespace
import numpy as np

from arena_evaluation.semantic_map import canonical_hash
from arena_3d_v1.multires_topology import TopologyRoute
from .free_space_topology import FreeSpaceTopology

_LIB=None


class FullGridFailure(RuntimeError):
    def __init__(self,trials):
        self.trials=trials
        super().__init__('V12_FULL_L2_NO_PATH')


def grid_search(free,start,goal,*,seconds,max_visited=1000000,potential=None,weight=2.):
    global _LIB
    if _LIB is None:
        _LIB=ctypes.CDLL(str(Path(__file__).resolve().parents[2]/'native/libv12_grid_astar.so'))
        _LIB.v12_grid_astar.argtypes=[ctypes.c_int32,ctypes.c_int32,
            np.ctypeslib.ndpointer(dtype=np.uint8,flags='C_CONTIGUOUS'),ctypes.POINTER(ctypes.c_float),
            ctypes.c_double,ctypes.c_int32,ctypes.c_int32,ctypes.c_double,ctypes.c_uint64,
            np.ctypeslib.ndpointer(dtype=np.int32,flags='C_CONTIGUOUS'),ctypes.c_uint64,
            np.ctypeslib.ndpointer(dtype=np.uint64,flags='C_CONTIGUOUS')]
        _LIB.v12_grid_astar.restype=ctypes.c_int
    free=np.ascontiguousarray(free,np.uint8);h,w=free.shape
    if any(r<0 or c<0 or r>=h or c>=w for r,c in (start,goal)):
        return None,{'reason':'ENDPOINT_OUTSIDE','expanded':0,'visited':0}
    field=None if potential is None else np.ascontiguousarray(potential,np.float32)
    if field is not None and (field.shape!=free.shape or not np.isfinite(field).all() or np.any(field<0)):
        raise ValueError('V12_L2_POTENTIAL_BINDING')
    path=np.empty(max_visited,np.int32);stats=np.zeros(3,np.uint64)
    status=_LIB.v12_grid_astar(h,w,free,None if field is None else field.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
        weight,start[0]*w+start[1],goal[0]*w+goal[1],seconds,max_visited,path,len(path),stats)
    reason={0:'SUCCESS',1:'DISCONNECTED',2:'LOCAL_TIME_BUDGET',3:'VISITED_STATE_BUDGET',4:'INVALID_INPUT',5:'ALLOCATION_FAILURE'}[status]
    if status==4:raise ValueError('V12_L2_INVALID_NATIVE_INPUT')
    count=int(stats[2]);cells=None if status else np.column_stack((path[:count]//w,path[:count]%w))
    return cells,{'reason':reason,'expanded':int(stats[0]),'visited':int(stats[1]),'path_cells':count,
                  'search_dimensions':['x','y'],'full_grid_shape':list(free.shape)}


class GridTopology:
    """Own only the fallback route/task; keep original traffic catalogue."""
    def __init__(self,base,task,context,route):
        self.base=base;self.world=base.world;self.regions=base.regions
        self.key=canonical_hash([base.key,'V12-full-map-grid-fallback'])
        self.tasks={**base.tasks,route.signature:task}
        self.contexts={**base.contexts,route.signature:context}
        self.route=route;self.mode=getattr(base,'mode',None)
        self._probe_stats={'L1_search_calls':0,'source':'full_map_2d_L2',**route.diagnostics}
    def __getattr__(self,name):return getattr(self.base,name)
    def routes(self,query,limit=1):
        if tuple(query.start)!=self.route.start or tuple(query.goal)!=self.route.goal:
            raise ValueError('V12_L2_ROUTE_ENDPOINT_BINDING')
        return [self.route] if limit else []
    def task(self,route):return self.tasks[route.signature]
    channel_plan=task
    def scope_mask(self,route):
        from .route_quality import scope_mask
        task=self.task(route)
        return scope_mask(self.world,np.asarray(task['right_reference_polyline']),task['scope_half_width_m'])


def _traffic_context(base,query,xy):
    if getattr(base,'mode',None)=='free_space':
        return [1],{'roads':[],'unique_roads':[],'endpoint_access':{
            role:{'semantic_region_id':1,'road_id':1,'allow_local_cross_road':False}
            for role in ('start','goal')}}
    access={}
    for role in ('start','goal'):
        options=base.endpoint_options(query,role)
        if not options:raise RuntimeError('V12_L2_NO_ENDPOINT_TRAFFIC_BINDING')
        rid,road,binding=options[0]
        access[role]={'semantic_region_id':int(rid),'road_id':int(road),
            'allowed_road_ids':[int(x[1]) for x in options],
            'travel_sides':binding.get('travel_sides',[]),
            'allow_local_cross_road':bool(binding.get('allow_local_cross_road',False))}
    # Traffic auditing uses25mm samples; use that same spatial convention
    # instead of mistaking isolated150mm grid centers for entire road visits.
    dense=[]
    for a,b in zip(xy[:-1],xy[1:]):
        n=max(2,int(math.ceil(np.linalg.norm(b-a)/.025))+1)
        dense.extend(a+(b-a)*np.linspace(0.,1.,n)[:-1,None])
    xy=np.vstack([np.asarray(dense),xy[-1]])
    rr,cc=base.world.cells(xy);labels=np.asarray(base.subregions[rr,cc],int)
    raw_ids=labels[np.r_[True,labels[1:]!=labels[:-1]]].tolist()
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
    run_starts=np.flatnonzero(np.r_[True,labels[1:]!=labels[:-1]])
    run_ends=np.r_[run_starts[1:],len(labels)]
    major={int(labels[a]) for a,b in zip(run_starts,run_ends)
           if arc[min(b,len(arc)-1)]-arc[a]>.15+1e-9
           and base.regions.get(int(labels[a]),{}).get('kind')=='lane'}
    for a,b in zip(run_starts,run_ends):
        rid=int(labels[a])
        if rid in major or base.regions.get(rid,{}).get('kind')!='lane':continue
        if arc[min(b,len(arc)-1)]-arc[a]>.15+1e-9:continue
        candidates=None
        for i in range(a,b):
            near=set(map(int,np.unique(base.subregions[max(0,rr[i]-1):rr[i]+2,max(0,cc[i]-1):cc[i]+2])))&major
            candidates=near if candidates is None else candidates&near
            if not candidates:break
        if candidates:
            neighbors=[int(labels[i]) for i in (a-1,b) if 0<=i<len(labels) and int(labels[i]) in candidates]
            labels[a:b]=neighbors[0] if neighbors else min(candidates)
    starts=np.flatnonzero(np.r_[True,labels[1:]!=labels[:-1]]);ends=np.r_[starts[1:],len(labels)]
    ids=labels[starts].tolist();roads=[]
    for lo,hi in zip(starts,ends):
        rid=int(labels[lo]);kind=base.regions.get(rid,{}).get('kind')
        if kind=='parking_area' and rid not in {a['semantic_region_id'] for a in access.values()}:
            raise RuntimeError('UNAUTHORIZED_PARKING_TRANSIT')
        if kind!='lane':continue
        axis=np.asarray(base.router.axes[rid],float)
        a=xy[max(0,lo-1)];b=xy[min(len(xy)-1,hi)]
        advance=float((b-a)@axis)
        if abs(advance)<1e-8:raise RuntimeError('V12_L2_AMBIGUOUS_ROAD_DIRECTION')
        direction=axis*(1 if advance>0 else -1)
        if not base._allowed_direction(rid,direction,{}):raise RuntimeError('WRONG_TRAVEL_DIRECTION')
        roads.append((rid,direction))
    for role,ordered in (('start',roads),('goal',roads[::-1])):
        item=access[role]
        if base.regions[item['semantic_region_id']]['kind']=='parking_area':
            if not ordered or ordered[0][0] not in item['allowed_road_ids']:
                raise RuntimeError('PARKING_OWNED_ROAD_ACCESS')
            item['road_id']=ordered[0][0]
            if not base._allowed_direction(ordered[0][0],ordered[0][1],item):
                raise RuntimeError('PARKING_ENDPOINT_TRAVEL_SIDE')
    unique=[];seen=set()
    for rid,d in roads:
        key=(rid,tuple(d))
        if key not in seen:unique.append((rid,d));seen.add(key)
    return ids,{'roads':roads,'unique_roads':unique,'endpoint_access':access,
                'raw_grid_region_sequence':raw_ids,
                'micro_boundary_policy':'same25mm sampling and <=.15m adjacent-pixel rule as unchanged V5Traffic final audit'}


def bind_reference_traffic(controller,xy):
    if not controller.task.get('v12_full_l2_grid_reference'):return
    from .controller import V5Traffic
    base=controller.topology.base
    query=SimpleNamespace(query_id=controller.task['query_id'],
        start=controller.topology_route.start,goal=controller.topology_route.goal)
    controller.timing['v12_grid_reference_xy']=np.asarray(xy).tolist()
    _,context=_traffic_context(base,query,np.asarray(xy,float))
    controller.topology.contexts[controller.topology_route.signature]=context
    if not controller.generic_mode:
        controller.traffic=V5Traffic(controller.world,controller.topology,controller.topology_route)
    controller.timing['v12_L2_selected_traffic']={
        'roads':[[rid,np.asarray(direction).tolist()] for rid,direction in context['roads']],
        'bound_after_actual_L2':True,'unchanged_final_traffic_audit':True}


def topology_from_grid(base,query,session,*,local_budget_s=5.,dynamic=None):
    """Attempt conservative15cm, then original5cm if aggregation rejects it."""
    world=base.world;state=session._curve_request;start=time.monotonic()
    deadline=min(state['deadline'],start+local_budget_s)
    hard=world.hard
    if dynamic is not None:hard=hard|dynamic.mask
    if getattr(base,'mode',None)=='free_space':allowed=~hard
    else:
        from .v9_r9_membership import semantic_membership
        endpoint_ids=set()
        for role in ('start','goal'):
            endpoint_ids.update(int(x[0]) for x in base.endpoint_options(query,role))
        ids=[rid for rid,r in base.regions.items() if r['kind'] in ('lane','junction_area') or rid in endpoint_ids]
        allowed=semantic_membership(base.subregions,ids)&~hard
    trials=[];solution=None;factor=None
    for factor in (3,1):
        remaining=deadline-time.monotonic()
        if remaining<=0:break
        free=FreeSpaceTopology._coarsen(allowed,factor)
        h,w=free.shape
        def cell(p):
            rr,cc=world.cells(np.asarray(p[:2])[None,:]);return int(rr[0]//factor),int(cc[0]//factor)
        from .v12_regional import current,apply_cost
        field=None
        if current().active:
            c=SimpleNamespace(world=world,topology=base,v12_policies=current())
            view=SimpleNamespace(world_points=lambda r,c:world.world_points(np.asarray(r)*factor+(factor-1)/2,np.asarray(c)*factor+(factor-1)/2))
            field=apply_cost(c,view,(0,h,0,w),np.zeros(free.shape,np.float32))
        cells,detail=grid_search(free,cell(query.start),cell(query.goal),seconds=max(.001,deadline-time.monotonic()),potential=field)
        trials.append({'factor':factor,'resolution_m':world.map.resolution*factor,**detail})
        if cells is not None:
            solution=world.world_points(cells[:,0]*factor+(factor-1)/2,cells[:,1]*factor+(factor-1)/2)
            break
        del free,field
    if solution is None:raise FullGridFailure(trials)
    if len(solution)<2:raise RuntimeError('V12_FULL_L2_DEGENERATE_REFERENCE')
    solution[0]=query.start[:2];solution[-1]=query.goal[:2]
    ids,context=_traffic_context(base,query,solution)
    # Preserve corner changes, not every collinear grid center.
    delta=np.diff(solution,axis=0);norm=np.linalg.norm(delta,axis=1)
    direction=delta/np.maximum(norm[:,None],1e-12)
    keep=np.r_[True,np.linalg.norm(np.diff(direction,axis=0),axis=1)>1e-7,True]
    line=solution[keep]
    task={'schema':'V12-full-map-2d-L2-reference-v1','source':'full_map_2d_L2',
        'query_id':query.query_id,
        'v12_full_l2_grid_reference':True,'right_reference_polyline':line.tolist(),
        'region_sequence':ids,'start_xy':list(query.start[:2]),'goal_xy':list(query.goal[:2]),
        'frame_id':'map','map_hash':world.map.sha256,'rules_hash':getattr(getattr(base,'router',None),'key',None),
        'scope_half_width_m':2.,'selected_route_has_branches':False,
        'L1_search_calls':0,'full_map_L2_trials':trials}
    signature=canonical_hash(task);task['signature']=signature
    key=canonical_hash([base.key,'V12-full-map-grid-fallback'])
    route=TopologyRoute(tuple(ids),(),(),tuple(query.start),tuple(query.goal),signature,
        {'source':'full_map_2d_L2','l1_expanded':0,'expanded':sum(t['expanded'] for t in trials),'trials':trials},
        {'world_key':world.key,'topology_key':key,**world.binding})
    wrapped=GridTopology(base,task,context,route)
    if getattr(base,'mode',None)!='free_space':
        from .controller import V5Traffic
        heading=np.arctan2(np.gradient(solution[:,1]),np.gradient(solution[:,0]))
        traffic=V5Traffic(world,wrapped,route).audit(np.column_stack((solution,heading)))
        if not traffic['valid']:raise RuntimeError('V12_FULL_L2_TRAFFIC_REJECT:'+str(traffic.get('failures')))
    return wrapped,{'wall_ms':(time.monotonic()-start)*1000,'trials':trials,
                    'selected_roads':[[rid,np.asarray(d).tolist()] for rid,d in context['roads']],
                    'region_sequence':ids,'guide_xy':line.tolist(),
                    'original_obstacle_and_traffic_contract_retained':True,'full_map_Smac':False}
