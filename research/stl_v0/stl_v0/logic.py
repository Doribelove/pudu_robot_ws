"""Finite ordered-eventually STL fragment, with explicit witness times.

Only this documented fragment is supported, not a general STL compiler.
A graph state is (convex-region index, completed ordered obligations).
"""
import heapq
import itertools
import time
import numpy as np


def overlap(a,b):
    lo=np.maximum(a.bounds[:2],b.bounds[:2])
    hi=np.minimum(a.bounds[2:],b.bounds[2:])
    if not np.all(hi-lo>1e-6):return None
    if a.halfspaces or b.halfspaces:
        from .semantic import clip_polygon
        poly=clip_polygon([[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]],a.halfspaces+b.halfspaces)
        if not poly:return None
        pts=np.asarray(poly);area=abs(np.sum(pts[:,0]*np.roll(pts[:,1],-1)-pts[:,1]*np.roll(pts[:,0],-1)))/2
        if area<1e-10:return None
        lo=pts.min(axis=0);hi=pts.max(axis=0)
        if not np.all(hi-lo>1e-6):return None
    return np.r_[lo,hi]


def advance(task, region, progress):
    while progress<len(task.visits) and task.visits[progress]['label'] in region.labels:
        progress+=1
    return progress


def candidate_routes(scene, limit=8, expansion_limit=30000, deadline=float("inf"), min_region_width=0., required_label=None, width_weight=1., geometry=None):
    started=time.monotonic();regions=scene.regions
    from .geometry_graph import prepare_geometry
    if geometry is None:
        try:geometry,build_info=prepare_geometry(scene,deadline)
        except TimeoutError:return [],{'status':'GRAPH_BUDGET_EXHAUSTED','expanded':0}
    else:build_info={'cache_hit':True,'wall_s':0.,'shared_within_request':True}
    allowed=[i for i,r in enumerate(regions) if not (r.labels & scene.task.avoid_labels) and geometry['region_widths'][i]>=min_region_width and (required_label is None or required_label in r.labels)]
    starts=[i for i in allowed if regions[i].contains(scene.start)]
    goals=set(i for i in allowed if regions[i].contains(scene.goal))
    if not starts or not goals:return [],{'status':'ENDPOINT_OUTSIDE_FEASIBLE_COVER','expanded':0,'wall_s':time.monotonic()-started}
    allowed_set=set(allowed);centers=geometry['centers'];source=(-1,0);sink=(-2,0);required=len(scene.task.visits)
    costs=geometry['distances']+.02+width_weight/(geometry['widths']+.05)
    graph={source:[],sink:[]};weights={};materialized_s=0.
    # Lazy product-state edges preserve ascending region order and the original
    # cost computation, but do not instantiate unreachable or unvisited states.
    def edges(node):
        nonlocal materialized_s
        if node not in graph:
            t=time.monotonic();i,k=node;row=[]
            for j,e in geometry['neighbors'][i]:
                if j not in allowed_set:continue
                nxt=(j,advance(scene.task,regions[j],k));cost=float(costs[e]);row.append((nxt,cost));weights[(node,nxt)]=cost
            if i in goals and k==required:
                cost=float(np.linalg.norm(centers[i]-scene.goal[:2]));row.append((sink,cost));weights[(node,sink)]=cost
            graph[node]=row;materialized_s+=time.monotonic()-t
        return graph[node]
    for i in starts:
        nxt=(i,advance(scene.task,regions[i],0));cost=float(np.linalg.norm(centers[i]-scene.start[:2]));graph[source].append((nxt,cost));weights[(source,nxt)]=cost
    serial=itertools.count();expanded=0;budget_hit=False

    def shortest(start,removed_edges=frozenset(),removed_nodes=frozenset()):
        nonlocal expanded,budget_hit
        distance={start:0.};parent={};queue=[(0.,next(serial),start)]
        while queue:
            if expanded>=expansion_limit or time.monotonic()>deadline:
                budget_hit=True;return None
            cost,_,node=heapq.heappop(queue)
            if cost>distance[node]+1e-12:continue
            expanded+=1
            if node==sink:
                path=[node]
                while path[-1]!=start:path.append(parent[path[-1]])
                return path[::-1]
            for nxt,w in edges(node):
                if nxt in removed_nodes or (node,nxt) in removed_edges:continue
                value=cost+w
                if value<distance.get(nxt,float('inf'))-1e-12:
                    distance[nxt]=value;parent[nxt]=node
                    heapq.heappush(queue,(value,next(serial),nxt))
        return None

    # Yen's k shortest simple PRODUCT-state paths. Graph costs only enumerate
    # candidates: the selected answer is ranked by solved continuous cost.
    first=shortest(source);accepted=[] if first is None else [first]
    pending=[];known=set([tuple(first)]) if first else set()
    while accepted and len(accepted)<limit and not budget_hit:
        previous=accepted[-1]
        for i in range(len(previous)-1):
            prefix=previous[:i+1]
            removed={(path[i],path[i+1]) for path in accepted if len(path)>i+1 and path[:i+1]==prefix}
            spur=shortest(prefix[-1],removed,set(prefix[:-1]))
            if spur is None:
                if budget_hit:break
                continue
            path=prefix[:-1]+spur;key=tuple(path)
            if key not in known:
                cost=sum(weights[(u,v)] for u,v in zip(path[:-1],path[1:]))
                heapq.heappush(pending,(cost,next(serial),path));known.add(key)
        if not pending:break
        accepted.append(heapq.heappop(pending)[2])
    paths=[[i for i,k in path if i>=0] for path in accepted]
    return paths, {'status':'CANDIDATES_FOUND' if paths else ('GRAPH_BUDGET_EXHAUSTED' if budget_hit else 'NO_ROUTE_IN_FINITE_COVER'),
                   'expanded':expanded,'candidate_limit':limit,'required_label':required_label,'width_weight':width_weight,'minimum_region_width_m':min_region_width,'expansion_limit':expansion_limit,
                   'graph_budget_hit':budget_hit,'cover_regions':len(regions),
                   'geometry_build':build_info,'materialized_product_states':len(graph)-2,'product_materialization_s':materialized_s,
                   'wall_s':time.monotonic()-started,
                   'undirected_edges':sum(sum(j in allowed_set for j,e in geometry['neighbors'][i]) for i in allowed)//2,
                   'candidate_geometry_cost':'center distance plus width_weight/(minimum overlap width + 0.05m)',
                   'enumeration':'Yen simple product-state paths with Dijkstra spur searches',
                   'completeness_claim':False}


def witness_segments(scene, route):
    k=0;result=[]
    for seg,index in enumerate(route):
        nk=advance(scene.task,scene.regions[index],k)
        for j in range(k,nk):
            result.append((seg,scene.task.visits[j]))
        k=nk
    return result
