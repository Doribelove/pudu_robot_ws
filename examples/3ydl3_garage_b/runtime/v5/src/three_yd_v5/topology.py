"""Position-only semantic routing; public output is a polyline and region list.

Region contact geometry is used only to establish adjacency and route length.
No directed outline graph, gate poses, vehicle trajectories, or obstacles enter
the route search. Fixed parking ownership and explicit traffic rules survive.
"""
import heapq
import itertools
import json
import math
import time
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
from scipy import ndimage
from arena_evaluation.semantic_map import canonical_hash
from arena_evaluation.semantic_rasterizer import rasterize_feature
from arena_3d_v1.multires_topology import TopologyRoute
from arena_3d_v1.semantic_world import json_write
from three_yd_v4.r5_guidance import cross_section
from three_yd_v4.r5_topology import R5Topology


def right_line(poly, direction, a, b):
    """Geometry only; no vehicle-dependent offset from the semantic boundary."""
    d = np.asarray(direction, float)
    lo, hi = sorted([float(np.asarray(a) @ d), float(np.asarray(b) @ d)])
    samples = np.linspace(lo, hi, max(2, int((hi-lo)/.2)+1))
    xy = samples[:, None]*d
    _, _, right = cross_section(poly, d, xy)
    xy += right[:, None]*np.array([d[1], -d[0]])
    xy = xy[np.all(np.isfinite(xy), axis=1)]
    if len(xy) < 2:
        raise ValueError('L1_RIGHT_BOUNDARY_UNDEFINED')
    return cv2.approxPolyDP(xy.astype(np.float32)[:, None, :], .04, False)[:, 0, :].astype(float)


def _line_intersection(p, u, q, v):
    """Intersection of two directed support lines, or None when parallel."""
    cross=float(u[0]*v[1]-u[1]*v[0])
    if abs(cross)<1e-6:return None
    delta=np.asarray(q,float)-np.asarray(p,float)
    t=float((delta[0]*v[1]-delta[1]*v[0])/cross)
    return np.asarray(p,float)+t*np.asarray(u,float)


def join_selected_road_lines(entries, topology, max_trim_m=12.):
    """Join only the chosen incoming/outgoing road boundaries at a junction.

    Each entry is ``(road_id, line, junction_after)``.  The complete topology
    retains all legal successors; this function compiles one selected route
    into one unbranched geometry.  Intersecting the terminal tangents trims
    the portal-end stubs that formerly produced triangular backtracking.
    """
    if not entries:return np.empty((0,2),float),[]
    lines=[np.asarray(item[1],float).copy() for item in entries]
    joins=[]
    for index in range(len(lines)-1):
        incoming,outgoing=lines[index],lines[index+1]
        junction_id=entries[index][2]
        accepted=False;point=None;reason='parallel_or_missing_junction'
        if junction_id is not None and len(incoming)>=2 and len(outgoing)>=2:
            u=incoming[-1]-incoming[-2];v=outgoing[1]-outgoing[0]
            nu=float(np.linalg.norm(u));nv=float(np.linalg.norm(v))
            if nu>1e-8 and nv>1e-8:
                point=_line_intersection(incoming[-1],u/nu,outgoing[0],v/nv)
            if point is not None:
                distances=(float(np.linalg.norm(point-incoming[-1])),float(np.linalg.norm(point-outgoing[0])))
                region=topology.regions.get(int(junction_id))
                polygon=None if region is None else np.asarray(
                    topology.router.features[region['name']]['coordinates'],np.float32)
                inside=polygon is not None and cv2.pointPolygonTest(polygon,tuple(map(float,point)),True)>=-.35
                accepted=inside and max(distances)<=max_trim_m
                reason='tangent_intersection' if accepted else 'intersection_outside_selected_junction'
                if accepted:
                    incoming[-1]=point;outgoing[0]=point
        joins.append({'junction_region_id':junction_id,'incoming_road_id':entries[index][0],
                      'outgoing_road_id':entries[index+1][0],
                      'selected_successor_only':True,'method':reason,
                      'point':None if point is None else point.tolist(),'accepted':accepted,
                      'legal_outgoing_count':0 if junction_id is None else max(0,len(topology.adj[int(junction_id)])-1)})
    points=[]
    for line in lines:
        for point in line:
            if not points or np.linalg.norm(np.asarray(points[-1])-point)>1e-7:points.append(point.tolist())
    return np.asarray(points,float),joins


class V5Topology:
    SCHEMA = '3YD-V5-r8-selected-junction-join-v1'
    endpoint_options = R5Topology.endpoint_options
    _allowed_direction = R5Topology._allowed_direction

    def __init__(self, world, graph_path, cache_root):
        tick = time.monotonic()
        self.world = world
        self.graph = json.loads(Path(graph_path).read_text())
        self.source_hash = canonical_hash(self.graph)
        semantic = json.loads((Path(graph_path).parent/'semantic.json').read_text())
        self.access = json.loads((Path(graph_path).parent/'parking_access.json').read_text())
        self.regions = {r['id']: r for r in self.graph['regions']}
        features = {f['properties']['source_name']: f for f in semantic['features']}
        self.parent_semantics = {rid: next(k for k, f in world.features.items() if f.properties.get('source_name') == r['name']) for rid, r in self.regions.items()}
        self.subregions = np.zeros(world.labels.shape, np.int16)
        def area(rid):
            p = np.asarray(features[self.regions[rid]['name']]['coordinates'])
            return abs(float(np.sum(p[:, 0]*np.roll(p[:, 1], -1)-p[:, 1]*np.roll(p[:, 0], -1))))
        for rid in sorted(self.regions, key=lambda r: ({'parking_area':10,'lane':20,'junction_area':30}[self.regions[r]['kind']], -area(r), r)):
            self.subregions[rasterize_feature(world.semantic, world.features[self.parent_semantics[rid]])] = rid
        axes, polygons = {}, {}
        for rid, region in self.regions.items():
            if region['kind'] != 'lane':
                continue
            p = np.asarray(features[region['name']]['coordinates'], float)
            polygons[rid] = p
            box = cv2.boxPoints(cv2.minAreaRect(p.astype(np.float32)))
            v = np.roll(box, -1, axis=0)-box
            lengths = np.linalg.norm(v, axis=1)
            axis = v[np.argmax(lengths)]/lengths.max()
            if axis[np.argmax(abs(axis))] < 0: axis = -axis
            axes[rid] = axis.astype(float)
        self.router = SimpleNamespace(axes=axes, polygons=polygons, features=features, rules=semantic.get('traffic_rules', {}))
        self.key = canonical_hash([self.SCHEMA, self.source_hash, semantic, self.access])
        self.router.key = self.key
        self.contacts = defaultdict(list)
        for edge in self.graph['edges']:
            if edge['kind'] == 'shared_outline' and len(edge['regions']) == 2:
                a, b = edge['regions']
                self.contacts[a,b].extend(edge['geometry']); self.contacts[b,a].extend(edge['geometry'])
        for rid, region in self.regions.items():
            if region['kind'] != 'parking_area': continue
            boundary = ndimage.binary_dilation(self.subregions == rid) & (self.subregions != rid)
            for other in np.unique(self.subregions[boundary]):
                other = int(other)
                if other not in self.regions or self.regions[other]['kind'] != 'lane': continue
                rc = np.argwhere(boundary & (self.subregions == other))
                xy = world.world_points(rc[:,0], rc[:,1]).tolist()
                self.contacts[rid,other].extend(xy); self.contacts[other,rid].extend(xy)
        self.adj = defaultdict(set)
        for a,b in self.contacts: self.adj[a].add(b)
        for a in list(self.adj):
            for b in list(self.adj[a]):
                if self.regions[a]['kind'] == self.regions[b]['kind'] == 'lane':
                    if any(self.regions[j]['kind'] == 'junction_area' for j in self.adj[a]&self.adj[b]): self.adj[a].discard(b)
        self.static_hard = world.hard
        self.tasks, self.contexts, self._cache = {}, {}, {}
        self.edges_count = sum(map(len, self.adj.values()))//2
        self.portal_count = 0
        self.preparation_ms = (time.monotonic()-tick)*1000
        json_write(Path(cache_root)/'v5_topology.json', {'schema':self.SCHEMA, 'regions':len(self.regions), 'adjacency_pairs':self.edges_count, 'gates':0, 'directed_outline_edges':0, 'obstacles_used_for_l1':False})

    def routes(self, query, limit=1):
        key = (query.query_id, tuple(query.start[:2]), tuple(query.goal[:2]))
        if key in self._cache: return self._cache[key][:limit]
        heap, found, seen = [], [], set()
        serial = itertools.count()
        for sr, road, binding in self.endpoint_options(query, 'start'):
            p = np.asarray(query.start[:2])
            if sr != road:
                contact = np.asarray(self.contacts[sr,road]); p = contact[np.argmin(np.linalg.norm(contact-p, axis=1))]
            for gr, groad, goal_binding in self.endpoint_options(query, 'goal'):
                heapq.heappush(heap, (0., next(serial), (road,), p, [], (sr,road,binding,gr,groad,goal_binding)))
        expansions = 0
        while heap and len(found) < limit and expansions < 10000:
            cost, _, seq, a, pieces, ends = heapq.heappop(heap); expansions += 1
            sr, sroad, sb, gr, groad, gb = ends
            rid = seq[-1]
            for nxt in ([None] if rid == groad else [])+sorted(self.adj[rid]):
                terminal = nxt is None
                if not terminal and (nxt in seq or self.regions[nxt]['kind'] == 'parking_area'): continue
                if terminal:
                    b = np.asarray(query.goal[:2])
                    if gr != groad:
                        contact = np.asarray(self.contacts[rid,gr]); b = contact[np.argmin(np.linalg.norm(contact-b, axis=1))]
                else: b = np.asarray(self.contacts[rid,nxt]).mean(axis=0)
                d = None
                if self.regions[rid]['kind'] == 'lane':
                    axis = self.router.axes[rid]; advance = float((b-a) @ axis)
                    if abs(advance) < .01: continue
                    d = axis*(1 if advance > 0 else -1)
                    if not self._allowed_direction(rid,d,sb if rid == sroad and len(seq)==1 else {}): continue
                    if terminal and not self._allowed_direction(rid,d,gb): continue
                if len(seq)>1 and self.regions[rid]['kind']=='junction_area' and nxt is not None:
                    sem = lambda x: self.router.features[self.regions[x]['name']]['semantic_id']
                    if [sem(seq[-2]),sem(rid),sem(nxt)] in self.router.rules.get('forbidden_transitions',[]): continue
                item = (rid, a.copy(), b.copy(), d)
                updated = pieces+[item]
                newcost = cost+float(np.linalg.norm(b-a))
                if not terminal:
                    heapq.heappush(heap, (newcost,next(serial),seq+(nxt,),b,updated,ends)); continue
                if seq in seen: continue
                seen.add(seq)
                roads=[];road_entries=[]
                for item_index,(road_id,begin,end,direction) in enumerate(updated):
                    if direction is None:continue
                    line=right_line(self.router.polygons[road_id],direction,begin,end)
                    junction_after=None
                    for next_id,_,_,next_direction in updated[item_index+1:]:
                        if next_direction is not None:break
                        if self.regions[next_id]['kind']=='junction_area':junction_after=next_id;break
                    road_entries.append((road_id,line,junction_after));roads.append((road_id,direction))
                points,junction_joins=join_selected_road_lines(road_entries,self)
                if len(points)<2:continue
                # Insert the semantic junctions crossed along a long road, but
                # emit no crossing position or mandatory portal to downstream.
                regions = [sr] if sr!=sroad else []
                expanded_roads = []
                for road_id, begin, end, direction in updated:
                    regions.append(road_id)
                    if direction is None: continue
                    expanded_roads.append((road_id,direction))
                    crossed=[]
                    for j in self.adj[road_id]:
                        if self.regions[j]['kind']!='junction_area' or j in seq: continue
                        t=float(np.mean(self.contacts[road_id,j],axis=0)@direction)
                        if begin@direction+.3<t<end@direction-.3: crossed.append((t,j))
                    for _,j in sorted(crossed): regions.extend([j,road_id]); expanded_roads.append((road_id,direction))
                if gr!=groad: regions.append(gr)
                task={'schema':self.SCHEMA,'right_reference_polyline':points.tolist(),'region_sequence':regions,
                      'start_xy':list(query.start[:2]),'goal_xy':list(query.goal[:2]),'frame_id':'map',
                      'map_hash':self.world.map.sha256,'rules_hash':self.router.key,
                      'junction_joins':junction_joins,'selected_route_has_branches':False}
                signature=canonical_hash(task); task['signature']=signature
                self.tasks[signature]=task
                endpoint_access={}
                for role,r,road in [('start',sr,sroad),('goal',gr,groad)]:
                    binding=next((x for x in self.access.get('bindings',[])
                        if x.get('query_id')==query.query_id and x.get('endpoint')==role),{})
                    endpoint_access[role]={'semantic_region_id':r,'road_id':road,
                        'allow_local_cross_road':bool(binding.get('allow_local_cross_road',False)),
                        'parking_binding_source':'confirmed parking_access.json' if binding else 'semantic endpoint'}
                self.contexts[signature]={'roads':expanded_roads,'unique_roads':roads,'endpoint_access':endpoint_access}
                found.append(TopologyRoute(tuple(regions),(),tuple(self.parent_semantics[x] for x in regions),tuple(query.start),tuple(query.goal),signature,{'l1_expanded':expansions,'l1_path_cost_m':newcost},{'world_key':self.world.key,'topology_key':self.key,**self.world.binding}))
        self._cache[key]=found
        return found

    def plan(self, query, **kwargs):
        routes=self.routes(query)
        if not routes: raise RuntimeError('NO_LEGAL_SEMANTIC_ROUTE')
        return routes[0]

    def task(self,route): return self.tasks[route.signature]
    def channel_plan(self,route): return self.task(route)
