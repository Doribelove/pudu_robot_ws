"""Traffic-aware uses of the frozen outline graph. No vehicle/obstacle input.

Interval endpoints are indexes on existing polylines, never new map nodes.
Screen side describes attachment ownership; direction describes traffic.
"""
from dataclasses import dataclass, asdict, replace
from collections import defaultdict
import hashlib, heapq, itertools, json, math, time
import numpy as np


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False).encode()).hexdigest()


def inside(point, rings):
    x, y = point; result = False
    for ring in rings:
        ring = np.asarray(ring, float)
        a, b = ring, np.roll(ring, -1, axis=0)
        crossing = ((a[:, 1] > y) != (b[:, 1] > y))
        dx = (b[:, 0]-a[:, 0])*(y-a[:, 1])/np.where(abs(b[:, 1]-a[:, 1]) > 1e-12, b[:, 1]-a[:, 1], 1.) + a[:, 0]
        result ^= bool(np.count_nonzero(crossing & (x < dx)) % 2)
    return result


@dataclass(frozen=True)
class EdgeUse:
    use_id: str
    edge_id: int
    region_id: int
    kind: str
    screen_side: str
    s_from: float
    s_to: float
    geometry: tuple
    start_key: tuple
    end_key: tuple

    @property
    def length(self): return abs(self.s_to-self.s_from)
    @property
    def direction(self): return 1 if self.s_to > self.s_from else -1
    def as_dict(self):
        return {**asdict(self), 'direction': self.direction,
                'is_executable_trajectory': False}


class AttachmentError(ValueError):
    def __init__(self, reason, details):
        super().__init__(reason); self.details = details


class EdgeRouter:
    SCHEMA = '3YD-V3-directed-boundary-channel-v1'
    def __init__(self, graph, semantic, access=None):
        tick = time.monotonic()
        self.graph = graph; self.graph_hash = fingerprint(graph)
        self.regions = {int(r['id']): r for r in graph['regions']}
        self.edges = {int(e['id']): e for e in graph['edges']}
        self.rings = {int(o['id']): o['geometry'] for o in graph['outlines']}
        self.rules = semantic.get('traffic_rules', {})
        self.access = access or {'bindings': []}
        self.features = {f['properties']['source_name']: f for f in semantic['features']}
        self.axes = {}; self.polygons = {}; self.uses = {}; self.outgoing = defaultdict(list)
        self.excluded_segments = []; self.compile_warnings = []
        for rid, region in self.regions.items():
            if region['kind'] != 'lane': continue
            polygon = np.asarray(self.features[region['name']]['coordinates'], float)
            self.polygons[rid] = polygon
            # Stable fixed-view road axis. The original map has elongated roads;
            # ambiguous/square shapes must be annotated, not silently classified.
            import cv2
            box = cv2.boxPoints(cv2.minAreaRect(polygon.astype(np.float32)))
            vectors = np.roll(box, -1, axis=0)-box
            lens = np.linalg.norm(vectors, axis=1); axis = vectors[np.argmax(lens)]/lens.max()
            if axis[np.argmax(abs(axis))] < 0: axis = -axis
            self.axes[rid] = axis
        self._compile()
        for use in self.uses.values(): self.outgoing[use.start_key].append(use.use_id)
        self.key = fingerprint([self.SCHEMA, self.graph_hash, semantic, self.access])
        self.preparation_ms = (time.monotonic()-tick)*1000

    def side_at(self, rid, xy):
        """Cross-section of the ORIGINAL road, never global nearest-road search."""
        p = np.asarray(xy, float); axis = self.axes[rid]
        vertical = abs(axis[1]) >= abs(axis[0])
        normal = np.array([axis[1], -axis[0]]) if vertical else np.array([-axis[1], axis[0]])
        poly = self.polygons[rid]; values = []
        for a, b in zip(poly, np.roll(poly, -1, axis=0)):
            delta = b-a; denominator = float(delta @ axis)
            if abs(denominator) < 1e-10: continue
            t = float((p-a) @ axis)/denominator
            if -1e-8 <= t <= 1+1e-8: values.append(float((a+t*delta-p) @ normal))
        if len(values) < 2: raise AttachmentError('ROAD_SIDE_UNRESOLVED', {'road_id': rid, 'xy': list(xy)})
        center_offset = (min(values)+max(values))/2
        positive = center_offset <= 0
        return ('right' if positive else 'left') if vertical else ('top' if positive else 'bottom')

    def _compile(self):
        for edge in self.edges.values():
            pts = np.asarray(edge['geometry'], float); lengths = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            ss = np.r_[0., np.cumsum(lengths)]
            def key(s):
                if abs(s) < 1e-7: return ('node', int(edge['from']))
                if abs(s-ss[-1]) < 1e-7: return ('node', int(edge['to']))
                return ('edge_position', int(edge['id']), round(float(s), 7))
            for rid in edge['regions']:
                kind = self.regions[rid]['kind']
                if kind not in ('lane', 'junction_area'): continue
                segments = []
                for i, length in enumerate(lengths):
                    if length < 1e-8: continue
                    tangent = (pts[i+1]-pts[i])/length; mid = (pts[i]+pts[i+1])/2
                    if kind == 'lane':
                        # Transverse lane caps are entrance/exit boundaries, not
                        # an implicit U-turn across the two traffic sides.
                        alignment = abs(float(tangent @ self.axes[rid]))
                        if alignment < .5 and length >= 2.:
                            self.excluded_segments.append({'edge_id': edge['id'], 'region_id': rid,
                                's0': float(ss[i]), 's1': float(ss[i+1]), 'reason': 'transverse_cap_not_lane_travel'})
                            continue
                        normal = np.array([-tangent[1], tangent[0]])
                        left = right = False
                        for eps in (.02, .06, .12):
                            left, right = inside(mid+normal*eps, self.rings[rid]), inside(mid-normal*eps, self.rings[rid])
                            if left != right: break
                        if left == right:
                            self.compile_warnings.append({'edge_id': edge['id'], 'region_id': rid, 'segment': i, 'reason': 'interior_side_ambiguous'})
                            continue
                        try: side = self.side_at(rid, mid)
                        except AttachmentError:
                            self.compile_warnings.append({'edge_id': edge['id'], 'region_id': rid, 'segment': i, 'reason': 'side_unresolved'})
                            continue
                        directions = [1 if left else -1]
                        if not self.rules.get('right_hand_drive', True): directions = [-directions[0]]
                        feature = self.features[self.regions[rid]['name']]
                        for rule in self.rules.get('one_way', []):
                            if rule['semantic_id'] == feature['semantic_id']:
                                yaw = float(rule['yaw']); v = np.array([math.cos(yaw), math.sin(yaw)])
                                directions = [d for d in directions if float(d*tangent@v) >= -1e-6]
                    else:
                        side = 'junction'; directions = [1, -1]
                    for direction in directions: segments.append((i, side, direction))
                # Merge adjacent segments with the same semantic role. Logical
                # positions preserve original edge IDs and no map mutation.
                for direction in (1, -1):
                    groups = []
                    for i, side, d in sorted(segments):
                        if d != direction: continue
                        if groups and groups[-1][1] == i and groups[-1][2] == side: groups[-1][1] = i+1
                        else: groups.append([i, i+1, side])
                    for a, b, side in groups:
                        start, end = (float(ss[a]), float(ss[b])) if direction == 1 else (float(ss[b]), float(ss[a]))
                        geom = pts[a:b+1] if direction == 1 else pts[a:b+1][::-1]
                        uid = f"e{edge['id']}:r{rid}:{a}-{b}:d{direction}"
                        self.uses[uid] = EdgeUse(uid, int(edge['id']), rid, kind, side, start, end,
                            tuple(map(tuple, geom.tolist())), key(start), key(end))

    def _project(self, point, use):
        pts = np.asarray(use.geometry); point = np.asarray(point); a, delta = pts[:-1], np.diff(pts, axis=0)
        lens = np.linalg.norm(delta, axis=1); t = np.clip(np.sum((point-a)*delta, axis=1)/np.maximum(lens*lens, 1e-20), 0., 1.)
        points = a+t[:, None]*delta; distances = np.linalg.norm(points-point, axis=1); i = int(np.argmin(distances))
        offset = float(lens[:i].sum()+t[i]*lens[i]); s = use.s_from+use.direction*offset
        return {'use_id': use.use_id, 'edge_id': use.edge_id, 'road_id': use.region_id,
                'screen_side': use.screen_side, 'side_id': f'road_{use.region_id}_{use.screen_side}',
                's_m': s, 'xy': points[i].tolist(), 'distance_m': float(distances[i]),
                'allowed_direction': use.direction, 'tangent': (use.direction*0+delta[i]/max(lens[i], 1e-12)).tolist()}

    def attach(self, query_id, endpoint, pose, semantic_region):
        rid = int(semantic_region); kind = self.regions[rid]['kind']; source = 'point_in_original_road'
        options=[]
        if kind == 'parking_area':
            bindings = [b for b in self.access.get('bindings', []) if b.get('query_id') == query_id and b.get('endpoint') == endpoint
                        and b.get('parking_region_id') == rid]
            # Location bindings make later deployment independent of query IDs.
            bindings += [b for b in self.access.get('bindings', []) if 'polygon' in b and b.get('parking_region_id') == rid and inside(pose[:2], [b['polygon']])]
            if len(bindings)!=1: raise AttachmentError('PARKING_ROAD_OWNERSHIP_UNRESOLVED', {'query_id': query_id, 'endpoint': endpoint, 'parking_region_id': rid, 'bindings': bindings})
            binding=bindings[0]; source = 'explicit_local_parking_access'
            if 'anchor_xy' in binding and math.dist(pose[:2],binding['anchor_xy'])>binding.get('anchor_tolerance_m',.25):
                raise AttachmentError('PARKING_ACCESS_ANCHOR_MISMATCH',{'query_id':query_id,'endpoint':endpoint})
            roads=binding.get('allowed_road_ids',[binding.get('road_id')])
            for road in roads:
                if self.regions.get(road, {}).get('kind') != 'lane': raise AttachmentError('ACCESS_ROAD_INVALID', {'road_id': road})
                access_side=binding.get('screen_side') or self.side_at(road,pose[:2])
                travel_sides=binding.get('travel_sides')
                if not binding.get('allow_local_cross_road',False):travel_sides=[access_side]
                options.append((road,access_side,travel_sides,bool(binding.get('allow_local_cross_road',False))))
        elif kind == 'lane':
            # A point already in the road has road ownership. Its fixed heading
            # selects the legal traffic side; its physical position is preserved
            # for downstream lateral joining and full vehicle checks.
            options=[(rid,self.side_at(rid,pose[:2]),None,True)]
        else: raise AttachmentError('ENDPOINT_REQUIRES_ROAD_ACCESS_BINDING', {'region_id': rid, 'endpoint': endpoint})
        candidates=[]
        for road,access_side,travel_sides,allow_cross in options:
            owned=[u for u in self.uses.values() if u.kind=='lane' and u.region_id==road]
            access_choices=sorted([self._project(pose[:2],u) for u in owned if u.screen_side==access_side],key=lambda x:x['distance_m'])
            if not access_choices:continue
            access_projection=access_choices[0]
            choices=[]
            for use in owned:
                if travel_sides and use.screen_side not in travel_sides:continue
                a=self._project(pose[:2],use)
                if kind=='lane' and np.dot(a['tangent'],[math.cos(pose[2]),math.sin(pose[2])])<.5:continue
                if a['screen_side']!=access_side and not allow_cross:continue
                choices.append(a)
            # Retain the nearest local interval of each explicitly permitted
            # traffic side. This allows Q10's two authorized road alternatives.
            for side in sorted({a['screen_side'] for a in choices}):
                group=sorted([a for a in choices if a['screen_side']==side],key=lambda a:(a['distance_m'],a['edge_id'],a['s_m']))
                for a in [a for a in group if a['distance_m']<=group[0]['distance_m']+.05][:3]:
                    candidates.append({**a,'access_screen_side':access_side,'access_projection':access_projection,
                        'local_cross_road_allowed':allow_cross,'local_side_transfer_required':a['screen_side']!=access_side,
                        'side_selection':'explicit_parking_access_then_permitted_traffic_side' if kind=='parking_area' else 'owned_road_and_fixed_endpoint_heading'})
        if not candidates:raise AttachmentError('NO_LEGAL_OWNED_SIDE_EDGE',{'options':options})
        return [{**a, 'semantic_region_id': rid, 'ownership_source': source,
                 'view_frame_id': self.access.get('view_frame_id', 'map'),
                 'endpoint': endpoint, 'original_pose': list(pose), 'query_only': True} for a in candidates]

    def transition(self, a, b, last_road):
        if a.end_key != b.start_key: return False
        if a.edge_id == b.edge_id and a.region_id == b.region_id and a.direction != b.direction: return False
        if a.kind == b.kind == 'lane':
            return a.region_id == b.region_id and a.screen_side == b.screen_side
        if a.kind == b.kind == 'junction_area': return a.region_id == b.region_id
        if a.kind == 'junction_area' and b.kind == 'lane' and last_road:
            incoming = self.uses[last_road]
            triple = [self.features[self.regions[x]['name']]['semantic_id'] for x in (incoming.region_id, a.region_id, b.region_id)]
            if triple in self.rules.get('forbidden_transitions', []): return False
            if incoming.region_id == b.region_id and incoming.screen_side != b.screen_side:
                # Map has no explicit U-turn permission. Leave it unresolved.
                allowed = self.rules.get('allowed_u_turn_junctions', [])
                if a.region_id not in allowed: return False
        return True

    def sliced(self, use, s0, s1):
        if use.direction*(s1-s0) < -1e-8: raise ValueError('ILLEGAL_REVERSE_EDGE_USE')
        pts = np.asarray(use.geometry); lengths = np.linalg.norm(np.diff(pts, axis=0), axis=1); offsets = np.r_[0., np.cumsum(lengths)]
        lo, hi = use.direction*(s0-use.s_from), use.direction*(s1-use.s_from)
        lo, hi = np.clip([lo, hi], 0., offsets[-1])
        def at(d):
            i = min(len(lengths)-1, max(0, int(np.searchsorted(offsets, d, side='right')-1)))
            return pts[i]+(pts[i+1]-pts[i])*(d-offsets[i])/max(lengths[i], 1e-12)
        geom = np.vstack([at(lo), pts[(offsets > lo+1e-8)&(offsets < hi-1e-8)], at(hi)])
        return replace(use, s_from=float(s0), s_to=float(s1), geometry=tuple(map(tuple, geom.tolist())))

    def plan(self, query_id, start, goal, start_region, goal_region, excluded=(), timeout=5., start_road=None, goal_road=None):
        tick = time.monotonic(); starts = self.attach(query_id, 'start', start, start_region); goals = self.attach(query_id, 'goal', goal, goal_region)
        if start_road is not None: starts=[a for a in starts if a['road_id']==start_road]
        if goal_road is not None: goals=[a for a in goals if a['road_id']==goal_road]
        if not starts or not goals: raise AttachmentError('NO_AUTHORIZED_ATTACHMENT_FOR_REQUESTED_ROAD', {'start_road':start_road,'goal_road':goal_road})
        goal_by_use = defaultdict(list)
        for a in goals: goal_by_use[a['use_id']].append(a)
        queue=[]; counter=itertools.count(); best={}; forbidden=set(excluded); solution=None; expanded=0
        for attach in starts:
            u=self.uses[attach['use_id']]; state=(u.use_id,u.use_id)
            cost=abs(u.s_to-attach['s_m']); best[state]=cost
            heapq.heappush(queue,(cost,next(counter),state,(u.use_id,),attach))
            for target in goal_by_use[u.use_id]:
                if u.direction*(target['s_m']-attach['s_m'])>=-1e-8:
                    value=abs(target['s_m']-attach['s_m'])
                    if solution is None or value<solution[0]:solution=(value,(u.use_id,),attach,target)
        while queue:
            if time.monotonic()-tick>timeout:raise TimeoutError('EDGE_TOPOLOGY_TIMEOUT')
            cost,_,state,path,attach=heapq.heappop(queue)
            if cost!=best.get(state):continue
            if solution is not None and cost>solution[0]+1e-9:continue
            uid,lastroad=state; u=self.uses[uid];expanded+=1
            for nextid in self.outgoing[u.end_key]:
                if (uid,nextid) in forbidden:continue
                v=self.uses[nextid]
                if not self.transition(u,v,lastroad):continue
                for target in goal_by_use[nextid]:
                    value=cost+abs(target['s_m']-v.s_from)
                    if solution is None or value<solution[0]:solution=(value,(*path,nextid),attach,target)
                newlast=nextid if v.kind=='lane' else lastroad; ns=(nextid,newlast); nc=cost+v.length
                if nc+1e-9<best.get(ns,math.inf):
                    best[ns]=nc;heapq.heappush(queue,(nc,next(counter),ns,(*path,nextid),attach))
        if solution is None:raise RuntimeError('EDGE_TOPOLOGY_NO_LEGAL_ROUTE')
        cost,path,a,b=solution;uses=[]
        for i,uid in enumerate(path):
            u=self.uses[uid]; uses.append(self.sliced(u,a['s_m'] if i==0 else u.s_from,b['s_m'] if i==len(path)-1 else u.s_to))
        transitions=[{'from_use':x.use_id,'to_use':y.use_id,'at':x.end_key,'traffic_checked':True} for x,y in zip(uses,uses[1:])]
        result={'schema':self.SCHEMA,'query_id':query_id,'start_attachment':a,'goal_attachment':b,
                'ordered_edge_uses':[u.as_dict() for u in uses], 'legal_transitions':transitions,
                'topology_hash':self.graph_hash,'rules_version':self.key,'cost_m':cost,
                'diagnostics':{'l1_ms':(time.monotonic()-tick)*1000,'expanded':expanded,
                'permanent_nodes_added':0,'permanent_edges_added':0,'occupancy_used':False,
                'global_nearest_road_fallback':False,'uturn_default':'requires_explicit_junction_permission'}}
        result['signature']=fingerprint(result | {'diagnostics':{}})
        return result

    def export(self):
        return {'schema':self.SCHEMA,'graph_hash':self.graph_hash,'rules_version':self.key,
                'original_nodes':len(self.graph['nodes']),'original_edges':len(self.graph['edges']),
                'directed_use_intervals':[u.as_dict() for u in self.uses.values()],
                'excluded_segments':self.excluded_segments,'warnings':self.compile_warnings,
                'preparation_ms':self.preparation_ms,'new_permanent_nodes':0,'new_permanent_edges':0}
