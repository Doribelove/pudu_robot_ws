"""Query-local, replayable forward Dubins endpoint edges (PLN-02 2A-V1 r3).

Negative certificates describe bounded search exhaustion, never a proof of
nonexistence in unrestricted SE(2). Persisted topology is never mutated.
"""
from __future__ import annotations

from collections import OrderedDict
from dataclasses import asdict, dataclass
import hashlib
import heapq
import json
import math
import time
from typing import Any

import cv2
import numpy as np
from scipy import ndimage
from skimage.graph import MCP_Geometric

from .endpoint_heading import _dubins_words, _mod2pi
from .topology import TopologyRoute, NodeSpatialIndex

try:
    from . import _endpoint_geometry_r3 as _native_geometry
except ImportError:
    _native_geometry = None
NATIVE_GEOMETRY_SHA256 = (hashlib.sha256(__import__('pathlib').Path(_native_geometry.__file__).read_bytes()).hexdigest()
                          if _native_geometry is not None else 'python-reference')

REVISION = 'r3-next3-runtime-contract-hard-query-feasibility'
FAILURES = {'ENDPOINT_NO_NEARBY_TOPOLOGY', 'ENDPOINT_LOCAL_FREE_DISCONNECTED',
            'ENDPOINT_LOCAL_SE2_NO_PATH', 'ENDPOINT_TANGENT_INCOMPATIBLE',
            'ENDPOINT_COMPONENT_MISMATCH', 'TOPOLOGY_ROUTE_NOT_FOUND'}


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def wrap(a):
    return (float(a) + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class ConnectorConfig:
    radius_m: float = 25.0
    minimum_turning_radius_m: float = 0.40
    heading_bins: int = 48
    sample_spacing_m: float = 0.025
    max_candidates: int = 32
    max_expanded: int = 2500
    max_length_m: float = 40.0
    cache_capacity: int = 64
    heuristic_weight: float = 1.25
    algorithm: str = REVISION

    def __post_init__(self):
        if self.minimum_turning_radius_m < .40 or self.heading_bins != 48:
            raise ValueError('frozen DUBIN/48/Rmin contract')
        if not 0 < self.sample_spacing_m <= .025 or self.radius_m <= 0:
            raise ValueError('invalid connector geometry')
        if not 1. <= self.heuristic_weight <= 2.:
            raise ValueError('invalid bounded-search heuristic weight')


def integrate(pose, kind, length, radius, spacing):
    x, y, yaw = map(float, pose)
    out = []
    steps = max(1, math.ceil(length / spacing))
    for _ in range(steps):
        ds = length / steps
        k = {'L': 1., 'R': -1., 'S': 0.}[kind] / radius
        if k:
            new = yaw + k * ds
            x += (math.sin(new) - math.sin(yaw)) / k
            y += (-math.cos(new) + math.cos(yaw)) / k
            yaw = new
        else:
            x += ds * math.cos(yaw); y += ds * math.sin(yaw)
        out.append((x, y, wrap(yaw)))
    return out


def dubins_distance(start,goal,radius):
    theta=math.atan2(goal[1]-start[1],goal[0]-start[0])
    words=_dubins_words(_mod2pi(start[2]-theta),_mod2pi(goal[2]-theta),
                        math.dist(start[:2],goal[:2])/radius)
    return min((sum(lengths)*radius for _,lengths in words),default=math.inf)


def dubins_paths(start, goal, config):
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    theta = math.atan2(dy, dx)
    words = _dubins_words(_mod2pi(start[2] - theta), _mod2pi(goal[2] - theta),
                         math.hypot(dx, dy) / config.minimum_turning_radius_m)
    for word, lengths in sorted(words, key=lambda v: (sum(v[1]), v[0])):
        total = sum(lengths) * config.minimum_turning_radius_m
        if total > config.max_length_m:
            continue
        points = [tuple(map(float, start))]
        for kind, length in zip(word, lengths):
            if length > 1e-12:
                points.extend(integrate(points[-1], kind, length * config.minimum_turning_radius_m,
                                        config.minimum_turning_radius_m, config.sample_spacing_m))
        if math.hypot(points[-1][0] - goal[0], points[-1][1] - goal[1]) > 1e-7:
            continue
        if abs(wrap(points[-1][2] - goal[2])) > 1e-7:
            continue
        points[-1] = tuple(map(float, goal))
        yield points, total, word


class LocalConnector:
    def __init__(self, hospital_map, footprint, config):
        self.map = hospital_map; self.footprint = footprint; self.config = config
        self.radius = max(math.hypot(*p) for p in footprint)
        self._native_checker = (_native_geometry.make_checker(
            hospital_map.occupancy, hospital_map.resolution, *hospital_map.origin[:2], footprint)
            if _native_geometry is not None else None)
        self._obstacle_field_cache = None

    def _pose_safe(self,p,padding=0.):
        cell=self.map.world_to_cell(p[0],p[1])
        if cell is None or self.map.occupancy[cell]!=0:return False
        radius=self.radius+math.sqrt(2.)*padding
        bounds=getattr(self,'_distance_window',None)
        if bounds is not None:
            r0,c0,dist=bounds;r,c=cell[0]-r0,cell[1]-c0
            if (0<=r<dist.shape[0] and 0<=c<dist.shape[1]
                and min(r+.5,c+.5,dist.shape[0]-r-.5,dist.shape[1]-c-.5)*self.map.resolution>radius+2*self.map.resolution
                and dist[r,c]>radius+2*self.map.resolution):return True
        if self._native_checker is not None:
            return not _native_geometry.collision(self._native_checker, *p, padding)
        footprint=self.footprint
        if padding:
            x0=min(p[0] for p in footprint)-padding;x1=max(p[0] for p in footprint)+padding
            y0=min(p[1] for p in footprint)-padding;y1=max(p[1] for p in footprint)+padding
            footprint=((x0,y0),(x0,y1),(x1,y1),(x1,y0))
        return not self.map.footprint_collision(p,footprint,unknown_is_collision=True)

    def _swept_safe(self,a,b,depth=0):
        angle=wrap(b[2]-a[2]);chord=math.dist(a[:2],b[:2])
        if chord<1e-12:return abs(angle)<1e-12
        if abs(angle)<1e-10:
            length=chord;mid=((a[0]+b[0])/2,(a[1]+b[1])/2,wrap(a[2]+angle/2))
        else:
            curvature=2*math.sin(angle/2)/chord
            length=abs(angle/curvature);yaw=a[2]+angle/2
            mid=(a[0]+(math.sin(yaw)-math.sin(a[2]))/curvature,
                 a[1]+(math.cos(a[2])-math.cos(yaw))/curvature,wrap(yaw))
        # Every footprint vertex along this constant-curvature primitive is
        # within translation + rotation displacement of the midpoint shape.
        # A locally expanded rectangle contains that complete swept envelope.
        margin=length/2+self.radius*abs(angle)/2
        if self._pose_safe(mid,padding=margin):return True
        if not self._pose_safe(mid):return False
        if depth>=8 or length<=.0005:return False  # unresolved envelope fails closed
        return self._swept_safe(a,mid,depth+1) and self._swept_safe(mid,b,depth+1)

    def safe(self,poses):
        previous=None
        for pose in poses:
            if not self._pose_safe(pose):return False
            if previous is not None and not self._swept_safe(previous,pose):return False
            previous=pose
        return True

    def free_component(self, pose):
        """Local all-heading footprint-safe sufficient connectivity filter.

        Near narrow endpoints, union of 48 full-footprint masks is used as a
        necessary positional screen. Its heading changes are NOT a connector.
        """
        m = self.map; cell = m.world_to_cell(*pose[:2])
        if cell is None:
            return None
        halo = math.ceil((self.config.radius_m + self.radius + .3) / m.resolution)
        r, c = cell; r0, r1 = max(0, r-halo), min(m.height, r+halo+1)
        c0, c1 = max(0, c-halo), min(m.width, c+halo+1)
        raw = (m.occupancy[r0:r1, c0:c1] == 0).astype(np.uint8)
        self._local_free_window=(r0,c0,raw)
        self._distance_window=(r0,c0,cv2.distanceTransform(raw,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*m.resolution)
        h = math.ceil(self.radius/m.resolution) + 2
        union = np.zeros_like(raw)
        for yaw in [pose[2]] + list(np.arange(48)*2*math.pi/48):
            pts = []
            for x,y in self.footprint:
                pts.append([round(h+(x*math.cos(yaw)-y*math.sin(yaw))/m.resolution),
                            round(h-(x*math.sin(yaw)+y*math.cos(yaw))/m.resolution)])
            kernel = np.zeros((2*h+1,2*h+1),np.uint8)
            cv2.fillPoly(kernel,[np.array(pts,np.int32)],1)
            # Account for intersecting cell squares rather than just centers.
            kernel = cv2.dilate(kernel, np.ones((3,3),np.uint8))
            union |= cv2.erode(raw,kernel,borderType=cv2.BORDER_CONSTANT,borderValue=0)
        _, labels = cv2.connectedComponents(union, connectivity=8)
        label = int(labels[r-r0,c-c0])
        return r0,c0,labels,label

    def obstacle_heuristic(self,goal):
        """Guide bounded SE(2) expansion around walls; never accept/prune a pose.

        A positional cost-to-go is only a ranking aid. Every motion still uses
        the same continuous swept-footprint predicate. Outside the local grid
        or beyond its reachable cells, retain Euclidean guidance.
        """
        window=getattr(self,'_local_free_window',None)
        if window is None:return lambda p:math.dist(p[:2],goal[:2])
        r0,c0,raw=window;m=self.map;cell=m.world_to_cell(*goal[:2])
        if cell is None:return lambda p:math.dist(p[:2],goal[:2])
        rr,cc=cell[0]-r0,cell[1]-c0
        if not (0<=rr<raw.shape[0] and 0<=cc<raw.shape[1] and raw[rr,cc]):
            return lambda p:math.dist(p[:2],goal[:2])
        key=(r0,c0,raw.shape,tuple(goal),m.resolution,m.origin,
             hashlib.sha256(raw.tobytes()).hexdigest())
        cached=self._obstacle_field_cache
        if cached is None or cached[0]!=key:
            costs=np.where(raw!=0,1.,np.inf)
            field,_=MCP_Geometric(costs,fully_connected=True,
                                  sampling=(m.resolution,m.resolution)).find_costs([(rr,cc)])
            self._obstacle_field_cache=(key,field.astype(np.float32))
        field=self._obstacle_field_cache[1]
        def estimate(p):
            euclidean=math.dist(p[:2],goal[:2]);cell=m.world_to_cell(*p[:2])
            if cell is None:return euclidean
            r,c=cell[0]-r0,cell[1]-c0
            if 0<=r<field.shape[0] and 0<=c<field.shape[1]:
                value=float(field[r,c])
                if math.isfinite(value):return max(euclidean,value)
            return euclidean
        return estimate

    def connect(self, start, goal, deadline, *, hybrid=True):
        cfg = self.config; generated = 0
        for path,length,word in dubins_paths(start,goal,cfg):
            generated += 1
            if time.monotonic() >= deadline:
                return None, {'expanded':0,'generated':generated,'budget_exhausted':True}
            if self.safe(path):
                return (path,length,word), {'expanded':0,'generated':generated}
        if not hybrid:
            return None, {'expanded':0,'generated':generated}
        # Bounded deterministic 48-bin Hybrid A*, with analytic exact goal.
        # Continuous poses are stored separately from dominance keys.
        quantum = 2*math.pi/cfg.heading_bins
        heuristic=self.obstacle_heuristic(goal)
        def key(p):
            return (round(p[0]/.05), round(p[1]/.05), round(wrap(p[2])/quantum)%48)
        start = tuple(start); goal=tuple(goal)
        # This is a bounded feasibility connector, not an optimal-path proof.
        # Mildly weighted guidance avoids expanding an entire equal-cost
        # positional wavefront before progressing around a long obstacle.
        queue=[(cfg.heuristic_weight*heuristic(start),0.,0,start)]; best={key(start):0.}
        parent={}; poses={0:start}; counter=0; expanded=0
        while queue and expanded < cfg.max_expanded and time.monotonic() < deadline:
            _,cost,idx,p=heapq.heappop(queue)
            if cost > best.get(key(p),math.inf)+1e-9: continue
            expanded += 1
            if expanded % 8 == 1:
                for tail,length,word in dubins_paths(p,goal,cfg):
                    generated += 1
                    if cost+length <= cfg.max_length_m and self.safe(tail):
                        chunks=[tail]; cur=idx
                        while cur in parent:
                            prev,segment=parent[cur]; chunks.append(segment);cur=prev
                        chunks.reverse(); path=[start]
                        for chunk in chunks: path.extend(chunk[1:] if chunk[0]==path[-1] else chunk)
                        return (path,cost+length,'HA*:'+word),{'expanded':expanded,'generated':generated}
            for kind in ('S','L','R'):
                length=.10 if kind=='S' else cfg.minimum_turning_radius_m*quantum*2
                nc=cost+length
                if nc>cfg.max_length_m: continue
                segment=integrate(p,kind,length,cfg.minimum_turning_radius_m,cfg.sample_spacing_m)
                target=segment[-1]; k=key(target)
                if nc>=best.get(k,math.inf)-1e-9: continue
                if math.dist(target[:2],start[:2])>cfg.max_length_m: continue
                generated+=1
                if not self.safe([p]+segment): continue
                best[k]=nc;counter+=1;poses[counter]=target;parent[counter]=(idx,[p]+segment)
                heapq.heappush(queue,(nc+cfg.heuristic_weight*heuristic(target),nc,counter,target))
        return None,{'expanded':expanded,'generated':generated,
                     'budget_exhausted':time.monotonic()>=deadline,'open_remaining':len(queue)}


class EdgeSpatialIndex:
    """Uniform buckets over individual polyline segments; exact projections."""
    def __init__(self, graph, size=5.):
        self.graph=graph;self.size=size;self.buckets={};self.segments={}
        self.nodes={n.node_id:n for n in graph.nodes}
        self.adjacency=graph.adjacency()
        self.node_index=NodeSpatialIndex.build(graph.nodes,size)
        for edge in sorted(graph.edges,key=lambda e:e.edge_id):
            offset=0.
            for i,(a,b) in enumerate(zip(edge.polyline,edge.polyline[1:])):
                length=math.dist(a,b)
                if length<1e-10:continue
                key=(edge.edge_id,i);self.segments[key]=(edge,a,b,offset,length)
                for x in range(math.floor(min(a[0],b[0])/size),math.floor(max(a[0],b[0])/size)+1):
                    for y in range(math.floor(min(a[1],b[1])/size),math.floor(max(a[1],b[1])/size)+1):
                        self.buckets.setdefault((x,y),[]).append(key)
                offset+=length

    def query(self,pose,radius,limit,*,reachable=None):
        x,y=pose[:2];keys=set()
        for bx in range(math.floor((x-radius)/self.size),math.floor((x+radius)/self.size)+1):
            for by in range(math.floor((y-radius)/self.size),math.floor((y+radius)/self.size)+1):
                keys.update(self.buckets.get((bx,by),()))
        # One closest projection per 0.5m edge interval preserves alternative
        # access around obstacles without crowding candidates at every pixel.
        found={}
        for key in sorted(keys):
            edge,a,b,offset,length=self.segments[key]
            t=max(0.,min(1.,((x-a[0])*(b[0]-a[0])+(y-a[1])*(b[1]-a[1]))/length**2))
            p=(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]));distance=math.dist((x,y),p)
            if distance>radius:continue
            s=offset+t*length;bin_key=(edge.edge_id,int(s/.5))
            v=(distance,edge.edge_id,key[1],t,p,s,math.atan2(b[1]-a[1],b[0]-a[0]),'edge_projection')
            if bin_key not in found or v[:4]<found[bin_key][:4]:found[bin_key]=v
        node_candidates=[]
        for node in self.node_index.query(x,y,radius):
            for _,edge,reverse in self.adjacency.get(node.node_id,()):
                if len(edge.polyline)<2:continue
                i=len(edge.polyline)-2 if reverse else 0;t=1. if reverse else 0.
                a,b=edge.polyline[i:i+2];yaw=math.atan2(b[1]-a[1],b[0]-a[0])
                s=sum(math.dist(a,b) for a,b in zip(edge.polyline,edge.polyline[1:])) if reverse else 0.
                node_candidates.append((math.hypot(x-node.x,y-node.y),edge.edge_id,i,t,(node.x,node.y),s,yaw,'node'))
        ordered=sorted(list(found.values())+node_candidates,key=lambda v:(v[:4],v[7]))
        if reachable is not None:
            # Apply positional reachability before allocating the bounded SE(2)
            # candidate quota. Nearby projections behind a wall must not use
            # every slot and hide reachable projections in the same radius.
            ordered=[v for v in ordered if reachable(v[4])]
        by_component={};edge_by_id={e.edge_id:e for e in self.graph.edges}
        for v in ordered:
            component=self.nodes[edge_by_id[v[1]].source].component_id
            by_component.setdefault(component,[]).append(v)
        groups=[];weights={}
        for edge in self.graph.edges:
            comp=self.nodes[edge.source].component_id
            weights[comp]=weights.get(comp,0.)+edge.length_m
        for component,values in sorted(by_component.items(),key=lambda item:(item[1][0][0],item[0])):
            primary=[];rest=[];seen=set()
            for v in values:
                if v[1] not in seen:primary.append(v);seen.add(v[1])
                else:rest.append(v)
            groups.append((component,primary+rest))
        # Reserve nearby component diversity, then apportion detail by graph
        # extent. Uniform component quotas let dozens of tiny skeleton islands
        # crowd out all useful projections of the principal route component.
        result=[];cursors={c:0 for c,_ in groups}
        for component,values in groups[:max(1,limit//4)]:
            result.append(values[0]);cursors[component]=1
            if len(result)>=limit:return result
        queue=[]
        for component,values in groups:
            rank=cursors[component]
            if rank<len(values):
                heapq.heappush(queue,((rank+1)/math.sqrt(max(.05,weights[component])),
                                     values[rank][0],component,rank,values))
        while queue and len(result)<limit:
            _,_,component,rank,values=heapq.heappop(queue)
            result.append(values[rank]);rank+=1
            if rank<len(values):
                heapq.heappush(queue,((rank+1)/math.sqrt(max(.05,weights[component])),
                                     values[rank][0],component,rank,values))
        return result


class ReachableEndpointSelector:
    def __init__(self,topology,footprint,config=ConnectorConfig()):
        self.topology=topology;self.config=config;self.footprint=footprint
        graph=topology.graph
        self.binding={'map':topology.metadata.get('map_sha256'),
                      'topology':digest({'nodes':[vars(n) for n in graph.nodes],
                                         'edges':[vars(e) for e in graph.edges]}),
                      'resolution':topology.hospital_map.resolution,'origin':topology.hospital_map.origin,
                      'shape':[topology.hospital_map.height,topology.hospital_map.width],'footprint':footprint,
                      'config':asdict(config),'revision':REVISION,
                      'connector_collision_certificate':'continuous_primitive_swept_rectangle_envelope_v1',
                      'native_geometry_sha256':NATIVE_GEOMETRY_SHA256,
                      'implementation_sha256':hashlib.sha256(__import__('pathlib').Path(__file__).read_bytes()).hexdigest()}
        self.index=EdgeSpatialIndex(graph);self.edges={e.edge_id:e for e in graph.edges}
        self.connector=LocalConnector(topology.hospital_map,footprint,config)
        self.cache=OrderedDict();self.last_certificate=None;self.calls=0

    def _candidates(self,pose,goal,deadline):
        raw=self.index.query(pose,self.config.radius_m,self.config.max_candidates)
        screen=self.connector.free_component(pose);out=[];records=[]
        attempts=expanded=generated=0
        def locally_reachable(p):
            r=self.topology.hospital_map.world_to_cell(*p)
            if r is not None and screen is not None:
                r0,c0,labels,label=screen
                rr,cc=r[0]-r0,r[1]-c0
                return 0<=rr<labels.shape[0] and 0<=cc<labels.shape[1] and label!=0 and int(labels[rr,cc])==label
            return False
        rejected=[v for v in raw if not locally_reachable(v[4])]
        if rejected:
            records.extend({'edge_id':v[1],'segment':v[2],'point':v[4],
                            'failure':'ENDPOINT_LOCAL_FREE_DISCONNECTED'} for v in rejected)
            raw=self.index.query(pose,self.config.radius_m,self.config.max_candidates,
                                 reachable=locally_reachable)
        for v in raw:
            distance,eid,i,t,p,s,yaw,kind=v;edge=self.edges[eid]
            for direction in (1,-1):
                tangent=wrap(yaw+(math.pi if direction<0 else 0))
                target=(*p,tangent);a,b=(target,pose) if goal else (pose,target)
                attempts+=1;result,stats=self.connector.connect(a,b,deadline,hybrid=False)
                expanded+=stats['expanded'];generated+=stats['generated']
                record={'candidate_type':kind,'edge_id':eid,'segment':i,'fraction':t,'arclength_m':s,'point':p,
                        'direction':direction,'tangent_yaw':tangent,'stats':stats,
                        'component':self.index.nodes[edge.source].component_id}
                if result is None:
                    record['failure']='ENDPOINT_LOCAL_SE2_NO_PATH'
                    record['secondary_failure']='ENDPOINT_TANGENT_INCOMPATIBLE' if abs(wrap(tangent-pose[2]))>math.pi/2 else ''
                    records.append(record);continue
                path,length,word=result
                clearance=min(float(self.topology.hospital_map.clearance(*p[:2]) or 0) for p in path)
                turn=sum(abs(wrap(b[2]-a[2])) for a,b in zip(path,path[1:]))
                record.update({'path':path,'length_m':length,'word':word,'min_clearance_m':clearance,
                               'turn_rad':turn,'tangent_error_deg':math.degrees(abs(wrap(path[0 if goal else -1][2]-tangent))),
                               'failure':'','component':self.index.nodes[edge.source].component_id})
                record['hash']=digest({'binding':self.binding,'endpoint':pose,'goal':goal,'connector':record})
                records.append(record);out.append(record)
        return out,records,{'attempts':attempts,'expanded':expanded,'generated':generated,
                            'candidate_count':len(raw)+len(rejected),
                            'local_screen_replenished':bool(rejected),
                            'initial_local_screen_rejections':len(rejected),
                            'projection_count':sum(v[7]=='edge_projection' for v in raw+rejected)}

    def _refine_candidates(self,pose,goal,out,records,stats,other,deadline,*,only_opposite=False,exclude_near=None):
        """Spend Hybrid A* only when the analytic virtual graph has no route.

        Prefer components already reached by the opposite endpoint. After each
        new certificate, stop as soon as the two virtual nodes have a route.
        """
        if time.monotonic()>=deadline:return
        reached={r['component'] for r in out}
        opposite={r['component'] for r in other}
        failed=[r for r in records if r.get('failure')=='ENDPOINT_LOCAL_SE2_NO_PATH'
                and r.get('component') not in reached and not r.get('hybrid_attempted')
                and (exclude_near is None or math.dist(r['point'],exclude_near)>=1.)
                and (not only_opposite or r.get('component') in opposite)]
        first=min(failed,key=lambda r:(r['component'] not in opposite,records.index(r))) if failed else None
        def rank(r):
            target=(*r['point'],r['tangent_yaw']);a,b=(target,pose) if goal else (pose,target)
            return (r['component'] not in opposite,
                    dubins_distance(a,b,self.config.minimum_turning_radius_m),records.index(r))
        failed.sort(key=rank)
        # Nearby opposing tangents remain available, but cannot consume the
        # entire four-attempt budget before spatially different alternatives.
        diverse=[first] if first is not None else [];deferred=[]
        for r in failed:
            if r is first:continue
            if any(r['component']==v['component'] and math.dist(r['point'],v['point'])<1. for v in diverse):deferred.append(r)
            else:diverse.append(r)
        failed=sorted(diverse+deferred,key=lambda r:r['component'] not in opposite)
        if not failed:return
        self.connector.free_component(pose)
        allowance=max(0,4-sum(bool(r.get('hybrid_attempted')) for r in records))
        for record in failed[:allowance]:
            if time.monotonic()>=deadline:break
            if any(v['component']==record['component'] for v in out):continue
            target=(*record['point'],record['tangent_yaw']);a,b=(target,pose) if goal else (pose,target)
            stats['attempts']+=1;record['hybrid_attempted']=True
            result,search=self.connector.connect(a,b,deadline)
            stats['expanded']+=search['expanded'];stats['generated']+=search['generated'];record['stats']=search
            if result is None:continue
            path,length,word=result
            clearance=min(float(self.topology.hospital_map.clearance(*p[:2]) or 0) for p in path)
            turn=sum(abs(wrap(b[2]-a[2])) for a,b in zip(path,path[1:]))
            record.update(path=path,length_m=length,word=word,min_clearance_m=clearance,turn_rad=turn,
                          tangent_error_deg=0.,failure='')
            record['hash']=digest({'binding':self.binding,'endpoint':pose,'goal':goal,'connector':record})
            out.append(record)
            route,_=self._route(other,out) if goal else self._route(out,other)
            if route is not None:break

    def _improve_detours(self,query,starts,goals,srecords,grecords,ss,gs,deadline):
        """Use leftover attempts for looping certificates, reserving L3 time.

        A feasible connector remains available if improvement exhausts its
        smaller allowance. Each endpoint retains the same four-attempt cap.
        """
        route,chosen=self._route(starts,goals)
        attempts_before=ss['attempts']+gs['attempts'];exhausted=False
        if chosen and deadline-time.monotonic()>2.:
            for goal,pose,out,records,stats,other,selected in (
                    (False,query.start,starts,srecords,ss,goals,chosen[0]),
                    (True,query.goal,goals,grecords,gs,starts,chosen[1])):
                if selected['turn_rad']<2*math.pi or selected['length_m']<2*max(1.,math.dist(pose[:2],selected['point'])):
                    continue
                if deadline-time.monotonic()<=2.:break
                alternatives=[];before={id(v) for v in records if v.get('hybrid_attempted')}
                self._refine_candidates(pose,goal,alternatives,records,stats,other,
                                        min(deadline-2.,time.monotonic()+1.),
                                        only_opposite=True,exclude_near=selected['point'])
                for record in records:
                    if record.get('hybrid_attempted') and id(record) not in before:
                        search=record.get('stats',{})
                        if search.pop('budget_exhausted',False):
                            search['quality_budget_exhausted']=True;exhausted=True
                out.extend(alternatives)
        return {'quality_connector_attempts':ss['attempts']+gs['attempts']-attempts_before,
                'quality_connector_budget_exhausted':exhausted}

    def __call__(self,topology,query,*,cache_mode=None,timing=None,deadline=None):
        assert topology is self.topology
        begin=time.monotonic();deadline=deadline if deadline is not None else begin+30.
        key=digest({'binding':self.binding,'start':query.start,'goal':query.goal})
        hit=key in self.cache;self.calls+=1
        if hit:
            # Keep the private cache immutable. JSON reconstructs independent
            # caller-owned geometry and certificates without a Python recursive
            # deepcopy or a redundant hash of an unchanged query identity.
            payload=self.cache.pop(key);self.cache[key]=payload
            decoded=json.loads(payload);cert=decoded['certificate']
            start,goal,route,reason=decoded['value']
            value=(start,goal,TopologyRoute(**route) if route is not None else None,reason)
            if cert['query_id']!=query.query_id:
                cert['query_id']=query.query_id
                cert['hash']=digest({k:v for k,v in cert.items() if k not in {'diagnostics','hash'}})
            self.last_certificate=cert
            if timing is not None:
                timing.update(cert['diagnostics'],endpoint_connector_cache_hit=True,
                              endpoint_spatial_index_cache_hit=True,
                              endpoint_cache_materialization_wall_ms=(time.monotonic()-begin)*1000,
                              quality_connector_attempts=0,quality_connector_budget_exhausted=False,
                              local_connector_wall_ms=0.,
                              local_connector_attempts=0,local_connector_successes=0,
                              local_connector_expanded=0,local_connector_generated=0)
            return value
        starts,srecords,ss=self._candidates(query.start,False,deadline)
        goals,grecords,gs=self._candidates(query.goal,True,deadline)
        # Share priority across both endpoints: a goal candidate that can reach
        # a start component must precede speculative start-only components.
        # Each endpoint retains its original total cap of four Hybrid attempts.
        for only_opposite in (True,False):
            if self._route(starts,goals)[0] is None:
                self._refine_candidates(query.start,False,starts,srecords,ss,goals,deadline,only_opposite=only_opposite)
            if self._route(starts,goals)[0] is None:
                self._refine_candidates(query.goal,True,goals,grecords,gs,starts,deadline,only_opposite=only_opposite)
        quality=self._improve_detours(query,starts,goals,srecords,grecords,ss,gs,deadline)
        diag={**quality,'start_candidate_count':ss['candidate_count'],'goal_candidate_count':gs['candidate_count'],
              'start_local_screen_replenished':ss.get('local_screen_replenished',False),
              'goal_local_screen_replenished':gs.get('local_screen_replenished',False),
              'initial_local_screen_rejections':ss.get('initial_local_screen_rejections',0)+gs.get('initial_local_screen_rejections',0),
              'edge_projection_candidate_count':ss['projection_count']+gs['projection_count'],
              'local_connector_attempts':ss['attempts']+gs['attempts'],
              'local_connector_successes':len(starts)+len(goals),'local_connector_expanded':ss['expanded']+gs['expanded'],
              'local_connector_generated':ss['generated']+gs['generated'],
              'endpoint_spatial_index_cache_hit':self.calls>1,'endpoint_connector_cache_hit':False}
        route,selected=self._route(starts,goals)
        # Refinement can stop before starting a connector, including while
        # preparing its free-space component. Such incomplete work is never a
        # stable negative result, even if no per-connector stats were emitted.
        exhausted=(time.monotonic()>=deadline or
                   any(v.get('stats',{}).get('budget_exhausted') for v in srecords+grecords))
        diag['local_connector_budget_exhausted']=exhausted
        if route is not None:
            reason='';a,b=selected
            for prefix,v in [('start',a),('goal',b)]:
                diag.update({f'selected_{prefix}_connector_length_m':v['length_m'],
                             f'selected_{prefix}_clearance_m':v['min_clearance_m'],
                             f'selected_{prefix}_tangent_error_deg':v['tangent_error_deg']})
            value=(a,b,route,'reachable_virtual_endpoints')
        else:
            if exhausted:reason='ENDPOINT_CONNECTOR_BUDGET_EXHAUSTED'
            elif topology.metadata.get('coarse_failure'):reason=topology.metadata['coarse_failure']
            elif not ss['candidate_count'] or not gs['candidate_count']:reason='ENDPOINT_NO_NEARBY_TOPOLOGY'
            elif not starts or not goals:
                failed=srecords if not starts else grecords
                reason='ENDPOINT_LOCAL_FREE_DISCONNECTED' if failed and all(v['failure']=='ENDPOINT_LOCAL_FREE_DISCONNECTED' for v in failed) else 'ENDPOINT_LOCAL_SE2_NO_PATH'
            elif not ({v['component'] for v in starts}&{v['component'] for v in goals}):reason='ENDPOINT_COMPONENT_MISMATCH'
            else:reason='TOPOLOGY_ROUTE_NOT_FOUND'
            value=(None,None,None,reason)
        diag['local_connector_wall_ms']=(time.monotonic()-begin)*1000
        diag['virtual_topology_edge_count']=len(starts)+len(goals)
        cert={'key':key,'binding':json.loads(json.dumps(self.binding,allow_nan=False)),
              'query_id':query.query_id,'start':list(query.start),'goal':list(query.goal),
              'start_candidates':srecords,'goal_candidates':grecords,'failure':reason,
              'negative_proof_scope':'bounded local candidates; not global SE2 infeasibility' if reason else '',
              'diagnostics':diag,'selected_hashes':[v['hash'] for v in selected] if selected else []}
        # Timing/resource counters do not enter deterministic certificate hash.
        cert['hash']=digest({k:v for k,v in cert.items() if k!='diagnostics'})
        if not exhausted and time.monotonic()<deadline:
            # Freeze before exposing either cold return value; subsequent caller
            # mutation cannot alter a cached route, selected connector or binding.
            a,b,selected_route,status=value
            self.cache[key]=json.dumps({'certificate':cert,
                'value':[a,b,asdict(selected_route) if selected_route is not None else None,status]},
                sort_keys=True,separators=(',',':'),allow_nan=False)
            while len(self.cache)>self.config.cache_capacity:self.cache.popitem(last=False)
        self.last_certificate=cert
        # The memoized connector certificate replaces search work on reuse;
        # do not retain a dense local heuristic for every cached query.
        self.connector._obstacle_field_cache=None
        if timing is not None:timing.update(diag)
        return value

    def _piece(self,c,side):
        e=self.edges[c['edge_id']];i=c['segment'];p=list(c['point'])
        left=e.polyline[:i+1]+[p];right=[p]+e.polyline[i+1:]
        if side=='start':return (right,e.target) if c['direction']==1 else (list(reversed(left)),e.source)
        return (left,e.source) if c['direction']==1 else (list(reversed(right)),e.target)

    def _route(self,starts,goals):
        if not starts or not goals:return None,None
        def cost(c):return c['length_m']+.02*c['turn_rad']+.01/max(.01,c['min_clearance_m'])
        graph=self.topology.graph;adj=graph.adjacency();heap=[];dist={};parents={};owners={};serial=0
        for i,c in enumerate(starts):
            piece,node=self._piece(c,'start');d=cost(c)+sum(math.dist(a,b) for a,b in zip(piece,piece[1:]))
            if (d,i)<(dist.get(node,math.inf),owners.get(node,math.inf)):
                dist[node]=d;owners[node]=i;parents[node]=None;heapq.heappush(heap,(d,node,i))
        while heap:
            d,node,owner=heapq.heappop(heap)
            if d!=dist[node] or owner!=owners[node]:continue
            for target,edge,reverse in sorted(adj.get(node,()),key=lambda v:(v[0],v[1].edge_id)):
                nd=d+edge.length_m
                if nd<dist.get(target,math.inf)-1e-10:
                    dist[target]=nd;owners[target]=owner;parents[target]=(node,edge,reverse);heapq.heappush(heap,(nd,target,owner))
        choices=[]
        for j,g in enumerate(goals):
            piece,node=self._piece(g,'goal')
            if node not in dist:continue
            choices.append((dist[node]+cost(g)+sum(math.dist(a,b) for a,b in zip(piece,piece[1:])),owners[node],j,node))
        # Direct same-edge routes do not detour to a junction.
        for i,s in enumerate(starts):
            for j,g in enumerate(goals):
                if s['edge_id']==g['edge_id'] and s['direction']==g['direction'] and (g['arclength_m']-s['arclength_m'])*s['direction']>=0:
                    choices.append((cost(s)+cost(g)+abs(g['arclength_m']-s['arclength_m']),i,j,None))
        if not choices:return None,None
        _,i,j,node=min(choices,key=lambda v:(v[0],v[1],v[2],-1 if v[3] is None else v[3]));s,g=starts[i],goals[j]
        ids=[];nodes=[]
        if node is None:
            e=self.edges[s['edge_id']];lo,hi=sorted([s['segment'],g['segment']]);mid=e.polyline[lo+1:hi+1]
            if s['direction']<0:mid=list(reversed(mid))
            points=[list(s['point'])]+mid+[list(g['point'])];ids=[e.edge_id]
        else:
            chain=[];cur=node
            while parents[cur] is not None:
                prev,edge,reverse=parents[cur];chain.append((edge,reverse));nodes.append(cur);cur=prev
            points,_=self._piece(s,'start')
            for edge,reverse in reversed(chain):
                points+=list(reversed(edge.polyline)) if reverse else edge.polyline;ids.append(edge.edge_id)
            piece,_=self._piece(g,'goal');points+=piece
        points=[list(p[:2]) for p in s['path']]+points+[list(p[:2]) for p in g['path']]
        compact=[list(points[0])]
        for p in points[1:]:
            if math.dist(p,compact[-1])>1e-10:compact.append(list(p))
        length=sum(math.dist(a,b) for a,b in zip(compact,compact[1:]))
        return TopologyRoute([-1]+list(reversed(nodes))+[-2],[-1]+ids+[-2],length,
                             2*min(s['min_clearance_m'],g['min_clearance_m']),compact),(s,g)
