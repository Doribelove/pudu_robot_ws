"""Finite route-progress Dubins graph with exact resource dominance.

Labels retain length and integer semantic history. This is an offline bounded
feasibility solver. OPEN exhaustion is a statement about the documented finite
graph only, never the continuous free space. No state changes the master grid.
"""
from __future__ import annotations

import argparse
from array import array
from collections import Counter
import heapq
import json
import math
from pathlib import Path
import resource
import shutil
import sys
import time

import numpy as np

from .semantic_constraint_core import ConstraintWorld, dubins_edge, wrap_angle
from .semantic_constraint_study import write_json, fresh, seal
from .semantic_map import canonical_hash, sha256_file


class Labels:
    """Compact immutable parent records; deletions only mark frontier activity."""
    def __init__(self):
        self.vertex = array("i")
        self.parent = array("i")
        self.length = array("d")
        self.side = array("i")
        self.target = array("i")
        self.active = bytearray()
        self.frontiers = {}
        self.dominated = 0

    def add(self, vertex, parent, length, side, target):
        prior = self.frontiers.setdefault(vertex, [])
        # No epsilon dominance, rounding, resource bucketing or per-vertex cap.
        for index in prior:
            if self.length[index] <= length and self.side[index] >= side and self.target[index] >= target:
                self.dominated += 1
                return None
        survivors = []
        for index in prior:
            if length <= self.length[index] and side >= self.side[index] and target >= self.target[index]:
                self.active[index] = 0
                self.dominated += 1
            else:
                survivors.append(index)
        index = len(self.vertex)
        self.vertex.append(vertex)
        self.parent.append(parent)
        self.length.append(length)
        self.side.append(side)
        self.target.append(target)
        self.active.append(1)
        survivors.append(index)
        self.frontiers[vertex] = survivors
        return index

    @property
    def storage_bytes(self):
        return sum(len(x)*x.itemsize for x in (self.vertex,self.parent,self.length,self.side,self.target))+len(self.active)


def station_centers(polyline, spacing, extension):
    line = np.asarray(polyline, dtype=float)[:, :2]
    keep = np.r_[True, np.linalg.norm(np.diff(line, axis=0), axis=1)>1e-8]
    line = line[keep]
    distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(line, axis=0), axis=1))]
    length = float(distances[-1])
    stations = np.arange(-extension, length+extension+spacing*.01, spacing)
    if length not in stations:
        stations = np.sort(np.r_[stations, length])

    def at(s):
        if s < 0:
            tangent = (line[min(10,len(line)-1)]-line[0])
            return line[0]+s*tangent/np.linalg.norm(tangent)
        if s > length:
            tangent = line[-1]-line[max(0,len(line)-11)]
            return line[-1]+(s-length)*tangent/np.linalg.norm(tangent)
        return np.array([np.interp(s, distances, line[:, i]) for i in (0,1)])
    centers = []
    for s in stations:
        tangent = at(min(length+extension,s+.5))-at(max(-extension,s-.5))
        if np.linalg.norm(tangent)<1e-8:
            tangent=line[-1]-line[0]
        tangent /= np.linalg.norm(tangent)
        centers.append((float(s), at(s), tangent))
    return length, centers


class ResourceLattice:
    def __init__(self, world, *, spacing=.4, lateral=.1, extension=0., yaw_offsets=(-4,-2,0,2,4),
                 skip=2, lateral_step=.35, timeout=120., max_labels=1_000_000):
        self.world = world
        self.config = dict(station_spacing_m=spacing, lateral_spacing_m=lateral, extension_m=extension,
                           yaw_offsets=list(yaw_offsets), yaw_bins=48, max_station_skip=skip,
                           lateral_neighbor_distance_m=lateral_step, timeout_s=timeout, max_labels=max_labels,
                           radius_m=.401, local_edge_length_ratio_max=1.7, local_edge_extra_m=.05,
                           local_edge_length_max_m=2., entry_exit_max_length_m=8.,
                           queue_order="length_plus_exact_suffix_distance_plus_resource_debt_hint",
                           resource_debt_hint_is_pruning_bound=False, optimality_claim=False,
                           node_filter="selected_lane_AND_R0_collision_only", master_grid_modified=False)
        self.started = time.monotonic()
        self.labels = Labels()
        self.poses = [world.start]
        self.station = [-1]
        self.lateral = [0.]
        self.layers = []
        self.rejected = Counter()
        self.adjacency = {}
        self.goal_connectors = {}
        self.expanded = 0
        self.best_rejected = None
        self.solution_edges = []
        self.solution_audit = None
        self.queue = []
        self.serial = 0
        self.route_length, self.centers = station_centers(world.meta["route_polyline"], spacing, extension)
        for index, (s, point, tangent) in enumerate(self.centers):
            normal = np.array([-tangent[1], tangent[0]])
            yawbin = round(math.atan2(tangent[1],tangent[0])/(2*math.pi/48)) % 48
            layer, seen = [], set()
            for offset in np.arange(-12.,12.+lateral*.01,lateral):
                position = point+offset*normal
                cell = world.map.world_to_cell(*position)
                if cell is None or cell in seen:
                    continue
                seen.add(cell)
                if world.grids["labels"][cell] not in world.selected:
                    continue
                x,y = world.map.cell_to_world(cell)
                for dyaw in yaw_offsets:
                    pose = (x,y,wrap_angle((yawbin+dyaw)*2*math.pi/48))
                    if not world.collision_free(np.array([pose])):
                        continue
                    vertex = len(self.poses)
                    self.poses.append(pose)
                    self.station.append(index)
                    self.lateral.append(float(offset))
                    layer.append(vertex)
            self.layers.append(np.asarray(layer,dtype=np.int32))
        self.poses = np.asarray(self.poses)
        self.lateral = np.asarray(self.lateral)
        self.suffix_side = None
        self.suffix_target = None
        self.suffix_length = None
        self.prepass_complete = False

    def _budget_check(self):
        if time.monotonic()-self.started>=self.config["timeout_s"]:
            raise TimeoutError("frozen wall budget")
        if resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024>2048:
            raise MemoryError("frozen RSS budget")

    def _edge(self, source, target, terminal=False):
        a = self.poses[source]
        b = self.world.goal if target == -1 else self.poses[target]
        direct = math.dist(a[:2], b[:2])
        bound = 8. if terminal else min(2.,1.7*direct+.05)
        if direct > bound:
            return None
        edge = dubins_edge(a,b)
        if edge is None or edge.length>bound:
            self.rejected["EDGE_LENGTH_FAMILY_BOUND"] += 1
            return None
        if not self.world.validate_edge(edge):
            self.rejected["COLLISION_OR_LANE"] += 1
            return None
        return (target, edge.length, edge.side_resource, edge.target_resource)

    def _neighbors(self, vertex):
        if vertex in self.adjacency:
            return self.adjacency[vertex]
        neighbors=[]
        index=self.station[vertex]
        if vertex==0:
            # Entry attachment may use an extended earlier station; all poses
            # remain exact, forward Dubins edges and explicit graph choices.
            candidates=[v for i,layer in enumerate(self.layers)
                        if self.centers[i][0] <= min(3., self.route_length*.35) for v in layer]
        else:
            candidates=[]
            for j in range(index+1,min(len(self.layers),index+1+self.config["max_station_skip"])):
                layer=self.layers[j]
                delta=j-index
                if not len(layer):
                    continue
                keep=np.abs(self.lateral[layer]-self.lateral[vertex]) <= self.config["lateral_neighbor_distance_m"]*delta
                candidates.extend(layer[keep])
        for target in candidates:
            self._budget_check()
            edge=self._edge(vertex,int(target),terminal=vertex==0)
            if edge is not None:
                neighbors.append(edge)
        self.adjacency[vertex]=neighbors
        return neighbors

    def _goal_connector(self, vertex):
        if vertex!=0 and self.centers[self.station[vertex]][0] < self.route_length-min(3.,self.route_length*.35):
            return None
        if vertex not in self.goal_connectors:
            self.goal_connectors[vertex]=self._edge(vertex,-1,terminal=True)
        return self.goal_connectors[vertex]

    def prepare_suffix_bounds(self):
        """Exact independent resource upper bounds on the reachable DAG.

        Max side and max target may come from different paths. Thus they are
        optimistic and safe for rejecting a prefix; they never certify a path.
        All graph edges and all prefixes use the same serialized samples.
        """
        reachable={0}
        for layer in ([0],*self.layers):
            for raw in layer:
                vertex=int(raw)
                if vertex not in reachable:
                    continue
                self._budget_check()
                for child,*_ in self._neighbors(vertex):
                    reachable.add(child)
                self._goal_connector(vertex)
        size=len(self.poses)
        side=np.full(size,-np.inf)
        target=np.full(size,-np.inf)
        length=np.full(size,np.inf)
        ordered=[0]+[int(v) for layer in self.layers for v in layer]
        for vertex in reversed(ordered):
            if vertex not in reachable:
                continue
            edge=self.goal_connectors.get(vertex)
            if edge is not None:
                _,distance,ds,dt=edge
                side[vertex],target[vertex],length[vertex]=ds,dt,distance
            for child,distance,ds,dt in self.adjacency.get(vertex,()):
                side[vertex]=max(side[vertex],ds+side[child])
                target[vertex]=max(target[vertex],dt+target[child])
                length[vertex]=min(length[vertex],distance+length[child])
        self.suffix_side,self.suffix_target,self.suffix_length=side,target,length
        self.prepass_complete=True
        self.config["reachable_vertices"]=len(reachable)
        self.config["suffix_bound_proof"]="independent_max_over_all_reachable_DAG_suffixes; no_resource_bucketing"

    def _chain(self, label, to_goal=True):
        chain=[]
        while label>=0:
            chain.append(self.labels.vertex[label])
            label=self.labels.parent[label]
        chain.reverse()
        poses=[tuple(self.poses[v]) for v in chain]
        if to_goal:
            poses.append(self.world.goal)
        edges=[dubins_edge(a,b) for a,b in zip(poses,poses[1:])]
        for edge in edges:
            self.world.validate_edge(edge)
        return edges

    def _try_goal(self,label):
        vertex=self.labels.vertex[label]
        edge=self._goal_connector(vertex)
        if edge is None:
            return False
        _,length,side,target=edge
        total_length=self.labels.length[label]+length
        total_side=self.labels.side[label]+side
        total_target=self.labels.target[label]+target
        if total_length>self.world.bound_length:
            return False
        # Signed resources are exactly additive on the serialized edge samples.
        # The final unchanged sampler is still authoritative at concatenation.
        rank=(min(total_side,0),min(total_target,0),-total_length)
        if self.best_rejected is None or rank>tuple(self.best_rejected["rank"]):
            self.best_rejected={"rank":rank,"side_resource":total_side,"target_resource":total_target,
                                "arc_length_m":total_length,"label":label,"vertex":vertex}
        if total_side<0 or total_target<=0:
            self.rejected["PATH_RESOURCE_GATE"]+=1
            return False
        edges=self._chain(label)
        audit,_=self.world.audit(edges)
        if audit["gate_passed"]:
            self.solution_edges=edges
            self.solution_audit=audit
            return True
        self.rejected["INDEPENDENT_FINAL_AUDIT"]+=1
        return False

    def search(self):
        initial=self.world.semantic_counts(np.array([self.world.start]))
        if initial is None or not self.world.collision_free(np.array([self.world.start,self.world.goal])):
            return "INVALID_INPUT"
        n,c,t=initial
        try:
            self.prepare_suffix_bounds()
        except TimeoutError:
            return "SEARCH_TIMEOUT_PREPASS"
        except MemoryError:
            return "RSS_LIMIT"
        if 5*c-4*n+self.suffix_side[0]<0 or 2*t-n+self.suffix_target[0]<=0:
            return "FINITE_GRAPH_RESOURCE_UPPER_BOUND_INFEASIBLE"
        index=self.labels.add(0,-1,0.,5*c-4*n,2*t-n)
        heapq.heappush(self.queue,(self.suffix_length[0]+.025*max(0,4*n-5*c,n-2*t),index))
        while self.queue:
            if time.monotonic()-self.started>=self.config["timeout_s"]:
                return "SEARCH_TIMEOUT"
            if len(self.labels.vertex)>=self.config["max_labels"]:
                return "MAX_LABELS"
            if resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024>2048:
                return "RSS_LIMIT"
            _,label=heapq.heappop(self.queue)
            if not self.labels.active[label]:
                continue
            self.expanded+=1
            if self._try_goal(label):
                return "WITNESS"
            for vertex,length,side,target in self._neighbors(self.labels.vertex[label]):
                new_length=self.labels.length[label]+length
                new_side=self.labels.side[label]+side
                new_target=self.labels.target[label]+target
                if new_side+self.suffix_side[vertex]<0 or new_target+self.suffix_target[vertex]<=0:
                    self.rejected["PROVED_SUFFIX_RESOURCE_BOUND"]+=1
                    continue
                h=self.suffix_length[vertex]
                if new_length+h>self.world.bound_length:
                    self.rejected["LENGTH_LOWER_BOUND"]+=1
                    continue
                if len(self.labels.vertex)>=self.config["max_labels"]:
                    return "MAX_LABELS"
                child=self.labels.add(vertex,label,new_length,new_side,new_target)
                if child is not None:
                    # This is queue ordering only, never a pruning inequality.
                    # It prioritizes repaying the measured semantic debt rather
                    # than enumerating every shorter semantically bad history.
                    debt_hint=.025*max(0,-new_side,-new_target)
                    heapq.heappush(self.queue,(new_length+h+debt_hint,child))
        return "FINITE_GRAPH_OPEN_EXHAUSTED"


def run(args):
    output=fresh(args.output)
    world=ConstraintWorld(args.inputs,args.query)
    source_files=[Path(__file__),Path(__file__).with_name("semantic_constraint_core.py")]
    before={str(p):sha256_file(p) for p in source_files}
    write_json(output/"variant_manifest.json",{
        "stage":"offline_feasibility", "architecture":"UNNAMED_STAGE1_FEASIBILITY",
        "input_binding":world.meta,"source_hashes":before,"arguments":vars(args),
        "frozen_protocol":"PLN-02-CONSTRAINED-FEASIBILITY-R0-V1", "online_started":False,
        "dominance_proof":"At identical graph vertex and progress, every continuation adds identical length and (5C-4N,2T-N). A no-longer prefix with componentwise greater resources preserves feasibility for every continuation of a dominated label. This assumes exact vertex identity, not same rounded SE2 cell with different continuous poses.",
    })
    for p in source_files:
        shutil.copy2(p,output/p.name)
    started=time.monotonic()
    cpu=time.process_time()
    lattice=ResourceLattice(world,spacing=args.station_spacing,lateral=args.lateral_spacing,
                            extension=args.extension,skip=args.skip,yaw_offsets=tuple(args.yaw_offsets),
                            lateral_step=args.lateral_step,timeout=args.timeout)
    code=lattice.search()
    result={"query_id":args.query,"result_code":code,"gate_passed":code=="WITNESS",
            "graph_config":lattice.config,"node_count":len(lattice.poses),"station_count":len(lattice.layers),
            "cached_edge_count":sum(map(len,lattice.adjacency.values())),"expanded_labels":lattice.expanded,
            "generated_labels":len(lattice.labels.vertex),"dominated_labels":lattice.labels.dominated,
            "compact_label_bytes":lattice.labels.storage_bytes,"open_size":len(lattice.queue),
            "wall_ms":(time.monotonic()-started)*1000,"cpu_ms":(time.process_time()-cpu)*1000,
            "peak_rss_mib":resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
            "rejections":dict(lattice.rejected),"best_rejected":lattice.best_rejected,
            "timeout_is_infeasibility_proof":False,"maximum_path_arc_length_m":world.bound_length,
            "audit":lattice.solution_audit}
    if lattice.prepass_complete:
        result["suffix_bounds_at_start"]={"side_upper":float(lattice.suffix_side[0]),
                                          "target_upper":float(lattice.suffix_target[0]),
                                          "length_lower":float(lattice.suffix_length[0])}
    edges=lattice.solution_edges
    if not edges and lattice.best_rejected is not None:
        edges=lattice._chain(lattice.best_rejected["label"])
        result["best_rejected_audit"],path=world.audit(edges)
    elif edges:
        _,path=world.audit(edges)
    else:
        path=np.empty((0,3))
    write_json(output/"result.json",result)
    write_json(output/"certificate.json",{"input_npz_sha256":world.meta["npz_sha256"],
               "qualifying":code=="WITNESS","edges":[e.certificate() for e in edges]})
    write_json(output/"path.json",[{"x":x,"y":y,"yaw":yaw} for x,y,yaw in path])
    np.savez_compressed(output/"graph_nodes.npz",poses=lattice.poses,station=lattice.station,lateral=lattice.lateral)
    if lattice.prepass_complete:
        graph_edges=[(a,*e) for a,neighbors in lattice.adjacency.items() for e in neighbors]
        goal_edges=[(a,*e) for a,e in lattice.goal_connectors.items() if e is not None]
        np.savez_compressed(output/"graph_certificate.npz",edges=np.asarray(graph_edges,dtype=float),
                            goal_edges=np.asarray(goal_edges,dtype=float),side_upper=lattice.suffix_side,
                            target_upper=lattice.suffix_target,length_lower=lattice.suffix_length)
    if before!={str(p):sha256_file(p) for p in source_files}:
        raise RuntimeError("source changed during run; result must be excluded")
    seal(output)
    print(json.dumps(result,indent=2),flush=True)
    return 0 if code=="WITNESS" else 2


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs",type=Path,required=True)
    parser.add_argument("--query",required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--station-spacing",type=float,default=.4)
    parser.add_argument("--lateral-spacing",type=float,default=.1)
    parser.add_argument("--extension",type=float,default=0.)
    parser.add_argument("--skip",type=int,default=2)
    parser.add_argument("--lateral-step",type=float,default=.35)
    parser.add_argument("--yaw-offsets",type=int,nargs="+",default=[-4,-2,0,2,4])
    parser.add_argument("--timeout",type=float,default=120.)
    args=parser.parse_args(argv)
    if args.timeout>120 or args.timeout<=0:
        parser.error("timeout must be in (0,120]")
    return run(args)


if __name__=="__main__":
    raise SystemExit(main())
