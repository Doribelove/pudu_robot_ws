"""Independent bounded piecewise-Dubins path witness diagnostic.

This is an offline feasibility generator, not an online planner. It minimizes
constraint violation of complete, newly integrated paths; historical paths are
never loaded. Output directories are write-once. Raw Dubins words and segment
lengths make every candidate exactly replayable.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import resource
import shutil
import sys
import time

import cv2
import numpy as np
from scipy.optimize import differential_evolution
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from .se2_semantic_guide import (
    EffectiveMasterCollisionChecker, SE2GuidePolicy, _advance,
    shortest_dubins_path, wrap_angle,
)
from .planner_benchmark.map_utils import HospitalMap


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    with Path(path).open("x", encoding="utf8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def integrate_waypoints(waypoints, radius=0.401, spacing=0.006):
    """Integrate analytic forward controls; never snap a generated endpoint."""
    path = [tuple(waypoints[0])]
    controls = []
    terminal_residuals = []
    length = 0.0
    for waypoint in waypoints[1:]:
        start = path[-1]
        word, params = shortest_dubins_path(start, waypoint, radius)
        for kind, parameter in zip(word, params):
            segment_length = float(parameter) * radius
            if segment_length < 1e-12:
                continue
            n = int(math.ceil(segment_length / spacing))
            initial = path[-1]
            values = np.arange(1, n + 1, dtype=float) * float(parameter) / n
            x, y, yaw = initial
            if kind == "S":
                distances = values * radius
                samples = np.column_stack((x + distances * math.cos(yaw),
                                           y + distances * math.sin(yaw),
                                           np.full(n, yaw)))
            else:
                sign = 1.0 if kind == "L" else -1.0
                angles = yaw + sign * values
                samples = np.column_stack((x + sign * radius * (np.sin(angles)-math.sin(yaw)),
                                           y + sign * radius * (math.cos(yaw)-np.cos(angles)),
                                           (angles + math.pi) % (2*math.pi)-math.pi))
            controls.append({"kind": kind, "parameter": float(parameter),
                             "length_m": segment_length, "radius_m": radius,
                             "curvature_1pm": 0.0 if kind == "S" else sign/radius,
                             "start_pose": list(initial), "start_index": len(path)-1,
                             "end_index": len(path)+n-1})
            path.extend(map(tuple, samples.tolist()))
            length += segment_length
        residual = math.hypot(path[-1][0]-waypoint[0], path[-1][1]-waypoint[1])
        residual_yaw = abs(wrap_angle(path[-1][2]-waypoint[2]))
        terminal_residuals.append([residual, residual_yaw])
        if residual > 1e-8 or residual_yaw > 1e-8:
            raise ValueError("analytic endpoint integration residual")
    # Exact endpoint representations only after documenting roundoff-sized
    # residuals, not geometrical reattachment or yaw quantization.
    path[0] = tuple(waypoints[0])
    path[-1] = tuple(waypoints[-1])
    return np.asarray(path), controls, length, terminal_residuals


class IndependentWitness:
    def __init__(self, prefix):
        self.prefix = Path(prefix)
        self.meta = json.loads(self.prefix.with_suffix(".json").read_text())
        if digest(self.prefix.with_suffix(".npz")) != self.meta["npz_sha256"]:
            raise ValueError("input npz hash mismatch")
        bundle = np.load(self.prefix.with_suffix(".npz"))
        self.arrays = {k: bundle[k] for k in bundle.files}
        info = self.meta["map"]
        self.map = HospitalMap(
            yaml_path=Path(info["image_path"]).with_suffix(".yaml"),
            image_path=Path(info["image_path"]), resolution=info["resolution"],
            origin=tuple(info["origin"]), width=info["width"], height=info["height"],
            occupancy=self.arrays["occupancy"],
            distance_m=np.zeros_like(self.arrays["error"]),
        )
        self.policy = SE2GuidePolicy()
        self.checker = EffectiveMasterCollisionChecker(self.map, self.arrays["master"], self.policy)
        self.start = tuple(self.meta["query"]["start"])
        self.goal = tuple(self.meta["query"]["goal"])
        self.limit = max(40.0, 4*math.dist(self.start[:2], self.goal[:2]))
        self.rows = []
        self.best = None
        self.best_score = math.inf
        self.counter = 0
        self.full_audit_count = 0

    def metrics(self, path, length):
        xs, ys = path[:, 0], path[:, 1]
        cols = np.floor((xs-self.map.origin[0])/self.map.resolution).astype(int)
        rows = self.map.height-1-np.floor((ys-self.map.origin[1])/self.map.resolution).astype(int)
        inside = (cols>=0)&(cols<self.map.width)&(rows>=0)&(rows<self.map.height)
        if not np.all(inside):
            return None
        idx = rows, cols
        arr = self.arrays
        lane = np.isin(arr["labels"][idx], self.meta["selected_lane_labels"])
        correct = lane & arr["correct"][idx]
        target = correct & (arr["error"][idx]<=.5)
        collision_center = arr["master"][idx]>=253
        clearance = self.checker.distance_m[idx]
        close = clearance < self.checker.circumscribed_radius + .05*math.sqrt(2)/2
        delta = np.diff(path, axis=0)
        ds = np.linalg.norm(delta[:, :2], axis=1)
        dyaw = (delta[:, 2]+math.pi)%(2*math.pi)-math.pi
        proj = delta[:,0]*np.cos(path[:-1,2])+delta[:,1]*np.sin(path[:-1,2])
        # Uniform arclength replay means semantic sample density cannot be
        # intentionally adjusted by motion type or by candidate acceptance.
        return {
            "path_length_m": float(length), "point_count": len(path),
            "lane_correct_side_ratio": float(np.mean(correct)),
            "lane_target_error_p50_m": float(np.median(arr["error"][idx][np.isfinite(arr["error"][idx])])) if np.any(np.isfinite(arr["error"][idx])) else 1e6,
            "lane_target_band_ratio": float(np.mean(target)),
            "same_lane_violations": int(np.count_nonzero(~lane)),
            "center_reject_count": int(np.count_nonzero(collision_center)),
            "close_footprint_pose_count": int(np.count_nonzero(close)),
            "max_curvature_from_yaw_chord_1pm": float(np.max(np.abs(dyaw)/np.maximum(ds,1e-20))),
            "reverse_distance_m": float(ds[proj < -1e-9].sum()),
            "in_place_rotation_count": int(np.count_nonzero((ds<1e-9)&(np.abs(dyaw)>1e-8))),
            "max_translation_spacing_m": float(ds.max()),
            "max_yaw_step_deg": float(np.rad2deg(np.abs(dyaw)).max()),
            "backward_station_distance_m": float(np.maximum(-delta[:,1],0).sum()),
            "minimum_approximate_collision_margin_m": float(clearance.min()-self.checker.circumscribed_radius),
            "conservative_clearance_deficit_integral": float(np.maximum(.385-clearance,0).sum()*.006),
        }

    def full_audit(self, path, controls, metrics):
        self.full_audit_count += 1
        valid, margin, failed = self.checker.sweep_status(path)
        metrics = dict(metrics)
        metrics.update({"path_collision_free": bool(valid), "first_collision_index": failed,
                        "maximum_control_curvature_1pm": max(abs(x["curvature_1pm"]) for x in controls),
                        "exact_start_pose": bool(np.array_equal(path[0], self.start)),
                        "exact_goal_pose": bool(np.array_equal(path[-1], self.goal)),
                        "endpoint_no_stopping_violation": bool(self.arrays["no_stopping"][self.map.world_to_cell(*self.goal[:2])])})
        metrics["gate_passed"] = bool(
            valid and not metrics["same_lane_violations"]
            and metrics["lane_correct_side_ratio"]>=.8
            and metrics["lane_target_error_p50_m"]<=.5
            and metrics["lane_target_band_ratio"]>.5
            and metrics["max_curvature_from_yaw_chord_1pm"]<=2.5
            and not metrics["reverse_distance_m"]
            and not metrics["in_place_rotation_count"]
            and not metrics["endpoint_no_stopping_violation"]
            and metrics["exact_start_pose"] and metrics["exact_goal_pose"])
        return metrics

    def evaluate(self, waypoints, require_monotonic=True):
        self.counter += 1
        path, controls, length, residuals = integrate_waypoints(waypoints)
        metrics = self.metrics(path, length)
        if metrics is None:
            return 1e5
        side = metrics["lane_correct_side_ratio"]
        band = metrics["lane_target_band_ratio"]
        error = metrics["lane_target_error_p50_m"]
        score = (1000*(max(0.,.8-side)+max(0.,.50001-band))
                 + 100*max(0.,error-.5)+length*.0001
                 + 10000*metrics["conservative_clearance_deficit_integral"]
                 + 100*metrics["same_lane_violations"]
                 + 1000*max(0.,length-self.limit))
        if require_monotonic:
            score += 1000*metrics["backward_station_distance_m"]
        if score < self.best_score:
            audited = self.full_audit(path, controls, metrics)
            audited["search_family_station_constraint_passed"] = bool(
                not require_monotonic or metrics["backward_station_distance_m"] <= 1e-8)
            audited["gate_passed"] = bool(audited["gate_passed"] and audited["search_family_station_constraint_passed"])
            if not audited["path_collision_free"]:
                score += 1000
            if score < self.best_score:
                self.best_score = score
                self.best = (waypoints, path, controls, residuals, audited)
                self.rows.append({"candidate":self.counter,"score":score,**audited})
                print(json.dumps({"candidate":self.counter,"score":score,"metrics":audited}), flush=True)
        return score

    def search(self, timeout, seed, variant):
        begin=time.monotonic(); cpu=time.process_time()
        # Two target-side attachments with exact 48-bin reference headings.
        # Bounds define an acyclic north-progress family for this positive query.
        if variant == "two_attachment_monotone":
            bounds=[(-22.65,-21.75),(-19.0,-17.8),(8,16),
                    (-22.65,-21.75),(-12.1,-10.85),(8,18)]
            def unpack(v):
                return [self.start,(v[0],v[1],round(v[2])*math.pi/24),
                        (v[3],v[4],round(v[5])*math.pi/24),self.goal]
        elif variant == "four_attachment_monotone":
            bounds=[(-23.1,-21.85),(-19.0,-18.2),(5,16),
                    (-22.65,-21.75),(-17.7,-16.6),(7,16),
                    (-22.8,-21.7),(-15.8,-13.5),(7,17),
                    (-22.65,-21.75),(-12.2,-10.9),(8,20)]
            def unpack(v):
                return [self.start]+[(v[i],v[i+1],round(v[i+2])*math.pi/24)
                                    for i in range(0,len(v),3)]+[self.goal]
        elif variant == "four_attachment_extended":
            bounds=[(-22.35,-21.75),(-24.,-19.8),(10,15),
                    (-23.0,-22.45),(-18.2,-17.6),(10,14),
                    (-22.2,-21.85),(-15.8,-15.0),(9,14),
                    (-22.7,-21.9),(-9.4,-7.7),(14,28)]
            def unpack(v):
                return [self.start]+[(v[i],v[i+1],round(v[i+2])*math.pi/24)
                                    for i in range(0,len(v),3)]+[self.goal]
        elif variant == "north_boundary_detour":
            bounds=[(-24.5,-23.7),(-18.4,-17.0),(9,14),
                    (-24.6,-23.8),(-6.4,-5.2),(10,17),
                    (-26.7,-24.8),(-2.8,-1.2),(16,28),
                    (-26.5,-25.5),(-6.,-4.2),(30,39)]
            def unpack(v):
                return [self.start]+[(v[i],v[i+1],round(v[i+2])*math.pi/24)
                                    for i in range(0,len(v),3)]+[self.goal]
        else:
            raise ValueError(variant)
        def objective(v):
            if time.monotonic()-begin>=timeout:
                raise TimeoutError()
            if self.counter>=1_000_000:
                raise RuntimeError("candidate budget exceeded")
            if resource.getrusage(resource.RUSAGE_SELF).ru_maxrss>2048*1024:
                raise MemoryError("frozen RSS budget exceeded")
            return self.evaluate(unpack(v),require_monotonic=variant.endswith("monotone"))
        stop="OPTIMIZER_COMPLETED_BOUNDED_FAMILY"
        try:
            differential_evolution(objective,bounds,seed=seed,maxiter=10000,
                                   popsize=12,polish=False,tol=1e-9,
                                   updating="immediate",workers=1)
        except TimeoutError:
            stop="SEARCH_TIMEOUT_NOT_INFEASIBILITY_PROOF"
        return {"wall_ms":(time.monotonic()-begin)*1000,
                "cpu_ms":(time.process_time()-cpu)*1000,"stop_reason":stop,
                "candidate_count":self.counter,"full_audit_count":self.full_audit_count,
                "peak_rss_bytes":resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024}

    def save(self, output, stats):
        write_json(output/"performance.json",stats)
        write_json(output/"improvement_trace.json",self.rows)
        if self.best is None:
            write_json(output/"result.json",{"gate_passed":False,"reason":"NO_CANDIDATE"})
            return
        waypoints,path,controls,residuals,metrics=self.best
        write_json(output/"result.json",metrics)
        write_json(output/"path.json",[{"x":float(p[0]),"y":float(p[1]),"yaw":float(p[2])} for p in path])
        write_json(output/"certificate.json",{
            "method":"new_piecewise_analytic_forward_Dubins",
            "raw_waypoints":[list(p) for p in waypoints],"controls":controls,
            "integrated_waypoint_roundoff_residuals_xy_yaw":residuals,
            "radius_m":.401,"replay_spacing_m":.006,"heading_attachment_bins":48,
            "input_json_sha256":digest(self.prefix.with_suffix(".json")),
            "input_npz_sha256":digest(self.prefix.with_suffix(".npz")),
            "input_expected_master_hash":self.meta["expected_master_hash"],
            "result":metrics,"scope":"offline_candidate_requires_parent_canonical_audit",
            "semantic_sampling":"uniform_arclength_at_most_0.006_m_all_motion_types",
            "roundoff_endpoint_replacement_max_m":1e-8,
            "online_ACK":"NOT_APPLICABLE_OFFLINE","new_experiment":True,
        })
        with (output/"runs.csv").open("x",newline="") as stream:
            row={"query_id":self.meta["query"]["query_id"],**metrics,**stats}
            writer=csv.DictWriter(stream,fieldnames=list(row));writer.writeheader();writer.writerow(row)
        image=cv2.imread(str(self.map.image_path),cv2.IMREAD_GRAYSCALE)
        image=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        cs=np.asarray([(self.map.world_to_cell(*p[:2])[1],self.map.world_to_cell(*p[:2])[0]) for p in path])
        target=self.arrays["correct"]&(self.arrays["error"]<=.5)&np.isin(self.arrays["labels"],self.meta["selected_lane_labels"])
        image[target]=(140,220,140)
        cv2.polylines(image,[cs.astype(np.int32)],False,(0,0,235),2)
        for p,c in [(self.start,(255,0,0)),(self.goal,(180,0,180))]:
            row,col=self.map.world_to_cell(*p[:2]); cv2.circle(image,(col,row),5,c,-1)
        x0,x1=max(0,cs[:,0].min()-40),min(self.map.width,cs[:,0].max()+41)
        y0,y1=max(0,cs[:,1].min()-40),min(self.map.height,cs[:,1].max()+41)
        cv2.imwrite(str(output/"overlay.png"),image[y0:y1,x0:x1])


def run_geometry_scan(solver, output):
    """Record discrete center reachability and exact-footprint slit tests.

    These are necessary diagnostics, not an SE(2) witness and not a continuous
    infeasibility proof. A center shortest path can contain return travel.
    """
    from .semantic_constraint_core import ConstraintWorld, dubins_edge
    started=time.monotonic(); cpu=time.process_time()
    w=solver
    all_lane=np.isin(w.arrays["labels"],w.meta["selected_lane_labels"])
    rr,cc=np.where(all_lane)
    r0,r1=int(rr.min()),int(rr.max()+1);c0,c1=int(cc.min()),int(cc.max()+1)
    lane=all_lane[r0:r1,c0:c1]
    master=w.arrays["master"][r0:r1,c0:c1]
    clear=w.checker.distance_m[r0:r1,c0:c1]
    target=(w.arrays["correct"]&(w.arrays["error"]<=.5))[r0:r1,c0:c1]
    sr,sc=w.map.world_to_cell(*w.start[:2]);gr,gc=w.map.world_to_cell(*w.goal[:2])
    s=(sr-r0,sc-c0);g=(gr-r0,gc-c0)
    summary=[];paths=[]
    for threshold in (0.,.225,.275,.348):
        mask=(lane&(master<253)&(clear>threshold)).astype(np.uint8)
        for connectivity in (4,8):
            n,components=cv2.connectedComponents(mask,connectivity=connectivity)
            sid=int(components[s]);gid=int(components[g])
            candidates=(components==sid)&target
            count=int(candidates.sum())
            row={"center_clearance_gt_m":threshold,"connectivity":connectivity,
                 "components":n-1,"start_component":sid,"goal_component":gid,
                 "reachable_target_cell_count":count}
            index=np.full(mask.shape,-1,dtype=np.int32)
            rs,cs=np.where(mask)
            index[rs,cs]=np.arange(len(rs),dtype=np.int32)
            sources=[];destinations=[];weights=[]
            moves=[(0,1),(1,0)]+([(1,1),(1,-1)] if connectivity==8 else [])
            for dr,dc in moves:
                ar,ac=rs+dr,cs+dc
                inside=(ar>=0)&(ar<mask.shape[0])&(ac>=0)&(ac<mask.shape[1])
                ids=np.where(inside)[0]
                dest=index[ar[ids],ac[ids]]; valid=dest>=0
                src=ids[valid];dst=dest[valid]
                sources.extend((src,dst));destinations.extend((dst,src))
                val=np.full(len(src),.05*math.hypot(dr,dc))
                weights.extend((val,val))
            graph=csr_matrix((np.concatenate(weights),(np.concatenate(sources),np.concatenate(destinations))),shape=(len(rs),len(rs)))
            distances,pred=dijkstra(graph,indices=[index[s],index[g]],return_predecessors=True)
            target_ids=index[candidates]
            costs=distances[0,target_ids]+distances[1,target_ids]
            if len(target_ids) and np.any(np.isfinite(costs)):
                best=int(target_ids[int(np.argmin(costs))])
                end=(int(rs[best]+r0),int(cs[best]+c0))
                bestxy=w.map.cell_to_world(end)
                def chain(origin_index):
                    node=best; chain=[]
                    while node>=0:
                        chain.append(w.map.cell_to_world((int(rs[node]+r0),int(cs[node]+c0))))
                        node=int(pred[origin_index,node])
                    return chain
                first=chain(0)[::-1];second=chain(1)
                row.update({"shortest_start_target_goal_center_length_m":float(costs.min()),
                            "nearest_target_xy":list(bestxy),
                            "start_target_center_length_m":float(distances[0,best]),
                            "goal_target_center_length_m":float(distances[1,best])})
                paths.append({**row,"path_xy":first+second[1:],"is_SE2_witness":False})
            summary.append(row)
    core=ConstraintWorld(w.prefix.parent,w.meta["query"]["query_id"])
    gap_rows=[]
    for ycenter in (-19.233104,-20.933104,-24.283104,-25.433104,-27.733104):
        successful=[]; attempts=[]
        for y in ycenter+np.linspace(-.03,.03,61):
            path=np.column_stack((np.linspace(-24,-22.1,381),np.full(381,y),np.zeros(381)))
            valid=core.collision_free(path)
            attempts.append({"y":float(y),"collision_free":bool(valid)})
            if valid:successful.append(float(y))
        gap_rows.append({"gap_center_y":ycenter,"x_interval":[-24,-22.1],
                         "yaw_rad":0.,"translation_step_m":.005,
                         "y_scan_half_range_m":.03,"y_scan_step_m":.001,
                         "successful_crossings":successful,"attempts":attempts})
    write_json(output/"center_connectivity_and_shortest_paths.json",summary)
    write_json(output/"center_paths_not_witnesses.json",paths)
    write_json(output/"padded_rectangle_gap_crossing_scan.json",gap_rows)
    write_json(output/"geometry_summary.json",{
        "query_id":w.meta["query"]["query_id"],
        "all_gap_straight_crossings_rejected":all(not v["successful_crossings"] for v in gap_rows),
        "gap_count":len(gap_rows),"crossing_candidates":sum(len(v["attempts"]) for v in gap_rows),
        "footprint":"exact rectangle SAT (+/-.265,+/-.225)",
        "center_graph_scope":"original .05 m cells in same lane, expected master<253; 8-neighbor allows diagonal center transitions",
        "not_a_continuous_infeasibility_proof":True,
        "wall_ms":(time.monotonic()-started)*1000,"cpu_ms":(time.process_time()-cpu)*1000,
        "peak_rss_bytes":resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
        "input_npz_sha256":w.meta["npz_sha256"],"expected_master_hash":w.meta["expected_master_hash"],
    })
    print(json.dumps(summary),flush=True)


def run_auxiliary(solver, output, timeout=120.):
    """Fresh controls for the other two queries, only from frozen input fields."""
    from .semantic_constraint_core import ConstraintWorld, dubins_edge
    w=ConstraintWorld(solver.prefix.parent,solver.meta["query"]["query_id"])
    start=time.monotonic();cpu=time.process_time();trials=[];best=None
    def variants():
        if w.query.query_id=="r3-mirror-2-negative":
            for x in np.arange(-27.35,-26.65,.05):
                for y1 in (-11.5,-12.,-12.5,-13.):
                    for y2 in (-17.,-18.,-18.5):
                        yield [w.start,(float(x),y1,-math.pi/2),(float(x),y2,-math.pi/2),w.goal]
        else:
            raw=np.asarray(w.meta["guide_polylines_world"][0])
            for stride in (3,4,5,6,7,8):
                for lateral in (0.,.10,.20,-.10,-.20):
                    chosen=raw[::stride].copy();chosen[:,0]+=lateral
                    poses=[]
                    for i,xy in enumerate(chosen):
                        before=chosen[max(i-1,0)];after=chosen[min(i+1,len(chosen)-1)]
                        angle=math.atan2(*(after-before)[::-1])
                        yaw=round(angle/(math.pi/24))*math.pi/24
                        poses.append((float(xy[0]),float(xy[1]),yaw))
                    yield [w.start]+poses+[w.goal]
    for poses in variants():
        if time.monotonic()-start>timeout:break
        edges=[dubins_edge(a,b) for a,b in zip(poses,poses[1:])]
        valid=all(w.validate_edge(e,dense=True) for e in edges)
        trial={"index":len(trials),"raw_waypoints":[list(p) for p in poses],"all_edges_valid":valid}
        if valid:
            n=1+sum(e.n for e in edges); c=sum(e.correct for e in edges); t=sum(e.target for e in edges)
            initial=w.semantic_counts([w.start]);c+=initial[1];t+=initial[2]
            trial.update({"lane_sample_count":n,"correct_samples":c,"target_samples":t,
                          "side_ratio":c/n,"target_ratio":t/n})
            if 5*c>=4*n and 2*t>n:
                audit,path=w.audit(edges)
                trial["canonical_and_dense_gate_passed"]=audit["gate_passed"]
                if audit["gate_passed"]:
                    best=(edges,audit,path);trials.append(trial);break
        trials.append(trial)
    write_json(output/"trials.json",trials)
    write_json(output/"performance.json",{"candidate_count":len(trials),"wall_ms":(time.monotonic()-start)*1000,
                                        "cpu_ms":(time.process_time()-cpu)*1000,
                                        "peak_rss_bytes":resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
                                        "stop_reason":"WITNESS" if best else "BOUNDED_FAMILY_NO_WITNESS"})
    if best:
        edges,audit,path=best
        write_json(output/"audit.json",audit)
        write_json(output/"certificate.json",{"edges":[e.certificate() for e in edges],"query":w.meta["query"],
                    "input_npz_sha256":w.meta["npz_sha256"],"newly_generated":True})
        write_json(output/"path.json",[{"x":float(x),"y":float(y),"yaw":float(yaw)} for x,y,yaw in path])
        print(json.dumps(audit),flush=True)
    else:
        write_json(output/"audit.json",{"gate_passed":False,"no_continuous_infeasibility_claim":True})
        print(json.dumps({"trials":len(trials),"gate_passed":False}),flush=True)
    return bool(best)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-prefix",required=True,type=Path)
    parser.add_argument("--output-dir",required=True,type=Path)
    parser.add_argument("--variant",choices=["two_attachment_monotone","four_attachment_monotone","four_attachment_extended","north_boundary_detour","geometry_scan","auxiliary_guide"],default="two_attachment_monotone")
    parser.add_argument("--seed",type=int,choices=range(4),default=0)
    parser.add_argument("--timeout",type=float,default=120.)
    args=parser.parse_args(argv)
    if not 0<args.timeout<=120:
        parser.error("timeout must be <=120 seconds")
    args.output_dir.mkdir(parents=True,exist_ok=False)
    write_json(args.output_dir/"protocol.json",{
        "variant":args.variant,"seed":args.seed,"timeout_s":args.timeout,
        "family":"piecewise_dubins_new_control_integration",
        "target_query":args.input_prefix.name,"radius_m":.401,
        "purpose":"bounded_independent_stage1_feasibility_diagnostic",
        "search_semantics":"path_constraints_not_costmap_weight_scan",
        "max_candidates":1_000_000,"max_rss_mib":2048,
        "require_nondecreasing_north_route_station":args.variant.endswith("monotone"),
        "source_sha256":digest(__file__),
        "input_json_sha256":digest(args.input_prefix.with_suffix(".json")),
        "command_argv":sys.argv,
    })
    shutil.copy2(__file__,args.output_dir/"source_snapshot.py")
    solver=IndependentWitness(args.input_prefix)
    if args.variant=="geometry_scan":
        run_geometry_scan(solver,args.output_dir)
        return 0
    if args.variant=="auxiliary_guide":
        return 0 if run_auxiliary(solver,args.output_dir,args.timeout) else 2
    stats=solver.search(args.timeout,args.seed,args.variant)
    solver.save(args.output_dir,stats)
    print(json.dumps(stats),flush=True)
    return 0 if solver.best and solver.best[-1]["gate_passed"] else 2


if __name__=="__main__":
    raise SystemExit(main())
