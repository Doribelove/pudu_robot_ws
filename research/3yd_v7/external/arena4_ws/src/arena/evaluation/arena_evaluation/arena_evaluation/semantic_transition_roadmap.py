"""Bounded, query-generated directed SE(2) roadmap for transition contract R2."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import resource
import time

import cv2
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.spatial import cKDTree
from .topology import extract_skeleton

from .semantic_constraint_core import ConstraintWorld, dubins_edge, dense_interpolate
from .semantic_map import sha256_file
from .semantic_transition_contract import audit_transition_samples, resample_path


def _write(path, payload):
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, default=lambda v: v.item() if isinstance(v,np.generic) else list(v)) + "\n")


def sample_sites(world, maximum=130):
    """Deterministic medial/target sites bounded by the frozen path budget."""
    rr,cc=np.indices(world.master.shape)
    x=world.map.full_origin[0]+(cc+world.map.col0+.5)*world.map.resolution
    y=world.map.full_origin[1]+(world.map.full_height-rr-world.map.row0-.5)*world.map.resolution
    feasible=((world.master<253)&world.grids['allowed']&~world.grids['hard']
              &np.isin(world.grids['labels'],world.selected))
    feasible&=(np.hypot(x-world.start[0],y-world.start[1])+np.hypot(x-world.goal[0],y-world.goal[1])<=world.bound_length)
    # The circle is only a conservative site generator. Exact padded rectangle
    # sweep remains authoritative for every directed Dubins edge.
    feasible&=world.map.distance_m>math.hypot(.265,.225)+.025
    _,components=cv2.connectedComponents(feasible.astype(np.uint8),8)
    start_cell=world.map.world_to_cell(*world.start[:2])
    if start_cell is not None and components[start_cell]>0:
        feasible&=components==components[start_cell]
    target=feasible&world.grids['correct']&(world.grids['error']<=.5)
    skeleton=extract_skeleton(feasible)
    target_skeleton=extract_skeleton(target)
    pools=[]
    for mask in (target_skeleton,skeleton):
        rows,cols=np.where(mask)
        points=np.column_stack((x[rows,cols],y[rows,cols]))
        if len(points):
            # Globally anchored bins prevent sites depending on query ID.
            keys=np.floor(points/.65).astype(np.int64)
            _,ids=np.unique(keys,axis=0,return_index=True)
            pools.extend(points[np.sort(ids)].tolist())
    if not pools:
        return np.asarray([world.start[:2],world.goal[:2]],float),feasible,target
    pool=np.unique(np.asarray(pools),axis=0)
    selected=[np.asarray(world.start[:2]),np.asarray(world.goal[:2])]
    distances=np.minimum(np.linalg.norm(pool-selected[0],axis=1),np.linalg.norm(pool-selected[1],axis=1))
    while len(selected)<maximum and float(distances.max())>.40:
        index=int(np.argmax(distances)); point=pool[index]; selected.append(point)
        distances=np.minimum(distances,np.linalg.norm(pool-point,axis=1))
    return np.asarray(selected),feasible,target


def _poses(sites,world):
    tree=cKDTree(sites)
    poses=[tuple(world.start),tuple(world.goal)]; owners=[0,1]
    for i,xy in enumerate(sites):
        directions=set()
        _,neighbors=tree.query(xy,k=min(7,len(sites)))
        for j in np.atleast_1d(neighbors):
            if i==j: continue
            direction=math.atan2(sites[j,1]-xy[1],sites[j,0]-xy[0])
            b=int(round(direction/(2*math.pi)*48))%48
            directions.add(b); directions.add((b+24)%48)
        # Include cardinal headings to cross flat target strips exactly.
        directions.update((0,12,24,36))
        for b in sorted(directions):
            pose=(float(xy[0]),float(xy[1]),float((b*2*math.pi/48+math.pi)%(2*math.pi)-math.pi))
            if world.collision_free(np.asarray([pose])):
                poses.append(pose);owners.append(i)
    return poses,np.asarray(owners),tree


def _metrics(world,path):
    points=[dict(x=float(x),y=float(y),yaw=float(a)) for x,y,a in path]
    sampled,station=resample_path(points)
    array=np.asarray([[p['x'],p['y'],p['yaw']] for p in sampled])
    rows,cols,inside=world.cells(array)
    if not np.all(inside): return {'semantic_gate_passed':False,'failure_reason':'OUTSIDE_CROP'}
    return audit_transition_samples(path_length_m=float(station[-1]),station_m=station,
        lane_mask=np.isin(world.grids['labels'][rows,cols],world.selected),
        lane_error_m=world.grids['error'][rows,cols],lane_correct_side=world.grids['correct'][rows,cols],raw_xy=path[:,:2])


def _backtrace(previous,goal,start):
    path=[int(goal)]
    while path[-1]!=start and len(path)<=len(previous):
        parent=int(previous[path[-1]])
        if parent<0:return []
        path.append(parent)
    return list(reversed(path)) if path[-1]==start else []


def proper_crossings(path):
    xy=np.asarray(path)[::8,:2]
    if len(xy)<4:return 0
    a,b=xy[:-1],xy[1:];v=b-a
    count=0
    for i in range(len(a)-2):
        c,d=a[i+2:],b[i+2:];w=d-c
        cross1=v[i,0]*(c[:,1]-a[i,1])-v[i,1]*(c[:,0]-a[i,0])
        cross2=v[i,0]*(d[:,1]-a[i,1])-v[i,1]*(d[:,0]-a[i,0])
        cross3=w[:,0]*(a[i,1]-c[:,1])-w[:,1]*(a[i,0]-c[:,0])
        cross4=w[:,0]*(b[i,1]-c[:,1])-w[:,1]*(b[i,0]-c[:,0])
        count+=int(np.count_nonzero((cross1*cross2 < -1e-12)&(cross3*cross4 < -1e-12)))
    return count


def run(inputs,query,output,timeout=60.0,max_sites=100):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    source_hash=sha256_file(Path(__file__))
    started=time.monotonic();cpu=time.process_time();deadline=started+timeout
    world=ConstraintWorld(inputs,query)
    sites,feasible,target=sample_sites(world,max_sites)
    poses,owners,tree=_poses(sites,world)
    by_site={i:np.where(owners==i)[0].tolist() for i in range(len(sites))}
    rows=[];cols=[];edges=[];edge_keys={};rejected=0;attempts=0
    source_order=list(range(len(poses)))
    # Process exact endpoint connectors first, then short local graph edges.
    pending=[]
    for a in source_order:
        distances,neighbors=tree.query(np.asarray(poses[a][:2]),k=min(10,len(sites)))
        for distance,j in zip(np.atleast_1d(distances),np.atleast_1d(neighbors)):
            if int(j)==owners[a] or distance>7.0: continue
            choices=by_site[int(j)]
            bearing=math.atan2(sites[j,1]-poses[a][1],sites[j,0]-poses[a][0])
            choices=sorted(choices,key=lambda b:abs(math.atan2(math.sin(poses[b][2]-bearing),math.cos(poses[b][2]-bearing))))[:3]
            if int(j)==1: choices=list(set(choices+[1]))
            for b in choices:
                priority=float(distance)+(0 if a<2 or b<2 else 2)
                pending.append((priority,a,b))
    pending.sort()
    for _,a,b in pending:
        if time.monotonic()>deadline-8: break
        attempts+=1
        e=dubins_edge(poses[a],poses[b])
        if e is None or e.length>max(1.7*math.dist(poses[a][:2],poses[b][:2]),2.5):
            rejected+=1;continue
        if any(k!='S' and p>math.pi+1e-9 for k,p in zip(e.word,e.params)) or not world.validate_edge(e,dense=True):
            rejected+=1;continue
        edge_keys[(a,b)]=len(edges);edges.append(e);rows.append(a);cols.append(b)
    trials=[];best=None;best_score=(-1.,-1.);candidate_count=0;crossing_rejections=0
    pose_rows,pose_cols,inside=world.cells(np.asarray(poses))
    target_nodes=np.flatnonzero(inside&target[pose_rows,pose_cols])
    for target_weight in (0.,2.,5.,12.,30.,80.):
        if time.monotonic()>deadline: break
        values=np.asarray([e.length*(1+target_weight*(1-e.target/max(1,e.n))) for e in edges])
        graph=csr_matrix((values,(rows,cols)),shape=(len(poses),len(poses)))
        distance,previous=dijkstra(graph,directed=True,indices=0,return_predecessors=True)
        if not math.isfinite(distance[1]):
            trials.append({'weight':target_weight,'status':'DISCONNECTED'});continue
        reverse_distance,reverse_previous=dijkstra(graph.T,directed=True,indices=1,return_predecessors=True)
        candidates=[]
        for portal in np.r_[1,target_nodes]:
            if time.monotonic()>deadline-1:break
            if not math.isfinite(distance[portal]) or not math.isfinite(reverse_distance[portal]):continue
            front=_backtrace(previous,int(portal),0)
            back=list(reversed(_backtrace(reverse_previous,int(portal),1)))
            ids=front+back[1:]
            if len(ids)!=len(set(ids)) or len(ids)<2:continue
            trace=[edges[edge_keys[(a,b)]] for a,b in zip(ids,ids[1:])]
            if sum(e.length for e in trace)>world.bound_length:continue
            path=np.vstack((trace[0].start,*(e.samples for e in trace)))
            metric=_metrics(world,path);candidate_count+=1
            lane=metric.get('active_window',{}).get('classes',{}).get('lane',{})
            score=(float(metric['semantic_gate_passed']),float(lane.get('target_band_ratio',0.)),float(lane.get('correct_side_ratio',0.)))
            crossings=proper_crossings(path)
            if crossings:
                crossing_rejections+=1
                continue
            candidates.append((score,int(portal),trace,path,metric))
        candidates.sort(key=lambda v:v[0],reverse=True)
        for score,portal,trace,path,metric in candidates[:8]:
            if time.monotonic()>deadline:break
            crossings=proper_crossings(path)
            if crossings:
                crossing_rejections+=1
                trials.append({'weight':target_weight,'portal_node':portal,'gate_passed':False,
                               'failure_code':'GEOMETRIC_REVISIT','proper_self_crossings':crossings,
                               'semantic':metric})
                continue
            audit,path=world.audit(trace)
            safety=bool(audit.get('canonical',{}).get('final_valid_success') and audit.get('padded_effective_master_collision_free')
                        and audit.get('exact_endpoint_xy_yaw') and audit.get('trace_replay_exact')
                        and audit.get('maximum_control_curvature_1pm',math.inf)<=2.5
                        and not audit.get('no_stopping_goal_violation') and audit.get('arc_length_m',math.inf)<=world.bound_length)
            passed=bool(safety and metric['semantic_gate_passed'])
            trial={'weight':target_weight,'portal_node':portal,'safety_valid':safety,'gate_passed':passed,'semantic':metric,'audit':audit,
                   'proper_self_crossings':crossings,'repeated_graph_states':0}
            trials.append(trial)
            if safety and score[:2]>best_score:best=(trace,path,trial);best_score=score[:2]
            if passed:break
        if best and best[2]['gate_passed']:break
    result={'method':'query_generated_sparse_directed_se2_roadmap','architecture_id':'UNNAMED_OFFLINE_CANDIDATE',
        'query':query,'yaw_bins':48,'budget_s':timeout,'site_count':len(sites),'pose_count':len(poses),
        'edge_attempts':attempts,'edge_count':len(edges),'edge_rejections':rejected,
        'pending_edges':len(pending),'graph_complete':attempts==len(pending),
        'candidate_count':candidate_count,
        'crossing_rejections':crossing_rejections,
        'gate_passed':bool(best and best[2]['gate_passed']),
        'failure_code':'' if best and best[2]['gate_passed'] else ('SEMANTIC_GATE_FAILED' if best else 'NO_SAFE_PATH_IN_BOUNDED_ROADMAP'),
        'best':best[2] if best else None,'trials':trials,'wall_ms':(time.monotonic()-started)*1000,
        'cpu_ms':(time.process_time()-cpu)*1000,'peak_rss_bytes':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
        'input_npz_sha256':world.meta['npz_sha256'],'map_hash':world.meta['map_hash'],
        'semantic_map_hash':world.meta['semantic_map_hash'],'expected_master_hash':world.meta['expected_master_hash'],
        'query_hash':sha256_file(Path(inputs)/(query+'.json')),'source_hash':source_hash,
        'used_historical_paths':False,'online_planner_run':False,'relaxation_level':'R0',
        'infeasibility_scope':'bounded constructed graph only; no continuous-space impossibility claim'}
    if best:
        trace,path,_=best
        _write(output/'path.json',[dict(x=float(x),y=float(y),yaw=float(yaw)) for x,y,yaw in path])
        _write(output/'controls.json',{'edges':[e.certificate() for e in trace]})
    _write(output/'sites.json',sites.tolist());_write(output/'result.json',result)
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs',type=Path,required=True);parser.add_argument('--query',required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--timeout',type=float,default=60)
    parser.add_argument('--max-sites',type=int,default=100)
    args=parser.parse_args(argv)
    result=run(args.inputs,args.query,args.output,args.timeout,args.max_sites)
    print(json.dumps({k:v for k,v in result.items() if k not in ('best','trials')},indent=2))
    return 0 if result['gate_passed'] else 2


if __name__=='__main__':raise SystemExit(main())
