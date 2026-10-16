"""Classify the first V5-r1 rectangle-component break and try a local vehicle bridge."""
import copy,json,math
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
from PIL import Image

from arena_3d_v1.semantic_world import SemanticWorld,SweptChecker,json_write
from arena_3d_v1.multires_grid import GridView
from three_yd_v5.topology import V5Topology
from three_yd_v5.controller import V5Traffic
from three_yd_v4.vehicle_search import search_vehicle

ROOT=Path(__file__).resolve().parents[1]


def query(raw):
    return SimpleNamespace(query_id=raw['id'],start=tuple(raw['start'][k] for k in ('x','y','yaw_rad')),
                           goal=tuple(raw['goal'][k] for k in ('x','y','yaw_rad')))


def pixel_evidence(world,gray,segment):
    segment=np.asarray(segment,float)
    r,c=world.cells(segment)
    seed=np.zeros(gray.shape,np.uint8);cv2.polylines(seed,[np.column_stack((c,r)).astype(np.int32)],False,1,1)
    near=cv2.dilate(seed,np.ones((15,15),np.uint8))>0  # 0.35 m each side
    values=gray[near]
    occupied=values <= round((1-.65)*255)
    free=values >= round((1-.196)*255)
    unknown=~occupied&~free
    return {'sample_cells':int(len(values)),'occupied_cells':int(occupied.sum()),
            'unknown_cells':int(unknown.sum()),'free_cells':int(free.sum()),
            'gray_histogram':{str(int(v)):int(n) for v,n in zip(*np.unique(values,return_counts=True))}}


def angle_distance(a,b):return abs((a-b+math.pi)%(2*math.pi)-math.pi)


def poses_in_cell(world,view,cell,hard):
    center=view.world_points(np.asarray([cell[0]]),np.asarray([cell[1]]))[0]
    checker=SweptChecker(hard,world.map.resolution,world.map.origin,*world.vehicle[:2])
    poses=[]
    for dx in (-.05,0.,.05):
        for dy in (-.05,0.,.05):
            for k in range(24):
                yaw=2*math.pi*k/24;pose=np.asarray([[center[0]+dx,center[1]+dy,yaw]])
                if checker.check(pose):poses.append(pose[0].tolist())
    return poses


def try_pose_pairs(world,topology,traffic,view,bp,segment,target,budget=1.):
    hard=world.hard|~target
    source_poses=poses_in_cell(world,view,bp['source_cell'],hard)
    target_poses=poses_in_cell(world,view,bp['target_cell'],hard)
    delta=np.asarray(bp['target_pose'][:2])-np.asarray(bp['source_pose'][:2]);forward=math.atan2(delta[1],delta[0])
    pairs=[]
    for a in source_poses:
        for b in target_poses:
            score=angle_distance(a[2],forward)+angle_distance(b[2],forward)+.5*angle_distance(a[2],b[2])
            pairs.append((score,a,b))
    attempts=[];path=None;log={'success':False,'status':-6,'failure':'NO_FEASIBLE_ANCHOR_PAIR'}
    for _,a,b in sorted(pairs,key=lambda x:x[0])[:8]:
        local_segment=np.asarray([a[:2],*segment[1:-1],b[:2]],float)
        fake=SimpleNamespace(world=world,topology=topology,traffic=traffic,
            reference_plan=SimpleNamespace(world_xy=local_segment),_target_mask=lambda:target)
        local_query=SimpleNamespace(start=tuple(a),goal=tuple(b))
        candidate,trial=search_vehicle(fake,local_query,radius=1.2,budget=budget,max_nodes=150000,local=True)
        attempts.append({'start':a,'goal':b,**trial})
        if candidate is not None:path,log=candidate,trial;break
        if trial.get('closest_goal_distance_m',math.inf)<log.get('closest_goal_distance_m',math.inf):log=trial
    return path,{**log,'anchor_pose_counts':[len(source_poses),len(target_poses)],'attempts':attempts}


def main():
    source=ROOT/'report/static_probe/l2';out=ROOT/'report/breakpoints';out.mkdir(parents=True,exist_ok=True)
    world=SemanticWorld(ROOT/'inputs/map.yaml',ROOT/'inputs/semantic.json',cache_root=out/'world')
    topology=V5Topology(world,ROOT/'inputs/topology.json',out/'graph')
    view=GridView(world,3)
    gray=np.asarray(Image.open(ROOT/'inputs/original_grid.png').convert('L'))
    raw_queries=json.loads((ROOT/'inputs/queries.json').read_text())['scenarios']
    rows=[]
    for raw in raw_queries:
        q=query(raw);routes=topology.routes(q,limit=2)
        primary=routes[0]
        failed=source/('failed_reference_'+primary.signature[:12]+'.json')
        if not failed.exists():
            rows.append({'query_id':q.query_id,'already_circle_connected':True,
                         'route_candidates':[list(r.region_ids) for r in routes]});continue
        attempt=json.loads(failed.read_text())['activation_attempts'][0]
        bp=attempt.get('first_component_break',{})
        row={'query_id':q.query_id,'already_circle_connected':False,'primary_regions':list(primary.region_ids),
             'route_candidates':[list(r.region_ids) for r in routes],
             'different_alternative_available':len(routes)>1 and routes[1].region_ids!=routes[0].region_ids,
             'breakpoint':bp}
        if not bp.get('found'):
            row['local_bridge']={'success':False,'failure':'NO_BREAKPOINT'};rows.append(row);continue
        segment=np.asarray(bp['guide_segment_xy'],float)
        row['pixel_evidence']=pixel_evidence(world,gray,segment)
        labels=topology.subregions
        sr,sc=world.cells(np.asarray([bp['source_pose'][:2],bp['target_pose'][:2]],float))
        row['semantic_regions_at_anchors']=[int(labels[r,c]) for r,c in zip(sr,sc)]
        traffic=V5Traffic(world,topology,primary)
        endpoint_parking=traffic.endpoint_parking
        physical_regions=np.isin(labels,[rid for rid,desc in topology.regions.items()
            if desc['kind'] in ('lane','junction_area') or rid in endpoint_parking])
        # Match the actual L3 search mask here.  L3 may use any physical lane
        # or junction inside its bounded local window; route/traffic legality
        # is checked independently after a candidate is generated.  Restricting
        # this diagnostic to only the selected L1 region made boundary anchors
        # look infeasible even when the production L3 target accepted them.
        target=(~world.hard)&physical_regions
        path,log=try_pose_pairs(world,topology,traffic,view,bp,segment,target)
        row['local_bridge']=log
        # Attribution-only ablation. Unknown pixels are never cleared from the
        # published map, planner input, or strict acceptance path.
        occupied=gray<=round((1-.65)*255)
        ablation_world=copy.copy(world);ablation_world.hard=occupied
        ablation_target=(~occupied)&physical_regions
        ablation_path,ablation_log=try_pose_pairs(ablation_world,topology,traffic,view,bp,segment,ablation_target)
        row['unknown_free_local_ablation']={**ablation_log,'counts_as_strict_success':False}
        if ablation_path is not None:
            ar,ac=world.cells(ablation_path[:,:2]);visited=sorted(set(map(int,labels[ar,ac]))-{0})
            row['unknown_free_local_ablation']['visited_semantic_regions']=visited
            row['unknown_free_local_ablation']['outside_selected_route_regions']=sorted(set(visited)-set(primary.region_ids)-set(endpoint_parking))
            np.save(out/(q.query_id+'_unknown_free_bridge.npy'),ablation_path)
        if path is not None:
            row['local_bridge']['path_points']=len(path)
            row['local_bridge']['length_m']=float(np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1).sum())
            np.save(out/(q.query_id+'_bridge.npy'),path)
        rows.append(row);print(q.query_id,'routes',len(routes),'gap',round(bp['guide_gap_m'],2),'bridge',log['success'],log['status'],flush=True)
    json_write(out/'breakpoints.json',{'schema':'3YD-V5-breakpoint-diagnosis-v1','map_sha256':world.map.sha256,
        'local_radius_m':1.2,'rows':rows,'published_navigation_paths':False})


if __name__=='__main__':main()
