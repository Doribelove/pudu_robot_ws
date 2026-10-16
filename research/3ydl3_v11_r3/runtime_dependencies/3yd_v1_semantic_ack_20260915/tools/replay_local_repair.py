"""Offline replay of stored rejected Smac poses; not an end-to-end ROS result."""
import json,time,os
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from arena_3d_v1.semantic_world import WORK,SemanticWorld,json_write,digest
from arena_3d_v1.multires_topology import SemanticTopology
from arena_3d_v1.curvature_repair import TurnaroundController
from arena_evaluation.path_audit import PathAuditor
from arena_evaluation.planner_benchmark.models import Query
from arena_evaluation.unified_four_backends_smoke import MapContext,PlanResult,backend_availability,_annotate_smac_points

os.sched_setaffinity(0,{14});records=[];output=WORK/'report'/f'local_repair_replay_{time.time_ns()}.json'
for root in sorted((WORK/'results').glob('holdout_*_r1')):
    protocol=json.loads((root/'protocol.json').read_text());qd=protocol['queries'][0]
    q=Query(qd['query_id'],tuple(qd['start']),tuple(qd['goal']),seed=qd['seed'])
    row=json.loads((root/'runs.jsonl').read_text().splitlines()[0]);case=Path(row['case'])
    w=SemanticWorld(root/'inputs/map.yaml',root/'inputs/semantic.json');g=SemanticTopology(w,root/'preparation_cache/graph',portal_samples=3)
    route=g.plan(q,excluded_portal_transitions={(-1,-2)})
    c=TurnaroundController(w,g,route,cache_root=WORK/'cache/replay'/root.name)
    assert digest(c._target_mask())==row['hard_corridor_hash']
    auditor=PathAuditor(MapContext('replay',w.map,w.safe,w.map.distance_m,w.map.sha256,'',root/'inputs/map.yaml'),source_commit='offline_replay')
    spec=backend_availability()['hybrid_astar'];p=np.load(case/'path.npy')
    class Session:
        bound_vehicle=w.vehicle;local_map_update_strategy='roi_ack';full_grid_settle_cycles=0
        def set_semantic_costmap(self,cost):pass
        def plan(self,*a,**k):
            points=_annotate_smac_points([dict(x=x,y=y,yaw=yaw) for x,y,yaw in p],spec,'l3_prime_smac_hybrid')
            return PlanResult(planner_success=True,points=points,source='l3_prime_smac_hybrid',planner_backend=spec.backend,backend_version=spec.version,diagnostics=row['l3'])
    started=time.monotonic();out=c.plan_l3(q,Session(),auditor,spec)
    record={'case':root.name,'success':out['success'],'elapsed_ms':(time.monotonic()-started)*1000,'repair':out.get('local_curvature_repair'),'audit':out.get('independent_audit'),'replayed_recorded_map_hash':row['hard_corridor_hash']}
    records.append(record);json_write(output,records)
    print(root.name,out['success'],out.get('local_curvature_repair',{}).get('wall_ms'),out.get('local_curvature_repair',{}).get('failure'),flush=True)
