"""L1/L2-only diagnostic: no ROS process or vehicle search."""
import argparse,json,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from arena_3d_v1.semantic_world import SemanticWorld,json_write
from three_yd_v5.topology import V5Topology
from three_yd_v5.controller import V5Controller

ROOT=Path(__file__).resolve().parents[1]
if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--map',default='map.yaml')
    parser.add_argument('--output',default='report/static_probe')
    args=parser.parse_args()
    out=ROOT/args.output;out.mkdir(parents=True,exist_ok=True)
    world=SemanticWorld(ROOT/'inputs'/args.map,ROOT/'inputs/semantic.json',cache_root=out/'world')
    topology=V5Topology(world,ROOT/'inputs/topology.json',out/'graph');views={};rows=[]
    for qd in json.loads((ROOT/'inputs/queries.json').read_text())['scenarios']:
        query=SimpleNamespace(query_id=qd['id'],start=tuple(qd['start'][k] for k in ('x','y','yaw_rad')),goal=tuple(qd['goal'][k] for k in ('x','y','yaw_rad')))
        tick=time.monotonic();route=topology.plan(query);task=topology.task(route)
        json_write(out/(qd['id']+'_l1.json'),task)
        row={'query':qd['id'],'regions':list(route.region_ids),'l1_points':len(task['right_reference_polyline'])}
        try:
            c=V5Controller(world,topology,route,cache_root=out/'l2',views=views)
            np.save(out/(qd['id']+'_l2.npy'),c.reference_plan.world_xy)
            row.update(success=True,l2_points=len(c.reference_plan.world_xy),corners=len(c.corner_records),mode=c.reference_mode,rectangle_bridge_cells=len(c.rectangle_bridge_cells))
        except Exception as error:row.update(success=False,error=str(error))
        row['seconds']=time.monotonic()-tick;rows.append(row);print(row,flush=True)
    json_write(out/'summary.json',rows)
