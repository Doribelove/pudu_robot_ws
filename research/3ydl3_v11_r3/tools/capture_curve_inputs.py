"""Explicit diagnostic-only snapshots for offline curve investigations."""
import json
from pathlib import Path
import time
import numpy as np


def install(directory):
    from three_yd_v5 import curve_l3, curve_backend
    root=Path(directory);root.mkdir(parents=True,exist_ok=False)
    from three_yd_v5.controller import V5Controller
    original_audit=V5Controller._audit
    original=curve_l3.optimize
    backend=curve_backend.plan
    context={};counter=0

    def plan(controller,query,*args,**kwargs):
        context['query_id']=query.query_id
        context['controller']=controller
        return backend(controller,query,*args,**kwargs)

    def optimize(world,mask,guide,start,goal,deadline,config=curve_l3.CONFIG):
        nonlocal counter
        path=root/f'{context.get("query_id","unknown")}_{counter:03d}'
        counter+=1
        c=context['controller']
        np.savez_compressed(path.with_suffix('.npz'),hard=world.hard,mask=mask,
                            guide=guide,start=start,goal=goal,reference=c.reference_plan.world_xy,
                            physical=c.plan.corridor_mask,search=c._search_mask,
                            labels=c.topology.subregions,raw_reference=c.raw_reference_xy)
        data={'query_id':context.get('query_id'),'resolution':world.map.resolution,
              'origin':list(world.map.origin),'vehicle':list(world.vehicle),
              'config':config,'diagnostic_only':True,'remaining_s':deadline-time.monotonic(),
              'regions':list(c.topology_route.region_ids),
              'region_kinds':{int(k):v['kind'] for k,v in c.topology.regions.items()}}
        path.with_suffix('.json').write_text(json.dumps(data,indent=2)+'\n')
        result=original(world,mask,guide,start,goal,deadline,config)
        path.with_suffix('.out.json').write_text(json.dumps(result[2],indent=2)+'\n')
        return result

    def audit(controller,query,poses,*args,**kwargs):
        result=original_audit(controller,query,poses,*args,**kwargs)
        if result[-1] and hasattr(controller.traffic,'_wrong_direction_indices'):
            dense,labels,bad=controller.traffic._wrong_direction_indices(poses)
            path=root/f'{query.query_id}_traffic_{time.monotonic_ns()}'
            np.savez_compressed(path.with_suffix('.npz'),poses=dense,labels=labels,bad=bad,
                                roads=np.asarray([[rid,*d] for rid,d in controller.traffic.roads]))
            path.with_suffix('.json').write_text(json.dumps({'failures':result[-1],
                'traffic':result[2],'endpoint_access':controller.traffic.task['endpoint_access']},
                default=lambda x:x.tolist() if isinstance(x,np.ndarray) else x.item(),indent=2)+'\n')
        return result

    V5Controller._audit=audit
    curve_l3.optimize=optimize
    curve_backend.plan=plan
