"""Offline connectivity diagnosis only; never feeds a relaxed map to planning."""
from pathlib import Path
from types import SimpleNamespace
import hashlib,json,math
import numpy as np
from scipy import ndimage
from PIL import Image
from arena_3d_v1.semantic_world import SemanticWorld,json_write
from arena_3d_v1.multires_grid import GridView,aggregate
from three_yd_v5.controller import inflate_circle
from three_yd_v5.topology import V5Topology

ROOT=Path(__file__).resolve().parents[1]

def main():
    out=ROOT/'report/connectivity';out.mkdir(parents=True,exist_ok=True)
    w=SemanticWorld(ROOT/'inputs/map.yaml',ROOT/'inputs/semantic.json',cache_root=out/'world')
    g=V5Topology(w,ROOT/'inputs/topology.json',out/'graph');v=GridView(w,3)
    queries=[]
    for raw in json.loads((ROOT/'inputs/queries.json').read_text())['scenarios']:
        q=SimpleNamespace(query_id=raw['id'],start=tuple(raw['start'][k] for k in ('x','y','yaw_rad')),goal=tuple(raw['goal'][k] for k in ('x','y','yaw_rad')))
        route=g.plan(q)
        queries.append((q,route))
    records={q.query_id:{'query_id':q.query_id,'regions':list(r.region_ids),'fine_connectivity':{}} for q,r in queries}
    radii=[0.,.225,.30,math.hypot(*w.vehicle[:2])]
    # The generous 8-neighbor component check can prove disconnection, but its
    # positive answer is not a corner-safe path or a rectangle certificate.
    for radius in radii:
        expanded=inflate_circle(w.hard,radius,.05) if radius else w.hard
        global_labels,_=ndimage.label(~expanded,np.ones((3,3),int))
        for q,route in queries:
            s=w.map.world_to_cell(*q.start[:2]);t=w.map.world_to_cell(*q.goal[:2])
            rec=records[q.query_id]
            global_connected=bool(global_labels[s] and global_labels[s]==global_labels[t])
            legal=np.isin(g.subregions,route.region_ids)
            local_labels,_=ndimage.label(~expanded&legal,np.ones((3,3),int))
            rec['fine_connectivity'][str(radius)]={'selected_regions_connected':bool(local_labels[s] and local_labels[s]==local_labels[t]),'all_physical_free_connected':global_connected,'start_component_cells':int(np.count_nonzero(local_labels==local_labels[s])) if local_labels[s] else 0}
            if abs(radius-radii[-1])<1e-9:
                rr,cc=np.indices(v.safe.shape);fr,fc=w.cells(v.world_points(rr.ravel(),cc.ravel()))
                valid=(fr>=0)&(fc>=0)&(fr<w.map.height)&(fc<w.map.width)
                free=np.zeros(len(fr),bool);free[valid]=~expanded[fr[valid],fc[valid]]
                coarse_labels,_=ndimage.label(free.reshape(v.safe.shape)&aggregate(legal,3),np.ones((3,3),int))
                cs,ct=v.cell(q.start),v.cell(q.goal)
                rec['coarse_center_sampling_connected']=bool(coarse_labels[cs] and coarse_labels[cs]==coarse_labels[ct])
            print(q.query_id,radius,rec['fine_connectivity'][str(radius)],flush=True)
    json_write(ROOT/'report/CIRCLE_DIAGNOSIS.json',{'map_sha256':w.map.sha256,'original_grid_png_sha256':hashlib.sha256((ROOT/'inputs/original_grid.png').read_bytes()).hexdigest(),'radii_m':radii,'records':list(records.values()),'diagnostic_only':True,'unknown_is_collision':True,'no_map_modification':True,'positive_connectivity_is_not_vehicle_certificate':True,'disconnected_circle_is_not_rectangle_impossibility':True})

if __name__=='__main__':main()
