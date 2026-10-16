"""Recompute L2 only after a proven out-of-access parking shortcut."""
import math,time
import cv2
import numpy as np


def repair_reference(c):
    if c.generic_mode:return
    goal=c.topology.contexts[c.topology_route.signature]['endpoint_access']['goal']
    rid=int(goal['semantic_region_id'])
    if c.topology.regions[rid]['kind']!='parking_area':return
    p=c.reference_plan.world_xy;rr,cc=c.world.cells(p)
    rows=c.plan.corridor_mask.shape[0];cols=c.plan.corridor_mask.shape[1]
    valid=(rr>=0)&(cc>=0)&(rr<rows)&(cc<cols)
    bad=[];radius=int(math.ceil(.3/c.world.map.resolution))
    for r,col in zip(rr[valid],cc[valid]):
        if c.topology.subregions[r,col]!=rid or c.plan.corridor_mask[r,col]:continue
        if not c.plan.corridor_mask[max(0,r-radius):min(rows,r+radius+1),max(0,col-radius):min(cols,col+radius+1)].any():
            bad.append((int(r),int(col)))
    if not bad:return
    tick=time.monotonic()
    c.priority=c.priority.copy()
    c._limit_l2_goal_parking_to_owned_road()
    c.search_legal=c.priority.copy()
    gap=cv2.dilate(c.search_legal.astype(np.uint8),np.ones((7,7),np.uint8))>0
    c.search_legal|=gap&(c.topology.subregions==0)
    del gap
    c.timing['r4_goal_access_repair']={'trigger':'L2 parking reference more than .3m outside final access',
         'outside_vertices':bad,'goal_parking_region':rid,'same_L1_route':True,'tightened_L2_scope':True}
    try:c._activate_reference()
    finally:c.timing['r4_goal_access_repair']['wall_ms']=(time.monotonic()-tick)*1000
