"""Bounded terminal crossing/merging permission on the explicitly owned road."""
import numpy as np
from scipy import ndimage

RADIUS_M=2.5
MAX_OPPOSING_TRAVEL_M=2.0

def maneuver_mask(world,topology,task):
    mask=np.zeros(world.hard.shape,bool);centers=[]
    for role,a in task['endpoint_access'].items():
        rid=a['semantic_region_id']
        if topology.regions[rid]['kind']!='parking_area' or not a.get('local_cross_road_allowed'):continue
        road=a['road_id'];points=[a['original_pose'][:2],a['xy'],a['access_projection']['xy']]
        other=task['endpoint_access']['goal' if role=='start' else 'start']
        # A journey starting immediately alongside its destination parking area
        # can turn into that area at the start; attachment projection is not a
        # mandatory entrance. It still has to use the same authorized road.
        if other['semantic_region_id']==road:
            distances=ndimage.distance_transform_edt(topology.subregions!=rid)*world.map.resolution
            r,c=world.cells(np.asarray([other['original_pose'][:2]]))
            if distances[r[0],c[0]]<=RADIUS_M:points.append(other['original_pose'][:2])
        for xy in points:
            r,c=world.cells(np.asarray([xy]));pad=int(np.ceil(RADIUS_M/world.map.resolution))+1
            r0=max(0,int(r[0])-pad);r1=min(world.map.height,int(r[0])+pad+1)
            c0=max(0,int(c[0])-pad);c1=min(world.map.width,int(c[0])+pad+1)
            rr,cc=np.indices((r1-r0,c1-c0));coords=world.world_points((rr+r0).ravel(),(cc+c0).ravel())
            disk=(np.linalg.norm(coords-np.asarray(xy),axis=1)<=RADIUS_M).reshape(rr.shape)
            mask[r0:r1,c0:c1]|=disk&(topology.subregions[r0:r1,c0:c1]==road)
            centers.append({'xy':list(xy),'owned_road':road,'parking':rid,'radius_m':RADIUS_M})
    return mask,centers
