"""Query-local orientation/curvature search using the unchanged sweep checker."""
import ctypes,math,time
from pathlib import Path
import numpy as np
import cv2
from scipy import ndimage
from arena_3d_v1.semantic_world import digest

def search_vehicle(c,query,*,radius=3.,budget=12.,max_nodes=1500000,local=False):
    w=c.world;top=c.topology;started=time.monotonic()
    xy=c.reference_plan.world_xy
    rr,cc=w.cells(xy);pad=math.ceil(radius/w.map.resolution)
    if radius<=0:
        ids={r for r,_ in c.traffic.roads}|c.traffic.endpoint_parking
        if len(c.traffic.roads)>1:ids|={rid for rid,v in top.regions.items() if v['kind']=='junction_area'}
        rr,cc=np.nonzero(np.isin(top.subregions,list(ids)));pad=12
    r0=max(0,int(rr.min())-pad);r1=min(w.map.height,int(rr.max())+pad+1)
    c0=max(0,int(cc.min())-pad);c1=min(w.map.width,int(cc.max())+pad+1)
    labels=top.subregions[r0:r1,c0:c1].copy()
    # Source semantic regions are soft preferences. Exact obstacles and
    # source prohibitions are independently audited before acceptance.
    labels=np.where(labels>0,1,0).astype(np.int16)
    phases=[1];directions=[[0.,0.]]
    phases=np.ascontiguousarray(phases,np.int16);directions=np.ascontiguousarray(directions,np.float64)
    labels=np.ascontiguousarray(labels,np.int16)
    maneuver=np.ascontiguousarray(getattr(c.traffic,"maneuver_mask",np.zeros(c.world.hard.shape,bool))[r0:r1,c0:c1],np.uint8)
    seed=np.ones(labels.shape,np.uint8)
    cv2.polylines(seed,[np.c_[cc-c0,rr-r0].astype(np.int32)],False,0,1)
    pref=np.ascontiguousarray(np.minimum(cv2.distanceTransform(seed,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*w.map.resolution,4.),np.float32)
    hard=np.ascontiguousarray(w.hard|~c._target_mask(),np.uint8)
    # A coarse cell centre can sit exactly on a narrow-passage collision
    # boundary. Probe half-cell phases without changing the original map:
    # each original occupied cell remains an occupied square. This field is
    # used only for the search heuristic; exact swept rectangles decide motion.
    fine_free=np.repeat(np.repeat(~w.hard[r0:r1,c0:c1],2,axis=0),2,axis=1)
    dd=ndimage.distance_transform_edt(fine_free)*(w.map.resolution/2)
    clearance=np.ascontiguousarray(dd.reshape(labels.shape[0],2,labels.shape[1],2).max(axis=(1,3))-math.sqrt(2)*w.map.resolution/4,np.float32)
    lib=ctypes.CDLL(str(Path(__file__).resolve().parents[2]/'build/libv4_vehicle_search_experiment_r2.so'))
    fn=lib.vehicle_search;ptr=lambda dtype:np.ctypeslib.ndpointer(dtype=dtype,flags='C_CONTIGUOUS')
    fn.argtypes=[ptr(np.uint8),ctypes.c_int,ctypes.c_int,*([ctypes.c_double]*3),ptr(np.int16),ptr(np.float32),ptr(np.float32),ptr(np.uint8),*([ctypes.c_int]*4),ptr(np.int16),ptr(np.float64),ctypes.c_int,ptr(np.float64),ptr(np.float64),*([ctypes.c_double]*4),ctypes.c_int,ptr(np.float64),ctypes.c_int,ptr(np.float64)]
    fn.argtypes += [ptr(np.float32),ctypes.c_int]
    fn.restype=ctypes.c_int
    # Optimistic subcell disc cover guides search; original exact rectangle
    # sweeps still certify each edge. Original occupancy pixels are untouched.
    f=4;ds=w.map.resolution/f
    fine=np.repeat(np.repeat(~w.hard[r0:r1,c0:c1],f,0),f,1)
    distance=ndimage.distance_transform_edt(fine)*ds
    cover=np.ascontiguousarray(fine & (distance >= w.vehicle[1]-math.sqrt(2)*ds/2),np.uint8)
    hc=np.ascontiguousarray(1+4*np.maximum(0.,(.4-distance))**2,np.float32)
    hg=np.full(cover.shape,np.inf,np.float32)
    dr=int((w.map.origin[1]+(w.map.height-r0)*w.map.resolution-query.goal[1])/ds)
    dc=int((query.goal[0]-w.map.origin[0]-c0*w.map.resolution)/ds)
    df=lib.grid_distance;df.argtypes=[ptr(np.uint8),ptr(np.float32),ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_float,ptr(np.float32)];df.restype=ctypes.c_int
    expanded_grid=df(cover,hc,*cover.shape,dr*cover.shape[1]+dc,ds,hg)

    out=np.zeros((100000,3),np.float64);stats=np.zeros(11,np.float64)
    n=fn(hard,*hard.shape,w.map.resolution,*w.map.origin[:2],labels,pref,clearance,maneuver,r0,c0,*labels.shape,
         phases,directions,len(phases),np.asarray(query.start,float),np.asarray(query.goal,float),*w.vehicle,budget,max_nodes,out,len(out),stats,hg,f)
    log={'subcell_factor':f,'subcell_heuristic_expanded':expanded_grid,'success':n>0,'status':int(n),'bbox':[r0,r1,c0,c1],'radius_m':radius,'budget_s':budget,
         'expanded':int(stats[1]),'nodes':int(stats[2]),'collision_rejections':int(stats[3]),'search_s':float(stats[4]),
         'closest_visited_pose':stats[6:9].tolist(),'closest_goal_distance_m':float(stats[9]),'heuristic_relaxed':bool(stats[10]),
         'wall_ms':(time.monotonic()-started)*1000,'vehicle':w.vehicle,'physical_hash':digest(w.hard),
         'model':'forward continuous circular integration; curvature steps 0.4/m; original exact sweep',
         'quantization_m':{'narrow':.025,'open':.1},'heading_bins':{'narrow':144,'open':72},'grid_resolution_unchanged_m':w.map.resolution,
         'not_optimality_proof':True}
    return (out[:n].copy() if n>0 else None),log
