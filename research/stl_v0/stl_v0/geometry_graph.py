"""Shared geometry only; no paths, task progress or query results are cached."""
from collections import OrderedDict
import hashlib
import json
import time
import numpy as np
from .spatial import box_pairs

_CACHE=OrderedDict()
CACHE_LIMIT=2


def clear_geometry_cache():_CACHE.clear()


def geometry_key(scene):
    data=[(r.bounds.tolist(),r.halfspaces) for r in scene.regions]
    return hashlib.sha256(json.dumps(data,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def prepare_geometry(scene,deadline=float('inf'),use_cache=True):
    started=time.monotonic();key=geometry_key(scene)
    if time.monotonic()>deadline:raise TimeoutError('GEOMETRY_BUILD_BUDGET_EXHAUSTED')
    if use_cache and key in _CACHE:
        value=_CACHE.pop(key);_CACHE[key]=value
        return value,{'cache_hit':True,'wall_s':time.monotonic()-started,'geometry_key':key,'retained_graphs':len(_CACHE)}
    boxes=np.array([r.bounds for r in scene.regions]);centers=(boxes[:,:2]+boxes[:,2:])/2
    pairs,stats=box_pairs(boxes,deadline=deadline);adj=[[] for _ in boxes];widths=[];distances=[];valid=[]
    from .logic import overlap
    for i,j in pairs:
        if (len(valid)%1024)==0 and time.monotonic()>deadline:raise TimeoutError('GEOMETRY_BUILD_BUDGET_EXHAUSTED')
        if scene.regions[i].halfspaces or scene.regions[j].halfspaces:
            shared=overlap(scene.regions[i],scene.regions[j])
            if shared is None:continue
            width=float(np.min(shared[2:]-shared[:2]))
        else:width=float(min(boxes[i,2]-max(boxes[i,0],boxes[j,0]),boxes[j,2]-max(boxes[i,0],boxes[j,0]),boxes[i,3]-max(boxes[i,1],boxes[j,1]),boxes[j,3]-max(boxes[i,1],boxes[j,1])))
        distance=float(np.linalg.norm(centers[j]-centers[i]))
        index=len(valid);valid.append((int(i),int(j)));widths.append(width);distances.append(distance)
        adj[i].append((int(j),index));adj[j].append((int(i),index))
    for row in adj:row.sort()
    value={'neighbors':tuple(tuple(row) for row in adj),'widths':np.array(widths),'distances':np.array(distances),
           'centers':centers,'region_widths':np.min(boxes[:,2:]-boxes[:,:2],axis=1),'geometry_key':key}
    if use_cache:
        _CACHE[key]=value
        while len(_CACHE)>CACHE_LIMIT:_CACHE.popitem(last=False)
    return value,{'cache_hit':False,'wall_s':time.monotonic()-started,'geometry_key':key,'undirected_edges':len(valid),**stats}
