# SPDX-License-Identifier: Apache-2.0
"""Portable file input adapter for the original position-only semantic L1."""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from .semantic_support import canonical_hash,rasterize_feature
from .semantic_astar import AStarTopology
from .semantic_topology import V5Topology


@dataclass(frozen=True)
class SemanticQuery:
    query_id:str
    start:tuple
    goal:tuple


class _GridMap:
    def __init__(self,width,height,resolution,origin,hard):
        self.width=width;self.height=height;self.resolution=resolution;self.origin=origin
        self.sha256=hashlib.sha256(hard.tobytes()+repr((width,height,resolution,origin)).encode()).hexdigest()
    def world_to_cell(self,x,y):
        c=math.floor((x-self.origin[0])/self.resolution)
        r=self.height-1-math.floor((y-self.origin[1])/self.resolution)
        return (r,c) if 0<=r<self.height and 0<=c<self.width else None


class SemanticPlanner:
    """Build original semantic adjacency/rules and produce real L1 route tasks.

    Input directory contains the original graph, semantic.json and parking_access.json.
    This adapter supplies grid geometry, not a fake routing engine. Polygons are
    rasterized and parking contacts detected by the original code.
    """
    def __init__(self,graph_path,cache_root,*,width,height,resolution,origin=(0.,0.,0.),hard=None,mode='astar'):
        if not isinstance(width,int) or not isinstance(height,int) or min(width,height)<1 or width*height>1000000:
            raise ValueError('grid dimensions must be positive and at most 1 million cells')
        if not math.isfinite(resolution) or resolution<=0 or len(origin)!=3 or not all(math.isfinite(x) for x in origin) or origin[2]!=0:
            raise ValueError('require finite positive resolution and zero-yaw origin')
        if mode not in ('astar','legacy'):raise ValueError('unknown semantic search mode')
        graph_path=Path(graph_path);doc=json.loads((graph_path.parent/'semantic.json').read_text())
        graph=json.loads(graph_path.read_text());names=set();features={}
        for item in doc['features']:
            name=item['properties']['source_name'];coords=np.asarray(item['coordinates'],float)
            if name in names or coords.ndim!=2 or coords.shape[1]!=2 or len(coords)<3 or not np.isfinite(coords).all():
                raise ValueError('feature must have a unique name and finite polygon')
            names.add(name)
            features[item['semantic_id']]=SimpleNamespace(properties=dict(item['properties']),coordinates=coords.tolist(),geometry_type='polygon')
        if len({r['id'] for r in graph['regions']})!=len(graph['regions']):raise ValueError('duplicate region id')
        for r in graph['regions']:
            if r['name'] not in names or r['kind'] not in ('lane','junction_area','parking_area'):raise ValueError('invalid region')
        mask=np.zeros((height,width),bool) if hard is None else np.asarray(hard)
        if mask.shape!=(height,width) or mask.dtype!=np.bool_:raise ValueError('hard must be a boolean grid with matching shape')
        mask=np.array(mask,copy=True);mask.flags.writeable=False
        semantic=SimpleNamespace(width=width,height=height,resolution=resolution,origin=origin)
        grid=_GridMap(width,height,resolution,origin,mask)
        labels=np.zeros((height,width),np.int32)
        for index,feature in enumerate(features.values(),1):labels[rasterize_feature(semantic,feature)]=index
        def world_points(rows,cols):
            return np.column_stack((origin[0]+(np.asarray(cols)+.5)*resolution,
                                    origin[1]+(height-np.asarray(rows)-.5)*resolution))
        world=SimpleNamespace(map=grid,semantic=semantic,features=features,labels=labels,hard=mask,
                              key=canonical_hash([grid.sha256,doc]),binding={'input':'portable-semantic-adapter'},world_points=world_points)
        self.topology=(AStarTopology if mode=='astar' else V5Topology)(world,graph_path,Path(cache_root))
    def routes(self,query_id,start,goal,limit=3):
        if not isinstance(query_id,str) or not isinstance(limit,int) or limit<1:raise ValueError('query id and positive candidate limit required')
        for point in (start,goal):
            if len(point)!=3 or not all(math.isfinite(v) for v in point):raise ValueError('poses must contain finite x,y,yaw')
        query=SemanticQuery(query_id,tuple(start),tuple(goal))
        routes=self.topology.routes(query,limit=limit)
        return [(route,self.topology.task(route),self.topology.contexts[route.signature]) for route in routes]
