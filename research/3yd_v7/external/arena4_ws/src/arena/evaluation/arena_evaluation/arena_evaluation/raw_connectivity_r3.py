"""Bounded-memory necessary positional reachability certificates.

Eight-connected free-cell paths overapproximate collision-free footprint paths.
A disconnected result therefore proves a static positional obstruction. A
connected result says nothing about footprint, heading, or Dubins feasibility.
No result from this module authorizes planner output or replaces PathAudit.
"""
from __future__ import annotations
import hashlib
import json
import math
import time
from dataclasses import dataclass
import cv2
import numpy as np

VERSION='raw-free-eight-connectivity-tiled-certificate-v1'


def stable_hash(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),
                                     allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class TileBoundary:
    top: tuple
    bottom: tuple
    left: tuple
    right: tuple


class RawConnectivity:
    def __init__(self,map_,map_hash,*,tile_cells=512,max_boundary_nodes=2_000_000):
        if tile_cells<2 or max_boundary_nodes<1:
            raise ValueError('invalid raw connectivity resource limit')
        self.map=map_;self.tile_cells=tile_cells;self.max_boundary_nodes=max_boundary_nodes
        self.binding={'algorithm':VERSION,'map_hash':map_hash,
                      'shape':[map_.height,map_.width],'resolution':map_.resolution,
                      'origin':list(map_.origin),'free_value':0,'connectivity':8,
                      'tile_cells':tile_cells,'max_boundary_nodes':max_boundary_nodes}
        self.parent={};self._completed=False

    def _root(self,a):
        if a is None:return None
        self.parent.setdefault(a,a)
        while self.parent[a]!=a:
            self.parent[a]=self.parent[self.parent[a]];a=self.parent[a]
        return a

    def _union(self,a,b):
        if a is None or b is None:return
        a,b=self._root(a),self._root(b)
        self.parent[max(a,b)]=min(a,b)

    def build(self,poses,*,deadline=None,progress=None):
        """Label one tile at a time; retain only seam and requested pose labels.

        `poses` maps immutable caller keys to world poses. All are registered
        before scanning; adding an unseen pose requires a new build/certificate.
        """
        if self._completed or self.parent:
            raise RuntimeError('RAW_CONNECTIVITY_BUILD_IS_WRITE_ONCE')
        started=time.monotonic();m=self.map;t=self.tile_cells
        self._poses={k:tuple(v) for k,v in poses.items()};self.pose_nodes={}
        per_tile={}
        for key,pose in sorted(self._poses.items()):
            cell=m.world_to_cell(*pose[:2])
            if cell is None:self.pose_nodes[key]=None;continue
            r,c=cell
            per_tile.setdefault((r//t,c//t),[]).append((key,r%t,c%t))
        previous_row={};tile_count=0;seam_pairs=0;max_tile_cells=0
        stream_hash=hashlib.sha256()
        for tr in range(math.ceil(m.height/t)):
            current_row={}
            for tc in range(math.ceil(m.width/t)):
                if deadline is not None and time.monotonic()>=deadline:
                    raise TimeoutError('RAW_CONNECTIVITY_DEADLINE')
                r,c=tr*t,tc*t
                raw=np.ascontiguousarray(m.occupancy[r:min(m.height,r+t),c:min(m.width,c+t)]==0,dtype=np.uint8)
                count,labels=cv2.connectedComponents(raw,connectivity=8)
                max_tile_cells=max(max_tile_cells,raw.size)
                # Canonical IDs are first occupied positions of each free
                # label, independent of the native labeling traversal order.
                unique,first=np.unique(labels,return_index=True)
                anchors=np.zeros(count,dtype=np.int64);anchors[unique]=first
                def node(label):return (tr,tc,int(anchors[label])) if label else None
                boundary=TileBoundary(tuple(map(node,labels[0,:])),tuple(map(node,labels[-1,:])),
                                      tuple(map(node,labels[:,0])),tuple(map(node,labels[:,-1])))
                for edge in [boundary.top,boundary.bottom,boundary.left,boundary.right]:
                    for value in edge:
                        if value is not None:self._root(value)
                for key,rr,cc in per_tile.get((tr,tc),[]):
                    value=node(labels[rr,cc]);self.pose_nodes[key]=value
                    if value is not None:self._root(value)
                if len(self.parent)>self.max_boundary_nodes:
                    raise MemoryError('RAW_CONNECTIVITY_BOUNDARY_NODE_LIMIT')
                # Vertical/horizontal seams include diagonal cell neighbors.
                for first,second in [(current_row[tc-1].right,boundary.left)] if tc else []:
                    for i,a in enumerate(first):
                        for j in range(max(0,i-1),min(len(second),i+2)):
                            self._union(a,second[j]);seam_pairs+=1
                if tr:
                    first,second=previous_row[tc].bottom,boundary.top
                    for i,a in enumerate(first):
                        for j in range(max(0,i-1),min(len(second),i+2)):
                            self._union(a,second[j]);seam_pairs+=1
                    # The two diagonal neighbors across four-tile corners are
                    # not covered by the same-column horizontal seam above.
                    if tc:
                        self._union(previous_row[tc-1].bottom[-1],boundary.top[0]);seam_pairs+=1
                    if tc+1 in previous_row:
                        self._union(previous_row[tc+1].bottom[0],boundary.top[-1]);seam_pairs+=1
                current_row[tc]=boundary;tile_count+=1
                stream_hash.update(stable_hash({'tile':[tr,tc],'shape':list(raw.shape),
                    'free_sha256':hashlib.sha256(raw.tobytes()).hexdigest(),
                    'components':int(count-1),'boundary':vars(boundary),
                    'pose_labels':[(key,self.pose_nodes[key]) for key,_,_ in per_tile.get((tr,tc),[])]}).encode())
                if progress:progress({'tile_completed_count':tile_count,'boundary_nodes':len(self.parent)})
            previous_row=current_row
        self.pose_roots={key:self._root(value) for key,value in self.pose_nodes.items()}
        self.stats={'tile_completed_count':tile_count,'boundary_nodes':len(self.parent),
                    'max_dense_tile_cells':max_tile_cells,'seam_neighbor_pairs_checked':seam_pairs,
                    'build_wall_ms':(time.monotonic()-started)*1000,'tile_stream_sha256':stream_hash.hexdigest()}
        self._completed=True
        return dict(self.stats)

    def certificate(self,start_key,goal_key):
        if not self._completed:raise RuntimeError('RAW_CONNECTIVITY_NOT_COMPLETE')
        a,b=self.pose_roots[start_key],self.pose_roots[goal_key]
        if a is None or b is None:status='ENDPOINT_CENTER_NOT_FREE'
        elif a!=b:status='PROVEN_RAW_FREE_DISCONNECTED'
        else:status='RAW_CONNECTED_SE2_UNPROVEN'
        payload={'binding':self.binding,'start':list(self._poses[start_key]),
                 'goal':list(self._poses[goal_key]),'start_component':a,'goal_component':b,
                 'status':status,'scope':'necessary positional condition only; no positive footprint/SE2 claim',
                 'tile_stream_sha256':self.stats['tile_stream_sha256']}
        return {**payload,'sha256':stable_hash(payload)}
