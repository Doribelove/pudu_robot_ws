"""Query-time edge occurrence progress. No required crossing point or gate.
Original region adjacency is retained for traffic order, not as L3 obstacles.
"""
import math,time
from types import SimpleNamespace
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components
from arena_evaluation.semantic_map import canonical_hash
from arena_3d_v1.l2_state_lifecycle import CompactCorridorGeometry,CompactGeometryBinding,MutableStateBinding,OFFSETS,OFFSET_INDEX
from arena_3d_v1.semantic_l2 import WeightedDStarState,WeightedCorridorDStar
from arena_3d_v1.semantic_world import digest

class ProgressGeometry:
    def __init__(self,roi,view,channel):
        base=CompactCorridorGeometry.build(roi);self.base=base;self.bbox=roi.bbox;self.roi_shape=roi.shape
        self.view=view;self.channel=channel;self.global_corridor_cells=roi.global_corridor_cells
        brc=np.asarray([base.global_cell(i) for i in range(base.state_count)],np.int32)
        xy=view.world_points(brc[:,0],brc[:,1]);labels=channel.sample(xy)
        self.regions=channel.regions;self.stage=[];self.physical=[];self.maps=[];start=0
        for stage,rid in enumerate(self.regions):
            ids=np.flatnonzero(channel.contains(stage,labels) if hasattr(channel,'contains') else labels==rid);mapping=np.full(base.state_count,-1,np.int32)
            mapping[ids]=np.arange(start,start+len(ids));start+=len(ids)
            self.maps.append(mapping);self.stage.extend([stage]*len(ids));self.physical.extend(ids)
        self.stage=np.asarray(self.stage,np.int16);self.physical=np.asarray(self.physical,np.int32)
        self.cells=brc[self.physical];self.state_count=len(self.cells)
        self.state_cells_linear=(self.cells[:,0]-roi.bbox[0])*roi.shape[1]+self.cells[:,1]-roi.bbox[2]
        self.neighbors=np.full((self.state_count,24),-1,np.int32)
        # Physical adjacency already enforces static corner clearance. Check
        # semantic segments too so thin third regions cannot be jumped over.
        physical_neighbors=np.asarray([[base.neighbor_id_at(i,j) for j in range(8)] for i in range(base.state_count)],np.int32)
        for stage,rid in enumerate(self.regions):
            own=np.flatnonzero(self.stage==stage);physical=self.physical[own]
            for direction,(dr,dc) in enumerate(OFFSETS):
                nb=physical_neighbors[physical,direction];ok=nb>=0;src=own[ok];dstphysical=nb[ok]
                if not len(src):continue
                a=xy[self.physical[src]];b=xy[dstphysical];targetlabels=labels[dstphysical]
                for targetstage in [stage-1,stage,stage+1]:
                    if targetstage<0 or targetstage>=len(self.regions):continue
                    dst=self.maps[targetstage][dstphysical];valid=dst>=0
                    # At most the two adjacent stages may be touched by a grid step.
                    for fraction in [.125,.375,.5,.625,.875]:
                        seen=channel.sample(a+(b-a)*fraction)
                        valid &= (channel.contains(stage,seen)|channel.contains(targetstage,seen)) if hasattr(channel,'contains') else ((seen==rid)|(seen==self.regions[targetstage]))
                    if hasattr(channel,'crossing'):
                        valid &= channel.crossing(stage,targetstage,labels[self.physical[src]],targetlabels)
                    # Separate occurrence slots retain BOTH reverse dependencies when
                    # a region appears before and after the current occurrence.
                    self.neighbors[src[valid],direction+8*(targetstage-stage+1)]=dst[valid]
        self._neighbor_view=memoryview(self.neighbors.reshape(-1))
        self._rows=self.cells[:,0].tolist();self._cols=self.cells[:,1].tolist();self._stages=self.stage.tolist()
        self.start_cell=roi.binding.start_cell;self.goal_cell=roi.binding.goal_cell
        self.start_id=self.id_at(0,self.start_cell);self.goal_id=self.id_at(len(self.regions)-1,self.goal_cell)
        self.binding=SimpleNamespace(digest=canonical_hash(['v4-edge-occurrence-geometry-v1',roi.binding.as_dict() if hasattr(roi.binding,'as_dict') else str(roi.binding),getattr(channel,'cache_contract',channel.plan),digest(self.cells),digest(self.stage),digest(self.neighbors)]))
        # Dependency graph must be symmetric even though traversal is directed.
        src,j=np.nonzero(self.neighbors>=0);dst=self.neighbors[src,j]
        opposite=np.asarray([OFFSET_INDEX[(-dr,-dc)] for dr,dc in OFFSETS])
        if not np.all(self.neighbors[dst,opposite[j%8]+8*(2-j//8)]==src):raise ValueError('PROGRESS_ASYMMETRIC_DEPENDENCIES')
        # Check directed stage reachability without expensive futile D* scans.
        forward=self.stage[dst]>=self.stage[src]
        graph=csr_matrix((np.ones(int(forward.sum()),np.int8),(src[forward],dst[forward])),shape=(self.state_count,)*2)
        from scipy.sparse.csgraph import breadth_first_order
        reached=breadth_first_order(graph,self.start_id,directed=True,return_predecessors=False)
        if self.goal_id not in reached:raise RuntimeError('GUIDE_NO_PATH')
    def id_at(self,stage,rc):
        pid=self.base.state_id_from_global(rc,required=False)
        result=-1 if pid<0 else int(self.maps[stage][pid])
        if result<0:raise ValueError('COARSE_ENDPOINT_UNREPRESENTABLE' if self.view.factor>1 else 'FINE_ENDPOINT_UNSAFE')
        return result
    def state_id_from_global(self,cell,required=True):
        if tuple(cell)==tuple(self.goal_cell):return self.goal_id
        if tuple(cell)==tuple(self.start_cell):return self.start_id
        pid=self.base.state_id_from_global(cell,required=False)
        ids=[] if pid<0 else [int(m[pid]) for m in self.maps if m[pid]>=0]
        if not ids and required:raise ValueError('CELL_OUTSIDE_CHANNEL')
        return ids[0] if ids else -1
    def state_ids_from_global(self,cells):
        physical=self.base.state_ids_from_global(cells)
        return {int(m[p]) for p in physical for m in self.maps if m[p]>=0}
    def global_cell(self,i):return self._rows[i],self._cols[i]
    def local_cell(self,i):return self._rows[i]-self.bbox[0],self._cols[i]-self.bbox[2]
    def neighbor_ids(self,i):
        start=int(i)*24
        for j in range(24):
            n=self._neighbor_view[start+j]
            if n>=0:yield n
    def neighbor_id_at(self,i,j):
        for slot in (8+j,j,16+j):
            n=int(self._neighbor_view[int(i)*24+slot])
            if n>=0:return n
        return -1
    @property
    def resident_bytes(self):return self.base.resident_bytes+sum(x.nbytes for x in [self.cells,self.physical,self.stage,self.neighbors,self.state_cells_linear,*self.maps])
