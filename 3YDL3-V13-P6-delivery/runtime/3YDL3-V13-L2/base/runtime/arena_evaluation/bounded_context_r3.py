"""Bounded lazy map fields for the static 2A-V1 online pipeline.

Occupancy is a verified disk mapping. Derived fields are local, byte-bounded
LRUs. Chamfer distances are returned only when an exterior lower bound proves
that omitted cells cannot improve them. No global distance array is allocated.
The audit field preserves the canonical near-obstacle decision, including
unknown cells; distant values are represented by a finite value above the threshold (fast-path only).
"""
from collections import OrderedDict
from pathlib import Path
import math
import hashlib
import json
import os
import sys
import tempfile
import time

import cv2
import numpy as np
from scipy import ndimage

from .tiled_topology_r3 import load_map_bounded, file_hash


class BoundedField:
    def __init__(self, hospital, kind, *, footprint=None, tile_cells=128,
                 max_halo_cells=2048, max_bytes=16*1024**2, safe_threshold=None,
                 disk_cache=None, map_hash=None, max_disk_bytes=512*1024**2):
        if tile_cells < 1 or max_halo_cells < 8 or max_bytes < tile_cells**2*8:
            raise ValueError('invalid bounded field capacity')
        self.map=hospital;self.kind=kind;self.footprint=footprint
        self.shape=(hospital.height,hospital.width);self.ndim=2
        self.tile_cells=tile_cells;self.max_halo_cells=max_halo_cells
        self.max_bytes=max_bytes;self.bytes=0;self.entries=OrderedDict()
        self.safe_threshold=safe_threshold
        self.disk_cache=None;self.max_disk_bytes=max_disk_bytes
        self.disk_bytes=0
        if disk_cache is not None:
            if kind!='occupied_distance' or not map_hash:
                raise ValueError('persistent clearance requires occupied field and map hash')
            binding={'algorithm':'bounded-chamfer-v2','map_hash':map_hash,'shape':self.shape,
                     'resolution':hospital.resolution,'tile_cells':tile_cells,'max_halo_cells':max_halo_cells,
                     'opencv':cv2.__version__,'byteorder':sys.byteorder}
            digest=hashlib.sha256(json.dumps(binding,sort_keys=True).encode()).hexdigest()
            self.disk_cache=Path(disk_cache)/digest;self.disk_cache.mkdir(parents=True,exist_ok=True)
            self.disk_bytes=sum(f.stat().st_size for f in self.disk_cache.glob('*.tile'))
            if self.disk_bytes>max_disk_bytes:raise MemoryError('CLEARANCE_DISK_CACHE_LIMIT')
        self.has_unsafe=True
        if kind=='audit':
            self.has_unsafe=any(np.any(hospital.occupancy[r:r+128]!=0)
                                for r in range(0,hospital.height,128))
        self.stats={'hits':0,'misses':0,'build_ms':0.,'peak_cache_bytes':0,
                    'max_window_cells':0,'uncertified_windows':0,
                    'disk_hits':0,'disk_misses':0,'disk_bytes':self.disk_bytes,'disk_io_ms':0.}

    def __array__(self,*args,**kwargs):
        raise MemoryError('BOUNDED_FIELD_FULL_MATERIALIZATION_FORBIDDEN')

    def __getitem__(self,key):
        if not isinstance(key,tuple) or len(key)!=2:
            raise TypeError('bounded fields require row/column indexing')
        r,c=key
        if isinstance(r,(int,np.integer)) and isinstance(c,(int,np.integer)):
            if not (0<=r<self.shape[0] and 0<=c<self.shape[1]):
                raise IndexError('bounded field coordinates outside map')
            tr,rr=divmod(int(r),self.tile_cells);tc,cc=divmod(int(c),self.tile_cells)
            return self._get_tile(tr,tc)[rr,cc].item()
        if isinstance(r,slice) or isinstance(c,slice):
            raise MemoryError('BOUNDED_FIELD_SLICE_MATERIALIZATION_FORBIDDEN')
        r,c=np.broadcast_arrays(np.asarray(r,dtype=np.int64),np.asarray(c,dtype=np.int64))
        if np.any(r<0) or np.any(c<0) or np.any(r>=self.shape[0]) or np.any(c>=self.shape[1]):
            raise IndexError('bounded field coordinates outside map')
        out=np.empty(r.shape,dtype=bool if self.kind=='free' else float)
        rf,cf,of=r.ravel(),c.ravel(),out.ravel();stride=(self.shape[1]+self.tile_cells-1)//self.tile_cells
        tiles=(rf//self.tile_cells)*stride+cf//self.tile_cells
        for value in np.unique(tiles):
            tr,tc=divmod(int(value),stride);v=self._get_tile(tr,tc);idx=np.flatnonzero(tiles==value)
            of[idx]=v[rf[idx]-tr*self.tile_cells,cf[idx]-tc*self.tile_cells]
        return out.item() if out.ndim==0 else out

    def capped_at(self, cells, cap_m):
        """Exact min(distance, cap), using a crop whose exterior exceeds cap.

        Intended for corridor widening thresholds, never a replacement for full
        audit clearance. The same OpenCV chamfer and float32 scale are used.
        """
        if self.kind!='occupied_distance' or not math.isfinite(cap_m) or cap_m<=0:
            raise ValueError('positive occupied-distance cap required')
        halo=math.ceil(cap_m/self.map.resolution/.98)+4
        if halo>self.max_halo_cells:raise ValueError('cap exceeds bounded halo')
        out=np.zeros(len(cells),dtype=np.float32);groups={};h,w=self.shape;t=self.tile_cells
        for i,cell in enumerate(cells):
            if cell is None:continue
            r,c=cell
            if not (0<=r<h and 0<=c<w):raise IndexError('capped clearance outside map')
            groups.setdefault((r//t,c//t),[]).append((i,r,c))
        for (tr,tc),values in sorted(groups.items()):
            indices,rr,cc=np.asarray(values,dtype=np.int64).T
            if (tr,tc) in self.entries:
                tile=self._get_tile(tr,tc);out[indices]=np.minimum(tile[rr-tr*t,cc-tc*t],cap_m);continue
            a,b=max(0,tr*t-halo),min(h,(tr+1)*t+halo)
            l,u=max(0,tc*t-halo),min(w,(tc+1)*t+halo)
            exterior=math.inf
            for active,d in ((a>0,rr-a-2),(b<h,b-1-rr-2),(l>0,cc-l-2),(u<w,u-1-cc-2)):
                if active:exterior=min(exterior,float(np.min(.98*d))*self.map.resolution)
            if exterior<=cap_m:raise RuntimeError('CAPPED_CLEARANCE_EXTERIOR_UNCERTIFIED')
            free=(self.map.occupancy[a:b,l:u]!=100).astype(np.uint8)
            distance=cv2.distanceTransform(free,cv2.DIST_L2,5)
            out[indices]=np.minimum(distance[rr-a,cc-l]*self.map.resolution,cap_m)
            self.stats['max_window_cells']=max(self.stats['max_window_cells'],free.size)
        return out

    def minimum_at(self, cells):
        """Exact sampled minimum, with exterior bounds pruning irrelevant tiles.

        This computes the same minimum as scalar clearance lookups. A crop may
        be skipped only if its certified lower bound exceeds an exact incumbent.
        No individual distance or collision predicate is approximated.
        """
        if self.kind!='occupied_distance':raise ValueError('occupied clearance only')
        if any(cell is None for cell in cells):return 0.
        groups={}
        for r,c in cells:groups.setdefault((r//self.tile_cells,c//self.tile_cells),[]).append((r,c))
        best=math.inf;h,w=self.shape;t=self.tile_cells
        for (tr,tc),values in sorted(groups.items()):
            rr,cc=np.asarray(values,dtype=np.int64).T
            if (tr,tc) in self.entries:
                tile=self._get_tile(tr,tc)
                best=min(best,float(np.min(tile[rr-tr*t,cc-tc*t])));continue
            halo=32
            while True:
                a,b=max(0,tr*t-halo),min(h,(tr+1)*t+halo)
                l,u=max(0,tc*t-halo),min(w,(tc+1)*t+halo)
                bound=math.inf
                for active,d in ((a>0,rr-a-2),(b<h,b-1-rr-2),
                                 (l>0,cc-l-2),(u<w,u-1-cc-2)):
                    if active:bound=min(bound,float(np.min(.98*d)))
                free=(self.map.occupancy[a:b,l:u]!=100).astype(np.uint8)
                distance=cv2.distanceTransform(free,cv2.DIST_L2,5)
                least=float(np.min(distance[rr-a,cc-l]))
                if best < min(least,bound)*self.map.resolution-1e-6:break
                if least<bound:
                    best=min(best,float(np.float32(least)*np.float32(self.map.resolution)));break
                if halo>=self.max_halo_cells:
                    raise RuntimeError('BOUNDED_DISTANCE_MINIMUM_UNCERTIFIED')
                halo=min(self.max_halo_cells,halo*2)
        return best

    def _get_tile(self,tr,tc):
        key=(tr,tc)
        if key in self.entries:
            self.stats['hits']+=1;v=self.entries.pop(key);self.entries[key]=v;return v
        begin=time.monotonic();self.stats['misses']+=1
        h,w=self.shape;t=self.tile_cells;r0,c0=tr*t,tc*t;r1,c1=min(h,r0+t),min(w,c0+t)
        path=self.disk_cache/f'{tr}_{tc}.tile' if self.disk_cache is not None else None
        if path is not None and path.exists():
            io=time.monotonic();size=(r1-r0)*(c1-c0)*4
            with path.open('rb') as stream:blob=stream.read(size+65)
            if len(blob)!=size+64 or hashlib.sha256(blob[64:]).hexdigest().encode()!=blob[:64]:
                raise RuntimeError('CLEARANCE_DISK_CACHE_HASH_MISMATCH')
            v=np.frombuffer(blob,offset=64,dtype=np.float32).reshape(r1-r0,c1-c0)
            self.stats['disk_hits']+=1;self.stats['disk_io_ms']+=(time.monotonic()-io)*1000
            return self._retain(key,v,begin)
        if path is not None:self.stats['disk_misses']+=1
        if self.kind=='audit':
            halo=math.ceil(self.safe_threshold/self.map.resolution)+2
        else:halo=32
        while True:
            a,b=max(0,r0-halo),min(h,r1+halo);l,u=max(0,c0-halo),min(w,c1+halo)
            occ=np.asarray(self.map.occupancy[a:b,l:u])
            self.stats['max_window_cells']=max(self.stats['max_window_cells'],int(occ.size))
            inner=(slice(r0-a,r1-a),slice(c0-l,c1-l))
            if self.kind=='audit':
                free=occ==0
                if np.any(~free):
                    distance=ndimage.distance_transform_edt(free,sampling=self.map.resolution)[inner]
                    # Exact near-threshold values; omitted cells are farther
                    # than the halo, so the remaining canonical fast path is safe.
                    v=np.where(distance<=self.safe_threshold+1e-12,distance,self.safe_threshold+self.map.resolution)
                elif self.has_unsafe:v=np.full((r1-r0,c1-c0),self.safe_threshold+self.map.resolution)
                else:
                    # Preserve SciPy EDT's all-free-array convention exactly.
                    rr=np.arange(r0,r1,dtype=float)[:,None]+1.
                    cc=np.arange(c0,c1,dtype=float)[None,:]
                    v=np.sqrt((rr*self.map.resolution)**2+(cc*self.map.resolution)**2)
                break
            if self.kind in ('free','inflated_distance'):
                from .topology import _footprint_kernel
                kernel=_footprint_kernel(self.map.resolution,self.footprint,.05,.05)
                free=~cv2.dilate((occ!=0).astype(np.uint8),kernel).astype(bool)
                free &= occ>=0
                if self.kind=='free':v=free[inner].copy();break
                guard=max(kernel.shape)//2+2
            else:free=occ!=100;guard=2
            distance=cv2.distanceTransform(free.astype(np.uint8),cv2.DIST_L2,5)[inner]
            # Every omitted path crosses a crop boundary. 0.98 is below the
            # minimum cost / Euclidean step of OpenCV's 5x5 chamfer stencil.
            rr=np.arange(r0,r1)[:,None];cc=np.arange(c0,c1)[None,:]
            bound=np.full((r1-r0,c1-c0),np.inf)
            for active,d in ((a>0,rr-a-guard),(b<h,b-1-rr-guard),
                             (l>0,cc-l-guard),(u<w,u-1-cc-guard)):
                if active:bound=np.minimum(bound,.98*d)
            if np.all(distance<bound):
                v=(distance*self.map.resolution).astype(np.float32);break
            if halo>=self.max_halo_cells:
                self.stats['uncertified_windows']+=1
                raise RuntimeError(f'BOUNDED_DISTANCE_WINDOW_UNCERTIFIED tile={r0,c0} max_distance_cells={float(distance.max())} min_exterior_bound={float(bound.min())}')
            halo=min(self.max_halo_cells,halo*2)
        v=np.ascontiguousarray(v);v.setflags(write=False)
        if path is not None and self.disk_bytes+64+v.nbytes<=self.max_disk_bytes:
            io=time.monotonic();payload=v.tobytes();blob=hashlib.sha256(payload).hexdigest().encode()+payload
            fd,tmp=tempfile.mkstemp(prefix='.tile-',dir=self.disk_cache)
            try:
                with os.fdopen(fd,'wb') as stream:stream.write(blob);stream.flush();os.fsync(stream.fileno())
                os.replace(tmp,path)
            finally:
                if os.path.exists(tmp):os.unlink(tmp)
            self.disk_bytes+=len(blob);self.stats['disk_bytes']=self.disk_bytes
            self.stats['disk_io_ms']+=(time.monotonic()-io)*1000
        return self._retain(key,v,begin)

    def _retain(self,key,v,begin):
        while self.entries and self.bytes+v.nbytes>self.max_bytes:
            _,old=self.entries.popitem(last=False);self.bytes-=old.nbytes
        self.entries[key]=v;self.bytes+=v.nbytes
        self.stats['peak_cache_bytes']=max(self.stats['peak_cache_bytes'],self.bytes)
        self.stats['build_ms']+=(time.monotonic()-begin)*1000
        return v


def bounded_context(map_yaml,map_id,cache,footprint):
    from .unified_four_backends_smoke import MapContext
    hospital=load_map_bounded(map_yaml,cache)
    if not np.isclose(hospital.resolution,.05):raise ValueError('frozen resolution required')
    hospital.distance_m=BoundedField(hospital,'occupied_distance',disk_cache=Path(cache)/'clearance',
                                    map_hash=hashlib.sha256((file_hash(hospital.image_path)+file_hash(hospital.yaml_path)).encode()).hexdigest())
    ctx=MapContext(map_id,hospital,BoundedField(hospital,'free',footprint=footprint),
                   BoundedField(hospital,'inflated_distance',footprint=footprint),
                   file_hash(hospital.image_path),file_hash(hospital.yaml_path),Path(map_yaml))
    ctx.bounded_memory=True
    return ctx
