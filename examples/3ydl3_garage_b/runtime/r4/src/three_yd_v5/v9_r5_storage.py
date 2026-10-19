"""A complete task's rectangle on the original pixel lattice; no moving window."""
import math
from types import SimpleNamespace
import numpy as np

def bounds(mask,resolution):
 if abs(resolution-.05)>1e-9:raise ValueError('R5 original 0.05m lattice required')
 a=np.asarray(mask,bool);rows=np.flatnonzero(a.any(axis=1));cols=np.flatnonzero(a.any(axis=0))
 if not len(rows):raise ValueError('R5 complete route scope is empty')
 halo=math.ceil(.55/resolution)+2
 return (max(0,int(rows[0])-halo),min(a.shape[0],int(rows[-1])+halo+1),
         max(0,int(cols[0])-halo),min(a.shape[1],int(cols[-1])+halo+1))

def union(a,b):
 if a is None:return b
 return min(a[0],b[0]),max(a[1],b[1]),min(a[2],b[2]),max(a[3],b[3])

def soft_context(source,box):
 """Exact sparse crop; preserve the original TiledSoft row ACK fast path.

 The old TiledSoft slice returns a dense float rectangle. A native map view
 must keep its cap, current drivable view and clipped original tile views.
 All mutable views are reconstructed on each binding, never value-cached.
 """
 from .r4_tiles import TiledSoft
 a,b,c,d=box
 if type(source) is not TiledSoft:return source[a:b,c:d]
 pieces=[]
 for (p,q,r,s),values in source.pieces:
  y0,y1=max(a,p),min(b,q);x0,x1=max(c,r),min(d,s)
  if y0<y1 and x0<x1:
   pieces.append(((y0-a,y1-a,x0-c,x1-c),values[y0-p:y1-p,x0-r:x1-r]))
 return TiledSoft((b-a,d-c),source.cap,source.drivable[a:b,c:d],pieces)

class CroppedMap:
 def __init__(self,source,box):
  a,b,c,d=box;self.source=source;self.bounds=tuple(box)
  self.height=b-a;self.width=d-c;self.resolution=source.resolution
  self.origin=(source.origin[0]+c*self.resolution,source.origin[1]+(source.height-b)*self.resolution,0.)
  self.occupancy=source.occupancy[a:b,c:d];self.distance_m=source.distance_m[a:b,c:d]
 def __getattr__(self,name):return getattr(self.source,name)
 def world_to_cell(self,x,y):
  q=self.source.world_to_cell(x,y)
  if q is None:return None
  a,b,c,d=self.bounds;r,s=q[0]-a,q[1]-c
  return (r,s) if 0<=r<b-a and 0<=s<d-c else None
 def cell_to_world(self,q):return self.source.cell_to_world((q[0]+self.bounds[0],q[1]+self.bounds[2]))

def context(global_ctx,box):
 a,b,c,d=box
 return SimpleNamespace(**{**vars(global_ctx),'hospital_map':CroppedMap(global_ctx.hospital_map,box),
                          'free_mask':global_ctx.free_mask[a:b,c:d],'distance_m':global_ctx.distance_m[a:b,c:d]})
