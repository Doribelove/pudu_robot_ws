"""Small, version-bound caches used by 3YD-V5-r8.

The caches are deliberately process-local.  Every reusable navigation object
is keyed by map, semantic, vehicle, dynamic-map and route/query identity; an
update therefore misses the cache instead of reusing stale planning state.
"""
from collections import OrderedDict
from collections.abc import MutableMapping
import sys
import numpy as np


def resident_bytes(value, seen=None):
    """Conservative resident-size estimate with ndarray buffers counted once."""
    if seen is None:
        seen=set()
    identity=id(value)
    if identity in seen:
        return 0
    seen.add(identity)
    if isinstance(value,np.ndarray):
        return int(value.nbytes)
    if isinstance(value,dict):
        return int(sys.getsizeof(value)+sum(resident_bytes(k,seen)+resident_bytes(v,seen)
                                             for k,v in value.items()))
    if isinstance(value,(tuple,list,set,frozenset)):
        return int(sys.getsizeof(value)+sum(resident_bytes(x,seen) for x in value))
    if hasattr(value,'__dict__'):
        return int(sys.getsizeof(value)+resident_bytes(vars(value),seen))
    return int(sys.getsizeof(value))


class BoundedLRU(MutableMapping):
    """A mapping with deterministic entry and approximate byte limits."""
    def __init__(self,max_entries,max_bytes=None):
        self.max_entries=int(max_entries)
        self.max_bytes=None if max_bytes is None else int(max_bytes)
        self._data=OrderedDict();self._sizes={};self.bytes=0;self.evictions=0

    def __getitem__(self,key):
        value=self._data.pop(key);self._data[key]=value
        return value

    def get(self,key,default=None):
        try:return self[key]
        except KeyError:return default

    def peek(self,key,default=None):
        return self._data.get(key,default)

    def __setitem__(self,key,value):
        if key in self._data:
            self._data.pop(key);self.bytes-=self._sizes.pop(key)
        # Entry-bounded caches may contain object graphs that point back to the
        # complete world.  Do not traverse those graphs merely for telemetry.
        size=resident_bytes(value) if self.max_bytes is not None else 0
        self._data[key]=value;self._sizes[key]=size;self.bytes+=size
        while len(self._data)>self.max_entries or (
                self.max_bytes is not None and self.bytes>self.max_bytes and len(self._data)>1):
            old,_=self._data.popitem(last=False)
            self.bytes-=self._sizes.pop(old);self.evictions+=1

    def __delitem__(self,key):
        del self._data[key];self.bytes-=self._sizes.pop(key)

    def __iter__(self):return iter(self._data)
    def __len__(self):return len(self._data)
    def clear(self):
        self._data.clear();self._sizes.clear();self.bytes=0

    def diagnostics(self):
        return {'entries':len(self),'resident_bytes_estimate':int(self.bytes),
                'max_entries':self.max_entries,'max_bytes':self.max_bytes,
                'evictions':self.evictions}


class RoiGrid:
    """Read-only ndarray-like field stored only inside a global-grid ROI."""
    def __init__(self,local,bbox,shape,fill):
        self.local=np.asarray(local);self.bbox=tuple(map(int,bbox));self.shape=tuple(shape)
        self.fill=fill

    @property
    def ndim(self):return len(self.shape)
    @property
    def dtype(self):return self.local.dtype
    @property
    def nbytes(self):return int(self.local.nbytes)

    def __getitem__(self,index):
        if not isinstance(index,tuple) or len(index)<2:
            raise TypeError('RoiGrid requires row/column indexing')
        rr,cc=np.broadcast_arrays(np.asarray(index[0]),np.asarray(index[1]))
        r0,r1,c0,c1=self.bbox
        inside=(rr>=r0)&(rr<r1)&(cc>=c0)&(cc<c1)
        tail=self.local.shape[2:]
        result=np.empty(rr.shape+tail,dtype=self.local.dtype)
        result[...] = self.fill
        if np.any(inside):result[inside]=self.local[rr[inside]-r0,cc[inside]-c0]
        if result.ndim==0:return result.item()
        return result


def get_cache(views,name,max_entries,max_bytes=None):
    key=('v5-r8-cache',name)
    cache=views.get(key)
    if cache is None:
        cache=BoundedLRU(max_entries,max_bytes);views[key]=cache
    return cache
