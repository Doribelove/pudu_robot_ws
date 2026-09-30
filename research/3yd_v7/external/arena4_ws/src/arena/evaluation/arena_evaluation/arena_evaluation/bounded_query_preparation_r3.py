"""Byte-bounded query artifacts; shared map storage is accounted separately."""
import time
from .reusable_tile_preparation_r3 import retained_size as python_retained_size
from .reachable_endpoint_r3 import _native_geometry

def retained_size(value,seen=None):
    excluded=set() if seen is None else seen
    if _native_geometry is not None and hasattr(_native_geometry,'retained_size'):
        return _native_geometry.retained_size(value,excluded)
    return python_retained_size(value,seen=excluded)
from .two_layer_v1_r3_benchmark import QueryTopologyPreparer
from .tiled_topology_r3 import check_deadline

MAX_QUERY_CACHE_BYTES=3*1024**3


class BoundedQueryTopologyPreparer(QueryTopologyPreparer):
    def __init__(self,*args,max_bytes=MAX_QUERY_CACHE_BYTES,**kwargs):
        if not isinstance(max_bytes,int) or isinstance(max_bytes,bool) or not 1<=max_bytes<=MAX_QUERY_CACHE_BYTES:
            raise ValueError('QUERY_CACHE_BYTE_LIMIT_INVALID')
        super().__init__(*args,**kwargs)
        self.max_bytes=max_bytes;self.bytes=0;self.sizes={};self.evictions=0
        self.stamps={};self.size_scans=0;self.static_sizes={};self.mutable_sizes={}
        self.immutable_size_scans=0

    @staticmethod
    def _mutable(value):
        _,selector=value;connector=getattr(selector,'connector',None)
        return (getattr(selector,'cache',None),getattr(selector,'last_certificate',None),
                *(getattr(connector,name,None) for name in
                  ('_distance_window','_local_free_window','_obstacle_field_cache')))

    @staticmethod
    def _static_roots(value):
        topology,selector=value;connector=getattr(selector,'connector',None)
        return (topology,selector,connector,topology.hospital_map,
                *(getattr(selector,name,None) for name in ('index','edges','binding','config','footprint')),
                *(getattr(connector,name,None) for name in ('map','config','footprint','_native_checker')))

    @staticmethod
    def _stamp(value):
        _,selector=value
        cache=getattr(selector,'cache',{})
        # Immutable certificate payloads reconstruct the same bounded result.
        # Changing request IDs and integer counters fit the per-entry reserve.
        if not isinstance(cache,dict):return None
        connector=getattr(selector,'connector',None)
        return (tuple((k,id(v),len(v)) for k,v in cache.items()),
                tuple(id(getattr(connector,name,None)) for name in
                      ('_distance_window','_local_free_window','_obstacle_field_cache')))

    def reconcile(self):
        active=(self.last_preparation or {}).get('key')
        for name in ('sizes','stamps','static_sizes','mutable_sizes'):
            setattr(self,name,{key:v for key,v in getattr(self,name).items() if key in self.entries})
        for key,value in self.entries.items():
            stamp=self._stamp(value)
            if key not in self.static_sizes:
                # Graphs and spatial indexes are immutable after construction.
                # Measure them once, without retaining a graph-sized set of IDs.
                excluded={id(value[0].hospital_map),*(id(v) for v in self._mutable(value))}
                self.static_sizes[key]=retained_size(value,seen=excluded)
                self.immutable_size_scans+=1
            if key not in self.sizes or (key==active and (stamp is None or self.stamps.get(key)!=stamp)):
                # Connector scratch windows and certificates can grow. Their
                # accounting must not walk the immutable graph a second time.
                excluded={id(v) for v in self._static_roots(value)}
                self.mutable_sizes[key]=retained_size(self._mutable(value),seen=excluded)
                self.sizes[key]=self.static_sizes[key]+self.mutable_sizes[key]+4096
                self.stamps[key]=stamp;self.size_scans+=1
        self.bytes=sum(self.sizes.values())
        while self.entries and self.bytes>self.max_bytes:
            key,_=self.entries.popitem(last=False)
            self.bytes-=self.sizes.pop(key);self.evictions+=1
            for values in (self.stamps,self.static_sizes,self.mutable_sizes):values.pop(key,None)
        return {'query_topology_retained_bytes':self.bytes,
                'query_topology_byte_limit':self.max_bytes,'query_topology_cache_entries':len(self.entries),
                'query_topology_evictions':self.evictions,'query_topology_size_scans':self.size_scans,
                'query_topology_immutable_size_scans':self.immutable_size_scans}

    def resolve(self,query,deadline,timing):
        try:result=super().resolve(query,deadline,timing)
        finally:
            started=time.monotonic();timing.update(self.reconcile())
            cost=(time.monotonic()-started)*1000
            timing['query_topology_size_accounting_ms']=cost
            timing['query_topology_prepare_wall_ms']=timing.get('query_topology_prepare_wall_ms',0.)+cost
        check_deadline(deadline,'query_cache_accounting')
        return result
