"""Query-independent compressed tile graph preparation with a byte-bounded LRU."""
from collections import OrderedDict
import sys
import time
from .tiled_topology_r3 import TiledTopology, check_deadline


def retained_size(value,seen=None):
    seen=set() if seen is None else seen
    if id(value) in seen:return 0
    seen.add(id(value));size=sys.getsizeof(value)
    if isinstance(value,dict):return size+sum(retained_size(k,seen)+retained_size(v,seen) for k,v in value.items())
    if isinstance(value,(tuple,list)):return size+sum(retained_size(v,seen) for v in value)
    if hasattr(value,'__dict__'):return size+retained_size(vars(value),seen)
    return size


class ReusableTileTopology(TiledTopology):
    """Dense tile arrays retain the original four-tile bound.

    This separate cache owns only compressed graphs, after the existing disk
    binding/hash checks. Query artifacts copy their mutable geometry as before.
    """
    def __init__(self,*args,graph_limit_bytes=128*1024**2,**kwargs):
        if graph_limit_bytes<1:raise ValueError('positive graph byte limit required')
        super().__init__(*args,**kwargs)
        self.reusable_graphs=OrderedDict();self.graph_bytes=0;self.graph_limit_bytes=graph_limit_bytes
        self.stats.update(reusable_graph_hits=0,reusable_graph_misses=0,reusable_graph_bytes=0)

    def refine(self,t,*,deadline=None):
        check_deadline(deadline,'reusable_tile_graph')
        if t in self.reusable_graphs:
            value,size=self.reusable_graphs.pop(t);self.reusable_graphs[t]=(value,size)
            self.stats['reusable_graph_hits']+=1;return value
        self.stats['reusable_graph_misses']+=1
        value=super().refine(t,deadline=deadline);size=retained_size(value)
        if size<=self.graph_limit_bytes:
            while self.reusable_graphs and self.graph_bytes+size>self.graph_limit_bytes:
                _,(_,old)=self.reusable_graphs.popitem(last=False);self.graph_bytes-=old
            self.reusable_graphs[t]=(value,size);self.graph_bytes+=size
        self.stats['reusable_graph_bytes']=self.graph_bytes
        return value

    def prepare_map(self):
        begin=time.monotonic();cpu=time.process_time()
        # Map-coordinate order only; no queries, poses, or query certificates.
        for tile in self.tiles:self.refine(tile)
        if not self.validate_seams():raise ValueError('prepared seam certificate invalid')
        return {'reusable_tile_prepare_wall_ms':(time.monotonic()-begin)*1000,
                'reusable_tile_prepare_cpu_ms':(time.process_time()-cpu)*1000,
                'reusable_tile_count':len(self.tiles),'reusable_graph_retained_count':len(self.reusable_graphs),
                'reusable_graph_bytes':self.graph_bytes,'reusable_graph_limit_bytes':self.graph_limit_bytes,
                'query_dependent_work':False,'seam_valid':True}
