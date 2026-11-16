"""Exact audit predicate storage and bounded-map conversion.

The frozen auditor only uses its unsafe-distance field to select exact
polygon checks. This representation preserves that decision for every cell;
it does not expose or approximate a numerical clearance field.
"""
import math,time
import numpy as np
from arena_evaluation.path_audit import PathAuditor


def audit_binding(auditor):
    m=auditor.ctx.hospital_map
    footprint=tuple(tuple(float(x) for x in v) for v in auditor.footprint)
    threshold=max(math.hypot(*v) for v in footprint)+math.sqrt(2.)*m.resolution/2.
    return footprint,(m.height,m.width),float(m.resolution),threshold


class AuditClearanceGate:
    """Bit-packed exact near/unsafe decisions for the frozen audit threshold."""
    def __init__(self,distance,binding,rows=64):
        if not isinstance(distance,np.ndarray) or distance.ndim!=2:raise TypeError('DENSE_DISTANCE_REQUIRED')
        self.binding=binding;self.shape=distance.shape;self.ndim=2
        self.cutoff=binding[-1]+1.e-12
        self.far=float(np.nextafter(self.cutoff,np.inf))
        if not math.isfinite(self.far):raise ValueError('UNSUPPORTED_AUDIT_THRESHOLD')
        packed=np.empty((distance.shape[0],(distance.shape[1]+7)//8),np.uint8)
        for a in range(0,len(distance),rows):
            values=distance[a:a+rows]
            near=~np.isfinite(values)|(values<=self.cutoff)
            packed[a:a+rows]=np.packbits(near,axis=1,bitorder='big')
            # Verify all decisions, including padding/non-finite boundaries.
            restored=np.unpackbits(packed[a:a+rows],axis=1,count=self.shape[1],bitorder='big')
            if not np.array_equal(near,restored):raise RuntimeError('AUDIT_GATE_PACKING_MISMATCH')
        self.packed=np.frombuffer(packed.tobytes(),np.uint8).reshape(packed.shape)
        self.original_bytes=int(distance.nbytes)

    @property
    def nbytes(self):return int(self.packed.nbytes)

    def __getitem__(self,key):
        if not isinstance(key,tuple) or len(key)!=2:raise TypeError('AUDIT_GATE_REQUIRES_CELL_INDICES')
        rr,cc=np.broadcast_arrays(np.asarray(key[0]),np.asarray(key[1]))
        if rr.dtype.kind not in 'iu' or cc.dtype.kind not in 'iu':raise TypeError('INTEGER_CELL_INDICES_REQUIRED')
        if np.any((rr< -self.shape[0])|(rr>=self.shape[0])|(cc< -self.shape[1])|(cc>=self.shape[1])):
            raise IndexError('audit cell outside map')
        rr=rr%self.shape[0];cc=cc%self.shape[1]
        near=((self.packed[rr,cc//8]>>(7-cc%8))&1)!=0
        return np.where(near,0.,self.far)

    def __array__(self,*args,**kwargs):
        raise TypeError('AUDIT_GATE_IS_A_DECISION_FIELD_NOT_NUMERICAL_CLEARANCE')


class StorageAuditor(PathAuditor):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        from .v9_review import enabled
        self.storage_diagnostics={'enabled':enabled('S'),'compacted':False}
        if enabled('S') and isinstance(self._distance_to_unsafe_m,np.ndarray):
            tick=time.monotonic();binding=audit_binding(self)
            gate=AuditClearanceGate(self._distance_to_unsafe_m,binding)
            self._distance_to_unsafe_m=gate
            self.storage_diagnostics.update(compacted=True,original_bytes=gate.original_bytes,
                stored_bytes=gate.nbytes,all_cell_near_decisions_verified=True,
                preparation_ms=(time.monotonic()-tick)*1000)

    def audit(self,*args,**kwargs):
        field=self._distance_to_unsafe_m
        if isinstance(field,AuditClearanceGate) and field.binding!=audit_binding(self):
            raise ValueError('AUDIT_GATE_FOOTPRINT_OR_MAP_GEOMETRY_CHANGED')
        return super().audit(*args,**kwargs)


def occupancy_on_free_support(source,soft,hard,rows=96):
    """Same original pre-bound occupancy, converting only non-hard cells."""
    from arena_evaluation.semantic_costmap_composer import internal_soft_to_occupancy
    result=np.full(source.shape,100,np.int8)
    for a in range(0,len(source),rows):
        b=min(len(source),a+rows);free=~hard[a:b]
        if not np.any(free):continue
        raw=source[a:b][free]
        cost=internal_soft_to_occupancy(soft[a:b,:][free])
        values=np.maximum(raw.astype(np.int16),cost).astype(np.int8)
        values[raw<0]=-1
        result[a:b][free]=values
    return result


def static_lut(occupancy):
    """Original StaticLayer conversion for all 256 int8 input values."""
    from arena_evaluation.semantic_costmap_composer import occupancy_to_static_layer
    table=occupancy_to_static_layer(np.arange(256,dtype=np.uint8).view(np.int8))
    from .v9_r9_lookup import exact_lookup
    result=exact_lookup(occupancy,table)
    if result is not None:return result
    return table[occupancy.view(np.uint8)]
from arena_evaluation.semantic_costmap_composer import SemanticCostmap


class HashSemanticCostmap(SemanticCostmap):
    """Identical properties, streamed live hashes or proven immutable hashes."""
    @property
    def expected_grid_hash(self):
        from .v9_maps import content_digest
        return content_digest(self.occupancy_grid)

    @property
    def expected_master_hash(self):
        from .v9_maps import content_digest
        return content_digest(self.expected_master_cost)
