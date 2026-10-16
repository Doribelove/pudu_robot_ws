"""Build the frozen audit's exact near-cell predicate without a global EDT.

The kernel uses the same SciPy EDT arithmetic on integer lattice offsets.
An ambiguity guard retains the parent constructor near threshold boundaries.
This is a decision field, never a replacement numerical distance field.
"""
import math,time,hashlib,inspect
import cv2
import numpy as np
from scipy import ndimage
from arena_evaluation.path_audit import PathAuditor,legacy
from .v9_storage import StorageAuditor,AuditClearanceGate,audit_binding

# Only bypass the frozen parent's known constructor, whose complete state is
# ctx/source_commit/footprint plus this decision field. An upstream change
# must run its own constructor rather than silently omit new state.
PARENT_INIT_SHA256='745cff76038ed567c287319415467d7b1f97a1086c926760f50b3f5d46e75960'


def lattice_kernel(binding):
    res=binding[2];cutoff=binding[-1]+1e-12
    if not math.isfinite(res) or res<=0 or not math.isfinite(cutoff):raise ValueError('GATE_INVALID_GEOMETRY')
    radius=int(math.ceil(cutoff/res))
    if radius<1 or radius>128:raise ValueError('GATE_KERNEL_LIMIT')
    shape=(2*radius+1,2*radius+1)
    source=np.ones(shape,bool);source[radius,radius]=False
    distance=ndimage.distance_transform_edt(source,sampling=res)
    # EDT tie choices can differ in floating arithmetic at an exact level.
    # Do not claim morphological equivalence when the cutoff is ambiguous.
    margin=float(np.min(abs(distance-cutoff)))
    if margin<=max(1e-10,128*np.finfo(float).eps*max(1.,cutoff)):
        raise ValueError('GATE_THRESHOLD_LATTICE_AMBIGUITY')
    return (distance<=cutoff).astype(np.uint8),radius,margin


def direct_gate(occupancy,binding,rows=96):
    occupancy=np.asarray(occupancy)
    if occupancy.ndim!=2 or occupancy.shape!=binding[1] or min(occupancy.shape)<=0:
        raise ValueError('GATE_DIMENSIONS')
    kernel,radius,margin=lattice_kernel(binding)
    gate=AuditClearanceGate.__new__(AuditClearanceGate)
    gate.binding=binding;gate.shape=occupancy.shape;gate.ndim=2
    gate.cutoff=binding[-1]+1e-12;gate.far=float(np.nextafter(gate.cutoff,np.inf))
    packed=np.empty((occupancy.shape[0],(occupancy.shape[1]+7)//8),np.uint8)
    unsafe=np.asarray(occupancy!=0,np.uint8)
    has_unsafe=bool(np.any(unsafe))
    for a in range(0,len(unsafe),rows):
        b=min(len(unsafe),a+rows)
        if has_unsafe:
            lo=max(0,a-radius);hi=min(len(unsafe),b+radius)
            # Zero outside the image means "no represented unsafe cell";
            # it must not invent obstacles beyond the map's EDT domain.
            expanded=cv2.dilate(unsafe[lo:hi],kernel,borderType=cv2.BORDER_CONSTANT,borderValue=0)
            near=expanded[a-lo:b-lo]!=0
        else:
            # SciPy's all-traversable special case uses virtual site (-1,0).
            rr=np.arange(a,b,dtype=float)[:,None]+1.;cc=np.arange(occupancy.shape[1],dtype=float)[None,:]
            near=np.sqrt((rr*binding[2])**2+(cc*binding[2])**2)<=gate.cutoff
        packed[a:b]=np.packbits(near,axis=1,bitorder='big')
    gate.packed=np.frombuffer(packed.tobytes(),np.uint8).reshape(packed.shape)
    gate.original_bytes=int(occupancy.size*8)
    return gate,{'method':'guarded_exact_lattice_dilation','kernel_shape':list(kernel.shape),
        'kernel_cells':int(kernel.sum()),'lattice_cutoff_margin_m':margin,
        'all_traversable_virtual_site':not has_unsafe,'global_float64_EDT_built':False,
        'scratch_rows':rows+2*radius,'stored_bytes':gate.nbytes,'equivalence_guard_passed':True}


class DirectStorageAuditor(StorageAuditor):
    def __init__(self,ctx,*,source_commit,footprint=legacy.FOOTPRINT):
        from .v9_r2_options import enabled
        from .v9_review import enabled as review_enabled
        tick=time.monotonic()
        parent_matches=hashlib.sha256(inspect.getsource(PathAuditor.__init__).encode()).hexdigest()==PARENT_INIT_SHA256
        if enabled('D') and review_enabled('S') and not getattr(ctx,'bounded_memory',False) and parent_matches:
            self.ctx=ctx;self.source_commit=str(source_commit or 'unknown')
            self.footprint=[[float(v) for v in vertex] for vertex in footprint]
            try:
                gate,detail=direct_gate(ctx.hospital_map.occupancy,audit_binding(self))
            except ValueError as e:
                fallback=str(e)
            else:
                self._distance_to_unsafe_m=gate
                self.storage_diagnostics={'enabled':True,'compacted':True,'direct_initialized':True,
                    'original_bytes':gate.original_bytes,**detail,'preparation_ms':(time.monotonic()-tick)*1000,
                    'parent_audit_source_sha256':hashlib.sha256(inspect.getsource(PathAuditor).encode()).hexdigest()}
                return
        else:fallback='PARENT_CONSTRUCTOR_CHANGED' if not parent_matches else 'DIRECT_DISABLED_OR_CONTEXT_UNSUPPORTED'
        super().__init__(ctx,source_commit=source_commit,footprint=footprint)
        self.storage_diagnostics.update(direct_initialized=False,direct_fallback=fallback,
            total_constructor_ms=(time.monotonic()-tick)*1000)
