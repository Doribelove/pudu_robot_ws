"""Lossless storage of owned global costmap auxiliaries.

Occupancy and expected-master publication bytes remain ordinary immutable
arrays. Static costs are the original LUT of occupancy; the hard mask stores
all original bits. Native crops are ordinary arrays and undergo the same ACK.
"""
from dataclasses import dataclass, replace
import numpy as np


@dataclass(frozen=True,eq=False)
class StaticFromOccupancy:
    occupancy: np.ndarray
    @property
    def shape(self):return self.occupancy.shape
    @property
    def dtype(self):return np.dtype('uint8')
    @property
    def ndim(self):return 2
    @property
    def size(self):return self.occupancy.size
    @property
    def nbytes(self):return 0  # shared publication owner is counted separately
    def __len__(self):return self.shape[0]
    def __getitem__(self,index):
        from .v9_storage import static_lut
        return static_lut(self.occupancy[index])
    def __array__(self,dtype=None):
        from .v9_storage import static_lut
        value=static_lut(self.occupancy)
        return value if dtype is None else value.astype(dtype,copy=False)
    def binding(self):
        from .v9_maps import content_digest
        return ('static-from-original-LUT',id(self),self.shape,str(self.dtype),content_digest(self.occupancy))


@dataclass(frozen=True,eq=False)
class PackedHardMask:
    packed: np.ndarray
    shape: tuple
    @classmethod
    def from_array(cls,value):
        a=np.asarray(value);assert a.ndim==2 and a.dtype==np.dtype(bool)
        packed=np.packbits(a,axis=1,bitorder='big')
        stored=np.frombuffer(packed.tobytes(),np.uint8).reshape(packed.shape)
        return cls(stored,tuple(a.shape))
    @property
    def dtype(self):return np.dtype(bool)
    @property
    def ndim(self):return 2
    @property
    def size(self):return int(np.prod(self.shape))
    @property
    def nbytes(self):return self.packed.nbytes
    def __len__(self):return self.shape[0]
    def __array__(self,dtype=None):
        value=np.unpackbits(self.packed,axis=1,count=self.shape[1],bitorder='big').view(bool)
        return value if dtype is None else value.astype(dtype,copy=False)
    def __getitem__(self,index):
        if isinstance(index,tuple) and len(index)==2 and all(isinstance(s,slice) for s in index):
            r0,r1,rs=index[0].indices(self.shape[0]);c0,c1,cs=index[1].indices(self.shape[1])
            if rs==1 and cs==1:
                nr=max(0,r1-r0);nc=max(0,c1-c0)
                if not nr or not nc:return np.empty((nr,nc),bool)
                first=c0//8;last=(c1+7)//8
                values=np.unpackbits(self.packed[r0:r1,first:last],axis=1,bitorder='big')
                return values[:,c0-first*8:c1-first*8].view(bool)
        return np.asarray(self)[index]
    def binding(self):
        from .v9_maps import content_digest
        return ('all-hard-bits-big-endian',id(self),self.shape,str(self.dtype),content_digest(self.packed))


TYPES=(StaticFromOccupancy,PackedHardMask)


def compact_owned_grid(grid,derive_static=True):
    """Only freshly composed, large global fields with the known exact LUT."""
    from .v9_maps import seal
    from .v9_storage import static_lut
    if (type(grid.occupancy_grid) is not np.ndarray or grid.occupancy_grid.dtype!=np.int8
        or grid.occupancy_grid.ndim!=2 or grid.occupancy_grid.size<2**20
        or type(grid.internal_cost) is not np.ndarray or type(grid.hard_semantic_mask) is not np.ndarray
        or grid.internal_cost.shape!=grid.occupancy_grid.shape
        or grid.hard_semantic_mask.shape!=grid.occupancy_grid.shape
        or grid.hard_semantic_mask.dtype!=np.dtype(bool)):
        return grid
    # Check the complete original static field before discarding its storage;
    # unusual/custom maps keep their original representation.
    if derive_static:
        for a in range(0,len(grid.occupancy_grid),96):
            if not np.array_equal(grid.internal_cost[a:a+96],static_lut(grid.occupancy_grid[a:a+96])):
                return grid
    occupancy=seal(grid.occupancy_grid)
    packed=PackedHardMask.from_array(grid.hard_semantic_mask)
    static=StaticFromOccupancy(occupancy) if derive_static else grid.internal_cost
    return replace(grid,occupancy_grid=occupancy,internal_cost=static,
        hard_semantic_mask=packed,diagnostics={**grid.diagnostics,'r11_compact_auxiliary':True,
        'r11_static_LUT_all_cells_verified':bool(derive_static),'r11_hard_mask_exact_packed':True,
        'r11_static_storage':'derived_original_LUT' if derive_static else 'original_dense',
        'r11_auxiliary_dense_bytes':int(grid.internal_cost.nbytes+grid.hard_semantic_mask.nbytes),
        'r11_auxiliary_stored_bytes':int(packed.nbytes+(0 if derive_static else static.nbytes))})
