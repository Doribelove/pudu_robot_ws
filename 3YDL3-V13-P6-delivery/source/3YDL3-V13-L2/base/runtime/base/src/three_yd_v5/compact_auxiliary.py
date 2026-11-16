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
    def from_array(cls,value,cache_digest=False):
        a=np.asarray(value);assert a.ndim==2 and a.dtype==np.dtype(bool)
        packed=np.packbits(a,axis=1,bitorder='big')
        stored=np.frombuffer(packed.tobytes(),np.uint8).reshape(packed.shape)
        result=cls(stored,tuple(a.shape))
        if cache_digest:result.content_sha256
        return result
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
        # Point gathers and row/rectangle readers never unpack the complete map.
        if not isinstance(index,tuple):index=(index,slice(None))
        if len(index)==2:
            rr,cc=index
            if isinstance(rr,(int,np.integer)) and isinstance(cc,(int,np.integer)):
                if rr < -self.shape[0] or rr >= self.shape[0] or cc < -self.shape[1] or cc >= self.shape[1]:raise IndexError('packed mask index')
                rr=int(rr)%self.shape[0];cc=int(cc)%self.shape[1]
                return np.bool_((self.packed[rr,cc//8] >> (7-cc%8))&1)
            if all(isinstance(v,(int,np.integer,slice)) for v in index):
                row_int=isinstance(rr,(int,np.integer));col_int=isinstance(cc,(int,np.integer))
                if row_int:
                    if rr < -self.shape[0] or rr >= self.shape[0]:raise IndexError('packed mask row')
                    rr=slice(int(rr)%self.shape[0],int(rr)%self.shape[0]+1)
                if col_int:
                    if cc < -self.shape[1] or cc >= self.shape[1]:raise IndexError('packed mask column')
                    cc=slice(int(cc)%self.shape[1],int(cc)%self.shape[1]+1)
                r0,r1,rs=rr.indices(self.shape[0]);c0,c1,cs=cc.indices(self.shape[1])
                # Row strides, including reversed rows, stay bounded by the requested rows.
                if cs==1:
                    nc=max(0,c1-c0);first=c0//8;last=(c1+7)//8
                    values=np.unpackbits(self.packed[index[0] if not row_int else rr,first:last],axis=-1,bitorder='big')
                    value=values[...,c0-first*8:c0-first*8+nc].view(bool)
                    if row_int:value=value[0]
                    if col_int:value=value[...,0]
                    return value
            if not isinstance(rr,slice) and not isinstance(cc,slice):
                yy,xx=np.broadcast_arrays(np.asarray(rr),np.asarray(cc))
                if np.any((yy < -self.shape[0])|(yy >= self.shape[0])|(xx < -self.shape[1])|(xx >= self.shape[1])):raise IndexError('packed mask gather')
                yy=yy%self.shape[0];xx=xx%self.shape[1]
                return ((self.packed[yy,xx//8] >> (7-xx%8))&1).astype(bool,copy=False)
        return np.asarray(self)[index]
    @property
    def content_sha256(self):
        # Only immutable original bits enter this cache; match the old bool-byte digest.
        old=self.__dict__.get('_content_sha256')
        if old is not None and old[0]() is self.packed and old[1]==self.shape:return old[2]
        import hashlib,weakref
        h=hashlib.sha256()
        for a in range(0,len(self),64):h.update(np.ascontiguousarray(self[a:a+64,:]))
        value=h.hexdigest();object.__setattr__(self,'_content_sha256',(weakref.ref(self.packed),self.shape,value));return value
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


def native_static_view(grid,occupancy,box):
    """Bind an already certified original LUT to the owned native occupancy crop.

    Do not keep a strided crop of the obsolete global publication alive. Other
    representations or a substituted source retain the original slice path.
    """
    if (type(grid.internal_cost) is StaticFromOccupancy
        and grid.internal_cost.occupancy is grid.occupancy_grid):
        return StaticFromOccupancy(occupancy)
    a,b,c,d=box
    return grid.internal_cost[a:b,c:d]
