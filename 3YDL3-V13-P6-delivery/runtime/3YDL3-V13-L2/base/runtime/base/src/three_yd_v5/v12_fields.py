"""P2 disk-backed distance/label planes with a shared bounded tile cache.

Values are the original global computation, never a truncated local EDT.
Implicit conversion to a complete ndarray is forbidden. Boolean masks are
explicit outputs of legacy mask consumers and remain separately accounted.
"""
from collections import OrderedDict
from pathlib import Path
import hashlib,json,math,os,shutil,tempfile,zlib,weakref
from contextlib import contextmanager
import numpy as np
from numpy.lib.mixins import NDArrayOperatorsMixin
from .v12_contracts import ContractError,FailureCode
from .v12_tiles import digest


class PlaneCache:
    def __init__(self,budget_bytes=16*2**20):
        self.budget=int(budget_bytes);self.data=OrderedDict();self.bytes=0
        self.planes=[]
        self.stats={'loads':0,'hits':0,'constant_reads':0,'evictions':0,'peak_bytes':0,
                    'gather_calls':0,'mask_scans':0,'rejected_full_conversions':0,'compressed_bytes_read':0}
    def clear(self):
        self.data.clear();self.bytes=0
    def snapshot(self):return {**self.stats,'resident_bytes':self.bytes,'budget_bytes':self.budget}
    def verify_files(self):
        for ref in self.planes:
            plane=ref()
            if plane is not None:plane.verify_files()
    def load(self,key,plane,meta,shape):
        if key in self.data:
            self.stats['hits']+=1;self.data.move_to_end(key);return self.data[key]
        try:
            raw=os.pread(plane.fd,meta['length'],meta['offset'])
            self.stats['compressed_bytes_read']+=len(raw)
            if hashlib.sha256(raw).hexdigest()!=meta['sha256']:raise ValueError('payload SHA')
            decoded=zlib.decompress(raw)
            if 'palette_hex' in meta:
                expected=(shape[0]*shape[1]*meta['bits']+7)//8
                if len(decoded)!=expected:raise ValueError('packed label length')
                a=np.frombuffer(decoded,np.uint8)
            else:
                if len(decoded)!=shape[0]*shape[1]*plane.dtype.itemsize:raise ValueError('tile length')
                a=np.frombuffer(decoded,dtype=plane.dtype).reshape(shape)
        except (OSError,ValueError,zlib.error) as e:raise ContractError(FailureCode.TILE_CORRUPT,key) from e
        if a.dtype.hasobject or a.nbytes>self.budget:raise ContractError(FailureCode.TILE_BUDGET,key)
        while self.data and self.bytes+a.nbytes>self.budget:
            _,old=self.data.popitem(last=False);self.bytes-=old.nbytes;self.stats['evictions']+=1
        # Immutable bytes prevent a caller from re-enabling writes to cache data.
        self.data[key]=a;self.bytes+=a.nbytes;self.stats['loads']+=1
        self.stats['peak_bytes']=max(self.stats['peak_bytes'],self.bytes)
        return a


class Plane:
    SCHEMA='3yd-v12-blocked-plane-v4'
    @classmethod
    def write(cls,root,a,binding,tile=256):
        root=Path(root);a=np.asarray(a)
        if a.ndim!=2 or a.dtype.hasobject:raise ValueError('numeric 2D plane required')
        root.parent.mkdir(parents=True,exist_ok=True)
        temp=Path(tempfile.mkdtemp(prefix='.plane-',dir=root.parent));blocks={}
        try:
            with (temp/'blocks.bin').open('wb') as stream:
                for r in range(0,a.shape[0],tile):
                    for c in range(0,a.shape[1],tile):
                        name=f'{r//tile}_{c//tile}';v=np.ascontiguousarray(a[r:r+tile,c:c+tile])
                        raw=v.view(np.uint8).reshape(-1,a.dtype.itemsize)
                        if np.all(raw==raw[0]):blocks[name]={'constant_hex':raw[0].tobytes().hex()}
                        else:
                            descriptor={};encoded=v.tobytes()
                            if a.dtype.kind in 'iu' and a.dtype.itemsize<=2:
                                palette,inverse=np.unique(v,return_inverse=True)
                                if len(palette)<=256:
                                    bits=next(b for b in (1,2,4,8) if len(palette)<=(1<<b))
                                    factor=8//bits;codes=inverse.astype(np.uint8)
                                    codes=np.pad(codes,(0,(-len(codes))%factor)).reshape(-1,factor)
                                    shifts=np.arange(factor-1,-1,-1,dtype=np.uint8)*bits
                                    packed=np.bitwise_or.reduce(codes<<shifts,axis=1)
                                    encoded=packed.tobytes();descriptor={'palette_hex':palette.tobytes().hex(),'bits':bits}
                            packet=zlib.compress(encoded,1);offset=stream.tell();stream.write(packet)
                            blocks[name]={**descriptor,'sha256':hashlib.sha256(packet).hexdigest(),'offset':offset,'length':len(packet)}
            values=hashlib.sha256()
            for y in range(0,a.shape[0],64):values.update(np.ascontiguousarray(a[y:y+64]))
            m={'schema':cls.SCHEMA,'shape':list(a.shape),'dtype':a.dtype.str,'tile':tile,
               'binding':binding,'blocks':blocks,'value_sha256':values.hexdigest(),
               'payload_sha256':digest(temp/'blocks.bin')}
            (temp/'plane.json').write_text(json.dumps(m,sort_keys=True,allow_nan=False)+'\n')
            (temp/'plane.sha256').write_text(digest(temp/'plane.json')+'\n')
            os.rename(temp,root)
        except BaseException:shutil.rmtree(temp,ignore_errors=True);raise
    def __init__(self,root,binding,cache):
        self.root=Path(root);self.cache=cache
        try:
            if digest(self.root/'plane.json')!=(self.root/'plane.sha256').read_text().strip():raise ValueError('manifest SHA')
            m=json.loads((self.root/'plane.json').read_text())
            if m['schema']!=self.SCHEMA or m['binding']!=binding:raise ValueError('binding/schema')
            self.shape=tuple(m['shape']);self.dtype=np.dtype(m['dtype']);self.tile=int(m['tile']);self.blocks=m['blocks']
            self.value_sha256=m['value_sha256']
            expected={f'{r}_{c}' for r in range(math.ceil(self.shape[0]/self.tile)) for c in range(math.ceil(self.shape[1]/self.tile))}
            if (len(self.shape)!=2 or min(self.shape)<1 or self.dtype.hasobject or self.tile<1
                    or set(self.blocks)!=expected):raise ValueError('plane geometry')
            for block in self.blocks.values():
                if 'constant_hex' in block and len(bytes.fromhex(block['constant_hex']))!=self.dtype.itemsize:raise ValueError('constant size')
                if 'palette_hex' in block and (block['bits'] not in (1,2,4,8) or len(bytes.fromhex(block['palette_hex']))%self.dtype.itemsize):raise ValueError('palette')
            size=(self.root/'blocks.bin').stat().st_size
            for b in self.blocks.values():
                if 'constant_hex' not in b and (b['offset']<0 or b['length']<=0 or b['offset']+b['length']>size):raise ValueError('tile offset')
            before=self.fingerprint(self.root/'blocks.bin')
            if digest(self.root/'blocks.bin')!=m['payload_sha256'] or self.fingerprint(self.root/'blocks.bin')!=before:raise ValueError('payload file SHA/change')
            self.fd=os.open(self.root/'blocks.bin',os.O_RDONLY)
            self.fingerprints={name:self.fingerprint(self.root/name) for name in ('plane.json','plane.sha256','blocks.bin')}
            self.descriptors=None;self._palettes=[];self.descriptor_keys=[]
            if self.dtype.kind in 'iu' and self.dtype.itemsize==2:
                descriptors=[]
                for r in range(math.ceil(self.shape[0]/self.tile)):
                    for c in range(math.ceil(self.shape[1]/self.tile)):
                        name,block,shape=self.geometry(r,c);row=[0,r*self.tile,c*self.tile,*shape,0,0,0,0,0,0]
                        if 'constant_hex' in block:row[9]=int(np.frombuffer(bytes.fromhex(block['constant_hex']),np.uint16)[0])
                        elif 'palette_hex' in block:
                            palette=np.frombuffer(bytes.fromhex(block['palette_hex']),np.uint16);self._palettes.append(palette)
                            row[0]=1;row[5]=block['bits'];row[6]=len(palette);row[8]=palette.ctypes.data
                        else:row[0]=2
                        descriptors.append(row);self.descriptor_keys.append((r,c))
                self.descriptors=np.asarray(descriptors,np.uint64)
            cache.planes.append(weakref.ref(self))
        except (ValueError,KeyError,TypeError,OSError) as e:raise ContractError(FailureCode.TILE_CORRUPT,self.root) from e
    def geometry(self,r,c):
        name=f'{r}_{c}';meta=self.blocks[name]
        shape=(min(self.tile,self.shape[0]-r*self.tile),min(self.tile,self.shape[1]-c*self.tile))
        return name,meta,shape
    @staticmethod
    def fingerprint(path):
        s=Path(path).stat();return (s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns)
    def verify_files(self):
        try:
            for name,old in self.fingerprints.items():
                if self.fingerprint(self.root/name)!=old:raise ValueError('plane modified during map lifetime')
        except (OSError,ValueError) as e:raise ContractError(FailureCode.TILE_CORRUPT,self.root) from e
    def mask_table(self,table):
        from .v12_palette_native import needed,full_mask
        table=np.ascontiguousarray(table,np.uint8);desc=self.descriptors.copy()
        required=needed(desc,table);out=np.empty(self.shape,np.bool_)
        # Decode bounded groups into the explicit legacy boolean output.
        # LRU eviction alone is insufficient if descriptors pin every block.
        begin=0;pending=0
        def flush(end):
            owners=[];chunk=desc[begin:end].copy()
            for j in np.flatnonzero(required[begin:end]):
                i=begin+j;a=self.data(*self.descriptor_keys[i]);owners.append(a)
                chunk[j,7]=a.ctypes.data;chunk[j,10]=a.nbytes
            full_mask(chunk,table,self.shape,out=out)
        for i in range(len(desc)):
            size=0
            if required[i]:
                _,meta,shape=self.geometry(*self.descriptor_keys[i])
                cells=shape[0]*shape[1]
                size=(cells*meta['bits']+7)//8 if 'palette_hex' in meta else cells*self.dtype.itemsize
                if size>self.cache.budget:raise ContractError(FailureCode.TILE_BUDGET,self.root)
            if pending+size>self.cache.budget:
                flush(i);begin=i;pending=0
            pending+=size
        if begin<len(desc):flush(len(desc))
        return out
    def data(self,r,c):
        name,meta,shape=self.geometry(r,c)
        return self.cache.load((str(self.root),name),self,meta,shape)
    def codes(self,packed,meta,shape):
        bits=meta['bits'];factor=8//bits;shifts=np.arange(factor-1,-1,-1,dtype=np.uint8)*bits
        return ((packed[:,None]>>shifts)&((1<<bits)-1)).ravel()[:shape[0]*shape[1]].reshape(shape)
    def block(self,r,c):
        name,meta,shape=self.geometry(r,c)
        if 'constant_hex' in meta:
            self.cache.stats['constant_reads']+=1
            return np.broadcast_to(np.frombuffer(bytes.fromhex(meta['constant_hex']),dtype=self.dtype),shape)
        a=self.data(r,c)
        if 'palette_hex' in meta:
            palette=np.frombuffer(bytes.fromhex(meta['palette_hex']),dtype=self.dtype)
            return palette[self.codes(a,meta,shape)]
        if a.shape!=shape or a.dtype!=self.dtype:raise ContractError(FailureCode.TILE_CORRUPT,name)
        return a
    def points(self,r,c,rr,cc):
        name,meta,shape=self.geometry(r,c)
        if 'constant_hex' in meta:
            self.cache.stats['constant_reads']+=1
            return np.full(len(rr),np.frombuffer(bytes.fromhex(meta['constant_hex']),dtype=self.dtype)[0],self.dtype)
        a=self.data(r,c)
        if 'palette_hex' in meta:
            palette=np.frombuffer(bytes.fromhex(meta['palette_hex']),dtype=self.dtype)
            from .v12_palette_native import points
            return points(a,meta['bits'],palette,rr*shape[1]+cc,shape[0]*shape[1])
        return a[rr,cc]
    def predicate(self,r,c,func):
        name,meta,shape=self.geometry(r,c)
        if 'constant_hex' in meta:
            self.cache.stats['constant_reads']+=1
            value=np.frombuffer(bytes.fromhex(meta['constant_hex']),dtype=self.dtype)
            return np.broadcast_to(func(value),shape)
        if 'palette_hex' in meta:
            palette=np.frombuffer(bytes.fromhex(meta['palette_hex']),dtype=self.dtype)
            values=func(palette)
            if np.all(values==values[0]):return np.broadcast_to(values[0],shape)
            from .v12_palette_native import mask
            return mask(self.data(r,c),meta['bits'],values,shape)
        return func(self.data(r,c))
    def field(self):return BlockField(self,(0,self.shape[0],0,self.shape[1]))
    def __del__(self):
        if hasattr(self,'fd'):
            os.close(self.fd);del self.fd


class BlockField(NDArrayOperatorsMixin):
    __array_priority__=1000
    def __init__(self,plane,box,overlay=None,writable=False):
        self.plane=plane;self.box=tuple(box);self.shape=(box[1]-box[0],box[3]-box[2])
        self.dtype=plane.dtype;self.ndim=2;self.size=self.shape[0]*self.shape[1]
        self.overlay={} if overlay is None else overlay
        self.writable=writable
    @property
    def nbytes(self):return self.size*self.dtype.itemsize
    def __len__(self):return self.shape[0]
    def __array__(self,*args,**kwargs):
        self.plane.cache.stats['rejected_full_conversions']+=1
        raise TypeError('V12_BLOCK_FIELD_REQUIRES_EXPLICIT_BOUNDED_READ')
    def __getitem__(self,key):
        if not isinstance(key,tuple):
            if isinstance(key,slice):key=(key,slice(None))
            else:raise TypeError('two cell axes required')
        if len(key)!=2:raise IndexError('two cell axes required')
        r,c=key
        if isinstance(r,slice) and isinstance(c,slice):
            a,b,s=r.indices(self.shape[0]);x,y,t=c.indices(self.shape[1])
            if s==1 and t==1 and a<b and x<y:
                return BlockField(self.plane,(a+self.box[0],b+self.box[0],x+self.box[2],y+self.box[2]),self.overlay,self.writable)
            rr,cc=np.meshgrid(np.arange(a,b,s),np.arange(x,y,t),indexing='ij');return self[rr,cc]
        if isinstance(r,slice) or isinstance(c,slice):
            rr=np.arange(*r.indices(self.shape[0])) if isinstance(r,slice) else np.asarray(r)
            cc=np.arange(*c.indices(self.shape[1])) if isinstance(c,slice) else np.asarray(c)
            if np.ndim(rr) and np.ndim(cc):rr,cc=np.meshgrid(rr,cc,indexing='ij')
            return self[rr,cc]
        rr,cc=np.broadcast_arrays(np.asarray(r),np.asarray(c))
        if rr.dtype.kind not in 'iu' or cc.dtype.kind not in 'iu':raise IndexError('integer cells required')
        if np.any((rr< -self.shape[0])|(rr>=self.shape[0])|(cc< -self.shape[1])|(cc>=self.shape[1])):raise IndexError('cell outside plane')
        rr=rr%self.shape[0]+self.box[0];cc=cc%self.shape[1]+self.box[2]
        shape=rr.shape;rr=rr.ravel();cc=cc.ravel();out=np.empty(len(rr),self.dtype)
        tile=self.plane.tile;width=math.ceil(self.plane.shape[1]/tile)
        code=(rr//tile)*width+cc//tile;order=np.argsort(code,kind='stable')
        sorted_code=code[order];edges=np.r_[0,np.flatnonzero(np.diff(sorted_code))+1,len(order)]
        for a,b in zip(edges[:-1],edges[1:]):
            ids=order[a:b]
            if not len(ids):continue
            tr,tc=divmod(int(sorted_code[a]),width)
            out[ids]=self.plane.points(tr,tc,rr[ids]-tr*tile,cc[ids]-tc*tile)
        if self.overlay:
            for i,linear in enumerate(rr*self.plane.shape[1]+cc):
                if int(linear) in self.overlay:out[i]=self.overlay[int(linear)]
        self.plane.cache.stats['gather_calls']+=1
        return out.reshape(shape)[()] if not shape else out.reshape(shape)
    def tiles(self):
        r0,r1,c0,c1=self.box;t=self.plane.tile
        for tr in range(r0//t,(r1-1)//t+1):
            for tc in range(c0//t,(c1-1)//t+1):
                a,b=max(r0,tr*t),min(r1,(tr+1)*t);x,y=max(c0,tc*t),min(c1,(tc+1)*t)
                v=self.plane.block(tr,tc)[a-tr*t:b-tr*t,x-tc*t:y-tc*t]
                if self.overlay:
                    v=v.copy()
                    for k,value in self.overlay.items():
                        rr,cc=divmod(k,self.plane.shape[1])
                        if a<=rr<b and x<=cc<y:v[rr-a,cc-x]=value
                yield (a-r0,b-r0,x-c0,y-c0),v
    def read(self):
        # Explicit bounded rectangles only. Full logical-plane conversion is
        # forbidden even when a caller asks for it through a slice or copy.
        if self.nbytes>8*2**20:raise TypeError('V12_DENSE_RECTANGLE_EXCEEDS_8_MIB')
        out=np.empty(self.shape,self.dtype)
        for (a,b,c,d),v in self.tiles():out[a:b,c:d]=v
        return out
    def copy(self,*args,**kwargs):
        if self.box!=(0,self.plane.shape[0],0,self.plane.shape[1]) and self.nbytes<=8*2**20:return self.read()
        return BlockField(self.plane,self.box,dict(self.overlay),True)
    def __setitem__(self,key,value):
        if not self.writable:raise TypeError('immutable blocked base plane')
        if not isinstance(key,np.ndarray) or key.dtype!=bool or key.shape!=self.shape or not np.isscalar(value):raise TypeError('sparse overlay requires bool mask and scalar')
        rr,cc=np.nonzero(key)
        for a,b in zip(rr+self.box[0],cc+self.box[2]):self.overlay[int(a*self.plane.shape[1]+b)]=value
    def predicate(self,func):
        hard=getattr(self,'derived_neutral_hard',None)
        if hard is not None and not self.overlay:
            values=func(np.array([0,1],dtype=self.dtype))
            self.plane.cache.stats['mask_scans']+=1
            if values[0]==values[1]:return np.full(self.shape,values[0],bool)
            return hard.copy() if values[0] else np.logical_not(hard)
        if self.plane.descriptors is not None and self.box==(0,self.plane.shape[0],0,self.plane.shape[1]):
            domain=np.arange(65536,dtype=np.uint16).view(self.dtype)
            out=self.plane.mask_table(func(domain));self.plane.cache.stats['mask_scans']+=1
            for k,value in self.overlay.items():
                r,c=divmod(k,self.plane.shape[1]);out[r,c]=func(np.asarray([value],self.dtype))[0]
            return out
        out=np.empty(self.shape,bool)
        r0,r1,c0,c1=self.box;t=self.plane.tile
        for tr in range(r0//t,(r1-1)//t+1):
            for tc in range(c0//t,(c1-1)//t+1):
                a,b=max(r0,tr*t),min(r1,(tr+1)*t);x,y=max(c0,tc*t),min(c1,(tc+1)*t)
                v=self.plane.predicate(tr,tc,func)
                out[a-r0:b-r0,x-c0:y-c0]=v[a-tr*t:b-tr*t,x-tc*t:y-tc*t]
        for k,value in self.overlay.items():
            r,c=divmod(k,self.plane.shape[1])
            if r0<=r<r1 and c0<=c<c1:out[r-r0,c-c0]=func(np.asarray([value],self.dtype))[0]
        self.plane.cache.stats['mask_scans']+=1
        return out
    def membership(self,elements):
        if self.dtype.kind in 'iu' and self.dtype.itemsize<=2:
            domain=np.arange(1<<(8*self.dtype.itemsize),dtype=np.uint16).view(self.dtype) if self.dtype.itemsize==2 else np.arange(256,dtype=np.uint8).view(self.dtype)
            table=np.isin(domain,elements)
            return self.predicate(lambda v:table[v.view(np.uint16 if self.dtype.itemsize==2 else np.uint8)])
        return self.predicate(lambda v:np.isin(v,elements))
    def __array_ufunc__(self,ufunc,method,*inputs,**kwargs):
        if method!='__call__' or kwargs or len(inputs)!=2:return NotImplemented
        if ufunc not in (np.equal,np.not_equal,np.less,np.less_equal,np.greater,np.greater_equal):return NotImplemented
        other=inputs[1] if inputs[0] is self else inputs[0]
        if not np.isscalar(other):return NotImplemented
        return self.predicate(lambda v:ufunc(v,other) if inputs[0] is self else ufunc(other,v))
    def __array_function__(self,func,types,args,kwargs):
        if func is np.isin:return self.membership(args[1]) if not kwargs else NotImplemented
        if func is np.shape:return self.shape
        if func is np.copy:return self.copy()
        if func is np.unique:
            if kwargs:return NotImplemented
            out=np.empty(0,self.dtype)
            for _,v in self.tiles():out=np.union1d(out,np.unique(v))
            return out
        return NotImplemented
    def minimum_at(self,cells):
        cells=list(cells)
        if not cells:return 0.
        if any(c is None for c in cells):return 0.
        a=np.asarray(cells);return float(np.min(self[a[:,0],a[:,1]]))
    def content_sha256(self):
        if not self.writable and not self.overlay and self.box==(0,*self.plane.shape[:1],0,self.plane.shape[1]):
            return self.plane.value_sha256
        h=hashlib.sha256()
        for y in range(0,self.shape[0],64):h.update(self[y:y+64,:].read())
        return h.hexdigest()


def prepare_fields(world,topology,root):
    """Release original owners before GridView/context retain references."""
    root=Path(root)/world.key;pool=PlaneCache();planes={};details=[]
    import inspect,cv2
    from arena_evaluation.planner_benchmark.map_utils import _distance_to_occupied
    from arena_3d_v1.semantic_world import SemanticWorld
    from .free_space_topology import FreeSpaceTopology,MapOnlyWorld
    from .topology import V5Topology
    source_hash=lambda f:hashlib.sha256(inspect.getsource(f).encode()).hexdigest()
    policies={'distance':{'code':source_hash(_distance_to_occupied),'OpenCV':cv2.__version__},
              'topology_labels':{'code':source_hash(FreeSpaceTopology.subregions.fget if getattr(world,'v7_map_only',False) else V5Topology.__init__),'topology_key':topology.key},
              'world_labels':{'code':source_hash(MapOnlyWorld.labels.fget if getattr(world,'v7_map_only',False) else SemanticWorld.__init__)}}
    def field(name,loader):
        binding={'world_key':world.key,'map_sha256':world.map.sha256,'name':name,
                 'source_policy':policies[name],'storage_format':Plane.SCHEMA}
        identity=hashlib.sha256(json.dumps(binding,sort_keys=True).encode()).hexdigest()
        directory=root/name/identity
        if not directory.exists():
            a=loader();Plane.write(directory,a,binding);a=None
        plane=Plane(directory,binding,pool);planes[name]=plane
        details.append({'name':name,'logical_bytes':plane.shape[0]*plane.shape[1]*plane.dtype.itemsize,
                        'disk_bytes':sum(p.stat().st_size for p in directory.iterdir()),
                        'uniform_tiles':sum('constant_hex' in b for b in plane.blocks.values()),
                        'tiles':len(plane.blocks)})
        return plane.field()
    distance=field('distance',lambda:world.map.distance_m)
    # DeferredOccupancyMap exposes a property, HospitalMap a plain field.
    if hasattr(type(world.map),'distance_m') and isinstance(type(world.map).distance_m,property):world.map._distance_m=distance
    else:world.map.distance_m=distance
    if getattr(world,'v7_map_only',False):
        labels=field('topology_labels',lambda:topology.subregions);topology._subregions=labels
        world._labels=field('world_labels',lambda:world.labels)
        # Exact frozen adapter identity: subregions=(~hard).astype(int16).
        # Keep only an alias to the existing immutable hard field. General
        # point/window reads still go through the blocked plane.
        if type(topology) is FreeSpaceTopology and not world.features and not world.rules:
            from .v9_maps import immutable
            if immutable(world.hard):labels.derived_neutral_hard=world.hard
    else:
        labels=field('topology_labels',lambda:topology.subregions);topology.subregions=labels
        world.labels=field('world_labels',lambda:world.labels)
    pool.clear();world.v12_p2_pool=pool;world.v12_p2_planes=planes
    # The allocator's release policy is the existing parent mechanism. No
    # query work/route/certificate has been computed in this map preparation.
    import gc,ctypes
    gc.collect();ctypes.CDLL(None).malloc_trim(0)
    return {'fields':details,'cache_budget_bytes':pool.budget,'cache_after_binding':pool.bytes,
            'dense_distance_and_labels_released':True,'full_map_conversion_fallback':False,
            'remaining_dense_maps':['hard/safe/occupancy','L2 boolean masks','composed/native task grids']}


@contextmanager
def blocked_request(service,query):
    """Request-owned integrity and release, independent of benchmark/export."""
    pool=service.world.v12_p2_pool
    pool.stats={key:0 for key in pool.stats};pool.stats['peak_bytes']=pool.bytes
    try:
        pool.verify_files()
        yield
    finally:
        detail=pool.snapshot();detail['query_id']=query.query_id
        try:pool.verify_files()
        finally:
            pool.clear();detail['bytes_after_release']=pool.bytes
            from .request_heap import release_retired_heap
            detail['heap_release']=release_retired_heap()
            service.v12_last_field_request=detail
