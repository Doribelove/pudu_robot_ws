"""Versioned reconstruction of full master snapshots from loss-safe deltas."""
import hashlib,struct,uuid
import numpy as np
HEADER=struct.Struct('<8sQQQIIII32s')
RECT=struct.Struct('<IIII')

class DeltaDecoder:
    def __init__(self,shape,version=1):
        if version not in (1,2):raise ValueError("delta version")
        self.version=version;self.protocol=f'3yd_delta_v{version}|'
        self.shape=tuple(shape);self.token=uuid.uuid4().hex;self.epoch=0;self.sequence=0;self.grid=None
        self._verified_digest=None
    def request_layer(self):return f'{self.protocol}{self.token}|{self.epoch}|{self.sequence}'
    def invalidate(self):self.epoch=0;self.sequence=0;self.grid=None;self._verified_digest=None
    def take_current_digest(self,server):
        """Consume only this decoder's freshly SHA-verified observation once."""
        proof=self._verified_digest;self._verified_digest=None
        if (proof is None or proof[0] is not server or self.grid is not server
                or proof[1:3]!=(self.epoch,self.sequence)):
            return None
        return proof[3]
    def decode(self,data,layer):
        self._verified_digest=None
        try:return self._decode(data,layer)
        except Exception:
            self.invalidate();raise
    def _decode(self,data,layer):
        if layer!=self.protocol+self.token:raise RuntimeError('delta protocol/owner mismatch')
        packet=memoryview(data).cast('B')
        if len(packet)<HEADER.size:raise RuntimeError('truncated delta header')
        magic,epoch,seq,base,w,h,count,full,digest=HEADER.unpack_from(packet)
        if magic!=(b'3YDACK06' if self.version==1 else b'3YDACK07') or (h,w)!=self.shape or full not in ((0,1) if self.version==1 else (0,1,2)) or not epoch or not seq:raise RuntimeError('invalid delta header')
        if epoch==self.epoch and seq<=self.sequence:raise RuntimeError('stale delta snapshot')
        offset=HEADER.size
        if full==1:
            if base or count or len(packet)-offset!=w*h:raise RuntimeError('invalid full snapshot')
            candidate=np.frombuffer(packet,dtype=np.uint8,count=w*h,offset=offset).reshape(h,w)
            if hashlib.sha256(candidate).digest()!=digest:raise RuntimeError('full snapshot checksum mismatch')
            self.grid=candidate.copy()
        else:
            if full==2:
                if base:raise RuntimeError('compressed full snapshot has delta base')
                self.grid=np.full((h,w),254,np.uint8)
            elif self.grid is None or epoch!=self.epoch or base!=self.sequence or seq!=base+1:raise RuntimeError('delta base not acknowledged')
            if count>((w+127)//128)*((h+127)//128):raise RuntimeError('delta patch limit')
            patches=[];seen=set()
            for _ in range(count):
                if offset+RECT.size>len(packet):raise RuntimeError('truncated rectangle')
                x,y,dx,dy=RECT.unpack_from(packet,offset);offset+=RECT.size
                if x>=w or y>=h or x%128 or y%128 or dx!=min(128,w-x) or dy!=min(128,h-y) or (x,y) in seen:raise RuntimeError('invalid/duplicate rectangle')
                seen.add((x,y));n=dx*dy
                if offset+n>len(packet):raise RuntimeError('truncated rectangle content')
                patches.append((x,y,dx,dy,offset));offset+=n
            if offset!=len(packet):raise RuntimeError('unexpected trailing data')
            for x,y,dx,dy,pos in patches:
                self.grid[y:y+dy,x:x+dx]=np.frombuffer(packet,np.uint8,dx*dy,pos).reshape(dy,dx)
            if hashlib.sha256(self.grid).digest()!=digest:raise RuntimeError('reconstructed full snapshot checksum mismatch')
        self.epoch=epoch;self.sequence=seq
        # All successful branches above computed SHA256 over the COMPLETE
        # current reconstructed bytes and checked it against the packet.
        self._verified_digest=(self.grid,epoch,seq,digest.hex())
        return self.grid,{'r6_readback_full':bool(full),'r4_full_closed_tiles':full==2,'r4_snapshot_protocol':self.version,'r6_readback_patch_count':count,'r6_readback_wire_bytes':len(packet),'r6_readback_logical_cells':w*h,'r6_server_epoch':epoch,'r6_snapshot_sequence':seq,'r6_complete_snapshot_verified':True}
