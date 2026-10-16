"""Versioned reconstruction of full master snapshots from loss-safe deltas."""
import hashlib,struct,uuid
import numpy as np
HEADER=struct.Struct('<8sQQQIIII32s')
RECT=struct.Struct('<IIII')

class DeltaDecoder:
    def __init__(self,shape):
        self.shape=tuple(shape);self.token=uuid.uuid4().hex;self.epoch=0;self.sequence=0;self.grid=None
    def request_layer(self):return f'3yd_delta_v1|{self.token}|{self.epoch}|{self.sequence}'
    def invalidate(self):self.epoch=0;self.sequence=0;self.grid=None
    def decode(self,data,layer):
        try:return self._decode(data,layer)
        except Exception:
            self.invalidate();raise
    def _decode(self,data,layer):
        if layer!='3yd_delta_v1|'+self.token:raise RuntimeError('delta protocol/owner mismatch')
        packet=memoryview(data).cast('B')
        if len(packet)<HEADER.size:raise RuntimeError('truncated delta header')
        magic,epoch,seq,base,w,h,count,full,digest=HEADER.unpack_from(packet)
        if magic!=b'3YDACK06' or (h,w)!=self.shape or full not in (0,1) or not epoch or not seq:raise RuntimeError('invalid delta header')
        if epoch==self.epoch and seq<=self.sequence:raise RuntimeError('stale delta snapshot')
        offset=HEADER.size
        if full:
            if base or count or len(packet)-offset!=w*h:raise RuntimeError('invalid full snapshot')
            candidate=np.frombuffer(packet,dtype=np.uint8,count=w*h,offset=offset).reshape(h,w)
            if hashlib.sha256(candidate).digest()!=digest:raise RuntimeError('full snapshot checksum mismatch')
            self.grid=candidate.copy()
        else:
            if self.grid is None or epoch!=self.epoch or base!=self.sequence or seq!=base+1:raise RuntimeError('delta base not acknowledged')
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
        return self.grid,{'r6_readback_full':bool(full),'r6_readback_patch_count':count,'r6_readback_wire_bytes':len(packet),'r6_readback_logical_cells':w*h,'r6_server_epoch':epoch,'r6_snapshot_sequence':seq,'r6_complete_snapshot_verified':True}
