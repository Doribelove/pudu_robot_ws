#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
import ctypes as C
import hashlib
import struct
import sys
import unittest
from trilayer_delta_codec import DeltaCodec
LIB = sys.argv.pop(1) if len(sys.argv)>1 and sys.argv[1].endswith('.so') else None


def unpack(packet, previous=None):
    magic=packet[:8]; epoch,seq,base,w,h,count,mode=struct.unpack_from('<QQQIIII',packet,8)
    assert magic in (b'3YDACK06',b'3YDACK07')
    digest=packet[48:80]; offset=80
    if mode==1:
        data=bytearray(packet[offset:]); assert count==base==0; assert len(data)==w*h
    else:
        data=bytearray([254])*(w*h) if mode==2 else bytearray(previous)
        if mode==2: assert base==0
        for _ in range(count):
            x,y,dx,dy=struct.unpack_from('<IIII',packet,offset); offset+=16
            assert x+dx<=w and y+dy<=h
            for row in range(dy):
                data[(y+row)*w+x:(y+row)*w+x+dx]=packet[offset:offset+dx];offset+=dx
        assert offset==len(packet)
    assert hashlib.sha256(data).digest()==digest
    return bytes(data),(epoch,seq,base,mode,count,magic)


class CodecTests(unittest.TestCase):
    def state(self): return DeltaCodec(123, LIB)
    def test_dense_delta_nochange_reset(self):
        for sparse in (False,True):
            with self.state() as c:
                data=bytes([1])*(256*256)
                p=c.encode(data,256,256,1,'a'*32,0,0,sparse); out,m=unpack(p)
                self.assertEqual(out,data); self.assertEqual(m[3],1)
                self.assertEqual(m[5],b'3YDACK07' if sparse else b'3YDACK06')
                changed=bytearray(data);changed[0]=2;changed=bytes(changed)
                out,m=unpack(c.encode(changed,256,256,1,'a'*32,123,1,sparse),data)
                self.assertEqual(out,changed);self.assertEqual(m[3:5],(0,1))
                out,m=unpack(c.encode(changed,256,256,1,'a'*32,123,2,sparse),changed)
                self.assertEqual(m[3:5],(0,0));self.assertEqual(m[2],2)
                # Out-of-order base forces a full snapshot, not stale delta.
                out,m=unpack(c.encode(changed,256,256,1,'a'*32,123,1,sparse));self.assertEqual(m[3],1)
                out,m=unpack(c.encode(changed,256,256,1,'b'*32,123,4,sparse));self.assertEqual(m[3],1)
                out,m=unpack(c.encode(changed,256,256,2,'b'*32,123,5,sparse));self.assertEqual(m[3],1)
                out,m=unpack(c.encode(changed,256,256,2,'b'*32,999,6,sparse));self.assertEqual(m[3],1)
    def test_closed_full_and_fallback(self):
        with self.state() as c:
            data=bytes([254])*(256*256)
            out,m=unpack(c.encode(data,256,256,1,'a'*32,0,0,True))
            self.assertEqual(out,data);self.assertEqual(m[3:5],(2,0))
            new=bytes([7])*len(data)
            out,m=unpack(c.encode(new,256,256,1,'a'*32,123,1,True),data)
            self.assertEqual(out,new);self.assertEqual(m[3],1)
    def test_dimensions_and_native_boundary(self):
        with self.state() as c:
            data=bytes([0])*4
            _,m=unpack(c.encode(data,2,2,1,'a'*32,0,0))
            _,m=unpack(c.encode(data,1,4,1,'a'*32,123,1));self.assertEqual(m[3],1)
            output=C.c_void_p();length=C.c_size_t()
            def raw(handle, payload, size, token):
                return c._lib.tl_delta_encode(handle,payload,size,1,1,1,token,32,123,2,0,
                                               C.byref(output),C.byref(length))
            self.assertNotEqual(raw(c._handle,b'\0',1,b'x'*32),0)
            self.assertFalse(output.value);self.assertEqual(length.value,0)
            self.assertNotEqual(raw(999999,b'\0',1,b'a'*32),0)
            self.assertNotEqual(raw(c._handle,b'\0',0,b'a'*32),0)
            self.assertNotEqual(raw(c._handle,None,1,b'a'*32),0)
            self.assertEqual(raw(c._handle,b'\0',1,b'a'*32),0)
            memory=output.value
            self.assertEqual(c._lib.tl_delta_free(memory),0)
            self.assertNotEqual(c._lib.tl_delta_free(memory),0)
            _,m=unpack(c.encode(b'\0',1,1,1,'a'*32,0,0));self.assertEqual(m[1],4)

    def test_invalid_and_close(self):
        with self.assertRaises(ValueError): DeltaCodec(0, LIB)
        c=self.state()
        handle=C.c_uint64()
        self.assertNotEqual(c._lib.tl_delta_create(0,C.byref(handle)),0)
        for token in ('x'*32,'a'*31,'A'*32,'a'*31+'\0'):
            with self.assertRaises(ValueError):c.encode(b'\0',1,1,1,token,0,0)
        for args in ((b'',1,1),(b'\0',0,1),(b'\0',2**32,1)):
            with self.assertRaises(ValueError):c.encode(*args,1,'a'*32,0,0)
        # Failed calls must not consume sequence numbers.
        _,m=unpack(c.encode(b'\0',1,1,1,'a'*32,0,0));self.assertEqual(m[1],1)
        c.close();c.close()
        with self.assertRaises(RuntimeError):c.encode(b'\0',1,1,1,'a'*32,0,0)

if __name__=='__main__':unittest.main(verbosity=2)
