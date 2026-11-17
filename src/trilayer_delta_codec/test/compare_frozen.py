#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Byte comparison with frozen historical test .so. Valid inputs only!"""
import argparse
import ctypes as C
import hashlib
from pathlib import Path
import struct
from trilayer_delta_codec import DeltaCodec


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('baseline',type=Path,help='frozen native directory, not copied into package')
    parser.add_argument('--libpath')
    args=parser.parse_args()
    epoch=123456789; token='abcdef0123456789'*2
    for sparse,name in ((False,'libdelta_codec_test.so'),(True,'libdelta_codec_v2_test.so')):
        path=args.baseline/name
        print(name,'sha256',hashlib.sha256(path.read_bytes()).hexdigest())
        lib=C.CDLL(str(path))
        lib.codec_create.argtypes=[C.c_uint64];lib.codec_create.restype=C.c_void_p
        lib.codec_destroy.argtypes=[C.c_void_p];lib.codec_destroy.restype=None
        lib.codec_encode.argtypes=[C.c_void_p,C.c_void_p,C.c_uint32,C.c_uint32,C.c_uint64,
                                   C.c_char_p,C.c_uint64,C.c_uint64,C.POINTER(C.c_uint64)]
        lib.codec_encode.restype=C.c_void_p
        old=lib.codec_create(epoch); new=DeltaCodec(epoch,args.libpath)
        seq=0
        def compare(label,data,w,h,identity=7,owner=token,want=epoch,base=None):
            nonlocal seq
            if base is None:base=seq
            size=C.c_uint64()
            ptr=lib.codec_encode(old,data,w,h,identity,owner.encode(),want,base,C.byref(size))
            expected=C.string_at(ptr,size.value)
            actual=new.encode(data,w,h,identity,owner,want,base,sparse)
            assert actual==expected,(name,label,len(actual),len(expected))
            seq=struct.unpack_from('<Q',actual,16)[0]
            mode=struct.unpack_from('<I',actual,44)[0]
            print('PASS',name,label,'bytes',len(actual),'mode',mode)
        try:
            w=h=256; closed=bytes([254])*(w*h); uniform=bytes([7])*(w*h)
            compare('initial-closed',closed,w,h,want=0)
            compare('closed-nochange',closed,w,h)
            change=bytearray(closed);change[0]=0;change=bytes(change)
            compare('one-tile-delta',change,w,h)
            compare('delta-nochange',change,w,h)
            compare('all-tiles-delta-fallback',uniform,w,h)
            compare('wrong-base',uniform,w,h,base=0)
            compare('wrong-epoch',uniform,w,h,want=epoch+1)
            compare('owner-switch',uniform,w,h,owner='b'*32)
            compare('identity-switch',uniform,w,h,identity=8,owner='b'*32)
            # 128x128 nonclosed tile + 1x128 closed tile: v2 sparse payload
            # (16384+16) < 16512, testing closed-full selection near threshold.
            near=bytes([7]*128+[254])*128
            compare('closed-full-near-threshold',near,129,128)
            # With only one row, payload metadata exceeds saved closed cells.
            compare('dense-fallback-near-threshold',bytes([7]*128+[254]),129,1)
            compare('single-cell',b'\xfe',1,1)
            compare('single-cell-change',b'\x00',1,1)
        finally:
            new.close();lib.codec_destroy(old)
    print('ALL 26 FROZEN BYTE COMPARISONS PASSED')

if __name__=='__main__':main()
