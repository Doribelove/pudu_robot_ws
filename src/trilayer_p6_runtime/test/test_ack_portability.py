#!/usr/bin/env python3
"""Exact ACK native/scalar/original parity; supplied libraries must be fresh builds."""
import ctypes as C
import json,sys
import numpy as np
class View(C.Structure):
 _fields_=[('data',C.c_void_p),('row',C.c_ssize_t),('col',C.c_ssize_t)]
def view(a):return View(a.ctypes.data,*a.strides)
libs=[]
for name in sys.argv[1:]:
 lib=C.CDLL(name)
 lib.classify_ack.argtypes=[C.POINTER(View),C.c_void_p,C.c_ssize_t,C.c_ssize_t,C.c_int,C.c_void_p,C.c_void_p];lib.classify_ack.restype=None
 lib.equal_ack.argtypes=[C.POINTER(View)]*2+[C.c_ssize_t]*2;lib.equal_ack.restype=C.c_int
 lib.vector_ack_available.restype=C.c_int
 libs.append(lib)
assert len(libs)>=2
rng=np.random.default_rng(94817);cases=0
for width in (1,2,31,32,33,63,64,65):
 for stride in ('contiguous','column2','negative','row2'):
  for previous in (0,1):
   for unchanged in (False,True):
    arrays=[rng.integers(0,256,(10,width*2),dtype=np.uint8) for _ in range(6)]
    if stride=='contiguous':arrays=[np.ascontiguousarray(a[:5,:width]) for a in arrays]
    elif stride=='column2':arrays=[a[:5,::2] for a in arrays]
    elif stride=='negative':arrays=[a[:5,::2][::-1,::-1] for a in arrays]
    else:arrays=[a[::2,:width] for a in arrays]
    arrays[0][:]=rng.choice([0,100,255],arrays[0].shape)
    if unchanged:arrays[5][:]=arrays[2]
    a,changed,master,h,s,prior=arrays
    dirty=(prior!=master) if previous else np.ones(a.shape,bool)
    affected=(changed!=0)|dirty|(h!=0)|(s!=0)
    hard=affected&((h!=0)|(a==100));soft=affected&(s!=0)&~hard;ordinary=affected&~hard&~soft
    masks=[affected,hard,soft,dirty,ordinary]
    flags=sum(m.astype(np.uint8)*(1<<i) for i,m in enumerate(masks))
    counts=np.array([m.sum() for m in masks],np.uint64)
    ys,xs=np.where(dirty);bbox=np.array([xs.min(),ys.min(),xs.max()+1,ys.max()+1] if len(xs) else [width,5,0,0],np.int64)
    for lib in libs:
     actual=np.empty(a.shape,np.uint8);cnt=np.empty(5,np.uint64);box=np.empty(4,np.int64)
     lib.classify_ack((View*6)(*(view(x) for x in arrays)),actual.ctypes.data,5,width,previous,cnt.ctypes.data,box.ctypes.data)
     for x,y in ((actual,flags),(cnt,counts),(box,bbox)):np.testing.assert_array_equal(x,y)
     same=np.array(master,copy=True);v,w=view(master),view(same)
     assert lib.equal_ack(C.byref(v),C.byref(w),5,width)==1
     same[2,0]^=1;assert lib.equal_ack(C.byref(v),C.byref(w),5,width)==0
    cases+=1
print(json.dumps({'cases':cases,'libraries':sys.argv[1:],'vector_available':[l.vector_ack_available() for l in libs],'result':'PASS exact flags/counts/bbox/equality against NumPy reference','architecture_validation':'host only; forced scalar is not ARM execution'}))
