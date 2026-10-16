"""Diagnostic reach samples only; finite search is not infeasibility proof."""
import ctypes,os,json,math
from pathlib import Path
import numpy as np
from replay_curve import load
from three_yd_v5.v10_storage import collision_checker
from three_yd_v5.curve_l3 import feasible_guide,interp
ROOT=Path(__file__).resolve().parents[1]
os.sched_setaffinity(0,{14})
out=ROOT/'results/reach_probe';out.mkdir(exist_ok=False)
lib=ctypes.CDLL(str(ROOT/'experiments/libendpoint_reach.so'))
fn=lib.endpoint_seed
fn.argtypes=[ctypes.c_size_t,ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_void_p,ctypes.c_void_p]+[ctypes.c_double]*7+[ctypes.c_int,ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p]
fn.restype=ctypes.c_int
for mid,q in [('mentor_map_20260825_005','A2B-07'),('arena_medium_200x200_2d_005','A2B-04')]:
    w,d,m=load(ROOT/'results/diagnostic_initial'/mid/'curve_inputs'/(q+'_000.npz'))
    p,s,_=feasible_guide(d['guide'],d['start'],d['goal'],w,d['physical'])
    at=s[-1]-3;xy=interp(p,s,[at])[0];v=np.diff(interp(p,s,[at-.05,at+.05]),axis=0)[0]
    a=d['goal'].copy();a[2]+=math.pi;b=np.r_[xy,np.arctan2(v[1],v[0])+math.pi]
    checker=collision_checker(w.hard,w);states=np.empty((200000,3),float);stats=np.zeros(3,np.int32)
    n=fn(ctypes.cast(checker.fn,ctypes.c_void_p).value,checker.hard.ctypes.data,*checker.hard.shape,a.ctypes.data,b.ctypes.data,checker.res,*checker.origin[:2],checker.half_length,checker.half_width,.400001,8.,2000000,states.ctypes.data,len(states),stats.ctypes.data)
    np.save(out/f'{q}.npy',states[:abs(n)]);rec={'case':mid+'/'+q,'return':n,'stats':stats.tolist(),'sample_xy_bounds':[states[:abs(n),:2].min(0).tolist(),states[:abs(n),:2].max(0).tolist()],'direction':'computational backward, robot still forward','physical_infeasibility_proven':False,'navigation_success':False}
    (out/f'{q}.json').write_text(json.dumps(rec,indent=2)+'\n');print(rec,flush=True)
