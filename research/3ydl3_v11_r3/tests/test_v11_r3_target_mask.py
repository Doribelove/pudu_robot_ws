from types import SimpleNamespace as S
import numpy as np
import pytest
from three_yd_v4.r5_controller import SoftEdgeController
from three_yd_v5.target_mask import build
original=SoftEdgeController._target_mask

@pytest.mark.parametrize('seed',range(8))
@pytest.mark.parametrize('mode',['none','broadcast_zero','broadcast_one','mutable_zero','dense','noncanonical','reverse','remainder'])
def test_all_target_bytes_and_inputs_match_original(seed,mode):
 rng=np.random.default_rng(seed);shape=(37,53)
 a=rng.random(shape)>.4;b=rng.random(shape)>.3;d=rng.random(shape)>.6;r=None
 if mode=='none':d=None
 elif mode=='broadcast_zero':d=np.broadcast_to(np.frombuffer(bytes(1),bool),shape)
 elif mode=='broadcast_one':d=np.broadcast_to(np.frombuffer(bytes([1]),bool),shape)
 elif mode=='mutable_zero':d=np.broadcast_to(np.zeros(1,bool),shape)
 elif mode=='noncanonical':
  a=rng.integers(0,256,shape,dtype=np.uint8).view(bool)
  b=rng.integers(0,256,shape,dtype=np.uint8).view(bool)
  d=rng.integers(0,256,shape,dtype=np.uint8).view(bool)
 elif mode=='reverse':a=a[::-1,::-1];b=b[::-1,::-1];d=d[::-1,::-1]
 elif mode=='remainder':r=rng.random(shape)>.5
 c=S(plan=S(corridor_mask=a),_search_mask=b,dynamic=None if d is None else S(mask=d),r13_remainder_mask=r)
 before=[x.tobytes() for x in [a,b,d,r] if x is not None]
 expected=original(c)&b
 if r is not None:expected=expected&r
 actual=build(c,original)
 assert actual.dtype==expected.dtype and actual.shape==expected.shape
 assert actual.tobytes()==expected.tobytes()
 assert before==[x.tobytes() for x in [a,b,d,r] if x is not None]
 actual[:]=False
 assert before==[x.tobytes() for x in [a,b,d,r] if x is not None]

def test_changed_parent_rule_uses_original_callable():
 calls=[]
 def custom(c):calls.append(1);return np.zeros((3,4),bool)
 c=S(plan=S(corridor_mask=np.ones((3,4),bool)),_search_mask=np.ones((3,4),bool),dynamic=None,r13_remainder_mask=None)
 assert not build(c,custom).any() and calls==[1]

def test_unknown_uint8_schema_keeps_parent_bitwise_behavior():
 a=np.arange(12,dtype=np.uint8).reshape(3,4);b=np.full((3,4),7,np.uint8);d=np.full((3,4),3,np.uint8)
 c=S(plan=S(corridor_mask=a),_search_mask=b,dynamic=S(mask=d),r13_remainder_mask=None)
 assert np.array_equal(build(c,original),original(c)&b)
