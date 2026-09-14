from collections import OrderedDict
from types import SimpleNamespace
import random
import sys
import pytest
import numpy as np
from arena_evaluation.reusable_tile_preparation_r3 import retained_size
from arena_evaluation.reachable_endpoint_r3 import _native_geometry


def test_native_accountant_matches_reference_for_cycles_sharing_and_arrays():
    assert hasattr(_native_geometry,'retained_size'),'DEPLOYMENT_NATIVE_ACCOUNTANT_REQUIRED'
    rng=random.Random(412)
    cases=[None,True,0,2**100,'x'*81,bytearray(237),np.zeros((43,57),dtype=np.float32)]
    shared=[1.,2.,3.];cycle={'shared':shared};cycle['self']=cycle
    cases.extend([cycle,OrderedDict([('a',shared),('b',shared)]),SimpleNamespace(value=cycle)])
    for _ in range(100):
        cases.append({'nodes':[(i,rng.random(),rng.random()) for i in range(rng.randrange(30,100))],
                      'shared':shared,'record':SimpleNamespace(cache=cycle)})
    for value in cases:
        for excluded in [set(),{id(shared)}, {id(value)}]:
            assert _native_geometry.retained_size(value,excluded)==retained_size(value,seen=set(excluded))


def test_native_accountant_does_not_steal_references_or_mutate_exclusions():
    value=SimpleNamespace(data={'a':[1,2,3]});excluded=set();before=sys.getrefcount(value)
    for _ in range(50):_native_geometry.retained_size(value,excluded)
    assert sys.getrefcount(value)==before and excluded==set()


def test_native_size_exception_is_propagated_without_reference_leak():
    class Broken:
        def __sizeof__(self):raise ValueError('size failed')
    value=Broken();before=sys.getrefcount(value)
    for _ in range(5):
        with pytest.raises(ValueError,match='size failed'):_native_geometry.retained_size({'v':value},set())
    assert sys.getrefcount(value)==before
