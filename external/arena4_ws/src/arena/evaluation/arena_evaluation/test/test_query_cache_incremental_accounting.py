from collections import OrderedDict
from types import SimpleNamespace
import random
import pytest
from arena_evaluation import bounded_query_preparation_r3 as module
from arena_evaluation.reusable_tile_preparation_r3 import retained_size


def fixture(limit=200000):
    hospital=SimpleNamespace(shared=bytearray(400000))
    topology=SimpleNamespace(hospital_map=hospital,graph={'fixed':bytearray(8000)})
    selector=SimpleNamespace(topology=topology,index={'nodes':bytearray(12000)},cache=OrderedDict(),
        last_certificate=None,connector=SimpleNamespace(map=hospital,_distance_window=None,
          _local_free_window=None,_obstacle_field_cache=None))
    p=module.BoundedQueryTopologyPreparer(SimpleNamespace(),(),max_bytes=limit)
    value=(topology,selector);p.entries['q']=value;p.last_preparation={'key':'q'}
    return p,value


def test_connector_growth_does_not_traverse_immutable_graph_twice(monkeypatch):
    p,value=fixture();calls=[];original=module.retained_size
    def measured(v,**kwargs):
        if v is value:calls.append(1)
        return original(v,**kwargs)
    monkeypatch.setattr(module,'retained_size',measured)
    p.reconcile();value[1].cache['connector']='x'*5000;p.reconcile()
    assert len(calls)==1
    assert p.bytes>=retained_size(value,seen={id(value[0].hospital_map)})


def test_incremental_bound_covers_full_reference_after_mutable_replacements():
    import numpy as np
    p,value=fixture();rng=random.Random(73)
    for i in range(30):
        selector=value[1]
        selector.cache['q']='x'*rng.randrange(1,10000)
        selector.last_certificate={'path':[[float(k),0.,0.] for k in range(rng.randrange(1,200))]}
        selector.connector._distance_window=(0,0,np.zeros((rng.randrange(1,70),70),dtype=np.float32))
        p.reconcile()
        assert p.entries
        assert retained_size(value,seen={id(value[0].hospital_map)})<=p.bytes<=p.max_bytes
    assert p.immutable_size_scans==1


def test_eviction_removes_all_accounting_and_does_not_retain_graph_ids():
    p,value=fixture(limit=40000);p.reconcile();assert p.entries
    value[1].cache['q']='x'*80000;p.reconcile()
    assert not p.entries and p.bytes==0
    assert not p.static_sizes and not p.mutable_sizes and not p.stamps and not p.sizes


@pytest.mark.parametrize('limit',[0,-1,True,module.MAX_QUERY_CACHE_BYTES+1])
def test_capacity_rejects_invalid_or_over_hardware_profile_values(limit):
    with pytest.raises(ValueError,match='QUERY_CACHE_BYTE_LIMIT_INVALID'):
        module.BoundedQueryTopologyPreparer(SimpleNamespace(),(),max_bytes=limit)
