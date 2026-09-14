from pathlib import Path
from types import SimpleNamespace

import pytest

from arena_evaluation.static_planner_engine_r3 import EngineConfig,StaticPlannerEngine
from arena_evaluation.static_planner_service_r3 import StaticPlannerSupervisor
from arena_evaluation.bounded_query_preparation_r3 import BoundedQueryTopologyPreparer


def test_relative_paths_are_normalized_before_native_map_yaml(tmp_path,monkeypatch):
    monkeypatch.chdir(tmp_path);Path('map.yaml').write_text('resolution: 0.05\n')
    config=EngineConfig('map.yaml','map','cache',186)
    assert Path(config.map_yaml)==tmp_path/'map.yaml'
    assert Path(config.cache)==tmp_path/'cache'
    assert StaticPlannerEngine(config,'out').output==tmp_path/'out'
    assert StaticPlannerSupervisor({},'other').output==tmp_path/'other'


@pytest.mark.parametrize('map_id',['../escape','a/b','a\\b','',True,'*','a\n'])
def test_map_identifier_cannot_escape_generated_directory(tmp_path,map_id):
    m=tmp_path/'map.yaml';m.touch()
    with pytest.raises(ValueError,match='MAP_ID_INVALID'):
        EngineConfig(str(m),map_id,str(tmp_path/'cache'),186)


def test_cache_accounts_indexes_and_evicts_without_copying_shared_map():
    m=SimpleNamespace(large=bytearray(1024*1024))
    p=BoundedQueryTopologyPreparer(SimpleNamespace(),(),max_bytes=16000)
    for i in range(4):
        topo=SimpleNamespace(hospital_map=m,graph={'payload':bytearray(4000)})
        selector=SimpleNamespace(topology=topo,index={'payload':bytearray(4000)})
        p.entries[str(i)]=(topo,selector);p.last_preparation={'key':str(i)};p.reconcile()
        assert p.bytes<=p.max_bytes
    assert list(p.entries)==['3'] and p.evictions==3
    assert len(m.large)==1024*1024


def test_post_connector_cache_growth_is_measured_and_evicted():
    m=object();p=BoundedQueryTopologyPreparer(SimpleNamespace(),(),max_bytes=10000)
    topo=SimpleNamespace(hospital_map=m,graph={})
    selector=SimpleNamespace(topology=topo,cache=[])
    p.entries['q']=(topo,selector);p.last_preparation={'key':'q'};p.reconcile()
    assert p.entries
    selector.cache.append(bytearray(20000));p.reconcile()
    assert not p.entries and p.bytes==0


def test_hot_certificate_replay_does_not_rescan_immutable_index():
    from collections import OrderedDict
    m=object();p=BoundedQueryTopologyPreparer(SimpleNamespace(),(),max_bytes=100000)
    topo=SimpleNamespace(hospital_map=m,graph={})
    selector=SimpleNamespace(topology=topo,cache=OrderedDict(),connector=SimpleNamespace())
    p.entries['q']=(topo,selector);p.last_preparation={'key':'q'};p.reconcile()
    scans=p.size_scans
    for _ in range(10):p.reconcile()
    assert p.size_scans==scans
    selector.cache['q']='certified path';p.reconcile()
    assert p.size_scans==scans+1
    p.reconcile();assert p.size_scans==scans+1


def test_cache_accounting_cannot_escape_request_deadline(monkeypatch):
    import time
    from arena_evaluation.two_layer_v1_r3_benchmark import QueryTopologyPreparer
    from arena_evaluation.tiled_topology_r3 import TopologyDeadlineExceeded
    monkeypatch.setattr(QueryTopologyPreparer,'resolve',lambda *args:(None,None))
    p=BoundedQueryTopologyPreparer(SimpleNamespace(),())
    def slow_accounting():time.sleep(.02);return {}
    monkeypatch.setattr(p,'reconcile',slow_accounting)
    timing={}
    with pytest.raises(TopologyDeadlineExceeded,match='query_cache_accounting'):
        p.resolve(None,time.monotonic()+.01,timing)
    assert timing['query_topology_size_accounting_ms']>=10.
