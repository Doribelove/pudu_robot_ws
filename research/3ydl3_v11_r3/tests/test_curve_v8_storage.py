from copy import deepcopy
import json
from types import SimpleNamespace
from pathlib import Path
import pytest
from three_yd_v5.curve_records import FrozenCurve,freeze_model,record_bytes
from three_yd_v5.r6_cache import resident_bytes
from three_yd_v5.fullroute import record_from,RouteCache


def test_shared_nested_geometry_has_conservative_budget():
    shared=[0.,1.,2.]
    source={'parameters':shared,'corridor':{'a':[shared]*50},'knots':shared}
    f=freeze_model(source)
    assert f._resident_bytes>=resident_bytes(f)
    suffix=f.suffix(1)
    assert suffix['corridor'] is f['corridor']
    assert suffix['knots'] is f['knots']
    assert suffix['parameters']==(1.,2.)
    assert suffix._resident_bytes>=resident_bytes(suffix)
    source['parameters'][0]=900.
    assert f['parameters'][0]==0.
    assert json.loads(json.dumps(suffix))['parameters']==[1.,2.]
    with pytest.raises(TypeError):suffix.__init__({})


def test_two_record_boundaries_freeze_model_once(monkeypatch):
    from arena_evaluation.unified_four_backends_smoke import PlanResult
    monkeypatch.setenv('THREEYD_V9_R9_MODE','PLMX')
    raw={'parameters':[0.,1.],'coefficients':[[1.,2.],[2.,3.]],'degree':3}
    result=PlanResult(points=[{'x':1.,'y':2.,'yaw':0.}],planner_success=True)
    out={'success':True,'result':result,'curve_model':raw}
    a=record_from(out);b=record_from(out)
    assert type(a['curve_model']) is FrozenCurve
    assert a['curve_model'] is b['curve_model'] is out['curve_model']
    raw['coefficients'][0][0]=99.
    assert a['curve_model']['coefficients'][0][0]==1.
    cache=RouteCache(max_entries=2,max_bytes=record_bytes(a)-1)
    assert not cache.put('a',a)
    cache=RouteCache(max_entries=2,max_bytes=record_bytes(a)*2)
    assert cache.put('a',a)
    assert cache.items['a'][0]['curve_model'] is a['curve_model']


def test_native_stack_reports_unique_live_pid(tmp_path):
    from three_yd_v5.curve_session import CurveStack
    import os
    log=tmp_path/'stack.log';log.write_text('CURVE_NAV2_NODES_CREATED\nManaged nodes are active\n')
    stack=CurveStack(native=True,map_yaml=Path('map'),params_file=Path('params'),log_file=log)
    stack.process=SimpleNamespace(pid=os.getpid(),poll=lambda:None)
    assert stack.pids()==(os.getpid(),[os.getpid()],'')
    stack.process=SimpleNamespace(pid=os.getpid(),poll=lambda:1)
    assert stack.pids()[0]==0


def test_component_stack_requires_all_nodes_loaded(tmp_path):
    from three_yd_v5.curve_session import CurveStack
    import os
    log=tmp_path/'stack.log'
    text=f'[component_container_isolated-1]: process started with pid [{os.getpid()}]\n'
    stack=CurveStack(composed=True,map_yaml=Path('map'),params_file=Path('params'),log_file=log)
    log.write_text(text);assert stack.pids()[0]==0
    for name in ('map_server','planner_server','lifecycle_manager_planner'):
        text+=f"Loaded node '/{name}' in container '/curve_nav2_container'\n"
    log.write_text(text);assert stack.pids()==(os.getpid(),[os.getpid()],'')
