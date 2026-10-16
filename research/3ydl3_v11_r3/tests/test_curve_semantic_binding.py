from types import SimpleNamespace as S
import copy
import pytest
from arena_evaluation.semantic_map import SemanticMapV1,SemanticFeature,ConversionDiagnostic
from three_yd_v5 import curve_semantic_binding as binding,curve_backend as backend

def semantic():
    return SemanticMapV1('地图',.05,(1.,2.,.1),20,30,'source',
        features=[SemanticFeature('路一','lane','line',[[1,2],[3,4]],
            properties={'中文':{'a':[1,False,None,{'x':-0.}],'b':(2,3)}})],
        traffic_rules={'direction':{'enabled':True}},
        diagnostics=[ConversionDiagnostic('测试','info','确认当前内容')])

def test_current_content_hash_matches_parent_and_changes_immediately():
    s=semantic();previous=None
    for change in (lambda:None,lambda:s.features[0].coordinates[0].__setitem__(0,2.3),
                   lambda:s.features[0].properties['中文']['a'].append('变化'),
                   lambda:s.traffic_rules['direction'].__setitem__('enabled',False),
                   lambda:setattr(s,'frame_id','地图二')):
        change();value=binding.phase_hash(s)
        assert value==s.semantic_map_hash
        if previous is not None:assert value!=previous
        previous=value
    assert binding.stats()['semantic_content_cached'] is False

def test_alias_and_custom_container_use_parent_behavior():
    s=semantic();shared=[1,2];s.features[0].properties={'a':shared,'b':shared}
    assert binding.phase_hash(s) is None
    assert binding.fresh_hash(s)==s.semantic_map_hash
    class CustomList(list):pass
    s.features[0].properties={'a':CustomList([2])}
    assert binding.phase_hash(s) is None
    assert binding.fresh_hash(s)==s.semantic_map_hash
    s.features[0].properties={'self':s.features[0]}
    assert binding.phase_hash(s) is None

def test_changed_parent_hash_and_subclass_are_not_overridden(monkeypatch):
    class SubMap(SemanticMapV1):pass
    s=semantic();sub=SubMap(**copy.deepcopy(s.__dict__))
    assert binding.phase_hash(sub) is None
    monkeypatch.setattr(SemanticMapV1,'semantic_map_hash',property(lambda self:'changed-parent'))
    assert binding.phase_hash(s) is None and binding.fresh_hash(s)=='changed-parent'

def test_nan_still_rejected():
    s=semantic();s.features[0].properties={'nan':float('nan')}
    with pytest.raises(ValueError):binding.phase_hash(s)
    with pytest.raises(ValueError):_ = s.semantic_map_hash

def test_each_request_phase_hashes_current_content_once(monkeypatch):
    s=semantic();service=S(world=S(semantic=s),controller=None,session=S(),record=None)
    monkeypatch.setattr(backend,'settings',lambda:('independent',{'total_request_budget_s':30},'config'))
    monkeypatch.setattr(backend,'identity',lambda:('test','independent','config'))
    captures=[]
    for name in ('service_binding','retained_binding'):
        monkeypatch.setattr(backend,name,lambda service,semantic_hash=None:captures.append(semantic_hash) or semantic_hash)
    before=binding.stats()['fresh_fast_hashes'];backend.request_start(service)
    start=s.semantic_map_hash;s.features[0].coordinates[0][0]=4.2
    finish=s.semantic_map_hash;backend.request_finish(service,{'success':True})
    assert captures==[start,start,finish,finish]
    assert binding.stats()['fresh_fast_hashes']-before==2
