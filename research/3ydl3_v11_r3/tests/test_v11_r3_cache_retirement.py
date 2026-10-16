from types import SimpleNamespace as S
import pytest
from three_yd_v5.controller import V5Controller
from three_yd_v5.r6_cache import get_cache


@pytest.mark.parametrize('mode,retained',[('ABRTHI',False),('control',True)])
def test_l2_route_miss_retires_only_owned_cached_composition(monkeypatch,mode,retained):
    monkeypatch.setenv('THREEYD_V10_R3_MODE',mode)
    views={};native_current=object();composition=object()
    get_cache(views,'bounded-costmap',max_entries=1)['old']=composition
    get_cache(views,'route-fields',max_entries=1)['old']={'old':True}
    c=S(views=views,_route_cache_key=lambda:'new',native_current=native_current)
    assert V5Controller._restore_static_route(c) is False
    assert (get_cache(views,'bounded-costmap',max_entries=1).get('old') is composition)==retained
    assert c.native_current is native_current
