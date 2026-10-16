from types import SimpleNamespace as S
import pytest
from three_yd_v5 import request_heap


@pytest.mark.parametrize('free,trigger',[(0,False),(64*2**20-1,False),(64*2**20,True),(400*2**20,True)])
def test_only_large_unused_heap_triggers_release_and_keeps_reserve(monkeypatch,free,trigger):
    calls=[]
    def info():return S(fordblks=free,arena=800*2**20)
    def trim(pad):calls.append(pad);return 1
    monkeypatch.setattr(request_heap,'backend',lambda:(info,trim))
    result=request_heap.release_retired_heap()
    assert result['triggered']==trigger and result['included_in_navigation']
    assert calls==([32*2**20] if trigger else [])


def test_unknown_allocator_is_untouched(monkeypatch):
    monkeypatch.setattr(request_heap,'backend',lambda:None)
    result=request_heap.release_retired_heap()
    assert not result['supported'] and not result['triggered'] and not result['released']
