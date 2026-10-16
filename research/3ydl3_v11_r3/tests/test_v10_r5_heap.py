"""Optional heap preparation cannot alter live planning data."""
import numpy as np
from three_yd_v5 import recovery_heap


def test_live_buffers_survive_native_heap_release():
    a=np.arange(100000,dtype=np.int32);b=a[::3]
    expected=a.tobytes()
    result=recovery_heap.release_preparation_heap()
    assert a.tobytes()==expected and np.array_equal(b,a[::3])
    assert result['wall_ms']>=0


def test_missing_native_trim_is_noop(monkeypatch):
    monkeypatch.setattr(recovery_heap.ctypes,'CDLL',lambda *_:object())
    assert recovery_heap.release_preparation_heap()=={
        'supported':False,'released':False,'wall_ms':0.}
