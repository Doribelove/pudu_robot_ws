import hashlib,os
from pathlib import Path
import numpy as np
import pytest
from arena_evaluation.planner_benchmark.map_utils import HospitalMap
from three_yd_v5 import tail_filehash as h


def make_map(tmp_path,monkeypatch):
    monkeypatch.setenv('THREEYD_V10_R4_MODE','H')
    f=tmp_path/'map.pgm';f.write_bytes(b'original map bytes')
    m=HospitalMap(tmp_path/'map.yaml',f,.05,(0,0,0),1,1,np.zeros((1,1),np.int8),np.zeros((1,1)))
    h.prepare(m)
    return m,f


def test_one_fresh_read_per_request(tmp_path,monkeypatch):
    m,f=make_map(tmp_path,monkeypatch);old=h._original;calls=[]
    monkeypatch.setattr(h,'_original',lambda x:(calls.append(x),old(x))[1])
    for _ in range(2):
        with h.request_scope():
            for _ in range(7):assert m.sha256==hashlib.sha256(f.read_bytes()).hexdigest()
    assert len(calls)==2


@pytest.mark.parametrize('operation',['rewrite','replace','restore_mtime','unlink','path_change'])
def test_changed_file_never_returns_cached_digest(tmp_path,monkeypatch,operation):
    m,f=make_map(tmp_path,monkeypatch)
    with pytest.raises((RuntimeError,FileNotFoundError)):
        with h.request_scope():
            before=m.sha256;stat=f.stat()
            if operation=='unlink':f.unlink()
            elif operation=='replace':
                new=tmp_path/'replacement';new.write_bytes(b'changed map bytes!');new.replace(f)
            elif operation=='path_change':
                # Original path is still monitored, even if the map switches files.
                m.image_path=tmp_path/'new';m.image_path.write_bytes(b'new');f.write_bytes(b'changed')
            else:
                f.write_bytes(b'changed map bytes!')
                if operation=='restore_mtime':os.utime(f,ns=(stat.st_atime_ns,stat.st_mtime_ns))
            if operation!='path_change':_ = m.sha256
    assert h._active.get() is None


def test_final_guard_and_next_request_freshness(tmp_path,monkeypatch):
    m,f=make_map(tmp_path,monkeypatch)
    with pytest.raises(RuntimeError):
        with h.request_scope():
            _=m.sha256;f.write_bytes(b'changed at end')
    with pytest.raises(RuntimeError,match='RELOAD_REQUIRED'):
        with h.request_scope():_ = m.sha256


def test_unknown_subclass_and_disabled_mode_keep_original(tmp_path,monkeypatch):
    class CustomMap(HospitalMap):
        @property
        def sha256(self):return 'custom'
    m,f=make_map(tmp_path,monkeypatch);m.__class__=CustomMap;h.prepare(m)
    assert type(m) is CustomMap and m.sha256=='custom'
    m.__class__=HospitalMap;monkeypatch.setenv('THREEYD_V10_R4_MODE','control');h.prepare(m)
    assert type(m) is HospitalMap
