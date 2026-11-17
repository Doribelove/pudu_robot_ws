"""Request-local file digests with fresh content reads and mutation guards.

No digest crosses a request boundary. Every lookup checks inode/size/mtime/
ctime, and the request checks them again before exposing a result. Unknown
map implementations retain their own getter. Native observations are untouched.
"""
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path
from arena_evaluation.planner_benchmark.map_utils import HospitalMap,sha256_file
from .tail_options import enabled

_active=ContextVar('r4_request_file_hashes',default=None)
_original=HospitalMap.sha256.fget


def fingerprint(path):
    s=Path(path).stat()
    return (s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns)


class RequestHashMap(HospitalMap):
    @property
    def sha256(self):
        entries=_active.get()
        if entries is None or not enabled('H'):return _original(self)
        path=Path(self.image_path).absolute();now=fingerprint(path)
        old=entries.get(path)
        if old is not None:
            if old[0]!=now:raise RuntimeError('CURVE_MAP_FILE_CHANGED_DURING_REQUEST')
            return old[1]
        value=_original(self)
        if fingerprint(path)!=now:raise RuntimeError('CURVE_MAP_FILE_CHANGED_DURING_HASH')
        loaded=getattr(self,'_r4_loaded_image',None)
        if loaded is not None and (path,value)!=loaded:
            raise RuntimeError('CURVE_REGISTERED_MAP_FILE_CHANGED_RELOAD_REQUIRED')
        entries[path]=(now,value)
        return value


def prepare(map_object):
    if enabled('H') and type(map_object) is HospitalMap:
        path=Path(map_object.image_path).absolute();before=fingerprint(path)
        value=_original(map_object)
        if fingerprint(path)!=before:raise RuntimeError('CURVE_MAP_FILE_CHANGED_DURING_PREPARATION')
        map_object._r4_loaded_image=(path,value)
        map_object.__class__=RequestHashMap


@contextmanager
def request_scope():
    if not enabled('H') or _active.get() is not None:
        yield
        return
    entries={};token=_active.set(entries)
    try:
        yield
        for path,(before,digest) in entries.items():
            # Metadata alone is insufficient (coarse or lazy timestamps).
            # Fresh bytes at commit validate the request's consistent snapshot.
            if fingerprint(path)!=before or sha256_file(path)!=digest or fingerprint(path)!=before:
                raise RuntimeError('CURVE_MAP_FILE_CHANGED_DURING_REQUEST')
    finally:_active.reset(token)
