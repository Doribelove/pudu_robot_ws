"""Owned immutable curve geometry; no ACK or audit result is memoized."""
from copy import deepcopy
import math,sys

def _freeze(value):
    return _freeze_sized(value)[0]

def _freeze_sized(value):
    """Own and account in one pass; repeated aliases are conservatively counted."""
    kind=type(value)
    if kind is FrozenCurve:return value,value._resident_bytes
    if kind is dict:
        result=FrozenCurve(value);return result,result._resident_bytes
    if kind in (list,tuple):
        items=[];size=0
        for item in value:
            frozen,weight=_freeze_sized(item);items.append(frozen);size+=weight
        result=tuple(items)
        return result,sys.getsizeof(result)+size
    if value is None or kind in (str,bool,int):return value,sys.getsizeof(value)
    if kind is float and math.isfinite(value):return value,sys.getsizeof(value)
    raise TypeError('CURVE_MODEL_NOT_PRIMITIVE_FINITE')

class FrozenCurve(dict):
    """JSON-compatible mapping with recursively owned immutable values."""
    def __init__(self,source):
        if getattr(self,'_sealed',False):raise TypeError('CURVE_MODEL_IMMUTABLE')
        if any(type(key) is not str for key in source):raise TypeError('CURVE_MODEL_KEY')
        fields={key:_freeze_sized(value) for key,value in source.items()}
        dict.__init__(self,((key,value) for key,(value,_) in fields.items()))
        self._field_bytes={key:weight for key,(_,weight) in fields.items()}
        self._resident_bytes=sys.getsizeof(self)+sum(sys.getsizeof(key)+weight for key,weight in self._field_bytes.items())
        self._sealed=True

    def _reject(self,*args,**kwargs):raise TypeError('CURVE_MODEL_IMMUTABLE')
    __setitem__=__delitem__=clear=pop=popitem=setdefault=update=__ior__=_reject

    def __deepcopy__(self,memo):
        memo[id(self)]=self
        return self

    def suffix(self,index):
        if index==0:return self
        # All unchanged members are already recursively immutable. Only the
        # new parameter tuple needs allocation and accounting.
        values=dict(self);values['parameters']=self['parameters'][index:]
        result=dict.__new__(FrozenCurve);dict.__init__(result,values)
        result._field_bytes=dict(self._field_bytes)
        result._field_bytes['parameters']=sys.getsizeof(values['parameters'])+sum(sys.getsizeof(x) for x in values['parameters'])
        result._resident_bytes=sys.getsizeof(result)+sum(sys.getsizeof(k)+v for k,v in result._field_bytes.items())
        result._sealed=True
        return result

def freeze_model(model):
    return None if model is None else _freeze(model)

def copy_internal_record(record):
    # Parent controls retain their established mutable-copy behavior.
    if type(record.get('curve_model')) is not FrozenCurve:return deepcopy(record)
    from .v9_r9_records import copy_for_cache
    return copy_for_cache(record)

def record_bytes(record):
    from .r6_cache import resident_bytes
    model=record.get('curve_model') if isinstance(record,dict) else None
    if type(model) is FrozenCurve:
        return resident_bytes(record,{id(model)})+model._resident_bytes
    return resident_bytes(record)

def repack_or_reuse(points,source=None):
    from .v9_r9_records import InternalPoints,pack_internal
    if type(source) is InternalPoints and type(points) is list and len(points)==len(source):
        import numpy as np
        keys=source.packed.keys
        valid=(len({id(row) for row in points})==len(points)
            and all(type(row) is dict and tuple(row)==keys for row in points))
        if valid:
            for key,(kind,value) in zip(keys,source.packed.columns):
                if kind=='float':
                    if any(type(row[key]) is not float for row in points):valid=False;break
                    column=np.fromiter((row[key] for row in points),np.float64,count=len(points))
                    if not np.array_equal(column.view(np.uint64),value.view(np.uint64)):valid=False;break
                elif kind=='constant':
                    if not all(type(row[key]) is type(value) and row[key]==value for row in points):valid=False;break
                else:valid=False;break # unfamiliar varying schemas retain the original pack path
        if valid:return source
    return pack_internal(points)
