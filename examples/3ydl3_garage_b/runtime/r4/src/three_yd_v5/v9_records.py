"""Compact immutable route geometry; every delivery gets new mutable points.

Only flat primitive point fields are packed. Unknown/nested schemas retain
the parent deepcopy path. No audit, ACK, controller or map is stored here.
"""
from copy import deepcopy
from dataclasses import dataclass, replace
import numpy as np
from .v9_maps import seal


@dataclass(frozen=True)
class Points:
    keys: tuple
    columns: tuple
    count: int

    @classmethod
    def encode(cls,points):
        if not points or any(not isinstance(p,dict) for p in points):return None
        keys=tuple(points[0])
        if any(tuple(p)!=keys for p in points):return None
        cols=[]
        for key in keys:
            values=tuple(p[key] for p in points)
            if not all(type(v) in (float,int,bool,str,type(None)) for v in values):return None
            # Float storage preserves every bit including signed zero/NaN.
            if all(type(v) is float for v in values):
                cols.append(('float',seal(np.asarray(values,np.float64))))
            elif all(type(v) is type(values[0]) and v==values[0] for v in values):
                cols.append(('constant',values[0]))
            else:cols.append(('tuple',values))
        return cls(keys,tuple(cols),len(points))

    def decode(self):
        columns=[(value.tolist() if kind=='float' else [value]*self.count if kind=='constant' else value)
                 for kind,value in self.columns]
        return [dict(zip(self.keys,row)) for row in zip(*columns)]


def encode(record):
    if not isinstance(record,dict):return deepcopy(record)
    result=record.get('result')
    if result is None:return deepcopy(record)
    points=Points.encode(result.points)
    if points is None:return deepcopy(record)
    other={key:deepcopy(value) for key,value in record.items() if key!='result'}
    # Copy all other result fields via the ordinary parent contract.
    other['result']=deepcopy(replace(result,points=[]))
    other['_v9_packed_points']=points
    return other


def decode(record):
    if not isinstance(record,dict):return deepcopy(record)
    points=record.get('_v9_packed_points')
    if points is None:return deepcopy(record)
    other={key:deepcopy(value) for key,value in record.items() if key not in ('result','_v9_packed_points')}
    other['result']=replace(deepcopy(record['result']),points=points.decode())
    return other
