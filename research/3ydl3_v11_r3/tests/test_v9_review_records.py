from dataclasses import dataclass
import numpy as np
from three_yd_v5.v9_records import Points,encode,decode
from three_yd_v5.fullroute import RouteCache
from three_yd_v5.r6_cache import resident_bytes


@dataclass
class Result:
    points:list
    diagnostics:dict


def test_packed_cache_preserves_bits_types_metadata_and_mutation_isolation(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R1_MODE','K')
    rows=[{'x':float(i),'y':-0.,'yaw':.3,'steering':.1,'velocity':0.,'source':'tested',
           'motion_direction':'forward','int':2**60+i,'bool':bool(i%2),'none':None} for i in range(2000)]
    record={'result':Result(rows,{}),'bounded_l3':{'limit':400000}}
    packed=encode(record)
    assert resident_bytes(packed)<resident_bytes(record)/3
    assert decode(packed)==record
    a=decode(packed);a['result'].points[0]['x']=-1;a['bounded_l3']['limit']=1
    assert decode(packed)==record and np.signbit(decode(packed)['result'].points[0]['y'])
    cache=RouteCache(2,2**20);assert cache.put('x',record)
    rows[0]['x']=100
    a=cache.get('x');assert a['result'].points[0]['x']==0
    a['result'].points[0]['source']='changed'
    assert cache.get('x')['result'].points[0]['source']=='tested'
    cache.remove('x');assert cache.bytes==0


def test_unknown_or_nested_point_schema_uses_original_copy():
    p=[{'x':1.,'extra':{'array':np.arange(5)}}]
    assert Points.encode(p) is None
    record={'result':Result(p,{})};packed=encode(record)
    assert '_v9_packed_points' not in packed
    decoded=decode(packed);decoded['result'].points[0]['extra']['array'][0]=9
    assert p[0]['extra']['array'][0]==0 and packed['result'].points[0]['extra']['array'][0]==0
    assert Points.encode([{'x':1.},{'different':2.}]) is None
