from types import SimpleNamespace
import math

import numpy as np
import pytest

from arena_evaluation.two_layer_v1_r3_benchmark import corridor,CorridorCache


def context():
    m=SimpleNamespace(height=300,width=500,resolution=.05,origin=(0.,0.,0.),
                      occupancy=np.zeros((300,500),np.int8),
                      world_to_cell=lambda x,y:(299-int(y/.05),int(x/.05)),
                      clearance=lambda *args:5.)
    return SimpleNamespace(hospital_map=m,map_sha256='fixed-map')


def test_narrow_profile_is_subset_and_retains_centerline_and_footprint():
    ctx=context();route=SimpleNamespace(polyline=[(x,5.) for x in np.linspace(2,22,401)])
    original,_=corridor(ctx,route);narrow,info=corridor(ctx,route,straight_half_width_m=.8)
    assert np.all(~narrow|original) and np.count_nonzero(narrow)<np.count_nonzero(original)
    radius=math.hypot(.255,.215)+.05
    for x in np.linspace(2,22,100):
        for theta in np.linspace(-math.pi,math.pi,48):
            assert narrow[ctx.hospital_map.world_to_cell(x+radius*math.cos(theta),5.+radius*math.sin(theta))]
    assert info['corridor_straight_half_width_m']==.8


def test_single_expansion_and_bend_margins_remain_original():
    ctx=context();route=SimpleNamespace(polyline=[(x,5.) for x in np.linspace(2,12,201)]+
                                       [(12.,y) for y in np.linspace(5,12,141)])
    a,ai=corridor(ctx,route,expansion=True)
    b,bi=corridor(ctx,route,expansion=True,straight_half_width_m=.8)
    assert np.array_equal(a,b) and ai['corner_count']==bi['corner_count']
    assert bi['corridor_straight_half_width_m']==2.+.05+.15


@pytest.mark.parametrize('width',[.1,.59,1.21,float('nan'),float('inf')])
def test_invalid_profile_rejected(width):
    with pytest.raises(ValueError):CorridorCache(straight_half_width_m=width)


def test_cache_result_matches_its_width_profile():
    ctx=context();route=SimpleNamespace(polyline=[(2.,5.),(22.,5.)])
    cache=CorridorCache(straight_half_width_m=.8)
    narrow,first=cache.get(ctx,route);replay,second=cache.get(ctx,route)
    assert not first['corridor_cache_hit'] and second['corridor_cache_hit']
    assert np.array_equal(narrow,replay)
    wider,_=CorridorCache(straight_half_width_m=1.2).get(ctx,route)
    assert np.count_nonzero(wider)>np.count_nonzero(narrow)
