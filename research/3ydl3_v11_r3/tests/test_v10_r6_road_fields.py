from types import SimpleNamespace as S
import numpy as np
import pytest
from arena_3d_v1.semantic_world import SemanticWorld
from arena_3d_v1.multires_grid import GridView
from three_yd_v5.r4_fields import _road_fields_original as original
from three_yd_v5.local_road_fields import road_fields


def fixture(shape,origin,resolution):
    w=SemanticWorld.__new__(SemanticWorld);w.map=S(height=shape[0],width=shape[1],origin=origin,resolution=resolution)
    v=GridView.__new__(GridView);v.map=S(height=(shape[0]+2)//3,width=(shape[1]+2)//3,origin=origin,resolution=resolution*3)
    rng=np.random.default_rng(921)
    labels=rng.choice(np.array([0,1,5,30000],np.int32),size=shape)
    poly=np.array([[-100.,-100.],[500.,-100.],[500.,500.],[-100.,500.]])
    top=S(subregions=labels,contexts={'q':{'unique_roads':[(1,np.array([1.,0.])),(5,np.array([.6,.8])),(30000,np.array([-.8,.6]))]}},router=S(polygons={i:poly for i in (1,5,30000)}))
    return S(world=w,topology=top,topology_route=S(signature='q'),generic_mode=False),v


@pytest.mark.parametrize('shape',[(97,136),(513,777)])
@pytest.mark.parametrize('origin',[(0.,0.,0.),(-11.135,7.005,0.)])
@pytest.mark.parametrize('resolution',[.05,.0500000000001])
@pytest.mark.parametrize('chunk',[13,65536])
def test_exact_original_fields_with_edges_rounding_and_nonzero_origins(shape,origin,resolution,chunk):
    c,v=fixture(shape,origin,resolution);box=(0,v.map.height,0,v.map.width)
    expected=original(c,v,box,chunk);actual=road_fields(c,v,box,chunk)
    assert actual is not None
    for a,b in zip(actual,expected):assert a.shape==b.shape and a.dtype==b.dtype and a.tobytes()==b.tobytes()


def test_unknown_coupled_coordinate_transform_falls_back():
    c,v=fixture((17,19),(0.,0.,0.),.05)
    v.world_points=lambda r,k:np.column_stack((np.asarray(r)+k,np.asarray(r)-k))
    assert road_fields(c,v,(0,5,0,6)) is None
