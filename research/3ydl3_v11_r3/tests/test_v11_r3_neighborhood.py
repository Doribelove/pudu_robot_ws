from types import SimpleNamespace as S
import numpy as np
import pytest
from three_yd_v5.geometry import variable_neighborhood, corners
from oracles.v11_r2_geometry import variable_neighborhood as original


@pytest.mark.parametrize('seed',range(8))
@pytest.mark.parametrize('kind',['interior','border','outside','long','reversed'])
def test_every_scope_pixel_and_metadata_match_frozen_original(seed,kind):
    rng=np.random.default_rng(seed);shape=(317,541);res=.05
    world=S(hard=np.zeros(shape,bool),map=S(resolution=res))
    world.cells=lambda xy:(np.floor(np.asarray(xy)[:,1]/res).astype(int),np.floor(np.asarray(xy)[:,0]/res).astype(int))
    world.world_points=lambda rr,cc:np.column_stack(((cc+.5)*res,(rr+.5)*res))
    points=np.array([[5.,5.],[7.,7.],[10.,6.]])+rng.uniform(-.1,.1,(3,2))
    if kind=='border':points-=5.
    if kind=='outside':points-=20.
    if kind=='long':points=np.array([[-3.,1.],[20.,10.],[30.,18.]])+rng.uniform(-.1,.1,(3,2))
    if kind=='reversed':points=points[::-1]
    records=corners(points);forced=points[1:2] if seed%2 else None
    a,detail0=original(world,points,records,forced_turning_points=forced)
    b,detail1=variable_neighborhood(world,points,records,forced_turning_points=forced)
    assert np.array_equal(a,b) and a.dtype==b.dtype==np.bool_
    assert detail0==detail1 and b.flags.owndata and b.flags.c_contiguous
