"""Native local predicate must reproduce the frozen Python collision contract."""
import math
from pathlib import Path

import numpy as np
import pytest

from arena_evaluation import _endpoint_geometry_r3 as native
from arena_evaluation.planner_benchmark.map_utils import HospitalMap

FOOTPRINT = ((.255,.215),(.255,-.215),(-.255,-.215),(-.255,.215))


@pytest.mark.parametrize('origin', [(0.,0.,0.),(-2.35,1.125,0.)])
def test_original_polygon_cell_predicate_matches_across_random_yaw_padding_and_unknown(origin):
    rng=np.random.default_rng(240901)
    grid=rng.choice(np.array([0,100,-1],np.int8),size=(128,160),p=[.995,.003,.002])
    grid[[0,-1],:]=100;grid[:,[0,-1]]=100
    m=HospitalMap(Path('/synthetic.yaml'),Path('/synthetic.pgm'),.05,origin,160,128,grid,np.zeros(grid.shape))
    checker=native.make_checker(grid,.05,*origin[:2],FOOTPRINT)
    for i in range(1500):
        pose=(origin[0]+rng.uniform(-.1,8.1),origin[1]+rng.uniform(-.1,6.5),rng.uniform(-math.pi,math.pi))
        padding=[0.,.0005,.015,.04,.12][i%5]
        fp=FOOTPRINT if not padding else ((-.255-padding,-.215-padding),(-.255-padding,.215+padding),(.255+padding,.215+padding),(.255+padding,-.215-padding))
        expected=m.footprint_collision(pose,fp,unknown_is_collision=True)
        assert native.collision(checker,*pose,padding)==expected,(i,pose,padding)


def test_native_checker_keeps_buffer_alive_and_observes_occupancy_changes():
    grid=np.zeros((100,100),np.int8)
    checker=native.make_checker(grid,.05,0.,0.,FOOTPRINT)
    assert not native.collision(checker,2.5,2.5,0.,0.)
    grid[50,50]=-1
    assert native.collision(checker,2.5,2.5,0.,0.)
    del grid
    assert native.collision(checker,2.5,2.5,0.,0.)


@pytest.mark.parametrize('grid',[np.zeros((4,4),np.uint8),np.zeros((4,4),np.int16),np.zeros((4,4),np.int8)[:,::2]])
def test_native_rejects_unsupported_buffer_layout(grid):
    with pytest.raises(ValueError):native.make_checker(grid,.05,0.,0.,FOOTPRINT)


@pytest.mark.parametrize('pose',[(float('nan'),1.,0.,0.),(1.,1.,float('inf'),0.),(1.,1.,0.,-.01)])
def test_native_nonfinite_and_invalid_padding_fail_closed(pose):
    checker=native.make_checker(np.zeros((100,100),np.int8),.05,0.,0.,FOOTPRINT)
    with pytest.raises(ValueError):native.collision(checker,*pose)
