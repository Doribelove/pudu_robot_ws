from pathlib import Path
import math

import numpy as np
import pytest

from arena_evaluation.static_feasibility_r3 import component_certificate, necessary_center_mask, inscribed_radius
from arena_evaluation.planner_benchmark.map_utils import HospitalMap
from arena_evaluation.unified_four_backends_smoke import FOOTPRINT


def wall_map(gap,unknown=False):
    occupancy=np.zeros((100,100),np.int8);occupancy[:,49:51]=-1 if unknown else 100
    occupancy[50-gap//2:50+(gap+1)//2,49:51]=0
    return HospitalMap(Path('/fixture/map.yaml'),Path('/fixture/map.pgm'),.05,(0.,0.,0.),100,100,occupancy,np.zeros((100,100)))


@pytest.mark.parametrize('unknown',[False,True])
def test_narrow_channel_certifies_disconnection(unknown):
    m=wall_map(7,unknown)
    c=component_certificate(m,FOOTPRINT,(1.,2.5,0.),(4.,2.5,0.),'wall')
    assert c['verdict']=='CANONICAL_STATIC_DISCONNECTED'


def test_wide_channel_with_actual_safe_straight_path_cannot_be_rejected():
    m=wall_map(14)
    assert all(not m.footprint_collision((x,2.5,0.),FOOTPRINT,unknown_is_collision=True) for x in np.linspace(1.,4.,200))
    assert component_certificate(m,FOOTPRINT,(1.,2.5,0.),(4.,2.5,0.),'wall')['verdict']=='INCONCLUSIVE_CONNECTED_OVERAPPROXIMATION'


def test_blocked_cells_have_no_safe_pose_at_sampled_subcell_positions_and_headings():
    m=wall_map(7);mask,_=necessary_center_mask(m.occupancy,.05,FOOTPRINT)
    # Challenge cell-offset and arbitrary heading, including near cell corners.
    for row,col in [(46,48),(47,48),(48,49),(51,49),(52,50),(53,51)]:
        if mask[row,col]:continue
        x,y=m.cell_to_world((row,col))
        for dx,dy in [(-.0249,-.0249),(.0249,.0249),(-.0249,.0249),(.0249,-.0249),(0.,0.)]:
            for yaw in np.linspace(-math.pi,math.pi,37):
                assert m.footprint_collision((x+dx,y+dy,yaw),FOOTPRINT,unknown_is_collision=True)


def test_certificate_replay_and_map_binding():
    m=wall_map(7);args=(m,FOOTPRINT,(1.,2.5,0.),(4.,2.5,0.))
    a=component_certificate(*args,'one');b=component_certificate(*args,'one');c=component_certificate(*args,'two')
    assert a==b and a['hash']!=c['hash']


def test_resource_bound_is_explicit():
    with pytest.raises(ValueError,match='RESOURCE_BOUND'):
        necessary_center_mask(np.zeros((10,10),np.int8),.05,FOOTPRINT,max_cells=50)


@pytest.mark.parametrize('footprint',[[(1.,1.),(2.,1.),(2.,2.),(1.,2.)],[(0.,0.),(1.,0.),(0.,1.)]])
def test_uncentered_footprint_cannot_generate_proof(footprint):
    with pytest.raises(ValueError):inscribed_radius(footprint)
