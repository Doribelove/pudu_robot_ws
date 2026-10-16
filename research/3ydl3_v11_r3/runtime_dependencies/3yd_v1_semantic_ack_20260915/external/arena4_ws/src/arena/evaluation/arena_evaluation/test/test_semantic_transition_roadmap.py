from types import SimpleNamespace

import numpy as np

from arena_evaluation.semantic_transition_roadmap import sample_sites, _poses


def test_sites_derive_only_from_geometry_and_keep_exact_endpoints():
    shape=(41,81)
    world=SimpleNamespace(master=np.zeros(shape,dtype=np.uint8),
        grids={'allowed':np.ones(shape,bool),'hard':np.zeros(shape,bool),
               'labels':np.ones(shape,np.int16),'correct':np.ones(shape,bool),
               'error':np.zeros(shape)},selected=[1],bound_length=8.,
        start=(.125,.425,.2),goal=(3.525,1.225,.7),
        map=SimpleNamespace(full_origin=(0.,0.),full_height=41,row0=0,col0=0,
                            resolution=.05,distance_m=np.ones(shape)))
    world.map.world_to_cell=lambda x,y: (40-int(y/.05),int(x/.05))
    first,_,_=sample_sites(world,20)
    second,_,_=sample_sites(world,20)
    assert np.array_equal(first,second)
    assert tuple(first[0])==world.start[:2]
    assert tuple(first[1])==world.goal[:2]
    assert len(first)<=20


def test_internal_headings_are_48_bins_and_exact_endpoint_yaws_survive():
    sites=np.asarray([[0.,0.],[1.,0.],[1.,1.]])
    world=SimpleNamespace(start=(0.,0.,.21),goal=(1.,0.,.45),
                          collision_free=lambda samples:True)
    poses,owners,_=_poses(sites,world)
    assert poses[:2]==[world.start,world.goal]
    for pose in poses[2:]:
        value=pose[2]*48/(2*np.pi)
        assert abs(value-round(value))<1e-12
    assert len(poses)==len(owners)
