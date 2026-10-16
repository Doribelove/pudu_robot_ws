from types import SimpleNamespace
import math
import numpy as np
from three_yd_v5.geometry import segment_cells,shortcut_path,corners,variable_neighborhood
from three_yd_v5.controller import inflate_circle


def test_supercover_blocks_diagonal_corner_cutting():
    assert set(map(tuple,segment_cells([1,1],[2,2])))=={(1,1),(2,2),(1,2),(2,1)}


def test_straightening_does_not_cross_wall():
    free=np.ones((20,20),bool);free[0:13,9]=False
    path=np.array([[5,c] for c in range(3,8)]+[[r,7] for r in range(6,15)]+[[14,c] for c in range(8,15)]+[[r,14] for r in range(13,4,-1)])
    result=shortcut_path(path,free,np.zeros_like(free,float))
    assert len(result)<len(path)
    for a,b in zip(result,result[1:]):
        rc=segment_cells(a,b);assert free[rc[:,0],rc[:,1]].all()


def test_straightening_preserves_soft_right_preference():
    free=np.ones((20,20),bool);cost=np.full((20,20),100.)
    path=np.array([[2,c] for c in range(2,16)]+[[r,15] for r in range(3,16)])
    cost[path[:,0],path[:,1]]=0
    result=shortcut_path(path,free,cost)
    assert len(result)>=3


def test_circle_inflation_preserves_obstacle_square():
    hard=np.zeros((61,61),bool);hard[30,30]=True
    radius=math.hypot(.265,.225)
    expanded=inflate_circle(hard,radius,.05)
    assert expanded[30,30] and expanded[30,37]
    assert not expanded[30,38]
    assert not expanded[40,40]


def test_variable_scope_widens_only_near_corner():
    world=SimpleNamespace(hard=np.zeros((400,400),bool),map=SimpleNamespace(resolution=.05))
    world.cells=lambda xy:(np.floor(xy[:,1]/.05).astype(int),np.floor(xy[:,0]/.05).astype(int))
    p=np.array([[3.,3.],[10.,3.],[10.,10.]])
    cr=corners(p);assert len(cr)==1 and abs(cr[0]['turn_angle_deg']-90)<1e-6
    mask,info=variable_neighborhood(world,p,cr)
    assert mask[60+21,200]
    assert not mask[60+21,90]
    assert not mask[60+25,200-25]
    assert info['max_sample_radius_m']<=1.2


def test_straight_path_has_no_corner_widening():
    assert corners(np.array([[0.,0.],[2.,0.],[4.,0.]]))==[]
