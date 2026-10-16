"""Boundary-label repair must not turn a real detour into a legal road route."""
from types import SimpleNamespace
import numpy as np
from three_yd_v5.controller import V5Traffic


def fixture(labels, roads):
    # One path point per 0.05 m map cell.  A 3x3 neighborhood is the only
    # semantic tolerance allowed by the r7 raster-boundary rule.
    grid=np.asarray([labels],dtype=np.int16)
    class World:
        map=SimpleNamespace(height=1,width=len(labels))
        def cells(self,points):
            col=np.clip(np.rint(np.asarray(points)[:,0]/.05).astype(int),0,len(labels)-1)
            return np.zeros(len(col),int),col
    obj=object.__new__(V5Traffic)
    obj.world=World()
    obj.topology=SimpleNamespace(subregions=grid,regions={
        1:{'kind':'lane'},2:{'kind':'lane'},3:{'kind':'lane'}})
    obj.roads=[(rid,np.asarray(direction,float)) for rid,direction in roads]
    x=np.arange(len(labels),dtype=float)*.05
    path=np.column_stack((x,np.zeros(len(x)),np.zeros(len(x))))
    return obj,path


def test_same_road_repeated_junction_stages_are_one_road():
    audit,path=fixture([1]*8,[(1,(1,0))]*3)
    result=audit._robust_road_sequence(path)
    assert result['valid']
    assert result['expected_distinct_roads']==[1]


def test_single_adjacent_raster_cell_is_boundary_not_new_road():
    audit,path=fixture([1,1,1,2,1,1],[(1,(1,0))])
    result=audit._robust_road_sequence(path)
    assert result['valid']
    assert len(result['raster_boundary_runs'])==1


def test_nonselected_road_travel_is_rejected():
    audit,path=fixture([1]*4+[2]*8+[1]*4,[(1,(1,0))])
    result=audit._robust_road_sequence(path)
    assert not result['valid']
    assert result['reason']=='NONSELECTED_ROAD_TRAVEL'


def test_genuine_different_road_stage_is_required():
    audit,path=fixture([1]*8,[(1,(1,0)),(3,(1,0))])
    result=audit._robust_road_sequence(path)
    assert not result['valid']
    assert result['expected_distinct_roads']==[1,3]


def test_same_road_reverse_direction_is_not_collapsed():
    audit,path=fixture([1]*8,[(1,(1,0)),(1,(-1,0))])
    result=audit._robust_road_sequence(path)
    assert not result['valid']
    assert result['reason']=='SAME_ROAD_DIRECTION_CHANGE'
