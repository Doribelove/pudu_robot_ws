from types import SimpleNamespace as S
import numpy as np
import pytest
from three_yd_v5.local_detour import extend_hint


def world():
    w=S(map=S(resolution=.5),hard=np.zeros((40,60),bool))
    w.cells=lambda xy:(np.floor((xy[:,1]+10)/.5).astype(int),np.floor((xy[:,0]-7)/.5).astype(int))
    w.world_points=lambda r,c:np.column_stack((7+(c+.5)*.5,-10+(r+.5)*.5))
    return w


@pytest.mark.parametrize('center',[(8.,-9.),(22.,0.),(36.,9.)])
@pytest.mark.parametrize('radius',[3.,5.,8.])
def test_hint_stays_within_metric_disk_and_neutral_region(center,radius):
    w=world();labels=np.ones(w.hard.shape,int);labels[:,25:30]=7
    before=labels.copy();extension=np.zeros(w.hard.shape,bool)
    extend_hint(extension,w,labels,np.array(center),radius)
    rr,cc=np.indices(w.hard.shape);xy=w.world_points(rr.ravel(),cc.ravel())
    expected=((np.linalg.norm(xy-center,axis=1)<=radius).reshape(w.hard.shape))&(labels==1)
    assert np.array_equal(extension,expected) and extension.any()
    assert np.array_equal(labels,before) and not w.hard.any()
    saved=extension.copy();extend_hint(extension,w,labels,np.array(center),3.)
    assert np.array_equal(saved,extension)


@pytest.mark.parametrize('radius',[0.,-3.,9.,np.inf])
def test_unbounded_extension_is_rejected(radius):
    w=world();extension=np.zeros(w.hard.shape,bool)
    with pytest.raises(ValueError):extend_hint(extension,w,np.ones(w.hard.shape,int),[22.,0.],radius)
    assert not extension.any()
