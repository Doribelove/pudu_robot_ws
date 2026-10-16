from dataclasses import fields
from types import SimpleNamespace as S
import numpy as np
import pytest
from arena_3d_v1.multires_grid import make_roi as original
from three_yd_v5.request_roi import make_roi


@pytest.mark.parametrize('seed',range(8))
@pytest.mark.parametrize('layout',['dense','reverse','stride'])
def test_roi_matches_original_and_preserves_all_input_cells(seed,layout):
    rng=np.random.default_rng(seed);mask=rng.random((47,61))<.3;safe=rng.random(mask.shape)<.8
    mask[:5]=False;mask[-4:]=False;mask[:,:7]=False;mask[:,-3:]=False
    if layout=='reverse':mask=mask[::-1,::-1];safe=safe[::-1,::-1]
    if layout=='stride':mask=mask[::2,::2];safe=safe[::2,::2]
    ends=np.argwhere(mask&safe)[[0,-1]];start,goal=(tuple(x) for x in ends)
    view=S(safe=safe,map=S(height=len(mask),width=mask.shape[1],origin=(1.,2.,.2),resolution=.15),
        key='original-view',factor=3,vehicle=(.225,.265,.4))
    oldmask=mask.copy();oldsafe=safe.copy()
    a=original(view,mask,start,goal,'route');b=make_roi(view,mask,start,goal,'route')
    for f in fields(a):
        x,y=getattr(a,f.name),getattr(b,f.name)
        if isinstance(x,np.ndarray):assert np.array_equal(x,y) and x.dtype==y.dtype
        else:assert x==y
    assert np.array_equal(mask,oldmask) and np.array_equal(safe,oldsafe)


@pytest.mark.parametrize('factor',[1,3])
@pytest.mark.parametrize('point',[(-1,0),(0,5),(1,1)])
def test_invalid_endpoints_preserve_original_errors(factor,point):
    mask=np.zeros((4,5),bool);safe=np.ones_like(mask)
    view=S(safe=safe,map=S(height=4,width=5),factor=factor)
    with pytest.raises(ValueError) as a:original(view,mask,point,(0,0),'r')
    with pytest.raises(ValueError) as b:make_roi(view,mask,point,(0,0),'r')
    assert str(a.value)==str(b.value)
