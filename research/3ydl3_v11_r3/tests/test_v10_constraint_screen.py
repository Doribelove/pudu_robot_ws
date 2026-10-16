import time
from types import SimpleNamespace as S
import numpy as np
import pytest
from three_yd_v5 import curve_l3 as l3,curve_qp as qp
from three_yd_v5.v10_storage import initial_scope,empty_overlay,retained_mask
from three_yd_v5.v9_maps import immutable

@pytest.mark.parametrize('guide,start,goal',[
    ([[2,3],[12,3]],(2,3,0),(12,3,0)),
    ([[2,1],[2,6],[7,6]],(2,1,np.pi/2),(7,6,0)),
])
def test_screened_qp_solution_satisfies_original_all_rows(monkeypatch,guide,start,goal):
    setup=qp.clarabel_setup;solve=qp.clarabel_solve;seen=[];solutions=[]
    def capture(P,q,A,lo,hi,*args):
        seen.append((A.copy(),lo.copy(),hi.copy()))
        return setup(P,q,A,lo,hi,*args)
    def answer(s):
        r=solve(s);solutions.append(r.x.copy());return r
    monkeypatch.setattr(qp,'clarabel_setup',capture);monkeypatch.setattr(qp,'clarabel_solve',answer)
    w=S(hard=np.zeros((300,300),bool),map=S(resolution=.05,origin=(0.,0.,0.)),vehicle=(.265,.225,.4))
    p,_,_=l3.optimize(w,~w.hard,guide,start,goal,time.monotonic()+5,dict(l3.CONFIG,certified_quadratic_start=False,trust_region_constraint_screen=False))
    assert p is not None
    A,lo,hi=seen[0];seen.clear();solutions.clear()
    p,_,log=l3.optimize(w,~w.hard,guide,start,goal,time.monotonic()+5,dict(l3.CONFIG,certified_quadratic_start=False,trust_region_constraint_screen=True))
    assert p is not None and log['certificate']['max_curvature_bound_1pm']<=2.50000001
    assert seen[0][0].shape[0]<A.shape[0]
    current=A@solutions[0]
    assert np.all(current>=lo-5e-4) and np.all(current<=hi+5e-4)
    assert log['iterations'][0]['redundant_rows_removed']>0

def test_scope_sharing_has_no_mutable_alias_and_semantic_scope_stays_separate():
    source=np.ones((40,50),bool);a,b=initial_scope(source,True)
    assert a is b and immutable(a);source[:]=False;assert a.all()
    a,b=initial_scope(source,False);b[1,1]=True;assert not a[1,1]
    r=retained_mask(a);assert immutable(r)
    with pytest.raises(ValueError):r.flags.writeable=True

def test_empty_overlay_has_constant_immutable_storage():
    a=empty_overlay((600,700));assert a.dtype==bool and not a.any()
    assert immutable(a) and a.strides==(0,0)
    with pytest.raises(ValueError):a[0,0]=True
