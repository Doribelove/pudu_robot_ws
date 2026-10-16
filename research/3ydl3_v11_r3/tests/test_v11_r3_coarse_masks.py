import numpy as np
import pytest
from three_yd_v5.coarse_masks import pair,hard_for,false_field
from three_yd_v5.v9_maps import seal,immutable

@pytest.mark.parametrize('shape',[(0,3),(3,0),(1,1),(7,11),(97,8),(203,127)])
@pytest.mark.parametrize('value',[0,1,2])
def test_all_cells_and_shared_storage(shape,value):
    a=np.random.default_rng(91).integers(0,2,shape,dtype=np.uint8).astype(bool)
    if value<2:a[:]=bool(value)
    safe,hard=pair(a)
    assert immutable(safe) and immutable(hard)
    assert hard.tobytes()==a.tobytes()
    assert safe.tobytes()==(~a).tobytes()
    assert hard_for(safe,safe,hard) is hard
    a[:]=~a
    assert hard_for(safe,safe,hard).tobytes()==(~safe).tobytes()

@pytest.mark.parametrize('kind',['safe_mutable','hard_mutable','readonly_alias','wrong_complement','wrong_shape','different_safe','reversed','uint8'])
def test_untrusted_reference_falls_back(kind):
    a=np.random.default_rng(8).integers(0,2,(201,17)).astype(bool)
    safe,hard=pair(a);reference=safe
    if kind=='safe_mutable':safe=safe.copy();reference=safe
    elif kind=='hard_mutable':hard=hard.copy()
    elif kind=='readonly_alias':
        hard=hard.copy();alias=hard.view();hard.flags.writeable=False;alias[199,2]=~alias[199,2]
    elif kind=='wrong_complement':
        h=hard.copy();h[199,2]=~h[199,2];hard=seal(h)
    elif kind=='wrong_shape':hard=hard[:3]
    elif kind=='different_safe':safe=seal(~safe)
    elif kind=='reversed':safe=safe[::-1]
    elif kind=='uint8':hard=seal(hard.astype(np.uint8))
    got=hard_for(safe,reference,hard)
    assert got is not hard
    assert got.tobytes()==(~np.asarray(safe,bool)).tobytes()

@pytest.mark.parametrize('shape',[(0,3),(1,1),(71,13),(203,127)])
def test_constant_field_exact_and_immutable(shape):
    a=false_field(shape)
    assert a.tobytes()==np.zeros(shape,bool).tobytes()
    assert immutable(a)
    with pytest.raises(ValueError):a.flags.writeable=True
