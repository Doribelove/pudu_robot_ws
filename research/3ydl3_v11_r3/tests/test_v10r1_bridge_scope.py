import numpy as np
from types import SimpleNamespace
from three_yd_v5.rectangle_bridge import first_component_break

def test_component_diagnostic_uses_actual_route_scope_not_outside_bypass():
    free=np.ones((7,9),bool);free[2:5,4]=False
    scope=np.zeros_like(free);scope[2:5,1:8]=True
    guide=np.c_[np.full(7,3),np.arange(1,8)]
    view=SimpleNamespace(world_points=lambda r,c:np.c_[c.astype(float),r.astype(float)])
    outside=first_component_break(guide,free,view)
    actual=first_component_break(guide,free&scope,view)
    assert not outside['found']
    assert actual['found'] and actual['source_cell']==[3,3] and actual['target_cell']==[3,5]
    assert actual['source_component']!=actual['target_component']
