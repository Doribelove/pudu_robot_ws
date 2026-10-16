import time
from types import SimpleNamespace as S
import numpy as np
import pytest
from scipy.interpolate import BSpline
from three_yd_v5.curve_l3 import certify
from three_yd_v5.curve_backend import validate_record
from test_curve_l3 import world


@pytest.mark.parametrize('change,expected',[
    ('same',[]),('start_position',['CURVE_ACTUAL_START_MISMATCH']),
    ('start_yaw',['CURVE_ACTUAL_START_MISMATCH']),
    ('goal_position',['CURVE_ACTUAL_GOAL_MISMATCH']),
    ('goal_yaw',['CURVE_ACTUAL_GOAL_MISMATCH']),
])
def test_curve_reuse_binds_the_actual_requested_endpoints(monkeypatch,change,expected):
    monkeypatch.setenv('THREEYD_L3_BACKEND','independent')
    w=world();sp=BSpline([0,0,0,0,5,5,5,5],[[2,2],[3,2],[5,2],[6,2]],3)
    p,cert=certify(sp,w,~w.hard,time.monotonic()+3)
    record={'curve_model':{'knots':sp.t.tolist(),'coefficients':sp.c.tolist(),
        'parameters':cert['sample_parameters']},
        'result':S(points=[dict(zip(('x','y','yaw'),v)) for v in p])}
    start=p[0].copy();goal=p[-1].copy()
    if change=='start_position':start[1]+=.03
    if change=='start_yaw':start[2]+=np.deg2rad(2)
    if change=='goal_position':goal[1]+=.03
    if change=='goal_yaw':goal[2]+=np.deg2rad(2)
    c=S(world=w,_target_mask=lambda:~w.hard)
    errors,_=validate_record(c,S(start=start,goal=goal),record,
        S(_curve_request={'deadline':time.monotonic()+3}))
    assert errors==expected


def test_no_change_event_cannot_leave_an_unvalidated_record(monkeypatch):
    from test_task_progress import fake_service
    from three_yd_v5.fullroute_service import FullRouteService
    service,q,path=fake_service()
    service.retained_record=service.record;service.retained_path=path.copy()
    service.record=None;service.remaining=None
    service.controller.trajectory=None;service.controller.trajectory_version=None
    def no_change(self,*args,**kwargs):
        assert np.array_equal(self.controller.traffic.prefix,self.prefix)
        return {'trajectory_available':False}
    monkeypatch.setattr(FullRouteService,'update',no_change)
    service._guarded_update(1)
    assert service.record is None and service.remaining is None
    assert service.retained_record is not None and service.task_query is q
