import numpy as np
from types import SimpleNamespace as S
from three_yd_v5.progress_connector import quintic,propose
from three_yd_v5.task_progress import wrap,ProgressTraffic


def test_quintic_real_endpoints_and_heading():
    a=np.array([0,.03,np.deg2rad(2)]);b=np.array([1.2,0,0])
    p=quintic(a,b,0,1.2)
    assert np.array_equal(p[0],a) and np.array_equal(p[-1],b)
    ds=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
    k=abs(wrap(np.diff(p[:,2])))/ds
    assert k.max()<2.5
    assert np.all(np.diff(p[:,0])>0)


def service():
    p=np.column_stack((np.arange(0,4,.05),np.zeros(80),np.zeros(80)))
    return S(remaining=p,prefix=np.empty((0,3)),base_traffic=S(audit=lambda p:{'valid':True,'failures':[]}),
        controller=S(_target_mask=lambda:np.ones((3,3),bool)),
        world=S(audit=lambda *a:{'valid':True,'failures':[]}))


def test_proposal_does_not_snap_start_and_has_bounded_join():
    s=service();q=S(start=(.50,.03,np.deg2rad(2)))
    value,reason=propose(s,q);assert reason=='LOCAL_CONNECTOR'
    index,tail,log=value;assert index==10 and np.array_equal(tail[0],q.start)
    j=log['trials'][-1]['join_vertex']
    assert .8<=s.remaining[j,0]-s.remaining[index,0]<=3.1


def test_body_rejection_never_returns_geometry():
    s=service();s.world.audit=lambda *a:{'valid':False,'failures':['COLLISION']}
    value,reason=propose(s,S(start=(.5,.03,0)))
    assert value is None and reason=='NO_STRICT_LOCAL_CONNECTOR'


def test_traffic_rejection_never_returns_geometry():
    s=service();s.base_traffic.audit=lambda p:{'valid':False,'failures':['PARKING_TRANSIT']}
    assert propose(s,S(start=(.5,.03,0)))[0] is None


def test_large_position_or_heading_difference_rejected():
    s=service()
    assert propose(s,S(start=(.5,.16,0)))[1]=='POSE_TOO_FAR_FROM_REMAINING_ROUTE'
    assert propose(s,S(start=(.5,0,np.deg2rad(11))))[1]=='HEADING_NOT_ALONG_ROUTE'
