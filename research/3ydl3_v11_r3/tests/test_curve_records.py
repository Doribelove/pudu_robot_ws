from copy import deepcopy
from types import SimpleNamespace as S
import json,time
import numpy as np
import pytest
from three_yd_v5.curve_records import FrozenCurve,freeze_model,copy_internal_record,record_bytes
from three_yd_v5.r6_cache import resident_bytes

def model():return {'knots':[0,0,0,0,1,1,1,1],'coefficients':[[2.,2.],[3.,2.],[4.,2.],[5.,2.]],'degree':3,'parameters':[0.,.5,1.],'corridor':None}

def test_owned_recursive_geometry_is_immutable_and_json_compatible():
    raw=model();f=freeze_model(raw);raw['coefficients'][0][0]=99
    assert f['coefficients'][0][0]==2
    for change in (lambda:f.update({'degree':4}),lambda:f.pop('knots'),lambda:f.__setitem__('degree',4)):
        with pytest.raises(TypeError,match='IMMUTABLE'):change()
    with pytest.raises(TypeError):f['coefficients'][0][0]=99
    assert deepcopy(f) is f
    assert json.loads(json.dumps(f))==model()

def test_suffix_preserves_curve_without_changing_parent_parameters():
    f=freeze_model(model());g=f.suffix(1)
    assert f.suffix(0) is f and f['parameters']==(0.,.5,1.)
    assert g['parameters']==(.5,1.)
    assert g['coefficients']==f['coefficients'] and g['knots']==f['knots']

def test_only_curve_internal_record_uses_shared_model_and_detached_metadata():
    r={'curve_model':freeze_model(model()),'curve_l3':{'attempts':[1]},'result':None}
    c=copy_internal_record(r);c['curve_l3']['attempts'].append(2)
    assert r['curve_l3']['attempts']==[1] and c['curve_model'] is r['curve_model']
    assert record_bytes(r)>=resident_bytes(r)
    parent={'curve_model':None,'points':[{'x':1.}]};child=copy_internal_record(parent)
    child['points'][0]['x']=2;assert parent['points'][0]['x']==1

def test_unknown_objects_and_nonfinite_geometry_are_not_shared():
    for value in ({'bad':object()},{'bad':float('nan')}):
        with pytest.raises(TypeError):freeze_model(value)

def test_curve_certificate_is_reexecuted_on_each_immutable_delivery(monkeypatch):
    from three_yd_v5 import curve_l3,curve_backend
    from scipy.interpolate import BSpline
    monkeypatch.setenv('THREEYD_L3_BACKEND','independent')
    hard=np.zeros((240,240),bool)
    w=S(hard=hard,map=S(resolution=.05,origin=(0,0,0)),vehicle=(.265,.225,.4))
    raw=model();raw['parameters']=np.linspace(0,1,121).tolist()
    sp=BSpline(raw['knots'],raw['coefficients'],3);u=np.array(raw['parameters']);p=np.c_[sp(u),np.zeros(len(u))]
    r={'curve_model':freeze_model(raw),'result':S(points=[dict(x=x,y=y,yaw=yaw) for x,y,yaw in p]),'l3_backend_identity':list(curve_backend.identity())}
    c=S(world=w,_target_mask=lambda:~hard);session=S(_curve_request={'deadline':time.monotonic()+3})
    original=curve_l3.certify;calls=[]
    def checked(*args,**kw):calls.append(1);return original(*args,**kw)
    monkeypatch.setattr(curve_l3,'certify',checked)
    for _ in range(2):assert curve_backend.validate_record(c,None,r,session)[0]==[]
    assert len(calls)==2
    r['curve_model']=freeze_model({**raw,'coefficients':[[2.,3.],[3.,3.],[4.,3.],[5.,3.]]})
    assert curve_backend.validate_record(c,None,r,session)[0]==['CURVE_OUTPUT_BINDING_CHANGED']
