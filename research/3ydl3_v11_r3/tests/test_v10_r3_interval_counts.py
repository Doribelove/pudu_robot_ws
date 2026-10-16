import math
import numpy as np
import pytest
from three_yd_v5.curve_interpolation import exact_interpolation
from three_yd_v5.request_interpolation import interval_data


def compare(monkeypatch,p,ds=.025,dyaw=math.pi/180):
    monkeypatch.setenv('THREEYD_V10_R3_MODE','control');a=exact_interpolation(p,ds,dyaw)
    monkeypatch.setenv('THREEYD_V10_R3_MODE','I');b=exact_interpolation(p,ds,dyaw)
    assert a.shape==b.shape and np.array_equal(a.view(np.uint64),b.view(np.uint64))


@pytest.mark.parametrize('seed',range(32))
def test_random_full_interpolated_bytes(monkeypatch,seed):
    rng=np.random.default_rng(seed)
    p=np.cumsum(rng.uniform(-.075,.075,(170,3)),axis=0)
    compare(monkeypatch,p,ds=[.025,.03,.1][seed%3],dyaw=[math.pi/180,.07][seed%2])


@pytest.mark.parametrize('scale',[1.,1e-155,1e-165,1e140])
def test_integer_count_boundaries_and_nextafter(monkeypatch,scale):
    ds=.025*scale
    for n in (1,2,3,7,32):
        x=n*ds
        for distance in (np.nextafter(x,-np.inf),x,np.nextafter(x,np.inf)):
            for yaw in (0.,math.pi/180,np.nextafter(math.pi/180,np.inf)):
                compare(monkeypatch,np.array([[0.,0.,0.],[distance,0.,yaw]]),ds)


def test_wrap_boundaries_signed_zero_and_repeated_points(monkeypatch):
    for yaw in (-0.,0.,-math.pi,math.pi,-2*math.pi,2*math.pi,1e16,-1e16):
        for value in (np.nextafter(yaw,-np.inf),yaw,np.nextafter(yaw,np.inf)):
            p=np.array([[0.,0.,0.],[0.,0.,value],[0.,0.,value]])
            compare(monkeypatch,p)


def test_count_envelope_against_scalar_norm_for_random_scales(monkeypatch):
    monkeypatch.setenv('THREEYD_V10_R3_MODE','I')
    rng=np.random.default_rng(117)
    for exponent in (-160,-150,-30,0,30,140):
        scale=10.**exponent;ds=.025*scale
        xy=rng.normal(size=(2000,2))*scale
        p=np.column_stack((np.cumsum(xy,axis=0),np.zeros(len(xy))))
        d,counts=interval_data(p,ds,.1)
        original=np.array([max(1,math.ceil(np.linalg.norm(row[:2])/ds)) for row in d])
        assert np.array_equal(counts,original)
