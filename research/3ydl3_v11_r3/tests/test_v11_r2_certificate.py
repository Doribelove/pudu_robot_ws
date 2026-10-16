import importlib.util
from pathlib import Path
import time
from types import SimpleNamespace as S
import numpy as np
import pytest
from scipy.interpolate import BSpline
from three_yd_v5.curve_l3 import bezier_pieces,certify,CurveFailure
spec=importlib.util.spec_from_file_location('v11_r1_certificate_oracle',Path(__file__).parent/'oracles/v11_r1_certificate.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)

@pytest.mark.parametrize('seed',range(8))
def test_batched_piece_controls_equal_original_bit_for_bit(seed):
    rng=np.random.default_rng(seed)
    internal=np.sort(rng.uniform(.01,.99,27))
    internal=np.repeat(internal,rng.integers(1,4,len(internal)))
    knots=np.r_[[0.]*4,internal,[1.]*4]
    sp=BSpline(knots,rng.normal(size=(len(knots)-4,2)),3)
    a=list(old.bezier_pieces(sp));b=list(bezier_pieces(sp))
    assert len(a)==len(b)
    for (al,ar,ac),(bl,br,bc) in zip(a,b):
        assert al==bl and ar==br and np.array_equal(ac,bc)

@pytest.mark.parametrize('case',['straight','bend','body_collision','curvature','ordered_corridor','piece_refinement'])
@pytest.mark.parametrize('as_arrays',[False,True])
def test_complete_interval_certificate_matches_frozen_oracle(case,as_arrays):
    hard=np.zeros((300,300),bool);hard[[0,-1],:]=True;hard[:,[0,-1]]=True
    cp=[[2,2],[3,2],[4,2],[5,2]];knots=[0]*4+[1]*4
    if case=='bend':cp=[[2,2],[3,2],[4,3],[4,4]]
    if case=='body_collision':hard[259,70]=True;cp=[[2,2.2],[3,2.2],[4,2.2],[5,2.2]]
    if case=='curvature':cp=[[2,2],[3,2],[2,2.01],[3,2.01]]
    if case=='piece_refinement':
        knots=[0]*4+[1]*3+[2]*4;cp=[[2,2.25001],[3,2.25001],[4,2.25001],[5,2.25001],[6,2.25001],[7,2.5],[8,3]]
        hard[260,68]=True
    w=S(hard=hard,map=S(resolution=.05,origin=(0.,0.,0.)),vehicle=(.265,.225,.4))
    sp=BSpline(knots,cp,3)
    box=[0,1,0,1] if case=='ordered_corridor' else [1,12,1,12]
    corridor={'stations':[0,1,2],'bounds':[box]*3,'axes':[[1,0,0,1]]*3}
    if as_arrays:corridor={k:np.asarray(v) for k,v in corridor.items()}
    def run(fn):
        try:
            path,cert=fn(sp,w,~hard,time.monotonic()+20,corridor=corridor)
            cert.pop('certificate_ms');return path,cert,None
        except CurveFailure as error:return None,None,str(error)
    a,b=run(old.certify),run(certify)
    assert a[2]==b[2]
    if a[2] is None:assert np.array_equal(a[0],b[0]) and a[1]==b[1]
