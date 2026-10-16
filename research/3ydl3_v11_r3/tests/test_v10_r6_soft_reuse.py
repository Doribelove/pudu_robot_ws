import numpy as np
import pytest
from three_yd_v5.r4_tiles import TiledSoft,reference_soft

@pytest.mark.parametrize('shape',[(17,31),(255,257),(513,777)])
@pytest.mark.parametrize('cap',[0.,140.,240.])
def test_reference_field_reuse_keeps_each_original_tile_byte(monkeypatch,shape,cap):
    rng=np.random.default_rng(88);drive=rng.random(shape)>.25
    ref=np.array([[0,0],[shape[0]//3,shape[1]//2],[shape[0]-1,shape[1]-1]])
    before=drive.copy();monkeypatch.setenv('THREEYD_V10_R6_MODE','control')
    old,od=reference_soft(shape,ref,.05,drive,cap,tile=128);m=old.max()
    monkeypatch.setenv('THREEYD_V10_R6_MODE','Y')
    new,nd=reference_soft(shape,ref,.05,drive,cap,tile=128)
    assert od==nd and m==new.max() and len(old.pieces)==len(new.pieces)
    for (bo,a),(bn,b) in zip(old.pieces,new.pieces):assert bo==bn and a.tobytes()==b.tobytes()
    assert np.array_equal(drive,before)

@pytest.mark.parametrize('kind',['all','none','partial','noncanonical','empty','uint8'])
def test_max_retains_original_truth_and_fallback_semantics(monkeypatch,kind):
    if kind=='empty':drive=np.empty((0,7),bool)
    else:
        drive=np.ones((17,31),bool)
        if kind=='none':drive[:]=False
        if kind=='partial':drive[3,5]=False
        if kind=='noncanonical':drive=np.full(drive.shape,255,np.uint8).view(bool)
        if kind=='uint8':drive=drive.astype(np.uint8)
    soft=TiledSoft(drive.shape,140.,drive,[])
    monkeypatch.setenv('THREEYD_V10_R6_MODE','control');old=soft.max()
    monkeypatch.setenv('THREEYD_V10_R6_MODE','Y');assert soft.max()==old
