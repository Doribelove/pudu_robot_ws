import cv2
import numpy as np
import pytest
from three_yd_v5.recovery_masks import initial_masks,dilated_boolean


@pytest.mark.parametrize('shape',[(17,31),(32,64),(67,29)])
@pytest.mark.parametrize('radius',[1,3,9])
def test_masks_match_original_and_preserve_live_inputs(shape,radius):
    rng=np.random.default_rng(709)
    hard=rng.random(shape)<.2;selected=rng.random(shape)<.05;all_physical=rng.random(shape)<.7
    before=[x.tobytes() for x in (hard,selected)]
    margin=cv2.dilate(selected.astype(np.uint8),np.ones((2*radius+1,)*2,np.uint8))>0
    expected=all_physical&margin
    drivable,physical=initial_masks(hard,selected,all_physical,radius)
    assert drivable is all_physical
    assert drivable.tobytes()==expected.tobytes()
    assert physical.tobytes()==(~hard&expected).tobytes()
    assert before==[x.tobytes() for x in (hard,selected)]


def test_immutable_all_physical_does_not_alias_output():
    hard=np.zeros((8,8),bool);selected=np.eye(8,dtype=bool)
    original=np.frombuffer(bytes([1]*64),dtype=bool).reshape(8,8)
    drivable,physical=initial_masks(hard,selected,original,1)
    assert original.all() and not np.shares_memory(drivable,original)
    assert np.array_equal(drivable,physical)


def test_dilation_normalizes_noncanonical_bool_bytes():
    a=np.zeros((9,9),np.uint8);a[4,4]=255;a=a.view(bool)
    kernel=np.ones((3,3),np.uint8)
    assert dilated_boolean(a,kernel).tobytes()==(cv2.dilate(a.astype(np.uint8),kernel)>0).tobytes()
