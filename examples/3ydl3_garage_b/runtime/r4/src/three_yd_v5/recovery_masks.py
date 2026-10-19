"""Consume request-owned masks without changing any selected map cell."""
import cv2
import numpy as np


def dilated_boolean(mask, kernel):
    # astype canonicalizes even unusual nonzero bool byte representations.
    # Dilation of 0/1 uint8 stays 0/1; its owned output can be viewed as bool.
    return cv2.dilate(np.asarray(mask).astype(np.uint8), kernel).view(np.bool_)


def initial_masks(hard, selected, all_physical, margin_cells):
    """Consume fresh all_physical; never modify hard or selected storage."""
    margin=dilated_boolean(selected,np.ones((2*margin_cells+1,)*2,np.uint8))
    drivable=np.asarray(all_physical)
    if not (drivable.dtype==np.bool_ and drivable.flags.owndata and drivable.flags.writeable):
        drivable=np.array(drivable,dtype=bool,copy=True)
    np.bitwise_and(drivable,margin,out=drivable)
    physical=np.logical_not(hard)
    np.bitwise_and(physical,drivable,out=physical)
    return drivable,physical
