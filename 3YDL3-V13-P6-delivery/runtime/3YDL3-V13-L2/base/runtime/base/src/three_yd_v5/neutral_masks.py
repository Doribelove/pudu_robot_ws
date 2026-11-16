"""Share only proven identical neutral masks, within the actual request."""
import numpy as np
from .v10_storage import enabled,retained_mask


def physical_masks(hard,subregions,rows=96):
    drivable=subregions==1
    # The parent's physical field is ~hard & drivable. It equals drivable
    # exactly iff no drivable cell is hard. Verify all cells, never samples.
    if (enabled() and type(hard) is np.ndarray and hard.ndim==2
        and hard.dtype==np.dtype(bool) and type(drivable) is np.ndarray
        and drivable.shape==hard.shape and drivable.dtype==np.dtype(bool)):
        clear=True
        for a in range(0,len(hard),rows):
            if np.any(hard[a:a+rows]&drivable[a:a+rows]):clear=False;break
        if clear:
            shared=retained_mask(drivable)
            return shared,shared
    return drivable,retained_mask(~hard&drivable)
