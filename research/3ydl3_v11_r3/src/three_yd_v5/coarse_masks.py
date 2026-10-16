"""Share only immutable, cell-for-cell complementary coarse obstacle masks."""
import numpy as np
from .v9_maps import immutable, seal


def pair(hard):
    hard=np.asarray(hard,bool)
    return seal(~hard),seal(hard)


def hard_for(safe,reference_safe,reference_hard):
    # A mutable reference or a replacement hard field is never trusted merely
    # because the safe field has the same identity. Validate the full complement.
    if (safe is reference_safe and type(safe) is np.ndarray and
        type(reference_hard) is np.ndarray and safe.dtype==np.dtype(bool) and
        reference_hard.dtype==np.dtype(bool) and safe.ndim==2 and
        safe.shape==reference_hard.shape and immutable(safe) and immutable(reference_hard)):
        for r in range(0,safe.shape[0],96):
            if np.any(safe[r:r+96]==reference_hard[r:r+96]):break
        else:return reference_hard
    return ~np.asarray(safe,bool)


def false_field(shape):
    return np.broadcast_to(np.frombuffer(bytes(1),bool),shape)
