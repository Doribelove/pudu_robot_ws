"""Warm the existing verified delta stream during the real startup reads.

The original semantic repair wrapper stays in the MRO and runs unchanged.
Non-fast readers receive immutable snapshots, matching their parent contract;
all delta observations are real server reads, never cached certificates.
"""
import numpy as np
from arena_3d_v1.stable_session import StableSmacSession
from .r6_session import MapFlowMixin
from .performance_session import PerformanceSession


class StartupDeltaReader(StableSmacSession):
    def _server_costmap_snapshot(self,deadline):
        from .v9_r3_options import enabled
        from .r6_options import enabled as r6_enabled
        if not enabled() or not r6_enabled('B'):return super()._server_costmap_snapshot(deadline)
        saved=getattr(self,'_r6_fast_readback',False)
        self._r6_fast_readback=True
        try:data,stamp=MapFlowMixin._server_costmap_snapshot(self,deadline)
        finally:self._r6_fast_readback=saved
        # The delta decoder owns a mutable reconstruction buffer. Callers of
        # the original non-fast reader may retain their immutable snapshot.
        snapshot=np.frombuffer(data.tobytes(),dtype=data.dtype).reshape(data.shape)
        self.readbacks[-1].update(r3_nonfast_delta=True,r3_immutable_snapshot=True)
        return snapshot,stamp


class StartupDeltaPerformanceSession(PerformanceSession,StartupDeltaReader):
    pass
