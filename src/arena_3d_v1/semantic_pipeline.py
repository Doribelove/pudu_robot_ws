import time, os
import numpy as np
from arena_evaluation.semantic_smac_session_r2 import ExactSemanticSmacSessionR2
from .semantic_world import WORK

class ReferenceSmacSession(ExactSemanticSmacSessionR2):
    PUBLICATION_VERSION = '3D-V1-r1-semantic-reference-adapter-v3'
    bound_vehicle = (0.265, 0.225, 0.4)

    def __init__(self, *args, **kwargs):
        profile = str(WORK / 'config/fastdds_large_map.xml')
        os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', profile)
        if os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] == profile:
            os.environ['ROS_LOCALHOST_ONLY'] = '0'
        kwargs.setdefault('force_full_on_semantic_signature_change', False)
        super().__init__(*args, **kwargs)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self._map_update_qos = QoSProfile(depth=128, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.roi_publish_pacing_s = 0.015

    def update_local_mask(self, allowed_mask, **kwargs):
        semantic = self._semantic_costmap
        changed_key = semantic is not None and self._content_signature(semantic) != self._last_exact_signature
        reuse = self.enable_mask_reuse_noop
        if changed_key:
            self.enable_mask_reuse_noop = False
        try:
            return super().update_local_mask(allowed_mask, **kwargs)
        finally:
            self.enable_mask_reuse_noop = reuse

    def _publish_dirty_roi(self, expected, changed):
        discovery = time.monotonic() + self.costmap_ack_timeout_s
        while self._local_update_publisher.get_subscription_count() == 0:
            if time.monotonic() >= discovery:
                raise RuntimeError('costmap update subscriber discovery timed out')
            self.client.executor.spin_once(timeout_sec=0.005)
        if self._semantic_costmap is not None and (not np.any(changed)):
            changed = np.asarray(changed, dtype=bool).copy()
            changed[0, 0] = True
        return super()._publish_dirty_roi(expected, changed)

    def _publish_full_grid(self, values, *, clear_costmap=True):
        clear_ms = self._clear_global_costmap() if clear_costmap else 0.0
        old = self._current_grid
        if old is None:
            self._current_grid = np.zeros_like(values)
        (_, timing) = self._publish_dirty_roi(values, np.ones(np.asarray(values).shape, dtype=bool))
        if old is not None:
            self._current_grid = old
        self._last_publish_timing = timing
        return clear_ms
