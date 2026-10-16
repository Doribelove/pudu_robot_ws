"""Read-only stage evidence around the frozen r1 session."""
import math
import time
from pathlib import Path
import numpy as np
from .semantic_pipeline import ReferenceSmacSession
from .semantic_world import json_write, digest


class DiagnosticSession(ReferenceSmacSession):
    def __init__(self, ctx, output, **kwargs):
        self.evidence_root = Path(output) / 'stage_evidence'
        self.evidence_root.mkdir(parents=True, exist_ok=True)
        self.events = []
        self.raw_messages = []
        self.call_index = 0
        self.raw_subscription = None
        super().__init__(ctx, output, **kwargs)

    def event(self, kind, **values):
        self.events.append({'kind': kind, 'monotonic': time.monotonic(), **values})
        json_write(self.evidence_root / 'events.json', self.events)

    def _server_costmap_snapshot(self, deadline):
        started = time.monotonic()
        try:
            grid, stamp = super()._server_costmap_snapshot(deadline)
        except Exception as error:
            self.event('readback_error', error=str(error), elapsed_s=time.monotonic()-started)
            raise
        self.event('readback', shape=list(grid.shape), stamp=stamp,
                   elapsed_s=time.monotonic()-started, content_hash=digest(grid))
        return grid, stamp

    def _wait_for_costmap_ack(self, expected, changed, **kwargs):
        result = super()._wait_for_costmap_ack(expected, changed, **kwargs)
        self.event('ack', semantic=self._semantic_costmap is not None, result=result)
        return result

    def start(self):
        self.event('start')
        try:
            super().start()
        except Exception as error:
            self.event('startup_failed', error=str(error))
            raise
        from nav_msgs.msg import Path as PathMessage
        def receive(message):
            poses = []
            for entry in message.poses:
                p, q = entry.pose.position, entry.pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                poses.append([p.x, p.y, yaw])
            self.raw_messages.append(np.asarray(poses, dtype=float))
        self.raw_subscription = self.client.node.create_subscription(PathMessage, '/unsmoothed_plan', receive, 10)
        deadline = time.monotonic()+3
        while self.client.node.count_publishers('/unsmoothed_plan') == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError('RAW_PATH_DISCOVERY_FAILED')
            self.client.executor.spin_once(timeout_sec=.01)
        self.event('started', startup_ms=self.stack_startup_time_ms)

    def plan(self, query, spec, **kwargs):
        self.call_index += 1
        case = self.evidence_root / f'call_{self.call_index:03d}'
        case.mkdir()
        self.raw_messages.clear()
        mask = kwargs.get('allowed_mask')
        if mask is not None:
            np.save(case/'corridor.npy', mask)
        semantic = self._semantic_costmap
        if semantic is not None:
            np.savez_compressed(case/'costmap.npz', occupancy=semantic.occupancy_grid,
                                master=semantic.expected_master_cost)
        result = super().plan(query, spec, **kwargs)
        # Drain already queued callbacks; never accept a previous call's raw path.
        for _ in range(8):
            self.client.executor.spin_once(timeout_sec=0.)
        for i, raw in enumerate(self.raw_messages):
            np.save(case/f'raw_{i}.npy', raw)
        if result.points:
            np.save(case/'final.npy', [[v['x'],v['y'],v['yaw']] for v in result.points])
        json_write(case/'result.json', {'query':query.__dict__, 'planner_success':result.planner_success,
                   'failure_code':result.failure_code, 'raw_messages':len(self.raw_messages),
                   'diagnostics':result.diagnostics})
        self.event('plan', call_index=self.call_index, success=result.planner_success,
                   failure=result.failure_code, raw_messages=len(self.raw_messages))
        return result

    def close(self):
        try:
            if self.raw_subscription is not None and self.client is not None:
                self.client.node.destroy_subscription(self.raw_subscription)
                self.raw_subscription = None
            super().close()
        finally:
            self.event('closed')
