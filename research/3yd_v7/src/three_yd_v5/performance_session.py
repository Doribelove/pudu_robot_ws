"""Control process memory separately from the untouched native planner."""
import os
from three_yd_v4.raw_diagnostics import RawDiagnosticSession

class PerformanceSession(RawDiagnosticSession):
    def start(self):
        # glibc reads these settings when a process starts. The Python worker
        # keeps its lean allocator; optionally leave the native ROS allocator
        # at baseline settings to avoid repeated small-allocation overhead.
        split=(os.environ.get('THREEYD_R9_OPTIMIZE')=='1' and
               os.environ.get('THREEYD_R9_NATIVE_ALLOCATOR','inherit')=='default')
        if not split:return super().start()
        names=('MALLOC_ARENA_MAX','MALLOC_TRIM_THRESHOLD_')
        saved={k:os.environ.pop(k,None) for k in names}
        try:return super().start()
        finally:
            for k,value in saved.items():
                if value is not None:os.environ[k]=value

    def _publish_dirty_roi(self,expected,changed):
        from .r10_options import enabled
        if not enabled('A'):return super()._publish_dirty_roi(expected,changed)
        import time
        from array import array
        import numpy as np
        from arena_3d_v1.semantic_ack_session import PacedPublisher
        from .r10_tiles import rectangles
        if self._current_grid is None:raise RuntimeError('ROI update has no initialized grid')
        from .r11_options import mode as r11_mode
        if r11_mode() != 'control':
            discovery=time.monotonic()+self.costmap_ack_timeout_s
            while self._local_update_publisher.get_subscription_count()==0:
                if time.monotonic()>=discovery:
                    raise RuntimeError('costmap update subscriber discovery timed out')
                self.client.executor.spin_once(timeout_sec=.005)
            if self._semantic_costmap is not None and not np.any(changed):
                changed=np.asarray(changed,dtype=bool).copy();changed[0,0]=True
            self._begin_publication(np.asarray(changed,dtype=bool),full=False)
        started=time.monotonic()
        limit=max(1024,int(getattr(self,'roi_max_payload_bytes',128000)))
        boxes,detail=rectangles(changed,limit)
        partition_ms=(time.monotonic()-started)*1000
        original=self._local_update_publisher
        publisher=PacedPublisher(self,original,'initial-r10-sparse')
        serialization_ms=publication_ms=0.
        applied=self._current_grid.copy()
        for y0,y1,x0,x1 in boxes:
            tick=time.monotonic()
            patch=np.ascontiguousarray(expected[y0:y1,x0:x1],dtype=np.int8)
            message=self.OccupancyGridUpdate()
            message.header.frame_id='map'
            message.header.stamp=self.client.node.get_clock().now().to_msg()
            message.x=x0;message.y=y0;message.width=x1-x0;message.height=y1-y0
            message.data=array('b',patch.tobytes())
            serialization_ms+=(time.monotonic()-tick)*1000
            tick=time.monotonic();publisher.publish(message)
            publication_ms+=(time.monotonic()-tick)*1000
            pacing=max(0.,float(getattr(self,'roi_publish_pacing_s',.001)))
            if pacing:self.client.executor.spin_once(timeout_sec=pacing)
            applied[y0:y1,x0:x1]=patch
        bbox=[0,0,0,0]
        if boxes:
            x0=min(b[2] for b in boxes);y0=min(b[0] for b in boxes)
            bbox=[x0,y0,max(b[3] for b in boxes)-x0,max(b[1] for b in boxes)-y0]
        self.r10_tile_events=getattr(self,'r10_tile_events',[])
        self.r10_tile_events.append({'query':self.current_query_id,'partition_ms':partition_ms,**detail})
        return applied,{'local_map_serialization_ms':serialization_ms+partition_ms,
            'local_map_publication_ms':publication_ms,'costmap_settle_ms':0.,
            'roi_bbox':bbox,'roi_changed_cells':detail['changed_cells'],
            'roi_published_cells':detail['published_cells'],'roi_message_count':len(boxes),
            'roi_max_message_bytes':max(((b-a)*(d-c) for a,b,c,d in boxes),default=0),
            'roi_publish_pacing_ms':float(getattr(self,'roi_publish_pacing_s',.001))*1000}

    def close(self):
        try:return super().close()
        finally:
            from arena_3d_v1.semantic_world import json_write
            json_write(self.lifecycle_root/'r10_sparse_tiles.json',getattr(self,'r10_tile_events',[]))
