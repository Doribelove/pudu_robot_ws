"""Opt-in no-copy bookkeeping, streaming ACK and full-snapshot delta transport."""
import time
import numpy as np
from .r6_options import enabled
from .r6_ack import wait_exact
from .v9_maps import flow_digest as grid_digest
from .r6_delta import DeltaDecoder

class MapFlowMixin:
    @staticmethod
    def _content_signature(semantic):
        if not enabled('A'):
            return str(semantic.policy_hash),str(semantic.expected_grid_hash),str(semantic.expected_master_hash)
        return str(semantic.policy_hash),grid_digest(semantic.occupancy_grid),grid_digest(semantic.expected_master_cost)
    def _current_source_grid_hash(self):
        if not enabled('A'):return super()._current_source_grid_hash()
        from .v9_maps import flipped_digest
        return '' if self._semantic_costmap is None else flipped_digest(self._semantic_costmap.occupancy_grid)
    def _grid_for_mask(self,allowed_mask):
        semantic=self._semantic_costmap
        if not enabled('A') or semantic is None:return super()._grid_for_mask(allowed_mask)
        mask=np.asarray(allowed_mask,bool);values=semantic.occupancy_grid
        # Alias only when the already-composed map implements this exact mask.
        # A smaller recovery mask still takes the original copy-and-close path.
        if mask.shape!=values.shape:raise ValueError('mask shape mismatch')
        from .v9_options import enabled as v9_enabled
        if (v9_enabled('A') and getattr(semantic,'diagnostics',{}).get('v9_immutable_composed_map')
            and getattr(semantic,'diagnostics',{}).get('search_bound_hash')==grid_digest(mask)):
            view=np.flipud(values).view();view.flags.writeable=False
            return mask,view
        for y in range(0,len(mask),64):
            if np.any((~mask[y:y+64])&(values[y:y+64]!=100)):
                return super()._grid_for_mask(mask)
        # Published arrays become immutable. The committed bookkeeping views
        # must never observe a later in-place change to a cached costmap.
        values.flags.writeable=False
        semantic.expected_master_cost.flags.writeable=False
        view=np.flipud(values).view();view.flags.writeable=False
        return mask,view
    def _wait_for_costmap_ack(self,expected,changed,*,timeout_s=None):
        if not enabled('A') or self._semantic_costmap is None:
            return super()._wait_for_costmap_ack(expected,changed,timeout_s=timeout_s)
        started=time.monotonic()
        try:
            result,remaining=wait_exact(self,expected,changed,timeout_s)
            if result is not None:return result
            if remaining<=0:raise RuntimeError('streaming ACK deadline exhausted')
            # The unchanged inherited full-readback repair is a bounded recovery,
            # not a second ACK budget. It still observes the actual server map.
            result=super()._wait_for_costmap_ack(expected,changed,timeout_s=remaining)
            result['r6_full_ack_recovery']=True
            result['costmap_ack_wait_ms']=(time.monotonic()-started)*1000
            return result
        except Exception:
            self._costmap_state_trusted=False;self._force_full_next_update=True
            self._last_exact_ack=None;self._last_exact_signature=None;self._exact_ack_cache.clear()
            raise
    def close(self):
        try:return super().close()
        finally:
            from arena_3d_v1.semantic_world import json_write
            json_write(self.lifecycle_root/'r6_ack_flow.json',getattr(self,'r6_flow_events',[]))
    def _server_costmap_snapshot(self,deadline):
        if not enabled('B') or not getattr(self,'_r6_fast_readback',False):
            return super()._server_costmap_snapshot(deadline)
        decoder=getattr(self,'_r6_decoder',None)
        if decoder is None:
            decoder=self._r6_decoder=DeltaDecoder((self.ctx.hospital_map.height,self.ctx.hospital_map.width))
        started=time.monotonic();attempt_deadline=min(deadline,started+.75);future=None
        record={'started':started,'attempt_deadline':attempt_deadline,'overall_deadline':deadline,'r6_delta_transport':True}
        try:
            if not self._get_costmap_client.wait_for_service(timeout_sec=min(.2,max(0.,attempt_deadline-started))):raise RuntimeError('delta readback service unavailable')
            request=self.GetCostmap.Request();request.specs.layer=decoder.request_layer()
            future=self._get_costmap_client.call_async(request)
            while not future.done() and time.monotonic()<attempt_deadline:
                self.client.executor.spin_once(timeout_sec=min(.01,max(0.,attempt_deadline-time.monotonic())))
            if not future.done():raise RuntimeError('delta readback attempt timed out')
            msg=future.result().map;meta=msg.metadata;m=self.ctx.hospital_map
            if (meta.size_y,meta.size_x)!=decoder.shape or msg.header.frame_id!='map':raise RuntimeError('delta map dimensions/frame mismatch')
            if abs(meta.resolution-m.resolution)>1e-7 or abs(meta.origin.position.x-m.origin[0])>1e-7 or abs(meta.origin.position.y-m.origin[1])>1e-7:raise RuntimeError('delta map geometry mismatch')
            if abs(meta.origin.orientation.w-1.)>1e-7 or any(abs(getattr(meta.origin.orientation,k))>1e-7 for k in ('x','y','z')):raise RuntimeError('delta map orientation mismatch')
            data,info=decoder.decode(msg.data,meta.layer);stamp=meta.update_time.sec*1000000000+meta.update_time.nanosec
            record.update(info,success=True,stamp_ns=stamp,bytes=len(msg.data));return data,stamp
        except Exception as error:
            decoder.invalidate();record.update(success=False,error=str(error))
            raise RuntimeError(str(error)) from error
        finally:
            if future is not None and not future.done():
                self._get_costmap_client.remove_pending_request(future);future.cancel();record['pending_request_removed']=True
            record['elapsed_s']=time.monotonic()-started;self.readbacks.append(record)
