"""Bind an existing, actually verified complete-task native frame to a batch.

The parent global batch cannot compare a global mask with a native crop. Keep
both bindings and pin the current semantic wrapper only while every source,
frame, mask, publication and real ACK still matches. Ordinary updates continue
through the untouched parent, including mutable-field refresh and readback.
"""
from contextlib import contextmanager
import numpy as np
from arena_evaluation.semantic_map import canonical_hash
from .v9_maps import content_digest
from .v9_r5_session import FullRouteStorageSession
from .v9_r7_options import enabled


def field_binding(value):
    from .r4_tiles import TiledSoft
    if type(value) is TiledSoft:
        return (id(value),value.shape,value.cap,str(value.dtype),
                field_binding(value.drivable),
                tuple((tuple(box),field_binding(v)) for box,v in value.pieces))
    if not isinstance(value,np.ndarray):raise RuntimeError('R7_BATCH_UNSUPPORTED_FIELD')
    return (id(value),value.shape,str(value.dtype),content_digest(value))


def grid_binding(grid):
    return (id(grid),str(grid.policy_hash),str(grid.semantic_map_hash),
            str(grid.preference_field_hash),bool(grid.semantics_enabled),
            field_binding(grid.occupancy_grid),field_binding(grid.expected_master_cost),
            field_binding(grid.internal_cost),field_binding(grid.hard_semantic_mask),
            field_binding(grid.affected_mask),field_binding(grid.soft_cost),
            canonical_hash(grid.diagnostics))


def bookkeeping_binding(value):
    # The parent may replace equal bookkeeping copies/views after each
    # checked update. Their storage identity is not a semantic source.
    return (value.shape,str(value.dtype),content_digest(value))


def equal_rows(left,right):
    return (isinstance(left,np.ndarray) and left.shape==right.shape and left.dtype==right.dtype
            and all(np.array_equal(left[a:a+64],right[a:a+64]) for a in range(0,len(right),64)))


class BoundStorageSession(FullRouteStorageSession):
    def _r7_binding(self):
        m=self.ctx.hospital_map;g=self.global_ctx.hospital_map
        return (self.r5_task_key,self.r5_scope_bounds,self.r5_native_bounds,id(self.ctx),
                (m.height,m.width,float(m.resolution),tuple(m.origin)),
                id(self.global_ctx),(g.height,g.width,float(g.resolution),tuple(g.origin)),
                self.global_ctx.map_sha256,tuple(self.bound_vehicle),
                grid_binding(self._r5_global_grid),grid_binding(self._semantic_costmap),
                self._semantic_publication_sequence,self._last_exact_signature,
                field_binding(self._last_exact_expected_master),
                bookkeeping_binding(self._current_grid),bookkeeping_binding(self._current_allowed_mask))

    def _r7_check(self,mask=None):
        state=self._r7_batch
        if (not self._costmap_state_trusted or self._r5_switching
                or self._r7_binding()!=state['binding']):
            raise RuntimeError('R7_BATCH_MAP_BINDING_CHANGED')
        if mask is not None:
            shape=tuple(np.shape(mask))
            expected=state['global_hash'] if shape==state['global_shape'] else (
                state['native_hash'] if shape==state['native_shape'] else None)
            if expected is None or content_digest(mask)!=expected:
                raise RuntimeError('R7_BATCH_MASK_BINDING_CHANGED')
        if content_digest(state['global_mask'])!=state['global_hash']:
            raise RuntimeError('R7_BATCH_GLOBAL_MASK_CHANGED')

    def _prepare_native(self,mask):
        if getattr(self,'_r7_batch',None) is None:return super()._prepare_native(mask)
        self._r7_check(mask)
        return self._r7_batch['native_mask']

    def set_semantic_costmap(self,costmap):
        if getattr(self,'_r7_batch',None) is not None:
            raise RuntimeError('R7_BATCH_SOURCE_REBIND_FORBIDDEN')
        return super().set_semantic_costmap(costmap)

    def v9_batch(self,mask,ack):
        if not enabled('T') or not self.r5_enabled:return super().v9_batch(mask,ack)
        @contextmanager
        def batch():
            if getattr(self,'_v9_batch',None) is not None or getattr(self,'_r7_batch',None) is not None:
                raise RuntimeError('NESTED_V9_BATCH')
            original_shape=(self.global_ctx.hospital_map.height,self.global_ctx.hospital_map.width)
            if tuple(np.shape(mask))!=original_shape or self.r5_native_bounds is None:
                raise RuntimeError('R7_BATCH_GLOBAL_SCOPE_REQUIRED')
            local=self._semantic_costmap;a,b,c,d=self.r5_native_bounds
            native=mask[a:b,c:d];global_hash=content_digest(mask)
            signature=self._content_signature(local);previous=self._last_exact_ack
            required=('semantic_policy_hash','semantic_expected_grid_hash','semantic_expected_master_hash',
                      'server_costmap_content_hash','semantic_expected_server_content_hash',
                      'costmap_ack_sequence','semantic_publication_sequence')
            if (not self._costmap_state_trusted or self._r5_switching or not previous
                or ack.get('costmap_update_acknowledged') is not True
                or previous.get('costmap_update_acknowledged') is not True
                or ack.get('costmap_ack_mismatch_cells')!=0 or previous.get('costmap_ack_mismatch_cells')!=0
                or ack.get('semantic_exact_stable_observations',0)<2
                or previous.get('semantic_exact_stable_observations',0)<2
                or not ack.get('server_costmap_content_hash')
                or ack.get('server_costmap_content_hash')!=ack.get('semantic_expected_server_content_hash')
                or any(ack.get(k) is None or ack.get(k)!=previous.get(k) for k in required)
                or ack['semantic_publication_sequence']!=self._semantic_publication_sequence
                or signature!=self._last_exact_signature
                or signature!=(str(ack['semantic_policy_hash']),str(ack['semantic_expected_grid_hash']),str(ack['semantic_expected_master_hash']))
                or local.diagnostics.get('r5_source_full_mask_hash')!=global_hash
                or local.diagnostics.get('r5_source_full_grid_hash')!=self._r5_global_grid.expected_grid_hash
                or local.diagnostics.get('r5_source_full_master_hash')!=self._r5_global_grid.expected_master_hash
                or not equal_rows(self._current_allowed_mask,native)
                or not equal_rows(self._current_grid,np.flipud(local.occupancy_grid))
                or not equal_rows(self._last_exact_expected_master,np.flipud(local.expected_master_cost))):
                raise RuntimeError('R7_BATCH_CURRENT_REAL_ACK_REQUIRED')
            state={'binding':self._r7_binding(),'global_mask':mask,'native_mask':native,
                   'global_shape':original_shape,'native_shape':native.shape,
                   'global_hash':global_hash,'native_hash':content_digest(native)}
            self._r7_batch=state
            try:
                # The original parent guard now pins its actual native frame.
                # It retains identity/content/sequence/trust checks and ACK.
                with super(BoundStorageSession,self).v9_batch(native,ack):
                    yield
                    self._r7_check()
            finally:self._r7_batch=None
        return batch()

    def update_local_mask(self,mask,**kwargs):
        active=getattr(self,'_r7_batch',None) is not None
        if active:self._r7_check(mask)
        result=super().update_local_mask(mask,**kwargs)
        if active:
            result.update(r7_global_and_native_batch_binding_verified=True,
                          r7_global_batch_mask_hash=self._r7_batch['global_hash'],
                          r7_native_batch_mask_hash=self._r7_batch['native_hash'])
        return result
