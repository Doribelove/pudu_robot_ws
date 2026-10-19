"""Paced map blocks and bounded exact-content repairs for the frozen planner."""
import hashlib,time
import numpy as np
import yaml
from arena_evaluation.unified_four_backends_smoke import SmacSession
from .stable_session import StableSmacSession
from .semantic_world import json_write

class PacedPublisher:
    def __init__(self, session, publisher, phase):
        self.session=session;self.publisher=publisher;self.phase=phase;self.last_sent=None
    def __getattr__(self,name):return getattr(self.publisher,name)
    def publish(self,message):
        s=self.session;began=time.monotonic();interval=s.tile_interval_s
        if self.last_sent is not None:
            until=self.last_sent+interval
            while time.monotonic()<until:
                self._check_deadline()
                s.client.executor.spin_once(timeout_sec=min(.005,max(0.,until-time.monotonic())))
        self._check_deadline();sent=time.monotonic();self.publisher.publish(message)
        s.tile_events.append({'query':s.current_query_id,'phase':self.phase,'sequence':s._semantic_publication_sequence,
          'bbox':[int(message.x),int(message.y),int(message.width),int(message.height)],
          'bytes':len(message.data),'sent':sent,'gap_s':None if self.last_sent is None else sent-self.last_sent,
          'pacing_wait_s':sent-began,'configured_interval_s':interval})
        self.last_sent=sent
    def _check_deadline(self):
        c=self.session._exact_repair_context
        if c is not None and c.get('repairing') and time.monotonic()>=c['deadline']:
            raise RuntimeError('semantic repair publication deadline exhausted')

class SemanticAckSession(StableSmacSession):
    PUBLICATION_VERSION='3YD-V1-r1-semantic-paced-exact-repair-v1'
    def __init__(self,*args,**kwargs):
        self.tile_interval_s=float(kwargs.pop('tile_interval_s',.015))
        self.enable_semantic_repair=bool(kwargs.pop('enable_semantic_repair',True))
        self.tile_events=[];self.semantic_repair_events=[];self._exact_repair_context=None
        if not 0<=self.tile_interval_s<=.05:raise ValueError('invalid map tile interval')
        super().__init__(*args,**kwargs)
        parameters=yaml.safe_load(self.params_file.read_text())
        self.inflation_radius_m=float(parameters['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']['inflation_radius'])
    def _publish_dirty_roi(self,expected,changed):
        original=self._local_update_publisher
        self._local_update_publisher=PacedPublisher(self,original,'initial')
        try:return super()._publish_dirty_roi(expected,changed)
        finally:self._local_update_publisher=original
    def _wait_for_costmap_ack(self,expected,changed,*,timeout_s=None):
        semantic=self._semantic_costmap
        if semantic is None or not self.enable_semantic_repair:return super()._wait_for_costmap_ack(expected,changed,timeout_s=timeout_s)
        started=time.monotonic();master=np.flipud(np.asarray(semantic.expected_master_cost,np.uint8))
        previous=self._last_exact_expected_master
        dirty=np.ones(master.shape,bool) if previous is None else previous!=master
        affected=np.asarray(changed,bool)|dirty|np.flipud(semantic.hard_semantic_mask)|np.flipud(semantic.soft_cost>0)
        context={'semantic':semantic,'signature':self._content_signature(semantic),'expected':np.asarray(expected,np.int8),
          'master':master,'affected':affected,'sequence':self._semantic_publication_sequence,
          'deadline':started+float(timeout_s or self.costmap_ack_timeout_s),'started':started,
          'repairs':0,'repairing':False,'last_bad':None,'same_observations':0,'last_send':started,'next_probe_at':started+.4,'events':[]}
        self._exact_repair_context=context
        try:
            remaining=context['deadline']-time.monotonic()
            if remaining<=0:raise RuntimeError('semantic ACK preparation deadline exhausted')
            result=super()._wait_for_costmap_ack(expected,changed,timeout_s=remaining)
            # Check the immutable content binding once on success instead of
            # hashing two whole maps on every readback. A policy or source
            # mutation cannot inherit the parent's already-computed evidence.
            if result.get('costmap_update_acknowledged') and (context['semantic'] is not self._semantic_costmap or context['signature']!=self._content_signature(self._semantic_costmap) or context['sequence']!=self._semantic_publication_sequence):
                result.update(costmap_update_acknowledged=False,costmap_ack_status='semantic_binding_changed_during_ack')
                self._costmap_state_trusted=False;self._force_full_next_update=True
                self._last_exact_ack=None;self._last_exact_signature=None;self._last_exact_expected_master=None;self._exact_ack_cache.clear()
            info={'semantic_gap_repair_count':context['repairs'],'semantic_gap_repair_events':context['events'],
                  'semantic_tile_interval_s':self.tile_interval_s}
            result.update(info)
            if self.ack_events:self.ack_events[-1].update(info)
            return result
        finally:
            self._exact_repair_context=None
    def _server_costmap_snapshot(self,deadline):
        server,stamp=super()._server_costmap_snapshot(deadline)
        c=self._exact_repair_context
        if c is None or not self.enable_semantic_repair:return server,stamp
        # A reassertion belongs to the same logical publication. Never let a
        # concurrent policy/sequence change be hidden by repair or ACK.
        if c['semantic'] is not self._semantic_costmap or c['signature'][0]!=str(self._semantic_costmap.policy_hash) or c['sequence']!=self._semantic_publication_sequence:
            raise RuntimeError('semantic publication changed during exact ACK')
        # The normal exact ACK gets first opportunity to complete, without
        # duplicate whole-grid comparisons/hashes on its fast path. Only a
        # still-unresolved publication gets a repair probe after 0.4 s.
        now=time.monotonic()
        if c['repairs']>=2 or now<c['next_probe_at']:return server,stamp
        c['next_probe_at']=now+.05
        mismatch=c['affected'] & (server!=c['master'])
        count=int(np.count_nonzero(mismatch))
        if not count:
            c['last_bad']=None;c['same_observations']=0;return server,stamp
        h=hashlib.sha256(np.packbits(mismatch).tobytes()+np.ascontiguousarray(server[mismatch]).tobytes()).hexdigest()
        c['same_observations']=c['same_observations']+1 if h==c['last_bad'] else 1;c['last_bad']=h
        if c['repairs']>=2 or c['same_observations']<2 or time.monotonic()-c['last_send']<.05:return server,stamp
        # Reassert the source around the affected master cells, including the
        # frozen inflation support. This also restores inflation when a lost
        # tile contained a neighboring lethal source rather than the cell.
        rr,cc=np.nonzero(mismatch);halo=int(np.ceil(self.inflation_radius_m/self.ctx.hospital_map.resolution))+1
        y0=max(0,int(rr.min())-halo);y1=min(server.shape[0],int(rr.max())+halo+1)
        x0=max(0,int(cc.min())-halo);x1=min(server.shape[1],int(cc.max())+halo+1)
        repair=np.zeros(server.shape,bool);repair[y0:y1,x0:x1]=True
        event={'query':self.current_query_id,'sequence':c['sequence'],'mismatch_cells':count,
               'mismatch_bbox':[int(cc.min()),int(rr.min()),int(cc.max()-cc.min()+1),int(rr.max()-rr.min()+1)],
               'repair_bbox':[x0,y0,x1-x0,y1-y0],'same_observations':c['same_observations'],'elapsed_before_s':time.monotonic()-c['started']}
        original=self._local_update_publisher;c['repairing']=True;c['repairs']+=1;begin=time.monotonic()
        self._local_update_publisher=PacedPublisher(self,original,'repair')
        try:
            # Bypass _begin_publication: no new policy, bbox or cache sequence.
            # The unchanged parent ACK still requires fresh exact master
            # content, the same publication sequence and two observations.
            _,info=SmacSession._publish_dirty_roi(self,c['expected'],repair)
            event.update(messages=int(info['roi_message_count']),published_cells=int(info['roi_published_cells']))
        finally:
            self._local_update_publisher=original;c['repairing']=False;c['last_send']=time.monotonic();c['next_probe_at']=c['last_send']+.25;c['same_observations']=0
            event['publication_s']=time.monotonic()-begin;c['events'].append(event);self.semantic_repair_events.append(event)
        return server,stamp
    def close(self):
        try:return super().close()
        finally:
            json_write(self.lifecycle_root/'publication_tiles.json',self.tile_events)
            json_write(self.lifecycle_root/'semantic_repairs.json',self.semantic_repair_events)
