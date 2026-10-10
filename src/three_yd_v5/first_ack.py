"""Classify one publication once; independently compare each actual readback.

Only the synchronous wait owns this plan. It is dropped on return/failure;
there is no reuse across publications, content changes, or calls.
"""
import hashlib
import numpy as np
from .r6_ack import grid_digest
from .firstcost import enabled as firstcost_enabled,soft_positive_rows,pack_flags


class ComparisonPlan:
    def __init__(self,semantic,expected,changed,previous,rows=64):
        self.master=np.flipud(semantic.expected_master_cost)
        hard_view=np.flipud(semantic.hard_semantic_mask)
        height,width=expected.shape
        self.flags=np.empty(expected.shape,np.uint8)
        self.counts={k:0 for k in ('affected','hard','soft','stale','ordinary')}
        bbox=[width,height,0,0];affected_hash=hashlib.sha256()
        for y in range(0,height,rows):
            end=min(height,y+rows);sl=slice(y,end)
            soft_source=(soft_positive_rows(semantic.soft_cost,height-end,height-y)[::-1] if firstcost_enabled('B') else
                         np.asarray(semantic.soft_cost[height-end:height-y,:])[::-1]>0)
            dirty=np.ones((end-y,width),bool) if previous is None else previous[sl]!=self.master[sl]
            affected=changed[sl]|dirty|hard_view[sl]|soft_source
            hard=affected&(hard_view[sl]|(expected[sl]==100));soft=affected&soft_source&~hard
            ordinary=affected&~hard&~soft
            f=self.flags[sl];f[:]=0
            if firstcost_enabled('B'):pack_flags(affected,hard,soft,dirty,ordinary,f)
            for bit,name,mask in [(1,'affected',affected),(2,'hard',hard),(4,'soft',soft),(8,'stale',dirty),(16,'ordinary',ordinary)]:
                if not firstcost_enabled('B'):f[mask]|=bit
                self.counts[name]+=int(np.count_nonzero(mask))
            affected_hash.update(np.ascontiguousarray(self.master[sl][affected]))
            if np.any(dirty):
                yy=np.flatnonzero(dirty.any(axis=1));xx=np.flatnonzero(dirty.any(axis=0))
                bbox=[min(bbox[0],int(xx[0])),min(bbox[1],y+int(yy[0])),max(bbox[2],int(xx[-1])+1),max(bbox[3],y+int(yy[-1])+1)]
        self.dirty_bbox=[bbox[0],bbox[1],bbox[2]-bbox[0],bbox[3]-bbox[1]] if self.counts['stale'] else [0,0,0,0]
        self.expected_affected_hash=affected_hash.hexdigest()
        self.expected_full_hash=grid_digest(self.master)

    def compare(self,server,rows=64):
        if server.shape!=self.master.shape:raise ValueError('ACK_DIMENSIONS_MISMATCH')
        # Compare ALL actual cells before using expected classification counts.
        # The actual server digest is still computed, not taken from metadata.
        if np.array_equal(server,self.master):
            return {**self.counts,'hard_bad':0,'soft_bad':0,'stale_bad':0,
                    'ordinary_bad':0,'mismatch':0,'dirty_bbox':self.dirty_bbox,
                    'full_hash':grid_digest(server),'affected_hash':self.expected_affected_hash}
        result={**self.counts,'hard_bad':0,'soft_bad':0,'stale_bad':0,'ordinary_bad':0,'mismatch':0}
        ah=hashlib.sha256();fh=hashlib.sha256()
        for y in range(0,len(server),rows):
            sl=slice(y,y+rows);f=self.flags[sl];bad=server[sl]!=self.master[sl]
            for bit,name in [(1,'mismatch'),(2,'hard_bad'),(4,'soft_bad'),(8,'stale_bad'),(16,'ordinary_bad')]:
                result[name]+=int(np.count_nonzero(bad&((f&bit)!=0)))
            ah.update(np.ascontiguousarray(server[sl][(f&1)!=0]));fh.update(np.ascontiguousarray(server[sl]))
        return {**result,'dirty_bbox':self.dirty_bbox,'full_hash':fh.hexdigest(),'affected_hash':ah.hexdigest()}
