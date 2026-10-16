"""Full-cell exact ACK with bounded scratch arrays; transport may be sparse."""
import hashlib,time
import numpy as np

def grid_digest(value,rows=64):
    a=np.asarray(value)
    h=hashlib.sha256()
    if a.flags.c_contiguous:h.update(memoryview(a))
    else:
        for y in range(0,a.shape[0],rows):h.update(np.ascontiguousarray(a[y:y+rows]))
    return h.hexdigest()

def exact_compare(semantic,expected,changed,previous,server,rows=64):
    """Same hard/soft/stale/ordinary classes as ExactSemanticSmacSessionR2."""
    height,width=expected.shape;master=np.flipud(semantic.expected_master_cost)
    hard_view=np.flipud(semantic.hard_semantic_mask)
    counts={k:0 for k in ('affected','hard','soft','stale','ordinary','hard_bad','soft_bad','stale_bad','ordinary_bad','mismatch')}
    affected_hash=hashlib.sha256();full_hash=hashlib.sha256();bbox=[width,height,0,0]
    for y in range(0,height,rows):
        end=min(height,y+rows);sl=slice(y,end)
        soft_source=np.asarray(semantic.soft_cost[height-end:height-y,:])[::-1]>0
        dirty=np.ones((end-y,width),bool) if previous is None else previous[sl]!=master[sl]
        affected=changed[sl]|dirty|hard_view[sl]|soft_source
        hard=affected&(hard_view[sl]|(expected[sl]==100));soft=affected&soft_source&~hard
        ordinary=affected&~hard&~soft;bad=server[sl]!=master[sl]
        for key,mask in [('affected',affected),('hard',hard),('soft',soft),('stale',dirty),('ordinary',ordinary)]:
            counts[key]+=int(np.count_nonzero(mask))
            if key!='affected':counts[key+'_bad']+=int(np.count_nonzero(mask&bad))
        counts['mismatch']+=int(np.count_nonzero(affected&bad))
        affected_hash.update(np.ascontiguousarray(server[sl][affected]));full_hash.update(np.ascontiguousarray(server[sl]))
        if np.any(dirty):
            yy=np.flatnonzero(dirty.any(axis=1));xx=np.flatnonzero(dirty.any(axis=0))
            bbox=[min(bbox[0],int(xx[0])),min(bbox[1],y+int(yy[0])),max(bbox[2],int(xx[-1])+1),max(bbox[3],y+int(yy[-1])+1)]
    bbox=[bbox[0],bbox[1],bbox[2]-bbox[0],bbox[3]-bbox[1]] if counts['stale'] else [0,0,0,0]
    return {**counts,'affected_hash':affected_hash.hexdigest(),'full_hash':full_hash.hexdigest(),'dirty_bbox':bbox}

def wait_exact(session,expected,changed,timeout_s=None):
    semantic=session._semantic_costmap;started=time.monotonic();deadline=started+float(timeout_s or session.costmap_ack_timeout_s)
    from .v9_maps import flow_digest,flipped_digest
    signature=session._content_signature(semantic);sequence=session._semantic_publication_sequence
    from .tail_digest import publication_source_digest
    roi=tuple(session._active_publication_bbox);source_hash=publication_source_digest(semantic.occupancy_grid,expected);attempts=stable=0
    # A previous master is immutable while this single-threaded update runs.
    previous=session._last_exact_expected_master
    from .firstplan import enabled
    comparison=None;classification_started=time.monotonic()
    if enabled("B"):
        from .first_ack import ComparisonPlan
        comparison=ComparisonPlan(semantic,np.asarray(expected,np.int8),np.asarray(changed,bool),previous)
    classification_ms=(time.monotonic()-classification_started)*1000
    while time.monotonic()<deadline:
        attempts+=1;session._r6_fast_readback=True
        try:server,stamp=session._server_costmap_snapshot(deadline)
        except RuntimeError:
            # The inherited reader/repair uses only the remainder of this deadline.
            return None,max(0.,deadline-time.monotonic())
        finally:session._r6_fast_readback=False
        detail=(comparison.compare(server) if comparison is not None else
                exact_compare(semantic,np.asarray(expected,np.int8),np.asarray(changed,bool),previous,server))
        fresh=session._active_publication_baseline_timestamp_ns<0 or stamp>session._active_publication_baseline_timestamp_ns
        unchanged=semantic is session._semantic_costmap and sequence==session._semantic_publication_sequence
        if not unchanged:raise RuntimeError('semantic publication changed during ACK')
        session.r6_flow_events=getattr(session,'r6_flow_events',[])
        session.r6_flow_events.append({'query':session.current_query_id,'attempt':attempts,
            'elapsed_s':time.monotonic()-started,'mismatch_cells':detail['mismatch'],'fresh':fresh,
            'publication':sequence})
        if detail['mismatch'] or not fresh:
            stable=0
            # A first read can land between StaticLayer and InflationLayer.
            # Give normal update processing a bounded opportunity to settle;
            # persistent gaps still enter the existing full-content repair.
            if time.monotonic()-started>=.6:return None,max(0.,deadline-time.monotonic())
            session.client.executor.spin_once(timeout_sec=min(.01,max(0.,deadline-time.monotonic())))
            continue
        stable+=1
        if stable<max(1,session.exact_stable_observations):
            session.client.executor.spin_once(timeout_sec=min(.01,max(0.,deadline-time.monotonic())));continue
        if session._content_signature(semantic)!=signature:raise RuntimeError('semantic content changed during ACK')
        session._last_server_update_time_ns=stamp;session._costmap_ack_sequence+=1;session._semantic_roi_sequence+=1
        result={'costmap_update_acknowledged':True,'costmap_ack_status':'exact_effective_master_content_verified',
            'costmap_ack_wait_ms':(time.monotonic()-started)*1000,'costmap_ack_attempts':attempts,
            'costmap_ack_checked_cells':detail['affected'],'costmap_ack_mismatch_cells':0,
            'costmap_ack_hard_checked_cells':detail['hard'],'costmap_ack_hard_mismatch_cells':0,
            'costmap_ack_soft_checked_cells':detail['soft'],'costmap_ack_soft_mismatch_cells':0,
            'costmap_ack_soft_exact_mismatch_cells':0,'costmap_ack_soft_exact_mismatch_ratio':0.,
            'costmap_ack_stale_checked_cells':detail['stale'],'costmap_ack_stale_roi_cells':0,
            'costmap_ack_hash_mismatch':0,'costmap_ack_sequence_mismatch':0,'costmap_ack_semantics':'exact_effective_master',
            'costmap_ack_sequence':session._costmap_ack_sequence,'semantic_roi_sequence':session._semantic_roi_sequence,
            'semantic_publication_sequence':sequence,'semantic_publication_version':session.PUBLICATION_VERSION,
            'semantic_policy_hash':str(semantic.policy_hash),'semantic_source_grid_hash':source_hash,
            'semantic_expected_grid_hash':signature[1],'semantic_expected_master_hash':signature[2],
            'semantic_expected_server_content_hash':flipped_digest(semantic.expected_master_cost),
            'semantic_ack_roi_bbox':list(roi),'semantic_effective_dirty_bbox':detail['dirty_bbox'],
            'semantic_exact_stable_observations':stable,'server_costmap_update_time_ns':stamp,
            'server_costmap_content_hash':detail['full_hash'],'server_affected_content_hash':detail['affected_hash'],
            'r11_ack_classification_ms':classification_ms,'firstplan_classification_once':comparison is not None,'r6_streaming_ack':True,'r6_ack_scratch_rows':64,'r6_exact_comparison_coverage':'all_original_affected_cells'}
        key=session._exact_cache_key(source_grid_hash=source_hash,expected_master_hash=signature[2],policy_hash=signature[0],publication_sequence=sequence,roi_bbox=roi,server_content_hash=detail['full_hash'])
        result['semantic_exact_ack_key_hash']=hashlib.sha256(repr(key).encode()).hexdigest()
        session._exact_ack_cache={key:dict(result)}
        previous=np.flipud(semantic.expected_master_cost).view();previous.flags.writeable=False
        session._last_exact_expected_master=previous;session._last_exact_signature=signature;session._last_exact_ack=dict(result)
        session._semantic_ack_cache.clear();session._append_ack_trace('semantic_exact_ack_successes.jsonl',result);session.ack_events.append(dict(result))
        return result,0.
    return None,0.
