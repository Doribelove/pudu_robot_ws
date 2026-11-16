"""Identical measurement hook for baseline and candidate, no solver changes."""
from contextvars import ContextVar
_rows=ContextVar('p5_search_rows',default=None)

def install():
    from three_yd_v4.r5_controller import BoundedReferenceDStar
    original=BoundedReferenceDStar._result
    def measured(self,**kw):
        out=original(self,**kw)
        rows=_rows.get()
        if rows is not None:
            ds=kw['stats'];ast=kw.get('fallback')
            rows.append({'backend':kw['backend'],'success':out.success,
                'failure_code':out.failure_code,'geometry_hash':self.geometry.binding.digest,
                'states':self.geometry.state_count,'dstar_expanded':int(ds.expanded_nodes),
                'astar_expanded':0 if ast is None else int(ast.expanded_nodes),
                'dstar_stats':dict(vars(ds)),
                'astar_queue_pops':getattr(self.state,'v13_queue_pops',None) if ast is not None else None,
                'state_bytes':out.diagnostics['state_memory_bytes'],
                'response_ms':out.response_ms,'reused':kw.get('reused',False)})
        return out
    BoundedReferenceDStar._result=measured
    from three_yd_v5 import v12_global_l2
    grid_original=v12_global_l2.grid_search
    def measured_grid(*args,**kwargs):
        path,detail=grid_original(*args,**kwargs)
        rows=_rows.get()
        if rows is not None:
            rows.append({'backend':'full_map_L2_grid_Astar','success':path is not None,
                'failure_code':detail['reason'],'geometry_hash':None,'states':None,
                'dstar_expanded':0,'astar_expanded':detail.get('expanded',0),
                'state_bytes':None,'reused':False})
        return path,detail
    v12_global_l2.grid_search=measured_grid

def start():
    rows=[];token=_rows.set(rows)
    return token,rows

def finish(token,rows):
    _rows.reset(token)
    return {'searches':rows,'search_count':len(rows),
        'expanded_total':sum(x['dstar_expanded']+x['astar_expanded'] for x in rows),
        'dstar_expanded_total':sum(x['dstar_expanded'] for x in rows),
        'astar_expanded_total':sum(x['astar_expanded'] for x in rows),
        'peak_single_L2_state_bytes':max([x['state_bytes'] for x in rows if x['state_bytes'] is not None],default=0),
        'scope':'all returned L2 planner searches including failed/reconstructed domains; repeated expansions counted; excludes L1 and connectivity-preflight visits',
        'distinct_nodes_across_domains':None}
