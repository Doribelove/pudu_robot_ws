import time
class R5Recovery:
    def __init__(self,world,topology,cache_root,views,dynamic=None,controller_class=None,**kw):
        self.world=world;self.topology=topology;self.root=cache_root;self.views=views;self.dynamic=dynamic;self.controller_class=controller_class
        self.controller_options={'weight':float(kw.get('weight',2.))}
    def plan(self,query,session,auditor,spec):
        c=r=None;out={};attempts=[];tick=time.monotonic();session.recovery_attempts=attempts
        for route in self.topology.routes(query,limit=2):
            if hasattr(session,'budget'):session.budget.require()
            r=route;c=None;t=time.monotonic();session.active_controller=None;session.active_route=r
            record={'regions':list(r.region_ids),'success':False,'route_signature':r.signature};attempts.append(record)
            try:
                c=self.controller_class(self.world,self.topology,r,cache_root=self.root,views=self.views,dynamic=self.dynamic,**self.controller_options)
                session.active_controller=c
                out=c.plan_l3(query,session,auditor,spec)
            except (ValueError,RuntimeError,TimeoutError) as e:
                if any(x in str(e) for x in ['CORRUPT','BINDING','VERSION']):raise
                out={'success':False,'failure_code':str(e)}
            finally:
                record.update(wall_ms=(time.monotonic()-t)*1000,partial_local_log=getattr(c,'r4_local_log',{}))
            record.update(success=out.get('success',False),failure=out.get('failure_code'),bounded_l3=out.get('bounded_l3',{}),local_l3=out.get('local_l3',{}),constraint_audit=out.get('constraint_audit',{}),independent_audit=out.get('independent_audit',{}),r6=out.get('r6',{}))
            if out.get('success'):break
            if any(x in out.get('failure_code','') for x in ['ACK','BUDGET','CONTENT']):break
            if c:c.lifecycle.clear()
        out=out or {'success':False,'failure_code':'NO_LEGAL_REGION_ROUTE'}
        out['candidate_recovery']={'attempts':attempts,'max_region_routes':2,'wall_ms':(time.monotonic()-tick)*1000,'directed_outline_dependencies':False}
        return c,r,out
