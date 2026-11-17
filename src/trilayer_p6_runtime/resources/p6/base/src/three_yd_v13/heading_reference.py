"""Experimental L2 polyline proposal, never an executable turn or L3 change."""
import numpy as np

def controller_class(base,distance):
    class HeadingReference(base):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            pose=np.asarray(self.topology_route.start,float)
            xy=np.asarray(self.reference_plan.world_xy,float)
            direction=np.array([np.cos(pose[2]),np.sin(pose[2])])
            first=xy[1]-xy[0]
            detail={'requested_lead_m':distance,'selected':False,'L2_geometric_proposal_only':True}
            self.timing['p5_heading_reference']=detail
            if getattr(self.topology,'mode',None)!='free_space' or np.dot(first,direction)>0:
                detail['reason']='not_opposed_nonsemantic_reference';return
            lead=pose[:2]+distance*direction
            proposed=np.vstack((xy[0],lead,xy[1:]))
            samples=[]
            for a,b in zip(proposed[:2],proposed[1:3]):
                samples.extend(a+(b-a)*np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(b-a)/.025))+1))[:,None])
            samples=np.asarray(samples);rr,cc=self.world.cells(samples)
            h,w=self.world.hard.shape
            inside=(rr>=0)&(cc>=0)&(rr<h)&(cc<w)
            if not inside.all() or np.asarray(self.world.hard[rr,cc]).any() or not np.asarray(self.global_drivable[rr,cc]).all():
                detail['reason']='proposal_exits_free_current_corridor';return
            self._set_reference(proposed)
            from three_yd_v5.v12_global_l2 import bind_reference_traffic
            from three_yd_v5.v12_regional import accepted_reference
            bind_reference_traffic(self,proposed);accepted_reference(self,proposed)
            detail.update(selected=True,lead_xy=lead.tolist(),reason='forward_lead_before_return')
    return HeadingReference
