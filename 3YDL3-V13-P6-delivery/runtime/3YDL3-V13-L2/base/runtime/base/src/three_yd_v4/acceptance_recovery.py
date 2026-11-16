"""A soft-search proposal never relaxes the baseline's acceptance contract."""
from pathlib import Path
from .controller import SoftEdgeController
from .strict_baseline import SoftEdgeController as StrictBaselineController

class AcceptanceRecoveryController:
    def __init__(self,world,topology,route,*,cache_root,views=None,**kwargs):
        self._published=None;self._published_version=None
        self._args=(world,topology,route)
        self._kwargs=dict(cache_root=cache_root,views=views,**kwargs)
        self._active=SoftEdgeController(*self._args,**self._kwargs)
    def __getattr__(self,name):return getattr(self._active,name)
    def current_trajectory(self):
        dynamic=getattr(self._active,'dynamic',None)
        if dynamic and dynamic.version!=self._published_version:return None
        return None if self._published is None else self._published.copy()
    @property
    def trajectory(self):return self.current_trajectory()
    def _publish(self):
        self._published=self._active.current_trajectory()
        self._published_version=self._active.trajectory_version
    def update_dynamic(self,*args,**kwargs):
        self._published=None;self._published_version=None
        return self._active.update_dynamic(*args,**kwargs)
    def plan_l3(self,query,session,auditor,spec,**kwargs):
        self._published=None;self._published_version=None
        out=self._active.plan_l3(query,session,auditor,spec,**kwargs)
        traffic=out.get('traffic_audit',{})
        nominal=traffic.get('nominal_route_audit',traffic)
        if not out.get('success') or nominal.get('valid',False):
            if out.get('success'):
                out['legacy_contract_success']=True;self._publish()
            return out
        # Do not expose the proposal while the original traffic constraints
        # are violated, even if its collision/vehicle checks passed.
        self._active.trajectory=None;self._active.trajectory_version=None
        rejected={'failure':nominal.get('failure'),'source_contract_valid':traffic.get('valid'),
                  'physical_geometry_valid':out.get('geometry_success'),
                  'attempt_timing':out.get('multires_diagnostics',{}),
                  'proposal_is_never_dispatchable':True}
        self._active.lifecycle.clear()
        options=dict(self._kwargs);options['cache_root']=Path(options['cache_root'])/'strict_fallback'
        self._active=StrictBaselineController(*self._args,**options)
        result=self._active.plan_l3(query,session,auditor,spec,**kwargs)
        result['acceptance_recovery']={'rejected_soft_proposal':rejected,'fallback':'unchanged_V4_strict_planner',
                                      'success':bool(result.get('success'))}
        result['legacy_contract_success']=bool(result.get('success'))
        if result.get('success'):self._publish()
        return result
