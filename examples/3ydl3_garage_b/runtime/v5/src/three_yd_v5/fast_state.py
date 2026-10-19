"""Precomputed static edge weights; unchanged XY D* queue and cost semantics."""
import math
import numpy as np
from arena_3d_v1.l2_state_lifecycle import OFFSETS, OFFSET_INDEX, NO_NEIGHBOR_DELTA
from three_yd_v4.r5_controller import EdgeState

class PrecomputedEdgeState(EdgeState):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self._rebuild_costs()
        self._refresh_heuristic()

    def _rebuild_costs(self):
        n=self.geometry.state_count
        costs=np.full((n,len(OFFSETS)),math.inf,np.float64)
        delta=np.asarray(self.geometry.neighbor_deltas).reshape(n,len(OFFSETS))
        ids=np.arange(n)
        for j,(dr,dc) in enumerate(OFFSETS):
            valid=delta[:,j]!=NO_NEIGHBOR_DELTA
            a=ids[valid];b=a+delta[valid,j]
            tolerance=math.sin(math.radians(5))*math.hypot(dr,dc)
            legal=((dc*self.xy_direction[a,0]-dr*self.xy_direction[a,1])>=-tolerance-1e-6)&((dc*self.xy_direction[b,0]-dr*self.xy_direction[b,1])>=-tolerance-1e-6)
            if hasattr(self.geometry,'stage'):legal&=self.geometry.stage[b]>=self.geometry.stage[a]
            if dr and dc:
                legal&=(delta[a,OFFSET_INDEX[(0,dc)]]!=NO_NEIGHBOR_DELTA)&(delta[a,OFFSET_INDEX[(dr,0)]]!=NO_NEIGHBOR_DELTA)
            a=a[legal];b=b[legal]
            costs[a,j]=(math.sqrt(2.) if dr and dc else 1.)*(1+self.weight*(self.potential[a]+self.potential[b])*.5)
        self.edge_weights=costs
        self._cost_view=memoryview(costs.reshape(-1))
        self._has_dynamic_blocks=bool(np.any(self.blocked))

    def _edge_cost(self,a,b):
        if self._has_dynamic_blocks or hasattr(self,'approach_strength'):
            return super()._edge_cost(a,b)
        dr=self._rows[b]-self._rows[a];dc=self._cols[b]-self._cols[a]
        return self._cost_view[a*len(OFFSETS)+OFFSET_INDEX[(dr,dc)]]

    def update_vertex(self,s):
        if not hasattr(self,'_cost_view') or self._has_dynamic_blocks or hasattr(self,'approach_strength'):
            return super().update_vertex(s)
        self.update_vertex_count+=1
        if s!=self.goal_id:
            best=math.inf;base=s*len(OFFSETS)
            neighbors=self.geometry._neighbor_view;costs=self._cost_view;g=self._g_view
            for j in range(len(OFFSETS)):
                delta=neighbors[base+j]
                if delta==NO_NEIGHBOR_DELTA:continue
                self.neighbor_visit_count+=1
                value=costs[base+j]+g[s+delta]
                if value<best:best=value
            self._rhs_view[s]=best
        self._push(s)

    def _refresh_heuristic(self):
        sr,sc=self._rows[self.start_id],self._cols[self.start_id]
        # Use the original math.hypot, including its rounding. No approximate
        # or quantized heuristic and no change to queue tie order.
        self._start_heuristic=np.asarray([math.hypot(sr-r,sc-c) for r,c in zip(self._rows,self._cols)],np.float64)
        self._heuristic_view=memoryview(self._start_heuristic)
        self._heuristic_origin=self.start_id

    def _calculate_key(self,s):
        if not hasattr(self,'_heuristic_origin'):
            return super()._calculate_key(s)
        if self._heuristic_origin!=self.start_id:self._refresh_heuristic()
        best=min(self._g_view[s],self._rhs_view[s])
        return best+self._heuristic_view[s]+self.km,best

    def _heuristic(self,a,b):
        return math.hypot(self._rows[a]-self._rows[b],self._cols[a]-self._cols[b])

    def set_blocked_ids(self,ids):
        self._has_dynamic_blocks=True
        result=super().set_blocked_ids(ids)
        self._has_dynamic_blocks=bool(np.any(self.blocked))
        return result

    def replace_potential(self,potential):
        # Parent updates vertices while replacing costs: use the uncached
        # implementation until the new potential is fully installed.
        self._has_dynamic_blocks=True
        result=super().replace_potential(potential)
        self._rebuild_costs()
        return result

    @property
    def resident_bytes(self):
        return super().resident_bytes+getattr(self,'edge_weights',np.empty(0)).nbytes+getattr(self,'_start_heuristic',np.empty(0)).nbytes
