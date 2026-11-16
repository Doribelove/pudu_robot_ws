"""Equal event-search policy for retained and cold D* state."""
import time
from three_yd_v4.r5_controller import BoundedReferenceDStar
from three_yd_v5.tail_astar import weighted_astar

class EventDStar(BoundedReferenceDStar):
    def event_solve(self,changed):
        tick=time.monotonic_ns();phases={};phase=tick
        # The original Python event cap of 20k nodes aborted the native loop
        # before its 0.5 s wall allowance. This is a search effort limit only;
        # both arms keep the same wall clock and unchanged physical checks.
        stats=self.state.compute_shortest_path(timeout_s=.5,max_expansions=80000)
        phases['search_call_ms']=(time.monotonic_ns()-phase)/1e6;phase=time.monotonic_ns()
        ids=None if stats.timeout_triggered else self.state.extract_path_ids()
        phases['path_extract_ms']=(time.monotonic_ns()-phase)/1e6;phase=time.monotonic_ns()
        ready=not stats.timeout_triggered
        if ids is not None and not self.state.path_is_valid(ids):ids=None;ready=False
        phases['path_validate_ms']=(time.monotonic_ns()-phase)/1e6;phase=time.monotonic_ns()
        self.state.ready=ready;fallback=None
        if not ready:
            self.fallback_count+=1
            ids,fallback=weighted_astar(self.state,timeout_s=2.)
        phases['fallback_ms']=(time.monotonic_ns()-phase)/1e6;phase=time.monotonic_ns()
        self.state.initialized=True
        failure='' if ids is not None else ('L2_TIMEOUT' if fallback and fallback.timeout_triggered else 'L2_UNVERIFIED_NO_PATH')
        result=self._result(started_ns=tick,path_ids=ids,failure=failure,
            backend='v13_native_incremental_dstar' if ready else 'v13_astar_after_dstar_budget',
            stats=stats,fallback=fallback,changed=changed,reused=True,oracle_cost_error=None)
        phases['result_build_ms']=(time.monotonic_ns()-phase)/1e6;phase=time.monotonic_ns()
        result.diagnostics.update(path_cost=None if ids is None else self.state.path_cost(ids),
            dstar_expanded=stats.expanded_nodes,astar_expanded=0 if fallback is None else fallback.expanded_nodes,
            timeout_is_physical_blockage=False)
        phases['path_cost_ms']=(time.monotonic_ns()-phase)/1e6
        total=(time.monotonic_ns()-tick)/1e6
        phases['unaccounted_ms']=total-sum(phases.values())
        result.diagnostics.update(phase_ms=phases,solve_wall_ms=total,
            extraction_backend=getattr(self.state,'v13_extraction_backend','skipped'))
        return result
