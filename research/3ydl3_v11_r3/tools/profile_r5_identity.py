"""Excluded diagnostic instrumentation; inspect immutable view reuse only."""
if __name__=='__main__':
 from owned_process import supervised_entry
 supervised_entry()
from pathlib import Path
import json
import run_task_progress
from three_yd_v5.v9_r5_session import FullRouteStorageSession
from three_yd_v5.v9_maps import immutable
original=FullRouteStorageSession._prepare_native
original_close=FullRouteStorageSession.close
def prepare(self,mask):
 g=self._r5_global_grid;c=self._r5_grid_cache;m=self._r5_mask_cache
 row={'query':self.current_query_id,'source_grid_identity':id(g),'mask_identity':id(mask),
      'mask_immutable':immutable(mask),'same_mask_identity':m is not None and m[0] is mask,
      'same_grid_wrapper':c is not None and c[0] is g,
      'same_occupancy_storage':c is not None and c[0].occupancy_grid is g.occupancy_grid,
      'same_master_storage':c is not None and c[0].expected_master_cost is g.expected_master_cost,
      'occupancy_immutable':immutable(g.occupancy_grid),'master_immutable':immutable(g.expected_master_cost)}
 old=None if c is None else c[3:]
 try:return original(self,mask)
 finally:
  row.update(local_occupancy_reused=old is not None and old[0] is self._semantic_costmap.occupancy_grid,
             local_master_reused=old is not None and old[1] is self._semantic_costmap.expected_master_cost)
  self.identity_trace=getattr(self,'identity_trace',[]);self.identity_trace.append(row)
def close(self):
 try:return original_close(self)
 finally:(self.lifecycle_root/'r5_identity_diagnostic.json').write_text(json.dumps(getattr(self,'identity_trace',[]),indent=2)+'\n')
FullRouteStorageSession._prepare_native=prepare
FullRouteStorageSession.close=close
if __name__=='__main__':run_task_progress.main()
