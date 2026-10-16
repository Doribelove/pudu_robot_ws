from pathlib import Path
from types import SimpleNamespace
import os,json,hashlib,time,gc
import numpy as np
from three_yd_v5.free_space_topology import DeferredOccupancyMap
from three_yd_v5.v9_storage import StorageAuditor
from three_yd_v5.v9_gate_init import DirectStorageAuditor
E=Path(__file__).resolve().parents[1];out=[];os.sched_setaffinity(0,{14});os.environ['THREEYD_V9_R1_MODE']='S'
for item in json.loads((E/'MAPS.json').read_text())['maps']:
 m=DeferredOccupancyMap(item['map_yaml']);ctx=SimpleNamespace(hospital_map=m);os.environ['THREEYD_V9_R2_MODE']='control';a=StorageAuditor(ctx,source_commit='3YDL3-V10-r1-audit-equivalence');os.environ['THREEYD_V9_R2_MODE']='D';b=DirectStorageAuditor(ctx,source_commit='3YDL3-V10-r1-audit-equivalence');assert b.storage_diagnostics['direct_initialized'];assert np.array_equal(a._distance_to_unsafe_m.packed,b._distance_to_unsafe_m.packed);row={'map_id':item['map_id'],'shape':list(m.occupancy.shape),'cells':m.occupancy.size,'all_original_audit_gate_decisions_equal':True,'packed_sha256':hashlib.sha256(a._distance_to_unsafe_m.packed.tobytes()).hexdigest(),'direct':b.storage_diagnostics};out.append(row);print(item['map_id'],'ALL_CELLS_IDENTICAL',flush=True);del a,b,ctx,m;gc.collect()
(E/'report/AUDIT_GATE_EQUIVALENCE.json').write_text(json.dumps(out,ensure_ascii=False,indent=2))
