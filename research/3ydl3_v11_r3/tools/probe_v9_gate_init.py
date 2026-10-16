"""Separate-process map predicate parity and actual constructor measurements."""
import argparse,hashlib,json,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from benchmark import Memory
from arena_evaluation.planner_benchmark.map_utils import HospitalMap
from arena_evaluation.path_audit import PathAuditor
from three_yd_v5.v9_gate_init import DirectStorageAuditor
from run_free_full import registered_input
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser();p.add_argument('--map-id',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 a.output.mkdir(parents=True,exist_ok=False)
 mp=ROOT/'inputs/corroborated_map.yaml' if a.map_id=='mengjia' else registered_input(a.map_id)[0]/'map.yaml'
 m=HospitalMap.load(mp);ctx=SimpleNamespace(hospital_map=m);tick=time.monotonic()
 with Memory() as mem:auditor=DirectStorageAuditor(ctx,source_commit='same')
 elapsed=(time.monotonic()-tick)*1000;gate=auditor._distance_to_unsafe_m
 np.save(a.output/'packed.npy',gate.packed)
 data={'map_id':a.map_id,'map_sha256':m.sha256,'shape':list(m.occupancy.shape),'constructor_wall_ms':elapsed,
       'python_constructor_peak_RSS_bytes':mem.peak,'stored_bytes':gate.nbytes,
       'packed_sha256':hashlib.sha256(gate.packed.tobytes()).hexdigest(),'diagnostics':auditor.storage_diagnostics}
 (a.output/'result.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n');print(data,flush=True)
if __name__=='__main__':main()
