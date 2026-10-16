"""R9 evidence: original strict/Native checks, explicit mode and profiler scope."""
from pathlib import Path
import json,hashlib
import numpy as np,yaml
from compare_v9_review import load,save,sha
from compare_v9_r7 import compare
from compare_v9 import pair,read,tag,summary
from compare_v8_r6 import verify_cleanup

def mode_evidence(directory,mode,protocol=True,expected_v9_mode='A',expected_r2_mode='control',expected_r8_mode='control',expected_profile=False,expected_r9_mode='control'):
 expected_h='H' if mode in ('CH','CHN','CHNT') else 'control'
 expected_n='NT' if mode=='CHNT' else 'N' if mode in ('N','CHN') else 'control'
 mode='C' if mode in ('CH','CHN','CHNT') else 'control' if mode=='N' else mode
 if protocol:
  p=load(directory/'protocol.json')
  assert p['version']=='3YD-V9-r9-L3' and p['r6_mode']==expected_h and p['r7_mode']==expected_n
  assert p['r5_mode']==mode and p['r2_mode']==expected_r2_mode and p['r3_mode']=='control'
  assert p['v9_mode']==expected_v9_mode and p['review_mode']=='S' and p['CPU']==[12]
  assert p['r8_mode']==expected_r8_mode and p['profiler']==expected_profile
  assert p.get('r9_mode','control')==expected_r9_mode
  assert not p['moving_window'] and not p['global_L3_fallback']
 else:
  p=load(directory/'summary.json')
  assert p['version']=='3YD-V9-r9' and p['r6_mode']==expected_h and p['r7_mode']==expected_n and p['r5_mode']==mode and p['r3_mode']=='control' and p['CPU']==[12]
  assert p['optimization_mode']=='AB' and p['v9_mode']==expected_v9_mode and p['review_mode']=='S' and p['r2_mode']==expected_r2_mode
  assert p['r8_mode']==expected_r8_mode
  assert p['static_event_replay'] and not p['live_execution']
 preparation=load(directory/'v9_preparation.json');neutral=preparation['neutral_labels']
 assert preparation['r7_mode']==expected_n
 applicable=expected_n in ('N','NT') and directory.name=='arena_large_250x300_2d_005'
 assert neutral['enabled']==applicable
 if applicable:
  assert neutral['storage_values_unchanged'] and neutral['broadcast_readonly']
  assert neutral['new_backing_buffer_bytes']==2 and neutral['logical_bytes']==60000000
 events=load(directory/'smac/lifecycle_evidence/r5_complete_geometry.json')
 if mode=='control':assert not events;return {'geometry_events':0}
 boot=directory/'smac/bootstrap/map.yaml';b=yaml.safe_load(boot.read_text())
 assert b['resolution']==.05 and Path(b['image']).name=='map.pgm'
 native_log=next((directory/'smac/logs').glob('*.log')).read_text()
 assert 'Loading yaml file: '+str(boot.resolve()) in native_log
 assert 'Read map '+b['image']+': 16 X 16 map @ 0.05 m/cell' in native_log
 pgm=Path(b['image']).read_bytes().split(b'\n',3)
 assert pgm[:3]==[b'P5',b'16 16',b'255'] and pgm[3]==bytes(256)
 # Every changed map is explicitly a complete task. Within a logical task it
 # may only grow, never shrink or recenter during start progress.
 previous={}
 for e in events:
  assert e['complete_task_scope'];key=json.dumps(e['task'],sort_keys=True);box=e['bounds']
  assert f"StaticLayer: Resizing costmap to {e['shape'][1]} X {e['shape'][0]} at 0.050000 m/pix" in native_log
  if key in previous:
   old=previous[key];assert box[0]<=old[0] and box[1]>=old[1] and box[2]<=old[2] and box[3]>=old[3]
  previous[key]=box
 reads=load(directory/'smac/lifecycle_evidence/readbacks.json')
 return {'bootstrap_cells':256,'original_map_yaml_not_loaded':True,
         'geometry_events':len(events),'complete_task_events':events,
         'actual_readback_requests':len(reads),'readback_bytes':sum(r.get('bytes',0) for r in reads),
         'readback_errors':[r for r in reads if not r.get('success')]}


def exact_compare(a,b):
 records=pair(a,b,strict=True)
 review=compare(a,b)
 assert review["upstream_gate"] and review["quality_gate"]
 review["all_physical_outputs_and_effective_maps_identical"]=all(all(v["same_outputs"].values()) and all(v["effective_map_parity"].values()) for v in records)
 assert review["all_physical_outputs_and_effective_maps_identical"]
 return review
