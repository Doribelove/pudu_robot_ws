"""Check the delivered run set without erasing exploratory failures."""
import json,xml.etree.ElementTree as ET
from arena_3d_v1.semantic_world import WORK,json_write

def main():
 d=json.loads((WORK/'report/performance.json').read_text());geometry=json.loads((WORK/'report/final_geometry_audit.json').read_text())
 suites=ET.parse(WORK/'report/regression_v3.xml').getroot().findall('testsuite')
 fault={(r['arm'],r['injection']):r for r in d['ack_fault_injection']}
 checks={
  'fixed_queries':d['main']['candidate']['success']==d['main']['candidate']['n']==60,
  'fixed_queries_independent_audit':d['main']['candidate']['accepted_invalid']==0,
  'curvature_examples':len(geometry['records'])==42 and all(r['success'] and r['canonical_valid'] and r['independent_valid'] for r in geometry['records']),
  'curvature_examples_source':not geometry['source_mismatches'],
  'startup_and_updates':sum(s['cycles'] for s in d['startup'].values())==100 and all(s['startup_success']==s['complete_success']==s['cycles'] and s['updates_success']==3*s['cycles'] for s in d['startup'].values()),
  'fault_first_readback_recovered':fault['candidate','drop_first']['startup_success'] and fault['candidate','drop_first']['requests_removed']==1,
  'fault_first_startup_tile_recovered':fault['candidate','drop_first_tile']['startup_success'],
  'fault_first_mask_tile_recovered':fault['candidate','drop_first_mask_tile'].get('update_success',False),
  'fault_wrong_content_rejected':not fault['candidate','corrupt_all']['startup_success'],
  'regression':sum(int(s.attrib['tests']) for s in suites)==74 and all(int(s.attrib['failures'])==int(s.attrib['errors'])==0 for s in suites),
  'source_integrity':not d['integrity']['baseline_runtime_changed'] and not d['integrity']['changed_frozen_files'] and d['integrity']['identical_smac_parameters'] and d['integrity']['identical_vehicle_sweep_binary'],
 }
 for name,v in d['controls'].items():
  checks['control_'+name]=v['success']==v['n'] and v['n']>0
  if 'right_left_right_pose_restored' in v:checks['preference_restore_'+name]=v['right_left_right_pose_restored']
 old=d['main']['3YD-V1-r1']['timing_s']['all']['p95'];new=d['main']['candidate']['timing_s']['all']['p95']
 result={'checks':checks,'functional_acceptance':all(checks.values()),'p95_change_percent':(new/old-1)*100,
         'performance_note':'Report measured latency including semantic map resends. Functional acceptance does not claim timing improvement or hard realtime.',
         'not_validated':['dynamic obstacles','in-motion route switching','physical robot execution','arbitrary multi-visit L2 completeness']}
 json_write(WORK/'report/acceptance.json',result);print(json.dumps(result,indent=2))
 if not result['functional_acceptance']:raise SystemExit(1)
if __name__=='__main__':main()
