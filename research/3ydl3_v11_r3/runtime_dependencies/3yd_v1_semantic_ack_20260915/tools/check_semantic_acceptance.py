"""Fail loudly if functional integrity, final denominators or cleanup regress."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
d=json.loads((ROOT/'report/performance.json').read_text());i=d['integrity'];checks={}
for arm,s in d['main'].items():
 checks[arm+'_120_queries']=s['n']==s['success']==120
 checks[arm+'_audit_and_exact_ack']=s['accepted_invalid']==0 and s['exact_ack_all_accepted']
 checks[arm+'_cleanup']=s['cleanup_complete']
checks['native_only_service_callback_changed']=i['native_copy_changes']==['src/costmap_2d_publisher.cpp']
checks['frozen_sources_unchanged']=not i['changed_frozen_files']
checks['baseline_and_production_sources_unchanged']=not i['baseline_changed'] and not i['native_source_changed_in_original']
checks['same_vehicle_params_and_smac']=i['parameters_identical'] and i['same_vehicle_sweep'] and i['same_smac_plugin']
checks['same_20_query_paths']=d['identical_query_paths']==20
checks['all_100_semantic_updates']=sum(s['success'] for s in d['stress'].values())==sum(s['requested_updates'] for s in d['stress'].values())==100
checks['candidate_no_natural_full_map_fallbacks']=d['main']['candidate']['map_repair_count']==0
checks['candidate_no_natural_gap_repairs']=d['main']['candidate']['gap_repairs']==0
checks['lifecycle_12_of_12']=d['lifecycle']['success']==d['lifecycle']['cycles']==12
checks['all_predeclared_jobs_pass']=len(d['dispatch'])==18 and all(r['assertions_passed'] and r['returncode']==0 for r in d['dispatch'])
checks['regression_82']=sum(int(s['tests']) for s in d['regression'])==82 and all(int(s['failures'])==int(s['errors'])==0 for s in d['regression'])
checks['controls_all_pass']=sum(s['success'] for s in d['controls'].values())==sum(s['n'] for s in d['controls'].values())==30
checks['preference_switch_preserved']=all(d['controls'][s]['right_left_right_pose_restored'] and d['controls'][s]['minimum_target_band_ratio']>=.99 for s in ['calibration','rotated'])
old=d['main']['baseline']['timing_s']['all']['p95'];new=d['main']['candidate']['timing_s']['all']['p95']
result={'checks':checks,'functional_acceptance':all(checks.values()),'p95_change_percent':100*(new/old-1),'p95_improved':new<old,'scope':'isolated static planning experiment; not production deployment or hard real-time guarantee'}
(ROOT/'report/acceptance.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));assert all(checks.values())
