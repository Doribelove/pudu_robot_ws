"""Descriptive statistics; failed rows remain in every run denominator."""
import argparse,json,math,collections
from pathlib import Path
import numpy as np
from arena_3d_v1.semantic_world import WORK,json_write

def quantile(values):
 values=[float(x) for x in values if x is not None and np.isfinite(x)]
 if not values:return {'n':0,'p50':None,'p95':None,'max':None}
 return {'n':len(values),'p50':float(np.quantile(values,.50)),'p95':float(np.quantile(values,.95)),'max':max(values)}
def category(row):
 if row['success']:return 'success'
 code=row.get('failure_code','').upper()
 if 'AUDIT' in code or 'COLLISION' in code or 'CURVATURE' in code:return 'unsafe_candidate_rejected'
 if 'ACK' in code or 'COSTMAP' in code:return 'input_content_not_confirmed'
 if 'TIMEOUT' in code or 'DEADLINE' in code or '40S' in code:return 'timeout'
 if 'UNVERIFIED' in code:return 'unverified_feasibility'
 return 'solver_or_pipeline_failure'
def stats(rows):
 return {'count':len(rows),'success':sum(r['success'] for r in rows),'outcomes':dict(collections.Counter(category(r) for r in rows)),
  'end_to_end_wall_ms':quantile([r.get('end_to_end_wall_ms') for r in rows]),
  'successful_end_to_end_wall_ms':quantile([r.get('end_to_end_wall_ms') for r in rows if r['success']]),
  'end_to_end_process_tree_cpu_ms':quantile([r.get('end_to_end_process_tree_cpu_ms') for r in rows]),
  'peak_process_tree_rss_bytes':quantile([r.get('peak_process_tree_rss_bytes') for r in rows]),
  'l1_wall_ms':quantile([r.get('shared_l1_ms') for r in rows]),'l1_cpu_ms':quantile([r.get('shared_l1_cpu_ms') for r in rows]),
  'l2_including_preference_wall_ms':quantile([r.get('l2_activation_ms') for r in rows]),
  'l2_without_preference_wall_ms':quantile([max(0,r['l2_activation_ms']-r.get('preference_build_ms',0)) for r in rows if 'l2_activation_ms' in r]),
  'l2_including_preference_cpu_ms':quantile([r.get('l2_activation_cpu_ms') for r in rows]),
  'preference_field_wall_ms':quantile([r.get('preference_build_ms') for r in rows]),
  'reference_adapter_wall_ms':quantile([r.get('reference_diagnostics',{}).get('adapter_ms') for r in rows]),
  'reference_adapter_cpu_ms':quantile([r.get('reference_diagnostics',{}).get('adapter_cpu_ms') for r in rows]),
  'content_ack_wall_ms':quantile([r.get('l3_diagnostics',{}).get('costmap_ack_wait_ms') for r in rows]),
  'all_content_update_wall_ms':quantile([r.get('l3_diagnostics',{}).get('local_map_update_ms') for r in rows]),
  'smac_search_plus_smoothing_wall_ms':quantile([r.get('l3_diagnostics',{}).get('planning_time_ms') for r in rows]),
  'smac_process_cpu_ms':quantile([r.get('l3_diagnostics',{}).get('planner_cpu_total_ms') for r in rows]),
  'independent_audit_wall_ms':quantile([r.get('independent_audit',{}).get('audit_wall_ms') for r in rows]),
  'independent_audit_cpu_ms':quantile([r.get('independent_audit',{}).get('audit_cpu_ms') for r in rows]),
  'l2_mutable_state_resident_bytes':quantile([r.get('l2',{}).get('mutable_state_resident_bytes') for r in rows]),
  'l2_geometry_resident_bytes':quantile([r.get('l2',{}).get('geometry_resident_bytes') for r in rows]),
  'l2_state_memory_bytes':quantile([r.get('l2',{}).get('state_memory_bytes') for r in rows]),
  'l2_reinitializations':sum(r.get('l2',{}).get('reinitialize_count',0) for r in rows),
  'l2_fallbacks':sum(r.get('l2',{}).get('fallback_count',0) for r in rows),
  'cache_invalidations':sum(bool(r.get('l2',{}).get('activation',{}).get(k)) for r in rows for k in ['geometry_reject','state_reject','geometry_cache_reject_reason','state_cache_reject_reason']),
  'active_cache_hits':sum(bool(r.get('l2',{}).get('activation',{}).get('active_hit')) for r in rows),
  'state_cache_hits':sum(bool(r.get('l2',{}).get('activation',{}).get('state_cache_hit')) for r in rows),
  'geometry_cache_hits':sum(bool(r.get('l2',{}).get('activation',{}).get('geometry_cache_hit')) for r in rows),
  'full_publication_fallbacks':sum(bool(r.get('l3_diagnostics',{}).get('local_map_update_fallback')) for r in rows),
  'l3_right_band_ratio':quantile([r.get('preference',{}).get('right_band_ratio') for r in rows if r['success']]),
  'l2_right_band_ratio':quantile([r.get('reference_metrics',{}).get('l2',{}).get('right_band_ratio') for r in rows if r['success']]),
  'path_length_m':quantile([r.get('preference',{}).get('length_m') for r in rows if r['success']]),
  'reference_deviation_mean_m':quantile([r.get('reference_metrics',{}).get('reference_deviation_mean_m') for r in rows if r['success']]),
  'reference_deviation_path_p95_m':quantile([r.get('reference_metrics',{}).get('reference_deviation_p95_m') for r in rows if r['success']])}

def main():
 p=argparse.ArgumentParser();p.add_argument('--real',required=True);p.add_argument('--controlled',required=True);p.add_argument('--output',required=True);args=p.parse_args()
 root=Path(args.real);rows=list(map(json.loads,(root/'runs.jsonl').read_text().splitlines()));controls=list(map(json.loads,(Path(args.controlled)/'runs.jsonl').read_text().splitlines()))
 arms=['original_r1','A','B','C'];byarm={a:stats([r for r in rows if r['arm']==a]) for a in arms}
 lookup={(r['query_id'],r['arm'],r['repetition']):r for r in rows};regressions=[]
 for (q,a,k),r in lookup.items():
  c=lookup.get((q,'C',k))
  if a=='original_r1' and r['success'] and (c is None or not c['success']):regressions.append({'query_id':q,'repetition':k,'C_failure':c.get('failure_code') if c else 'missing'})
 controlled=[]
 for scene in ['calibration','heldout']:
  for rep in range(1,4):
   r=[x for x in controls if x['scene']==scene and x['repetition']==rep]
   a=next(x for x in r if x['arm']=='A');b=next(x for x in r if x['arm']=='B');right=[x for x in r if x['arm']=='C' and x['side']=='right'];left=next(x for x in r if x['arm']=='C' and x['side']=='left')
   ratios=[x.get('metrics',{}).get('target_band_ratio',0) for x in right]
   monotonic=[];means={}
   for label,x in [('right',right[0]),('left',left)]:
    poses=np.load(Path(x['case'])/'path.npy');axis=1 if scene=='heldout' else 0
    monotonic.append(bool(np.all(np.diff(poses[:,axis])>=-1e-6)))
    ds=np.linalg.norm(np.diff(poses[:,:2],axis=0),axis=1);arc=np.r_[0,np.cumsum(ds)]
    s=np.arange(5.,arc[-1]-5.,.025);means[label]=float(np.mean(np.interp(s,arc,poses[:,1-axis])))
   shift=(means['right']-means['left']) if scene=='heldout' else means['left']-means['right']
   a_ratio=a.get('metrics',{}).get('target_band_ratio',0)
   response=all(x['success'] for x in r) and min(ratios)>=.8 and ((min(ratios)-a_ratio>=.2) if a_ratio<.8 else shift>=.5) and shift>=.5 and all(monotonic) and left.get('metrics',{}).get('target_band_ratio',0)>=.8 and all(x.get('metrics',{}).get('trimmed_eligible_arc_length_m',0)>=20. for x in r)
   controlled.append({'scene':scene,'repetition':rep,'A_band_ratio':a_ratio,'B_band_ratio':b.get('metrics',{}).get('target_band_ratio'),
    'C_right_band_ratios':ratios,'C_left_band_ratio':left.get('metrics',{}).get('target_band_ratio'),
    'mirror_signed_arc_lateral_shift_m':shift,'no_longitudinal_backtracking':all(monotonic),'right_switchback_pose_hash_matches':right[0].get('pose_hash')==right[1].get('pose_hash'),'pass':response})
 audit_invalid=[{'query_id':r['query_id'],'arm':r['arm'],'rep':r['repetition']} for r in rows if r['success'] and not r.get('independent_audit',{}).get('valid')]
 result={'real_root':str(root),'controlled_root':args.controlled,'quantile_method':'numpy linear, descriptive; 20 unique queries and 3 dependent repetitions, not population tail estimates',
  'by_arm':byarm,'by_arm_cold_first_repeat':{a:stats([r for r in rows if r['arm']==a and r['repetition']==1]) for a in arms},
  'by_arm_later_repeats':{a:stats([r for r in rows if r['arm']==a and r['repetition']>1]) for a in arms},
  'controlled':controlled,'regressions':regressions,'accepted_invalid_trajectories':audit_invalid,
  'failures':[{'query_id':r['query_id'],'arm':r['arm'],'repetition':r['repetition'],'failure_code':r['failure_code'],'category':category(r)} for r in rows if not r['success']],
  'worst_cases':sorted([{'query_id':r['query_id'],'arm':r['arm'],'repetition':r['repetition'],'end_to_end_wall_ms':r.get('end_to_end_wall_ms'),'success':r['success'],'failure_code':r['failure_code']} for r in rows],key=lambda x:x['end_to_end_wall_ms'] or -1,reverse=True)[:10],
  'gates':{'real_expected_240_rows':len(rows)==240 and len(lookup)==240 and len({r['query_id'] for r in rows})==20 and all(v['count']==60 for v in byarm.values()),'controlled_expected_30_rows':len(controls)==30,'controlled_reference_only_hard_unchanged':all(r['hard_changed_cells']==0 for r in controls if r.get('reference_only_change')),'controlled_switchback_identical':all(x['right_switchback_pose_hash_matches'] for x in controlled),'every_success_exact_content_ack':all(r.get('l3_diagnostics',{}).get('costmap_update_acknowledged') and r.get('l3_diagnostics',{}).get('costmap_ack_mismatch_cells')==0 for r in rows if r['success']),'control_preference':all(x['pass'] for x in controlled),'accepted_unsafe_zero':not audit_invalid,'no_original_success_to_C_failure':not regressions},
  'ack_attempt_failures':len((root/'smac/logs/semantic_exact_ack_failures.jsonl').read_text().splitlines()) if (root/'smac/logs/semantic_exact_ack_failures.jsonl').exists() else 0,
  'formal_l2_cache_bytes':sum(p.stat().st_size for p in (root/'l2_caches').rglob('*') if p.is_file())}
 json_write(Path(args.output),result);print(json.dumps(result['gates']),flush=True)
if __name__=='__main__':main()
