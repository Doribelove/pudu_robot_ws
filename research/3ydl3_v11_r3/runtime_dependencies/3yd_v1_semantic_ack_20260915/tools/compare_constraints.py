"""Read-only replay of frozen trajectories, including unclipped steering evidence."""
import argparse,copy,hashlib,json,math,os,csv
from pathlib import Path
from collections import Counter
from types import SimpleNamespace
import numpy as np,yaml
from scipy import ndimage
from arena_3d_v1.semantic_world import WORK,SemanticWorld,SweptChecker,densify,wrap,digest,json_write
from arena_evaluation.unified_four_backends_smoke import MapContext
from arena_evaluation.path_audit import PathAuditor
from arena_evaluation.planner_benchmark.models import Query

BATCHES={'baseline':['final_1_baseline','final_4_baseline'],'candidate':['final_2_combined','final_3_combined']}
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):return json.loads(Path(p).read_text())
def jl(p):return [json.loads(s) for s in Path(p).read_text().splitlines()]
def qstat(a):
 a=[v for v in a if v is not None]
 return {'n':len(a),'min':float(min(a)) if a else None,'p50':float(np.quantile(a,.5)) if a else None,'p95':float(np.quantile(a,.95)) if a else None,'max':float(max(a)) if a else None}

def steering_check(p,points):
 delta=np.diff(p,axis=0);length=np.linalg.norm(delta[:,:2],axis=1);dyaw=wrap(delta[:,2])
 # Exactly the existing annotator's pre-clipping formula, no new convention.
 raw=np.arctan(.5*np.divide(dyaw,length,out=np.zeros(len(length)),where=length>1e-9));raw=np.r_[raw,raw[-1]]
 metadata=np.array([s['steering'] for s in points]);step=np.abs(np.diff(raw));threshold=math.radians(15)+1e-6
 arc_steer=np.arctan(.5*np.divide(2*np.sin(dyaw/2),length,out=np.zeros(len(length)),where=length>1e-9));arc_step=abs(np.diff(arc_steer))
 mismatch=abs(metadata-raw);bad=np.flatnonzero(step>threshold);geometric=copy.deepcopy(points)
 for point,value in zip(geometric,raw):point['steering']=float(value)
 middle=p[:-1,2]+dyaw/2;projection=delta[:,0]*np.cos(middle)+delta[:,1]*np.sin(middle)
 tangent=np.arctan2(delta[:,1],delta[:,0]);alignment=np.rad2deg(abs(wrap(tangent-middle)))
 result={'raw_max_step_deg':float(np.rad2deg(step.max(initial=0))),'raw_jump_count':int(len(bad)),
  'metadata_max_step_deg':float(np.rad2deg(abs(np.diff(metadata)).max(initial=0))),
  'metadata_jump_count':int(np.sum(abs(np.diff(metadata))>threshold)),
  'metadata_mismatched_points':int(np.sum(mismatch>1e-6)),'max_metadata_error_deg':float(np.rad2deg(mismatch.max(initial=0))),
  'circular_arc_max_step_deg':float(np.rad2deg(arc_step.max(initial=0))),'circular_arc_jump_count':int(np.sum(arc_step>threshold)),
  'max_pose_spacing_m':float(length.max(initial=0)),'max_yaw_step_deg':float(np.rad2deg(abs(dyaw).max(initial=0))),
  'negative_forward_segments':int(np.sum(projection < -1e-6)),
  'max_midheading_tangent_error_deg':float(alignment[length>1e-8].max(initial=0)),
  'violation_point_indices_zero_based':bad.tolist()}
 if len(bad):
  i=int(step.argmax());result['worst_example']={'index_zero_based':i,'poses':p[i:i+3].tolist(),'raw_steering_deg':np.rad2deg(raw[i:i+2]).tolist(),'metadata_steering_deg':np.rad2deg(metadata[i:i+2]).tolist()}
 return result,geometric

def replay(world,auditor,query,case,record,graph,regions,edt):
 p=np.load(case/'path.npy');mask=np.load(case/'corridor.npy');points=load(case/'path_points.json')
 assert digest(p)==record['pose_hash'] and digest(mask)==record['hard_corridor_hash']
 assert np.array_equal(p,np.array([[v['x'],v['y'],v['yaw']] for v in points]))
 frozen=load(case/'outcome.json');canonical=auditor.audit(query,copy.deepcopy(points),mask);independent=world.audit(query,p,mask)
 raw_info,correct_points=steering_check(p,points);unclipped=auditor.audit(query,correct_points,mask)
 dense=densify(p);rows,cols=world.cells(dense)
 occupancy_ok=SweptChecker(world.map.occupancy!=0,world.map.resolution,world.map.origin,*world.vehicle[:2]).check(dense)
 semantic_ok=world.checker.check(dense)
 # Geometric lower bound: exact unsafe-cell-center EDT -> cell square and
 # actual pose uncertainty -> body bounding disk -> between-sample motion.
 max_step=float(np.linalg.norm(np.diff(dense[:,:2],axis=0),axis=1).max(initial=0))
 rr=rows;cc=cols;min_center=float(edt[rr,cc].min())
 body_lower_bound=min_center-math.sqrt(2)*world.map.resolution-math.hypot(*world.vehicle[:2])-max_step/2
 nodes={n['id']:n for n in graph['nodes']};start_cell=world.map.world_to_cell(*query.start[:2]);needed=[int(regions[start_cell])]
 successful=[v for v in record['recovery']['attempts'] if v['success']];assert len(successful)==1
 portals=successful[0]['portal_sequence'];assert portals[0]==-1 and portals[-1]==-2
 for portal_id in portals[1:-1]:
  targets={n['to_region'] for n in nodes.values() if n['portal']==portal_id and n['from_region']==needed[-1]}
  assert len(targets)==1, (portal_id,needed[-1],targets)
  needed.append(targets.pop())
 values=regions[rr,cc];visits=list(map(int,values[np.r_[True,values[1:]!=values[:-1]]]))
 index=0
 for r in visits:
  if index<len(needed) and r==needed[index]:index+=1
 sequence_ok=index==len(needed)
 endpoint_ok=all(e['position_error_m']<=.125+1e-9 and e['yaw_error_rad']<=math.pi/36+1e-9 for e in independent['endpoints'].values())
 result={'canonical_original_valid':canonical.final_valid_success,'canonical_replayed_metrics':canonical.metrics,
  'stored_canonical_metrics':frozen['metrics'],'independent_replay':independent,
  'static_occupancy_swept_valid':occupancy_ok,'semantic_hard_swept_valid':semantic_ok,
  'body_and_corridor_swept_valid':'SWEPT_BODY_COLLISION_OR_CORRIDOR_EXIT' not in independent['failures'],
  'strict_endpoint_valid':endpoint_ok,'curvature_valid':'CURVATURE' not in independent['failures'],
  'minimum_radius_m':1/independent['max_curvature_1pm'] if independent['max_curvature_1pm']>0 else None,
  'clearance_body_disk_lower_bound_m':body_lower_bound,'center_unsafe_cell_distance_min_m':min_center,
  'ordered_regions':{'required':needed,'visited':visits,'as_subsequence':sequence_ok,'strict_equal':needed==visits,'note':'Same subsequence criterion as the existing turnaround final check; not strict no-extra-visit proof'},
  'steering':raw_info,'canonical_unclipped_valid':unclipped.final_valid_success,'canonical_unclipped_metrics':unclipped.metrics}
 # Isolate the steering representation effect while preserving all poses.
 result['replay_matches_recorded_original_gate']=canonical.final_valid_success==frozen['metrics']['final_valid_success'] and independent['valid']==frozen['independent_audit']['valid']
 result['unclipped_only_steering_failure']=unclipped.metrics['failure_detail']=='STEERING_DISCONTINUITY'
 return result

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args();out=Path(args.output)
 if (out/'comparison.json').exists():raise FileExistsError(out/'comparison.json')
 out.mkdir(parents=True,exist_ok=True);os.sched_setaffinity(0,{12})
 cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());world=SemanticWorld(cfg['inputs']['map_yaml'],cfg['inputs']['semantic_path'],cache_root=out/'world_cache')
 ctx=MapContext('constraint_replay',world.map,world.safe,world.map.distance_m,world.map.sha256,sha(cfg['inputs']['map_yaml']),Path(cfg['inputs']['map_yaml']))
 auditor=PathAuditor(ctx,source_commit='saved-trajectory-constraint-replay')
 edt=ndimage.distance_transform_edt(~world.hard)*world.map.resolution
 quality={(r['batch'],r['query'],r['repetition']):r for r in load(WORK/'report/path_quality.json')}
 source_manifest={};rows=[];cache={};batch_meta={}
 for arm,batches in BATCHES.items():
  for batch in batches:
   root=WORK/'results'/batch;protocol=load(root/'protocol.json');queries={q['query_id']:q for q in protocol['queries']}
   graph_path=next((root/'preparation_cache/graph').rglob('graph.json'));graph=load(graph_path)
   with np.load(graph_path.with_name('regions.npz')) as z:regions=z['subregions']
   source_manifest[str(graph_path)]=sha(graph_path);source_manifest[str(graph_path.with_name('regions.npz'))]=sha(graph_path.with_name('regions.npz'))
   batch_meta[batch]={'graph_key':graph['key'],'map_hash':protocol['map_hash'],'vehicle':protocol['vehicle']}
   for f in [root/'runs.jsonl',root/'protocol.json',root/'summary.json']:source_manifest[str(f)]=sha(f)
   batch_rows=jl(root/'runs.jsonl')
   for record in batch_rows:
    case=Path(record['case']);q=queries[record['query_id']];query=Query(q['query_id'],tuple(q['start']),tuple(q['goal']),seed=q.get('seed',20260914))
    for name in ['path.npy','path_points.json','corridor.npy','outcome.json','reference_fine_cells.npy']:
     f=case/name;source_manifest[str(f)]=sha(f)
    key=(record['pose_hash'],record['hard_corridor_hash'],sha(case/'path_points.json'),graph['key'],tuple(record['recovery']['attempts'][-1]['portal_sequence']))
    if key not in cache:
     cache[key]=replay(world,auditor,query,case,record,graph,regions,edt)
     print(batch,q['query_id'],'original',cache[key]['canonical_original_valid'],'unclipped',cache[key]['canonical_unclipped_valid'],'raw_step',round(cache[key]['steering']['raw_max_step_deg'],3),flush=True)
    else:
     # Every repeated artifact must independently match its logged hashes.
     assert digest(np.load(case/'path.npy'))==record['pose_hash'] and digest(np.load(case/'corridor.npy'))==record['hard_corridor_hash']
    a=record['l3'];r={'arm':arm,'batch':batch,'query':q['query_id'],'repetition':record['repetition'],'case':str(case),'recorded_success':record['success'],**cache[key],
     'exact_ack_accepted':a.get('costmap_update_acknowledged') is True and a.get('costmap_ack_mismatch_cells')==0 and not a.get('costmap_ack_hash_mismatch') and not a.get('costmap_ack_sequence_mismatch'),
     'ack_status':a.get('costmap_ack_status'),'soft_cost_valid':record['reference']['finite_nonnegative'] and record['reference']['max_soft_cost']<=140,
     'pose_hash':record['pose_hash'],'reference_hash':sha(case/'reference_fine_cells.npy'),'quality':quality[(batch,q['query_id'],record['repetition'])]}
    rows.append(r)
   del regions
 source_manifest[str(WORK/'config/acceptance.yaml')]=sha(WORK/'config/acceptance.yaml');source_manifest[str(Path(__file__).resolve())]=sha(Path(__file__).resolve())
 for name in ['semantic_world.py','curvature_repair.py']:
  f=WORK/'external/arena4_ws/src/arena/three_d_v1/arena_3d_v1'/name;source_manifest[str(f)]=sha(f)
 for name in ['path_audit.py','unified_four_backends_smoke.py']:
  f=WORK/'external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation'/name;source_manifest[str(f)]=sha(f)
 def summary(rr):
  return {'count':len(rr),'unique_queries':len({r['query'] for r in rr}),
   'passes':{k:sum(bool(r[k]) for r in rr) for k in ['recorded_success','canonical_original_valid','static_occupancy_swept_valid','semantic_hard_swept_valid','body_and_corridor_swept_valid','strict_endpoint_valid','curvature_valid','canonical_unclipped_valid','exact_ack_accepted','soft_cost_valid','replay_matches_recorded_original_gate']},
   'ordered_region_subsequence_passes':sum(r['ordered_regions']['as_subsequence'] for r in rr),
   'strict_region_sequence_matches':sum(r['ordered_regions']['strict_equal'] for r in rr),
   'unclipped_only_steering_failures':sum(r['unclipped_only_steering_failure'] for r in rr),
   'raw_steering_violating_paths':sum(r['steering']['raw_jump_count']>0 for r in rr),'raw_steering_violating_transitions':sum(r['steering']['raw_jump_count'] for r in rr),
   'raw_steering_max_step_deg':qstat([r['steering']['raw_max_step_deg'] for r in rr]),
   'raw_arc_steering_max_step_deg':qstat([r['steering']['circular_arc_max_step_deg'] for r in rr]),
   'metadata_steering_max_step_deg':qstat([r['steering']['metadata_max_step_deg'] for r in rr]),
   'maximum_curvature_1pm':qstat([r['independent_replay']['max_curvature_1pm'] for r in rr]),
   'minimum_radius_m':qstat([r['minimum_radius_m'] for r in rr]),
   'position_error_m':qstat([e['position_error_m'] for r in rr for e in r['independent_replay']['endpoints'].values()]),
   'heading_error_deg':qstat([math.degrees(e['yaw_error_rad']) for r in rr for e in r['independent_replay']['endpoints'].values()]),
   'max_pose_spacing_m':qstat([r['steering']['max_pose_spacing_m'] for r in rr]),
   'max_yaw_step_deg':qstat([r['steering']['max_yaw_step_deg'] for r in rr]),
   'max_midheading_tangent_error_deg':qstat([r['steering']['max_midheading_tangent_error_deg'] for r in rr]),
   'legacy_center_clearance_m':qstat([r['stored_canonical_metrics']['minimum_clearance_m'] for r in rr]),
   'body_disk_clearance_lower_bound_m':qstat([r['clearance_body_disk_lower_bound_m'] for r in rr]),
   'recorded_metric_totals':{k:sum(r['stored_canonical_metrics'][k] for r in rr) for k in ['reverse_distance_m','in_place_rotation_count','position_discontinuity_count','heading_discontinuity_count','steering_jump_count']},
   'length_m':qstat([r['independent_replay']['length_m'] for r in rr]),
   'right_band_ratio':qstat([r['quality']['right_band_ratio'] for r in rr]),
   'right_band_arc_weighted_ratio':sum(r['quality']['right_band_ratio']*r['quality']['trimmed_eligible_arc_length_m'] for r in rr)/sum(r['quality']['trimmed_eligible_arc_length_m'] for r in rr),
   'unique_queries_at_least_80_percent':len({r['query'] for r in rr if r['quality']['right_band_ratio']>=.8}),
   'unique_queries_zero_right_ratio':sorted({r['query'] for r in rr if r['quality']['right_band_ratio']==0})}
 arms={a:summary([r for r in rows if r['arm']==a]) for a in BATCHES};pairs=[]
 for query in dict.fromkeys(r['query'] for r in rows):
  rr=[r for r in rows if r['query']==query];x=rr[0]
  pairs.append({'query':query,'same_poses_all_12':len({r['pose_hash'] for r in rr})==1,'same_references_all_12':len({r['reference_hash'] for r in rr})==1,
   'raw_steering_max_step_deg':x['steering']['raw_max_step_deg'],'raw_steering_jump_count':x['steering']['raw_jump_count'],
   'curvature_max_1pm':x['independent_replay']['max_curvature_1pm'],'minimum_radius_m':x['minimum_radius_m'],
   'endpoint_position_max_m':max(e['position_error_m'] for e in x['independent_replay']['endpoints'].values()),
   'endpoint_heading_max_deg':max(math.degrees(e['yaw_error_rad']) for e in x['independent_replay']['endpoints'].values()),
   'right_ratio':x['quality']['right_band_ratio'],'length_m':x['independent_replay']['length_m'],'ordered_subsequence':x['ordered_regions']['as_subsequence'],
   'clearance_body_disk_lower_bound_m':x['clearance_body_disk_lower_bound_m'],'raw_reaudit_failure':x['canonical_unclipped_metrics']['failure_detail'],'case':x['case']})
 semantic=load(cfg['inputs']['semantic_path']);freeze=load(WORK/'report/FINAL_IMPLEMENTATION_FREEZE.json')
 result={'comparison':'上一轮掉头/启动 ACK 实现重测 vs 本轮语义 ACK/退出清理实现','schema':'3yd-saved-trajectory-constraints-v1','arms':arms,'paired_queries':pairs,
  'replay_unique_keys':len(cache),'denominator':'20 queries x 3 repeats x 2 batches per arm; fixed saved trajectories; no new planning or timing samples',
  'rule_coverage':{'one_way_count':len(world.rules.get('one_way',[])),'forbidden_transition_count':len(world.rules.get('forbidden_transitions',[])),
   'input_traffic_rules':semantic['traffic_rules'],'semantic_classes':dict(Counter(f['semantic_class'] for f in semantic['features'])),
   'hard_semantic_features':sum(f.get('hard',False) for f in semantic['features']),'semantic_additional_hard_cells':int(np.count_nonzero(world.hard & (world.map.occupancy==0))),
   'unconfirmed_fences':sum(f['semantic_class']=='fence_area' and not f['properties'].get('confirmed_impassable_entity',False) for f in semantic['features']),
   'one_way_turn_bans_assessment':'N/A: no active directional or prohibited-turn rules in these real-map runs; right-hand preference is soft, not one-way',
   'speed_acceleration_nonstop':'not assessed: geometric paths have placeholder velocity=0 and no execution time'},
  'thresholds':{'curvature_limit_1pm':2.5,'curvature_numerical_tolerance_1pm':1e-6,'position_m':.125,'heading_deg':5,'sample_spacing_m':.15,'heading_step_deg':25,'steering_step_deg':15,'steering_step_tolerance_rad':1e-6,'right_band_normalized':[.125,.375],'controlled_band_min':.8},
  'methods':{'steering':'atan(0.50 * wrap(yaw[i+1]-yaw[i]) / chord[i]) before legacy annotation clipping; also repeat with signed circular-arc curvature',
   'replay':'Original PathAuditor run on a deep copy with original steering, then on same positions/yaws with unclipped steering; original files unchanged',
   'curvature':'max of heading-chord circular curvature and 3-point circumcircle curvature; no steering-metadata assumption',
   'body_clearance_lower_bound':'exact EDT on physical+semantic unsafe cell centers; subtract sqrt(2)*resolution, body bounding-circle radius, half max dense translation step. Conservative lower bound; not exact body clearance or a newly introduced hard threshold',
   'preference':'frozen common field, 0.025 m arc quadrature; original 5 m endpoint exclusion; 80% threshold applies to controlled scenes, real-map count >=80% is descriptive'},
  'integrity':{'frozen_runtime_changes':[k for k,v in freeze.items() if sha(WORK/k)!=v],'source_files':len(source_manifest),'source_hashes_unchanged':all(sha(k)==v for k,v in source_manifest.items())},
  'prior_acceptance_interpretation':'Historical 120/120 stays recorded; it is not proof of all geometry-derived constraints. Metadata steering step checks masked a shared continuity violation.'}
 json_write(out/'comparison.json',result);json_write(out/'rows.json',rows);json_write(out/'source_manifest.json',source_manifest)
 with (out/'per_query.csv').open('w',newline='') as f:
  writer=csv.DictWriter(f,fieldnames=list(pairs[0]));writer.writeheader();writer.writerows(pairs)
 assert all(r['replay_matches_recorded_original_gate'] for r in rows)
 assert result['integrity']['source_hashes_unchanged'] and not result['integrity']['frozen_runtime_changes']
 print(json.dumps({'arms':arms,'coverage':result['rule_coverage'],'integrity':result['integrity']},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
