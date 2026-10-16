from pathlib import Path
import json,hashlib,ast,collections,re
import numpy as np
E=Path(__file__).resolve().parents[1];P=Path(json.loads((E/'TASK.json').read_text())['parent']);R=E/'report';F=E/'results/paired_final'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest=json.loads((E/'PARENT_MANIFEST.json').read_text());assert sha(P/'DELIVERY_MANIFEST.json')==manifest['delivery_manifest_sha256']
for p,h in manifest['files'].items():assert sha(P/p)==h,('PARENT_CHANGED',p)
locked={};l=json.loads((E/'L1_LOCK.json').read_text())
for group in ('source_files','input_files','pixel_input_files'):
 for p,h in l[group].items():assert sha(Path(p) if Path(p).is_absolute() else E/p)==h,(group,p)
 locked[group]=len(l[group])
assert sha(E/'MAPS.json')==sha(P/'MAPS.json')
for m in json.loads((E/'MAPS.json').read_text())['maps']:
 for p,h in m['input_sha256'].items():assert sha(p)==h,p
changed_sources=[];unchanged_sources=0
for oldfile in (P/'src').rglob('*.py'):
 rel=oldfile.relative_to(P)
 if sha(oldfile)!=sha(E/rel):changed_sources.append(str(rel))
 else:unchanged_sources+=1
assert set(changed_sources)=={'src/three_yd_v3/dynamic.py','src/three_yd_v5/controller.py','src/three_yd_v5/curve_backend.py','src/three_yd_v5/curve_l3.py','src/three_yd_v5/curve_native.py','src/three_yd_v5/fullroute_service.py'},changed_sources
source=json.loads((F/'source.json').read_text())
for p,h in source.items():assert sha(p)==h,('FINAL_SOURCE_CHANGED',p)
config=json.loads((E/'config/l3.json').read_text());old=json.loads((P/'config/l3.json').read_text())
exclude={'schema','certified_quadratic_start','trust_region_constraint_screen'}
assert {k:v for k,v in config.items() if k not in exclude}=={k:v for k,v in old.items() if k not in exclude}
def node(base,file,name):
 tree=ast.parse((base/file).read_text())
 return next(x for x in ast.walk(tree) if isinstance(x,(ast.FunctionDef,ast.AsyncFunctionDef)) and x.name==name)
def same_function(file,name):assert ast.dump(node(E,file,name))==ast.dump(node(P,file,name)),(file,name)
functions=[]
for file,names in [('src/three_yd_v5/curve_l3.py',['rectangles','differential','footprint_constraints','violation','bezier_pieces','split_bezier','knot_vector']),('src/three_yd_v5/controller.py',['_audit','_activate_reference','_target_mask','_prepare_grid_cached','_set_reference']),('src/three_yd_v3/dynamic.py',['apply','blocked_cells']),('src/three_yd_v5/fullroute_service.py',['update']),('src/three_yd_v5/curve_backend.py',['validate_record','displaced_continuation'])]:
 for name in names:same_function(file,name);functions.append(file+':'+name)
def bounds_loop(base):
 cert=node(base,'src/three_yd_v5/curve_l3.py','certify')
 return next(x for x in ast.walk(cert) if isinstance(x,ast.For) and isinstance(x.target,ast.Name) and x.target.id=='depth')
assert ast.dump(bounds_loop(E))==ast.dump(bounds_loop(P))
assert sha(E/'src/three_yd_v5/curve_qp.py')==sha(P/'src/three_yd_v5/curve_qp.py')
native=[]
for p in (P/'native').rglob('*'):
 if p.is_file():assert sha(p)==sha(E/p.relative_to(P));native.append(str(p.relative_to(P)))
quality=[];successful_baseline=0;success_new=0;early=0;fallback=0;rowcounts=[]
for rnd in range(1,4):
 for m in json.loads((E/'MAPS.json').read_text())['maps']:
  paths={a:F/f'round{rnd}'/m['map_id']/a for a in ('v9','v10')};rr={a:[json.loads(x) for x in (p/'runs.jsonl').read_text().splitlines()] for a,p in paths.items()}
  history={x['query_id']:x['success'] for x in map(json.loads,(P/'results/formal/round1'/m['map_id']/'runs.jsonl').read_text().splitlines())}
  for a,p in paths.items():
   snap=json.loads((p/'source_snapshot/hashes.json').read_text())
   for original,h in snap.items():
    q=Path(original);flat=str(q.relative_to(P if a=='v9' else E)).replace('/','_')
    assert sha(p/'source_snapshot'/flat)==h
    if str(q) in source:assert source[str(q)]==h
  for old,new in zip(rr['v9'],rr['v10']):
   if history[new['query_id']]:successful_baseline+=1;assert new['success'],('HISTORICAL_SUCCESS_LOST',m['map_id'],new['query_id'])
   success_new+=new['success']
   early+=bool(new['success'] and new.get('curve_l3',{}).get('accepted_quadratic_initialization'))
   for attempt in new.get('l3_backend',{}).get('attempts',[]):
    fallback+=bool(attempt.get('quadratic_strict_fallback'))
    for it in attempt.get('iterations',[]):rowcounts.append([it.get('original_geometric_constraint_rows',it['geometric_constraint_rows']),it['geometric_constraint_rows']])
   if old['success'] and new['success']:
    q={'round':rnd,'map_id':m['map_id'],'query_id':new['query_id'],'length_v9_m':old['length_m'],'length_v10_m':new['length_m'],'length_change_pct':(new['length_m']/old['length_m']-1)*100}
    for a,r in [('v9',old),('v10',new)]:
     q[a+'_curvature_bound_1pm']=r['curve_l3']['certificate']['max_curvature_bound_1pm']
     assert q[a+'_curvature_bound_1pm']<=2.50000001
    for key in ('semantic_expected_grid_hash','semantic_expected_master_hash','semantic_expected_server_content_hash','server_costmap_content_hash','r5_native_shape','r5_source_full_grid_hash','r5_source_full_master_hash','r5_source_full_mask_hash'):
     assert old['l3'][key]==new['l3'][key],('EFFECTIVE_MAP_CHANGED',m['map_id'],new['query_id'],key)
    for file in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json'):
     assert sha(paths['v9']/'cases'/f"{new['query_id']}_1"/file)==sha(paths['v10']/'cases'/f"{new['query_id']}_1"/file)
    quality.append(q)
cleanup=[]
for p in (E/'results').rglob('*.supervisor.json'):
 d=json.loads(p.read_text());c=d.get('cleanup',d.get('recovered_cleanup',{}));assert c.get('complete'),p
 cleanup.append({'journal':str(p),'complete':True,'recovered':bool(d.get('recovered_cleanup')),'returncode':d.get('returncode'),'formal':str(F) in str(p)})
tests=(R/'tests_final.log').read_text();assert '731 passed' in tests
result={'version':'3YDL3-V10-r0','parent_files_unchanged':len(manifest['files']),'changed_inherited_source_files':changed_sources,'unchanged_inherited_source_files':unchanged_sources,'L1_LOCK':locked,'native_files_identical':native,'current_source_locked_files':len(source),'unchanged_algorithm_functions':functions,'continuous_certificate_depth_loop_AST_identical':True,'same_original_QP_solver':True,'common_success_effective_maps_exactly_identical':True,'fixed_poses_and_strict_ACK_checks':'COMPARISON.json','historical_successes_preserved':successful_baseline,'strict_successes':success_new,'certified_quadratic_deliveries':early,'strict_rejection_fallbacks':fallback,'linear_constraints_original_vs_retained':np.sum(rowcounts,axis=0).tolist(),'quality':quality,'max_path_length_change_pct':max(x['length_change_pct'] for x in quality),'min_path_length_change_pct':min(x['length_change_pct'] for x in quality),'cleanup':cleanup,'tests':'731 passed; one inherited NumPy warning','static_only':True}
(R/'SOURCE_REVIEW.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print({k:v for k,v in result.items() if k not in ('quality','cleanup','native_files_identical','unchanged_algorithm_functions')})
