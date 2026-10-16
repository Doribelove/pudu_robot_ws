"""Review immutable sources, raw records, map equivalence and path variations."""
from pathlib import Path
import json,hashlib,numpy as np
E=Path(__file__).resolve().parents[1];P=Path(json.loads((E/'TASK.json').read_text())['parent']);R=E/'report';F=E/'results/paired_final'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
parent=json.loads((E/'PARENT_MANIFEST.json').read_text());assert sha(P/'DELIVERY_MANIFEST.json')==parent['manifest_sha256']
for name,h in parent['files'].items():assert sha(P/name)==h,('PARENT_CHANGED',name)
lock=json.loads((E/'L1_LOCK.json').read_text());locked={}
for group in ('source_files','input_files','pixel_input_files'):
 for name,h in lock[group].items():assert sha(Path(name) if Path(name).is_absolute() else E/name)==h,(group,name)
 locked[group]=len(lock[group])
assert sha(E/'MAPS.json')==sha(P/'MAPS.json')
maps=json.loads((E/'MAPS.json').read_text())['maps']
for m in maps:
 for name,h in m['input_sha256'].items():assert sha(name)==h,name
core=json.loads((F/'source.json').read_text())
for name,h in core.items():assert sha(name)==h,('FORMAL_SOURCE_CHANGED',name)
changed=[str(p.relative_to(P)) for p in (P/'src').rglob('*.py') if sha(p)!=sha(E/p.relative_to(P))]
assert set(changed)=={'src/three_yd_v5/curve_backend.py','src/three_yd_v5/curve_session.py'},changed
f='src/three_yd_v5/curve_backend.py';assert (E/f).read_text().replace('3YDL3-V10-r3','3YDL3-V10-r1').replace('3ydl3_v10_r2','3ydl3_v10_r1')==(P/f).read_text()
assert sha(E/'config/l3.json')==sha(P/'config/l3.json')
for p in (P/'native').rglob('*'):
 if p.is_file():assert sha(p)==sha(E/p.relative_to(P))
proof=json.loads((R/'PREPARATION_EQUIVALENCE.json').read_text());assert len(proof)==5 and all(v['all_world_cells_topology_and_views_equal'] for v in proof)
assert '749 passed' in (R/'tests_final.log').read_text()
comparison=json.loads((R/'COMPARISON.json').read_text());assert comparison['complete'] and not comparison['regressions']
quality=[];differences=[];semantic_equal=[];clean=[]
for rnd in (1,2,3):
 for m in maps:
  base=F/f'round{rnd}'/m['map_id'];rows={}
  for arm in ('r1','r2'):
   out=base/arm;rows[arm]=[json.loads(x) for x in (out/'runs.jsonl').read_text().splitlines()]
   sup=json.loads(out.with_suffix('.supervisor.json').read_text());assert sup['returncode']==0 and sup['cleanup']['complete'];clean.append(str(out))
   snap=json.loads((out/'source_snapshot/hashes.json').read_text())
   for name,h in snap.items():
    rel=str(Path(name).relative_to(P if arm=='r1' else E));assert sha(out/'source_snapshot'/rel.replace('/','_'))==h
  if m['kind']!='map_only':
   with np.load(next((base/'r1/world').glob('*.npz')),allow_pickle=False) as a,np.load(next((base/'r2/world').glob('*.npz')),allow_pickle=False) as b:
    assert set(a.files)==set(b.files)
    assert all(np.array_equal(a[name],b[name]) for name in a.files)
   semantic_equal.append({'round':rnd,'map_id':m['map_id'],'labels_hard_safe_equal':True})
   construction=json.loads((base/'r2/raster_storage.json').read_text());assert construction['enabled'] and construction['safe_predicate']['direct']
  for old,new in zip(rows['r1'],rows['r2']):
   assert old['query_id']==new['query_id']
   if not old['success']:continue
   assert new['success'];qid=old['query_id']
   for name in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json'):
    assert sha(base/'r1/cases'/f'{qid}_1'/name)==sha(base/'r2/cases'/f'{qid}_1'/name),(rnd,m['map_id'],qid,name)
   for key in ('semantic_expected_grid_hash','semantic_expected_master_hash','semantic_expected_server_content_hash','server_costmap_content_hash','r5_native_shape','r5_source_full_grid_hash','r5_source_full_master_hash','r5_source_full_mask_hash'):
    assert old['l3'][key]==new['l3'][key],(rnd,m['map_id'],qid,key)
   for row in (old,new):
    assert row['audit']['valid'] and row['curve_l3']['certificate']['max_curvature_bound_1pm']<=2.50000001
   item={'round':rnd,'map_id':m['map_id'],'query_id':qid,'r1_path_sha256':sha(base/'r1/cases'/f'{qid}_1/path.npy'),'r2_path_sha256':sha(base/'r2/cases'/f'{qid}_1/path.npy')}
   quality.append(item)
   if item['r1_path_sha256']!=item['r2_path_sha256']:
    item.update(r1_quadratic=old['curve_l3'].get('quadratic_initialization'),r2_quadratic=new['curve_l3'].get('quadratic_initialization'),r1_length_m=old['length_m'],r2_length_m=new['length_m'],length_change_pct=(new['length_m']/old['length_m']-1)*100,r1_curvature_bound=old['curve_l3']['certificate']['max_curvature_bound_1pm'],r2_curvature_bound=new['curve_l3']['certificate']['max_curvature_bound_1pm'],both_strict=True)
    differences.append(item)
assert len(quality)==282
event_journals=list((E/'results/events').glob('*.supervisor.json'));assert len(event_journals)==2
for p in event_journals:
 j=json.loads(p.read_text());assert j['returncode']==0 and j['cleanup']['complete']
(R/'OUTPUT_VARIATIONS.json').write_text(json.dumps(differences,ensure_ascii=False,indent=2))
result={'version':'3YDL3-V10-r3','parent_registered_files_unchanged':len(parent['files']),'frozen_L1':locked,'changed_inherited_sources':changed,'configuration_and_native_libraries_unchanged':True,'original_curve_optimizer_continuous_certificate_and_strict_audits_unchanged':True,'formal_locked_files':len(core),'formal_clean_batches':len(clean),'event_clean_batches':len(event_journals),'strict_r1':comparison['r1_strict'],'strict_r2':comparison['r2_strict'],'common_success_L1_L2_soft_and_effective_costmap_equal':len(quality),'common_success_path_equal':len(quality)-len(differences),'output_variation_count':len(differences),'semantic_raw_equivalent_batches':semantic_equal,'preparation_graph_equivalence_maps':len(proof),'preparation_equivalence_cells':sum(v['cells'] for v in proof),'project_tests':749,'static_only':True}
(R/'SOURCE_REVIEW.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False))
