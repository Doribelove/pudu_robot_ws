from pathlib import Path
import json,hashlib,ast,numpy as np
E=Path(__file__).resolve().parents[1];P=Path(json.loads((E/'TASK.json').read_text())['parent']);R=E/'report';F=E/'results/paired_final';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
m=json.loads((E/'PARENT_MANIFEST.json').read_text());assert sha(P/'DELIVERY_MANIFEST.json')==m['manifest_sha256']
for name,h in m['files'].items():assert sha(P/name)==h,('PARENT_CHANGED',name)
lock=json.loads((E/'L1_LOCK.json').read_text());locked={}
for group in ('source_files','input_files','pixel_input_files'):
 for name,h in lock[group].items():assert sha(Path(name) if Path(name).is_absolute() else E/name)==h,(group,name)
 locked[group]=len(lock[group])
assert sha(E/'MAPS.json')==sha(P/'MAPS.json')
for item in json.loads((E/'MAPS.json').read_text())['maps']:
 for name,h in item['input_sha256'].items():assert sha(name)==h,name
source=json.loads((F/'source.json').read_text())
for name,h in source.items():assert sha(name)==h,name
changed=[str(p.relative_to(P)) for p in (P/'src').rglob('*.py') if sha(p)!=sha(E/p.relative_to(P))]
assert set(changed)=={'src/three_yd_v5/controller.py','src/three_yd_v5/curve_backend.py'},changed
old=(P/'src/three_yd_v5/controller.py').read_text();new=(E/'src/three_yd_v5/controller.py').read_text();assert new.replace('first_component_break(point_raw,(circle_view.safe|opened)&mask,circle_view,certificates)','first_component_break(point_raw,circle_view.safe|opened,circle_view,certificates)')==old
name='src/three_yd_v5/curve_backend.py';assert (E/name).read_text().replace('3YDL3-V10-r1','3YDL3-V10-r0').replace('3ydl3_v10_r1','3ydl3_v10_r0')==(P/name).read_text()
assert sha(E/'config/l3.json')==sha(P/'config/l3.json')
for p in (P/'native').rglob('*'):
 if p.is_file():assert sha(p)==sha(E/p.relative_to(P))
equality=[];quality=[];cleanup=[];historical_success=0;new_success=0;path_identical=0
variation=json.loads((R/'OUTPUT_VARIATION.json').read_text());assert variation['all_six_strictly_valid'] and variation['source_and_configuration_unchanged']
for rnd in range(1,4):
 for item in json.loads((E/'MAPS.json').read_text())['maps']:
  mid=item['map_id'];base=F/f'round{rnd}'/mid;rows={}
  for arm in ('r0','r1'):
   out=base/arm;rows[arm]=[json.loads(x) for x in (out/'runs.jsonl').read_text().splitlines()];sup=json.loads(out.with_suffix('.supervisor.json').read_text());assert sup['returncode']==0 and sup['cleanup']['complete'];cleanup.append(str(out))
   snap=json.loads((out/'source_snapshot/hashes.json').read_text())
   for name,h in snap.items():
    rel=str(Path(name).relative_to(P if arm=='r0' else E));assert sha(out/'source_snapshot'/rel.replace('/','_'))==h
  if item['kind']!='map_only':
   a=np.load(next((base/'r0/world').glob('*.npz')),allow_pickle=False);b=np.load(next((base/'r1/world').glob('*.npz')),allow_pickle=False)
   assert set(a.files)==set(b.files)
   exact={name:bool(np.array_equal(a[name],b[name])) for name in a.files};assert all(exact.values());equality.append({'map_id':mid,'round':rnd,'semantic_world_arrays':exact});a.close();b.close()
  for old,new in zip(rows['r0'],rows['r1']):
   assert old['query_id']==new['query_id'];historical_success+=old['success'];new_success+=new['success']
   if not old['success']:continue
   assert new['success'];qid=old['query_id']
   for f in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json'):
    assert sha(base/'r0/cases'/f'{qid}_1'/f)==sha(base/'r1/cases'/f'{qid}_1'/f),(mid,qid,f)
   identical=sha(base/'r0/cases'/f'{qid}_1/path.npy')==sha(base/'r1/cases'/f'{qid}_1/path.npy');path_identical+=identical
   if not identical:
    assert (rnd,mid,qid)==(variation['paired_difference_round'],variation['map_id'],variation['query_id'])
    for arm,record in zip(('r0','r1'),variation['records'][:2]):assert sha(base/arm/'cases'/f'{qid}_1/path.npy')==record['path_sha256']
   for key in ('semantic_expected_grid_hash','semantic_expected_master_hash','semantic_expected_server_content_hash','server_costmap_content_hash','r5_native_shape','r5_source_full_grid_hash','r5_source_full_master_hash','r5_source_full_mask_hash'):
    assert old['l3'][key]==new['l3'][key],(mid,qid,key)
   assert new['curve_l3']['certificate']['max_curvature_bound_1pm']<=2.50000001
   quality.append({'round':rnd,'map_id':mid,'query_id':qid,'path_hash':sha(base/'r1/cases'/f'{qid}_1/path.npy')})
assert historical_success==282 and new_success>=historical_success
assert path_identical==281
assert '738 passed' in (R/'tests_final.log').read_text()
gates=json.loads((R/'AUDIT_GATE_EQUIVALENCE.json').read_text());assert len(gates)==10 and all(v['all_original_audit_gate_decisions_equal'] for v in gates)
assert len(json.loads((F/'status.json').read_text()))==60
bridge_diagnostics=[]
for rnd in range(1,4):
 data={}
 for arm in ('r0','r1'):
  files=list((F/f'round{rnd}'/'mentor_map_20260825_005'/arm/'l2_cache').glob('failed_reference_*.json'));assert len(files)==1
  data[arm]=json.loads(files[0].read_text())['activation_attempts'][-1]['first_component_break']
 assert not data['r0']['found'] and data['r1']['found']
 bridge_diagnostics.append({'round':rnd,**data})
(R/'BRIDGE_DIAGNOSTIC_COMPARISON.json').write_text(json.dumps(bridge_diagnostics,ensure_ascii=False,indent=2))
event_journals=list((E/'results/events').glob('*.supervisor.json'));assert len(event_journals)==2
for p in event_journals:
 j=json.loads(p.read_text());assert j['returncode']==0 and j['cleanup']['complete']
report={'version':'3YDL3-V10-r1','parent_registered_files_unchanged':len(m['files']),'frozen_L1':locked,'changed_inherited_sources':changed,'controller_change':'Only disconnect diagnostics now AND the actual L2 mask; original planning and acceptance code unchanged','curve_backend_change':'Version identity strings only','curve_configuration_unchanged':True,'all_original_strict_audits_and_continuous_certifier_unchanged':True,'native_libraries_unchanged':True,'formal_source_locked_files':len(source),'formal_clean_batches':len(cleanup),'event_clean_batches':len(event_journals),'strict_success_parent':historical_success,'strict_success_candidate':new_success,'common_success_L1_L2_soft_output_hashes_equal':len(quality),'semantic_world_equivalence':equality,'audit_gate_equivalence_maps':len(gates),'audit_gate_equivalence_cells':sum(v['cells'] for v in gates),'tests':738,'static_only':True}
report.update(common_success_L1_L2_soft_output_hashes_equal=len(quality),common_success_final_path_hashes_equal=path_identical,inherited_deadline_path_variation=str(R/'OUTPUT_VARIATION.json'))
(R/'SOURCE_REVIEW.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False))
