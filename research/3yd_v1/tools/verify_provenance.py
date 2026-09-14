import hashlib,json,datetime,xml.etree.ElementTree as ET
from pathlib import Path
W=Path('/home/robot/workspaces/semantic_dual_map_r1_20260914');O=Path('/home/robot/pudu_robot_ws')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
m=json.loads((W/'SOURCE_MANIFEST.json').read_text());changed=[]
for f in m['files']:
 p=Path(f['source_resolved'])
 if not p.exists() or sha(p)!=f['sha256']:changed.append(str(p))
core=['r1_pipeline.py','l2_state_lifecycle.py','production_l1.py','pipeline.py','dynamic_policy.py','l2_incremental.py'];rel=Path('external/arena4_ws/src/arena/three_d_v1');pairs={}
for name in core:
 p=rel/'arena_3d_v1'/name;pairs[str(p)]={'source_sha256':sha(O/p),'isolated_sha256':sha(W/p),'equal':sha(O/p)==sha(W/p)}
p=rel/'config/three_d_v1_r1_l2_lifecycle.yaml';pairs[str(p)]={'source_sha256':sha(O/p),'isolated_sha256':sha(W/p),'equal':sha(O/p)==sha(W/p)}
root=ET.parse(W/'results/final_unit_regression.xml').getroot();tests=[{'class':x.attrib.get('classname'),'name':x.attrib['name'],'passed':x.find('failure') is None and x.find('error') is None and x.find('skipped') is None} for x in root.iter('testcase')]
freeze=json.loads((W/'config/FREEZE.json').read_text());inputs={}
for name,key in [('acceptance.yaml','acceptance_sha256'),('queries.yaml','queries_sha256')]:inputs[name]={'actual':sha(W/'config'/name),'frozen':freeze[key],'equal':sha(W/'config'/name)==freeze[key]}
x={'verified_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'original_manifest_files_checked':len(m['files']),'original_changed_files':changed,'original_source_unchanged':not changed,'frozen_r1_core':pairs,'frozen_inputs':inputs,'unit_tests':tests,'all_unit_tests_passed':all(t['passed'] for t in tests)}
(W/'report/provenance_verification.json').write_text(json.dumps(x,ensure_ascii=False,indent=2));print({k:v for k,v in x.items() if k not in ['unit_tests','frozen_r1_core','frozen_inputs']})
