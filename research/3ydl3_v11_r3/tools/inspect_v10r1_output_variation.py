"""Explain, retain, and quantify the observed inherited deadline branch variation."""
from pathlib import Path
import json,hashlib,numpy as np
E=Path(__file__).resolve().parents[1];R=E/'report';P=Path(json.loads((E/'TASK.json').read_text())['parent'])
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
d=json.loads((R/'COMPARISON.json').read_text());diff=d['output_differences'];assert len(diff)==1
x=diff[0];assert (x['round'],x['map_id'],x['query_id'])==(1,'mentor_map_20260825_005_4x_area','A2B-05')
assert sha(E/'src/three_yd_v5/curve_l3.py')==sha(P/'src/three_yd_v5/curve_l3.py')
assert sha(E/'config/l3.json')==sha(P/'config/l3.json')
records=[]
for rnd in (1,2,3):
 for arm in ('r0','r1'):
  out=E/'results/paired_final'/f'round{rnd}'/x['map_id']/arm
  row=next(v for v in map(json.loads,(out/'runs.jsonl').read_text().splitlines()) if v['query_id']==x['query_id'])
  path=out/'cases'/f"{x['query_id']}_1"/'path.npy';p=np.load(path);curve=row['curve_l3']
  assert row['success'] and row['audit']['valid'] and curve['certificate']['max_curvature_bound_1pm']<=2.5
  records.append({'round':rnd,'arm':arm,'path_sha256':sha(path),'path_points':len(p),'length_m':float(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1).sum()),'max_curvature_bound_1pm':curve['certificate']['max_curvature_bound_1pm'],'quadratic_initialization':curve['quadratic_initialization'],'strict_success':row['success'],'original_audit_valid':row['audit']['valid']})
assert records[0]['quadratic_initialization']['certificate_failure']=='CURVE_BUDGET'
assert records[0]['quadratic_initialization']['wall_ms']>50
assert all(v['path_sha256']==records[1]['path_sha256'] for v in records[1:])
assert all(v['quadratic_initialization']['selected'] for v in records[1:])
length_change_pct=(records[1]['length_m']/records[0]['length_m']-1)*100
result={'map_id':x['map_id'],'query_id':x['query_id'],'paired_difference_round':1,'observed_reason':'Unchanged 50 ms quadratic candidate certification deadline triggered CURVE_BUDGET only in r0 round 1, selecting its existing SQP fallback. The other five paths, including r0 rounds 2/3, are byte-identical. Timing-sensitive branch is inherited; no claim of a new trajectory algorithm or bitwise determinism.','source_and_configuration_unchanged':True,'all_six_strictly_valid':True,'length_change_pct_in_differing_pair':length_change_pct,'records':records,'performance_policy':'Retain all 300 pairs unchanged; do not replace the differing run or exclude it from primary statistics.'}
(R/'OUTPUT_VARIATION.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False))
