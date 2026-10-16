from pathlib import Path
import json,collections
import numpy as np
E=Path(__file__).resolve().parents[1];F=E/'results/paired_final';R=E/'report';records=collections.defaultdict(dict)
for p in F.glob('round*/*/*/runs.jsonl'):
 arm=p.parent.name
 if arm not in ('v9','v10'):continue
 rnd=p.parents[2].name;mid=p.parents[1].name
 for x in p.read_text().splitlines():
  r=json.loads(x);records[(mid,r['query_id'],rnd)][arm]=r
assert len(records)==300 and all(set(x)=={'v9','v10'} for x in records.values())
failed=collections.defaultdict(list);clusters=collections.defaultdict(list);length=[];initial=collections.Counter();l1=collections.Counter();l2=collections.Counter()
for (mid,q,rnd),v in records.items():
 for arm,r in v.items():
  p=F/rnd/mid/arm/'cases'/f'{q}_1'
  l1[arm]+=(p/'l1_output.json').exists();l2[arm]+=(p/'l2_reference_xy.npy').exists()
 if not v['v10']['success']:failed[(mid,q)].append(v)
 if all(v[a]['success'] for a in v):
  clusters[(mid,q)].append(v)
  length.append((v['v10']['length_m']/v['v9']['length_m']-1)*100)
 initial['accepted_deliveries']+=bool(v['v10']['success'] and v['v10'].get('curve_l3',{}).get('accepted_quadratic_initialization'))
 for a in v['v10'].get('l3_backend',{}).get('attempts',[]):
  if a.get('quadratic_initialization'):initial['trials']+=1
  if a.get('accepted_quadratic_initialization'):initial['geometric_candidates']+=1
  if a.get('quadratic_strict_fallback'):initial['strict_fallbacks']+=1
fail=[]
for (mid,q),vs in sorted(failed.items()):
 values={a:[v[a]['wall_ms']/1000 for v in vs] for a in ['v9','v10']}
 fail.append({'map_id':mid,'query_id':q,'n':len(vs),'v9_median_s':float(np.median(values['v9'])),'v10_median_s':float(np.median(values['v10'])),'change_pct':float((np.median(values['v10'])/np.median(values['v9'])-1)*100),'v9_failure_codes':list({v['v9']['failure_code'] for v in vs}),'v10_failure_codes':list({v['v10']['failure_code'] for v in vs})})
# Each Q's three rounds are sampled as one correlated cluster. Maps stay fixed.
rng=np.random.default_rng(20261012);by_map=collections.defaultdict(list)
for (mid,q),vs in sorted(clusters.items()):
 assert len(vs)==3
 by_map[mid].append(vs)
intervals={}
for metric in ['wall_ms','rss_peak_bytes']:
 samples=[]
 for _ in range(3000):
  paired=[]
  for mid in sorted(by_map):
   groups=by_map[mid]
   for idx in rng.integers(0,len(groups),len(groups)):paired.extend(groups[idx])
  samples.append((np.median([v['v10'][metric] for v in paired])/np.median([v['v9'][metric] for v in paired])-1)*100)
 intervals[metric]={'change_pct_95pct_interval':np.quantile(samples,[.025,.975]).tolist(),'method':'paired query-cluster bootstrap, 3000 resamples, fixed maps; three correlated rounds remain together','not_evidence_for_unseen_maps':True}
result={'failure_queries':fail,'bootstrap':intervals,'saved_L1_outputs':l1,'saved_L2_references':l2,'L1_artifact_presence_is_not_strict_success':True,'quadratic_initialization':initial,'path_length_change_pct':{'min':float(np.min(length)),'median':float(np.median(length)),'max':float(np.max(length))}}
(R/'DETAILS.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False))
