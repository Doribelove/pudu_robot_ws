"""One-round paired diagnostic; never a formal acceptance replacement."""
from pathlib import Path
import json,hashlib
import numpy as np
R=Path(__file__).resolve().parents[1];D=R/'results/tenmap_pilot'
def rows(p):return [json.loads(s) for s in p.read_text().splitlines()]
allrows={'control':[],'candidate':[]};bymap={};differences=[]
assert json.loads((D/'COMPLETE.json').read_text())['complete']
for m in json.loads((R/'MAPS.json').read_text())['maps']:
 mid=m['map_id'];pair={}
 for arm in allrows:
  p=D/mid/arm;j=json.loads(p.with_suffix('.supervisor.json').read_text());assert j['cleanup']['complete'] and j['returncode']==0
  data=rows(p/'runs.jsonl');assert len(data)==10 and {r['query_id'] for r in data}==set(m['query_ids'])
  allrows[arm]+=data;pair[arm]=data
 for a,b in zip(pair['control'],pair['candidate']):
  assert a['query_id']==b['query_id']
  assert a['start']==b['start'] and a['goal']==b['goal']
  if a['success']!=b['success'] or a.get('exact_endpoint_success')!=b.get('exact_endpoint_success'):differences.append([mid,a['query_id'],a['success'],b['success'],a.get('exact_endpoint_success'),b.get('exact_endpoint_success')])
 def med(rs,k):return float(np.median([r[k] for r in rs]))
 bymap[mid]={'success':{arm:sum(r['success'] for r in data) for arm,data in pair.items()},'median_change_percent':{k:100*(med(pair['candidate'],k)/med(pair['control'],k)-1) for k in ['wall_ms','rss_peak_bytes']}}
base={(r['map_id'],r['query_id']) for r in allrows['control'] if r['exact_endpoint_success']};assert len(base)==99
cohorts={}
for cohort,predicate in [('original99',lambda r:(r['map_id'],r['query_id']) in base),('all100',lambda r:True)]:
 groups={arm:[r for r in rs if predicate(r)] for arm,rs in allrows.items()};statistics={}
 for k,unit,scale in [('wall_ms','navigation_s',.001),('rss_peak_bytes','tree_rss_mib',1/2**20)]:
  stats={arm:{'n':len(rs),'median':med(rs,k)*scale,'p95':float(np.quantile([r[k]*scale for r in rs],.95)),'max':max(r[k]*scale for r in rs)} for arm,rs in groups.items()}
  statistics[unit]={**stats,'change_percent':{s:100*(stats['candidate'][s]/stats['control'][s]-1) for s in ['median','p95','max']}}
 cohorts[cohort]=statistics
out={'diagnostic_only':True,'formal_acceptance':False,'rounds':1,'maps':bymap,'success':{arm:sum(r['success'] for r in data) for arm,data in allrows.items()},'functional_differences':differences,'cohorts':cohorts}
(R/'report/TENMAP_PILOT.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:out[k] for k in ['success','functional_differences','cohorts']},ensure_ascii=False,indent=2))
