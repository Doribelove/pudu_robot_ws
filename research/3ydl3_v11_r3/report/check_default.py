from pathlib import Path
import json
R=Path(__file__).resolve().parents[1]
checks=[]
for name,exact in [('default_exact',True),('default_tolerant',False)]:
 p=R/'results'/name;j=json.loads(p.with_suffix('.supervisor.json').read_text())
 assert j['returncode']==0 and j['cleanup']['complete']
 rows=[json.loads(x) for x in (p/'runs.jsonl').read_text().splitlines()];assert len(rows)==1
 r=rows[0];assert r['success'] and r['exact_endpoint_success']==exact and r['tolerant_endpoint_success']==(not exact)
 out=json.loads((p/'cases'/f"{r['query_id']}_1"/'outcome.json').read_text())
 assert out['diagnostics']['costmap_update_acknowledged'] and out['r11_request_heap']['included_in_navigation']
 assert r['curve_l3']['certificate']['max_curvature_bound_1pm']<=2.5+1e-8
 checks.append({'path':str(p),'query_id':r['query_id'],'exact_success':exact,'tolerant_success':not exact,'navigation_s':r['wall_ms']/1000,'heap_policy_in_navigation':True,'cleanup_complete':True})
(R/'report/DEFAULT_ENTRY_CHECK.json').write_text(json.dumps({'complete':True,'checks':checks},ensure_ascii=False,indent=2)+'\n')
print(json.dumps(checks,indent=2))
