"""Paired first-navigation metrics; fixed historical success cohort."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/home/robot/workspaces/3ydl3_v10_r2_firstready_20261012')


def summarize(pairs):
    result={'n':len(pairs)}
    for field,label,scale in [('wall_ms','navigation_s',1000),('rss_peak_bytes','tree_rss_mib',2**20),
                             ('tree_cpu_ms','tree_cpu_s',1000)]:
        vals={a:np.array([p[a][field]/scale for p in pairs]) for a in ('r2','r3')}
        result[label]={a:{'median':float(np.median(v)), 'p95':float(np.percentile(v,95)),
                          'max':float(max(v))} for a,v in vals.items()}
        result[label]['median_reduction_percent']=100*(1-np.median(vals['r3'])/np.median(vals['r2']))
        result[label]['p95_reduction_percent']=100*(1-np.percentile(vals['r3'],95)/np.percentile(vals['r2'],95))
    return result


def analyze(folder):
    maps=json.loads((ROOT/'MAPS.json').read_text())['maps']
    historical=set()
    for f in (PARENT/'results/paired_final/round1').glob('*/r2/runs.jsonl'):
        for r in map(json.loads,f.read_text().splitlines()):
            if r['success']:historical.add((r['map_id'],r['query_id']))
    assert len(historical)==94
    records={};flat=[];hashes={};clean=[]
    for f in sorted(folder.glob('round*/*/r*/runs.jsonl')):
        rnd,mid,arm=f.parts[-4:-1]
        hashes[str(f)]=hashlib.sha256(f.read_bytes()).hexdigest()
        for r in map(json.loads,f.read_text().splitlines()):
            key=(rnd,mid,r['query_id'])
            assert arm not in records.setdefault(key,{})
            records[key][arm]=r
            flat.append({'round':rnd,'map_id':mid,'query_id':r['query_id'],'arm':arm,
                'strict_success':r['success'],'navigation_s':r['wall_ms']/1000,
                'tree_rss_peak_mib':r['rss_peak_bytes']/2**20,
                'tree_cpu_s':r['tree_cpu_ms']/1000,'failure_code':r['failure_code']})
        journal=f.parent.with_suffix('.supervisor.json')
        if journal.exists():
            j=json.loads(journal.read_text())
            if j.get('returncode')==0 and j.get('cleanup',{}).get('complete'):clean.append(str(f.parent))
    complete=len(flat)==600 and len(clean)==60 and (folder/'COMPLETE.json').exists()
    all_pairs=[dict(key=k,**v) for k,v in records.items() if len(v)==2]
    pairs=[p for p in all_pairs if p['key'][1:] in historical and p['r2']['success'] and p['r3']['success']]
    regressions=[p['key'] for p in all_pairs if p['key'][1:] in historical and not p['r3']['success']]
    new_success=[p['key'] for p in all_pairs if p['key'][1:] not in historical and p['r3']['success']]
    per_map=[]
    for m in maps:
        subset=[p for p in pairs if p['key'][1]==m['map_id']]
        if not subset:continue
        per_map.append({'map_id':m['map_id'],'name':m['name'],**summarize(subset),
            'strict':{a:sum(r['strict_success'] for r in flat if r['map_id']==m['map_id'] and r['arm']==a) for a in ('r2','r3')}})
    overall=summarize(pairs) if pairs else {}
    equal_weight={metric:float(100*(1-np.exp(np.mean([np.log(m[metric]['r3']['median']/m[metric]['r2']['median']) for m in per_map]))))
                  for metric in ('navigation_s','tree_rss_mib')} if per_map else {}
    failures=[]
    for mid,qid in sorted({p['key'][1:] for p in all_pairs if not(p['r2']['success'] and p['r3']['success'])}):
        group=[p for p in all_pairs if p['key'][1:]==(mid,qid)]
        failures.append({'map_id':mid,'query_id':qid,'arms':{a:{'success':sum(p[a]['success'] for p in group),
            'codes':[p[a]['failure_code'] for p in group],'median_s':float(np.median([p[a]['wall_ms']/1000 for p in group]))} for a in ('r2','r3')}})
    output={'complete':complete,'request_count':len(flat),'paired_requests':len(all_pairs),
        'strict':{a:sum(r['strict_success'] for r in flat if r['arm']==a) for a in ('r2','r3')},
        'historical_success_queries':94,'common_historical_success_requests':len(pairs),
        'regressions':regressions,'new_success':new_success,'overall':overall,
        'equal_map_weight_reduction_percent':equal_weight,'per_map':per_map,'failures':failures,
        'raw_files_sha256':hashes,'clean_batches':len(clean),
        'scope':'first request for each own fixed Q per round; whole request with real ACK and strict audits; no map-load/startup/export time; whole planner tree RSS includes resident maps'}
    if pairs:
        output['gates']={'complete':complete,'original_success_preserved':len(pairs)==282 and not regressions,
            'navigation_median_minus_5pct':overall['navigation_s']['median_reduction_percent']>=5,
            'tree_rss_median_minus_5pct':overall['tree_rss_mib']['median_reduction_percent']>=5,
            'equal_map_navigation_minus_5pct':equal_weight['navigation_s']>=5,
            'equal_map_tree_rss_minus_5pct':equal_weight['tree_rss_mib']>=5,
            'navigation_p95_nonregression':overall['navigation_s']['p95_reduction_percent']>=0}
        output['accepted']=all(output['gates'].values())
    folder.mkdir(exist_ok=True)
    (folder/'COMPARISON.json').write_text(json.dumps(output,ensure_ascii=False,indent=2,default=lambda x:x.item())+'\n')
    if flat:
        with (folder/'requests.csv').open('w') as stream:
            w=csv.DictWriter(stream,fieldnames=list(flat[0]));w.writeheader();w.writerows(flat)
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);a=p.parse_args()
    out=analyze(a.folder)
    print(json.dumps({k:v for k,v in out.items() if k not in ('raw_files_sha256','per_map','failures')},ensure_ascii=False,indent=2,default=lambda x:x.item()))
