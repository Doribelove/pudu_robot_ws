"""Same-environment r10 ablations, raw phase clocks, and path/ACK parity."""
import json
from pathlib import Path
import numpy as np
from summarize_r9 import summarize
ROOT=Path(__file__).resolve().parents[1]
CONTROL='r9_control_10x3'

def rows(folder):return [json.loads(x) for x in (folder/'runs.jsonl').read_text().splitlines()]
def parity(name,baseline=CONTROL):
    result=[]
    for row in rows(ROOT/'results'/name):
        case=f"{row['query_id']}_{row['repeat']}"
        a=ROOT/'results'/baseline/'cases'/case;b=ROOT/'results'/name/'cases'/case
        checks={}
        for field in ('path.npy','l2_reference_xy.npy'):
            checks[field]=(a/field).exists() and (b/field).exists() and np.array_equal(np.load(a/field),np.load(b/field))
        for file,field in [('l1_output.json','right_reference_polyline'),('l2_soft_target.json','points')]:
            checks[file]=(a/file).exists() and (b/file).exists() and json.loads((a/file).read_text())[field]==json.loads((b/file).read_text())[field]
        result.append({'case':case,**{k:bool(v) for k,v in checks.items()}})
    return result

def main():
    import argparse
    ap=argparse.ArgumentParser();ap.add_argument('names',nargs='+');args=ap.parse_args()
    output={'baseline':CONTROL,'batches':{}}
    for name in args.names:
        rs=rows(ROOT/'results'/name)
        successful=all(r.get('success') for r in rs)
        entry={'runs':len(rs),'success':sum(r.get('success',False) for r in rs)}
        if successful and len(rs)>=3:
            entry.update(summarize(rs))
        entry['parity']=parity(name)
        entry['all_paths_equal']=all(all(v for k,v in p.items() if k!='case') for p in entry['parity'])
        entry['right_metric_equal']=all(
            next(b for b in rows(ROOT/'results'/CONTROL) if (b['query_id'],b['repeat'])==(r['query_id'],r['repeat'])).get('right_side_fraction')==r.get('right_side_fraction') for r in rs)
        entry['actual_maps_acknowledged']=all(r.get('l3',{}).get('costmap_update_acknowledged',False) for r in rs)
        entry['within_mask']=all(r.get('l3',{}).get('returned_path_within_mask',False) for r in rs)
        entry['connector_hits']=sum(r.get('r10',{}).get('connector_delta',{}).get('hits',0) for r in rs)
        entry['connector_calls']=sum(r.get('r10',{}).get('connector_delta',{}).get('calls',0) for r in rs)
        entry['tile_updates']=[t for r in rs for t in r.get('r10',{}).get('tile_updates',[])]
        entry['preflight']=[a.get('r10_preflight') for r in rs for a in r.get('timing',{}).get('activation_attempts',[]) if 'r10_preflight' in a]
        output['batches'][name]=entry
        summary={k:entry[k] for k in ('runs','success','all_paths_equal','right_metric_equal','actual_maps_acknowledged','within_mask','connector_hits')}
        if 'all' in entry:
            summary.update({k:entry['all'][k] for k in ('wall_s','rss_gib')})
            summary.update(first=entry['first']['wall_s'],warm=entry['warm']['wall_s'])
        print(name,json.dumps(summary,ensure_ascii=False))
    (ROOT/'report/PERFORMANCE.json').write_text(json.dumps(output,indent=2,ensure_ascii=False))
if __name__=='__main__':main()
