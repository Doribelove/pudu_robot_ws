"""Raw-data summaries with unchanged geometry and ACK comparison."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]

def read(name):return [json.loads(x) for x in (ROOT/'results'/name/'runs.jsonl').read_text().splitlines()]
def stats(a):
    a=[x for x in a if x is not None]
    if not a:return None
    return {'n':len(a),'median':float(np.median(a)),'p95':float(np.quantile(a,.95)),'max':float(max(a)),'min':float(min(a))}
def summary(rs):
    getters={
      'wall_s':lambda r:r['wall_ms']/1000,'rss_gib':lambda r:r['rss_peak_bytes']/2**30,
      'l1_ms':lambda r:r.get('recovery',{}).get('l1_actual_ms'),
      'l2_all_ms':lambda r:r.get('recovery',{}).get('all_candidates_controller_ms'),
      'adapter_ms':lambda r:r.get('recovery',{}).get('all_candidates_adapter_ms'),
      'l3_all_ms':lambda r:r.get('recovery',{}).get('all_candidates_l3_pipeline_ms'),
      'planner_pss_gib':lambda r:r.get('l3',{}).get('planner_pss_peak_bytes',0)/2**30,
      'stack_pss_gib':lambda r:r.get('l3',{}).get('stack_pss_peak_bytes',0)/2**30,
      'post_export_tree_pss_gib':lambda r:sum(p['pss'] for p in r.get('memory_processes',[]))/2**30,
      'right_fraction':lambda r:r.get('right_side_fraction'),
      'length_m':lambda r:r.get('length_m')}
    result={'runs':len(rs),'strict_success':sum(bool(r['success']) for r in rs),
      'ack_all':all(r.get('l3',{}).get('costmap_update_acknowledged') for r in rs if r['success']),
      'bounds_all':all(r.get('l3',{}).get('returned_path_within_mask') for r in rs if r['success']),
      'failures':[{'query':r['query_id'],'repeat':r['repeat'],'code':r.get('failure_code')} for r in rs if not r['success']]}
    for name,items in [('all',rs),('first',[r for r in rs if r['repeat']==1]),('repeat',[r for r in rs if r['repeat']>1])]:
        result[name]={k:stats([fn(r) for r in items]) for k,fn in getters.items()}
    result['per_round']={str(i):{'time_s':stats([r['wall_ms']/1000 for r in rs if r['repeat']==i]),'post_export_tree_pss_gib':stats([sum(p['pss'] for p in r['memory_processes'])/2**30 for r in rs if r['repeat']==i])} for i in sorted(set(r['repeat'] for r in rs))}
    result['per_query']={q:stats([r['wall_ms']/1000 for r in rs if r['query_id']==q]) for q in sorted(set(r['query_id'] for r in rs))}
    return result

def parity(name,base):
    baseline={(r['query_id'],r['repeat']):r for r in read(base)};records=[]
    for r in read(name):
        b=baseline[(r['query_id'],r['repeat'])];tag=f"{r['query_id']}_{r['repeat']}"
        paths=[ROOT/'results'/v/'cases'/tag for v in (base,name)]
        check={'case':tag,'success_equal':r['success']==b['success'],'right_equal':r.get('right_side_fraction')==b.get('right_side_fraction'),'length_equal':r.get('length_m')==b.get('length_m')}
        for f in ('path.npy','l2_reference_xy.npy'):
            exists=[(p/f).exists() for p in paths]
            check[f]=(not any(exists)) or (all(exists) and np.array_equal(*(np.load(p/f) for p in paths)))
        for f,k in [('l1_output.json','right_reference_polyline'),('l2_soft_target.json','points')]:
            exists=[(p/f).exists() for p in paths]
            check[f]=(not any(exists)) or (all(exists) and json.loads((paths[0]/f).read_text())[k]==json.loads((paths[1]/f).read_text())[k])
        records.append({k:bool(v) if k!='case' else v for k,v in check.items()})
    return {'all_equal':all(all(v for k,v in r.items() if k!='case') for r in records),'records':records}

def main():
    out={}
    names=['r10_control_10x3','r11_A_formal_10x3','r11_B_formal_10x3','r11_AB_formal_10x3','r11_ABC_crop_pilot','r11_ABC_crop_fixed10','r11_control_interleaved','r11_AB_interleaved','r11_control_perturbed','r11_AB_perturbed']
    for name in names:
        if not (ROOT/'results'/name/'summary.json').exists():continue
        d=summary(read(name))
        if '_formal_' in name:d['parity']=parity(name,'r10_control_10x3')
        if name.endswith(('interleaved','perturbed')) and '_AB_' in name:d['parity']=parity(name,name.replace('_AB_','_control_'))
        out[name]=d
        a=d['all'];print(name,d['strict_success'],'/',d['runs'],'wall',round(a['wall_s']['median'],3),round(a['wall_s']['p95'],3),'first/repeat',round(d['first']['wall_s']['median'],3),round(d['repeat']['wall_s']['median'],3) if d['repeat']['wall_s'] else None,'RSS',round(a['rss_gib']['median'],3),round(a['rss_gib']['max'],3),'plannerPSS',round(a['planner_pss_gib']['median'],3),'parity',d.get('parity',{}).get('all_equal'))
    (ROOT/'report/PERFORMANCE.json').write_text(json.dumps(out,indent=2,ensure_ascii=False))
if __name__=='__main__':main()
