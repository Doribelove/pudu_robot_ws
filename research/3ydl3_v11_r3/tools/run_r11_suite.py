"""Serial gated batches; every batch preserves raw evidence and owned cleanup."""
import argparse,json,os,subprocess,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]

def validate(out,repeats):
    rows=[json.loads(x) for x in (out/'runs.jsonl').read_text().splitlines()]
    checks={'count':len(rows)==10*repeats,'success':all(r['success'] for r in rows),
        'ACK':all(r.get('l3',{}).get('costmap_update_acknowledged') for r in rows),
        'bounds':all(r.get('l3',{}).get('returned_path_within_mask') for r in rows),
        'path_parity':True,'reference_parity':True,'L1_parity':True,'soft_target_parity':True,'right_parity':True}
    baseline=[json.loads(x) for x in (ROOT/'results/r10_control_10x3/runs.jsonl').read_text().splitlines()]
    for r in rows:
        case=f"{r['query_id']}_{r['repeat']}";b=ROOT/'results/r10_control_10x3/cases'/case;c=out/'cases'/case
        for file,key in [('path.npy','path_parity'),('l2_reference_xy.npy','reference_parity')]:
            checks[key]&=(c/file).exists() and np.array_equal(np.load(b/file),np.load(c/file))
        for file,key,field in [('l1_output.json','L1_parity','right_reference_polyline'),('l2_soft_target.json','soft_target_parity','points')]:
            checks[key]&=(c/file).exists() and json.loads((c/file).read_text())[field]==json.loads((b/file).read_text())[field]
        base=next(v for v in baseline if (v['query_id'],v['repeat'])==(r['query_id'],r['repeat']))
        checks['right_parity']&=r.get('right_side_fraction')==base.get('right_side_fraction')
    journal=json.loads(out.with_suffix('.supervisor.json').read_text())
    checks['cleanup']=journal['state']=='complete'
    checks={k:bool(v) for k,v in checks.items()}
    (out/'GATE.json').write_text(json.dumps(checks,indent=2))
    if not all(checks.values()):raise RuntimeError(f'{out.name}: gate failed: {checks}')
    return checks

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--phase',choices=['preflight','formal'],required=True);ap.add_argument('--modes',default='A,B,AB');a=ap.parse_args()
    repeats=1 if a.phase=='preflight' else 3
    for mode in a.modes.split(','):
        out=ROOT/'results'/f'r11_{mode}_{a.phase}_10x{repeats}'
        env=dict(os.environ,THREEYD_R11_MODE=mode,OPENBLAS_NUM_THREADS='1')
        print('START',out.name,flush=True)
        with (ROOT/'report'/f'{out.name}.log').open('w') as log:
            subprocess.run([sys.executable,str(ROOT/'tools/run_benchmark.py'),'--map','corroborated_map.yaml','--queries','all','--repeats',str(repeats),'--output',str(out),'--domain',{'A':'230','B':'232','C':'229','ABC':'228','AB':'231'}[mode]],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
        print('PASS',out.name,validate(out,repeats),flush=True)
if __name__=='__main__':main()
