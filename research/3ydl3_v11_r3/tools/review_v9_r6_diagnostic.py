"""Review original 03/Q09 diagnostic subsets; never formal fixed-10 evidence."""
import argparse
from pathlib import Path
from compare_v9 import read, tag, pair, summary
from compare_v9_review import save, sha
from compare_v9_r6 import compare, mode_evidence
from compare_v8_r6 import verify_cleanup
from analyze_v8 import MAPS
ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser();p.add_argument('--results-root',type=Path,required=True);a=p.parse_args()
    out={'version':'3YD-V9-r6','diagnostic_only':True,'results_root':str(a.results_root),'workloads':{}}
    for kind in ('moving','perturbed'):
        maps={}
        for mid,label in MAPS:
            ds={m:a.results_root/'diagnostic'/kind/m/mid for m in ('control','C','CH')}
            rows={m:read(d) for m,d in ds.items()}
            assert all(len(rs)==4 for rs in rows.values())
            maps[mid]={'label':label,'arms':{m:summary(rs) for m,rs in rows.items()},
                'parity':{m:compare(ds['control'],ds[m],report_upstream_mismatch=True) for m in ('C','CH')},
                'mode_evidence':{m:mode_evidence(d,m) for m,d in ds.items()},
                'cleanup':{m:verify_cleanup(d) for m,d in ds.items()},
                'runs_sha256':{m:sha(d/'runs.jsonl') for m,d in ds.items()},
                'queries':{m:[{'tag':tag(r),'success':r['success'],'wall_ms':r['wall_ms'],
                    'task_progress':r['task_progress']} for r in rs] for m,rs in rows.items()}}
            print(kind,mid,{m:[(tag(r),r['success'],r['task_progress'].get('reason',r['task_progress'].get('match')),round(r['wall_ms'])) for r in rs] for m,rs in rows.items()},flush=True)
            print('PARITY', {m:{k:maps[mid]['parity'][m][k] for k in ('upstream_gate','quality_gate','upstream_failed_tags')} for m in ('C','CH')},flush=True)
        out['workloads'][kind]=maps
    save(ROOT/'report/R6_DIAGNOSTIC_COMPARISON.json',out)
    print('DIAGNOSTIC_CH_PASS',all(v['parity']['CH']['upstream_gate'] and v['parity']['CH']['quality_gate'] for maps in out['workloads'].values() for v in maps.values()),flush=True)
if __name__=='__main__':main()
