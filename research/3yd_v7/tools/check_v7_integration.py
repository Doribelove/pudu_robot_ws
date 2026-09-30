"""Repeat r0's five integration examples, outside L1-only performance data."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

import psutil

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/integration_smoke'
CASES = [('arena_medium_200x200_2d_005','A2B-09'),
         ('mentor_map_20260825_005','A2B-01'),
         ('mentor_map_20260825_005_4x_area','A2B-01'),
         ('arena_large_250x300_2d_005','A2B-01'), ('mengjia','Q06')]


def main():
    OUT.mkdir(exist_ok=False)
    initial={p.pid for p in psutil.process_iter()}
    summaries=[]
    for mid,qid in CASES:
        target=OUT/(mid+'_'+qid)
        if mid=='mengjia':
            command=[sys.executable,str(ROOT/'tools/run_benchmark.py'),
                     '--map-id',mid,'--output',str(target),'--queries',qid,'--repeats','1']
        else:
            command=[sys.executable,str(ROOT/'tools/smoke_v7_pipeline.py'),
                     '--map-id',mid,'--query-id',qid,'--output',str(target)]
        print('INTEGRATION',mid,qid,flush=True)
        with (OUT/(mid+'_'+qid+'.log')).open('w') as stream:
            process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            try:code=process.wait(timeout=240)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait()
                raise
        if code:raise RuntimeError(f'Integration failed: {mid}, see log')
        if mid=='mengjia':
            row=json.loads((target/'runs.jsonl').read_text().splitlines()[0])
        else:row=json.loads((target/'result.json').read_text())
        acknowledgements=json.loads((target/'smac/lifecycle_evidence/acks.json').read_text())
        final_ack=acknowledgements[-1]
        ack_ok=all(a.get('costmap_update_acknowledged') and
                   a.get('costmap_ack_mismatch_cells')==0 for a in acknowledgements)
        independent=row.get('independent_audit',row.get('audit'))
        assert ack_ok and independent and independent.get('valid'), (mid,row)
        summaries.append({'map_id':mid,'query_id':qid,'success':bool(row['success']),
                          'formal_benchmark':False,'path':str(target),
                          'map_sync':{'ack_verified':ack_ok,
                                      'mismatch_cells':final_ack['costmap_ack_mismatch_cells'],
                                      'status':final_ack['costmap_ack_status']},
                          'independent_audit':independent})
        print('SUCCESS',row['success'],flush=True)
    remaining=[]
    for p in psutil.process_iter(['pid','name','cmdline']):
        if p.pid in initial:continue
        cmd=' '.join(p.info.get('cmdline') or [])
        if any(x in cmd for x in ('planner_server','map_server','lifecycle_manager','smoke_v7_pipeline','run_wanda3w')):
            remaining.append(p.info)
    (OUT/'SUMMARY.json').write_text(json.dumps({'cases':summaries,'remaining_task_processes':remaining},ensure_ascii=False,indent=2)+'\n')
    assert all(x['success'] for x in summaries), summaries
    assert not remaining,remaining
    print('INTEGRATION COMPLETE: 5/5, no task process residue',flush=True)


if __name__=='__main__':main()
