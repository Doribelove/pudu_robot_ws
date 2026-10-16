"""Serial, source-locked V11 verification with a same-batch V11-r2 control."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/home/robot/workspaces/3ydl3_v11_r2_tailperf_20261016')
FLAGS=['--workload','fixed','--l3-backend','independent','--v9-mode','A',
       '--review-mode','S','--r2-mode','D','--r3-mode','control','--r5-mode','C',
       '--r6-mode','H','--r7-mode','NT','--r8-mode','OGR','--r9-mode','PLMX']


def run(destination,mid,arm='candidate',extra=(),event=False):
    destination=Path(destination).resolve();source=PARENT if arm=='control' else ROOT
    env=dict(os.environ,THREEYD_BENCH_SOURCE=str(source),THREEYD_BENCH_CPU='14',
        THREEYD_V10_R3_MODE='ABRTHI',THREEYD_V10_R4_MODE='PHFBDJNTV',
        THREEYD_V10_R5_MODE='MLSEKU',THREEYD_V10_R6_MODE='DCGXY',
        THREEYD_V10_R7_MODE='ABCD',THREEYD_V10_R8_MODE='STPBE',THREEYD_ENDPOINT_TOLERANCE='1')
    for name in ('THREEYD_R4_CAPTURE','THREEYD_R6_COMPARE_FIELDS','THREEYD_R6_CAPTURE_L2','THREEYD_R4_DIAG_FINE_POSE'):
        env.pop(name,None)
    command=['python3',str(ROOT/'tools'/('run_v11_events.py' if event else 'run_crossmap_v10.py')),
             '--map-id',mid,'--output',str(destination)]
    if not event:command += FLAGS+['--domain','218','--arm','control' if arm=='control' else 'candidate']
    command+=list(extra)
    destination.parent.mkdir(parents=True,exist_ok=True)
    assert not destination.exists(),destination
    print('START',destination,flush=True)
    with open('/tmp/3yd_v10_r4_bench.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        with destination.with_suffix('.log').open('w') as log:
            result=subprocess.run(['bash','-c','source '+shlex.quote(str(source/'env.bash'))+'\nexec '+shlex.join(command)],env=env,stdout=log,stderr=subprocess.STDOUT)
    journal=json.loads(destination.with_suffix('.supervisor.json').read_text())
    assert result.returncode==0 and journal['returncode']==0 and journal['cleanup']['complete'],str(destination)
    summary=json.loads((destination/'summary.json').read_text())
    print('DONE',destination,summary,flush=True)
    return summary


def main():
    parser=argparse.ArgumentParser();parser.add_argument('suite',choices=['checks','formal'])
    parser.add_argument('--output',required=True,type=Path);args=parser.parse_args()
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    files=[ROOT/'env.bash',ROOT/'run.sh',ROOT/'BASELINE.json']
    for name in ('src','tools','native','config','install'):
        files.extend(p for p in (ROOT/name).rglob('*') if p.is_file() and '__pycache__' not in str(p))
    lock={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    (out/'source.json').write_text(json.dumps(lock,indent=2)+'\n')
    if args.suite=='checks':
        for i,order in enumerate((['A2B-07','A2B-01'],['A2B-01','A2B-07'],['A2B-07','A2B-03']),1):
            summary=run(out/f'order{i}','mentor_map_20260825_005',extra=['--queries',*order,'--query-order',*order])
            assert summary['success']==len(order) and summary['tolerant_endpoint_success']==1
        for mid in ('mengjia','arena_large_250x300_2d_005','pudu_wanda_3f','arena_medium_200x200_2d_005','mentor_map_20260825_005'):
            run(out/'events'/mid,mid,event=True)
        run(out/'events/tolerant_mentor','mentor_map_20260825_005',event=True,extra=['--query-id','A2B-07'])
    else:
        maps=json.loads((ROOT/'MAPS.json').read_text())['maps']
        for rnd in range(1,4):
            for index,m in enumerate(maps):
                arms=['control','candidate']
                if (rnd+index)%2:arms.reverse()
                for arm in arms:run(out/f'round{rnd}'/m['map_id']/arm,m['map_id'],arm)
    changed=[p for p,h in lock.items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
    assert not changed,changed
    (out/'COMPLETE.json').write_text(json.dumps({'complete':True,'suite':args.suite,'version':'3YDL3-V11-r3','source_lock_verified':True},indent=2)+'\n')


if __name__=='__main__':main()
