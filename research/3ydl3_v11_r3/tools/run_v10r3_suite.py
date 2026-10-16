"""Sequential paired requests using one navigation-only harness for both arms."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PARENT = Path('/home/robot/workspaces/3ydl3_v10_r2_firstready_20261012')
FLAGS = ['--workload','fixed','--l3-backend','independent','--v9-mode','A',
         '--review-mode','S','--r2-mode','D','--r3-mode','control','--r5-mode','C',
         '--r6-mode','H','--r7-mode','NT','--r8-mode','OGR','--r9-mode','PLMX']


def run(destination, mid, arm, mode, queries=(), order=()):
    source = PARENT if arm == 'r2' else ROOT
    command = ['python3',str(ROOT/'tools/run_crossmap_v10.py'),*FLAGS,
               '--map-id',mid,'--output',str(destination),'--domain','218',
               '--arm','control' if arm=='r2' else 'candidate']
    if queries: command += ['--queries',*queries]
    if order: command += ['--query-order',*order]
    env = dict(os.environ, THREEYD_BENCH_SOURCE=str(source),
               THREEYD_V10_R3_MODE=mode, THREEYD_BENCH_CPU='14')
    destination.parent.mkdir(parents=True, exist_ok=True)
    print(f'START {destination.relative_to(ROOT)} {mode}', flush=True)
    with destination.with_suffix('.log').open('w') as stream:
        result = subprocess.run(['bash','-c','source '+shlex.quote(str(source/'env.bash'))+
                                 '\nexec '+shlex.join(command)],env=env,stdout=stream,stderr=subprocess.STDOUT)
    journal = json.loads(destination.with_suffix('.supervisor.json').read_text())
    if result.returncode or not journal.get('cleanup',{}).get('complete'):
        raise RuntimeError(f'INCOMPLETE_BATCH: {destination} exit={result.returncode}')
    print('DONE '+str(destination.relative_to(ROOT))+' '+
          (destination/'summary.json').read_text().replace('\n',' '), flush=True)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('suite',choices=['ablation','order','formal'])
    parser.add_argument('--mode', default='AB')
    parser.add_argument('--output', required=True,type=Path)
    args=parser.parse_args()
    args.output=args.output.resolve()
    if args.output.exists(): raise ValueError('OUTPUT_MUST_BE_NEW')
    args.output.mkdir(parents=True)
    if args.suite == 'formal':
        files=[ROOT/'env.bash',ROOT/'run.sh']
        for directory in ('src','tools','native','config','install'):
            files.extend(p for p in (ROOT/directory).rglob('*') if p.is_file() and '__pycache__' not in str(p))
        (args.output/'source.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                                                       for p in files},indent=2)+'\n')
    maps=json.loads((ROOT/'MAPS.json').read_text())['maps']
    if args.suite == 'ablation':
        for mode in ('control','A','B','AB'):
            run(args.output/'3yd_synth_trizone_c_v2'/mode,'3yd_synth_trizone_c_v2',
                'r2' if mode=='control' else 'r3',mode,['Q01','Q06'])
        for mid in ('mengjia','arena_large_250x300_2d_005'):
            ids=next(m['query_ids'] for m in maps if m['map_id']==mid)
            for arm,mode in [('r3',args.mode),('r2','control')]:
                run(args.output/mid/arm,mid,arm,mode,[ids[0],ids[5]])
    elif args.suite == 'order':
        for mode in ('control',args.mode):
            for order in (['Q01','Q06'],['Q06','Q01']):
                run(args.output/mode/('-'.join(order)),'3yd_synth_trizone_c_v2',
                    'r2' if mode=='control' else 'r3',mode,['Q01','Q06'],order)
    else:
        for rnd in range(1,4):
            for index,m in enumerate(maps):
                arms=[('r2','control'),('r3',args.mode)]
                if (rnd+index)%2: arms.reverse()
                for arm,mode in arms:
                    run(args.output/f'round{rnd}'/m['map_id']/arm,m['map_id'],arm,mode)
    (args.output/'COMPLETE.json').write_text(json.dumps({'complete':True,'suite':args.suite,
        'mode':args.mode,'navigation_only':True},indent=2)+'\n')


if __name__=='__main__': main()
