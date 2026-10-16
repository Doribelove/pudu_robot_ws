"""Serial r6/r7 paired harness, preserving first-navigation timing."""
import argparse,hashlib,json,os,shlex,subprocess,fcntl
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/home/robot/workspaces/3ydl3_v10_r6_localfix_b_20261015')
FLAGS=['--workload','fixed','--l3-backend','independent','--v9-mode','A',
       '--review-mode','S','--r2-mode','D','--r3-mode','control','--r5-mode','C',
       '--r6-mode','H','--r7-mode','NT','--r8-mode','OGR','--r9-mode','PLMX']


def _run(destination,mid,arm='r7',mode='control',queries=(),extra=(),capture=False,fine_pose=False,field_check=False):
    destination=Path(destination).resolve()
    source=PARENT if arm=='r6' else ROOT
    command=['python3',str(ROOT/'tools/run_crossmap_v10.py'),*FLAGS,
             '--map-id',mid,'--output',str(destination),'--domain','218',
             '--arm','control' if arm=='r6' else 'candidate']
    if queries:command+=['--queries',*queries]
    command+=list(extra)
    env=dict(os.environ,THREEYD_BENCH_SOURCE=str(source),THREEYD_BENCH_CPU='14',
             THREEYD_V10_R3_MODE='ABRTHI',THREEYD_V10_R4_MODE='PHFBDJNTV',THREEYD_V10_R5_MODE='MLSEKU',THREEYD_V10_R6_MODE='DCGXY',THREEYD_V10_R7_MODE=mode if arm=='r7' else 'control')
    env.pop('THREEYD_R4_CAPTURE',None)
    env.pop('THREEYD_R6_COMPARE_FIELDS',None)
    env.pop('THREEYD_R6_CAPTURE_L2',None)
    if field_check:env['THREEYD_R6_COMPARE_FIELDS']='1'
    env.pop('THREEYD_R4_DIAG_FINE_POSE',None)
    if fine_pose:env['THREEYD_R4_DIAG_FINE_POSE']='1'
    if capture:env['THREEYD_R4_CAPTURE']=str(destination/'curve_inputs')
    destination.parent.mkdir(parents=True,exist_ok=True)
    print('START',destination,arm,mode,flush=True)
    with destination.with_suffix('.log').open('w') as stream:
        result=subprocess.run(['bash','-c','source '+shlex.quote(str(source/'env.bash'))+
                              '\nexec '+shlex.join(command)],env=env,stdout=stream,stderr=subprocess.STDOUT)
    journal=json.loads(destination.with_suffix('.supervisor.json').read_text())
    assert result.returncode==0 and journal['returncode']==0 and journal['cleanup']['complete'],str(destination)
    print('DONE',destination,json.loads((destination/'summary.json').read_text()),flush=True)


def run(*args,**kwargs):
    # Shared DDS services must never overlap across diagnostic/formal drivers.
    with open('/tmp/3yd_v10_r4_bench.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        return _run(*args,**kwargs)


def main():
    a=argparse.ArgumentParser();a.add_argument('suite',choices=['diagnostic','formal'])
    a.add_argument('--mode',default='control');a.add_argument('--output',type=Path,required=True)
    args=a.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    if args.suite=='diagnostic':
        cases={'pudu_wanda_3f':['Q08','Q09','Q10'],'3yd_synth_trizone_c_v2':['Q01','Q10'],
               'arena_medium_200x200_2d_005':['A2B-04'],'mentor_map_20260825_005':['A2B-03','A2B-07']}
        for mid,queries in cases.items():run(out/mid,mid,mode=args.mode,queries=queries,capture=True)
    else:
        files=[ROOT/'env.bash',ROOT/'run.sh']
        for name in ('src','tools','native','config','install'):
            files.extend(p for p in (ROOT/name).rglob('*') if p.is_file() and '__pycache__' not in str(p))
        (out/'source.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2)+'\n')
        maps=json.loads((ROOT/'MAPS.json').read_text())['maps']
        for rnd in range(1,4):
            for index,m in enumerate(maps):
                arms=['r6','r7']
                if (rnd+index)%2:arms.reverse()
                for arm in arms:run(out/f'round{rnd}'/m['map_id']/arm,m['map_id'],arm,args.mode if arm=='r7' else 'control')
    (out/'COMPLETE.json').write_text(json.dumps({'complete':True,'suite':args.suite,'mode':args.mode},indent=2)+'\n')


if __name__=='__main__':main()
