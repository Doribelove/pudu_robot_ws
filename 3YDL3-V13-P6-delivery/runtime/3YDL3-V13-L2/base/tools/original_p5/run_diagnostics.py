"""Separate trace collection after formal timing, source remains frozen."""
import json,os,subprocess,shlex,sys,hashlib
from pathlib import Path
R=Path(__file__).resolve().parents[1]
P4=Path('/home/robot/workspaces/3ydl3_v13_p4_corridor_recovery_20261027')
assert json.loads((R/'report/QUEUE_STATUS.json').read_text())['status']=='completed'
jobs=[]
for pose,modes,arms in [(4,['small','large'],['H-A','H-I']),(8,['large'],['H-I']),(2,['large'],['H-I'])]:
    for mode in modes:
        for version in ['baseline','candidate']:
            for arm in arms:jobs.append(dict(pose=pose,mode=mode,version=version,arm=arm))
(R/'docs/DIAGNOSTIC_QUEUE.json').write_text(json.dumps(jobs,indent=2))
(R/'diagnostics').mkdir(exist_ok=True)
for j in jobs:
    for f,h in json.loads((R/'docs/CANDIDATE_SOURCE_LOCK.json').read_text()).items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==h
    name=f"diag_{j['pose']}_{j['mode']}_{j['version']}_{j['arm']}";out=R/'diagnostics'/name;assert not out.exists()
    event=R/'docs'/('MENTOR_EVENTS.json' if j['pose']==4 else f"MENTOR_POSE_{j['pose']}M.json")
    args=[sys.executable,str(R/'tools/pilot.py'),'--map-id','mentor_map_20260825_005','--query','A2B-07','--arm',j['arm'],
        '--events',str(event),'--offset',str(j['pose']),'--only-'+j['mode'],'--domain','220','--output',str(out)]
    envfile=P4/'env_delivery.bash' if j['version']=='baseline' else R/'env.bash'
    env=dict(os.environ,V13_P5_SCHEDULE='0' if j['version']=='baseline' else '1',V13_P5_CACHE='1',V13_FORMAL='0',V13_CAPTURE_TRACE='1',V13_P5_HEADING_REFERENCE='0')
    print('START',name,flush=True)
    with (R/'report'/f'{name}.log').open('w') as log:
        p=subprocess.run(['bash','-c','source '+shlex.quote(str(envfile))+'\n'+shlex.join(args)],cwd=R,env=env,stdout=log,stderr=subprocess.STDOUT)
    assert p.returncode==0
    assert json.loads(out.with_suffix('.supervisor.json').read_text())['cleanup']['complete']
    print('DONE',name,flush=True)
(R/'report/DIAGNOSTIC_STATUS.json').write_text(json.dumps({'status':'completed','batches':len(jobs),'requests':7*len(jobs)},indent=2))
