"""Registered, serial, same-machine comparison; never overwrite a batch."""
from pathlib import Path
import hashlib
import json
import os
import shlex
import subprocess
import sys

R=Path(__file__).resolve().parents[1]
P4=Path('/home/robot/workspaces/3ydl3_v13_p4_corridor_recovery_20261027')
queue=json.loads((R/'docs/FORMAL_QUEUE.json').read_text())
done=[]
for job in queue:
    for file,digest in json.loads((R/'docs/CANDIDATE_SOURCE_LOCK.json').read_text()).items():
        assert hashlib.sha256(Path(file).read_bytes()).hexdigest()==digest,file
    out=R/'results'/job['name'];assert not out.exists(),out
    args=[sys.executable,str(R/'tools/pilot.py'),'--map-id',job['map'],'--query',job['query'],
          '--arm',job['arm'],'--events',job['event'],'--offset',str(job['offset']),
          '--output',str(out),'--domain','220']
    if job['mode']!='sequence':args+=['--only-'+job['mode']]
    if job['fixed']:args+=['--fixed-start']
    envfile=P4/'env_delivery.bash' if job['version']=='baseline' else R/'env.bash'
    env=dict(os.environ,V13_P5_SCHEDULE='0' if job['version']=='baseline' else '1',
             V13_P5_CACHE='0' if job['version']=='no_cache' else '1',
             V13_FORMAL='1',V13_CAPTURE_TRACE='0',V13_P5_HEADING_REFERENCE='0')
    command='source '+shlex.quote(str(envfile))+'\n'+shlex.join(args)
    (R/'report/QUEUE_STATUS.json').write_text(json.dumps({'status':'running','current':job,'completed':done,'total':len(queue)},indent=2))
    print('START',job['name'],flush=True)
    with (R/'report'/f"{job['name']}.log").open('w') as log:
        result=subprocess.run(['bash','-c',command],cwd=R,env=env,stdout=log,stderr=subprocess.STDOUT)
    assert result.returncode==0,(job['name'],result.returncode)
    summary=json.loads((out/'summary.json').read_text())
    assert summary['strict_recheck_passed'] and summary['all_failures_withdraw_trajectory']
    assert json.loads(out.with_suffix('.supervisor.json').read_text())['cleanup']['complete']
    for row in summary['rows']:
        negative=row['label']=='start_blocked' or (job['offset']==2 and row['label']=='large_add')
        assert row['success'] or negative,(job['name'],row)
    done.append(job['name']);print('DONE',job['name'],flush=True)
(R/'report/QUEUE_STATUS.json').write_text(json.dumps({'status':'completed','completed':done,'total':len(queue)},indent=2))
