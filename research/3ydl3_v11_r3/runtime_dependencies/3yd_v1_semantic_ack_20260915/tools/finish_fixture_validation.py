"""Keep the rejected old EDT test fixture, validate using the production oracle."""
import json,time,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
until=time.monotonic()+1800
while True:
 f=ROOT/'report/dispatch.jsonl';records=[json.loads(s) for s in f.read_text().splitlines()] if f.exists() else []
 if any(r['name']=='control_two_routes_optimistic_edge' for r in records):break
 if time.monotonic()>until:raise RuntimeError('Primary validation did not finish')
 time.sleep(1)
for scene,count in [('calibration',40),('rotated',40),('real',20)]:
 name='final_stress_'+scene;out=ROOT/'results'/name
 if out.exists():raise FileExistsError(out)
 cmd=[sys.executable,str(ROOT/'tools/semantic_stress.py'),'--output',str(out),'--scene',scene,'--updates',str(count),'--domain','104']
 t=time.monotonic();print('START',name,flush=True)
 with (ROOT/'report'/f'{name}.log').open('w') as f:code=subprocess.call(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
 record={'name':name,'command':cmd,'returncode':code,'elapsed_s':time.monotonic()-t}
 with (ROOT/'report/final_stress_dispatch.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
 print('FINISH',name,code,flush=True)
 if code:raise RuntimeError(name)
 summary=json.loads((out/'summary.json').read_text())
 if summary['success']!=count:raise RuntimeError(f'Incomplete semantic stress: {summary}')
