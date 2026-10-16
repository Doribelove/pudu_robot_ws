"""Run analysis only after every predeclared validation job has passed."""
import json,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
end=time.monotonic()+900
while True:
 p=ROOT/'report/final_dispatch.jsonl';rows=[json.loads(l) for l in p.read_text().splitlines()] if p.exists() else []
 if any(not r['assertions_passed'] or r['returncode'] for r in rows):raise RuntimeError('A recorded final validation failed')
 if len(rows)==18:break
 if time.monotonic()>end:raise TimeoutError('Final validation did not finish; leave all evidence intact')
 time.sleep(2)
for script in ['summarize_semantic.py','check_semantic_acceptance.py','write_semantic_report.py']:
 with (ROOT/'report'/script.replace('.py','.log')).open('w') as output:
  code=subprocess.call([sys.executable,str(ROOT/'tools'/script)],cwd=ROOT,stdout=output,stderr=subprocess.STDOUT)
 print(script,code,flush=True)
 if code:raise RuntimeError(script)
