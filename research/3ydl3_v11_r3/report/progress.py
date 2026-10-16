from pathlib import Path
import json
R=Path(__file__).resolve().parents[1];D=R/'results/paired_final';out={}
for rnd in range(1,4):
 out[rnd]={arm:{'batches':0,'requests':0,'success':0,'exact':0,'tolerant':0} for arm in ['control','candidate']}
 for arm in ['control','candidate']:
  for p in (D/f'round{rnd}').glob('*/'+arm+'/summary.json'):
   j=p.parent.with_suffix('.supervisor.json')
   if not j.exists() or not json.loads(j.read_text()).get('cleanup',{}).get('complete'):continue
   d=json.loads(p.read_text());s=out[rnd][arm];s['batches']+=1;s['requests']+=d['runs'];s['success']+=d['success'];s['exact']+=d['exact_endpoint_success'];s['tolerant']+=d['tolerant_endpoint_success']
print(json.dumps(out))
