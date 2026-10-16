import psutil,time,subprocess,json
from pathlib import Path
W=Path('/home/robot/workspaces/semantic_dual_map_r1_20260914');R=W/'results'
for p in psutil.process_iter(['cmdline']):
 c=p.info['cmdline'] or []
 if len(c)>1 and c[1]==str(W/'tools/finish_campaign.py'):
  while p.is_running() and p.status()!=psutil.STATUS_ZOMBIE:time.sleep(2)
steps=[('controlled_formal_03',['controlled_benchmark.py','--output',str(R/'controlled_formal_03')]),('summary_final',['summarize_results.py','--real',str(R/'real_formal_03'),'--controlled',str(R/'controlled_formal_03'),'--output',str(W/'report/statistics.json')]),('figures_final',['figures.py','--real',str(R/'real_formal_03'),'--controlled',str(R/'controlled_formal_03'),'--dynamic',str(R/'dynamic_formal_01'),'--output',str(W/'report/figures')]),('build_report',['build_report.py'])]
history=[]
for name,args in steps:
 print('START',name,flush=True);(R/'replication_current_stage.json').write_text(json.dumps({'stage':name,'status':'running','history':history}))
 args[0]=str(W/'tools'/args[0])
 with (R/(name+'_console.log')).open('w') as f:code=subprocess.call(['/usr/bin/python3',*args],stdout=f,stderr=subprocess.STDOUT)
 history.append({'stage':name,'exit_code':code});print('END',name,code,flush=True)
 (R/'replication_current_stage.json').write_text(json.dumps({'stage':name,'status':'completed' if code==0 else 'failed','history':history}))
