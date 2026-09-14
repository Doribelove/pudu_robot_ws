"""Sequential experiments for this active turn; never concurrent with real timing."""
import subprocess,time,psutil,json,sys
from pathlib import Path
W=Path('/home/robot/workspaces/semantic_dual_map_r1_20260914');R=W/'results';python='/usr/bin/python3'
wait_for=[]
for p in psutil.process_iter(['cmdline']):
 c=p.info['cmdline'] or []
 if len(c)>2 and c[1]==str(W/'tools/benchmark.py') and str(R/'real_formal_03') in c:wait_for.append(p)
for p in wait_for:
 while p.is_running() and p.status()!=psutil.STATUS_ZOMBIE:time.sleep(2)
steps=[
 ('controlled_formal_02',['controlled_benchmark.py','--output',str(R/'controlled_formal_02')]),
 ('dynamic_formal_01',['dynamic_benchmark.py','--output',str(R/'dynamic_formal_01')]),
 ('preparation_profile_01',['profile_preparation.py','--output',str(R/'preparation_profile_01')]),
 ('original_r1_strict_regression_01',['run_frozen_regression.py','--output-dir',str(R/'original_r1_strict_regression_01'),'--run-root','/media/robot/363A60C354861021/robot-experiment-archive/pudu_robot_ws/experiments/layered_planner_benchmark/nav2_3d_v1_r1_teb_actual2p5_20260907T133647_SGT/inputs','--repetitions','10','--warmups','1','--l3-repetitions','3','--cpu','15','--ros-domain-id','231']),
 ('derive_preference_retention',['derive_preference_retention.py','--run',str(R/'real_formal_03')]),
 ('summary',['summarize_results.py','--real',str(R/'real_formal_03'),'--controlled',str(R/'controlled_formal_02'),'--output',str(W/'report/statistics.json')]),
 ('figures',['figures.py','--real',str(R/'real_formal_03'),'--controlled',str(R/'controlled_formal_02'),'--dynamic',str(R/'dynamic_formal_01'),'--output',str(W/'report/figures')])]
history=[]
for name,args in steps:
 (R/'campaign_current_stage.json').write_text(json.dumps({'stage':name,'status':'running','history':history},indent=2));print('START',name,flush=True)
 args[0]=str(W/'tools'/args[0])
 with (R/(name+'_console.log')).open('w') as f:code=subprocess.call([python,*args],stdout=f,stderr=subprocess.STDOUT)
 history.append({'stage':name,'exit_code':code});print('END',name,code,flush=True)
 (R/'campaign_current_stage.json').write_text(json.dumps({'stage':name,'status':'completed' if code==0 else 'failed','history':history},indent=2))
print('SEQUENTIAL CAMPAIGN FINISHED',flush=True)
