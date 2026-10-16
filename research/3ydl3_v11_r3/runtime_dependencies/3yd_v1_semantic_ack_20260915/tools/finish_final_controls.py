"""Retain failed startup attempts and use fresh isolated domains for retries."""
import subprocess,sys,json
from arena_3d_v1.semantic_world import WORK,json_write
scenes=['rotated','narrow','two_routes','two_routes_optimistic_edge','return_bulb','return_bulb_rotated'];records=[]
for index,scene in enumerate(scenes):
    for attempt in range(1,3):
        subprocess.run([sys.executable,str(WORK/'tools/cleanup_experiment_shm.py')],check=True)
        output=WORK/'results'/f'control_final_{scene}_recovery_isolated_{attempt}'
        domain=80+index*2+attempt-1
        cmd=[sys.executable,str(WORK/'tools/static_benchmark.py'),'--output',str(output),'--arm','recovery','--scene',scene,'--repeats','3','--domain',str(domain)]
        with (WORK/'report'/f'control_final_{scene}_isolated_{attempt}.log').open('w') as log:result=subprocess.run(cmd,cwd=WORK,stdout=log,stderr=subprocess.STDOUT)
        records.append({'scene':scene,'attempt':attempt,'domain':domain,'returncode':result.returncode,'output':str(output)})
        json_write(WORK/'report/final_control_dispatches.json',records)
        print(scene,attempt,result.returncode,flush=True)
        if result.returncode==0:break
    else:raise RuntimeError(f'{scene}: two startup/process attempts failed; records retained')
