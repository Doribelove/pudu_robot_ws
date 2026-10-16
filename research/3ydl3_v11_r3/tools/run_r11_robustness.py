"""Serial order/pose tests. Baseline failures remain visible; no data filtering."""
import os,sys,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def main():
    for test,repeats,extra in [('interleaved',5,['--interleave']),('perturbed',1,['--query-file',str(ROOT/'inputs/queries_perturbed.json')])]:
        for mode in ['control','AB']:
            name=f'r11_{mode}_{test}';out=ROOT/'results'/name
            print('START',name,flush=True)
            env=dict(os.environ,THREEYD_R11_MODE=mode,OPENBLAS_NUM_THREADS='1')
            with (ROOT/'report'/f'{name}.log').open('w') as f:
                subprocess.run([sys.executable,str(ROOT/'tools/run_benchmark.py'),'--map','corroborated_map.yaml','--queries','all','--repeats',str(repeats),'--output',str(out),'--domain','231',*extra],env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
            summary=json.loads((out/'summary.json').read_text())
            journal=json.loads(out.with_suffix('.supervisor.json').read_text())
            if journal['state']!='complete':raise RuntimeError('Cleanup incomplete')
            print('DONE',name,summary,flush=True)
if __name__=='__main__':main()
