import os,subprocess,json,time
from pathlib import Path
root=Path(__file__).resolve().parents[1]
for mode in ("control","A","B","AB"):
    out="results/diagnostic_A03/"+mode
    print("START",mode,flush=True)
    env=dict(os.environ,THREEYD_V8_R4_MODE=mode,OPENBLAS_NUM_THREADS="1",OMP_NUM_THREADS="1",MKL_NUM_THREADS="1")
    with (root/"report"/("diag_"+mode+".log")).open("w") as log:
        r=subprocess.run(["taskset","-c","12","./run.sh","--mode","free-space","--map-id","arena_large_250x300_2d_005","--queries","A2B-03","--repeats","1","--output",out],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
    if r.returncode:raise SystemExit(r.returncode)
    rows=[json.loads(x) for x in (root/out/"runs.jsonl").read_text().splitlines()]
    print(mode,[(r["success"],r["wall_ms"],r["rss_peak_bytes"]/2**30,r.get("failure_code")) for r in rows],flush=True)
