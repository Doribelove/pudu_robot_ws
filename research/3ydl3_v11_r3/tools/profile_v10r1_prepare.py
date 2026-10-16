import json,os,sys,cProfile,pstats,time
from pathlib import Path
E=Path(__file__).resolve().parents[1]
for k,v in {'THREEYD_V9_MODE':'A','THREEYD_V9_R1_MODE':'S','THREEYD_V9_R2_MODE':'control','THREEYD_V9_R3_MODE':'control','THREEYD_V9_R5_MODE':'C','THREEYD_V9_R6_MODE':'H','THREEYD_V9_R7_MODE':'NT','THREEYD_V9_R8_MODE':'OGR','THREEYD_V9_R9_MODE':'PLMX'}.items():os.environ[k]=v
os.sched_setaffinity(0,{14})
from crossmap_inputs import load_world
from arena_3d_v1.multires_grid import GridView
from arena_evaluation import unified_four_backends_smoke as runtime
from three_yd_v5.v9_gate_init import DirectStorageAuditor
import hashlib
mid=sys.argv[1];out=E/'results'/('preparation_profile'+('_'+sys.argv[2] if len(sys.argv)>2 else ''))/mid;out.mkdir(parents=True,exist_ok=False);prof=cProfile.Profile();prof.enable();started=time.monotonic()
_,mp,w,g,qs,qf=load_world(mid,out);views={3:GridView(w,3),1:GridView(w,1)};loaded=time.monotonic();ctx=runtime.MapContext(mid,w.map,w.safe,w.map.distance_m,w.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp);mapped=time.monotonic();a=DirectStorageAuditor(ctx,source_commit='3YDL3-V10-r1-prepare-profile');ended=time.monotonic();prof.disable();prof.dump_stats(str(out/'prepare.prof'))
with (out/'profile.txt').open('w') as f:pstats.Stats(prof,stream=f).sort_stats('cumtime').print_stats(45)
(out/'summary.json').write_text(json.dumps({'load_world_views_s':loaded-started,'context_s':mapped-loaded,'auditor_s':ended-mapped,'total_s':ended-started,'CPU':[14],'profiled_diagnostic_not_performance':True},indent=2));print((out/'summary.json').read_text())
