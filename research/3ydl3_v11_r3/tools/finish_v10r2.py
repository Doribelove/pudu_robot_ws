from pathlib import Path
import json,subprocess,sys,os
E=Path(__file__).resolve().parents[1];R=E/'report';F=E/'results/paired_final'
assert '749 passed' in (R/'tests_final.log').read_text()
proof=json.loads((R/'PREPARATION_EQUIVALENCE.json').read_text());assert len(proof)==5 and all(x['all_world_cells_topology_and_views_equal'] for x in proof)
for n in (1,2,3):
 out=E/f'results/ready_probe_{n}';j=json.loads((out/'summary.json').read_text());s=json.loads(out.with_suffix('.supervisor.json').read_text());assert j['success']==j['runs']==1 and s['returncode']==0 and s['cleanup']['complete']
 assert json.loads((out/'session_startup.json').read_text())['readiness']['fixed_settle_sleep_s']==0
for rnd in (1,2,3):
 print('RUN_ROUND',rnd,flush=True)
 with (E/f'paired_round{rnd}.log').open('w') as f:p=subprocess.run([sys.executable,str(E/'tools/run_paired_v10r2.py'),'--root',str(F),'--round',str(rnd)],stdout=f,stderr=subprocess.STDOUT,cwd=E)
 if p.returncode:raise RuntimeError('ROUND_FAILED:'+str(rnd))
 print('ROUND_COMPLETE',rnd,flush=True)
status=json.loads((F/'status.json').read_text());assert len(status)==60 and all(x['returncode']==0 and x['cleanup_complete'] for x in status)
print('ALL_600_REQUESTS_COMPLETE',flush=True);events=[];os.environ['THREEYD_BENCH_CPU']='14'
for mid in ('mengjia','arena_large_250x300_2d_005'):
 out=E/'results/events'/mid;out.parent.mkdir(exist_ok=True)
 with out.with_suffix('.log').open('w') as f:p=subprocess.run([sys.executable,str(E/'tools/run_v10r2_events.py'),'--map-id',mid,'--output',str(out)],stdout=f,stderr=subprocess.STDOUT,cwd=E)
 if p.returncode:raise RuntimeError('EVENT_FAILED:'+mid)
 summary=json.loads((out/'summary.json').read_text());assert summary['assertions_passed'];events.append({'path':str(out),'summary':summary});print('EVENT_COMPLETE',mid,summary['events'],flush=True)
(R/'EVENTS.json').write_text(json.dumps({'passed_events':sum(e['summary']['events'] for e in events),'maps':events,'scope':'two-map controlled event replay; includes expected withdrawal/restoration'},ensure_ascii=False,indent=2))
(R/'SELECTED_DATA.json').write_text(json.dumps({'version':'3YDL3-V10-r3','formal_root':str(F),'control_version':'3YDL3-V10-r1','candidate_version':'3YDL3-V10-r3','arms':{'r1':'control','r2':'candidate'},'maps':10,'queries_per_map':10,'rounds':3,'requests_per_version':300,'all_60_batches_included':True,'cold_sampling':'Both arms --cold-memory, fresh processes and fresh artifact caches; OS file cache not flushed; loading through first response including startup/export, excludes imports','exclusions':['preparation equality and diagnostic timings','pilot_c before readiness change','three ready_probe diagnostic starts','events'],'source_lock':str(F/'source.json')},ensure_ascii=False,indent=2))
with (R/'analysis.log').open('w') as f:p=subprocess.run([sys.executable,str(E/'tools/analyze_paired_v10r2.py'),'--root',str(F)],stdout=f,stderr=subprocess.STDOUT,cwd=E)
assert p.returncode==0
print('ANALYSIS_READY_FOR_RELEASE_REVIEW',flush=True)
