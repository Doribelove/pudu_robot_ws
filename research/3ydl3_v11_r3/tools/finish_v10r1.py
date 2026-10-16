from pathlib import Path
import json,subprocess,time,os,sys
E=Path(__file__).resolve().parents[1];R=E/'report';F=E/'results/paired_final';deadline=time.monotonic()+1800
while 'ROUND_COMPLETE 1' not in (E/'paired_round1.log').read_text():
 if 'Traceback (most recent call last)' in (E/'paired_round1.log').read_text():raise RuntimeError('ROUND1_FAILED')
 if time.monotonic()>deadline:raise RuntimeError('ROUND1_WAIT_TIMEOUT')
 time.sleep(2)
for rnd in (2,3):
 print('RUN_ROUND',rnd,flush=True)
 with (E/f'paired_round{rnd}.log').open('w') as f:p=subprocess.run([sys.executable,str(E/'tools/run_paired_v10r1.py'),'--root',str(F),'--round',str(rnd)],stdout=f,stderr=subprocess.STDOUT,cwd=E)
 if p.returncode:raise RuntimeError('ROUND_FAILED:'+str(rnd))
 print('ROUND_COMPLETE',rnd,flush=True)
status=json.loads((F/'status.json').read_text());assert len(status)==60 and all(x['returncode']==0 and x['cleanup_complete'] for x in status)
print('ALL_600_REQUESTS_COMPLETE',flush=True);events=[];os.environ['THREEYD_BENCH_CPU']='14'
for mid in ('mengjia','arena_large_250x300_2d_005'):
 out=E/'results/events'/mid;out.parent.mkdir(exist_ok=True)
 with out.with_suffix('.log').open('w') as f:p=subprocess.run([sys.executable,str(E/'tools/run_v10r1_events.py'),'--map-id',mid,'--output',str(out)],stdout=f,stderr=subprocess.STDOUT,cwd=E)
 if p.returncode:raise RuntimeError('EVENT_FAILED:'+mid)
 summary=json.loads((out/'summary.json').read_text());assert summary['assertions_passed'];events.append({'path':str(out),'summary':summary});print('EVENT_COMPLETE',mid,summary['events'],flush=True)
(R/'EVENTS.json').write_text(json.dumps({'passed_events':sum(e['summary']['events'] for e in events),'maps':events,'scope':'two-map controlled event replay; includes expected failed planning and withdrawal/restoration'},ensure_ascii=False,indent=2))
(R/'SELECTED_DATA.json').write_text(json.dumps({'version':'3YDL3-V10-r1','formal_root':str(F),'control_version':'3YDL3-V10-r0','candidate_version':'3YDL3-V10-r1','arms':{'r0':'control','r1':'candidate'},'maps':10,'queries_per_map':10,'rounds':3,'requests_per_version':300,'all_60_batches_included':True,'cold_sampling':'Both arms enabled --cold-memory; 20 ms process-tree sampling from map loading to first response, excludes module imports','exclusions':['all endpoint and bridge diagnostic probes','profiled preparation data','audit equivalence setup','events'],'events_separate':str(R/'EVENTS.json'),'source_lock':str(F/'source.json')},ensure_ascii=False,indent=2))
for script,extra in [('analyze_paired_v10r1.py',['--root',str(F)]),('inspect_v10r1_output_variation.py',[]),('audit_v10r1_release.py',[]),('write_v10r1_report.py',[])]:
 print('RUN',script,flush=True)
 with (R/(script+'.log')).open('w') as f:p=subprocess.run([sys.executable,str(E/'tools'/script),*extra],stdout=f,stderr=subprocess.STDOUT,cwd=E)
 if p.returncode:raise RuntimeError('REVIEW_FAILED:'+script)
print('REPORT_READY',flush=True)
