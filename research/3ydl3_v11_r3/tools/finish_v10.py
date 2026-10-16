from pathlib import Path
import time,json,subprocess,sys,os
E=Path(__file__).resolve().parents[1];R=E/'report';F=E/'results/paired_final';end=time.monotonic()+1800
while time.monotonic()<end:
 text=(E/'paired_final.log').read_text()
 if 'COMPLETE_600_REQUESTS' in text:break
 if 'Traceback (most recent call last)' in text:raise RuntimeError('FORMAL_BATCH_ERROR; inspect original log')
 time.sleep(2)
else:raise RuntimeError('FORMAL_NOT_FINISHED')
status=json.loads((F/'status.json').read_text());assert len(status)==60 and all(v['cleanup_complete'] and v['returncode']==0 for v in status)
print('FORMAL_COMPLETE_START_EVENTS',flush=True);events=[]
os.environ['THREEYD_BENCH_CPU']='14'
for mid in ('mengjia','arena_large_250x300_2d_005'):
 out=E/'results/events'/mid;out.parent.mkdir(exist_ok=True)
 with out.with_suffix('.log').open('w') as f:
  p=subprocess.run([sys.executable,str(E/'tools/run_v10_storage_events.py'),'--map-id',mid,'--output',str(out)],cwd=E,stdout=f,stderr=subprocess.STDOUT)
 if p.returncode:raise RuntimeError('EVENT_FAILED:'+mid)
 summary=json.loads((out/'summary.json').read_text());assert summary['assertions_passed'];events.append({'path':str(out),'summary':summary})
 print('EVENT_COMPLETE',mid,summary['events'],flush=True)
(R/'EVENTS.json').write_text(json.dumps({'passed_events':sum(e['summary']['events'] for e in events),'maps':events,'scope':'two-map controlled event replay only, not ten-map dynamic or real-vehicle validation'},ensure_ascii=False,indent=2))
selected={'version':'3YDL3-V10-r0','source':3,'formal_root':str(F),'all_60_batches_included':True,'control':'3YDL3-V9','candidate_requests':300,'control_requests':300,'maps':10,'rounds':3,'queries':'each map own unchanged fixed Q','performance_excluded':['results/paired','results/diagnostic','results/source2_diagnostic','results/source3_diagnostic','results/profile_source1','results/endpoint_probe','results/events'],'events_separate':str(R/'EVENTS.json'),'source_lock':str(F/'source.json')}
(R/'SELECTED_DATA.json').write_text(json.dumps(selected,ensure_ascii=False,indent=2))
for script,extra in [('analyze_paired_v10.py',['--root',str(F)]),('audit_v10_release.py',[]),('add_v10_diagnostics.py',[]),('write_v10_report.py',[])]:
 print('RUN',script,flush=True)
 with (R/(script+'.log')).open('w') as f:p=subprocess.run([sys.executable,str(E/'tools'/script),*extra],cwd=E,stdout=f,stderr=subprocess.STDOUT)
 if p.returncode:raise RuntimeError('REVIEW_FAILED:'+script)
print('REPORT_READY',flush=True)
