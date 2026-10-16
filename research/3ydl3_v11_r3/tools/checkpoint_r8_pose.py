"""Reconcile functional evidence; this does not certify formal performance."""
import hashlib,json
from pathlib import Path

R=Path(__file__).resolve().parents[1]
P=Path('/home/robot/workspaces/3ydl3_v10_r7_tailcost_b_20261015')
def read(p):return json.loads(p.read_text())
def rows(p):return [json.loads(s) for s in p.read_text().splitlines()]
def digest(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for part in iter(lambda:f.read(1024*1024),b''):h.update(part)
    return h.hexdigest()
def save(name,data):(R/'report'/name).write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')

def validate(path):
    records=rows(path/'runs.jsonl')
    for r in records:
        if not r['success']:continue
        assert r['audit']['valid'] and r['constraints']['traffic']['valid']
        assert r['constraints']['steering_jump_valid']
        assert r['curve_l3']['certificate']['max_curvature_bound_1pm']<=2.5+1e-8
        out=read(path/'cases'/f"{r['query_id']}_{r['repeat']}"/'outcome.json')
        assert out['diagnostics']['costmap_update_acknowledged'] is True
        assert out['independent_audit']['valid'] and out['traffic_audit']['valid']
    journal=read(path.with_suffix('.supervisor.json'))
    assert journal['returncode']==0 and journal['cleanup']['complete']
    return records

all_rows=[];baseline=set();maps=[];fingerprints={}
for m in read(R/'MAPS.json')['maps']:
    mid=m['map_id'];path=R/'results/pose_final_tenmap_validation'/mid
    current=validate(path);assert len(current)==10
    all_rows+=current;maps.append({'map_id':mid,'success':sum(r['success'] for r in current),'queries':10})
    fingerprints[str(path/'runs.jsonl')]=digest(path/'runs.jsonl')
    baseline|={(mid,r['query_id']) for r in rows(P/'results/paired_final/round1'/mid/'r7/runs.jsonl') if r['success']}
success={(r['map_id'],r['query_id']) for r in all_rows if r['success']}
assert len(baseline)==97 and baseline<=success and len(success)==99
assert [(r['map_id'],r['query_id']) for r in all_rows if not r['success']]==[('mentor_map_20260825_005','A2B-07')]
save('POSE_TEN_MAP_VALIDATION.json',{'diagnostic_only':True,'formal_performance':False,'requests':100,'strict_success':99,'original_97_preserved':True,'new_queries':sorted(success-baseline),'maps':maps,'raw_sha256':fingerprints})

confirm=[]
for folder,mid,q in [('pose_confirmation_heading_order','pudu_wanda_3f','Q09'),('terminal_confirmation_ordered','arena_medium_200x200_2d_005','A2B-04')]:
    rec=[]
    for rnd in range(1,4):
        path=R/'results'/folder/f'round{rnd}'/mid;r=next(r for r in validate(path) if r['query_id']==q)
        assert r['success'];protocol=read(path/'protocol.json');assert protocol['query_order_diagnostic']
        rec.append({'round':rnd,'navigation_s':r['wall_ms']/1000,'actual_order':protocol['actual_query_order'],'raw_file':str(path/'runs.jsonl'),'raw_sha256':digest(path/'runs.jsonl')})
    confirm.append({'map_id':mid,'query':q,'strict_success':3,'runs':3,'records':rec})
entry=validate(R/'results/pose_default_entry_b/pudu_wanda_3f');assert len(entry)==1 and entry[0]['success']
save('POSE_CASE_CONFIRMATION.json',{'diagnostic_only':True,'cases':confirm,'default_entry_strict_success':True,'default_entry_navigation_s':entry[0]['wall_ms']/1000})

events=[]
for mid in ['pudu_wanda_3f','arena_medium_200x200_2d_005','mentor_map_20260825_005','mengjia','arena_large_250x300_2d_005']:
    root='events_pose_final' if mid=='pudu_wanda_3f' else 'events_pose_checkpoint'
    path=R/'results'/root/mid;summary=read(path/'summary.json');assert summary['assertions_passed']
    journal=read(path.with_suffix('.supervisor.json'));assert journal['returncode']==0 and journal['cleanup']['complete']
    events.append({'map_id':mid,'events':summary['events'],'source':str(path),'all_assertions_passed':True})
journals=list((R/'results').rglob('*.supervisor.json'));bad=[str(p) for p in journals if not read(p).get('cleanup',{}).get('complete')]
assert not bad
save('POSE_RESOURCE_CHECKPOINT.json',{'journals':len(journals),'all_owned_resources_cleaned':True,'events':events,'asserted_events':sum(r['events'] for r in events),'static_replay_only':True})

parent=read(R/'PARENT_MANIFEST.json');failures=[]
for name,sha in parent['files'].items():
    p=P/name
    if not p.is_file() or digest(p)!=sha:failures.append(str(p))
counts={}
for key in ('source_files','input_files','pixel_input_files'):
    registered=read(R/'L1_LOCK.json')[key];counts[key]=len(registered)
    for name,sha in registered.items():
        p=R/name
        if not p.is_file() or digest(p)!=sha:failures.append(str(p))
protected=[]
for name,sha in parent['files'].items():
    if name.startswith(('native/','config/','inputs/','datasets/','install/')) or name=='MAPS.json':
        p=R/name
        if not p.is_file() or digest(p)!=sha:failures.append(str(p))
        protected.append(name)
assert not failures,failures
save('POSE_PROTECTED_INPUTS.json',{'parent_files_verified':len(parent['files']),'frozen_L1':counts,'protected_inherited_files':len(protected),'parent_unchanged':True,'failures':failures})
core={}
for name in ('src','native','config','install'):
    for p in (R/name).rglob('*'):
        if p.is_file() and '__pycache__' not in str(p):core[str(p.relative_to(R))]=digest(p)
save('POSE_FINAL_CHECKPOINT_SOURCE.json',core)
print({'strict_success':99,'requests':100,'confirmed_cases':len(confirm),'asserted_events':sum(r['events'] for r in events),'owned_clean_batches':len(journals),'protected_parent_files':len(parent['files']),'formal_performance':False})
