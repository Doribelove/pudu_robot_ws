"""Audit complete paired records, protected inputs and actual output equality."""
import hashlib
import json
from pathlib import Path
import numpy as np
from analyze_v10r3 import analyze, ROOT, PARENT


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    folder=ROOT/'results/paired_final'
    result=analyze(folder)
    assert result['complete']
    manifest=json.loads((ROOT/'PARENT_MANIFEST.json').read_text())
    assert sha(ROOT/'PARENT_MANIFEST.json')==sha(PARENT/'DELIVERY_MANIFEST.json')
    for name,digest in manifest['files'].items():assert sha(PARENT/name)==digest,('PARENT_CHANGED',name)
    frozen=json.loads((ROOT/'L1_LOCK.json').read_text());locked={}
    for group in ('source_files','input_files','pixel_input_files'):
        for name,digest in frozen[group].items():
            p=Path(name) if Path(name).is_absolute() else ROOT/name
            assert sha(p)==digest,('FROZEN_L1_CHANGED',group,name)
        locked[group]=len(frozen[group])
    core=json.loads((folder/'source.json').read_text())
    for name,digest in core.items():assert sha(name)==digest,('FORMAL_SOURCE_CHANGED',name)
    assert sha(ROOT/'config/l3.json')==sha(PARENT/'config/l3.json')
    assert sha(ROOT/'MAPS.json')==sha(PARENT/'MAPS.json')
    for p in (PARENT/'native').rglob('*'):
        if p.is_file():assert sha(p)==sha(ROOT/p.relative_to(PARENT)),('OLD_NATIVE_CHANGED',str(p))
    changed=[]
    for p in (PARENT/'src').rglob('*.py'):
        rel=p.relative_to(PARENT)
        if sha(p)!=sha(ROOT/rel):changed.append(str(rel))
    expected={'src/three_yd_v5/curve_backend.py','src/three_yd_v5/fast_state.py',
              'src/three_yd_v5/v9_r9_membership.py','src/three_yd_v5/controller.py',
              'src/three_yd_v5/fullroute_service.py','src/three_yd_v5/fast_costmap.py','src/three_yd_v5/first_ack.py','src/three_yd_v5/curve_interpolation.py'}
    assert set(changed)==expected,changed
    relocation='src/three_yd_v5/curve_backend.py'
    assert (ROOT/relocation).read_text().replace(str(ROOT),str(PARENT)).replace('3YDL3-V10-r3','3YDL3-V10-r2').replace('3ydl3_v10_r3','3ydl3_v10_r2')==(PARENT/relocation).read_text()
    assert '896 passed' in (ROOT/'report/tests_final.log').read_text()
    maps=json.loads((ROOT/'MAPS.json').read_text())['maps']
    for m in maps:
        for name,digest in m['input_sha256'].items():assert sha(name)==digest,('MAP_OR_Q_CHANGED',name)
    quality=[];failures=[];snapshots=0
    grid_keys=('semantic_expected_grid_hash','semantic_expected_master_hash',
               'semantic_expected_server_content_hash','server_costmap_content_hash',
               'r5_native_shape','r5_source_full_grid_hash','r5_source_full_master_hash','r5_source_full_mask_hash')
    for rnd in range(1,4):
        for m in maps:
            base=folder/f'round{rnd}'/m['map_id'];rows={};protocol={}
            for arm,source in [('r2',PARENT),('r3',ROOT)]:
                out=base/arm
                rows[arm]=list(map(json.loads,(out/'runs.jsonl').read_text().splitlines()))
                assert len(rows[arm])==10 and [r['query_id'] for r in rows[arm]]==m['query_ids']
                assert json.loads((out/'summary.json').read_text())['continuation_hits']==0
                protocol[arm]=json.loads((out/'protocol.json').read_text())
                p=protocol[arm]
                assert p['CPU']==[14] and p['repeats']==1 and p['workload']=='fixed'
                assert not p['profiler'] and not p['diagnostic_subset'] and not p['query_order_diagnostic']
                assert p['r3_request_mode']==('control' if arm=='r2' else 'ABRTHI')
                assert p['export_controller_references_released_before_next_request']
                assert not (out/'startup.json').exists()
                assert all('cold_first_response_ms' not in r for r in rows[arm])
                for name,digest in json.loads((out/'source_snapshot/hashes.json').read_text()).items():
                    rel=Path(name).relative_to(source)
                    assert sha(out/'source_snapshot'/str(rel).replace('/','_'))==digest
                    assert sha(source/rel)==digest
                snapshots+=1
            for key in ('map_sha256','queries_sha256','curve_config_sha256','vehicle','common_harness_sha256'):
                assert protocol['r2'][key]==protocol['r3'][key],(m['map_id'],key)
            for old,new in zip(rows['r2'],rows['r3']):
                q=old['query_id'];assert old['start']==new['start'] and old['goal']==new['goal']
                if not(old['success'] and new['success']):
                    failures.append({'round':rnd,'map_id':m['map_id'],'query_id':q,
                        'r2_success':old['success'],'r3_success':new['success'],
                        'r2_failure':old['failure_code'],'r3_failure':new['failure_code']})
                    continue
                paths={a:base/a/'cases'/f'{q}_1' for a in ('r2','r3')}
                equality={name:sha(paths['r2']/name)==sha(paths['r3']/name) for name in
                          ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','path.npy')}
                for key in grid_keys:assert old['l3'][key]==new['l3'][key],(m['map_id'],q,key)
                with np.load(paths['r2']/'l3_scope.npz') as a,np.load(paths['r3']/'l3_scope.npz') as b:
                    assert np.array_equal(a['mask'],b['mask'])
                for r in (old,new):
                    assert r['audit']['valid'] and r['curve_l3']['certificate']['max_curvature_bound_1pm']<=2.50000001
                quality.append({'round':rnd,'map_id':m['map_id'],'query_id':q,
                                'equal_files':equality,'scope_and_effective_costmap_equal':True})
    events=[]
    for mid in ('mengjia','arena_large_250x300_2d_005'):
        out=ROOT/'results/events_final'/mid
        s=json.loads((out/'summary.json').read_text());j=json.loads(out.with_suffix('.supervisor.json').read_text())
        assert s['assertions_passed'] and s['r3_request_mode']=='ABRTHI' and j['cleanup']['complete'] and j['returncode']==0
        events.append(s)
    result={'parent_files_unchanged':len(manifest['files']),'frozen_L1':locked,
        'formal_source_files_locked':len(core),'configuration_and_inherited_native_libraries_unchanged':True,
        'changed_inherited_sources':changed,'verified_source_snapshots':snapshots,
        'common_success_outputs':quality,'failures':failures,'event_summaries':events,
        'strict_success':result['strict'],'gates':result['gates'],'accepted':result['accepted'],
        'project_tests':896,'static_replay_only':True,
        'new_native_lookup_sha256':sha(ROOT/'native/librequest_membership.so')}
    (ROOT/'report/SOURCE_REVIEW.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,default=lambda x:x.item())+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('common_success_outputs','failures','event_summaries')},ensure_ascii=False,indent=2,default=lambda x:x.item()))


if __name__=='__main__':main()
