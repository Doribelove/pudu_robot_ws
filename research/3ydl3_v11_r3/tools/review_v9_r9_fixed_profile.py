"""Original Q07/Q09 profiling: exact parity, real ACK, no formal timing claim."""
from pathlib import Path
import pstats
import numpy as np
from compare_v9 import read, tag, summary
from compare_v9_review import load, save, sha
from compare_v8_r3 import libraries
from review_v9_r9_diagnostic import source_proof, ROOT
from review_v9_r9_source2 import features, pose


def main():
    spec = load(ROOT/'report/SOURCE2_FAILURE_PROFILE_PROTOCOL.json')
    original = load(Path(spec['original_fixed_input']))['scenarios']
    chosen = {q['id']:q for q in original if q['id'] in spec['queries']}
    assert len(chosen)==2 and sha(Path(spec['original_fixed_input']))==spec['original_queries_sha256']
    directories = {mode:Path(spec['output_root'])/'fixed'/mode/'mengjia' for mode in spec['modes']}
    data = {mode:read(directory) for mode,directory in directories.items()}
    report = {'source_cycle':2,'diagnostic_only':True,'profiler':True,'formal_performance_claim':False,
              'original_queries':['Q07','Q09'],'modes':{},'exact_parity':{}}
    for mode,directory in directories.items():
        protocol = load(directory/'protocol.json')
        assert protocol['profiler'] and protocol['diagnostic_subset'] and protocol['workload']=='fixed' and protocol['repeats']==3
        assert protocol['queries_sha256']==spec['original_queries_sha256']
        rows = data[mode]
        assert len(rows)==6 and {r['query_id'] for r in rows}==set(chosen)
        for qid in chosen:
            assert [r['repeat'] for r in rows if r['query_id']==qid]==[1,2,3]
        for row in rows:
            query = chosen[row['query_id']]
            assert row['start']==pose(query['start']) and row['goal']==pose(query['goal'])
        strict = summary(rows)
        assert strict['success']==6
        profiles=[]
        for row in rows:
            profile_path=directory/'cases'/tag(row)/'query.prof'
            stats=pstats.Stats(str(profile_path))
            functions=[{'file':f,'line':line,'function':name,'primitive_calls':cc,'calls':nc,
                        'self_ms':tt*1000,'cumulative_ms':ct*1000}
                       for (f,line,name),(cc,nc,tt,ct,callers) in stats.stats.items()]
            functions.sort(key=lambda x:x['cumulative_ms'],reverse=True)
            profiles.append({'tag':tag(row),'profile_sha256':sha(profile_path),
                             'instrumented_request_wall_ms':row['wall_ms'],'instrumented_request_CPU_ms':row['cpu_ms'],
                             'functions':functions})
        report['modes'][mode]={'strict':6,'source_native_cleanup':source_proof(directory,mode,cycle=2,profile=True),
                               'actual_features':features(rows,mode),'profiles':profiles}
    before=directories['control']
    old={tag(row):row for row in data['control']}
    for mode in ('P','PL'):
        after=directories[mode]
        assert {Path(f).name:h for f,h in libraries(before).items()}=={Path(f).name:h for f,h in libraries(after).items()}
        checked=[]
        for row in data[mode]:
            key=tag(row);previous=old[key]
            for name in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','path.npy'):
                assert sha(before/'cases'/key/name)==sha(after/'cases'/key/name)
            x=np.load(before/'cases'/key/'l3_scope.npz')['mask']
            y=np.load(after/'cases'/key/'l3_scope.npz')['mask']
            assert np.array_equal(x,y)
            for field in ('actual_cost_hash','path_cost','dstar_expanded','state_memory_bytes'):
                assert previous['l2'][field]==row['l2'][field]
            detail=row['l3']
            for field in ('semantic_expected_grid_hash','semantic_expected_master_hash','server_costmap_content_hash',
                          'r5_source_full_grid_hash','r5_source_full_master_hash','r5_source_full_mask_hash'):
                assert detail[field]==previous['l3'][field]
            import hashlib
            assert detail['r5_source_full_mask_hash']==hashlib.sha256(np.ascontiguousarray(y).tobytes()).hexdigest()
            assert detail['costmap_update_acknowledged'] and detail['costmap_ack_mismatch_cells']==0
            assert detail['semantic_exact_stable_observations']>=2
            assert detail['server_costmap_content_hash']==detail['semantic_expected_server_content_hash']
            assert not detail['r5_original_map_loaded_in_nav2'] and detail['r5_complete_scope_not_rolling']
            r0,r1,c0,c1=detail['r5_complete_task_bounds'];h,w=detail['r5_native_shape']
            assert [h,w]==[r1-r0,c1-c0]
            assert not(y[:r0].any() or y[r1:].any() or y[r0:r1,:c0].any() or y[r0:r1,c1:].any())
            assert row['success'] and row['audit']['valid'] and row['constraints']['traffic']['valid'] and row['constraints']['steering_jump_valid']
            checked.append({'tag':key,'upstream_final_path_effective_map_exact':True,'current_ACK_and_original_audits_checked':True})
        report['exact_parity'][mode]=checked
    report['strict_total']=18
    save(ROOT/'report/PROFILE_FIXED_SOURCE2_REVIEW.json',report)
    print('SOURCE2 ORIGINAL FIXED Q07/Q09 PROFILING18/18 strict/source/native/cleanup; physical outputs/maps exact; NO formal performance claim')
    names={'_retain','resident_bytes','decode','pack_internal','encode','validated_reuse','_audit','_activate_reference','prepare','plan','prepare_grid','compose_bounded','__init__'}
    for mode,info in report['modes'].items():
        for profile in info['profiles']:
            values=[(Path(f['file']).name,f['function'],round(f['cumulative_ms'],2)) for f in profile['functions'] if f['function'] in names]
            print(mode,profile['tag'],values[:24])


if __name__=='__main__':
    main()
