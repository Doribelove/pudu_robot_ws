"""Immutable per-case profile evidence; overlapping clocks stay separate."""
from pathlib import Path
import statistics
import pstats
from compare_v9_review import load, save, sha

ROOT=Path(__file__).resolve().parents[1]


def main():
    review_path=ROOT/'report/SOURCE4_SHARED_PROFILE_REVIEW.json'
    review=load(review_path)
    assert review['strict_total']==48 and review['diagnostic_only'] and review['profiler']
    protocol=load(ROOT/'report/SOURCE4_SHARED_PROFILE_PROTOCOL.json')
    output={'source_cycle':4,'diagnostic_only':True,'profiler':True,'formal_performance_claim':False,
        'review_sha256':sha(review_path),'no_summing_overlapping_cumulative_timers':True,'maps':{}}
    for mid,map_info in review['maps'].items():
        output['maps'][mid]={}
        for mode,info in map_info['modes'].items():
            output['maps'][mid][mode]={}
            for arm,item in info['arms'].items():
                raw=[]
                for row in item['raw']:
                    profile_path=Path(protocol['root'])/arm/mode/mid/'cases'/row['tag']/'query.prof'
                    assert sha(profile_path)==row['profile']['profile_sha256']
                    functions=[]
                    for (file,line,name),(primitive,calls,self_s,cum_s,callers) in pstats.Stats(str(profile_path)).stats.items():
                        if (name in ('semantic_map_hash','asdict','canonical_hash','_route_cache_key','_bind_endpoint_parking_access') or
                            (Path(file).name in ('v9_segments.py','v9_local.py','v9_risk.py','fast_costmap.py') and name in ('cache_for','propose','store','plan','compose_bounded','risk_segments')) or
                            any(f['file']==file and f['line']==line and f['function']==name for f in row['profile']['functions'])):
                            functions.append({'file':file,'line':line,'function':name,'calls':calls,
                                'self_ms':1000*self_s,'cumulative_ms':1000*cum_s,
                                'callers':[{'file':key[0],'line':key[1],'function':key[2],
                                    'primitive_calls':values[0],'calls':values[1],
                                    'self_ms':1000*values[2],'cumulative_ms':1000*values[3]}
                                    for key,values in callers.items()]})
                    raw.append({'tag':row['tag'],'profile_sha256':sha(profile_path),
                        'strict':row['strict_success'],'local':row['local_accepted_in_strict_task'],
                        'segment':row['segment_accepted_in_strict_task'],'functions':functions})
                new=raw[1:]
                medians={}
                for key in [('semantic_map.py','semantic_map_hash'),('v9_segments.py','cache_for'),
                            ('v9_segments.py','propose'),('v9_segments.py','store'),('v9_local.py','plan')]:
                    values=[f for row in new for f in row['functions'] if (Path(f['file']).name,f['function'])==key]
                    if values:
                        medians['.'.join(key)]={'n_related_tasks':3,'calls':[f['calls'] for f in values],
                            'self_ms_median':statistics.median(f['self_ms'] for f in values),
                            'cumulative_ms_median':statistics.median(f['cumulative_ms'] for f in values)}
                checkers=[f for row in item['raw'][1:] for f in row['profile']['functions'] if f['label']=='SweptChecker.__init__']
                medians['SweptChecker.__init__']={'n_related_tasks':3,'calls':[f['calls'] for f in checkers],
                    'cumulative_ms_median':statistics.median(f['cumulative_ms'] for f in checkers)}
                output['maps'][mid][mode][arm]={'raw':raw,'new_task_function_medians_diagnostic':medians,
                    'new_task_local_strict':sum(row['local'] for row in new),
                    'new_task_segment_strict':sum(row['segment'] for row in new)}
    output['inference']={
        'checker_creation_small_relative_to_total_and_not_chosen_for_new_planner_change':True,
        'semantic_hash_property_current_fresh_serialization_repeated_6_A_or8_ABP_calls':True,
        'future_experiment_may_reduce_representation_copying_but_must_read_current_fields_and_preserve_exact_hash':True,
        'no_hash_or_audit_or_ACK_cache_proposed':True,
        'original_world_semantic_hash_is_mutable_metadata_never_assumed_frozen':True}
    path=ROOT/'report/SOURCE4_SHARED_PROFILE_COST_ANALYSIS.json'
    assert not path.exists()
    save(path,output)
    print('profile48 per-case costs/callers saved; original semantic serialization next hypothesis, no formal timing claim')


if __name__=='__main__':main()
