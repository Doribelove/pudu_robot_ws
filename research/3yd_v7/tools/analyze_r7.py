"""Recompute the r7/r6 five-map fixed50 comparison from raw per-query data."""
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
R6=Path('/home/robot/workspaces/3yd_v6_r6_20260924/results/formal_r6')
R7=ROOT/'results/formal_r7'
MAPS=('pudu_wanda_3f','mengjia','3yd_synth_multiloop_a_v1',
      '3yd_synth_twinwing_b_v1','3yd_synth_trizone_c_v2')
NAMES=('万达2w地库','万达3w地库','合成地库A','合成地库B','合成地库C')
FIELDS={'end_to_end_s':lambda r:r['wall_ms']/1000,
        'l1_ms':lambda r:r['recovery']['l1_actual_ms'],
        'l2_controller_ms':lambda r:r['recovery']['all_candidates_controller_ms'],
        'adapter_ms':lambda r:r['recovery']['all_candidates_adapter_ms'],
        'l3_pipeline_ms':lambda r:r['recovery']['all_candidates_l3_pipeline_ms'],
        'query_tree_rss_gib':lambda r:r['rss_peak_bytes']/2**30}

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def rows(path):
    data=[json.loads(line) for line in path.read_text().splitlines()]
    assert len(data)==10 and {r['query_id'] for r in data}=={f'Q{i:02d}' for i in range(1,11)}
    return {r['query_id']:r for r in data}

def stats(data):
    v=np.asarray(list(data),float)
    return {'median':float(np.median(v)),'p95':float(np.quantile(v,.95)),
            'maximum':float(np.max(v)),'minimum':float(np.min(v))}

def main():
    old=[];new=[];per_map={};problems=[];new_successes=[];original_paths=[]
    protocol_fields=('map_sha256','semantic_hash','queries_sha256','parking_access_sha256',
                     'vehicle','CPU','l2_resolution_m','l3_resolution_m','l3_normal_radius_m',
                     'l3_corner_radius_m','global_L3_fallback','l2_soft_target_offset_m')
    for map_id,name in zip(MAPS,NAMES):
        bp,np_=R6/map_id,R7/map_id
        before,after=rows(bp/'runs.jsonl'),rows(np_/'runs.jsonl')
        old.extend(before.values());new.extend(after.values())
        p0=json.loads((bp/'protocol.json').read_text());p1=json.loads((np_/'protocol.json').read_text())
        mismatches={key:[p0.get(key),p1.get(key)] for key in protocol_fields if p0.get(key)!=p1.get(key)}
        assert not mismatches,(map_id,mismatches)
        assert all(r['l3'].get('costmap_update_acknowledged') is True for r in after.values())
        for query_id,first in before.items():
            second=after[query_id]
            if first['success'] and not second['success']:problems.append([map_id,query_id,'regressed_success'])
            if first['success']:
                a=np.load(bp/'cases'/f'{query_id}_1'/'path.npy')
                b=np.load(np_/'cases'/f'{query_id}_1'/'path.npy')
                original_paths.append({'map_id':map_id,'query_id':query_id,
                                       'pointwise_identical':bool(np.array_equal(a,b)),
                                       'r6_length_m':first['length_m'],
                                       'r7_length_m':second['length_m'],
                                       'r6_right_fraction':first['right_side_fraction'],
                                       'r7_right_fraction':second['right_side_fraction']})
            if not first['success'] and second['success']:
                outcome=json.loads((np_/'cases'/f'{query_id}_1'/'outcome.json').read_text())
                new_successes.append({'map_id':map_id,'query_id':query_id,
                    'length_m':second['length_m'],'right_fraction':second['right_side_fraction'],
                    'repair_ms':sum(a.get('curvature_repair_ms',0.) for a in second['recovery']['attempts']),
                    'raster_boundary_runs':outcome['traffic_audit'].get('r7_road_sequence_diagnostic',{}).get('raster_boundary_runs',[]),
                    'strict_audit':bool(second['audit'].get('valid')),
                    'steering_valid':bool(second['constraints'].get('steering_jump_valid')),
                    'traffic_valid':bool(second['constraints'].get('traffic',{}).get('valid'))})
            if not second['success']:problems.append([map_id,query_id,second['failure_code']])
        per_map[map_id]={'name':name,'strict_r6':sum(r['success'] for r in before.values()),
                         'strict_r7':sum(r['success'] for r in after.values()),
                         'end_to_end_s_r6':stats(FIELDS['end_to_end_s'](r) for r in before.values()),
                         'end_to_end_s_r7':stats(FIELDS['end_to_end_s'](r) for r in after.values()),
                         'rss_gib_r6':stats(FIELDS['query_tree_rss_gib'](r) for r in before.values()),
                         'rss_gib_r7':stats(FIELDS['query_tree_rss_gib'](r) for r in after.values()),
                         'protocol_fields_equal':True,'r6_sha256':sha(bp/'runs.jsonl'),
                         'r7_sha256':sha(np_/'runs.jsonl')}
    assert len(old)==len(new)==50
    assert len(original_paths)==43 and all(x['strict_audit'] and x['steering_valid'] and x['traffic_valid'] for x in new_successes)
    assert sum(r['success'] for r in new)==48
    report={'schema':'3YD-V6-r7-five-map-fixed50-v1',
            'success_r6':sum(r['success'] for r in old),'success_r7':sum(r['success'] for r in new),
            'all_ack_r7':True,'original_success_count':len(original_paths),
            'original_success_pointwise_identical':sum(x['pointwise_identical'] for x in original_paths),
            'original_success_changed':[x for x in original_paths if not x['pointwise_identical']],
            'new_successes':new_successes,'remaining_failures':[x for x in problems if x[2]!='regressed_success'],
            'regressions':[x for x in problems if x[2]=='regressed_success'],
            'overall':{key:{'r6':stats(fn(r) for r in old),'r7':stats(fn(r) for r in new)}
                       for key,fn in FIELDS.items()},'maps':per_map}
    report['overall']['paired_end_to_end_delta_s']=stats(FIELDS['end_to_end_s'](b)-FIELDS['end_to_end_s'](a)
                                                for a,b in zip(old,new))
    report['overall']['paired_rss_delta_gib']=stats(FIELDS['query_tree_rss_gib'](b)-FIELDS['query_tree_rss_gib'](a)
                                               for a,b in zip(old,new))
    out=ROOT/'report/PERFORMANCE.json'
    out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'strict':[report['success_r6'],report['success_r7']],
          'end_to_end':report['overall']['end_to_end_s'],
          'rss':report['overall']['query_tree_rss_gib'],
          'original_same':report['original_success_pointwise_identical'],
          'new':[(x['map_id'],x['query_id']) for x in new_successes]},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
