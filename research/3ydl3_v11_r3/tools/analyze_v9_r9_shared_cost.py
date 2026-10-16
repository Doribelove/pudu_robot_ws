"""Read existing source2 shared diagnostics; overlapping timers are not summed."""
import argparse
import json
import statistics
from pathlib import Path
from compare_v9_review import load, save, sha
from compare_v9 import read
from review_v9_r9_source2 import pose

ROOT = Path(__file__).resolve().parents[1]
MAPS = ('mengjia', 'arena_large_250x300_2d_005')
MODES = ('control', 'P', 'L', 'PL')
TIMERS = ('l2_total_ms', 'r11_initial_masks_ms', 'r11_l2_parking_bind_ms',
          'r11_l2_search_ms', 'adapter_actual_ms', 'strict_audit_total_ms')


def summarize(rows):
    fields = ('wall_ms', *TIMERS, 'l3_action_wall_ms', 'local_map_update_ms',
              'local_wall_ms', 'analytic_preflight_ms', 'connector_fit_ms')
    return {
        'n':len(rows),
        'strict_success':sum(r['strict_success'] for r in rows),
        'complete_task_local_accepted':sum(r['complete_task_local_accepted'] for r in rows),
        'complete_task_segment_accepted':sum(r['complete_task_segment_accepted'] for r in rows),
        'backend_calls':sum(r['backend_calls'] for r in rows),
        'timers_ms':{name:{'n_observed':len(values), 'median':statistics.median(values) if values else None,
                          'raw':values}
                     for name in fields
                     for values in [[r[name] for r in rows if r[name] is not None]]}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--report-path', type=Path, default=ROOT/'report/SOURCE2_SHARED_COST_DIAGNOSTIC.json')
    args = parser.parse_args()
    original = ROOT/'report/SOURCE2_SHARED_REVIEW.json'
    reviewed = load(original)
    assert reviewed['source_cycle'] == 2 and reviewed['strict_total'] == 64 and reviewed['diagnostic_only']
    report = {'source_cycle':2, 'read_only_historical_diagnostic':True,
              'current_source4_performance_claim':False, 'new_Native_records':0,
              'timers_overlap_and_cannot_be_summed':True,
              'n3_related_new_tasks_not_full10_or_tail_acceptance':True,
              'original_review_sha256':sha(original), 'maps':{}}
    count = 0
    for mid in MAPS:
        fixture = ROOT/'datasets/v9_shared_segments_v2'/(mid+'.json')
        definition = load(fixture)
        assert len(definition['queries']) == 4 and not definition['fixed10']
        assert sha(Path(definition['source_path'])) == definition['source_sha256']
        info = {'fixture_sha256':sha(fixture), 'arms':{}, 'ABP_minus_A_new_task_median_pct':{}}
        for arm in ('A', 'ABP'):
            info['arms'][arm] = {}
            for mode in MODES:
                directory = ROOT/'results/shared_source2/shared'/mode/arm/mid
                rows = read(directory)
                protocol = load(directory/'protocol.json')
                assert protocol['query_file'] == str(fixture) and protocol['queries_sha256'] == sha(fixture)
                assert protocol['r2_mode'] == ('control' if arm == 'A' else 'P')
                proof = reviewed['maps'][mid]['arms'][arm]['source_native_cleanup'][mode]
                assert proof['runs_sha256'] == sha(directory/'runs.jsonl')
                raw = []
                for row, query in zip(rows, definition['queries']):
                    assert row['query_id'] == query['id'] and row['start'] == pose(query['start']) and row['goal'] == pose(query['goal'])
                    assert row['success'] and not row['fullroute'].get('reused') and not row['task_progress'].get('reused')
                    timing = row['timing']; local = row.get('v9_local') or {}; diagnostics = row.get('l3') or {}
                    segments = local.get('segments') or {}; connections = local.get('connections') or []
                    raw.append({'query_id':row['query_id'], 'strict_success':row['success'], 'wall_ms':row['wall_ms'],
                        **{name:timing.get(name) for name in TIMERS},
                        'l3_action_wall_ms':diagnostics.get('l3_action_wall_ms'),
                        'local_map_update_ms':diagnostics.get('local_map_update_ms'),
                        'local_wall_ms':local.get('wall_ms'), 'analytic_preflight_ms':local.get('preflight_ms'),
                        'connector_fit_ms':sum((c.get('anchor_fit') or {}).get('wall_ms',0) for c in connections) if connections else None,
                        'backend_calls':diagnostics.get('backend_call_count',0),
                        'complete_task_local_accepted':bool(row['success'] and local.get('success')),
                        'complete_task_segment_accepted':bool(row['success'] and local.get('success') and segments.get('matched_segments',0)>0),
                        'matched_segments':segments.get('matched_segments',0),
                        'connections':len(connections),
                        'source_planning_files':proof['planning_files']})
                assert len(rows) == len(raw) == 4
                count += len(rows)
                info['arms'][arm][mode] = {'runs_sha256':sha(directory/'runs.jsonl'), 'raw':raw,
                                           'initial':summarize(raw[:1]), 'new_tasks':summarize(raw[1:])}
        for mode in MODES:
            before = info['arms']['A'][mode]['new_tasks']['timers_ms']['wall_ms']['median']
            after = info['arms']['ABP'][mode]['new_tasks']['timers_ms']['wall_ms']['median']
            info['ABP_minus_A_new_task_median_pct'][mode] = 100*(after/before-1)
        report['maps'][mid] = info
    assert count == 64
    report['existing_strict_records_inspected'] = count
    report['next_action'] = ('Profile original shared fixtures in contemporary source4 with A/ABP and existing AB+PM data-preparation configuration; '
        'inspect checker construction/adapter/masks and segment proposal separately before any planner edits. '
        'Preserve current whole-task body/traffic/curvature audits, real ACK, search bounds, all-map speed/memory gates and default B/P off.')
    save(args.report_path.resolve(),report)
    print('READONLY existing64, no newNative, overlapping timers not additive')
    for mid, info in report['maps'].items():
        print(mid, 'newtask n3 ABP change', info['ABP_minus_A_new_task_median_pct'])
        for arm in ('A','ABP'):
            z = info['arms'][arm]['PL']['new_tasks']
            print(arm,'PL','strict/local/segment',z['strict_success'],z['complete_task_local_accepted'],z['complete_task_segment_accepted'],
                  {name:v['median'] for name,v in z['timers_ms'].items()})


if __name__ == '__main__':
    main()
