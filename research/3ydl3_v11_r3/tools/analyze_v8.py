"""Summarize actual V8 calls and render registered raw-data distributions."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MAPS = [('mengjia', '万达3w地库（语义）'),
        ('arena_large_250x300_2d_005', 'arena_large（无语义）')]


def stats(values):
    values = [float(x) for x in values if x is not None and np.isfinite(x)]
    if not values:
        return {'n': 0, 'median': None, 'p95': None, 'max': None}
    return {'n': len(values), 'median': float(np.median(values)),
            'p95': float(np.quantile(values, .95)), 'max': float(max(values)),
            'q1': float(np.quantile(values, .25)), 'q3': float(np.quantile(values, .75))}


def measure(row, name):
    recovery = row.get('recovery', {})
    attempts = recovery.get('attempts', [])
    if name == 'e2e_s':
        return row.get('wall_ms', 0)/1000
    fields = {'l1_ms': 'l1_actual_ms', 'l2_s': 'all_candidates_controller_ms',
              'adapter_s': 'all_candidates_adapter_ms', 'l3_s': 'all_candidates_l3_pipeline_ms'}
    if name in fields:
        value = recovery.get(fields[name])
        return value if value is None or name == 'l1_ms' else value/1000
    if name == 'dstar_s':
        return sum(a.get('l2_timing', {}).get('l2_current_search_response_ms', 0.)
                   for a in attempts)/1000 if 'attempts' in recovery else None
    if name in ('map_sync_s', 'smac_action_s', 'curvature_repair_s', 'strict_audit_s'):
        if 'attempts' not in recovery:
            return None
        if name == 'curvature_repair_s':
            return sum(a.get('curvature_repair_ms', 0.)+a.get('local_geometry_repair_ms', 0.)
                       for a in attempts)/1000
        if name == 'strict_audit_s':
            return sum(a.get('l2_timing', {}).get('strict_audit_total_ms', 0.)
                       for a in attempts)/1000
        field = 'local_map_update_ms' if name == 'map_sync_s' else 'l3_action_wall_ms'
        return sum(a.get('l3_diagnostics', {}).get(field, 0.) for a in attempts)/1000
    if name == 'tree_rss_gib':
        return row['rss_peak_bytes']/2**30
    if name == 'python_pss_gib':
        values = [p['pss'] for p in row.get('memory_processes', []) if p['name'] == 'python3']
        return sum(values)/2**30 if values else None
    if name == 'l2_state_mib':
        value = row.get('l2', {}).get('state_memory_bytes')
        return None if value is None else value/2**20
    if name in ('planner_pss_gib', 'stack_pss_gib'):
        field = 'planner_pss_peak_bytes' if name == 'planner_pss_gib' else 'stack_pss_peak_bytes'
        values = [a.get('l3_diagnostics', {}).get(field) for a in attempts]
        values = [v for v in values if v is not None]
        return max(values)/2**30 if values else None
    raise KeyError(name)


METRICS = ['e2e_s', 'l1_ms', 'l2_s', 'dstar_s', 'adapter_s', 'l3_s', 'map_sync_s',
           'smac_action_s', 'curvature_repair_s', 'tree_rss_gib', 'python_pss_gib',
           'l2_state_mib', 'planner_pss_gib', 'stack_pss_gib']


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, default=ROOT/'results/formal')
    parser.add_argument('--output', type=Path, default=ROOT/'report')
    args = parser.parse_args()
    args.output.mkdir(exist_ok=True, parents=True)
    data = {}
    summary = {}
    for map_id, label in MAPS:
        path = args.input/map_id/'runs.jsonl'
        rows = [json.loads(s) for s in path.read_text().splitlines() if s.strip()]
        if len(rows) != 30 or len({r['query_id'] for r in rows}) != 10:
            raise ValueError('INCOMPLETE_FORMAL_DATA:'+map_id)
        data[map_id] = rows
        success = [r for r in rows if r['success']]
        bad_evidence = []
        for r in success:
            if not r.get('audit', {}).get('valid'):
                bad_evidence.append([r['query_id'], r['repeat'], 'independent audit'])
            if r.get('constraints', {}).get('steering_jump_valid') is not True:
                bad_evidence.append([r['query_id'], r['repeat'], 'steering'])
            if r.get('constraints', {}).get('traffic', {}).get('valid') is not True:
                bad_evidence.append([r['query_id'], r['repeat'], 'traffic/mode'])
            d = r.get('l3', {})
            if not d.get('costmap_update_acknowledged') or d.get('costmap_ack_mismatch_cells') != 0:
                bad_evidence.append([r['query_id'], r['repeat'], 'map ACK'])
            bounds = r.get('bounded_l3', {})
            if bounds.get('global_fallback') is not False or bounds.get('max_sample_radius_m', 2) > 1.2:
                bad_evidence.append([r['query_id'], r['repeat'], 'search bound'])
        if bad_evidence:
            raise ValueError('SUCCESS_WITH_INCOMPLETE_EVIDENCE:'+str(bad_evidence))
        failed = [{'query_id': r['query_id'], 'repeat': r['repeat'], 'failure': r['failure_code'],
                   'attempts': [{'rank': a.get('route_rank'), 'failure': a.get('failure'),
                                'termination_reason': a.get('l3_diagnostics', {}).get('termination_reason'),
                                'failure_cache_hit': a.get('failure_cache_hit')}
                               for a in r.get('recovery', {}).get('attempts', [])]}
                  for r in rows if not r['success']]
        calls = [a.get('l3_diagnostics', {}) for r in rows
                 for a in r.get('recovery', {}).get('attempts', [])
                 if a.get('l3_diagnostics')]
        per_query = {qid: sum(r['success'] for r in rows if r['query_id'] == qid)
                     for qid in dict.fromkeys(r['query_id'] for r in rows)}
        result = {'label': label, 'input_file': str(path),
                  'input_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                  'strict_success': len(success), 'runs': len(rows),
                  'distinct_queries': len(per_query), 'queries_passed_all_repeats': sum(n == 3 for n in per_query.values()),
                  'per_query_success': per_query, 'failures': failed,
                  'verified_map_calls': sum(d.get('costmap_update_acknowledged') and
                       d.get('costmap_ack_mismatch_cells') == 0 for d in calls),
                  'map_calls': len(calls), 'success_evidence_checked': len(success),
                  'preparation': json.loads((path.parent/'preparation.json').read_text()),
                  'startup': json.loads((path.parent/'session_startup.json').read_text()),
                  'metrics': {m: {part: stats([measure(r, m) for r in selected])
                      for part, selected in [('all', rows), ('first', [r for r in rows if r['repeat'] == 1]),
                          ('repeat', [r for r in rows if r['repeat'] > 1]), ('success', success)]}
                      for m in METRICS},
                  'right_side_fraction': stats([r.get('right_side_fraction') for r in success]),
                  'path_length_m': stats([r.get('length_m') for r in success])}
        summary[map_id] = result
    (args.output/'STATISTICS.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font = Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc')
    font_manager.fontManager.addfont(str(font))
    if font_manager.FontProperties(fname=str(font)).get_name() != 'SimSun':
        raise ValueError('SIMSUN_REQUIRED')
    plt.rcParams.update({'font.family': 'SimSun', 'axes.unicode_minus': False, 'font.size': 13})
    for filename, panels in [
        ('timing_boxplots.png', [('l1_ms', 'L1路线选择 (ms)'), ('l2_s', 'L2整体处理 (s)'),
            ('adapter_s', 'L2→L3适配 (s)'), ('l3_s', 'L3流水线，含适配 (s)'),
            ('e2e_s', '端到端 (s)'), ('map_sync_s', '地图更新与确认 (s)')]),
        ('memory_boxplots.png', [('l2_state_mib', 'L2注册状态 (MiB)'),
            ('python_pss_gib', '工作Python进程PSS快照 (GiB)'),
            ('planner_pss_gib', 'Smac进程峰值PSS (GiB)'),
            ('stack_pss_gib', '服务栈峰值PSS，含Smac (GiB)'),
            ('tree_rss_gib', '完整进程树查询峰值RSS (GiB)')])]:
        figure, axes = plt.subplots(2, 3, figsize=(17, 10))
        for ax, (metric, title) in zip(axes.flat, panels):
            values = [[measure(r, metric) for r in data[mid]] for mid, _ in MAPS]
            values = [[v for v in group if v is not None] for group in values]
            boxes = ax.boxplot(values, labels=[label+'\nn='+str(len(v))
                for (_, label), v in zip(MAPS, values)], patch_artist=True, showfliers=True)
            for box, color in zip(boxes['boxes'], ['#83b8d8', '#efbc78']):
                box.set_facecolor(color)
            ax.set_title(title, fontweight='bold')
            ax.grid(axis='y', alpha=.25)
        for ax in list(axes.flat)[len(panels):]:
            ax.axis('off')
        figure.suptitle('3YD-V8-r2 两模式完整三层：固定10组×3次', fontsize=20, fontweight='bold')
        figure.text(.5, .015, '箱体Q1-Q3；中线中位数；须为1.5 IQR内最远点；圆点为离群值。\n'
            '线性坐标；失败查询计入，缺失指标不填0。不同地图负载不构成模式速度配对。',
            ha='center', fontsize=13)
        figure.tight_layout(rect=[0, .07, 1, .95])
        figure.savefig(args.output/filename, dpi=170)
        plt.close(figure)
    print(json.dumps({mid: {'strict_success': s['strict_success'],
        'runs': s['runs'], 'e2e': s['metrics']['e2e_s'],
        'tree_rss': s['metrics']['tree_rss_gib']['all']}
        for mid, s in summary.items()}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
