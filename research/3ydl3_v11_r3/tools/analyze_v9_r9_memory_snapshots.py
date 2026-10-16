"""Read original after-request process snapshots, separately from sampled peaks."""
import argparse
from pathlib import Path
import numpy as np
from compare_v9 import read
from compare_v9_review import save, sha


def key(row):
    return row['query_id'], row['moving_start']['step'], row['repeat']


def stats(values):
    values = np.asarray(values, dtype=float)
    return {'n': len(values), 'median': float(np.median(values)),
            'p95': float(np.percentile(values, 95)),
            'min': float(np.min(values)), 'max': float(np.max(values))}


def snapshots(row):
    processes = row['memory_processes']
    assert processes and processes[0]['name'] == 'python3'
    result = {name: {metric: 0. for metric in ('rss', 'pss', 'uss')}
              for name in ('python_worker', 'map_server', 'planner_server', 'other_ros', 'snapshot_tree')}
    for index, proc in enumerate(processes):
        name = ('python_worker' if index == 0 else proc['name']
                if proc['name'] in ('map_server', 'planner_server') else 'other_ros')
        for metric in ('rss', 'pss', 'uss'):
            value = float(proc[metric]) / 2**20
            result[name][metric] += value
            result['snapshot_tree'][metric] += value
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = {'schema': 'v9-r9-after-request-process-snapshots-v1',
              'source_cycle': 3, 'read_only_existing_records': True,
              'diagnostic_only': True, 'unit': 'MiB',
              'scope': 'memory_full_info snapshots after original request and exports, outside request timing',
              'limits': ['Snapshot values are not request sampled peak RSS.',
                         'Sum processes within each original snapshot row before statistics; never sum process medians.',
                         'Process snapshots alone cannot prove buffer ownership or allocator retention.',
                         'Original ten queries, each with three related changed requests; no new runs or resampling.'],
              'sampled_tree_peak_used_for_acceptance': False, 'groups': {}}
    for kind in ('moving', 'perturbed'):
        output['groups'][kind] = {}
        for mid in ('mengjia', 'arena_large_250x300_2d_005'):
            rows = {mode: read(args.results_root/kind/mode/mid) for mode in ('AS', 'control', 'PLM')}
            assert all(len(v) == 40 and all(r['success'] for r in v) for v in rows.values())
            assert all([key(r) for r in rows[m]] == [key(r) for r in rows['AS']] for m in rows)
            item = {'source_run_sha256': {m: sha(args.results_root/kind/m/mid/'runs.jsonl') for m in rows},
                    'modes': {}, 'paired_snapshot_change': {}}
            for mode, records in rows.items():
                raw = [{'query_id': r['query_id'], 'step': r['moving_start']['step'],
                        'repeat': r['repeat'], 'process_count': len(r['memory_processes']),
                        'snapshot_MiB': snapshots(r),
                        'original_sampled_tree_rss_peak_MiB': r['rss_peak_bytes']/2**20,
                        'l2_state_memory_MiB': r['l2']['state_memory_bytes']/2**20}
                       for r in records]
                summary = {}
                for split in ('all', 'initial', 'moving'):
                    subset = raw if split == 'all' else [v for v in raw if (v['step'] == 'initial') == (split == 'initial')]
                    summary[split] = {p: {metric: stats([v['snapshot_MiB'][p][metric] for v in subset])
                                         for metric in ('rss', 'pss', 'uss')}
                                      for p in subset[0]['snapshot_MiB']}
                item['modes'][mode] = {'raw': raw, 'summary': summary}
            for reference in ('AS', 'control'):
                base = item['modes'][reference]['raw']; candidate = item['modes']['PLM']['raw']
                item['paired_snapshot_change'][reference] = {
                    p: {metric: stats([b['snapshot_MiB'][p][metric]-a['snapshot_MiB'][p][metric]
                                       for a, b in zip(base, candidate)])
                        for metric in ('rss', 'pss', 'uss')}
                    for p in base[0]['snapshot_MiB']}
            output['groups'][kind][mid] = item
    save(args.output, output)
    arena = output['groups']['moving']['arena_large_250x300_2d_005']
    for mode, item in arena['modes'].items():
        print(mode, 'snapshot PSS MiB medians',
              {p: round(v['pss']['median'], 2) for p, v in item['summary']['all'].items()})
        print(mode, 'initial Python PSS MiB',
              [(v['query_id'], round(v['snapshot_MiB']['python_worker']['pss'], 2))
               for v in item['raw'] if v['step'] == 'initial'])
    print('Saved read-only diagnostic; request peak memory gate remains separately failed.')


if __name__ == '__main__':
    main()
