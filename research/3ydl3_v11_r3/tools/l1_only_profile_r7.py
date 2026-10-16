"""Reproduce the V6-r7 isolated L1 timing and resource figures.

The host lacks SimSun. The imported plotting setup explicitly uses and
discloses Noto Serif CJK SC as a fallback, never as a purported SimSun face.
"""
import csv
import hashlib
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from layer_boxplots import MAPS, ROOT, SHORT_MAPS, FONT_NAME, stats

SOURCE = ROOT / 'results/l1_only_r7_formal'
OUT = ROOT / 'report/l1_only_profile'
MIB = 2 ** 20


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def style(ax, title, unit):
    ax.set_xticks(range(len(MAPS)), SHORT_MAPS)
    ax.set_xlim(-.5, len(MAPS) - .5)
    ax.set_title(title, loc='left', pad=12)
    ax.set_ylabel(unit)
    ax.grid(axis='y', color='#b9c3cc', alpha=.35)
    ax.spines[['top', 'right']].set_visible(False)


def finish(fig, name):
    fig.subplots_adjust(left=.09, right=.985, top=.86, bottom=.2)
    fig.savefig(OUT / name, dpi=180, facecolor='white')
    plt.close(fig)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    by_map = {}
    query_rows = []
    launch_rows = []
    hashes = {}
    for map_id, map_name in MAPS:
        elapsed = []
        starts = []
        per_query = {}
        hashes[map_id] = {}
        for round_index in (1, 2, 3):
            directory = SOURCE / map_id / f'round_{round_index}'
            init_path = directory / 'initialization.json'
            batch_path = directory / 'batch.json'
            init = json.loads(init_path.read_text())
            batch = json.loads(batch_path.read_text())
            hashes[map_id][f'round_{round_index}'] = {
                'initialization_json': sha256(init_path),
                'batch_json': sha256(batch_path),
            }
            assert len(batch['queries']) == 10
            assert [q['query_id'] for q in batch['queries']] == [f'Q{i:02}' for i in range(1, 11)]
            assert batch['version'] == '3YD-V6-r7-L1'
            assert batch['first_budget'] == 50_000 and batch['alternate_budget'] == 10_000
            assert batch['candidate_limit'] == 3 and batch['transition_table'] is True
            assert batch['query_cache_hits'] == 0
            assert batch['L2_L3_initialized'] is False
            assert batch['ROS_initialized'] is False and batch['occupancy_loaded'] is False
            assert init['L2_L3_initialized'] is False and init['occupancy_loaded'] is False
            assert all(q['success'] and q['formal_navigation_success'] is None
                       and not q['query_cache_hit'] for q in batch['queries'])
            for query in batch['queries']:
                elapsed.append(query['wall_ms'])
                per_query.setdefault(query['query_id'], set()).add(query['output_sha256'])
                query_rows.append({
                    'map_id': map_id, 'map_name': map_name,
                    'round': round_index, 'query_id': query['query_id'],
                    'wall_ms': query['wall_ms'], 'cpu_ms': query['cpu_ms'],
                    'candidate_count': query['candidate_count'],
                    'search_expansions': query['search']['total_expansions'],
                    'output_sha256': query['output_sha256'],
                })
            final = batch['final_resources']
            start = {
                'map_id': map_id, 'map_name': map_name, 'round': round_index,
                'initialization_s': init['wall_ms'] / 1000,
                'post_init_incremental_rss_mib':
                    (init['after']['rss_bytes'] - init['before']['rss_bytes']) / MIB,
                'post_init_rss_mib': init['after']['rss_bytes'] / MIB,
                # max guards against occasional ru_maxrss < a simultaneous RSS snapshot.
                'process_peak_rss_mib': max(final['rss_bytes'], final['peak_rss_bytes']) / MIB,
                'adjacency_pairs': init['l1_adjacency_pairs'],
            }
            starts.append(start)
            launch_rows.append(start)
        assert len(elapsed) == 30 and all(len(digests) == 1 for digests in per_query.values())
        by_map[map_id] = {
            'name': map_name, 'n_queries': len(elapsed), 'n_fresh_processes': len(starts),
            'l1_valid_outputs': len(elapsed), 'stable_output_hashes': True,
            'adjacency_pairs': starts[0]['adjacency_pairs'],
            'query_wall_ms': stats(elapsed),
            'initialization_s': stats([x['initialization_s'] for x in starts]),
            'post_init_incremental_rss_mib': stats(
                [x['post_init_incremental_rss_mib'] for x in starts]),
            'process_peak_rss_mib': stats([x['process_peak_rss_mib'] for x in starts]),
        }

    fig, ax = plt.subplots(figsize=(13, 4.6), dpi=180)
    values = [[q['wall_ms'] for q in query_rows if q['map_id'] == map_id]
              for map_id, _ in MAPS]
    box = ax.boxplot(values, positions=range(len(MAPS)), widths=.48,
                     patch_artist=True, whis=1.5, showfliers=True,
                     medianprops={'color': '#202020', 'linewidth': 1.8},
                     flierprops={'marker': 'o', 'markersize': 4,
                                 'markerfacecolor': '#df7521', 'markeredgecolor': '#df7521'})
    for patch in box['boxes']:
        patch.set(facecolor='#e99955', edgecolor='#bc641c', alpha=.72)
    style(ax, '3YD-V6-r7 独立 L1 查询耗时：各图 10 条查询 × 3 次启动', 'ms（对数纵轴）')
    ax.set_yscale('log')
    finish(fig, 'l1_query_time_boxplot.png')

    fig, ax = plt.subplots(figsize=(13, 4.3), dpi=180)
    for i, (map_id, _) in enumerate(MAPS):
        values = [x['initialization_s'] for x in launch_rows if x['map_id'] == map_id]
        ax.scatter(np.array([i-.08, i, i+.08]), values, s=48, color='#3669a8')
        ax.hlines(np.median(values), i-.18, i+.18, color='#202020', linewidth=1.6)
    style(ax, '3YD-V6-r7 独立 L1 一次性初始化：各图 3 次启动', 's')
    finish(fig, 'l1_initialization_points.png')

    fig, ax = plt.subplots(figsize=(13, 4.3), dpi=180)
    for i, (map_id, _) in enumerate(MAPS):
        runs = [x for x in launch_rows if x['map_id'] == map_id]
        ax.scatter(np.array([i-.14, i-.09, i-.04]),
                   [x['post_init_incremental_rss_mib'] for x in runs],
                   s=45, color='#3669a8')
        ax.scatter(np.array([i+.04, i+.09, i+.14]),
                   [x['process_peak_rss_mib'] for x in runs],
                   s=45, color='#e48428')
    style(ax, '3YD-V6-r7 独立 L1 进程资源：各图 3 次启动', 'MiB')
    ax.scatter([], [], s=45, color='#3669a8', label='初始化后净增 RSS')
    ax.scatter([], [], s=45, color='#e48428', label='进程峰值 RSS')
    ax.legend(loc='upper left', frameon=False, ncol=2)
    finish(fig, 'l1_memory_points.png')

    with (OUT / 'query_samples.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=query_rows[0]); writer.writeheader(); writer.writerows(query_rows)
    with (OUT / 'launch_samples.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=launch_rows[0]); writer.writeheader(); writer.writerows(launch_rows)
    summary = {
        'schema': '3YD-V6-r7-L1-only-profile-v1',
        'source': str(SOURCE), 'font_status': f'SimSun unavailable; labeled graphics use {FONT_NAME}',
        'sampling': '5 fixed maps, Q01-Q10, 3 fresh L1-only processes per map; no query cache',
        'box_definition': 'NumPy linear Q1/Q2/Q3; Tukey whiskers to furthest point within 1.5 IQR; circles outliers',
        'memory_definition': 'Process RSS, not L1 heap ownership; peak=max(final current RSS, ru_maxrss)',
        'maps': by_map, 'source_sha256': hashes,
    }
    (OUT / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'query_rows': len(query_rows), 'launches': len(launch_rows),
                      'l1_valid_outputs': sum(x['l1_valid_outputs'] for x in by_map.values()),
                      'out': str(OUT)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
