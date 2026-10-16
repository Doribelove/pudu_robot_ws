"""Summarize the fixed four-map V7 L1-only records; never rerun navigation."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'results/formal_l1_10x1_graph'
REPORT = ROOT / 'report'
MAPS = (
    'arena_medium_200x200_2d_005',
    'mentor_map_20260825_005',
    'mentor_map_20260825_005_4x_area',
    'arena_large_250x300_2d_005',
)
COLORS = ('#2274A5', '#E09F3E', '#4F9D69', '#8D5A97')


def stats(values):
    a = np.asarray(values, float)
    return {'median': float(np.median(a)), 'p95': float(np.quantile(a, .95)),
            'min': float(a.min()), 'max': float(a.max()),
            'q1': float(np.quantile(a, .25)), 'q3': float(np.quantile(a, .75))}


def boxplot(series, path, *, log=False):
    # SimSun is unavailable on this host.  Render no text in the graphic;
    # names, units, sample counts, quartiles and whisker convention are given
    # in the immediately adjacent report caption/table.
    fig, ax = plt.subplots(figsize=(8.4, 3.2))
    items = ax.boxplot(series, vert=False, positions=np.arange(1, 5),
                       widths=.48, whis=1.5, patch_artist=True,
                       showfliers=True)
    for patch, color in zip(items['boxes'], COLORS):
        patch.set_facecolor(color)
        patch.set_alpha(.70)
    for element in ('whiskers', 'caps', 'medians'):
        for artist in items[element]:
            artist.set_color('#24323C')
    for artist in items['fliers']:
        artist.set_markerfacecolor('#24323C')
        artist.set_markeredgecolor('#24323C')
        artist.set_markersize(4)
    if log:
        ax.set_xscale('log')
    ax.set_yticks([])
    ax.set_xticks([])
    ax.set_xlim(left=min(min(a) for a in series)*.8,
                right=max(max(a) for a in series)*1.18)
    ax.grid(axis='x', color='#CCD4D8', alpha=.45)
    for spine in ax.spines.values():
        spine.set_visible(False)
    fig.tight_layout(pad=.3)
    fig.savefig(path, format='svg')
    fig.savefig(path.with_suffix('.png'), dpi=160)
    plt.close(fig)


def main():
    REPORT.mkdir(exist_ok=True)
    summary = {'version': '3YD-V7-r0', 'scope': 'L1 only', 'sample_count_per_map': 10,
               'boxplot_whisker': 'last sample within 1.5*IQR; farther samples are outliers',
               'maps': {}}
    time_series, pss_series = [], []
    for map_id in MAPS:
        source = SOURCE / map_id / 'runs.json'
        record = json.loads(source.read_text())
        queries = record['queries']
        if len(queries) != 10:
            raise ValueError(f'{map_id}: EXPECTED_TEN_QUERIES')
        times = [q['wall_ms'] for q in queries]
        pss = [q['memory_after']['pss_bytes'] / 2**20 for q in queries]
        rss = [q['memory_after']['rss_bytes'] / 2**20 for q in queries]
        time_series.append(times)
        pss_series.append(pss)
        summary['maps'][map_id] = {
            'source': str(source),
            'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            'query_ids': record['query_ids'],
            'l1_valid': sum(bool(q['success']) for q in queries),
            'query_wall_ms': stats(times), 'query_cpu_ms': stats([q['cpu_ms'] for q in queries]),
            'query_process_pss_mib': stats(pss),
            'query_process_rss_mib': stats(rss),
            'process_peak_rss_mib': max(record['memory_final']['peak_rss_bytes'],
                                        record['memory_final']['rss_bytes'])/2**20,
            'world_load_ms': record['world_init_ms'],
            'l1_graph_init_ms': record['l1_graph_init_ms'],
            'post_init_process_pss_mib': record['memory_after_init']['pss_bytes']/2**20,
            'post_init_process_rss_mib': record['memory_after_init']['rss_bytes']/2**20,
            'search_methods': {method: sum(q['diagnostics'].get('search') == method
                                           for q in queries)
                               for method in ('direct', 'coarse_astar')},
        }
    (REPORT/'L1_STATISTICS.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    boxplot(time_series, REPORT/'L1_QUERY_TIME_BOXPLOT.svg', log=True)
    boxplot(pss_series, REPORT/'L1_PROCESS_PSS_BOXPLOT.svg')
    for map_id, row in summary['maps'].items():
        print(map_id, row['l1_valid'],
              round(row['query_wall_ms']['median'], 3),
              round(row['query_wall_ms']['p95'], 3),
              round(row['query_process_pss_mib']['median'], 3),
              round(row['process_peak_rss_mib'], 3))


if __name__ == '__main__':
    main()
