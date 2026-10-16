"""Render actual per-case L1/L2/final trajectories with SimSun labels."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    font = '/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
    font_manager.fontManager.addfont(font)
    if font_manager.FontProperties(fname=font).get_name() != 'SimSun':
        raise ValueError('SIMSUN_REQUIRED')
    plt.rcParams.update({'font.family': 'SimSun', 'axes.unicode_minus': False, 'font.size': 12})
    for mid, label in [('mengjia', '万达3w地库：语义模式'),
                       ('arena_large_250x300_2d_005', 'arena_large：无语义模式')]:
        run = ROOT/'results/formal'/mid
        protocol = json.loads((run/'protocol.json').read_text())
        map_path = Path(protocol['map_file'])
        info = yaml.safe_load(map_path.read_text())
        pixels = np.asarray(Image.open(map_path.parent/info['image']).convert('L'))
        resolution = float(info['resolution'])
        ox, oy, _ = info['origin']
        extent = [ox, ox+pixels.shape[1]*resolution, oy, oy+pixels.shape[0]*resolution]
        rows = [json.loads(s) for s in (run/'runs.jsonl').read_text().splitlines()]
        rows = [r for r in rows if r['repeat'] == 1]
        fig, axes = plt.subplots(2, 5, figsize=(23, 10))
        for ax, row in zip(axes.flat, rows):
            case = run/'cases'/(row['query_id']+'_1')
            ax.imshow(pixels, origin='upper', extent=extent, cmap='gray', vmin=0, vmax=255)
            lines = []
            plan = case/'l1_output.json'
            if plan.exists():
                line = np.asarray(json.loads(plan.read_text())['right_reference_polyline'])
                ax.plot(line[:, 0], line[:, 1], color='#d99b00', lw=2.4, label='L1引导')
                lines.append(line[:, :2])
            reference = case/'l2_reference_xy.npy'
            if reference.exists():
                line = np.load(reference)
                ax.plot(line[:, 0], line[:, 1], color='#1e88e5', lw=1.8, label='L2参考')
                lines.append(line[:, :2])
            final = case/'path.npy'
            if final.exists():
                line = np.load(final)
                ax.plot(line[:, 0], line[:, 1], color='#139f55', lw=1.2, label='L3最终')
                lines.append(line[:, :2])
            if lines:
                points = np.concatenate(lines)
                span = np.ptp(points, axis=0)
                margin = max(3., float(max(span))*.06)
                ax.set_xlim(max(extent[0], points[:, 0].min()-margin),
                            min(extent[1], points[:, 0].max()+margin))
                ax.set_ylim(max(extent[2], points[:, 1].min()-margin),
                            min(extent[3], points[:, 1].max()+margin))
                ax.scatter([lines[0][0, 0]], [lines[0][0, 1]], c='#d22d2d', marker='o', s=25, zorder=10)
                ax.scatter([lines[0][-1, 0]], [lines[0][-1, 1]], c='#8d31b0', marker='*', s=50, zorder=10)
            ax.set_title(row['query_id']+'：'+('严格通过' if row['success'] else '严格失败'), fontweight='bold')
            ax.set_xlabel('世界坐标 x (m)')
            ax.set_ylabel('世界坐标 y (m)')
            ax.set_aspect('equal')
        handles, labels = axes.flat[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc='lower center', ncol=3, bbox_to_anchor=(.5, .02))
        fig.suptitle('3YD-V8-r1 '+label+'；10组首次调用的真实输出', fontsize=21, fontweight='bold')
        fig.text(.5, .005, '红圆点起点、紫星终点；绿色仅表示通过严格终验的最终轨迹。失败不绘制为成功路线。',
                 ha='center', fontsize=13)
        fig.tight_layout(rect=[0, .08, 1, .94])
        fig.savefig(ROOT/'report'/(mid+'_paths.png'), dpi=150)
        plt.close(fig)


if __name__ == '__main__':
    main()
