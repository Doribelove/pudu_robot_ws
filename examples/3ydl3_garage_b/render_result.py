# SPDX-License-Identifier: Apache-2.0
"""Render the public map and exported path. Labels stay in the surrounding docs."""
import argparse
import gzip
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parent


def render(case, output):
    with gzip.open(ROOT / 'map/optemap.pgm.gz', 'rb') as raw:
        image = Image.open(raw).convert('RGB')
    width, height = image.size
    image.thumbnail((800, 800), Image.Resampling.LANCZOS if hasattr(Image, 'Resampling') else Image.LANCZOS)
    sx, sy = image.width / width, image.height / height
    path = np.load(Path(case) / 'path.npy', allow_pickle=False)
    if path.ndim != 2 or path.shape[1] != 3 or len(path) < 2 or not np.isfinite(path).all():
        raise ValueError('Expected a finite N x 3 path')
    # Fixed garage B: origin (0,0), resolution 0.05m; image rows increase down.
    points = [(float(x / .05 * sx), float((height - y / .05) * sy)) for x, y in path[:, :2]]
    draw = ImageDraw.Draw(image)
    draw.line(points, fill=(21, 116, 85), width=3)
    for p, color in [(points[0], (36, 99, 235)), (points[-1], (220, 38, 38))]:
        x, y = p
        draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill=color, outline='white')
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('case', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    render(a.case, a.output)
