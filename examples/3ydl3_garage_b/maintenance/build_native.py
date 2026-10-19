#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Optional developer rebuild of the supplied native components (not a benchmark)."""
from pathlib import Path
import argparse
import os
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--jobs', type=int, default=1)
    a = p.parse_args()
    if a.jobs < 1:
        p.error('--jobs must be positive')
    r = ROOT / 'runtime/r4'
    for src in sorted((r / 'native').glob('*.cpp')):
        if src.stem == 'delta_codec_test':
            continue  # Development-only v1 wire-format probe.
        target = r / 'native' / ('lib' + src.stem + '.so')
        subprocess.run(['g++', '-O3', '-std=c++17', '-fPIC', '-shared', str(src), '-o', str(target)], check=True)
    v1 = ROOT / 'runtime/v1'
    subprocess.run(['g++', '-O3', '-std=c++17', '-fPIC', '-shared', str(v1 / 'native/swept_rect.cpp'),
                    '-o', str(v1 / 'build/libsemantic_sweep.so')], check=True)
    build = ROOT / '.native-build'
    env = dict(os.environ)
    for project, source in [('costmap', r / 'external/nav2_costmap_2d'), ('stack', r / 'native/nav2_stack')]:
        args = ['cmake', '-S', str(source), '-B', str(build / project), '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=OFF']
        if project == 'costmap':
            args += ['-DCMAKE_INSTALL_PREFIX=' + str(r / 'install/nav2_snapshot_local')]
        subprocess.run(args, check=True, env=env)
        subprocess.run(['cmake', '--build', str(build / project), '-j', str(a.jobs)], check=True, env=env)
        if project == 'costmap':
            subprocess.run(['cmake', '--install', str(build / project)], check=True, env=env)
        else:
            shutil.copy2(build / project / 'curve_nav2_stack', r / 'native/curve_nav2_stack')
    print('Rebuilt in this checkout. Inspect changes, refresh the manifest, then rerun the full example.')


if __name__ == '__main__':
    main()
