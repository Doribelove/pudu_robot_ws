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
    sdk = ROOT / 'runtime/nav2_sdk'
    # Build a coherent Nav2 1.1.19 ABI, including the accepted costmap patch.
    build_env = dict(os.environ, CMAKE_BUILD_PARALLEL_LEVEL=str(a.jobs))
    subprocess.run(['colcon', 'build', '--base-paths', str(ROOT / 'runtime/nav2_sources'),
                    str(r / 'external/nav2_costmap_2d'), '--merge-install',
                    '--build-base', str(build / 'sdk'), '--install-base', str(sdk),
                    '--executor', 'sequential', '--cmake-args', '-DCMAKE_BUILD_TYPE=Release',
                    '-DBUILD_TESTING=OFF'], cwd=build.parent, check=True, env=build_env)
    costmap_prefix = r / 'install/nav2_snapshot_local/lib'
    costmap_prefix.mkdir(parents=True, exist_ok=True)
    for lib in (sdk / 'lib').glob('*.so'):
        if lib.name in ['libnav2_costmap_2d_core.so', 'liblayers.so', 'libfilters.so', 'libnav2_costmap_2d_client.so']:
            shutil.copy2(lib, costmap_prefix / lib.name)
    env = dict(os.environ)
    env['AMENT_PREFIX_PATH'] = str(sdk) + ':' + env.get('AMENT_PREFIX_PATH', '')
    env['CMAKE_PREFIX_PATH'] = str(sdk) + ':' + env.get('CMAKE_PREFIX_PATH', '')
    subprocess.run(['cmake', '-S', str(r / 'native/nav2_stack'), '-B', str(build / 'stack'),
                    '-DCMAKE_BUILD_TYPE=Release'], check=True, env=env)
    subprocess.run(['cmake', '--build', str(build / 'stack'), '-j', str(a.jobs)], check=True, env=env)
    shutil.copy2(build / 'stack/curve_nav2_stack', r / 'native/curve_nav2_stack')
    print('Rebuilt in this checkout. Inspect changes, refresh the manifest, then rerun the full example.')


if __name__ == '__main__':
    main()
