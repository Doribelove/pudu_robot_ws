#!/usr/bin/env python3
"""Read-only P6 delivery preflight; no planner import, ROS node, or output writes."""
from __future__ import annotations
import argparse
import hashlib
import importlib.metadata
import importlib.machinery
import json
import os
from pathlib import Path
import platform
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, default=ROOT / 'runtime/3YDL3-V13-L2')
    parser.add_argument('--dependencies', type=Path, default=ROOT / 'docs/DEPENDENCIES.json')
    parser.add_argument('--registry', type=Path)
    parser.add_argument('--runtime-env', type=Path)
    parser.add_argument('--l3-config', type=Path)
    parser.add_argument('--verify-inputs', action='store_true', help='Also SHA256 every registered map input; read-only but can be slower.')
    parser.add_argument('--json', action='store_true', help='Print structured report to stdout.')
    args = parser.parse_args()
    dep = json.loads(args.dependencies.read_text())
    pkg = args.package.resolve()
    checks = []
    warnings = []

    def check(name, ok, detail):
        checks.append({'name': name, 'ok': bool(ok), 'detail': detail})

    check('platform', platform.system() == 'Linux' and platform.machine() == 'x86_64', platform.platform())
    check('python', sys.version_info[:2] == (3, 10), {'version': platform.python_version(), 'executable': sys.executable})
    for relative in ['env.bash', 'MANIFEST.json', 'profiles/p6/src/three_yd_v13/schedule.py', 'profiles/p6/src/three_yd_v5/p6_hint.py']:
        check('package:' + relative, (pkg / relative).is_file(), str(pkg / relative))
    runtime = args.runtime_env or Path(os.environ.get('V13_RUNTIME_ENV', dep['runtime_env']))
    check('runtime_env', runtime.is_file(), str(runtime))
    if str(runtime) == dep['runtime_env']:
        for path in dep['runtime_env_chain']:
            check('runtime_chain:' + path, Path(path).is_file(), path)
    else:
        warnings.append('Custom runtime env selected: its transitive ROS/Nav2 setup must be supplied by the recipient.')
    for item in dep['external_paths']:
        check('external:' + item['role'], Path(item['path']).exists(), item['path'])
    config = args.l3_config or pkg / dep['l3_config']['package_expected_relative']
    try:
        raw = config.read_bytes()
        json.loads(raw)
        actual = hashlib.sha256(raw).hexdigest()
        check('l3_config', actual == dep['l3_config']['sha256'], {'path': str(config), 'sha256': actual, 'expected': dep['l3_config']['sha256']})
    except (OSError, ValueError) as exc:
        check('l3_config', False, str(exc))
    for rel in dep['packaged_native_libraries']:
        lib = pkg / rel
        try:
            with lib.open('rb') as stream:
                head = stream.read(20)
            valid = head[:4] == b'\x7fELF' and len(head) >= 20 and head[4] == 2 and int.from_bytes(head[18:20], 'little') == 62
        except OSError:
            valid = False
        check('native:' + rel, valid, str(lib))
    for relative, asset in dep['runtime_assets_added'].items():
        target = pkg / relative
        try:
            digest = hashlib.sha256(target.read_bytes()).hexdigest()
        except OSError:
            digest = None
        check('runtime_asset:' + relative, digest == asset['sha256'], {'path': str(target), 'sha256': digest, 'expected': asset['sha256']})
    gaps = [rel for rel in dep['known_package_external_assets'] if not (pkg / rel).exists()]
    executable = pkg / 'base/runtime/base/native/curve_nav2_stack'
    check('runtime_nav2_executable', executable.is_file() and os.access(executable, os.X_OK), str(executable))
    if gaps:
        warnings.append('Package needs runtime asset binding: ' + ', '.join(gaps))
    registry = args.registry or Path(dep['registry'])
    try:
        maps = json.loads(registry.read_text())['maps']
        check('map_registry', len(maps) == 10 and sum(len(m['query_ids']) for m in maps) == 100, str(registry))
        for entry in maps:
            missing = [p for p in entry['input_sha256'] if not Path(p).is_file()]
            check('map_inputs:' + entry['map_id'], not missing, {'missing': missing, 'query_ids': entry['query_ids']})
            if args.verify_inputs:
                bad = []
                for path, expected in entry['input_sha256'].items():
                    try:
                        digest = hashlib.sha256()
                        with open(path, 'rb') as stream:
                            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                                digest.update(chunk)
                        sha = digest.hexdigest()
                        if sha != expected:
                            bad.append(path)
                    except OSError:
                        bad.append(path)
                check('map_sha256:' + entry['map_id'], not bad, {'mismatches': bad})
    except (OSError, ValueError, KeyError) as exc:
        check('map_registry', False, str(exc))
    for dist, expected in dep['python_distributions_observed'].items():
        try:
            value = importlib.metadata.version(dist)
        except importlib.metadata.PackageNotFoundError:
            value = None
        check('python_distribution:' + dist, value == expected, {'observed': value, 'registered': expected})
    for module in ['rclpy', 'nav2_msgs', 'geometry_msgs', 'sensor_msgs']:
        spec = importlib.machinery.PathFinder.find_spec(module, sys.path)
        check('ros_module:' + module, spec is not None, None if spec is None else spec.origin)
    expected_modules = {
        'three_yd_v13/schedule.py': 'profiles/p6/src/three_yd_v13/schedule.py',
        'three_yd_v5/p6_hint.py': 'profiles/p6/src/three_yd_v5/p6_hint.py',
        'three_yd_v13/service.py': 'base/src/three_yd_v13/service.py',
        'three_yd_v5/controller.py': 'base/runtime/base/src/three_yd_v5/controller.py',
        'three_yd_v5/curve_l3.py': 'base/runtime/base/src/three_yd_v5/curve_l3.py',
    }
    for relative, expected in expected_modules.items():
        candidates = [(Path(p) / relative).resolve() for p in sys.path if p and (Path(p) / relative).is_file()]
        found = candidates[0] if candidates else None
        check('module_path:' + relative, found == (pkg / expected).resolve(), {'first_candidate': str(found), 'expected': str(pkg / expected)})
    for key, value in {'V13_PROFILE': 'p6', 'V13_P7_OBSTACLE_L1': '0', 'V13_P6_HINT_MODE': 'local', 'V13_P5_HEADING_REFERENCE': '0', 'ROS_DISTRO': 'humble', 'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}.items():
        check('environment:' + key, os.environ.get(key) == value, {'actual': os.environ.get(key), 'expected': value})
    report = {'schema': 'p6-readonly-environment-check-v1', 'ok': all(c['ok'] for c in checks), 'checks': checks, 'warnings': warnings, 'scope': dep['read_only_check_scope'], 'packaged_runtime_assets_complete': not gaps, 'external_ros_and_map_dependencies_required': True, 'output_writes': 0, 'navigation_runs': 0}
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        for item in checks:
            if not item['ok']:
                print('FAIL', item['name'], json.dumps(item['detail'], ensure_ascii=False))
        print(f"{sum(c['ok'] for c in checks)}/{len(checks)} checks passed; local static preflight {'passed' if report['ok'] else 'incomplete'}")
        for warning in warnings:
            print('NOTE', warning)
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
