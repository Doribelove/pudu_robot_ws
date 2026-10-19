"""Opt-in isolated workspace Nav2 1.1.19 readback lock; never changes installed packages."""
import argparse, hashlib, json, os, sys
from pathlib import Path

def configure():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--snapshot', choices=['system', 'locked'], default='system')
    args, rest = parser.parse_known_args()
    sys.argv = [sys.argv[0], *rest]
    os.environ['THREEYD_SNAPSHOT'] = args.snapshot
    if args.snapshot == 'locked':
        prefix = Path(__file__).resolve().parents[1] / 'install/nav2_snapshot_local'
        if not (prefix / 'lib/libnav2_costmap_2d_core.so').is_file():
            raise FileNotFoundError(prefix)
        for key, value in [('LD_LIBRARY_PATH', prefix / 'lib'), ('AMENT_PREFIX_PATH', prefix)]:
            os.environ[key] = str(value) + ':' + os.environ.get(key, '')

def record(output):
    import psutil
    rows = []
    for proc in psutil.Process().children(recursive=True):
        try:
            libraries = sorted({m.path for m in proc.memory_maps() if 'nav2_costmap' in m.path or 'smac' in m.path or m.path.endswith('/liblayers.so')})
            if libraries:
                rows.append({'pid': proc.pid, 'command': proc.cmdline(), 'libraries': {
                    path: hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in libraries}})
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    (Path(output) / 'loaded_nav2_libraries.json').write_text(json.dumps({'snapshot': os.environ.get('THREEYD_SNAPSHOT'), 'processes': rows}, indent=2) + '\n')
    if os.environ.get('THREEYD_SNAPSHOT') == 'locked' and not any('install/nav2_snapshot_local/lib/libnav2_costmap_2d_core.so' in p for row in rows for p in row['libraries']):
        raise RuntimeError('Patched costmap library is not loaded')
