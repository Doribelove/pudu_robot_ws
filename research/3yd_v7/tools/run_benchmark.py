"""V6-r7 traffic-boundary experiment on five fixed static map/query sets."""
import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXED = Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_fixed10_20260922')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--queries', default='all')
    parser.add_argument('--repeats', type=int, default=1)
    args, rest = parser.parse_known_args()
    if args.repeats < 1:
        parser.error('--repeats must be positive')
    if args.map_id == 'mengjia':
        script = ROOT / 'tools/run_wanda3w.py'
        parameters = ['--map', 'corroborated_map.yaml']
    else:
        sys.path.insert(0, str(FIXED))
        from fixed10_binding import resolve
        _, binding, _ = resolve(args.map_id)
        script = ROOT / 'tools/run_crossmap.py'
        parameters = ['--map-id', binding['map_id']]
        if binding['map_id'] == 'pudu_wanda_3f':
            parameters += ['--effective-map-yaml', str(ROOT / 'inputs/wanda2w_effective_map/map.yaml')]
        if binding['map_id'] == '3yd_synth_trizone_c_v2':
            parameters += ['--ack-timeout-s', '12']
    command = [sys.executable, str(script), *parameters, '--output', args.output,
               '--queries', args.queries, '--repeats', str(args.repeats), *rest]
    os.execv(sys.executable, command)

if __name__ == '__main__':
    main()
