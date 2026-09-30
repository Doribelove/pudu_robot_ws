"""Explicit V7 mode dispatcher; semantic and map-only inputs stay separate."""
import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', required=True, choices=('semantic', 'free-space'))
    parser.add_argument('--stage', choices=('l1', 'full'), default='l1')
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--query-id')
    args, other = parser.parse_known_args()
    if args.mode == 'semantic':
        if args.query_id is not None:
            parser.error('semantic mode uses the existing --queries option')
        if args.stage == 'l1':
            script = ROOT / 'tools/l1_only_worker.py'
            command = [sys.executable, str(script), '--map-id', args.map_id,
                       '--output', args.output, '--round', '1',
                       '--first-budget', '50000', '--alternate-budget', '10000', *other]
        else:
            script = ROOT / 'tools/run_benchmark.py'
            command = [sys.executable, str(script), '--map-id', args.map_id,
                       '--output', args.output, *other]
    elif args.stage == 'l1':
        if args.query_id is not None or other:
            parser.error('free-space L1 benchmark accepts exactly the fixed ten queries')
        script = ROOT / 'tools/run_v7_l1.py'
        command = [sys.executable, str(script), '--map-id', args.map_id,
                   '--output', args.output]
    else:
        if other:
            parser.error('free-space integration run has no extra arguments')
        script = ROOT / 'tools/smoke_v7_pipeline.py'
        command = [sys.executable, str(script), '--map-id', args.map_id,
                   '--query-id', args.query_id or 'A2B-01', '--output', args.output]
    os.execv(sys.executable, command)


if __name__ == '__main__':
    main()
