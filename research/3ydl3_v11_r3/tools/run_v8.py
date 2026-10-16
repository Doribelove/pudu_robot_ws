"""V8-r5 full-validation dispatcher; frozen L1 with tested L2/L3 changes."""
import argparse
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', required=True, choices=['semantic', 'free-space'])
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--repeats', type=int, default=3)
    args, rest = parser.parse_known_args()
    script = 'run_benchmark.py' if args.mode == 'semantic' else 'run_free_full.py'
    command = [sys.executable, str(ROOT/'tools'/script), '--map-id', args.map_id,
               '--output', args.output, '--repeats', str(args.repeats), *rest]
    os.execv(sys.executable, command)


if __name__ == '__main__':
    main()
