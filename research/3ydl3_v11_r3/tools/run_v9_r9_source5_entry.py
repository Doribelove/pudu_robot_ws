"""Run the actual workspace entry with saved selection and no mode overrides."""
import json
import os
from pathlib import Path
import subprocess

from compare_v9_review import sha

ROOT = Path(__file__).resolve().parents[1]


def main():
    spec = json.loads((ROOT/'report/SOURCE5_ENTRY_PROTOCOL_V2.json').read_text())
    assert spec['declared_before_entry_native'] and spec['no_mode_overrides']
    assert spec['expected_records'] == 20 and spec['B_P_default_enabled'] is False
    for name, expected in spec['files'].items():
        assert sha(ROOT/name) == expected, name
    chosen = json.loads((ROOT/'BASELINE.json').read_text())
    assert {k: chosen[k] for k in spec['selection']} == spec['selection']
    out = Path(spec['root'])
    assert not out.exists()
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    for mid in spec['maps']:
        command = ['taskset', '-c', '12', 'bash', str(ROOT/'run.sh'), '--map-id', mid,
                   '--output', str(out/'fixed'/mid), '--repeats', '1']
        assert not any(v.startswith(('--v9-mode', '--r9-mode', '--r2-mode', '--review-mode', '--query-file')) for v in command)
        print('SOURCE5_ACTUAL_ENTRY', mid, 'SAVED_SELECTION_NO_MODE_OVERRIDES', flush=True)
        subprocess.run(command, cwd=ROOT, env=env, check=True)
    for name, expected in spec['files'].items():
        assert sha(ROOT/name) == expected, name
    print('SOURCE5_ENTRY20_NATIVE_COMPLETE_REVIEW_REQUIRED', flush=True)


if __name__ == '__main__':
    main()
