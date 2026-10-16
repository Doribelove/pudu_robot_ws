"""The actual selected-entry dispatcher must forward and override the R9 choice."""
import json
import os
from pathlib import Path
import runpy
import sys
import pytest


@pytest.mark.parametrize('override', [[], ['--r9-mode', 'control'], ['--r9-mode=control']])
def test_saved_r9_choice_reaches_planner_and_explicit_cli_still_wins(tmp_path, monkeypatch, override):
    tools = tmp_path/'tools'; tools.mkdir()
    entry = tools/'run_v9_selected.py'
    entry.write_text((Path(__file__).resolve().parents[1]/'tools/run_v9_selected.py').read_text())
    choice = {'v9_mode': 'A', 'review_mode': 'S', 'r2_mode': 'control', 'r3_mode': 'control',
              'r5_mode': 'C', 'r6_mode': 'H', 'r7_mode': 'NT', 'r9_mode': 'P'}
    (tmp_path/'BASELINE.json').write_text(json.dumps(choice))
    monkeypatch.setattr(sys, 'argv', [str(entry), '--fixed-default', '--map-id', 'mengjia', *override])
    calls = []
    def capture(program, argv): calls.append((program, argv))
    monkeypatch.setattr(os, 'execv', capture)
    runpy.run_path(str(entry), run_name='__main__')
    assert len(calls) == 1 and calls[0][0] == sys.executable
    args = calls[0][1]; assert args[1] == str(tools/'run_task_progress.py') and '--fixed-default' not in args
    assert args[args.index('--workload')+1] == 'fixed'
    actual = [v.split('=', 1)[1] if v.startswith('--r9-mode=') else args[i+1]
              for i, v in enumerate(args) if v == '--r9-mode' or v.startswith('--r9-mode=')]
    assert actual == ['control' if override else 'P']
    for key in ('r5_mode', 'r6_mode', 'r7_mode'):
        assert args[args.index('--'+key.replace('_', '-'))+1] == choice[key]
