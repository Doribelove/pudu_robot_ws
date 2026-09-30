"""Protect the query-specific fallback when a compiled contact does not match."""

import json
import subprocess
import sys
from pathlib import Path


def test_transition_entry_mismatch_falls_back_exactly():
    probe = Path(__file__).with_name('transition_table_probe.py')
    result = subprocess.run([sys.executable, str(probe)], check=True,
                            text=True, capture_output=True, timeout=60)
    evidence = json.loads(result.stdout.strip().splitlines()[-1])
    assert evidence == {'candidate_count': 3, 'fallback_exact': True}
