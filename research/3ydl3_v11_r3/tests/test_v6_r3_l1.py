"""Exercise the isolated L1 harness without contaminating the pytest process."""

import json
import subprocess
import sys
from pathlib import Path


def test_same_lane_certificate_and_budget_fallback():
    probe = Path(__file__).with_name('same_lane_budget_probe.py')
    result = subprocess.run([sys.executable, str(probe)], check=True,
                            text=True, capture_output=True, timeout=90)
    evidence = json.loads(result.stdout.strip().splitlines()[-1])
    assert evidence['candidate_count'] == 1
    assert evidence['same_signature']
    assert evidence['first_phase_resumed']
    assert evidence['certificate']
    assert evidence['alternate_expansions'] == 0
    assert evidence['legacy_on_normal'] == 0
    assert evidence['legacy_on_budget_loss'] == 1

