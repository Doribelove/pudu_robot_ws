"""Static map inputs are fingerprinted once per L1 graph."""

import json
import subprocess
import sys
from pathlib import Path


def test_map_hash_cache_and_potential_cap():
    probe = Path(__file__).with_name('r6_hash_probe.py')
    result = subprocess.run([sys.executable, str(probe)], check=True,
                            text=True, capture_output=True, timeout=30)
    evidence = json.loads(result.stdout.strip().splitlines()[-1])
    assert evidence['candidate_counts'] == [3, 3]
    assert evidence['query_time_map_fingerprint_calls'] == 0
