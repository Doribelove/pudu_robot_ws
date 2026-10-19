"""Normalize two reports of one steering defect; never normalize acceptance."""
import os

def repairable_steering_failure(failures):
    if os.environ.get('THREEYD_V8_R5_REPAIR','0')!='1':
        return failures==['RAW_STEERING_JUMP']
    # Canonical truthfully-derived steering and raw geometry may both report
    # this defect. Their agreement must not accidentally disable repair.
    # Collision, scope, endpoint, traffic, curvature and unknown failures are
    # deliberately excluded. The full original audits run after real repair.
    return bool(failures) and set(failures)<={'RAW_STEERING_JUMP','STEERING_DISCONTINUITY'}
