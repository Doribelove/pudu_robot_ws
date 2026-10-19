"""R6 candidate selection only; original trajectory hard audits are unchanged."""
import os


def heading_limit_degrees():
    mode = os.environ.get('THREEYD_V9_R6_MODE', 'control')
    if mode not in ('control', 'H'):
        raise ValueError('invalid V9-r6 mode: ' + mode)
    return 15.0 if mode == 'H' else 10.0
