"""Independent exact data-processing candidates; parent remains the default."""
import os


def enabled(flag):
    mode = os.environ.get('THREEYD_V9_R8_MODE', 'control')
    if mode not in ('control', 'O', 'F', 'OF', 'G', 'OG', 'FG', 'OFG', 'R', 'GR', 'OGR'):
        raise ValueError('invalid V9-r8 mode: ' + mode)
    return mode != 'control' and flag in mode
