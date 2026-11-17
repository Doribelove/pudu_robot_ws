"""Explicit R9 data representation controls; direct imports are conservative."""
import os


def mode():
    value = os.environ.get('THREEYD_V9_R9_MODE', 'control')
    if value not in ('control', 'P', 'L', 'PL', 'M', 'PM', 'LM', 'PLM', 'U', 'PLU', 'PLMU', 'X', 'PLMX', 'PLMUX'):
        raise ValueError('INVALID_V9_R9_MODE:' + value)
    return value


def enabled(feature='P'):
    value = mode()
    return value != 'control' and feature in value
