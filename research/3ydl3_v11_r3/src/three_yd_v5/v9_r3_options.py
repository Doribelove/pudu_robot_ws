"""Independent startup readback experiment; no parent/default change."""
import os
def mode():
    value=os.environ.get('THREEYD_V9_R3_MODE','control')
    if value not in ('control','I'):raise ValueError('UNKNOWN_V9_R3_MODE')
    return value
def enabled():return mode()=='I'
