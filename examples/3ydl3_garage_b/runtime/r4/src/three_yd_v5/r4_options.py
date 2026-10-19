"""Independent V8-r4 controls; parent V8-r3 remains unchanged."""
import os
def enabled(part):
    mode=os.environ.get('THREEYD_V8_R4_MODE','control')
    if mode not in ('control','A','B','AB','ABC','C'):raise ValueError('Invalid V8-r4 mode: '+mode)
    return mode!='control' and part in mode
