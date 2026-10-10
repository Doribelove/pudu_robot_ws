"""V8-r2 independent exact-equivalent optimizations, never acceptance flags."""
import os
MODES={'control':frozenset(),'A':frozenset('A'),'B':frozenset('B'),'AB':frozenset('AB')}
def mode():
    value=os.environ.get('THREEYD_V8_R2_MODE','AB')
    if value not in MODES:raise ValueError('Unknown V8-r2 mode: '+value)
    return value
def enabled(part):return part in MODES[mode()]
