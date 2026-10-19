"""A: exact per-cell pose batching; B: separate early-section experiment."""
import os
MODES={'control':frozenset(),'A':frozenset('A'),'B':frozenset('B'),'AB':frozenset('AB')}
def mode():
    value=os.environ.get('THREEYD_V8_R3_MODE','A')
    if value not in MODES:raise ValueError('Unknown V8-r3 mode: '+value)
    return value
def enabled(part):return part in MODES[mode()]
